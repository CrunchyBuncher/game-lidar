// fake_game: a tiny D3D11 "game" that renders the test level along a scripted
// camera path and publishes depth + camera matrices to the shared-memory ring,
// exactly as the ReShade addon will. It also keeps view/proj in a constant
// buffer (b0) the way real games do, as a target for the M2 sniffer.
//
// Usage: fake_game [--depth standard|reversed|reversed-infinite] [--capture-width 480]
//                  [--capture-every 1] [--fov 70] [--no-npc] [--no-color]
//                  [--size 1280x720] [--duration seconds]
// Keys:  M toggle manual camera (WASD/QE + right-drag), Space pause capture, Esc quit.
#include <DirectXMath.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "app.h"
#include "ring.h"
#include "scene.h"

using namespace DirectX;
using namespace lidar;

namespace {

const char* kSceneHlsl = R"(
cbuffer Camera : register(b0) {
    row_major float4x4 view;
    row_major float4x4 proj;
    row_major float4x4 view_proj;
    float4 cam_pos;
};
cbuffer Object : register(b1) {
    row_major float4x4 world;
    float4 tint;
};
struct VSIn  { float3 pos : POSITION; float3 nrm : NORMAL; float4 col : COLOR; };
struct VSOut { float4 pos : SV_Position; float3 wpos : WPOS; float3 nrm : NORMAL; float4 col : COLOR; };

VSOut vs_main(VSIn i) {
    VSOut o;
    float4 w = mul(float4(i.pos, 1), world);
    o.pos = mul(mul(w, view), proj);
    o.wpos = w.xyz;
    o.nrm = mul(i.nrm, (float3x3)world);
    o.col = i.col * tint;
    return o;
}

float4 ps_main(VSOut i) : SV_Target {
    float3 n = normalize(i.nrm);
    float diff = saturate(dot(n, normalize(float3(0.4, 1.0, 0.3)))) * 0.7 + 0.3;
    float3 cell = floor(i.wpos + 1e-3);
    float checker = frac((cell.x + cell.y + cell.z) * 0.5) * 2.0;
    float fog = saturate(length(i.wpos - cam_pos.xyz) / 150.0);
    float3 c = i.col.rgb * diff * (0.85 + 0.15 * checker);
    return float4(lerp(c, float3(0.55, 0.7, 0.9), fog * fog), 1);
}
)";

// Point-samples the full-res depth/color down to the capture size, using the
// source-pixel mapping defined in protocol.h.
const char* kDownsampleHlsl = R"(
Texture2D<float> src_depth : register(t0);
Texture2D<float4> src_color : register(t1);
RWTexture2D<float> dst_depth : register(u0);
RWTexture2D<unorm float4> dst_color : register(u1);
cbuffer Dims : register(b0) { uint2 src_dims; uint2 dst_dims; };

[numthreads(8, 8, 1)]
void cs_main(uint3 id : SV_DispatchThreadID) {
    if (any(id.xy >= dst_dims)) return;
    uint2 s = min((uint2)(((float2)id.xy + 0.5) * (float2)src_dims / (float2)dst_dims), src_dims - 1);
    dst_depth[id.xy] = src_depth.Load(int3(s, 0));
    dst_color[id.xy] = src_color.Load(int3(s, 0));
}
)";

struct CameraCB {
    XMFLOAT4X4 view, proj, view_proj;
    XMFLOAT4 cam_pos;
};
struct ObjectCB {
    XMFLOAT4X4 world;
    XMFLOAT4 tint;
};
struct DimsCB {
    uint32_t src_w, src_h, dst_w, dst_h;
};
struct Vertex {
    XMFLOAT3 pos, nrm;
    uint32_t color;
};

enum class DepthMode { Standard, Reversed, ReversedInfinite };

struct Options {
    DepthMode depth = DepthMode::Reversed;
    uint32_t capture_width = 480;
    uint32_t capture_every = 1;
    float fov_deg = 70.0f;
    bool npc = true;
    bool color = true;
    int width = 1280, height = 720;
    float duration = 0;  // 0 = run until closed
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--depth") {
            std::string d = next();
            o.depth = d == "standard" ? DepthMode::Standard
                      : d == "reversed-infinite" ? DepthMode::ReversedInfinite
                                                 : DepthMode::Reversed;
        } else if (a == "--capture-width") {
            o.capture_width = uint32_t(std::atoi(next()));
        } else if (a == "--capture-every") {
            o.capture_every = uint32_t(std::max(1, std::atoi(next())));
        } else if (a == "--fov") {
            o.fov_deg = float(std::atof(next()));
        } else if (a == "--no-npc") {
            o.npc = false;
        } else if (a == "--no-color") {
            o.color = false;
        } else if (a == "--size") {
            std::sscanf(next(), "%dx%d", &o.width, &o.height);
        } else if (a == "--duration") {
            o.duration = float(std::atof(next()));
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
        }
    }
    return o;
}

void append_box(std::vector<Vertex>& v, const float mn[3], const float mx[3], uint32_t color) {
    for (int a = 0; a < 3; ++a) {
        for (int s = 0; s < 2; ++s) {
            const int b = (a + 1) % 3, c = (a + 2) % 3;
            float n[3] = {0, 0, 0};
            n[a] = s ? 1.0f : -1.0f;
            auto corner = [&](int ub, int uc) {
                float p[3];
                p[a] = s ? mx[a] : mn[a];
                p[b] = ub ? mx[b] : mn[b];
                p[c] = uc ? mx[c] : mn[c];
                return Vertex{{p[0], p[1], p[2]}, {n[0], n[1], n[2]}, color};
            };
            const Vertex q[4] = {corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)};
            for (int idx : {0, 1, 2, 0, 2, 3}) v.push_back(q[idx]);
        }
    }
}

XMMATRIX make_proj(DepthMode mode, float fov, float aspect) {
    const float n = 0.1f, f = 1000.0f;
    switch (mode) {
        case DepthMode::Standard:
            return XMMatrixPerspectiveFovLH(fov, aspect, n, f);
        case DepthMode::Reversed:
            return XMMatrixPerspectiveFovLH(fov, aspect, f, n);
        case DepthMode::ReversedInfinite: {
            const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
            return XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, n, 0);
        }
    }
    return XMMatrixIdentity();
}

template <class T>
ComPtr<ID3D11Buffer> make_cbuffer(ID3D11Device* dev) {
    D3D11_BUFFER_DESC d{};
    d.ByteWidth = (sizeof(T) + 15) & ~15u;
    d.Usage = D3D11_USAGE_DYNAMIC;
    d.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    d.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    ComPtr<ID3D11Buffer> b;
    check(dev->CreateBuffer(&d, nullptr, &b), "CreateBuffer(cb)");
    return b;
}

template <class T>
void upload(ID3D11DeviceContext* ctx, ID3D11Buffer* b, const T& data) {
    D3D11_MAPPED_SUBRESOURCE m;
    check(ctx->Map(b, 0, D3D11_MAP_WRITE_DISCARD, 0, &m), "Map(cb)");
    std::memcpy(m.pData, &data, sizeof(T));
    ctx->Unmap(b, 0);
}

// Full-res render targets + capture-size targets, recreated on resize.
struct Targets {
    ComPtr<ID3D11Texture2D> color;
    ComPtr<ID3D11RenderTargetView> color_rtv;
    ComPtr<ID3D11ShaderResourceView> color_srv;
    ComPtr<ID3D11Texture2D> depth;
    ComPtr<ID3D11DepthStencilView> depth_dsv;
    ComPtr<ID3D11ShaderResourceView> depth_srv;

    uint32_t cap_w = 0, cap_h = 0;
    ComPtr<ID3D11Texture2D> cap_depth, cap_color;
    ComPtr<ID3D11UnorderedAccessView> cap_depth_uav, cap_color_uav;

    static constexpr int kStaging = 3;
    ComPtr<ID3D11Texture2D> stage_depth[kStaging], stage_color[kStaging];

    void create(ID3D11Device* dev, int w, int h, uint32_t capture_width) {
        D3D11_TEXTURE2D_DESC d{};
        d.Width = w;
        d.Height = h;
        d.MipLevels = d.ArraySize = 1;
        d.SampleDesc.Count = 1;
        d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        d.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
        check(dev->CreateTexture2D(&d, nullptr, &color), "color");
        check(dev->CreateRenderTargetView(color.Get(), nullptr, &color_rtv), "color rtv");
        check(dev->CreateShaderResourceView(color.Get(), nullptr, &color_srv), "color srv");

        d.Format = DXGI_FORMAT_R32_TYPELESS;
        d.BindFlags = D3D11_BIND_DEPTH_STENCIL | D3D11_BIND_SHADER_RESOURCE;
        check(dev->CreateTexture2D(&d, nullptr, &depth), "depth");
        D3D11_DEPTH_STENCIL_VIEW_DESC dsv{DXGI_FORMAT_D32_FLOAT, D3D11_DSV_DIMENSION_TEXTURE2D};
        check(dev->CreateDepthStencilView(depth.Get(), &dsv, &depth_dsv), "dsv");
        D3D11_SHADER_RESOURCE_VIEW_DESC srv{DXGI_FORMAT_R32_FLOAT, D3D11_SRV_DIMENSION_TEXTURE2D};
        srv.Texture2D.MipLevels = 1;
        check(dev->CreateShaderResourceView(depth.Get(), &srv, &depth_srv), "depth srv");

        cap_w = std::min(capture_width, kMaxWidth);
        cap_h = std::min(uint32_t(std::lround(double(cap_w) * h / w)), kMaxHeight);
        d.Width = cap_w;
        d.Height = cap_h;
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        d.Format = DXGI_FORMAT_R32_FLOAT;
        check(dev->CreateTexture2D(&d, nullptr, &cap_depth), "cap depth");
        check(dev->CreateUnorderedAccessView(cap_depth.Get(), nullptr, &cap_depth_uav), "cap depth uav");
        d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        check(dev->CreateTexture2D(&d, nullptr, &cap_color), "cap color");
        check(dev->CreateUnorderedAccessView(cap_color.Get(), nullptr, &cap_color_uav), "cap color uav");

        d.BindFlags = 0;
        d.Usage = D3D11_USAGE_STAGING;
        d.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        for (int i = 0; i < kStaging; ++i) {
            d.Format = DXGI_FORMAT_R32_FLOAT;
            check(dev->CreateTexture2D(&d, nullptr, &stage_depth[i]), "stage depth");
            d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            check(dev->CreateTexture2D(&d, nullptr, &stage_color[i]), "stage color");
        }
    }
};

struct PendingCapture {
    int stage;
    FrameHeader header;
};

}  // namespace

int main(int argc, char** argv) {
    const Options opt = parse(argc, argv);
    App app;
    if (!app.create(L"fake_game", opt.width, opt.height)) return 1;
    ID3D11Device* dev = app.dev.Get();
    ID3D11DeviceContext* ctx = app.ctx.Get();

    RingWriter ring;
    if (!ring.open()) {
        std::fprintf(stderr, "failed to create shared memory ring\n");
        return 1;
    }

    // Geometry.
    const auto boxes = scene::build();
    std::vector<Vertex> verts;
    for (const auto& b : boxes) append_box(verts, b.min, b.max, b.color);
    const UINT static_count = UINT(verts.size());
    const float npc_min[3] = {-0.3f, 0.0f, -0.3f}, npc_max[3] = {0.3f, 1.8f, 0.3f};
    append_box(verts, npc_min, npc_max, scene::rgb(255, 255, 255));

    ComPtr<ID3D11Buffer> vb;
    {
        D3D11_BUFFER_DESC d{UINT(verts.size() * sizeof(Vertex)), D3D11_USAGE_IMMUTABLE, D3D11_BIND_VERTEX_BUFFER};
        D3D11_SUBRESOURCE_DATA init{verts.data()};
        check(dev->CreateBuffer(&d, &init, &vb), "vertex buffer");
    }

    auto vs_blob = compile_shader(kSceneHlsl, "vs_main", "vs_5_0");
    auto ps_blob = compile_shader(kSceneHlsl, "ps_main", "ps_5_0");
    auto cs_blob = compile_shader(kDownsampleHlsl, "cs_main", "cs_5_0");
    ComPtr<ID3D11VertexShader> vs;
    ComPtr<ID3D11PixelShader> ps;
    ComPtr<ID3D11ComputeShader> cs;
    check(dev->CreateVertexShader(vs_blob->GetBufferPointer(), vs_blob->GetBufferSize(), nullptr, &vs), "vs");
    check(dev->CreatePixelShader(ps_blob->GetBufferPointer(), ps_blob->GetBufferSize(), nullptr, &ps), "ps");
    check(dev->CreateComputeShader(cs_blob->GetBufferPointer(), cs_blob->GetBufferSize(), nullptr, &cs), "cs");

    const D3D11_INPUT_ELEMENT_DESC layout_desc[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0},
    };
    ComPtr<ID3D11InputLayout> layout;
    check(dev->CreateInputLayout(layout_desc, 3, vs_blob->GetBufferPointer(), vs_blob->GetBufferSize(), &layout),
          "input layout");

    const bool reversed = opt.depth != DepthMode::Standard;
    ComPtr<ID3D11DepthStencilState> dss;
    {
        D3D11_DEPTH_STENCIL_DESC d{};
        d.DepthEnable = TRUE;
        d.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        d.DepthFunc = reversed ? D3D11_COMPARISON_GREATER : D3D11_COMPARISON_LESS;
        check(dev->CreateDepthStencilState(&d, &dss), "depth state");
    }
    ComPtr<ID3D11RasterizerState> rs;
    {
        D3D11_RASTERIZER_DESC d{};
        d.FillMode = D3D11_FILL_SOLID;
        d.CullMode = D3D11_CULL_NONE;
        d.DepthClipEnable = TRUE;
        check(dev->CreateRasterizerState(&d, &rs), "raster state");
    }

    auto camera_cb = make_cbuffer<CameraCB>(dev);
    auto object_cb = make_cbuffer<ObjectCB>(dev);
    auto dims_cb = make_cbuffer<DimsCB>(dev);

    Targets tg;
    tg.create(dev, app.width, app.height, opt.capture_width);
    std::deque<PendingCapture> pending;
    int next_stage = 0;

    LARGE_INTEGER qpf, t0, now;
    QueryPerformanceFrequency(&qpf);
    QueryPerformanceCounter(&t0);
    double last = 0, path_time = 0, title_timer = 0;
    uint64_t frame_index = 0, published = 0, skipped = 0;
    bool manual = false, paused = false;
    scene::Pose pose = scene::camera_path(0);

    while (app.pump()) {
        if (app.resized) {
            pending.clear();  // staging textures are about to be replaced
            tg = Targets{};
            tg.create(dev, app.width, app.height, opt.capture_width);
        }
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - t0.QuadPart) / double(qpf.QuadPart);
        const float dt = float(std::min(t - last, 0.1));
        last = t;
        if (opt.duration > 0 && t > opt.duration) break;

        if (app.key_pressed('M')) manual = !manual;
        if (app.key_pressed(VK_SPACE)) paused = !paused;

        // Camera.
        if (manual) {
            if (app.rmb_down()) {
                pose.yaw += app.mouse_dx * 0.003f;
                pose.pitch = std::clamp(pose.pitch - app.mouse_dy * 0.003f, -1.5f, 1.5f);
            }
            const float speed = (app.key_down(VK_SHIFT) ? 12.0f : 4.0f) * dt;
            const float fx = std::sin(pose.yaw), fz = std::cos(pose.yaw);
            if (app.key_down('W')) pose.pos[0] += fx * speed, pose.pos[2] += fz * speed;
            if (app.key_down('S')) pose.pos[0] -= fx * speed, pose.pos[2] -= fz * speed;
            if (app.key_down('D')) pose.pos[0] += fz * speed, pose.pos[2] -= fx * speed;
            if (app.key_down('A')) pose.pos[0] -= fz * speed, pose.pos[2] += fx * speed;
            if (app.key_down('E')) pose.pos[1] += speed;
            if (app.key_down('Q')) pose.pos[1] -= speed;
        } else {
            path_time += dt;
            pose = scene::camera_path(float(path_time));
        }

        const float cp = std::cos(pose.pitch);
        const XMVECTOR eye = XMVectorSet(pose.pos[0], pose.pos[1], pose.pos[2], 1);
        const XMVECTOR fwd = XMVectorSet(std::sin(pose.yaw) * cp, std::sin(pose.pitch), std::cos(pose.yaw) * cp, 0);
        const XMMATRIX view = XMMatrixLookToLH(eye, fwd, XMVectorSet(0, 1, 0, 0));
        const XMMATRIX proj =
            make_proj(opt.depth, XMConvertToRadians(opt.fov_deg), float(app.width) / float(app.height));

        CameraCB cam;
        XMStoreFloat4x4(&cam.view, view);
        XMStoreFloat4x4(&cam.proj, proj);
        XMStoreFloat4x4(&cam.view_proj, view * proj);
        cam.cam_pos = {pose.pos[0], pose.pos[1], pose.pos[2], 1};
        upload(ctx, camera_cb.Get(), cam);

        // Draw the scene.
        const float sky[4] = {0.55f, 0.7f, 0.9f, 1.0f};
        ctx->ClearRenderTargetView(tg.color_rtv.Get(), sky);
        ctx->ClearDepthStencilView(tg.depth_dsv.Get(), D3D11_CLEAR_DEPTH, reversed ? 0.0f : 1.0f, 0);
        ctx->OMSetRenderTargets(1, tg.color_rtv.GetAddressOf(), tg.depth_dsv.Get());
        ctx->OMSetDepthStencilState(dss.Get(), 0);
        ctx->RSSetState(rs.Get());
        D3D11_VIEWPORT vp{0, 0, float(app.width), float(app.height), 0, 1};
        ctx->RSSetViewports(1, &vp);
        const UINT stride = sizeof(Vertex), offset = 0;
        ctx->IASetVertexBuffers(0, 1, vb.GetAddressOf(), &stride, &offset);
        ctx->IASetInputLayout(layout.Get());
        ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx->VSSetShader(vs.Get(), nullptr, 0);
        ctx->PSSetShader(ps.Get(), nullptr, 0);
        ID3D11Buffer* cbs[2] = {camera_cb.Get(), object_cb.Get()};
        ctx->VSSetConstantBuffers(0, 2, cbs);
        ctx->PSSetConstantBuffers(0, 2, cbs);

        ObjectCB obj;
        XMStoreFloat4x4(&obj.world, XMMatrixIdentity());
        obj.tint = {1, 1, 1, 1};
        upload(ctx, object_cb.Get(), obj);
        ctx->Draw(static_count, 0);
        if (opt.npc) {
            XMStoreFloat4x4(&obj.world, XMMatrixTranslation(20.0f * std::sin(float(t) * 0.3f), 0, -26.5f));
            obj.tint = {0.9f, 0.25f, 0.2f, 1};
            upload(ctx, object_cb.Get(), obj);
            ctx->Draw(UINT(verts.size()) - static_count, static_count);
        }
        ctx->OMSetRenderTargets(0, nullptr, nullptr);

        // Capture: downsample on the GPU, copy to a staging texture, read back later.
        const bool want_capture = !paused && frame_index % opt.capture_every == 0;
        if (want_capture && pending.size() < Targets::kStaging) {
            upload(ctx, dims_cb.Get(), DimsCB{uint32_t(app.width), uint32_t(app.height), tg.cap_w, tg.cap_h});
            ID3D11ShaderResourceView* srvs[2] = {tg.depth_srv.Get(), tg.color_srv.Get()};
            ID3D11UnorderedAccessView* uavs[2] = {tg.cap_depth_uav.Get(), tg.cap_color_uav.Get()};
            ctx->CSSetShader(cs.Get(), nullptr, 0);
            ctx->CSSetShaderResources(0, 2, srvs);
            ctx->CSSetUnorderedAccessViews(0, 2, uavs, nullptr);
            ctx->CSSetConstantBuffers(0, 1, dims_cb.GetAddressOf());
            ctx->Dispatch((tg.cap_w + 7) / 8, (tg.cap_h + 7) / 8, 1);
            ID3D11ShaderResourceView* null_srvs[2] = {};
            ID3D11UnorderedAccessView* null_uavs[2] = {};
            ctx->CSSetShaderResources(0, 2, null_srvs);
            ctx->CSSetUnorderedAccessViews(0, 2, null_uavs, nullptr);

            const int s = next_stage;
            next_stage = (next_stage + 1) % Targets::kStaging;
            ctx->CopyResource(tg.stage_depth[s].Get(), tg.cap_depth.Get());
            if (opt.color) ctx->CopyResource(tg.stage_color[s].Get(), tg.cap_color.Get());

            PendingCapture pc{s, {}};
            FrameHeader& h = pc.header;
            h.frame_index = frame_index;
            h.timestamp_qpc = uint64_t(now.QuadPart);
            h.width = tg.cap_w;
            h.height = tg.cap_h;
            h.src_width = uint32_t(app.width);
            h.src_height = uint32_t(app.height);
            h.depth_format = uint32_t(DepthFormat::Float32Ndc);
            h.flags = kFlagPoseValid | (opt.color ? kFlagHasColor : 0u);
            std::memcpy(h.view, &cam.view, sizeof(h.view));
            std::memcpy(h.proj, &cam.proj, sizeof(h.proj));
            pending.push_back(pc);
        } else if (want_capture) {
            ++skipped;
        }

        // Publish any readbacks the GPU has finished, without stalling.
        while (!pending.empty()) {
            const PendingCapture& pc = pending.front();
            D3D11_MAPPED_SUBRESOURCE md, mc{};
            if (ctx->Map(tg.stage_depth[pc.stage].Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &md) != S_OK)
                break;
            if (opt.color) check(ctx->Map(tg.stage_color[pc.stage].Get(), 0, D3D11_MAP_READ, 0, &mc), "Map color");

            Slot* slot = ring.begin_frame();
            slot->frame = pc.header;
            const uint32_t w = pc.header.width, h = pc.header.height;
            for (uint32_t y = 0; y < h; ++y) {
                std::memcpy(&slot->depth[y * w], static_cast<const uint8_t*>(md.pData) + y * md.RowPitch,
                            w * sizeof(float));
                if (opt.color)
                    std::memcpy(&slot->color[y * w], static_cast<const uint8_t*>(mc.pData) + y * mc.RowPitch,
                                w * sizeof(uint32_t));
            }
            ring.commit();
            ctx->Unmap(tg.stage_depth[pc.stage].Get(), 0);
            if (opt.color) ctx->Unmap(tg.stage_color[pc.stage].Get(), 0);
            pending.pop_front();
            ++published;
        }

        ctx->CopyResource(app.back_buffer.Get(), tg.color.Get());
        app.present(true);
        ++frame_index;

        title_timer += dt;
        if (title_timer > 0.5) {
            title_timer = 0;
            const wchar_t* modes[] = {L"standard", L"reversed", L"reversed-infinite"};
            wchar_t buf[256];
            swprintf(buf, 256, L"fake_game  [%s]  depth=%s  capture %ux%u  published=%llu skipped=%llu%s",
                     manual ? L"manual" : L"auto", modes[int(opt.depth)], tg.cap_w, tg.cap_h, published, skipped,
                     paused ? L"  PAUSED" : L"");
            app.set_title(buf);
        }
    }
    return 0;
}
