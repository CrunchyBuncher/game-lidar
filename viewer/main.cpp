// lidar_viewer: live point-cloud viewer. Reads depth frames + camera matrices
// from the shared-memory ring, unprojects them on the GPU into a voxel-deduped
// point pool, and renders it with a free-fly camera. Points that a newer frame
// sees straight through (things that moved away) are carved out.
//
// Usage: lidar_viewer [--voxel 0.05] [--capacity-m 16] [--table-bits 25]
//                     [--near-cut 0.3] [--max-range 500] [--height-range -1 20]
//                     [--no-carve] [--carve-margin 0.15] [--carve-rel 0.02]
//                     [--size 1600x900] [--out file.ply] [--save-after s] [--exit-after s]
// Keys:  right-drag look, WASD move, Q/E down/up, Shift fast, wheel speed,
//        F follow player, H color mode, T trail, X carving, +/- point size,
//        C clear, P save .ply, Space pause ingest, Esc quit.
#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include "app.h"
#include "ring.h"
#include "shaders.h"

using namespace DirectX;
using namespace lidar;

namespace {

struct Options {
    float voxel = 0.05f;
    uint32_t capacity = 16u << 20;
    uint32_t table_bits = 25;
    float near_cut = 0.3f;
    float max_range = 500.0f;
    float height_min = -1.0f, height_max = 20.0f;
    int width = 1600, height = 900;
    std::string out;
    float save_after = 0, exit_after = 0;
    bool carve = true;
    float carve_margin = 0.15f;  // meters
    float carve_rel = 0.02f;     // fraction of distance
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : "0"; };
        if (a == "--voxel") o.voxel = float(std::atof(next()));
        else if (a == "--capacity-m") o.capacity = uint32_t(std::atof(next()) * (1 << 20));
        else if (a == "--table-bits") o.table_bits = uint32_t(std::clamp(std::atoi(next()), 16, 28));
        else if (a == "--near-cut") o.near_cut = float(std::atof(next()));
        else if (a == "--max-range") o.max_range = float(std::atof(next()));
        else if (a == "--height-range") {
            o.height_min = float(std::atof(next()));
            o.height_max = float(std::atof(next()));
        } else if (a == "--size") std::sscanf(next(), "%dx%d", &o.width, &o.height);
        else if (a == "--out") o.out = next();
        else if (a == "--save-after") o.save_after = float(std::atof(next()));
        else if (a == "--exit-after") o.exit_after = float(std::atof(next()));
        else if (a == "--no-carve") o.carve = false;
        else if (a == "--carve-margin") o.carve_margin = float(std::atof(next()));
        else if (a == "--carve-rel") o.carve_rel = float(std::atof(next()));
        else std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
    }
    return o;
}

struct FrameCB {
    XMFLOAT4X4 view, proj, inv_view, inv_proj;
    uint32_t dims[2], src_dims[2];
    float near_cut, max_range, voxel_size;
    uint32_t table_mask, capacity, has_color;
    float carve_margin_abs, carve_margin_rel;
};
struct DrawCB {
    XMFLOAT4X4 view_proj;
    float px_to_ndc[2];
    float point_size;
    uint32_t color_mode;
    float height_min, height_max, pad[2];
};
struct PointData {
    float pos[3];
    uint32_t color;
};
struct LineVertex {
    XMFLOAT3 pos;
    uint32_t color;
};

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

// GPU point pool + voxel hash table.
struct PointCloud {
    uint32_t capacity = 0, table_size = 0;
    ComPtr<ID3D11Buffer> counter, table, points, free_list, args;
    ComPtr<ID3D11UnorderedAccessView> counter_uav, table_uav, points_uav, free_list_uav, args_uav;
    ComPtr<ID3D11ShaderResourceView> points_srv;

    void create(ID3D11Device* dev, uint32_t cap, uint32_t table_bits) {
        capacity = cap;
        table_size = 1u << table_bits;

        D3D11_BUFFER_DESC d{};
        d.ByteWidth = 16;
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        d.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
        check(dev->CreateBuffer(&d, nullptr, &counter), "counter");
        D3D11_UNORDERED_ACCESS_VIEW_DESC u{DXGI_FORMAT_R32_TYPELESS, D3D11_UAV_DIMENSION_BUFFER};
        u.Buffer.NumElements = 4;
        u.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
        check(dev->CreateUnorderedAccessView(counter.Get(), &u, &counter_uav), "counter uav");

        d.ByteWidth = table_size * 4;
        d.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
        d.StructureByteStride = 4;
        check(dev->CreateBuffer(&d, nullptr, &table), "hash table");
        check(dev->CreateUnorderedAccessView(table.Get(), nullptr, &table_uav), "table uav");

        d.ByteWidth = capacity * 4;
        check(dev->CreateBuffer(&d, nullptr, &free_list), "free list");
        check(dev->CreateUnorderedAccessView(free_list.Get(), nullptr, &free_list_uav), "free list uav");

        d.ByteWidth = capacity * sizeof(PointData);
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
        d.StructureByteStride = sizeof(PointData);
        check(dev->CreateBuffer(&d, nullptr, &points), "points");
        check(dev->CreateUnorderedAccessView(points.Get(), nullptr, &points_uav), "points uav");
        check(dev->CreateShaderResourceView(points.Get(), nullptr, &points_srv), "points srv");

        // [0..3] DrawInstancedIndirect, [4..6] DispatchIndirect (carving).
        d = {};
        d.ByteWidth = 32;
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        d.MiscFlags = D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
        check(dev->CreateBuffer(&d, nullptr, &args), "args");
        u = {DXGI_FORMAT_R32_UINT, D3D11_UAV_DIMENSION_BUFFER};
        u.Buffer.NumElements = 8;
        check(dev->CreateUnorderedAccessView(args.Get(), &u, &args_uav), "args uav");
    }

    void clear(ID3D11DeviceContext* ctx) {
        const UINT zero[4] = {};
        ctx->ClearUnorderedAccessViewUint(counter_uav.Get(), zero);
        ctx->ClearUnorderedAccessViewUint(table_uav.Get(), zero);
        ctx->ClearUnorderedAccessViewUint(args_uav.Get(), zero);
    }
};

uint32_t read_counter_blocking(ID3D11Device* dev, ID3D11DeviceContext* ctx, ID3D11Buffer* counter) {
    D3D11_BUFFER_DESC d{16, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
    ComPtr<ID3D11Buffer> s;
    check(dev->CreateBuffer(&d, nullptr, &s), "staging");
    ctx->CopyResource(s.Get(), counter);
    D3D11_MAPPED_SUBRESOURCE m;
    check(ctx->Map(s.Get(), 0, D3D11_MAP_READ, 0, &m), "Map counter");
    const uint32_t n = *static_cast<const uint32_t*>(m.pData);
    ctx->Unmap(s.Get(), 0);
    return n;
}

bool save_ply(ID3D11Device* dev, ID3D11DeviceContext* ctx, const PointCloud& pc, const std::string& path) {
    const uint32_t n = std::min(read_counter_blocking(dev, ctx, pc.counter.Get()), pc.capacity);
    std::vector<PointData> pts(n);
    if (n > 0) {
        D3D11_BUFFER_DESC d{UINT(n * sizeof(PointData)), D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
        ComPtr<ID3D11Buffer> s;
        check(dev->CreateBuffer(&d, nullptr, &s), "staging points");
        D3D11_BOX box{0, 0, 0, d.ByteWidth, 1, 1};
        ctx->CopySubresourceRegion(s.Get(), 0, 0, 0, 0, pc.points.Get(), 0, &box);
        D3D11_MAPPED_SUBRESOURCE m;
        check(ctx->Map(s.Get(), 0, D3D11_MAP_READ, 0, &m), "Map points");
        std::memcpy(pts.data(), m.pData, d.ByteWidth);
        ctx->Unmap(s.Get(), 0);
    }
    // Drop carved (deleted) slots.
    std::erase_if(pts, [](const PointData& p) { return std::isnan(p.pos[0]); });
    const uint32_t live = uint32_t(pts.size());
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::printf("cannot write %s\n", path.c_str());
        return false;
    }
    std::fprintf(f,
                 "ply\nformat binary_little_endian 1.0\ncomment game-lidar scan\nelement vertex %u\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n",
                 live);
    for (const PointData& p : pts) {
        std::fwrite(p.pos, sizeof(float), 3, f);
        const uint8_t rgb[3] = {uint8_t(p.color), uint8_t(p.color >> 8), uint8_t(p.color >> 16)};
        std::fwrite(rgb, 1, 3, f);
    }
    std::fclose(f);
    std::printf("saved %u points to %s\n", live, path.c_str());
    return true;
}

std::string default_scan_path() {
    std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &t);
    char buf[64];
    std::strftime(buf, sizeof(buf), "scans/scan_%Y%m%d_%H%M%S.ply", &tm);
    return buf;
}

}  // namespace

int main(int argc, char** argv) {
    const Options opt = parse(argc, argv);
    App app;
    if (!app.create(L"lidar_viewer", opt.width, opt.height)) return 1;
    ID3D11Device* dev = app.dev.Get();
    ID3D11DeviceContext* ctx = app.ctx.Get();

    // Shaders.
    auto carve_blob = compile_shader(shaders::kCompute, "cs_carve", "cs_5_0");
    auto ingest_blob = compile_shader(shaders::kCompute, "cs_ingest", "cs_5_0");
    auto args_blob = compile_shader(shaders::kCompute, "cs_args", "cs_5_0");
    auto vsp_blob = compile_shader(shaders::kDraw, "vs_points", "vs_5_0");
    auto vsl_blob = compile_shader(shaders::kDraw, "vs_lines", "vs_5_0");
    auto ps_blob = compile_shader(shaders::kDraw, "ps_main", "ps_5_0");
    ComPtr<ID3D11ComputeShader> cs_carve, cs_ingest, cs_args;
    ComPtr<ID3D11VertexShader> vs_points, vs_lines;
    ComPtr<ID3D11PixelShader> ps;
    check(dev->CreateComputeShader(carve_blob->GetBufferPointer(), carve_blob->GetBufferSize(), nullptr, &cs_carve),
          "cs_carve");
    check(dev->CreateComputeShader(ingest_blob->GetBufferPointer(), ingest_blob->GetBufferSize(), nullptr,
                                   &cs_ingest),
          "cs_ingest");
    check(dev->CreateComputeShader(args_blob->GetBufferPointer(), args_blob->GetBufferSize(), nullptr, &cs_args),
          "cs_args");
    check(dev->CreateVertexShader(vsp_blob->GetBufferPointer(), vsp_blob->GetBufferSize(), nullptr, &vs_points),
          "vs_points");
    check(dev->CreateVertexShader(vsl_blob->GetBufferPointer(), vsl_blob->GetBufferSize(), nullptr, &vs_lines),
          "vs_lines");
    check(dev->CreatePixelShader(ps_blob->GetBufferPointer(), ps_blob->GetBufferSize(), nullptr, &ps), "ps");

    const D3D11_INPUT_ELEMENT_DESC line_layout_desc[] = {
        {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
        {"COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
    };
    ComPtr<ID3D11InputLayout> line_layout;
    check(dev->CreateInputLayout(line_layout_desc, 2, vsl_blob->GetBufferPointer(), vsl_blob->GetBufferSize(),
                                 &line_layout),
          "line layout");

    // Frame upload textures (max protocol size; only a sub-rect is used).
    ComPtr<ID3D11Texture2D> depth_tex, color_tex;
    ComPtr<ID3D11ShaderResourceView> depth_srv, color_srv;
    {
        D3D11_TEXTURE2D_DESC d{};
        d.Width = kMaxWidth;
        d.Height = kMaxHeight;
        d.MipLevels = d.ArraySize = 1;
        d.SampleDesc.Count = 1;
        d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        d.Format = DXGI_FORMAT_R32_FLOAT;
        check(dev->CreateTexture2D(&d, nullptr, &depth_tex), "depth tex");
        check(dev->CreateShaderResourceView(depth_tex.Get(), nullptr, &depth_srv), "depth srv");
        d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        check(dev->CreateTexture2D(&d, nullptr, &color_tex), "color tex");
        check(dev->CreateShaderResourceView(color_tex.Get(), nullptr, &color_srv), "color srv");
    }

    PointCloud cloud;
    cloud.create(dev, opt.capacity, opt.table_bits);
    cloud.clear(ctx);

    auto frame_cb = make_cbuffer<FrameCB>(dev);
    auto draw_cb = make_cbuffer<DrawCB>(dev);
    {
        // cs_args reads capacity from the frame cbuffer, even before the first frame arrives.
        FrameCB cb{};
        cb.capacity = cloud.capacity;
        upload(ctx, frame_cb.Get(), cb);
    }
    // Binds the compute UAVs. The args buffer is only bound for cs_args, since
    // it's also the source of DispatchIndirect/DrawInstancedIndirect.
    auto bind_uavs = [&](bool with_args) {
        ID3D11UnorderedAccessView* uavs[5] = {cloud.counter_uav.Get(), cloud.table_uav.Get(),
                                              cloud.points_uav.Get(), cloud.free_list_uav.Get(),
                                              with_args ? cloud.args_uav.Get() : nullptr};
        ctx->CSSetUnorderedAccessViews(0, 5, uavs, nullptr);
    };
    auto unbind_compute = [&]() {
        ID3D11ShaderResourceView* null_srvs[2] = {};
        ID3D11UnorderedAccessView* null_uavs[5] = {};
        ctx->CSSetShaderResources(0, 2, null_srvs);
        ctx->CSSetUnorderedAccessViews(0, 5, null_uavs, nullptr);
    };
    auto update_args = [&]() {
        ctx->CSSetShader(cs_args.Get(), nullptr, 0);
        ctx->CSSetConstantBuffers(0, 1, frame_cb.GetAddressOf());
        bind_uavs(true);
        ctx->Dispatch(1, 1, 1);
        unbind_compute();
    };

    constexpr UINT kMaxLineVerts = 65536;
    ComPtr<ID3D11Buffer> line_vb;
    {
        D3D11_BUFFER_DESC d{kMaxLineVerts * sizeof(LineVertex), D3D11_USAGE_DYNAMIC, D3D11_BIND_VERTEX_BUFFER,
                            D3D11_CPU_ACCESS_WRITE};
        check(dev->CreateBuffer(&d, nullptr, &line_vb), "line vb");
    }

    // Reversed-Z for the viewer's own depth buffer.
    ComPtr<ID3D11Texture2D> zbuf;
    ComPtr<ID3D11DepthStencilView> zbuf_dsv;
    auto create_zbuf = [&]() {
        D3D11_TEXTURE2D_DESC d{};
        d.Width = app.width;
        d.Height = app.height;
        d.MipLevels = d.ArraySize = 1;
        d.SampleDesc.Count = 1;
        d.Format = DXGI_FORMAT_D32_FLOAT;
        d.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        check(dev->CreateTexture2D(&d, nullptr, &zbuf), "zbuf");
        check(dev->CreateDepthStencilView(zbuf.Get(), nullptr, &zbuf_dsv), "zbuf dsv");
    };
    create_zbuf();
    ComPtr<ID3D11DepthStencilState> dss;
    {
        D3D11_DEPTH_STENCIL_DESC d{};
        d.DepthEnable = TRUE;
        d.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
        d.DepthFunc = D3D11_COMPARISON_GREATER;
        check(dev->CreateDepthStencilState(&d, &dss), "dss");
    }
    ComPtr<ID3D11RasterizerState> rs;
    {
        D3D11_RASTERIZER_DESC d{};
        d.FillMode = D3D11_FILL_SOLID;
        d.CullMode = D3D11_CULL_NONE;
        d.DepthClipEnable = TRUE;
        check(dev->CreateRasterizerState(&d, &rs), "rs");
    }

    // Async stats readback of the point counter.
    // Deep enough that the oldest copy is done even when the GPU runs several frames behind.
    constexpr int kStatStages = 8;
    ComPtr<ID3D11Buffer> stat_stage[kStatStages];
    for (auto& s : stat_stage) {
        D3D11_BUFFER_DESC d{16, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
        check(dev->CreateBuffer(&d, nullptr, &s), "stat staging");
    }
    int stat_frame = 0;
    uint32_t point_count = 0, free_count = 0;  // slots ever allocated, slots freed by carving

    RingReader ring;
    Frame frame;
    bool have_player = false;
    XMFLOAT4X4 player_inv_view{}, player_inv_proj{};
    std::vector<XMFLOAT3> trail;

    // Viewer camera: start above the level looking down at it.
    float cam_pos[3] = {0.0f, 45.0f, -75.0f};
    float cam_yaw = 0.0f, cam_pitch = -0.5f, cam_speed = 10.0f;
    float point_size = 2.0f;
    uint32_t color_mode = 0;
    bool follow = false, show_trail = true, paused = false, saved = false, carve = opt.carve;

    LARGE_INTEGER qpf, t0, now;
    QueryPerformanceFrequency(&qpf);
    QueryPerformanceCounter(&t0);
    double last = 0, title_timer = 0, reconnect_timer = 0;
    uint64_t ingested = 0;
    int fps_frames = 0;
    double fps = 0;

    while (app.pump()) {
        if (app.resized) create_zbuf();
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - t0.QuadPart) / double(qpf.QuadPart);
        const float dt = float(std::min(t - last, 0.1));
        last = t;

        // Input.
        if (app.key_pressed('F')) follow = !follow;
        if (app.key_pressed('H')) color_mode ^= 1;
        if (app.key_pressed('T')) show_trail = !show_trail;
        if (app.key_pressed('X')) carve = !carve;
        if (app.key_pressed(VK_SPACE)) paused = !paused;
        if (app.key_pressed(VK_OEM_PLUS) || app.key_pressed(VK_ADD)) point_size = std::min(point_size + 1, 16.0f);
        if (app.key_pressed(VK_OEM_MINUS) || app.key_pressed(VK_SUBTRACT))
            point_size = std::max(point_size - 1, 1.0f);
        if (app.key_pressed('C')) {
            cloud.clear(ctx);
            update_args();
            trail.clear();
        }
        if (app.key_pressed('P')) save_ply(dev, ctx, cloud, opt.out.empty() ? default_scan_path() : opt.out);
        if (opt.save_after > 0 && !saved && t > opt.save_after) {
            save_ply(dev, ctx, cloud, opt.out.empty() ? default_scan_path() : opt.out);
            saved = true;
        }
        if (opt.exit_after > 0 && t > opt.exit_after) break;

        // Ingest new frames.
        if (!ring.is_open()) {
            reconnect_timer -= dt;
            if (reconnect_timer <= 0) {
                ring.try_open();
                reconnect_timer = 0.5;
            }
        }
        for (uint32_t i = 0; i < kSlotCount && ring.is_open() && ring.read_next(frame); ++i) {
            const FrameHeader& h = frame.header;
            if (h.flags & kFlagPaused) continue;
            // Without a pose (addon before M2) the frame is camera-relative: show it as a live
            // snapshot at the origin instead of accumulating it.
            const bool posed = (h.flags & kFlagPoseValid) != 0;
            const XMMATRIX view =
                posed ? XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(h.view)) : XMMatrixIdentity();
            const XMMATRIX proj = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(h.proj));
            XMStoreFloat4x4(&player_inv_view, XMMatrixInverse(nullptr, view));
            XMStoreFloat4x4(&player_inv_proj, XMMatrixInverse(nullptr, proj));
            have_player = true;
            const XMFLOAT3 p{player_inv_view._41, player_inv_view._42, player_inv_view._43};
            if (posed && (trail.empty() || XMVectorGetX(XMVector3Length(XMLoadFloat3(&p) - XMLoadFloat3(&trail.back()))) > 0.25f))
                trail.push_back(p);
            if (paused) continue;
            if (!posed) {
                cloud.clear(ctx);
                trail.clear();
            }

            D3D11_BOX box{0, 0, 0, h.width, h.height, 1};
            ctx->UpdateSubresource(depth_tex.Get(), 0, &box, frame.depth.data(), h.width * 4, 0);
            const bool has_color = (h.flags & kFlagHasColor) != 0;
            if (has_color) ctx->UpdateSubresource(color_tex.Get(), 0, &box, frame.color.data(), h.width * 4, 0);

            FrameCB cb{};
            XMStoreFloat4x4(&cb.view, view);
            XMStoreFloat4x4(&cb.proj, proj);
            cb.inv_proj = player_inv_proj;
            cb.inv_view = player_inv_view;
            cb.dims[0] = h.width;
            cb.dims[1] = h.height;
            cb.src_dims[0] = h.src_width;
            cb.src_dims[1] = h.src_height;
            cb.near_cut = opt.near_cut;
            cb.max_range = opt.max_range;
            cb.voxel_size = opt.voxel;
            cb.table_mask = cloud.table_size - 1;
            cb.capacity = cloud.capacity;
            cb.has_color = has_color;
            cb.carve_margin_abs = opt.carve_margin;
            cb.carve_margin_rel = opt.carve_rel;
            upload(ctx, frame_cb.Get(), cb);

            ID3D11ShaderResourceView* srvs[2] = {depth_srv.Get(), color_srv.Get()};
            ctx->CSSetConstantBuffers(0, 1, frame_cb.GetAddressOf());
            ctx->CSSetShaderResources(0, 2, srvs);
            bind_uavs(false);
            // Carve first, so this frame's own points aren't tested against itself
            // and freed slots can be reused right away.
            if (carve) {
                ctx->CSSetShader(cs_carve.Get(), nullptr, 0);
                ctx->DispatchIndirect(cloud.args.Get(), 16);
            }
            ctx->CSSetShader(cs_ingest.Get(), nullptr, 0);
            ctx->Dispatch((h.width + 7) / 8, (h.height + 7) / 8, 1);
            unbind_compute();
            update_args();
            ++ingested;
        }

        // Stats readback (a couple of frames late, never stalls).
        ctx->CopyResource(stat_stage[stat_frame % kStatStages].Get(), cloud.counter.Get());
        if (stat_frame >= kStatStages - 1) {
            D3D11_MAPPED_SUBRESOURCE m;
            ID3D11Buffer* s = stat_stage[(stat_frame + 1) % kStatStages].Get();
            if (ctx->Map(s, 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m) == S_OK) {
                const uint32_t* c = static_cast<const uint32_t*>(m.pData);
                point_count = c[0];
                free_count = c[1];
                ctx->Unmap(s, 0);
            }
        }
        ++stat_frame;

        // Viewer camera.
        if (follow && have_player) {
            const XMFLOAT3 p{player_inv_view._41, player_inv_view._42, player_inv_view._43};
            const float fx = player_inv_view._31, fz = player_inv_view._33;
            const float len = std::max(std::sqrt(fx * fx + fz * fz), 1e-4f);
            cam_pos[0] = p.x - fx / len * 8.0f;
            cam_pos[1] = p.y + 4.0f;
            cam_pos[2] = p.z - fz / len * 8.0f;
            cam_yaw = std::atan2(fx, fz);
            cam_pitch = -0.35f;
        } else {
            if (app.rmb_down()) {
                cam_yaw += app.mouse_dx * 0.003f;
                cam_pitch = std::clamp(cam_pitch - app.mouse_dy * 0.003f, -1.55f, 1.55f);
            }
            cam_speed = std::clamp(cam_speed * std::pow(1.2f, app.wheel), 0.5f, 500.0f);
            const float speed = cam_speed * (app.key_down(VK_SHIFT) ? 4.0f : 1.0f) * dt;
            const float cp = std::cos(cam_pitch);
            const float f[3] = {std::sin(cam_yaw) * cp, std::sin(cam_pitch), std::cos(cam_yaw) * cp};
            const float r[3] = {std::cos(cam_yaw), 0, -std::sin(cam_yaw)};
            for (int k = 0; k < 3; ++k) {
                if (app.key_down('W')) cam_pos[k] += f[k] * speed;
                if (app.key_down('S')) cam_pos[k] -= f[k] * speed;
                if (app.key_down('D')) cam_pos[k] += r[k] * speed;
                if (app.key_down('A')) cam_pos[k] -= r[k] * speed;
            }
            if (app.key_down('E')) cam_pos[1] += speed;
            if (app.key_down('Q')) cam_pos[1] -= speed;
        }
        const float cp = std::cos(cam_pitch);
        const XMMATRIX view =
            XMMatrixLookToLH(XMVectorSet(cam_pos[0], cam_pos[1], cam_pos[2], 1),
                             XMVectorSet(std::sin(cam_yaw) * cp, std::sin(cam_pitch), std::cos(cam_yaw) * cp, 0),
                             XMVectorSet(0, 1, 0, 0));
        const XMMATRIX proj = XMMatrixPerspectiveFovLH(XMConvertToRadians(60.0f),
                                                       float(app.width) / float(app.height), 10000.0f, 0.05f);

        DrawCB dcb{};
        XMStoreFloat4x4(&dcb.view_proj, view * proj);
        dcb.px_to_ndc[0] = 2.0f / float(app.width);
        dcb.px_to_ndc[1] = 2.0f / float(app.height);
        dcb.point_size = point_size;
        dcb.color_mode = color_mode;
        dcb.height_min = opt.height_min;
        dcb.height_max = opt.height_max;
        upload(ctx, draw_cb.Get(), dcb);

        // Lines: player frustum + trail.
        std::vector<LineVertex> lines;
        if (have_player) {
            const XMMATRIX inv_proj = XMLoadFloat4x4(&player_inv_proj);
            const XMMATRIX inv_view = XMLoadFloat4x4(&player_inv_view);
            const XMVECTOR apex = XMVector3TransformCoord(XMVectorZero(), inv_view);
            XMVECTOR corners[4];
            const float cx[4] = {-1, 1, 1, -1}, cy[4] = {-1, -1, 1, 1};
            for (int k = 0; k < 4; ++k) {
                // Any depth works for a direction; scale the ray to 2 m of view depth.
                XMVECTOR v = XMVector3TransformCoord(XMVectorSet(cx[k], cy[k], 0.5f, 1), inv_proj);
                v = XMVectorScale(v, 2.0f / std::max(std::fabs(XMVectorGetZ(v)), 1e-6f));
                corners[k] = XMVector3TransformCoord(v, inv_view);
            }
            const uint32_t yellow = 0xFF00FFFF;
            auto add = [&](XMVECTOR a, XMVECTOR b) {
                LineVertex va{{}, yellow}, vb{{}, yellow};
                XMStoreFloat3(&va.pos, a);
                XMStoreFloat3(&vb.pos, b);
                lines.push_back(va);
                lines.push_back(vb);
            };
            for (int k = 0; k < 4; ++k) {
                add(apex, corners[k]);
                add(corners[k], corners[(k + 1) % 4]);
            }
        }
        if (show_trail && trail.size() > 1) {
            const size_t max_segments = (kMaxLineVerts - lines.size()) / 2;
            const size_t start = trail.size() - 1 > max_segments ? trail.size() - 1 - max_segments : 0;
            for (size_t k = start; k + 1 < trail.size(); ++k) {
                lines.push_back({trail[k], 0xFFFFFFFF});
                lines.push_back({trail[k + 1], 0xFFFFFFFF});
            }
        }

        // Render.
        const float bg[4] = {0.03f, 0.035f, 0.045f, 1.0f};
        ctx->ClearRenderTargetView(app.back_rtv.Get(), bg);
        ctx->ClearDepthStencilView(zbuf_dsv.Get(), D3D11_CLEAR_DEPTH, 0.0f, 0);
        ctx->OMSetRenderTargets(1, app.back_rtv.GetAddressOf(), zbuf_dsv.Get());
        ctx->OMSetDepthStencilState(dss.Get(), 0);
        ctx->RSSetState(rs.Get());
        D3D11_VIEWPORT vp{0, 0, float(app.width), float(app.height), 0, 1};
        ctx->RSSetViewports(1, &vp);
        ctx->VSSetConstantBuffers(0, 1, draw_cb.GetAddressOf());
        ctx->PSSetShader(ps.Get(), nullptr, 0);

        ctx->IASetInputLayout(nullptr);
        ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx->VSSetShader(vs_points.Get(), nullptr, 0);
        ctx->VSSetShaderResources(0, 1, cloud.points_srv.GetAddressOf());
        ctx->DrawInstancedIndirect(cloud.args.Get(), 0);
        ID3D11ShaderResourceView* null_srv = nullptr;
        ctx->VSSetShaderResources(0, 1, &null_srv);

        if (!lines.empty()) {
            D3D11_MAPPED_SUBRESOURCE m;
            check(ctx->Map(line_vb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m), "Map lines");
            std::memcpy(m.pData, lines.data(), lines.size() * sizeof(LineVertex));
            ctx->Unmap(line_vb.Get(), 0);
            const UINT stride = sizeof(LineVertex), offset = 0;
            ctx->IASetVertexBuffers(0, 1, line_vb.GetAddressOf(), &stride, &offset);
            ctx->IASetInputLayout(line_layout.Get());
            ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
            ctx->VSSetShader(vs_lines.Get(), nullptr, 0);
            ctx->Draw(UINT(lines.size()), 0);
        }
        app.present(true);

        ++fps_frames;
        title_timer += dt;
        if (title_timer > 0.5) {
            fps = fps_frames / title_timer;
            fps_frames = 0;
            title_timer = 0;
            wchar_t buf[256];
            const uint32_t used = std::min(point_count, cloud.capacity);
            const uint32_t live = used - std::min(free_count, used);
            swprintf(buf, 256,
                     L"lidar_viewer  %.0f fps  |  %s  |  frames %llu (dropped %llu)  |  points %.2fM / %.1fM%s  |  "
                     L"carving %s%s",
                     fps, ring.is_open() ? L"connected" : L"waiting for producer", ingested, ring.dropped(),
                     live / 1e6, cloud.capacity / 1e6, point_count >= cloud.capacity && free_count == 0 ? L" FULL" : L"",
                     carve ? L"on" : L"off", paused ? L"  PAUSED" : L"");
            app.set_title(buf);
        }
    }
    return 0;
}
