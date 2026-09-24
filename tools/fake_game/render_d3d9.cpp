// The fake game's D3D9 renderer, laid out the way D3D9 games keep their camera: there are no
// constant buffers, so the matrices go into vertex shader constant registers with
// SetVertexShaderConstantF. The camera block (same bytes as the D3D11 cbuffer b0) fills c0-c12,
// then the per-object decoy (world + tint) c13-c17, rewritten before every draw. The same profile
// therefore works in all three APIs: slot 0, view at byte 0 (c0), proj at byte 64 (c4).
//
// Depth comes from the device's auto depth-stencil (D24S8), or with --own-depth from a separate
// CreateDepthStencilSurface, the two ways D3D9 games get one. Render only: D3D9 depth can't be read
// back without the INTZ trick, which is the addon's job.
//
// The vertex shader applies the D3D9 half-pixel offset, so pixels cover the same scene positions
// as in D3D10+ and the depth matches a D3D11 reference with a 24-bit depth buffer (--d24).
#include <d3d9.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "fake_game.h"

using namespace DirectX;

namespace lidar::fake {
namespace {

const char* kSceneHlsl9 = R"(
row_major float4x4 view      : register(c0);
row_major float4x4 proj      : register(c4);
row_major float4x4 view_proj : register(c8);
float4 cam_pos               : register(c12);
row_major float4x4 world     : register(c13);
float4 tint                  : register(c17);
float4 half_pixel            : register(c18);  // (-1/width, 1/height): D3D9 pixel centers vs D3D10+

struct VSIn  { float3 pos : POSITION; float3 nrm : NORMAL; float4 col : COLOR; };
struct VSOut { float4 pos : POSITION; float3 wpos : TEXCOORD0; float3 nrm : TEXCOORD1; float4 col : COLOR0; };

VSOut vs_main(VSIn i) {
    VSOut o;
    float4 w = mul(float4(i.pos, 1), world);
    o.pos = mul(mul(w, view), proj);
    o.pos.xy += half_pixel.xy * o.pos.w;
    o.wpos = w.xyz;
    o.nrm = mul(i.nrm, (float3x3)world);
    o.col = i.col * tint;
    return o;
}

float4 ps_main(VSOut i) : COLOR0 {
    float3 n = normalize(i.nrm);
    float diff = saturate(dot(n, normalize(float3(0.4, 1.0, 0.3)))) * 0.7 + 0.3;
    float3 cell = floor(i.wpos + 1e-3);
    float checker = frac((cell.x + cell.y + cell.z) * 0.5) * 2.0;
    float fog = saturate(length(i.wpos - cam_pos.xyz) / 150.0);
    float3 c = i.col.rgb * diff * (0.85 + 0.15 * checker);
    return float4(lerp(c, float3(0.55, 0.7, 0.9), fog * fog), 1);
}
)";

constexpr UINT kCameraRegs = sizeof(CameraCB) / 16;  // c0-c12
constexpr UINT kObjectReg = kCameraRegs;              // c13
constexpr UINT kObjectRegs = sizeof(ObjectCB) / 16;   // c13-c17
constexpr UINT kHalfPixelReg = kObjectReg + kObjectRegs;
static_assert(sizeof(CameraCB) % 16 == 0 && sizeof(ObjectCB) % 16 == 0);

struct Renderer {
    const Options& opt;
    App& app;
    ComPtr<IDirect3D9> d3d;
    ComPtr<IDirect3DDevice9> dev;
    D3DPRESENT_PARAMETERS pp{};
    ComPtr<IDirect3DSurface9> own_depth;  // --own-depth
    std::vector<uint32_t> up_indices;     // --up: 0, 1, 2, ...
    ComPtr<IDirect3DVertexBuffer9> vb;
    ComPtr<IDirect3DVertexDeclaration9> decl;
    ComPtr<IDirect3DVertexShader9> vs;
    ComPtr<IDirect3DPixelShader9> ps;
    Geometry geo;
    bool lost = false;

    Renderer(const Options& o, App& a) : opt(o), app(a) {}

    void create() {
        d3d.Attach(Direct3DCreate9(D3D_SDK_VERSION));
        if (!d3d) check(E_FAIL, "Direct3DCreate9");
        pp.BackBufferWidth = UINT(app.width);
        pp.BackBufferHeight = UINT(app.height);
        pp.BackBufferFormat = D3DFMT_X8R8G8B8;
        pp.BackBufferCount = 1;
        pp.SwapEffect = D3DSWAPEFFECT_DISCARD;
        pp.hDeviceWindow = app.hwnd;
        pp.Windowed = TRUE;
        pp.EnableAutoDepthStencil = !opt.own_depth;
        pp.AutoDepthStencilFormat = D3DFMT_D24S8;
        pp.PresentationInterval = D3DPRESENT_INTERVAL_ONE;
        check(d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, app.hwnd,
                                D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &pp, &dev),
              "CreateDevice");

        geo = build_geometry();
        const UINT bytes = UINT(geo.verts.size() * sizeof(Vertex));
        check(dev->CreateVertexBuffer(bytes, D3DUSAGE_WRITEONLY, 0, D3DPOOL_MANAGED, &vb, nullptr), "vertex buffer");
        void* p = nullptr;
        check(vb->Lock(0, bytes, &p, 0), "Lock");
        std::memcpy(p, geo.verts.data(), bytes);
        vb->Unlock();

        const D3DVERTEXELEMENT9 elems[] = {
            {0, 0, D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0},
            {0, 12, D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_NORMAL, 0},
            {0, 24, D3DDECLTYPE_UBYTE4N, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_COLOR, 0},  // RGBA bytes, like D3D11
            D3DDECL_END(),
        };
        check(dev->CreateVertexDeclaration(elems, &decl), "vertex declaration");
        auto vs_blob = compile_shader(kSceneHlsl9, "vs_main", "vs_3_0");
        auto ps_blob = compile_shader(kSceneHlsl9, "ps_main", "ps_3_0");
        check(dev->CreateVertexShader(static_cast<const DWORD*>(vs_blob->GetBufferPointer()), &vs), "vs");
        check(dev->CreatePixelShader(static_cast<const DWORD*>(ps_blob->GetBufferPointer()), &ps), "ps");
        create_depth();
    }

    // D3DPOOL_DEFAULT resources: released before Reset, recreated after.
    void create_depth() {
        if (!opt.own_depth) return;
        check(dev->CreateDepthStencilSurface(pp.BackBufferWidth, pp.BackBufferHeight, D3DFMT_D24S8,
                                             D3DMULTISAMPLE_NONE, 0, FALSE, &own_depth, nullptr),
              "CreateDepthStencilSurface");
    }

    void reset() {
        own_depth.Reset();
        pp.BackBufferWidth = UINT(app.width);
        pp.BackBufferHeight = UINT(app.height);
        const HRESULT hr = dev->Reset(&pp);
        lost = hr == D3DERR_DEVICELOST;
        if (!lost) check(hr, "Reset");
        if (!lost) create_depth();
    }

    void render(const CameraCB& cam, double t) {
        if (lost) {  // e.g. another app went fullscreen: wait until the device can be reset
            if (dev->TestCooperativeLevel() != D3DERR_DEVICENOTRESET) return;
            reset();
            if (lost) return;
        }
        const bool reversed = opt.depth != DepthMode::Standard;
        if (own_depth) dev->SetDepthStencilSurface(own_depth.Get());  // else the auto one stays bound
        dev->Clear(0, nullptr, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER | D3DCLEAR_STENCIL,
                   D3DCOLOR_COLORVALUE(0.55f, 0.7f, 0.9f, 1.0f), reversed ? 0.0f : 1.0f, 0);
        check(dev->BeginScene(), "BeginScene");
        dev->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE);
        dev->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
        dev->SetRenderState(D3DRS_ZFUNC, reversed ? D3DCMP_GREATER : D3DCMP_LESS);
        dev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
        dev->SetRenderState(D3DRS_LIGHTING, FALSE);
        dev->SetVertexDeclaration(decl.Get());
        dev->SetStreamSource(0, vb.Get(), 0, sizeof(Vertex));
        dev->SetVertexShader(vs.Get());
        dev->SetPixelShader(ps.Get());

        const CameraCB gpu = gpu_camera(opt, cam);
        dev->SetVertexShaderConstantF(0, reinterpret_cast<const float*>(&gpu), kCameraRegs);
        dev->SetPixelShaderConstantF(0, reinterpret_cast<const float*>(&gpu), kCameraRegs);
        const float half_pixel[4] = {-1.0f / float(pp.BackBufferWidth), 1.0f / float(pp.BackBufferHeight), 0, 0};
        dev->SetVertexShaderConstantF(kHalfPixelReg, half_pixel, 1);

        uint32_t n = 0;
        for (const DrawItem& d : scene_draws(opt, geo, cam, t)) {
            dev->SetVertexShaderConstantF(kObjectReg, reinterpret_cast<const float*>(&d.object), kObjectRegs);
            if (opt.layout == ConstantsLayout::ModelView)
                dev->SetVertexShaderConstantF(0, reinterpret_cast<const float*>(&d.model_view), 4);
            if (!opt.up) {
                dev->DrawPrimitive(D3DPT_TRIANGLELIST, d.first, d.count / 3);
            } else if (n++ % 2 == 0) {
                dev->DrawPrimitiveUP(D3DPT_TRIANGLELIST, d.count / 3, &geo.verts[d.first], sizeof(Vertex));
            } else {
                if (up_indices.size() < d.count)
                    for (uint32_t i = uint32_t(up_indices.size()); i < d.count; ++i) up_indices.push_back(i);
                dev->DrawIndexedPrimitiveUP(D3DPT_TRIANGLELIST, 0, d.count, d.count / 3, up_indices.data(),
                                            D3DFMT_INDEX32, &geo.verts[d.first], sizeof(Vertex));
            }
        }
        dev->EndScene();
        const HRESULT hr = dev->Present(nullptr, nullptr, nullptr, nullptr);
        lost = hr == D3DERR_DEVICELOST;
        if (!lost) check(hr, "Present");
    }
};

}  // namespace

int run_d3d9(const Options& opt) {
    App app;
    if (!app.create(L"fake_game", opt.width, opt.height, /*d3d11=*/false)) return 1;
    Renderer r(opt, app);
    r.create();

    LARGE_INTEGER qpf, t0, now;
    QueryPerformanceFrequency(&qpf);
    QueryPerformanceCounter(&t0);
    double last = 0, title_timer = 0;
    uint64_t frame_index = 0;
    CameraRig rig;

    while (app.pump()) {
        if (app.resized) r.reset();
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - t0.QuadPart) / double(qpf.QuadPart);
        const float dt = float(std::min(t - last, 0.1));
        last = t;
        if (opt.duration > 0 && t > opt.duration) break;

        rig.update(app, opt, dt);
        r.render(rig.constants(opt, float(app.width) / float(app.height)), t);
        ++frame_index;

        title_timer += dt;
        if (title_timer > 0.5) {
            title_timer = 0;
            const wchar_t* modes[] = {L"standard", L"reversed", L"reversed-infinite"};
            wchar_t buf[256];
            swprintf(buf, 256, L"fake_game D3D9 (%s)  [%s]  depth=%s  frame %llu",
                     opt.own_depth ? L"own depth surface" : L"auto depth", rig.manual ? L"manual" : L"auto",
                     modes[int(opt.depth)], frame_index);
            app.set_title(buf);
        }
    }
    return 0;
}

}  // namespace lidar::fake
