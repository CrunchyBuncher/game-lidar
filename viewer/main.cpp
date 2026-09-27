// lidar_viewer: live point-cloud viewer. Reads depth frames + camera matrices
// from the shared-memory ring, unprojects them on the GPU into a voxel-deduped
// point pool, and renders it with a free-fly camera. Points that a newer frame
// sees straight through (things that moved away) are carved out.
//
// Usage: lidar_viewer [--voxel 0.1] [--capacity-m 50] [--table-bits auto]
//                     [--near-cut 0.3] [--max-range 500] [--height-range -1 20]
//                     [--no-carve] [--carve-margin 0.15] [--carve-rel 0.02]
//                     [--color-update first|closest|latest]
//                     [--size 1600x900] [--out file.ply] [--save-after s] [--exit-after s]
// Keys:  right-drag look, WASD move, Q/E down/up, Shift fast, wheel speed,
//        F follow player (right-drag orbits it, wheel zooms, R resets), V attach to the player's camera, H color mode, T trail,
//        M carving, U hide above the player, O hide between you and the player,
//        +/- point size, arrows tilt the view, L level it, C clear,
//        P save .ply, Space pause ingest, F1 settings panel, Esc quit.
// The settings panel changes voxel size, capacity, range, carving and colors while it runs.
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
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "ring.h"
#include "shaders.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);

using namespace DirectX;
using namespace lidar;

namespace {

// Feeds the settings panel; keeps what it's using (typing, clicks, the wheel) from the viewer's
// own controls. Key and button releases always go through so nothing sticks.
bool ui_message(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp, LRESULT& result) {
    if ((result = ImGui_ImplWin32_WndProcHandler(hwnd, msg, wp, lp)) != 0) return true;
    const ImGuiIO& io = ImGui::GetIO();
    switch (msg) {
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
        case WM_CHAR: return io.WantCaptureKeyboard;
        case WM_LBUTTONDOWN:
        case WM_RBUTTONDOWN:
        case WM_MOUSEWHEEL: return io.WantCaptureMouse;
    }
    return false;
}

// Hash table slots for a pool: at least twice the points, so probing stays short.
uint32_t table_bits_for(uint32_t capacity) {
    uint32_t bits = 16;
    while (bits < 28 && (1ull << bits) < 2ull * capacity) ++bits;
    return bits;
}

// Inverse of the shaders' height_key().
float height_from_key(uint32_t key) {
    const uint32_t u = (key & 0x80000000u) ? (key & 0x7FFFFFFFu) : ~key;
    float f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

// A camera's position, forward and up from its inverse view and projection. Forward and up come
// from unprojecting the screen center and top edge, which holds whichever handedness the game
// uses. half_fov_y is the vertical half angle. False if the matrices are degenerate.
bool camera_axes(FXMMATRIX inv_view, CXMMATRIX inv_proj, XMVECTOR& apex, XMVECTOR& fwd, XMVECTOR& up,
                 float& half_fov_y) {
    const XMVECTOR vc = XMVector3TransformCoord(XMVectorSet(0, 0, 0.5f, 1), inv_proj);  // view space
    XMVECTOR vt = XMVector3TransformCoord(XMVectorSet(0, 1, 0.5f, 1), inv_proj);
    vt = XMVectorScale(vt, XMVectorGetZ(vc) / XMVectorGetZ(vt));  // the top edge at the center's depth
    apex = XMVector3TransformCoord(XMVectorZero(), inv_view);
    fwd = XMVector3Normalize(XMVector3TransformCoord(vc, inv_view) - apex);
    up = XMVector3Normalize(XMVector3TransformCoord(vt, inv_view) - XMVector3TransformCoord(vc, inv_view));
    half_fov_y = std::atan2(XMVectorGetY(vt), std::fabs(XMVectorGetZ(vc)));
    return !XMVector3IsNaN(fwd) && !XMVector3IsNaN(up) &&
           XMVectorGetX(XMVector3LengthSq(XMVector3Cross(fwd, up))) > 1e-6f;
}

struct Options {
    float voxel = 0.1f;
    uint32_t capacity = 50u << 20;
    uint32_t table_bits = 0;  // 0: sized for the capacity
    float near_cut = 0.3f;
    float max_range = 500.0f;
    float height_min = -1.0f, height_max = 20.0f;
    int width = 1600, height = 900;
    std::string out;
    float save_after = 0, exit_after = 0;
    bool carve = true;
    float carve_margin = 0.15f;  // meters
    float carve_rel = 0.02f;     // fraction of distance
    uint32_t color_update = 1;   // a point's color: 0 first sighting's, 1 closest's, 2 latest's
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
        else if (a == "--color-update") {
            const std::string m = next();
            o.color_update = m == "first" ? 0 : m == "latest" ? 2 : 1;
        }
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
    uint32_t carve_on, color_update;
};
struct DrawCB {
    XMFLOAT4X4 view_proj;
    float px_to_ndc[2];
    float point_size;
    uint32_t color_mode;
    float height_min, height_max;
    uint32_t capacity;
    float px_per_m;  // 0: points are point_size pixels; else world_size metres, clamped to 1..max_px pixels
    float height_axis[4];  // a point's height as shown: dot((pos, 1), height_axis), for the tilt
    float cut_plane[4];    // hides points where dot((pos, 1), cut_plane) > 0; (0, 0, 0, -1) is off
    float cut_a[3], cut_r2;  // line-of-sight cylinder from cut_a to cut_a + cut_ab; radius 0 is off
    float cut_ab[3], cut_inv_ab2;
    float cut_base[4];  // the cylinder hides only where dot((pos, 1), cut_base) > 0
    float world_size, max_px, pad[2];
};
struct HistCB {
    float height_axis[4];
    float lo, hi;
    uint32_t capacity, bins;
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

// GPU time between kMarks points in the frame, read back a few frames late without stalling.
// begin(), then mark(0..kMarks-1) in order, then end(), once a frame.
struct GpuTimer {
    static constexpr int kFrames = 6, kMarks = 4;
    ComPtr<ID3D11Query> disjoint[kFrames], stamps[kFrames][kMarks];
    int frame = 0;
    float ms[kMarks - 1] = {};  // smoothed time from each mark to the next

    void create(ID3D11Device* dev) {
        D3D11_QUERY_DESC d{D3D11_QUERY_TIMESTAMP_DISJOINT};
        for (auto& q : disjoint) check(dev->CreateQuery(&d, &q), "timer query");
        d.Query = D3D11_QUERY_TIMESTAMP;
        for (auto& f : stamps)
            for (auto& q : f) check(dev->CreateQuery(&d, &q), "timer query");
    }
    void begin(ID3D11DeviceContext* ctx) { ctx->Begin(disjoint[frame % kFrames].Get()); }
    void mark(ID3D11DeviceContext* ctx, int i) { ctx->End(stamps[frame % kFrames][i].Get()); }
    void end(ID3D11DeviceContext* ctx) {
        ctx->End(disjoint[frame % kFrames].Get());
        if (++frame < kFrames) return;
        const int f = frame % kFrames;  // the oldest, reused next frame: read it now or never
        D3D11_QUERY_DATA_TIMESTAMP_DISJOINT dj;
        if (ctx->GetData(disjoint[f].Get(), &dj, sizeof(dj), D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK || dj.Disjoint)
            return;
        UINT64 t[kMarks];
        for (int i = 0; i < kMarks; ++i)
            if (ctx->GetData(stamps[f][i].Get(), &t[i], sizeof(t[i]), D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK) return;
        for (int i = 0; i + 1 < kMarks; ++i) {
            const float now = float(double(t[i + 1] - t[i]) * 1000.0 / double(dj.Frequency));
            ms[i] += (now - ms[i]) * 0.1f;
        }
    }
};

// GPU point pool + voxel hash table.
struct PointCloud {
    uint32_t capacity = 0, table_size = 0;
    ComPtr<ID3D11Buffer> counter, table, points, free_list, args;
    ComPtr<ID3D11UnorderedAccessView> counter_uav, table_uav, points_uav, free_list_uav, args_uav;
    ComPtr<ID3D11ShaderResourceView> points_srv, counter_srv;

    // False if the GPU can't allocate it (e.g. a capacity set too high in the UI).
    bool create(ID3D11Device* dev, uint32_t cap, uint32_t table_bits) {
        capacity = cap;
        table_size = 1u << table_bits;

        D3D11_BUFFER_DESC d{};
        d.ByteWidth = 16;
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;  // the draw reads the count
        d.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
        if (FAILED(dev->CreateBuffer(&d, nullptr, &counter))) return false;
        D3D11_UNORDERED_ACCESS_VIEW_DESC u{DXGI_FORMAT_R32_TYPELESS, D3D11_UAV_DIMENSION_BUFFER};
        u.Buffer.NumElements = 4;
        u.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
        if (FAILED(dev->CreateUnorderedAccessView(counter.Get(), &u, &counter_uav))) return false;
        D3D11_SHADER_RESOURCE_VIEW_DESC sv{DXGI_FORMAT_R32_TYPELESS, D3D11_SRV_DIMENSION_BUFFEREX};
        sv.BufferEx.NumElements = 4;
        sv.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;
        if (FAILED(dev->CreateShaderResourceView(counter.Get(), &sv, &counter_srv))) return false;

        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        d.ByteWidth = table_size * 4;
        d.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
        d.StructureByteStride = 4;
        if (FAILED(dev->CreateBuffer(&d, nullptr, &table))) return false;
        if (FAILED(dev->CreateUnorderedAccessView(table.Get(), nullptr, &table_uav))) return false;

        d.ByteWidth = capacity * 4;
        if (FAILED(dev->CreateBuffer(&d, nullptr, &free_list))) return false;
        if (FAILED(dev->CreateUnorderedAccessView(free_list.Get(), nullptr, &free_list_uav))) return false;

        d.ByteWidth = capacity * sizeof(PointData);
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
        d.StructureByteStride = sizeof(PointData);
        if (FAILED(dev->CreateBuffer(&d, nullptr, &points))) return false;
        if (FAILED(dev->CreateUnorderedAccessView(points.Get(), nullptr, &points_uav))) return false;
        if (FAILED(dev->CreateShaderResourceView(points.Get(), nullptr, &points_srv))) return false;

        // [0..4] DrawIndexedInstancedIndirect, [5..7] DispatchIndirect (carving).
        d = {};
        d.ByteWidth = 32;
        d.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
        d.MiscFlags = D3D11_RESOURCE_MISC_DRAWINDIRECT_ARGS;
        if (FAILED(dev->CreateBuffer(&d, nullptr, &args))) return false;
        u = {DXGI_FORMAT_R32_UINT, D3D11_UAV_DIMENSION_BUFFER};
        u.Buffer.NumElements = 8;
        if (FAILED(dev->CreateUnorderedAccessView(args.Get(), &u, &args_uav))) return false;
        return true;
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
    Options opt = parse(argc, argv);  // the panel edits it live
    App app;
    if (!app.create(L"lidar_viewer", opt.width, opt.height)) return 1;
    ID3D11Device* dev = app.dev.Get();
    ID3D11DeviceContext* ctx = app.ctx.Get();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;  // nothing to remember between runs
    ImGui::StyleColorsDark();
    {
        const float dpi = ImGui_ImplWin32_GetDpiScaleForHwnd(app.hwnd);
        ImGui::GetStyle().ScaleAllSizes(dpi);
        ImGui::GetStyle().FontScaleDpi = dpi;
    }
    ImGui_ImplWin32_Init(app.hwnd);
    ImGui_ImplDX11_Init(dev, ctx);
    app.message_hook = ui_message;

    // Shaders.
    auto min_dist_blob = compile_shader(shaders::kCompute, "cs_min_dist", "cs_5_0");
    auto carve_blob = compile_shader(shaders::kCompute, "cs_carve", "cs_5_0");
    auto ingest_blob = compile_shader(shaders::kCompute, "cs_ingest", "cs_5_0");
    auto args_blob = compile_shader(shaders::kCompute, "cs_args", "cs_5_0");
    auto vsp_blob = compile_shader(shaders::kDraw, "vs_points", "vs_5_0");
    auto vsl_blob = compile_shader(shaders::kDraw, "vs_lines", "vs_5_0");
    auto ps_blob = compile_shader(shaders::kDraw, "ps_main", "ps_5_0");
    auto hist_blob = compile_shader(shaders::kHeightHist, "cs_height_hist", "cs_5_0");
    ComPtr<ID3D11ComputeShader> cs_min_dist, cs_carve, cs_ingest, cs_args, cs_hist;
    ComPtr<ID3D11VertexShader> vs_points, vs_lines;
    ComPtr<ID3D11PixelShader> ps;
    check(dev->CreateComputeShader(min_dist_blob->GetBufferPointer(), min_dist_blob->GetBufferSize(), nullptr,
                                   &cs_min_dist),
          "cs_min_dist");
    check(dev->CreateComputeShader(carve_blob->GetBufferPointer(), carve_blob->GetBufferSize(), nullptr, &cs_carve),
          "cs_carve");
    check(dev->CreateComputeShader(ingest_blob->GetBufferPointer(), ingest_blob->GetBufferSize(), nullptr,
                                   &cs_ingest),
          "cs_ingest");
    check(dev->CreateComputeShader(args_blob->GetBufferPointer(), args_blob->GetBufferSize(), nullptr, &cs_args),
          "cs_args");
    check(dev->CreateComputeShader(hist_blob->GetBufferPointer(), hist_blob->GetBufferSize(), nullptr, &cs_hist),
          "cs_height_hist");
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
    // Per-pixel nearest observed distance over the 3x3 neighborhood, for carving.
    ComPtr<ID3D11Texture2D> min_dist_tex;
    ComPtr<ID3D11ShaderResourceView> min_dist_srv;
    ComPtr<ID3D11UnorderedAccessView> min_dist_uav;
    {
        D3D11_TEXTURE2D_DESC d{};
        d.Width = kMaxWidth;
        d.Height = kMaxHeight;
        d.MipLevels = d.ArraySize = 1;
        d.SampleDesc.Count = 1;
        d.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
        d.Format = DXGI_FORMAT_R32_FLOAT;
        check(dev->CreateTexture2D(&d, nullptr, &min_dist_tex), "min dist tex");
        check(dev->CreateShaderResourceView(min_dist_tex.Get(), nullptr, &min_dist_srv), "min dist srv");
        check(dev->CreateUnorderedAccessView(min_dist_tex.Get(), nullptr, &min_dist_uav), "min dist uav");
    }

    PointCloud cloud;
    if (!cloud.create(dev, opt.capacity, opt.table_bits != 0 ? opt.table_bits : table_bits_for(opt.capacity))) {
        std::fprintf(stderr, "can't allocate a %u-point pool: try a smaller --capacity-m\n", opt.capacity);
        return 1;
    }
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
        ID3D11ShaderResourceView* null_srvs[3] = {};
        ID3D11UnorderedAccessView* null_uavs[6] = {};
        ctx->CSSetShaderResources(0, 3, null_srvs);
        ctx->CSSetUnorderedAccessViews(0, 6, null_uavs, nullptr);
    };
    auto update_args = [&]() {
        ctx->CSSetShader(cs_args.Get(), nullptr, 0);
        ctx->CSSetConstantBuffers(0, 1, frame_cb.GetAddressOf());
        bind_uavs(true);
        ctx->Dispatch(1, 1, 1);
        unbind_compute();
    };

    // Point quads, one batch's worth: 4 vertices each, drawn as two triangles. The draw repeats it
    // (one instance per batch) and the vertex shader finds the point from the instance and vertex ids.
    ComPtr<ID3D11Buffer> quad_ib;
    {
        std::vector<uint16_t> idx(size_t(shaders::kQuadsPerBatch) * 6);
        for (uint32_t q = 0; q < shaders::kQuadsPerBatch; ++q) {
            const uint16_t b = uint16_t(q * 4);
            const uint16_t tri[6] = {b, uint16_t(b + 1), uint16_t(b + 2), b, uint16_t(b + 2), uint16_t(b + 3)};
            std::memcpy(&idx[q * 6], tri, sizeof(tri));
        }
        D3D11_BUFFER_DESC d{UINT(idx.size() * sizeof(uint16_t)), D3D11_USAGE_IMMUTABLE, D3D11_BIND_INDEX_BUFFER};
        D3D11_SUBRESOURCE_DATA init{idx.data()};
        check(dev->CreateBuffer(&d, &init, &quad_ib), "quad ib");
    }

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
    int stat_frame = 0, stat_fresh_from = 0;   // copies made before stat_fresh_from predate a clear
    uint32_t point_count = 0, free_count = 0;  // slots ever allocated, slots freed by carving

    // Height histogram for the automatic color range, a few times a second, read back without
    // stalling. Coloring over the 1st to 99th percentile keeps a few stray points (sky, far
    // geometry) from squashing everything else into one end of the ramp.
    constexpr UINT kHistBytes = (2 + shaders::kHistBins) * 4;
    ComPtr<ID3D11Buffer> hist_buf, hist_stage;
    ComPtr<ID3D11UnorderedAccessView> hist_uav;
    auto hist_cb = make_cbuffer<HistCB>(dev);
    {
        D3D11_BUFFER_DESC d{kHistBytes, D3D11_USAGE_DEFAULT, D3D11_BIND_UNORDERED_ACCESS, 0,
                            D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS};
        check(dev->CreateBuffer(&d, nullptr, &hist_buf), "hist");
        D3D11_UNORDERED_ACCESS_VIEW_DESC u{DXGI_FORMAT_R32_TYPELESS, D3D11_UAV_DIMENSION_BUFFER};
        u.Buffer.NumElements = kHistBytes / 4;
        u.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
        check(dev->CreateUnorderedAccessView(hist_buf.Get(), &u, &hist_uav), "hist uav");
        d = {kHistBytes, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ};
        check(dev->CreateBuffer(&d, nullptr, &hist_stage), "hist staging");
    }
    bool hist_pending = false;       // a copy in hist_stage waiting to be read
    float hist_lo = 0, hist_hi = 0;  // the bins of the pending histogram
    double hist_timer = 0;
    float cloud_low = 0, cloud_high = 0;  // the cloud's height extremes, as shown (valid if have_bounds)
    float range_low = 0, range_high = 0;  // its 1st and 99th percentile heights (the auto color range)
    bool have_bounds = false;

    // Marks: 0 capture (carve + ingest) 1 ... 2 point draw 3.
    GpuTimer gpu_timer;
    gpu_timer.create(dev);

    RingReader ring;
    Frame frame;
    bool have_player = false;
    XMFLOAT4X4 player_inv_view{}, player_inv_proj{};
    std::vector<XMFLOAT3> trail;

    // Viewer camera: start above the level looking down at it.
    float cam_pos[3] = {0.0f, 45.0f, -75.0f};
    float cam_yaw = 0.0f, cam_pitch = -0.5f, cam_speed = 10.0f;
    // Follow camera: orbits the player's camera, from behind their heading by default. Right-drag
    // swings it around (yaw relative to the heading, so it still turns with them), the wheel zooms, R resets.
    constexpr float kFollowDist = 8.94f, kFollowPitch = -0.46f;  // 8 m back and 4 m up, looking at them
    float follow_yaw = 0.0f, follow_pitch = kFollowPitch, follow_dist = kFollowDist;
    float point_size = 2.0f;
    // Distance-scaled points: world_point_scale voxels wide, so they close up into surfaces up close
    // and shrink to single pixels far away.
    bool world_points = false;
    float world_point_scale = 1.5f, world_point_max_px = 64.0f;
    uint32_t color_mode = 0;
    bool follow = false, attach = false, show_trail = true, show_player_cam = true, paused = false, saved = false, carve = opt.carve;
    // Settings panel (F1). Voxel size and capacity only apply with the button: both rebuild the pool.
    bool show_ui = true, auto_height = true;
    // Display-only tilt (degrees about world X and Z) around a pivot, for leveling a tilted scan.
    float tilt_x = 0, tilt_z = 0;
    XMFLOAT3 tilt_pivot{0, 0, 0};
    // Display-only cutaways that clear the view of the player. The plane hides everything above the
    // player's camera plus an offset, level (as displayed) or square to the camera's up. The cylinder
    // hides everything within a radius of the line from the viewer's camera to the player's, down to
    // a base plane of the same orientation, which its own offset slides along that line.
    bool cut_plane_on = false, cut_plane_camera_up = false, cut_sight_on = false;
    float cut_plane_offset = 0.5f, cut_sight_radius = 1.5f, cut_sight_offset = 0.0f;
    float ui_voxel = opt.voxel, ui_capacity_m = float(opt.capacity) / float(1 << 20);
    std::string pool_message;

    auto clear_cloud = [&]() {
        cloud.clear(ctx);
        update_args();
        trail.clear();
        point_count = free_count = 0;
        have_bounds = false;
        hist_pending = false;  // counts the old points
        hist_timer = 1e9;      // measure the new ones right away
        stat_fresh_from = stat_frame + kStatStages;
    };
    // Rebuilds the pool with the panel's voxel size and capacity. If the GPU can't allocate that
    // much, the old size comes back.
    auto apply_pool = [&]() {
        const uint32_t old_capacity = cloud.capacity, old_bits = table_bits_for(old_capacity);
        const uint32_t capacity = uint32_t(double(ui_capacity_m) * (1 << 20));
        cloud = PointCloud();
        ctx->Flush();
        if (cloud.create(dev, capacity, table_bits_for(capacity))) {
            pool_message.clear();
        } else {
            cloud = PointCloud();
            pool_message = "Couldn't allocate " + std::to_string(int(ui_capacity_m)) + "M points: kept the old size.";
            if (!cloud.create(dev, old_capacity, old_bits)) check(E_OUTOFMEMORY, "recreating the point pool");
            ui_capacity_m = float(old_capacity) / float(1 << 20);
        }
        opt.voxel = ui_voxel;
        opt.capacity = cloud.capacity;
        FrameCB cb{};  // cs_args reads the capacity from it
        cb.capacity = cloud.capacity;
        upload(ctx, frame_cb.Get(), cb);
        clear_cloud();
    };

    LARGE_INTEGER qpf, t0, now;
    QueryPerformanceFrequency(&qpf);
    QueryPerformanceCounter(&t0);
    double last = 0, title_timer = 0, reconnect_timer = 0;
    double last_posed = -1;  // when the last posed frame came in
    uint64_t ingested = 0;
    uint64_t seen_clear = 0;  // the producer's last clear request handled
    int fps_frames = 0;
    double fps = 0;

    while (app.pump()) {
        if (app.resized) create_zbuf();
        QueryPerformanceCounter(&now);
        const double t = double(now.QuadPart - t0.QuadPart) / double(qpf.QuadPart);
        const float dt = float(std::min(t - last, 0.1));
        last = t;

        gpu_timer.begin(ctx);
        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Input.
        if (app.key_pressed(VK_F1)) show_ui = !show_ui;
        if (app.key_pressed('F')) {
            follow = !follow;
            if (follow) attach = false;
        }
        if (app.key_pressed('V')) {
            attach = !attach;
            if (attach) follow = false;
        }
        if (app.key_pressed('H')) color_mode ^= 1;
        if (app.key_pressed('T')) show_trail = !show_trail;
        if (app.key_pressed('M')) carve = !carve;
        if (app.key_pressed('U')) cut_plane_on = !cut_plane_on;
        if (app.key_pressed('O')) cut_sight_on = !cut_sight_on;
        if (app.key_pressed(VK_SPACE)) paused = !paused;
        if (app.key_pressed(VK_OEM_PLUS) || app.key_pressed(VK_ADD)) point_size = std::min(point_size + 1, 16.0f);
        if (app.key_pressed(VK_OEM_MINUS) || app.key_pressed(VK_SUBTRACT))
            point_size = std::max(point_size - 1, 1.0f);
        if (app.key_pressed('C')) clear_cloud();
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
        // The producer asked for a clear (its pose's frame changed). A restarted producer starts at 0:
        // that keeps the points.
        if (const uint64_t cs = ring.clear_seq(); cs != seen_clear) {
            if (cs != 0) clear_cloud();
            seen_clear = cs;
        }
        // Carving (and the color refresh that shares its pass) is a pass over the whole pool, so it
        // runs for one frame per viewer frame at most: when the viewer falls behind, a backlog of
        // frames doesn't multiply its cost.
        bool carved = false;
        gpu_timer.mark(ctx, 0);
        for (uint32_t i = 0; i < kSlotCount && ring.is_open() && ring.read_next(frame); ++i) {
            const FrameHeader& h = frame.header;
            if (h.flags & kFlagPaused) continue;
            if (frame.seq <= seen_clear) continue;  // posed before the clear
            // Without a pose (addon before M2) the frame is camera-relative: show it as a live
            // snapshot at the origin instead of accumulating it.
            const bool posed = (h.flags & kFlagPoseValid) != 0;
            // An unposed frame among posed ones is a camera that failed for a moment: skip it rather
            // than start over.
            if (posed) last_posed = t;
            else if (last_posed >= 0 && t - last_posed < 2.0) continue;
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
            cb.carve_on = carve;
            cb.color_update = opt.color_update;
            upload(ctx, frame_cb.Get(), cb);

            ID3D11ShaderResourceView* srvs[2] = {depth_srv.Get(), color_srv.Get()};
            ctx->CSSetConstantBuffers(0, 1, frame_cb.GetAddressOf());
            ctx->CSSetShaderResources(0, 2, srvs);
            // Carve first, so this frame's own points aren't tested against itself
            // and freed slots can be reused right away.
            if ((carve || has_color) && !carved) {  // has_color: the color refresh (even first-seen fills in)
                carved = true;
                ctx->CSSetShader(cs_min_dist.Get(), nullptr, 0);
                ctx->CSSetUnorderedAccessViews(5, 1, min_dist_uav.GetAddressOf(), nullptr);
                ctx->Dispatch((h.width + 7) / 8, (h.height + 7) / 8, 1);
                ID3D11UnorderedAccessView* null_uav = nullptr;
                ctx->CSSetUnorderedAccessViews(5, 1, &null_uav, nullptr);
                ctx->CSSetShaderResources(2, 1, min_dist_srv.GetAddressOf());
                bind_uavs(false);
                ctx->CSSetShader(cs_carve.Get(), nullptr, 0);
                ctx->DispatchIndirect(cloud.args.Get(), 20);
            }
            bind_uavs(false);
            ctx->CSSetShader(cs_ingest.Get(), nullptr, 0);
            ctx->Dispatch((h.width + 7) / 8, (h.height + 7) / 8, 1);
            unbind_compute();
            update_args();
            ++ingested;
        }
        gpu_timer.mark(ctx, 1);

        // Stats readback (a couple of frames late, never stalls).
        ctx->CopyResource(stat_stage[stat_frame % kStatStages].Get(), cloud.counter.Get());
        if (stat_frame >= kStatStages - 1 && stat_frame >= stat_fresh_from) {
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

        // Settings panel.
        const uint32_t used = std::min(point_count, cloud.capacity);
        const uint32_t live = used - std::min(free_count, used);
        const bool full = point_count >= cloud.capacity && free_count == 0;
        if (show_ui) {
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::Begin("lidar_viewer (F1 hides)", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("%s, %llu frames (%llu dropped)", ring.is_open() ? "Connected" : "Waiting for the game",
                        (unsigned long long)ingested, (unsigned long long)ring.dropped());
            char overlay[64];
            std::snprintf(overlay, sizeof(overlay), "%.2fM / %.1fM points%s", live / 1e6, cloud.capacity / 1e6,
                          full ? " (FULL)" : "");
            ImGui::ProgressBar(float(used) / float(cloud.capacity), ImVec2(-FLT_MIN, 0), overlay);
            ImGui::TextDisabled("%.0f fps, GPU: capture %.2f ms, points %.2f ms", fps, gpu_timer.ms[0],
                                gpu_timer.ms[2]);
            if (ImGui::Button("Clear points (C)")) clear_cloud();
            ImGui::SameLine();
            if (ImGui::Button("Save .ply (P)")) save_ply(dev, ctx, cloud, opt.out.empty() ? default_scan_path() : opt.out);
            ImGui::SameLine();
            ImGui::Checkbox("Pause (Space)", &paused);

            ImGui::SeparatorText("Point pool");
            ImGui::SetNextItemWidth(160);
            ImGui::InputFloat("Voxel size (m)", &ui_voxel, 0.01f, 0.1f, "%.3f");
            ui_voxel = std::clamp(ui_voxel, 0.001f, 100.0f);
            ImGui::SetNextItemWidth(160);
            ImGui::InputFloat("Capacity (M points)", &ui_capacity_m, 4, 16, "%.0f");
            ui_capacity_m = std::clamp(std::round(ui_capacity_m), 1.0f, 128.0f);
            const uint32_t ui_capacity = uint32_t(double(ui_capacity_m) * (1 << 20));
            ImGui::TextDisabled("%.0f MB of GPU memory",
                                (double(ui_capacity) * (sizeof(PointData) + 4) +
                                 double(1ull << table_bits_for(ui_capacity)) * 4) / (1 << 20));
            const bool pool_changed = ui_voxel != opt.voxel || ui_capacity != cloud.capacity;
            ImGui::BeginDisabled(!pool_changed);
            if (ImGui::Button("Apply (clears points)")) apply_pool();
            ImGui::SameLine();
            if (ImGui::Button("Revert")) {
                ui_voxel = opt.voxel;
                ui_capacity_m = float(cloud.capacity) / float(1 << 20);
            }
            ImGui::EndDisabled();
            if (!pool_message.empty()) ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "%s", pool_message.c_str());

            ImGui::SeparatorText("Capture");
            ImGui::SetNextItemWidth(160);
            ImGui::SliderFloat("Max range (m)", &opt.max_range, 10, 20000, "%.0f", ImGuiSliderFlags_Logarithmic);
            ImGui::SetNextItemWidth(160);
            ImGui::SliderFloat("Near cut (m)", &opt.near_cut, 0, 10, "%.2f");
            ImGui::Checkbox("Carve out moved things (M)", &carve);
            if (carve) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Carve margin (m)", &opt.carve_margin, 0.01f, 5, "%.2f", ImGuiSliderFlags_Logarithmic);
            }

            ImGui::SeparatorText("Color (H)");
            int mode = int(color_mode);
            ImGui::RadioButton("Height", &mode, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Captured color", &mode, 1);
            color_mode = uint32_t(mode);
            int update = int(opt.color_update);
            ImGui::TextUnformatted("Keep the color");
            ImGui::SameLine();
            ImGui::RadioButton("first seen", &update, 0);
            ImGui::SameLine();
            ImGui::RadioButton("seen closest", &update, 1);
            ImGui::SameLine();
            ImGui::RadioButton("seen last", &update, 2);
            opt.color_update = uint32_t(update);
            ImGui::Checkbox("Height range from the cloud", &auto_height);
            if (auto_height) {
                if (have_bounds)
                    ImGui::TextDisabled("coloring %.1f to %.1f m (lowest %.1f, highest %.1f)", range_low,
                                        range_high, cloud_low, cloud_high);
                else
                    ImGui::TextDisabled("(no points yet)");
            } else {
                ImGui::SetNextItemWidth(220);
                ImGui::DragFloatRange2("Height range (m)", &opt.height_min, &opt.height_max, 0.5f, -100000, 100000,
                                       "%.1f", "%.1f");
                if (have_bounds && ImGui::Button("Set to the cloud's")) {
                    opt.height_min = range_low;
                    opt.height_max = range_high;
                }
            }

            ImGui::SeparatorText("View");
            ImGui::SetNextItemWidth(160);
            ImGui::Checkbox("Scale points with distance", &world_points);
            ImGui::SetNextItemWidth(160);
            if (world_points) {
                ImGui::SliderFloat("Point size (x voxel)", &world_point_scale, 0.25f, 4.0f, "%.2f");
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Max point size (px)", &world_point_max_px, 2, 256, "%.0f",
                                   ImGuiSliderFlags_Logarithmic);
            } else {
                ImGui::SliderFloat("Point size (+/-)", &point_size, 1, 16, "%.0f");
            }
            ImGui::Checkbox("Follow the player (F)", &follow);
            ImGui::SameLine();
            ImGui::Checkbox("Trail (T)", &show_trail);
            ImGui::SameLine();
            ImGui::Checkbox("Show game camera", &show_player_cam);
            ImGui::SameLine();
            if (ImGui::Checkbox("Attach to camera (V)", &attach) && attach) follow = false;
            if (follow) attach = false;
            if (follow)
                ImGui::TextDisabled("Right-drag orbit the player, wheel zoom (%.1f m), R reset", follow_dist);
            else
                ImGui::TextDisabled("Right-drag look, WASD/arrows move, Q/E down/up, Shift fast, wheel speed");
            ImGui::TextDisabled("Z/X roll, PgUp/PgDn tilt the view, L levels it (display only)");

            ImGui::SeparatorText("Clear the view (display only)");
            ImGui::Checkbox("Hide above the player (U)", &cut_plane_on);
            if (cut_plane_on) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Offset above camera (m)", &cut_plane_offset, -5, 20, "%.2f");
            }
            ImGui::Checkbox("Hide between you and the player (O)", &cut_sight_on);
            if (cut_sight_on) {
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Radius (m)", &cut_sight_radius, 0.1f, 20, "%.2f", ImGuiSliderFlags_Logarithmic);
                ImGui::SetNextItemWidth(160);
                ImGui::SliderFloat("Offset toward you (m)", &cut_sight_offset, -5, 20, "%.2f");
                if (attach) ImGui::TextDisabled("(off while attached to the camera)");
            }
            if (cut_plane_on || cut_sight_on) {  // the orientation of both bases
                int up = cut_plane_camera_up ? 1 : 0;
                ImGui::TextUnformatted("Base");
                ImGui::SameLine();
                ImGui::RadioButton("level", &up, 0);
                ImGui::SameLine();
                ImGui::RadioButton("follows camera's up", &up, 1);
                cut_plane_camera_up = up == 1;
            }
            ImGui::End();
        }

        // Tilt: PgUp/PgDn tilt about world X, Z/X roll about world Z, Shift faster, L levels. Display only (.ply files
        // keep the capture's frame). It pivots about where the player was when it left level, so the
        // scene doesn't swing away.
        {
            const bool was_level = tilt_x == 0 && tilt_z == 0;
            const float rate = (app.key_down(VK_SHIFT) ? 40.0f : 10.0f) * dt;
            if (app.key_down(VK_PRIOR)) tilt_x += rate;
            if (app.key_down(VK_NEXT)) tilt_x -= rate;
            if (app.key_down('Z')) tilt_z += rate;
            if (app.key_down('X')) tilt_z -= rate;
            // No limit: a capture can come in on its side or upside down. Just keep the angles in range.
            tilt_x = std::remainder(tilt_x, 360.0f);
            tilt_z = std::remainder(tilt_z, 360.0f);
            if (app.key_pressed('L')) tilt_x = tilt_z = 0;
            if (was_level && (tilt_x != 0 || tilt_z != 0))
                tilt_pivot = have_player ? XMFLOAT3{player_inv_view._41, player_inv_view._42, player_inv_view._43}
                                         : XMFLOAT3{cam_pos[0], cam_pos[1], cam_pos[2]};
        }
        const XMMATRIX tilt =
            XMMatrixTranslation(-tilt_pivot.x, -tilt_pivot.y, -tilt_pivot.z) *
            XMMatrixRotationX(XMConvertToRadians(tilt_x)) * XMMatrixRotationZ(XMConvertToRadians(tilt_z)) *
            XMMatrixTranslation(tilt_pivot.x, tilt_pivot.y, tilt_pivot.z);

        // Viewer camera.
        float fov_y = XMConvertToRadians(60.0f);
        bool attached = false;
        XMVECTOR attach_fwd{}, attach_up{};
        if (attach && have_player) {
            // Look through the player's camera (as displayed, so tilted) with its full orientation,
            // roll included, and its field of view. Detaching leaves the free-fly camera where the
            // player's was.
            XMVECTOR apex;
            float half;
            attached = camera_axes(XMLoadFloat4x4(&player_inv_view) * tilt, XMLoadFloat4x4(&player_inv_proj), apex,
                                   attach_fwd, attach_up, half);
            if (attached) {
                XMFLOAT3 f, p;
                XMStoreFloat3(&f, attach_fwd);
                XMStoreFloat3(&p, apex);
                cam_pos[0] = p.x, cam_pos[1] = p.y, cam_pos[2] = p.z;
                cam_yaw = std::atan2(f.x, f.z);
                cam_pitch = std::clamp(std::asin(std::clamp(f.y, -1.0f, 1.0f)), -1.55f, 1.55f);
                if (std::isfinite(half) && half > 0.05f && half < 1.4f) fov_y = 2.0f * half;
            }
        } else if (follow && have_player) {
            XMFLOAT4X4 shown;  // the player's camera as displayed (tilted)
            XMStoreFloat4x4(&shown, XMLoadFloat4x4(&player_inv_view) * tilt);
            const XMFLOAT3 p{shown._41, shown._42, shown._43};
            if (app.rmb_down()) {
                follow_yaw = std::remainder(follow_yaw + app.mouse_dx * 0.003f, XM_2PI);
                follow_pitch = std::clamp(follow_pitch - app.mouse_dy * 0.003f, -1.55f, 1.55f);
            }
            follow_dist = std::clamp(follow_dist * std::pow(1.0f / 1.2f, app.wheel), 0.5f, 2000.0f);
            if (app.key_pressed('R')) {
                follow_yaw = 0;
                follow_pitch = kFollowPitch;
                follow_dist = kFollowDist;
            }
            cam_yaw = std::atan2(shown._31, shown._33) + follow_yaw;
            cam_pitch = follow_pitch;
            const float cp = std::cos(cam_pitch);
            cam_pos[0] = p.x - std::sin(cam_yaw) * cp * follow_dist;
            cam_pos[1] = p.y - std::sin(cam_pitch) * follow_dist;
            cam_pos[2] = p.z - std::cos(cam_yaw) * cp * follow_dist;
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
                if (app.key_down('W') || app.key_down(VK_UP)) cam_pos[k] += f[k] * speed;
                if (app.key_down('S') || app.key_down(VK_DOWN)) cam_pos[k] -= f[k] * speed;
                if (app.key_down('D') || app.key_down(VK_RIGHT)) cam_pos[k] += r[k] * speed;
                if (app.key_down('A') || app.key_down(VK_LEFT)) cam_pos[k] -= r[k] * speed;
            }
            if (app.key_down('E')) cam_pos[1] += speed;
            if (app.key_down('Q')) cam_pos[1] -= speed;
        }
        const float cp = std::cos(cam_pitch);
        const XMMATRIX view =
            attached ? XMMatrixLookToLH(XMVectorSet(cam_pos[0], cam_pos[1], cam_pos[2], 1), attach_fwd, attach_up)
                     : XMMatrixLookToLH(XMVectorSet(cam_pos[0], cam_pos[1], cam_pos[2], 1),
                                        XMVectorSet(std::sin(cam_yaw) * cp, std::sin(cam_pitch), std::cos(cam_yaw) * cp, 0),
                                        XMVectorSet(0, 1, 0, 0));
        const XMMATRIX proj = XMMatrixPerspectiveFovLH(fov_y,
                                                       float(app.width) / float(app.height), 10000.0f, 0.05f);

        DrawCB dcb{};
        XMStoreFloat4x4(&dcb.view_proj, tilt * view * proj);
        {
            XMFLOAT4X4 tm;  // the tilted y: the matrix's second column
            XMStoreFloat4x4(&tm, tilt);
            dcb.height_axis[0] = tm._12, dcb.height_axis[1] = tm._22, dcb.height_axis[2] = tm._32;
            dcb.height_axis[3] = tm._42;
        }
        // Cutaways, in the capture's frame (the points before the tilt).
        dcb.cut_plane[3] = -1;  // off: never hides
        if (have_player && (cut_plane_on || (cut_sight_on && !attach))) {
            // Both are planes, level (up as displayed: the tilted y, which the tilt levels) or square to
            // the camera's own up, through their base point.
            XMVECTOR n = XMVectorSet(dcb.height_axis[0], dcb.height_axis[1], dcb.height_axis[2], 0);
            {
                XMVECTOR apex, fwd, up;
                float half;
                if (cut_plane_camera_up &&
                    camera_axes(XMLoadFloat4x4(&player_inv_view), XMLoadFloat4x4(&player_inv_proj), apex, fwd, up, half))
                    n = up;
            }
            const XMVECTOR player = XMVectorSet(player_inv_view._41, player_inv_view._42, player_inv_view._43, 1);
            auto plane_through = [&](FXMVECTOR p) {  // positive above p
                XMFLOAT4 f;
                XMStoreFloat4(&f, XMVectorSetW(n, -XMVectorGetX(XMVector3Dot(n, p))));
                return f;
            };
            if (cut_plane_on) {  // the player's camera raised by its offset
                const XMFLOAT4 plane = plane_through(player + XMVectorScale(n, cut_plane_offset));
                std::memcpy(dcb.cut_plane, &plane, sizeof(dcb.cut_plane));
            }
            if (cut_sight_on && !attach) {  // attached, the line has no length: it would just hide what's near
                // From the base up to the viewer, hiding only on the viewer's side of the base (above it,
                // unless the viewer is below), so the floor under the player stays. The base starts at
                // the player's camera moved along the line by its offset: toward the viewer, or past
                // the player when negative. It stops short of the viewer.
                const XMVECTOR eye = XMVector3TransformCoord(XMVectorSet(cam_pos[0], cam_pos[1], cam_pos[2], 1),
                                                             XMMatrixInverse(nullptr, tilt));
                const float dist = XMVectorGetX(XMVector3Length(eye - player));
                const XMVECTOR base =
                    player + XMVectorScale(eye - player, std::min(cut_sight_offset, 0.9f * dist) / std::max(dist, 1e-6f));
                const XMFLOAT4 plane = plane_through(base);
                const XMVECTOR ab = eye - base;
                XMFLOAT3 a, abf;
                XMStoreFloat3(&a, base);
                XMStoreFloat3(&abf, ab);
                std::memcpy(dcb.cut_a, &a, sizeof(dcb.cut_a));
                std::memcpy(dcb.cut_ab, &abf, sizeof(dcb.cut_ab));
                dcb.cut_r2 = cut_sight_radius * cut_sight_radius;
                dcb.cut_inv_ab2 = 1.0f / std::max(XMVectorGetX(XMVector3LengthSq(ab)), 1e-8f);
                const float side = XMVectorGetX(XMVector3Dot(n, ab)) < 0 ? -1.0f : 1.0f;
                dcb.cut_base[0] = side * plane.x, dcb.cut_base[1] = side * plane.y;
                dcb.cut_base[2] = side * plane.z, dcb.cut_base[3] = side * plane.w;
            }
        }
        dcb.px_to_ndc[0] = 2.0f / float(app.width);
        dcb.px_to_ndc[1] = 2.0f / float(app.height);
        dcb.point_size = point_size;
        if (world_points) {
            dcb.px_per_m = 0.5f * float(app.height) / std::tan(0.5f * fov_y);
            dcb.world_size = world_point_scale * opt.voxel;
            dcb.max_px = world_point_max_px;
        }
        dcb.color_mode = color_mode;
        dcb.capacity = cloud.capacity;
        dcb.height_min = auto_height && have_bounds ? range_low : opt.height_min;
        dcb.height_max = auto_height && have_bounds ? range_high : opt.height_max;
        if (!(dcb.height_max - dcb.height_min > 0.01f)) dcb.height_max = dcb.height_min + 0.01f;  // flat cloud
        upload(ctx, draw_cb.Get(), dcb);

        // Auto color range: read the last height histogram if it's done, then start the next one.
        if (hist_pending) {
            D3D11_MAPPED_SUBRESOURCE m;
            if (ctx->Map(hist_stage.Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &m) == S_OK) {
                const uint32_t* h = static_cast<const uint32_t*>(m.pData);
                const uint32_t* bins = h + 2;
                hist_pending = false;
                have_bounds = h[1] != 0;
                if (have_bounds) {
                    cloud_low = height_from_key(~h[0]);
                    cloud_high = height_from_key(h[1]);
                    range_low = cloud_low, range_high = cloud_high;
                    if (hist_hi > hist_lo) {
                        uint64_t total = 0;
                        for (uint32_t i = 0; i < shaders::kHistBins; ++i) total += bins[i];
                        const float bin_size = (hist_hi - hist_lo) / float(shaders::kHistBins);
                        // The height below which a fraction q of the points lie, interpolated in its bin.
                        auto percentile = [&](double q) {
                            const double target = q * double(total);
                            double below = 0;
                            for (uint32_t i = 0; i < shaders::kHistBins; ++i) {
                                if (below + bins[i] >= target && bins[i] > 0)
                                    return hist_lo + (float(i) + float((target - below) / bins[i])) * bin_size;
                                below += bins[i];
                            }
                            return hist_hi;
                        };
                        range_low = std::clamp(percentile(0.01), cloud_low, cloud_high);
                        range_high = std::clamp(percentile(0.99), cloud_low, cloud_high);
                    } else {
                        hist_timer = 1e9;  // that one only found the extremes: bin over them next
                    }
                }
                ctx->Unmap(hist_stage.Get(), 0);
            }
        }
        hist_timer += dt;
        if (!hist_pending && hist_timer > 0.25) {
            hist_timer = 0;
            // Bin over the extremes last measured. The tilt or new points can move them; the next
            // histogram catches up.
            hist_lo = have_bounds ? cloud_low : 0;
            hist_hi = have_bounds ? cloud_high : 0;
            HistCB hcb{};
            std::memcpy(hcb.height_axis, dcb.height_axis, sizeof(hcb.height_axis));
            hcb.lo = hist_lo;
            hcb.hi = hist_hi;
            hcb.capacity = cloud.capacity;
            hcb.bins = shaders::kHistBins;
            upload(ctx, hist_cb.Get(), hcb);
            const UINT zero[4] = {};
            ctx->ClearUnorderedAccessViewUint(hist_uav.Get(), zero);
            ID3D11UnorderedAccessView* uavs[2] = {hist_uav.Get(), cloud.counter_uav.Get()};
            ctx->CSSetShader(cs_hist.Get(), nullptr, 0);
            ctx->CSSetConstantBuffers(0, 1, hist_cb.GetAddressOf());
            ctx->CSSetShaderResources(0, 1, cloud.points_srv.GetAddressOf());
            ctx->CSSetUnorderedAccessViews(0, 2, uavs, nullptr);
            ctx->DispatchIndirect(cloud.args.Get(), 20);  // one thread per slot, like carving
            unbind_compute();
            ctx->CopyResource(hist_stage.Get(), hist_buf.Get());
            hist_pending = true;
        }

        // Lines: player frustum + trail.
        std::vector<LineVertex> lines;
        if (show_player_cam && have_player && !attach) {  // attached, it would just frame the screen
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

        gpu_timer.mark(ctx, 2);
        ctx->IASetInputLayout(nullptr);
        ctx->IASetIndexBuffer(quad_ib.Get(), DXGI_FORMAT_R16_UINT, 0);
        ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx->VSSetShader(vs_points.Get(), nullptr, 0);
        ID3D11ShaderResourceView* point_srvs[2] = {cloud.points_srv.Get(), cloud.counter_srv.Get()};
        ctx->VSSetShaderResources(0, 2, point_srvs);
        ctx->DrawIndexedInstancedIndirect(cloud.args.Get(), 0);
        ID3D11ShaderResourceView* null_srvs[2] = {};
        ctx->VSSetShaderResources(0, 2, null_srvs);
        gpu_timer.mark(ctx, 3);

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
        ImGui::Render();
        ctx->OMSetRenderTargets(1, app.back_rtv.GetAddressOf(), nullptr);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        gpu_timer.end(ctx);
        app.present(true);

        ++fps_frames;
        title_timer += dt;
        if (title_timer > 0.5) {
            fps = fps_frames / title_timer;
            fps_frames = 0;
            title_timer = 0;
            wchar_t buf[256];
            swprintf(buf, 256,
                     L"lidar_viewer  %.0f fps  |  %s  |  frames %llu (dropped %llu)  |  points %.2fM / %.1fM%s  |  "
                     L"carving %s%s",
                     fps, ring.is_open() ? L"connected" : L"waiting for producer", ingested, ring.dropped(),
                     live / 1e6, cloud.capacity / 1e6, full ? L" FULL" : L"",
                     carve ? L"on" : L"off", paused ? L"  PAUSED" : L"");
            app.set_title(buf);
        }
    }
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    return 0;
}
