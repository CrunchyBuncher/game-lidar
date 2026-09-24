// Unit tests: ring buffer semantics, unprojection across depth conventions, the addon's camera
// profiles (parsing, matrix layouts, projection analysis) and D3D12 root signature lookups.
#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "d3d12/root_layout.h"
#include "profile.h"
#include "ring.h"
#include "unproject.h"

using namespace DirectX;
using namespace lidar;

static int g_failures = 0;
#define EXPECT(cond)                                                    \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                               \
        }                                                               \
    } while (0)

static void write_frame(RingWriter& w, uint64_t index) {
    Slot* s = w.begin_frame();
    s->frame = {};
    s->frame.frame_index = index;
    s->frame.width = 4;
    s->frame.height = 2;
    s->frame.flags = kFlagPoseValid;
    for (int i = 0; i < 8; ++i) s->depth[i] = float(index) + i * 0.1f;
    w.commit();
}

static void test_ring() {
    const wchar_t* name = L"Local\\game_lidar_test_ring";
    RingWriter w;
    EXPECT(w.open(name));
    RingReader r;
    EXPECT(r.try_open(name));

    Frame f;
    EXPECT(!r.read_next(f));  // empty

    for (uint64_t i = 1; i <= 10; ++i) write_frame(w, i);
    // Only the last kSlotCount frames survive, delivered oldest first.
    for (uint64_t expect = 10 - kSlotCount + 1; expect <= 10; ++expect) {
        EXPECT(r.read_next(f));
        EXPECT(f.header.frame_index == expect);
        EXPECT(f.depth.size() == 8 && f.depth[7] == float(expect) + 0.7f);
    }
    EXPECT(!r.read_next(f));

    write_frame(w, 11);
    EXPECT(r.read_next(f) && f.header.frame_index == 11);

    // Reader falls behind by more than the ring: skipped frames count as dropped.
    const uint64_t dropped_before = r.dropped();
    for (uint64_t i = 12; i <= 20; ++i) write_frame(w, i);
    uint64_t first = 0;
    int got = 0;
    while (r.read_next(f)) {
        if (!got) first = f.header.frame_index;
        ++got;
    }
    EXPECT(got == int(kSlotCount) && first == 20 - kSlotCount + 1);
    EXPECT(r.dropped() - dropped_before == 9 - kSlotCount);

    // Producer restart: reader resyncs to the new session.
    EXPECT(w.open(name));
    write_frame(w, 1);
    EXPECT(r.read_next(f) && f.header.frame_index == 1);
}

static XMMATRIX make_proj(int mode, float fov, float aspect) {
    const float n = 0.1f, fa = 1000.0f;
    if (mode == 0) return XMMatrixPerspectiveFovLH(fov, aspect, n, fa);
    if (mode == 1) return XMMatrixPerspectiveFovLH(fov, aspect, fa, n);
    const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
    return XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, n, 0);
}

static void test_unproject() {
    const XMMATRIX view = XMMatrixLookToLH(XMVectorSet(3, 1.7f, -8, 1), XMVectorSet(0.3f, -0.1f, 1, 0),
                                           XMVectorSet(0, 1, 0, 0));
    for (int mode = 0; mode < 3; ++mode) {
        const XMMATRIX proj = make_proj(mode, XMConvertToRadians(70.0f), 16.0f / 9.0f);
        const XMMATRIX view_proj = view * proj;
        FrameHeader h{};
        h.width = 480;
        h.height = 270;
        h.src_width = 1280;
        h.src_height = 720;
        XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(h.view), view);
        XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(h.proj), proj);
        const Unprojector up(h);
        const XMMATRIX inv_view = XMMatrixInverse(nullptr, view);
        const XMMATRIX inv_proj = XMMatrixInverse(nullptr, proj);

        const uint32_t pixels[][2] = {{0, 0}, {479, 269}, {240, 135}, {17, 200}};
        for (const auto& px : pixels) {
            for (float dist : {0.5f, 12.3f, 180.0f}) {
                // Build the true world point along this pixel's ray at view depth `dist`.
                float nx, ny;
                up.ndc(px[0], px[1], nx, ny);
                XMVECTOR dir = XMVector3TransformCoord(XMVectorSet(nx, ny, 0.5f, 1), inv_proj);
                dir = XMVectorScale(dir, dist / XMVectorGetZ(dir));
                const XMVECTOR world = XMVector3TransformCoord(dir, inv_view);
                const float z = XMVectorGetZ(XMVector3TransformCoord(world, view_proj));

                float p[3];
                const bool ok = up.unproject(px[0], px[1], z, 0.3f, 500.0f, p);
                if (dist < 0.3f) {
                    EXPECT(!ok);
                    continue;
                }
                EXPECT(ok);
                const float err = std::sqrt((p[0] - XMVectorGetX(world)) * (p[0] - XMVectorGetX(world)) +
                                            (p[1] - XMVectorGetY(world)) * (p[1] - XMVectorGetY(world)) +
                                            (p[2] - XMVectorGetZ(world)) * (p[2] - XMVectorGetZ(world)));
                // Float depth precision: allow 0.1% of distance.
                if (!(err <= 1e-3f * dist + 1e-4f))
                    std::printf("  mode %d px (%u,%u) dist %.1f err %.5f\n", mode, px[0], px[1], dist, err);
                EXPECT(err <= 1e-3f * dist + 1e-4f);
            }
        }

        // Sky / far plane must be rejected.
        float p[3];
        const float far_z = mode == 0 ? 1.0f : 0.0f;
        EXPECT(!up.unproject(10, 10, far_z, 0.3f, 500.0f, p));
    }
}

static const char kFakeGameProfile[] = R"(
# comment
[camera]
stage       = "vertex"
slot        = 0
size        = 208   # bytes
layout      = "view+proj"
view_offset = 0
proj_offset = 0x40
major       = "row"
handed      = "left"
latch       = "last"

[depth]
buffer_hint = "auto"
)";

static bool parse_fails(const char* text, const char* expect_in_error) {
    Profile p;
    std::string err;
    if (parse_profile(text, p, err)) return false;
    if (std::strstr(err.c_str(), expect_in_error) == nullptr) {
        std::printf("  unexpected error: %s\n", err.c_str());
        return false;
    }
    return true;
}

static void test_profile_parse() {
    Profile p;
    std::string err;
    EXPECT(parse_profile(kFakeGameProfile, p, err));
    EXPECT(err.empty());
    EXPECT(p.has_camera);
    const CameraProfile& c = p.camera;
    EXPECT(c.key.stage == reshade::api::shader_stage::vertex);
    EXPECT(c.key.slot == 0 && c.key.size == 208);
    EXPECT(c.layout == CameraLayout::ViewAndProj);
    EXPECT(c.view_offset == 0 && c.proj_offset == 64);
    EXPECT(!c.column_major && !c.right_handed && c.latch_last);
    EXPECT(c.window_offset() == 0 && c.window_size() == 128);

    const char* base = "[camera]\nstage=\"pixel\"\nslot=1\nlayout=\"invviewproj+proj\"\n";
    EXPECT(parse_profile(std::string(base) + "view_offset=128\nproj_offset=0\n", p, err));
    EXPECT(p.camera.key.stage == reshade::api::shader_stage::pixel && p.camera.key.size == 0);
    EXPECT(p.camera.key.space == 0);
    EXPECT(p.camera.window_offset() == 0 && p.camera.window_size() == 192);
    EXPECT(parse_profile(std::string(base) + "view_offset=0\nproj_offset=64\nspace=3\n", p, err));
    EXPECT(p.camera.key.slot == 1 && p.camera.key.space == 3);

    EXPECT(parse_fails("[camera]\nstage=\"vertex\"\n", "needs 'slot'"));
    EXPECT(parse_fails("[camera]\nstage=\"vertx\"\n", "stage must be one of"));
    EXPECT(parse_fails((std::string(base) + "view_offset=0\nproj_offset=64\nviewoffset=3\n").c_str(),
                       "unknown key 'viewoffset'"));
    EXPECT(parse_fails((std::string(base) + "view_offset=0\nproj_offset=32\n").c_str(), "overlap"));
    EXPECT(parse_fails((std::string(base) + "view_offset=0\nproj_offset=64\nsize=100\n").c_str(),
                       "past the buffer size"));
    EXPECT(parse_fails("[camera]\nstage=vertex\n", "line 2: can't parse value"));
    EXPECT(parse_fails("[depth]\nx=1\n", "no [camera]"));
    EXPECT(parse_fails("[camera]\nslot=1\nslot=2\n", "duplicate key"));
}

static XMMATRIX test_proj(int mode, float aspect) {
    const float fov = XMConvertToRadians(70.0f), n = 0.1f, f = 1000.0f;
    if (mode == 0) return XMMatrixPerspectiveFovLH(fov, aspect, n, f);
    if (mode == 1) return XMMatrixPerspectiveFovLH(fov, aspect, f, n);
    const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
    return XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, n, 0);
}

static void put(uint8_t* buf, uint32_t offset, const XMMATRIX& m, bool column_major) {
    XMFLOAT4X4 f;
    XMStoreFloat4x4(&f, column_major ? XMMatrixTranspose(m) : m);
    std::memcpy(buf + offset, &f, sizeof(f));
}

static float max_diff(const float a[16], const XMMATRIX& b) {
    XMFLOAT4X4 f;
    XMStoreFloat4x4(&f, b);
    float d = 0;
    for (int i = 0; i < 16; ++i) d = std::max(d, std::abs(a[i] - (&f._11)[i]));
    return d;
}

static void test_profile_decode() {
    const XMMATRIX view = XMMatrixLookToLH(XMVectorSet(12.5f, 3.2f, -40.0f, 1), XMVectorSet(0.3f, -0.2f, 0.9f, 0),
                                           XMVectorSet(0, 1, 0, 0));
    for (int mode = 0; mode < 3; ++mode) {
        const XMMATRIX proj = test_proj(mode, 16.0f / 9.0f);
        const XMMATRIX view_proj = view * proj;
        for (bool column_major : {false, true}) {
            for (int layout = 0; layout < 4; ++layout) {
                CameraProfile c;
                c.layout = CameraLayout(layout);
                c.column_major = column_major;
                c.view_offset = 128;
                c.proj_offset = 64;
                uint8_t buf[208] = {};
                put(buf, 64, proj, column_major);
                const XMMATRIX first = layout == 0   ? view
                                       : layout == 1 ? view_proj
                                       : layout == 2 ? XMMatrixInverse(nullptr, view)
                                                     : XMMatrixInverse(nullptr, view_proj);
                put(buf, 128, first, column_major);

                float v[16], p[16];
                std::string why;
                const bool ok = decode_camera(c, buf + c.window_offset(), c.window_size(), v, p, &why);
                if (!ok) std::printf("  mode %d layout %d: %s\n", mode, layout, why.c_str());
                EXPECT(ok);
                EXPECT(max_diff(p, proj) == 0.0f);  // proj passes through bit-exact
                const float dv = max_diff(v, view);
                if (layout == 0) EXPECT(dv == 0.0f);  // so does view in the view+proj layout
                // Derived views: float inputs, double math, camera ~40 m out. The one loose case is
                // a float inverse of a standard-depth viewProj: the stored input itself is only good
                // to ~2e-5 relative (0.7 mm here), which no decoding can recover.
                const float tol = layout == 3 && mode == 0 ? 1e-3f : 1e-4f;
                if (!(dv < tol)) std::printf("  mode %d layout %d view diff %g\n", mode, layout, dv);
                EXPECT(dv < tol);
            }
        }

        const ProjectionInfo info = analyze_projection([&] {
            static float f[16];
            XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(f), proj);
            return f;
        }());
        EXPECT(info.valid);
        EXPECT(std::abs(info.fov_y_deg - 70.0f) < 1e-3f);
        EXPECT(std::abs(info.aspect - 16.0f / 9.0f) < 1e-4f);
        EXPECT(std::abs(info.near_z - 0.1f) < 1e-4f);
        EXPECT(info.reversed == (mode != 0));
        EXPECT(!info.right_handed);
        if (mode == 2) EXPECT(std::isinf(info.far_z));
        else EXPECT(std::abs(info.far_z - 1000.0f) < 1.0f);
    }

    // Right-handed projections are recognized; garbage (e.g. the wrong offset) is rejected.
    float rh[16];
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(rh), XMMatrixPerspectiveFovRH(1.0f, 1.5f, 0.5f, 200.0f));
    const ProjectionInfo ri = analyze_projection(rh);
    EXPECT(ri.valid && ri.right_handed && !ri.reversed);
    EXPECT(std::abs(ri.near_z - 0.5f) < 1e-4f && std::abs(ri.far_z - 200.0f) < 0.1f);

    CameraProfile c;
    uint8_t zeros[128] = {};
    float v[16], p[16];
    EXPECT(!decode_camera(c, zeros, sizeof(zeros), v, p));
    uint8_t swapped[128];
    put(swapped, 0, test_proj(1, 1.5f), false);  // proj where the view should be, and vice versa
    put(swapped, 64, view, false);
    EXPECT(!decode_camera(c, swapped, sizeof(swapped), v, p));
}

// The layouts ReShade reports for the fake game's two D3D12 root signatures, plus the corner cases.
static void test_root_layout() {
    using namespace reshade::api;
    using namespace lidar::cam;
    using d3d12::CbvLocation;

    auto root_cbv = [](uint32_t reg, uint32_t space, shader_stage vis) {
        descriptor_range r{};
        r.dx_register_index = reg;
        r.dx_register_space = space;
        r.count = 1;
        r.visibility = vis;
        r.type = descriptor_type::constant_buffer;
        return r;
    };
    auto key = [](shader_stage stage, uint32_t slot, uint32_t space = 0) {
        CbufferKey k;
        k.stage = stage;
        k.slot = slot;
        k.space = space;
        return k;
    };
    CbvLocation loc;

    // Root CBVs, version 1.0 (push_descriptors) and 1.1 (a single range with flags): b1 then b0.
    {
        const descriptor_range_with_flags b0{root_cbv(0, 0, shader_stage::all)};
        const pipeline_layout_param params[2] = {root_cbv(1, 0, shader_stage::all), b0};
        const auto layout = d3d12::convert_layout(2, params);
        EXPECT(layout.size() == 2 && layout[0].kind == d3d12::RootParam::RootCbv && layout[1].kind == d3d12::RootParam::RootCbv);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 0), loc) && loc.param == 1 && !loc.table);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::pixel, 1), loc) && loc.param == 0 && !loc.table);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 2), loc));
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 0, 1), loc));  // other space
    }
    // Stage visibility: b0 only visible to the pixel shader.
    {
        const pipeline_layout_param params[1] = {root_cbv(0, 0, shader_stage::pixel)};
        const auto layout = d3d12::convert_layout(1, params);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 0), loc));
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::pixel, 0), loc));
    }
    // A table [SRV t0..t3, CBV b1, CBV b0 x2 in space 2], after a root constant: offsets are
    // ReShade's resolved `binding`s (APPEND already applied).
    {
        descriptor_range ranges[3] = {};
        ranges[0].type = descriptor_type::texture_shader_resource_view;
        ranges[0].count = 4;
        ranges[1] = root_cbv(1, 0, shader_stage::all);
        ranges[1].binding = 4;
        ranges[2] = root_cbv(0, 2, shader_stage::all);
        ranges[2].binding = 5;
        ranges[2].count = 2;
        const pipeline_layout_param params[2] = {constant_range{0, 3, 0, 4, shader_stage::all},
                                                 pipeline_layout_param(3, ranges)};
        const auto layout = d3d12::convert_layout(2, params);
        EXPECT(layout[0].kind == d3d12::RootParam::Other && layout[1].kind == d3d12::RootParam::Table);
        EXPECT(layout[1].ranges.size() == 2);  // only the CBV ranges
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 1), loc) && loc.param == 1 && loc.table &&
               loc.table_offset == 4);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 1, 2), loc) && loc.table_offset == 6);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 2, 2), loc));
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 0), loc));  // b0 is in space 2
    }
    // Unbounded range (count UINT32_MAX) must not overflow.
    {
        descriptor_range_with_flags r{root_cbv(10, 0, shader_stage::all)};
        r.count = UINT32_MAX;
        const pipeline_layout_param params[1] = {pipeline_layout_param(1, &r)};
        const auto layout = d3d12::convert_layout(1, params);
        EXPECT(layout[0].kind == d3d12::RootParam::Table);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 1000), loc) && loc.table_offset == 990);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 9), loc));
    }
}

int main() {
    test_ring();
    test_unproject();
    test_profile_parse();
    test_profile_decode();
    test_root_layout();
    if (g_failures == 0) std::printf("all tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
