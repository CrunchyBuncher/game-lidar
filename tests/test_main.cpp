// Unit tests: ring buffer semantics, unprojection across depth conventions, the addon's camera
// profiles (parsing, matrix layouts, projection analysis) and D3D12 root signature lookups.
#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "camera_math.h"
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
    EXPECT(!c.column_major && !c.right_handed && c.latch == Latch::Last);
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

    // Single-matrix layouts: no proj_offset, a 64-byte window. The common latch.
    const char* single = "[camera]\nstage=\"vertex\"\nslot=0\nlayout=\"viewproj\"\nview_offset=208\nlatch=\"common\"\n";
    EXPECT(parse_profile(single, p, err));
    EXPECT(p.camera.layout == CameraLayout::ViewProj && p.camera.single_matrix() && p.camera.latch == Latch::Common);
    EXPECT(p.camera.window_offset() == 208 && p.camera.window_size() == 64);
    EXPECT(parse_fails((std::string(single) + "proj_offset=0\n").c_str(), "doesn't apply"));
    EXPECT(parse_fails("[camera]\nstage=\"vertex\"\nslot=0\nlayout=\"invviewproj\"\nview_offset=0\nsize=48\n",
                       "past the buffer size"));

    // format_profile writes what parse_profile reads.
    for (int layout = 0; layout < 6; ++layout) {
        CameraProfile c0;
        c0.key = {reshade::api::shader_stage::pixel, 3, 2, 512};
        c0.layout = CameraLayout(layout);
        c0.view_offset = 256;
        c0.proj_offset = 64;
        c0.column_major = layout % 2 == 0;
        c0.right_handed = layout % 3 == 0;
        c0.latch = Latch(layout % 3);
        Profile back;
        EXPECT(parse_profile(format_profile(c0, "line one\nline two"), back, err));
        const CameraProfile& c1 = back.camera;
        EXPECT(c1.key.stage == c0.key.stage && c1.key.slot == 3 && c1.key.space == 2 && c1.key.size == 512);
        EXPECT(c1.layout == c0.layout && c1.view_offset == 256 && c1.column_major == c0.column_major);
        EXPECT(c1.right_handed == c0.right_handed && c1.latch == c0.latch);
        EXPECT(c1.single_matrix() || c1.proj_offset == 64);
    }
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
            for (int layout = 0; layout < 6; ++layout) {
                CameraProfile c;
                c.layout = CameraLayout(layout);
                c.column_major = column_major;
                c.view_offset = 128;
                c.proj_offset = 64;
                uint8_t buf[208] = {};
                put(buf, 64, proj, column_major);
                const XMMATRIX first = layout == 0                ? view
                                       : layout == 1 || layout == 4 ? view_proj
                                       : layout == 2              ? XMMatrixInverse(nullptr, view)
                                                                  : XMMatrixInverse(nullptr, view_proj);
                put(buf, 128, first, column_major);

                float v[16], p[16];
                std::string why;
                const bool ok = decode_camera(c, buf + c.window_offset(), c.window_size(), v, p, &why);
                if (!ok) std::printf("  mode %d layout %d: %s\n", mode, layout, why.c_str());
                EXPECT(ok);
                if (c.single_matrix()) {
                    // Split out of view * proj: as good as the float product allows.
                    const float dp = max_diff(p, proj);
                    if (!(dp < 1e-4f)) std::printf("  mode %d layout %d proj diff %g\n", mode, layout, dp);
                    EXPECT(dp < 1e-4f);
                } else {
                    EXPECT(max_diff(p, proj) == 0.0f);  // proj passes through bit-exact
                }
                const float dv = max_diff(v, view);
                if (layout == 0) EXPECT(dv == 0.0f);  // so does view in the view+proj layout
                // Derived views: float inputs, double math, camera ~40 m out. The one loose case is
                // a float inverse of a standard-depth viewProj: the stored input itself is only good
                // to ~2e-5 relative (0.7 mm here), which no decoding can recover.
                const float tol = (layout == 3 || layout == 5) && mode == 0 ? 1e-3f : 1e-4f;
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

// ---- Discovery: classifiers, view-projection decomposition, reprojection ----

static void store_f(const XMMATRIX& m, bool column_major, float f[16]) {
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(f), column_major ? XMMatrixTranspose(m) : m);
}
static uint32_t classify_xm(const XMMATRIX& m, bool column_major) {
    float f[16];
    store_f(m, column_major, f);
    return classify_matrix(f);
}
static bool is(uint32_t bits, MatrixKind k, bool column_major) { return bits == kind_bit(k, column_major); }

// Every projection style discovery must recognize: standard, reversed, reversed-infinite, and
// right-handed standard / reversed, optionally with TAA-style jitter.
static XMMATRIX any_proj(int mode, bool jitter) {
    XMMATRIX p;
    if (mode < 3) {
        p = test_proj(mode, 16.0f / 9.0f);
    } else {
        p = mode == 3 ? XMMatrixPerspectiveFovRH(1.0f, 1.5f, 0.5f, 200.0f)
                      : XMMatrixPerspectiveFovRH(1.0f, 1.5f, 200.0f, 0.5f);
    }
    if (jitter) {
        XMFLOAT4X4 f;
        XMStoreFloat4x4(&f, p);
        f._31 += 0.0004f;
        f._32 -= 0.0007f;
        p = XMLoadFloat4x4(&f);
    }
    return p;
}

static void test_classify() {
    const XMMATRIX lh = XMMatrixLookToLH(XMVectorSet(12.5f, 3.2f, -40.0f, 1), XMVectorSet(0.3f, -0.2f, 0.9f, 0),
                                         XMVectorSet(0, 1, 0, 0));
    const XMMATRIX rh = XMMatrixLookToRH(XMVectorSet(-7.0f, 20.0f, 3.0f, 1), XMVectorSet(-0.5f, -0.4f, 0.2f, 0),
                                         XMVectorSet(0, 1, 0, 0));
    const XMMATRIX world = XMMatrixRotationRollPitchYaw(0.3f, 1.1f, -0.4f) * XMMatrixTranslation(5, 1, -3);
    for (bool col : {false, true}) {
        // Views and inverse views are rigid. With a translation, only the right major matches.
        EXPECT(is(classify_xm(lh, col), MatrixKind::Rigid, col));
        EXPECT(is(classify_xm(XMMatrixInverse(nullptr, lh), col), MatrixKind::Rigid, col));
        EXPECT(is(classify_xm(rh, col), MatrixKind::Rigid, col));
        for (int mode = 0; mode < 5; ++mode) {
            for (bool jitter : {false, true}) {
                const XMMATRIX proj = any_proj(mode, jitter);
                const XMMATRIX view = mode < 3 ? lh : rh;
                const uint32_t pb = classify_xm(proj, col);
                if (!is(pb, MatrixKind::Proj, col)) std::printf("  proj mode %d jitter %d col %d: bits %x\n", mode, jitter, col, pb);
                EXPECT(is(pb, MatrixKind::Proj, col));
                const uint32_t vb = classify_xm(view * proj, col);
                if (!is(vb, MatrixKind::ViewProj, col)) std::printf("  viewproj mode %d jitter %d col %d: bits %x\n", mode, jitter, col, vb);
                EXPECT(is(vb, MatrixKind::ViewProj, col));
                EXPECT(is(classify_xm(XMMatrixInverse(nullptr, view * proj), col), MatrixKind::InvViewProj, col));
                // world * view * proj with a rigid world still looks like a camera (the reprojection
                // check tells them apart); a scaling world doesn't.
                EXPECT(is(classify_xm(world * view * proj, col), MatrixKind::ViewProj, col));
                EXPECT(classify_xm(XMMatrixScaling(2, 2, 2) * world * view * proj, col) == 0);
            }
        }
    }
    // Decoys.
    EXPECT(classify_xm(XMMatrixScaling(1, 3, 1) * world, false) == 0);  // world with scale
    EXPECT(classify_xm(XMMatrixIdentity(), false) ==
           (kind_bit(MatrixKind::Rigid, false) | kind_bit(MatrixKind::Rigid, true)));  // no translation: both
    float f[16] = {};
    EXPECT(classify_matrix(f) == 0);
    for (int i = 0; i < 16; ++i) f[i] = float(i + 1);
    EXPECT(classify_matrix(f) == 0);
    f[5] = std::nanf("");
    EXPECT(classify_matrix(f) == 0);
    // Colors and positions packed as float4s (e.g. tint, light position, time).
    const float junk[16] = {1, 1, 1, 1, 0.9f, 0.25f, 0.2f, 1, 20, 0, -26.5f, 1, 0.016f, 12.5f, 0, 0};
    EXPECT(classify_matrix(junk) == 0);
}

static float max_diff_m(const mat::Mat& a, const XMMATRIX& b) {
    float f[16];
    mat::store(a, f);
    return max_diff(f, b);
}

static void test_decompose() {
    const XMMATRIX lh = XMMatrixLookToLH(XMVectorSet(12.5f, 3.2f, -40.0f, 1), XMVectorSet(0.3f, -0.2f, 0.9f, 0),
                                         XMVectorSet(0, 1, 0, 0));
    const XMMATRIX rh = XMMatrixLookToRH(XMVectorSet(-7.0f, 20.0f, 3.0f, 1), XMVectorSet(-0.5f, -0.4f, 0.2f, 0),
                                         XMVectorSet(0, 1, 0, 0));
    for (int mode = 0; mode < 5; ++mode) {
        for (bool jitter : {false, true}) {
            const XMMATRIX view = mode < 3 ? lh : rh, proj = any_proj(mode, jitter);
            float f[16];
            store_f(view * proj, false, f);
            mat::Mat v, p;
            EXPECT(decompose_view_proj(mat::load(f, false), v, p));
            const float dv = max_diff_m(v, view), dp = max_diff_m(p, proj);
            if (!(dv < 1e-4f && dp < 1e-4f)) std::printf("  decompose mode %d jitter %d: view %g proj %g\n", mode, jitter, dv, dp);
            EXPECT(dv < 1e-4f && dp < 1e-4f);
        }
    }
    mat::Mat v, p;
    EXPECT(!decompose_view_proj(mat::identity(), v, p));
}

// Depth of a tiny scene (ground y = 0, a wall at z = 30, a box) through view * proj, as a protocol
// frame of w x h sampled from sw x sh.
static std::vector<float> render_depth(const XMMATRIX& view, const XMMATRIX& proj, bool reversed, uint32_t w, uint32_t h,
                                       uint32_t sw, uint32_t sh) {
    const XMMATRIX vp = view * proj, inv_vp = XMMatrixInverse(nullptr, vp);
    const XMVECTOR eye = XMMatrixInverse(nullptr, view).r[3];
    FrameHeader hd{};
    hd.width = w, hd.height = h, hd.src_width = sw, hd.src_height = sh;
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(hd.view), view);
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(hd.proj), proj);
    const Unprojector up(hd);
    std::vector<float> depth(size_t(w) * h);
    for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = 0; x < w; ++x) {
            float nx, ny;
            up.ndc(x, y, nx, ny);
            const XMVECTOR p = XMVector3TransformCoord(XMVectorSet(nx, ny, 0.5f, 1), inv_vp);
            const XMVECTOR d = XMVector3Normalize(XMVectorSubtract(p, eye));
            float e[3], dir[3];
            for (int k = 0; k < 3; ++k) e[k] = XMVectorGetByIndex(eye, k), dir[k] = XMVectorGetByIndex(d, k);
            float t = 1e30f;
            if (dir[1] < 0) t = std::min(t, -e[1] / dir[1]);
            if (dir[2] > 0) t = std::min(t, (30 - e[2]) / dir[2]);
            const float lo[3] = {-2, 0, 8}, hi[3] = {2, 3, 12};  // slab test
            float t0 = 0, t1 = 1e30f;
            for (int k = 0; k < 3; ++k) {
                const float a = (lo[k] - e[k]) / dir[k], b = (hi[k] - e[k]) / dir[k];
                t0 = std::max(t0, std::min(a, b));
                t1 = std::min(t1, std::max(a, b));
            }
            if (t0 <= t1) t = std::min(t, t0);
            float z = reversed ? 0.0f : 1.0f;  // clear value: sky
            if (t < 1e29f) {
                const XMVECTOR hit = XMVectorAdd(eye, XMVectorScale(d, t));
                z = XMVectorGetZ(XMVector3TransformCoord(hit, vp));
            }
            depth[size_t(y) * w + x] = z;
        }
    return depth;
}

static void test_reproject() {
    const uint32_t w = 160, h = 90, sw = 1280, sh = 720;
    auto look = [](float x, float z, float yaw) {
        return XMMatrixLookToLH(XMVectorSet(x, 1.7f, z, 1), XMVectorSet(std::sin(yaw), -0.15f, std::cos(yaw), 0),
                                XMVectorSet(0, 1, 0, 0));
    };
    const XMMATRIX va = look(0, 0, 0), vb = look(0.5f, 0.4f, 0.17f);
    for (int mode = 0; mode < 3; ++mode) {
        const XMMATRIX proj = test_proj(mode, 16.0f / 9.0f);
        const auto da = render_depth(va, proj, mode != 0, w, h, sw, sh);
        const auto db = render_depth(vb, proj, mode != 0, w, h, sw, sh);
        DepthGrid a, b;
        a.build(da.data(), w, h, sw, sh, 1);
        b.build(db.data(), w, h, sw, sh, 1);
        EXPECT(a.standard == (mode == 0));
        EXPECT(depth_change(a, a) == 0.0);
        EXPECT(depth_change(a, b) > 0.1);

        auto m = [](const XMMATRIX& x) {
            float f[16];
            store_f(x, false, f);
            return mat::load(f, false);
        };
        const ReprojStats right = reproject(a, m(va), m(proj), b, m(vb), m(proj));
        const ReprojStats still = reproject(a, m(va), m(proj), b, m(va), m(proj));  // a matrix that didn't move
        // 30 cm off. Sideways would be invisible here: it's along both the ground and the wall.
        const ReprojStats off = reproject(a, m(va), m(proj), b, m(vb * XMMatrixTranslation(0, 0.3f, 0.3f)), m(proj));
        if (!(right.median_rel < 1e-4 && still.median_rel > 0.01 && off.median_rel > 0.005))
            std::printf("  reproject mode %d: right %g (%u/%u), still %g, off %g\n", mode, right.median_rel,
                        right.landed, right.tested, still.median_rel, off.median_rel);
        EXPECT(right.tested > 300 && right.landed > right.tested / 2);
        EXPECT(right.median_rel < 1e-4);
        EXPECT(still.median_rel > 0.01);
        EXPECT(off.median_rel > 0.005);
    }
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
    test_classify();
    test_decompose();
    test_reproject();
    test_root_layout();
    if (g_failures == 0) std::printf("all tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
