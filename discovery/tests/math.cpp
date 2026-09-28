// The math under discovery: the matrix classifier, view-projection decomposition and reprojection scoring.
#include <DirectXMath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "camera_math.h"
#include "matrices.h"
#include "profile.h"
#include "synthetic.h"
#include "test.h"

using namespace DirectX;
using namespace lidar;
using namespace lidar::test;

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

TEST_CASE(math, classify) {
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

TEST_CASE(math, decompose) {
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

TEST_CASE(math, reproject) {
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
        if (!(right.explained > 0.95 && still.explained < 0.05 && off.explained < 0.5))
            std::printf("  reproject mode %d explained: right %g, still %g, off %g\n", mode, right.explained,
                        still.explained, off.explained);
        EXPECT(right.explained > 0.95);
        EXPECT(still.explained < 0.05);
        EXPECT(off.explained < 0.5);

        // Memory that isn't depth: NaN and values outside [0, 1].
        EXPECT(invalid_depth(a) == 0.0);
        DepthGrid junk = a;
        for (size_t i = 0; i < junk.depth.size(); i += 2) junk.depth[i] = i % 4 ? std::nanf("") : -3e38f;
        EXPECT(invalid_depth(junk) > 0.4);
    }
}

// A view-projection from MGS Delta's View buffer (pixel b0 @ 6576) that splits into a view and a
// projection with near 0.9998 and far 1: every depth reads as the same distance, so it "explained"
// every depth image perfectly and outscored the camera. It isn't a camera.
TEST_CASE(math, degenerate_projection) {
    const float vp[16] = {0, 0, -5861.8f, 1, 0.095916f, 0, 0, 0, 0, 0.11989f, 0, 0, 0, 0, 5861.8f, 0};
    CameraProfile p;
    p.layout = CameraLayout::ViewProj;
    float view[16], proj[16];
    EXPECT(!decode_camera(p, reinterpret_cast<const uint8_t*>(vp), sizeof(vp), view, proj));
    // A real one of the same shape (reversed infinite, near 10) still decodes.
    const float ok[16] = {0, 0, 0, 1, 0.095916f, 0, 0, 0, 0, 0.11989f, 0, 0, 0, 0, 10, 0};
    EXPECT(decode_camera(p, reinterpret_cast<const uint8_t*>(ok), sizeof(ok), view, proj));
}
