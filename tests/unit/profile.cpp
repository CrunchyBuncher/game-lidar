// Camera profiles: parsing and writing them, decoding the camera from each matrix layout,
// projection analysis, camera-relative (translated) layouts and pose normalization.
#include "profile.h"

#include <DirectXMath.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "camera_math.h"
#include "matrices.h"
#include "test.h"

using namespace DirectX;
using namespace lidar;
using namespace lidar::test;

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

TEST_CASE(profile, parse) {
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

TEST_CASE(profile, decode) {
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

// UE3: a camera-relative view-projection (TranslatedViewProjection, the camera at the origin) with
// the world offset in another register (PreViewTranslation = -camera), and a projection whose
// depth tends to 1 - 1e-3 instead of reaching 1.
TEST_CASE(profile, translated_camera) {
    const XMVECTOR eye = XMVectorSet(-375.5f, 1286.3f, 236.1f, 1);
    const XMVECTOR dir = XMVectorSet(0.16f, -0.93f, -0.32f, 0);
    const XMMATRIX view = XMMatrixLookToLH(eye, dir, XMVectorSet(0, 0, 1, 0));
    const XMMATRIX rel_view = XMMatrixLookToLH(XMVectorSet(0, 0, 0, 1), dir, XMVectorSet(0, 0, 1, 0));
    const float a = 1 - 1e-3f, near_z = 10;
    const XMMATRIX proj(1, 0, 0, 0, 0, 16.0f / 9.0f, 0, 0, 0, 0, a, 1, 0, 0, -near_z * a, 0);

    float pf[16];
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(pf), proj);
    const ProjectionInfo info = analyze_projection(pf);
    EXPECT(info.valid && !info.reversed && std::isinf(info.far_z));
    EXPECT(std::abs(info.near_z - near_z) < 1e-3f);
    EXPECT(classify_matrix([&] {
               static float f[16];
               XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(f), rel_view * proj);
               return f;
           }()) == kind_bit(MatrixKind::ViewProj, false));

    for (bool subtract : {false, true}) {
        CameraProfile c;
        c.layout = CameraLayout::ViewProj;
        c.view_offset = 0;
        c.has_translation = true;
        c.translation_offset = 80;  // c5
        c.translation_subtract = subtract;
        EXPECT(c.window_offset() == 0 && c.window_size() == 92);
        uint8_t buf[96] = {};
        put(buf, 0, rel_view * proj, false);
        const float t[3] = {XMVectorGetX(eye), XMVectorGetY(eye), XMVectorGetZ(eye)};
        for (int i = 0; i < 3; ++i) reinterpret_cast<float*>(buf + 80)[i] = subtract ? t[i] : -t[i];
        float v[16], p[16];
        std::string why;
        const bool ok = decode_camera(c, buf, c.window_size(), v, p, &why);
        if (!ok) std::printf("  translated camera: %s\n", why.c_str());
        EXPECT(ok);
        const float dv = max_diff(v, view);
        if (!(dv < 1e-3f)) std::printf("  translated camera (%s): view diff %g\n", subtract ? "-" : "+", dv);
        EXPECT(dv < 1e-3f);
    }

    const std::string base = "[camera]\nstage=\"vertex\"\nslot=0\nlayout=\"viewproj\"\nview_offset=0\n";
    Profile pr;
    std::string err;
    EXPECT(parse_profile(base + "translation_offset=80\ntranslation=\"add\"\n", pr, err));
    EXPECT(pr.camera.has_translation && pr.camera.translation_offset == 80 && !pr.camera.translation_subtract);
    Profile back;
    EXPECT(parse_profile(format_profile(pr.camera, ""), back, err) && back.camera.has_translation &&
           back.camera.translation_offset == 80 && !back.camera.translation_subtract);
    EXPECT(parse_fails((base + "translation_offset=80\n").c_str(), "translation"));
    EXPECT(parse_fails((base + "translation=\"add\"\n").c_str(), "needs translation_offset"));
    EXPECT(parse_fails("[camera]\nstage=\"vertex\"\nslot=0\nlayout=\"modelview\"\nview_offset=0\nproj_offset=128\n"
                       "translation_offset=256\ntranslation=\"add\"\n",
                       "doesn't apply"));
}

// A right-handed game in decimeters: normalized, the pose unprojects its depth to the world point
// in meters with z negated (so the viewer's left-handed display isn't mirrored).
TEST_CASE(profile, normalize_pose) {
    const XMMATRIX view = XMMatrixLookToRH(XMVectorSet(300, 170, 800, 1), XMVectorSet(0.4f, -0.2f, -1, 0),
                                           XMVectorSet(0, 1, 0, 0));
    const XMMATRIX proj = XMMatrixPerspectiveFovRH(1.0f, 16.0f / 9.0f, 1.0f, 100000.0f);
    float v[16], p[16];
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(v), view);
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(p), proj);
    const bool rh = analyze_projection(p).right_handed;
    EXPECT(rh);
    normalize_pose(v, p, rh, 10);
    const ProjectionInfo info = analyze_projection(p);
    EXPECT(info.valid && !info.right_handed);
    EXPECT(std::abs(info.near_z - 0.1f) < 1e-5f);  // meters now
    const XMMATRIX v2 = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(v));
    const XMMATRIX p2 = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(p));
    EXPECT(std::abs(XMVectorGetX(XMMatrixDeterminant(v2)) - 1) < 1e-4f);  // still a rotation, not a mirror

    for (const XMVECTOR world : {XMVectorSet(320, 160, 700, 1), XMVectorSet(250, 175, 500, 1)}) {
        const XMVECTOR clip = XMVector4Transform(world, view * proj);
        const XMVECTOR ndc = XMVectorScale(clip, 1 / XMVectorGetW(clip));
        XMVECTOR back = XMVector4Transform(ndc, XMMatrixInverse(nullptr, p2));
        back = XMVectorScale(back, 1 / XMVectorGetW(back));
        back = XMVector4Transform(back, XMMatrixInverse(nullptr, v2));
        const XMVECTOR expect = XMVectorMultiply(world, XMVectorSet(0.1f, 0.1f, -0.1f, 1));
        const float err = XMVectorGetX(XMVector3Length(back - expect));
        // Float depth at 1..100000 resolves ~1 mm this far out (10-35 m).
        if (!(err < 5e-3f)) std::printf("  normalize_pose: error %g m\n", err);
        EXPECT(err < 5e-3f);
    }

    // Left-handed, in meters: untouched.
    float lv[16], lp[16], lv0[16], lp0[16];
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(lv0), XMMatrixLookToLH(XMVectorSet(1, 2, 3, 1), XMVectorSet(0, 0, 1, 0),
                                                                         XMVectorSet(0, 1, 0, 0)));
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(lp0), test_proj(0, 1.5f));
    std::memcpy(lv, lv0, sizeof(lv));
    std::memcpy(lp, lp0, sizeof(lp));
    normalize_pose(lv, lp, false, 1);
    EXPECT(std::memcmp(lv, lv0, sizeof(lv)) == 0 && std::memcmp(lp, lp0, sizeof(lp)) == 0);

    // IW (Black Ops III): a right-handed z-up world (x forward, y left) under a left-handed view, in
    // inches. The view's rotation is a mirror. Normalized with z_up: a rotation, y-up, points in meters
    // at (x, z, y).
    {
        const XMMATRIX to_d3d(0, 0, 1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1);  // forward, left, up -> z, -x, y
        const XMMATRIX iw_view = XMMatrixTranslation(-193.6f, 1281.7f, -140.1f) * XMMatrixRotationZ(-0.7f) *
                                 XMMatrixRotationY(0.2f) * to_d3d;
        const XMMATRIX iw_proj = XMMatrixPerspectiveFovLH(1.0f, 16.0f / 9.0f, 1.0f, 100000.0f);
        float iv[16], ip[16];
        XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(iv), iw_view);
        XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(ip), iw_proj);
        EXPECT(XMVectorGetX(XMMatrixDeterminant(iw_view)) < 0);
        EXPECT(!analyze_projection(ip).right_handed);
        const float k = 1 / 39.37f;
        normalize_pose(iv, ip, false, 39.37f, true);
        const XMMATRIX v3 = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(iv));
        const XMMATRIX p3 = XMLoadFloat4x4(reinterpret_cast<const XMFLOAT4X4*>(ip));
        EXPECT(std::abs(XMVectorGetX(XMMatrixDeterminant(v3)) - 1) < 1e-4f);
        for (const XMVECTOR world : {XMVectorSet(-100, -1200, 150, 1), XMVectorSet(-250, -1300, 120, 1)}) {
            const XMVECTOR clip = XMVector4Transform(world, iw_view * iw_proj);
            if (!(XMVectorGetW(clip) > 0)) continue;  // behind the camera
            const XMVECTOR ndc = XMVectorScale(clip, 1 / XMVectorGetW(clip));
            XMVECTOR back = XMVector4Transform(ndc, XMMatrixInverse(nullptr, p3));
            back = XMVectorScale(back, 1 / XMVectorGetW(back));
            back = XMVector4Transform(back, XMMatrixInverse(nullptr, v3));
            const XMVECTOR expect =
                XMVectorSet(XMVectorGetX(world) * k, XMVectorGetZ(world) * k, XMVectorGetY(world) * k, 1);
            const float err = XMVectorGetX(XMVector3Length(back - expect));
            if (!(err < 5e-3f)) std::printf("  normalize_pose (IW): error %g m\n", err);
            EXPECT(err < 5e-3f);
        }
    }

    // up in profiles: y by default, z parsed and written back.
    {
        const std::string b = "[camera]\nstage=\"vertex\"\nslot=0\nlayout=\"viewproj\"\nview_offset=0\n";
        Profile pu, back;
        std::string e;
        EXPECT(parse_profile(b, pu, e) && !pu.camera.z_up);
        EXPECT(parse_profile(b + "up=\"z\"\n", pu, e) && pu.camera.z_up);
        EXPECT(parse_profile(format_profile(pu.camera, ""), back, e) && back.camera.z_up);
        EXPECT(parse_fails((b + "up=\"x\"\n").c_str(), "up"));
    }

    // units_per_meter in profiles: parsed, validated, written back.
    const std::string base = "[camera]\nstage=\"vertex\"\nslot=0\nlayout=\"modelview\"\nview_offset=0\nproj_offset=128\n";
    Profile pr;
    std::string err;
    EXPECT(parse_profile(base, pr, err) && pr.camera.units_per_meter == 1);
    EXPECT(parse_profile(base + "units_per_meter = 10\n", pr, err) && pr.camera.units_per_meter == 10);
    EXPECT(parse_profile(base + "units_per_meter = 2.5\n", pr, err) && pr.camera.units_per_meter == 2.5f);
    EXPECT(parse_fails((base + "units_per_meter = 0\n").c_str(), "positive number"));
    Profile back;
    pr.camera.units_per_meter = 12.5f;
    EXPECT(parse_profile(format_profile(pr.camera, ""), back, err) && back.camera.units_per_meter == 12.5f);
}
