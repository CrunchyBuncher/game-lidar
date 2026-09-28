// fake_game: a tiny "game" that renders the test level along a scripted camera path, with
// view/proj in a constant buffer (b0) the way real games keep them, as a target for the
// addon's cbuffer sniffer. A per-object cbuffer in b1 is a decoy that changes between draws.
//
// D3D11 (default): dynamic cbuffers updated with Map/Unmap. Also publishes depth + camera to
// the shared-memory ring itself, exactly as the ReShade addon does.
// D3D12 (--api d3d12): render only (implies --no-publish). The cbuffers live in a persistently
// mapped upload heap with a region per frame in flight, bound as root CBVs, or through a
// descriptor table with --cbv-tables.
// D3D9 (--api d3d9): render only. The camera sits in vertex shader constant registers c0-c12
// (same bytes as b0), the decoy in c13-c17. --own-depth uses a CreateDepthStencilSurface depth
// buffer instead of the device's auto depth-stencil. --up draws from system memory, alternating
// DrawPrimitiveUP and DrawIndexedPrimitiveUP (like Ninja-engine ports such as SA2 may).
//
// --camera-layout viewproj | wvp: only a view-projection in b0, or only world * view * proj per
// draw (in b1), instead of separate view and proj; for discovery mode. The level is drawn in three
// parts after the NPC in these layouts. --camera-layout modelview (D3D9 only): world * view per
// draw in c0, the level in 24 parts, each placed on its own (plan_modelview.md).
//
// Usage: fake_game [--api d3d11|d3d12|d3d9] [--camera-layout separate|viewproj|wvp|modelview]
//                  [--cbv-tables] [--d3d12-debug] [--own-depth] [--up] [--d24]
//                  [--depth standard|reversed|reversed-infinite] [--capture-width 480]
//                  [--capture-every 1] [--fov 70] [--no-npc] [--no-color]
//                  [--size 1280x720] [--duration seconds] [--no-publish]
//                  [--ring NAME] [--freeze T] [--speed X] [--tint-until T] [--decoy-draws N]
//                  [--background]
//        --no-publish: render only, leave the ring to the ReShade addon.
//        --background: open behind other windows without taking the focus (the e2e tests).
//        --ring: publish to another mapping (e.g. a reference for `lidar_verify addon`).
//        --freeze: hold the scripted camera at path time T seconds.
//        --speed: move the scripted camera X times as fast (walk, turns, bob); the NPC keeps its pace.
//        --tint-until: publish magenta color, with the true depth, for the first T seconds (D3D11): a
//                      stand-in for an effect over the scene, to test the viewer's color updates.
//        --d3d12-debug: D3D12 debug layer on; its warnings/errors go to stderr (exit code 3 on errors).
//        --d24: D3D11 with a D24S8 depth buffer instead of D32F, the reference for D3D9's 24-bit depth.
//        --decoy-draws: D3D12 root CBVs: N draws (of no instances) before the scene each frame, with b0
//                       bound to other views' constants (a cube capture, a second camera), as in UE,
//                       whose registers are assigned per shader.
// Keys:  M toggle manual camera (WASD/QE + right-drag), Space pause capture, Esc unfocus.
#include "fake_game.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "protocol.h"

using namespace DirectX;

namespace lidar::fake {
namespace {

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

}  // namespace

Options parse(int argc, char** argv) {
    Options o;
    o.ring = kFramesMappingName;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--api") {
            const std::string api = next();
            o.api = api == "d3d12" ? Api::D3D12 : api == "d3d9" ? Api::D3D9 : Api::D3D11;
        } else if (a == "--camera-layout") {
            const std::string l = next();
            o.layout = l == "viewproj"    ? ConstantsLayout::ViewProj
                       : l == "wvp"       ? ConstantsLayout::Wvp
                       : l == "modelview" ? ConstantsLayout::ModelView
                                          : ConstantsLayout::Separate;
            if (l != "viewproj" && l != "wvp" && l != "modelview" && l != "separate")
                std::fprintf(stderr, "unknown --camera-layout %s (separate | viewproj | wvp | modelview)\n",
                             l.c_str());
        } else if (a == "--cbv-tables") {
            o.cbv_tables = true;
        } else if (a == "--d3d12-debug") {
            o.d3d12_debug = true;
        } else if (a == "--own-depth") {
            o.own_depth = true;
        } else if (a == "--up") {
            o.up = true;
        } else if (a == "--d24") {
            o.d24 = true;
        } else if (a == "--depth") {
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
        } else if (a == "--background") {
            o.background = true;
        } else if (a == "--no-color") {
            o.color = false;
        } else if (a == "--size") {
            std::sscanf(next(), "%dx%d", &o.width, &o.height);
        } else if (a == "--duration") {
            o.duration = float(std::atof(next()));
        } else if (a == "--no-publish") {
            o.publish = false;
        } else if (a == "--ring") {
            const std::string n = next();
            o.ring.assign(n.begin(), n.end());
        } else if (a == "--freeze") {
            o.freeze = float(std::atof(next()));
        } else if (a == "--speed") {
            o.speed = float(std::atof(next()));
        } else if (a == "--tint-until") {
            o.tint_until = float(std::atof(next()));
        } else if (a == "--decoy-draws") {
            o.decoy_draws = uint32_t(std::max(0, std::atoi(next())));
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
        }
    }
    return o;
}

Geometry build_geometry() {
    Geometry g;
    for (const auto& b : scene::build()) append_box(g.verts, b.min, b.max, b.color);
    g.static_count = uint32_t(g.verts.size());
    const float npc_min[3] = {-0.3f, 0.0f, -0.3f}, npc_max[3] = {0.3f, 1.8f, 0.3f};
    append_box(g.verts, npc_min, npc_max, scene::rgb(255, 255, 255));
    return g;
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

void CameraRig::update(const App& app, const Options& opt, float dt) {
    if (app.key_pressed('M')) manual = !manual;
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
        path_time = opt.freeze >= 0 ? opt.freeze : path_time + dt * opt.speed;
        pose = scene::camera_path(float(path_time));
    }
}

CameraCB CameraRig::constants(const Options& opt, float aspect) const {
    const float cp = std::cos(pose.pitch);
    const XMVECTOR eye = XMVectorSet(pose.pos[0], pose.pos[1], pose.pos[2], 1);
    const XMVECTOR fwd = XMVectorSet(std::sin(pose.yaw) * cp, std::sin(pose.pitch), std::cos(pose.yaw) * cp, 0);
    const XMMATRIX view = XMMatrixLookToLH(eye, fwd, XMVectorSet(0, 1, 0, 0));
    const XMMATRIX proj = make_proj(opt.depth, XMConvertToRadians(opt.fov_deg), aspect);
    CameraCB cam;
    XMStoreFloat4x4(&cam.view, view);
    XMStoreFloat4x4(&cam.proj, proj);
    XMStoreFloat4x4(&cam.view_proj, view * proj);
    cam.cam_pos = {pose.pos[0], pose.pos[1], pose.pos[2], 1};
    return cam;
}

ObjectCB static_object() {
    ObjectCB obj;
    XMStoreFloat4x4(&obj.world, XMMatrixIdentity());
    obj.tint = {1, 1, 1, 1};
    return obj;
}

ObjectCB npc_object(double t) {
    ObjectCB obj;
    XMStoreFloat4x4(&obj.world, XMMatrixTranslation(20.0f * std::sin(float(t) * 0.3f), 0, -26.5f));
    obj.tint = {0.9f, 0.25f, 0.2f, 1};
    return obj;
}

CameraCB gpu_camera(const Options& opt, const CameraCB& cam) {
    CameraCB g = cam;
    if (opt.layout == ConstantsLayout::ViewProj) {
        g.view = cam.view_proj;
        XMStoreFloat4x4(&g.proj, XMMatrixIdentity());
    } else if (opt.layout == ConstantsLayout::Wvp) {
        XMStoreFloat4x4(&g.view, XMMatrixIdentity());
        g.proj = g.view_proj = g.view;
    } else if (opt.layout == ConstantsLayout::ModelView) {
        XMStoreFloat4x4(&g.view, XMMatrixIdentity());  // set per draw
        g.view_proj = g.view;
    }
    return g;
}

std::vector<DrawItem> scene_draws(const Options& opt, const Geometry& geo, const CameraCB& cam, double t) {
    std::vector<DrawItem> draws;
    if (opt.npc) draws.push_back({npc_object(t), geo.static_count, uint32_t(geo.verts.size()) - geo.static_count});
    constexpr uint32_t kBox = 36;  // vertices per box
    const uint32_t kParts = opt.layout == ConstantsLayout::ModelView ? 24 : kMaxDraws - 1;
    const uint32_t boxes = geo.static_count / kBox;
    for (uint32_t i = 0; i < kParts; ++i) {
        const uint32_t b0 = boxes * i / kParts, b1 = boxes * (i + 1) / kParts;
        draws.push_back({static_object(), b0 * kBox, (b1 - b0) * kBox});
    }
    if (opt.layout == ConstantsLayout::Wvp)
        for (DrawItem& d : draws)
            XMStoreFloat4x4(&d.object.world, XMLoadFloat4x4(&d.object.world) * XMLoadFloat4x4(&cam.view_proj));
    if (opt.layout == ConstantsLayout::ModelView) {
        // Each part's own placement P: the NPC's is its world; level parts get a yaw in 90-degree steps
        // plus a little, a translation, and every third a scale. c0 = P * view, world = world * P^-1.
        // The solver anchors its world on the heaviest draw: that part stays at the origin, so the
        // solved world is the true one and lidar_verify checks it unchanged.
        size_t anchor = opt.npc ? 1 : 0;
        for (size_t i = anchor; i < draws.size(); ++i)
            if (draws[i].count > draws[anchor].count) anchor = i;
        for (size_t i = 0; i < draws.size(); ++i) {
            DrawItem& d = draws[i];
            const XMMATRIX world = XMLoadFloat4x4(&d.object.world);
            XMMATRIX p = world;
            if (i == anchor) {
                p = XMMatrixIdentity();
            } else if (!(opt.npc && i == 0)) {
                const float k = float(i);
                p = XMMatrixScaling(i % 3 == 0 ? 1.5f : 1, 1, i % 3 == 0 ? 0.75f : 1) *
                    XMMatrixRotationY(float(i % 4) * XM_PIDIV2 + 0.05f * k) *
                    XMMatrixTranslation(7.0f * k - 80, float(i % 5), 30 - 3.0f * k);
            }
            XMStoreFloat4x4(&d.model_view, p * XMLoadFloat4x4(&cam.view));
            XMStoreFloat4x4(&d.object.world, world * XMMatrixInverse(nullptr, p));
        }
    }
    return draws;
}

}  // namespace lidar::fake

int main(int argc, char** argv) {
    using namespace lidar::fake;
    Options opt = parse(argc, argv);
    if (opt.cbv_tables && opt.api != Api::D3D12) std::fprintf(stderr, "--cbv-tables only applies to --api d3d12\n");
    if (opt.own_depth && opt.api != Api::D3D9) std::fprintf(stderr, "--own-depth only applies to --api d3d9\n");
    if (opt.up && opt.api != Api::D3D9) std::fprintf(stderr, "--up only applies to --api d3d9\n");
    if (opt.layout == ConstantsLayout::ModelView && opt.api != Api::D3D9) {
        std::fprintf(stderr, "--camera-layout modelview only applies to --api d3d9\n");
        return 2;
    }
    if (opt.decoy_draws != 0 && (opt.api != Api::D3D12 || opt.cbv_tables))
        std::fprintf(stderr, "--decoy-draws only applies to --api d3d12 without --cbv-tables\n");
    if (opt.d24 && opt.api != Api::D3D11) std::fprintf(stderr, "--d24 only applies to --api d3d11\n");
    if (opt.api != Api::D3D11 && opt.publish) {
        std::fprintf(stderr, "%s renders only: --no-publish implied\n", opt.api == Api::D3D12 ? "D3D12" : "D3D9");
        opt.publish = false;
    }
    if (opt.api == Api::D3D12) return run_d3d12(opt);
    if (opt.api == Api::D3D9) return run_d3d9(opt);
    return run_d3d11(opt);
}
