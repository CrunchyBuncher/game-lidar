// Shared between the fake game's D3D11, D3D12 and D3D9 renderers: options, the scene shader and
// geometry, and the camera (scripted path or manual).
#pragma once
#include <DirectXMath.h>

#include <cstdint>
#include <string>
#include <vector>

#include "app.h"
#include "scene.h"

namespace lidar::fake {

// Camera cbuffer in b0 (vertex + pixel), per-object decoy in b1. Same source for D3D11 and D3D12
// (D3D9 has its own, with the same layout in constant registers).
inline const char* kSceneHlsl = R"(
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

struct CameraCB {
    DirectX::XMFLOAT4X4 view, proj, view_proj;
    DirectX::XMFLOAT4 cam_pos;
};
struct ObjectCB {
    DirectX::XMFLOAT4X4 world;
    DirectX::XMFLOAT4 tint;
};
struct Vertex {
    DirectX::XMFLOAT3 pos, nrm;
    uint32_t color;
};

enum class DepthMode { Standard, Reversed, ReversedInfinite };
enum class Api { D3D11, D3D12, D3D9 };
// How the camera reaches the shader (the shader always computes world * view * proj):
//  Separate: view, proj and view_proj in b0 (c0-c12), the usual case.
//  ViewProj: only view_proj: b0 holds {view_proj, identity, view_proj}.
//  Wvp:      only world * view * proj, per draw, in b1's world (c13); b0's matrices are identity.
//            Like many D3D9 games. The level's draws have world = identity, the NPC's doesn't.
//  ModelView: D3D9 only. Only world * view, per draw, in c0 (b0's view), proj constant in c4, no
//            view_proj. The level is split into many parts, each with its own placement (translated,
//            yawed, some scaled), so no draw ever has the camera alone. Like Sonic Adventure 2.
enum class ConstantsLayout { Separate, ViewProj, Wvp, ModelView };

struct Options {
    Api api = Api::D3D11;
    ConstantsLayout layout = ConstantsLayout::Separate;
    bool cbv_tables = false;  // D3D12: bind the cbuffers through a descriptor table instead of root CBVs
    bool d3d12_debug = false;  // D3D12: enable the debug layer, print its messages, exit 3 on errors
    bool own_depth = false;    // D3D9: CreateDepthStencilSurface instead of the auto depth-stencil
    bool up = false;           // D3D9: Draw[Indexed]PrimitiveUP from system memory instead of a vertex buffer
    bool d24 = false;          // D3D11: 24-bit depth buffer (a reference for D3D9, whose depth is 24-bit)
    DepthMode depth = DepthMode::Reversed;
    uint32_t capture_width = 480;
    uint32_t capture_every = 1;
    float fov_deg = 70.0f;
    bool npc = true;
    bool color = true;
    float tint_until = 0;  // published color is magenta until this many seconds in (D3D11)
    int width = 1280, height = 720;
    float duration = 0;  // 0 = run until closed
    bool publish = true;
    std::wstring ring;
    float freeze = -1;  // < 0: camera follows the path
    uint32_t decoy_draws = 0;  // D3D12 root CBVs: draws before the scene with other constants at b0
    bool background = false;   // open behind other windows without taking the focus
};

Options parse(int argc, char** argv);

// The static level, then the NPC box (drawn separately, with its own world matrix).
struct Geometry {
    std::vector<Vertex> verts;
    uint32_t static_count = 0;
};
Geometry build_geometry();

DirectX::XMMATRIX make_proj(DepthMode mode, float fov, float aspect);

// Scripted path, or manual fly (M toggles; WASD/QE + right-drag).
struct CameraRig {
    scene::Pose pose = scene::camera_path(0);
    bool manual = false;
    double path_time = 0;

    void update(const App& app, const Options& opt, float dt);
    CameraCB constants(const Options& opt, float aspect) const;
};

// World matrix and tint of the NPC at time t.
ObjectCB npc_object(double t);
ObjectCB static_object();

// The frame's draws: the NPC first (if on), then the level in three parts, so the level's
// constants are what most draws see. Objects as uploaded for opt.layout.
constexpr uint32_t kMaxDraws = 4;
struct DrawItem {
    ObjectCB object;
    uint32_t first = 0, count = 0;  // vertices
    DirectX::XMFLOAT4X4 model_view{};  // ModelView: placement * view, for c0; object.world then holds
                                       // world * placement^-1, so the vertices still land in place
};
std::vector<DrawItem> scene_draws(const Options& opt, const Geometry& geo, const CameraCB& cam, double t);
// The camera constants as uploaded for opt.layout. `cam` (the true matrices) is what gets published.
CameraCB gpu_camera(const Options& opt, const CameraCB& cam);

int run_d3d11(const Options& opt);
int run_d3d12(const Options& opt);
int run_d3d9(const Options& opt);

}  // namespace lidar::fake
