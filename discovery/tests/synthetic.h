// Synthetic games for the discovery tests: a small scene, a camera that stands, walks and turns, and
// constant buffers laid out the way real engines do it, decoys included. generate() produces what the
// live service would hand the analyzer, in the same order (depth a couple of frames behind samples).
#pragma once
#include <DirectXMath.h>

#include <cstdint>
#include <vector>

#include "profile.h"
#include "recording.h"

namespace lidar::test {

// Depth of the scene (ground y = 0, a wall at z = 30, a box) through view * proj, as a protocol frame
// of w x h sampled from sw x sh. Sky is the clear value (1 standard, 0 reversed).
std::vector<float> render_depth(const DirectX::XMMATRIX& view, const DirectX::XMMATRIX& proj, bool reversed,
                                uint32_t w, uint32_t h, uint32_t sw, uint32_t sh);

// Where the game keeps its camera, in vertex b0.
enum class GameLayout {
    ViewAndProj,         // view @ 0, proj @ 64
    ViewProj,            // view * proj @ 0
    InvViewProjAndProj,  // inverse(view * proj) @ 0, proj @ 64 (UE's View buffer, MGS Delta)
    CameraRelative,      // (view without translation) * proj @ 0, camera position @ 128 (UE3, IW)
    WorldViewProj,       // world * view * proj @ 0 per draw; most draws are level geometry (world = I)
};

struct Scenario {
    GameLayout layout = GameLayout::ViewAndProj;
    bool column_major = false;
    int depth_mode = 2;          // test_proj: 0 standard, 1 reversed, 2 reversed-infinite
    uint32_t frames = 360;       // at 60 fps
    uint32_t draws = 40;         // per frame, all sampled
    uint32_t depth_lag = 2;      // frames between a frame's samples and its depth
    bool moving = true;          // false: the camera never moves
    bool decoys = true;          // last frame's view-projection next to the camera, a spot light's
                                 // view + proj, per-object world matrices, junk constants
};

// What discovery should find.
CameraProfile expected_camera(const Scenario& s);

std::vector<disc::RecordedInput> generate(const Scenario& s);

}  // namespace lidar::test
