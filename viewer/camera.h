// The viewer's cameras: the game's own camera as the frames report it (PlayerCamera), the one the
// cloud is looked at through (ViewerCamera), and the display-only tilt between the capture's frame
// and what's shown (ViewTilt). No GPU state here.
#pragma once
#include <DirectXMath.h>

#include <vector>

#include "app.h"

namespace lidar {

// A camera's position, forward and up from its inverse view and projection. Forward and up come
// from unprojecting the screen center and top edge, which holds whichever handedness the game
// uses. half_fov_y is the vertical half angle. False if the matrices are degenerate.
bool camera_axes(DirectX::FXMMATRIX inv_view, DirectX::CXMMATRIX inv_proj, DirectX::XMVECTOR& apex,
                 DirectX::XMVECTOR& fwd, DirectX::XMVECTOR& up, float& half_fov_y);

// The game's camera, from the latest frame's matrices, and the path it has taken.
struct PlayerCamera {
    bool valid = false;  // a frame has come in
    DirectX::XMFLOAT4X4 inv_view{}, inv_proj{};
    std::vector<DirectX::XMFLOAT3> trail;  // posed positions, 0.25 m apart

    void observe(DirectX::FXMMATRIX view, DirectX::CXMMATRIX proj, bool posed);
    DirectX::XMFLOAT3 position() const { return {inv_view._41, inv_view._42, inv_view._43}; }
};

// Display-only tilt (degrees about world X and Z) around a pivot, for leveling a tilted scan.
// PgUp/PgDn tilt about world X, Z/X roll about world Z, Shift faster, L levels. .ply files keep the
// capture's frame.
struct ViewTilt {
    float x = 0, z = 0;
    DirectX::XMFLOAT3 pivot{0, 0, 0};

    // pivot_if_tilting: where to pivot if the view leaves level this frame (so the scene doesn't
    // swing away).
    void update(const App& app, float dt, DirectX::XMFLOAT3 pivot_if_tilting);
    DirectX::XMMATRIX matrix() const;  // capture frame -> as shown
    // A point's height as shown: dot((pos, 1), axis), the tilted y.
    void height_axis(float axis[4]) const;
};

struct ViewParams {
    DirectX::XMMATRIX view, proj;  // from the shown (tilted) frame
    float fov_y;
};

// Free-fly by default. Follow orbits the player's camera, from behind their heading by default:
// right-drag swings it around (yaw relative to the heading, so it still turns with them), the wheel
// zooms, R resets. Attach looks through the player's camera.
struct ViewerCamera {
    static constexpr float kFollowDist = 8.94f, kFollowPitch = -0.46f;  // 8 m back and 4 m up, looking at them

    bool follow = false, attach = false;  // at most one
    // Start above the level looking down at it.
    float pos[3] = {0.0f, 45.0f, -75.0f};
    float yaw = 0.0f, pitch = -0.5f, speed = 10.0f;
    float follow_yaw = 0.0f, follow_pitch = kFollowPitch, follow_dist = kFollowDist;

    void toggle_follow() {
        follow = !follow;
        if (follow) attach = false;
    }
    void toggle_attach() {
        attach = !attach;
        if (attach) follow = false;
    }
    // Moves the camera for this frame's input and returns what to draw with.
    ViewParams update(const App& app, float dt, const PlayerCamera& player, DirectX::FXMMATRIX tilt);
};

}  // namespace lidar
