// Display-only cutaways that clear the view of the player. The plane hides everything above the
// player's camera plus an offset, level (as displayed) or square to the camera's up. The cylinder
// hides everything within a radius of the line from the viewer's camera to the player's, down to
// a base plane of the same orientation, which its own offset slides along that line.
#pragma once
#include <DirectXMath.h>

#include "camera.h"

namespace lidar {

struct CutawaySettings {
    bool plane_on = false, sight_on = false;
    bool camera_up = false;  // both bases square to the camera's up instead of level
    float plane_offset = 0.5f, sight_radius = 1.5f, sight_offset = 0.0f;
};

// As the point shader takes them, in the capture's frame (the points before the tilt). The
// defaults never hide anything.
struct Cutaways {
    float plane[4] = {0, 0, 0, -1};  // hides points where dot((pos, 1), plane) > 0
    float a[3] = {}, r2 = 0;         // the cylinder from a to a + ab; radius 0 is off
    float ab[3] = {}, inv_ab2 = 0;
    float base[4] = {};  // the cylinder hides only where dot((pos, 1), base) > 0
};

// The cylinder runs toward the viewer camera's position; it's off while attached to the player's
// camera. height_axis: see ViewTilt::height_axis.
Cutaways make_cutaways(const CutawaySettings& s, const PlayerCamera& player, const ViewerCamera& viewer,
                       DirectX::FXMMATRIX tilt, const float height_axis[4]);

}  // namespace lidar
