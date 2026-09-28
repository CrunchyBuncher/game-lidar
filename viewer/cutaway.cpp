#include "cutaway.h"

#include <algorithm>
#include <cstring>

using namespace DirectX;

namespace lidar {

Cutaways make_cutaways(const CutawaySettings& s, const PlayerCamera& player, const ViewerCamera& viewer,
                       FXMMATRIX tilt, const float height_axis[4]) {
    Cutaways c;
    // Attached, the line of sight has no length: it would just hide what's near.
    const bool sight = s.sight_on && !viewer.attach;
    if (!player.valid || !(s.plane_on || sight)) return c;
    // Both are planes, level (up as displayed: the tilted y, which the tilt levels) or square to
    // the camera's own up, through their base point.
    XMVECTOR n = XMVectorSet(height_axis[0], height_axis[1], height_axis[2], 0);
    {
        XMVECTOR apex, fwd, up;
        float half;
        if (s.camera_up &&
            camera_axes(XMLoadFloat4x4(&player.inv_view), XMLoadFloat4x4(&player.inv_proj), apex, fwd, up, half))
            n = up;
    }
    const XMFLOAT3 pp = player.position();
    const XMVECTOR p = XMVectorSet(pp.x, pp.y, pp.z, 1);
    auto plane_through = [&](FXMVECTOR q) {  // positive above q
        XMFLOAT4 f;
        XMStoreFloat4(&f, XMVectorSetW(n, -XMVectorGetX(XMVector3Dot(n, q))));
        return f;
    };
    if (s.plane_on) {  // the player's camera raised by its offset
        const XMFLOAT4 plane = plane_through(p + XMVectorScale(n, s.plane_offset));
        std::memcpy(c.plane, &plane, sizeof(c.plane));
    }
    if (sight) {
        // From the base up to the viewer, hiding only on the viewer's side of the base (above it,
        // unless the viewer is below), so the floor under the player stays. The base starts at
        // the player's camera moved along the line by its offset: toward the viewer, or past
        // the player when negative. It stops short of the viewer.
        const XMVECTOR eye = XMVector3TransformCoord(XMVectorSet(viewer.pos[0], viewer.pos[1], viewer.pos[2], 1),
                                                     XMMatrixInverse(nullptr, tilt));
        const float dist = XMVectorGetX(XMVector3Length(eye - p));
        const XMVECTOR base = p + XMVectorScale(eye - p, std::min(s.sight_offset, 0.9f * dist) / std::max(dist, 1e-6f));
        const XMFLOAT4 plane = plane_through(base);
        const XMVECTOR ab = eye - base;
        XMFLOAT3 a, abf;
        XMStoreFloat3(&a, base);
        XMStoreFloat3(&abf, ab);
        std::memcpy(c.a, &a, sizeof(c.a));
        std::memcpy(c.ab, &abf, sizeof(c.ab));
        c.r2 = s.sight_radius * s.sight_radius;
        c.inv_ab2 = 1.0f / std::max(XMVectorGetX(XMVector3LengthSq(ab)), 1e-8f);
        const float side = XMVectorGetX(XMVector3Dot(n, ab)) < 0 ? -1.0f : 1.0f;
        c.base[0] = side * plane.x, c.base[1] = side * plane.y;
        c.base[2] = side * plane.z, c.base[3] = side * plane.w;
    }
    return c;
}

}  // namespace lidar
