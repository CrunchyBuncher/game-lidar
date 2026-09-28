#include "camera.h"

#include <algorithm>
#include <cmath>

using namespace DirectX;

namespace lidar {

bool camera_axes(FXMMATRIX inv_view, CXMMATRIX inv_proj, XMVECTOR& apex, XMVECTOR& fwd, XMVECTOR& up,
                 float& half_fov_y) {
    const XMVECTOR vc = XMVector3TransformCoord(XMVectorSet(0, 0, 0.5f, 1), inv_proj);  // view space
    XMVECTOR vt = XMVector3TransformCoord(XMVectorSet(0, 1, 0.5f, 1), inv_proj);
    vt = XMVectorScale(vt, XMVectorGetZ(vc) / XMVectorGetZ(vt));  // the top edge at the center's depth
    apex = XMVector3TransformCoord(XMVectorZero(), inv_view);
    fwd = XMVector3Normalize(XMVector3TransformCoord(vc, inv_view) - apex);
    up = XMVector3Normalize(XMVector3TransformCoord(vt, inv_view) - XMVector3TransformCoord(vc, inv_view));
    half_fov_y = std::atan2(XMVectorGetY(vt), std::fabs(XMVectorGetZ(vc)));
    return !XMVector3IsNaN(fwd) && !XMVector3IsNaN(up) &&
           XMVectorGetX(XMVector3LengthSq(XMVector3Cross(fwd, up))) > 1e-6f;
}

void PlayerCamera::observe(FXMMATRIX view, CXMMATRIX proj, bool posed) {
    XMStoreFloat4x4(&inv_view, XMMatrixInverse(nullptr, view));
    XMStoreFloat4x4(&inv_proj, XMMatrixInverse(nullptr, proj));
    valid = true;
    const XMFLOAT3 p = position();
    if (posed && (trail.empty() || XMVectorGetX(XMVector3Length(XMLoadFloat3(&p) - XMLoadFloat3(&trail.back()))) > 0.25f))
        trail.push_back(p);
}

void ViewTilt::update(const App& app, float dt, XMFLOAT3 pivot_if_tilting) {
    const bool was_level = x == 0 && z == 0;
    const float rate = (app.key_down(VK_SHIFT) ? 40.0f : 10.0f) * dt;
    if (app.key_down(VK_PRIOR)) x += rate;
    if (app.key_down(VK_NEXT)) x -= rate;
    if (app.key_down('Z')) z += rate;
    if (app.key_down('X')) z -= rate;
    // No limit: a capture can come in on its side or upside down. Just keep the angles in range.
    x = std::remainder(x, 360.0f);
    z = std::remainder(z, 360.0f);
    if (app.key_pressed('L')) x = z = 0;
    if (was_level && (x != 0 || z != 0)) pivot = pivot_if_tilting;
}

XMMATRIX ViewTilt::matrix() const {
    return XMMatrixTranslation(-pivot.x, -pivot.y, -pivot.z) * XMMatrixRotationX(XMConvertToRadians(x)) *
           XMMatrixRotationZ(XMConvertToRadians(z)) * XMMatrixTranslation(pivot.x, pivot.y, pivot.z);
}

void ViewTilt::height_axis(float axis[4]) const {
    XMFLOAT4X4 tm;  // the tilted y: the matrix's second column
    XMStoreFloat4x4(&tm, matrix());
    axis[0] = tm._12, axis[1] = tm._22, axis[2] = tm._32, axis[3] = tm._42;
}

ViewParams ViewerCamera::update(const App& app, float dt, const PlayerCamera& player, FXMMATRIX tilt) {
    float fov_y = XMConvertToRadians(60.0f);
    bool attached = false;
    XMVECTOR attach_fwd{}, attach_up{};
    if (attach && player.valid) {
        // Look through the player's camera (as displayed, so tilted) with its full orientation,
        // roll included, and its field of view. Detaching leaves the free-fly camera where the
        // player's was.
        XMVECTOR apex;
        float half;
        attached = camera_axes(XMLoadFloat4x4(&player.inv_view) * tilt, XMLoadFloat4x4(&player.inv_proj), apex,
                               attach_fwd, attach_up, half);
        if (attached) {
            XMFLOAT3 f, p;
            XMStoreFloat3(&f, attach_fwd);
            XMStoreFloat3(&p, apex);
            pos[0] = p.x, pos[1] = p.y, pos[2] = p.z;
            yaw = std::atan2(f.x, f.z);
            pitch = std::clamp(std::asin(std::clamp(f.y, -1.0f, 1.0f)), -1.55f, 1.55f);
            if (std::isfinite(half) && half > 0.05f && half < 1.4f) fov_y = 2.0f * half;
        }
    } else if (follow && player.valid) {
        XMFLOAT4X4 shown;  // the player's camera as displayed (tilted)
        XMStoreFloat4x4(&shown, XMLoadFloat4x4(&player.inv_view) * tilt);
        const XMFLOAT3 p{shown._41, shown._42, shown._43};
        if (app.rmb_down()) {
            follow_yaw = std::remainder(follow_yaw + app.mouse_dx * 0.003f, XM_2PI);
            follow_pitch = std::clamp(follow_pitch - app.mouse_dy * 0.003f, -1.55f, 1.55f);
        }
        follow_dist = std::clamp(follow_dist * std::pow(1.0f / 1.2f, app.wheel), 0.5f, 2000.0f);
        if (app.key_pressed('R')) {
            follow_yaw = 0;
            follow_pitch = kFollowPitch;
            follow_dist = kFollowDist;
        }
        yaw = std::atan2(shown._31, shown._33) + follow_yaw;
        pitch = follow_pitch;
        const float cp = std::cos(pitch);
        pos[0] = p.x - std::sin(yaw) * cp * follow_dist;
        pos[1] = p.y - std::sin(pitch) * follow_dist;
        pos[2] = p.z - std::cos(yaw) * cp * follow_dist;
    } else {
        if (app.rmb_down()) {
            yaw += app.mouse_dx * 0.003f;
            pitch = std::clamp(pitch - app.mouse_dy * 0.003f, -1.55f, 1.55f);
        }
        speed = std::clamp(speed * std::pow(1.2f, app.wheel), 0.5f, 500.0f);
        const float step = speed * (app.key_down(VK_SHIFT) ? 4.0f : 1.0f) * dt;
        const float cp = std::cos(pitch);
        const float f[3] = {std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp};
        const float r[3] = {std::cos(yaw), 0, -std::sin(yaw)};
        for (int k = 0; k < 3; ++k) {
            if (app.key_down('W') || app.key_down(VK_UP)) pos[k] += f[k] * step;
            if (app.key_down('S') || app.key_down(VK_DOWN)) pos[k] -= f[k] * step;
            if (app.key_down('D') || app.key_down(VK_RIGHT)) pos[k] += r[k] * step;
            if (app.key_down('A') || app.key_down(VK_LEFT)) pos[k] -= r[k] * step;
        }
        if (app.key_down('E')) pos[1] += step;
        if (app.key_down('Q')) pos[1] -= step;
    }
    const float cp = std::cos(pitch);
    const XMVECTOR eye = XMVectorSet(pos[0], pos[1], pos[2], 1);
    ViewParams v;
    v.view = attached ? XMMatrixLookToLH(eye, attach_fwd, attach_up)
                      : XMMatrixLookToLH(eye, XMVectorSet(std::sin(yaw) * cp, std::sin(pitch), std::cos(yaw) * cp, 0),
                                         XMVectorSet(0, 1, 0, 0));
    v.proj = XMMatrixPerspectiveFovLH(fov_y, float(app.width) / float(app.height), 10000.0f, 0.05f);  // reversed-Z
    v.fov_y = fov_y;
    return v;
}

}  // namespace lidar
