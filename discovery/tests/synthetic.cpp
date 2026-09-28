#include "synthetic.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "matrices.h"
#include "unproject.h"

using namespace DirectX;

namespace lidar::test {

std::vector<float> render_depth(const XMMATRIX& view, const XMMATRIX& proj, bool reversed, uint32_t w, uint32_t h,
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

namespace {

constexpr uint32_t kCameraBytes = 512, kObjectBytes = 128, kLightBytes = 256;
constexpr uint32_t kW = 160, kH = 90, kSrcW = 1280, kSrcH = 720;
constexpr double kFps = 60;

// Still at the start and twice on the way (the still/moving check needs both), otherwise walking
// forward, swaying and turning. Stays in front of the box.
struct Pose {
    XMVECTOR eye;
    XMMATRIX view;
};
Pose pose_at(uint32_t frame, bool moving) {
    uint32_t t = 0;  // moving frames so far
    for (uint32_t f = 0; f < frame && moving; ++f) t += !(f < 30 || (f >= 150 && f < 180) || (f >= 270 && f < 290));
    const float u = float(t);
    const XMVECTOR eye = XMVectorSet(0.8f * std::sin(u * 0.02f), 1.7f, -4.0f + 0.035f * u, 1);
    const float yaw = 0.35f * std::sin(u * 0.045f);
    return {eye, XMMatrixLookToLH(eye, XMVectorSet(std::sin(yaw), -0.15f, std::cos(yaw), 0), XMVectorSet(0, 1, 0, 0))};
}

// TAA-style sub-pixel jitter, a different offset every frame.
XMMATRIX jittered(const XMMATRIX& proj, uint32_t frame) {
    static const float kOffsets[4][2] = {{0.25f, -0.25f}, {-0.25f, 0.25f}, {0.125f, 0.375f}, {-0.375f, -0.125f}};
    XMFLOAT4X4 f;
    XMStoreFloat4x4(&f, proj);
    f._31 += 2 * kOffsets[frame % 4][0] / kSrcW;
    f._32 += 2 * kOffsets[frame % 4][1] / kSrcH;
    return XMLoadFloat4x4(&f);
}

// An object's world matrix: rigid, somewhere in the scene, turning slowly.
XMMATRIX object_world(uint32_t draw, uint32_t frame) {
    return XMMatrixRotationY(0.01f * float(frame) + float(draw)) *
           XMMatrixTranslation(float(draw % 7) - 3, 0.5f, 10 + float(draw % 5));
}

cam::BoundBuffer buffer(reshade::api::shader_stage stage, uint32_t slot, std::vector<uint8_t> bytes) {
    cam::BoundBuffer b;
    b.key = {stage, slot, 0, uint32_t(bytes.size())};
    b.read.bytes = std::move(bytes);
    b.read.ready = true;
    b.read.source_size = b.key.size;
    return b;
}

void put_floats(std::vector<uint8_t>& buf, uint32_t offset, const float* f, size_t n) {
    std::memcpy(buf.data() + offset, f, n * sizeof(float));
}

}  // namespace

CameraProfile expected_camera(const Scenario& s) {
    CameraProfile p;
    p.key = {reshade::api::shader_stage::vertex, 0, 0, kCameraBytes};
    p.column_major = s.column_major;
    p.view_offset = 0;
    p.proj_offset = 64;
    switch (s.layout) {
        case GameLayout::ViewAndProj: p.layout = CameraLayout::ViewAndProj; break;
        case GameLayout::ViewProj: p.layout = CameraLayout::ViewProj; break;
        case GameLayout::InvViewProjAndProj: p.layout = CameraLayout::InvViewProjAndProj; break;
        case GameLayout::CameraRelative:
            p.layout = CameraLayout::ViewProj;
            p.has_translation = true;
            p.translation_offset = 128;
            p.translation_subtract = true;
            break;
        case GameLayout::WorldViewProj:
            p.layout = CameraLayout::ViewProj;
            p.latch = Latch::Common;
            break;
    }
    return p;
}

std::vector<disc::RecordedInput> generate(const Scenario& s) {
    using reshade::api::shader_stage;
    const XMMATRIX base_proj = test_proj(s.depth_mode, float(kSrcW) / kSrcH);
    // A spot light looking down at the box: a perspective view + proj that never moves.
    const XMMATRIX light_view =
        XMMatrixLookAtLH(XMVectorSet(4, 12, 6, 1), XMVectorSet(0, 0, 10, 1), XMVectorSet(0, 1, 0, 0));
    const XMMATRIX light_proj = XMMatrixPerspectiveFovLH(0.9f, 1, 0.5f, 60);

    std::vector<disc::RecordedInput> out;
    std::vector<uint8_t> prev_first(64);  // last frame's matrix at camera offset 0 (level geometry's)
    for (uint32_t f = 0; f < s.frames + s.depth_lag; ++f) {
        const double time = f / kFps;
        if (f < s.frames) {
            const Pose pose = pose_at(f, s.moving);
            const XMMATRIX proj = jittered(base_proj, f), vp = pose.view * proj;
            disc::RecordedInput in;
            in.samples.frame = f;
            in.samples.time = time;
            cam::DepthSamples& d = in.samples.samples;
            d.draws = s.draws;
            std::vector<uint8_t> first_value;
            for (uint32_t i = 0; i < s.draws; ++i) {
                cam::DrawSample& sample = d.samples.emplace_back();
                sample.draw = i;
                sample.call = {cam::DrawType::Indexed, 300 + 36 * i, 1, 0, 0, 0};
                // Every fourth draw is an object, the rest level geometry.
                const bool object = i % 4 == 3;
                const XMMATRIX world = object ? object_world(i, f) : XMMatrixIdentity();

                std::vector<uint8_t> cam(kCameraBytes);
                switch (s.layout) {
                    case GameLayout::ViewAndProj:
                        put(cam.data(), 0, pose.view, s.column_major);
                        put(cam.data(), 64, proj, s.column_major);
                        break;
                    case GameLayout::ViewProj: put(cam.data(), 0, vp, s.column_major); break;
                    case GameLayout::InvViewProjAndProj:
                        put(cam.data(), 0, XMMatrixInverse(nullptr, vp), s.column_major);
                        put(cam.data(), 64, proj, s.column_major);
                        break;
                    case GameLayout::CameraRelative: {
                        XMMATRIX rotation = pose.view;
                        rotation.r[3] = XMVectorSet(0, 0, 0, 1);
                        put(cam.data(), 0, rotation * proj, s.column_major);
                        float eye[4] = {XMVectorGetX(pose.eye), XMVectorGetY(pose.eye), XMVectorGetZ(pose.eye), 1};
                        put_floats(cam, 128, eye, 4);
                        break;
                    }
                    case GameLayout::WorldViewProj: put(cam.data(), 0, world * vp, s.column_major); break;
                }
                if (i == 0) first_value.assign(cam.begin(), cam.begin() + 64);
                if (s.decoys) {
                    put_floats(cam, 192, reinterpret_cast<const float*>(prev_first.data()), 16);
                    const float misc[8] = {float(time), float(f), 1.0f / 60, 0, kSrcW, kSrcH, 1.0f / kSrcW, 1.0f / kSrcH};
                    put_floats(cam, 320, misc, 8);
                }
                sample.buffers.push_back(buffer(shader_stage::vertex, 0, std::move(cam)));
                if (s.decoys) {
                    std::vector<uint8_t> obj(kObjectBytes);
                    put(obj.data(), 0, world, s.column_major);
                    const float tint[4] = {0.9f, 0.25f, 0.2f, 1};
                    put_floats(obj, 64, tint, 4);
                    sample.buffers.push_back(buffer(shader_stage::vertex, 1, std::move(obj)));
                    std::vector<uint8_t> light(kLightBytes);
                    put(light.data(), 0, light_view, s.column_major);
                    put(light.data(), 64, light_proj, s.column_major);
                    sample.buffers.push_back(buffer(shader_stage::pixel, 0, std::move(light)));
                }
            }
            prev_first = first_value;
            out.push_back(std::move(in));
        }
        if (f >= s.depth_lag) {
            const uint32_t df = f - s.depth_lag;
            const Pose pose = pose_at(df, s.moving);
            const std::vector<float> depth =
                render_depth(pose.view, jittered(base_proj, df), s.depth_mode != 0, kW, kH, kSrcW, kSrcH);
            disc::RecordedInput in;
            in.is_depth = true;
            in.depth = std::make_shared<disc::DepthFrame>();
            in.depth->frame = df;
            in.depth->time = time;
            in.depth->grid.build(depth.data(), kW, kH, kSrcW, kSrcH, std::max(1u, kW / 128));
            out.push_back(std::move(in));
        }
    }
    return out;
}

}  // namespace lidar::test
