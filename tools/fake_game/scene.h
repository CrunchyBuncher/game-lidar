// Deterministic test level for the fake game, shared with lidar_verify so
// captured points can be checked against the true geometry.
// World: left-handed, Y up, meters. The camera path runs along a square
// "road" at x/z = +-30.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace lidar::scene {

struct Box {
    float min[3];
    float max[3];
    uint32_t color;  // RGBA8, R in low byte
};

inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) { return r | (g << 8) | (b << 16) | 0xFF000000u; }

inline std::vector<Box> build() {
    std::vector<Box> boxes;
    auto add = [&](float x0, float y0, float z0, float x1, float y1, float z1, uint32_t c) {
        boxes.push_back({{x0, y0, z0}, {x1, y1, z1}, c});
    };
    uint32_t seed = 12345;
    auto rnd = [&]() {  // [0,1)
        seed = seed * 1664525u + 1013904223u;
        return float(seed >> 8) / float(1 << 24);
    };

    // Ground and perimeter walls.
    add(-40.5f, -0.2f, -40.5f, 40.5f, 0.0f, 40.5f, rgb(120, 125, 118));
    const uint32_t wall = rgb(150, 110, 80);
    add(-40.5f, 0, 40.0f, 40.5f, 6, 40.5f, wall);
    add(-40.5f, 0, -40.5f, 40.5f, 6, -40.0f, wall);
    add(40.0f, 0, -40.0f, 40.5f, 6, 40.0f, wall);
    add(-40.5f, 0, -40.0f, -40.0f, 6, 40.0f, wall);

    // Inner "city" blocks.
    const uint32_t palette[] = {rgb(200, 190, 170), rgb(170, 190, 210), rgb(210, 170, 160), rgb(180, 200, 170),
                                rgb(220, 210, 150)};
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            const float cx = i * 9.0f, cz = j * 9.0f;
            if (i == 0 && j == 0) {  // plaza with a tower
                add(-1.0f, 0, -1.0f, 1.0f, 20.0f, 1.0f, rgb(230, 230, 235));
                continue;
            }
            if (rnd() < 0.15f) continue;  // empty lot
            const float h = 3.0f + rnd() * 11.0f;
            const float hw = 2.5f + rnd() * 0.8f;
            add(cx - hw, 0, cz - hw, cx + hw, h, cz + hw, palette[(i + 2 + (j + 2) * 3) % 5]);
        }
    }

    // Crates along the outer band, outside the road.
    const uint32_t crate = rgb(200, 140, 60);
    for (float t = -34.0f; t <= 34.0f; t += 6.0f) {
        for (int side = 0; side < 4; ++side) {
            if (rnd() < 0.3f) continue;
            const float s = 0.8f + rnd() * 0.6f, h = 1.0f + rnd() * 2.0f;
            const float off = 36.5f + rnd() * 1.5f;
            float x = t, z = t;
            if (side == 0) z = off;
            if (side == 1) z = -off;
            if (side == 2) x = off;
            if (side == 3) x = -off;
            add(x - s, 0, z - s, x + s, h, z + s, crate);
        }
    }

    // Gate over the east road.
    const uint32_t gate = rgb(90, 90, 110);
    add(25.0f, 0, -0.5f, 26.0f, 5.0f, 0.5f, gate);
    add(34.0f, 0, -0.5f, 35.0f, 5.0f, 0.5f, gate);
    add(25.0f, 5.0f, -0.5f, 35.0f, 5.8f, 0.5f, gate);

    // Tunnel over the north road.
    const uint32_t tunnel = rgb(110, 120, 100);
    add(-10.0f, 4.0f, 25.0f, 10.0f, 4.5f, 35.0f, tunnel);
    add(-10.0f, 0, 25.0f, 10.0f, 4.0f, 25.5f, tunnel);
    add(-10.0f, 0, 34.5f, 10.0f, 4.0f, 35.0f, tunnel);

    // Curbs along the south road.
    for (float x = -24.0f; x <= 24.0f; x += 8.0f) add(x - 2.0f, 0, -34.2f, x + 2.0f, 0.3f, -33.8f, rgb(170, 170, 170));

    return boxes;
}

// Unsigned distance from p to the nearest box surface.
inline float distance(const std::vector<Box>& boxes, const float p[3]) {
    float best = 1e30f;
    for (const Box& b : boxes) {
        float q[3], outside2 = 0, inside = -1e30f;
        for (int k = 0; k < 3; ++k) {
            const float c = 0.5f * (b.min[k] + b.max[k]), h = 0.5f * (b.max[k] - b.min[k]);
            q[k] = std::fabs(p[k] - c) - h;
            outside2 += std::max(q[k], 0.0f) * std::max(q[k], 0.0f);
            inside = std::max(inside, q[k]);
        }
        const float sd = std::sqrt(outside2) + std::min(inside, 0.0f);
        best = std::min(best, std::fabs(sd));
    }
    return best;
}

// Scripted camera: a 360-degree turn in place, then laps of the road.
struct Pose {
    float pos[3];
    float yaw, pitch;  // radians; yaw 0 looks along +Z, positive yaw turns toward +X
};

inline Pose camera_path(float t) {
    constexpr float kPi = 3.14159265f;
    constexpr float kSpinTime = 8.0f, kSpeed = 4.0f, kEye = 1.7f;
    const float corners[4][2] = {{-30, -30}, {30, -30}, {30, 30}, {-30, 30}};
    if (t < kSpinTime) {
        return {{-30.0f, kEye, -30.0f}, (t / kSpinTime) * 2 * kPi + kPi * 0.5f, 0.0f};
    }
    const float lap = 240.0f;  // 4 sides x 60 m
    float d = std::fmod((t - kSpinTime) * kSpeed, lap);
    const int side = int(d / 60.0f);
    const float f = (d - side * 60.0f) / 60.0f;
    const float* a = corners[side];
    const float* b = corners[(side + 1) % 4];
    Pose p;
    p.pos[0] = a[0] + (b[0] - a[0]) * f;
    p.pos[1] = kEye + 0.05f * std::sin(t * 9.0f);  // head bob
    p.pos[2] = a[1] + (b[1] - a[1]) * f;
    const float heading = std::atan2(b[0] - a[0], b[1] - a[1]);
    // Glance around while walking, turning into corners smoothly.
    const float corner_blend = std::clamp((f - 0.9f) / 0.1f, 0.0f, 1.0f);
    p.yaw = heading - corner_blend * (kPi * 0.5f) + 0.6f * std::sin(t * 0.8f);
    p.pitch = 0.15f * std::sin(t * 0.5f);
    return p;
}

}  // namespace lidar::scene
