// The model-view camera solver (plan_modelview.md MV1). A synthetic level at SA2's scale (~2000
// units from the origin), row-vector matrices rounded to float like a game uploads them.
#include "modelview.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "camera_math.h"
#include "test.h"

using namespace lidar;

namespace mvt {

using mat::Mat;

Mat translate(double x, double y, double z) {
    Mat r = mat::identity();
    r.m[3][0] = x, r.m[3][1] = y, r.m[3][2] = z;
    return r;
}
Mat scale(double x, double y, double z) {
    Mat r = mat::identity();
    r.m[0][0] = x, r.m[1][1] = y, r.m[2][2] = z;
    return r;
}
Mat yaw(double a) {
    Mat r = mat::identity();
    r.m[0][0] = std::cos(a), r.m[0][2] = -std::sin(a), r.m[2][0] = std::sin(a), r.m[2][2] = std::cos(a);
    return r;
}
Mat pitch(double a) {
    Mat r = mat::identity();
    r.m[1][1] = std::cos(a), r.m[1][2] = std::sin(a), r.m[2][1] = -std::sin(a), r.m[2][2] = std::cos(a);
    return r;
}
Mat inv(const Mat& a) {
    Mat r;
    mat::inverse(a, r);
    return r;
}
Mat rounded(Mat a) {
    for (auto& row : a.m)
        for (double& v : row) v = double(float(v));
    return a;
}

struct Level {
    uint64_t first_key = 0;
    double c[3] = {};
    double spacing = 0;  // between pieces; the camera laps at 3x, so view distances reach ~6x
    std::vector<Mat> pieces;  // world placements
    std::vector<double> weights;
};

// 64 pieces on an 8x8 grid, in 90-degree yaw steps plus a little, some scaled (uniformly, or not).
// Piece 10 is the heaviest, so it anchors the segment.
Level make_level(uint64_t first_key, double cx, double cy, double cz, double spacing, double twist,
                 bool scaled_anchor) {
    Level l;
    l.first_key = first_key;
    l.c[0] = cx, l.c[1] = cy, l.c[2] = cz;
    l.spacing = spacing;
    for (int i = 0; i < 64; ++i) {
        Mat s = mat::identity();
        if (i % 5 == 1) s = scale(1.5, 1.5, 1.5);
        if (i % 7 == 2 || (i == 10 && scaled_anchor)) s = scale(2, 1, 0.5);
        const Mat w = mat::mul(mat::mul(s, yaw((i % 4) * 1.5707963267948966 + 0.01 * i + twist)),
                               translate(cx + (i % 8 - 3.5) * spacing, cy + (i * 37 % 11) * spacing / 20,
                                         cz + (i / 8 - 3.5) * spacing));
        l.pieces.push_back(w);
        l.weights.push_back(i == 10 ? 1000 : 50 + i);
    }
    return l;
}

// Still for 60 frames, then laps of 600 frames around the level's center, looking along the path
// (so pieces come into and out of view) and a little down.
Mat camera(const Level& l, uint32_t f) {
    const double t = f < 60 ? 0 : double((f - 60) % 600) / 600 * 6.283185307179586;
    const Mat c = mat::mul(mat::mul(pitch(0.15), yaw(-t - 0.3)),
                           translate(l.c[0] + 3 * l.spacing * std::cos(t), l.c[1] + l.spacing,
                                     l.c[2] + 3 * l.spacing * std::sin(t)));
    return c;  // camera to world; the view is its inverse
}

bool visible(const Mat& w, const Mat& view) {
    const double p[4] = {w.m[3][0], w.m[3][1], w.m[3][2], 1};
    double v[4];
    mat::transform(p, view, v);
    return v[2] > 1 && std::hypot(v[0], v[1]) < v[2] * 1.7;
}

// The frame's draws: the visible pieces, an NPC walking in circles, a sky that follows the camera,
// a HUD quad, a prop drawn at three places, and five particles with keys never seen again.
std::vector<mv::Draw> draws(const Level& l, uint32_t f, const Mat& view, const Mat& cam, uint32_t& rng) {
    std::vector<mv::Draw> out;
    for (size_t i = 0; i < l.pieces.size(); ++i)
        if (visible(l.pieces[i], view))
            out.push_back({l.first_key + i, rounded(mat::mul(l.pieces[i], view)), l.weights[i]});
    const Mat npc = mat::mul(yaw(f * 0.03), translate(l.c[0] + 1.5 * l.spacing * std::cos(f * 0.01), l.c[1] + 5,
                                                     l.c[2] + 1.5 * l.spacing * std::sin(f * 0.01)));
    out.push_back({1, rounded(mat::mul(npc, view)), 800});
    out.push_back({2, rounded(mat::mul(translate(cam.m[3][0], cam.m[3][1], cam.m[3][2]), view)), 60});
    out.push_back({3, translate(0, 0, 5), 4});
    for (int k = 0; k < 3; ++k)
        out.push_back({4, rounded(mat::mul(translate(l.c[0] + k * l.spacing / 2, l.c[1], l.c[2]), view)), 36});
    for (uint32_t j = 0; j < 5; ++j) {
        auto next = [&] { return double((rng = rng * 1664525u + 1013904223u) >> 8) / (1 << 24) * 6 - 3; };
        const double x = next() * l.spacing, y = next() * l.spacing, z = next() * l.spacing;
        const Mat w = translate(l.c[0] + x, l.c[1] + y, l.c[2] + z);
        out.push_back({1000000 + uint64_t(f) * 5 + j, rounded(mat::mul(w, view)), 4});
    }
    return out;
}

// Runs `frames` frames of `l` through the solver. Our world differs from the true one by a fixed
// transform G per segment (the anchor's placement), found from the segment's first frame, so every
// later frame is checked against G * V_true: drift shows as a growing error.
struct Run {
    uint32_t unposed = 0, segments_started = 0, max_ambiguous = 0;
    double max_pos = 0, max_rot = 0;  // camera position error / 2000, rotation error in degrees
    double last_lap_pos = 0;          // over the final lap
    Mat gauge = mat::identity();
    mv::Result last;
};
Run run(mv::Solver& solver, const Level& l, uint32_t frames, uint32_t& rng) {
    Run r;
    bool have_gauge = false;
    for (uint32_t f = 0; f < frames; ++f) {
        const Mat cam = camera(l, f);
        const Mat view = inv(cam);
        const mv::Result res = solver.solve(draws(l, f, view, cam, rng));
        r.last = res;
        r.max_ambiguous = std::max(r.max_ambiguous, res.ambiguous);
        r.segments_started += res.new_segment;
        if (!res.posed) {
            ++r.unposed;
            continue;
        }
        if (res.new_segment) {
            r.gauge = mat::mul(res.view, cam);  // G = V_solved * V_true^-1
            have_gauge = true;
        }
        if (!have_gauge) continue;
        const Mat expect = mat::mul(r.gauge, view);
        const Mat cam_s = inv(res.view), cam_e = inv(expect);
        const double pos = std::hypot(cam_s.m[3][0] - cam_e.m[3][0], cam_s.m[3][1] - cam_e.m[3][1],
                                      cam_s.m[3][2] - cam_e.m[3][2]) / 2000;
        r.max_pos = std::max(r.max_pos, pos);
        r.max_rot = std::max(r.max_rot, mat::rotation_between_deg(res.view, expect));
        if (f + 600 >= frames) r.last_lap_pos = std::max(r.last_lap_pos, pos);
    }
    return r;
}

}  // namespace mvt

TEST_CASE(modelview, solver) {
    uint32_t rng = 12345;
    mv::Solver solver;

    // 10k frames: 16 laps. Every frame gets a pose, in one segment, within 1e-5 of the truth.
    const mvt::Level a = mvt::make_level(100, 1500, 40, -1800, 500, 0, false);
    const mvt::Run ra = mvt::run(solver, a, 10000, rng);
    std::printf("modelview: level A %u segments, %u unposed, max error pos %.2e (last lap %.2e) rot %.2e deg; "
                "%u static, %u provisional, %u dynamic, %zu objects\n",
                ra.segments_started, ra.unposed, ra.max_pos, ra.last_lap_pos, ra.max_rot, ra.last.statics,
                ra.last.provisional, ra.last.dynamic, solver.objects());
    EXPECT(ra.segments_started == 1);
    EXPECT(ra.unposed == 0);
    EXPECT(ra.max_pos < 1e-5);
    EXPECT(ra.max_rot < 1e-3);
    EXPECT(ra.max_ambiguous == 1);  // the prop
    // The anchor is yaw-only: our world keeps the game's up.
    EXPECT(std::abs(ra.gauge.m[1][1] - 1) < 1e-6);
    // Moving things are voted out, the repeated prop is never placed, the level is static.
    EXPECT(solver.state(1) == mv::State::Dynamic);  // NPC
    EXPECT(solver.state(2) == mv::State::Dynamic);  // sky (static while the camera stood still)
    EXPECT(solver.state(3) == mv::State::Dynamic);  // HUD
    EXPECT(solver.state(4) == mv::State::Unknown);  // prop
    uint32_t statics = 0;
    for (uint64_t i = 0; i < 64; ++i) statics += solver.state(a.first_key + i) == mv::State::Static;
    EXPECT(statics >= 60);
    // Particles (a new key every frame) are forgotten.
    EXPECT(solver.objects() < 64 + 3 + 5 * (mv::Settings{}.forget_after + 64));

    // A level change: all new keys (the HUD, NPC and sky keys stay). A new segment starts at once,
    // with a scaled anchor: the view stays rigid, and our frame still keeps the game's up.
    const mvt::Level b = mvt::make_level(5000, -3000, 0, 4000, 300, 0.4, true);
    const mvt::Run rb = mvt::run(solver, b, 1300, rng);
    std::printf("modelview: level B %u segments, %u unposed, max error pos %.2e rot %.2e deg, segment %u\n",
                rb.segments_started, rb.unposed, rb.max_pos, rb.max_rot, rb.last.segment);
    EXPECT(rb.segments_started == 1);
    EXPECT(rb.unposed == 0);
    EXPECT(rb.last.segment == 2);
    EXPECT(rb.max_pos < 1e-5);
    EXPECT(rb.max_rot < 1e-3);
    EXPECT(is_rigid(rb.last.view));
    EXPECT(std::abs(rb.gauge.m[1][1] - 1) < 1e-6);
    EXPECT(solver.state(a.first_key) == mv::State::Unknown);  // level A forgotten

    // Too few objects: no pose, and no segment from nothing.
    mv::Solver empty;
    const mv::Result r0 = empty.solve({{7, mvt::translate(0, 0, 10), 1}});
    EXPECT(!r0.posed && r0.segment == 0);

    // A mirrored heaviest draw (negative scale) doesn't anchor: the world would come out mirrored.
    mv::Solver mirror;
    const mv::Result rm = mirror.solve({{20, mat::mul(mvt::scale(-1, 1, 1), mvt::translate(0, 0, 10)), 1000},
                                        {21, mvt::translate(1, 0, 12), 10},
                                        {22, mvt::translate(-1, 0, 14), 10}});
    const mat::Mat& m = rm.view;
    const double det = m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) -
                       m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0]) +
                       m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]);
    EXPECT(rm.posed && rm.new_segment && det > 0);

    // Level geometry drawn with an identity world matrix (three keys, the same model-view: the view)
    // anchors the world, not the heavier tilted rock, and not the HUD quads sharing the identity:
    // the solved view is the game's own, so up stays up after every reset.
    const mat::Mat view = mvt::inv(mat::mul(mat::mul(mvt::pitch(0.2), mvt::yaw(0.7)), mvt::translate(40, 12, -30)));
    const mat::Mat rock = mat::mul(mat::mul(mvt::pitch(0.5), mvt::translate(45, 10, -20)), view);
    const mat::Mat eye = mat::identity();
    for (int reset = 0; reset < 2; ++reset) {
        mv::Solver shared;
        const mv::Result rs = shared.solve({{30, view, 100}, {31, view, 80}, {32, view, 60}, {33, rock, 5000},
                                            {34, eye, 4}, {35, eye, 4}, {36, eye, 4}, {37, eye, 4}});
        EXPECT(rs.posed && rs.new_segment && rs.anchor_shared == 3);
        EXPECT(std::memcmp(&rs.view, &view, sizeof(view)) == 0 || mat::rotation_between_deg(rs.view, view) < 1e-9);
        EXPECT(mat::translation_between(rs.view, view) < 1e-9);
    }
}
