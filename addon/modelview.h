// Camera recovery from per-draw model-view matrices (plan_modelview.md), for games that never
// upload the camera alone. Pure (no ReShade calls), so the unit tests drive it with synthetic scenes.
//
// Row-vector convention (matrix.h): a draw's matrix is M = W * V, the object's world placement
// then the view. For a static object W is fixed, so once its placement is known, every frame it's
// drawn in gives a view V = W^-1 * M. Static objects agree on it; moving ones (the player, enemies,
// a sky that follows the camera, the HUD) don't, and get voted out.
//
// The world frame is ours, not the game's: a segment starts from one anchor object (the heaviest
// draw, e.g. the most vertices: level geometry). Every other object is placed from the view the
// known ones solve, W = M * V^-1, averaged while provisional, and frozen once it has agreed for
// `promote_after` frames. Frozen placements keep the frame from drifting while they stay in view;
// only the chain into newly seen areas can add error. A camera cut to a place with no known
// object, or a long stretch without consensus, starts a new segment (a new, unrelated frame).
#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "matrix.h"

namespace lidar::mv {

struct Draw {
    uint64_t key = 0;   // what identifies the object across frames (the capture's hash of the draw)
    mat::Mat m;         // model-view, row-vector form (M = W * V)
    double weight = 1;  // e.g. the vertex count: the segment's anchor is the heaviest draw
};

struct Settings {
    double rotation_tol_deg = 0.05;   // two views agree within this rotation ...
    double translation_tol = 1e-4;    // ... and translation, relative to the frame's scene scale
    uint32_t min_inliers = 3;         // a pose needs this many agreeing objects ...
    double min_support = 0.3;         // ... and this share of the known ones present (by vote)
    uint32_t promote_after = 10;      // frames a new object must agree before its placement freezes
    uint32_t strikes_dynamic = 3;     // consecutive disagreements before an object is dynamic
    uint32_t reset_after = 30;        // frames without a pose before a new segment starts
    uint32_t forget_after = 300;      // frames unseen before a non-static object is dropped
    uint32_t seeds = 48;              // consensus hypotheses tried per frame
};

enum class State : uint8_t { Unknown, Provisional, Static, Dynamic };

struct Result {
    bool posed = false;
    mat::Mat view = mat::identity();  // our world -> view, rigid
    uint32_t segment = 0;             // 1, 2, ...: frames of one segment share a world frame
    uint32_t anchor_shared = 0;       // objects sharing the segment's anchor matrix (0: a single
                                      // object anchors it, so the frame may be tilted)
    bool new_segment = false;         // this frame started it (view = the anchor's)
    // This frame.
    uint32_t draws = 0, ambiguous = 0;  // draws, and keys dropped for several different matrices
    uint32_t known = 0, inliers = 0, placed = 0;
    double scene_scale = 0;  // median distance of the drawn objects from the camera
    // The segment's objects.
    uint32_t statics = 0, provisional = 0, dynamic = 0;
};

class Solver {
public:
    explicit Solver(const Settings& s = {}) : s_(s) {}

    // One frame's draws (any order). Keys drawn several times with different matrices are ignored.
    Result solve(const std::vector<Draw>& draws);
    void reset();  // forgets everything: the next frame starts a new segment

    State state(uint64_t key) const;
    size_t objects() const { return objects_.size(); }

private:
    struct Object {
        State state = State::Provisional;
        mat::Mat w, w_inv;  // placement in our world, and its inverse
        mat::Mat w_sum{};   // provisional: sum of the placements it agreed with
        uint32_t agreed = 0, strikes = 0;
        uint64_t last_seen = 0;
    };
    struct Hypothesis {
        Object* object;
        const Draw* draw;
        mat::Mat view;  // W^-1 * M
        double vote;    // statics count double
    };

    bool consensus(const std::vector<Hypothesis>& hyps, double tol_t, mat::Mat& view,
                   std::vector<uint8_t>& inlier) const;
    void start_segment(const std::vector<const Draw*>& draws, Result& r);
    void place(Object& o, const Draw& d, const mat::Mat& view_inv, bool refine);
    void evict();

    Settings s_;
    std::unordered_map<uint64_t, Object> objects_;
    uint64_t frame_ = 0;
    uint32_t segment_ = 0;
    uint32_t lost_ = 0;  // frames since the last pose
    uint32_t anchor_shared_ = 0;
};

}  // namespace lidar::mv
