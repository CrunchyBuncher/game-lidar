#include "modelview.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace lidar::mv {
namespace {

// The rotation part made orthonormal (Gram-Schmidt on the rows), translation kept. Normalizing the
// rows first also strips an object's scale off a model-view: W = S * R * T scales the rows of M's
// 3x3 and leaves its translation row alone.
mat::Mat rigid_part(const mat::Mat& a) {
    mat::Mat r = a;
    auto dot = [&](int i, int j) { return r.m[i][0] * r.m[j][0] + r.m[i][1] * r.m[j][1] + r.m[i][2] * r.m[j][2]; };
    auto normalize = [&](int i) {
        const double n = std::sqrt(dot(i, i));
        for (int k = 0; k < 3; ++k) r.m[i][k] /= n;
    };
    auto remove = [&](int i, int j) {  // row i -= its component along (unit) row j
        const double d = dot(i, j);
        for (int k = 0; k < 3; ++k) r.m[i][k] -= d * r.m[j][k];
    };
    for (int i = 0; i < 3; ++i) normalize(i);
    remove(1, 0);
    normalize(1);
    remove(2, 0);
    remove(2, 1);
    normalize(2);
    r.m[0][3] = r.m[1][3] = r.m[2][3] = 0;
    r.m[3][3] = 1;
    return r;
}

// Rotations within `tol_rad` of each other: |Ra - Rb|_F^2 = 4 (1 - cos angle) ~ 2 angle^2. Cheaper
// than the angle itself, and exact enough at these tolerances.
bool rotation_close(const mat::Mat& a, const mat::Mat& b, double tol_rad) {
    double d = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) d += (a.m[i][j] - b.m[i][j]) * (a.m[i][j] - b.m[i][j]);
    return d <= 2 * tol_rad * tol_rad;
}

double det3(const mat::Mat& a) {
    return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) -
           a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) +
           a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}

}  // namespace

Result Solver::solve(const std::vector<Draw>& draws) {
    ++frame_;
    Result r;
    r.draws = uint32_t(draws.size());

    // One matrix per key: a key drawn with several different matrices (a repeated prop) says
    // nothing about where its object is. The same matrix twice (e.g. two passes) is fine.
    std::vector<const Draw*> frame;
    std::vector<uint8_t> ambiguous;
    std::unordered_map<uint64_t, uint32_t> index;
    frame.reserve(draws.size());
    for (const Draw& d : draws) {
        if (!mat::finite(d.m)) continue;
        const auto [it, fresh] = index.try_emplace(d.key, uint32_t(frame.size()));
        if (fresh) {
            frame.push_back(&d);
            ambiguous.push_back(0);
        } else if (std::memcmp(&frame[it->second]->m, &d.m, sizeof(d.m)) != 0) {
            ambiguous[it->second] = 1;
        }
    }
    size_t n = 0;
    for (size_t i = 0; i < frame.size(); ++i) {
        if (ambiguous[i])
            ++r.ambiguous;
        else
            frame[n++] = frame[i];
    }
    frame.resize(n);

    if (!frame.empty()) {
        std::vector<double> dist;
        dist.reserve(frame.size());
        for (const Draw* d : frame) dist.push_back(std::hypot(d->m.m[3][0], d->m.m[3][1], d->m.m[3][2]));
        std::nth_element(dist.begin(), dist.begin() + ptrdiff_t(dist.size() / 2), dist.end());
        r.scene_scale = dist[dist.size() / 2];
    }
    const double tol_t = s_.translation_tol * std::max(1.0, r.scene_scale);

    // The view each known object implies.
    std::vector<Hypothesis> hyps;
    size_t unknown = 0;
    for (const Draw* d : frame) {
        const auto it = objects_.find(d->key);
        if (it == objects_.end()) {
            ++unknown;
            continue;
        }
        Object& o = it->second;
        o.last_seen = frame_;
        if (o.state == State::Dynamic) continue;
        hyps.push_back({&o, d, mat::mul(o.w_inv, d->m), o.state == State::Static ? 2.0 : 1.0});
    }
    r.known = uint32_t(hyps.size());

    mat::Mat view;
    std::vector<uint8_t> inlier;
    if (consensus(hyps, tol_t, view, inlier)) {
        lost_ = 0;
        r.posed = true;
        r.view = view;
        r.segment = segment_;
        mat::Mat view_inv;
        mat::inverse(view, view_inv);
        for (size_t i = 0; i < hyps.size(); ++i) {
            Object& o = *hyps[i].object;
            if (inlier[i]) {
                ++r.inliers;
                o.strikes = 0;
                if (o.state == State::Provisional) place(o, *hyps[i].draw, view_inv, true);
            } else if (++o.strikes >= s_.strikes_dynamic) {
                o.state = State::Dynamic;
            } else if (o.state == State::Provisional) {
                place(o, *hyps[i].draw, view_inv, false);  // start over where it is now
            }
        }
        for (const Draw* d : frame) {
            const auto [it, fresh] = objects_.try_emplace(d->key);
            if (!fresh) continue;
            it->second.last_seen = frame_;
            place(it->second, *d, view_inv, false);
            ++r.placed;
        }
    } else {
        ++lost_;
        // No usable known object in view (the first frame, a level change, a cut to an unseen place),
        // or lost for too long; and enough new ones to start from. Known dynamic draws (the HUD, the
        // player) alone don't count: a pause screen mustn't wipe the map.
        if ((hyps.empty() && unknown >= s_.min_inliers) || (lost_ >= s_.reset_after && frame.size() >= s_.min_inliers))
            start_segment(frame, r);
    }

    evict();
    for (const auto& [key, o] : objects_) {
        r.statics += o.state == State::Static;
        r.provisional += o.state == State::Provisional;
        r.dynamic += o.state == State::Dynamic;
    }
    return r;
}

// Seeds hypotheses (statics first) and keeps the one the most votes agree with; the view is then
// the mean of its inliers (statics only, if any agree) and the inliers are recounted against it.
bool Solver::consensus(const std::vector<Hypothesis>& hyps, double tol_t, mat::Mat& view,
                       std::vector<uint8_t>& inlier) const {
    if (hyps.size() < s_.min_inliers) return false;
    const double tol_r = s_.rotation_tol_deg * 3.14159265358979 / 180;
    auto agree = [&](const mat::Mat& a, const mat::Mat& b) {
        return mat::translation_between(a, b) <= tol_t && rotation_close(a, b, tol_r);
    };

    std::vector<uint32_t> order(hyps.size());
    std::iota(order.begin(), order.end(), 0u);
    std::stable_partition(order.begin(), order.end(),
                          [&](uint32_t i) { return hyps[i].object->state == State::Static; });
    const size_t seeds = std::min<size_t>(hyps.size(), std::max(1u, s_.seeds));
    size_t best = order[0];
    double best_votes = -1;
    for (size_t k = 0; k < seeds; ++k) {
        const size_t i = order[k * hyps.size() / seeds];
        double votes = 0;
        for (const Hypothesis& h : hyps)
            if (agree(hyps[i].view, h.view)) votes += h.vote;
        if (votes > best_votes) best = i, best_votes = votes;
    }

    bool any_static = false;
    for (const Hypothesis& h : hyps)
        any_static = any_static || (h.object->state == State::Static && agree(hyps[best].view, h.view));
    mat::Mat sum{};
    uint32_t count = 0;
    for (const Hypothesis& h : hyps) {
        if ((any_static && h.object->state != State::Static) || !agree(hyps[best].view, h.view)) continue;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) sum.m[i][j] += h.view.m[i][j];
        ++count;
    }
    for (auto& row : sum.m)
        for (double& v : row) v /= count;
    view = rigid_part(sum);

    inlier.assign(hyps.size(), 0);
    double votes = 0, total = 0;
    uint32_t inliers = 0;
    for (size_t i = 0; i < hyps.size(); ++i) {
        total += hyps[i].vote;
        if (!agree(view, hyps[i].view)) continue;
        inlier[i] = 1;
        votes += hyps[i].vote;
        ++inliers;
    }
    return inliers >= s_.min_inliers && votes >= s_.min_support * total;
}

// Forgets every placement. The heaviest draw anchors the new world frame: its model-view, without
// its scale, is the view. Everything else in the frame is placed provisionally around it. A
// mirrored object (negative determinant: the view itself is a rotation) would mirror the whole
// world, so it anchors only if nothing else can.
void Solver::start_segment(const std::vector<const Draw*>& draws, Result& r) {
    objects_.clear();
    ++segment_;
    lost_ = 0;
    const Draw* anchor = draws.front();
    for (const Draw* d : draws) {
        const bool mirrored = det3(d->m) < 0, anchor_mirrored = det3(anchor->m) < 0;
        if (mirrored != anchor_mirrored ? anchor_mirrored : d->weight > anchor->weight) anchor = d;
    }
    const mat::Mat view = rigid_part(anchor->m);
    mat::Mat view_inv;
    mat::inverse(view, view_inv);
    for (const Draw* d : draws) {
        Object& o = objects_[d->key];
        o.last_seen = frame_;
        place(o, *d, view_inv, false);
    }
    r.posed = true;
    r.view = view;
    r.segment = segment_;
    r.new_segment = true;
    r.placed = uint32_t(draws.size());
}

// W = M * V^-1. `refine`: averaged into the placements it agreed with so far (it freezes once
// promoted); else the placement starts over from this one.
void Solver::place(Object& o, const Draw& d, const mat::Mat& view_inv, bool refine) {
    const mat::Mat w = mat::mul(d.m, view_inv);
    if (refine) {
        ++o.agreed;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                o.w_sum.m[i][j] += w.m[i][j];
                o.w.m[i][j] = o.w_sum.m[i][j] / o.agreed;
            }
        if (o.agreed >= s_.promote_after) o.state = State::Static;
    } else {
        o.w = o.w_sum = w;
        o.agreed = 1;
    }
    if (!mat::inverse(o.w, o.w_inv)) o.state = State::Dynamic;  // degenerate (a zero scale)
}

// Keys that stop appearing (streamed or refilled buffers, UP data that changes every frame) would
// pile up. Static objects stay: they're the map, and bounded by the level.
void Solver::evict() {
    if (frame_ % 64 != 0) return;
    std::erase_if(objects_, [&](const auto& kv) {
        return kv.second.state != State::Static && frame_ - kv.second.last_seen > s_.forget_after;
    });
}

void Solver::reset() {
    objects_.clear();
    lost_ = 0;
}

State Solver::state(uint64_t key) const {
    const auto it = objects_.find(key);
    return it == objects_.end() ? State::Unknown : it->second.state;
}

}  // namespace lidar::mv
