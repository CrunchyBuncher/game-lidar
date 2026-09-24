#include "discovery.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "camera_math.h"

using reshade::api::shader_stage;

namespace lidar::disc {
namespace {

using Clock = std::chrono::steady_clock;

constexpr size_t kMaxHypotheses = 1024;
constexpr size_t kHistory = 48;          // frames of values per hypothesis (depth arrives a few frames late)
constexpr size_t kMaxCandidates = 96;
constexpr size_t kPublished = 16;        // candidates in the status snapshot
constexpr size_t kDepthFrames = 16;
constexpr uint32_t kRebuildEvery = 30;   // analyzed frames between candidate rebuilds
constexpr double kStill = 0.005;         // depth change below this: the camera didn't move
constexpr double kMoving = 0.05;         // above: it did
constexpr double kReprojMotion = 0.08;   // a reprojection test needs at least this much change
constexpr double kPassError = 0.002;     // median relative depth error of a passing test (right: < 1e-4)
constexpr auto kReprojInterval = std::chrono::milliseconds(100);

// A buffer as discovery identifies it (a profile's key).
struct BufId {
    shader_stage stage = shader_stage::vertex;
    uint32_t slot = 0, space = 0, size = 0;
    bool operator==(const BufId&) const = default;
};
struct BufIdHash {
    size_t operator()(const BufId& b) const {
        return std::hash<uint64_t>()((uint64_t(uint32_t(b.stage)) << 48) ^ (uint64_t(b.slot) << 32) ^
                                     (uint64_t(b.space) << 20) ^ b.size);
    }
};

// A hypothesis: "the 4x4 at this offset of this buffer is a <kind>, stored <major>".
struct Loc {
    BufId buf;
    uint32_t offset = 0;
    bool column_major = false;
    MatrixKind kind = MatrixKind::Rigid;
    bool operator==(const Loc&) const = default;
};
struct LocHash {
    size_t operator()(const Loc& l) const {
        return BufIdHash()(l.buf) * 31 + (size_t(l.offset) << 3) + (l.column_major ? 4 : 0) + size_t(l.kind);
    }
};

uint64_t hash_bytes(const void* p, size_t n, uint64_t h = 0xcbf29ce484222325ull) {
    const auto* b = static_cast<const uint8_t*>(p);
    for (size_t i = 0; i < n; ++i) h = (h ^ b[i]) * 0x100000001b3ull;
    return h;
}

struct Record {
    uint64_t frame = 0;
    float first[16];   // at the first sampled draw where the pattern held
    float common[16];  // the value most sampled draws had
    uint16_t distinct = 0;
};

struct Hypothesis {
    Loc loc;
    uint32_t frames_seen = 0;   // frames its buffer was bound at a sampled draw, since creation
    uint32_t frames_valid = 0;  // ... and the pattern held at this offset
    uint32_t per_draw = 0;      // frames where draws disagreed on the value
    uint32_t changes = 0;       // frames where the common value changed from the previous one
    std::deque<Record> history;

    const Record* at(uint64_t frame) const {
        for (auto it = history.rbegin(); it != history.rend(); ++it) {
            if (it->frame == frame) return &*it;
            if (it->frame < frame) break;
        }
        return nullptr;
    }
    double validity() const { return frames_seen ? double(frames_valid) / frames_seen : 0; }
};

struct Candidate {
    CameraProfile profile;
    Loc a, b;  // the first-named matrix, and the projection for pair layouts
    bool pair = false;
    uint32_t reproj_tests = 0, reproj_passes = 0;
    std::deque<float> errors;  // recent test errors
    uint32_t temporal_checks = 0, temporal_agree = 0;

    double temporal() const { return temporal_checks ? double(temporal_agree) / temporal_checks : 0.5; }
    double error() const {
        if (errors.empty()) return 1;
        std::vector<float> e(errors.begin(), errors.end());
        std::nth_element(e.begin(), e.begin() + ptrdiff_t(e.size() / 2), e.end());
        return e[e.size() / 2];
    }
    double score() const {
        if (reproj_tests >= 3) return 0.75 * reproj_passes / reproj_tests + 0.25 * temporal();
        return 0.25 * temporal() * std::min(1.0, temporal_checks / 30.0);
    }
    bool confident() const {
        return reproj_tests >= 5 && reproj_passes >= 0.9 * reproj_tests &&
               (temporal_checks < 10 || temporal() >= 0.8);
    }
};

int layout_preference(CameraLayout l) {
    switch (l) {  // exact passthrough first, derived views last
        case CameraLayout::ViewAndProj: return 0;
        case CameraLayout::ViewProjAndProj: return 1;
        case CameraLayout::InvViewAndProj: return 2;
        case CameraLayout::InvViewProjAndProj: return 3;
        case CameraLayout::ViewProj: return 4;
        case CameraLayout::InvViewProj: return 5;
    }
    return 9;
}

bool better(const Candidate& x, const Candidate& y) {
    const int bx = int(x.score() * 20), by = int(y.score() * 20);  // 0.05 buckets, then preferences
    if (bx != by) return bx > by;
    if (layout_preference(x.profile.layout) != layout_preference(y.profile.layout))
        return layout_preference(x.profile.layout) < layout_preference(y.profile.layout);
    // Only values that differ per draw get a Common variant, and then the camera is what most draws
    // see (world = identity); the first draw may be any object.
    if (x.profile.latch != y.profile.latch) return x.profile.latch == Latch::Common;
    if (x.profile.key.stage != y.profile.key.stage) return x.profile.key.stage == shader_stage::vertex;
    if (x.error() != y.error()) return x.error() < y.error();
    if (x.profile.key.slot != y.profile.key.slot) return x.profile.key.slot < y.profile.key.slot;
    return x.profile.view_offset < y.profile.view_offset;
}

bool view_changed(const mat::Mat& a, const mat::Mat& b) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(a.m[i][j] - b.m[i][j]) > 1e-5) return true;
    for (int j = 0; j < 3; ++j)
        if (std::abs(a.m[3][j] - b.m[3][j]) > 1e-5 * (1 + std::abs(a.m[3][j]))) return true;
    return false;
}

std::string describe(const CameraProfile& c, bool registers) {
    char buf[160];
    const char* stage = stage_name(c.key.stage);
    if (registers) {
        if (c.single_matrix())
            std::snprintf(buf, sizeof(buf), "%s c%u", stage, c.view_offset / 16);
        else
            std::snprintf(buf, sizeof(buf), "%s c%u / c%u", stage, c.view_offset / 16, c.proj_offset / 16);
    } else {
        int n = std::snprintf(buf, sizeof(buf), "%s b%u", stage, c.key.slot);
        if (c.key.space) n += std::snprintf(buf + n, sizeof(buf) - n, " space%u", c.key.space);
        if (c.single_matrix())
            n += std::snprintf(buf + n, sizeof(buf) - n, " @ %u", c.view_offset);
        else
            n += std::snprintf(buf + n, sizeof(buf) - n, " @ %u / %u", c.view_offset, c.proj_offset);
        if (c.key.size) std::snprintf(buf + n, sizeof(buf) - n, " (%u bytes)", c.key.size);
    }
    return buf;
}

}  // namespace

struct Discovery::DepthFrame {
    uint64_t frame = 0;
    DepthGrid grid;
};

// Owned by the worker thread.
struct Discovery::State {
    bool registers = false;  // D3D9: offsets are registers
    Clock::time_point start = Clock::now(), last_reproj{};
    Status stats;
    std::unordered_map<Loc, Hypothesis, LocHash> hyps;
    std::map<std::string, Candidate> candidates;  // keyed by the profile text
    std::deque<std::shared_ptr<DepthFrame>> depths;
    uint64_t last_frame = 0;  // latest analyzed sample frame
    std::vector<uint8_t> window;
    std::unordered_map<uint64_t, uint32_t> window_bits;  // classify_matrix by window content

    // ---- 1. Scan --------------------------------------------------------------------------

    void analyze(const SampleFrame& sf) {
        const Clock::time_point t0 = Clock::now();
        scan(sf);
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        stats.analyze_ms = stats.frames_analyzed <= 1 ? ms : stats.analyze_ms * 0.95 + ms * 0.05;
    }

    void scan(const SampleFrame& sf) {
        ++stats.frames_analyzed;
        if (window_bits.size() > (1u << 16)) window_bits.clear();
        last_frame = sf.frame;
        stats.last_samples = uint32_t(sf.samples.samples.size());
        stats.last_draws = sf.samples.draws;

        // Every window's classification, once per distinct buffer content.
        std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>> classified;
        std::unordered_map<Loc, std::vector<const float*>, LocHash> values;  // in draw order
        std::unordered_set<BufId, BufIdHash> bound;
        uint32_t buffers = 0;
        for (const cam::DrawSample& s : sf.samples.samples) {
            for (const cam::BoundBuffer& b : s.buffers) {
                ++buffers;
                const BufId id{b.key.stage, b.key.slot, b.key.space, b.key.size};
                bound.insert(id);
                const std::vector<uint8_t>& bytes = b.read.bytes;
                const uint64_t h = hash_bytes(bytes.data(), bytes.size(), BufIdHash()(id));
                auto [it, fresh] = classified.try_emplace(h);
                if (fresh) {
                    // Per window too: D3D9 register files differ per draw in a few registers only.
                    for (uint32_t off = 0; off + 64 <= bytes.size(); off += 16) {
                        const uint64_t wh = hash_bytes(bytes.data() + off, 64);
                        auto [wit, wfresh] = window_bits.try_emplace(wh, 0);
                        if (wfresh) {
                            float f[16];
                            std::memcpy(f, bytes.data() + off, sizeof(f));
                            wit->second = classify_matrix(f);
                        }
                        if (wit->second) it->second.emplace_back(off, wit->second);
                    }
                }
                for (const auto& [off, bits] : it->second)
                    for (uint32_t k = 0; k < 8; ++k)
                        if (bits & (1u << k)) {
                            const Loc loc{id, off, (k & 1) != 0, MatrixKind(k >> 1)};
                            values[loc].push_back(reinterpret_cast<const float*>(bytes.data() + off));
                        }
            }
        }
        stats.last_buffers = buffers;

        for (auto& [loc, v] : values) {
            auto it = hyps.find(loc);
            if (it == hyps.end()) {
                if (hyps.size() >= kMaxHypotheses) continue;
                it = hyps.emplace(loc, Hypothesis{loc}).first;
            }
            Hypothesis& h = it->second;
            Record r;
            r.frame = sf.frame;
            std::memcpy(r.first, v.front(), sizeof(r.first));
            // The most common value (first seen wins ties).
            std::unordered_map<uint64_t, std::pair<uint32_t, const float*>> counts;
            const float* best = v.front();
            uint32_t best_count = 0;
            for (const float* p : v) {
                auto& [n, first] = counts[hash_bytes(p, 64)];
                if (n++ == 0) first = p;
                if (n > best_count) best_count = n, best = first;
            }
            std::memcpy(r.common, best, sizeof(r.common));
            r.distinct = uint16_t(std::min<size_t>(counts.size(), 0xFFFF));
            ++h.frames_valid;
            h.per_draw += r.distinct > 1;
            if (!h.history.empty() && std::memcmp(h.history.back().common, r.common, sizeof(r.common)) != 0) ++h.changes;
            h.history.push_back(r);
            if (h.history.size() > kHistory) h.history.pop_front();
        }
        for (auto& [loc, h] : hyps) h.frames_seen += bound.contains(loc.buf);

        if (stats.frames_analyzed % kRebuildEvery == 0) {
            prune();
            rebuild_candidates();
        }
    }

    // Hypotheses that mostly don't hold (e.g. a matrix that only sometimes looks rigid) go, and so
    // do candidates built on them.
    void prune() {
        std::erase_if(hyps, [](const auto& kv) {
            const Hypothesis& h = kv.second;
            return h.frames_seen >= 60 && h.validity() < 0.5;
        });
        std::erase_if(candidates, [&](const auto& kv) {
            const Candidate& c = kv.second;
            return !hyps.contains(c.a) || (c.pair && !hyps.contains(c.b));
        });
    }

    // ---- 2. Candidates --------------------------------------------------------------------

    void add_candidate(CameraLayout layout, const Hypothesis& a, const Hypothesis* b, Latch latch) {
        if (candidates.size() >= kMaxCandidates) return;
        CameraProfile p;
        p.key = {a.loc.buf.stage, a.loc.buf.slot, a.loc.buf.space, a.loc.buf.size};
        p.layout = layout;
        p.view_offset = a.loc.offset;
        if (b) p.proj_offset = b->loc.offset;
        p.column_major = a.loc.column_major;
        p.latch = latch;
        Candidate c;
        c.profile = p;
        c.a = a.loc;
        if (b) c.b = b->loc, c.pair = true;
        candidates.try_emplace(format_profile(p, {}), std::move(c));
    }

    void rebuild_candidates() {
        // Eligible: seen for a while and nearly always holding. Grouped by buffer + major, since a
        // profile reads both matrices from one buffer the same way.
        struct Group {
            std::vector<const Hypothesis*> kinds[4];
        };
        std::unordered_map<Loc, Group, LocHash> groups;  // keyed by the buffer + major (offset 0, Rigid)
        for (const auto& [loc, h] : hyps) {
            if (h.frames_seen < 20 || h.validity() < 0.9) continue;
            Loc g{loc.buf, 0, loc.column_major, MatrixKind::Rigid};
            groups[g].kinds[int(loc.kind)].push_back(&h);
        }
        // Moving hypotheses first, so the candidate cap keeps the interesting ones.
        auto order = [](std::vector<const Hypothesis*>& v) {
            std::sort(v.begin(), v.end(), [](const Hypothesis* x, const Hypothesis* y) {
                if ((x->changes > 0) != (y->changes > 0)) return x->changes > 0;
                return x->loc.offset < y->loc.offset;
            });
        };
        auto per_draw = [](const Hypothesis& h) { return h.per_draw * 5 > h.frames_valid; };
        auto add = [&](CameraLayout layout, const Hypothesis& a, const Hypothesis* b) {
            add_candidate(layout, a, b, Latch::First);
            if (per_draw(a) || (b && per_draw(*b))) add_candidate(layout, a, b, Latch::Common);
        };
        for (auto& [g, grp] : groups) {
            for (auto& v : grp.kinds) order(v);
            const auto& rigid = grp.kinds[int(MatrixKind::Rigid)];
            const auto& proj = grp.kinds[int(MatrixKind::Proj)];
            const auto& vp = grp.kinds[int(MatrixKind::ViewProj)];
            const auto& ivp = grp.kinds[int(MatrixKind::InvViewProj)];
            for (const Hypothesis* h : vp) add(CameraLayout::ViewProj, *h, nullptr);
            for (const Hypothesis* h : ivp) add(CameraLayout::InvViewProj, *h, nullptr);
            for (const Hypothesis* p : proj) {
                auto apart = [&](const Hypothesis* h) {
                    return std::max(h->loc.offset, p->loc.offset) - std::min(h->loc.offset, p->loc.offset) >= 64;
                };
                for (const Hypothesis* h : vp)
                    if (apart(h)) add(CameraLayout::ViewProjAndProj, *h, p);
                for (const Hypothesis* h : ivp)
                    if (apart(h)) add(CameraLayout::InvViewProjAndProj, *h, p);
                for (const Hypothesis* h : rigid)
                    if (apart(h)) {
                        add(CameraLayout::ViewAndProj, *h, p);
                        add(CameraLayout::InvViewAndProj, *h, p);
                    }
            }
        }
    }

    // The candidate's matrices at `frame`, decoded exactly as a profile would. Returns 0 if they
    // decoded, 1 if values are missing for that frame (not the candidate's fault), 2 if they
    // didn't decode.
    int decode(const Candidate& c, uint64_t frame, mat::Mat& view, mat::Mat& proj, float* view_f = nullptr,
               float* proj_f = nullptr) {
        const auto ha = hyps.find(c.a);
        if (ha == hyps.end()) return 1;
        const Record* ra = ha->second.at(frame);
        const Record* rb = nullptr;
        if (c.pair) {
            const auto hb = hyps.find(c.b);
            if (hb == hyps.end()) return 1;
            rb = hb->second.at(frame);
        }
        if (ra == nullptr || (c.pair && rb == nullptr)) return 1;
        const bool common = c.profile.latch == Latch::Common;
        const CameraProfile& p = c.profile;
        window.assign(p.window_size(), 0);
        std::memcpy(window.data() + (p.view_offset - p.window_offset()), common ? ra->common : ra->first, 64);
        if (rb) std::memcpy(window.data() + (p.proj_offset - p.window_offset()), common ? rb->common : rb->first, 64);
        float v[16], pr[16];
        if (!decode_camera(p, window.data(), window.size(), v, pr)) return 2;
        view = mat::load(v, false);
        proj = mat::load(pr, false);
        if (view_f) std::memcpy(view_f, v, sizeof(v));
        if (proj_f) std::memcpy(proj_f, pr, sizeof(pr));
        return 0;
    }

    // ---- 3. Scoring against depth ----------------------------------------------------------

    void add_depth(const std::shared_ptr<DepthFrame>& df) {
        ++stats.depth_frames;
        const DepthFrame* prev = nullptr;
        for (auto it = depths.rbegin(); it != depths.rend(); ++it)
            if ((*it)->frame < df->frame) {
                prev = it->get();
                break;
            }

        // Still vs moving: the view must stay put, or change, with the depth image.
        if (prev != nullptr && df->frame - prev->frame <= 3) {
            const double change = depth_change(prev->grid, df->grid);
            const bool still = change < kStill, moving = change > kMoving;
            if (still || moving) {
                ++(still ? stats.still_frames : stats.moving_frames);
                for (auto& [k, c] : candidates) {
                    mat::Mat v0, p0, v1, p1;
                    const int r0 = decode(c, prev->frame, v0, p0), r1 = decode(c, df->frame, v1, p1);
                    if (r0 == 1 || r1 == 1) continue;
                    ++c.temporal_checks;
                    if (r0 == 0 && r1 == 0 && view_changed(v0, v1) == moving) ++c.temporal_agree;
                }
            }
        }

        // Reprojection: from the newest earlier frame that differs enough (camera moved).
        const Clock::time_point now = Clock::now();
        if (now - last_reproj >= kReprojInterval) {
            const DepthFrame* from = nullptr;
            int looked = 0;
            for (auto it = depths.rbegin(); it != depths.rend() && looked < 12; ++it, ++looked)
                if ((*it)->frame < df->frame && df->frame - (*it)->frame <= 30 &&
                    depth_change((*it)->grid, df->grid, 0.01) >= kReprojMotion) {
                    from = it->get();
                    break;
                }
            if (from != nullptr) {
                last_reproj = now;
                ++stats.reproj_rounds;
                for (auto& [k, c] : candidates) {
                    mat::Mat va, pa, vb, pb;
                    const int ra = decode(c, from->frame, va, pa), rb = decode(c, df->frame, vb, pb);
                    if (ra == 1 || rb == 1) continue;
                    double err = 1;
                    if (ra == 0 && rb == 0) {
                        const ReprojStats rs = reproject(from->grid, va, pa, df->grid, vb, pb);
                        if (!rs.conclusive()) continue;
                        err = rs.median_rel;
                    }
                    ++c.reproj_tests;
                    c.reproj_passes += err < kPassError;
                    c.errors.push_back(float(err));
                    if (c.errors.size() > 64) c.errors.pop_front();
                }
            }
        }

        depths.push_back(df);
        std::sort(depths.begin(), depths.end(), [](const auto& x, const auto& y) { return x->frame < y->frame; });
        while (depths.size() > kDepthFrames) depths.pop_front();
    }

    // ---- Output ---------------------------------------------------------------------------

    std::vector<const Candidate*> ranked() const {
        std::vector<const Candidate*> v;
        for (const auto& [k, c] : candidates) v.push_back(&c);
        std::sort(v.begin(), v.end(), [](const Candidate* x, const Candidate* y) { return better(*x, *y); });
        return v;
    }

    CandidateInfo info(const Candidate& c) {
        CandidateInfo i;
        i.profile = c.profile;
        i.where = describe(c.profile, registers);
        i.score = c.score();
        i.confident = c.confident();
        i.reproj_tests = c.reproj_tests;
        i.reproj_passes = c.reproj_passes;
        i.reproj_error = c.error();
        i.temporal_checks = c.temporal_checks;
        i.temporal_agree = c.temporal_agree;
        mat::Mat v, p;
        i.have_values = decode(c, last_frame, v, p, i.view, i.proj) == 0;
        if (i.have_values) {
            i.info = analyze_projection(i.proj);
            i.profile.right_handed = i.info.right_handed;
        }
        return i;
    }

    void publish(Status& out) {
        stats.running = true;
        stats.seconds = std::chrono::duration<double>(Clock::now() - start).count();
        for (size_t& n : stats.hypotheses) n = 0;
        for (const auto& [loc, h] : hyps) ++stats.hypotheses[int(loc.kind)];
        stats.candidates.clear();
        for (const Candidate* c : ranked()) {
            if (stats.candidates.size() >= kPublished) break;
            stats.candidates.push_back(info(*c));
        }
        out = stats;
    }

    void write_report(const std::filesystem::path& path) {
        std::ofstream f(path);
        if (!f) return;
        char line[512];
        std::snprintf(line, sizeof(line),
                      "game-lidar discovery report\n%.1f s, %llu frames analyzed (%llu dropped, %.2f ms each), %llu "
                      "depth frames (%llu still, %llu moving), %llu reprojection rounds\nlast frame: %u draws, %u "
                      "sampled, %u buffers\n\n",
                      stats.seconds, (unsigned long long)stats.frames_analyzed, (unsigned long long)stats.frames_dropped,
                      stats.analyze_ms,
                      (unsigned long long)stats.depth_frames, (unsigned long long)stats.still_frames,
                      (unsigned long long)stats.moving_frames, (unsigned long long)stats.reproj_rounds,
                      stats.last_draws, stats.last_samples, stats.last_buffers);
        f << line;
        f << "== Candidates (best first) ==\n";
        int rank = 0;
        for (const Candidate* c : ranked()) {
            const CandidateInfo i = info(*c);
            std::snprintf(line, sizeof(line),
                          "#%d %-18s %-34s %s-major latch=%-6s score %.2f%s  reproj %u/%u (err %.4f)  "
                          "still/moving %u/%u",
                          ++rank, layout_name(c->profile.layout), i.where.c_str(),
                          c->profile.column_major ? "column" : "row", latch_name(c->profile.latch), i.score,
                          i.confident ? " CONFIDENT" : "", i.reproj_passes, i.reproj_tests, i.reproj_error,
                          i.temporal_agree, i.temporal_checks);
            f << line;
            if (i.have_values && i.info.valid)
                std::snprintf(line, sizeof(line), "  fov %.1f near %.4g far %.6g %s\n", i.info.fov_y_deg, i.info.near_z,
                              i.info.far_z, i.info.reversed ? "reversed" : "standard");
            else
                std::snprintf(line, sizeof(line), "\n");
            f << line;
        }
        f << "\n== Hypotheses ==\n";
        std::vector<const Hypothesis*> hs;
        for (const auto& [loc, h] : hyps) hs.push_back(&h);
        std::sort(hs.begin(), hs.end(), [](const Hypothesis* x, const Hypothesis* y) {
            if (x->loc.buf.stage != y->loc.buf.stage) return uint32_t(x->loc.buf.stage) < uint32_t(y->loc.buf.stage);
            if (x->loc.buf.slot != y->loc.buf.slot) return x->loc.buf.slot < y->loc.buf.slot;
            return x->loc.offset < y->loc.offset;
        });
        for (const Hypothesis* h : hs) {
            std::snprintf(line, sizeof(line),
                          "%s slot %u space %u size %u offset %u (c%u) %-11s %s-major  valid %u/%u  changes %u  "
                          "per-draw %u\n",
                          stage_name(h->loc.buf.stage), h->loc.buf.slot, h->loc.buf.space, h->loc.buf.size,
                          h->loc.offset, h->loc.offset / 16, kind_name(h->loc.kind),
                          h->loc.column_major ? "column" : "row", h->frames_valid, h->frames_seen, h->changes,
                          h->per_draw);
            f << line;
            if (!h->history.empty()) {
                const float* m = h->history.back().common;
                for (int r = 0; r < 4; ++r) {
                    std::snprintf(line, sizeof(line), "    %12.5g %12.5g %12.5g %12.5g\n", m[r * 4], m[r * 4 + 1],
                                  m[r * 4 + 2], m[r * 4 + 3]);
                    f << line;
                }
            }
        }
    }
};

Discovery::~Discovery() { stop(); }

void Discovery::start(bool registers) {
    stop();
    registers_ = registers;
    {
        const std::lock_guard lock(mutex_);
        quit_ = false;
        sample_queue_.clear();
        depth_queue_.clear();
        dropped_ = 0;
        published_ = {};
        published_.running = true;
    }
    worker_ = std::thread([this] { run(); });
}

void Discovery::stop() {
    if (!worker_.joinable()) return;
    {
        const std::lock_guard lock(mutex_);
        quit_ = true;
    }
    wake_.notify_all();
    worker_.join();
    const std::lock_guard lock(mutex_);
    published_.running = false;
    sample_queue_.clear();
    depth_queue_.clear();
}

void Discovery::submit_samples(uint64_t frame, cam::DepthSamples&& samples) {
    {
        const std::lock_guard lock(mutex_);
        if (!worker_.joinable() || quit_) return;
        if (sample_queue_.size() >= 2) {
            ++dropped_;
            return;
        }
        sample_queue_.push_back({frame, std::move(samples)});
    }
    wake_.notify_one();
}

void Discovery::submit_depth(const FrameHeader& h, const float* depth) {
    if (!running() || h.width == 0 || h.height == 0 || h.src_width == 0 || h.src_height == 0) return;
    auto df = std::make_shared<DepthFrame>();
    df->frame = h.frame_index;
    df->grid.build(depth, h.width, h.height, h.src_width, h.src_height, std::max(1u, h.width / 128));
    {
        const std::lock_guard lock(mutex_);
        if (quit_) return;
        depth_queue_.push_back(std::move(df));
        while (depth_queue_.size() > 8) depth_queue_.pop_front();
    }
    wake_.notify_one();
}

Status Discovery::status() const {
    const std::lock_guard lock(mutex_);
    return published_;
}

void Discovery::request_report(const std::filesystem::path& path) {
    {
        const std::lock_guard lock(mutex_);
        report_path_ = path;
    }
    wake_.notify_one();
}

void Discovery::run() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
    State st;
    st.registers = registers_;
    for (;;) {
        SampleFrame sf;
        std::shared_ptr<DepthFrame> df;
        std::filesystem::path report;
        bool have_samples = false;
        {
            std::unique_lock lock(mutex_);
            wake_.wait(lock, [&] {
                return quit_ || !sample_queue_.empty() || !depth_queue_.empty() || !report_path_.empty();
            });
            if (quit_) return;
            // Samples first: a depth frame arrives a few frames after its samples.
            if (!sample_queue_.empty()) {
                sf = std::move(sample_queue_.front());
                sample_queue_.pop_front();
                have_samples = true;
            } else if (!depth_queue_.empty()) {
                df = std::move(depth_queue_.front());
                depth_queue_.pop_front();
            }
            report.swap(report_path_);
            st.stats.frames_dropped = dropped_;
        }
        if (have_samples) st.analyze(sf);
        if (df) st.add_depth(df);
        if (!report.empty()) st.write_report(report);
        Status s;
        st.publish(s);
        const std::lock_guard lock(mutex_);
        published_ = std::move(s);
    }
}

}  // namespace lidar::disc
