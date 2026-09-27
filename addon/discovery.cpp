#include "discovery.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include "camera_math.h"
#include "modelview.h"

using reshade::api::shader_stage;

namespace lidar::disc {
namespace {

using Clock = std::chrono::steady_clock;

constexpr size_t kMaxHypotheses = 1024;
constexpr size_t kMaxTranslations = 256;  // of those, translation vectors
constexpr size_t kTranslationsTried = 4;  // per camera-relative candidate, most changing first
constexpr size_t kRelativeTried = 4;      // camera-relative candidates given translations, best first
constexpr size_t kHistory = 48;          // frames of values per hypothesis (depth arrives a few frames late)
constexpr size_t kMaxCandidates = 96;
constexpr size_t kPublished = 16;        // candidates in the status snapshot
constexpr size_t kDepthFrames = 16;
constexpr uint32_t kRebuildEvery = 30;   // analyzed frames between candidate rebuilds
constexpr uint64_t kCensusEvery = 30;    // frames between the report's censuses of every draw
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
    // The values in draw order: runs of consecutive sampled draws (indices into the frame's samples)
    // with the same value. Decoding pairs matrices from the same draws with these, as the camera
    // tracker reads them: one buffer, at one draw.
    struct Run {
        uint16_t from = 0, to = 0;  // inclusive
        float f[16];
    };
    std::vector<Run> runs;
};
constexpr size_t kMaxRuns = 8;  // per record: the camera tracker tries this many places or so too

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
    Loc t;  // the translation, if profile.has_translation
    uint32_t reproj_tests = 0, reproj_passes = 0;
    std::deque<float> errors;  // recent test errors
    uint32_t temporal_checks = 0, temporal_agree = 0;
    // The camera's right axis in the world, |component| summed over decoded frames: an FPS camera
    // doesn't roll, so it stays level and the up axis' component stays near 0 as the camera turns.
    double right_abs[3] = {};
    uint32_t right_frames = 0;

    // Z-up, if turning showed it: z's component clearly the smallest (both others vary with yaw).
    bool z_up() const {
        if (right_frames < 30) return false;
        return right_abs[2] < 0.5 * right_abs[0] && right_abs[2] < 0.5 * right_abs[1];
    }
    // ModelView: the solver over every frame's draws, and the views it found (nullopt: no pose).
    std::shared_ptr<mv::Solver> solver;
    std::deque<std::pair<uint64_t, std::optional<mat::Mat>>> views;

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
        case CameraLayout::ModelView: return 6;
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

// A float3 that could be a world position (a translation candidate): finite, not tiny, not huge, and
// no denormals (bits of an int or a packed value, not a coordinate).
bool looks_like_position(const float f[3]) {
    double largest = 0;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(f[i]) || std::abs(f[i]) > 1e7f) return false;
        if (f[i] != 0 && std::abs(f[i]) < 1e-30f) return false;
        largest = std::max(largest, double(std::abs(f[i])));
    }
    return largest >= 1;
}

// Every distinct raw value of a window over one frame's sampled draws (report only): shows what a
// latch would have to pick from when the value differs per draw.
struct Snapshot {
    struct Value {
        float f[16];
        uint32_t bits = 0;            // classify_matrix
        std::vector<uint32_t> draws;  // draw numbers that saw it
    };
    struct Window {
        BufId buf;
        uint32_t offset = 0;
        std::vector<Value> values;  // in order of first use
    };
    struct Draw {
        uint32_t draw = 0;
        cam::DrawCall call;
        cam::DrawGeometry geometry;
    };
    uint64_t frame = 0;
    uint32_t draws = 0, sampled = 0;
    std::vector<Window> windows;
    std::vector<Draw> calls;  // the sampled draws' calls and geometry
};

std::string kinds_text(uint32_t bits) {
    std::string s;
    for (uint32_t k = 0; k < 8; ++k)
        if (bits & (1u << k)) {
            if (!s.empty()) s += ' ';
            s += kind_name(MatrixKind(k >> 1));
            s += (k & 1) ? "(col)" : "(row)";
        }
    return s.empty() ? "-" : s;
}

// ---- Census: model-view diagnostics (plan_modelview.md) ----------------------------------------
// Can the same object be found again in a later frame, and do matched objects agree on one camera
// motion? For a static object, M(t0)^-1 * M(t1) = V(t0)^-1 * V(t1) (row vectors, M = W * V).

struct Census {
    uint64_t frame = 0;
    uint32_t draws = 0;
    std::vector<cam::DrawRecord> records;
};

const char* draw_kind(const cam::DrawCall& c, const cam::DrawGeometry& g) {
    if (c.type == cam::DrawType::Indirect) return "indirect";
    if (g.up) return c.type == cam::DrawType::Indexed ? "indexed-UP" : "UP";
    return c.type == cam::DrawType::Indexed ? "indexed" : "draw";
}

std::string draw_text(const cam::DrawCall& c, const cam::DrawGeometry& g) {
    char buf[256];
    int n = std::snprintf(buf, sizeof(buf), "%-10s n=%u", draw_kind(c, g), c.count);
    auto add = [&](const char* fmt, auto... args) {
        if (n >= 0 && size_t(n) < sizeof(buf)) n += std::snprintf(buf + n, sizeof(buf) - size_t(n), fmt, args...);
    };
    if (c.instances != 1) add(" inst=%u", c.instances);
    if (c.first) add(" first=%u", c.first);
    if (c.vertex_offset) add(" base=%d", c.vertex_offset);
    if (g.up) {
        add(" | up %llx", (unsigned long long)g.up_vertices);
        if (g.up_indices) add(" idx %llx", (unsigned long long)g.up_indices);
        add(" %u B hash %016llx", g.up_bytes, (unsigned long long)g.up_hash);
    } else {
        add(" | vb %llx+%u/%u", (unsigned long long)g.vb, g.vb_offset, g.vb_stride);
        if (g.ib) add(" ib %llx", (unsigned long long)g.ib);
    }
    if (g.vs) add(" vs %llx", (unsigned long long)g.vs);
    return buf;
}

// UP draws by the hash of their data (`content`, what the solver uses), or by the game's pointers.
uint64_t report_key(const cam::DrawRecord& r, bool content) { return cam::object_key(r, !content); }

using mat::translation_between;
double rotation_between(const mat::Mat& a, const mat::Mat& b) { return mat::rotation_between_deg(a, b); }

bool same_motion(const mat::Mat& a, const mat::Mat& b) {
    const double scale = std::max({1.0, std::abs(a.m[3][0]), std::abs(a.m[3][1]), std::abs(a.m[3][2])});
    return rotation_between(a, b) < 0.05 && translation_between(a, b) < 0.01 + 1e-4 * scale;
}

using KeyMap = std::unordered_map<uint64_t, std::vector<const cam::DrawRecord*>>;
KeyMap group_by_key(const Census& c, bool content) {
    KeyMap m;
    for (const cam::DrawRecord& r : c.records) m[report_key(r, content)].push_back(&r);
    return m;
}

void write_census(std::ofstream& f, const Census& c, bool list) {
    char line[512];
    uint32_t kinds[5] = {}, windows = 0, rigid_col = 0, rigid_row = 0;
    std::unordered_set<uint64_t> distinct;
    for (const cam::DrawRecord& r : c.records) {
        const cam::DrawCall& call = r.call;
        ++kinds[call.type == cam::DrawType::Indirect      ? 4
                : r.geometry.up                           ? (call.type == cam::DrawType::Indexed ? 3 : 2)
                : call.type == cam::DrawType::Indexed     ? 1
                                                          : 0];
        if (!r.has_window) continue;
        ++windows;
        distinct.insert(hash_bytes(r.window, sizeof(r.window)));
        const uint32_t bits = classify_matrix(r.window);
        rigid_col += (bits & kind_bit(MatrixKind::Rigid, true)) != 0;
        rigid_row += (bits & kind_bit(MatrixKind::Rigid, false)) != 0;
    }
    std::snprintf(line, sizeof(line),
                  "\n== Census, frame %llu: %zu draws (draw %u, indexed %u, UP %u, indexed-UP %u, indirect %u) ==\n"
                  "vertex c0-c3: read at %u draws, %zu distinct values, rigid column-major %u, row-major %u\n",
                  (unsigned long long)c.frame, c.records.size(), kinds[0], kinds[1], kinds[2], kinds[3], kinds[4],
                  windows, distinct.size(), rigid_col, rigid_row);
    f << line;
    for (bool content : {false, true}) {
        const KeyMap keys = group_by_key(c, content);
        uint32_t once = 0, repeated_same = 0, repeated_diff = 0;
        for (const auto& [k, rs] : keys) {
            if (rs.size() == 1) {
                ++once;
                continue;
            }
            bool same = true;
            for (const cam::DrawRecord* r : rs)
                same = same && r->has_window == rs[0]->has_window &&
                       std::memcmp(r->window, rs[0]->window, sizeof(r->window)) == 0;
            ++(same ? repeated_same : repeated_diff);
        }
        std::snprintf(line, sizeof(line),
                      "%s keys: %zu distinct, %u drawn once, %u drawn repeatedly with one c0-c3, %u with several "
                      "(ambiguous)\n",
                      content ? "content (UP by data hash)" : "pointer (UP by game pointer)", keys.size(), once,
                      repeated_same, repeated_diff);
        f << line;
    }
    if (!list) return;
    f << "draw  call | geometry | c0-c3 hash, kinds | pointer key / content key\n";
    size_t n = 0;
    for (const cam::DrawRecord& r : c.records) {
        if (++n > 2500) {
            f << "...\n";
            break;
        }
        std::snprintf(line, sizeof(line), "%5u %s | %08x %s | %08x / %08x\n", r.draw,
                      draw_text(r.call, r.geometry).c_str(),
                      r.has_window ? uint32_t(hash_bytes(r.window, sizeof(r.window))) : 0u,
                      r.has_window ? kinds_text(classify_matrix(r.window)).c_str() : "(unread)",
                      uint32_t(report_key(r, false)), uint32_t(report_key(r, true)));
        f << line;
    }
}

// Objects drawn exactly once in both frames, and the camera motion each implies.
void write_census_pair(std::ofstream& f, const Census& a, const Census& b) {
    char line[512];
    std::snprintf(line, sizeof(line), "\n== Census frames %llu -> %llu: objects matched by key ==\n",
                  (unsigned long long)a.frame, (unsigned long long)b.frame);
    f << line;
    struct Match {
        const cam::DrawRecord *a, *b;
        mat::Mat d;
        bool inlier = false;
    };
    std::vector<Match> listed;
    size_t listed_inliers = 0;
    for (bool content : {false, true}) {
        const KeyMap ka = group_by_key(a, content), kb = group_by_key(b, content);
        std::vector<std::pair<const cam::DrawRecord*, const cam::DrawRecord*>> common;
        for (const auto& [k, ra] : ka) {
            const auto it = kb.find(k);
            if (ra.size() != 1 || it == kb.end() || it->second.size() != 1) continue;
            if (ra[0]->has_window && it->second[0]->has_window) common.emplace_back(ra[0], it->second[0]);
        }
        std::sort(common.begin(), common.end(), [](const auto& x, const auto& y) { return x.first->draw < y.first->draw; });
        uint32_t identical = 0;
        for (const auto& [ra, rb] : common) identical += std::memcmp(ra->window, rb->window, sizeof(ra->window)) == 0;
        std::snprintf(line, sizeof(line), "%s keys: %zu objects drawn once in both frames, %u with identical c0-c3\n",
                      content ? "content" : "pointer", common.size(), identical);
        f << line;
        for (bool column_major : {true, false}) {
            std::vector<Match> ms;
            for (const auto& [ra, rb] : common) {
                mat::Mat ia;
                if (!mat::inverse(mat::load(ra->window, column_major), ia)) continue;
                Match m{ra, rb, mat::mul(ia, mat::load(rb->window, column_major))};
                if (mat::finite(m.d)) ms.push_back(m);
            }
            size_t best = 0, best_n = 0;
            for (size_t i = 0; i < ms.size(); ++i) {
                size_t n = 0;
                for (const Match& m : ms) n += same_motion(ms[i].d, m.d);
                if (n > best_n) best = i, best_n = n;
            }
            if (ms.empty()) continue;
            for (Match& m : ms) m.inlier = same_motion(ms[best].d, m.d);
            const mat::Mat& d = ms[best].d;
            std::snprintf(line, sizeof(line),
                          "  %s-major: %zu invertible, motion consensus %zu (%.0f%%): rotation %.3f deg, "
                          "translation (%.3f %.3f %.3f)\n",
                          column_major ? "column" : "row", ms.size(), best_n, 100.0 * best_n / ms.size(),
                          rotation_between(d, mat::identity()), d.m[3][0], d.m[3][1], d.m[3][2]);
            f << line;
            if (best_n > listed_inliers) listed = std::move(ms), listed_inliers = best_n;
        }
    }
    if (listed.empty()) return;
    f << "best consensus, per object: draw A -> B, call, inlier, rotation/translation off the consensus\n";
    const mat::Mat* ref = nullptr;
    for (const Match& m : listed)
        if (m.inlier) ref = &m.d;
    size_t n = 0;
    for (const Match& m : listed) {
        if (++n > 400) {
            f << "...\n";
            break;
        }
        std::snprintf(line, sizeof(line), "  %5u -> %5u %s  %s  %.4f deg  %.4f\n", m.a->draw, m.b->draw,
                      draw_text(m.a->call, m.a->geometry).c_str(), m.inlier ? "in " : "OUT",
                      rotation_between(m.d, *ref), translation_between(m.d, *ref));
        f << line;
    }
}

std::string describe(const CameraProfile& c, bool registers) {
    char buf[160];
    const char* stage = stage_name(c.key.stage);
    if (registers) {
        if (c.single_matrix())
            std::snprintf(buf, sizeof(buf), "%s c%u", stage, c.view_offset / 16);
        else if (c.model_view())
            std::snprintf(buf, sizeof(buf), "%s c%u per draw / c%u", stage, c.view_offset / 16, c.proj_offset / 16);
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
    if (c.has_translation) {
        const size_t n = std::strlen(buf);
        const char* sign = c.translation_subtract ? "-" : "+";
        if (registers)
            std::snprintf(buf + n, sizeof(buf) - n, " %s c%u", sign, c.translation_offset / 16);
        else
            std::snprintf(buf + n, sizeof(buf) - n, " %s @ %u", sign, c.translation_offset);
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
    size_t translations = 0;  // hyps of kind Translation
    // Buffers holding a camera-relative candidate's matrices: their float3s are tracked as translations.
    std::unordered_set<BufId, BufIdHash> translated_bufs;
    std::map<std::string, Candidate> candidates;  // keyed by the profile text
    std::deque<std::shared_ptr<DepthFrame>> depths;
    uint64_t last_frame = 0;  // latest analyzed sample frame
    std::vector<uint8_t> window;
    // decode_by_draw's: sampled draws where the candidate's matrices hold one value (pair: both).
    struct Segment {
        uint32_t from = 0, weight = 0;
        const float* a = nullptr;
        const float* b = nullptr;
    };
    std::vector<Segment> segments;
    std::unordered_map<uint64_t, uint32_t> window_bits;  // classify_matrix by window content
    Snapshot snapshots[2];                               // latest, and the one before (report)
    std::deque<Census> censuses;                         // latest last (report)
    bool have_draws = false;                             // the tracker records every draw (D3D9)
    cam::DrawRequest draw_window;                        // ... reading their window here
    std::vector<mv::Draw> solver_input;

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
        if (!sf.samples.all.empty()) {
            have_draws = true;
            draw_window = sf.samples.window;
            if (censuses.empty() || sf.frame >= censuses.back().frame + kCensusEvery) {
                censuses.push_back({sf.frame, sf.samples.draws, sf.samples.all});
                if (censuses.size() > 3) censuses.pop_front();
            }
            solve_model_views(sf);
        }

        // Every window's classification, once per distinct buffer content.
        std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>> classified;
        // Per location: (sample index, value), in draw order.
        std::unordered_map<Loc, std::vector<std::pair<uint32_t, const float*>>, LocHash> values;
        std::unordered_set<BufId, BufIdHash> bound;
        uint32_t buffers = 0;
        for (uint32_t si = 0; si < uint32_t(sf.samples.samples.size()); ++si) {
            for (const cam::BoundBuffer& b : sf.samples.samples[si].buffers) {
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
                            values[loc].emplace_back(si, reinterpret_cast<const float*>(bytes.data() + off));
                        }
                if (translated_bufs.contains(id))
                    for (uint32_t off = 0; off + 64 <= bytes.size(); off += 16) {
                        const auto* f = reinterpret_cast<const float*>(bytes.data() + off);
                        if (looks_like_position(f)) values[Loc{id, off, false, MatrixKind::Translation}].emplace_back(si, f);
                    }
            }
        }
        stats.last_buffers = buffers;

        for (auto& [loc, v] : values) {
            const bool translation = loc.kind == MatrixKind::Translation;
            auto it = hyps.find(loc);
            if (it == hyps.end()) {
                if (hyps.size() >= kMaxHypotheses || (translation && translations >= kMaxTranslations)) continue;
                it = hyps.emplace(loc, Hypothesis{loc}).first;
                translations += translation;
            }
            Hypothesis& h = it->second;
            // A translation is the float3 alone: whatever follows it may differ per draw.
            const size_t n = translation ? 12 : 64;
            Record r;
            r.frame = sf.frame;
            std::memcpy(r.first, v.front().second, sizeof(r.first));
            // The most common value (first seen wins ties).
            std::unordered_map<uint64_t, std::pair<uint32_t, const float*>> counts;
            const float* best = v.front().second;
            uint32_t best_count = 0;
            for (const auto& [si, p] : v) {
                auto& [count, first] = counts[hash_bytes(p, n)];
                if (count++ == 0) first = p;
                if (count > best_count) best_count = count, best = first;
                if (!r.runs.empty() && r.runs.back().to + 1u == si && std::memcmp(r.runs.back().f, p, n) == 0)
                    r.runs.back().to = uint16_t(si);
                else if (r.runs.size() < kMaxRuns)
                    std::memcpy(r.runs.emplace_back(Record::Run{uint16_t(si), uint16_t(si)}).f, p, sizeof(float) * 16);
            }
            std::memcpy(r.common, best, sizeof(r.common));
            r.distinct = uint16_t(std::min<size_t>(counts.size(), 0xFFFF));
            ++h.frames_valid;
            h.per_draw += r.distinct > 1;
            if (!h.history.empty() && std::memcmp(h.history.back().common, r.common, n) != 0) ++h.changes;
            h.history.push_back(r);
            if (h.history.size() > kHistory) h.history.pop_front();
        }
        for (auto& [loc, h] : hyps) h.frames_seen += bound.contains(loc.buf);

        if (stats.frames_analyzed % kRebuildEvery == 0) {
            prune();
            rebuild_candidates();
            snapshot(sf);
        }
    }

    // Every model-view candidate's solver takes this frame's draws; its view (or no pose) is kept
    // for scoring like any other candidate's matrices.
    void solve_model_views(const SampleFrame& sf) {
        for (auto& [k, c] : candidates) {
            if (!c.profile.model_view()) continue;
            if (!c.solver) c.solver = std::make_shared<mv::Solver>();
            cam::solver_draws(sf.samples.all, c.profile.column_major, solver_input);
            const mv::Result r = c.solver->solve(solver_input);
            c.views.emplace_back(sf.frame, r.posed ? std::optional<mat::Mat>(r.view) : std::nullopt);
            if (c.views.size() > kHistory) c.views.pop_front();
        }
    }

    // The windows of every hypothesis, plus the first four of each buffer (where a world-view or
    // world-view-projection that never classifies would sit), over this frame's sampled draws.
    void snapshot(const SampleFrame& sf) {
        constexpr size_t kMaxValues = 24;
        snapshots[1] = std::move(snapshots[0]);
        Snapshot& s = snapshots[0];
        s = {};
        s.frame = sf.frame;
        s.draws = sf.samples.draws;
        s.sampled = uint32_t(sf.samples.samples.size());
        for (const cam::DrawSample& d : sf.samples.samples) s.calls.push_back({d.draw, d.call, d.geometry});
        auto slot = [&](const BufId& id, uint32_t off) -> Snapshot::Window& {
            for (Snapshot::Window& w : s.windows)
                if (w.buf == id && w.offset == off) return w;
            return s.windows.emplace_back(Snapshot::Window{id, off, {}});
        };
        for (const auto& [loc, h] : hyps)
            if (loc.kind != MatrixKind::Translation) slot(loc.buf, loc.offset);
        for (const cam::DrawSample& d : sf.samples.samples)
            for (const cam::BoundBuffer& b : d.buffers)
                for (uint32_t off = 0; off < 256; off += 64) slot({b.key.stage, b.key.slot, b.key.space, b.key.size}, off);
        std::sort(s.windows.begin(), s.windows.end(), [](const Snapshot::Window& x, const Snapshot::Window& y) {
            if (x.buf.stage != y.buf.stage) return uint32_t(x.buf.stage) < uint32_t(y.buf.stage);
            if (x.buf.slot != y.buf.slot) return x.buf.slot < y.buf.slot;
            return x.offset < y.offset;
        });

        for (const cam::DrawSample& d : sf.samples.samples)
            for (const cam::BoundBuffer& b : d.buffers) {
                const BufId id{b.key.stage, b.key.slot, b.key.space, b.key.size};
                const std::vector<uint8_t>& bytes = b.read.bytes;
                for (Snapshot::Window& w : s.windows) {
                    if (!(w.buf == id) || w.offset + 64 > bytes.size()) continue;
                    const uint8_t* p = bytes.data() + w.offset;
                    auto it = std::find_if(w.values.begin(), w.values.end(),
                                           [&](const Snapshot::Value& v) { return std::memcmp(v.f, p, 64) == 0; });
                    if (it == w.values.end()) {
                        if (w.values.size() >= kMaxValues) continue;
                        it = w.values.emplace(w.values.end());
                        std::memcpy(it->f, p, 64);
                        it->bits = classify_matrix(it->f);
                    }
                    it->draws.push_back(d.draw);
                }
            }
    }

    // Hypotheses that mostly don't hold (e.g. a matrix that only sometimes looks rigid) go, and so
    // do candidates built on them.
    void prune() {
        std::erase_if(hyps, [](const auto& kv) {
            const Hypothesis& h = kv.second;
            // A translation that differs per draw isn't the camera's: make room for others.
            const bool per_draw = h.loc.kind == MatrixKind::Translation && h.per_draw * 5 > h.frames_valid;
            return h.frames_seen >= 60 && (h.validity() < 0.5 || per_draw);
        });
        translations = 0;
        for (const auto& [loc, h] : hyps) translations += loc.kind == MatrixKind::Translation;
        std::erase_if(candidates, [&](const auto& kv) {
            const Candidate& c = kv.second;
            return !hyps.contains(c.a) || (c.pair && !hyps.contains(c.b)) ||
                   (c.profile.has_translation && !hyps.contains(c.t));
        });
    }

    // ---- 2. Candidates --------------------------------------------------------------------

    // A candidate whose camera sits at the origin: world points are moved into camera-relative space
    // before its matrices (UE3's TranslatedViewProjection, IW's view origin), so it only has the
    // rotation and needs a translation.
    bool camera_at_origin(const Candidate& c) {
        const auto h = hyps.find(c.a);
        if (h == hyps.end() || h->second.history.empty()) return false;
        mat::Mat v, p;
        if (decode(c, h->second.history.back().frame, v, p) != 0) return false;
        return std::abs(v.m[3][0]) < 1e-3 && std::abs(v.m[3][1]) < 1e-3 && std::abs(v.m[3][2]) < 1e-3;
    }

    // `supersede`: when full, takes the place of the worst candidate without a translation.
    void add_candidate(CameraLayout layout, const Hypothesis& a, const Hypothesis* b, Latch latch,
                       const Hypothesis* t = nullptr, bool subtract = false, bool supersede = false) {
        CameraProfile p;
        p.key = {a.loc.buf.stage, a.loc.buf.slot, a.loc.buf.space, a.loc.buf.size};
        p.layout = layout;
        p.view_offset = a.loc.offset;
        if (b) p.proj_offset = b->loc.offset;
        p.column_major = a.loc.column_major;
        p.latch = latch;
        if (t) p.has_translation = true, p.translation_offset = t->loc.offset, p.translation_subtract = subtract;
        Candidate c;
        c.profile = p;
        c.a = a.loc;
        if (b) c.b = b->loc, c.pair = true;
        if (t) c.t = t->loc;
        std::string key = format_profile(p, {});
        if (candidates.contains(key)) return;
        if (candidates.size() >= kMaxCandidates) {
            if (!supersede) return;
            auto worst = candidates.end();
            for (auto it = candidates.begin(); it != candidates.end(); ++it)
                if (!it->second.profile.has_translation && (worst == candidates.end() || better(worst->second, it->second)))
                    worst = it;
            if (worst == candidates.end()) return;
            candidates.erase(worst);
        }
        candidates.emplace(std::move(key), std::move(c));
    }

    // The best camera-relative candidates (one per layout and window, whichever stage) get a variant
    // per likely translation: the float3s of their buffer that change most, outside any matrix, either
    // sign. Their buffers' float3s are tracked from now on. Scores come from the reprojection tests,
    // which rotation alone passes often enough to rank the right matrices first.
    void add_translated(const std::unordered_map<BufId, std::vector<const Hypothesis*>, BufIdHash>& positions) {
        std::vector<Candidate> relative;
        for (const Candidate* c : ranked()) {
            if (relative.size() >= kRelativeTried) break;
            if (c->profile.has_translation || c->profile.model_view() || !camera_at_origin(*c)) continue;
            translated_bufs.insert(c->a.buf);
            const bool seen = std::any_of(relative.begin(), relative.end(), [&](const Candidate& r) {
                return r.profile.layout == c->profile.layout && r.a.offset == c->a.offset && r.b.offset == c->b.offset &&
                       r.a.column_major == c->a.column_major && r.profile.latch == c->profile.latch &&
                       r.a.buf.slot == c->a.buf.slot && r.a.buf.size == c->a.buf.size;
            });
            if (!seen) relative.push_back(*c);
        }
        for (const Candidate& c : relative) {
            const auto pos = positions.find(c.a.buf);
            if (pos == positions.end()) continue;
            const Hypothesis& a = hyps.at(c.a);
            const Hypothesis* b = c.pair ? &hyps.at(c.b) : nullptr;
            size_t tried = 0;
            for (const Hypothesis* t : pos->second) {
                const bool in_matrix = std::any_of(hyps.begin(), hyps.end(), [&](const auto& kv) {
                    const Loc& m = kv.first;
                    return m.kind != MatrixKind::Translation && m.buf == t->loc.buf && t->loc.offset + 12 > m.offset &&
                           t->loc.offset < m.offset + 64;
                });
                if (in_matrix) continue;
                if (tried++ >= kTranslationsTried) break;
                for (bool subtract : {true, false})
                    add_candidate(c.profile.layout, a, b, c.profile.latch, t, subtract, true);
            }
        }
    }

    void rebuild_candidates() {
        // Eligible: seen for a while and nearly always holding. Grouped by buffer + major, since a
        // profile reads both matrices from one buffer the same way.
        struct Group {
            std::vector<const Hypothesis*> kinds[4];
        };
        std::unordered_map<Loc, Group, LocHash> groups;  // keyed by the buffer + major (offset 0, Rigid)
        std::unordered_map<BufId, std::vector<const Hypothesis*>, BufIdHash> positions;  // translations
        for (const auto& [loc, h] : hyps) {
            if (h.frames_seen < 20 || h.validity() < 0.9) continue;
            if (loc.kind == MatrixKind::Translation) {
                // The camera's: one value per frame, changing as it moves.
                if (h.changes > 0 && h.per_draw * 5 <= h.frames_valid) positions[loc.buf].push_back(&h);
                continue;
            }
            Loc g{loc.buf, 0, loc.column_major, MatrixKind::Rigid};
            groups[g].kinds[int(loc.kind)].push_back(&h);
        }
        for (auto& [buf, v] : positions)
            std::sort(v.begin(), v.end(), [](const Hypothesis* x, const Hypothesis* y) {
                if (x->changes != y->changes) return x->changes > y->changes;
                return x->loc.offset < y->loc.offset;
            });
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
                // A rigid matrix that differs per draw, next to a constant projection: world * view.
                // Only where the tracker records every draw's window.
                for (const Hypothesis* h : rigid)
                    if (apart(h) && per_draw(*h) && p->changes * 5 < p->frames_valid && have_draws &&
                        h->loc.buf.stage == draw_window.key.stage && h->loc.buf.slot == draw_window.key.slot &&
                        h->loc.offset == draw_window.offset)
                        add_candidate(CameraLayout::ModelView, *h, p, Latch::First);
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
        add_translated(positions);
    }

    // The candidate's matrices at `frame`, decoded exactly as a profile would. Returns 0 if they
    // decoded, 1 if values are missing for that frame (not the candidate's fault), 2 if they
    // didn't decode.
    int decode(const Candidate& c, uint64_t frame, mat::Mat& view, mat::Mat& proj, float* view_f = nullptr,
               float* proj_f = nullptr) {
        if (c.profile.model_view()) return decode_model_view(c, frame, view, proj, view_f, proj_f);
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
        if (!p.has_translation) return decode_by_draw(c, *ra, rb, view, proj, view_f, proj_f);
        window.assign(p.window_size(), 0);
        std::memcpy(window.data() + (p.view_offset - p.window_offset()), common ? ra->common : ra->first, 64);
        if (rb) std::memcpy(window.data() + (p.proj_offset - p.window_offset()), common ? rb->common : rb->first, 64);
        if (p.has_translation) {
            const auto ht = hyps.find(c.t);
            const Record* rt = ht != hyps.end() ? ht->second.at(frame) : nullptr;
            if (rt == nullptr) return 1;
            std::memcpy(window.data() + (p.translation_offset - p.window_offset()), common ? rt->common : rt->first, 12);
        }
        float v[16], pr[16];
        if (!decode_camera(p, window.data(), window.size(), v, pr)) return 2;
        view = mat::load(v, false);
        proj = mat::load(pr, false);
        if (view_f) std::memcpy(view_f, v, sizeof(v));
        if (proj_f) std::memcpy(proj_f, pr, sizeof(pr));
        return 0;
    }

    // Without a translation: the camera the tracker would latch. The matrices come from the same draws
    // (the same buffer), and the pick is the first draw whose window decodes, or for Latch::Common, the
    // value most draws had among those that decode. Returns 2 if none decodes.
    int decode_by_draw(const Candidate& c, const Record& ra, const Record* rb, mat::Mat& view, mat::Mat& proj,
                       float* view_f, float* proj_f) {
        segments.clear();
        for (const Record::Run& x : ra.runs) {
            if (rb == nullptr) {
                segments.push_back({x.from, x.to - x.from + 1u, x.f, nullptr});
                continue;
            }
            for (const Record::Run& y : rb->runs) {
                const uint32_t from = std::max(x.from, y.from), to = std::min(x.to, y.to);
                if (from <= to) segments.push_back({from, to - from + 1, x.f, y.f});
            }
        }
        std::sort(segments.begin(), segments.end(), [](const Segment& l, const Segment& r) { return l.from < r.from; });
        if (c.profile.latch == Latch::Common) {  // most draws first; the first seen wins ties
            for (size_t i = 0; i < segments.size(); ++i)
                for (size_t j = i + 1; j < segments.size(); ++j)
                    if (std::memcmp(segments[i].a, segments[j].a, 64) == 0 &&
                        (segments[i].b == nullptr || std::memcmp(segments[i].b, segments[j].b, 64) == 0)) {
                        segments[i].weight += segments[j].weight;
                        segments[j].weight = 0;
                    }
            std::stable_sort(segments.begin(), segments.end(),
                             [](const Segment& l, const Segment& r) { return l.weight > r.weight; });
        }
        const CameraProfile& p = c.profile;
        window.assign(p.window_size(), 0);
        for (const Segment& sg : segments) {
            if (sg.weight == 0) break;
            std::memcpy(window.data() + (p.view_offset - p.window_offset()), sg.a, 64);
            if (sg.b) std::memcpy(window.data() + (p.proj_offset - p.window_offset()), sg.b, 64);
            float v[16], pr[16];
            if (!decode_camera(p, window.data(), window.size(), v, pr)) continue;
            view = mat::load(v, false);
            proj = mat::load(pr, false);
            if (view_f) std::memcpy(view_f, v, sizeof(v));
            if (proj_f) std::memcpy(proj_f, pr, sizeof(pr));
            return 0;
        }
        return 2;
    }

    // The solver's view at `frame`, and the projection hypothesis' value.
    int decode_model_view(const Candidate& c, uint64_t frame, mat::Mat& view, mat::Mat& proj, float* view_f,
                          float* proj_f) {
        const auto v = std::find_if(c.views.rbegin(), c.views.rend(), [&](const auto& e) { return e.first == frame; });
        const auto hb = hyps.find(c.b);
        const Record* rb = hb != hyps.end() ? hb->second.at(frame) : nullptr;
        if (v == c.views.rend() || rb == nullptr) return 1;
        if (!v->second) return 2;
        float pr[16];
        if (!decode_projection(c.profile, reinterpret_cast<const uint8_t*>(rb->common), 64, pr)) return 2;
        view = *v->second;
        proj = mat::load(pr, false);
        if (view_f) mat::store(view, view_f);
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
                    if (r1 == 0) {
                        for (int i = 0; i < 3; ++i) c.right_abs[i] += std::abs(v1.m[i][0]);
                        ++c.right_frames;
                    }
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
            i.profile.z_up = c.z_up();
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
            if (!h->history.empty() && h->loc.kind == MatrixKind::Translation) {
                const float* t = h->history.back().common;
                std::snprintf(line, sizeof(line), "    %12.5g %12.5g %12.5g\n", t[0], t[1], t[2]);
                f << line;
            } else if (!h->history.empty()) {
                const float* m = h->history.back().common;
                for (int r = 0; r < 4; ++r) {
                    std::snprintf(line, sizeof(line), "    %12.5g %12.5g %12.5g %12.5g\n", m[r * 4], m[r * 4 + 1],
                                  m[r * 4 + 2], m[r * 4 + 3]);
                    f << line;
                }
            }
        }

        for (const Snapshot& s : snapshots) {
            if (s.frame == 0) continue;
            std::snprintf(line, sizeof(line),
                          "\n== Per-draw values, frame %llu (%u draws, %u sampled; raw memory order, draws "
                          "listed by number) ==\n",
                          (unsigned long long)s.frame, s.draws, s.sampled);
            f << line;
            for (const Snapshot::Window& w : s.windows) {
                if (w.values.empty()) continue;
                std::snprintf(line, sizeof(line), "%s slot %u offset %u (c%u): %zu distinct%s\n", stage_name(w.buf.stage),
                              w.buf.slot, w.offset, w.offset / 16, w.values.size(),
                              w.values.size() >= 24 ? " (capped)" : "");
                f << line;
                for (const Snapshot::Value& v : w.values) {
                    std::string draws;
                    for (size_t i = 0; i < v.draws.size() && i < 16; ++i) draws += ' ' + std::to_string(v.draws[i]);
                    if (v.draws.size() > 16) draws += " ...";
                    std::snprintf(line, sizeof(line), "  %zu draws [%s ] %s\n", v.draws.size(), draws.c_str() + 1,
                                  kinds_text(v.bits).c_str());
                    f << line;
                    for (int r = 0; r < 4; ++r) {
                        std::snprintf(line, sizeof(line), "    %12.5g %12.5g %12.5g %12.5g\n", v.f[r * 4],
                                      v.f[r * 4 + 1], v.f[r * 4 + 2], v.f[r * 4 + 3]);
                        f << line;
                    }
                }
            }
            f << "sampled draws (call | geometry):\n";
            for (const Snapshot::Draw& d : s.calls) {
                std::snprintf(line, sizeof(line), "  %5u %s\n", d.draw, draw_text(d.call, d.geometry).c_str());
                f << line;
            }
        }

        for (size_t i = 0; i < censuses.size(); ++i) write_census(f, censuses[i], i + 1 == censuses.size());
        for (size_t i = 1; i < censuses.size(); ++i) write_census_pair(f, censuses[i - 1], censuses[i]);
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
