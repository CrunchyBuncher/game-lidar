// The discovery analyzer: finds a game's camera in its constant data without a profile (spec.md §3.2).
// Single-threaded and deterministic: the same inputs in the same order give the same result (time
// comes from the inputs, not a clock), so recordings replay exactly and tests can drive it directly.
//
//  1. Scans every 16-byte-aligned 4x4 window for view / inverse view (rigid), projection and
//     view-projection (or its inverse) patterns, row- and column-major. Each (buffer, offset,
//     kind, major) is a hypothesis with a per-frame history: the first draw's value and the value
//     most draws saw (world * view * proj uploads differ per draw).
//  2. Combines hypotheses into camera candidates, each a complete CameraProfile (view+proj pairs in
//     one buffer, or a single view-projection), decoded exactly as the profile path would.
//  3. Scores each candidate over time: its view must stay put when the depth image is still and
//     change when it changes, and reprojecting depth frame N into N+k with its matrices must land on
//     frame N+k's depth. Only the right matrices pass that consistently.
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "input.h"
#include "profile.h"

namespace lidar::disc {

struct CandidateInfo {
    CameraProfile profile;
    std::string where;        // e.g. "vertex c0 / c4" or "pixel b0 @ 0 / 64"
    double score = 0;         // 0..1
    bool confident = false;
    uint32_t reproj_tests = 0, reproj_passes = 0;
    double reproj_error = 1;  // median relative error over its tests
    uint32_t temporal_checks = 0, temporal_agree = 0;
    bool have_values = false;  // latest decoded matrices
    float view[16] = {}, proj[16] = {};
    ProjectionInfo info;
};

struct Status {
    bool running = false;
    double seconds = 0;  // input time of the latest input
    uint64_t frames_analyzed = 0, frames_dropped = 0;
    uint64_t depth_frames = 0, still_frames = 0, moving_frames = 0, reproj_rounds = 0;
    uint32_t last_samples = 0, last_draws = 0, last_buffers = 0;
    double analyze_ms = 0;      // analyzer time per sample frame (smoothed; measured, not deterministic)
    size_t hypotheses[5] = {};  // per MatrixKind
    std::vector<CandidateInfo> candidates;  // best first
};

// The status' counters as one line (candidates aside), for ReShade.log and the report.
std::string status_line(const Status& s);
// One candidate as one line (log, replay output).
std::string candidate_line(const CandidateInfo& c);
// Whether two profiles name the same camera: buffer, layout, offsets, major, translation (not latch,
// handedness, units or up axis, which don't change where the camera is read).
bool same_camera(const CameraProfile& a, const CameraProfile& b);

class Analyzer {
public:
    // `registers`: D3D9, offsets are shader constant registers (only changes how places are named).
    explicit Analyzer(bool registers);
    ~Analyzer();
    Analyzer(const Analyzer&) = delete;
    Analyzer& operator=(const Analyzer&) = delete;

    void add_samples(const SampleFrame& frame);
    // Depth frames are kept by pointer (the live service shares them with its queue).
    void add_depth(std::shared_ptr<const DepthFrame> frame);

    // The counters and the best `candidates` candidates.
    Status status(size_t candidates = 16) const;
    // Everything it knows: all candidates, hypotheses, per-draw values and draw censuses.
    void write_report(std::ostream& out) const;

private:
    struct State;
    std::unique_ptr<State> state_;
};

}  // namespace lidar::disc
