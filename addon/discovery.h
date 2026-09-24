// Discovery mode: finds a game's camera in its constant data without a profile (spec.md §3.2).
//
// The present thread hands over what the camera tracker sampled at draws into the captured
// depth-stencil (every bound constant buffer) and the depth frames the capture published. A
// low-priority worker thread does the rest, so nothing heavy runs on the game's threads:
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
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "camera_tracker.h"
#include "profile.h"
#include "protocol.h"

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
    double seconds = 0;
    uint64_t frames_analyzed = 0, frames_dropped = 0;
    uint64_t depth_frames = 0, still_frames = 0, moving_frames = 0, reproj_rounds = 0;
    uint32_t last_samples = 0, last_draws = 0, last_buffers = 0;
    double analyze_ms = 0;  // worker time per analyzed frame (smoothed)
    size_t hypotheses[4] = {};  // per MatrixKind
    std::vector<CandidateInfo> candidates;  // best first
};

class Discovery {
public:
    Discovery() = default;
    ~Discovery();
    Discovery(const Discovery&) = delete;
    Discovery& operator=(const Discovery&) = delete;

    // Starts (or restarts, from scratch) the worker. `registers`: D3D9, offsets are registers.
    void start(bool registers);
    void stop();   // joins the worker; keeps the last status
    bool running() const { return worker_.joinable(); }

    // Present thread. Frame `frame`'s samples for the captured depth-stencil. Dropped if the
    // worker is behind.
    void submit_samples(uint64_t frame, cam::DepthSamples&& samples);
    // Present thread. A depth frame as published to the ring (header.frame_index pairs it with
    // the samples of the same frame).
    void submit_depth(const FrameHeader& header, const float* depth);

    Status status() const;
    // Has the worker write everything it knows (all hypotheses and candidates) to `path`.
    void request_report(const std::filesystem::path& path);

private:
    struct State;
    struct SampleFrame {
        uint64_t frame = 0;
        cam::DepthSamples samples;
    };
    struct DepthFrame;

    void run();

    std::thread worker_;
    bool registers_ = false;
    mutable std::mutex mutex_;  // everything below
    std::condition_variable wake_;
    bool quit_ = false;
    std::deque<SampleFrame> sample_queue_;
    std::deque<std::shared_ptr<DepthFrame>> depth_queue_;
    uint64_t dropped_ = 0;
    Status published_;
    std::filesystem::path report_path_;
};

}  // namespace lidar::disc
