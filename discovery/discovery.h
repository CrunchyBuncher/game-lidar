// Discovery as the addon runs it: the analyzer (analyzer.h) on a low-priority worker thread, so
// nothing heavy runs on the game's threads, and optionally a recording of its inputs (recording.h).
//
// The present thread hands over what the camera tracker sampled at draws into the captured
// depth-stencil (every bound constant buffer) and the depth frames the capture published.
#pragma once
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "analyzer.h"
#include "protocol.h"

namespace lidar::disc {

struct RecordingStatus {
    bool active = false;
    std::filesystem::path path;  // the latest recording
    double seconds = 0;          // recorded so far (or in total, once stopped)
    uint64_t bytes = 0, sample_frames = 0, depth_frames = 0;
    std::string error;
};

class Discovery {
public:
    Discovery() = default;
    ~Discovery();
    Discovery(const Discovery&) = delete;
    Discovery& operator=(const Discovery&) = delete;

    // Starts (or restarts, from scratch) the worker. `registers`: D3D9, offsets are registers.
    void start(bool registers);
    void stop();  // joins the worker; keeps the last status
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

    // Records what the analyzer gets from now on to `path`, for `seconds` or up to `max_bytes`,
    // whichever comes first. The worker writes it, off the game's threads. `note` goes in the file.
    void start_recording(const std::filesystem::path& path, const std::string& note, double seconds,
                         uint64_t max_bytes);
    void stop_recording();
    RecordingStatus recording() const;

private:
    struct Request {
        std::filesystem::path path;  // empty: stop
        std::string note;
        double seconds = 0;
        uint64_t max_bytes = 0;
    };
    void run();
    double now() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count(); }

    std::thread worker_;
    bool registers_ = false;
    std::chrono::steady_clock::time_point start_;
    mutable std::mutex mutex_;  // everything below
    std::condition_variable wake_;
    bool quit_ = false;
    std::deque<SampleFrame> sample_queue_;
    std::deque<std::shared_ptr<DepthFrame>> depth_queue_;
    uint64_t dropped_ = 0;
    Status published_;
    std::filesystem::path report_path_;
    std::unique_ptr<Request> record_request_;  // start (path set) or stop (path empty)
    RecordingStatus recording_;
};

}  // namespace lidar::disc
