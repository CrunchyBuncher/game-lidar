#include "discovery.h"

#include <windows.h>

#include <algorithm>
#include <fstream>

#include "recording.h"

namespace lidar::disc {

Discovery::~Discovery() { stop(); }

void Discovery::start(bool registers) {
    stop();
    registers_ = registers;
    start_ = std::chrono::steady_clock::now();
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
    record_request_.reset();
}

void Discovery::submit_samples(uint64_t frame, cam::DepthSamples&& samples) {
    {
        const std::lock_guard lock(mutex_);
        if (!worker_.joinable() || quit_) return;
        if (sample_queue_.size() >= 2) {
            ++dropped_;
            return;
        }
        sample_queue_.push_back({frame, now(), std::move(samples)});
    }
    wake_.notify_one();
}

void Discovery::submit_depth(const FrameHeader& h, const float* depth) {
    if (!running() || h.width == 0 || h.height == 0 || h.src_width == 0 || h.src_height == 0) return;
    auto df = std::make_shared<DepthFrame>();
    df->frame = h.frame_index;
    df->time = now();
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

void Discovery::start_recording(const std::filesystem::path& path, const std::string& note, double seconds,
                                uint64_t max_bytes) {
    {
        const std::lock_guard lock(mutex_);
        record_request_ = std::make_unique<Request>(Request{path, note, seconds, max_bytes});
    }
    wake_.notify_one();
}

void Discovery::stop_recording() {
    {
        const std::lock_guard lock(mutex_);
        record_request_ = std::make_unique<Request>();
    }
    wake_.notify_one();
}

RecordingStatus Discovery::recording() const {
    const std::lock_guard lock(mutex_);
    return recording_;
}

void Discovery::run() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
    Analyzer analyzer(registers_);
    Recorder recorder;
    RecordingStatus rec;  // published with the status
    Request limits;
    double record_start = 0;
    for (;;) {
        SampleFrame sf;
        std::shared_ptr<DepthFrame> df;
        std::filesystem::path report;
        std::unique_ptr<Request> record;
        bool have_samples = false;
        uint64_t dropped = 0;
        {
            std::unique_lock lock(mutex_);
            wake_.wait(lock, [&] {
                return quit_ || !sample_queue_.empty() || !depth_queue_.empty() || !report_path_.empty() ||
                       record_request_ != nullptr;
            });
            if (quit_) break;
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
            record.swap(record_request_);
            dropped = dropped_;
        }

        if (record != nullptr) {
            recorder.close();
            rec.active = false;
            if (!record->path.empty()) {
                rec = {};
                rec.path = record->path;
                if (recorder.open(record->path, registers_, record->note, rec.error)) {
                    rec.active = true;
                    limits = *record;
                    record_start = now();
                }
            }
        }
        // What the analyzer gets, in the order it gets it: the recording replays exactly.
        if (recorder.is_open()) {
            if (have_samples) recorder.write(sf);
            if (df) recorder.write(*df);
            rec.seconds = now() - record_start;
            rec.bytes = recorder.bytes();
            rec.sample_frames = recorder.sample_frames();
            rec.depth_frames = recorder.depth_frames();
            if (rec.seconds >= limits.seconds || rec.bytes >= limits.max_bytes) {
                recorder.close();
                rec.active = false;
            }
        }

        if (have_samples) analyzer.add_samples(sf);
        if (df) analyzer.add_depth(std::move(df));
        if (!report.empty()) {
            std::ofstream f(report);
            if (f) analyzer.write_report(f);
        }
        Status s = analyzer.status();
        s.running = true;
        s.frames_dropped = dropped;
        const std::lock_guard lock(mutex_);
        published_ = std::move(s);
        recording_ = rec;
    }
    recorder.close();
    const std::lock_guard lock(mutex_);
    recording_.active = false;
}

}  // namespace lidar::disc
