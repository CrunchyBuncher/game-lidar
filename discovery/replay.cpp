#include "replay.h"

#include <chrono>

#include "recording.h"

namespace lidar::disc {

ReplayResult replay(const std::filesystem::path& recording, const ReplayOptions& o) {
    ReplayResult r;
    RecordingReader reader;
    if (!reader.open(recording, r.error)) return r;
    r.note = reader.note();
    r.registers = reader.registers();
    const auto wall0 = std::chrono::steady_clock::now();

    Analyzer analyzer(reader.registers());
    RecordedInput in;
    double first = -1, next_progress = o.progress_every;
    bool was_top = false;
    while (reader.next(in)) {
        const double t = in.is_depth ? in.depth->time : in.samples.time;
        if (first < 0) first = t;
        r.seconds = t - first;
        if (in.is_depth) {
            ++r.depth_frames;
            analyzer.add_depth(in.depth);
        } else {
            ++r.sample_frames;
            analyzer.add_samples(in.samples);
        }

        const bool progress = o.log != nullptr && o.progress_every > 0 && r.seconds >= next_progress;
        // The best candidate after each sample frame (candidates are rebuilt on those; checking after
        // every depth frame too would double the cost for nothing).
        if (!o.expected && !progress) continue;
        if (in.is_depth && !progress) continue;
        const Status s = analyzer.status(1);
        const bool top = o.expected && !s.candidates.empty() && same_camera(s.candidates.front().profile, *o.expected);
        if (o.expected) {
            if (top && r.expected_first_top < 0) r.expected_first_top = r.seconds;
            if (top && !was_top) r.expected_top_since = r.seconds;
            if (!top) r.expected_top_since = -1;
            if (top && s.candidates.front().confident && r.expected_confident_at < 0) r.expected_confident_at = r.seconds;
            was_top = top;
        }
        if (progress) {
            next_progress += o.progress_every;
            *o.log << status_line(s) << "\n";
            if (!s.candidates.empty())
                *o.log << "  best: " << candidate_line(s.candidates.front()) << (top ? "  <- expected" : "")
                       << "\n";
        }
    }
    r.ok = true;
    r.error = reader.error();  // a recording cut short: everything before the cut replayed

    r.final = analyzer.status(o.candidates);
    if (o.expected)
        for (size_t i = 0; i < r.final.candidates.size(); ++i)
            if (same_camera(r.final.candidates[i].profile, *o.expected)) {
                r.expected_rank = int(i);
                break;
            }
    if (o.report != nullptr) analyzer.write_report(*o.report);
    r.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    return r;
}

}  // namespace lidar::disc
