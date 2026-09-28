// lidar_discover: discovery offline, on recordings made in a game (discovery/README.md).
//
//   lidar_discover info <file.disc>
//       What's in a recording: the note, duration, frames, draws / samples / buffers per frame.
//   lidar_discover replay <file.disc> [--expect <profile.toml>] [--report <out.txt>] [--progress <s>] [--top <n>]
//       Runs discovery on it and prints the ranking. --expect: the right camera (exit code 1 if it
//       doesn't end as the confident best candidate). --report: the full report, as the addon writes it.
//   lidar_discover check [<dir>]
//       Replays every <name>.disc in the folder that has a <name>.toml next to it, and prints a table
//       (default folder: discovery/tests/recordings). Exit code 1 if any camera wasn't found.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "recording.h"
#include "replay.h"

using namespace lidar;
using namespace lidar::disc;

namespace {

int usage() {
    std::fprintf(stderr,
                 "usage: lidar_discover info <file.disc>\n"
                 "       lidar_discover replay <file.disc> [--expect <profile.toml>] [--report <out.txt>] "
                 "[--progress <seconds>] [--top <n>]\n"
                 "       lidar_discover check [<dir>]\n");
    return 2;
}

std::string time_text(double t) {
    if (t < 0) return "never";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f s", t);
    return buf;
}

int info(const std::filesystem::path& path) {
    RecordingReader reader;
    std::string error;
    if (!reader.open(path, error)) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    std::printf("%s\n%s", path.string().c_str(), reader.note().c_str());
    std::printf("registers (D3D9): %s\n", reader.registers() ? "yes" : "no");
    RecordedInput in;
    uint64_t samples = 0, depths = 0, draws = 0, sampled = 0, buffers = 0, all = 0, bytes = 0;
    double first = -1, last = 0;
    uint32_t grid_w = 0, grid_h = 0;
    std::set<std::tuple<uint32_t, uint32_t, uint32_t, uint32_t>> keys;
    while (reader.next(in)) {
        const double t = in.is_depth ? in.depth->time : in.samples.time;
        if (first < 0) first = t;
        last = t;
        if (in.is_depth) {
            ++depths;
            grid_w = in.depth->grid.w, grid_h = in.depth->grid.h;
            continue;
        }
        ++samples;
        const cam::DepthSamples& d = in.samples.samples;
        draws += d.draws;
        sampled += d.samples.size();
        all += d.all.size();
        for (const cam::DrawSample& s : d.samples)
            for (const cam::BoundBuffer& b : s.buffers) {
                ++buffers;
                bytes += b.read.bytes.size();
                keys.insert({uint32_t(b.key.stage), b.key.slot, b.key.space, b.key.size});
            }
    }
    if (!reader.error().empty()) std::printf("warning: %s\n", reader.error().c_str());
    const double n = double(std::max<uint64_t>(samples, 1));
    std::printf("%.1f s: %llu sample frames, %llu depth frames (grid %ux%u)\n", first < 0 ? 0 : last - first,
                (unsigned long long)samples, (unsigned long long)depths, grid_w, grid_h);
    std::printf("per sample frame: %.0f draws, %.1f sampled, %.1f buffers (%.0f KB), %.0f recorded draws\n", draws / n,
                sampled / n, buffers / n, bytes / n / 1024, all / n);
    std::printf("distinct buffers (stage, slot, space, size): %zu\n", keys.size());
    return 0;
}

bool load_expected(const std::string& path, CameraProfile& out) {
    Profile p;
    std::string error;
    if (!load_profile(path, p, error) || !p.has_camera) {
        std::fprintf(stderr, "%s: %s\n", path.c_str(), error.empty() ? "no [camera]" : error.c_str());
        return false;
    }
    out = p.camera;
    return true;
}

void print_expected(const ReplayResult& r) {
    std::printf("expected camera: %s, rank %s; first best at %s, best since %s, confident best at %s\n",
                r.found() ? "FOUND" : "NOT FOUND",
                r.expected_rank < 0 ? "- (not a candidate)" : std::to_string(r.expected_rank + 1).c_str(),
                time_text(r.expected_first_top).c_str(), time_text(r.expected_top_since).c_str(),
                time_text(r.expected_confident_at).c_str());
}

int replay_cmd(int argc, char** argv) {
    if (argc < 3) return usage();
    const std::filesystem::path path = argv[2];
    ReplayOptions o;
    std::string report_path;
    size_t top = 10;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (i + 1 >= argc) return usage();
        if (a == "--expect") {
            CameraProfile c;
            if (!load_expected(argv[++i], c)) return 2;
            o.expected = c;
        } else if (a == "--report") {
            report_path = argv[++i];
        } else if (a == "--progress") {
            o.progress_every = std::atof(argv[++i]);
        } else if (a == "--top") {
            top = size_t(std::max(1, std::atoi(argv[++i])));
        } else {
            return usage();
        }
    }
    std::ofstream report;
    if (!report_path.empty()) {
        report.open(report_path);
        if (!report) {
            std::fprintf(stderr, "can't write %s\n", report_path.c_str());
            return 2;
        }
        o.report = &report;
    }
    o.log = &std::cout;
    o.candidates = std::max<size_t>(top, 64);
    const ReplayResult r = replay(path, o);
    if (!r.ok) {
        std::fprintf(stderr, "%s\n", r.error.c_str());
        return 2;
    }
    if (!r.error.empty()) std::printf("warning: %s\n", r.error.c_str());
    std::printf("\n%s\n", status_line(r.final).c_str());
    std::printf("replayed %.1f s of recording in %.1f s\n\n", r.seconds, r.wall_seconds);
    for (size_t i = 0; i < r.final.candidates.size() && i < top; ++i) {
        const CandidateInfo& c = r.final.candidates[i];
        const bool expected = o.expected && same_camera(c.profile, *o.expected);
        std::printf("#%zu %s%s\n", i + 1, candidate_line(c).c_str(), expected ? "  <- expected" : "");
    }
    if (r.final.candidates.empty()) std::printf("no candidates\n");
    if (!o.expected) return 0;
    std::printf("\n");
    print_expected(r);
    return r.found() ? 0 : 1;
}

#ifndef LIDAR_DISCOVERY_RECORDINGS
#define LIDAR_DISCOVERY_RECORDINGS "."
#endif

int check(int argc, char** argv) {
    const std::filesystem::path dir = argc >= 3 ? argv[2] : LIDAR_DISCOVERY_RECORDINGS;
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec))
        if (e.path().extension() == ".disc") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) {
        std::printf("no recordings in %s\n", dir.string().c_str());
        return 0;
    }
    int failed = 0;
    std::printf("%-40s %-10s %6s %5s %10s %10s %8s\n", "recording", "result", "length", "rank", "best since",
                "confident", "replay");
    for (const std::filesystem::path& f : files) {
        const std::filesystem::path toml = std::filesystem::path(f).replace_extension(".toml");
        if (!std::filesystem::exists(toml)) {
            std::printf("%-40s (no %s: skipped)\n", f.filename().string().c_str(), toml.filename().string().c_str());
            continue;
        }
        ReplayOptions o;
        CameraProfile c;
        if (!load_expected(toml.string(), c)) {
            ++failed;
            continue;
        }
        o.expected = c;
        const ReplayResult r = replay(f, o);
        if (!r.ok) {
            std::printf("%-40s ERROR %s\n", f.filename().string().c_str(), r.error.c_str());
            ++failed;
            continue;
        }
        failed += !r.found();
        std::printf("%-40s %-10s %5.1fs %5s %10s %10s %7.1fs\n", f.filename().string().c_str(),
                    r.found() ? "found" : "NOT FOUND", r.seconds,
                    r.expected_rank < 0 ? "-" : std::to_string(r.expected_rank + 1).c_str(),
                    time_text(r.expected_top_since).c_str(), time_text(r.expected_confident_at).c_str(), r.wall_seconds);
    }
    return failed == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string cmd = argv[1];
    if (cmd == "info" && argc == 3) return info(argv[2]);
    if (cmd == "replay") return replay_cmd(argc, argv);
    if (cmd == "check") return check(argc, argv);
    return usage();
}
