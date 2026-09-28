// Recordings: what goes in comes back out, and a replay gives the result the live analyzer had.
#include <cstring>
#include <filesystem>
#include <fstream>

#include "recording.h"
#include "replay.h"
#include "synthetic.h"
#include "test.h"

using namespace lidar;
using namespace lidar::test;

namespace {

std::filesystem::path temp_file(const char* name) {
    return std::filesystem::temp_directory_path() / (std::string("lidar_discovery_test_") + name + ".disc");
}

bool same_samples(const cam::DepthSamples& a, const cam::DepthSamples& b) {
    if (a.draws != b.draws || a.samples.size() != b.samples.size() || a.all.size() != b.all.size()) return false;
    if (a.window.key.slot != b.window.key.slot || a.window.offset != b.window.offset) return false;
    for (size_t i = 0; i < a.samples.size(); ++i) {
        const cam::DrawSample &x = a.samples[i], &y = b.samples[i];
        if (x.draw != y.draw || x.call.count != y.call.count || x.call.type != y.call.type ||
            x.buffers.size() != y.buffers.size())
            return false;
        for (size_t k = 0; k < x.buffers.size(); ++k) {
            const cam::BoundBuffer &p = x.buffers[k], &q = y.buffers[k];
            if (p.key.stage != q.key.stage || p.key.slot != q.key.slot || p.key.space != q.key.space ||
                p.key.size != q.key.size || p.read.bytes != q.read.bytes || !q.read.ready)
                return false;
        }
    }
    for (size_t i = 0; i < a.all.size(); ++i)
        if (a.all[i].draw != b.all[i].draw || a.all[i].has_window != b.all[i].has_window ||
            std::memcmp(a.all[i].window, b.all[i].window, sizeof(a.all[i].window)) != 0 ||
            a.all[i].geometry.up_hash != b.all[i].geometry.up_hash)
            return false;
    return true;
}

bool same_depth(const disc::DepthFrame& a, const disc::DepthFrame& b) {
    return a.frame == b.frame && a.time == b.time && a.grid.w == b.grid.w && a.grid.h == b.grid.h &&
           a.grid.standard == b.grid.standard && a.grid.depth == b.grid.depth && a.grid.ndc_x == b.grid.ndc_x &&
           a.grid.ndc_y == b.grid.ndc_y;
}

bool write(const std::filesystem::path& path, const std::vector<disc::RecordedInput>& inputs, const std::string& note) {
    disc::Recorder r;
    std::string error;
    if (!r.open(path, false, note, error)) return false;
    for (const disc::RecordedInput& in : inputs) {
        if (in.is_depth)
            r.write(*in.depth);
        else
            r.write(in.samples);
    }
    return true;
}

// Everything in a recording, with the reader closed again after (Windows can't delete an open file).
struct ReadBack {
    bool opened = false;
    std::string error, note;  // open()'s error, else next()'s
    bool registers = false;
    std::vector<disc::RecordedInput> inputs;
};
ReadBack read_back(const std::filesystem::path& path) {
    ReadBack r;
    disc::RecordingReader reader;
    r.opened = reader.open(path, r.error);
    if (!r.opened) return r;
    r.note = reader.note();
    r.registers = reader.registers();
    disc::RecordedInput in;
    while (reader.next(in)) r.inputs.push_back(std::move(in));
    r.error = reader.error();
    return r;
}

}  // namespace

TEST_CASE(recording, round_trip) {
    Scenario s;
    s.frames = 40;
    std::vector<disc::RecordedInput> inputs = generate(s);
    // A recorded draw list too (D3D9 records every draw), with a UP draw's hash.
    cam::DrawRecord rec;
    rec.draw = 7;
    rec.has_window = true;
    rec.window[5] = 2.5f;
    rec.geometry.up = true;
    rec.geometry.up_hash = 0x1234567890abcdefull;
    inputs.front().samples.samples.all.push_back(rec);
    inputs.front().samples.samples.window.offset = 48;

    const std::filesystem::path path = temp_file("round_trip");
    EXPECT(write(path, inputs, "game=synthetic\n"));
    const ReadBack r = read_back(path);
    EXPECT(r.opened && r.error.empty());
    EXPECT(r.note == "game=synthetic\n");
    EXPECT(!r.registers);
    EXPECT(r.inputs.size() == inputs.size());
    bool same = r.inputs.size() == inputs.size();
    for (size_t i = 0; same && i < inputs.size(); ++i) {
        const disc::RecordedInput &got = r.inputs[i], &want = inputs[i];
        same = got.is_depth == want.is_depth &&
               (got.is_depth ? same_depth(*got.depth, *want.depth)
                             : got.samples.frame == want.samples.frame && got.samples.time == want.samples.time &&
                                   same_samples(got.samples.samples, want.samples.samples));
    }
    EXPECT(same);
    // Buffers repeat between draws and frames: each distinct one is stored once.
    size_t raw = 0, depth = 0;
    for (const disc::RecordedInput& i : inputs)
        if (i.is_depth)
            depth += i.depth->grid.depth.size() * sizeof(float);
        else
            for (const cam::DrawSample& d : i.samples.samples.samples)
                for (const cam::BoundBuffer& b : d.buffers) raw += b.read.bytes.size();
    EXPECT(std::filesystem::file_size(path) < depth + raw / 4);
    std::filesystem::remove(path);
}

// A game closed mid-recording leaves a cut file: everything before the cut still replays.
TEST_CASE(recording, truncated) {
    Scenario s;
    s.frames = 20;
    const std::vector<disc::RecordedInput> inputs = generate(s);
    const std::filesystem::path path = temp_file("truncated");
    EXPECT(write(path, inputs, ""));
    std::filesystem::resize_file(path, std::filesystem::file_size(path) - 100);
    const ReadBack cut = read_back(path);
    EXPECT(cut.opened);
    EXPECT(!cut.inputs.empty() && cut.inputs.size() < inputs.size());
    EXPECT(!cut.error.empty());

    std::ofstream(path, std::ios::binary) << "not a recording";
    const ReadBack bad = read_back(path);
    EXPECT(!bad.opened && !bad.error.empty());
    std::filesystem::remove(path);
}

// Replaying a recording gives what analyzing the inputs directly gave.
TEST_CASE(recording, replay_matches_live) {
    Scenario s;
    s.frames = 240;
    const std::vector<disc::RecordedInput> inputs = generate(s);
    disc::Analyzer live(false);
    for (const disc::RecordedInput& in : inputs) {
        if (in.is_depth)
            live.add_depth(in.depth);
        else
            live.add_samples(in.samples);
    }
    const disc::Status want = live.status(64);

    const std::filesystem::path path = temp_file("replay");
    EXPECT(write(path, inputs, ""));
    disc::ReplayOptions o;
    o.expected = expected_camera(s);
    const disc::ReplayResult r = disc::replay(path, o);
    std::filesystem::remove(path);
    EXPECT(r.ok && r.error.empty());
    EXPECT(r.sample_frames == s.frames && r.depth_frames == s.frames);
    EXPECT(r.final.candidates.size() == want.candidates.size());
    for (size_t i = 0; i < r.final.candidates.size() && i < want.candidates.size(); ++i)
        EXPECT(disc::same_camera(r.final.candidates[i].profile, want.candidates[i].profile) &&
               r.final.candidates[i].score == want.candidates[i].score);
    // The expected camera's timeline is filled in.
    EXPECT(r.found());
    EXPECT(r.expected_first_top >= 0 && r.expected_top_since >= r.expected_first_top);
    EXPECT(r.expected_confident_at >= r.expected_first_top);
}
