// Discovery recordings (.disc): the analyzer's inputs, in the order it got them, so a session in a
// game can be replayed offline (lidar_discover) and kept as a regression case (tests/recordings).
//
// Format, little-endian: an 8-byte magic "LDISCREC", u32 version, u32 flags (bit 0: D3D9 registers),
// u32 note length and the note's text, then chunks of { u8 type, u32 payload bytes, payload }.
// Unknown chunk types are skipped. Buffer contents are stored once each (Blob chunks, numbered in
// order) and referenced by number, since most bound buffers don't change between draws or frames.
#pragma once
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "input.h"

namespace lidar::disc {

class Recorder {
public:
    Recorder() = default;
    ~Recorder() { close(); }
    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    // `note`: free text stored in the file (game, API, settings), shown by lidar_discover info.
    bool open(const std::filesystem::path& path, bool registers, const std::string& note, std::string& error);
    void close();
    bool is_open() const { return file_ != nullptr; }

    void write(const SampleFrame& f);
    void write(const DepthFrame& f);

    uint64_t bytes() const { return bytes_; }
    uint64_t sample_frames() const { return sample_frames_; }
    uint64_t depth_frames() const { return depth_frames_; }

private:
    uint32_t blob(const std::vector<uint8_t>& bytes);
    void chunk(uint8_t type, const std::string& payload);

    std::FILE* file_ = nullptr;
    struct BlobKey {
        uint64_t hash;
        uint32_t size;
        bool operator==(const BlobKey&) const = default;
    };
    struct BlobKeyHash {
        size_t operator()(const BlobKey& k) const { return size_t(k.hash ^ k.size); }
    };
    std::unordered_map<BlobKey, uint32_t, BlobKeyHash> blobs_;
    std::string scratch_;
    uint64_t bytes_ = 0, sample_frames_ = 0, depth_frames_ = 0;
};

// One recorded input: samples or a depth frame.
struct RecordedInput {
    bool is_depth = false;
    SampleFrame samples;
    std::shared_ptr<DepthFrame> depth;
};

class RecordingReader {
public:
    RecordingReader() = default;
    ~RecordingReader();
    RecordingReader(const RecordingReader&) = delete;
    RecordingReader& operator=(const RecordingReader&) = delete;

    bool open(const std::filesystem::path& path, std::string& error);
    bool registers() const { return registers_; }
    const std::string& note() const { return note_; }
    // The next input, false at the end. error() is non-empty if the file ended badly.
    bool next(RecordedInput& out);
    const std::string& error() const { return error_; }

private:
    std::FILE* file_ = nullptr;
    bool registers_ = false;
    std::string note_, error_, payload_;
    std::vector<std::vector<uint8_t>> blobs_;
};

}  // namespace lidar::disc
