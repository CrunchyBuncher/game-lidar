#include "recording.h"

#include <cstring>

namespace lidar::disc {
namespace {

constexpr char kMagic[8] = {'L', 'D', 'I', 'S', 'C', 'R', 'E', 'C'};
constexpr uint32_t kVersion = 1;
constexpr uint32_t kFlagRegisters = 1;
enum Chunk : uint8_t { kBlob = 1, kSamples = 2, kDepth = 3 };

// Word-at-a-time: every bound buffer of every sampled draw goes through this.
uint64_t hash_blob(const std::vector<uint8_t>& b) {
    uint64_t h = 0x9e3779b97f4a7c15ull ^ b.size();
    size_t i = 0;
    for (; i + 8 <= b.size(); i += 8) {
        uint64_t w;
        std::memcpy(&w, b.data() + i, 8);
        h = (h ^ w) * 0x100000001b3ull;
        h ^= h >> 29;
    }
    for (; i < b.size(); ++i) h = (h ^ b[i]) * 0x100000001b3ull;
    return h;
}

template <typename T>
void put(std::string& s, T v) {
    s.append(reinterpret_cast<const char*>(&v), sizeof(v));
}
void put_floats(std::string& s, const float* f, size_t n) { s.append(reinterpret_cast<const char*>(f), n * sizeof(float)); }

void put_key(std::string& s, const cam::CbufferKey& k) {
    put(s, uint32_t(k.stage));
    put(s, k.slot);
    put(s, k.space);
    put(s, k.size);
}
void put_call(std::string& s, const cam::DrawCall& c) {
    put(s, uint8_t(c.type));
    put(s, c.count);
    put(s, c.instances);
    put(s, c.first);
    put(s, c.vertex_offset);
    put(s, c.first_instance);
}
void put_geometry(std::string& s, const cam::DrawGeometry& g) {
    put(s, g.vb);
    put(s, g.ib);
    put(s, g.vs);
    put(s, g.vb_offset);
    put(s, g.vb_stride);
    put(s, uint8_t(g.up));
    put(s, g.up_vertices);
    put(s, g.up_indices);
    put(s, g.up_bytes);
    put(s, g.up_hash);
}

// Reads a payload front to back; any overrun sets `bad` and yields zeros.
struct Cursor {
    const std::string& s;
    size_t at = 0;
    bool bad = false;

    template <typename T>
    T get() {
        T v{};
        if (at + sizeof(T) > s.size()) {
            bad = true;
            return v;
        }
        std::memcpy(&v, s.data() + at, sizeof(T));
        at += sizeof(T);
        return v;
    }
    void floats(float* out, size_t n) {
        if (at + n * sizeof(float) > s.size()) {
            bad = true;
            return;
        }
        std::memcpy(out, s.data() + at, n * sizeof(float));
        at += n * sizeof(float);
    }
    cam::CbufferKey key() {
        cam::CbufferKey k;
        k.stage = reshade::api::shader_stage(get<uint32_t>());
        k.slot = get<uint32_t>();
        k.space = get<uint32_t>();
        k.size = get<uint32_t>();
        return k;
    }
    cam::DrawCall call() {
        cam::DrawCall c;
        c.type = cam::DrawType(get<uint8_t>());
        c.count = get<uint32_t>();
        c.instances = get<uint32_t>();
        c.first = get<uint32_t>();
        c.vertex_offset = get<int32_t>();
        c.first_instance = get<uint32_t>();
        return c;
    }
    cam::DrawGeometry geometry() {
        cam::DrawGeometry g;
        g.vb = get<uint64_t>();
        g.ib = get<uint64_t>();
        g.vs = get<uint64_t>();
        g.vb_offset = get<uint32_t>();
        g.vb_stride = get<uint32_t>();
        g.up = get<uint8_t>() != 0;
        g.up_vertices = get<uint64_t>();
        g.up_indices = get<uint64_t>();
        g.up_bytes = get<uint32_t>();
        g.up_hash = get<uint64_t>();
        return g;
    }
    // A count of items at least `min_bytes` each: bounded by what's left, so a corrupt count can't
    // make the reader allocate gigabytes.
    uint32_t count(size_t min_bytes) {
        const uint32_t n = get<uint32_t>();
        if (uint64_t(n) * min_bytes > s.size() - std::min(at, s.size())) {
            bad = true;
            return 0;
        }
        return n;
    }
};

}  // namespace

// ---- Recorder -----------------------------------------------------------------------------------

bool Recorder::open(const std::filesystem::path& path, bool registers, const std::string& note, std::string& error) {
    close();
    file_ = _wfopen(path.c_str(), L"wb");
    if (file_ == nullptr) {
        error = "can't write " + path.string();
        return false;
    }
    std::string h(kMagic, sizeof(kMagic));
    put(h, kVersion);
    put(h, registers ? kFlagRegisters : 0u);
    put(h, uint32_t(note.size()));
    h += note;
    std::fwrite(h.data(), 1, h.size(), file_);
    bytes_ = h.size();
    return true;
}

void Recorder::close() {
    if (file_ != nullptr) std::fclose(file_);
    file_ = nullptr;
    blobs_.clear();
    sample_frames_ = depth_frames_ = 0;
}

void Recorder::chunk(uint8_t type, const std::string& payload) {
    std::string h;
    put(h, type);
    put(h, uint32_t(payload.size()));
    std::fwrite(h.data(), 1, h.size(), file_);
    std::fwrite(payload.data(), 1, payload.size(), file_);
    bytes_ += h.size() + payload.size();
}

uint32_t Recorder::blob(const std::vector<uint8_t>& bytes) {
    const BlobKey key{hash_blob(bytes), uint32_t(bytes.size())};
    const auto [it, fresh] = blobs_.try_emplace(key, uint32_t(blobs_.size()));
    if (fresh) chunk(kBlob, std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()));
    return it->second;
}

void Recorder::write(const SampleFrame& f) {
    if (file_ == nullptr) return;
    const cam::DepthSamples& d = f.samples;
    // Blobs first: the samples chunk refers to them.
    std::vector<uint32_t> ids;
    for (const cam::DrawSample& s : d.samples)
        for (const cam::BoundBuffer& b : s.buffers) ids.push_back(blob(b.read.bytes));
    std::string& p = scratch_;
    p.clear();
    put(p, f.frame);
    put(p, f.time);
    put(p, d.draws);
    put_key(p, d.window.key);
    put(p, d.window.offset);
    put(p, uint32_t(d.samples.size()));
    size_t next = 0;
    for (const cam::DrawSample& s : d.samples) {
        put(p, s.draw);
        put_call(p, s.call);
        put_geometry(p, s.geometry);
        put(p, uint32_t(s.buffers.size()));
        for (const cam::BoundBuffer& b : s.buffers) {
            put_key(p, b.key);
            put(p, ids[next++]);
        }
    }
    put(p, uint32_t(d.all.size()));
    for (const cam::DrawRecord& r : d.all) {
        put(p, r.draw);
        put_call(p, r.call);
        put_geometry(p, r.geometry);
        put(p, uint8_t(r.has_window));
        put_floats(p, r.window, 16);
    }
    chunk(kSamples, p);
    ++sample_frames_;
}

void Recorder::write(const DepthFrame& f) {
    if (file_ == nullptr) return;
    const DepthGrid& g = f.grid;
    std::string& p = scratch_;
    p.clear();
    put(p, f.frame);
    put(p, f.time);
    put(p, g.w);
    put(p, g.h);
    put(p, uint8_t(g.standard));
    put_floats(p, g.depth.data(), g.depth.size());
    put_floats(p, g.ndc_x.data(), g.ndc_x.size());
    put_floats(p, g.ndc_y.data(), g.ndc_y.size());
    chunk(kDepth, p);
    ++depth_frames_;
}

// ---- RecordingReader ----------------------------------------------------------------------------

RecordingReader::~RecordingReader() {
    if (file_ != nullptr) std::fclose(file_);
}

bool RecordingReader::open(const std::filesystem::path& path, std::string& error) {
    file_ = _wfopen(path.c_str(), L"rb");
    if (file_ == nullptr) {
        error = "can't open " + path.string();
        return false;
    }
    char magic[8];
    uint32_t head[3];
    if (std::fread(magic, 1, 8, file_) != 8 || std::memcmp(magic, kMagic, 8) != 0 ||
        std::fread(head, 4, 3, file_) != 3) {
        error = path.string() + " isn't a discovery recording";
        return false;
    }
    if (head[0] != kVersion) {
        error = path.string() + ": recording version " + std::to_string(head[0]) + ", this build reads " +
                std::to_string(kVersion);
        return false;
    }
    registers_ = (head[1] & kFlagRegisters) != 0;
    note_.resize(head[2]);
    if (head[2] > (1u << 20) || std::fread(note_.data(), 1, note_.size(), file_) != note_.size()) {
        error = path.string() + ": truncated header";
        return false;
    }
    return true;
}

bool RecordingReader::next(RecordedInput& out) {
    for (;;) {
        uint8_t type = 0;
        uint32_t size = 0;
        if (std::fread(&type, 1, 1, file_) != 1) return false;  // clean end
        if (std::fread(&size, 4, 1, file_) != 1) {
            error_ = "truncated chunk header";
            return false;
        }
        payload_.resize(size);
        if (std::fread(payload_.data(), 1, size, file_) != size) {
            // A recording cut short (game closed mid-write): everything before this still replays.
            error_ = "truncated chunk (the recording was cut short)";
            return false;
        }
        Cursor c{payload_};
        if (type == kBlob) {
            blobs_.emplace_back(payload_.begin(), payload_.end());
            continue;
        }
        if (type == kSamples) {
            out.is_depth = false;
            out.depth.reset();
            SampleFrame& f = out.samples;
            f = {};
            f.frame = c.get<uint64_t>();
            f.time = c.get<double>();
            cam::DepthSamples& d = f.samples;
            d.draws = c.get<uint32_t>();
            d.window.key = c.key();
            d.window.offset = c.get<uint32_t>();
            d.samples.resize(c.count(8));
            for (cam::DrawSample& s : d.samples) {
                s.draw = c.get<uint32_t>();
                s.call = c.call();
                s.geometry = c.geometry();
                s.buffers.resize(c.count(20));
                for (cam::BoundBuffer& b : s.buffers) {
                    b.key = c.key();
                    const uint32_t id = c.get<uint32_t>();
                    if (id >= blobs_.size()) {
                        c.bad = true;
                        break;
                    }
                    b.read.bytes = blobs_[id];
                    b.read.ready = true;
                    b.read.source_size = b.key.size;
                }
            }
            d.all.resize(c.count(8));
            for (cam::DrawRecord& r : d.all) {
                r.draw = c.get<uint32_t>();
                r.call = c.call();
                r.geometry = c.geometry();
                r.has_window = c.get<uint8_t>() != 0;
                c.floats(r.window, 16);
            }
        } else if (type == kDepth) {
            out.is_depth = true;
            out.depth = std::make_shared<DepthFrame>();
            DepthFrame& f = *out.depth;
            f.frame = c.get<uint64_t>();
            f.time = c.get<double>();
            DepthGrid& g = f.grid;
            g.w = c.get<uint32_t>();
            g.h = c.get<uint32_t>();
            g.standard = c.get<uint8_t>() != 0;
            if (uint64_t(g.w) * g.h * 4 > payload_.size()) {
                c.bad = true;
            } else {
                g.depth.resize(size_t(g.w) * g.h);
                g.ndc_x.resize(g.w);
                g.ndc_y.resize(g.h);
                c.floats(g.depth.data(), g.depth.size());
                c.floats(g.ndc_x.data(), g.ndc_x.size());
                c.floats(g.ndc_y.data(), g.ndc_y.size());
            }
        } else {
            continue;  // a newer chunk type
        }
        if (c.bad) {
            error_ = "corrupt chunk";
            return false;
        }
        return true;
    }
}

}  // namespace lidar::disc
