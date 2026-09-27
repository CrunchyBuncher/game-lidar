#include "camera_tracker.h"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <utility>

using namespace reshade::api;

namespace lidar::cam {
namespace {

uint64_t hash_bytes(const std::vector<uint8_t>& b) {
    uint64_t h = 0xcbf29ce484222325ull;  // FNV-1a
    for (uint8_t c : b) h = (h ^ c) * 0x100000001b3ull;
    return h;
}

// The aspect ratio (width / height) of depth-stencil `ds`, 0 if unknown: the scene camera's has to match.
float target_aspect(device* dev, uint64_t ds) {
    const resource_desc d = dev->get_resource_desc(resource{ds});
    return d.type == resource_type::texture_2d && d.texture.height != 0 ? float(d.texture.width) / float(d.texture.height)
                                                                         : 0.0f;
}

bool accepted(const LatchRequest& req, const CbufferRead& read, float aspect) {
    return !req.accept || req.accept(read.bytes, aspect);
}

// A pass keeps each distinct accepted window (with how many draws had it), up to this many: other views
// can share the register (UE: cube captures, a second camera), so which one is the camera is decided at
// present, against the frames before (main.cpp).
constexpr size_t kMaxAlternatives = 8;

// Adds `r` to `list`, one entry per distinct window (draws add up). `front`: it came first.
void add_alternative(std::vector<LatchedRead>& list, LatchedRead&& r, bool front) {
    for (LatchedRead& e : list)
        if (e.read.bytes == r.read.bytes) {
            e.draws += r.draws;
            return;
        }
    list.insert(front ? list.begin() : list.end(), std::move(r));
}

// Keeps only the accepted windows if there are any, else the first (the reason it's rejected is worth
// showing), and at most kMaxAlternatives.
void settle(std::vector<LatchedRead>& list) {
    if (std::any_of(list.begin(), list.end(), [](const LatchedRead& e) { return e.accepted; }))
        std::erase_if(list, [](const LatchedRead& e) { return !e.accepted; });
    else if (list.size() > 1)
        list.resize(1);
    if (list.size() > kMaxAlternatives) list.resize(kMaxAlternatives);
}

// Latch::Common: how many draws saw each distinct value.
struct Tally {
    struct Entry {
        uint32_t count = 0;
        uint32_t order = 0;  // first seen, for ties
        bool accepted = false;
        CbufferRead read;
    };
    std::unordered_map<uint64_t, Entry> values;
    std::vector<CbufferRead> pending;  // not readable at the draw (D3D12): counted once resolved
    uint32_t next_order = 0;
    float aspect = 0;
    bool aspect_known = false;

    void count(const CbufferRead& read, const LatchRequest& req) {
        Entry& e = values[hash_bytes(read.bytes)];
        if (e.count == 0) {
            e.order = next_order++;
            e.accepted = accepted(req, read, aspect);
            e.read = read;
        }
        ++e.count;
    }
    void resolve(CbufferSource* source, const LatchRequest& req) {
        for (CbufferRead& r : pending)
            if (r.ready || (source != nullptr && source->resolve(r))) count(r, req);
        pending.clear();
    }
    // Moves `src` in, after this tally's own draws.
    void merge(Tally& src, CbufferSource* source, const LatchRequest& req) {
        if (!aspect_known) aspect = src.aspect, aspect_known = src.aspect_known;
        src.resolve(source, req);
        const uint32_t bias = next_order;
        for (auto& [h, e] : src.values) {
            Entry& d = values[h];
            if (d.count == 0) {
                d.order = bias + e.order;
                d.accepted = e.accepted;
                d.read = std::move(e.read);
            }
            d.count += e.count;
        }
        next_order = bias + src.next_order;
        src = {};
    }
    // The values, most common first (the first seen wins ties), settled.
    std::vector<LatchedRead> alternatives() const {
        std::vector<const Entry*> order;
        for (const auto& [h, e] : values) order.push_back(&e);
        std::sort(order.begin(), order.end(), [](const Entry* a, const Entry* b) {
            return a->count != b->count ? a->count > b->count : a->order < b->order;
        });
        std::vector<LatchedRead> out;
        for (const Entry* e : order) out.push_back({e->read, e->count, e->accepted});
        settle(out);
        return out;
    }
};

// Latch::First / Last: a pass' reads, until they can be resolved and checked. Only reads from distinct
// places are kept (most draws bind the same buffer), up to kMaxCandidates: enough for the camera's
// buffer to turn up among the other buffers the register holds, without resolving every draw's.
constexpr size_t kMaxCandidates = 16;
constexpr uint32_t kMaxTries = 64;  // ready reads rejected in a pass before it stops reading

struct Candidates {
    std::vector<CbufferRead> reads;  // in draw order
    std::vector<uint32_t> draws;     // per read: the draws that read that place
    bool settled = false;            // reads is one ready, accepted read (Latch::First: done)
    uint32_t tries = 0;
    float aspect = 0;
    bool aspect_known = false;
};

bool same_place(const CbufferRead& a, const CbufferRead& b) {
    return a.buffer == b.buffer && a.offset == b.offset && a.deferred.descriptor_heap == b.deferred.descriptor_heap &&
           a.deferred.descriptor == b.deferred.descriptor;
}

using PassLatches = std::unordered_map<PassKey, std::vector<LatchedRead>, PassKeyHash>;

// `c`'s reads that resolve, as settled alternatives in latch order (Latch::Last: the last first).
std::vector<LatchedRead> pick(Candidates& c, CbufferSource* source, const LatchRequest& req) {
    std::vector<LatchedRead> out;
    auto take = [&](size_t i) {
        CbufferRead& r = c.reads[i];
        if (!r.ready && (source == nullptr || !source->resolve(r))) return;
        const bool ok = accepted(req, r, c.aspect);
        add_alternative(out, {std::move(r), c.draws[i], ok}, false);
    };
    if (req.latch == Latch::Last)
        for (size_t i = c.reads.size(); i-- > 0;) take(i);
    else
        for (size_t i = 0; i < c.reads.size(); ++i) take(i);
    settle(out);
    return out;
}

// Per command list / queue. The immediate D3D11 context is both, so it gets queue state.
struct __declspec(uuid("8e3a51c7-2b94-4f6d-9c08-d4a7e61f3b25")) CmdState {
    explicit CmdState(bool queue) : is_queue(queue) {}
    const bool is_queue;
    resource current_ds{0};
    std::unordered_map<uint64_t, uint32_t> passes;  // per depth-stencil: depth clears so far
    PassLatches latches;                                             // from command lists merged in
    std::unordered_map<PassKey, Candidates, PassKeyHash> candidates;  // this one's own draws
    std::unordered_map<PassKey, Tally, PassKeyHash> tallies;  // Latch::Common
    FrameSamples samples;                         // discovery
    FrameDraws draws;                             // model-view camera, or discovery
    CbufferRead scratch;  // reused for reads, so steady state doesn't allocate
    // The last recorded draw's geometry, until the source can complete it (UP draws), and a copy
    // to update (the draw's sample, when it's also recorded).
    DrawGeometry* pending_geometry = nullptr;
    DrawGeometry* pending_copy = nullptr;
};

struct __declspec(uuid("f04c9d26-7a1e-4b83-8e5f-39c2d7a6b0e1")) DeviceData {
    std::vector<command_queue*> queues;
    CbufferSource* source = nullptr;
    LatchRequest req;

    CbufferSource* discovery = nullptr;
    uint32_t samples_per_frame = 0, max_bytes = 0;
    std::unordered_map<uint64_t, uint32_t> last_draws;  // per depth-stencil, last frame (sample spacing)
    CbufferSource* recorder = nullptr;  // model-view camera: records every draw
    DrawRequest draw_req;
    // What records draws: the model-view camera's source, else discovery's.
    CbufferSource* record_source() const { return recorder != nullptr ? recorder : discovery; }
};

// Unique: configure, merges and end_frame. Shared: draws, which only touch their own command
// list's state (recorded by a single thread) and read the configuration.
std::shared_mutex g_mutex;

uint32_t pass_of(const std::unordered_map<uint64_t, uint32_t>& passes, uint64_t ds) {
    const auto it = passes.find(ds);
    return it != passes.end() ? it->second : 0;
}

// `later` into `into`, which came first. Latch::Last puts the later windows first.
void combine(Latch latch, std::vector<LatchedRead>& into, std::vector<LatchedRead>&& later) {
    if (latch == Latch::Last)
        for (auto it = later.rbegin(); it != later.rend(); ++it) add_alternative(into, std::move(*it), true);
    else
        for (LatchedRead& e : later) add_alternative(into, std::move(e), false);
    settle(into);
}

// `src`'s latches and candidates into `dst`. `src`'s passes follow `dst_passes`' (a command list
// executed after what the queue has so far).
void merge_latches(CbufferSource* source, const LatchRequest& req,
                   const std::unordered_map<uint64_t, uint32_t>& dst_passes, PassLatches& dst, CmdState& src) {
    // Within `src`, what was merged into it counts as before its own draws (a queue's: the immediate
    // context's).
    for (auto& [key, c] : src.candidates) {
        std::vector<LatchedRead> l = pick(c, source, req);
        if (l.empty()) continue;
        if (const auto it = src.latches.find(key); it != src.latches.end())
            combine(req.latch, it->second, std::move(l));
        else
            src.latches.emplace(key, std::move(l));
    }
    src.candidates.clear();
    for (auto& [key, l] : src.latches) {
        const PassKey k{key.ds, key.pass + pass_of(dst_passes, key.ds)};
        if (const auto it = dst.find(k); it != dst.end())
            combine(req.latch, it->second, std::move(l));
        else
            dst.emplace(k, std::move(l));
    }
    src.latches.clear();
}

void merge_draws(FrameDraws& dst, FrameDraws& src) {
    for (auto& [ds, v] : src) {
        std::vector<DrawRecord>& d = dst[ds];
        const uint32_t base = uint32_t(d.size());
        for (DrawRecord& r : v) {
            r.draw += base;
            d.push_back(r);
        }
    }
    src.clear();
}

void merge_samples(CbufferSource* source, FrameSamples& dst, FrameSamples& src) {
    for (auto& [ds, s] : src) {
        DepthSamples& d = dst[ds];
        for (DrawSample& sample : s.samples) {
            std::erase_if(sample.buffers, [&](BoundBuffer& b) {
                if (!b.read.ready && (source == nullptr || !source->resolve(b.read))) return true;
                b.key.size = b.read.source_size;
                return false;
            });
            sample.draw += d.draws;
            d.samples.push_back(std::move(sample));
        }
        d.draws += s.draws;
    }
    src.clear();
}

void complete_geometry(const DeviceData& dd, CmdState& s) {
    if (s.pending_geometry == nullptr) return;
    if (CbufferSource* src = dd.record_source()) src->complete_geometry(*s.pending_geometry);
    if (s.pending_copy != nullptr) *s.pending_copy = *s.pending_geometry;
    s.pending_geometry = s.pending_copy = nullptr;
}

void merge(const DeviceData& dd, CmdState& dst, CmdState& src) {
    complete_geometry(dd, src);
    merge_latches(dd.source, dd.req, dst.passes, dst.latches, src);
    for (auto& [key, t] : src.tallies)
        dst.tallies[{key.ds, key.pass + pass_of(dst.passes, key.ds)}].merge(t, dd.source, dd.req);
    src.tallies.clear();
    for (const auto& [ds, n] : src.passes) dst.passes[ds] += n;
    src.passes.clear();
    merge_samples(dd.discovery, dst.samples, src.samples);
    merge_draws(dst.draws, src.draws);
}

void on_destroy_device(device* dev) { dev->destroy_private_data<DeviceData>(); }

void on_init_command_list(command_list* cmd) { cmd->create_private_data<CmdState>(false); }
void on_destroy_command_list(command_list* cmd) { cmd->destroy_private_data<CmdState>(); }

void on_init_command_queue(command_queue* q) {
    q->create_private_data<CmdState>(true);
    if ((q->get_type() & command_queue_type::graphics) == 0) return;
    DeviceData* dd = q->get_device()->get_private_data<DeviceData>();
    if (dd == nullptr) return;  // a device the addon doesn't handle
    const std::unique_lock lock(g_mutex);
    dd->queues.push_back(q);
}
void on_destroy_command_queue(command_queue* q) {
    q->destroy_private_data<CmdState>();
    if (DeviceData* dd = q->get_device()->get_private_data<DeviceData>()) {
        const std::unique_lock lock(g_mutex);
        std::erase(dd->queues, q);
    }
}

void on_bind_depth_stencil(command_list* cmd, uint32_t, const resource_view*, resource_view dsv) {
    auto& s = *cmd->get_private_data<CmdState>();
    s.current_ds = dsv != 0 ? cmd->get_device()->get_resource_from_view(dsv) : resource{0};
}

void latch_profile(command_list* cmd, CmdState& s, const DeviceData& dd) {
    const PassKey key{s.current_ds.handle, pass_of(s.passes, s.current_ds.handle)};
    const LatchRequest& req = dd.req;
    if (req.latch == Latch::Common) {
        if (!dd.source->read_at_draw(cmd, req.key, req.offset, req.size, s.scratch)) return;
        Tally& t = s.tallies[key];
        if (!t.aspect_known) t.aspect = target_aspect(cmd->get_device(), key.ds), t.aspect_known = true;
        if (s.scratch.ready) {
            t.count(s.scratch, req);
        } else {
            t.pending.push_back(std::move(s.scratch));
            s.scratch = {};
        }
        return;
    }
    Candidates& c = s.candidates[key];
    if ((c.settled && req.latch == Latch::First) || c.tries >= kMaxTries) return;
    if (!dd.source->read_at_draw(cmd, req.key, req.offset, req.size, s.scratch)) return;
    if (!c.aspect_known) c.aspect = target_aspect(cmd->get_device(), key.ds), c.aspect_known = true;
    if (s.scratch.ready) {  // checked now (D3D11, D3D9)
        if (c.settled && c.reads.front().bytes == s.scratch.bytes) {  // Latch::Last: the same camera
            ++c.draws.front();
            return;
        }
        if (accepted(req, s.scratch, c.aspect)) {
            c.reads.clear();
            c.draws.clear();
            c.settled = true;
        } else {
            ++c.tries;
            if (!c.reads.empty()) return;  // only the first rejected one is kept, to say why
        }
        c.reads.push_back(std::move(s.scratch));
        c.draws.push_back(1);
        s.scratch = {};
        return;
    }
    // Checked once resolved (D3D12): keep it, unless it's a place already kept.
    const auto same = std::find_if(c.reads.begin(), c.reads.end(),
                                   [&](const CbufferRead& r) { return same_place(r, s.scratch); });
    if (same != c.reads.end()) {
        const ptrdiff_t i = same - c.reads.begin();
        ++c.draws[size_t(i)];
        if (req.latch == Latch::Last) {  // now the latest
            std::rotate(same, same + 1, c.reads.end());
            std::rotate(c.draws.begin() + i, c.draws.begin() + i + 1, c.draws.end());
        }
        return;
    }
    if (c.reads.size() >= kMaxCandidates) {
        if (req.latch == Latch::First) return;
        c.reads.erase(c.reads.begin());
        c.draws.erase(c.draws.begin());
    }
    c.reads.push_back(std::move(s.scratch));
    c.draws.push_back(1);
    s.scratch = {};
}

// Every draw's call, geometry and window (model-view camera, discovery). Only where the source reads
// geometry (D3D9 so far): without it, draws can't be told apart. A window that isn't readable at the
// draw (D3D12's deferred reads) is left out.
const DrawRecord* record_draw(command_list* cmd, CmdState& s, const DeviceData& dd, const DrawCall& call) {
    CbufferSource* src = dd.record_source();
    std::vector<DrawRecord>& v = s.draws[s.current_ds.handle];
    DrawRecord& r = v.emplace_back();
    if (!src->read_geometry_at_draw(cmd, r.geometry)) {
        v.pop_back();
        return nullptr;
    }
    r.draw = uint32_t(v.size() - 1);
    r.call = call;
    if (src->read_at_draw(cmd, dd.draw_req.key, dd.draw_req.offset, sizeof(r.window), s.scratch) &&
        s.scratch.ready && s.scratch.bytes.size() == sizeof(r.window)) {
        std::memcpy(r.window, s.scratch.bytes.data(), sizeof(r.window));
        r.has_window = true;
    }
    s.pending_geometry = &r.geometry;
    return &r;
}

// The first few draws, then one every (last frame's draws / budget).
void sample_draw(command_list* cmd, CmdState& s, const DeviceData& dd, const DrawCall& call, const DrawRecord* rec) {
    DepthSamples& d = s.samples[s.current_ds.handle];
    const uint32_t index = d.draws++;
    if (d.samples.size() >= dd.samples_per_frame) return;
    uint32_t stride = 1;
    if (const auto it = dd.last_draws.find(s.current_ds.handle); it != dd.last_draws.end())
        stride = std::max(1u, it->second / dd.samples_per_frame);
    if (index >= 4 && index % stride != 0) return;
    DrawSample& sample = d.samples.emplace_back();
    sample.draw = index;
    sample.call = call;
    if (rec != nullptr) {
        sample.geometry = rec->geometry;
        s.pending_copy = &sample.geometry;
    } else if (dd.discovery->read_geometry_at_draw(cmd, sample.geometry)) {
        s.pending_geometry = &sample.geometry;
    }
    dd.discovery->read_all_at_draw(cmd, dd.max_bytes, sample.buffers);
}

void on_any_draw(command_list* cmd, const DrawCall& call) {
    auto& s = *cmd->get_private_data<CmdState>();
    if (s.current_ds == 0 && s.pending_geometry == nullptr) return;
    const DeviceData* dd = cmd->get_device()->get_private_data<DeviceData>();
    const std::shared_lock lock(g_mutex);
    complete_geometry(*dd, s);  // the previous recorded draw has reached the driver by now
    if (s.current_ds == 0) return;
    if (dd->source != nullptr) latch_profile(cmd, s, *dd);
    const DrawRecord* rec = dd->record_source() != nullptr ? record_draw(cmd, s, *dd, call) : nullptr;
    if (dd->discovery != nullptr) sample_draw(cmd, s, *dd, call, rec);
}

bool on_draw(command_list* cmd, uint32_t vertices, uint32_t instances, uint32_t first_vertex, uint32_t first_instance) {
    on_any_draw(cmd, {DrawType::Draw, vertices, instances, first_vertex, 0, first_instance});
    return false;
}
bool on_draw_indexed(command_list* cmd, uint32_t indices, uint32_t instances, uint32_t first_index,
                     int32_t vertex_offset, uint32_t first_instance) {
    on_any_draw(cmd, {DrawType::Indexed, indices, instances, first_index, vertex_offset, first_instance});
    return false;
}
bool on_draw_indirect(command_list* cmd, indirect_command type, resource, uint64_t, uint32_t draws, uint32_t) {
    if (type != indirect_command::dispatch) on_any_draw(cmd, {DrawType::Indirect, draws});
    return false;
}

// A depth clear starts the depth-stencil's next pass.
bool on_clear_depth_stencil(command_list* cmd, resource_view dsv, const float* depth, const uint8_t*, uint32_t,
                            const rect*) {
    if (dsv == 0 || depth == nullptr) return false;
    auto& s = *cmd->get_private_data<CmdState>();
    const resource ds = cmd->get_device()->get_resource_from_view(dsv);
    const std::shared_lock lock(g_mutex);  // like a draw: only this command list's state
    ++s.passes[ds.handle];
    return false;
}

void on_reset_command_list(command_list* cmd) {
    auto& s = *cmd->get_private_data<CmdState>();
    s.current_ds = {0};
    s.passes.clear();
    s.pending_geometry = s.pending_copy = nullptr;
    s.latches.clear();
    s.candidates.clear();
    s.tallies.clear();
    s.samples.clear();
    s.draws.clear();
}
void on_execute_command_list(command_queue* q, command_list* cmd) {
    if (cmd == q->get_immediate_command_list()) return;  // just the immediate context flushing
    const std::unique_lock lock(g_mutex);
    const DeviceData* dd = q->get_device()->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    auto& dst = *q->get_private_data<CmdState>();
    auto& src = *cmd->get_private_data<CmdState>();
    dst.current_ds = src.current_ds;
    merge(*dd, dst, src);
}

void clear_queues(DeviceData& dd) {
    for (command_queue* q : dd.queues) {
        auto& s = *q->get_private_data<CmdState>();
        s.pending_geometry = s.pending_copy = nullptr;
        s.passes.clear();
        s.latches.clear();
        s.candidates.clear();
        s.tallies.clear();
        s.samples.clear();
        s.draws.clear();
    }
}

}  // namespace

uint64_t object_key(const DrawRecord& r, bool up_by_pointer) {
    const DrawCall& c = r.call;
    const DrawGeometry& g = r.geometry;
    const uint64_t parts[] = {uint64_t(c.type) | (uint64_t(g.up) << 8),
                              c.count,
                              c.instances,
                              c.first,
                              uint64_t(uint32_t(c.vertex_offset)),
                              g.vb,
                              (uint64_t(g.vb_offset) << 32) | g.vb_stride,
                              g.ib,
                              g.vs,
                              g.up ? (up_by_pointer ? g.up_vertices : g.up_hash) : 0,
                              g.up && up_by_pointer ? g.up_indices : 0};
    uint64_t h = 0xcbf29ce484222325ull;  // FNV-1a
    for (uint64_t p : parts)
        for (int i = 0; i < 8; ++i) h = (h ^ ((p >> (i * 8)) & 0xFF)) * 0x100000001b3ull;
    return h;
}

void solver_draws(const std::vector<DrawRecord>& records, bool column_major, std::vector<mv::Draw>& out) {
    out.clear();
    for (const DrawRecord& r : records)
        if (r.has_window) out.push_back({object_key(r), mat::load(r.window, column_major), double(r.call.count)});
}

void init_device(device* dev) {
    if (dev->get_private_data<DeviceData>() == nullptr) dev->create_private_data<DeviceData>();
}

void configure(device* dev, CbufferSource* source, const LatchRequest* req) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::unique_lock lock(g_mutex);
    dd->source = req != nullptr ? source : nullptr;
    if (req != nullptr) dd->req = *req;
    clear_queues(*dd);
}

void configure_draws(device* dev, CbufferSource* source, const DrawRequest* req) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::unique_lock lock(g_mutex);
    dd->recorder = req != nullptr ? source : nullptr;
    dd->draw_req = req != nullptr ? *req : DrawRequest{};
    clear_queues(*dd);
}

void configure_discovery(device* dev, CbufferSource* source, uint32_t samples_per_frame, uint32_t max_bytes) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::unique_lock lock(g_mutex);
    dd->discovery = samples_per_frame != 0 ? source : nullptr;
    dd->samples_per_frame = samples_per_frame;
    dd->max_bytes = max_bytes;
    dd->last_draws.clear();
    clear_queues(*dd);
}

FrameLatches end_frame(device* dev, FrameSamples* samples, FrameDraws* draws) {
    FrameLatches out;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return out;
    const std::unique_lock lock(g_mutex);
    CmdState all(true);
    for (command_queue* q : dd->queues) merge(*dd, all, *q->get_private_data<CmdState>());
    for (auto& [key, l] : all.latches) out.emplace(key, std::move(l));
    for (auto& [key, t] : all.tallies) {
        t.resolve(dd->source, dd->req);
        if (std::vector<LatchedRead> l = t.alternatives(); !l.empty()) out[key] = std::move(l);
    }
    dd->last_draws.clear();
    for (const auto& [ds, s] : all.samples) dd->last_draws[ds] = s.draws;
    if (samples != nullptr) {
        for (auto& [ds, v] : all.draws) {
            DepthSamples& d = all.samples[ds];
            if (draws != nullptr)
                d.all = v;
            else
                d.all = std::move(v);
            d.window = dd->draw_req;
        }
        *samples = std::move(all.samples);
    }
    if (draws != nullptr) *draws = std::move(all.draws);
    return out;
}

void register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(init_device);
    reshade::register_event<addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<addon_event::init_command_list>(on_init_command_list);
    reshade::register_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::register_event<addon_event::init_command_queue>(on_init_command_queue);
    reshade::register_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
    reshade::register_event<addon_event::bind_render_targets_and_depth_stencil>(on_bind_depth_stencil);
    reshade::register_event<addon_event::draw>(on_draw);
    reshade::register_event<addon_event::draw_indexed>(on_draw_indexed);
    reshade::register_event<addon_event::draw_or_dispatch_indirect>(on_draw_indirect);
    reshade::register_event<addon_event::clear_depth_stencil_view>(on_clear_depth_stencil);
    reshade::register_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::register_event<addon_event::execute_command_list>(on_execute_command_list);
}

void unregister_events() {
    using reshade::addon_event;
    reshade::unregister_event<addon_event::init_device>(init_device);
    reshade::unregister_event<addon_event::destroy_device>(on_destroy_device);
    reshade::unregister_event<addon_event::init_command_list>(on_init_command_list);
    reshade::unregister_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::unregister_event<addon_event::init_command_queue>(on_init_command_queue);
    reshade::unregister_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
    reshade::unregister_event<addon_event::bind_render_targets_and_depth_stencil>(on_bind_depth_stencil);
    reshade::unregister_event<addon_event::draw>(on_draw);
    reshade::unregister_event<addon_event::draw_indexed>(on_draw_indexed);
    reshade::unregister_event<addon_event::draw_or_dispatch_indirect>(on_draw_indirect);
    reshade::unregister_event<addon_event::clear_depth_stencil_view>(on_clear_depth_stencil);
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::unregister_event<addon_event::execute_command_list>(on_execute_command_list);
}

}  // namespace lidar::cam
