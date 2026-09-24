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

// Latch::Common: how many draws saw each distinct value.
struct Tally {
    struct Entry {
        uint32_t count = 0;
        uint32_t order = 0;  // first seen, for ties
        CbufferRead read;
    };
    std::unordered_map<uint64_t, Entry> values;
    std::vector<CbufferRead> pending;  // not readable at the draw (D3D12): counted once resolved
    uint32_t next_order = 0;

    void count(const CbufferRead& read, uint32_t n = 1, uint32_t order_bias = 0) {
        Entry& e = values[hash_bytes(read.bytes)];
        if (e.count == 0) {
            e.order = order_bias + next_order++;
            e.read = read;
        }
        e.count += n;
    }
    void resolve(CbufferSource* source) {
        for (CbufferRead& r : pending)
            if (r.ready || (source != nullptr && source->resolve(r))) count(r);
        pending.clear();
    }
    // Moves `src` in, after this tally's own draws.
    void merge(Tally& src, CbufferSource* source) {
        src.resolve(source);
        const uint32_t bias = next_order;
        for (auto& [h, e] : src.values) {
            Entry& d = values[h];
            if (d.count == 0) {
                d.order = bias + e.order;
                d.read = std::move(e.read);
            }
            d.count += e.count;
        }
        next_order = bias + src.next_order;
        src = {};
    }
    const CbufferRead* winner() const {
        const Entry* best = nullptr;
        for (const auto& [h, e] : values)
            if (best == nullptr || e.count > best->count || (e.count == best->count && e.order < best->order)) best = &e;
        return best ? &best->read : nullptr;
    }
};

// Per command list / queue. The immediate D3D11 context is both, so it gets queue state.
struct __declspec(uuid("8e3a51c7-2b94-4f6d-9c08-d4a7e61f3b25")) CmdState {
    explicit CmdState(bool queue) : is_queue(queue) {}
    const bool is_queue;
    resource current_ds{0};
    FrameLatches latches;
    std::unordered_map<uint64_t, Tally> tallies;  // Latch::Common
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

void merge_latches(CbufferSource* source, Latch latch, FrameLatches& dst, FrameLatches& src) {
    for (auto& [ds, read] : src) {
        if (!read.ready && (source == nullptr || !source->resolve(read))) continue;
        if (latch == Latch::Last)
            dst[ds] = std::move(read);
        else
            dst.try_emplace(ds, std::move(read));
    }
    src.clear();
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
    merge_latches(dd.source, dd.req.latch, dst.latches, src.latches);
    for (auto& [ds, t] : src.tallies) dst.tallies[ds].merge(t, dd.source);
    src.tallies.clear();
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
    const uint64_t ds = s.current_ds.handle;
    if (dd.req.latch == Latch::First && s.latches.contains(ds)) return;  // first one wins
    if (!dd.source->read_at_draw(cmd, dd.req.key, dd.req.offset, dd.req.size, s.scratch)) return;
    if (dd.req.latch != Latch::Common) {
        std::swap(s.latches[ds], s.scratch);
        return;
    }
    Tally& t = s.tallies[ds];
    if (s.scratch.ready) {
        t.count(s.scratch);
    } else {
        t.pending.push_back(std::move(s.scratch));
        s.scratch = {};
    }
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

void on_reset_command_list(command_list* cmd) {
    auto& s = *cmd->get_private_data<CmdState>();
    s.current_ds = {0};
    s.pending_geometry = s.pending_copy = nullptr;
    s.latches.clear();
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
        s.latches.clear();
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
    out = std::move(all.latches);
    for (auto& [ds, t] : all.tallies) {
        t.resolve(dd->source);
        if (const CbufferRead* r = t.winner()) out[ds] = *r;
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
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::unregister_event<addon_event::execute_command_list>(on_execute_command_list);
}

}  // namespace lidar::cam
