#include "depth_tracker.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

using namespace reshade::api;

namespace lidar::depth {
namespace {

struct DsFrameStats {
    DrawStats total;
    DrawStats segment;  // since the last depth clear
    uint32_t clears = 0;  // depth clears so far: the current pass' number
    bool cleared = false;
    bool reversed_clear = false;
};

void add(DrawStats& d, const DrawStats& s) {
    d.vertices += s.vertices;
    d.drawcalls += s.drawcalls;
    d.drawcalls_indirect += s.drawcalls_indirect;
}

// Per command list / queue. The immediate D3D11 context is both, so it gets queue state.
struct __declspec(uuid("5b2f7a0e-3c1d-4e8a-9f61-0d7c2b4a8e15")) CmdState {
    explicit CmdState(bool queue) : is_queue(queue) {}
    const bool is_queue;
    resource current_ds{0};
    std::unordered_map<uint64_t, DsFrameStats> stats;

    void merge(const CmdState& src) {
        current_ds = src.current_ds;
        for (const auto& [h, s] : src.stats) {
            DsFrameStats& d = stats[h];
            add(d.total, s.total);
            if (s.cleared)
                d.segment = s.segment;
            else
                add(d.segment, s.segment);
            d.clears += s.clears;
            d.cleared |= s.cleared;
            d.reversed_clear |= s.reversed_clear;
        }
    }
};

struct __declspec(uuid("a3e4d6c1-8b27-4f0e-b5a9-6e1c3d2f7b40")) DeviceData {
    std::vector<command_queue*> queues;
    std::unordered_map<uint64_t, resource_desc> depth_stencils;  // alive depth-stencil textures
};

// Guards queue state (draws on the immediate context vs. present) and the resource map.
std::shared_mutex g_mutex;
std::atomic<ClearHook> g_clear_hook{nullptr};

// Shared lock only for queue state: other command lists are recorded by a single thread.
std::shared_lock<std::shared_mutex> lock_if_queue(const CmdState& s) {
    std::shared_lock<std::shared_mutex> lock(g_mutex, std::defer_lock);
    if (s.is_queue) lock.lock();
    return lock;
}

bool fits_frame(float w, float h, float frame_w, float frame_h) {
    if (frame_w == 0 || frame_h == 0) return true;
    const float wr = frame_w / w, hr = frame_h / h;
    return std::abs(frame_w / frame_h - w / h) <= 0.1f && wr >= 0.5f && wr <= 1.85f && hr >= 0.5f && hr <= 1.85f;
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

void on_init_resource(device* dev, const resource_desc& desc, const subresource_data*, resource_usage, resource res) {
    if (desc.type != resource_type::texture_2d || (desc.usage & resource_usage::depth_stencil) == 0) return;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::unique_lock lock(g_mutex);
    dd->depth_stencils[res.handle] = desc;
}
void on_destroy_resource(device* dev, resource res) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;  // destroy_device can come before the last resources go
    const std::unique_lock lock(g_mutex);
    dd->depth_stencils.erase(res.handle);
}

void on_bind_depth_stencil(command_list* cmd, uint32_t, const resource_view*, resource_view dsv) {
    auto& s = *cmd->get_private_data<CmdState>();
    s.current_ds = dsv != 0 ? cmd->get_device()->get_resource_from_view(dsv) : resource{0};
}

bool on_draw(command_list* cmd, uint32_t vertices, uint32_t instances, uint32_t, uint32_t) {
    auto& s = *cmd->get_private_data<CmdState>();
    if (s.current_ds == 0) return false;
    const auto lock = lock_if_queue(s);
    DsFrameStats& st = s.stats[s.current_ds.handle];
    for (DrawStats* d : {&st.total, &st.segment}) {
        d->vertices += vertices * instances;
        d->drawcalls += 1;
    }
    return false;
}
bool on_draw_indexed(command_list* cmd, uint32_t indices, uint32_t instances, uint32_t, int32_t, uint32_t) {
    return on_draw(cmd, indices, instances, 0, 0);
}
bool on_draw_indirect(command_list* cmd, indirect_command type, resource, uint64_t, uint32_t count, uint32_t) {
    if (type == indirect_command::dispatch) return false;
    auto& s = *cmd->get_private_data<CmdState>();
    if (s.current_ds == 0) return false;
    const auto lock = lock_if_queue(s);
    DsFrameStats& st = s.stats[s.current_ds.handle];
    for (DrawStats* d : {&st.total, &st.segment}) {
        d->drawcalls += count;
        d->drawcalls_indirect += count;
    }
    return false;
}

bool on_clear_depth_stencil(command_list* cmd, resource_view dsv, const float* depth, const uint8_t*, uint32_t,
                            const rect*) {
    if (dsv == 0 || depth == nullptr) return false;
    auto& s = *cmd->get_private_data<CmdState>();
    const resource ds = cmd->get_device()->get_resource_from_view(dsv);
    DrawStats segment;
    uint32_t pass = 0;
    {
        const auto lock = lock_if_queue(s);
        DsFrameStats& st = s.stats[ds.handle];
        if (*depth != 1.0f) st.reversed_clear = true;
        segment = st.segment;
        pass = st.clears++;
        st.segment = {};
        st.cleared = true;
    }
    // Outside the lock: the hook captures, which takes the addon's own lock.
    if (const ClearHook hook = g_clear_hook.load(); hook != nullptr && segment.drawcalls > 0)
        hook(cmd, ds, segment, pass);
    return false;
}

void on_reset_command_list(command_list* cmd) {
    auto& s = *cmd->get_private_data<CmdState>();
    s.current_ds = {0};
    s.stats.clear();
}
void on_execute_command_list(command_queue* q, command_list* cmd) {
    if (cmd == q->get_immediate_command_list()) return;  // just the immediate context flushing
    const std::unique_lock lock(g_mutex);
    q->get_private_data<CmdState>()->merge(*cmd->get_private_data<CmdState>());
}

}  // namespace

void init_device(device* dev) {
    if (dev->get_private_data<DeviceData>() == nullptr) dev->create_private_data<DeviceData>();
}

void set_clear_hook(ClearHook hook) { g_clear_hook = hook; }

std::vector<Candidate> end_frame(device* dev, uint32_t frame_w, uint32_t frame_h) {
    std::vector<Candidate> out;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return out;

    const std::unique_lock lock(g_mutex);
    CmdState frame(true);
    for (command_queue* q : dd->queues) {
        auto& s = *q->get_private_data<CmdState>();
        frame.merge(s);
        s.stats.clear();
    }
    for (const auto& [h, s] : frame.stats) {
        const auto it = dd->depth_stencils.find(h);
        if (it == dd->depth_stencils.end()) continue;  // destroyed since
        if (s.total.drawcalls == 0 || (s.total.vertices <= 3 && s.total.drawcalls_indirect == 0)) continue;
        Candidate c;
        c.resource = {h};
        c.desc = it->second;
        c.stats = s.total;
        c.last_segment = s.segment;
        c.last_pass = s.clears;
        c.reversed_clear = s.reversed_clear;
        c.fits_frame = c.desc.texture.samples <= 1 &&
                       fits_frame(float(c.desc.texture.width), float(c.desc.texture.height), float(frame_w),
                                  float(frame_h));
        out.push_back(c);
    }
    std::sort(out.begin(), out.end(), [](const Candidate& a, const Candidate& b) {
        if (a.fits_frame != b.fits_frame) return a.fits_frame;
        if (a.stats.better_than(b.stats) != b.stats.better_than(a.stats)) return a.stats.better_than(b.stats);
        return a.resource.handle < b.resource.handle;  // stable order for the overlay list
    });
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
    reshade::register_event<addon_event::init_resource>(on_init_resource);
    reshade::register_event<addon_event::destroy_resource>(on_destroy_resource);
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
    reshade::unregister_event<addon_event::init_resource>(on_init_resource);
    reshade::unregister_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::unregister_event<addon_event::bind_render_targets_and_depth_stencil>(on_bind_depth_stencil);
    reshade::unregister_event<addon_event::draw>(on_draw);
    reshade::unregister_event<addon_event::draw_indexed>(on_draw_indexed);
    reshade::unregister_event<addon_event::draw_or_dispatch_indirect>(on_draw_indirect);
    reshade::unregister_event<addon_event::clear_depth_stencil_view>(on_clear_depth_stencil);
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::unregister_event<addon_event::execute_command_list>(on_execute_command_list);
}

}  // namespace lidar::depth
