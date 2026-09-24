#include "camera_tracker.h"

#include <mutex>
#include <shared_mutex>
#include <utility>

using namespace reshade::api;

namespace lidar::cam {
namespace {

// Per command list / queue. The immediate D3D11 context is both, so it gets queue state.
struct __declspec(uuid("8e3a51c7-2b94-4f6d-9c08-d4a7e61f3b25")) CmdState {
    explicit CmdState(bool queue) : is_queue(queue) {}
    const bool is_queue;
    resource current_ds{0};
    FrameLatches latches;
    CbufferRead scratch;  // reused for reads, so steady state doesn't allocate
};

struct __declspec(uuid("f04c9d26-7a1e-4b83-8e5f-39c2d7a6b0e1")) DeviceData {
    std::vector<command_queue*> queues;
    CbufferSource* source = nullptr;
    LatchRequest req;
};

// Unique: configure, merges and end_frame. Shared: draws, which only touch their own command
// list's state (recorded by a single thread) and read the configuration.
std::shared_mutex g_mutex;

void merge(CbufferSource* source, bool last, FrameLatches& dst, FrameLatches& src) {
    for (auto& [ds, read] : src) {
        if (!read.ready && (source == nullptr || !source->resolve(read))) continue;
        if (last)
            dst[ds] = std::move(read);
        else
            dst.try_emplace(ds, std::move(read));
    }
    src.clear();
}

void on_init_device(device* dev) { dev->create_private_data<DeviceData>(); }
void on_destroy_device(device* dev) { dev->destroy_private_data<DeviceData>(); }

void on_init_command_list(command_list* cmd) { cmd->create_private_data<CmdState>(false); }
void on_destroy_command_list(command_list* cmd) { cmd->destroy_private_data<CmdState>(); }

void on_init_command_queue(command_queue* q) {
    q->create_private_data<CmdState>(true);
    if ((q->get_type() & command_queue_type::graphics) == 0) return;
    const std::unique_lock lock(g_mutex);
    q->get_device()->get_private_data<DeviceData>()->queues.push_back(q);
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

void latch(command_list* cmd) {
    auto& s = *cmd->get_private_data<CmdState>();
    if (s.current_ds == 0) return;
    const DeviceData* dd = cmd->get_device()->get_private_data<DeviceData>();
    const std::shared_lock lock(g_mutex);
    if (dd->source == nullptr) return;
    if (!dd->req.last && s.latches.contains(s.current_ds.handle)) return;  // first one wins
    if (!dd->source->read_at_draw(cmd, dd->req.key, dd->req.offset, dd->req.size, s.scratch)) return;
    std::swap(s.latches[s.current_ds.handle], s.scratch);
}

bool on_draw(command_list* cmd, uint32_t, uint32_t, uint32_t, uint32_t) {
    latch(cmd);
    return false;
}
bool on_draw_indexed(command_list* cmd, uint32_t, uint32_t, uint32_t, int32_t, uint32_t) {
    latch(cmd);
    return false;
}
bool on_draw_indirect(command_list* cmd, indirect_command type, resource, uint64_t, uint32_t, uint32_t) {
    if (type != indirect_command::dispatch) latch(cmd);
    return false;
}

void on_reset_command_list(command_list* cmd) {
    auto& s = *cmd->get_private_data<CmdState>();
    s.current_ds = {0};
    s.latches.clear();
}
void on_execute_command_list(command_queue* q, command_list* cmd) {
    if (cmd == q->get_immediate_command_list()) return;  // just the immediate context flushing
    const std::unique_lock lock(g_mutex);
    const DeviceData* dd = q->get_device()->get_private_data<DeviceData>();
    auto& dst = *q->get_private_data<CmdState>();
    auto& src = *cmd->get_private_data<CmdState>();
    dst.current_ds = src.current_ds;
    merge(dd->source, dd->req.last, dst.latches, src.latches);
}

}  // namespace

void configure(device* dev, CbufferSource* source, const LatchRequest* req) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::unique_lock lock(g_mutex);
    dd->source = req != nullptr ? source : nullptr;
    if (req != nullptr) dd->req = *req;
    for (command_queue* q : dd->queues) q->get_private_data<CmdState>()->latches.clear();
}

FrameLatches end_frame(device* dev) {
    FrameLatches out;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return out;
    const std::unique_lock lock(g_mutex);
    for (command_queue* q : dd->queues) merge(dd->source, dd->req.last, out, q->get_private_data<CmdState>()->latches);
    return out;
}

void register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(on_init_device);
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
    reshade::unregister_event<addon_event::init_device>(on_init_device);
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
