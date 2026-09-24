#include "cbuffer_d3d11.h"

#include <d3d11.h>
#include <reshade.hpp>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <unordered_map>

using namespace reshade::api;

namespace lidar::cam::d3d11 {
namespace {

constexpr int kStages = 6;  // vertex, hull, domain, geometry, pixel, compute
constexpr uint32_t kSlots = D3D11_COMMONSHADER_CONSTANT_BUFFER_API_SLOT_COUNT;

int stage_index(shader_stage s) {
    switch (s) {
        case shader_stage::vertex: return 0;
        case shader_stage::hull: return 1;
        case shader_stage::domain: return 2;
        case shader_stage::geometry: return 3;
        case shader_stage::pixel: return 4;
        case shader_stage::compute: return 5;
        default: return -1;
    }
}

struct Shadow {
    std::vector<uint8_t> data;  // sized to the buffer
    bool valid = false;         // written at least once (or created with initial data)
};

// Map/Unmap and UpdateSubresource are device-level events and can come from deferred contexts
// on other threads, so the table has its own lock.
struct __declspec(uuid("c71e9a52-6f0b-4d38-a2e4-915b3c8d07f6")) DeviceData {
    std::mutex mutex;
    std::unordered_map<uint64_t, Shadow> buffers;   // alive constant buffers
    std::unordered_map<uint64_t, uint8_t*> mapped;  // between Map and Unmap: where the game writes
};

// Per context. Only touched by the thread recording on that context.
struct __declspec(uuid("2d8b4f13-95a7-4c6e-b0d2-7e3f1a6c5b98")) Bindings {
    buffer_range cb[kStages][kSlots] = {};
    void clear() { std::fill(&cb[0][0], &cb[0][0] + kStages * kSlots, buffer_range{}); }
};

bool is_d3d11(device* dev) { return dev->get_api() == device_api::d3d11; }

void on_init_device(device* dev) {
    if (is_d3d11(dev)) dev->create_private_data<DeviceData>();
}
void on_destroy_device(device* dev) {
    if (is_d3d11(dev)) dev->destroy_private_data<DeviceData>();
}

void on_init_command_list(command_list* cmd) {
    if (is_d3d11(cmd->get_device())) cmd->create_private_data<Bindings>();
}
void on_destroy_command_list(command_list* cmd) {
    if (is_d3d11(cmd->get_device())) cmd->destroy_private_data<Bindings>();
}
// The immediate context is a queue; its command list is the same object.
void on_init_command_queue(command_queue* q) {
    if (is_d3d11(q->get_device())) q->create_private_data<Bindings>();
}
void on_destroy_command_queue(command_queue* q) {
    if (is_d3d11(q->get_device())) q->destroy_private_data<Bindings>();
}

void on_init_resource(device* dev, const resource_desc& desc, const subresource_data* initial, resource_usage,
                      resource res) {
    if (desc.type != resource_type::buffer || (desc.usage & resource_usage::constant_buffer) == 0) return;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    Shadow s;
    s.data.assign(size_t(desc.buffer.size), 0);
    if (initial != nullptr && initial->data != nullptr) {
        std::memcpy(s.data.data(), initial->data, s.data.size());
        s.valid = true;
    }
    const std::lock_guard lock(dd->mutex);
    dd->buffers[res.handle] = std::move(s);
}
void on_destroy_resource(device* dev, resource res) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;  // not D3D11, or destroy_device came first
    const std::lock_guard lock(dd->mutex);
    dd->buffers.erase(res.handle);
    dd->mapped.erase(res.handle);
}

// D3D11 always maps the whole buffer (offset 0, size -1).
void on_map_buffer_region(device* dev, resource res, uint64_t offset, uint64_t, map_access access, void** data) {
    if (access == map_access::read_only || data == nullptr || *data == nullptr) return;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    if (dd->buffers.contains(res.handle)) dd->mapped[res.handle] = static_cast<uint8_t*>(*data) + offset;
}
// Fires before the real Unmap, so the game's writes are complete and the pointer still valid.
// With WRITE_DISCARD the new memory holds exactly what the GPU will see, so copy all of it.
void on_unmap_buffer_region(device* dev, resource res) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    const auto m = dd->mapped.find(res.handle);
    if (m == dd->mapped.end()) return;
    Shadow& s = dd->buffers[res.handle];
    std::memcpy(s.data.data(), m->second, s.data.size());
    s.valid = true;
    dd->mapped.erase(m);
}

void update_shadow(device* dev, const void* data, resource res, uint64_t offset, uint64_t size) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr || data == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    const auto it = dd->buffers.find(res.handle);
    if (it == dd->buffers.end()) return;
    Shadow& s = it->second;
    if (offset >= s.data.size()) return;
    const size_t n = size_t(std::min<uint64_t>(size, s.data.size() - offset));  // size -1 = to the end
    std::memcpy(s.data.data() + offset, data, n);
    s.valid = true;
}
bool on_update_buffer_region(device* dev, const void* data, resource res, uint64_t offset, uint64_t size) {
    update_shadow(dev, data, res, offset, size);
    return false;
}
// UpdateSubresource on a deferred context.
bool on_update_buffer_region_command(command_list* cmd, const void* data, resource res, uint64_t offset,
                                     uint64_t size) {
    update_shadow(cmd->get_device(), data, res, offset, size);
    return false;
}

// xSSetConstantBuffers[1]: binding = start slot, one buffer_range per slot.
void on_push_descriptors(command_list* cmd, shader_stage stages, pipeline_layout, uint32_t,
                         const descriptor_table_update& update) {
    if (update.type != descriptor_type::constant_buffer) return;
    Bindings* b = cmd->get_private_data<Bindings>();
    if (b == nullptr) return;
    const auto* ranges = static_cast<const buffer_range*>(update.descriptors);
    for (uint32_t bit = 1; bit <= uint32_t(shader_stage::compute); bit <<= 1) {
        if ((uint32_t(stages) & bit) == 0) continue;
        const int si = stage_index(shader_stage(bit));
        if (si < 0) continue;
        for (uint32_t i = 0; i < update.count; ++i) {
            const uint32_t slot = update.binding + update.array_offset + i;
            if (slot < kSlots) b->cb[si][slot] = ranges ? ranges[i] : buffer_range{};
        }
    }
}

void on_reset_command_list(command_list* cmd) {
    if (Bindings* b = cmd->get_private_data<Bindings>()) b->clear();
}

class Source final : public CbufferSource {
public:
    explicit Source(device* dev) : dev_(dev) {}

    bool read_at_draw(command_list* cmd, const CbufferKey& key, uint32_t offset, uint32_t size,
                      CbufferRead& out) override {
        const Bindings* b = cmd->get_private_data<Bindings>();
        const int si = stage_index(key.stage);
        if (b == nullptr || si < 0 || key.slot >= kSlots) return false;
        const buffer_range& r = b->cb[si][key.slot];
        if (r.buffer == 0) return false;

        DeviceData* dd = dev_->get_private_data<DeviceData>();
        const std::lock_guard lock(dd->mutex);
        const auto it = dd->buffers.find(r.buffer.handle);
        if (it == dd->buffers.end() || !it->second.valid) return false;
        const std::vector<uint8_t>& data = it->second.data;
        if (key.size != 0 && data.size() != key.size) return false;
        // D3D11.1 can bind a sub-range (first constant / constant count).
        if (r.offset >= data.size()) return false;
        const uint64_t avail = std::min<uint64_t>(r.size, data.size() - r.offset);
        if (uint64_t(offset) + size > avail) return false;

        out.buffer = r.buffer.handle;
        out.offset = r.offset + offset;
        out.bytes.assign(data.begin() + ptrdiff_t(out.offset), data.begin() + ptrdiff_t(out.offset + size));
        out.ready = true;
        return true;
    }

    size_t tracked_buffers() const override {
        DeviceData* dd = dev_->get_private_data<DeviceData>();
        const std::lock_guard lock(dd->mutex);
        return dd->buffers.size();
    }

private:
    device* dev_;
};

}  // namespace

std::unique_ptr<CbufferSource> create(device* dev) {
    if (!is_d3d11(dev) || dev->get_private_data<DeviceData>() == nullptr) return nullptr;
    return std::make_unique<Source>(dev);
}

void register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(on_init_device);
    reshade::register_event<addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<addon_event::init_command_list>(on_init_command_list);
    reshade::register_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::register_event<addon_event::init_command_queue>(on_init_command_queue);
    reshade::register_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
    reshade::register_event<addon_event::init_resource>(on_init_resource);
    reshade::register_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::register_event<addon_event::map_buffer_region>(on_map_buffer_region);
    reshade::register_event<addon_event::unmap_buffer_region>(on_unmap_buffer_region);
    reshade::register_event<addon_event::update_buffer_region>(on_update_buffer_region);
    reshade::register_event<addon_event::update_buffer_region_command>(on_update_buffer_region_command);
    reshade::register_event<addon_event::push_descriptors>(on_push_descriptors);
    reshade::register_event<addon_event::reset_command_list>(on_reset_command_list);
}

void unregister_events() {
    using reshade::addon_event;
    reshade::unregister_event<addon_event::init_device>(on_init_device);
    reshade::unregister_event<addon_event::destroy_device>(on_destroy_device);
    reshade::unregister_event<addon_event::init_command_list>(on_init_command_list);
    reshade::unregister_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::unregister_event<addon_event::init_command_queue>(on_init_command_queue);
    reshade::unregister_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
    reshade::unregister_event<addon_event::init_resource>(on_init_resource);
    reshade::unregister_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::unregister_event<addon_event::map_buffer_region>(on_map_buffer_region);
    reshade::unregister_event<addon_event::unmap_buffer_region>(on_unmap_buffer_region);
    reshade::unregister_event<addon_event::update_buffer_region>(on_update_buffer_region);
    reshade::unregister_event<addon_event::update_buffer_region_command>(on_update_buffer_region_command);
    reshade::unregister_event<addon_event::push_descriptors>(on_push_descriptors);
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
}

}  // namespace lidar::cam::d3d11
