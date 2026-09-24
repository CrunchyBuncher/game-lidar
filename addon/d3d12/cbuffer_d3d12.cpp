#include "cbuffer_d3d12.h"

#include <reshade.hpp>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>

#include "root_layout.h"

using namespace reshade::api;

namespace lidar::cam::d3d12 {
namespace {

struct UploadBuffer {
    uint64_t size = 0;
    uint8_t* mapped = nullptr;  // the game's Map pointer (buffers map whole, from offset 0)
    uint32_t map_count = 0;     // D3D12 Map/Unmap nest
};

struct __declspec(uuid("4b0e7d93-1c6a-4f25-8e3d-a92f5b17c604")) DeviceData {
    // Resources and descriptors change on any thread, through device calls.
    std::mutex mutex;
    std::unordered_map<uint64_t, UploadBuffer> buffers;
    // CBV descriptors per heap (the native heap, which is what get_descriptor_heap_offset returns
    // for CPU and GPU handles alike), indexed by descriptor. buffer == 0: not a CBV.
    std::unordered_map<uint64_t, std::vector<buffer_range>> descriptors;

    // Root signatures: written at creation, read at every latching draw.
    std::shared_mutex layout_mutex;
    std::unordered_map<uint64_t, RootLayout> layouts;
};

// What a graphics root parameter is bound to on a command list.
struct Bound {
    buffer_range cbv{};   // root CBV
    uint64_t table = 0;   // descriptor table (GPU descriptor handle)
};

// Per command list. Only touched by the thread recording it.
struct __declspec(uuid("9e61c2a8-37d4-4b0f-a5e2-6c8d19f3b7e0")) Bindings {
    pipeline_layout layout{0};  // graphics root signature
    std::vector<Bound> params;

    // find_cbv() result for (layout, key), since the same draws repeat all frame.
    pipeline_layout cached_layout{0};
    CbufferKey cached_key;
    bool cached_found = false;
    CbvLocation cached_loc;

    void clear() {
        layout = {0};
        params.clear();
    }
};

// Set while the addon maps a buffer itself, so its own map events are ignored.
thread_local bool t_self_map = false;

bool is_d3d12(device* dev) { return dev->get_api() == device_api::d3d12; }
bool graphics(shader_stage stages) { return (stages & shader_stage::all_graphics) != 0; }
uint64_t align256(uint64_t v) { return (v + 255) & ~uint64_t(255); }

void on_destroy_device(device* dev) {
    if (is_d3d12(dev)) dev->destroy_private_data<DeviceData>();
}
void on_init_command_list(command_list* cmd) {
    if (is_d3d12(cmd->get_device())) cmd->create_private_data<Bindings>();
}
void on_destroy_command_list(command_list* cmd) {
    if (is_d3d12(cmd->get_device())) cmd->destroy_private_data<Bindings>();
}

// ---- Upload buffers and their Map pointers ----

void on_init_resource(device* dev, const resource_desc& desc, const subresource_data*, resource_usage, resource res) {
    if (desc.type != resource_type::buffer || desc.heap != memory_heap::upload) return;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    dd->buffers[res.handle] = {desc.buffer.size};
}
void on_destroy_resource(device* dev, resource res) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;  // not D3D12, or destroy_device came first
    const std::lock_guard lock(dd->mutex);
    dd->buffers.erase(res.handle);
}

void on_map_buffer_region(device* dev, resource res, uint64_t, uint64_t, map_access, void** data) {
    if (t_self_map) return;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    const auto it = dd->buffers.find(res.handle);
    if (it == dd->buffers.end()) return;
    ++it->second.map_count;
    if (data != nullptr && *data != nullptr) it->second.mapped = static_cast<uint8_t*>(*data);
}
void on_unmap_buffer_region(device* dev, resource res) {
    if (t_self_map) return;
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    const auto it = dd->buffers.find(res.handle);
    if (it == dd->buffers.end() || it->second.map_count == 0) return;
    if (--it->second.map_count == 0) it->second.mapped = nullptr;
}

// ---- Root signatures ----

void on_init_pipeline_layout(device* dev, uint32_t count, const pipeline_layout_param* params, pipeline_layout layout) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    RootLayout converted = convert_layout(count, params);
    const std::unique_lock lock(dd->layout_mutex);
    dd->layouts[layout.handle] = std::move(converted);
}
void on_destroy_pipeline_layout(device* dev, pipeline_layout layout) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::unique_lock lock(dd->layout_mutex);
    dd->layouts.erase(layout.handle);
}

// ---- Descriptors ----

// Slot `index` of `heap`, growing the heap's table as needed. Caller holds dd->mutex.
buffer_range& slot(DeviceData& dd, uint64_t heap, uint32_t index) {
    std::vector<buffer_range>& v = dd.descriptors[heap];
    if (index >= v.size()) v.resize(size_t(index) + 1);
    return v[index];
}

bool on_update_descriptor_tables(device* dev, uint32_t count, const descriptor_table_update* updates) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return false;
    for (uint32_t u = 0; u < count; ++u) {
        const descriptor_table_update& up = updates[u];
        descriptor_heap heap{0};
        uint32_t base = 0;
        dev->get_descriptor_heap_offset(up.table, up.binding, 0, &heap, &base);
        if (heap == 0) continue;
        const bool cbv = up.type == descriptor_type::constant_buffer && up.descriptors != nullptr;
        const std::lock_guard lock(dd->mutex);
        for (uint32_t i = 0; i < up.count; ++i)
            slot(*dd, heap.handle, base + i) = cbv ? static_cast<const buffer_range*>(up.descriptors)[i] : buffer_range{};
    }
    return false;
}

bool on_copy_descriptor_tables(device* dev, uint32_t count, const descriptor_table_copy* copies) {
    DeviceData* dd = dev->get_private_data<DeviceData>();
    if (dd == nullptr) return false;
    for (uint32_t c = 0; c < count; ++c) {
        const descriptor_table_copy& cp = copies[c];
        descriptor_heap src_heap{0}, dst_heap{0};
        uint32_t src = 0, dst = 0;
        dev->get_descriptor_heap_offset(cp.source_table, cp.source_binding, 0, &src_heap, &src);
        dev->get_descriptor_heap_offset(cp.dest_table, cp.dest_binding, 0, &dst_heap, &dst);
        if (dst_heap == 0) continue;
        const std::lock_guard lock(dd->mutex);
        for (uint32_t i = 0; i < cp.count; ++i) {
            buffer_range value{};
            if (src_heap != 0) {
                const auto it = dd->descriptors.find(src_heap.handle);
                if (it != dd->descriptors.end() && src + i < it->second.size()) value = it->second[src + i];
            }
            slot(*dd, dst_heap.handle, dst + i) = value;
        }
    }
    return false;
}

// ---- Bindings per command list ----

// Root signature changes (count == 0; ClearState passes a null layout) and descriptor tables.
void on_bind_descriptor_tables(command_list* cmd, shader_stage stages, pipeline_layout layout, uint32_t first,
                               uint32_t count, const descriptor_table* tables, uint32_t, const uint32_t*) {
    if (!graphics(stages)) return;
    Bindings* b = cmd->get_private_data<Bindings>();
    if (b == nullptr) return;
    if (count == 0) {
        // Setting a different root signature invalidates every binding; the same one keeps them.
        if (layout != b->layout) {
            b->clear();
            b->layout = layout;
        }
        return;
    }
    if (b->params.size() < size_t(first) + count) b->params.resize(size_t(first) + count);
    for (uint32_t i = 0; i < count; ++i) b->params[first + i].table = tables[i].handle;
}

// Root CBVs: ReShade has already resolved the GPU address to buffer + offset.
void on_push_descriptors(command_list* cmd, shader_stage stages, pipeline_layout, uint32_t param,
                         const descriptor_table_update& update) {
    if (!graphics(stages) || update.type != descriptor_type::constant_buffer || update.count == 0) return;
    Bindings* b = cmd->get_private_data<Bindings>();
    if (b == nullptr) return;
    if (b->params.size() <= param) b->params.resize(size_t(param) + 1);
    b->params[param].cbv = update.descriptors ? *static_cast<const buffer_range*>(update.descriptors) : buffer_range{};
}

void on_reset_command_list(command_list* cmd) {
    if (Bindings* b = cmd->get_private_data<Bindings>()) b->clear();
}

bool same_key(const CbufferKey& a, const CbufferKey& b) {
    return a.stage == b.stage && a.slot == b.slot && a.space == b.space;
}

class Source final : public CbufferSource {
public:
    explicit Source(device* dev) : dev_(dev), dd_(dev->get_private_data<DeviceData>()) {}

    bool read_at_draw(command_list* cmd, const CbufferKey& key, uint32_t offset, uint32_t size,
                      CbufferRead& out) override {
        Bindings* b = cmd->get_private_data<Bindings>();
        if (b == nullptr || b->layout == 0) return false;
        if (b->cached_layout != b->layout || !same_key(b->cached_key, key)) {
            const std::shared_lock lock(dd_->layout_mutex);
            const auto it = dd_->layouts.find(b->layout.handle);
            b->cached_found = it != dd_->layouts.end() && find_cbv(it->second, key, b->cached_loc);
            b->cached_layout = b->layout;
            b->cached_key = key;
        }
        if (!b->cached_found) return false;
        const CbvLocation& loc = b->cached_loc;
        if (loc.param >= b->params.size()) return false;
        const Bound& bound = b->params[loc.param];

        out.ready = false;
        out.bytes.clear();
        out.deferred = {size, key.size, 0, 0};
        if (!loc.table) {
            if (bound.cbv.buffer == 0) return false;
            out.buffer = bound.cbv.buffer.handle;
            out.offset = bound.cbv.offset + offset;
            return true;
        }
        if (bound.table == 0) return false;
        descriptor_heap heap{0};
        uint32_t index = 0;
        dev_->get_descriptor_heap_offset(descriptor_table{bound.table}, loc.table_offset, 0, &heap, &index);
        if (heap == 0) return false;
        out.buffer = 0;
        out.offset = offset;
        out.deferred.descriptor_heap = heap.handle;
        out.deferred.descriptor = index;
        return true;
    }

    bool resolve(CbufferRead& read) override {
        if (read.ready) return true;
        const uint32_t size = read.deferred.size;
        resource buffer{read.buffer};
        uint64_t offset = read.offset;
        read.bytes.resize(size);
        {
            const std::lock_guard lock(dd_->mutex);
            const UploadBuffer* ub = locate(read.deferred, buffer, offset);
            if (ub == nullptr) return false;
            if (ub->mapped != nullptr) {
                std::memcpy(read.bytes.data(), ub->mapped + offset, size);
                return finish(read, buffer, offset);
            }
        }
        // Not mapped right now (Map, write, Unmap each frame): map it ourselves, without holding the
        // lock across ReShade's Map hook. Upload heaps can be mapped any number of times.
        void* p = nullptr;
        t_self_map = true;
        const bool mapped = dev_->map_buffer_region(buffer, 0, UINT64_MAX, map_access::read_only, &p);
        if (mapped && p != nullptr) std::memcpy(read.bytes.data(), static_cast<uint8_t*>(p) + offset, size);
        if (mapped) dev_->unmap_buffer_region(buffer);
        t_self_map = false;
        return mapped && p != nullptr && finish(read, buffer, offset);
    }

    size_t tracked_buffers() const override {
        const std::lock_guard lock(dd_->mutex);
        return dd_->buffers.size();
    }

private:
    // The upload buffer the read refers to, with `buffer`/`offset` made absolute for descriptor
    // reads, or nullptr if it's gone or the window doesn't fit. Caller holds dd_->mutex.
    const UploadBuffer* locate(const CbufferRead::Deferred& d, resource& buffer, uint64_t& offset) const {
        if (d.descriptor_heap != 0) {
            const auto it = dd_->descriptors.find(d.descriptor_heap);
            if (it == dd_->descriptors.end() || d.descriptor >= it->second.size()) return nullptr;
            const buffer_range& view = it->second[d.descriptor];
            if (view.buffer == 0 || offset + d.size > view.size) return nullptr;
            if (d.key_size != 0 && align256(view.size) != align256(d.key_size)) return nullptr;
            buffer = view.buffer;
            offset += view.offset;
        }
        const auto it = dd_->buffers.find(buffer.handle);
        if (it == dd_->buffers.end() || offset + d.size > it->second.size) return nullptr;
        return &it->second;
    }

    static bool finish(CbufferRead& read, resource buffer, uint64_t offset) {
        read.buffer = buffer.handle;
        read.offset = offset;
        read.deferred.descriptor_heap = 0;
        read.ready = true;
        return true;
    }

    device* dev_;
    DeviceData* dd_;
};

}  // namespace

void init_device(device* dev) {
    if (is_d3d12(dev) && dev->get_private_data<DeviceData>() == nullptr) dev->create_private_data<DeviceData>();
}

std::unique_ptr<CbufferSource> create(device* dev) {
    if (!is_d3d12(dev) || dev->get_private_data<DeviceData>() == nullptr) return nullptr;
    return std::make_unique<Source>(dev);
}

void register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(init_device);
    reshade::register_event<addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<addon_event::init_command_list>(on_init_command_list);
    reshade::register_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::register_event<addon_event::init_resource>(on_init_resource);
    reshade::register_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::register_event<addon_event::map_buffer_region>(on_map_buffer_region);
    reshade::register_event<addon_event::unmap_buffer_region>(on_unmap_buffer_region);
    reshade::register_event<addon_event::init_pipeline_layout>(on_init_pipeline_layout);
    reshade::register_event<addon_event::destroy_pipeline_layout>(on_destroy_pipeline_layout);
    reshade::register_event<addon_event::update_descriptor_tables>(on_update_descriptor_tables);
    reshade::register_event<addon_event::copy_descriptor_tables>(on_copy_descriptor_tables);
    reshade::register_event<addon_event::bind_descriptor_tables>(on_bind_descriptor_tables);
    reshade::register_event<addon_event::push_descriptors>(on_push_descriptors);
    reshade::register_event<addon_event::reset_command_list>(on_reset_command_list);
}

void unregister_events() {
    using reshade::addon_event;
    reshade::unregister_event<addon_event::init_device>(init_device);
    reshade::unregister_event<addon_event::destroy_device>(on_destroy_device);
    reshade::unregister_event<addon_event::init_command_list>(on_init_command_list);
    reshade::unregister_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::unregister_event<addon_event::init_resource>(on_init_resource);
    reshade::unregister_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::unregister_event<addon_event::map_buffer_region>(on_map_buffer_region);
    reshade::unregister_event<addon_event::unmap_buffer_region>(on_unmap_buffer_region);
    reshade::unregister_event<addon_event::init_pipeline_layout>(on_init_pipeline_layout);
    reshade::unregister_event<addon_event::destroy_pipeline_layout>(on_destroy_pipeline_layout);
    reshade::unregister_event<addon_event::update_descriptor_tables>(on_update_descriptor_tables);
    reshade::unregister_event<addon_event::copy_descriptor_tables>(on_copy_descriptor_tables);
    reshade::unregister_event<addon_event::bind_descriptor_tables>(on_bind_descriptor_tables);
    reshade::unregister_event<addon_event::push_descriptors>(on_push_descriptors);
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
}

}  // namespace lidar::cam::d3d12
