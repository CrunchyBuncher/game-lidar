#include "capture_d3d12.h"

#include <d3dcompiler.h>
#include <wrl/client.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <shared_mutex>
#include <unordered_map>

#include "downsample_hlsl.h"

using namespace reshade::api;

namespace lidar {
namespace {

// ---- Depth-stencil state tracking ----

struct __declspec(uuid("d2a85f41-6e3b-4c97-8f10-b54e2c9a7d36")) StateData {
    std::shared_mutex mutex;
    // Every alive depth-stencil texture, with its state after the last submitted command list.
    std::unordered_map<uint64_t, resource_usage> states;
};

// Per command list: the depth-stencils it transitions, and their state at its end.
struct __declspec(uuid("61f7c3b0-9a2e-4d58-b6e4-0c83d5f1a92b")) CmdStates {
    std::unordered_map<uint64_t, resource_usage> changes;
};

bool is_d3d12(device* dev) { return dev->get_api() == device_api::d3d12; }

void on_destroy_device(device* dev) {
    if (is_d3d12(dev)) dev->destroy_private_data<StateData>();
}
void on_init_command_list(command_list* cmd) {
    if (is_d3d12(cmd->get_device())) cmd->create_private_data<CmdStates>();
}
void on_destroy_command_list(command_list* cmd) {
    if (is_d3d12(cmd->get_device())) cmd->destroy_private_data<CmdStates>();
}

void on_init_resource(device* dev, const resource_desc& desc, const subresource_data*, resource_usage initial,
                      resource res) {
    if (desc.type != resource_type::texture_2d || (desc.usage & resource_usage::depth_stencil) == 0) return;
    StateData* sd = dev->get_private_data<StateData>();
    if (sd == nullptr) return;
    const std::unique_lock lock(sd->mutex);
    sd->states[res.handle] = initial;
}
void on_destroy_resource(device* dev, resource res) {
    StateData* sd = dev->get_private_data<StateData>();
    if (sd == nullptr) return;
    const std::unique_lock lock(sd->mutex);
    sd->states.erase(res.handle);
}

void on_barrier(command_list* cmd, uint32_t count, const resource* resources, const resource_usage*,
                const resource_usage* new_states) {
    CmdStates* cs = cmd->get_private_data<CmdStates>();
    if (cs == nullptr) return;
    StateData* sd = cmd->get_device()->get_private_data<StateData>();
    const std::shared_lock lock(sd->mutex);
    for (uint32_t i = 0; i < count; ++i)
        if (resources[i] != 0 && sd->states.contains(resources[i].handle)) cs->changes[resources[i].handle] = new_states[i];
}
void on_reset_command_list(command_list* cmd) {
    if (CmdStates* cs = cmd->get_private_data<CmdStates>()) cs->changes.clear();
}
void on_execute_command_list(command_queue* q, command_list* cmd) {
    const CmdStates* cs = cmd->get_private_data<CmdStates>();
    if (cs == nullptr || cs->changes.empty()) return;
    StateData* sd = q->get_device()->get_private_data<StateData>();
    const std::unique_lock lock(sd->mutex);
    for (const auto& [res, state] : cs->changes)
        if (const auto it = sd->states.find(res); it != sd->states.end()) it->second = state;
}

// The state `depth` is in once everything submitted so far has run. Unknown: assume the usual.
resource_usage depth_state(device* dev, resource depth) {
    StateData* sd = dev->get_private_data<StateData>();
    if (sd == nullptr) return resource_usage::depth_stencil_write;
    const std::shared_lock lock(sd->mutex);
    const auto it = sd->states.find(depth.handle);
    return it != sd->states.end() && it->second != resource_usage::undefined ? it->second
                                                                             : resource_usage::depth_stencil_write;
}

// ---- Capture ----

// Depth-stencil formats (typed or typeless) -> typeless copy format + depth-only SRV format.
bool depth_formats(format f, format& typeless, format& srv) {
    switch (f) {
        case format::d24_unorm_s8_uint:
        case format::r24_g8_typeless:
        case format::r24_unorm_x8_uint:
            typeless = format::r24_g8_typeless, srv = format::r24_unorm_x8_uint;
            return true;
        case format::d32_float:
        case format::r32_typeless:
        case format::r32_float:
            typeless = format::r32_typeless, srv = format::r32_float;
            return true;
        case format::d16_unorm:
        case format::r16_typeless:
        case format::r16_unorm:
            typeless = format::r16_typeless, srv = format::r16_unorm;
            return true;
        case format::d32_float_s8_uint:
        case format::r32_g8_typeless:
        case format::r32_float_x8_uint:
            typeless = format::r32_g8_typeless, srv = format::r32_float_x8_uint;
            return true;
        default:
            return false;
    }
}

struct Dims {
    uint32_t src_w, src_h, dst_w, dst_h;
};

}  // namespace

void D3D12Capture::init_device(device* dev) {
    if (is_d3d12(dev) && dev->get_private_data<StateData>() == nullptr) dev->create_private_data<StateData>();
}

void D3D12Capture::register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(init_device);
    reshade::register_event<addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<addon_event::init_command_list>(on_init_command_list);
    reshade::register_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::register_event<addon_event::init_resource>(on_init_resource);
    reshade::register_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::register_event<addon_event::barrier>(on_barrier);
    reshade::register_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::register_event<addon_event::execute_command_list>(on_execute_command_list);
}

void D3D12Capture::unregister_events() {
    using reshade::addon_event;
    reshade::unregister_event<addon_event::init_device>(init_device);
    reshade::unregister_event<addon_event::destroy_device>(on_destroy_device);
    reshade::unregister_event<addon_event::init_command_list>(on_init_command_list);
    reshade::unregister_event<addon_event::destroy_command_list>(on_destroy_command_list);
    reshade::unregister_event<addon_event::init_resource>(on_init_resource);
    reshade::unregister_event<addon_event::destroy_resource>(on_destroy_resource);
    reshade::unregister_event<addon_event::barrier>(on_barrier);
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::unregister_event<addon_event::execute_command_list>(on_execute_command_list);
}

bool D3D12Capture::init(device* dev) {
    dev_ = dev;
    Microsoft::WRL::ComPtr<ID3DBlob> code, errors;
    if (FAILED(D3DCompile(kDownsampleHlsl, std::strlen(kDownsampleHlsl), "lidar_downsample", nullptr, nullptr,
                          "cs_main", "cs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors))) {
        error_ = errors ? static_cast<const char*>(errors->GetBufferPointer()) : "D3DCompile failed";
        return false;
    }

    descriptor_range srv{};
    srv.count = 1;
    srv.visibility = shader_stage::compute;
    srv.type = descriptor_type::texture_shader_resource_view;
    descriptor_range uav = srv;
    uav.type = descriptor_type::texture_unordered_access_view;
    const pipeline_layout_param params[3] = {
        constant_range{0, 0, 0, sizeof(Dims) / 4, shader_stage::compute},  // b0
        srv,                                                                // t0
        uav,                                                                // u0
    };
    if (!dev->create_pipeline_layout(3, params, &layout_)) {
        error_ = "create_pipeline_layout failed";
        return false;
    }
    shader_desc cs{code->GetBufferPointer(), code->GetBufferSize()};
    const pipeline_subobject sub{pipeline_subobject_type::compute_shader, 1, &cs};
    if (!dev->create_pipeline(layout_, 1, &sub, &pipeline_)) {
        error_ = "create_pipeline failed";
        return false;
    }
    if (!dev->create_fence(0, fence_flags::none, &fence_)) {
        error_ = "create_fence failed";
        return false;
    }
    return true;
}

void D3D12Capture::wait_idle() {
    if (fence_ != 0 && fence_value_ != 0) dev_->wait(fence_, fence_value_);
}

void D3D12Capture::destroy_targets() {
    pending_.clear();
    for (int i = 0; i < kReadback; ++i) {
        if (readback_data_[i]) dev_->unmap_buffer_region(readback_[i]);
        if (readback_[i] != 0) dev_->destroy_resource(readback_[i]);
        readback_[i] = {0};
        readback_data_[i] = nullptr;
    }
    if (cap_uav_ != 0) dev_->destroy_resource_view(cap_uav_);
    if (cap_ != 0) dev_->destroy_resource(cap_);
    if (copy_srv_ != 0) dev_->destroy_resource_view(copy_srv_);
    if (copy_ != 0) dev_->destroy_resource(copy_);
    cap_uav_ = {0}, cap_ = {0}, copy_srv_ = {0}, copy_ = {0};
    src_desc_ = {};
    cap_w_ = cap_h_ = 0;
}

void D3D12Capture::release() {
    if (dev_ == nullptr) return;
    wait_idle();
    destroy_targets();
    if (pipeline_ != 0) dev_->destroy_pipeline(pipeline_);
    if (layout_ != 0) dev_->destroy_pipeline_layout(layout_);
    if (fence_ != 0) dev_->destroy_fence(fence_);
    pipeline_ = {0}, layout_ = {0}, fence_ = {0};
    dev_ = nullptr;
}

bool D3D12Capture::ensure_targets(const resource_desc& src, uint32_t capture_width) {
    const uint32_t cap_w = std::clamp(capture_width, 16u, std::min(kMaxWidth, src.texture.width));
    const uint32_t cap_h =
        std::min(uint32_t(std::lround(double(cap_w) * src.texture.height / src.texture.width)), kMaxHeight);
    if (copy_ != 0 && src.texture.width == src_desc_.texture.width && src.texture.height == src_desc_.texture.height &&
        src.texture.format == src_desc_.texture.format && cap_w == cap_w_ && cap_h == cap_h_)
        return true;

    // Anything in flight refers to the old targets.
    wait_idle();
    destroy_targets();

    format typeless, srv_format;
    if (!depth_formats(src.texture.format, typeless, srv_format)) {
        error_ = "unsupported depth format " + std::to_string(int(src.texture.format));
        return false;
    }
    const resource_desc copy_desc(src.texture.width, src.texture.height, 1, 1, typeless, 1, memory_heap::default_,
                                  resource_usage::copy_dest | resource_usage::shader_resource);
    if (!dev_->create_resource(copy_desc, nullptr, resource_usage::copy_dest, &copy_) ||
        !dev_->create_resource_view(copy_, resource_usage::shader_resource,
                                    resource_view_desc(resource_view_type::texture_2d, srv_format, 0, 1, 0, 1),
                                    &copy_srv_)) {
        error_ = "failed to create depth copy";
        return false;
    }
    const resource_desc cap_desc(cap_w, cap_h, 1, 1, format::r32_float, 1, memory_heap::default_,
                                 resource_usage::unordered_access | resource_usage::copy_source);
    if (!dev_->create_resource(cap_desc, nullptr, resource_usage::unordered_access, &cap_) ||
        !dev_->create_resource_view(cap_, resource_usage::unordered_access,
                                    resource_view_desc(resource_view_type::texture_2d, format::r32_float, 0, 1, 0, 1),
                                    &cap_uav_)) {
        error_ = "failed to create capture target";
        return false;
    }
    // copy_texture_to_buffer lays rows out with D3D12's 256-byte pitch alignment.
    row_pitch_ = (cap_w * 4 + 255) & ~255u;
    for (int i = 0; i < kReadback; ++i) {
        void* p = nullptr;
        if (!dev_->create_resource(resource_desc(uint64_t(row_pitch_) * cap_h, memory_heap::readback,
                                                 resource_usage::copy_dest),
                                   nullptr, resource_usage::copy_dest, &readback_[i]) ||
            !dev_->map_buffer_region(readback_[i], 0, UINT64_MAX, map_access::read_only, &p)) {
            error_ = "failed to create readback buffer";
            return false;
        }
        readback_data_[i] = static_cast<const uint8_t*>(p);
    }

    src_desc_ = src;
    cap_w_ = cap_w;
    cap_h_ = cap_h;
    return true;
}

bool D3D12Capture::capture(command_queue* queue, resource depth, resource, uint32_t capture_width,
                           const FrameHeader& header) {
    const resource_desc src = dev_->get_resource_desc(depth);
    if (src.type != resource_type::texture_2d) {
        error_ = "depth buffer is not a 2D texture";
        return false;
    }
    if (src.texture.samples > 1) {
        error_ = "multisampled depth buffers are not supported yet";
        return false;
    }
    if (!ensure_targets(src, capture_width)) return false;
    error_.clear();
    if (pending_.size() >= kReadback) {
        ++skipped_;
        return true;
    }

    command_list* cmd = queue->get_immediate_command_list();
    const resource_usage state = depth_state(dev_, depth);

    // Snapshot subresource 0 (mip 0 of the first slice, depth plane), then hand the depth buffer
    // back in the state the game left it.
    if (state != resource_usage::copy_source) cmd->barrier(depth, state, resource_usage::copy_source);
    cmd->copy_texture_region(depth, 0, nullptr, copy_, 0, nullptr);
    {
        const resource res[2] = {depth, copy_};
        const resource_usage from[2] = {resource_usage::copy_source, resource_usage::copy_dest};
        const resource_usage to[2] = {state, resource_usage::shader_resource_non_pixel};
        if (state != resource_usage::copy_source)
            cmd->barrier(2, res, from, to);
        else
            cmd->barrier(1, res + 1, from + 1, to + 1);
    }

    const Dims dims{src.texture.width, src.texture.height, cap_w_, cap_h_};
    cmd->bind_pipeline(pipeline_stage::compute_shader, pipeline_);
    cmd->push_constants(shader_stage::compute, layout_, 0, 0, sizeof(dims) / 4, &dims);
    cmd->push_descriptors(shader_stage::compute, layout_, 1,
                          descriptor_table_update{{}, 0, 0, 1, descriptor_type::texture_shader_resource_view, &copy_srv_});
    cmd->push_descriptors(shader_stage::compute, layout_, 2,
                          descriptor_table_update{{}, 0, 0, 1, descriptor_type::texture_unordered_access_view, &cap_uav_});
    cmd->dispatch((cap_w_ + 7) / 8, (cap_h_ + 7) / 8, 1);

    const int s = next_slot_;
    next_slot_ = (next_slot_ + 1) % kReadback;
    cmd->barrier(cap_, resource_usage::unordered_access, resource_usage::copy_source);
    cmd->copy_texture_to_buffer(cap_, 0, nullptr, readback_[s], 0);
    {
        const resource res[2] = {cap_, copy_};
        const resource_usage from[2] = {resource_usage::copy_source, resource_usage::shader_resource_non_pixel};
        const resource_usage to[2] = {resource_usage::unordered_access, resource_usage::copy_dest};
        cmd->barrier(2, res, from, to);
    }
    // Submits the immediate command list, then signals.
    queue->signal(fence_, ++fence_value_);

    Pending p{s, fence_value_, header};
    p.header.width = cap_w_;
    p.header.height = cap_h_;
    p.header.src_width = src.texture.width;
    p.header.src_height = src.texture.height;
    p.header.depth_format = uint32_t(DepthFormat::Float32Ndc);
    pending_.push_back(p);
    return true;
}

void D3D12Capture::publish(command_queue*, RingWriter& ring) {
    if (pending_.empty()) return;
    const uint64_t done = dev_->get_completed_fence_value(fence_);
    while (!pending_.empty() && pending_.front().fence <= done) {
        const Pending& p = pending_.front();
        if (ring.is_open()) {
            Slot* slot = ring.begin_frame();
            slot->frame = p.header;
            const uint32_t w = p.header.width, h = p.header.height;
            for (uint32_t y = 0; y < h; ++y)
                std::memcpy(&slot->depth[y * w], readback_data_[p.slot] + size_t(y) * row_pitch_, w * sizeof(float));
            ring.commit();
            ++published_;
        }
        pending_.pop_front();
    }
}

}  // namespace lidar
