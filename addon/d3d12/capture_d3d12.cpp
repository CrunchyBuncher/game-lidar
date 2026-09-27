#include "capture_d3d12.h"

#include <d3dcompiler.h>
#include <wrl/client.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "downsample_hlsl.h"

using namespace reshade::api;

namespace lidar {
namespace {

// ---- Depth-stencil and render target state tracking ----

struct __declspec(uuid("d2a85f41-6e3b-4c97-8f10-b54e2c9a7d36")) StateData {
    std::shared_mutex mutex;
    // Every alive depth-stencil and render target texture (swap chain back buffers included), with its
    // state after the last submitted command list.
    std::unordered_map<uint64_t, resource_usage> states;
};

// Per command list: the tracked textures it transitions, and their state at its end.
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
    if (desc.type != resource_type::texture_2d ||
        (desc.usage & (resource_usage::depth_stencil | resource_usage::render_target)) == 0)
        return;
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

// Back buffers start out (and are presented) in the present state.
void on_init_swapchain(swapchain* sc, bool) {
    StateData* sd = sc->get_device()->get_private_data<StateData>();
    if (sd == nullptr) return;
    const std::unique_lock lock(sd->mutex);
    for (uint32_t i = 0; i < sc->get_back_buffer_count(); ++i) sd->states[sc->get_back_buffer(i).handle] = resource_usage::present;
}
void on_destroy_swapchain(swapchain* sc, bool) {
    StateData* sd = sc->get_device()->get_private_data<StateData>();
    if (sd == nullptr) return;
    const std::unique_lock lock(sd->mutex);
    for (uint32_t i = 0; i < sc->get_back_buffer_count(); ++i) sd->states.erase(sc->get_back_buffer(i).handle);
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

// The state `res` is in once everything submitted so far has run, or undefined if it isn't known.
resource_usage tracked_state(device* dev, resource res) {
    StateData* sd = dev->get_private_data<StateData>();
    if (sd == nullptr) return resource_usage::undefined;
    const std::shared_lock lock(sd->mutex);
    const auto it = sd->states.find(res.handle);
    return it != sd->states.end() ? it->second : resource_usage::undefined;
}

// The depth buffer's state. Unknown: assume the usual.
resource_usage depth_state(device* dev, resource depth) {
    const resource_usage state = tracked_state(dev, depth);
    return state != resource_usage::undefined ? state : resource_usage::depth_stencil_write;
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

// Render target formats (typed or typeless) -> typeless copy format + the format to read it as, and
// whether it's a float format (linear HDR: tonemapped). sRGB ones are read as UNORM: the viewer wants
// the stored (gamma-encoded) values, as displayed. Same list as D3D11's, plus RGBA32F.
bool color_formats(format f, format& typeless, format& srv, bool& hdr) {
    hdr = false;
    switch (f) {
        case format::r8g8b8a8_typeless:
        case format::r8g8b8a8_unorm:
        case format::r8g8b8a8_unorm_srgb:
            typeless = format::r8g8b8a8_typeless, srv = format::r8g8b8a8_unorm;
            return true;
        case format::b8g8r8a8_typeless:
        case format::b8g8r8a8_unorm:
        case format::b8g8r8a8_unorm_srgb:
            typeless = format::b8g8r8a8_typeless, srv = format::b8g8r8a8_unorm;
            return true;
        case format::b8g8r8x8_typeless:
        case format::b8g8r8x8_unorm:
        case format::b8g8r8x8_unorm_srgb:
            typeless = format::b8g8r8x8_typeless, srv = format::b8g8r8x8_unorm;
            return true;
        case format::r10g10b10a2_typeless:
        case format::r10g10b10a2_unorm:
            typeless = format::r10g10b10a2_typeless, srv = format::r10g10b10a2_unorm;
            return true;
        case format::r16g16b16a16_typeless:
        case format::r16g16b16a16_float:
            typeless = format::r16g16b16a16_typeless, srv = format::r16g16b16a16_float, hdr = true;
            return true;
        case format::r32g32b32a32_typeless:
        case format::r32g32b32a32_float:
            typeless = format::r32g32b32a32_typeless, srv = format::r32g32b32a32_float, hdr = true;
            return true;
        case format::r11g11b10_float:
            typeless = srv = format::r11g11b10_float, hdr = true;
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
    reshade::register_event<addon_event::init_swapchain>(on_init_swapchain);
    reshade::register_event<addon_event::destroy_swapchain>(on_destroy_swapchain);
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
    reshade::unregister_event<addon_event::init_swapchain>(on_init_swapchain);
    reshade::unregister_event<addon_event::destroy_swapchain>(on_destroy_swapchain);
    reshade::unregister_event<addon_event::barrier>(on_barrier);
    reshade::unregister_event<addon_event::reset_command_list>(on_reset_command_list);
    reshade::unregister_event<addon_event::execute_command_list>(on_execute_command_list);
}

bool D3D12Capture::init(device* dev) {
    dev_ = dev;
    descriptor_range range{};
    range.count = 1;
    range.visibility = shader_stage::compute;
    auto table = [&](descriptor_type type, uint32_t reg) {
        descriptor_range r = range;
        r.type = type;
        r.dx_register_index = reg;
        return r;
    };
    const pipeline_layout_param params[5] = {
        constant_range{0, 0, 0, sizeof(Dims) / 4, shader_stage::compute},  // b0
        table(descriptor_type::texture_shader_resource_view, 0),            // t0
        table(descriptor_type::texture_unordered_access_view, 0),           // u0
        table(descriptor_type::texture_shader_resource_view, 1),            // t1
        table(descriptor_type::texture_unordered_access_view, 1),           // u1
    };
    if (!dev->create_pipeline_layout(5, params, &layout_)) {
        error_ = "create_pipeline_layout failed";
        return false;
    }
    const std::pair<const char*, pipeline*> shaders[] = {
        {"cs_main", &pipeline_}, {"cs_color", &color_pipeline_}, {"cs_color_hdr", &color_hdr_pipeline_}};
    for (const auto& [entry, out] : shaders) {
        Microsoft::WRL::ComPtr<ID3DBlob> code, errors;
        if (FAILED(D3DCompile(kDownsampleHlsl, std::strlen(kDownsampleHlsl), "lidar_downsample", nullptr, nullptr,
                              entry, "cs_5_0", D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors))) {
            error_ = errors ? static_cast<const char*>(errors->GetBufferPointer()) : "D3DCompile failed";
            return false;
        }
        shader_desc cs{code->GetBufferPointer(), code->GetBufferSize()};
        const pipeline_subobject sub{pipeline_subobject_type::compute_shader, 1, &cs};
        if (!dev->create_pipeline(layout_, 1, &sub, out)) {
            error_ = "create_pipeline failed";
            return false;
        }
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

void D3D12Capture::destroy_color() {
    pending_.clear();  // anything in flight may refer to the old targets
    for (int i = 0; i < kReadback; ++i) {
        if (color_readback_data_[i]) dev_->unmap_buffer_region(color_readback_[i]);
        if (color_readback_[i] != 0) dev_->destroy_resource(color_readback_[i]);
        color_readback_[i] = {0};
        color_readback_data_[i] = nullptr;
    }
    if (color_cap_uav_ != 0) dev_->destroy_resource_view(color_cap_uav_);
    if (color_cap_ != 0) dev_->destroy_resource(color_cap_);
    if (color_copy_srv_ != 0) dev_->destroy_resource_view(color_copy_srv_);
    if (color_copy_ != 0) dev_->destroy_resource(color_copy_);
    color_cap_uav_ = {0}, color_cap_ = {0}, color_copy_srv_ = {0}, color_copy_ = {0};
    color_desc_ = {};
    color_w_ = color_h_ = 0;
}

void D3D12Capture::destroy_targets() {
    destroy_color();
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
    for (pipeline* p : {&pipeline_, &color_pipeline_, &color_hdr_pipeline_}) {
        if (*p != 0) dev_->destroy_pipeline(*p);
        *p = {0};
    }
    if (layout_ != 0) dev_->destroy_pipeline_layout(layout_);
    if (fence_ != 0) dev_->destroy_fence(fence_);
    layout_ = {0}, fence_ = {0};
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

bool D3D12Capture::color_source(resource color, const resource_desc& depth) {
    if (color == 0) {
        color_note_ = "no render target was bound with the depth buffer";
        return false;
    }
    const resource_desc d = dev_->get_resource_desc(color);
    if (d.type != resource_type::texture_2d) {
        color_note_ = "the render target is not a 2D texture";
        return false;
    }
    if (d.texture.width != depth.texture.width || d.texture.height != depth.texture.height || d.texture.samples > 1) {
        color_note_ = "the render target is " + std::to_string(d.texture.width) + "x" +
                      std::to_string(d.texture.height) + (d.texture.samples > 1 ? " MSAA" : "") +
                      ", not the depth buffer's size";
        return false;
    }
    format typeless, srv;
    bool hdr;
    if (!color_formats(d.texture.format, typeless, srv, hdr)) {
        color_note_ = "unsupported render target format " + std::to_string(int(d.texture.format));
        return false;
    }
    if (tracked_state(dev_, color) == resource_usage::undefined) {
        color_note_ = "the render target's state isn't known";
        return false;
    }
    return ensure_color(d);
}

bool D3D12Capture::ensure_color(const resource_desc& src) {
    if (color_copy_ != 0 && src.texture.width == color_desc_.texture.width &&
        src.texture.height == color_desc_.texture.height && src.texture.format == color_desc_.texture.format &&
        color_w_ == cap_w_ && color_h_ == cap_h_)
        return true;

    wait_idle();
    destroy_color();

    format typeless, srv_format;
    color_formats(src.texture.format, typeless, srv_format, color_hdr_);
    const resource_desc copy_desc(src.texture.width, src.texture.height, 1, 1, typeless, 1, memory_heap::default_,
                                  resource_usage::copy_dest | resource_usage::shader_resource);
    if (!dev_->create_resource(copy_desc, nullptr, resource_usage::copy_dest, &color_copy_) ||
        !dev_->create_resource_view(color_copy_, resource_usage::shader_resource,
                                    resource_view_desc(resource_view_type::texture_2d, srv_format, 0, 1, 0, 1),
                                    &color_copy_srv_)) {
        color_note_ = "failed to create the color copy";
        return false;
    }
    // protocol.h: R in the lowest byte.
    const resource_desc cap_desc(cap_w_, cap_h_, 1, 1, format::r8g8b8a8_unorm, 1, memory_heap::default_,
                                 resource_usage::unordered_access | resource_usage::copy_source);
    if (!dev_->create_resource(cap_desc, nullptr, resource_usage::unordered_access, &color_cap_) ||
        !dev_->create_resource_view(color_cap_, resource_usage::unordered_access,
                                    resource_view_desc(resource_view_type::texture_2d, format::r8g8b8a8_unorm, 0, 1, 0, 1),
                                    &color_cap_uav_)) {
        color_note_ = "failed to create the color capture target";
        return false;
    }
    for (int i = 0; i < kReadback; ++i) {
        void* p = nullptr;
        if (!dev_->create_resource(resource_desc(uint64_t(row_pitch_) * cap_h_, memory_heap::readback,
                                                 resource_usage::copy_dest),
                                   nullptr, resource_usage::copy_dest, &color_readback_[i]) ||
            !dev_->map_buffer_region(color_readback_[i], 0, UINT64_MAX, map_access::read_only, &p)) {
            color_note_ = "failed to create the color readback buffers";
            return false;
        }
        color_readback_data_[i] = static_cast<const uint8_t*>(p);
    }
    color_desc_ = src;
    color_w_ = cap_w_;
    color_h_ = cap_h_;
    return true;
}

bool D3D12Capture::capture(command_queue* queue, resource depth, resource color, uint32_t capture_width,
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

    bool has_color = false;
    if (color == 0)
        color_note_ = "turned off";
    else if ((has_color = color_source(color, src)))
        color_note_.clear();

    command_list* cmd = queue->get_immediate_command_list();
    const resource_usage state = depth_state(dev_, depth);
    const resource_usage color_state = has_color ? tracked_state(dev_, color) : resource_usage::undefined;

    // Snapshot subresource 0 (mip 0 of the first slice: the depth plane, the scene color), then hand
    // the game's resources back in the states it left them in.
    std::vector<resource> res;
    std::vector<resource_usage> from, to;
    auto transition = [&](resource r, resource_usage a, resource_usage b) {
        if (a == b) return;
        res.push_back(r);
        from.push_back(a);
        to.push_back(b);
    };
    auto flush = [&]() {
        if (!res.empty()) cmd->barrier(uint32_t(res.size()), res.data(), from.data(), to.data());
        res.clear(), from.clear(), to.clear();
    };
    transition(depth, state, resource_usage::copy_source);
    if (has_color) transition(color, color_state, resource_usage::copy_source);
    flush();
    cmd->copy_texture_region(depth, 0, nullptr, copy_, 0, nullptr);
    if (has_color) cmd->copy_texture_region(color, 0, nullptr, color_copy_, 0, nullptr);
    transition(depth, resource_usage::copy_source, state);
    transition(copy_, resource_usage::copy_dest, resource_usage::shader_resource_non_pixel);
    if (has_color) {
        transition(color, resource_usage::copy_source, color_state);
        transition(color_copy_, resource_usage::copy_dest, resource_usage::shader_resource_non_pixel);
    }
    flush();

    const Dims dims{src.texture.width, src.texture.height, cap_w_, cap_h_};
    cmd->bind_pipeline(pipeline_stage::compute_shader, pipeline_);
    cmd->push_constants(shader_stage::compute, layout_, 0, 0, sizeof(dims) / 4, &dims);
    cmd->push_descriptors(shader_stage::compute, layout_, 1,
                          descriptor_table_update{{}, 0, 0, 1, descriptor_type::texture_shader_resource_view, &copy_srv_});
    cmd->push_descriptors(shader_stage::compute, layout_, 2,
                          descriptor_table_update{{}, 0, 0, 1, descriptor_type::texture_unordered_access_view, &cap_uav_});
    cmd->dispatch((cap_w_ + 7) / 8, (cap_h_ + 7) / 8, 1);
    if (has_color) {
        cmd->bind_pipeline(pipeline_stage::compute_shader, color_hdr_ ? color_hdr_pipeline_ : color_pipeline_);
        cmd->push_constants(shader_stage::compute, layout_, 0, 0, sizeof(dims) / 4, &dims);
        cmd->push_descriptors(
            shader_stage::compute, layout_, 3,
            descriptor_table_update{{}, 0, 0, 1, descriptor_type::texture_shader_resource_view, &color_copy_srv_});
        cmd->push_descriptors(
            shader_stage::compute, layout_, 4,
            descriptor_table_update{{}, 0, 0, 1, descriptor_type::texture_unordered_access_view, &color_cap_uav_});
        cmd->dispatch((cap_w_ + 7) / 8, (cap_h_ + 7) / 8, 1);
    }

    const int s = next_slot_;
    next_slot_ = (next_slot_ + 1) % kReadback;
    transition(cap_, resource_usage::unordered_access, resource_usage::copy_source);
    if (has_color) transition(color_cap_, resource_usage::unordered_access, resource_usage::copy_source);
    flush();
    cmd->copy_texture_to_buffer(cap_, 0, nullptr, readback_[s], 0);
    if (has_color) cmd->copy_texture_to_buffer(color_cap_, 0, nullptr, color_readback_[s], 0);
    transition(cap_, resource_usage::copy_source, resource_usage::unordered_access);
    transition(copy_, resource_usage::shader_resource_non_pixel, resource_usage::copy_dest);
    if (has_color) {
        transition(color_cap_, resource_usage::copy_source, resource_usage::unordered_access);
        transition(color_copy_, resource_usage::shader_resource_non_pixel, resource_usage::copy_dest);
    }
    flush();
    // Submits the immediate command list, then signals.
    queue->signal(fence_, ++fence_value_);

    Pending p{s, fence_value_, header};
    if (has_color) p.header.flags |= kFlagHasColor;
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
            if (p.header.flags & kFlagHasColor) {
                for (uint32_t y = 0; y < h; ++y)
                    std::memcpy(&slot->color[y * w], color_readback_data_[p.slot] + size_t(y) * row_pitch_,
                                w * sizeof(uint32_t));
                crop_color(*slot);
            }
            ring.commit();
            ++published_;
        }
        pending_.pop_front();
    }
}

}  // namespace lidar
