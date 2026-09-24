#include "constants_d3d9.h"

#include <reshade.hpp>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <vector>

using namespace reshade::api;

namespace lidar::cam::d3d9 {
namespace {

// Parameters of ReShade's global D3D9 pipeline layout that push_constants reports float
// constants with (see Direct3DDevice9's constructor in ReShade): c# of each stage.
constexpr uint32_t kVertexFloatParam = 2;
constexpr uint32_t kPixelFloatParam = 5;

struct RegisterFile {
    std::vector<float> values;     // 4 per register, grown to the highest register written
    std::vector<uint8_t> written;  // per register: set at least once since the device (re)started
};

// D3D9 games can call the device from several threads (D3DCREATE_MULTITHREADED) and ReShade fires
// the events outside the runtime's lock, so the shadows have their own.
struct __declspec(uuid("6a0f3c85-d2e1-4b7a-8c49-1f5e7b3d9a26")) DeviceData {
    std::mutex mutex;
    RegisterFile stages[2];  // vertex, pixel
};

int stage_index(shader_stage s) {
    switch (s) {
        case shader_stage::vertex: return 0;
        case shader_stage::pixel: return 1;
        default: return -1;
    }
}

bool is_d3d9(device* dev) { return dev->get_api() == device_api::d3d9; }

void on_destroy_device(device* dev) {
    if (is_d3d9(dev)) dev->destroy_private_data<DeviceData>();
}

// first/count are in 32-bit values: register * 4.
void on_push_constants(command_list* cmd, shader_stage stages, pipeline_layout, uint32_t param, uint32_t first,
                       uint32_t count, const void* values) {
    if (param != kVertexFloatParam && param != kPixelFloatParam) return;
    DeviceData* dd = cmd->get_device()->get_private_data<DeviceData>();
    const int si = stage_index(stages);
    if (dd == nullptr || si < 0 || values == nullptr || count == 0) return;
    const std::lock_guard lock(dd->mutex);
    RegisterFile& f = dd->stages[si];
    if (f.values.size() < size_t(first) + count) {
        f.values.resize(size_t(first) + count, 0.0f);
        f.written.resize(f.values.size() / 4, 0);
    }
    std::memcpy(f.values.data() + first, values, size_t(count) * sizeof(float));
    std::fill_n(f.written.begin() + first / 4, count / 4, uint8_t(1));
}

// A Reset drops the device's state: registers read back as zero until the game sets them again.
void on_destroy_command_queue(command_queue* q) {
    DeviceData* dd = q->get_device()->get_private_data<DeviceData>();
    if (dd == nullptr) return;
    const std::lock_guard lock(dd->mutex);
    for (RegisterFile& f : dd->stages) f = {};
}

class Source final : public CbufferSource {
public:
    explicit Source(device* dev) : dev_(dev) {}

    bool read_at_draw(command_list*, const CbufferKey& key, uint32_t offset, uint32_t size,
                      CbufferRead& out) override {
        const int si = stage_index(key.stage);
        if (si < 0 || key.slot != 0 || offset % 4 != 0 || size % 4 != 0) return false;
        DeviceData* dd = dev_->get_private_data<DeviceData>();
        const std::lock_guard lock(dd->mutex);
        const RegisterFile& f = dd->stages[si];
        const size_t first = offset / 4, n = size / 4;
        if (first + n > f.values.size()) return false;
        const size_t reg_end = (first + n + 3) / 4;
        if (std::find(f.written.begin() + ptrdiff_t(first / 4), f.written.begin() + ptrdiff_t(reg_end), 0) !=
            f.written.begin() + ptrdiff_t(reg_end))
            return false;  // part of the window was never set

        const auto* bytes = reinterpret_cast<const uint8_t*>(f.values.data() + first);
        out.buffer = 0;
        out.offset = offset;
        out.bytes.assign(bytes, bytes + size);
        out.ready = true;
        return true;
    }

    // Registers ever set, vertex + pixel (overlay readout).
    size_t tracked_buffers() const override {
        DeviceData* dd = dev_->get_private_data<DeviceData>();
        const std::lock_guard lock(dd->mutex);
        size_t n = 0;
        for (const RegisterFile& f : dd->stages) n += size_t(std::count(f.written.begin(), f.written.end(), 1));
        return n;
    }

private:
    device* dev_;
};

}  // namespace

void init_device(device* dev) {
    if (is_d3d9(dev) && dev->get_private_data<DeviceData>() == nullptr) dev->create_private_data<DeviceData>();
}

std::unique_ptr<CbufferSource> create(device* dev) {
    if (!is_d3d9(dev) || dev->get_private_data<DeviceData>() == nullptr) return nullptr;
    return std::make_unique<Source>(dev);
}

void register_events() {
    using reshade::addon_event;
    reshade::register_event<addon_event::init_device>(init_device);
    reshade::register_event<addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<addon_event::push_constants>(on_push_constants);
    reshade::register_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
}

void unregister_events() {
    using reshade::addon_event;
    reshade::unregister_event<addon_event::init_device>(init_device);
    reshade::unregister_event<addon_event::destroy_device>(on_destroy_device);
    reshade::unregister_event<addon_event::push_constants>(on_push_constants);
    reshade::unregister_event<addon_event::destroy_command_queue>(on_destroy_command_queue);
}

}  // namespace lidar::cam::d3d9
