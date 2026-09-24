#include "constants_d3d9.h"

#include <d3d9.h>
#include <reshade.hpp>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <utility>
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

// ---- Draw geometry (discovery diagnostics) -------------------------------------------------------
// ReShade 6.8.0 reports Draw*PrimitiveUP with one shared stand-in vertex buffer, and only if some
// addon registers bind_vertex_buffers / bind_index_buffer, which also makes it leak a D3D9 buffer
// per DrawIndexedPrimitiveUP (resize_primitive_up_buffers creates the index buffer into the vertex
// buffer's handle). So the bindings are read back from the device instead, and the game's UP
// pointers come from a hook on the driver device's two UP methods, armed only for recorded draws.

struct UpCapture {
    std::mutex mutex;
    bool armed = false, seen = false;
    uint64_t vertices = 0, indices = 0;
    uint32_t bytes = 0;
    uint64_t hash = 0;
};
UpCapture g_up;

using DrawUpFn = HRESULT(STDMETHODCALLTYPE*)(IDirect3DDevice9*, D3DPRIMITIVETYPE, UINT, const void*, UINT);
using DrawIndexedUpFn = HRESULT(STDMETHODCALLTYPE*)(IDirect3DDevice9*, D3DPRIMITIVETYPE, UINT, UINT, UINT,
                                                    const void*, D3DFORMAT, const void*, UINT);
constexpr size_t kDrawUpSlot = 83, kDrawIndexedUpSlot = 84;  // IDirect3DDevice9 vtable
void** g_hooked_vtable = nullptr;
DrawUpFn g_draw_up = nullptr;
DrawIndexedUpFn g_draw_indexed_up = nullptr;
constexpr size_t kMaxUpBytes = 256 * 1024;  // hashed per draw

uint32_t vertex_count(D3DPRIMITIVETYPE type, UINT prims) {
    switch (type) {
        case D3DPT_LINELIST: return prims * 2;
        case D3DPT_LINESTRIP: return prims + 1;
        case D3DPT_TRIANGLELIST: return prims * 3;
        case D3DPT_TRIANGLESTRIP:
        case D3DPT_TRIANGLEFAN: return prims + 2;
        default: return prims;
    }
}

// FNV-1a over 8-byte words (the game's vertex data can be large).
uint64_t hash_words(const void* p, size_t n, uint64_t h) {
    const auto* b = static_cast<const uint8_t*>(p);
    size_t i = 0;
    for (; i + 8 <= n; i += 8) {
        uint64_t w;
        std::memcpy(&w, b + i, 8);
        h = (h ^ w) * 0x100000001b3ull;
        h ^= h >> 29;
    }
    for (; i < n; ++i) h = (h ^ b[i]) * 0x100000001b3ull;
    return h;
}

void capture_up(const void* vertices, size_t vertex_bytes, const void* indices, size_t index_bytes) {
    const std::lock_guard lock(g_up.mutex);
    if (!g_up.armed) return;
    g_up.armed = false;
    g_up.seen = true;
    g_up.vertices = uint64_t(uintptr_t(vertices));
    g_up.indices = uint64_t(uintptr_t(indices));
    vertex_bytes = std::min(vertex_bytes, kMaxUpBytes);
    index_bytes = std::min(index_bytes, kMaxUpBytes - vertex_bytes);
    g_up.bytes = uint32_t(vertex_bytes + index_bytes);
    uint64_t h = hash_words(vertices, vertex_bytes, 0xcbf29ce484222325ull);
    if (indices != nullptr) h = hash_words(indices, index_bytes, h);
    g_up.hash = h;
}

HRESULT STDMETHODCALLTYPE hook_draw_up(IDirect3DDevice9* d, D3DPRIMITIVETYPE type, UINT prims, const void* data,
                                       UINT stride) {
    capture_up(data, size_t(vertex_count(type, prims)) * stride, nullptr, 0);
    return g_draw_up(d, type, prims, data, stride);
}
HRESULT STDMETHODCALLTYPE hook_draw_indexed_up(IDirect3DDevice9* d, D3DPRIMITIVETYPE type, UINT min_vertex,
                                               UINT vertices, UINT prims, const void* index_data, D3DFORMAT format,
                                               const void* vertex_data, UINT stride) {
    // The driver reads vertices [min_vertex, min_vertex + vertices).
    const auto* v = static_cast<const uint8_t*>(vertex_data) + size_t(min_vertex) * stride;
    capture_up(v, size_t(vertices) * stride, index_data,
               size_t(vertex_count(type, prims)) * (format == D3DFMT_INDEX32 ? 4 : 2));
    return g_draw_indexed_up(d, type, min_vertex, vertices, prims, index_data, format, vertex_data, stride);
}

bool patch_vtable(void** vt, void* up, void* indexed_up) {
    DWORD old = 0;
    if (!VirtualProtect(vt + kDrawUpSlot, 2 * sizeof(void*), PAGE_READWRITE, &old)) return false;
    vt[kDrawUpSlot] = up;
    vt[kDrawIndexedUpSlot] = indexed_up;
    VirtualProtect(vt + kDrawUpSlot, 2 * sizeof(void*), old, &old);
    return true;
}

// Once per process: every device of the driver's class shares the vtable.
void install_up_hooks(IDirect3DDevice9* d) {
    static std::once_flag once;
    std::call_once(once, [d] {
        void** vt = *reinterpret_cast<void***>(d);
        g_draw_up = reinterpret_cast<DrawUpFn>(vt[kDrawUpSlot]);
        g_draw_indexed_up = reinterpret_cast<DrawIndexedUpFn>(vt[kDrawIndexedUpSlot]);
        if (patch_vtable(vt, reinterpret_cast<void*>(&hook_draw_up), reinterpret_cast<void*>(&hook_draw_indexed_up)))
            g_hooked_vtable = vt;
    });
}

void remove_up_hooks() {
    void** vt = std::exchange(g_hooked_vtable, nullptr);
    if (vt != nullptr && vt[kDrawUpSlot] == reinterpret_cast<void*>(&hook_draw_up))
        patch_vtable(vt, reinterpret_cast<void*>(g_draw_up), reinterpret_cast<void*>(g_draw_indexed_up));
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

    // Each stage's register file as one buffer, up to the highest register written (never-set ones
    // read as zero).
    void read_all_at_draw(command_list*, uint32_t max_bytes, std::vector<BoundBuffer>& out) override {
        DeviceData* dd = dev_->get_private_data<DeviceData>();
        const std::lock_guard lock(dd->mutex);
        for (int si = 0; si < 2; ++si) {
            const RegisterFile& f = dd->stages[si];
            const size_t n = std::min(f.values.size() * sizeof(float), size_t(max_bytes) & ~size_t(15));
            if (n == 0) continue;
            BoundBuffer& b = out.emplace_back();
            b.key.stage = si == 0 ? shader_stage::vertex : shader_stage::pixel;
            const auto* bytes = reinterpret_cast<const uint8_t*>(f.values.data());
            b.read.bytes.assign(bytes, bytes + n);
            b.read.ready = true;
        }
    }

    // Reads the bindings back from the device (the addon doesn't track them), and arms the UP hook:
    // whether this draw is an UP one is only known once it reaches the driver. Stream 0 can't tell,
    // it still holds the last buffer until the first UP call unbinds it.
    bool read_geometry_at_draw(command_list* cmd, DrawGeometry& out) override {
        auto* d = reinterpret_cast<IDirect3DDevice9*>(cmd->get_device()->get_native());
        install_up_hooks(d);
        out = {};
        IDirect3DVertexBuffer9* vb = nullptr;
        UINT offset = 0, stride = 0;
        if (SUCCEEDED(d->GetStreamSource(0, &vb, &offset, &stride)) && vb != nullptr) {
            out.vb = uint64_t(uintptr_t(vb));  // = ReShade's resource handle
            out.vb_offset = offset;
            out.vb_stride = stride;
            vb->Release();
        }
        IDirect3DIndexBuffer9* ib = nullptr;
        if (SUCCEEDED(d->GetIndices(&ib)) && ib != nullptr) {
            out.ib = uint64_t(uintptr_t(ib));
            ib->Release();
        }
        IDirect3DVertexShader9* vs = nullptr;
        if (SUCCEEDED(d->GetVertexShader(&vs)) && vs != nullptr) {
            out.vs = uint64_t(uintptr_t(vs));
            vs->Release();
        }
        const std::lock_guard lock(g_up.mutex);
        g_up.armed = true;
        g_up.seen = false;
        return true;
    }

    void complete_geometry(DrawGeometry& g) override {
        const std::lock_guard lock(g_up.mutex);
        g_up.armed = false;
        if (!std::exchange(g_up.seen, false)) return;
        g.up = true;
        g.vb = g.ib = 0;  // stale bindings, the UP draw doesn't use them
        g.vb_offset = g.vb_stride = 0;
        g.up_vertices = g_up.vertices;
        g.up_indices = g_up.indices;
        g.up_bytes = g_up.bytes;
        g.up_hash = g_up.hash;
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
    remove_up_hooks();
}

}  // namespace lidar::cam::d3d9
