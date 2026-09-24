// The API-specific half of the camera path: "which bytes does the buffer bound at this
// stage/slot hold for this draw?". The shared camera tracker decides which draws matter
// and pairs the result with the depth frame.
//
// D3D11 keeps CPU shadow copies of every cbuffer (updated on Unmap / UpdateSubresource), so it
// answers at the draw. D3D12/Vulkan games write persistently mapped memory without any event;
// their sources will record where the bytes live at the draw (ready = false) and read them in
// resolve(), once the command list is submitted and the game must have written them.
#pragma once
#include <reshade_api.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace lidar::cam {

// Identifies the camera cbuffer at a draw: shader stage + register (bN) + register space, which
// stay the same across runs (handles don't). D3D12 finds the root parameter holding that register
// from the root signature. `size` tells apart buffers that share a register: D3D11 compares it to
// the buffer size, D3D12 (rounded up to 256 bytes) to the CBV size. Root CBVs have no size to check.
struct CbufferKey {
    reshade::api::shader_stage stage = reshade::api::shader_stage::vertex;
    uint32_t slot = 0;   // register
    uint32_t space = 0;  // register space (D3D12)
    uint32_t size = 0;   // 0 = any size
};

// A read of [offset, offset + size) of the buffer bound at a draw, offsets relative to the bound range.
struct CbufferRead {
    uint64_t buffer = 0;          // resource handle
    uint64_t offset = 0;          // absolute byte offset in the buffer
    std::vector<uint8_t> bytes;   // the data, once ready
    bool ready = false;
    uint32_t source_size = 0;     // what CbufferKey::size compares to (buffer or CBV size), 0 if unknown

    // What resolve() needs when the source couldn't read at the draw. Only that source uses these.
    struct Deferred {
        uint32_t size = 0;             // window size
        uint32_t key_size = 0;         // CbufferKey::size, when the binding's size is only known later
        uint64_t descriptor_heap = 0;  // D3D12 descriptor table: the CBV's heap + index, looked up at
        uint32_t descriptor = 0;       // submit (descriptors may be written after the draw). `buffer` is
                                       // then 0 and `offset` relative to the view until resolved.
        bool clamp = false;            // read up to `size` bytes, fewer if the buffer/view ends first
    } deferred;
};

// A constant buffer bound at a draw, as discovery sees it: where it's bound and its contents.
struct BoundBuffer {
    CbufferKey key;  // key.size = read.source_size
    CbufferRead read;
};

class CbufferSource {
public:
    virtual ~CbufferSource() = default;

    // Called at a draw on `cmd`. Fills `out` for the buffer bound at `key`, or returns false if
    // nothing matching is bound (or its contents aren't known yet). `out.bytes` keeps its capacity.
    virtual bool read_at_draw(reshade::api::command_list* cmd, const CbufferKey& key, uint32_t offset, uint32_t size,
                              CbufferRead& out) = 0;

    // Discovery: appends a read of every constant buffer bound to a graphics stage at this draw, from
    // the start of its bound range, up to `max_bytes` each. Reads that aren't ready go through resolve()
    // like read_at_draw's.
    virtual void read_all_at_draw(reshade::api::command_list* cmd, uint32_t max_bytes,
                                  std::vector<BoundBuffer>& out) = 0;

    // Completes a read that wasn't ready at the draw. Called once the draw's command list has been
    // submitted (or at present for the immediate context). Returns out.ready.
    virtual bool resolve(CbufferRead& read) { return read.ready; }

    // Constant buffers currently tracked (overlay readout).
    virtual size_t tracked_buffers() const = 0;
};

// The implementation for `dev`'s API, or nullptr if the API isn't supported yet.
std::unique_ptr<CbufferSource> create_cbuffer_source(reshade::api::device* dev);

// Registers the event handlers of `api`'s backend (once per API: registering events can change
// how ReShade hooks an API, so nothing is registered for one the game doesn't use).
void register_source_events(reshade::api::device_api api);
void init_source_device(reshade::api::device* dev);  // for a device created before registration

}  // namespace lidar::cam
