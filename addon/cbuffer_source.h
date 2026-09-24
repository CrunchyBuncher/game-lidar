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

// Identifies the camera cbuffer at a draw. D3D11: shader stage + register slot, with the buffer
// size to tell apart buffers that share a slot (handles change every run, sizes don't).
// Other APIs will add their own binding identity (root parameter, descriptor set/binding).
struct CbufferKey {
    reshade::api::shader_stage stage = reshade::api::shader_stage::vertex;
    uint32_t slot = 0;
    uint32_t size = 0;  // 0 = any size
};

// A read of [offset, offset + size) of the buffer bound at a draw, offsets relative to the bound range.
struct CbufferRead {
    uint64_t buffer = 0;          // resource handle
    uint64_t offset = 0;          // absolute byte offset in the buffer
    std::vector<uint8_t> bytes;   // the data, once ready
    bool ready = false;
};

class CbufferSource {
public:
    virtual ~CbufferSource() = default;

    // Called at a draw on `cmd`. Fills `out` for the buffer bound at `key`, or returns false if
    // nothing matching is bound (or its contents aren't known yet). `out.bytes` keeps its capacity.
    virtual bool read_at_draw(reshade::api::command_list* cmd, const CbufferKey& key, uint32_t offset, uint32_t size,
                              CbufferRead& out) = 0;

    // Completes a read that wasn't ready at the draw. Called once the draw's command list has been
    // submitted (or at present for the immediate context). Returns out.ready.
    virtual bool resolve(CbufferRead& read) { return read.ready; }

    // Constant buffers currently tracked (overlay readout).
    virtual size_t tracked_buffers() const = 0;
};

// The implementation for `dev`'s API, or nullptr if the API isn't supported yet.
std::unique_ptr<CbufferSource> create_cbuffer_source(reshade::api::device* dev);

// Registers the event handlers of every backend. Backends only act on devices of their own API.
void register_source_events();
void unregister_source_events();

}  // namespace lidar::cam
