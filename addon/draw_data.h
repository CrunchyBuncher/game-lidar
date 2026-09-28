// What the camera tracker records at draws: plain data, no ReShade calls. Discovery consumes it
// (live, and from recordings), so it lives apart from the tracker itself.
#pragma once
#include <cstdint>
#include <vector>

#include "cbuffer_source.h"
#include "modelview.h"

namespace lidar::cam {

// A draw call's arguments as ReShade reports them (D3D9 UP draws: count only, the rest 0).
enum class DrawType : uint8_t { Draw, Indexed, Indirect };
struct DrawCall {
    DrawType type = DrawType::Draw;
    uint32_t count = 0, instances = 0;  // vertices or indices
    uint32_t first = 0;                 // first vertex or index
    int32_t vertex_offset = 0;          // indexed: base vertex
    uint32_t first_instance = 0;
};

// Discovery: one sampled draw. `draw` is its index among the frame's draws into that depth-stencil.
struct DrawSample {
    uint32_t draw = 0;
    DrawCall call;
    DrawGeometry geometry;             // if the source reads it (diagnostics)
    std::vector<BoundBuffer> buffers;  // all resolved
};
// One draw into a depth-stencil, recorded for a model-view camera (or discovery): its call and
// geometry (the object's identity) and the 64-byte window at the requested place (world * view).
struct DrawRecord {
    uint32_t draw = 0;
    DrawCall call;
    DrawGeometry geometry;
    bool has_window = false;
    float window[16] = {};
};

// Where recorded draws read their window: the model-view camera's, or while only discovering, the
// vertex stage's first 64 constant bytes (D3D9 c0-c3, where SA2 keeps its world * view).
struct DrawRequest {
    CbufferKey key;
    uint32_t offset = 0;
};

// What identifies a draw's object across frames: its call and bindings, and for D3D9 UP draws the
// hash of their data (or with `up_by_pointer`, the game's pointers: SA2 reuses them for other data).
uint64_t object_key(const DrawRecord& r, bool up_by_pointer = false);
// The solver's input: every draw with a window, keyed, weighted by its vertex/index count.
void solver_draws(const std::vector<DrawRecord>& records, bool column_major, std::vector<mv::Draw>& out);

// One depth-stencil's discovery samples for a frame.
struct DepthSamples {
    uint32_t draws = 0;               // draws into the depth-stencil this frame
    std::vector<DrawSample> samples;  // in draw order
    std::vector<DrawRecord> all;      // every draw, if the source reads draw geometry
    DrawRequest window;               // where all[].window was read
};

}  // namespace lidar::cam
