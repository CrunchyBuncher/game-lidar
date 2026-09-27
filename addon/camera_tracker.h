// Latches the camera cbuffer at the draws that go into each depth-stencil, so the frame's
// camera can be paired with whichever depth-stencil turns out to be the scene depth at present.
// API-agnostic: the bytes come from a CbufferSource.
//
// A model-view camera (plan_modelview.md) instead records every draw: what identifies its object,
// and the per-draw world * view window, for mv::Solver at present.
//
// Discovery mode also samples some draws per depth-stencil and keeps everything bound at them
// (all constant buffers, up to a size), for the discovery scanner to analyze off the draw path.
#pragma once
#include <reshade.hpp>

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "cbuffer_source.h"
#include "modelview.h"
#include "profile.h"

namespace lidar::cam {

struct LatchRequest {
    CbufferKey key;
    uint32_t offset = 0, size = 0;  // byte window to read, relative to the bound range
    Latch latch = Latch::First;
    // Whether a window holds a usable camera for a depth-stencil of aspect ratio `aspect` (width /
    // height, 0 if unknown). A pass latches the windows that pass, so draws whose register holds some
    // other buffer don't cost the frame (UE assigns registers per shader: vertex b0 is the view's
    // buffer at some draws only). If none passes, the pass still latches one (the reason it's
    // rejected is worth showing). Empty: every window passes. Called from draw threads: must be
    // thread-safe.
    std::function<bool(const std::vector<uint8_t>& window, float aspect)> accept;
};

// A pass of a depth-stencil: its draws between two depth clears in a frame, numbered by the depth
// clears before it this frame (depth_tracker numbers them the same way). Games that reuse the scene
// depth-stencil for other views (UE3) latch the scene's camera only in the scene's pass.
struct PassKey {
    uint64_t ds = 0;
    uint32_t pass = 0;
    bool operator==(const PassKey&) const = default;
};
struct PassKeyHash {
    size_t operator()(const PassKey& k) const { return std::hash<uint64_t>()(k.ds ^ (uint64_t(k.pass) << 48)); }
};

// A window a pass latched: how many of the pass' draws read it, and whether it passed
// LatchRequest::accept.
struct LatchedRead {
    CbufferRead read;
    uint32_t draws = 0;
    bool accepted = false;
};
// Per depth-stencil pass: the distinct windows that passed LatchRequest::accept, in latch order (first
// drawn first; Latch::Last: last drawn first; Latch::Common: most draws first). More than one when
// other views' buffers share the register: the caller picks the camera. If none passed, one that
// didn't.
using FrameLatches = std::unordered_map<PassKey, std::vector<LatchedRead>, PassKeyHash>;

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
using FrameDraws = std::unordered_map<uint64_t, std::vector<DrawRecord>>;  // per depth-stencil

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

struct DepthSamples {
    uint32_t draws = 0;               // draws into the depth-stencil this frame
    std::vector<DrawSample> samples;  // in draw order
    std::vector<DrawRecord> all;      // every draw, if the source reads draw geometry
    DrawRequest window;               // where all[].window was read
};
using FrameSamples = std::unordered_map<uint64_t, DepthSamples>;

void register_events();
void unregister_events();
// Sets up `dev` (idempotent). The init_device handler does it for devices created after
// register_events(); call it for a device created before.
void init_device(reshade::api::device* dev);

// Starts latching `req` on `dev` through `source`. Pass nullptr for either to stop.
// `source` must outlive the configuration.
void configure(reshade::api::device* dev, CbufferSource* source, const LatchRequest* req);

// Starts recording every draw into each depth-stencil (a model-view camera), its window read at
// `req`. Pass nullptr to stop.
void configure_draws(reshade::api::device* dev, CbufferSource* source, const DrawRequest* req);

// Starts sampling up to `samples_per_frame` draws per depth-stencil (the first few, then spread
// over the frame by last frame's draw count), reading up to `max_bytes` of each bound buffer.
// samples_per_frame = 0 (or a null source) stops. If the source reads draw geometry, every draw is
// recorded too (DepthSamples::all), for model-view candidates.
void configure_discovery(reshade::api::device* dev, CbufferSource* source, uint32_t samples_per_frame,
                         uint32_t max_bytes);

// Call once per present: this frame's latches from every graphics queue, and resets them. With
// `samples`, also this frame's discovery samples; with `draws`, the recorded draws.
FrameLatches end_frame(reshade::api::device* dev, FrameSamples* samples = nullptr, FrameDraws* draws = nullptr);

}  // namespace lidar::cam
