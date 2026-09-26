// Finds the game's scene depth buffer: counts draws/vertices per bound depth-stencil during
// each frame and ranks them at present. The heuristics follow ReShade's generic_depth addon
// (Copyright (C) 2021 Patrick Mours, BSD-3-Clause), trimmed to what the lidar capture needs.
#pragma once
#include <reshade.hpp>

#include <cstdint>
#include <vector>

namespace lidar::depth {

struct DrawStats {
    uint32_t vertices = 0;
    uint32_t drawcalls = 0;
    uint32_t drawcalls_indirect = 0;

    // More vertices wins, unless most draws are indirect (vertex counts unknown), then draw calls.
    bool better_than(const DrawStats& o) const {
        return drawcalls_indirect < drawcalls / 3 ? vertices > o.vertices : drawcalls > o.drawcalls;
    }
};

struct Candidate {
    reshade::api::resource resource{0};
    reshade::api::resource_desc desc;
    DrawStats stats;
    DrawStats last_segment;       // drawn since its last depth clear (or the frame's start)
    reshade::api::resource color{0};  // render target 0 at the last draw into it (0: none)
    uint32_t last_pass = 0;       // that pass' number: depth clears before it (cam::PassKey)
    bool reversed_clear = false;  // cleared to something other than 1.0 this frame
    bool fits_frame = false;      // aspect ratio / size similar to the back buffer
};

void register_events();
void unregister_events();
// Sets up `dev` (idempotent). The init_device handler does it for devices created after
// register_events(); call it for a device created before.
void init_device(reshade::api::device* dev);

// Called before a depth clear of a depth-stencil that was drawn into since its last one, on the
// clearing thread: `segment` is what that pass drew, `pass` its number (cam::PassKey). Games that
// reuse the scene depth-stencil for a later pass (UE3) only have the scene in it until that clear.
// `color` is render target 0 at that pass' last draw (0: none). nullptr to stop.
using ClearHook = void (*)(reshade::api::command_list* cmd, reshade::api::resource ds, reshade::api::resource color,
                           const DrawStats& segment, uint32_t pass);
void set_clear_hook(ClearHook hook);

// Call once per present. Collects the frame's stats from every graphics queue of the device,
// resets them, and returns the depth-stencils used this frame, best scene-depth guess first
// (only candidates that fit the frame can come first).
std::vector<Candidate> end_frame(reshade::api::device* dev, uint32_t frame_width, uint32_t frame_height);

}  // namespace lidar::depth
