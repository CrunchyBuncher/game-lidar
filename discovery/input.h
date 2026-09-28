// Everything discovery consumes: the sampled draws of the captured depth-stencil, and depth frames.
// The analyzer sees nothing else, so a recording of these (recording.h) replays a session exactly.
#pragma once
#include <cstdint>

#include "camera_math.h"
#include "draw_data.h"

namespace lidar::disc {

// One frame's samples for the captured depth-stencil.
struct SampleFrame {
    uint64_t frame = 0;  // the addon's frame counter (pairs with DepthFrame::frame)
    double time = 0;     // seconds since discovery started
    cam::DepthSamples samples;
};

// A published depth frame, reduced to the grid the checks use.
struct DepthFrame {
    uint64_t frame = 0;
    double time = 0;
    DepthGrid grid;
};

}  // namespace lidar::disc
