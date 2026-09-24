// D3D9 camera source. D3D9 has no constant buffers: shaders read float constant registers (c#)
// that the game sets with Set{Vertex,Pixel}ShaderConstantF, and ReShade reports every such call
// as push_constants. The source shadows each stage's register file, so it always holds what the
// next draw will see, and reads the camera window from it at the draw.
//
// Profiles address the register file as one buffer: slot 0, byte offset = register * 16
// (c4 = offset 64). CbufferKey::size has no meaning here and is ignored, and so is `space`.
#pragma once
#include "cbuffer_source.h"

namespace lidar::cam::d3d9 {

std::unique_ptr<CbufferSource> create(reshade::api::device* dev);

void register_events();
void unregister_events();
void init_device(reshade::api::device* dev);  // idempotent, ignores other APIs

}  // namespace lidar::cam::d3d9
