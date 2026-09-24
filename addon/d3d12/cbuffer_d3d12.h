// D3D12 cbuffer source. Games write constants into persistently mapped upload memory without
// any API event, so nothing can be shadowed. Instead: at the draw, record where the camera
// cbuffer lives (root CBV: buffer + offset; descriptor table: descriptor heap + index), and in
// resolve(), once the command list is submitted, read the bytes through the game's Map pointer.
//
// Tracked: upload-heap buffers and their Map pointers, root signature layouts
// (init_pipeline_layout), CBV descriptors (create + copies), and per command list the graphics
// root signature, root CBVs (push_descriptors) and descriptor tables (bind_descriptor_tables).
#pragma once
#include "cbuffer_source.h"

namespace lidar::cam::d3d12 {

std::unique_ptr<CbufferSource> create(reshade::api::device* dev);

void register_events();
void unregister_events();
void init_device(reshade::api::device* dev);  // idempotent, ignores other APIs

}  // namespace lidar::cam::d3d12
