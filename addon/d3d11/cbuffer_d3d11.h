// D3D11 cbuffer source: CPU shadow copies of every constant buffer, updated on Unmap and
// UpdateSubresource, plus the constant buffers bound per stage/slot on each context.
#pragma once
#include "cbuffer_source.h"

namespace lidar::cam::d3d11 {

std::unique_ptr<CbufferSource> create(reshade::api::device* dev);

void register_events();
void unregister_events();

}  // namespace lidar::cam::d3d11
