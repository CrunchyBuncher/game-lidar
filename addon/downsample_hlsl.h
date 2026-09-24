// The capture downsample shader, shared by every DepthCapture. Same point-sampling as fake_game,
// so both producers publish identical frames. D3D11 binds Dims as a cbuffer, D3D12 as root constants.
#pragma once

namespace lidar {

inline const char* kDownsampleHlsl = R"(
Texture2D<float> src_depth : register(t0);
RWTexture2D<float> dst_depth : register(u0);
cbuffer Dims : register(b0) { uint2 src_dims; uint2 dst_dims; };

[numthreads(8, 8, 1)]
void cs_main(uint3 id : SV_DispatchThreadID) {
    if (any(id.xy >= dst_dims)) return;
    uint2 s = min(((2 * id.xy + 1) * src_dims) / (2 * dst_dims), src_dims - 1);  // exact, see protocol.h
    dst_depth[id.xy] = src_depth.Load(int3(s, 0));
}
)";

}  // namespace lidar
