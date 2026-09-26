// The capture downsample shader, shared by every DepthCapture. Same point-sampling as fake_game,
// so both producers publish identical frames. D3D11 binds Dims as a cbuffer, D3D12 as root constants.
// cs_color samples the scene's color at the same pixels (D3D11), clamped to [0, 1] (HDR isn't mapped).
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

Texture2D<float4> src_color : register(t1);
RWTexture2D<unorm float4> dst_color : register(u1);

[numthreads(8, 8, 1)]
void cs_color(uint3 id : SV_DispatchThreadID) {
    if (any(id.xy >= dst_dims)) return;
    uint2 s = min(((2 * id.xy + 1) * src_dims) / (2 * dst_dims), src_dims - 1);
    dst_color[id.xy] = float4(saturate(src_color.Load(int3(s, 0)).rgb), 1);
}
)";

}  // namespace lidar
