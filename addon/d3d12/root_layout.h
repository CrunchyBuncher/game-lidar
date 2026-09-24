// D3D12 root signatures reduced to what the cbuffer sniffer needs: which root parameter holds a
// given CBV register (bN, space S) for a shader stage. Pure, so the unit tests can cover it.
#pragma once
#include <reshade_api_pipeline.hpp>

#include <cstdint>
#include <vector>

#include "cbuffer_source.h"

namespace lidar::cam::d3d12 {

// CBVs in a descriptor table.
struct CbvRange {
    uint32_t table_offset = 0;  // in descriptors from the table start
    uint32_t reg = 0, space = 0, count = 0;
    reshade::api::shader_stage visibility = reshade::api::shader_stage::all;
};

struct RootParam {
    enum Kind : uint8_t { Other, RootCbv, Table } kind = Other;
    uint32_t reg = 0, space = 0;  // RootCbv
    reshade::api::shader_stage visibility = reshade::api::shader_stage::all;
    std::vector<CbvRange> ranges;  // Table: its CBV ranges only
};

using RootLayout = std::vector<RootParam>;

// From the parameters ReShade reports in init_pipeline_layout (root parameter i = params[i];
// static samplers may add a trailing parameter, which ends up as Other).
RootLayout convert_layout(uint32_t count, const reshade::api::pipeline_layout_param* params);

// Where `key`'s register is bound in `layout`.
struct CbvLocation {
    uint32_t param = 0;
    bool table = false;
    uint32_t table_offset = 0;  // for tables: descriptor offset from the table start
};
bool find_cbv(const RootLayout& layout, const CbufferKey& key, CbvLocation& out);

}  // namespace lidar::cam::d3d12
