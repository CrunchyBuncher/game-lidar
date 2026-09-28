// D3D12 root signatures: the layouts ReShade reports for the fake game's two root signatures, plus
// the corner cases.
#include "d3d12/root_layout.h"

#include <cstdint>

#include "test.h"

TEST_CASE(root_layout, lookups) {
    using namespace reshade::api;
    using namespace lidar::cam;
    using d3d12::CbvLocation;

    auto root_cbv = [](uint32_t reg, uint32_t space, shader_stage vis) {
        descriptor_range r{};
        r.dx_register_index = reg;
        r.dx_register_space = space;
        r.count = 1;
        r.visibility = vis;
        r.type = descriptor_type::constant_buffer;
        return r;
    };
    auto key = [](shader_stage stage, uint32_t slot, uint32_t space = 0) {
        CbufferKey k;
        k.stage = stage;
        k.slot = slot;
        k.space = space;
        return k;
    };
    CbvLocation loc;

    // Root CBVs, version 1.0 (push_descriptors) and 1.1 (a single range with flags): b1 then b0.
    {
        const descriptor_range_with_flags b0{root_cbv(0, 0, shader_stage::all)};
        const pipeline_layout_param params[2] = {root_cbv(1, 0, shader_stage::all), b0};
        const auto layout = d3d12::convert_layout(2, params);
        EXPECT(layout.size() == 2 && layout[0].kind == d3d12::RootParam::RootCbv && layout[1].kind == d3d12::RootParam::RootCbv);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 0), loc) && loc.param == 1 && !loc.table);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::pixel, 1), loc) && loc.param == 0 && !loc.table);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 2), loc));
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 0, 1), loc));  // other space
    }
    // Stage visibility: b0 only visible to the pixel shader.
    {
        const pipeline_layout_param params[1] = {root_cbv(0, 0, shader_stage::pixel)};
        const auto layout = d3d12::convert_layout(1, params);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 0), loc));
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::pixel, 0), loc));
    }
    // A table [SRV t0..t3, CBV b1, CBV b0 x2 in space 2], after a root constant: offsets are
    // ReShade's resolved `binding`s (APPEND already applied).
    {
        descriptor_range ranges[3] = {};
        ranges[0].type = descriptor_type::texture_shader_resource_view;
        ranges[0].count = 4;
        ranges[1] = root_cbv(1, 0, shader_stage::all);
        ranges[1].binding = 4;
        ranges[2] = root_cbv(0, 2, shader_stage::all);
        ranges[2].binding = 5;
        ranges[2].count = 2;
        const pipeline_layout_param params[2] = {constant_range{0, 3, 0, 4, shader_stage::all},
                                                 pipeline_layout_param(3, ranges)};
        const auto layout = d3d12::convert_layout(2, params);
        EXPECT(layout[0].kind == d3d12::RootParam::Other && layout[1].kind == d3d12::RootParam::Table);
        EXPECT(layout[1].ranges.size() == 2);  // only the CBV ranges
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 1), loc) && loc.param == 1 && loc.table &&
               loc.table_offset == 4);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 1, 2), loc) && loc.table_offset == 6);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 2, 2), loc));
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 0), loc));  // b0 is in space 2
    }
    // Unbounded range (count UINT32_MAX) must not overflow.
    {
        descriptor_range_with_flags r{root_cbv(10, 0, shader_stage::all)};
        r.count = UINT32_MAX;
        const pipeline_layout_param params[1] = {pipeline_layout_param(1, &r)};
        const auto layout = d3d12::convert_layout(1, params);
        EXPECT(layout[0].kind == d3d12::RootParam::Table);
        EXPECT(d3d12::find_cbv(layout, key(shader_stage::vertex, 1000), loc) && loc.table_offset == 990);
        EXPECT(!d3d12::find_cbv(layout, key(shader_stage::vertex, 9), loc));
    }
}
