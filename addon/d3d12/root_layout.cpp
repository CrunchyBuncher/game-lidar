#include "root_layout.h"

using namespace reshade::api;

namespace lidar::cam::d3d12 {
namespace {

// R is descriptor_range or descriptor_range_with_flags.
template <class R>
void add_ranges(RootParam& p, uint32_t count, const R* ranges) {
    p.kind = RootParam::Table;
    for (uint32_t i = 0; i < count; ++i) {
        const descriptor_range& r = ranges[i];
        if (r.type != descriptor_type::constant_buffer) continue;
        p.ranges.push_back({r.binding, r.dx_register_index, r.dx_register_space, r.count, r.visibility});
    }
}

void set_root_cbv(RootParam& p, const descriptor_range& r) {
    if (r.type != descriptor_type::constant_buffer || r.count != 1) return;
    p.kind = RootParam::RootCbv;
    p.reg = r.dx_register_index;
    p.space = r.dx_register_space;
    p.visibility = r.visibility;
}

}  // namespace

RootLayout convert_layout(uint32_t count, const pipeline_layout_param* params) {
    RootLayout layout(count);
    for (uint32_t i = 0; i < count; ++i) {
        const pipeline_layout_param& src = params[i];
        RootParam& p = layout[i];
        switch (src.type) {
            case pipeline_layout_param_type::push_descriptors:  // root descriptor, root signature 1.0
                set_root_cbv(p, src.push_descriptors);
                break;
            case pipeline_layout_param_type::push_descriptors_with_ranges:
                if (src.descriptor_table.count == 1) set_root_cbv(p, src.descriptor_table.ranges[0]);
                break;
            case pipeline_layout_param_type::push_descriptors_with_ranges_and_flags:  // 1.1, or static samplers
                if (src.descriptor_table_with_flags.count == 1) set_root_cbv(p, src.descriptor_table_with_flags.ranges[0]);
                break;
            case pipeline_layout_param_type::descriptor_table:
                add_ranges(p, src.descriptor_table.count, src.descriptor_table.ranges);
                break;
            case pipeline_layout_param_type::descriptor_table_with_flags:
                add_ranges(p, src.descriptor_table_with_flags.count, src.descriptor_table_with_flags.ranges);
                break;
            default:
                break;
        }
    }
    return layout;
}

bool find_cbv(const RootLayout& layout, const CbufferKey& key, CbvLocation& out) {
    for (uint32_t i = 0; i < uint32_t(layout.size()); ++i) {
        const RootParam& p = layout[i];
        if (p.kind == RootParam::RootCbv) {
            if (p.reg == key.slot && p.space == key.space && (p.visibility & key.stage) != 0) {
                out = {i, false, 0};
                return true;
            }
        } else if (p.kind == RootParam::Table) {
            for (const CbvRange& r : p.ranges) {
                // count may be UINT32_MAX (unbounded), so compare the distance, not reg + count.
                if (r.space == key.space && key.slot >= r.reg && key.slot - r.reg < r.count &&
                    (r.visibility & key.stage) != 0) {
                    out = {i, true, r.table_offset + (key.slot - r.reg)};
                    return true;
                }
            }
        }
    }
    return false;
}

}  // namespace lidar::cam::d3d12
