#include "draw_data.h"

namespace lidar::cam {

uint64_t object_key(const DrawRecord& r, bool up_by_pointer) {
    const DrawCall& c = r.call;
    const DrawGeometry& g = r.geometry;
    const uint64_t parts[] = {uint64_t(c.type) | (uint64_t(g.up) << 8),
                              c.count,
                              c.instances,
                              c.first,
                              uint64_t(uint32_t(c.vertex_offset)),
                              g.vb,
                              (uint64_t(g.vb_offset) << 32) | g.vb_stride,
                              g.ib,
                              g.vs,
                              g.up ? (up_by_pointer ? g.up_vertices : g.up_hash) : 0,
                              g.up && up_by_pointer ? g.up_indices : 0};
    uint64_t h = 0xcbf29ce484222325ull;  // FNV-1a
    for (uint64_t p : parts)
        for (int i = 0; i < 8; ++i) h = (h ^ ((p >> (i * 8)) & 0xFF)) * 0x100000001b3ull;
    return h;
}

void solver_draws(const std::vector<DrawRecord>& records, bool column_major, std::vector<mv::Draw>& out) {
    out.clear();
    for (const DrawRecord& r : records)
        if (r.has_window) out.push_back({object_key(r), mat::load(r.window, column_major), double(r.call.count)});
}

}  // namespace lidar::cam
