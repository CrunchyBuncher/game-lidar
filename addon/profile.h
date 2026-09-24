// Game profiles (profiles/<game>.toml, see spec.md §3.7) and turning the latched cbuffer bytes
// into the protocol's matrices (row-major, row-vector: p_clip = p_world * view * proj).
#pragma once
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "cbuffer_source.h"

namespace lidar {

enum class CameraLayout {
    ViewAndProj,         // "view+proj"
    ViewProjAndProj,     // "viewproj+proj":     view = viewProj * proj^-1
    InvViewAndProj,      // "invview+proj":      view = invView^-1
    InvViewProjAndProj,  // "invviewproj+proj":  view = invViewProj^-1 * proj^-1
};

struct CameraProfile {
    cam::CbufferKey key;  // stage, slot, space, size (0 = any)
    CameraLayout layout = CameraLayout::ViewAndProj;
    uint32_t view_offset = 0;  // bytes: the matrix the layout names first
    uint32_t proj_offset = 64;
    bool column_major = false;
    bool right_handed = false;  // informational: unprojection doesn't depend on it
    bool latch_last = false;

    // The contiguous byte window covering both matrices, relative to the bound range.
    uint32_t window_offset() const;
    uint32_t window_size() const;
};

struct Profile {
    bool has_camera = false;
    CameraProfile camera;
};

// Parses the TOML subset profiles use: [sections], key = "string" | integer | float | bool,
// and # comments. Unknown keys in known sections are errors (typos would silently do nothing).
bool parse_profile(std::string_view text, Profile& out, std::string& error);
bool load_profile(const std::filesystem::path& path, Profile& out, std::string& error);

const char* layout_name(CameraLayout layout);
const char* stage_name(reshade::api::shader_stage stage);

// Decodes a latched window (starting at window_offset()) into protocol matrices. Returns false,
// with a reason, for anything that isn't a plausible camera.
bool decode_camera(const CameraProfile& profile, const uint8_t* window, size_t window_size, float view[16],
                   float proj[16], std::string* why = nullptr);

// What a perspective projection says about the camera. D3D clip space (z in [0, 1]).
struct ProjectionInfo {
    bool valid = false;
    float fov_y_deg = 0, aspect = 0;
    float near_z = 0, far_z = 0;  // far_z is +inf for infinite projections
    bool reversed = false;        // depth 1 at the near plane
    bool right_handed = false;    // camera looks down -z
};
ProjectionInfo analyze_projection(const float proj[16]);

}  // namespace lidar
