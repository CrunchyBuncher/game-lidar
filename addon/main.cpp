// lidar_capture.addon64: ReShade addon that captures the scene depth buffer and the game's
// camera matrices and publishes them to the shared-memory ring for the viewer.
//
// Depth path: depth_tracker picks the scene depth-stencil, a DepthCapture reads it back.
// Camera path: camera_tracker latches the profile's cbuffer window at the draws into each
// depth-stencil (bytes from a CbufferSource), and at present the latch of the captured
// depth-stencil becomes the frame's pose. Without a profile or a latch, frames go out
// camera-relative (no pose flag) with a projection from the overlay settings.
//
// Only the DepthCapture and CbufferSource implementations are API-specific (backends.cpp).
#include <imgui.h>
#include <reshade.hpp>

#include <DirectXMath.h>
#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "camera_tracker.h"
#include "cbuffer_source.h"
#include "depth_capture.h"
#include "depth_tracker.h"
#include "profile.h"
#include "ring.h"

using namespace reshade::api;
using namespace lidar;

extern "C" __declspec(dllexport) const char* NAME = "game-lidar capture";
extern "C" __declspec(dllexport) const char* DESCRIPTION =
    "Captures depth and camera matrices and streams them to lidar_viewer.";

namespace {

constexpr char kSection[] = "LIDAR";
constexpr char kDefaultProfile[] = "lidar_profile.toml";

enum class DepthMode : int { Standard = 0, Reversed = 1, ReversedInfinite = 2 };

struct Settings {
    bool enabled = true;
    int capture_width = 480;
    std::string profile = kDefaultProfile;  // relative to the game's folder
    // Fallback projection for frames without a camera pose. Defaults match fake_game.
    float fov_y_deg = 70.0f;
    float near_z = 0.1f;
    float far_z = 1000.0f;
    int depth_mode = int(DepthMode::Reversed);

    void load() {
        reshade::get_config_value(nullptr, kSection, "Enabled", enabled);
        reshade::get_config_value(nullptr, kSection, "CaptureWidth", capture_width);
        char buf[1024];
        size_t n = sizeof(buf);
        if (reshade::get_config_value(nullptr, kSection, "Profile", buf, &n) && buf[0] != '\0') profile = buf;
        reshade::get_config_value(nullptr, kSection, "FovY", fov_y_deg);
        reshade::get_config_value(nullptr, kSection, "Near", near_z);
        reshade::get_config_value(nullptr, kSection, "Far", far_z);
        reshade::get_config_value(nullptr, kSection, "DepthMode", depth_mode);
    }
    void save() const {
        reshade::set_config_value(nullptr, kSection, "Enabled", enabled);
        reshade::set_config_value(nullptr, kSection, "CaptureWidth", capture_width);
        reshade::set_config_value(nullptr, kSection, "Profile", profile.c_str());
        reshade::set_config_value(nullptr, kSection, "FovY", fov_y_deg);
        reshade::set_config_value(nullptr, kSection, "Near", near_z);
        reshade::set_config_value(nullptr, kSection, "Far", far_z);
        reshade::set_config_value(nullptr, kSection, "DepthMode", depth_mode);
    }
};

// What happened to the camera on the last captured frame (overlay readout).
struct CameraStatus {
    enum State { NoProfile, NoLatch, Rejected, Latched } state = NoProfile;
    std::string why;  // for Rejected
    float view[16] = {}, proj[16] = {};
    ProjectionInfo info;
    uint64_t with_pose = 0, without_pose = 0;
};

// Touched from the game's render thread (present) and the overlay, which ReShade draws on the
// same thread; the mutex covers games that differ.
std::mutex g_mutex;
Settings g_settings;
device* g_device = nullptr;  // the device we capture from (first supported one created)
std::unique_ptr<DepthCapture> g_capture;
std::unique_ptr<cam::CbufferSource> g_source;
RingWriter g_ring;
std::string g_init_error;
uint64_t g_frame = 0;

Profile g_profile;
std::filesystem::path g_profile_path;
std::string g_profile_error;
CameraStatus g_camera;

std::vector<depth::Candidate> g_candidates;  // last frame's depth-stencils, best first
uint64_t g_selected = 0;                     // handle captured last frame
uint64_t g_override = 0;                     // manual pick from the overlay, 0 = auto
double g_cpu_us = 0;                         // smoothed CPU cost of our present work

std::filesystem::path game_dir() {
    wchar_t buf[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    return std::filesystem::path(std::wstring(buf, n)).parent_path();
}

// (Re)loads the profile and points the camera tracker at it. Caller holds g_mutex.
void reload_profile() {
    g_profile = {};
    g_profile_error.clear();
    g_camera = {};
    g_profile_path = std::filesystem::path(std::u8string(g_settings.profile.begin(), g_settings.profile.end()));
    if (g_profile_path.is_relative()) g_profile_path = game_dir() / g_profile_path;
    if (g_source == nullptr)
        g_profile_error = "camera sniffing isn't supported for this graphics API yet";
    else
        load_profile(g_profile_path, g_profile, g_profile_error);

    if (g_profile.has_camera) {
        const CameraProfile& c = g_profile.camera;
        const cam::LatchRequest req{c.key, c.window_offset(), c.window_size(), c.latch_last};
        cam::configure(g_device, g_source.get(), &req);
    } else {
        cam::configure(g_device, nullptr, nullptr);
    }
}

// Row-major, row-vector (D3D) projection, as protocol.h expects. Same math as fake_game.
void make_proj(const Settings& s, float aspect, float out[16]) {
    using namespace DirectX;
    const float fov = XMConvertToRadians(s.fov_y_deg);
    XMMATRIX p;
    switch (DepthMode(s.depth_mode)) {
        case DepthMode::Standard:
            p = XMMatrixPerspectiveFovLH(fov, aspect, s.near_z, s.far_z);
            break;
        case DepthMode::ReversedInfinite: {
            const float ys = 1.0f / std::tan(fov * 0.5f), xs = ys / aspect;
            p = XMMATRIX(xs, 0, 0, 0, 0, ys, 0, 0, 0, 0, 0, 1, 0, 0, s.near_z, 0);
            break;
        }
        default:
            p = XMMatrixPerspectiveFovLH(fov, aspect, s.far_z, s.near_z);
            break;
    }
    XMStoreFloat4x4(reinterpret_cast<XMFLOAT4X4*>(out), p);
}

// Fills the header's pose from this frame's latch for the captured depth-stencil.
bool apply_camera(const cam::FrameLatches& latches, uint64_t depth_stencil, FrameHeader& h) {
    if (!g_profile.has_camera) {
        g_camera.state = CameraStatus::NoProfile;
        return false;
    }
    const auto it = latches.find(depth_stencil);
    if (it == latches.end()) {
        g_camera.state = CameraStatus::NoLatch;
        return false;
    }
    float view[16], proj[16];
    if (!decode_camera(g_profile.camera, it->second.bytes.data(), it->second.bytes.size(), view, proj, &g_camera.why)) {
        g_camera.state = CameraStatus::Rejected;
        return false;
    }
    g_camera.state = CameraStatus::Latched;
    std::memcpy(g_camera.view, view, sizeof(view));
    std::memcpy(g_camera.proj, proj, sizeof(proj));
    g_camera.info = analyze_projection(proj);
    std::memcpy(h.view, view, sizeof(h.view));
    std::memcpy(h.proj, proj, sizeof(h.proj));
    h.flags |= kFlagPoseValid;
    return true;
}

const char* api_name(device_api api) {
    switch (api) {
        case device_api::d3d9: return "Direct3D 9";
        case device_api::d3d10: return "Direct3D 10";
        case device_api::d3d11: return "Direct3D 11";
        case device_api::d3d12: return "Direct3D 12";
        case device_api::opengl: return "OpenGL";
        case device_api::vulkan: return "Vulkan";
    }
    return "this graphics API";
}

void register_active_handlers();  // below, next to the handlers it registers

// Until a supported device shows up, the addon's only handler is init_device. Registering an
// event can change how ReShade hooks the game (on D3D9, map events wrap every buffer Lock), so on
// an API we don't support the addon must not register anything else: the game has to run exactly
// as it does without the addon. On the first supported device, everything registers. That device
// was created before the modules' handlers existed, so it's set up by hand.
void attach_modules(device* dev) {
    static bool registered = false;
    if (!registered) {
        depth::register_events();
        cam::register_events();
        cam::register_source_events();
        register_capture_events();
        register_active_handlers();
        registered = true;
    }
    depth::init_device(dev);
    cam::init_device(dev);
    cam::init_source_device(dev);
    init_capture_device(dev);
}

void on_init_device(device* dev) {
    const std::lock_guard lock(g_mutex);
    if (!is_supported(dev)) {
        static bool logged = false;
        if (!logged) {
            const std::string msg = std::string("Inactive: the game created a ") + api_name(dev->get_api()) +
                                    " device, which isn't supported (D3D11 and D3D12 are). Nothing else is hooked.";
            reshade::log::message(reshade::log::level::warning, msg.c_str());
            logged = true;
        }
        return;
    }
    // Later supported devices get this from the modules' own init_device handlers too, but those
    // run after this one.
    attach_modules(dev);
    if (g_device != nullptr) return;
    std::string error;
    std::unique_ptr<DepthCapture> capture = create_depth_capture(dev, error);
    if (capture == nullptr && g_init_error.empty()) {
        g_init_error = error;  // keep looking: a later device may be supported
        return;
    }
    if (capture == nullptr) return;
    g_device = dev;
    g_init_error.clear();
    g_capture = std::move(capture);
    g_source = cam::create_cbuffer_source(dev);
    g_settings.load();
    if (!g_ring.open()) g_init_error = "failed to create the shared-memory ring";
    reload_profile();
}

void on_destroy_device(device* dev) {
    const std::lock_guard lock(g_mutex);
    if (dev != g_device) return;
    cam::configure(dev, nullptr, nullptr);
    g_source.reset();
    g_capture.reset();
    g_ring.close();
    g_candidates.clear();
    g_device = nullptr;
}

void on_present(command_queue* queue, swapchain* sc, const rect*, const rect*, uint32_t, const rect*) {
    device* const dev = sc->get_device();
    const std::lock_guard lock(g_mutex);
    if (dev != g_device || !g_init_error.empty()) return;

    LARGE_INTEGER t0, t1, freq;
    QueryPerformanceCounter(&t0);

    const cam::FrameLatches latches = cam::end_frame(dev);
    const resource_desc bb = dev->get_resource_desc(sc->get_current_back_buffer());
    std::vector<depth::Candidate> candidates = depth::end_frame(dev, bb.texture.width, bb.texture.height);
    if (candidates.empty()) return;  // e.g. a second present without any rendering in between
    g_candidates = std::move(candidates);

    g_capture->publish(queue, g_ring);

    const depth::Candidate* pick = nullptr;
    for (const auto& c : g_candidates)
        if (c.resource.handle == g_override) pick = &c;
    if (pick == nullptr && g_candidates.front().fits_frame) pick = &g_candidates.front();
    g_selected = pick ? pick->resource.handle : 0;

    if (g_settings.enabled && pick != nullptr) {
        FrameHeader h{};
        h.frame_index = g_frame;
        h.timestamp_qpc = uint64_t(t0.QuadPart);
        if (apply_camera(latches, pick->resource.handle, h)) {
            ++g_camera.with_pose;
        } else {
            ++g_camera.without_pose;
            const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            std::memcpy(h.view, identity, sizeof(h.view));
            make_proj(g_settings, float(pick->desc.texture.width) / float(pick->desc.texture.height), h.proj);
        }
        g_capture->capture(queue, pick->resource, uint32_t(g_settings.capture_width), h);
    }
    ++g_frame;

    QueryPerformanceCounter(&t1);
    QueryPerformanceFrequency(&freq);
    const double us = double(t1.QuadPart - t0.QuadPart) * 1e6 / double(freq.QuadPart);
    g_cpu_us = g_cpu_us * 0.95 + us * 0.05;
}

const char* format_name(format f) {
    switch (f) {
        case format::d16_unorm: return "D16";
        case format::r16_typeless: return "R16 typeless";
        case format::d24_unorm_s8_uint: return "D24S8";
        case format::r24_g8_typeless: return "R24G8 typeless";
        case format::d32_float: return "D32F";
        case format::r32_typeless: return "R32 typeless";
        case format::d32_float_s8_uint: return "D32FS8";
        case format::r32_g8_typeless: return "R32G8 typeless";
        default: return "other";
    }
}

void matrix_table(const char* id, const float m[16]) {
    if (!ImGui::BeginTable(id, 4, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit)) return;
    for (int r = 0; r < 4; ++r) {
        ImGui::TableNextRow();
        for (int c = 0; c < 4; ++c) {
            ImGui::TableNextColumn();
            ImGui::Text("%10.4f", m[r * 4 + c]);
        }
    }
    ImGui::EndTable();
}

bool draw_camera_section() {
    bool changed = false;
    if (!ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) return changed;

    char path[1024];
    strncpy_s(path, g_settings.profile.c_str(), _TRUNCATE);
    if (ImGui::InputText("Profile", path, sizeof(path), ImGuiInputTextFlags_EnterReturnsTrue)) {
        g_settings.profile = path;
        changed = true;
        reload_profile();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reload")) reload_profile();
    if (!g_profile_error.empty()) {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", g_profile_error.c_str());
        return changed;
    }

    const CameraProfile& c = g_profile.camera;
    ImGui::TextDisabled("%s b%u%s%s, %s at %u / %u, %s-major, %s-handed, %s latch", stage_name(c.key.stage), c.key.slot,
                        c.key.space ? (" space" + std::to_string(c.key.space)).c_str() : "",
                        c.key.size ? (" (" + std::to_string(c.key.size) + " bytes)").c_str() : "",
                        layout_name(c.layout), c.view_offset, c.proj_offset, c.column_major ? "column" : "row",
                        c.right_handed ? "right" : "left", c.latch_last ? "last" : "first");
    if (g_source) ImGui::TextDisabled("Tracking %zu constant buffers", g_source->tracked_buffers());

    switch (g_camera.state) {
        case CameraStatus::NoProfile: break;
        case CameraStatus::NoLatch:
            ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1),
                               "No latch: no draw into the captured depth buffer had that cbuffer bound.");
            break;
        case CameraStatus::Rejected:
            ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "Latched, but rejected: %s", g_camera.why.c_str());
            break;
        case CameraStatus::Latched: ImGui::TextColored(ImVec4(0.3f, 1, 0.4f, 1), "Pose latched"); break;
    }
    ImGui::Text("Frames with pose %llu, without %llu", static_cast<unsigned long long>(g_camera.with_pose),
                static_cast<unsigned long long>(g_camera.without_pose));

    if (g_camera.state == CameraStatus::Latched) {
        const ProjectionInfo& p = g_camera.info;
        char far_text[32] = "infinite";
        if (!std::isinf(p.far_z)) std::snprintf(far_text, sizeof(far_text), "%.6g", p.far_z);
        ImGui::Text("From proj: FOV %.2f deg, aspect %.3f, near %.4g, far %s, %s, %s-handed", p.fov_y_deg, p.aspect,
                    p.near_z, far_text, p.reversed ? "reversed depth" : "standard depth",
                    p.right_handed ? "right" : "left");
        if (p.right_handed != c.right_handed)
            ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "Hint: the projection is %s-handed but the profile says %s.",
                               p.right_handed ? "right" : "left", c.right_handed ? "right" : "left");
        if (ImGui::TreeNode("Matrices")) {
            ImGui::TextUnformatted("view");
            matrix_table("view", g_camera.view);
            ImGui::TextUnformatted("proj");
            matrix_table("proj", g_camera.proj);
            ImGui::TreePop();
        }
    }
    return changed;
}

void draw_overlay(effect_runtime*) {
    const std::lock_guard lock(g_mutex);
    if (g_device == nullptr) {
        ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "No supported device: %s",
                           g_init_error.empty() ? "none created yet" : g_init_error.c_str());
        return;
    }
    if (!g_init_error.empty()) {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", g_init_error.c_str());
        return;
    }

    bool changed = ImGui::Checkbox("Capture", &g_settings.enabled);
    ImGui::SameLine();
    ImGui::TextDisabled("(%.1f fps, addon CPU %.0f us/frame)", ImGui::GetIO().Framerate, g_cpu_us);
    if (!g_capture->error().empty()) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", g_capture->error().c_str());
    ImGui::Text("Published %llu frames at %ux%u, skipped %llu (GPU readback busy)",
                static_cast<unsigned long long>(g_capture->published()), g_capture->width(), g_capture->height(),
                static_cast<unsigned long long>(g_capture->skipped()));
    changed |= ImGui::SliderInt("Capture width", &g_settings.capture_width, 64, int(kMaxWidth));

    changed |= draw_camera_section();

    if (ImGui::CollapsingHeader("Fallback projection (frames without a pose)")) {
        changed |= ImGui::SliderFloat("Vertical FOV", &g_settings.fov_y_deg, 20.0f, 120.0f, "%.1f deg");
        changed |= ImGui::InputFloat("Near", &g_settings.near_z, 0.01f, 0.1f, "%.3f");
        changed |= ImGui::InputFloat("Far", &g_settings.far_z, 10.0f, 100.0f, "%.1f");
        changed |= ImGui::Combo("Depth mode", &g_settings.depth_mode, "Standard\0Reversed\0Reversed infinite\0");
        for (const auto& c : g_candidates)
            if (c.resource.handle == g_selected) {
                const bool reversed = g_settings.depth_mode != int(DepthMode::Standard);
                if (c.reversed_clear != reversed)
                    ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "Hint: the game clears depth to %s, which suggests %s.",
                                       c.reversed_clear ? "0" : "1", c.reversed_clear ? "reversed" : "standard");
            }
    }
    g_settings.near_z = std::max(g_settings.near_z, 1e-4f);
    g_settings.far_z = std::max(g_settings.far_z, g_settings.near_z * 2);

    if (ImGui::CollapsingHeader("Depth buffer", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::RadioButton("Automatic", g_override == 0)) g_override = 0;
        if (ImGui::BeginTable("ds", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Use");
            ImGui::TableSetupColumn("Size");
            ImGui::TableSetupColumn("Format");
            ImGui::TableSetupColumn("Draws");
            ImGui::TableSetupColumn("Vertices");
            ImGui::TableHeadersRow();
            for (const auto& c : g_candidates) {
                ImGui::PushID(reinterpret_cast<void*>(c.resource.handle));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (ImGui::RadioButton("##use", g_override == c.resource.handle)) g_override = c.resource.handle;
                if (c.resource.handle == g_selected) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.3f, 1, 0.4f, 1), "captured");
                }
                ImGui::TableNextColumn();
                ImGui::Text("%ux%u%s", c.desc.texture.width, c.desc.texture.height,
                            c.desc.texture.samples > 1 ? " MSAA" : (c.fits_frame ? "" : " (off-size)"));
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(format_name(c.desc.texture.format));
                ImGui::TableNextColumn();
                ImGui::Text("%u", c.stats.drawcalls);
                ImGui::TableNextColumn();
                ImGui::Text("%u", c.stats.vertices);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }
    if (changed) g_settings.save();
}

void register_active_handlers() {
    reshade::register_event<reshade::addon_event::destroy_device>(on_destroy_device);
    reshade::register_event<reshade::addon_event::present>(on_present);
    reshade::register_overlay("LiDAR", draw_overlay);
}

}  // namespace

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    switch (reason) {
        case DLL_PROCESS_ATTACH:
            if (!reshade::register_addon(module)) return FALSE;
            // Everything else registers in attach_modules(), for supported devices only.
            reshade::register_event<reshade::addon_event::init_device>(on_init_device);
            break;
        case DLL_PROCESS_DETACH:
            reshade::unregister_addon(module);
            break;
    }
    return TRUE;
}
