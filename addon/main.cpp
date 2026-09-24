// lidar_capture.addon64: ReShade addon that captures the scene depth buffer (and, from M2,
// the game's camera matrices) and publishes them to the shared-memory ring for the viewer.
//
// M1: depth only. Frames go out camera-relative (no pose flag) with a projection built from
// the overlay's FOV / near / far / depth-mode settings, which the viewer uses to unproject.
#include <imgui.h>
#include <reshade.hpp>

#include <DirectXMath.h>
#include <d3d11.h>

#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "capture_d3d11.h"
#include "depth_tracker.h"
#include "ring.h"

using namespace reshade::api;
using namespace lidar;

extern "C" __declspec(dllexport) const char* NAME = "game-lidar capture";
extern "C" __declspec(dllexport) const char* DESCRIPTION =
    "Captures depth and camera matrices and streams them to lidar_viewer.";

namespace {

constexpr char kSection[] = "LIDAR";

enum class DepthMode : int { Standard = 0, Reversed = 1, ReversedInfinite = 2 };

struct Settings {
    bool enabled = true;
    int capture_width = 480;
    // Projection used until the camera sniffer (M2) provides the real one. Defaults match fake_game.
    float fov_y_deg = 70.0f;
    float near_z = 0.1f;
    float far_z = 1000.0f;
    int depth_mode = int(DepthMode::Reversed);

    void load() {
        reshade::get_config_value(nullptr, kSection, "Enabled", enabled);
        reshade::get_config_value(nullptr, kSection, "CaptureWidth", capture_width);
        reshade::get_config_value(nullptr, kSection, "FovY", fov_y_deg);
        reshade::get_config_value(nullptr, kSection, "Near", near_z);
        reshade::get_config_value(nullptr, kSection, "Far", far_z);
        reshade::get_config_value(nullptr, kSection, "DepthMode", depth_mode);
    }
    void save() const {
        reshade::set_config_value(nullptr, kSection, "Enabled", enabled);
        reshade::set_config_value(nullptr, kSection, "CaptureWidth", capture_width);
        reshade::set_config_value(nullptr, kSection, "FovY", fov_y_deg);
        reshade::set_config_value(nullptr, kSection, "Near", near_z);
        reshade::set_config_value(nullptr, kSection, "Far", far_z);
        reshade::set_config_value(nullptr, kSection, "DepthMode", depth_mode);
    }
};

// Touched from the game's render thread (present) and the overlay, which ReShade draws on the
// same thread; the mutex covers games that differ.
std::mutex g_mutex;
Settings g_settings;
device* g_device = nullptr;  // the D3D11 device we capture from (first one created)
D3D11Capture g_capture;
RingWriter g_ring;
std::string g_init_error;
uint64_t g_frame = 0;

std::vector<depth::Candidate> g_candidates;  // last frame's depth-stencils, best first
uint64_t g_selected = 0;                     // handle captured last frame
uint64_t g_override = 0;                     // manual pick from the overlay, 0 = auto
double g_cpu_us = 0;                         // smoothed CPU cost of our present work

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

void on_init_device(device* dev) {
    if (dev->get_api() != device_api::d3d11) return;
    const std::lock_guard lock(g_mutex);
    if (g_device != nullptr) return;
    g_device = dev;
    g_settings.load();
    if (!g_capture.init(reinterpret_cast<ID3D11Device*>(dev->get_native())))
        g_init_error = "capture init failed: " + g_capture.error();
    else if (!g_ring.open())
        g_init_error = "failed to create the shared-memory ring";
}

void on_destroy_device(device* dev) {
    const std::lock_guard lock(g_mutex);
    if (dev != g_device) return;
    g_capture.release();
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

    const resource_desc bb = dev->get_resource_desc(sc->get_current_back_buffer());
    std::vector<depth::Candidate> candidates = depth::end_frame(dev, bb.texture.width, bb.texture.height);
    if (candidates.empty()) return;  // e.g. a second present without any rendering in between
    g_candidates = std::move(candidates);

    // Native D3D11 calls bypass ReShade's hooks, so our own work isn't counted as game draws.
    auto* ctx = reinterpret_cast<ID3D11DeviceContext*>(queue->get_native());
    g_capture.publish(ctx, g_ring);

    const depth::Candidate* pick = nullptr;
    for (const auto& c : g_candidates)
        if (c.resource.handle == g_override) pick = &c;
    if (pick == nullptr && g_candidates.front().fits_frame) pick = &g_candidates.front();
    g_selected = pick ? pick->resource.handle : 0;

    if (g_settings.enabled && pick != nullptr) {
        FrameHeader h{};
        h.frame_index = g_frame;
        h.timestamp_qpc = uint64_t(t0.QuadPart);
        h.flags = 0;  // no pose until M2: camera-relative
        const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        std::memcpy(h.view, identity, sizeof(h.view));
        make_proj(g_settings, float(pick->desc.texture.width) / float(pick->desc.texture.height), h.proj);
        g_capture.capture(ctx, reinterpret_cast<ID3D11Resource*>(pick->resource.handle),
                          uint32_t(g_settings.capture_width), h);
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

void draw_overlay(effect_runtime*) {
    const std::lock_guard lock(g_mutex);
    if (g_device == nullptr) {
        ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "No D3D11 device. Only D3D11 is supported for now.");
        return;
    }
    if (!g_init_error.empty()) {
        ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", g_init_error.c_str());
        return;
    }

    bool changed = ImGui::Checkbox("Capture", &g_settings.enabled);
    ImGui::SameLine();
    ImGui::TextDisabled("(%.1f fps, addon CPU %.0f us/frame)", ImGui::GetIO().Framerate, g_cpu_us);
    if (!g_capture.error().empty()) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "%s", g_capture.error().c_str());
    ImGui::Text("Published %llu frames at %ux%u, skipped %llu (GPU readback busy)",
                static_cast<unsigned long long>(g_capture.published()), g_capture.width(), g_capture.height(),
                static_cast<unsigned long long>(g_capture.skipped()));
    changed |= ImGui::SliderInt("Capture width", &g_settings.capture_width, 64, int(kMaxWidth));

    if (ImGui::CollapsingHeader("Projection (until camera sniffing in M2)", ImGuiTreeNodeFlags_DefaultOpen)) {
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

}  // namespace

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    switch (reason) {
        case DLL_PROCESS_ATTACH:
            if (!reshade::register_addon(module)) return FALSE;
            // The tracker's init_device must run first: it creates the per-device data.
            depth::register_events();
            reshade::register_event<reshade::addon_event::init_device>(on_init_device);
            reshade::register_event<reshade::addon_event::destroy_device>(on_destroy_device);
            reshade::register_event<reshade::addon_event::present>(on_present);
            reshade::register_overlay("LiDAR", draw_overlay);
            break;
        case DLL_PROCESS_DETACH:
            reshade::unregister_addon(module);
            break;
    }
    return TRUE;
}
