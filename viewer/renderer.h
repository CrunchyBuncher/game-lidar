// Draws a frame: the point cloud's points (as screen- or distance-sized quads) and lines (the
// player's camera and trail), into the back buffer with the viewer's reversed-Z depth buffer.
#pragma once
#include <DirectXMath.h>

#include <cstdint>
#include <vector>

#include "app.h"
#include "camera.h"
#include "cutaway.h"
#include "point_cloud.h"

namespace lidar {

struct LineVertex {
    DirectX::XMFLOAT3 pos;
    uint32_t color;
};

// How points look.
struct PointStyle {
    float point_size = 2.0f;  // pixels
    // Distance-scaled points: world_scale voxels wide, so they close up into surfaces up close
    // and shrink to single pixels far away, up to world_max_px.
    bool world_points = false;
    float world_scale = 1.5f, world_max_px = 64.0f;
    uint32_t color_mode = 0;  // 0 height, 1 captured color
    float height_min = -1.0f, height_max = 20.0f;  // the height color ramp's ends
};

// Everything a frame's draw depends on.
struct DrawParams {
    DirectX::XMFLOAT4X4 view_proj;  // capture frame -> clip: tilt * view * proj
    float fov_y = 1.0f;
    float height_axis[4] = {0, 1, 0, 0};  // see ViewTilt::height_axis
    float voxel = 0.1f;
    PointStyle style;
    Cutaways cut;
};

DrawParams make_draw_params(const ViewParams& view, const ViewTilt& tilt, const PointStyle& style, float voxel,
                            const Cutaways& cut);

// The player's camera frustum (2 m deep) and trail.
void player_lines(const PlayerCamera& player, bool frustum, bool trail, std::vector<LineVertex>& out);

class Renderer {
public:
    void create(ID3D11Device* dev, ID3D11DeviceContext* ctx, int width, int height);
    void resize(int width, int height);  // the depth buffer, after the window's

    // Clears the target and sets up the frame's draw state.
    void begin(ID3D11RenderTargetView* rtv, const DrawParams& p, uint32_t capacity);
    void draw_points(const PointCloud& cloud);
    void draw_lines(const std::vector<LineVertex>& lines);

private:
    ID3D11Device* dev_ = nullptr;
    ID3D11DeviceContext* ctx_ = nullptr;
    int width_ = 0, height_ = 0;
    ComPtr<ID3D11VertexShader> vs_points_, vs_lines_;
    ComPtr<ID3D11PixelShader> ps_;
    ComPtr<ID3D11InputLayout> line_layout_;
    ComPtr<ID3D11Buffer> draw_cb_, quad_ib_, line_vb_;
    ComPtr<ID3D11Texture2D> zbuf_;
    ComPtr<ID3D11DepthStencilView> zbuf_dsv_;
    ComPtr<ID3D11DepthStencilState> dss_;
    ComPtr<ID3D11RasterizerState> rs_;
};

}  // namespace lidar
