// Minimal Win32 window + D3D11 device/swap chain shared by the fake game and viewer.
#pragma once
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi1_2.h>
#include <windows.h>
#include <wrl/client.h>

#include <string>

namespace lidar {

using Microsoft::WRL::ComPtr;

class App {
public:
    // d3d11 = false: window only, for callers that create their own device (the D3D12 fake game).
    bool create(const wchar_t* title, int width, int height, bool d3d11 = true);
    // Processes pending window messages. Returns false once the window closed.
    bool pump();
    void present(bool vsync);  // D3D11 only
    void set_title(const std::wstring& title);

    bool key_down(int vk) const { return keys_[vk & 0xFF]; }
    bool key_pressed(int vk) const { return pressed_[vk & 0xFF]; }  // this pump only
    bool rmb_down() const { return rmb_; }

    // Sees every window message first (e.g. a UI). Returns true to consume it, with `result`
    // as the message's result: the App's own input state doesn't see it then.
    using MessageHook = bool (*)(HWND, UINT, WPARAM, LPARAM, LRESULT& result);
    MessageHook message_hook = nullptr;

    HWND hwnd = nullptr;
    int width = 0, height = 0;
    bool resized = false;  // set by pump() when the window (and the D3D11 back buffer) was resized
    float mouse_dx = 0, mouse_dy = 0, wheel = 0;  // accumulated during the last pump

    ComPtr<ID3D11Device> dev;
    ComPtr<ID3D11DeviceContext> ctx;
    ComPtr<IDXGISwapChain1> swap;
    ComPtr<ID3D11Texture2D> back_buffer;
    ComPtr<ID3D11RenderTargetView> back_rtv;

private:
    static LRESULT CALLBACK wnd_proc(HWND, UINT, WPARAM, LPARAM);
    LRESULT handle(UINT msg, WPARAM wp, LPARAM lp);
    void create_back_buffer();

    bool keys_[256] = {};
    bool pressed_[256] = {};
    bool rmb_ = false;
    bool quit_ = false;
    bool pending_resize_ = false;
};

// Compiles HLSL source; on failure prints the error log and exits.
ComPtr<ID3DBlob> compile_shader(const char* src, const char* entry, const char* target);

// Aborts with a message if hr failed.
void check(HRESULT hr, const char* what);

}  // namespace lidar
