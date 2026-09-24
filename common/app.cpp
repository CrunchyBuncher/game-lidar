#include "app.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace lidar {

void check(HRESULT hr, const char* what) {
    if (FAILED(hr)) {
        std::fprintf(stderr, "%s failed: 0x%08lX\n", what, static_cast<unsigned long>(hr));
        std::exit(1);
    }
}

ComPtr<ID3DBlob> compile_shader(const char* src, const char* entry, const char* target) {
    ComPtr<ID3DBlob> code, errors;
    HRESULT hr = D3DCompile(src, std::strlen(src), entry, nullptr, nullptr, entry, target,
                            D3DCOMPILE_OPTIMIZATION_LEVEL3 | D3DCOMPILE_IEEE_STRICTNESS, 0, &code, &errors);
    if (FAILED(hr)) {
        std::fprintf(stderr, "shader %s (%s) failed:\n%s\n", entry, target,
                     errors ? static_cast<const char*>(errors->GetBufferPointer()) : "?");
        std::exit(1);
    }
    return code;
}

bool App::create(const wchar_t* title, int w, int h, bool d3d11) {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

    WNDCLASSEXW wc{sizeof(wc)};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.lpszClassName = L"game_lidar_window";
    RegisterClassExW(&wc);

    RECT r{0, 0, w, h};
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd = CreateWindowExW(0, wc.lpszClassName, title, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
                           r.right - r.left, r.bottom - r.top, nullptr, nullptr, wc.hInstance, this);
    if (!hwnd) {
        std::fprintf(stderr, "CreateWindowEx failed: %lu\n", GetLastError());
        return false;
    }

    RAWINPUTDEVICE rid{0x01, 0x02, 0, hwnd};  // generic desktop / mouse
    RegisterRawInputDevices(&rid, 1, sizeof(rid));

    RECT cr;
    GetClientRect(hwnd, &cr);
    width = cr.right - cr.left;
    height = cr.bottom - cr.top;
    if (!d3d11) {  // the caller brings its own device and swap chain
        ShowWindow(hwnd, SW_SHOW);
        return true;
    }

    UINT flags = 0;
#ifdef _DEBUG
    flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
    D3D_FEATURE_LEVEL fl = D3D_FEATURE_LEVEL_11_0;
    HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, flags, &fl, 1, D3D11_SDK_VERSION,
                                   &dev, nullptr, &ctx);
    if (FAILED(hr) && (flags & D3D11_CREATE_DEVICE_DEBUG)) {  // debug layer not installed
        hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, &fl, 1, D3D11_SDK_VERSION, &dev,
                               nullptr, &ctx);
    }
    check(hr, "D3D11CreateDevice");

    ComPtr<IDXGIDevice> dxgi_dev;
    ComPtr<IDXGIAdapter> adapter;
    ComPtr<IDXGIFactory2> factory;
    check(dev.As(&dxgi_dev), "IDXGIDevice");
    check(dxgi_dev->GetAdapter(&adapter), "GetAdapter");
    check(adapter->GetParent(IID_PPV_ARGS(&factory)), "IDXGIFactory2");

    DXGI_SWAP_CHAIN_DESC1 sd{};
    sd.Width = width;
    sd.Height = height;
    sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.SampleDesc.Count = 1;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.BufferCount = 2;
    sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    check(factory->CreateSwapChainForHwnd(dev.Get(), hwnd, &sd, nullptr, nullptr, &swap), "CreateSwapChain");
    factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);
    create_back_buffer();

    ShowWindow(hwnd, SW_SHOW);
    return true;
}

void App::create_back_buffer() {
    back_rtv.Reset();
    back_buffer.Reset();
    check(swap->GetBuffer(0, IID_PPV_ARGS(&back_buffer)), "GetBuffer");
    check(dev->CreateRenderTargetView(back_buffer.Get(), nullptr, &back_rtv), "CreateRenderTargetView");
}

bool App::pump() {
    std::memset(pressed_, 0, sizeof(pressed_));
    mouse_dx = mouse_dy = wheel = 0;
    resized = false;
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    if (pending_resize_ && width > 0 && height > 0) {
        pending_resize_ = false;
        if (swap) {
            ctx->OMSetRenderTargets(0, nullptr, nullptr);
            back_rtv.Reset();
            back_buffer.Reset();
            ctx->Flush();
            check(swap->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN, 0), "ResizeBuffers");
            create_back_buffer();
        }
        resized = true;
    }
    return !quit_;
}

void App::present(bool vsync) { swap->Present(vsync ? 1 : 0, 0); }

void App::set_title(const std::wstring& title) { SetWindowTextW(hwnd, title.c_str()); }

LRESULT CALLBACK App::wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_NCCREATE) {
        auto* cs = reinterpret_cast<CREATESTRUCTW*>(lp);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
        static_cast<App*>(cs->lpCreateParams)->hwnd = hwnd;  // CreateWindowEx hasn't returned yet
    }
    auto* app = reinterpret_cast<App*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (app) return app->handle(msg, wp, lp);
    return DefWindowProcW(hwnd, msg, wp, lp);
}

LRESULT App::handle(UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CLOSE:
            quit_ = true;
            return 0;
        case WM_SIZE: {
            const int w = LOWORD(lp), h = HIWORD(lp);
            if (w > 0 && h > 0 && (w != width || h != height)) {
                width = w;
                height = h;
                pending_resize_ = true;
            }
            return 0;
        }
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
            if (!(lp & (1 << 30))) pressed_[wp & 0xFF] = true;
            keys_[wp & 0xFF] = true;
            if (wp == VK_ESCAPE) quit_ = true;
            return 0;
        case WM_KEYUP:
        case WM_SYSKEYUP:
            keys_[wp & 0xFF] = false;
            return 0;
        case WM_KILLFOCUS:
            std::memset(keys_, 0, sizeof(keys_));
            rmb_ = false;
            ReleaseCapture();
            return 0;
        case WM_RBUTTONDOWN:
            rmb_ = true;
            SetCapture(hwnd);
            return 0;
        case WM_RBUTTONUP:
            rmb_ = false;
            ReleaseCapture();
            return 0;
        case WM_MOUSEWHEEL:
            wheel += float(GET_WHEEL_DELTA_WPARAM(wp)) / WHEEL_DELTA;
            return 0;
        case WM_INPUT: {
            RAWINPUT ri;
            UINT size = sizeof(ri);
            if (GetRawInputData(reinterpret_cast<HRAWINPUT>(lp), RID_INPUT, &ri, &size, sizeof(RAWINPUTHEADER)) !=
                    UINT(-1) &&
                ri.header.dwType == RIM_TYPEMOUSE && !(ri.data.mouse.usFlags & MOUSE_MOVE_ABSOLUTE)) {
                mouse_dx += float(ri.data.mouse.lLastX);
                mouse_dy += float(ri.data.mouse.lLastY);
            }
            break;
        }
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

}  // namespace lidar
