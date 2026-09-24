// The one place that knows which graphics APIs have an implementation. Adding an API means
// adding its DepthCapture and CbufferSource here; nothing else in the addon changes.
#include <d3d11.h>

#include "cbuffer_source.h"
#include "d3d11/capture_d3d11.h"
#include "d3d11/cbuffer_d3d11.h"
#include "d3d12/capture_d3d12.h"
#include "d3d12/cbuffer_d3d12.h"
#include "depth_capture.h"

using namespace reshade::api;

namespace lidar {

std::unique_ptr<DepthCapture> create_depth_capture(device* dev, std::string& error) {
    switch (dev->get_api()) {
        case device_api::d3d11: {
            auto c = std::make_unique<D3D11Capture>();
            if (!c->init(reinterpret_cast<ID3D11Device*>(dev->get_native()))) {
                error = "capture init failed: " + c->error();
                return nullptr;
            }
            return c;
        }
        case device_api::d3d12: {
            auto c = std::make_unique<D3D12Capture>();
            if (!c->init(dev)) {
                error = "capture init failed: " + c->error();
                return nullptr;
            }
            return c;
        }
        default:
            error = "only D3D11 and D3D12 are supported for now";
            return nullptr;
    }
}

bool is_supported(device* dev) {
    return dev->get_api() == device_api::d3d11 || dev->get_api() == device_api::d3d12;
}

void register_capture_events() { D3D12Capture::register_events(); }
void unregister_capture_events() { D3D12Capture::unregister_events(); }
void init_capture_device(device* dev) { D3D12Capture::init_device(dev); }

namespace cam {

std::unique_ptr<CbufferSource> create_cbuffer_source(device* dev) {
    switch (dev->get_api()) {
        case device_api::d3d11: return d3d11::create(dev);
        case device_api::d3d12: return d3d12::create(dev);
        default: return nullptr;
    }
}

void register_source_events() {
    d3d11::register_events();
    d3d12::register_events();
}
void unregister_source_events() {
    d3d11::unregister_events();
    d3d12::unregister_events();
}
void init_source_device(device* dev) {
    d3d11::init_device(dev);
    d3d12::init_device(dev);
}

}  // namespace cam
}  // namespace lidar
