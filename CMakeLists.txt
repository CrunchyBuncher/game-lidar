cmake_minimum_required(VERSION 3.24)
project(game_lidar LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_compile_definitions(UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
if(MSVC)
  # C4324: padding from alignas in the shared-memory structs is intentional.
  add_compile_options(/W4 /utf-8 /permissive- /wd4324)
endif()

# Shared: protocol, shared-memory ring, window/D3D11 helpers.
add_library(lidar_common STATIC common/ring.cpp common/app.cpp)
target_include_directories(lidar_common PUBLIC common)
target_link_libraries(lidar_common PUBLIC d3d11 dxgi d3dcompiler)

add_executable(fake_game tools/fake_game/main.cpp tools/fake_game/render_d3d11.cpp tools/fake_game/render_d3d12.cpp
  tools/fake_game/render_d3d9.cpp)
target_link_libraries(fake_game PRIVATE lidar_common d3d12 d3d9)
# Own folder outside build/ so ReShade's dxgi.dll next to it doesn't hook the viewer and
# survives clean rebuilds. The $<1:> stops VS from appending a per-config subdirectory.
set_target_properties(fake_game PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}/sandbox/fake_game>)
# D3D9 needs ReShade as d3d9.dll, in a folder of its own: next to the dxgi.dll copy it would load twice.
set(FAKE_GAME_D3D9_DIR ${CMAKE_SOURCE_DIR}/sandbox/fake_game_d3d9)
add_custom_command(TARGET fake_game POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${FAKE_GAME_D3D9_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fake_game> ${FAKE_GAME_D3D9_DIR}/)

add_executable(lidar_viewer viewer/main.cpp)
target_link_libraries(lidar_viewer PRIVATE lidar_common)

add_executable(lidar_verify tools/verify/main.cpp)
target_include_directories(lidar_verify PRIVATE tools/fake_game)
target_link_libraries(lidar_verify PRIVATE lidar_common)

# ReShade addon. SDK headers are pinned to the installed ReShade (6.8.0 => API 20), and ImGui
# to the exact commit that release was built with (reshade_overlay.hpp checks IMGUI_VERSION_NUM).
include(FetchContent)
FetchContent_Declare(reshade_sdk
  URL https://github.com/crosire/reshade/archive/refs/tags/v6.8.0.zip
  SOURCE_SUBDIR none)  # headers only: don't add ReShade's own CMake project
FetchContent_Declare(reshade_imgui
  URL https://github.com/ocornut/imgui/archive/3912b3d9a9c1b3f17431aebafd86d2f40ee6e59c.zip
  SOURCE_SUBDIR none)
FetchContent_MakeAvailable(reshade_sdk reshade_imgui)

add_library(lidar_capture SHARED
  addon/main.cpp addon/backends.cpp addon/depth_tracker.cpp addon/camera_tracker.cpp addon/profile.cpp addon/watchdog.cpp
  addon/camera_math.cpp addon/discovery.cpp
  addon/d3d11/capture_d3d11.cpp addon/d3d11/cbuffer_d3d11.cpp
  addon/d3d12/capture_d3d12.cpp addon/d3d12/cbuffer_d3d12.cpp addon/d3d12/root_layout.cpp
  addon/d3d9/capture_d3d9.cpp addon/d3d9/constants_d3d9.cpp)
target_include_directories(lidar_capture PRIVATE addon ${reshade_sdk_SOURCE_DIR}/include ${reshade_imgui_SOURCE_DIR})
target_compile_definitions(lidar_capture PRIVATE ImTextureID=ImU64)
target_link_libraries(lidar_capture PRIVATE lidar_common delayimp)
# Only load the shader compiler once a capture actually starts, not in every game the addon is in.
target_link_options(lidar_capture PRIVATE /DELAYLOAD:d3dcompiler_47.dll)
# ReShade only looks for the suffix matching its own bitness (32-bit games need .addon32).
if(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set(ADDON_SUFFIX ".addon32")
else()
  set(ADDON_SUFFIX ".addon64")
endif()
set_target_properties(lidar_capture PROPERTIES PREFIX "" SUFFIX ${ADDON_SUFFIX})
# ReShade loads addons from the game's folder, so drop a copy next to both fake games, along with
# the profile under the name the addon looks for by default.
foreach(dir ${CMAKE_SOURCE_DIR}/sandbox/fake_game ${FAKE_GAME_D3D9_DIR})
  add_custom_command(TARGET lidar_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${dir}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:lidar_capture> ${dir}/
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/profiles/fake_game.toml ${dir}/lidar_profile.toml
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/profiles/fake_game_viewproj.toml ${dir}/)
endforeach()

enable_testing()
add_executable(lidar_tests tests/test_main.cpp addon/profile.cpp addon/camera_math.cpp addon/d3d12/root_layout.cpp)
target_include_directories(lidar_tests PRIVATE addon ${reshade_sdk_SOURCE_DIR}/include)
target_link_libraries(lidar_tests PRIVATE lidar_common)
add_test(NAME lidar_tests COMMAND lidar_tests)
