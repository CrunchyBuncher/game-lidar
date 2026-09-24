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

add_executable(fake_game tools/fake_game/main.cpp)
target_link_libraries(fake_game PRIVATE lidar_common)

add_executable(lidar_viewer viewer/main.cpp)
target_link_libraries(lidar_viewer PRIVATE lidar_common)

add_executable(lidar_verify tools/verify/main.cpp)
target_include_directories(lidar_verify PRIVATE tools/fake_game)
target_link_libraries(lidar_verify PRIVATE lidar_common)

enable_testing()
add_executable(lidar_tests tests/test_main.cpp)
target_link_libraries(lidar_tests PRIVATE lidar_common)
add_test(NAME lidar_tests COMMAND lidar_tests)
