# Discovery: finding a game's camera without a profile (README.md). The analyzer core, recordings,
# the threaded service the addon runs, the lidar_discover replay tool and the module's own tests.
set(DISCOVERY_RECORDINGS ${CMAKE_CURRENT_SOURCE_DIR}/tests/recordings)

add_library(lidar_discovery STATIC analyzer.cpp recording.cpp replay.cpp discovery.cpp)
target_include_directories(lidar_discovery PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lidar_discovery PUBLIC lidar_camera)

add_executable(lidar_discover tools/lidar_discover.cpp)
target_link_libraries(lidar_discover PRIVATE lidar_discovery)
target_compile_definitions(lidar_discover PRIVATE LIDAR_DISCOVERY_RECORDINGS="${DISCOVERY_RECORDINGS}")

# One CTest entry per group (lidar_discovery_tests <group>).
set(DISCOVERY_TEST_GROUPS math analyzer recording recordings)
list(TRANSFORM DISCOVERY_TEST_GROUPS REPLACE "(.+)" "tests/\\1.cpp" OUTPUT_VARIABLE DISCOVERY_TEST_SOURCES)
add_executable(lidar_discovery_tests ${CMAKE_SOURCE_DIR}/tests/common/test_main.cpp tests/synthetic.cpp
  ${DISCOVERY_TEST_SOURCES})
target_include_directories(lidar_discovery_tests PRIVATE tests ${CMAKE_SOURCE_DIR}/tests/common)
target_link_libraries(lidar_discovery_tests PRIVATE lidar_discovery)
target_compile_definitions(lidar_discovery_tests PRIVATE LIDAR_DISCOVERY_RECORDINGS="${DISCOVERY_RECORDINGS}")
foreach(group ${DISCOVERY_TEST_GROUPS})
  add_test(NAME discovery.${group} COMMAND lidar_discovery_tests ${group})
endforeach()
