// A minimal test harness: TEST_CASE(group, name) registers a test, EXPECT(cond) records a failure
// and carries on. lidar_tests runs every test, or only the groups / group.name tests on its command
// line (CTest runs one group per test, see CMakeLists.txt).
#pragma once
#include <cstdio>

namespace lidar::test {

inline int g_failures = 0;

using TestFn = void (*)();
struct Registrar {
    Registrar(const char* group, const char* name, TestFn fn);
};

}  // namespace lidar::test

#define EXPECT(cond)                                                    \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++::lidar::test::g_failures;                                \
        }                                                               \
    } while (0)

#define TEST_CASE(group, name)                                                                \
    static void test_##group##_##name();                                                      \
    static const ::lidar::test::Registrar reg_##group##_##name(#group, #name, test_##group##_##name); \
    static void test_##group##_##name()
