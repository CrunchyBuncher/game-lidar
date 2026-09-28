// lidar_tests [--list] [group | group.name ...]: runs the registered unit tests, all of them by
// default. Exits non-zero if any check failed, or if a filter matched nothing.
#include <cstring>
#include <string>
#include <vector>

#include "test.h"

namespace lidar::test {

namespace {
struct Test {
    const char* group;
    const char* name;
    TestFn fn;
};
std::vector<Test>& registry() {
    static std::vector<Test> tests;
    return tests;
}
}  // namespace

Registrar::Registrar(const char* group, const char* name, TestFn fn) { registry().push_back({group, name, fn}); }

}  // namespace lidar::test

int main(int argc, char** argv) {
    using namespace lidar::test;
    std::vector<std::string> filters;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--list") == 0) {
            for (const Test& t : registry()) std::printf("%s.%s\n", t.group, t.name);
            return 0;
        }
        filters.push_back(argv[i]);
    }
    std::vector<bool> used(filters.size());
    int ran = 0;
    for (const Test& t : registry()) {
        const std::string full = std::string(t.group) + "." + t.name;
        bool wanted = filters.empty();
        for (size_t i = 0; i < filters.size(); ++i)
            if (filters[i] == t.group || filters[i] == full) wanted = used[i] = true;
        if (!wanted) continue;
        const int before = g_failures;
        t.fn();
        ++ran;
        std::printf("%s %s\n", g_failures == before ? "ok    " : "FAILED", full.c_str());
    }
    for (size_t i = 0; i < filters.size(); ++i)
        if (!used[i]) {
            std::printf("no test matches '%s' (see --list)\n", filters[i].c_str());
            ++g_failures;
        }
    if (g_failures == 0) std::printf("all %d tests passed\n", ran);
    return g_failures == 0 ? 0 : 1;
}
