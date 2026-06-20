// Minimal test framework. No dependencies, no macros beyond the basics.
//
// Usage:
//   TEST_CASE("kinematics: axis-aligned") {
//       CHECK(some_bool);
//       CHECK_EQ(a, b);
//       CHECK_NEAR(x, y, tol);
//   }
//
// test_main.cpp calls RUN_ALL_TESTS() which executes every registered case
// and prints a summary. Nonzero exit status if any assertion failed.

#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

struct TestCase {
    const char *name;
    void (*fn)();
};

std::vector<TestCase> &test_registry();

struct TestRegistrar {
    TestRegistrar(const char *name, void (*fn)()) {
        test_registry().push_back({name, fn});
    }
};

extern int tests_failed_current;    // incremented on CHECK failure within a test
extern int tests_run;
extern int tests_passed;
extern int tests_failed;
extern int checks_passed;
extern int checks_failed;

#define _TEST_CAT(a, b) a##b
#define _TEST_CAT2(a, b) _TEST_CAT(a, b)

#define TEST_CASE(name)                                                 \
    static void _TEST_CAT2(test_fn_, __LINE__)();                       \
    static TestRegistrar _TEST_CAT2(reg_, __LINE__)(name, _TEST_CAT2(test_fn_, __LINE__)); \
    static void _TEST_CAT2(test_fn_, __LINE__)()

#define FAIL_LOG(msg)                                                   \
    do {                                                                \
        tests_failed_current++;                                         \
        checks_failed++;                                                \
        std::fprintf(stderr, "    FAIL %s:%d: %s\n", __FILE__, __LINE__, msg); \
    } while (0)

#define PASS_LOG() do { checks_passed++; } while (0)

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) FAIL_LOG(#cond);                                   \
        else         PASS_LOG();                                        \
    } while (0)

#define CHECK_EQ(a, b)                                                  \
    do {                                                                \
        auto _aa = (a); auto _bb = (b);                                 \
        if (!(_aa == _bb)) {                                            \
            char buf[160];                                              \
            std::snprintf(buf, sizeof(buf), "%s == %s  (lhs=%lld rhs=%lld)", \
                          #a, #b, (long long)_aa, (long long)_bb);      \
            FAIL_LOG(buf);                                              \
        } else PASS_LOG();                                              \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                           \
    do {                                                                \
        double _aa = (double)(a); double _bb = (double)(b);             \
        if (std::fabs(_aa - _bb) > (double)(tol)) {                     \
            char buf[160];                                              \
            std::snprintf(buf, sizeof(buf), "|%s - %s| <= %s  (lhs=%g rhs=%g)", \
                          #a, #b, #tol, _aa, _bb);                      \
            FAIL_LOG(buf);                                              \
        } else PASS_LOG();                                              \
    } while (0)

#define CHECK_FALSE(cond) CHECK(!(cond))

int run_all_tests();
