#include "test.h"

// Shared counters.
int tests_failed_current = 0;
int tests_run    = 0;
int tests_passed = 0;
int tests_failed = 0;
int checks_passed = 0;
int checks_failed = 0;

std::vector<TestCase> &test_registry() {
    static std::vector<TestCase> reg;
    return reg;
}

int run_all_tests() {
    auto &reg = test_registry();
    std::printf("Running %zu test cases...\n\n", reg.size());

    for (const auto &tc : reg) {
        tests_run++;
        tests_failed_current = 0;
        tc.fn();
        if (tests_failed_current == 0) {
            tests_passed++;
            std::printf("  ok   %s\n", tc.name);
        } else {
            tests_failed++;
            std::printf("  FAIL %s\n", tc.name);
        }
    }

    std::printf("\n-------------------------------------------\n");
    std::printf("Tests:   %d total   %d passed   %d failed\n",
                tests_run, tests_passed, tests_failed);
    std::printf("Checks:  %d total   %d passed   %d failed\n",
                checks_passed + checks_failed, checks_passed, checks_failed);
    return tests_failed == 0 ? 0 : 1;
}

int main() {
    return run_all_tests();
}
