#pragma once
#include <stdint.h>

// The cycle counter. Each read advances simulated time by one CPU cycle, so
// the step-pulse spin really takes its 2µs of sim time.
uint32_t sim_ccount();
#define XTHAL_GET_CCOUNT() sim_ccount()
