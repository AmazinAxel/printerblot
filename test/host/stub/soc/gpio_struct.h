#pragma once
#include <stdint.h>

// Write-1-to-set/clear registers: every assignment becomes pin edges in sim.
void sim_gpio_mask(uint32_t mask, int bank, bool level);

struct SimW1 {
    int bank; bool level;
    void operator=(uint32_t mask) { sim_gpio_mask(mask, bank, level); }
};
struct SimW1Val { SimW1 val; };
struct SimGpio {
    SimW1 out_w1ts{0, true}, out_w1tc{0, false};
    SimW1Val out1_w1ts{{1, true}}, out1_w1tc{{1, false}};
};
inline SimGpio GPIO;
