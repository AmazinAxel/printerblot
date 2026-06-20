#pragma once

#include <cstdint>

#define TIMER_IRQ_0 0
#define TIMER_IRQ_1 1
#define TIMER_IRQ_2 2
#define TIMER_IRQ_3 3

// Minimal simulated-timer peripheral. The host advances timerawl; the
// firmware writes target fire times into alarm[]. Tests use these together
// to know when the ISR is "due".
typedef struct {
    uint32_t timerawl;
    uint32_t alarm[4];
    uint32_t intr;
    uint32_t inte;
} timer_hw_t;

extern timer_hw_t *const timer_hw;

inline void hw_clear_bits(volatile uint32_t *p, uint32_t bits) { *p &= ~bits; }
inline void hw_set_bits  (volatile uint32_t *p, uint32_t bits) { *p |=  bits; }

inline void hardware_alarm_claim(unsigned int) {}
