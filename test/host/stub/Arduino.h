// Host stand-in for the Arduino/ESP-IDF surface the firmware uses. Hardware
// effects (pins, timers, flash, CPU clock) are routed into sim.cpp, which
// models time and checks every edge.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#define IRAM_ATTR
#define F_CPU 240000000L
#define OUTPUT 1
#define HIGH 1
#define LOW 0

static const uint8_t LED_BUILTIN = 21;
static const uint8_t D1 = 2;
static const uint8_t D6 = 43;
static const uint8_t D7 = 44;
static const uint8_t D8 = 7;
static const uint8_t D9 = 8;
static const uint8_t D10 = 9;

// sim.cpp
void sim_pin_write(uint8_t pin, bool level);
bool sim_pin_read(uint8_t pin);
void sim_set_cpu_mhz(uint32_t mhz);

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t v) { sim_pin_write(pin, v); }
inline int digitalRead(uint8_t pin) { return sim_pin_read(pin); }
inline bool setCpuFrequencyMhz(uint32_t mhz) { sim_set_cpu_mhz(mhz); return true; }
// Interrupts never preempt the loop inside these sections in the sim (ISRs
// only fire at sim-chosen points), so they are no-ops.
inline void noInterrupts() {}
inline void interrupts() {}

// GPIO driver bits penservo.cpp uses.
typedef int gpio_num_t;
enum { GPIO_MODE_OUTPUT = 1, GPIO_PULLUP_DISABLE = 0, GPIO_PULLDOWN_DISABLE = 0,
       GPIO_INTR_DISABLE = 0 };
struct gpio_config_t {
    uint64_t pin_bit_mask; int mode, pull_up_en, pull_down_en, intr_type;
};
inline int gpio_config(const gpio_config_t *) { return 0; }
inline int gpio_set_level(gpio_num_t pin, uint32_t v) { sim_pin_write(pin, v); return 0; }
