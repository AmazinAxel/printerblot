#pragma once

#include <Arduino.h>

// Code reached from the stepper ISR must live in RAM — a flash cache miss
// (triggered by WiFi or LittleFS activity on the other core) would otherwise
// stall step generation and show up as a stutter in the plot.
#include "esp_attr.h"
#include "soc/gpio_struct.h"
#include "xtensa/core-macros.h"

// Arduino's digitalWrite() and delayMicroseconds() live in FLASH on this core
// (verified: 0x420116e8 and 0x42011768, outside the IRAM window). Calling
// them from an ESP_INTR_FLAG_IRAM handler panics with "Cache disabled but
// cached memory region accessed" as soon as anything writes to flash — which
// LittleFS and the WiFi stack both do. The ISR must use these instead.

// Write-1-to-set / write-1-to-clear: no read-modify-write, no races.
static inline void IRAM_ATTR isr_pin_write(uint8_t pin, bool level) {
    if (pin < 32) {
        if (level) GPIO.out_w1ts = (1u << pin);
        else       GPIO.out_w1tc = (1u << pin);
    } else {
        if (level) GPIO.out1_w1ts.val = (1u << (pin - 32));
        else       GPIO.out1_w1tc.val = (1u << (pin - 32));
    }
}

// Spin until `us` have passed since cycle count `start` (XTHAL_GET_CCOUNT()).
// The firmware's one deliberate wait: the remainder of the A4988's minimum
// STEP high time, after the ISR has done its math during the pulse. F_CPU is
// 240MHz whenever steps run; the count wraps but unsigned subtraction
// handles it.
static inline void IRAM_ATTR isr_delay_us_since(uint32_t start, uint32_t us) {
    uint32_t cycles = us * (F_CPU / 1000000);
    while ((XTHAL_GET_CCOUNT() - start) < cycles) { }
}

// ───── Hardware pins (XIAO ESP32-S3 + A4988 + servo) ─────
// The D0-D10 aliases map to the same physical pads as the RP2040 XIAO this
// replaced, so the Blot carrier board works unchanged.
#define PIN_MOTOR1_STEP  D10
#define PIN_MOTOR1_DIR   D9
#define PIN_MOTOR2_STEP  D8
#define PIN_MOTOR2_DIR   D7
#define PIN_DRIVER_EN    D1
#define PIN_SERVO        D6

// ───── Mechanical constants ─────
// 200 full steps/rev * 16 microsteps / (20 teeth * 2mm pitch) = 80 steps/mm
// This is motor-space steps-per-mm (not Cartesian). Both motors use the
// same value because they're identical.
#define STEPS_PER_MM     80.0f

// ───── Motor physical limits ─────
#define MOTOR_MAX_RATE_MM_MIN  16402.0f   // mm/min, motor-space
#define MOTOR_MAX_ACCEL_MM_S2  16606.0f   // mm/s², motor-space

// ───── Cartesian defaults ─────
#define DEFAULT_MAX_FEEDRATE_MM_MIN  16402.0f   // mm/min
#define DEFAULT_MAX_RAPID_MM_MIN     16401.0f   // mm/min
#define DEFAULT_MAX_ACCEL_MM_S2      16606.0f   // mm/s²
#define DEFAULT_JUNCTION_DEVIATION       0.05f

// ───── Work envelope ─────
// Physical Blot work area: 125 × 125 mm, measured from wherever the pen sat
// at power-on (machine 0,0). Jobs set their own origin with `G92 X0 Y0`.
#define DEFAULT_MAX_X_MM   125.0f
#define DEFAULT_MAX_Y_MM   125.0f

// ───── Servo / pen ─────
// Standard position servo: 1000 = one end, 1500 = center, 2000 = other end.
#define DEFAULT_PEN_UP_US        1000
#define DEFAULT_PEN_DOWN_US      1350
#define DEFAULT_PEN_MOVE_MS        60   // time for servo to reach target position

// ───── Planner ─────
#define PLANNER_BUFFER_SIZE   16        // number of blocks — must be power of 2
// Most planner blocks one G-code line can queue: two pen commands (the
// tokenizer allows at most two M-codes) plus one move. job_service() only
// feeds a line when this many slots are free, so queueing never fails for
// lack of room and nothing ever has to wait for a slot.
#define MAX_BLOCKS_PER_LINE    3

// ───── Stepper ISR ─────
// Minimum step pulse HIGH time for A4988 (datasheet: 1 µs). We hold 2 µs.
#define STEP_PULSE_US         2
// Guard against sub-microsecond ISR re-entry; minimum scheduled interval.
#define MIN_STEP_INTERVAL_US  25        // → max ~40 kHz step events / 500 mm/s

// ───── G-code ─────
#define LINE_BUFFER_SIZE     96

// What became of a G-code line. Only the job runner looks at this: OK and
// SKIPPED carry on, ERROR aborts the print.
enum StatusCode : uint8_t {
    STATUS_OK,
    STATUS_SKIPPED,  // unsupported but harmless (G17, M18, ...): no geometry
    STATUS_ERROR,    // malformed, out of bounds, or would misplace geometry
};
