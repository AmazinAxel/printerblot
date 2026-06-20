#pragma once

#include <Arduino.h>

// ───── Hardware pins (XIAO RP2040 + A4988 + servo) ─────
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
#define DEFAULT_ARC_TOLERANCE            0.002f

// ───── Work envelope ─────
// Physical Blot work area: 125 × 125 mm. Origin (0, 0) is the bottom-left
// corner, +X right, +Y up. Homing is manual — user positions the carriage
// at the desired origin and sends `G92 X0 Y0`.
#define DEFAULT_MAX_X_MM   125.0f
#define DEFAULT_MAX_Y_MM   125.0f

// ───── Servo / pen ─────
// Standard position servo: 1000 = one end, 1500 = center, 2000 = other end.
#define DEFAULT_PEN_UP_US        1000
#define DEFAULT_PEN_DOWN_US      1350
#define DEFAULT_PEN_MOVE_MS        60   // time for servo to reach target position

// ───── Planner ─────
#define PLANNER_BUFFER_SIZE   16        // number of blocks — must be power of 2

// ───── Stepper ISR ─────
// Minimum step pulse HIGH time for A4988 (datasheet: 1 µs). We hold 2 µs.
#define STEP_PULSE_US         2
// Guard against sub-microsecond ISR re-entry; minimum scheduled interval.
#define MIN_STEP_INTERVAL_US  25        // → max ~40 kHz step events / 500 mm/s

// ───── Serial / protocol ─────
#define SERIAL_BAUD         115200
#define RX_BUFFER_SIZE       128         // GRBL convention
#define LINE_BUFFER_SIZE     96

// ───── GRBL-style error codes (subset we actually emit) ─────
enum StatusCode : uint8_t {
    STATUS_OK                  = 0,
    STATUS_EXPECTED_COMMAND    = 1,
    STATUS_BAD_NUMBER_FORMAT   = 2,
    STATUS_INVALID_STATEMENT   = 3,
    STATUS_NEGATIVE_VALUE      = 4,
    STATUS_SETTING_DISABLED    = 5,
    STATUS_UNSUPPORTED_COMMAND = 20,
    STATUS_MODAL_GROUP_CONFLICT= 21,
    STATUS_INVALID_TARGET      = 31,
    STATUS_ARC_RADIUS_ERROR    = 33,
    STATUS_OVERFLOW            = 60,
};

// Realtime command bytes (intercepted before line-buffering)
#define RT_STATUS_REPORT   '?'
#define RT_FEED_HOLD       '!'
#define RT_CYCLE_START     '~'
#define RT_SOFT_RESET      0x18   // ctrl-X
#define RT_JOG_CANCEL      0x85   // GRBL 1.1: abort any in-flight jog
