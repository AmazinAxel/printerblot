#pragma once

#include <Arduino.h>
#include "config.h"

// Commands that can be synchronized with the motion buffer. A sync command
// block is a zero-distance entry in the planner that fires its side-effect
// when the stepper reaches it. That way pen up/down and dwells happen at the
// correct point in the motion stream, not prematurely from the main loop.
enum SyncCmd : uint8_t {
    SYNC_NONE       = 0,
    SYNC_SERVO      = 1,  // sync_arg = pulse width in µs
    SYNC_DWELL      = 2,  // sync_arg = dwell ms
    SYNC_SET_ENABLE = 3,  // sync_arg = 1 enable, 0 disable
};

typedef struct {
    // Motor-space step deltas (signed). direction_bits packs the signs:
    // bit 0 = motor1 negative, bit 1 = motor2 negative.
    int32_t  steps[2];
    uint8_t  direction_bits;
    uint32_t step_event_count;      // = max(|steps[0]|, |steps[1]|)

    // Cartesian geometry of the move
    float millimeters;              // Cartesian length L (mm)
    float unit_vec[2];              // Cartesian unit direction (for junction deviation)

    // Cartesian velocity profile (mm/s). This is the authoritative profile.
    // Motor rates are derived from here by the stepper ISR via Bresenham.
    float nominal_speed;            // the block's commanded tip speed
    float entry_speed;              // tip speed at block start (set by lookahead)
    float max_entry_speed;          // cap from junction deviation
    float acceleration;             // mm/s², Cartesian, already clamped by motor constraint

    // Step-event-space profile (what the stepper ISR consumes).
    // Derived from the Cartesian profile via: rate_events = v_tip * step_event_count / L
    uint32_t initial_rate;          // step events / s at block entry
    uint32_t nominal_rate;          // step events / s at nominal speed
    uint32_t final_rate;            // step events / s at block exit
    uint32_t accel_st;              // step events / s² in step-event space
    uint32_t accelerate_until;      // step-event index where accel ends
    uint32_t decelerate_after;      // step-event index where decel begins

    // Synchronized side-effect (executed after this block's motion completes)
    uint8_t  sync_cmd;
    int32_t  sync_arg;

    // Flag: block needs its trapezoid recomputed
    bool recalculate_flag;
} plan_block_t;


void plan_init();
void plan_reset();                          // flush buffer; call on soft reset
bool plan_is_empty();
uint8_t plan_buffer_available();            // slots free (for host flow control hints)

// Linear Cartesian move. Returns true if queued, false if buffer full.
// feedrate is in mm/min (GRBL convention). 'rapid' uses the G0 feedrate cap.
bool plan_buffer_line(float x_target_mm, float y_target_mm,
                      float feedrate_mm_min, bool rapid);

// Queue a sync command as a zero-distance block. Guarantees the side-effect
// fires in-order relative to motion.
bool plan_sync_command(uint8_t cmd, int32_t arg);

// Return the block currently being executed by the stepper (at the tail of
// the ring buffer), or nullptr if empty.
plan_block_t *plan_get_current_block();

// Stepper calls this when it finishes a block.
void plan_discard_current_block();

// Planner's idea of where the tool is (mm, Cartesian). Updated on every queued
// line so successive moves compute correct deltas.
void plan_get_position(float *x, float *y);
void plan_set_position(float x, float y);   // for G92

// Stream-side tunables (exposed for settings.cpp)
extern float planner_max_feedrate_mm_min;
extern float planner_max_rapid_mm_min;
extern float planner_max_accel_mm_s2;       // Cartesian cap (user)
extern float planner_motor_max_accel_mm_s2; // Motor-space physical limit
extern float planner_motor_max_rate_mm_min; // Motor-space physical limit
extern float planner_junction_deviation_mm;
