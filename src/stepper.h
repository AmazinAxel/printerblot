#pragma once

#include <Arduino.h>
#include "config.h"
#include "planner.h"   // plan_block_t, for stepper_active_block()

// Stepper motion executor. Drives STEP pins from the planner's Cartesian
// velocity profile using variable-period hardware alarms and Bresenham for
// CoreXY motor mixing.

void stepper_init();
void stepper_reset();                // flush on soft reset

// Kick the ISR if idle. Call after queueing new blocks.
void stepper_wake();

bool stepper_is_idle();
bool stepper_is_in_hold();

// The block the ISR is currently stepping, or nullptr if none. The planner
// uses this to leave the in-flight block alone: its velocity ramp is already
// partly executed, so rewriting initial_rate/accelerate_until underneath the
// ISR produces a discontinuity in the middle of a move.
const plan_block_t *stepper_active_block();

// Realtime motion controls
void stepper_feed_hold();            // decel to zero, retain buffer
void stepper_cycle_start();          // resume after hold

// The main loop calls this every iteration; if the ISR is waiting on a
// sync command (servo, dwell), the main loop handles it and wakes the ISR.
void stepper_service_sync();

// Machine position (motor steps). Updated by the ISR as it emits pulses.
// Read from main loop only — treat as best-effort snapshot.
void stepper_get_machine_position_mm(float *x, float *y);

// For status reports and flow control hints.
uint32_t stepper_current_rate_events_per_sec();

