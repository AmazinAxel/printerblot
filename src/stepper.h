#pragma once

#include <Arduino.h>
#include "config.h"
#include "planner.h"   // plan_block_t, for stepper_is_executing()

// Stepper motion executor. Drives STEP pins from the planner's Cartesian
// velocity profile using variable-period hardware alarms and Bresenham for
// CoreXY motor mixing. Wakes the loop (wake.h) whenever a planner slot
// frees, a pen block is reached, or motion stops.

void stepper_init();
void stepper_reset();                // flush: drop the plan, keep position

// Start executing queued blocks if idle (restores the full CPU clock first).
// Call after queueing.
void stepper_wake();
void stepper_power_save();           // drop to 80MHz if fully idle

bool stepper_is_idle();
// Motors at rest: idle, held, or parked on a pen block.
bool stepper_is_stopped();

void stepper_feed_hold();            // decel to zero, retain buffer

// Loop side of pen blocks: moves the pen, then releases the block once it
// has arrived. Call on every wake.
void stepper_service();

// A4988 enable, written directly: only ever changed while no motion is
// queued (job start, idle release), so it can't race the planner.
void stepper_set_drivers(bool on);
bool stepper_drivers_on();

// Machine position (motor steps). Updated by the ISR as it emits pulses.
// Read from main loop only — treat as best-effort snapshot.
void stepper_get_machine_position_mm(float *x, float *y);

// True if the ISR is executing `b` right now (mid-block, including a hold).
// Its velocity ramp is already partly executed, so the planner must not
// change its entry speed — that would step the velocity discontinuously.
// If events_done is non-null it receives the events already emitted.
// Call with interrupts disabled for a consistent answer.
bool stepper_is_executing(const plan_block_t *b, uint32_t *events_done);
