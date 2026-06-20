#pragma once

#include <cstdint>

// Simulated-machine harness. Drives the real firmware's stepper ISR in
// software by advancing virtual time to the next scheduled alarm fire.
// Tests queue G-code, call sim_run_until_idle(), then assert on the
// resulting motor positions, pulse counts, and elapsed time.

// Reset the simulator and firmware to a clean power-on state. Call at the
// start of every test.
void sim_reset();

// Drive the stepper until the planner is empty and the stepper is idle.
// Budget ceiling prevents runaway in the case of logic bugs.
void sim_run_until_idle(uint64_t max_sim_us = 10ULL * 1000 * 1000);

// Motor positions in raw steps (signed). These accumulate from the ISR as
// it emits step pulses.
int32_t sim_motor_steps_m1();
int32_t sim_motor_steps_m2();

// Cartesian tool position computed from motor steps via CoreXY forward.
void sim_get_cartesian_mm(float *x, float *y);

// Pulse counts observed on STEP pins. Equivalent to |motor_steps|.
uint32_t sim_pulses_m1();
uint32_t sim_pulses_m2();

// Virtual time elapsed since last sim_reset (µs).
uint64_t sim_elapsed_us();

// Peak step-event rate observed during the run (events/s). Reflects the
// highest cruise rate reached across all blocks.
uint32_t sim_peak_rate_events_per_sec();

// Helper: feed a G-code line (without trailing newline) and capture status.
// Returns the status code from gcode_execute_line.
int sim_gcode(const char *line);
