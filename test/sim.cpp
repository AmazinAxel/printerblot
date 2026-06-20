#include "sim.h"
#include "mocks/Arduino.h"
#include "mocks/EEPROM.h"
#include "mocks/hardware/timer.h"
#include "../src/config.h"
#include "../src/planner.h"
#include "../src/stepper.h"
#include "../src/gcode.h"
#include "../src/kinematics.h"
#include "../src/arc.h"
#include "../src/settings.h"

extern "C" void stepper_tick_for_test();
extern "C" bool stepper_test_is_running();
extern "C" bool stepper_test_is_wait_sync();
extern "C" void stepper_test_zero_position();

static uint32_t _peak_rate = 0;

// Drive one simulated stepper tick. Called from busy-waits inside the
// firmware (gcode queue-when-full, arc interpolation) via
// tight_loop_contents() so that those loops actually make progress when
// running on the host.
static void sim_one_tick() {
    if (stepper_test_is_wait_sync()) {
        stepper_service_sync();
        return;
    }
    if (!stepper_test_is_running()) {
        if (plan_is_empty()) return;
        stepper_wake();
    }
    uint32_t next = timer_hw->alarm[1];
    if ((int32_t)(next - timer_hw->timerawl) > 0) {
        host_set_time_us(next);
    }
    stepper_tick_for_test();
    uint32_t r = stepper_current_rate_events_per_sec();
    if (r > _peak_rate) _peak_rate = r;
}

// Host-side override of the pico SDK's loop hint. Every busy-wait in the
// firmware calls this; in tests, each call advances the simulated stepper
// by one event.
void tight_loop_contents() {
    sim_one_tick();
}

// yield() replaces tight_loop_contents() in production spin loops (to service
// USB on real hardware). In tests it must also advance the sim.
void yield() {
    sim_one_tick();
}

void sim_reset() {
    host_reset_pin_state();
    host_eeprom_reset();                    // clear persisted settings blob
    plan_reset();
    plan_set_position(0.0f, 0.0f);
    stepper_reset();
    stepper_test_zero_position();
    gcode_reset();
    // Restore compile-time defaults (prior tests may have mutated them).
    planner_max_feedrate_mm_min    = DEFAULT_MAX_FEEDRATE_MM_MIN;
    planner_max_rapid_mm_min       = DEFAULT_MAX_RAPID_MM_MIN;
    planner_max_accel_mm_s2        = DEFAULT_MAX_ACCEL_MM_S2;
    planner_motor_max_accel_mm_s2  = MOTOR_MAX_ACCEL_MM_S2;
    planner_motor_max_rate_mm_min  = MOTOR_MAX_RATE_MM_MIN;
    planner_junction_deviation_mm  = DEFAULT_JUNCTION_DEVIATION;
    planner_arc_tolerance_mm       = DEFAULT_ARC_TOLERANCE;
    work_area_max_x_mm             = DEFAULT_MAX_X_MM;
    work_area_max_y_mm             = DEFAULT_MAX_Y_MM;
    servo_pen_up_us                = DEFAULT_PEN_UP_US;
    servo_pen_down_us              = DEFAULT_PEN_DOWN_US;
    _peak_rate = 0;
    stepper_init();
    host_reset_pin_state();
    host_set_time_us(0);
}

void sim_run_until_idle(uint64_t max_sim_us) {
    uint64_t start = sim_elapsed_us();

    // Prime: if stepper is idle but planner has a block, wake it once.
    if (!stepper_test_is_running() && !stepper_test_is_wait_sync()) {
        stepper_wake();
    }

    // Safety iteration cap — distinct from the time budget, to detect
    // infinite ISR loops regardless of virtual clock.
    const uint64_t max_ticks = 200ULL * 1000 * 1000;
    uint64_t ticks = 0;

    while (true) {
        if (sim_elapsed_us() - start > max_sim_us) break;
        if (ticks++ > max_ticks) break;

        if (!stepper_test_is_running() && !stepper_test_is_wait_sync()
            && plan_is_empty()) break;

        sim_one_tick();
    }
}

int32_t sim_motor_steps_m1() {
    float x, y;
    stepper_get_machine_position_mm(&x, &y);
    // Invert forward kinematics to get motor-space mm, then to steps.
    float a = x + y;
    return (int32_t)lroundf(a * STEPS_PER_MM);
}

int32_t sim_motor_steps_m2() {
    float x, y;
    stepper_get_machine_position_mm(&x, &y);
    float b = y - x;
    return (int32_t)lroundf(b * STEPS_PER_MM);
}

void sim_get_cartesian_mm(float *x, float *y) {
    stepper_get_machine_position_mm(x, y);
}

uint32_t sim_pulses_m1() { return host_pin_rising_edges(PIN_MOTOR1_STEP); }
uint32_t sim_pulses_m2() { return host_pin_rising_edges(PIN_MOTOR2_STEP); }

uint64_t sim_elapsed_us() { return timer_hw->timerawl; }
uint32_t sim_peak_rate_events_per_sec() { return _peak_rate; }

int sim_gcode(const char *line) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s", line);
    return (int)gcode_execute_line(buf);
}
