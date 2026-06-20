#include "test.h"
#include "../src/settings.h"
#include "../src/planner.h"
#include "../src/stepper.h"
#include "../src/arc.h"
#include "../src/gcode.h"
#include "sim.h"
#include "mocks/Arduino.h"

static StatusCode run(const char *line) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s", line);
    return gcode_execute_line(buf);
}

// ══════ $130 / $131 work-area settings ══════

TEST_CASE("settings: $130 and $131 default to the physical Blot envelope") {
    sim_reset();
    CHECK_NEAR(work_area_max_x_mm, 125.0f, 1e-4);
    CHECK_NEAR(work_area_max_y_mm, 125.0f, 1e-4);
}

TEST_CASE("settings: $130= sets max X travel") {
    sim_reset();
    CHECK_EQ((int)run("$130=150"), (int)STATUS_OK);
    CHECK_NEAR(work_area_max_x_mm, 150.0f, 1e-4);
}

TEST_CASE("settings: $131= sets max Y travel") {
    sim_reset();
    CHECK_EQ((int)run("$131=200"), (int)STATUS_OK);
    CHECK_NEAR(work_area_max_y_mm, 200.0f, 1e-4);
}

TEST_CASE("settings: $130/$131 show up in $$ dump") {
    sim_reset();
    Serial.host_clear();
    run("$$");
    std::string out = Serial.host_drain_output();
    CHECK(out.find("$130=125.") != std::string::npos);
    CHECK(out.find("$131=125.") != std::string::npos);
}

// ══════ persistence across "boot" ══════

TEST_CASE("settings: runtime changes survive simulated reboot") {
    sim_reset();
    // Change several settings — each $N= marks dirty; advance time and idle
    // so the debounced save actually runs.
    run("$110=7500");
    run("$112=7500");
    run("$120=800");
    run("$130=150");
    run("$131=175");
    run("$150=900");
    run("$151=1800");
    // Advance virtual time past the debounce window and trigger a service.
    host_advance_time_us(1000 * 1000);   // 1 second
    settings_service_save();

    // Simulate a reboot: host_reset_pin_state wipes EVERYTHING except the
    // persisted EEPROM, then settings_load reconstitutes from flash.
    host_reset_pin_state();
    plan_reset();
    stepper_reset();
    gcode_reset();
    // Reset in-memory values to compile defaults so we can see load actually works.
    planner_max_feedrate_mm_min    = DEFAULT_MAX_FEEDRATE_MM_MIN;
    planner_motor_max_rate_mm_min  = MOTOR_MAX_RATE_MM_MIN;
    planner_max_accel_mm_s2        = DEFAULT_MAX_ACCEL_MM_S2;
    work_area_max_x_mm             = DEFAULT_MAX_X_MM;
    work_area_max_y_mm             = DEFAULT_MAX_Y_MM;
    servo_pen_up_us                = DEFAULT_PEN_UP_US;
    servo_pen_down_us              = DEFAULT_PEN_DOWN_US;

    settings_load();

    CHECK_NEAR(planner_max_feedrate_mm_min,   7500.0f, 1e-2);
    CHECK_NEAR(planner_motor_max_rate_mm_min, 7500.0f, 1e-2);
    CHECK_NEAR(planner_max_accel_mm_s2,        800.0f, 1e-2);
    CHECK_NEAR(work_area_max_x_mm,             150.0f, 1e-3);
    CHECK_NEAR(work_area_max_y_mm,             175.0f, 1e-3);
    CHECK_EQ(servo_pen_up_us,    900);
    CHECK_EQ(servo_pen_down_us, 1800);
}

TEST_CASE("settings: fresh boot with no persisted blob uses compile defaults") {
    // host_eeprom_reset() is called by sim_reset(), so EEPROM is all 0xFF
    // (invalid magic) when settings_load runs.
    sim_reset();
    // Compile defaults should be in effect.
    CHECK_NEAR(planner_max_feedrate_mm_min, DEFAULT_MAX_FEEDRATE_MM_MIN, 1e-3);
    CHECK_NEAR(work_area_max_x_mm,          DEFAULT_MAX_X_MM,            1e-3);
}

TEST_CASE("settings: $RST=* restores every setting to compile defaults") {
    sim_reset();
    // Mutate a bunch of things away from defaults.
    run("$110=9999");
    run("$120=1108418");
    run("$130=300");
    run("$140=0.001");
    run("$150=2200");
    // Wipe everything.
    CHECK_EQ((int)run("$RST=*"), (int)STATUS_OK);
    CHECK_NEAR(planner_max_feedrate_mm_min, DEFAULT_MAX_FEEDRATE_MM_MIN, 1e-3);
    CHECK_NEAR(planner_max_accel_mm_s2,     DEFAULT_MAX_ACCEL_MM_S2,    1e-3);
    CHECK_NEAR(work_area_max_x_mm,          DEFAULT_MAX_X_MM,            1e-3);
    CHECK_NEAR(planner_junction_deviation_mm, DEFAULT_JUNCTION_DEVIATION, 1e-5);
    CHECK_EQ(servo_pen_up_us,   DEFAULT_PEN_UP_US);
    CHECK_EQ(servo_pen_down_us, DEFAULT_PEN_DOWN_US);
}

TEST_CASE("settings: $RST with a bad scope char is an error") {
    sim_reset();
    CHECK_EQ((int)run("$RST=Q"), (int)STATUS_INVALID_STATEMENT);
}

TEST_CASE("settings: M3 S<val> updates the persisted down position") {
    sim_reset();
    run("M3 S1550");
    CHECK_EQ(servo_pen_down_us, 1550);
    // Let the motion complete, then trigger the debounced save.
    sim_run_until_idle();
    host_advance_time_us(1000 * 1000);
    settings_service_save();
    // Simulate reboot — down value should survive.
    servo_pen_down_us = DEFAULT_PEN_DOWN_US;
    settings_load();
    CHECK_EQ(servo_pen_down_us, 1550);
}
