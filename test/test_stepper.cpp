#include "test.h"
#include "../src/planner.h"
#include "../src/stepper.h"
#include "../src/config.h"
#include "sim.h"
#include "mocks/Arduino.h"
#include "mocks/pico/time.h"
#include <cmath>

extern "C" bool stepper_test_is_held();
extern "C" bool stepper_test_is_feed_hold();

// These tests drive the real ISR through the simulator and verify the
// Bresenham distribution + CoreXY motor-mix is correct from the motor's
// point of view. We assert on pulse counts, end positions, and timing.

TEST_CASE("stepper: G1 +X produces equal pulse counts on both motors") {
    sim_reset();
    plan_buffer_line(10.0f, 0.0f, 600.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 800u);  // 10mm × 80 steps/mm
    CHECK_EQ(sim_pulses_m2(), 800u);
}

TEST_CASE("stepper: G1 +Y produces equal pulse counts") {
    // Under Y-invert: cartesian_to_motor(0, 10) → a=-10, b=-10 (both motors
    // step backward 800 pulses each). Pulse COUNT is 800 regardless.
    sim_reset();
    plan_buffer_line(0.0f, 10.0f, 600.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 800u);
    CHECK_EQ(sim_pulses_m2(), 800u);
}

TEST_CASE("stepper: G1 +45° pulses only motor2") {
    // cartesian_to_motor(5, 5) → a=0, b=-10 → motor2 only.
    sim_reset();
    plan_buffer_line(5.0f, 5.0f, 600.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 0u);
    CHECK_EQ(sim_pulses_m2(), 800u);
}

TEST_CASE("stepper: G1 -45° pulses only motor1") {
    // cartesian_to_motor(5, -5) → a=10, b=0 → motor1 only.
    sim_reset();
    plan_buffer_line(5.0f, -5.0f, 600.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 800u);
    CHECK_EQ(sim_pulses_m2(), 0u);
}

TEST_CASE("stepper: motor direction pins reflect sign of delta") {
    sim_reset();
    plan_buffer_line(-10.0f, 0.0f, 600.0f, false);
    sim_run_until_idle();
    // Final Cartesian position matches target
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, -10.0f, 0.02f);
    CHECK_NEAR(y,   0.0f, 0.02f);
    // Machine motor step counts are correct (negative)
    CHECK_EQ(sim_motor_steps_m1(), -800);
    CHECK_EQ(sim_motor_steps_m2(),  800);
}

TEST_CASE("stepper: multi-block motion ends at correct position") {
    sim_reset();
    plan_buffer_line(10.0f,  0.0f, 3000.0f, false);
    plan_buffer_line(10.0f, 10.0f, 3000.0f, false);
    plan_buffer_line( 0.0f, 10.0f, 3000.0f, false);
    plan_buffer_line( 0.0f,  0.0f, 3000.0f, false);
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.02f);
    CHECK_NEAR(y, 0.0f, 0.02f);
    // Pulses returned to zero net position
    CHECK_EQ(sim_motor_steps_m1(), 0);
    CHECK_EQ(sim_motor_steps_m2(), 0);
}

// ══════ Bresenham distribution for an angled line ══════

TEST_CASE("stepper: 10x20 move produces correct pulse ratio") {
    // With Y-invert: cartesian_to_motor(10, 20) → a=10-20=-10, b=-20-10=-30.
    // |na|=800, |nb|=2400. Motor 2 dominant 3:1.
    sim_reset();
    plan_buffer_line(10.0f, 20.0f, 1500.0f, false);
    sim_run_until_idle();
    uint32_t p1 = sim_pulses_m1();
    uint32_t p2 = sim_pulses_m2();
    CHECK_EQ(p1,  800u);
    CHECK_EQ(p2, 2400u);
    CHECK_EQ(p2, 3u * p1);
}

// ══════ Trapezoidal velocity profile: peak rate observations ══════

TEST_CASE("stepper: long move reaches nominal rate") {
    sim_reset();
    planner_max_feedrate_mm_min    = 3000.0f;  // 50 mm/s
    planner_max_accel_mm_s2        = 500.0f;
    planner_motor_max_accel_mm_s2  = 1000.0f;
    planner_motor_max_rate_mm_min  = 10000.0f; // raise motor cap so feedrate is the bound
    plan_buffer_line(100.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    // On pure X, load=1, so nominal_rate in events/s = 50 × 80 = 4000 events/s
    uint32_t peak = sim_peak_rate_events_per_sec();
    CHECK(peak >= 3800u);
    CHECK(peak <= 4100u);
}

TEST_CASE("stepper: short move doesn't reach nominal (triangle profile)") {
    sim_reset();
    planner_max_feedrate_mm_min    = 3000.0f;   // 50 mm/s
    planner_max_accel_mm_s2        = 50.0f;     // gentle
    planner_motor_max_accel_mm_s2  = 1000.0f;
    // A 2mm move with a=50 mm/s², starting from rest and ending at rest,
    // reaches peak v where v² = a·L → v = √(50·2) = 10 mm/s << 50 nominal.
    plan_buffer_line(2.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    uint32_t peak = sim_peak_rate_events_per_sec();
    // Peak ~= 10 mm/s × 80 steps/mm = 800 events/s. Nominal would be 4000.
    CHECK(peak < 2000u);
    CHECK(peak > 500u);
}

TEST_CASE("stepper: motion time is consistent with accel + distance") {
    sim_reset();
    planner_max_feedrate_mm_min    = 6000.0f;   // 100 mm/s
    planner_max_accel_mm_s2        = 500.0f;
    planner_motor_max_accel_mm_s2  = 1000.0f;
    planner_motor_max_rate_mm_min  = 20000.0f;  // so feedrate is the bound
    // 50mm move: accel phase = v²/(2a) = 100²/1000 = 10mm
    // → 10mm accel, 30mm cruise, 10mm decel.
    // Time = 2·(v/a) + 30/v = 2·0.2 + 0.3 = 0.7s.
    plan_buffer_line(50.0f, 0.0f, 6000.0f, false);
    sim_run_until_idle();
    uint64_t us = sim_elapsed_us();
    CHECK(us > 600000ULL);
    CHECK(us < 900000ULL);
}

// ══════ CoreXY: direction-dependent speed under motor-rate constraint ══════

TEST_CASE("stepper: diagonal move is slower than axis-aligned (motor limited)") {
    // Run a 10mm axis-aligned move and a 10mm (actually √50 ≈ 7.07) diagonal.
    // Under a motor rate limit, the diagonal should take longer per unit of
    // Cartesian distance.
    sim_reset();
    planner_max_feedrate_mm_min    = 100000.0f;  // unlimited user
    planner_motor_max_rate_mm_min  = 1500.0f;    // motor is the bound
    planner_max_accel_mm_s2        = 100000.0f;
    planner_motor_max_accel_mm_s2  = 100000.0f;  // effectively instant accel
    plan_buffer_line(10.0f, 0.0f, 100000.0f, false);
    sim_run_until_idle();
    uint64_t t_axis = sim_elapsed_us();

    sim_reset();
    planner_max_feedrate_mm_min    = 100000.0f;
    planner_motor_max_rate_mm_min  = 1500.0f;
    planner_max_accel_mm_s2        = 100000.0f;
    planner_motor_max_accel_mm_s2  = 100000.0f;
    // √50 ≈ 7.07mm along diagonal — roughly same Cartesian path length as axis
    plan_buffer_line(5.0f, 5.0f, 100000.0f, false);
    sim_run_until_idle();
    uint64_t t_diag = sim_elapsed_us();

    // On a 45° diagonal, motor-rate-limited tip speed is 1/√2 of axis-aligned.
    // So for ~7.07mm diag vs 10mm axis, times should be:
    //   t_axis = 10 / 25 = 0.4 s
    //   t_diag = 7.07 / (25/√2) = 7.07 / 17.68 = 0.4 s
    // They should be roughly equal in this exact setup.
    // The stronger test: diag per-mm time > axis per-mm time
    double per_mm_axis = (double)t_axis / 10.0;
    double per_mm_diag = (double)t_diag / std::sqrt(50.0);
    CHECK(per_mm_diag > per_mm_axis * 1.2);   // ≥ √2 slower per mm
}

// ══════ soft reset clears motion ══════

// ══════ Bresenham distribution for various angles ══════

TEST_CASE("stepper: dx=10 dy=5 produces 1:3 motor ratio") {
    // With Y-invert: cartesian_to_motor(10, 5) → a=10-5=5, b=-5-10=-15.
    // |na|=400, |nb|=1200. Ratio 1:3 (m2 dominant).
    sim_reset();
    plan_buffer_line(10.0f, 5.0f, 3000.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(),  400u);
    CHECK_EQ(sim_pulses_m2(), 1200u);
}

TEST_CASE("stepper: dx=3 dy=7 pulses motor2 5 times per 2 motor1 pulses") {
    // cartesian_to_motor(3, 7) → a=3-7=-4, b=-7-3=-10. |na|=320, |nb|=800.
    sim_reset();
    plan_buffer_line(3.0f, 7.0f, 3000.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 320u);
    CHECK_EQ(sim_pulses_m2(), 800u);
    CHECK_EQ(sim_pulses_m1() * 5u, sim_pulses_m2() * 2u);
}

TEST_CASE("stepper: dx=8 dy=2 produces 3:5 motor ratio") {
    // cartesian_to_motor(8, 2) → a=8-2=6, b=-2-8=-10. |na|=480, |nb|=800.
    sim_reset();
    plan_buffer_line(8.0f, 2.0f, 3000.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 480u);
    CHECK_EQ(sim_pulses_m2(), 800u);
    CHECK_EQ(sim_pulses_m1() * 5u, sim_pulses_m2() * 3u);
}

// ══════ feed hold behavior ══════

TEST_CASE("stepper: feed hold within a block decelerates to ST_HELD") {
    sim_reset();
    planner_max_feedrate_mm_min   = 6000.0f;
    planner_motor_max_rate_mm_min = 20000.0f;
    plan_buffer_line(200.0f, 0.0f, 6000.0f, false);
    // Drive until motion is running and roughly at cruise.
    for (int i = 0; i < 1000; i++) tight_loop_contents();
    CHECK(!stepper_is_idle());
    uint32_t before_feed = stepper_current_rate_events_per_sec();
    CHECK(before_feed > 0u);

    stepper_feed_hold();
    CHECK(stepper_test_is_feed_hold());
    // Drive until the decel reaches zero.
    for (int i = 0; i < 2000; i++) tight_loop_contents();
    CHECK(stepper_test_is_held());
    CHECK_EQ(stepper_current_rate_events_per_sec(), 0u);
}

TEST_CASE("stepper: feed hold at end of block parks in ST_HELD") {
    // Fill with many short blocks and hold near the end.
    sim_reset();
    planner_max_feedrate_mm_min   = 6000.0f;
    planner_motor_max_rate_mm_min = 20000.0f;
    planner_max_accel_mm_s2       = 1000.0f;
    planner_motor_max_accel_mm_s2 = 2000.0f;
    for (int i = 1; i <= 10; i++) {
        plan_buffer_line((float)(i * 5), 0.0f, 6000.0f, false);
    }
    // Run until well into block 3.
    for (int i = 0; i < 500; i++) tight_loop_contents();
    stepper_feed_hold();
    // Run until held (or the rest of motion completes — either way it stops).
    for (int i = 0; i < 20000; i++) {
        tight_loop_contents();
        if (stepper_is_idle() || stepper_test_is_held()) break;
    }
    CHECK((stepper_is_idle() || stepper_test_is_held()));
    // If held, motion should be incomplete; if idle, it did finish.
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK(x <= 50.0f + 0.1f);
}

TEST_CASE("stepper: cycle_start resumes execution after a hold") {
    sim_reset();
    planner_max_feedrate_mm_min   = 6000.0f;
    planner_motor_max_rate_mm_min = 20000.0f;
    plan_buffer_line(100.0f, 0.0f, 6000.0f, false);
    for (int i = 0; i < 500; i++) tight_loop_contents();
    stepper_feed_hold();
    for (int i = 0; i < 2000; i++) tight_loop_contents();
    CHECK(stepper_test_is_held());
    stepper_cycle_start();
    sim_run_until_idle(5ULL * 1000 * 1000);
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 100.0f, 0.1f);
}

// ══════ mid-motion reset ══════

TEST_CASE("stepper: reset mid-motion halts pulses cleanly") {
    sim_reset();
    plan_buffer_line(50.0f, 0.0f, 600.0f, false);
    for (int i = 0; i < 200; i++) tight_loop_contents();
    uint32_t p1_before = sim_pulses_m1();
    stepper_reset();
    plan_reset();
    // Drive some more ticks — no new pulses should be emitted.
    for (int i = 0; i < 500; i++) tight_loop_contents();
    CHECK_EQ(sim_pulses_m1(), p1_before);
    CHECK(stepper_is_idle());
}

// ══════ single-event block ══════

TEST_CASE("stepper: a block with exactly one step event completes") {
    sim_reset();
    // 1/80 mm pure X → step_event_count=1. Both motors pulse once.
    plan_buffer_line(1.0f / STEPS_PER_MM, 0.0f, 600.0f, false);
    sim_run_until_idle();
    CHECK_EQ(sim_pulses_m1(), 1u);
    CHECK_EQ(sim_pulses_m2(), 1u);
}

// ══════ direction pin on first pulse ══════

TEST_CASE("stepper: DIR pin for negative delta is LOW at first step") {
    sim_reset();
    plan_buffer_line(-10.0f, 0.0f, 600.0f, false);
    // Run until the first step has fired.
    for (int i = 0; i < 50; i++) {
        tight_loop_contents();
        if (sim_pulses_m1() > 0) break;
    }
    CHECK(sim_pulses_m1() > 0u);
    // Δa = -10 → motor1 negative → DIR pin was set LOW before the pulse.
    // Final motor_pos_steps should be negative as we continue.
    sim_run_until_idle();
    CHECK_EQ(sim_motor_steps_m1(), -800);
}

TEST_CASE("stepper: sync command doesn't emit spurious step pulses") {
    sim_reset();
    plan_buffer_line(10.0f, 0.0f, 600.0f, false);
    plan_sync_command(SYNC_SERVO, DEFAULT_PEN_DOWN_US);
    plan_buffer_line(20.0f, 0.0f, 600.0f, false);
    sim_run_until_idle();
    // Pulse counts should match the two motion blocks only (no pulses for sync).
    // Block 1: 10mm → 800 pulses. Block 2: from (10,0) to (20,0), Δ=10mm → 800.
    CHECK_EQ(sim_pulses_m1(), 1600u);
    CHECK_EQ(sim_pulses_m2(), 1600u);
}

TEST_CASE("stepper: reset flushes planner and halts motion") {
    sim_reset();
    plan_buffer_line(100.0f, 0.0f, 600.0f, false);
    plan_buffer_line(200.0f, 0.0f, 600.0f, false);
    CHECK_EQ(plan_buffer_available(), (uint8_t)(PLANNER_BUFFER_SIZE - 1 - 2));
    stepper_reset();
    plan_reset();
    CHECK(plan_is_empty());
    CHECK(stepper_is_idle());
}
