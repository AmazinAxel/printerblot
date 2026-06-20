#include "test.h"
#include "../src/planner.h"
#include "../src/stepper.h"
#include "../src/gcode.h"
#include "../src/config.h"
#include "sim.h"
#include "mocks/Arduino.h"
#include "mocks/Servo.h"
#include <cmath>

// End-to-end: feed GCODE through the parser and verify the machine ends
// where the program asks it to, with the right pulse tallies and pen events.
// This is the test that mirrors "if I send this GCODE, will the right lines
// appear on paper?"

TEST_CASE("integration: square traces back to origin") {
    sim_reset();
    sim_gcode("G21 G90");
    sim_gcode("G1 X10 Y0 F600");
    sim_gcode("X10 Y10");
    sim_gcode("X0 Y10");
    sim_gcode("X0 Y0");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.02f);
    CHECK_NEAR(y, 0.0f, 0.02f);
    CHECK_EQ(sim_motor_steps_m1(), 0);
    CHECK_EQ(sim_motor_steps_m2(), 0);
}

TEST_CASE("integration: pen down / up cycle leaves servo in correct state") {
    sim_reset();
    sim_gcode("G21 G90");
    sim_gcode("M5");                 // pen up
    sim_gcode("G1 X5 F600");
    sim_gcode("M3 S1700");           // pen down
    sim_gcode("G1 X10");
    sim_gcode("M5");                 // pen up
    sim_gcode("G1 X0");
    sim_run_until_idle();
    Servo *s = host_last_servo();
    CHECK(s != nullptr);
    // Standard servo: holds pen-up position after M5
    if (s) CHECK_EQ(s->last_us(), DEFAULT_PEN_UP_US);
}

TEST_CASE("integration: G2 full circle via GCODE") {
    sim_reset();
    sim_gcode("G21 G90");
    sim_gcode("G1 X10 Y0 F600");
    sim_run_until_idle();
    sim_gcode("G2 X10 Y0 I-10 J0 F600");   // full CW circle
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 10.0f, 0.05f);
    CHECK_NEAR(y,  0.0f, 0.05f);
}

TEST_CASE("integration: G91 relative + mixed absolute moves") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X5 Y5");           // abs (5, 5)
    sim_gcode("G91");
    sim_gcode("X3");                 // rel +3 X → (8, 5)
    sim_gcode("Y2");                 // rel +2 Y → (8, 7)
    sim_gcode("G90 X0 Y0");          // abs origin
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.05f);
    CHECK_NEAR(y, 0.0f, 0.05f);
}

TEST_CASE("integration: G20 inches — 1 inch square in mm comes out 25.4×25.4") {
    sim_reset();
    sim_gcode("G20 G90 F60");        // 60 in/min = 1524 mm/min
    sim_gcode("G1 X1 Y0");           // 25.4 mm
    sim_gcode("X1 Y1");              // (25.4, 25.4)
    sim_gcode("X0 Y1");              // (0, 25.4)
    sim_gcode("X0 Y0");              // (0, 0)
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.02f);
    CHECK_NEAR(y, 0.0f, 0.02f);
}

TEST_CASE("integration: G92 mid-program redefines origin correctly") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10 Y0");
    sim_run_until_idle();
    sim_gcode("G92 X0 Y0");          // now this spot is (0, 0) in work coords
    sim_gcode("G1 X5 Y0");           // work X=5 → absolute X=15
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 15.0f, 0.05f);
}

TEST_CASE("integration: buffered streaming — many lines don't lose position") {
    sim_reset();
    sim_gcode("G21 G90 F1500");
    // 20 tiny moves along a zig-zag; endpoints should sum exactly.
    for (int i = 1; i <= 20; i++) {
        char line[32];
        std::snprintf(line, sizeof(line), "X%d Y%d", i, (i % 2) ? 1 : 0);
        sim_gcode(line);
    }
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 20.0f, 0.05f);
    CHECK_NEAR(y,  0.0f, 0.05f);
}

TEST_CASE("integration: dwell blocks the timeline") {
    sim_reset();
    sim_gcode("G21 G90 F6000");   // 100 mm/s
    sim_gcode("G1 X1");           // brief move (<< 1s)
    sim_gcode("G4 P500");         // 500 ms dwell
    sim_gcode("G1 X0");
    uint64_t t0 = sim_elapsed_us();
    sim_run_until_idle();
    uint64_t dt = sim_elapsed_us() - t0;
    CHECK(dt >= 500000u);
}

// ══════ a zig-zag that forces junction-deviation to engage at corners ══════

// ══════ step-count conservation ══════

TEST_CASE("integration: return to origin leaves motor steps at zero") {
    // For any commanded target, a follow-up G1 X0 Y0 must bring both motor
    // step counters back to zero exactly. This is the step-conservation
    // invariant — the one thing that should NEVER be violated.
    sim_reset();
    sim_gcode("G21 G90 F1500");
    const float targets[][2] = {
        { 23.4f,  17.8f}, {-12.3f,  8.1f}, {0.5f, -4.2f},
        {100.0f, 0.0f}, {0.0f, 100.0f}, {-50.0f, -50.0f},
    };
    for (auto &p : targets) {
        char line[40];
        std::snprintf(line, sizeof(line), "G1 X%f Y%f", (double)p[0], (double)p[1]);
        sim_gcode(line);
    }
    sim_gcode("G1 X0 Y0");
    sim_run_until_idle(30ULL * 1000 * 1000);   // longer budget: total path is >150mm
    CHECK_EQ(sim_motor_steps_m1(), 0);
    CHECK_EQ(sim_motor_steps_m2(), 0);
}

TEST_CASE("integration: step conservation holds for nasty fractional targets") {
    // Use targets that do NOT multiply to integer step counts, so per-segment
    // rounding would leak. Absolute-target rounding in the planner ensures
    // that after returning to (0, 0), motor_pos_steps is *exactly* zero.
    sim_reset();
    sim_gcode("G21 G90 F1500");
    const float targets[][2] = {
        { 1.333f,  2.777f},
        {-4.1234f, 0.5678f},
        { 7.91011f, -3.14159f},
        {-0.01f,   99.99f},
    };
    for (auto &p : targets) {
        char line[48];
        std::snprintf(line, sizeof(line), "G1 X%f Y%f", (double)p[0], (double)p[1]);
        sim_gcode(line);
    }
    sim_gcode("G1 X0 Y0");
    sim_run_until_idle(30ULL * 1000 * 1000);
    CHECK_EQ(sim_motor_steps_m1(), 0);
    CHECK_EQ(sim_motor_steps_m2(), 0);
}

// ══════ long programs ══════

TEST_CASE("integration: 80-line program ends at correct position") {
    sim_reset();
    sim_gcode("G21 G90 F1500");
    // 40 line segments forward along a diagonal, then 40 back.
    for (int i = 1; i <= 40; i++) {
        char line[32];
        std::snprintf(line, sizeof(line), "G1 X%d Y%d", i, i);
        sim_gcode(line);
    }
    for (int i = 39; i >= 0; i--) {
        char line[32];
        std::snprintf(line, sizeof(line), "G1 X%d Y%d", i, i);
        sim_gcode(line);
    }
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.1f);
    CHECK_NEAR(y, 0.0f, 0.1f);
    CHECK_EQ(sim_motor_steps_m1(), 0);
    CHECK_EQ(sim_motor_steps_m2(), 0);
}

// ══════ mixed units mid-stream ══════

TEST_CASE("integration: G20 mid-stream doesn't corrupt position") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X25.4 Y0");        // in mm
    sim_run_until_idle();
    sim_gcode("G20");                 // switch to inches
    sim_gcode("G1 X1 Y0");            // 1 inch = 25.4 mm (same place)
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 25.4f, 0.05f);
    sim_gcode("G21");                 // back to mm
    sim_gcode("G1 X0 Y0");
    sim_run_until_idle();
    sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.05f);
}

// ══════ arc → arc junction ══════

TEST_CASE("integration: G2 followed by G3 (S-curve) runs cleanly") {
    // Two half-circles: first CW around (10, 0) from (0, 0) to (20, 0),
    // then CCW around (30, 0) from (20, 0) to (40, 0).
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G2 X20 Y0 I10 J0");   // CW half from origin, back on X axis
    sim_gcode("G3 X40 Y0 I10 J0");   // CCW half continuing
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 40.0f, 0.2f);
    CHECK_NEAR(y,  0.0f, 0.2f);
}

// ══════ pen-cycle timing ══════

TEST_CASE("integration: M3/G1/M5 cycle shows servo dwells in elapsed time") {
    sim_reset();
    sim_gcode("G21 G90 F6000");   // 100 mm/s so motion is quick
    sim_gcode("M5");              // pen up
    sim_gcode("G1 X1");           // brief motion
    sim_gcode("M3 S1700");        // pen down
    sim_gcode("G1 X2");
    sim_gcode("M5");              // pen up
    uint64_t t0 = sim_elapsed_us();
    sim_run_until_idle();
    uint64_t dt = sim_elapsed_us() - t0;
    // 3 × DEFAULT_PEN_MOVE_MS from servo dwells
    CHECK(dt >= 3ULL * DEFAULT_PEN_MOVE_MS * 1000ULL);
}

TEST_CASE("integration: sharp zig-zag ends at correct position") {
    sim_reset();
    planner_junction_deviation_mm = 0.01f;
    planner_max_accel_mm_s2 = 200.0f;
    sim_gcode("G21 G90 F1500");
    sim_gcode("G1 X10 Y0");
    sim_gcode("X10 Y1");   // sharp bend
    sim_gcode("X0  Y1");
    sim_gcode("X0  Y0");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 0.0f, 0.05f);
    CHECK_NEAR(y, 0.0f, 0.05f);
    // Back-to-zero: net step counts should be zero.
    CHECK_EQ(sim_motor_steps_m1(), 0);
    CHECK_EQ(sim_motor_steps_m2(), 0);
}
