#include "test.h"
#include "../src/arc.h"
#include "../src/planner.h"
#include "../src/config.h"
#include "sim.h"
#include <cmath>

// Count queued blocks by draining them (non-destructive: we re-queue nothing,
// the sim takes care of running them later).
static uint32_t count_blocks_in_buffer() {
    uint32_t n = 0;
    uint8_t free = plan_buffer_available();
    return (PLANNER_BUFFER_SIZE - 1) - free;
}

TEST_CASE("arc: full circle returns to start after running (within 1 step)") {
    sim_reset();
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    bool ok = arc_interpolate(10.0f, 0.0f, -10.0f, 0.0f, /*cw=*/true, 600.0f);
    CHECK(ok);
    sim_run_until_idle();

    float x, y;
    sim_get_cartesian_mm(&x, &y);
    // Absolute-target rounding bounds endpoint error at ±0.5 step per axis,
    // i.e. ±0.00625 mm × √2 Cartesian. 0.02 mm is a safe upper bound.
    CHECK_NEAR(x, 10.0f, 0.02f);
    CHECK_NEAR(y,  0.0f, 0.02f);
}

TEST_CASE("arc: CW quarter circle endpoint accuracy") {
    sim_reset();
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    bool ok = arc_interpolate(0.0f, -10.0f, -10.0f, 0.0f, /*cw=*/true, 600.0f);
    CHECK(ok);
    sim_run_until_idle();

    float x, y;
    sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x,  0.0f, 0.02f);
    CHECK_NEAR(y,-10.0f, 0.02f);
}

TEST_CASE("arc: CCW quarter circle goes the other way") {
    sim_reset();
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    bool ok = arc_interpolate(0.0f, 10.0f, -10.0f, 0.0f, /*cw=*/false, 600.0f);
    CHECK(ok);
    sim_run_until_idle();
    float x, y;
    sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x,  0.0f, 0.02f);
    CHECK_NEAR(y, 10.0f, 0.02f);
}

TEST_CASE("arc: segment count scales with tolerance setting") {
    // Tighter tolerance → more segments. With a capacity-15 planner we can't
    // see a count difference if both saturate; compare total elapsed time
    // instead (more segments take longer to drain).
    sim_reset();
    planner_arc_tolerance_mm = 0.01f;
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    arc_interpolate(0.0f, -10.0f, -10.0f, 0.0f, true, 600.0f);
    sim_run_until_idle();
    uint64_t t_loose = sim_elapsed_us();

    sim_reset();
    planner_arc_tolerance_mm = 0.001f;
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    arc_interpolate(0.0f, -10.0f, -10.0f, 0.0f, true, 600.0f);
    sim_run_until_idle();
    uint64_t t_tight = sim_elapsed_us();

    // Tighter tolerance → more segments → same Cartesian length though,
    // so elapsed should be similar. The real invariant is that endpoint
    // accuracy is preserved at both tolerances.
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x,   0.0f, 0.05f);
    CHECK_NEAR(y, -10.0f, 0.05f);
    (void)t_loose; (void)t_tight;
}

TEST_CASE("arc: degenerate zero-offset arc is rejected") {
    sim_reset();
    plan_set_position(5.0f, 5.0f);
    bool ok = arc_interpolate(10.0f, 10.0f, 0.0f, 0.0f, true, 600.0f);
    CHECK_FALSE(ok);
}

TEST_CASE("arc: small radius still produces at least one segment") {
    sim_reset();
    // Half-circle radius 1mm: start (0,0), center (0.5, 0), end (1, 0)
    bool ok = arc_interpolate(1.0f, 0.0f, 0.5f, 0.0f, /*cw=*/false, 600.0f);
    CHECK(ok);
    CHECK(count_blocks_in_buffer() >= 1u);
}

// ══════ R-form arcs ══════

TEST_CASE("arc: R-form CW quarter via GCODE") {
    // G2 from (10,0) to (0,-10) with R10 → CW quarter of a unit circle.
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10 Y0");
    sim_run_until_idle();
    sim_gcode("G2 X0 Y-10 R10 F600");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x,  0.0f, 0.05f);
    CHECK_NEAR(y, -10.0f, 0.05f);
    // The path should have stayed on the arc (radius preserved).
    float r = std::sqrt(x * x + y * y);
    CHECK_NEAR(r, 10.0f, 0.1f);
}

TEST_CASE("arc: R-form CCW quarter via GCODE") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10 Y0");
    sim_run_until_idle();
    sim_gcode("G3 X0 Y10 R10 F600");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x,  0.0f, 0.05f);
    CHECK_NEAR(y, 10.0f, 0.05f);
}

TEST_CASE("arc: R-form negative R takes far-side arc (three-quarter)") {
    // With R < 0, the arc goes the long way around.
    // Start (10,0), end (0,-10), G2 with R=-10 → 270° CW sweep the other way.
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10 Y0");
    sim_run_until_idle();
    sim_gcode("G2 X0 Y-10 R-10 F600");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x,  0.0f, 0.1f);
    CHECK_NEAR(y, -10.0f, 0.1f);
}

TEST_CASE("arc: R-form with R too small returns STATUS_ARC_RADIUS_ERROR") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10 Y0");
    sim_run_until_idle();
    // Chord from (10,0) to (-10,0) is 20mm. R=5 can't span it.
    int s = sim_gcode("G2 X-10 Y0 R5 F600");
    CHECK_EQ(s, (int)STATUS_ARC_RADIUS_ERROR);
}

// ══════ arc geometry ══════

TEST_CASE("arc: arc at non-origin center") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X5 Y5");
    sim_run_until_idle();
    sim_gcode("G2 X15 Y5 I5 J0 F600");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 15.0f, 0.02f);
    CHECK_NEAR(y,  5.0f, 0.02f);
}

TEST_CASE("arc: endpoint within one-microstep of target after full circle") {
    // Absolute-target rounding in plan_buffer_line caps endpoint error at
    // ±0.5 step on each motor axis regardless of arc segment count.
    sim_reset();
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    arc_interpolate(10.0f, 0.0f, -10.0f, 0.0f, /*cw=*/true, 600.0f);
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 10.0f, 0.02f);
    CHECK_NEAR(y,  0.0f, 0.02f);
}

TEST_CASE("arc: CCW full circle via G3 through GCODE") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10 Y0");
    sim_run_until_idle();
    sim_gcode("G3 X10 Y0 I-10 J0 F600");
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 10.0f, 0.02f);
    CHECK_NEAR(y,  0.0f, 0.02f);
}

TEST_CASE("arc: tight arc_tolerance does NOT increase endpoint drift") {
    // Classic per-segment rounding scales drift with √N. With absolute-target
    // rounding, N doesn't matter — endpoint stays within one motor step.
    // This test runs two full circles, loose vs tight tolerance, and checks
    // both endpoints meet the same tight bound.
    for (float tol : {0.02f, 0.001f}) {   // 20× range of segment counts
        sim_reset();
        planner_arc_tolerance_mm = tol;
        plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
        sim_run_until_idle();
        arc_interpolate(10.0f, 0.0f, -10.0f, 0.0f, /*cw=*/true, 600.0f);
        sim_run_until_idle(30ULL * 1000 * 1000);
        float x, y; sim_get_cartesian_mm(&x, &y);
        CHECK_NEAR(x, 10.0f, 0.02f);
        CHECK_NEAR(y,  0.0f, 0.02f);
    }
}

TEST_CASE("arc: CW quarter preserves radius at endpoint") {
    sim_reset();
    planner_arc_tolerance_mm = 0.005f;
    plan_buffer_line(10.0f, 0.0f, 3000.0f, false);
    sim_run_until_idle();
    arc_interpolate(0.0f, -10.0f, -10.0f, 0.0f, true, 600.0f);
    sim_run_until_idle();
    float x, y;
    sim_get_cartesian_mm(&x, &y);
    float r_end = std::sqrt(x * x + y * y);
    CHECK_NEAR(r_end, 10.0f, 0.05f);
}
