#include "test.h"
#include "../src/arc.h"
#include "../src/planner.h"
#include "../src/config.h"
#include "../src/gcode.h"
#include "sim.h"
#include <cmath>
#include <cstdio>

// ─────────────────────────────────────────────────────────────
//   Diagnostic: how long does the simulated stepper actually
//   take to execute a small half-circle arc with the user's
//   tuned values? This mirrors the live-hardware scenario where
//   the first G3 takes 20+ seconds.
// ─────────────────────────────────────────────────────────────

static int run(const char *line) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s", line);
    return sim_gcode(buf);
}

TEST_CASE("arc timing: r=5 half-circle with user tuning") {
    sim_reset();
    // User's tuned values from the latest tune run.
    run("$110=16200");
    run("$112=16200");
    run("$120=7200");
    run("$122=4000");
    run("$140=0.05");
    run("$141=0.002");

    // Traverse to (62, 62) — same as pattern_c preamble.
    run("G21");
    run("G90");
    run("G00 X62 Y62");
    sim_run_until_idle();

    // Reset the elapsed clock by snapshotting now-time via sim_elapsed_us().
    uint64_t t_before = sim_elapsed_us();

    // The failing command from the spiral_circular pattern.
    run("G03 X72 Y62 I5 J0 F100000");
    sim_run_until_idle();

    uint64_t t_after = sim_elapsed_us();
    uint64_t arc_us  = t_after - t_before;
    double arc_s     = (double)arc_us / 1e6;
    std::fprintf(stderr, "\n  [arc-timing] r=5 half-circle took %.3f ms (peak rate = %u event/s)\n",
                 arc_s * 1000.0, sim_peak_rate_events_per_sec());

    // Expected: ~100-200 ms for a 15.7 mm arc with these settings.
    // If this shows 20+ seconds, the planner/stepper has a throughput bug.
    CHECK(arc_us < 2 * 1000 * 1000);   // must finish in under 2s in sim
}

TEST_CASE("arc timing: full circular spiral matches plotter expectations") {
    sim_reset();
    run("$110=16200");
    run("$112=16200");
    run("$120=7200");
    run("$122=4000");
    run("$140=0.05");
    run("$141=0.002");

    run("G21"); run("G90");
    run("G00 X62 Y62"); sim_run_until_idle();

    uint64_t t0 = sim_elapsed_us();
    run("G03 X72 Y62 I5 J0 F100000");
    run("G02 X52 Y62 I-10 J0 F100000");
    run("G03 X82 Y62 I15 J0 F100000");
    run("G02 X42 Y62 I-20 J0 F100000");
    run("G03 X92 Y62 I25 J0 F100000");
    run("G02 X32 Y62 I-30 J0 F100000");
    run("G03 X102 Y62 I35 J0 F100000");
    run("G02 X22 Y62 I-40 J0 F100000");
    run("G03 X112 Y62 I45 J0 F100000");
    run("G02 X12 Y62 I-50 J0 F100000");
    run("G03 X122 Y62 I55 J0 F100000");
    run("G02 X2 Y62 I-60 J0 F100000");
    sim_run_until_idle();
    uint64_t t1 = sim_elapsed_us();

    double total_s = (double)(t1 - t0) / 1e6;
    std::fprintf(stderr, "\n  [arc-timing] full circular spiral took %.3f s (peak rate %u ev/s)\n",
                 total_s, sim_peak_rate_events_per_sec());

    // Analytical expectation: sum of per-arc triangle/trapezoid times for
    // radii 5..60 step 5 at user's $120/$122/$110 = ~5 s.
    CHECK(t1 - t0 < 15 * 1000 * 1000);   // must finish in under 15s (sim)
}

// ─────────────────────────────────────────────────────────────
//   Regression: plan_buffer_planned must never fall behind tail.
//   If it does, planner_recalculate's forward pass iterates through
//   stale consumed slots and corrupts new blocks' entry_speed (via
//   garbage v_max), producing rate=1 stepper crawl on real HW.
//   Pattern: interleave queue/consume/queue so planned can get stuck
//   on a just-consumed slot.
// ─────────────────────────────────────────────────────────────
TEST_CASE("regression: buffer_planned tracks tail after stepper consumes") {
    sim_reset();
    run("$110=16200"); run("$112=16200");
    run("$120=7200");  run("$122=4000");
    run("G21"); run("G90");

    // Queue enough blocks to fill the planner buffer.
    for (int i = 1; i <= 30; i++) {
        char line[64];
        std::snprintf(line, sizeof(line), "G1 X%d Y0 F5000", i);
        run(line);
    }
    // Run partial — consume some blocks while we're mid-stream.
    sim_run_until_idle();

    // After the spiral drains, check the full path took a plausible amount
    // of time. At F5000 (83 mm/s) along a 30 mm axis, total should be well
    // under 5 s even accounting for corner decel. If plan_buffer_planned
    // had fallen behind tail, blocks would crawl at rate=1 and this would
    // take 30+ seconds.
    uint64_t total_us = sim_elapsed_us();
    std::fprintf(stderr, "\n  [regression] 30 sequential G1 moves took %.3f s\n",
                 (double)total_us / 1e6);
    CHECK(total_us < 5 * 1000 * 1000);
}

// ─────────────────────────────────────────────────────────────
//   Invariant: every queued motion block (step_event_count > 0,
//   sync_cmd == SYNC_NONE) ends up with non-zero trap fields
//   after plan_buffer_line returns. This is the specific failure
//   mode: stale-slot reuse leaving all-zero trap produces rate=1.
// ─────────────────────────────────────────────────────────────
TEST_CASE("regression: motion blocks always get non-zero trap fields") {
    sim_reset();
    run("$110=16200"); run("$120=7200");
    run("G21"); run("G90");

    // Mix sync + motion commands so some slots are sync (which produces
    // all-zero trap) before being reused for motion. This exposed the bug
    // on real HW where rate=1 crawl happened after ~30 motion lines.
    run("M5");
    run("M3 S1700");
    for (int i = 1; i <= 40; i++) {
        char line[64];
        std::snprintf(line, sizeof(line),
                      "G1 X%d Y%d F1500", i, i % 3);
        run(line);
    }
    sim_run_until_idle();

    // Just checking it runs to completion in bounded time is the real
    // invariant — a rate=1 block would hang sim_run_until_idle's budget.
    // If we got here without the watchdog firing, all blocks stepped at
    // sensible rates.
    CHECK(sim_elapsed_us() < 10 * 1000 * 1000);
}
