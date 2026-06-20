# Firmware test harness

Host-side tests for the Blot GCODE firmware. Builds the real firmware `.cpp`
files against host stubs for Arduino and the Pico SDK, then exercises them
through unit tests and a simulated-machine integration harness.

## Run

```
make        # compile the suite into ./run_tests
make test   # compile + run
make clean
```

Requires a C++17 compiler (`g++` or `clang++`). No external dependencies —
the test framework and mocks are vendored here.

## What gets tested

- **[test_kinematics.cpp](test_kinematics.cpp)** — CoreXY forward/inverse,
  `motor_load_for_unit_dir` (=1 on axes, =√2 on ±45°, in [1, √2] for all
  directions).
- **[test_planner.cpp](test_planner.cpp)** — step-count/direction math,
  CoreXY-aware acceleration clamping (user cap vs motor cap, per-direction),
  feedrate clamping, buffer availability, overflow, junction deviation for
  straight lines / reversals / 90° corners.
- **[test_arc.cpp](test_arc.cpp)** — G2/G3 endpoint accuracy, CW vs CCW,
  radius preservation, degenerate cases, tolerance behavior.
- **[test_gcode.cpp](test_gcode.cpp)** — parser (comments, whitespace,
  case-insensitivity, bad numbers, unknown codes, modal conflicts), modal
  persistence (motion, F, units, G90/G91), G20 inch conversion, G92,
  rapid vs feed, M3/M5 pen control, M17/M18 drivers, G4 dwell, `$`-settings.
- **[test_stepper.cpp](test_stepper.cpp)** — simulated-machine Bresenham
  distribution (pure X, pure Y, ±45°, angled 1:2 ratio), trapezoidal peak
  rate, triangle profile for short moves, motion time, CoreXY
  direction-dependent speed under motor-rate constraint, reset behavior.
- **[test_integration.cpp](test_integration.cpp)** — end-to-end GCODE
  programs (square, pen cycle, full circle, mixed G90/G91, inches,
  G92 mid-program, buffered streaming, dwells, sharp zig-zag).

## How the simulator works

`sim.cpp` drives the **real stepper ISR** in software. Each call to
`sim_one_tick` advances the virtual clock to the next scheduled alarm fire,
then invokes `stepper_tick_for_test` (exposed from `stepper.cpp` under
`#ifdef FIRMWARE_TEST`). Pin writes go into a recording layer so pulse
counts are observable; servo writes are captured by the `Servo` mock.

The same `tight_loop_contents` hook that the firmware uses in its
busy-waits is overridden in the sim to run one stepper tick per call, so
`arc_interpolate` and the G-code main-loop backpressure paths actually
make progress in tests.

Tests queue G-code or call the planner directly, then use
`sim_run_until_idle()` to let the stepper drain. After that they assert on:

- `sim_motor_steps_m1/m2()` — signed step counts on each motor
- `sim_get_cartesian_mm(x, y)` — Cartesian position derived from motor steps
- `sim_pulses_m1/m2()` — total positive-going STEP transitions
- `sim_elapsed_us()` — virtual time the run took
- `sim_peak_rate_events_per_sec()` — highest step-event rate observed

## Adding tests

```cpp
#include "test.h"
#include "sim.h"

TEST_CASE("my new check") {
    sim_reset();
    sim_gcode("G21 G90 F600");
    sim_gcode("G1 X10");
    sim_run_until_idle();

    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 10.0f, 0.02f);
}
```

Macros: `CHECK(cond)`, `CHECK_EQ(a, b)`, `CHECK_NEAR(a, b, tol)`,
`CHECK_FALSE(cond)`.

## Limitations

- No concurrency model — the ISR runs synchronously. Real-hardware races
  between planner recalculate and stepper ISR aren't exercised.
- Servo timing is simulated as an instantaneous call plus a `delay()`; the
  150 ms default pen-move delay IS modelled (it advances virtual time).
- No modelling of belt elasticity, step loss, or stall.
- Flash persistence isn't tested (settings live in RAM in V1 firmware).
