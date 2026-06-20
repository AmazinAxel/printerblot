#pragma once

// Forward declaration only — the real implementation lives in sim.cpp so
// that host-side busy-waits (e.g. arc_interpolate waiting on planner slots)
// can drive one simulated stepper tick on each call.
void tight_loop_contents();
