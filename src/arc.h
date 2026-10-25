#pragma once

#include <Arduino.h>

// G2/G3 arc interpolation. Decomposes an arc into short linear segments
// chosen to stay within planner_arc_tolerance_mm of the true arc, subject
// to a 0.5mm minimum segment length (see arc.cpp).
// Each segment is queued into the planner; this function blocks until
// all segments are accepted (yielding to stepper/sync servicing if the
// planner buffer fills).
//
// `target_*` is the arc end point (mm, absolute).
// `offset_i, offset_j` is the center offset from the current position.
// `clockwise` = true for G2, false for G3.
//
// Returns false on degenerate geometry, or if any part of the arc lies
// outside the soft-limit envelope — the bounds test runs before the first
// segment is queued, so a rejected arc draws nothing at all. When it returns
// false, `*out_of_bounds` (if given) distinguishes the two cases so the
// caller can report the right error. A feed hold while waiting for the
// planner also abandons the arc and returns false; a soft reset abandons it
// and returns true.

bool arc_interpolate(float target_x, float target_y,
                     float offset_i, float offset_j,
                     bool clockwise,
                     float feedrate_mm_min,
                     bool *out_of_bounds = nullptr);

// Persisted arc tolerance (same value the planner uses).
extern float planner_arc_tolerance_mm;
