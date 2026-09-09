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
// Returns false only on degenerate geometry.

bool arc_interpolate(float target_x, float target_y,
                     float offset_i, float offset_j,
                     bool clockwise,
                     float feedrate_mm_min);

// Persisted arc tolerance (same value the planner uses).
extern float planner_arc_tolerance_mm;
