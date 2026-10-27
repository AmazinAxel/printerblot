#pragma once

#include "config.h"

// Work area (physical envelope) in machine coordinates. Enforced as soft
// limits in plan_buffer_line() whenever planner_soft_limits_enabled ($20) is
// set — see planner.h.
extern float work_area_max_x_mm;
extern float work_area_max_y_mm;

// Pen servo pulse widths (µs) for M5 (up) and M3 (down).
extern int servo_pen_up_us;
extern int servo_pen_down_us;

// Which quality preset the settings were last set by (job.cpp), so the UI
// shows the right one after a reboot.
extern bool quality_poster;

// Apply compile-time defaults, then whatever valid blob is on flash. Call
// once on boot, before anything reads a setting.
void settings_load();

// Set setting `id` (see docs/CALIBRATION.md for the numbering). False if the
// id is unknown or the value invalid. RAM only; settings_save() persists.
bool settings_set(int id, float value);

// Write the settings to flash, skipped when they haven't changed.
void settings_save();
