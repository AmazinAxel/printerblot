#pragma once

#include "config.h"

// Persistent settings stored in flash-backed EEPROM emulation. Loaded once
// at boot; saved every time a `$N=value` command updates a setting.
//
// The planner's tunables (planner_*) live in planner.cpp as globals; this
// module owns the non-planner settables (work area, servo PWM) and the
// load/save plumbing. See settings_load()/settings_save() for the field list.

// Work area (physical envelope). Origin (0, 0) is bottom-left; +X right,
// +Y up. Used for documentation today; when we add $20 soft-limits they
// become hard bounds.
extern float work_area_max_x_mm;
extern float work_area_max_y_mm;

// Pen servo pulse widths (µs). Extracted from the gcode modal state so
// they persist across soft-reset (gcode_reset() no longer touches them)
// and can be saved to flash.
extern int servo_pen_up_us;
extern int servo_pen_down_us;

// Load settings from EEPROM-on-flash. If the blob is unversioned or
// uninitialized, compile-time defaults are kept. Call once on boot.
void settings_load();

// Mark settings as dirty (needs writing to flash). Call after a successful
// `$N=value`. Does NOT actually touch flash — the expensive EEPROM.commit
// happens lazily via settings_service_save() once the stepper is idle and
// some time has passed since the last change, so back-to-back $N= commands
// don't each stall the main loop with a flash erase.
void settings_mark_dirty();

// Reset every settable field (planner caps, work area, servo pulses) to
// the compile-time defaults. Does NOT touch flash — the caller should
// `settings_mark_dirty()` afterwards if the reset should persist.
void settings_restore_defaults();

// Call from the main loop each iteration. Writes dirty settings to flash
// if the debounce window has elapsed AND the stepper is idle (so motion
// isn't stalled by the flash erase). No-op otherwise.
void settings_service_save();
