#include "settings.h"
#include "planner.h"
#include "arc.h"
#include "stepper.h"
#include <EEPROM.h>
#include <Arduino.h>

// Default values for persistent settings. Compile-time only; the user can
// override at runtime via $N= and those values get saved to flash.
float work_area_max_x_mm = DEFAULT_MAX_X_MM;
float work_area_max_y_mm = DEFAULT_MAX_Y_MM;
int   servo_pen_up_us    = DEFAULT_PEN_UP_US;
int   servo_pen_down_us  = DEFAULT_PEN_DOWN_US;

// On-flash layout. Bump VERSION if the struct changes — older blobs will
// be treated as invalid and compile defaults will be used until the next
// save.
static const uint32_t SETTINGS_MAGIC   = 0xB107B107u;
static const uint32_t SETTINGS_VERSION = 1u;

struct SavedSettings {
    uint32_t magic;
    uint32_t version;
    float    max_feedrate_mm_min;
    float    max_rapid_mm_min;
    float    motor_max_rate_mm_min;
    float    max_accel_mm_s2;
    float    motor_max_accel_mm_s2;
    float    max_x_mm;
    float    max_y_mm;
    float    junction_deviation_mm;
    float    arc_tolerance_mm;
    int      pen_up_us;
    int      pen_down_us;
};

#define EEPROM_BYTES  (sizeof(SavedSettings) + 16)

void settings_load() {
    EEPROM.begin(EEPROM_BYTES);
    SavedSettings s;
    EEPROM.get(0, s);
    if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) return;

    planner_max_feedrate_mm_min    = s.max_feedrate_mm_min;
    planner_max_rapid_mm_min       = s.max_rapid_mm_min;
    planner_motor_max_rate_mm_min  = s.motor_max_rate_mm_min;
    planner_max_accel_mm_s2        = s.max_accel_mm_s2;
    planner_motor_max_accel_mm_s2  = s.motor_max_accel_mm_s2;
    planner_junction_deviation_mm  = s.junction_deviation_mm;
    planner_arc_tolerance_mm       = s.arc_tolerance_mm;
    work_area_max_x_mm             = s.max_x_mm;
    work_area_max_y_mm             = s.max_y_mm;
    servo_pen_up_us                = s.pen_up_us;
    servo_pen_down_us              = s.pen_down_us;
}

// ───── debounced / idle-gated save ─────
//
// EEPROM.commit() on RP2040 erases and rewrites a whole flash sector — it's
// blocking, takes tens of milliseconds, and leaves the main loop unable to
// service realtime serial commands (like `?`) while it runs. Back-to-back
// $N= commands (e.g. what the tuning script sends) used to fire a commit
// per command, which piled up and made UGS think the port was stuck.
//
// Now we just mark a dirty bit and let settings_service_save() flush once
// the stepper is idle AND some time has passed since the last change.
static volatile bool     _dirty = false;
static volatile uint32_t _dirty_since_ms = 0;

// How long to wait after the LAST setting change before flushing to flash.
// Keeps rapid $N= bursts coalesced into a single write.
static const uint32_t SAVE_DEBOUNCE_MS = 500;

void settings_mark_dirty() {
    _dirty_since_ms = millis();
    _dirty = true;
}

static void do_flash_write() {
    SavedSettings s;
    s.magic                   = SETTINGS_MAGIC;
    s.version                 = SETTINGS_VERSION;
    s.max_feedrate_mm_min     = planner_max_feedrate_mm_min;
    s.max_rapid_mm_min        = planner_max_rapid_mm_min;
    s.motor_max_rate_mm_min   = planner_motor_max_rate_mm_min;
    s.max_accel_mm_s2         = planner_max_accel_mm_s2;
    s.motor_max_accel_mm_s2   = planner_motor_max_accel_mm_s2;
    s.max_x_mm                = work_area_max_x_mm;
    s.max_y_mm                = work_area_max_y_mm;
    s.junction_deviation_mm   = planner_junction_deviation_mm;
    s.arc_tolerance_mm        = planner_arc_tolerance_mm;
    s.pen_up_us               = servo_pen_up_us;
    s.pen_down_us             = servo_pen_down_us;
    EEPROM.put(0, s);
    EEPROM.commit();
}

void settings_restore_defaults() {
    planner_max_feedrate_mm_min    = DEFAULT_MAX_FEEDRATE_MM_MIN;
    planner_max_rapid_mm_min       = DEFAULT_MAX_RAPID_MM_MIN;
    planner_max_accel_mm_s2        = DEFAULT_MAX_ACCEL_MM_S2;
    planner_motor_max_accel_mm_s2  = MOTOR_MAX_ACCEL_MM_S2;
    planner_motor_max_rate_mm_min  = MOTOR_MAX_RATE_MM_MIN;
    planner_junction_deviation_mm  = DEFAULT_JUNCTION_DEVIATION;
    planner_arc_tolerance_mm       = DEFAULT_ARC_TOLERANCE;
    work_area_max_x_mm             = DEFAULT_MAX_X_MM;
    work_area_max_y_mm             = DEFAULT_MAX_Y_MM;
    servo_pen_up_us                = DEFAULT_PEN_UP_US;
    servo_pen_down_us              = DEFAULT_PEN_DOWN_US;
}

void settings_service_save() {
    if (!_dirty) return;
    uint32_t now = millis();
    // Coalesce bursts: wait for a quiet window after the last change.
    if ((int32_t)(now - _dirty_since_ms) < (int32_t)SAVE_DEBOUNCE_MS) return;
    // Avoid stalling in-progress motion — a flash erase can freeze the
    // main loop for ~50 ms, during which realtime serial goes unanswered.
    if (!stepper_is_idle()) return;

    _dirty = false;  // clear first, so a subsequent change during the
                     // commit re-raises it and gets saved next cycle.
    do_flash_write();
}
