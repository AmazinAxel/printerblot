#include "settings.h"
#include "planner.h"
#include <EEPROM.h>
#include <Arduino.h>

// Persistent settings owned here. Every setting's default lives only in the
// SETTINGS table below, applied by settings_load() before anything runs.
float work_area_max_x_mm;
float work_area_max_y_mm;
int   servo_pen_up_us;
int   servo_pen_down_us;
bool  quality_poster;

// On-flash layout. Bump VERSION if the struct changes — older blobs will
// be treated as invalid and compile defaults will be used until the next
// save.
static const uint32_t SETTINGS_MAGIC   = 0xB107B107u;
// v2 added soft_limits_enabled. A v1 blob is rejected outright rather than
// partially loaded — which means an existing machine silently reverts to
// compile-time defaults on the first boot after this upgrade. That's the
// safe direction (soft limits default ON), but recalibrate afterwards.
static const uint32_t SETTINGS_VERSION = 2u;

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
    float    unused;               // was the arc tolerance; kept for layout
    int      pen_up_us;
    int      pen_down_us;
    bool     soft_limits_enabled;
    // Added in what used to be tail padding, so the struct is unchanged in
    // size and v2 blobs still load: every save zero-inits the struct, so an
    // older blob reads this as 0 = draft. No version bump, no reset.
    bool     quality_poster;
};

#define EEPROM_BYTES  (sizeof(SavedSettings) + 16)

// Every setting, once: its id (GRBL's $ numbering, which docs/CALIBRATION.md
// uses), where it lives, its compile-time default and its slot in the flash
// blob. Defaults, settings_set(), load and save all walk this table.
enum SettingType : uint8_t { T_FLOAT, T_INT, T_BOOL };
enum : uint8_t {
    NEG_OK  = 1,  // may be negative (the work envelope)
    NONZERO = 2,  // a divisor in the planner: must be > 0
};
struct Setting {
    uint8_t     id;
    SettingType type;
    uint8_t     flags;
    uint8_t     offset;    // into SavedSettings
    void       *ptr;
    float       def;
};
#define SLOT(field) (uint8_t)offsetof(SavedSettings, field)

static const Setting SETTINGS[] = {
    { 110, T_FLOAT, NONZERO, SLOT(max_feedrate_mm_min),   &planner_max_feedrate_mm_min,   DEFAULT_MAX_FEEDRATE_MM_MIN },
    { 111, T_FLOAT, NONZERO, SLOT(max_rapid_mm_min),      &planner_max_rapid_mm_min,      DEFAULT_MAX_RAPID_MM_MIN },
    { 112, T_FLOAT, NONZERO, SLOT(motor_max_rate_mm_min), &planner_motor_max_rate_mm_min, MOTOR_MAX_RATE_MM_MIN },
    { 120, T_FLOAT, NONZERO, SLOT(max_accel_mm_s2),       &planner_max_accel_mm_s2,       DEFAULT_MAX_ACCEL_MM_S2 },
    { 122, T_FLOAT, NONZERO, SLOT(motor_max_accel_mm_s2), &planner_motor_max_accel_mm_s2, MOTOR_MAX_ACCEL_MM_S2 },
    // Safe default: there are no limit switches.
    {  20, T_BOOL,  0,       SLOT(soft_limits_enabled),   &planner_soft_limits_enabled,   1 },
    { 130, T_FLOAT, NEG_OK,  SLOT(max_x_mm),              &work_area_max_x_mm,            DEFAULT_MAX_X_MM },
    { 131, T_FLOAT, NEG_OK,  SLOT(max_y_mm),              &work_area_max_y_mm,            DEFAULT_MAX_Y_MM },
    { 140, T_FLOAT, 0,       SLOT(junction_deviation_mm), &planner_junction_deviation_mm, DEFAULT_JUNCTION_DEVIATION },
    { 150, T_INT,   0,       SLOT(pen_up_us),             &servo_pen_up_us,               DEFAULT_PEN_UP_US },
    { 151, T_INT,   0,       SLOT(pen_down_us),           &servo_pen_down_us,             DEFAULT_PEN_DOWN_US },
};

static size_t value_size(const Setting &s) {
    return s.type == T_BOOL ? sizeof(bool) : 4;
}

static void put_value(const Setting &s, float v) {
    switch (s.type) {
        case T_INT:  *(int *)s.ptr = (int)v;      break;
        case T_BOOL: *(bool *)s.ptr = v != 0.0f;  break;
        default:     *(float *)s.ptr = v;         break;
    }
}

static void restore_defaults() {
    for (const Setting &s : SETTINGS) put_value(s, s.def);
    quality_poster = false;
}

void settings_load() {
    restore_defaults();
    EEPROM.begin(EEPROM_BYTES);
    SavedSettings s;
    EEPROM.get(0, s);
    if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) return;

    for (const Setting &st : SETTINGS)
        memcpy(st.ptr, (const uint8_t *)&s + st.offset, value_size(st));
    quality_poster = s.quality_poster;
}

bool settings_set(int id, float v) {
    for (const Setting &s : SETTINGS) {
        if (s.id != id) continue;
        if (v < 0.0f && !(s.flags & NEG_OK)) return false;
        if ((s.flags & NONZERO) && v <= 0.0f) return false;
        put_value(s, v);
        return true;
    }
    return false;
}

// EEPROM.commit() erases and rewrites a whole flash sector — tens of ms on
// the loop's core. That's fine at any time: the step ISR runs from IRAM and
// keeps stepping through it, and a full planner covers far longer.
void settings_save() {
    // Zero-init so the struct's tail padding is deterministic — the memcmp
    // below compares raw bytes, and indeterminate padding would make an
    // unchanged blob look different and burn an erase cycle every time.
    SavedSettings s = {};
    s.magic   = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;
    for (const Setting &st : SETTINGS)
        memcpy((uint8_t *)&s + st.offset, st.ptr, value_size(st));
    s.quality_poster = quality_poster;

    // Skip the write when nothing actually changed. Flash endurance is finite
    // (~100k erase cycles) and the web UI re-sends the whole quality preset on
    // every toggle, so without this a few hundred UI clicks would each burn an
    // erase cycle for identical data.
    SavedSettings cur;
    EEPROM.get(0, cur);
    if (memcmp(&cur, &s, sizeof(s)) == 0) return;

    EEPROM.put(0, s);
    EEPROM.commit();
}
