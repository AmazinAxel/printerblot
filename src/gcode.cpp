#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "protocol.h"
#include "arc.h"
#include "settings.h"
#include <ctype.h>
#include <stdlib.h>
#include <Arduino.h>  // for yield() — services USB during tight loops

// Modal state — persists between lines per GRBL/NIST convention. Servo
// pulse widths live in settings.cpp so they survive reset and are saved
// to flash; only pen_down state (open/closed) lives here.
typedef struct {
    uint8_t motion;        // 0=G0, 1=G1, 2=G2, 3=G3
    uint8_t units;         // 20=inches, 21=mm
    uint8_t distance;      // 90=abs, 91=rel
    float   feedrate;      // mm/min (already in mm after units normalization)
    // G92 offset: programmed_position = machine_position + offset
    float   offset[2];
    bool    pen_down;      // true if pen is currently lowered
} ModalState;
static ModalState modal;

// Tracks what the last *queued* SYNC_SET_ENABLE asked for, so motion lines
// only queue an enable when the drivers were actually turned off. Matches
// stepper_init(), which energises the A4988s at boot.
static bool drivers_enabled = true;

static void set_defaults() {
    modal.motion    = 1;
    modal.units     = 21;
    modal.distance  = 90;
    modal.feedrate  = DEFAULT_MAX_FEEDRATE_MM_MIN / 2.0f;
    modal.offset[0] = modal.offset[1] = 0.0f;
    modal.pen_down  = false;
}

void gcode_reset() {
    set_defaults();
    // A soft reset flushes the planner, so any queued enable/disable is
    // discarded and the drivers keep whatever state the last *executed* block
    // left on the pin. Read it back rather than assuming: after an idle
    // auto-unlock the motors are off, and guessing "enabled" here would skip
    // the enable the next move needs.
    drivers_enabled = (digitalRead(PIN_DRIVER_EN) == LOW);  // A4988: LOW = on
}

// ───────── tokenizer / parser ─────────

struct Line {
    uint32_t present;
    float    val[26];
    int      g_codes[8];
    uint8_t  g_count;
    int      m_codes[4];
    uint8_t  m_count;
};

static void line_init(Line *ln) {
    ln->present = 0;
    ln->g_count = 0;
    ln->m_count = 0;
}

static bool line_has(const Line *ln, char letter) {
    return ln->present & (1u << (letter - 'A'));
}

static float line_get(const Line *ln, char letter) {
    return ln->val[letter - 'A'];
}

static void sanitize(char *s) {
    char *w = s;
    bool in_paren = false;
    for (char *r = s; *r; r++) {
        char c = *r;
        if (in_paren) { if (c == ')') in_paren = false; continue; }
        if (c == ';') break;
        if (c == '(') { in_paren = true; continue; }
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
        *w++ = (char)toupper((unsigned char)c);
    }
    *w = '\0';
}

static bool parse_number(const char *p, char **endp, float *out) {
    const char *start = p;
    bool neg = false;
    if (*p == '-') { neg = true; p++; }
    else if (*p == '+') p++;
    float val = 0.0f;
    bool has_digits = false;
    while (*p >= '0' && *p <= '9') {
        val = val * 10.0f + (float)(*p - '0');
        p++; has_digits = true;
    }
    if (*p == '.') {
        p++;
        float frac = 0.1f;
        while (*p >= '0' && *p <= '9') {
            val += (float)(*p - '0') * frac;
            frac *= 0.1f; p++; has_digits = true;
        }
    }
    if (!has_digits) { *endp = (char *)start; return false; }
    *out = neg ? -val : val;
    *endp = (char *)p;
    return true;
}

static StatusCode tokenize(char *s, Line *ln) {
    line_init(ln);
    char *p = s;
    while (*p) {
        char letter = *p;
        if (letter < 'A' || letter > 'Z') return STATUS_EXPECTED_COMMAND;
        p++;
        char *endp = nullptr;
        float v = 0.0f;
        if (!parse_number(p, &endp, &v)) return STATUS_BAD_NUMBER_FORMAT;
        // Rounding a decimal code would silently run a different command —
        // G92.1 (clear offsets) as G92 (set them). G92.1 is the only one we
        // implement, stored as 921; any other decimal code is unsupported.
        bool whole = fabsf(v - roundf(v)) < 1e-3f;
        if (letter == 'G') {
            if (ln->g_count >= 8) return STATUS_MODAL_GROUP_CONFLICT;
            int g = (int)lroundf(v);
            if (!whole) {
                if (lroundf(v * 10.0f) != 921) return STATUS_UNSUPPORTED_COMMAND;
                g = 921;
            }
            ln->g_codes[ln->g_count++] = g;
        } else if (letter == 'M') {
            if (ln->m_count >= 4) return STATUS_MODAL_GROUP_CONFLICT;
            if (!whole) return STATUS_UNSUPPORTED_COMMAND;
            ln->m_codes[ln->m_count++] = (int)lroundf(v);
        } else {
            ln->val[letter - 'A'] = v;
            ln->present |= (1u << (letter - 'A'));
        }
        p = endp;
    }
    return STATUS_OK;
}

// apply_modal() lives below — it's large. Forward-declare.
static StatusCode apply_modal(Line *ln);

// ───────── settings ─────────
// Changes are held in RAM and flushed to flash lazily by settings.cpp.

static StatusCode handle_dollar(char *line) {
    // `$$` / `$`   → dump settings
    // `$RST=*`     → restore compile-time defaults
    // `$N=value`   → set setting N
    if (line[1] == '\0' || line[1] == '$') {
        // Only the settings this firmware actually implements. The GRBL
        // $0-$32 stubs (homing, limits, spindle, laser, pin inverts) existed
        // for UGS's Firmware Settings panel; nothing reads them now.
        Serial.print(F("$100=")); Serial.println(STEPS_PER_MM, 3);
        Serial.print(F("$101=")); Serial.println(STEPS_PER_MM, 3);  // Y mirror of X
        Serial.print(F("$110=")); Serial.println(planner_max_feedrate_mm_min, 3);
        Serial.print(F("$111=")); Serial.println(planner_max_rapid_mm_min, 3);
        Serial.print(F("$112=")); Serial.println(planner_motor_max_rate_mm_min, 3);
        Serial.print(F("$120=")); Serial.println(planner_max_accel_mm_s2, 3);
        Serial.print(F("$121=")); Serial.println(planner_max_accel_mm_s2, 3);  // Y mirror
        Serial.print(F("$122=")); Serial.println(planner_motor_max_accel_mm_s2, 3);
        Serial.print(F("$20=")); Serial.println(planner_soft_limits_enabled ? 1 : 0);
        Serial.print(F("$130=")); Serial.println(work_area_max_x_mm, 3);
        Serial.print(F("$131=")); Serial.println(work_area_max_y_mm, 3);
        Serial.print(F("$140=")); Serial.println(planner_junction_deviation_mm, 4);
        Serial.print(F("$141=")); Serial.println(planner_arc_tolerance_mm, 4);
        Serial.print(F("$150=")); Serial.println(servo_pen_up_us);
        Serial.print(F("$151=")); Serial.println(servo_pen_down_us);
        return STATUS_OK;
    }
    // `$RST=*` / `$RST=$` / `$RST=#` — reset all settings to compile defaults
    // (GRBL convention). We don't distinguish scopes; all reset everything.
    if (line[1] == 'R' && line[2] == 'S' && line[3] == 'T' && line[4] == '=') {
        char scope = line[5];
        if (scope == '*' || scope == '$' || scope == '#') {
            settings_restore_defaults();
            settings_mark_dirty();
            return STATUS_OK;
        }
        return STATUS_INVALID_STATEMENT;
    }

    // `$N=value`
    char *eq = strchr(line, '=');
    if (!eq) return STATUS_INVALID_STATEMENT;
    int id = atoi(line + 1);
    char *vend = nullptr;
    float v = 0.0f;
    if (!parse_number(eq + 1, &vend, &v)) return STATUS_BAD_NUMBER_FORMAT;
    // Trailing garbage after the number is a format error too.
    if (*vend != '\0') return STATUS_BAD_NUMBER_FORMAT;
    if (v < 0.0f && id != 130 && id != 131) return STATUS_NEGATIVE_VALUE;
    // Speeds and accelerations are divisors in the planner.
    bool divisor = id == 110 || id == 111 || id == 112 || id == 120 || id == 122;
    if (divisor && v <= 0.0f) return STATUS_INVALID_STATEMENT;
    // Only an actual change is persisted: the web UI re-sends the whole
    // quality preset on every toggle, and each save is a flash sector erase.
    bool changed = false;
    auto setf = [&](float &dst) { changed = dst != v; dst = v; };
    auto seti = [&](int &dst) { changed = dst != (int)v; dst = (int)v; };
    switch (id) {
        case 20: {
            bool on = v != 0.0f;
            changed = planner_soft_limits_enabled != on;
            planner_soft_limits_enabled = on;
            break;
        }
        case 110: setf(planner_max_feedrate_mm_min);    break;
        case 111: setf(planner_max_rapid_mm_min);       break;
        case 112: setf(planner_motor_max_rate_mm_min);  break;
        case 120: setf(planner_max_accel_mm_s2);        break;
        case 122: setf(planner_motor_max_accel_mm_s2);  break;
        case 130: setf(work_area_max_x_mm);             break;
        case 131: setf(work_area_max_y_mm);             break;
        case 140: setf(planner_junction_deviation_mm);  break;
        case 141: setf(planner_arc_tolerance_mm);       break;
        case 150: seti(servo_pen_up_us);                break;
        case 151: seti(servo_pen_down_us);              break;
        default: return STATUS_UNSUPPORTED_COMMAND;
    }
    if (changed) settings_mark_dirty();  // persisted lazily when idle + debounced
    return STATUS_OK;
}

// ───────── execution ─────────

static StatusCode apply_modal(Line *ln) {
    // Distinguish group 1 motion (G0/G1/G2/G3/G28), group 6 units (G20/G21),
    // group 3 distance (G90/G91), non-modal (G4, G92).
    bool motion_set = false, units_set = false, dist_set = false;
    int  non_modal = 0;  // 0=none, 4, 28, 92, 921 (G92.1)

    for (uint8_t i = 0; i < ln->g_count; i++) {
        int g = ln->g_codes[i];
        switch (g) {
            case 0: case 1: case 2: case 3:
                if (motion_set) return STATUS_MODAL_GROUP_CONFLICT;
                modal.motion = (uint8_t)g; motion_set = true; break;
            case 4:
                if (non_modal) return STATUS_MODAL_GROUP_CONFLICT;
                non_modal = 4; break;
            case 20:
                if (units_set) return STATUS_MODAL_GROUP_CONFLICT;
                modal.units = 20; units_set = true; break;
            case 21:
                if (units_set) return STATUS_MODAL_GROUP_CONFLICT;
                modal.units = 21; units_set = true; break;
            case 28:
                if (non_modal || motion_set) return STATUS_MODAL_GROUP_CONFLICT;
                non_modal = 28; break;
            case 90:
                if (dist_set) return STATUS_MODAL_GROUP_CONFLICT;
                modal.distance = 90; dist_set = true; break;
            case 91:
                if (dist_set) return STATUS_MODAL_GROUP_CONFLICT;
                modal.distance = 91; dist_set = true; break;
            case 92: case 921:
                if (non_modal) return STATUS_MODAL_GROUP_CONFLICT;
                non_modal = g; break;
            default:
                return STATUS_UNSUPPORTED_COMMAND;
        }
    }

    // F is a modal feedrate setting.
    if (line_has(ln, 'F')) {
        float f = line_get(ln, 'F');
        if (f < 0.0f) return STATUS_NEGATIVE_VALUE;
        // Normalize inches/min → mm/min
        if (modal.units == 20) f *= 25.4f;
        modal.feedrate = f;
    }

    // Wait for a planner slot before queueing a sync command. Without this,
    // a sync (M3/M5/M17/M18/G4) immediately after many motion lines would
    // fail with error:60 (STATUS_OVERFLOW) whenever the buffer was full
    // from the preceding motion — e.g., M5 at the end of a long arc-heavy
    // pattern. The motion path already waits this way; sync commands
    // should behave the same.
    //
    // In feed hold the wait gives up instead of spinning until the 30s
    // watchdog reboots mid-print; that becomes error:60, which is
    // recoverable. A reset arriving meanwhile aborts the line; its status is
    // never sent.
#define WAIT_FOR_SLOT() do { \
        if (!protocol_wait_for_planner_slot()) \
            return protocol_reset_pending() ? STATUS_OK : STATUS_OVERFLOW; \
    } while (0)

    // M-codes: pen (M3/M5), motors (M17/M18), pause/end (M0/M2).
    for (uint8_t i = 0; i < ln->m_count; i++) {
        int m = ln->m_codes[i];
        switch (m) {
            case 3: {
                int us = line_has(ln, 'S') ? (int)line_get(ln, 'S')
                                           : servo_pen_down_us;
                if (line_has(ln, 'S') && us != servo_pen_down_us) {
                    servo_pen_down_us = us;
                    settings_mark_dirty();
                }
                modal.pen_down = true;
                WAIT_FOR_SLOT();
                if (!plan_sync_command(SYNC_SERVO, us)) return STATUS_OVERFLOW;
                break;
            }
            case 5: {
                modal.pen_down = false;
                WAIT_FOR_SLOT();
                if (!plan_sync_command(SYNC_SERVO, servo_pen_up_us))
                    return STATUS_OVERFLOW;
                break;
            }
            case 17:
                WAIT_FOR_SLOT();
                if (!plan_sync_command(SYNC_SET_ENABLE, 1)) return STATUS_OVERFLOW;
                drivers_enabled = true;
                break;
            case 18: case 84:
                WAIT_FOR_SLOT();
                if (!plan_sync_command(SYNC_SET_ENABLE, 0)) return STATUS_OVERFLOW;
                drivers_enabled = false;
                break;
            case 0: case 1:
                // Program pause — treat as feed hold that releases on ~
                stepper_feed_hold();
                break;
            case 2: case 30:
                // Program end — do nothing meaningful in a streaming plotter.
                break;
            default:
                return STATUS_UNSUPPORTED_COMMAND;
        }
    }

    // Non-modal: G4 dwell
    if (non_modal == 4) {
        int32_t ms = 0;
        if (line_has(ln, 'P')) ms = (int32_t)line_get(ln, 'P');
        else if (line_has(ln, 'S')) ms = (int32_t)(line_get(ln, 'S') * 1000.0f);
        if (ms < 0) ms = 0;
        WAIT_FOR_SLOT();
        if (!plan_sync_command(SYNC_DWELL, ms)) return STATUS_OVERFLOW;
    }

    // G92: set current position as new work origin for subsequent words.
    if (non_modal == 92) {
        float x, y;
        plan_get_position(&x, &y);
        // New offset = current_machine - (new_work_position requested)
        if (line_has(ln, 'X')) {
            float nx = line_get(ln, 'X');
            if (modal.units == 20) nx *= 25.4f;
            modal.offset[0] = x - nx;
        }
        if (line_has(ln, 'Y')) {
            float ny = line_get(ln, 'Y');
            if (modal.units == 20) ny *= 25.4f;
            modal.offset[1] = y - ny;
        }
    }

    // G92.1: clear the G92 offsets; work coordinates = machine coordinates.
    if (non_modal == 921) {
        modal.offset[0] = 0.0f;
        modal.offset[1] = 0.0f;
    }

    // G28: move to (0, 0) in work coordinates
    if (non_modal == 28) {
        float tx = modal.offset[0];
        float ty = modal.offset[1];
        // The G92 origin can itself sit outside the envelope, so G28 is not
        // implicitly safe — check it like any other move.
        if (!plan_target_in_bounds(tx, ty)) return STATUS_TRAVEL_EXCEEDED;
        WAIT_FOR_SLOT();
        if (!plan_buffer_line(tx, ty, planner_max_rapid_mm_min, true))
            return STATUS_OVERFLOW;
    }

    // ───── motion (G0/G1/G2/G3) ─────
    bool has_axis_word = line_has(ln, 'X') || line_has(ln, 'Y');
    if (has_axis_word && non_modal == 0) {
        // Auto-enable stepper drivers on any motion command so the user
        // doesn't have to send M17 after an M18 at end-of-program.
        //
        // Queued, not written directly: parsing runs well ahead of execution,
        // so an immediate write would re-energise the motors while an earlier
        // M18 was still draining through the planner — and would defeat the
        // idle power-down, since the drivers would come back on the moment a
        // later line was parsed. `drivers_enabled` keeps this to one sync
        // block per M18→motion transition instead of one per motion line.
        if (!drivers_enabled) {
            WAIT_FOR_SLOT();
            if (!plan_sync_command(SYNC_SET_ENABLE, 1)) return STATUS_OVERFLOW;
            drivers_enabled = true;
        }

        // Compute target in machine coordinates.
        float cur_x, cur_y;
        plan_get_position(&cur_x, &cur_y);

        float wx = cur_x - modal.offset[0];  // current work position
        float wy = cur_y - modal.offset[1];
        float tx_work = wx;
        float ty_work = wy;

        if (line_has(ln, 'X')) {
            float vx = line_get(ln, 'X');
            if (modal.units == 20) vx *= 25.4f;
            tx_work = (modal.distance == 90) ? vx : (wx + vx);
        }
        if (line_has(ln, 'Y')) {
            float vy = line_get(ln, 'Y');
            if (modal.units == 20) vy *= 25.4f;
            ty_work = (modal.distance == 90) ? vy : (wy + vy);
        }

        float tx = tx_work + modal.offset[0];
        float ty = ty_work + modal.offset[1];

        if (modal.motion == 0 || modal.motion == 1) {
            float fr = (modal.motion == 0) ? planner_max_rapid_mm_min
                                           : modal.feedrate;
            // Check bounds before waiting for a slot: an out-of-range move
            // should report immediately, not block until the buffer drains.
            if (!plan_target_in_bounds(tx, ty)) return STATUS_TRAVEL_EXCEEDED;
            WAIT_FOR_SLOT();
            plan_buffer_line(tx, ty, fr, modal.motion == 0);
        } else if (modal.motion == 2 || modal.motion == 3) {
            // G2/G3: I/J are center offsets from current position (mm).
            float i = line_has(ln, 'I') ? line_get(ln, 'I') : 0.0f;
            float j = line_has(ln, 'J') ? line_get(ln, 'J') : 0.0f;
            if (modal.units == 20) { i *= 25.4f; j *= 25.4f; }
            if (!line_has(ln, 'I') && !line_has(ln, 'J') && line_has(ln, 'R')) {
                // R-form: resolve to I/J. This is geometry most senders already
                // expand, but we accept it for completeness.
                float r = line_get(ln, 'R');
                if (modal.units == 20) r *= 25.4f;
                float dx = tx - cur_x;
                float dy = ty - cur_y;
                float dsq = dx * dx + dy * dy;
                if (dsq < 1e-10f) return STATUS_ARC_RADIUS_ERROR;
                float h_sq = r * r - dsq * 0.25f;
                if (h_sq < 0.0f) return STATUS_ARC_RADIUS_ERROR;
                float h = sqrtf(h_sq);
                // Offset midpoint toward center
                float mx = (cur_x + tx) * 0.5f;
                float my = (cur_y + ty) * 0.5f;
                // Perpendicular (left-hand if R>0, swapped for CW)
                float inv_d = 1.0f / sqrtf(dsq);
                float px = -dy * inv_d;
                float py =  dx * inv_d;
                // For G2 (CW) + R>0, center is on the right of travel; flip.
                bool cw = (modal.motion == 2);
                bool right_side = (cw && r > 0.0f) || (!cw && r < 0.0f);
                if (right_side) { px = -px; py = -py; }
                float cx = mx + px * h;
                float cy = my + py * h;
                i = cx - cur_x;
                j = cy - cur_y;
            }
            bool cw = (modal.motion == 2);
            bool oob = false;
            if (!arc_interpolate(tx, ty, i, j, cw, modal.feedrate, &oob))
                return oob ? STATUS_TRAVEL_EXCEEDED : STATUS_ARC_RADIUS_ERROR;
        }
    }

    // Wake the stepper unconditionally. Any path above (motion, sync
    // M3/M5/M17/M18, G4 dwell, G28) may have queued a block; without this
    // kick, sync-only commands like `M18` stay in the planner until the
    // next motion arrives.
    stepper_wake();
    return STATUS_OK;
}

StatusCode gcode_execute_line(char *line) {
    sanitize(line);
    if (line[0] == '\0') return STATUS_OK;

    if (line[0] == '$') return handle_dollar(line);

    Line ln;
    StatusCode s = tokenize(line, &ln);
    if (s != STATUS_OK) return s;
    return apply_modal(&ln);
}

