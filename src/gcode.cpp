#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "arc.h"
#include "settings.h"
#include "pico/time.h"
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
}

// ───────── tokenizer / parser ─────────
// Kept above handle_dollar() so $J= can reuse the tokenizer.

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
        if (letter == 'G') {
            if (ln->g_count >= 8) return STATUS_MODAL_GROUP_CONFLICT;
            ln->g_codes[ln->g_count++] = (int)lroundf(v);
        } else if (letter == 'M') {
            if (ln->m_count >= 4) return STATUS_MODAL_GROUP_CONFLICT;
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

// ───────── settings (persist in RAM only for V1) ─────────

static StatusCode handle_dollar(char *line) {
    // `$$`  → dump settings
    // `$X`  → clear alarm (we have no alarms; treat as no-op ok)
    // `$H`  → home (unsupported — error)
    // `$N=value` → set setting N
    if (line[1] == '\0' || line[1] == '$') {
        // UGS's Firmware Settings panel expects the standard GRBL $0-$32
        // namespace. Emitting plausible stubs for the ones Blot doesn't use
        // keeps the panel from flagging the firmware as broken and stops
        // UGS from mis-attributing errors on subsequent commands.
        Serial.print(F("$0=10\r\n"));       // step pulse µs (unused on RP2040)
        Serial.print(F("$1=25\r\n"));       // step idle delay
        Serial.print(F("$2=0\r\n"));        // step port invert
        Serial.print(F("$3=0\r\n"));        // direction port invert
        Serial.print(F("$4=0\r\n"));        // step enable invert
        Serial.print(F("$5=0\r\n"));        // limit pins invert
        Serial.print(F("$6=0\r\n"));        // probe pin invert
        Serial.print(F("$10=1\r\n"));       // status report mask
        Serial.print(F("$11=")); Serial.println(planner_junction_deviation_mm, 3);
        Serial.print(F("$12=")); Serial.println(planner_arc_tolerance_mm, 4);
        Serial.print(F("$13=0\r\n"));       // report inches
        Serial.print(F("$20=0\r\n"));       // soft limits
        Serial.print(F("$21=0\r\n"));       // hard limits
        Serial.print(F("$22=0\r\n"));       // homing cycle (disabled — manual G92)
        Serial.print(F("$23=0\r\n"));       // homing dir invert
        Serial.print(F("$24=25.000\r\n"));  // homing feed
        Serial.print(F("$25=500.000\r\n")); // homing seek
        Serial.print(F("$26=250\r\n"));     // homing debounce
        Serial.print(F("$27=1.000\r\n"));   // homing pull-off
        Serial.print(F("$30=1000\r\n"));    // max spindle speed
        Serial.print(F("$31=0\r\n"));       // min spindle speed
        Serial.print(F("$32=0\r\n"));       // laser mode
        Serial.print(F("$100=")); Serial.println(STEPS_PER_MM, 3);
        Serial.print(F("$101=")); Serial.println(STEPS_PER_MM, 3);  // Y mirror of X
        Serial.print(F("$110=")); Serial.println(planner_max_feedrate_mm_min, 3);
        Serial.print(F("$111=")); Serial.println(planner_max_rapid_mm_min, 3);
        Serial.print(F("$112=")); Serial.println(planner_motor_max_rate_mm_min, 3);
        Serial.print(F("$120=")); Serial.println(planner_max_accel_mm_s2, 3);
        Serial.print(F("$121=")); Serial.println(planner_max_accel_mm_s2, 3);  // Y mirror
        Serial.print(F("$122=")); Serial.println(planner_motor_max_accel_mm_s2, 3);
        Serial.print(F("$130=")); Serial.println(work_area_max_x_mm, 3);
        Serial.print(F("$131=")); Serial.println(work_area_max_y_mm, 3);
        Serial.print(F("$140=")); Serial.println(planner_junction_deviation_mm, 4);
        Serial.print(F("$141=")); Serial.println(planner_arc_tolerance_mm, 4);
        Serial.print(F("$150=")); Serial.println(servo_pen_up_us);
        Serial.print(F("$151=")); Serial.println(servo_pen_down_us);
        return STATUS_OK;
    }
    if (line[1] == 'X' || line[1] == 'x') return STATUS_OK;
    if (line[1] == 'H' || line[1] == 'h') return STATUS_UNSUPPORTED_COMMAND;
    // `$C` check mode, `$N0=`/`$N1=` startup blocks, `$SLP` sleep — UGS
    // probes these at connect time. Accept as no-ops so it doesn't stash
    // errors and mis-attribute them to later lines.
    if (line[1] == 'C') return STATUS_OK;
    if (line[1] == 'N') {
        // `$N` alone → dump startup lines (empty).
        if (line[2] == '\0') {
            Serial.print(F("$N0=\r\n$N1=\r\n"));
            return STATUS_OK;
        }
        // `$N0=...` / `$N1=...` → accept & ignore.
        return STATUS_OK;
    }
    if (line[1] == 'S' && line[2] == 'L' && line[3] == 'P') return STATUS_OK;

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

    // `$I` → build info. UGS parses the VER line for version detection.
    if (line[1] == 'I') {
        Serial.print(F("[VER:1.1f.Blot:]\r\n[OPT:V,"));
        Serial.print(PLANNER_BUFFER_SIZE);
        Serial.print(F(","));
        Serial.print(RX_BUFFER_SIZE);
        Serial.print(F("]\r\n"));
        return STATUS_OK;
    }

    // `$J=...` → one-shot jog move. GRBL convention: the text after `=` is
    // parsed as a motion line but does NOT modify persistent modal state
    // (any G90/G91/G20/G21 inside is for this jog only). Used by UGS for
    // arrow-key jogging and step-jog.
    if (line[1] == 'J' && line[2] == '=') {
        ModalState saved = modal;
        modal.motion = 1;
        char *body = line + 3;   // skip "$J="
        if (*body == '\0') { modal = saved; return STATUS_EXPECTED_COMMAND; }
        Line ln;
        StatusCode s = tokenize(body, &ln);
        if (s == STATUS_OK) s = apply_modal(&ln);
        modal = saved;
        return s;
    }

    // `$G` → parser state (for UGS's "G-code State" panel).
    if (line[1] == 'G') {
        Serial.print(F("[GC:G"));
        Serial.print(modal.motion);
        Serial.print(F(" G"));
        Serial.print(modal.units);
        Serial.print(F(" G"));
        Serial.print(modal.distance);
        Serial.print(F(" G94 M"));
        Serial.print(modal.pen_down ? 3 : 5);
        Serial.print(F(" T0 F"));
        Serial.print(modal.feedrate, 0);
        Serial.print(F(" S"));
        Serial.print(modal.pen_down ? servo_pen_down_us : 0);
        Serial.print(F("]\r\n"));
        return STATUS_OK;
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
    switch (id) {
        case 110: planner_max_feedrate_mm_min    = v; break;
        case 111: planner_max_rapid_mm_min       = v; break;
        case 112: planner_motor_max_rate_mm_min  = v; break;
        case 120: planner_max_accel_mm_s2        = v; break;
        case 122: planner_motor_max_accel_mm_s2  = v; break;
        case 130: work_area_max_x_mm             = v; break;
        case 131: work_area_max_y_mm             = v; break;
        case 140: planner_junction_deviation_mm  = v; break;
        case 141: planner_arc_tolerance_mm       = v; break;
        case 150: servo_pen_up_us                = (int)v; break;
        case 151: servo_pen_down_us              = (int)v; break;
        default: return STATUS_UNSUPPORTED_COMMAND;
    }
    settings_mark_dirty();  // persisted lazily when idle + debounced
    return STATUS_OK;
}

// ───────── execution ─────────

static StatusCode apply_modal(Line *ln) {
    // Distinguish group 1 motion (G0/G1/G2/G3/G28), group 6 units (G20/G21),
    // group 3 distance (G90/G91), non-modal (G4, G92).
    bool motion_set = false, units_set = false, dist_set = false;
    int  non_modal = 0;  // 0=none, 4, 28, 92

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
            case 92:
                if (non_modal) return STATUS_MODAL_GROUP_CONFLICT;
                non_modal = 92; break;
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
    auto wait_for_slot = [&]() {
        while (plan_buffer_available() == 0) {
            stepper_wake();
            stepper_service_sync();
            yield();  // service USB so Linux doesn't reset us
        }
    };

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
                wait_for_slot();
                if (!plan_sync_command(SYNC_SERVO, us)) return STATUS_OVERFLOW;
                break;
            }
            case 5: {
                modal.pen_down = false;
                wait_for_slot();
                if (!plan_sync_command(SYNC_SERVO, servo_pen_up_us))
                    return STATUS_OVERFLOW;
                break;
            }
            case 17:
                wait_for_slot();
                if (!plan_sync_command(SYNC_SET_ENABLE, 1)) return STATUS_OVERFLOW;
                break;
            case 18: case 84:
                wait_for_slot();
                if (!plan_sync_command(SYNC_SET_ENABLE, 0)) return STATUS_OVERFLOW;
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
        wait_for_slot();
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

    // G28: move to (0, 0) in work coordinates
    if (non_modal == 28) {
        float tx = modal.offset[0];
        float ty = modal.offset[1];
        if (!plan_buffer_line(tx, ty, planner_max_rapid_mm_min, true))
            return STATUS_OVERFLOW;
    }

    // ───── motion (G0/G1/G2/G3) ─────
    bool has_axis_word = line_has(ln, 'X') || line_has(ln, 'Y');
    if (has_axis_word && non_modal == 0) {
        // Auto-enable stepper drivers on any motion command so the user
        // doesn't have to send M17 after an M18 at end-of-program.
        digitalWrite(PIN_DRIVER_EN, LOW);

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
            while (plan_buffer_available() == 0) {
                stepper_wake();
                stepper_service_sync();
                yield();
            }
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
            if (!arc_interpolate(tx, ty, i, j, cw, modal.feedrate))
                return STATUS_ARC_RADIUS_ERROR;
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

