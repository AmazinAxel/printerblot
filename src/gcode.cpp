#include "gcode.h"
#include "planner.h"
#include "stepper.h"
#include "settings.h"
#include <ctype.h>
#include <math.h>

// Supported: exactly what the web UI's generators emit (convert.js and the
// sticker tool) — G0/G1, G21, G90, G92, M3, M5. See
// docs/SUPPORTED_COMMANDS.md.
//
// Anything else is either harmless and skipped (G17, G64, M18, M30...) or
// would put geometry in the wrong place if ignored (arcs, inches, relative
// mode...) and is an error, which aborts the print rather than drawing it
// wrong.

// Modal state, persisting between lines.
static bool  rapid;       // G0 (true) or G1
static float feedrate;    // mm/min
static float offset[2];   // G92: machine = work + offset

void gcode_reset() {
    rapid = false;
    feedrate = DEFAULT_MAX_FEEDRATE_MM_MIN / 2.0f;
    offset[0] = offset[1] = 0.0f;
}

// ───────── tokenizer ─────────

struct Line {
    uint32_t present;     // bit per letter A-Z, for non-G/M words
    float    val[26];
    int      g_codes[4];
    uint8_t  g_count;
    int      m_codes[2];  // see MAX_BLOCKS_PER_LINE
    uint8_t  m_count;
};

static bool line_has(const Line *ln, char letter) {
    return ln->present & (1u << (letter - 'A'));
}

static float line_get(const Line *ln, char letter) {
    return ln->val[letter - 'A'];
}

// Strip whitespace, ( ) and ; comments, and fold to upper case, in place.
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

static bool parse_number(const char *p, const char **endp, float *out) {
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
    *endp = has_digits ? p : start;
    if (has_digits) *out = neg ? -val : val;
    return has_digits;
}

static StatusCode tokenize(const char *p, Line *ln) {
    ln->present = 0;
    ln->g_count = ln->m_count = 0;
    while (*p) {
        char letter = *p++;
        if (letter < 'A' || letter > 'Z') return STATUS_ERROR;
        float v;
        if (!parse_number(p, &p, &v)) return STATUS_ERROR;
        if (letter == 'G' || letter == 'M') {
            // Decimal codes (G92.1, G61.1, ...) are never ours. Rounding one
            // would silently run a different command, so they're -1: unknown.
            int code = fabsf(v - roundf(v)) < 1e-3f ? (int)lroundf(v) : -1;
            if (letter == 'G') {
                if (ln->g_count == 4) return STATUS_ERROR;
                ln->g_codes[ln->g_count++] = code;
            } else {
                if (ln->m_count == 2) return STATUS_ERROR;
                ln->m_codes[ln->m_count++] = code;
            }
        } else {
            ln->val[letter - 'A'] = v;
            ln->present |= (1u << (letter - 'A'));
        }
    }
    return STATUS_OK;
}

// ───────── execution ─────────

static StatusCode execute(const Line *ln) {
    bool motion_set = false, set_origin = false, skipped = false;

    for (uint8_t i = 0; i < ln->g_count; i++) {
        switch (ln->g_codes[i]) {
            case 0: case 1:
                if (motion_set) return STATUS_ERROR;
                rapid = ln->g_codes[i] == 0;
                motion_set = true;
                break;
            case 21: case 90:  // mm, absolute: the only modes there are
                break;
            case 92:
                set_origin = true;
                break;
            // Ignoring these would draw in the wrong place or scale.
            case -1:                 // decimal codes, e.g. G92.1
            case 2: case 3:          // arcs
            case 20:                 // inches
            case 28: case 30:        // go home / predefined position
            case 53:                 // machine-coordinate move
            case 91:                 // relative distances
                return STATUS_ERROR;
            default:                 // G17, G40, G64, G94, ...: no geometry
                skipped = true;
                break;
        }
    }

    if (line_has(ln, 'F')) {
        float f = line_get(ln, 'F');
        if (f < 0.0f) return STATUS_ERROR;
        feedrate = f;
    }

    // Pen. Queued as sync blocks so it changes at the right point in the
    // motion stream, not when this line is parsed (parsing runs ahead).
    // Never full (MAX_BLOCKS_PER_LINE), but a dropped pen command would draw
    // the rest of the job wrong, so it's an error rather than ignored.
    for (uint8_t i = 0; i < ln->m_count; i++) {
        int us;
        switch (ln->m_codes[i]) {
            case 3: us = servo_pen_down_us; break;
            case 5: us = servo_pen_up_us;   break;
            default: skipped = true; continue;  // M18, M30, ...: motors and
                                                // the end are handled already
        }
        if (!plan_sync_command(SYNC_SERVO, us)) return STATUS_ERROR;
    }

    float cur_x, cur_y;
    plan_get_position(&cur_x, &cur_y);

    if (set_origin) {
        // The current position becomes the given work coordinates.
        if (line_has(ln, 'X')) offset[0] = cur_x - line_get(ln, 'X');
        if (line_has(ln, 'Y')) offset[1] = cur_y - line_get(ln, 'Y');
    } else if (line_has(ln, 'X') || line_has(ln, 'Y')) {
        float tx = line_has(ln, 'X') ? line_get(ln, 'X') + offset[0] : cur_x;
        float ty = line_has(ln, 'Y') ? line_get(ln, 'Y') + offset[1] : cur_y;
        // False only for a target outside the envelope: the job feeder
        // guarantees the room (MAX_BLOCKS_PER_LINE).
        if (!plan_buffer_line(tx, ty, rapid ? planner_max_rapid_mm_min : feedrate,
                              rapid))
            return STATUS_ERROR;
    }

    stepper_wake();
    return skipped ? STATUS_SKIPPED : STATUS_OK;
}

StatusCode gcode_execute_line(char *line) {
    sanitize(line);
    if (line[0] == '\0') return STATUS_OK;
    Line ln;
    StatusCode s = tokenize(line, &ln);
    return s == STATUS_OK ? execute(&ln) : s;
}
