#include "job.h"

#include <Arduino.h>
#include <LittleFS.h>

#include "config.h"
#include "gcode.h"
#include "planner.h"
#include "settings.h"
#include "stepper.h"

// Path the upload handler writes to. One job at a time.
const char *JOB_PATH = "/job.gcode";

// Feedrate for the return-to-origin move after a stop. Deliberately well
// under the machine's cap: this runs pen-up on an aborted job, and a gentle
// return is less likely to skip than a full-speed rapid.
static const uint32_t RETURN_RATE_MM_MIN = 3000;

// Keep at least this many planner slots free so a burst of short segments
// can't starve the parser; mirrors GRBL-style flow control.
static const uint8_t PLANNER_HEADROOM = 2;

// Minimum gap between progress pushes to connected browsers.
static const uint32_t PROGRESS_INTERVAL_MS = 500;
static uint32_t last_progress_ms = 0;

static File     job_file;
static size_t   job_size = 0;
static JobState state = JOB_IDLE;
// Percent of the file fed to the parser: byte position / size, so it costs
// nothing to know — no pre-pass to count lines. Runs a planner's depth ahead
// of the pen, which is invisible on a bar. Written only on loop()'s core, so
// the HTTP task never touches the File.
static volatile uint8_t progress_pct = 0;
static bool     pen_is_down = false;
// Set when a pause has actually lifted the pen (which happens once the
// planner drains, not when the button is pressed). Keeps resume from issuing
// an M3 that was never matched by an M5.
static bool     pen_lifted_for_pause = false;
// Set when a job stops because of an error rather than reaching the end, so
// the UI can say so instead of reporting a normal finish.
static bool     job_error = false;
static char     quality[8] = "draft";

// Machine coordinates where the job started, so a stop can drive back.
static float origin_x = 0.0f, origin_y = 0.0f;

// Requests from the HTTP task (other core). Applied in job_service() on the
// core that owns the parser, so the parser is only ever touched from loop().
static volatile bool req_stop = false;
static volatile bool req_pause = false;
static volatile bool req_resume = false;
static volatile bool req_start = false;
// Quality preset to apply: nullptr, or one of the preset name literals.
static const char *volatile req_quality = nullptr;

// Set while a stop is decelerating the machine. The planner is only flushed
// once the motors are at rest: stepper_reset() at speed is an instant stop,
// which skips steps and sends the return-to-origin move somewhere else.
static bool stopping = false;

static void (*on_change)() = nullptr;

void job_set_on_change(void (*cb)()) { on_change = cb; }

// Change state and tell the UI. Every assignment to `state` goes through
// here so a browser is never left showing a stale value.
static void set_state(JobState s) {
    if (state == s) return;
    state = s;
    if (on_change) on_change();
}

static void run_line(const char *s) {
    char buf[LINE_BUFFER_SIZE];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    gcode_execute_line(buf);
}

// Read one newline-terminated line into `buf`, trimmed of surrounding
// whitespace. Returns false at end of file.
//
// Deliberately not String/readStringUntil: that heap-allocates twice per
// line, and a job is tens of thousands of lines. Churning the heap that hard
// while WiFi and the async web server are allocating from it is how you get
// fragmentation failures partway through a long print.
//
// A line longer than `buf` sets `*too_long` and its remainder is discarded:
// read as a line of its own, the tail could be a bare "X10 Y20" — a move.
static bool read_line(File &f, char *buf, size_t cap, bool *too_long = nullptr) {
    if (!f.available()) return false;
    size_t n = f.readBytesUntil('\n', buf, cap - 1);
    buf[n] = '\0';
    bool longer = false;
    if (n == cap - 1) {
        int c;
        while ((c = f.read()) >= 0 && c != '\n')
            if (c != '\r') longer = true;
    }
    if (too_long) *too_long = longer;
    // Trim trailing whitespace (notably the \r of CRLF files).
    while (n && (uint8_t)buf[n - 1] <= ' ') buf[--n] = '\0';
    return true;
}

// The M-code a line starts with, or -1. Parsed as a number so M30 isn't
// taken for M3, nor M00 missed as M0.
static int m_code(const char *p) {
    if (toupper((unsigned char)p[0]) != 'M' || !isdigit((unsigned char)p[1]))
        return -1;
    return (int)strtol(p + 1, nullptr, 10);
}

void job_init() {
    // Do NOT pass formatOnFail=true: formatting the ~3.9MB partition takes
    // tens of seconds with no output, which is indistinguishable from a hung
    // board. A failed mount just means `pio run -t uploadfs` wasn't run yet.
    if (!LittleFS.begin(false)) {
        Serial.println(F("[job] LittleFS mount failed — run: pio run -t uploadfs"));
    }
}

bool job_start() {
    if (state != JOB_IDLE) return false;
    // A failed upload deletes the file, so absent or empty means nothing to
    // draw. begin_job() re-checks on loop()'s core.
    File f = LittleFS.open(JOB_PATH, "r");
    if (!f) return false;
    bool empty = f.size() == 0;
    f.close();
    if (empty) return false;
    req_start = true;
    return true;
}

void job_pause()  { if (state == JOB_RUNNING) req_pause = true; }
void job_resume() { if (state == JOB_PAUSED)  req_resume = true; }
void job_stop()   { if (state != JOB_IDLE)    req_stop = true; }

JobState job_state()       { return state; }
uint8_t  job_progress()    { return progress_pct; }
// Internal only: the UI manages motors automatically and has no lock control,
// so this isn't reported over HTTP. Reads the pin rather than a cached flag
// because motion sent over USB energises the drivers behind job.cpp's back.
static bool motors_energised() { return digitalRead(PIN_DRIVER_EN) == LOW; }
bool     job_had_error()     { return job_error; }
const char *job_quality()    { return quality; }

// Quality presets, derived from tune.py on this machine:
//   $110=19682  $111=13121  $112=13121  $120=29890  $122=19927
//
// $112/$122 are the *motor-space* caps — real hardware limits measured by
// tune.py — so neither preset raises them; exceeding them skips steps. Only
// the Cartesian tip caps ($110/$120) and the path tolerances move.
//
// Note the motor cap binds first on diagonals: at load √2 the effective tip
// speed is $112/1.414 ≈ 9280 mm/min regardless of $110, which is why draft
// doesn't gain much from pushing $110 higher than the tuned value.
//
// What actually buys quality is acceleration and tolerance, not top speed:
// lower $120 means less ringing when the carriage reverses, and a smaller
// $140 makes the planner slow down harder at corners instead of rounding
// them off.
//
// Called from the HTTP task, so it only queues the preset; apply_quality()
// runs it through the parser on loop()'s core. Refused mid-job: changing the
// accel limits under a running plan would reshape motion already queued.
bool job_set_quality(const char *q) {
    if (state != JOB_IDLE) return false;
    if (strcmp(q, "poster") == 0)     req_quality = "poster";
    else if (strcmp(q, "draft") == 0) req_quality = "draft";
    else return false;
    return true;
}

static void apply_quality(const char *q) {
    bool poster = (strcmp(q, "poster") == 0);
    strncpy(quality, q, sizeof(quality) - 1);
    quality[sizeof(quality) - 1] = '\0';

    // Motor caps: the machine's measured ceiling, identical for both presets.
    run_line("$112=13121");
    run_line("$122=19927");

    if (poster) {
        // ~40% speed, ~20% acceleration, tight corners and smooth arcs.
        run_line("$110=8000");  run_line("$111=8000");
        run_line("$120=6000");
        run_line("$140=0.01");  run_line("$141=0.002");
    } else {
        // Full measured speed and acceleration; looser corner rounding is
        // what makes it quick on dense stipple work.
        run_line("$110=19682"); run_line("$111=13121");
        run_line("$120=29890");
        run_line("$140=0.05");  run_line("$141=0.008");
    }
}

static void begin_job() {
    job_file = LittleFS.open(JOB_PATH, "r");
    if (!job_file) return;
    // Stream's default 1s timeout would stall at EOF on a file that doesn't
    // end in a newline; a file never has more data coming.
    job_file.setTimeout(0);
    job_size = job_file.size();
    progress_pct = 0;

    stepper_get_machine_position_mm(&origin_x, &origin_y);
    pen_is_down = false;
    pen_lifted_for_pause = false;
    job_error = false;
    // Energise before the first move so the carriage is held from the start;
    // the idle auto-unlock may well have released it since the last print.
    run_line("M17");
    set_state(JOB_RUNNING);
}

// Abort: stop feeding and decelerate. job_service() finishes the stop with
// end_job(true) once the motors are at rest.
static void begin_stop() {
    if (job_file) job_file.close();
    stopping = true;
    stepper_feed_hold();
}

// Lift the pen and drive back to where the job started, then release the
// motors (via the idle auto-unlock, once the return move has drained).
// Returning to origin flushes the planner, so the machine must be at rest.
static void end_job(bool return_to_origin) {
    if (job_file) job_file.close();

    if (return_to_origin) {
        stepper_reset();
        plan_reset();
        gcode_reset();
        // Re-anchor the planner to the machine's true position, same as
        // protocol.cpp's sync_planner_to_machine() does after a soft reset.
        float sx, sy;
        stepper_get_machine_position_mm(&sx, &sy);
        plan_set_position(sx, sy);

        run_line("M5");   // pen up
        run_line("G90");
        char buf[64];
        snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F%lu",
                 origin_x, origin_y, (unsigned long)RETURN_RATE_MM_MIN);
        run_line(buf);
    }

    // Don't send M18 here: the return-to-origin move above is still draining
    // through the planner. service_idle_unlock() releases once it's actually
    // finished, which is the same tick in practice.
    pen_is_down = false;
    pen_lifted_for_pause = false;
    set_state(JOB_IDLE);
}

// Drop the steppers to zero current as soon as the machine is standing still.
// Holding a stationary stepper is its worst thermal case (rated current, no
// back-EMF, no airflow) and a plotter has no gravity load to hold, so there's
// nothing to gain from staying energised. Applies after a finished print, an
// aborted one, or simply being powered on without printing.
static void service_idle_unlock() {
    // Pin state, not a cached flag: motion sent over USB re-enables the
    // drivers behind job.cpp's back, and that motion should re-arm the
    // auto-unlock exactly like a print does.
    if (!motors_energised()) return;
    // JOB_PAUSED must keep holding: releasing mid-job would lose registration
    // and the resumed print would be offset.
    if (state != JOB_IDLE) return;
    // These two already prove every queued move has finished executing —
    // a stepper that has decelerated to a stop doesn't coast — so there's
    // nothing left to wait for and no reason to keep the windings hot.
    if (!stepper_is_idle() || !plan_is_empty()) return;

    run_line("M18");
    if (on_change) on_change();
}

void job_service() {
    service_idle_unlock();

    // Apply cross-core requests on the parser's own core.
    if (req_quality) {
        const char *q = req_quality;
        req_quality = nullptr;
        if (state == JOB_IDLE) { apply_quality(q); if (on_change) on_change(); }
    }
    if (req_start)  { req_start = false;  begin_job(); }
    if (req_stop) {
        req_stop = false;
        if (state != JOB_IDLE && !stopping) begin_stop();
    }
    if (stopping) {
        // Re-issued every pass: a hold is ignored while the stepper is on a
        // sync block (pen, dwell), and motion resumes after it.
        stepper_feed_hold();
        if (!stepper_is_stopped()) return;
        stopping = false;
        end_job(true);
        return;
    }
    // Pause stops feeding new lines, but the planner still holds up to a full
    // buffer of queued motion. Lifting the pen the instant the button is
    // pressed would let that motion run pen-up and silently drop the geometry
    // it was supposed to draw. So: stop feeding now, and only lift once the
    // machine has actually come to rest. Resume puts the pen back down at
    // that same point, which is where the next line continues from.
    if (req_pause && state == JOB_RUNNING) {
        req_pause = false;
        set_state(JOB_PAUSED);
    }
    if (state == JOB_PAUSED && pen_is_down && !pen_lifted_for_pause &&
        stepper_is_idle() && plan_is_empty()) {
        run_line("M5");
        pen_lifted_for_pause = true;
    }
    if (req_resume && state == JOB_PAUSED) {
        req_resume = false;
        if (pen_lifted_for_pause) {
            run_line("M3");
            pen_lifted_for_pause = false;
        }
        set_state(JOB_RUNNING);
    }

    if (state != JOB_RUNNING || !job_file) return;
    // A feed hold (`!` over USB) stops the planner draining; feeding into it
    // would end in a line that can't get a slot and fails.
    if (stepper_is_in_hold()) return;

    // Keep the planner topped up. Refilling until it's nearly full (rather
    // than one line per loop) is what lets lookahead see a corner coming and
    // carry speed through it; a shallow buffer makes the planner decelerate
    // to a near-stop at the newest block every time, which on dense stipple
    // work is the difference between a smooth plot and a visible stutter.
    // Blank and comment lines are skipped without spending a slot, so a
    // heavily-commented file doesn't drain the buffer either.
    char buf[LINE_BUFFER_SIZE];
    bool too_long = false;
    while (plan_buffer_available() > PLANNER_HEADROOM &&
           read_line(job_file, buf, sizeof(buf), &too_long)) {
        const char *p = buf;
        while (*p == ' ' || *p == '\t') p++;
        // `$` settings don't belong in a job: they would change speed and
        // acceleration under a plan already queued, and persist past the job.
        // Quality presets are the web UI's job.
        if (!*p || *p == ';' || *p == '$') continue;

        // Track pen state so pause/stop know whether to lift.
        int m = m_code(p);
        if (m == 3) pen_is_down = true;
        else if (m == 5) pen_is_down = false;

        // M0/M1 in a job is a job pause, not a feed hold: a hold could only
        // be released by `~` over USB, while this resumes from the web UI.
        if (m == 0 || m == 1) {
            set_state(JOB_PAUSED);
            return;
        }

        StatusCode st = too_long ? STATUS_OVERFLOW : gcode_execute_line(buf);

        // A failed line aborts the job rather than being skipped. Carrying on
        // would silently drop geometry and draw a mangled partial image with
        // no indication why — and the usual cause (a G92 origin set too far
        // across the bed) affects the whole remainder of the file, not one
        // line. Unsupported commands (G17, G64, ...) are the exception: they
        // carry no geometry, and converters emit them freely.
        if (st != STATUS_OK && st != STATUS_UNSUPPORTED_COMMAND) {
            job_error = true;
            begin_stop();
            return;
        }
    }

    // Progress, once per refill rather than per line. Pushed only when the
    // whole percent changes, and at most every PROGRESS_INTERVAL_MS: each
    // push formats JSON and queues a message per connected browser, on the
    // parser's core. A job therefore sends at most ~100 updates in total.
    uint8_t pct = job_size ? (uint8_t)((uint64_t)job_file.position() * 100 / job_size) : 100;
    if (pct != progress_pct) {
        uint32_t now = millis();
        if ((uint32_t)(now - last_progress_ms) >= PROGRESS_INTERVAL_MS) {
            last_progress_ms = now;
            progress_pct = pct;
            if (on_change) on_change();
        }
    }

    if (!job_file.available()) {
        // Wait for the machine to finish draining before declaring done.
        if (stepper_is_idle() && plan_is_empty()) end_job(false);
    }
}
