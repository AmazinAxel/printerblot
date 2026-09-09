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

static File     job_file;
static JobState state = JOB_IDLE;
static uint32_t lines_done = 0;
static uint32_t lines_total = 0;
static bool     pen_is_down = false;
// Set when a pause has actually lifted the pen (which happens once the
// planner drains, not when the button is pressed). Keeps resume from issuing
// an M3 that was never matched by an M5.
static bool     pen_lifted_for_pause = false;
static char     quality[8] = "draft";

// Machine coordinates where the job started, so a stop can drive back.
static float origin_x = 0.0f, origin_y = 0.0f;

// Requests from the HTTP task (other core). Applied in job_service() on the
// core that owns the parser, so the parser is only ever touched from loop().
static volatile bool req_stop = false;
static volatile bool req_pause = false;
static volatile bool req_resume = false;
static volatile bool req_start = false;

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
static bool read_line(File &f, char *buf, size_t cap) {
    if (!f.available()) return false;
    size_t n = f.readBytesUntil('\n', buf, cap - 1);
    buf[n] = '\0';
    // Trim trailing whitespace (notably the \r of CRLF files).
    while (n && (uint8_t)buf[n - 1] <= ' ') buf[--n] = '\0';
    return true;
}

// Count non-blank, non-comment lines so the UI can show progress.
static uint32_t count_lines(File &f) {
    uint32_t n = 0;
    char buf[LINE_BUFFER_SIZE];
    f.seek(0);
    while (read_line(f, buf, sizeof(buf))) {
        const char *p = buf;
        while (*p == ' ' || *p == '\t') p++;
        if (*p && *p != ';') n++;
    }
    f.seek(0);
    return n;
}

void job_init() {
    // Do NOT pass formatOnFail=true: formatting the ~3.9MB partition takes
    // tens of seconds with no output, which is indistinguishable from a hung
    // board. A failed mount just means `pio run -t uploadfs` wasn't run yet.
    if (!LittleFS.begin(false)) {
        Serial.println(F("[job] LittleFS mount failed — run: pio run -t uploadfs"));
        return;
    }
    // Count any job left over from before the power cycle. The whole point of
    // storing it in flash is reprinting without re-uploading, and job_start()
    // refuses a job whose line count is zero.
    job_notify_new_file();
}

void job_notify_new_file() {
    if (state != JOB_IDLE) return;
    if (job_file) job_file.close();
    lines_total = 0;   // reset first: a failed upload deletes the file, and a
                       // stale count would make the UI offer a job that isn't
                       // there any more.
    File f = LittleFS.open(JOB_PATH, "r");
    if (f) {
        lines_total = count_lines(f);
        f.close();
    }
    lines_done = 0;
}

bool job_start() {
    if (state != JOB_IDLE) return false;
    if (!LittleFS.exists(JOB_PATH)) return false;
    if (lines_total == 0) return false;  // nothing to draw (empty or failed upload)
    req_start = true;
    return true;
}

void job_pause()  { if (state == JOB_RUNNING) req_pause = true; }
void job_resume() { if (state == JOB_PAUSED)  req_resume = true; }
void job_stop()   { if (state != JOB_IDLE)    req_stop = true; }

JobState job_state()       { return state; }
uint32_t job_lines_done()  { return lines_done; }
uint32_t job_lines_total() { return lines_total; }
// Read the enable pin rather than the cached flag: a jog or a G-code move
// sent over USB re-energises the drivers without going through job.cpp, and
// the UI should show what the machine is actually doing.
bool     job_motors_locked() { return digitalRead(PIN_DRIVER_EN) == LOW; }
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
void job_set_quality(const char *q) {
    bool poster = (strcmp(q, "poster") == 0);
    if (!poster && strcmp(q, "draft") != 0) return;
    strncpy(quality, poster ? "poster" : "draft", sizeof(quality) - 1);
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

    stepper_get_machine_position_mm(&origin_x, &origin_y);
    pen_is_down = false;
    pen_lifted_for_pause = false;
    lines_done = 0;
    // Energise before the first move so the carriage is held from the start;
    // the idle auto-unlock may well have released it since the last print.
    run_line("M17");
    set_state(JOB_RUNNING);
}

// Lift the pen and drive back to where the job started, then release the
// motors (via the idle auto-unlock, once the return move has drained).
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
    if (!job_motors_locked()) return;
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
    if (req_start)  { req_start = false;  begin_job(); }
    if (req_stop)   { req_stop = false;   if (state != JOB_IDLE) end_job(true); return; }
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

    // Keep the planner topped up. Refilling until it's nearly full (rather
    // than one line per loop) is what lets lookahead see a corner coming and
    // carry speed through it; a shallow buffer makes the planner decelerate
    // to a near-stop at the newest block every time, which on dense stipple
    // work is the difference between a smooth plot and a visible stutter.
    // Blank and comment lines are skipped without spending a slot, so a
    // heavily-commented file doesn't drain the buffer either.
    char buf[LINE_BUFFER_SIZE];
    while (plan_buffer_available() > PLANNER_HEADROOM &&
           read_line(job_file, buf, sizeof(buf))) {
        const char *p = buf;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == ';') continue;

        // Track pen state so pause/stop know whether to lift.
        if (p[0] == 'M' && p[1] == '3') pen_is_down = true;
        else if (p[0] == 'M' && p[1] == '5') pen_is_down = false;

        gcode_execute_line(buf);
        lines_done++;

        // Push progress occasionally. Every line would flood the SSE stream
        // and steal time from the parser; every 64 is smooth enough to watch.
        if ((lines_done & 63) == 0 && on_change) on_change();
    }

    if (!job_file.available()) {
        // Wait for the machine to finish draining before declaring done.
        if (stepper_is_idle() && plan_is_empty()) end_job(false);
    }
}
