#include "job.h"

#include <Arduino.h>
#include <LittleFS.h>

#include "config.h"
#include "gcode.h"
#include "planner.h"
#include "settings.h"
#include "stepper.h"
#include "wake.h"

// Path the upload handler writes to. One job at a time.
const char *JOB_PATH = "/job.gcode";

// Feedrate for the return-to-origin move after a stop. Deliberately well
// under the machine's cap: this runs pen-up on an aborted job, and a gentle
// return is less likely to skip than a full-speed rapid.
static const uint32_t RETURN_RATE_MM_MIN = 3000;

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

// Machine coordinates where the job started, so a stop can drive back.
static float origin_x = 0.0f, origin_y = 0.0f;

// Requests from the HTTP task (other core). Each sets its flag and wakes the
// loop, which applies it in job_service() on the core that owns the parser.
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

// Read-ahead over job_file. Stream::readBytesUntil() goes byte by byte, and
// each byte is a File::read() → locked fread() plus a millis() timeout check;
// one block read per 512 bytes is a fraction of that on the parser's core.
static uint8_t  rd_buf[512];
static uint16_t rd_pos = 0, rd_len = 0;

static int job_getc() {
    if (rd_pos == rd_len) {
        rd_len = (uint16_t)job_file.read(rd_buf, sizeof(rd_buf));
        rd_pos = 0;
        if (rd_len == 0) return -1;
    }
    return rd_buf[rd_pos++];
}

static bool job_eof() { return rd_pos == rd_len && !job_file.available(); }

// Bytes handed to the parser so far (the file position less what's still
// buffered), for the progress bar.
static size_t job_consumed() { return job_file.position() - (rd_len - rd_pos); }

// Read one newline-terminated line into `buf`, trimmed of surrounding
// whitespace. Returns false at end of file.
//
// Deliberately not String/readStringUntil: that heap-allocates twice per
// line, and a job is tens of thousands of lines. Churning the heap that hard
// while WiFi and the async web server are allocating from it is how you get
// fragmentation failures partway through a long print.
//
// A line longer than `buf` has its remainder discarded — read as a line of
// its own, the tail could be a bare "X10 Y20", a move. `*too_long` is set
// if that remainder held anything that matters: not whitespace, and not the
// rest of a `;` comment (converters write long ones).
static bool read_line(char *buf, size_t cap, bool *too_long) {
    int c = job_getc();
    if (c < 0) return false;
    size_t n = 0;
    bool longer = false;
    for (; c >= 0 && c != '\n'; c = job_getc()) {
        if (n < cap - 1) buf[n++] = (char)c;
        else if (c > ' ') longer = true;
    }
    if (longer && memchr(buf, ';', n)) longer = false;
    buf[n] = '\0';
    *too_long = longer;
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
    // Do NOT pass formatOnFail=true: formatting the ~6.7MB partition takes
    // tens of seconds, which is indistinguishable from a hung board. A failed
    // mount just means `pio run -t uploadfs` wasn't run yet (the web UI then
    // 404s).
    LittleFS.begin(false);
}

bool job_start() {
    if (state != JOB_IDLE || req_start) return false;
    // A failed upload deletes the file, so absent or empty means nothing to
    // draw. begin_job() re-checks on loop()'s core.
    File f = LittleFS.open(JOB_PATH, "r");
    if (!f) return false;
    bool empty = f.size() == 0;
    f.close();
    if (empty) return false;
    req_start = true;
    wake_loop();
    return true;
}

void job_pause()  { if (state == JOB_RUNNING) { req_pause = true;  wake_loop(); } }
void job_resume() { if (state == JOB_PAUSED)  { req_resume = true; wake_loop(); } }
void job_stop()   { if (state != JOB_IDLE)    { req_stop = true;   wake_loop(); } }

JobState job_state()       { return state; }
uint8_t  job_progress()    { return progress_pct; }
bool     job_had_error()     { return job_error; }
const char *job_quality()    { return quality_poster ? "poster" : "draft"; }

// Quality presets, from calibrating this machine (docs/CALIBRATION.md):
//   $110=19682  $111=13121  $112=13121  $120=29890  $122=19927
//
// $112/$122 are the *motor-space* caps — real hardware limits measured by
// the calibrator — so neither preset raises them; exceeding them skips
// steps. Only the Cartesian tip caps ($110/$120) and the corner tolerance
// move.
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
// runs on loop()'s core. Refused mid-job: changing the accel limits under a
// running plan would reshape motion already queued.
bool job_set_quality(const char *q) {
    if (state != JOB_IDLE) return false;
    if (strcmp(q, "poster") == 0)     req_quality = "poster";
    else if (strcmp(q, "draft") == 0) req_quality = "draft";
    else return false;
    wake_loop();
    return true;
}

static void apply_quality(const char *q) {
    bool poster = (strcmp(q, "poster") == 0);
    quality_poster = poster;

    // Motor caps: the machine's measured ceiling, identical for both presets.
    settings_set(112, 13121);
    settings_set(122, 19927);

    if (poster) {
        // ~40% speed, ~20% acceleration, tight corners.
        settings_set(110, 8000);  settings_set(111, 8000);
        settings_set(120, 6000);
        settings_set(140, 0.01f);
    } else {
        // Full measured speed and acceleration; looser corner rounding is
        // what makes it quick on dense stipple work.
        settings_set(110, 19682); settings_set(111, 13121);
        settings_set(120, 29890);
        settings_set(140, 0.05f);
    }
    settings_save();  // no-op when re-sent unchanged
}

static void begin_job() {
    job_file = LittleFS.open(JOB_PATH, "r");
    if (!job_file) return;
    rd_pos = rd_len = 0;
    job_size = job_file.size();
    progress_pct = 0;

    stepper_get_machine_position_mm(&origin_x, &origin_y);
    pen_is_down = false;
    pen_lifted_for_pause = false;
    job_error = false;
    // Energise before the first move so the carriage is held from the start;
    // the idle release may well have dropped it since the last print.
    stepper_set_drivers(true);
    set_state(JOB_RUNNING);
}

// Abort: stop feeding and decelerate. job_service() finishes the stop with
// end_job(true) once the motors are at rest, which the step ISR wakes it for.
static void begin_stop() {
    if (job_file) job_file.close();
    stopping = true;
    stepper_feed_hold();
}

// Lift the pen and drive back to where the job started; the motors are
// released once that has drained (see job_service()).
// Returning to origin flushes the planner, so the machine must be at rest.
static void end_job(bool return_to_origin) {
    if (job_file) job_file.close();

    if (return_to_origin) {
        stepper_reset();
        plan_reset();
        gcode_reset();
        // Re-anchor the planner to the machine's true position: the motors
        // are physically where they stopped, not at a phantom 0,0.
        float sx, sy;
        stepper_get_machine_position_mm(&sx, &sy);
        plan_set_position(sx, sy);

        run_line("M5");   // pen up
        char buf[64];
        snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F%lu",
                 origin_x, origin_y, (unsigned long)RETURN_RATE_MM_MIN);
        run_line(buf);
    }

    pen_is_down = false;
    pen_lifted_for_pause = false;
    set_state(JOB_IDLE);
}

// Keep the planner topped up. Refilling until it's nearly full (rather than
// one line per wake) is what lets lookahead see a corner coming and carry
// speed through it; a shallow buffer makes the planner decelerate to a
// near-stop at the newest block every time, which on dense stipple work is
// the difference between a smooth plot and a visible stutter. Blank and
// comment lines are skipped without spending a slot, so a heavily-commented
// file doesn't drain the buffer either.
static void feed() {
    char buf[LINE_BUFFER_SIZE];
    bool too_long = false;
    while (plan_buffer_available() >= MAX_BLOCKS_PER_LINE &&
           read_line(buf, sizeof(buf), &too_long)) {
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

        // M0/M1 in a job is a job pause, resumed from the web UI.
        if (m == 0 || m == 1) {
            set_state(JOB_PAUSED);
            return;
        }

        // A failed line aborts the job rather than being skipped. Carrying on
        // would silently drop geometry and draw a mangled partial image with
        // no indication why — and the usual cause (a G92 origin set too far
        // across the bed) affects the whole remainder of the file, not one
        // line.
        if (too_long || gcode_execute_line(buf) == STATUS_ERROR) {
            job_error = true;
            begin_stop();
            return;
        }
    }

    // Progress, once per refill rather than per line, and pushed only when
    // the whole percent changes: a job sends at most ~100 updates in total.
    uint8_t pct = job_size ? (uint8_t)((uint64_t)job_consumed() * 100 / job_size) : 100;
    if (pct != progress_pct) {
        progress_pct = pct;
        if (on_change) on_change();
    }
}

void job_service() {
    // Apply cross-core requests on the parser's own core.
    if (req_quality) {
        const char *q = req_quality;
        req_quality = nullptr;
        if (state == JOB_IDLE) { apply_quality(q); if (on_change) on_change(); }
    }
    if (req_start) { req_start = false; if (state == JOB_IDLE) begin_job(); }
    if (req_stop) {
        req_stop = false;
        if (state != JOB_IDLE && !stopping) begin_stop();
    }
    // Pause stops feeding new lines, but the planner still holds up to a full
    // buffer of queued motion. Lifting the pen the instant the button is
    // pressed would let that motion run pen-up and silently drop the geometry
    // it was supposed to draw. So: stop feeding now, and only lift once the
    // machine has actually come to rest. Resume puts the pen back down at
    // that same point, which is where the next line continues from.
    if (req_pause) {
        req_pause = false;
        if (state == JOB_RUNNING && !stopping) set_state(JOB_PAUSED);
    }
    if (req_resume) {
        req_resume = false;
        if (state == JOB_PAUSED && !stopping) {
            if (pen_lifted_for_pause) {
                run_line("M3");
                pen_lifted_for_pause = false;
            }
            set_state(JOB_RUNNING);
        }
    }

    if (state == JOB_RUNNING && job_file) feed();  // may begin a stop itself

    // After feed(), so a stop it began on an error completes in this same
    // pass when the machine is already at rest — nothing else would wake us.
    if (stopping) {
        if (!stepper_is_stopped()) return;  // the ISR wakes us when it is
        stopping = false;
        end_job(true);
    }

    // Everything below needs the machine at rest with nothing queued.
    if (!stepper_is_idle() || !plan_is_empty()) return;

    if (state == JOB_RUNNING && job_file && job_eof()) end_job(false);

    if (state == JOB_PAUSED && pen_is_down && !pen_lifted_for_pause) {
        run_line("M5");
        pen_lifted_for_pause = true;
        return;  // the pen block wakes us again when it's done
    }

    if (state == JOB_RUNNING) return;
    // Drop the steppers to zero current as soon as the machine is standing
    // still with no job. Holding a stationary stepper is its worst thermal
    // case (rated current, no back-EMF, no airflow) and a plotter has no
    // gravity load to hold. A paused job keeps holding: releasing mid-job
    // would lose registration and the resumed print would be offset.
    if (state == JOB_IDLE && stepper_drivers_on()) stepper_set_drivers(false);
    stepper_power_save();
}
