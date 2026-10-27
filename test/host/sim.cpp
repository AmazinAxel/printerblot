// Host simulation of the firmware: the real planner, stepper ISR, servo ISR,
// G-code parser, job runner and settings, against modelled hardware.
//
// Time is simulated in ns. Timers fire their (real) ISRs at their alarm
// times; the step-pulse spin advances time through the cycle counter; file
// reads advance time and let due interrupts preempt the loop mid-feed. Every
// pin edge is checked as it happens:
//   - STEP pulses ≥ 2µs, ≥ 25µs apart, never with the drivers off, never
//     below 240MHz, DIR never changed under a high STEP;
//   - no step while the pen is travelling (60ms after a new servo width);
// and each job is checked end to end against a reference reading of the
// G-code: the motors must be exactly at each move's rounded target whenever
// the pen changes, and at the last one when the job ends.
//
// Usage: sim            run every test (each in its own process)
//        sim <name>     run one

#include <Arduino.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include "driver/timer.h"

#include "config.h"
#include "gcode.h"
#include "job.h"
#include "planner.h"
#include "settings.h"
#include "stepper.h"
#include "wake.h"

#include <stdarg.h>
#include <functional>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

// ───────── failure reporting ─────────

static uint64_t now_ns = 0;
static int failures = 0;

#define CHECK(cond, ...) do { if (!(cond)) { \
    fprintf(stderr, "  FAIL @%.3fms %s:%d: ", now_ns / 1e6, __FILE__, __LINE__); \
    fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); \
    if (++failures > 20) exit(1); } } while (0)

// ───────── clock / CPU ─────────

static uint32_t cpu_mhz = 240;
static uint64_t cycles = 0;
static int clock_switches = 0;

void sim_set_cpu_mhz(uint32_t mhz) {
    if (mhz != cpu_mhz) clock_switches++;
    cpu_mhz = mhz;
}

uint32_t sim_ccount() {
    static uint32_t frac = 0;  // exact: 1000ns per cpu_mhz cycles
    cycles++;
    frac += 1000;
    now_ns += frac / cpu_mhz;
    frac %= cpu_mhz;
    return (uint32_t)cycles;
}

// ───────── wake (FreeRTOS task notification) ─────────

static int wake_pending = 0;
static int wakes_total = 0;
void wake_init() {}
void wake_wait() {}
void wake_loop() { wake_pending++; wakes_total++; }
bool wake_loop_from_isr() { wake_pending++; wakes_total++; return true; }

// ───────── timers ─────────

struct SimTimer { timer_isr_t isr = nullptr; bool enabled = false; uint64_t alarm_us = 0; };
static SimTimer timers[2];
static bool in_isr = false;

static uint64_t now_us() { return now_ns / 1000; }
int timer_init(timer_group_t, timer_idx_t, const timer_config_t *) { return 0; }
int timer_set_counter_value(timer_group_t, timer_idx_t, uint64_t) { return 0; }
int timer_isr_callback_add(timer_group_t g, timer_idx_t, timer_isr_t f, void *, int) {
    timers[g].isr = f; return 0;
}
int timer_start(timer_group_t, timer_idx_t) { return 0; }
int timer_set_alarm_value(timer_group_t g, timer_idx_t, uint64_t v) { timers[g].alarm_us = v; return 0; }
int timer_set_alarm(timer_group_t g, timer_idx_t, int en) { timers[g].enabled = en; return 0; }
int timer_get_counter_value(timer_group_t, timer_idx_t, uint64_t *v) { *v = now_us(); return 0; }
uint64_t timer_group_get_counter_value_in_isr(timer_group_t, timer_idx_t) { return now_us(); }
void timer_group_set_alarm_value_in_isr(timer_group_t g, timer_idx_t, uint64_t v) { timers[g].alarm_us = v; }
void timer_group_enable_alarm_in_isr(timer_group_t g, timer_idx_t) { timers[g].enabled = true; }

// Earliest pending alarm (ns), or UINT64_MAX.
static uint64_t next_alarm(int *which) {
    uint64_t best = UINT64_MAX;
    for (int g = 0; g < 2; g++) {
        if (!timers[g].enabled) continue;
        uint64_t t = timers[g].alarm_us * 1000;
        if (t < now_ns) t = now_ns;  // already due: fires at once
        if (t < best) { best = t; *which = g; }
    }
    return best;
}

static void fire(int g) {
    CHECK(!in_isr, "nested interrupt");
    timers[g].enabled = false;  // the hardware clears the alarm enable
    in_isr = true;
    timers[g].isr(nullptr);
    in_isr = false;
}

// Fire every interrupt due at or before `until` (ns), in time order.
static void run_isrs_until(uint64_t until) {
    for (;;) {
        int g = -1;
        uint64_t t = next_alarm(&g);
        if (t > until) break;
        if (t > now_ns) now_ns = t;
        fire(g);
    }
    if (until > now_ns) now_ns = until;
}

// ───────── pins and their checks ─────────

static const int P_STEP1 = 9, P_DIR1 = 8, P_STEP2 = 7, P_DIR2 = 44, P_EN = 2, P_SERVO = 43;
static bool pins[64];
static uint64_t step_rise_ns[2];
static uint64_t last_event_ns = 0;
static uint64_t last_interval_ns = UINT64_MAX;
static int events_since_start = 0;
static bool any_step = false;
static int64_t motor[2];
static uint64_t steps_total = 0;
struct StepRec { uint64_t ns; int motor; bool dir; };
static std::vector<StepRec> trace;  // every step, for tests that look closer

static uint64_t servo_rise_ns = 0;
static uint32_t servo_width = 0;        // last completed pulse width (µs)
static uint64_t pen_change_ns = 0;      // first pulse at a new width
static bool pen_travelling = false;
struct PenEvent { uint32_t width; int64_t m1, m2; };
static std::vector<PenEvent> pen_events;

void sim_pin_write(uint8_t pin, bool level) {
    if (pins[pin] == level) return;
    pins[pin] = level;

    int m = pin == P_STEP1 ? 0 : pin == P_STEP2 ? 1 : -1;
    if (m >= 0 && level) {
        CHECK(!pins[P_EN], "step with drivers disabled");
        CHECK(cpu_mhz == 240, "step at %uMHz", cpu_mhz);
        // Both motors of one step event rise within the same ISR.
        if (!any_step || now_ns - last_event_ns > 1000) {
            uint64_t gap = now_ns - last_event_ns;
            if (any_step) {
                CHECK(gap >= 25000 - 1000, "step events %.1fus apart", gap / 1e3);
                // Every stop and start is a ramp: the step before a pause in
                // motion, and the one after it, are slow ones (a stop from
                // cruise would be ~40µs apart).
                if (gap > 5000000) {
                    CHECK(last_interval_ns >= 150000, "stopped from %.0f events/s",
                          1e9 / last_interval_ns);
                    events_since_start = 0;
                } else {
                    if (events_since_start == 1)
                        CHECK(gap >= 150000, "started at %.0f events/s", 1e9 / gap);
                    last_interval_ns = gap;
                }
            }
            events_since_start++;
            last_event_ns = now_ns;
        }
        any_step = true;
        if (pen_travelling)
            CHECK(now_ns - pen_change_ns >= 59000000ull,
                  "step %.1fms after a pen change", (now_ns - pen_change_ns) / 1e6);
        pen_travelling = false;
        step_rise_ns[m] = now_ns;
        motor[m] += pins[m == 0 ? P_DIR1 : P_DIR2] ? 1 : -1;
        trace.push_back({now_ns, m, pins[m == 0 ? P_DIR1 : P_DIR2]});
        steps_total++;
    } else if (m >= 0) {
        CHECK(now_ns - step_rise_ns[m] >= 2000, "step pulse %.2fus",
              (now_ns - step_rise_ns[m]) / 1e3);
    } else if (pin == P_DIR1 || pin == P_DIR2) {
        CHECK(!pins[pin == P_DIR1 ? P_STEP1 : P_STEP2], "DIR changed under high STEP");
    } else if (pin == P_SERVO) {
        if (level) {
            servo_rise_ns = now_ns;
        } else {
            uint32_t w = (uint32_t)((now_ns - servo_rise_ns + 500) / 1000);
            // The step ISR can delay a servo edge by its own few µs (same
            // core, same priority) — jitter far inside a servo's deadband.
            int jitter = (int)w - (int)servo_width;
            if (servo_width && abs(jitter) <= 5) {
                ;
            } else if (w != servo_width) {
                if (servo_width) {
                    pen_events.push_back({w, motor[0], motor[1]});
                    pen_change_ns = servo_rise_ns;
                    pen_travelling = true;
                }
                servo_width = w;
            }
        }
    }
}

bool sim_pin_read(uint8_t pin) { return pins[pin]; }

void sim_gpio_mask(uint32_t mask, int bank, bool level) {
    for (int b = 0; b < 32; b++)
        if (mask & (1u << b)) sim_pin_write(bank * 32 + b, level);
}

// ───────── job file ─────────

static std::string job_path;
const char *sim_job_path() { return job_path.c_str(); }

// Parsing costs time; meanwhile interrupts preempt the loop.
static uint64_t parse_ns_per_byte = 100;
static bool chaos = false;  // random parse cost per read
void sim_on_file_read(size_t bytes) {
    if (in_isr) return;
    uint64_t per = chaos ? (uint64_t)(rand() % 3 == 0 ? rand() % 20000 : rand() % 300)
                         : parse_ns_per_byte;
    run_isrs_until(now_ns + bytes * per);
}

static void write_job(const std::string &gcode) {
    char tmpl[] = "/tmp/blotsimXXXXXX";
    int fd = mkstemp(tmpl);
    CHECK(fd >= 0, "mkstemp");
    CHECK(write(fd, gcode.data(), gcode.size()) == (ssize_t)gcode.size(), "write");
    close(fd);
    job_path = tmpl;
}

// ───────── the loop ─────────

static int ui_updates = 0;
struct Ext { uint64_t at_ns; std::function<void()> fn; };
static std::vector<Ext> externals;

static void boot() {
    wake_init();
    settings_load();
    plan_reset();
    stepper_init();
    gcode_reset();
    job_init();
    job_set_on_change([] { ui_updates++; });
}

// loop() = wake_wait(); stepper_service(); job_service(); — run whenever
// woken, otherwise advance time to the next interrupt or web request. Returns
// when nothing at all is pending: the machine is quiescent.
static void run(uint64_t limit_ns = 600ull * 1000000000ull) {
    uint64_t deadline = now_ns + limit_ns;  // sim time: a hang fails, not spins
    for (;;) {
        if (wake_pending) {
            wake_pending = 0;
            stepper_service();
            job_service();
            continue;
        }
        int g = -1;
        uint64_t t = next_alarm(&g);
        size_t ext = SIZE_MAX;
        for (size_t i = 0; i < externals.size(); i++)
            if (externals[i].at_ns <= t && (ext == SIZE_MAX || externals[i].at_ns < externals[ext].at_ns))
                ext = i;
        if (ext != SIZE_MAX && externals[ext].at_ns < deadline) {
            if (externals[ext].at_ns > now_ns) now_ns = externals[ext].at_ns;
            auto fn = externals[ext].fn;
            externals.erase(externals.begin() + ext);
            fn();
            continue;
        }
        if (t == UINT64_MAX) return;
        CHECK(t < deadline, "still busy at the time limit");
        if (t >= deadline) return;
        if (t > now_ns) now_ns = t;
        fire(g);
    }
}

static void at_ms(double ms, std::function<void()> fn) {
    externals.push_back({(uint64_t)(ms * 1e6), fn});
}

// ───────── reference model ─────────

static void to_motor_steps(float x, float y, int64_t *m1, int64_t *m2) {
    float a, b;
    // kinematics.h: a = x - y, b = -y - x
    a = x + -y;
    b = -y - x;
    *m1 = lroundf(a * STEPS_PER_MM);
    *m2 = lroundf(b * STEPS_PER_MM);
}

// Expected pen changes (width + motor position) and final position, for a
// supported-subset file run from machine (x0, y0).
struct Expect { std::vector<PenEvent> pens; int64_t m1, m2; };

static Expect reference(const std::string &gcode, float x0, float y0) {
    Expect e{};
    float x = x0, y = y0, ox = 0, oy = 0;
    uint32_t width = (uint32_t)servo_pen_up_us;
    size_t pos = 0;
    while (pos < gcode.size()) {
        size_t nl = gcode.find('\n', pos);
        std::string ln = gcode.substr(pos, nl == std::string::npos ? std::string::npos : nl - pos);
        pos = nl == std::string::npos ? gcode.size() : nl + 1;
        size_t c = ln.find(';');
        if (c != std::string::npos) ln.resize(c);
        bool g92 = ln.find("G92") != std::string::npos;
        int mcode = -1;
        if (ln.find("M3") != std::string::npos && ln.find("M30") == std::string::npos) mcode = 3;
        if (ln.find("M5") != std::string::npos) mcode = 5;
        float vx = NAN, vy = NAN;
        size_t px = ln.find('X'), py = ln.find('Y');
        if (px != std::string::npos) vx = strtof(ln.c_str() + px + 1, nullptr);
        if (py != std::string::npos) vy = strtof(ln.c_str() + py + 1, nullptr);
        if (mcode >= 0) {
            uint32_t w = mcode == 3 ? servo_pen_down_us : servo_pen_up_us;
            if (w != width) {
                int64_t m1, m2;
                to_motor_steps(x, y, &m1, &m2);
                e.pens.push_back({w, m1, m2});
                width = w;
            }
        }
        if (g92) {
            if (!std::isnan(vx)) ox = x - vx;
            if (!std::isnan(vy)) oy = y - vy;
        } else {
            if (!std::isnan(vx)) x = vx + ox;
            if (!std::isnan(vy)) y = vy + oy;
        }
    }
    to_motor_steps(x, y, &e.m1, &e.m2);
    return e;
}

static void check_pens(const std::vector<PenEvent> &want) {
    CHECK(pen_events.size() == want.size(), "%zu pen changes, want %zu",
          pen_events.size(), want.size());
    for (size_t i = 0; i < want.size() && i < pen_events.size(); i++) {
        CHECK(pen_events[i].width == want[i].width &&
              pen_events[i].m1 == want[i].m1 && pen_events[i].m2 == want[i].m2,
              "pen change %zu: %uus at (%lld,%lld), want %uus at (%lld,%lld)", i,
              pen_events[i].width, (long long)pen_events[i].m1, (long long)pen_events[i].m2,
              want[i].width, (long long)want[i].m1, (long long)want[i].m2);
    }
}

// Everything at rest and released, as after any job.
static void check_quiescent(JobState want_state = JOB_IDLE) {
    CHECK(job_state() == want_state, "job state %d, want %d", job_state(), want_state);
    CHECK(stepper_is_idle() && plan_is_empty(), "motion still queued");
    if (want_state == JOB_IDLE)
        CHECK(pins[P_EN], "drivers still energised");
    CHECK(cpu_mhz == 80, "CPU at %uMHz while idle", cpu_mhz);
    CHECK(!timers[0].enabled && !timers[1].enabled, "a timer is still running");
    CHECK(!pins[P_STEP1] && !pins[P_STEP2] && !pins[P_SERVO], "a pin left high");
}

static void run_job(const std::string &gcode) {
    write_job(gcode);
    CHECK(job_start(), "job_start refused");
    run();
}

// ───────── G-code generators ─────────

static std::string fmt(const char *f, ...) {
    char b[160];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    return b;
}

// Like convert.js: header, then pen-up travel + pen-down polylines.
static std::string random_drawing(unsigned seed, int paths, float span = 120.0f) {
    srand(seed);
    auto r = [] { return rand() / (float)RAND_MAX; };
    std::string g = "G21\nG90\nG92 X0 Y0\nM5\n";
    for (int p = 0; p < paths; p++) {
        float x = 2 + r() * (span - 4), y = 2 + r() * (span - 4), a = r() * 6.283f;
        g += fmt("G1 X%.3f Y%.3f F16000\nM3\n", x, y);
        int n = 1 + rand() % 40;
        for (int i = 0; i < n; i++) {
            a += (r() - 0.5f) * (r() < 0.2f ? 3.0f : 0.5f);
            float len = r() < 0.6f ? 0.05f + r() * 0.6f : r() * 6;
            float nx = x + cosf(a) * len, ny = y + sinf(a) * len;
            if (nx < 1 || nx > span - 1 || ny < 1 || ny > span - 1) { a += 3.1416f; continue; }
            x = nx; y = ny;
            g += fmt("G1 X%.3f Y%.3f F4000\n", x, y);
        }
        g += "M5\n";
    }
    g += "G1 X0 Y0 F16000\nM18\n";
    return g;
}

// ───────── tests ─────────

static void test_square() {
    boot();
    std::string g = "G21\nG90\nG92 X0 Y0\nM5\nG1 X10 Y10 F16000\nM3\n"
                    "G1 X60 Y10 F4000\nG1 X60 Y60\nG1 X10 Y60\nG1 X10 Y10\nM5\n"
                    "G1 X0 Y0 F16000\nM18\n";
    run_job(g);
    Expect e = reference(g, 0, 0);
    check_pens(e.pens);
    CHECK(motor[0] == e.m1 && motor[1] == e.m2, "ended at (%lld,%lld)",
          (long long)motor[0], (long long)motor[1]);
    CHECK(!job_had_error(), "error flagged");
    CHECK(job_progress() == 100, "progress %u", job_progress());
    check_quiescent();
}

static void test_random_drawings() {
    boot();
    for (unsigned seed = 1; seed <= 6; seed++) {
        pen_events.clear();
        int64_t s1 = motor[0], s2 = motor[1];
        float x0, y0;
        stepper_get_machine_position_mm(&x0, &y0);
        std::string g = random_drawing(seed, 60);
        run_job(g);
        Expect e = reference(g, x0, y0);
        check_pens(e.pens);
        CHECK(motor[0] == e.m1 && motor[1] == e.m2,
              "seed %u ended at (%lld,%lld) want (%lld,%lld), from (%lld,%lld)", seed,
              (long long)motor[0], (long long)motor[1], (long long)e.m1, (long long)e.m2,
              (long long)s1, (long long)s2);
        CHECK(!job_had_error(), "seed %u: error flagged", seed);
        check_quiescent();
    }
}

// Very slow parsing: the planner drains between lines, so every refill comes
// from a step-ISR wake with the machine nearly or fully stopped.
static void test_starved_feed() {
    parse_ns_per_byte = 200000;
    boot();
    std::string g = random_drawing(7, 15);
    run_job(g);
    check_pens(reference(g, 0, 0).pens);
    check_quiescent();
}

static void test_dense_stipple() {
    boot();
    std::string g = "G92 X0 Y0\n";
    srand(3);
    for (int i = 0; i < 400; i++) {
        float x = 5 + rand() % 110, y = 5 + rand() % 110;
        g += fmt("G1 X%.2f Y%.2f F16000\nM3\nG1 X%.2f Y%.2f F4000\nM5\n", x, y, x + 0.1f, y);
    }
    run_job(g);
    Expect e = reference(g, 0, 0);
    check_pens(e.pens);
    CHECK(motor[0] == e.m1 && motor[1] == e.m2, "end position");
    check_quiescent();
}

// Lines that queue several blocks at once (pen commands plus a move), with
// the planner kept full: each needs MAX_BLOCKS_PER_LINE free slots.
static void test_multi_block_lines() {
    boot();
    std::string g = "G92 X0 Y0\n";
    srand(4);
    for (int i = 0; i < 300; i++) {
        float x = 5 + rand() % 110, y = 5 + rand() % 110;
        g += fmt(i % 2 ? "M5 M3 G1 X%.2f Y%.2f F6000\n" : "M3 G1 X%.2f Y%.2f F6000\n", x, y);
        g += fmt("G1 X%.2f Y%.2f\n", x + 1, y);
    }
    g += "M5\n";
    run_job(g);
    CHECK(!job_had_error(), "multi-block line failed");
    Expect e = reference(g, 0, 0);
    CHECK(motor[0] == e.m1 && motor[1] == e.m2, "end position");
    CHECK(servo_width == (uint32_t)servo_pen_up_us, "pen not up");
    check_quiescent();
}

static void test_out_of_bounds_aborts() {
    boot();
    std::string g = "G92 X0 Y0\nM5\nG1 X20 Y20 F8000\nM3\nG1 X40 Y20\nG1 X200 Y20\nG1 X40 Y40\nM5\n";
    run_job(g);
    CHECK(job_had_error(), "no error flagged");
    // Returned to the job's origin with the pen up.
    CHECK(motor[0] == 0 && motor[1] == 0, "not back at origin: (%lld,%lld)",
          (long long)motor[0], (long long)motor[1]);
    CHECK(servo_width == (uint32_t)servo_pen_up_us, "pen not up");
    check_quiescent();
}

static void test_unsupported() {
    boot();
    // Harmless: skipped, the job completes.
    std::string ok = "G17\nG21 G90 G94\nG92 X0 Y0\nG4 P100\nM5\nG1 X5 Y5 F3000\nM3\n"
                     "G1 X15 Y5 (inline comment)\nM5 ; trailing\nM18\nM84\nM30\n";
    run_job(ok);
    CHECK(!job_had_error(), "harmless codes aborted the job");
    CHECK(motor[0] == reference(ok, 0, 0).m1 && motor[1] == reference(ok, 0, 0).m2, "end position");
    check_quiescent();
    // Geometry-changing: each aborts.
    const char *bad[] = {"G2 X10 Y10 I5 J0", "G3 X10 Y10 I5 J0", "G91", "G20", "G28",
                         "G92.1", "G53 G1 X5", "X5 Q", "G1 X--5", "M3 M5 M3", "G1 F-5 X1"};
    for (const char *b : bad) {
        std::string g = std::string("G92 X0 Y0\nG1 X5 Y5 F3000\n") + b + "\nG1 X10 Y10\n";
        run_job(g);
        CHECK(job_had_error(), "'%s' didn't abort", b);
        check_quiescent();
    }
}

static void test_pause_resume() {
    boot();
    std::string g = random_drawing(11, 40);
    at_ms(300, [] { job_pause(); });
    at_ms(2500, [] {
        CHECK(job_state() == JOB_PAUSED, "not paused");
        CHECK(stepper_is_idle() && plan_is_empty(), "still moving while paused");
        CHECK(servo_width == (uint32_t)servo_pen_up_us, "pen down while paused");
        CHECK(!pins[P_EN], "motors released while paused");
        CHECK(cpu_mhz == 80, "CPU not slowed while paused");
        job_resume();
    });
    run_job(g);
    Expect e = reference(g, 0, 0);
    CHECK(motor[0] == e.m1 && motor[1] == e.m2, "end position after pause");
    // The drawing itself is unchanged: pause adds one up/down pair at most.
    CHECK(pen_events.size() == e.pens.size() || pen_events.size() == e.pens.size() + 2,
          "%zu pen changes vs %zu", pen_events.size(), e.pens.size());
    CHECK(!job_had_error(), "error");
    check_quiescent();
}

static void test_stop_mid_move() {
    boot();
    std::string g = "G92 X0 Y0\nM5\nG1 X100 Y10 F16000\nM3\nG1 X100 Y100 F4000\nG1 X10 Y100\nM5\n";
    uint64_t t_stop = 0;
    at_ms(400, [&] { t_stop = now_ns; job_stop(); });
    run_job(g);
    CHECK(t_stop != 0, "stop never sent");
    CHECK(!job_had_error(), "stop isn't an error");
    CHECK(motor[0] == 0 && motor[1] == 0, "not back at origin");
    CHECK(servo_width == (uint32_t)servo_pen_up_us, "pen not up");
    check_quiescent();
    // It decelerated rather than stopping dead: motor 1 was heading away
    // from the origin at full speed, and kept stepping that way, ever more
    // slowly, for a ramp's worth of steps before turning back.
    size_t i = 0;
    while (i < trace.size() && trace[i].ns < t_stop) i++;
    int ramp = 0;
    uint64_t prev = 0, prev_gap = 0;
    bool slowing = true;
    // Step events (either motor; Bresenham makes one motor's own gaps uneven)
    // until motor 1 turns back.
    for (; i < trace.size(); i++) {
        if (trace[i].motor == 0 && !trace[i].dir) break;
        if (prev && trace[i].ns - prev < 1000) continue;  // same event
        if (prev) {
            uint64_t gap = trace[i].ns - prev;
            if (prev_gap && gap + 2000 < prev_gap) slowing = false;
            prev_gap = gap;
        }
        prev = trace[i].ns;
        ramp++;
    }
    CHECK(ramp > 50, "stopped within %d steps", ramp);
    CHECK(slowing, "sped up while stopping");
    CHECK(prev_gap >= 300000, "last step before reversing %.0fus apart", prev_gap / 1e3);
}

static void test_stop_during_pen_move() {
    boot();
    std::string g = "G92 X0 Y0\nM5\nG1 X10 Y10 F16000\nM3\nG1 X50 Y10 F4000\nM5\n";
    // Stop as soon as the pen starts down.
    bool sent = false;
    std::function<void()> poll;
    poll = [&] {
        if (!sent && pen_travelling) { job_stop(); sent = true; return; }
        if (!sent) at_ms(now_ns / 1e6 + 1, poll);
    };
    at_ms(1, poll);
    run_job(g);
    CHECK(sent, "pen never moved");
    CHECK(motor[0] == 0 && motor[1] == 0, "not back at origin");
    CHECK(servo_width == (uint32_t)servo_pen_up_us, "pen not up");
    check_quiescent();
}

static void test_m0_pause_in_file() {
    boot();
    std::string g = "G92 X0 Y0\nM5\nG1 X10 Y10 F8000\nM3\nG1 X30 Y10\nM0\nG1 X30 Y30\nM5\n";
    at_ms(3000, [] {
        CHECK(job_state() == JOB_PAUSED, "M0 didn't pause");
        job_resume();
    });
    run_job(g);
    Expect e = reference(g, 0, 0);
    CHECK(motor[0] == e.m1 && motor[1] == e.m2, "end position");
    check_quiescent();
}

static void test_edge_files() {
    boot();
    write_job("");
    CHECK(!job_start(), "empty file started");
    run_job("; only\n\n(comments)\n   \n");
    CHECK(!job_had_error(), "comments-only errored");
    check_quiescent();
    std::string crlf = "G92 X0 Y0\r\nM5\r\nG1 X5 Y5 F3000\r\nM3\r\nG1 X9 Y5\r\nM5\r\n";
    run_job(crlf);
    CHECK(!job_had_error(), "CRLF errored");
    check_quiescent();
    // Overflowing the line buffer is only an error if a move could be lost.
    run_job("G92 X0 Y0\nG1 X1 Y1 F3000 ;" + std::string(200, 'x') + "\n");
    CHECK(!job_had_error(), "overlong comment aborted");
    check_quiescent();
    run_job("G92 X0 Y0\nG1 X1 Y1 F3000" + std::string(200, ' ') + "\nG1 X2 Y2\n");
    CHECK(!job_had_error(), "overlong trailing whitespace aborted");
    check_quiescent();
    run_job("G92 X0 Y0\n" + std::string(90, ' ') + "G1 X1 Y1 F3000 X2\n");
    CHECK(job_had_error(), "overlong move accepted");
    check_quiescent();
    run_job("G92 X0 Y0\nG1 X5 Y5 F3000\n");  // no trailing pen, no newline issues
    CHECK(!job_had_error(), "plain file errored");
    check_quiescent();
    run_job("G92 X0 Y0\nG1 X5 Y5 F3000");  // last line without newline
    CHECK(!job_had_error(), "no-final-newline errored");
    check_quiescent();
}

static void test_quality_settings() {
    boot();
    int commits0 = EEPROM.commits;
    CHECK(strcmp(job_quality(), "draft") == 0, "default quality");
    job_set_quality("poster"); run();
    CHECK(strcmp(job_quality(), "poster") == 0, "poster not applied");
    CHECK(planner_max_accel_mm_s2 == 6000, "poster accel");
    CHECK(EEPROM.commits == commits0 + 1, "poster: %d commits", EEPROM.commits - commits0);
    job_set_quality("poster"); run();
    CHECK(EEPROM.commits == commits0 + 1, "re-sent preset rewrote flash");
    CHECK(!job_set_quality("bogus"), "bogus accepted");
    // Survives a reboot.
    settings_load();
    CHECK(quality_poster && planner_max_accel_mm_s2 == 6000, "lost across reboot");
    // A poster-quality job still completes correctly.
    std::string g = random_drawing(5, 10);
    run_job(g);
    CHECK(motor[0] == reference(g, 0, 0).m1 && motor[1] == reference(g, 0, 0).m2, "end");
    check_quiescent();
    // Refused mid-job.
    run_job(random_drawing(6, 5));
    write_job(random_drawing(6, 20));
    job_start();
    at_ms(now_ns / 1e6 + 50, [] { CHECK(!job_set_quality("draft"), "quality changed mid-job"); });
    run();
    check_quiescent();
}

static void test_requests_while_idle() {
    boot();
    job_pause(); job_resume(); job_stop(); run();
    CHECK(job_state() == JOB_IDLE, "idle requests changed state");
    CHECK(wakes_total == 0, "idle no-op requests woke the loop %d times", wakes_total);
    std::string g = random_drawing(9, 5);
    write_job(g);
    CHECK(job_start(), "start");
    CHECK(!job_start(), "second start while starting accepted");
    run();
    check_quiescent();
}

// Idle: once a job is done nothing is scheduled at all — no wakes, no
// interrupts, for as long as the sim cares to look.
static void test_idle_is_silent() {
    boot();
    run_job(random_drawing(2, 3));
    int w = wakes_total;
    now_ns += 3600ull * 1000000000ull;
    run();
    CHECK(wakes_total == w, "woke while idle");
    check_quiescent();
}

static void test_soft_limits_and_offsets() {
    boot();
    // A G92 origin mid-bed shrinks the drawable area: X-20 is machine 30.
    std::string g = "G1 X50 Y50 F8000\nG92 X0 Y0\nM3\nG1 X-20 Y-20\nG1 X20 Y-40\nM5\n";
    run_job(g);
    CHECK(!job_had_error(), "in-bounds offsets aborted");
    check_quiescent();
    float x, y;
    stepper_get_machine_position_mm(&x, &y);
    std::string g2 = fmt("G92 X0 Y0\nG1 X%.3f Y0 F8000\n", 125.0f - x + 1);
    run_job(g2);
    CHECK(job_had_error(), "past the envelope accepted");
    check_quiescent();
}

// Random parse timing plus web requests landing at random moments: pauses,
// resumes, stops, duplicate and out-of-place requests. Every edge check
// still applies; each job must end quiescent, and either complete exactly or
// (when stopped) return to where it started.
static void test_chaos() {
    boot();
    chaos = true;
    for (unsigned seed = 1; seed <= 25; seed++) {
        srand(seed * 7919);
        std::string g = random_drawing(seed + 100, 8 + rand() % 25);
        float x0, y0;
        stepper_get_machine_position_mm(&x0, &y0);
        int64_t o1 = motor[0], o2 = motor[1];
        bool stopped = false;
        double t = now_ns / 1e6;
        int actions = rand() % 8;
        for (int i = 0; i < actions; i++) {
            t += rand() % 1500;
            switch (rand() % 6) {
                case 0: case 1: at_ms(t, [] { job_pause(); }); break;
                case 2: case 3: at_ms(t, [] { job_resume(); }); break;
                case 4: at_ms(t, [] { job_set_quality(rand() % 2 ? "poster" : "draft"); }); break;
                case 5: at_ms(t, [&stopped] { if (job_state() != JOB_IDLE) stopped = true; job_stop(); }); break;
            }
        }
        // Anything still paused at the end gets resumed.
        std::function<void()> unstick;
        unstick = [&] { if (job_state() == JOB_PAUSED) job_resume(); };
        at_ms(t + 2000, unstick);
        write_job(g);
        CHECK(job_start(), "seed %u: start", seed);
        run();
        if (job_state() == JOB_PAUSED) { job_resume(); run(); }
        CHECK(!job_had_error(), "seed %u: error", seed);
        Expect e = reference(g, x0, y0);
        if (stopped)
            CHECK(motor[0] == o1 && motor[1] == o2, "seed %u: not back at origin", seed);
        else
            CHECK(motor[0] == e.m1 && motor[1] == e.m2, "seed %u: end position", seed);
        CHECK(servo_width == (uint32_t)servo_pen_up_us, "seed %u: pen left down", seed);
        check_quiescent();
    }
}

// ───────── main ─────────

struct Test { const char *name; void (*fn)(); };
static const Test tests[] = {
    {"square", test_square},
    {"random_drawings", test_random_drawings},
    {"starved_feed", test_starved_feed},
    {"dense_stipple", test_dense_stipple},
    {"multi_block_lines", test_multi_block_lines},
    {"out_of_bounds_aborts", test_out_of_bounds_aborts},
    {"unsupported", test_unsupported},
    {"pause_resume", test_pause_resume},
    {"stop_mid_move", test_stop_mid_move},
    {"stop_during_pen_move", test_stop_during_pen_move},
    {"m0_pause_in_file", test_m0_pause_in_file},
    {"edge_files", test_edge_files},
    {"quality_settings", test_quality_settings},
    {"requests_while_idle", test_requests_while_idle},
    {"idle_is_silent", test_idle_is_silent},
    {"soft_limits_and_offsets", test_soft_limits_and_offsets},
    {"chaos", test_chaos},
};

int main(int argc, char **argv) {
    if (argc > 1) {
        alarm(300);  // a firmware-side infinite loop fails the test too
        for (const Test &t : tests)
            if (strcmp(t.name, argv[1]) == 0) {
                t.fn();
                if (!job_path.empty()) unlink(job_path.c_str());
                printf("  %s: %llu steps, %d wakes, %d clock switches, %.1fs simulated\n",
                       t.name, (unsigned long long)steps_total, wakes_total,
                       clock_switches, now_ns / 1e9);
                return failures ? 1 : 0;
            }
        fprintf(stderr, "no test %s\n", argv[1]);
        return 2;
    }
    int failed = 0;
    for (const Test &t : tests) {
        pid_t pid = fork();
        if (pid == 0) { execl(argv[0], argv[0], t.name, (char *)nullptr); _exit(3); }
        int st = 0;
        waitpid(pid, &st, 0);
        bool ok = WIFEXITED(st) && WEXITSTATUS(st) == 0;
        printf("%s %s\n", ok ? "PASS" : "FAIL", t.name);
        failed += !ok;
    }
    printf("%d/%zu passed\n", (int)(sizeof tests / sizeof *tests) - failed,
           sizeof tests / sizeof *tests);
    return failed ? 1 : 0;
}
