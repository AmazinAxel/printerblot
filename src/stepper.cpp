#include "stepper.h"
#include "planner.h"
#include "kinematics.h"
#include "steptimer.h"
#include "wake.h"

// IRAM_ATTR comes from config.h (via stepper.h).
// The pen servo is behind penservo.h because on ESP32-S3 it can't use PWM at
// all — see the explanation there.
#include "penservo.h"

enum StepperState : uint8_t {
    ST_IDLE,
    ST_RUNNING,
    ST_WAIT_SYNC,     // parked on a pen block; the loop services it
    ST_FEED_HOLD,     // decelerating to zero (job stop)
    ST_HELD,          // stopped by the hold, buffer retained
};

static volatile StepperState state = ST_IDLE;

// Current block state
static plan_block_t *cur = nullptr;
static int32_t  counter_m1;       // Bresenham accumulator, motor 1
static int32_t  counter_m2;       // Bresenham accumulator, motor 2
static uint32_t events_done;      // step events completed in current block

// Accumulated motor position (in steps, signed), converted to Cartesian in
// stepper_get_machine_position_mm.
static volatile int32_t motor_pos_steps[2] = {0, 0};

// Current step-event rate (events/s): where a feed hold decelerates from.
static uint32_t current_rate = 0;

// For decel-to-stop in feed-hold: we keep decelerating the current block
// until rate reaches zero, then park in ST_HELD.
static uint32_t hold_events_at_start;
static uint32_t hold_start_rate;  // integer: read from the ISR, no FPU

// Set once the loop has sent the pen of the parked sync block on its way.
static bool sync_started = false;

static bool drivers_on = false;

static inline void IRAM_ATTR schedule_next_isr(uint32_t delay_us) {
    if (delay_us < MIN_STEP_INTERVAL_US) delay_us = MIN_STEP_INTERVAL_US;
    steptimer_arm(delay_us);
}

static bool IRAM_ATTR stepper_isr();

void stepper_init() {
    // Disable the drivers BEFORE making the pin an output. pinMode(OUTPUT)
    // immediately drives whatever is in the output register, which after reset
    // is 0 — and 0 is A4988 "enabled". Setting the level first means the pin
    // goes straight from floating to disabled, with no enabled window.
    digitalWrite(PIN_DRIVER_EN, HIGH);  // A4988: HIGH = disabled
    pinMode(PIN_DRIVER_EN,   OUTPUT);
    digitalWrite(PIN_DRIVER_EN, HIGH);  // again: pinMode() reasserts the register

    // Park STEP/DIR low before driving them, for the same reason. PIN_MOTOR2_DIR
    // is GPIO44 (U0RXD) and PIN_SERVO is GPIO43 (U0TXD): the ROM bootloader owns
    // both until these lines run, so they arrive here in an unknown state. With
    // the drivers already disabled above, any glitch on them is harmless.
    digitalWrite(PIN_MOTOR1_STEP, LOW);
    digitalWrite(PIN_MOTOR1_DIR,  LOW);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    digitalWrite(PIN_MOTOR2_DIR,  LOW);
    pinMode(PIN_MOTOR1_STEP, OUTPUT);
    pinMode(PIN_MOTOR1_DIR,  OUTPUT);
    pinMode(PIN_MOTOR2_STEP, OUTPUT);
    pinMode(PIN_MOTOR2_DIR,  OUTPUT);

    // Standard position servo: command pen-up position and hold. The target is
    // passed into init so the very first pulse is already the right one.
    penservo_init(PIN_SERVO, DEFAULT_PEN_UP_US);

    steptimer_init(stepper_isr);

    state = ST_IDLE;
}

void stepper_reset() {
    // Clear execution state but DO NOT zero motor_pos_steps: the motors are
    // physically wherever they are, and the next plan starts from there.
    // Disarm first so the ISR can't fire midway through clearing state.
    steptimer_cancel();
    digitalWrite(PIN_MOTOR1_STEP, LOW);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    state = ST_IDLE;
    cur = nullptr;
    sync_started = false;
    events_done = 0;
    counter_m1 = counter_m2 = 0;
    current_rate = 0;
}

bool stepper_is_idle()    { return state == ST_IDLE; }
// A parked sync block counts: the motors are at rest while the pen moves.
bool stepper_is_stopped() {
    return state == ST_IDLE || state == ST_HELD || state == ST_WAIT_SYNC;
}

void stepper_set_drivers(bool on) {
    digitalWrite(PIN_DRIVER_EN, on ? LOW : HIGH);  // A4988: LOW = enabled
    drivers_on = on;
}

bool stepper_drivers_on() { return drivers_on; }

// CPU clock: 80MHz (the floor WiFi allows) while idle, 240MHz for motion.
// The step ISR's pulse timing and cycle budget assume F_CPU, so the only
// path that starts the step timer from rest, stepper_wake(), goes through
// cpu_full_speed() first. APB stays 80MHz either way, so the step/servo
// timers are unaffected by the switch.
static bool cpu_slow = false;

static void cpu_full_speed() {
    if (!cpu_slow) return;
    setCpuFrequencyMhz(F_CPU / 1000000);
    cpu_slow = false;
}

void stepper_power_save() {
    if (cpu_slow || state != ST_IDLE || !plan_is_empty()) return;
    setCpuFrequencyMhz(80);
    cpu_slow = true;
}

void stepper_wake() {
    if (state == ST_IDLE && !plan_is_empty()) {
        cpu_full_speed();
        schedule_next_isr(10);
    }
}

void stepper_feed_hold() {
    if (state == ST_RUNNING) {
        hold_events_at_start = events_done;
        hold_start_rate = current_rate;
        state = ST_FEED_HOLD;
    }
}

// In ST_WAIT_SYNC the block is a zero-length sync marker the main loop owns,
// and in ST_IDLE `cur` is already nullptr — neither needs protecting. ST_HELD
// does: a hold that ran out mid-block parks with `cur` partly executed.
bool stepper_is_executing(const plan_block_t *b, uint32_t *events) {
    if (b == nullptr || cur != b || state == ST_IDLE || state == ST_WAIT_SYNC)
        return false;
    if (events) *events = events_done;
    return true;
}

void stepper_get_machine_position_mm(float *x, float *y) {
    // Convert motor steps back to Cartesian. Snapshot volatile reads.
    int32_t m1 = motor_pos_steps[0];
    int32_t m2 = motor_pos_steps[1];
    float a = (float)m1 / STEPS_PER_MM;
    float b = (float)m2 / STEPS_PER_MM;
    motor_to_cartesian(a, b, x, y);
}

// ─────── Sync blocks (main loop side) ───────
//
// The ISR wakes the loop when it parks on a pen block. The first pass sends
// the pen on its way; if it has to travel, the servo ISR wakes the loop
// again once it has arrived, and that pass releases the block. No ISR is
// armed while parked, so nothing races with this.
void stepper_service() {
    if (state != ST_WAIT_SYNC || cur == nullptr) return;

    if (!sync_started) {
        sync_started = true;
        if (penservo_move((uint16_t)cur->sync_arg)) return;  // travelling
    } else if (!penservo_arrived()) {
        return;
    }

    sync_started = false;
    cur = nullptr;
    plan_discard_current_block();
    state = ST_IDLE;
    stepper_wake();
}

// ─────── ISR ───────

static void IRAM_ATTR load_next_block() {
    cur = plan_get_current_block();
    if (cur == nullptr) {
        state = ST_IDLE;
        current_rate = 0;
        return;
    }
    if (cur->sync_cmd != SYNC_NONE) {
        // Sync block; hand off to main loop.
        state = ST_WAIT_SYNC;
        current_rate = 0;
        return;
    }
    // Regular motion block.
    events_done = 0;
    uint32_t N = cur->step_event_count;
    counter_m1 = -(int32_t)(N >> 1);
    counter_m2 = -(int32_t)(N >> 1);
    isr_pin_write(PIN_MOTOR1_DIR, !(cur->direction_bits & 0x01));
    isr_pin_write(PIN_MOTOR2_DIR, !(cur->direction_bits & 0x02));
    state = ST_RUNNING;
}

// The stepper ISR is registered with ESP_INTR_FLAG_IRAM, which requires
// everything it calls to be resident in IRAM. newlib's sqrtf lives in flash
// (verified: it links at 0x42081d18, outside the 0x40370000-0x403E0000 IRAM
// window), so calling it from the ISR is illegal.
//
// Every caller here immediately truncates to an integer step rate, so an
// integer sqrt is exact for our purposes and pulls in no libm at all. Binary
// digit-by-digit method: fully branch-predictable, no FPU, no flash access.
//
// Real inputs are v² ≤ ~1e9, so the 32-bit path (half the iterations, and
// native-width ops on this 32-bit core) is the one that runs; it matches the
// 64-bit one exactly over the whole uint32 range.
static inline uint32_t IRAM_ATTR isqrt(uint64_t n) {
    if (n <= 0xFFFFFFFFu) {
        uint32_t m = (uint32_t)n, rem = 0, root = 0;
        for (int i = 0; i < 16; i++) {
            root <<= 1;
            rem = (rem << 2) | (m >> 30);
            m <<= 2;
            if (root < rem) {
                rem -= root | 1;
                root += 2;
            }
        }
        return root >> 1;
    }
    uint64_t rem = 0, root = 0;
    for (int i = 0; i < 32; i++) {
        root <<= 1;
        rem = (rem << 2) | (n >> 62);
        n <<= 2;
        if (root < rem) {
            rem -= root | 1;
            root += 2;
        }
    }
    return (uint32_t)(root >> 1);
}

// NOTE: strictly integer math. Xtensa FreeRTOS allocates the FPU lazily per
// task and does NOT save coprocessor state across interrupts, so *any* float
// operation in this ISR raises EXCCAUSE 4 (Coprocessor exception) as soon as
// another task owns the FPU — which showed up as a crash mid-print whenever
// the network stack was busy. All the inputs here are integer step rates
// anyway, so 64-bit ints cover the range with room to spare
// (max rate 40000 => v² = 1.6e9, fits in uint64 easily).
static inline uint32_t IRAM_ATTR compute_rate(uint32_t k) {
    if (k < cur->accelerate_until) {
        // v² = v0² + 2·a·k  (in step-event space)
        uint64_t vsq = (uint64_t)cur->initial_rate * cur->initial_rate
                     + 2ULL * cur->accel_st * k;
        return isqrt(vsq);
    } else if (k < cur->decelerate_after) {
        return cur->nominal_rate;
    } else {
        uint32_t remaining = cur->step_event_count - k;
        uint64_t vsq = (uint64_t)cur->final_rate * cur->final_rate
                     + 2ULL * cur->accel_st * remaining;
        return isqrt(vsq);
    }
}

// Finish a step pulse started at cycle count `start`: wait out whatever is
// left of the A4988's minimum high time (1µs; we hold 2µs for margin), then
// drop both STEP lines. Idempotent, so every exit path can call it.
static inline void IRAM_ATTR end_step_pulse(uint32_t start) {
    isr_delay_us_since(start, STEP_PULSE_US);
    isr_pin_write(PIN_MOTOR1_STEP, false);
    isr_pin_write(PIN_MOTOR2_STEP, false);
}

// Called from the timer alarm IRQ. Returns true if it woke the loop task
// (a planner slot freed, a pen block reached, or motion stopped).
static bool IRAM_ATTR stepper_isr() {
    // Parked states are only left by the loop, which re-arms the timer.
    if (state == ST_HELD || state == ST_WAIT_SYNC) return false;

    if (cur == nullptr) {
        load_next_block();
        if (state == ST_WAIT_SYNC) return wake_loop_from_isr();
        if (state == ST_IDLE) return false;
        // ST_RUNNING with fresh block: fall through to emit first step event
    }

    // Emit one step event (Bresenham).
    int32_t N = (int32_t)cur->step_event_count;
    bool pulse_m1 = false, pulse_m2 = false;

    counter_m1 += cur->steps[0];
    if (counter_m1 > 0) {
        pulse_m1 = true;
        counter_m1 -= N;
    }
    counter_m2 += cur->steps[1];
    if (counter_m2 > 0) {
        pulse_m2 = true;
        counter_m2 -= N;
    }

    if (pulse_m1) isr_pin_write(PIN_MOTOR1_STEP, true);
    if (pulse_m2) isr_pin_write(PIN_MOTOR2_STEP, true);
    // The pins come down in end_step_pulse(): the bookkeeping and rate math
    // below run during the pulse instead of a dead busy-wait.
    uint32_t pulse_start = XTHAL_GET_CCOUNT();

    if (pulse_m1) motor_pos_steps[0] += (cur->direction_bits & 0x01) ? -1 : 1;
    if (pulse_m2) motor_pos_steps[1] += (cur->direction_bits & 0x02) ? -1 : 1;

    events_done++;
    bool woke = false;

    // Block done?
    if (events_done >= cur->step_event_count) {
        // Before the next block can change DIR under a still-high STEP.
        end_step_pulse(pulse_start);
        // A planner slot is free: the job feeder can queue another line.
        woke = wake_loop_from_isr();
        // A feed hold that hasn't finished decelerating carries on into the
        // next motion block, so the stop is a continuous ramp rather than an
        // abrupt halt at the block boundary. The rate is in step events/s,
        // whose scale differs per block, so convert it at the same tip speed.
        // A pen block or an empty planner ends the hold here: the planner
        // already brought the speed to zero for those.
        if (state == ST_FEED_HOLD) {
            uint32_t prev_epm = cur->events_per_mm_q8;
            cur = nullptr;
            plan_discard_current_block();
            plan_block_t *next = plan_get_current_block();
            if (next == nullptr || next->sync_cmd != SYNC_NONE ||
                prev_epm == 0 || next->events_per_mm_q8 == 0) {
                state = ST_HELD;
                current_rate = 0;
                return woke;
            }
            load_next_block();                 // sets ST_RUNNING
            hold_start_rate = current_rate * next->events_per_mm_q8 / prev_epm;
            hold_events_at_start = 0;
            state = ST_FEED_HOLD;
        } else {
            cur = nullptr;
            plan_discard_current_block();
            load_next_block();
            if (state == ST_IDLE || state == ST_WAIT_SYNC) return woke;
        }
        // Need to compute first-step interval of the new block.
    }

    // Compute next step-event rate.
    uint32_t rate;
    if (state == ST_FEED_HOLD) {
        // Decel to zero from hold_start_rate at block's accel rate.
        uint32_t k_since_hold = events_done - hold_events_at_start;
        // Integer, for the same FPU reason as compute_rate(). This subtracts,
        // so compare before subtracting rather than letting it wrap.
        uint64_t v0sq = (uint64_t)hold_start_rate * hold_start_rate;
        uint64_t drop = 2ULL * cur->accel_st * k_since_hold;
        // Stop once down to the speed a start from rest begins at (the
        // sqrt(a/2) below). Ramping on toward zero only adds a crawl: a last
        // step or two tens of ms apart, delaying the stop for nothing.
        if (drop + cur->accel_st / 2 >= v0sq) {
            end_step_pulse(pulse_start);
            state = ST_HELD;
            current_rate = 0;
            return wake_loop_from_isr() || woke;
        }
        rate = isqrt(v0sq - drop);
    } else {
        rate = compute_rate(events_done);
    }

    if (rate == 0) {
        // Starting from rest (v_entry=0): compute_rate(0) returns 0 because
        // the accel ramp is v=sqrt(2·a·k), which is zero at k=0. Falling
        // back to rate=1 gives a 1-second first-step period, which compounds
        // to 10+ s of stall on any pattern with many full-stop junctions
        // (180° reversals, pen blocks). Use the kinematic first-step rate
        // sqrt(a/2) — the speed you'd reach after advancing one step at
        // constant accel from rest.
        // Never 0 after this: it's a divisor below.
        rate = (cur->accel_st > 0) ? isqrt((uint64_t)cur->accel_st / 2) : 1;
        if (rate < 1) rate = 1;
    }
    current_rate = rate;

    uint32_t interval_us = 1000000UL / rate;
    end_step_pulse(pulse_start);
    schedule_next_isr(interval_us);
    return woke;
}
