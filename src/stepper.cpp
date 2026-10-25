#include "stepper.h"
#include "planner.h"
#include "kinematics.h"
#include "steptimer.h"

// IRAM_ATTR comes from config.h (via stepper.h).
// The pen servo is behind penservo.h because on ESP32-S3 it can't use PWM at
// all — see the explanation there.
#include "penservo.h"

enum StepperState : uint8_t {
    ST_IDLE,
    ST_RUNNING,
    ST_WAIT_SYNC,     // blocked on a sync command; main loop services it
    ST_FEED_HOLD,     // decelerating to zero from realtime `!`
    ST_HELD,          // stopped, buffer retained
};

static volatile StepperState state = ST_IDLE;

// Current block state
static plan_block_t *cur = nullptr;
static int32_t  counter_m1;       // Bresenham accumulator, motor 1
static int32_t  counter_m2;       // Bresenham accumulator, motor 2
static uint32_t events_done;      // step events completed in current block

// Accumulated motor position (in steps, signed). Used for status reports;
// converted to Cartesian in stepper_get_machine_position_mm.
static volatile int32_t motor_pos_steps[2] = {0, 0};

// Current step-event rate (events/s), exposed for status report.
static volatile uint32_t current_rate = 0;


// For decel-to-stop in feed-hold: we keep decelerating the current block
// until rate reaches zero, then park in ST_HELD.
static uint32_t hold_events_at_start;
static uint32_t hold_start_rate;  // integer: read from the ISR, no FPU

// After a hold the motors are at rest, but the planned profile may want
// cruise speed at this point of the block (or a nonzero junction speed at
// the next one). Until the profile drops below it, the rate is capped by an
// accel ramp from zero: v² grows by 2·a per step event, across blocks.
static volatile bool resuming = false;

static inline void IRAM_ATTR schedule_next_isr(uint32_t delay_us) {
    if (delay_us < MIN_STEP_INTERVAL_US) delay_us = MIN_STEP_INTERVAL_US;
    steptimer_arm(delay_us);
}

static void IRAM_ATTR stepper_isr_body();

// Fired by the step timer. The platform layer has already acknowledged the
// interrupt by the time we get here.
static void IRAM_ATTR stepper_alarm_handler() {
    stepper_isr_body();
}

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

    steptimer_init(stepper_alarm_handler);

    state = ST_IDLE;
}

void stepper_reset() {
    // Soft reset: clear execution state but DO NOT zero motor_pos_steps —
    // on real hardware the motors are physically wherever they are; resetting
    // the counter would cause the next motion plan to be wrong.
    // Disarm first so the ISR can't fire midway through clearing state.
    steptimer_cancel();
    digitalWrite(PIN_MOTOR1_STEP, LOW);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    state = ST_IDLE;
    cur = nullptr;
    resuming = false;
    events_done = 0;
    counter_m1 = counter_m2 = 0;
    current_rate = 0;
}

bool stepper_is_idle()    { return state == ST_IDLE; }
bool stepper_is_in_hold() { return state == ST_FEED_HOLD || state == ST_HELD; }
bool stepper_is_stopped() { return state == ST_IDLE || state == ST_HELD; }

void stepper_wake() {
    if (state == ST_IDLE) {
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

void stepper_cycle_start() {
    if (state == ST_HELD) {
        resuming = true;
        state = cur ? ST_RUNNING : ST_IDLE;
        schedule_next_isr(10);
    }
}

uint32_t stepper_current_rate_events_per_sec() {
    return current_rate;
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

// ─────── Sync-command servicing (main loop side) ───────

void stepper_service_sync() {
    if (state != ST_WAIT_SYNC || cur == nullptr) return;

    switch (cur->sync_cmd) {
        case SYNC_SERVO:
            // Standard position servo: just set the target angle and hold.
            penservo_write_us((uint16_t)cur->sync_arg);
            delay(DEFAULT_PEN_MOVE_MS);
            break;
        case SYNC_DWELL:
            if (cur->sync_arg > 0) delay((unsigned long)cur->sync_arg);
            break;
        case SYNC_SET_ENABLE:
            isr_pin_write(PIN_DRIVER_EN, !cur->sync_arg);
            break;
    }

    // Consume the sync block and resume.
    cur = nullptr;
    plan_discard_current_block();
    state = ST_IDLE;
    stepper_wake();
}

// ─────── ISR body ───────

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
static inline uint32_t IRAM_ATTR isqrt(uint64_t n) {
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

// Called from the timer alarm IRQ.
static void IRAM_ATTR stepper_isr_body() {
    if (state == ST_HELD) return;  // ignore stale fires

    if (state == ST_WAIT_SYNC) return;  // main loop will wake us

    if (cur == nullptr) {
        load_next_block();
        if (state == ST_IDLE || state == ST_WAIT_SYNC) return;
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
    // A4988 min pulse width = 1µs; hold 2µs for margin.
    isr_delay_us(STEP_PULSE_US);
    if (pulse_m1) isr_pin_write(PIN_MOTOR1_STEP, false);
    if (pulse_m2) isr_pin_write(PIN_MOTOR2_STEP, false);

    if (pulse_m1) motor_pos_steps[0] += (cur->direction_bits & 0x01) ? -1 : 1;
    if (pulse_m2) motor_pos_steps[1] += (cur->direction_bits & 0x02) ? -1 : 1;

    events_done++;

    // Block done?
    if (events_done >= cur->step_event_count) {
        // A feed hold that hasn't finished decelerating carries on into the
        // next motion block, so the stop is a continuous ramp rather than an
        // abrupt halt at the block boundary. The rate is in step events/s,
        // whose scale differs per block, so convert it at the same tip speed.
        // A sync block (pen, dwell) or an empty planner ends the hold here:
        // the planner already brought the speed to zero for those.
        if (state == ST_FEED_HOLD) {
            uint32_t prev_epm = cur->events_per_mm_q8;
            cur = nullptr;
            plan_discard_current_block();
            plan_block_t *next = plan_get_current_block();
            if (next == nullptr || next->sync_cmd != SYNC_NONE ||
                prev_epm == 0 || next->events_per_mm_q8 == 0) {
                state = ST_HELD;
                current_rate = 0;
                return;
            }
            load_next_block();                 // sets ST_RUNNING
            hold_start_rate = current_rate * next->events_per_mm_q8 / prev_epm;
            hold_events_at_start = 0;
            state = ST_FEED_HOLD;
        } else {
            cur = nullptr;
            plan_discard_current_block();
            load_next_block();
            if (state == ST_IDLE || state == ST_WAIT_SYNC) return;
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
        if (drop >= v0sq) {
            state = ST_HELD;
            current_rate = 0;
            return;
        }
        rate = isqrt(v0sq - drop);
    } else {
        rate = compute_rate(events_done);
        if (resuming) {
            // Integer, for the same FPU reason as compute_rate().
            uint32_t cap = isqrt((uint64_t)current_rate * current_rate
                                 + 2ULL * cur->accel_st);
            if (rate > cap) rate = cap;
            else resuming = false;  // profile is below the ramp: caught up
        }
    }

    if (rate == 0) {
        // Starting from rest (v_entry=0): compute_rate(0) returns 0 because
        // the accel ramp is v=sqrt(2·a·k), which is zero at k=0. Falling
        // back to rate=1 gives a 1-second first-step period, which compounds
        // to 10+ s of stall on any pattern with many full-stop junctions
        // (arc-to-arc cusps, 180° reversals, sync barriers). Use the
        // kinematic first-step rate sqrt(a/2) — the speed you'd reach after
        // advancing one step at constant accel from rest.
        if (cur && cur->accel_st > 0) {
            rate = isqrt((uint64_t)cur->accel_st / 2);
            if (rate < 1) rate = 1;
        } else {
            rate = 1;
        }
    }
    if (rate == 0) rate = 1;  // safety: prevent division by zero → HardFault
    current_rate = rate;

    uint32_t interval_us = 1000000UL / rate;
    schedule_next_isr(interval_us);
}

