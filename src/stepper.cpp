#include "stepper.h"
#include "planner.h"
#include "kinematics.h"
#include <Servo.h>
#include "hardware/timer.h"
#include "hardware/irq.h"

// RP2040 hardware alarm used for stepper scheduling.
// Alarms 0–3 exist; alarm 0 is sometimes used by the SDK default pool. We
// claim alarm 1 explicitly to avoid conflict.
#define STEPPER_ALARM_NUM  1
#define STEPPER_ALARM_IRQ  (TIMER_IRQ_0 + STEPPER_ALARM_NUM)

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

// Servo handle (must be constructed in stepper_init after core is up).
static Servo pen_servo;

// For decel-to-stop in feed-hold: we keep decelerating the current block
// until rate reaches zero, then park in ST_HELD.
static uint32_t hold_events_at_start;
static float    hold_start_rate;

static inline void schedule_next_isr(uint32_t delay_us) {
    if (delay_us < MIN_STEP_INTERVAL_US) delay_us = MIN_STEP_INTERVAL_US;
    timer_hw->alarm[STEPPER_ALARM_NUM] = timer_hw->timerawl + delay_us;
}

static void stepper_isr_body();

// Called by IRQ dispatcher. Clears the alarm interrupt and runs the ISR body.
static void stepper_alarm_handler() {
    hw_clear_bits(&timer_hw->intr, 1u << STEPPER_ALARM_NUM);
    stepper_isr_body();
}

void stepper_init() {
    pinMode(PIN_MOTOR1_STEP, OUTPUT);
    pinMode(PIN_MOTOR1_DIR,  OUTPUT);
    pinMode(PIN_MOTOR2_STEP, OUTPUT);
    pinMode(PIN_MOTOR2_DIR,  OUTPUT);
    pinMode(PIN_DRIVER_EN,   OUTPUT);
    digitalWrite(PIN_DRIVER_EN, LOW);   // A4988: LOW = enable

    // Standard position servo: command pen-up position and hold.
    pen_servo.attach(PIN_SERVO);
    pen_servo.writeMicroseconds(DEFAULT_PEN_UP_US);

    // Claim alarm + install IRQ
    hardware_alarm_claim(STEPPER_ALARM_NUM);
    irq_set_exclusive_handler(STEPPER_ALARM_IRQ, stepper_alarm_handler);
    irq_set_enabled(STEPPER_ALARM_IRQ, true);
    hw_set_bits(&timer_hw->inte, 1u << STEPPER_ALARM_NUM);

    state = ST_IDLE;
}

void stepper_reset() {
    // Soft reset: clear execution state but DO NOT zero motor_pos_steps —
    // on real hardware the motors are physically wherever they are; resetting
    // the counter would cause the next motion plan to be wrong.
    hw_clear_bits(&timer_hw->inte, 1u << STEPPER_ALARM_NUM);
    digitalWrite(PIN_MOTOR1_STEP, LOW);
    digitalWrite(PIN_MOTOR2_STEP, LOW);
    state = ST_IDLE;
    cur = nullptr;
    events_done = 0;
    counter_m1 = counter_m2 = 0;
    current_rate = 0;
    hw_set_bits(&timer_hw->inte, 1u << STEPPER_ALARM_NUM);
}

bool stepper_is_idle()    { return state == ST_IDLE; }
bool stepper_is_in_hold() { return state == ST_FEED_HOLD || state == ST_HELD; }

void stepper_wake() {
    if (state == ST_IDLE) {
        schedule_next_isr(10);
    }
}

void stepper_feed_hold() {
    if (state == ST_RUNNING) {
        hold_events_at_start = events_done;
        hold_start_rate = (float)current_rate;
        state = ST_FEED_HOLD;
    }
}

void stepper_cycle_start() {
    if (state == ST_HELD) {
        state = cur ? ST_RUNNING : ST_IDLE;
        schedule_next_isr(10);
    }
}

uint32_t stepper_current_rate_events_per_sec() {
    return current_rate;
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
            pen_servo.writeMicroseconds((int)cur->sync_arg);
            delay(DEFAULT_PEN_MOVE_MS);
            break;
        case SYNC_DWELL:
            if (cur->sync_arg > 0) delay((unsigned long)cur->sync_arg);
            break;
        case SYNC_SET_ENABLE:
            digitalWrite(PIN_DRIVER_EN, cur->sync_arg ? LOW : HIGH);
            break;
    }

    // Consume the sync block and resume.
    cur = nullptr;
    plan_discard_current_block();
    state = ST_IDLE;
    stepper_wake();
}

// ─────── ISR body ───────

static void load_next_block() {
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
    digitalWrite(PIN_MOTOR1_DIR, (cur->direction_bits & 0x01) ? LOW : HIGH);
    digitalWrite(PIN_MOTOR2_DIR, (cur->direction_bits & 0x02) ? LOW : HIGH);
    state = ST_RUNNING;
}

static inline uint32_t compute_rate(uint32_t k) {
    if (k < cur->accelerate_until) {
        // v² = v0² + 2·a·k  (in step-event space)
        float vsq = (float)cur->initial_rate * (float)cur->initial_rate
                    + 2.0f * (float)cur->accel_st * (float)k;
        if (vsq <= 0.0f) return 1;  // guard against NaN from sqrtf(negative)
        return (uint32_t)sqrtf(vsq);
    } else if (k < cur->decelerate_after) {
        return cur->nominal_rate;
    } else {
        uint32_t remaining = cur->step_event_count - k;
        float vsq = (float)cur->final_rate * (float)cur->final_rate
                    + 2.0f * (float)cur->accel_st * (float)remaining;
        if (vsq <= 0.0f) return 1;  // guard against NaN from sqrtf(negative)
        return (uint32_t)sqrtf(vsq);
    }
}

// Called from the timer alarm IRQ.
static void stepper_isr_body() {
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

    if (pulse_m1) digitalWrite(PIN_MOTOR1_STEP, HIGH);
    if (pulse_m2) digitalWrite(PIN_MOTOR2_STEP, HIGH);
    // A4988 min pulse width = 1µs; hold 2µs for margin.
    delayMicroseconds(STEP_PULSE_US);
    if (pulse_m1) digitalWrite(PIN_MOTOR1_STEP, LOW);
    if (pulse_m2) digitalWrite(PIN_MOTOR2_STEP, LOW);

    if (pulse_m1) motor_pos_steps[0] += (cur->direction_bits & 0x01) ? -1 : 1;
    if (pulse_m2) motor_pos_steps[1] += (cur->direction_bits & 0x02) ? -1 : 1;

    events_done++;

    // Block done?
    if (events_done >= cur->step_event_count) {
        // If we're in feed hold when a block completes, don't load the next
        // one — park in ST_HELD. Otherwise we'd lose the hold state and the
        // next block would start at full nominal rate. Letting the decel chain
        // span blocks would be nicer but adds complexity; for a pen plotter
        // an abrupt stop at a block boundary is acceptable.
        if (state == ST_FEED_HOLD) {
            cur = nullptr;
            plan_discard_current_block();
            state = ST_HELD;
            current_rate = 0;
            return;
        }
        cur = nullptr;
        plan_discard_current_block();
        load_next_block();
        if (state == ST_IDLE || state == ST_WAIT_SYNC) return;
        // Need to compute first-step interval of the new block.
    }

    // Compute next step-event rate.
    uint32_t rate;
    if (state == ST_FEED_HOLD) {
        // Decel to zero from hold_start_rate at block's accel rate.
        uint32_t k_since_hold = events_done - hold_events_at_start;
        float vsq = hold_start_rate * hold_start_rate
                    - 2.0f * (float)cur->accel_st * (float)k_since_hold;
        if (vsq <= 0.0f) {
            state = ST_HELD;
            current_rate = 0;
            return;
        }
        rate = (uint32_t)sqrtf(vsq);
    } else {
        rate = compute_rate(events_done);
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
            rate = (uint32_t)sqrtf((float)cur->accel_st * 0.5f);
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

#ifdef FIRMWARE_TEST
// Host-test entry point: run one ISR tick without touching any hardware IRQ.
// The simulator calls this after advancing virtual time to `timer_hw->alarm[]`.
extern "C" void stepper_tick_for_test() { stepper_isr_body(); }
extern "C" bool stepper_test_is_running()   { return state == ST_RUNNING;   }
extern "C" bool stepper_test_is_wait_sync() { return state == ST_WAIT_SYNC; }
extern "C" bool stepper_test_is_held()      { return state == ST_HELD;      }
extern "C" bool stepper_test_is_feed_hold() { return state == ST_FEED_HOLD; }
// Zero motor_pos_steps for test isolation. Not called by any production
// path — real soft reset preserves physical motor position.
extern "C" void stepper_test_zero_position() {
    motor_pos_steps[0] = 0;
    motor_pos_steps[1] = 0;
}
#endif

