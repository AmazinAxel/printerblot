#include "penservo.h"

#include <Arduino.h>

#include "config.h"
#include "wake.h"

#include "driver/timer.h"
#include "esp_attr.h"
#include "soc/gpio_struct.h"

// Timer group 1 / timer 0. The stepper owns group 0 / timer 0, so these share
// no registers and the servo can never disturb step timing.
#define SERVO_TG   TIMER_GROUP_1
#define SERVO_IDX  TIMER_0
#define SERVO_DIV  (TIMER_BASE_CLK / 1000000) // 1 tick = 1µs

#define SERVO_FRAME_US 20000  // 50Hz refresh, standard for hobby servos

// How long to keep pulsing after a command before going quiet. Must
// comfortably exceed the travel time, so allow a healthy margin over it.
#define SERVO_HOLD_MS  400

// Counted on falling edges: the travel time is over once this many pulses
// of the new width have gone out (their first rising edge + 60ms, rounded up
// to whole frames). A pulse already in flight when the target changes still
// has the old width and is counted on top.
#define SETTLE_FRAMES (DEFAULT_PEN_MOVE_MS * 1000 / SERVO_FRAME_US + 1)
#define HOLD_FRAMES   (SERVO_HOLD_MS * 1000 / SERVO_FRAME_US)

static uint8_t  s_pin = 0;
static volatile uint16_t s_pulse_us = 1000;
static volatile bool     s_high = false;
static volatile bool     s_active = false;  // pulse train running
static volatile uint8_t  s_settle_frames = 0;  // until arrival; 0 = arrived
static volatile uint8_t  s_hold_frames = 0;    // until the train goes quiet

static inline void IRAM_ATTR arm_in_isr(uint32_t delay_us) {
    uint64_t now = timer_group_get_counter_value_in_isr(SERVO_TG, SERVO_IDX);
    timer_group_set_alarm_value_in_isr(SERVO_TG, SERVO_IDX, now + delay_us);
    timer_group_enable_alarm_in_isr(SERVO_TG, SERVO_IDX);
}

// Pin writes go through isr_pin_write() from config.h. gpio_set_level() lives
// in flash, so calling it from an ESP_INTR_FLAG_IRAM handler panics with
// "Cache disabled but cached memory region accessed" the moment anything
// writes to flash.
//
// Emits the pulse: go high, schedule the fall; go low, count the frame and
// schedule the next one — or, once the hold is over, schedule nothing and
// leave the line low. The alarm doesn't re-enable itself, so not re-arming
// is all it takes to go quiet.
static bool IRAM_ATTR servo_isr(void *) {
    if (!s_high) {
        isr_pin_write(s_pin, true);
        s_high = true;
        arm_in_isr(s_pulse_us);
        return false;
    }

    isr_pin_write(s_pin, false);
    s_high = false;

    bool woke = false;
    if (s_settle_frames && --s_settle_frames == 0) woke = wake_loop_from_isr();
    if (--s_hold_frames == 0) {
        s_active = false;
        return woke;
    }
    arm_in_isr(SERVO_FRAME_US - s_pulse_us);
    return woke;
}

// Start the pulse train from rest: first rising edge almost immediately.
static void start_train() {
    s_high = false;
    uint64_t now = 0;
    timer_get_counter_value(SERVO_TG, SERVO_IDX, &now);
    timer_set_alarm_value(SERVO_TG, SERVO_IDX, now + 100);
    timer_set_alarm(SERVO_TG, SERVO_IDX, TIMER_ALARM_EN);
}

void penservo_init(uint8_t pin, uint16_t initial_us) {
    s_pin = pin;
    // Set the target BEFORE the first frame goes out, so the servo's very
    // first pulse already commands the resting (pen-up) position. Otherwise
    // it would see the default and snap there, then move again — a visible
    // double twitch at power-up.
    s_pulse_us = initial_us;
    s_hold_frames = HOLD_FRAMES;
    s_active = true;

    // Plain GPIO output. GPIO43/44 are valid outputs (only 22-25 are not on
    // the S3); it's specifically the LEDC PWM matrix that can't reach them.
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << pin;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)pin, 0);

    timer_config_t cfg = {};
    cfg.alarm_en    = TIMER_ALARM_DIS;
    cfg.counter_en  = TIMER_PAUSE;
    cfg.intr_type   = TIMER_INTR_LEVEL;
    cfg.counter_dir = TIMER_COUNT_UP;
    cfg.auto_reload = TIMER_AUTORELOAD_DIS;
    cfg.divider     = SERVO_DIV;

    timer_init(SERVO_TG, SERVO_IDX, &cfg);
    timer_set_counter_value(SERVO_TG, SERVO_IDX, 0);
    timer_isr_callback_add(SERVO_TG, SERVO_IDX, servo_isr, nullptr,
                           ESP_INTR_FLAG_IRAM);
    timer_start(SERVO_TG, SERVO_IDX);
    start_train();
}

bool penservo_move(uint16_t us) {
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    // The servo ISR runs on this core, so masking interrupts makes the
    // counter update atomic with respect to it.
    noInterrupts();
    bool travel = us != s_pulse_us;
    s_pulse_us = us;  // picked up by the next frame
    s_settle_frames = travel ? SETTLE_FRAMES + s_high : 0;
    s_hold_frames = HOLD_FRAMES;
    bool restart = !s_active;
    s_active = true;
    interrupts();
    if (restart) start_train();
    return travel;
}

bool penservo_arrived() { return s_settle_frames == 0; }
