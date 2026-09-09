#include "penservo.h"

#include <Arduino.h>

#include "config.h"

#include "driver/timer.h"
#include "esp_attr.h"
#include "soc/gpio_struct.h"

// Timer group 1 / timer 0. The stepper owns group 0 / timer 0, so these share
// no registers and the servo can never disturb step timing.
#define SERVO_TG   TIMER_GROUP_1
#define SERVO_IDX  TIMER_0
#define SERVO_DIV  (TIMER_BASE_CLK / 1000000) // 1 tick = 1µs

#define SERVO_FRAME_US 20000  // 50Hz refresh, standard for hobby servos

static uint8_t  s_pin = 0;
static volatile uint16_t s_pulse_us = 1000;
static volatile bool     s_high = false;

// Pin writes go through isr_pin_write() from config.h. gpio_set_level() lives
// in flash, so calling it from an ESP_INTR_FLAG_IRAM handler panics with
// "Cache disabled but cached memory region accessed" the moment anything
// writes to flash.
#define pin_write isr_pin_write

// Emits the pulse: go high, schedule the fall; go low, schedule the next
// frame. Two interrupts per 20ms frame — negligible next to the step ISR.
static bool IRAM_ATTR servo_isr(void *) {
    uint32_t next;
    if (s_high) {
        // End of the pulse: drop the line, wait out the rest of the frame.
        pin_write(s_pin, false);
        s_high = false;
        next = SERVO_FRAME_US - s_pulse_us;
    } else {
        pin_write(s_pin, true);
        s_high = true;
        next = s_pulse_us;
    }

    uint64_t now = timer_group_get_counter_value_in_isr(SERVO_TG, SERVO_IDX);
    timer_group_set_alarm_value_in_isr(SERVO_TG, SERVO_IDX, now + next);
    timer_group_enable_alarm_in_isr(SERVO_TG, SERVO_IDX);
    return false;
}

void penservo_init(uint8_t pin, uint16_t initial_us) {
    s_pin = pin;
    // Set the target BEFORE the first frame goes out, so the servo's very
    // first pulse already commands the resting (pen-up) position. Otherwise
    // it would see the default and snap there, then move again — a visible
    // double twitch at power-up.
    if (initial_us) penservo_write_us(initial_us);

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

    // Kick off the first frame.
    timer_set_alarm_value(SERVO_TG, SERVO_IDX, SERVO_FRAME_US);
    timer_set_alarm(SERVO_TG, SERVO_IDX, TIMER_ALARM_EN);
}

// How long to keep pulsing after a move before going quiet. Must comfortably
// exceed the servo's travel time; DEFAULT_PEN_MOVE_MS (60ms) is the figure the
// motion code already waits, so allow a healthy margin over it.
#define SERVO_HOLD_MS 400

static volatile uint32_t s_last_cmd_ms = 0;
static volatile bool     s_idle = false;

void penservo_write_us(uint16_t us) {
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    s_pulse_us = us; // picked up by the next frame
    s_last_cmd_ms = millis();

    if (s_idle) {
        // Restart the pulse train: re-arm the timer and resume framing.
        s_idle = false;
        s_high = false;
        uint64_t now = 0;
        timer_get_counter_value(SERVO_TG, SERVO_IDX, &now);
        timer_set_alarm_value(SERVO_TG, SERVO_IDX, now + 100);
        timer_set_alarm(SERVO_TG, SERVO_IDX, TIMER_ALARM_EN);
    }
}

void penservo_service() {
    if (s_idle) return;
    if ((int32_t)(millis() - s_last_cmd_ms) < SERVO_HOLD_MS) return;

    // Pen has arrived. Stop pulsing and leave the line low so the servo
    // de-energises instead of hunting against its setpoint indefinitely.
    s_idle = true;
    timer_set_alarm(SERVO_TG, SERVO_IDX, TIMER_ALARM_DIS);
    pin_write(s_pin, false);
}

uint16_t penservo_current_us() { return s_pulse_us; }
