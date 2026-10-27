#include "steptimer.h"

#include "driver/timer.h"
#include "esp_attr.h"

// Timer group 0, timer 0, free-running at 1MHz (1 tick = 1µs) so a delay in
// microseconds can be added to the counter directly.
#define STEP_TG    TIMER_GROUP_0
#define STEP_IDX   TIMER_0
#define STEP_DIV   (TIMER_BASE_CLK / 1000000) // 80MHz APB / 80 = 1MHz

static bool (*s_handler)() = nullptr;

// IRAM_ATTR is the important part: without it this ISR lives in flash, and a
// flash cache miss (which any WiFi or LittleFS activity can cause) would
// stall step generation for tens of microseconds — visible as a stutter in
// the plot. ESP_INTR_FLAG_IRAM below keeps the whole path resident.
static bool IRAM_ATTR on_alarm(void *) {
    return s_handler && s_handler();  // true = woke a task: yield on return
}

void steptimer_init(bool (*handler)()) {
    s_handler = handler;

    timer_config_t cfg = {};
    cfg.alarm_en    = TIMER_ALARM_DIS;   // armed per-step by steptimer_arm()
    cfg.counter_en  = TIMER_PAUSE;
    cfg.intr_type   = TIMER_INTR_LEVEL;
    cfg.counter_dir = TIMER_COUNT_UP;
    cfg.auto_reload = TIMER_AUTORELOAD_DIS;
    cfg.divider     = STEP_DIV;

    timer_init(STEP_TG, STEP_IDX, &cfg);
    timer_set_counter_value(STEP_TG, STEP_IDX, 0);
    timer_isr_callback_add(STEP_TG, STEP_IDX, on_alarm, nullptr,
                           ESP_INTR_FLAG_IRAM);
    timer_start(STEP_TG, STEP_IDX);
}

void IRAM_ATTR steptimer_arm(uint32_t delay_us) {
    // The *_in_isr helpers are the IRAM-safe variants; the ordinary
    // timer_set_alarm_value() takes a lock and lives in flash, so calling it
    // from the step ISR would defeat the point.
    uint64_t now = timer_group_get_counter_value_in_isr(STEP_TG, STEP_IDX);
    timer_group_set_alarm_value_in_isr(STEP_TG, STEP_IDX, now + delay_us);
    timer_group_enable_alarm_in_isr(STEP_TG, STEP_IDX);
}

void steptimer_cancel() {
    timer_set_alarm(STEP_TG, STEP_IDX, TIMER_ALARM_DIS);
}
