#pragma once
#include <stdint.h>

// Legacy timer-group driver, modelled at 1 tick = 1µs of sim time.
typedef int timer_group_t;
typedef int timer_idx_t;
enum { TIMER_GROUP_0 = 0, TIMER_GROUP_1 = 1 };
enum { TIMER_0 = 0 };
enum { TIMER_ALARM_DIS = 0, TIMER_ALARM_EN = 1 };
enum { TIMER_PAUSE = 0, TIMER_INTR_LEVEL = 0, TIMER_COUNT_UP = 1,
       TIMER_AUTORELOAD_DIS = 0 };
#define TIMER_BASE_CLK 80000000
#define ESP_INTR_FLAG_IRAM 0

struct timer_config_t {
    int alarm_en, counter_en, intr_type, counter_dir, auto_reload;
    uint32_t divider;
};
typedef bool (*timer_isr_t)(void *);

int timer_init(timer_group_t, timer_idx_t, const timer_config_t *);
int timer_set_counter_value(timer_group_t, timer_idx_t, uint64_t);
int timer_isr_callback_add(timer_group_t, timer_idx_t, timer_isr_t, void *, int);
int timer_start(timer_group_t, timer_idx_t);
int timer_set_alarm_value(timer_group_t, timer_idx_t, uint64_t);
int timer_set_alarm(timer_group_t, timer_idx_t, int);
int timer_get_counter_value(timer_group_t, timer_idx_t, uint64_t *);
uint64_t timer_group_get_counter_value_in_isr(timer_group_t, timer_idx_t);
void timer_group_set_alarm_value_in_isr(timer_group_t, timer_idx_t, uint64_t);
void timer_group_enable_alarm_in_isr(timer_group_t, timer_idx_t);
