#include "wake.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static TaskHandle_t loop_task;

void wake_init() { loop_task = xTaskGetCurrentTaskHandle(); }

void wake_wait() { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); }

void wake_loop() { xTaskNotifyGive(loop_task); }

bool IRAM_ATTR wake_loop_from_isr() {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(loop_task, &woken);
    return woken == pdTRUE;
}
