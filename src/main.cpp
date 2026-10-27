#include "config.h"
#include "planner.h"
#include "stepper.h"
#include "gcode.h"
#include "settings.h"
#include "webui.h"
#include "job.h"
#include "wake.h"

#include "esp_log.h"

void setup() {
  // Silence ESP-IDF's own logging (errors-only by default). Its console is
  // UART0, whose TX pad is the pen servo's pin, and nothing reads it anyway.
  esp_log_level_set("*", ESP_LOG_NONE);

  // The user LED stays off for good. It's active LOW, so it's driven HIGH
  // rather than left floating, where it can glow faintly. Level first, then
  // output, so it never flashes on in between.
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);

  wake_init();  // before anything that can wake the loop
  settings_load();
  plan_reset();
  stepper_init();
  gcode_reset();

  // WiFi + HTTP run on core 0; Arduino's loop() (and so the gcode parser)
  // stays on core 1 alongside the step timer ISR.
  job_init();
  webui_init();
}

// Event-driven: sleep until woken (see wake.h), then let each part act on
// whatever changed. No polling, no timeouts — idle, the core does nothing.
//
// There is deliberately no task watchdog on this loop: it sleeps
// indefinitely by design, so feeding one would need a periodic timer.
void loop() {
  wake_wait();
  stepper_service();  // pen blocks: move the pen, release when arrived
  job_service();      // web requests, feeding the planner, job end, idle
}
