#include "config.h"
#include "protocol.h"
#include "planner.h"
#include "stepper.h"
#include "gcode.h"
#include "penservo.h"
#include "settings.h"
#include "webui.h"
#include "job.h"

#include "esp_task_wdt.h"

void setup() {
  // Heartbeat on the user LED. USB CDC only enumerates once setup() finishes,
  // so if setup() crashes you get no serial output at all and no way to tell
  // a hung board from a flashing problem. The LED is independent of that.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // XIAO user LED is active LOW: on = booting

  settings_load();
  plan_init();
  stepper_init();
  gcode_reset();
  protocol_init();

  // WiFi + HTTP run on core 0; Arduino's loop() (and so the gcode parser)
  // stays on core 1 alongside the step timer ISR.
  job_init();
  webui_init();

  // Watch loop() so a wedged firmware reboots itself instead of sitting dead
  // until someone power-cycles it — this thing is meant to be left on.
  // 30s, not the 5s default: a G4 dwell or a settings flash commit can
  // legitimately block the loop for a while, and a spurious reboot mid-print
  // is worse than a slow recovery from a real hang.
  esp_task_wdt_init(30, true);   // true = panic (reboot) on timeout
  esp_task_wdt_add(NULL);        // subscribe the Arduino loop task

  // setup() completed: user LED off (it's active LOW). If the LED stays lit
  // after power-up, something above hung — worth keeping, since USB CDC only
  // enumerates once setup() returns and a hang is otherwise silent.
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  protocol_service();
  // Stop driving the pen servo once it has arrived; see penservo.h.
  penservo_service();
  // Feed queued gcode lines from the active job into the parser.
  job_service();
  // Finish WiFi association / report the URL once it's up.
  webui_service();

  // When there's nothing to do, hand the core back to FreeRTOS so it can
  // enter its idle state instead of spinning at 240MHz. Strictly gated on
  // being idle: job_service() keeps the planner topped up during a print and
  // any delay here would eat into that.
  if (job_state() == JOB_IDLE && stepper_is_idle()) {
    delay(2);
  }

  esp_task_wdt_reset(); // we got all the way round; firmware is alive
}
