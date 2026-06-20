#include "config.h"
#include "protocol.h"
#include "planner.h"
#include "stepper.h"
#include "gcode.h"
#include "settings.h"

void setup() {
  // turn off these status LEDs since we already have a power LED
  pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, HIGH);
  pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, HIGH);
  pinMode(PIN_LED_B, OUTPUT); digitalWrite(PIN_LED_B, HIGH);

  settings_load();

  plan_init();
  stepper_init();
  gcode_reset();
  protocol_init();
}

void loop() {
  protocol_service();
}
