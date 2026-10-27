#pragma once

#include "esp_attr.h"

// The firmware is event-driven: loop() sleeps in wake_wait() until a
// producer that changed something the loop must act on wakes it —
//
//   step ISR   a planner slot freed, a pen block reached, motion stopped
//   servo ISR  the pen has arrived
//   HTTP task  print / pause / resume / stop / quality requests
//
// Wakes are counted (a FreeRTOS task notification), so one that lands while
// the loop is busy isn't lost, and several coalesce into one pass.

void wake_init();        // from setup(), first thing: the loop task waits
void wake_wait();        // loop(): sleep until woken
void wake_loop();        // from another task
// From an IRAM interrupt handler. Returns true if a context switch is due,
// which the timer driver performs when the handler returns it.
bool IRAM_ATTR wake_loop_from_isr();
