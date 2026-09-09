#pragma once

#include <stdint.h>

// Step-timer abstraction.
//
// The stepper ISR is scheduled by a one-shot hardware timer that we re-arm
// from inside the ISR itself, so the period *is* the velocity profile.
// Backed by timer group 0 / timer 0 at 1MHz, with the handler placed in IRAM
// so the WiFi stack on the other core can never stall a step.
//
// Keeping this behind three functions leaves stepper_isr_body() — the
// Bresenham and velocity math that decides print quality — free of any
// peripheral detail.

// Set up the timer and install `handler` as the fire callback. The handler
// runs in interrupt context.
void steptimer_init(void (*handler)());

// Fire the handler `delay_us` microseconds from now. Called from inside the
// handler to schedule the next step event.
void steptimer_arm(uint32_t delay_us);

// Cancel any pending fire. Used by stepper_reset().
void steptimer_cancel();
