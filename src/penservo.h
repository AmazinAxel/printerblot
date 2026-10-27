#pragma once

#include <stdint.h>

// Pen servo driver.
//
// The pen servo sits on D6 = GPIO43, which is hardwired to UART0. The LEDC
// PWM matrix cannot route there (ESP32Servo's own pin table allows only
// 1-21, 35-45, 47, 48 — 43 and 44 are excluded), so the usual Servo library
// hangs on attach(). That's why this driver exists instead of a library.
//
// The Blot carrier board can't be rewired, so instead of PWM we generate the
// 50Hz servo pulse from a hardware timer and a plain GPIO write, which GPIO43
// does support. A servo only needs a 1000-2000µs pulse every 20ms; that's two
// timer interrupts per frame, and the jitter tolerance is milliseconds.
//
// Everything time-based is counted in those 20ms frames by the servo's own
// interrupt — no software timers, nothing for loop() to poll:
//   - travel: DEFAULT_PEN_MOVE_MS after a new target, the ISR flags the pen
//     as arrived and wakes the loop (see wake.h);
//   - hold: a hobby servo only holds while pulsed, and holding is its worst
//     case (powered, hunting around the setpoint, wearing the gears). The pen
//     has no load to hold, so after SERVO_HOLD_MS the ISR simply stops
//     re-arming itself and leaves the line low. The next move restarts it.

void penservo_init(uint8_t pin, uint16_t initial_us);

// Command a pulse width in µs (1000 = one end, 2000 = the other). Returns
// true if the target changed, i.e. the pen has to travel: the loop is woken
// once it has arrived (penservo_arrived()). False means it's already there.
bool penservo_move(uint16_t us);

bool penservo_arrived();
