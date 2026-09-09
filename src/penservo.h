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
// timer callbacks, and the jitter tolerance is milliseconds, so this is well
// within reach even alongside the step ISR.

void penservo_init(uint8_t pin, uint16_t initial_us = 0);

// Command a pulse width in microseconds (1000 = one end, 2000 = the other).
void penservo_write_us(uint16_t us);

uint16_t penservo_current_us();

// Called from loop(). Stops the pulse train once the pen has had time to
// reach its commanded position.
//
// A hobby servo holds position only while it's being pulsed, and holding is
// its worst case: the motor stays powered and hunts around the setpoint,
// buzzing and wearing the gear train and feedback pot. The pen has no load
// to hold — it rests either up or down — so once it has arrived there's
// nothing to gain from continuing to drive it. The next write_us() restarts
// the train automatically.
void penservo_service();
