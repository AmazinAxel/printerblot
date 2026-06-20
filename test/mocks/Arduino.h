// Host-side stubs for the Arduino API. Provides just enough surface for
// the firmware source files to compile and run in tests. Hardware-side
// effects (digitalWrite, delay) are recorded so tests can inspect them.

#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ──────── Pin constants ────────
#define D1  1
#define D6  6
#define D7  7
#define D8  8
#define D9  9
#define D10 10
#define D11 11
#define D13 13
#define PIN_LED 25

#define HIGH   1
#define LOW    0
#define INPUT  0
#define OUTPUT 1

// ──────── Pin state recording ────────
void host_reset_pin_state();
int  host_pin_state(int pin);
// Count positive-going transitions on a pin. Useful for step-pulse tallying.
uint32_t host_pin_rising_edges(int pin);
// Record all pin-state changes (pin, new_state, sim_time_us) for trace tests.
struct PinEvent { int pin; int value; uint32_t time_us; };
const std::vector<PinEvent> *host_pin_event_log();  // defined in mock_impl.cpp

// ──────── Arduino API ────────
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int  digitalRead(int pin);

void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield();
unsigned long millis();
unsigned long micros();

// Advance the simulated clock. Tests drive this to unblock the stepper.
void host_advance_time_us(uint32_t us);
void host_set_time_us(uint32_t us);

// ──────── Serial (Stream-like) ────────
class HostSerial {
public:
    void begin(unsigned long baud);
    operator bool() const { return true; }
    int  available();
    int  read();
    size_t write(uint8_t b);

    void print(const char *s);
    void print(char c);
    void print(int v);
    void print(unsigned int v);
    void print(long v);
    void print(unsigned long v);
    void print(float v, int digits = 2);
    void print(double v, int digits = 2) { print((float)v, digits); }

    void println(const char *s) { print(s); print("\r\n"); }
    void println(int v)         { print(v); print("\r\n"); }
    void println(unsigned int v){ print(v); print("\r\n"); }
    void println(long v)        { print(v); print("\r\n"); }
    void println(unsigned long v){ print(v); print("\r\n"); }
    void println(float v, int d = 2) { print(v, d); print("\r\n"); }
    void println(double v, int d = 2){ print(v, d); print("\r\n"); }
    void println()              { print("\r\n"); }

    // Inject bytes into the read queue (simulates host-sent GCODE).
    void host_push_input(const char *s);
    // Drain output the firmware has written.
    std::string host_drain_output();
    void host_clear();
};
extern HostSerial Serial;

// ──────── Progmem-string passthrough ────────
#define F(x) x

// ──────── String fallback (used by legacy code; not in our new firmware) ────────
// Not defined — new firmware uses plain C strings.

// ──────── Interrupt guards ────────
inline void noInterrupts() {}
inline void interrupts()   {}
