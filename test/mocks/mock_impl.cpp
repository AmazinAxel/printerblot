// Implementations for mocked Arduino/Pico SDK symbols and shared test state.

#include "Arduino.h"
#include "Servo.h"
#include "hardware/timer.h"

#include <vector>
#include <string>
#include <deque>

// ──────── Timer peripheral ────────
static timer_hw_t _timer_state = {};
timer_hw_t *const timer_hw = &_timer_state;

// ──────── Pin state ────────
static int      _pin_state[64]     = {};
static uint32_t _pin_rise_count[64]= {};
static std::vector<PinEvent> _pin_log;
static uint32_t _sim_time_us = 0;

void host_reset_pin_state() {
    for (int i = 0; i < 64; i++) { _pin_state[i] = 0; _pin_rise_count[i] = 0; }
    _pin_log.clear();
    _timer_state = {};
    _sim_time_us = 0;
}

int host_pin_state(int pin) {
    if (pin < 0 || pin >= 64) return 0;
    return _pin_state[pin];
}
uint32_t host_pin_rising_edges(int pin) {
    if (pin < 0 || pin >= 64) return 0;
    return _pin_rise_count[pin];
}
const std::vector<PinEvent> *host_pin_event_log() { return &_pin_log; }

void pinMode(int, int) {}
void digitalWrite(int pin, int value) {
    if (pin < 0 || pin >= 64) return;
    int prev = _pin_state[pin];
    _pin_state[pin] = value;
    if (value == HIGH && prev == LOW) _pin_rise_count[pin]++;
    _pin_log.push_back({pin, value, _sim_time_us});
}
int digitalRead(int pin) { return (pin>=0 && pin<64) ? _pin_state[pin] : 0; }

// ──────── Time ────────
void delay(unsigned long ms) { _sim_time_us += (uint32_t)(ms * 1000); _timer_state.timerawl = _sim_time_us; }
void delayMicroseconds(unsigned int us) { _sim_time_us += us; _timer_state.timerawl = _sim_time_us; }
// yield() defined in sim.cpp — advances simulated stepper like tight_loop_contents
unsigned long millis() { return _sim_time_us / 1000; }
unsigned long micros() { return _sim_time_us; }

void host_advance_time_us(uint32_t us) { _sim_time_us += us; _timer_state.timerawl = _sim_time_us; }
void host_set_time_us(uint32_t us)     { _sim_time_us  = us; _timer_state.timerawl = _sim_time_us; }

// ──────── Serial ────────
HostSerial Serial;

static std::deque<char> _serial_in;
static std::string      _serial_out;

void HostSerial::begin(unsigned long) {}
int  HostSerial::available() { return (int)_serial_in.size(); }
int  HostSerial::read() {
    if (_serial_in.empty()) return -1;
    int c = (unsigned char)_serial_in.front();
    _serial_in.pop_front();
    return c;
}
size_t HostSerial::write(uint8_t b) { _serial_out.push_back((char)b); return 1; }

void HostSerial::print(const char *s)  { if (s) _serial_out.append(s); }
void HostSerial::print(char c)         { _serial_out.push_back(c); }
void HostSerial::print(int v)          { char b[32]; std::snprintf(b,sizeof(b),"%d",v);  _serial_out.append(b); }
void HostSerial::print(unsigned int v) { char b[32]; std::snprintf(b,sizeof(b),"%u",v);  _serial_out.append(b); }
void HostSerial::print(long v)         { char b[32]; std::snprintf(b,sizeof(b),"%ld",v); _serial_out.append(b); }
void HostSerial::print(unsigned long v){ char b[32]; std::snprintf(b,sizeof(b),"%lu",v); _serial_out.append(b); }
void HostSerial::print(float v, int d) {
    char fmt[16]; std::snprintf(fmt,sizeof(fmt),"%%.%df", d);
    char b[40];  std::snprintf(b,sizeof(b),fmt,(double)v);
    _serial_out.append(b);
}

void HostSerial::host_push_input(const char *s)   { while (s && *s) _serial_in.push_back(*s++); }
std::string HostSerial::host_drain_output()       { std::string out = _serial_out; _serial_out.clear(); return out; }
void HostSerial::host_clear()                     { _serial_in.clear(); _serial_out.clear(); }

// ──────── Servo ────────
static Servo *_last_servo = nullptr;
void Servo::attach(int pin) { _pin = pin; _attached = true; _last_servo = this; }
void Servo::writeMicroseconds(int us) { _last_us = us; _calls++; }
Servo *host_last_servo() { return _last_servo; }

// ──────── EEPROM ────────
#include "EEPROM.h"
HostEEPROM EEPROM;
void host_eeprom_reset() { EEPROM.host_reset(); }
