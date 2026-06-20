#pragma once

#include <cstdint>

// Host stub for the Arduino Servo library. Records writeMicroseconds() calls
// so tests can verify pen up/down events.

class Servo {
public:
    void attach(int pin);
    void detach() { _attached = false; }
    bool attached() const { return _attached; }
    void writeMicroseconds(int us);
    int  last_us() const { return _last_us; }
    int  call_count() const { return _calls; }
private:
    int _pin = -1;
    int _last_us = 0;
    int _calls = 0;
    bool _attached = false;
};

// Global accessor — tests inspect the most recently attached servo.
Servo *host_last_servo();
