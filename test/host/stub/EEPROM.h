#pragma once
#include <string.h>
#include <stdint.h>

// Flash-backed EEPROM emulation; commits are counted (flash wear).
struct EEPROMClass {
    uint8_t mem[256];
    uint8_t flash[256];
    int commits = 0;
    EEPROMClass() { memset(flash, 0xFF, sizeof(flash)); }
    void begin(size_t) { memcpy(mem, flash, sizeof(mem)); }
    template <class T> void get(int a, T &t) { memcpy(&t, mem + a, sizeof(T)); }
    template <class T> void put(int a, const T &t) { memcpy(mem + a, &t, sizeof(T)); }
    bool commit() { memcpy(flash, mem, sizeof(flash)); commits++; return true; }
};
inline EEPROMClass EEPROM;
