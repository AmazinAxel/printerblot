#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

// Host stub for the arduino-pico EEPROM library. Stores bytes in a RAM
// buffer sized by begin(). Tests can inspect and reset via host_eeprom_reset().

class HostEEPROM {
public:
    void begin(size_t size) {
        if (_data.size() < size) _data.resize(size, 0xFF);
    }
    bool commit() { return true; }
    void end()    {}

    template <typename T>
    T &get(int addr, T &t) const {
        if ((size_t)addr + sizeof(T) <= _data.size())
            std::memcpy(&t, _data.data() + addr, sizeof(T));
        return t;
    }

    template <typename T>
    const T &put(int addr, const T &t) {
        if ((size_t)addr + sizeof(T) > _data.size())
            _data.resize((size_t)addr + sizeof(T), 0xFF);
        std::memcpy(_data.data() + addr, &t, sizeof(T));
        return t;
    }

    uint8_t read(int addr) const {
        return ((size_t)addr < _data.size()) ? _data[addr] : 0xFF;
    }
    void write(int addr, uint8_t v) {
        if ((size_t)addr < _data.size()) _data[addr] = v;
    }

    // Test-only helpers.
    void host_reset() { _data.assign(_data.size(), 0xFF); }

private:
    std::vector<uint8_t> _data;
};
extern HostEEPROM EEPROM;
void host_eeprom_reset();
