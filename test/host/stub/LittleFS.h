#pragma once
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

// Reads advance sim time and let due interrupts fire, modelling the step and
// servo ISRs preempting the loop mid-feed.
void sim_on_file_read(size_t bytes);
const char *sim_job_path();

class File {
public:
    File(FILE *f = nullptr) : f_(f) {}
    explicit operator bool() const { return f_ != nullptr; }
    size_t read(uint8_t *buf, size_t n) {
        if (!f_) return 0;
        size_t r = fread(buf, 1, n, f_);
        sim_on_file_read(r);
        return r;
    }
    int available() {
        if (!f_) return 0;
        long p = ftell(f_);
        return (int)(size() - (size_t)p);
    }
    size_t position() { return f_ ? (size_t)ftell(f_) : 0; }
    size_t size() {
        if (!f_) return 0;
        long p = ftell(f_);
        fseek(f_, 0, SEEK_END);
        long s = ftell(f_);
        fseek(f_, p, SEEK_SET);
        return (size_t)s;
    }
    void close() { if (f_) fclose(f_); f_ = nullptr; }
private:
    FILE *f_;
};

struct LittleFSClass {
    bool begin(bool) { return true; }
    File open(const char *, const char *) { return File(fopen(sim_job_path(), "rb")); }
};
inline LittleFSClass LittleFS;
