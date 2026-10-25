#include "protocol.h"
#include "gcode.h"
#include "stepper.h"
#include "planner.h"
#include "settings.h"

static char line_buf[LINE_BUFFER_SIZE];
static uint8_t line_len = 0;
static bool line_overflow = false;

static void emit_ok()                      { Serial.print(F("ok\r\n")); }
static void emit_error(uint8_t code)       { Serial.print(F("error:")); Serial.print(code); Serial.print(F("\r\n")); }

static void emit_status_report() {
    float x, y;
    stepper_get_machine_position_mm(&x, &y);
    const char *state =
        stepper_is_in_hold() ? "Hold" :
        stepper_is_idle()    ? "Idle" : "Run";
    Serial.print('<');
    Serial.print(state);
    Serial.print(F("|MPos:"));
    Serial.print(x, 3);
    Serial.print(',');
    Serial.print(y, 3);
    Serial.print(F("|FS:"));
    // Approx current Cartesian feedrate in mm/min from the current block.
    plan_block_t *b = plan_get_current_block();
    uint32_t rate_events = stepper_current_rate_events_per_sec();
    float mm_min = 0.0f;
    if (b && b->step_event_count > 0 && rate_events > 0) {
        float mm_per_event = b->millimeters / (float)b->step_event_count;
        mm_min = (float)rate_events * mm_per_event * 60.0f;
    }
    Serial.print((uint32_t)mm_min);
    Serial.print(F(",0>\r\n"));
}

// After a reset the stepper keeps its true motor position, but plan_reset()
// zeroes the planner's step anchor. Re-anchor the planner (and the gcode
// parser, which reads its position from the planner) to where the machine
// physically is, so the next absolute move computes the correct delta instead
// of driving from a phantom 0,0.
static void sync_planner_to_machine() {
    float sx, sy;
    stepper_get_machine_position_mm(&sx, &sy);
    plan_set_position(sx, sy);
}

void protocol_init() {
    Serial.begin(SERIAL_BAUD);
    // No wait for USB here: on USB CDC there is often no host at all (mains
    // adapter), so blocking would hang a board on a charger.
    line_len = 0;
}

// Bytes read off the serial port while a line was blocked waiting for the
// planner (protocol_wait_for_planner_slot) — everything but the realtime
// commands, which were acted on immediately. Drained before the port itself so
// the stream stays in order. tune.py keeps at most 127 bytes in flight, so
// this only overflows on a misbehaving sender; then the port is simply left
// unread until there is room.
static uint8_t  pending[256];
static uint16_t pending_head = 0, pending_len = 0;
static bool     reset_pending = false;

static int next_byte() {
    if (pending_len > 0) {
        uint8_t c = pending[pending_head];
        pending_head = (uint16_t)((pending_head + 1) % sizeof(pending));
        pending_len--;
        return c;
    }
    return Serial.available() ? Serial.read() : -1;
}

static void soft_reset() {
    stepper_reset();
    plan_reset();
    gcode_reset();
    sync_planner_to_machine();
    line_len = 0;
    line_overflow = false;
    reset_pending = false;
}

bool protocol_reset_pending() { return reset_pending; }

bool protocol_wait_for_planner_slot() {
    while (plan_buffer_available() == 0) {
        while (!reset_pending && pending_len < sizeof(pending) && Serial.available()) {
            int c = Serial.read();
            if (c < 0) break;
            switch (c) {
                case RT_STATUS_REPORT: emit_status_report();   break;
                case RT_FEED_HOLD:     stepper_feed_hold();    break;
                case RT_CYCLE_START:   stepper_cycle_start();  break;
                case RT_SOFT_RESET:
                    // Everything before the reset is void; anything after it
                    // stays in the port for protocol_service to read.
                    reset_pending = true;
                    pending_len = 0;
                    break;
                default:
                    pending[(pending_head + pending_len) % sizeof(pending)] = (uint8_t)c;
                    pending_len++;
            }
        }
        if (reset_pending) return false;
        // A held stepper never drains the buffer, and a wait that outlasts the
        // hold would starve job_service() (so the web UI's stop never lands)
        // until the 30s watchdog reboots mid-print. Give up instead.
        if (stepper_is_in_hold()) return false;
        stepper_wake();
        stepper_service_sync();
        yield();  // service USB so the host doesn't reset us
    }
    return true;
}

void protocol_service() {
    // Let the stepper main-loop side handle any queued sync command.
    stepper_service_sync();
    // Lazy-flush any pending setting changes (only when idle & debounced).
    settings_service_save();
    // A reset can also land while a job line (not a serial one) was blocked.
    if (reset_pending) soft_reset();

    int c;
    while ((c = next_byte()) >= 0) {
        // Realtime commands — intercept before the line buffer.
        switch (c) {
            case RT_STATUS_REPORT:
                emit_status_report();
                continue;
            case RT_FEED_HOLD:
                stepper_feed_hold();
                continue;
            case RT_CYCLE_START:
                stepper_cycle_start();
                continue;
            case RT_SOFT_RESET:
                soft_reset();
                continue;
        }

        if (c == '\n' || c == '\r') {
            if (line_overflow) {
                emit_error((uint8_t)STATUS_OVERFLOW);
                line_len = 0;
                line_overflow = false;
                continue;
            }
            if (line_len == 0) continue;  // blank line
            line_buf[line_len] = '\0';
            StatusCode s = gcode_execute_line(line_buf);
            line_len = 0;
            // A reset that arrived while the line was blocked aborted it:
            // no ok/error for it, just the reset.
            if (reset_pending) { soft_reset(); continue; }
            if (s == STATUS_OK) emit_ok();
            else emit_error((uint8_t)s);
            continue;
        }

        if (line_overflow) continue;   // discard until newline

        if (line_len < LINE_BUFFER_SIZE - 1) {
            line_buf[line_len++] = (char)c;
        } else {
            line_overflow = true;
        }
    }
}
