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
    // Standard GRBL reports MPos as X,Y,Z. We're 2-axis (Z is servo), but
    // senders like UGS parse 3 comma-separated values and fall back to
    // 0,0,0 if only 2 are present. Always emit Z=0.000 for compatibility.
    Serial.print(F(",0.000"));
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

// GRBL welcome — exact format senders expect. Build suffix goes in $I.
static void emit_welcome() {
    Serial.print(F("\r\nGrbl 1.1f ['$' for help]\r\n"));
}

void protocol_init() {
    Serial.begin(SERIAL_BAUD);
    uint32_t deadline = millis() + 800;
    while (!Serial && (int32_t)(millis() - deadline) < 0) { /* wait for USB */ }
    line_len = 0;
    emit_welcome();
}

void protocol_service() {
    // Re-emit the welcome banner when the host opens the USB CDC port so
    // senders connecting after boot can detect our GRBL version. Rate-
    // limited because `(bool)Serial` can spuriously flap during flash
    // erases (USB CDC pauses while interrupts are off), and a stray
    // welcome mid-session looks like a reconnect to UGS → desync.
    static bool was_connected = false;
    static uint32_t last_welcome_ms = 0;
    bool now_connected = (bool)Serial;
    if (now_connected && !was_connected) {
        uint32_t now = millis();
        if (now - last_welcome_ms > 2000) {
            emit_welcome();
            last_welcome_ms = now;
        }
    }
    was_connected = now_connected;

    // Let the stepper main-loop side handle any queued sync command.
    stepper_service_sync();
    // Lazy-flush any pending setting changes (only when idle & debounced).
    settings_service_save();

    while (Serial.available()) {
        int c = Serial.read();
        if (c < 0) break;

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
                stepper_reset();
                plan_reset();
                gcode_reset();
                line_len = 0;
                line_overflow = false;
                emit_welcome();
                continue;
            case RT_JOG_CANCEL:
                // Discard any buffered jog motion immediately. Simpler than
                // GRBL's "decel and flush"; a pen plotter can stop abruptly.
                stepper_reset();
                plan_reset();
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
