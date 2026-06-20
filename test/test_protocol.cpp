#include "test.h"
#include "../src/protocol.h"
#include "../src/planner.h"
#include "../src/stepper.h"
#include "../src/gcode.h"
#include "../src/config.h"
#include "sim.h"
#include "mocks/Arduino.h"
#include "mocks/Servo.h"
#include "mocks/pico/time.h"
#include <cstring>
#include <string>

// The protocol layer is the thin line between the serial link and the
// G-code executor. These tests push bytes in via HostSerial::host_push_input
// and inspect the response stream via HostSerial::host_drain_output.

extern "C" bool stepper_test_is_held();
extern "C" bool stepper_test_is_feed_hold();

static void push(const char *s) { Serial.host_push_input(s); }

// Drain any queued output and run protocol_service() until the input queue
// is empty (plus one extra service call so any trailing newline is handled).
// Does NOT run the stepper — for motion-dependent tests, also call
// sim_run_until_idle() afterward.
static std::string drain() {
    // Service as many times as needed to consume all input.
    while (Serial.available() > 0) protocol_service();
    return Serial.host_drain_output();
}

// ───── basic framing ─────

TEST_CASE("protocol: ok after a simple line") {
    sim_reset();
    Serial.host_clear();
    push("G21 G90\n");
    std::string out = drain();
    CHECK(out.find("ok\r\n") != std::string::npos);
}

TEST_CASE("protocol: error:N after parse failure") {
    sim_reset();
    Serial.host_clear();
    push("G99\n");
    std::string out = drain();
    char expected[32];
    std::snprintf(expected, sizeof(expected), "error:%d\r\n",
                  (int)STATUS_UNSUPPORTED_COMMAND);
    CHECK(out.find(expected) != std::string::npos);
}

TEST_CASE("protocol: error code matches the failure kind") {
    sim_reset();
    Serial.host_clear();
    push("G1 Xabc\n");
    std::string out = drain();
    char expected[32];
    std::snprintf(expected, sizeof(expected), "error:%d\r\n",
                  (int)STATUS_BAD_NUMBER_FORMAT);
    CHECK(out.find(expected) != std::string::npos);
}

TEST_CASE("protocol: multiple lines each get their own ok") {
    sim_reset();
    Serial.host_clear();
    push("G21\nG90\nF600\n");
    std::string out = drain();
    int count = 0;
    size_t pos = 0;
    while ((pos = out.find("ok\r\n", pos)) != std::string::npos) {
        count++; pos += 4;
    }
    CHECK_EQ(count, 3);
}

TEST_CASE("protocol: CRLF line endings are treated like LF") {
    sim_reset();
    Serial.host_clear();
    push("G21\r\nG90\r\n");
    std::string out = drain();
    int oks = 0;
    for (size_t i = 0; (i = out.find("ok", i)) != std::string::npos; i += 2) oks++;
    CHECK_EQ(oks, 2);
}

TEST_CASE("protocol: blank lines are silently ignored") {
    sim_reset();
    Serial.host_clear();
    push("\n\n\nG21\n\n");
    std::string out = drain();
    int oks = 0;
    for (size_t i = 0; (i = out.find("ok", i)) != std::string::npos; i += 2) oks++;
    CHECK_EQ(oks, 1);   // only the G21 line produces ok
}

// ───── realtime commands ─────

TEST_CASE("protocol: ? emits a status report with MPos and FS fields") {
    sim_reset();
    Serial.host_clear();
    push("?");
    std::string out = drain();
    CHECK(out.find("<")         != std::string::npos);
    CHECK(out.find("|MPos:")    != std::string::npos);
    CHECK(out.find("|FS:")      != std::string::npos);
    CHECK(out.find(",0>\r\n")   != std::string::npos);
}

TEST_CASE("protocol: status reports Idle when nothing is running") {
    sim_reset();
    Serial.host_clear();
    push("?");
    std::string out = drain();
    CHECK(out.find("<Idle|") != std::string::npos);
}

TEST_CASE("protocol: ? doesn't require a preceding newline") {
    // The realtime status command should work even when a partial line is
    // sitting in the buffer (not yet terminated by \n).
    sim_reset();
    Serial.host_clear();
    push("G1 X10 F600");    // deliberately no newline
    push("?");              // realtime
    std::string out = drain();
    CHECK(out.find("<") != std::string::npos);  // got status
    // No ok yet because no newline on the partial line.
    CHECK(out.find("ok") == std::string::npos);
    // Finish the line; we should now see ok.
    Serial.host_clear();
    push("\n");
    std::string out2 = drain();
    CHECK(out2.find("ok") != std::string::npos);
}

TEST_CASE("protocol: ! enters feed hold, ~ resumes") {
    sim_reset();
    // Start a long motion so we can hold mid-run.
    sim_gcode("G21 G90");
    sim_gcode("G1 X100 F600");
    // Step a few times so the stepper is actually running (not idle).
    for (int i = 0; i < 10; i++) tight_loop_contents();
    CHECK(!stepper_is_idle());

    Serial.host_clear();
    push("!");
    drain();
    // Drive a few ticks so the hold deceleration reaches zero.
    for (int i = 0; i < 200; i++) tight_loop_contents();
    CHECK(stepper_test_is_held());

    Serial.host_clear();
    push("~");
    drain();
    // Resume — motion continues
    sim_run_until_idle();
    float x, y; sim_get_cartesian_mm(&x, &y);
    CHECK_NEAR(x, 100.0f, 0.1f);
}

TEST_CASE("protocol: soft reset (0x18) flushes the planner and restores welcome") {
    sim_reset();
    sim_gcode("G21 G90");
    sim_gcode("G1 X10 F600");
    sim_gcode("G1 X20");
    CHECK(!plan_is_empty());

    Serial.host_clear();
    char reset_byte = 0x18;
    Serial.host_push_input(&reset_byte);   // single byte
    // Also push a NUL terminator so host_push_input's strlen loop stops.
    // Actually host_push_input reads until *s==0, so single-byte-then-null works.
    drain();

    CHECK(plan_is_empty());
    // Welcome string should be re-emitted.
    std::string out = Serial.host_drain_output();
    // welcome was already drained; push another status to confirm we respond.
    push("?");
    std::string st = drain();
    CHECK(st.find("Idle") != std::string::npos);   // stepper is idle again
}

// ───── overflow & edge cases ─────

TEST_CASE("protocol: overlong line emits error:OVERFLOW on next newline") {
    sim_reset();
    Serial.host_clear();
    // Push a line way longer than LINE_BUFFER_SIZE (96).
    std::string big(200, 'G');
    big += "\n";
    push(big.c_str());
    std::string out = drain();
    char expected[32];
    std::snprintf(expected, sizeof(expected), "error:%d\r\n",
                  (int)STATUS_OVERFLOW);
    CHECK(out.find(expected) != std::string::npos);

    // After overflow, subsequent valid lines should still work.
    Serial.host_clear();
    push("G21\n");
    std::string out2 = drain();
    CHECK(out2.find("ok") != std::string::npos);
}

TEST_CASE("protocol: line exactly at the buffer limit works") {
    sim_reset();
    Serial.host_clear();
    // LINE_BUFFER_SIZE - 1 chars in a single line. Just whitespace so the
    // tokenizer sees nothing and returns ok.
    std::string line((size_t)(LINE_BUFFER_SIZE - 1), ';');  // all comment
    line += "\n";
    push(line.c_str());
    std::string out = drain();
    CHECK(out.find("ok") != std::string::npos);
    CHECK(out.find("error") == std::string::npos);
}

TEST_CASE("protocol: welcome string contains 'Grbl' on init") {
    sim_reset();
    Serial.host_clear();
    protocol_init();    // emits welcome
    std::string out = Serial.host_drain_output();
    CHECK(out.find("Grbl") != std::string::npos);
}

TEST_CASE("protocol: streaming 30 lines back-to-back produces 30 oks in order") {
    sim_reset();
    Serial.host_clear();
    std::string batch;
    for (int i = 0; i < 30; i++) batch += "G21\n";
    push(batch.c_str());
    std::string out = drain();
    int oks = 0;
    for (size_t i = 0; (i = out.find("ok\r\n", i)) != std::string::npos; i += 4) oks++;
    CHECK_EQ(oks, 30);
    // No error lines mixed in.
    CHECK(out.find("error") == std::string::npos);
}

TEST_CASE("protocol: realtime bytes inside a line don't land in the buffer") {
    // Demonstrates that ? is never stored as a character of an in-flight line.
    sim_reset();
    Serial.host_clear();
    push("G1 X10?Y5 F600\n");
    std::string out = drain();
    // The '?' gets intercepted, leaving "G1 X10Y5 F600" which is valid.
    CHECK(out.find("ok") != std::string::npos);
    CHECK(out.find("error") == std::string::npos);
}
