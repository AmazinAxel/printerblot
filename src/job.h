#pragma once

#include <stdint.h>

// Job runner: streams the stored gcode file into the gcode parser. This is
// the only way anything gets printed — there is no USB or serial input.

// LittleFS path the upload handler writes and the runner reads.
extern const char *JOB_PATH;

enum JobState : uint8_t {
    JOB_IDLE,
    JOB_RUNNING,
    JOB_PAUSED,
};

void job_init();

// Called by job.cpp whenever the state it reports changes (start, pause,
// resume, finish, progress). webui.cpp installs a callback that pushes the
// new state to connected browsers over SSE — without this the UI only
// updates on the request that caused the change, so a job starting or
// finishing on its own is never reflected.
void job_set_on_change(void (*cb)());

// Called from loop() on every wake. Applies web requests, keeps the planner
// topped up (so lookahead always has blocks to plan a corner against),
// finishes jobs, and releases the motors once idle.
void job_service();

// Controls. Safe to call from the HTTP task on the other core: these only
// set a flag and wake the loop, which applies it on the parser's core.
bool job_start();
void job_pause();   // pause at the next line boundary, lifting the pen
void job_resume();
void job_stop();    // abort, lift pen, return to job origin

JobState job_state();
uint8_t  job_progress();   // 0-100: share of the file fed to the parser

// True if the last job stopped on an error (e.g. a move outside the work
// envelope) rather than running to completion. Cleared when a job starts.
bool job_had_error();

// Print quality preset ("draft" or "poster"), persisted in settings. Safe
// from the HTTP task like the controls above; false if unknown or not idle.
bool job_set_quality(const char *q);
const char *job_quality();
