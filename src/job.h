#pragma once

#include <stdint.h>

// Job runner: streams the stored gcode file into the gcode parser.
//
// The web server and the parser live on the same chip, so there's no serial
// protocol in between — job_service() hands lines straight to
// gcode_execute_line().

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

// Called from loop(). Tops the planner up to within PLANNER_HEADROOM slots
// of full, so lookahead always has blocks to plan a corner against.
void job_service();

// Controls. Safe to call from the HTTP task on the other core: these only
// set a flag, which job_service() applies on the core that owns the parser.
bool job_start();
void job_pause();   // pause at the next line boundary, lifting the pen
void job_resume();
void job_stop();    // abort, lift pen, return to job origin

JobState job_state();
uint32_t job_lines_done();
uint32_t job_lines_total();

// True if the last job stopped on an error (e.g. a move outside the work
// envelope) rather than running to completion. Cleared when a job starts.
bool job_had_error();

// Print quality preset ("draft" or "poster"), persisted in settings.
void job_set_quality(const char *q);
const char *job_quality();

// Called by the upload handler once a new gcode file has landed.
void job_notify_new_file();
