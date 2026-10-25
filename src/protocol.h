#pragma once

#include "config.h"

void protocol_init();

// Called from the main loop. Reads serial bytes, handles realtime commands
// inline, buffers non-realtime bytes into a line, and dispatches complete
// lines to the G-code executor. Emits `ok` / `error:N` responses.
void protocol_service();

// Block until the planner has a free slot. Every spin loop that waits on the
// planner must use this: it services sync blocks (otherwise a pen command at
// the head of a full planner deadlocks) and keeps realtime commands live, so
// `?`, `!` and `~` act immediately instead of after the whole line is queued.
// Returns false if a soft reset arrived (protocol_reset_pending() is then
// true; the caller must stop queueing and return, and protocol_service()
// performs the reset) or if the stepper is in feed hold (the caller reports
// the line as failed rather than spin until the watchdog fires).
bool protocol_wait_for_planner_slot();
bool protocol_reset_pending();
