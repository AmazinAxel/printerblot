#pragma once

#include "config.h"

void protocol_init();

// Called from the main loop. Reads serial bytes, handles realtime commands
// inline, buffers non-realtime bytes into a line, and dispatches complete
// lines to the G-code executor. Emits `ok` / `error:N` responses.
void protocol_service();
