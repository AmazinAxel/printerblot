#pragma once

#include <Arduino.h>
#include "config.h"

// Parse and execute a single G-code line. Returns STATUS_OK on success or a
// GRBL-compatible error code. The line must be null-terminated; the parser
// strips whitespace, comments, and case. Executes immediately: linear moves
// and arcs are queued into the planner, sync commands (servo/dwell) into
// the sync stream, and settings ($) are applied right away.
StatusCode gcode_execute_line(char *line);

// Reset modal state to power-on defaults. Called on `0x18` soft reset.
void gcode_reset();
