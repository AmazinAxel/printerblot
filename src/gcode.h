#pragma once

#include <Arduino.h>
#include "config.h"

// Parse and execute a single G-code line (null-terminated, modified in
// place: whitespace, comments and case are stripped). Moves and pen commands
// are queued into the planner. The caller must have MAX_BLOCKS_PER_LINE free
// planner slots.
StatusCode gcode_execute_line(char *line);

// Reset modal state (G0/G1, feedrate, G92 origin) to power-on defaults.
void gcode_reset();
