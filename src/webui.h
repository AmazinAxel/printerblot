#pragma once

// WiFi + HTTP server. Serves the UI and accepts the gcode the browser
// produces from a PDF.
//
// The HTTP handlers run on core 0 but never touch the parser directly — they
// call into job.h, which defers the work to core 1. See job.h.

void webui_init();

// Called from loop(). Finishes the WiFi association that webui_init() starts,
// so a slow or absent router never blocks boot, and reports the URL once it
// resolves.
void webui_service();
