#pragma once

// WiFi + HTTP server. Serves the UI and accepts the gcode the browser
// produces from a PDF.
//
// The HTTP handlers run on core 0 but never touch the parser directly — they
// call into job.h, which defers the work to core 1. See job.h.

// Starts WiFi (non-blocking: a slow or absent router never delays boot),
// mDNS and the HTTP server. Everything after that is event-driven.
void webui_init();
