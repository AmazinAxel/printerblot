#include "webui.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

#include "config.h"
#include "job.h"

// Hard error rather than a warning: without credentials the board has no
// network at all, which looks like a WiFi bug later.
#if !__has_include("wifi_config.h")
  #error "src/wifi_config.h missing. Run: cp src/wifi_config.h.example src/wifi_config.h"
#endif
#include "wifi_config.h"

// Radio transmit power. The default is the 19.5dBm maximum, but a plotter
// sits near its router and the PA is the biggest heat source on the board
// while transmitting. Override in wifi_config.h if the link is marginal.
#ifndef WIFI_TX_POWER
#define WIFI_TX_POWER WIFI_POWER_11dBm
#endif

static AsyncWebServer server(80);
static AsyncEventSource events("/events");

// Upload target. Written in chunks so a large gcode file never has to fit
// in RAM at once — the S3 has 8MB of flash but only ~320KB of usable heap.
static File upload_file;
// Latched when any part of an upload fails, so the completion handler knows
// the file on flash is incomplete. See the /upload handler.
static bool upload_failed = false;

static const char *state_name() {
    switch (job_state()) {
        case JOB_RUNNING: return "running";
        case JOB_PAUSED:  return "paused";
        default:          return "idle";
    }
}

// The state blob the UI renders from. Also the SSE payload; see below.
//
// snprintf into a caller-supplied buffer rather than building a String: this
// runs on every progress tick and every SSE connect, and the String version did
// ten reallocating appends plus two temporary Strings for the floats. The
// output is bounded and small, so a stack buffer is both faster and takes no
// heap — which matters because the parser is allocating on the same core.
// Only what the page renders: no position or float formatting.
static void state_json(char *out, size_t cap) {
    snprintf(out, cap,
             "{\"state\":\"%s\",\"quality\":\"%s\",\"error\":%s,\"progress\":%u}",
             state_name(),
             job_quality(),
             job_had_error() ? "true" : "false",
             (unsigned)job_progress());
}

// Longest payload is ~75 bytes.
#define STATE_JSON_CAP 96

// Pushed to every connected browser on each state change, so a UI that
// didn't cause the change still sees it.
static void broadcast_state() {
    // Skip the work entirely when nobody's watching. events.send() with zero
    // clients still formats the payload and walks the client list, and this
    // is called on the parser's core several times a second during a print —
    // the common case for this machine is running unattended with no browser
    // open at all.
    if (events.count() == 0) return;
    char buf[STATE_JSON_CAP];
    state_json(buf, sizeof(buf));
    events.send(buf, "message", millis());
}

static void send_ok(AsyncWebServerRequest *req, bool ok) {
    req->send(ok ? 200 : 400, "application/json",
              ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// The only client is the homelab's proxy (flake: hosts/alechomelab/
// services.nix), which looks this name up over IPv4 mDNS for every
// connection. So: an A record and nothing else — no IPv6, no DNS-SD service
// advertisement, nothing extra on the air.
#define HOSTNAME "permablot"

// Never blocks setup(): the HTTP server comes up regardless and
// setAutoReconnect() joins whenever the router appears (after a power cut it often boots slower
// than the board), and ESP-IDF's mDNS responder follows the STA interface's
// IP events itself, so it is started once here and announces on every
// (re)connect with nothing to poll.
static void start_wifi() {
    WiFi.mode(WIFI_STA);
    // Modem-sleep unless a page is open (see the /events handlers): the radio
    // parks its receiver between beacons, saving ~60-80mA and a matching
    // amount of self-heating. Printing doesn't need it awake — the job
    // streams from flash on the other core.
    WiFi.setSleep(true);
    WiFi.setTxPower(WIFI_TX_POWER);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    MDNS.begin(HOSTNAME);
}

void webui_init() {
    start_wifi();

    // Push state to browsers whenever the job changes it on its own — a job
    // starting, pausing, finishing or progressing isn't tied to any request,
    // so without this the UI shows whatever it saw last.
    job_set_on_change(broadcast_state);

    // A dozing radio answers each request only at its next wake-up (100-300ms
    // depending on the router), and a page makes several in a row. So the
    // radio stays fully awake while any page is open — each holds this
    // stream — and dozes again when the last one closes. Both callbacks run
    // on the one async-TCP task, so the count needs no lock.
    static int pages_open = 0;
    events.onConnect([](AsyncEventSourceClient *c) {
        if (pages_open++ == 0) WiFi.setSleep(false);
        char buf[STATE_JSON_CAP];
        state_json(buf, sizeof(buf));
        c->send(buf, "message", millis());
    });
    events.onDisconnect([](AsyncEventSourceClient *) {
        if (--pages_open == 0) WiFi.setSleep(true);
    });
    server.addHandler(&events);

    // Requests are applied on loop()'s core, which pushes the resulting state
    // to every browser itself — pushing here would only resend the old one.
    server.on("/print", HTTP_POST, [](AsyncWebServerRequest *req) {
        bool ok = job_start();
        send_ok(req, ok);
    });

    server.on("/pause", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (job_state() == JOB_PAUSED) job_resume(); else job_pause();
        send_ok(req, true);
    });

    server.on("/stop", HTTP_POST, [](AsyncWebServerRequest *req) {
        job_stop();
        send_ok(req, true);
    });

    server.on("/quality", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("q", true)) { send_ok(req, false); return; }
        send_ok(req, job_set_quality(req->getParam("q", true)->value().c_str()));
    });

    // Gcode upload. The browser has already done the PDF conversion, so this
    // is a plain text body streamed straight to flash.
    // An upload that doesn't run to completion must not leave a truncated
    // /job.gcode behind — that file is what gets printed. `upload_failed`
    // latches any aborted write so the completion handler can delete the
    // partial file and report the failure, rather than the browser seeing
    // "ok" and printing half a drawing.
    server.on("/upload", HTTP_POST,
        [](AsyncWebServerRequest *req) {
            bool ok = !upload_failed && !upload_file; // closed => write completed
            if (!ok) {
                if (upload_file) upload_file.close();
                LittleFS.remove(JOB_PATH);
            }
            upload_failed = false;
            req->send(200, "application/json",
                      ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"write failed\"}");
        },
        [](AsyncWebServerRequest *req, String filename, size_t index,
           uint8_t *data, size_t len, bool final) {
            if (index == 0) {
                upload_failed = false;
                // Never overwrite the file a running job is reading from.
                if (job_state() != JOB_IDLE) { upload_failed = true; return; }
                upload_file = LittleFS.open(JOB_PATH, "w");
                if (!upload_file) upload_failed = true;
            }
            if (upload_failed) return;
            // A short write means the filesystem is full; stop rather than
            // silently produce a truncated job.
            if (upload_file && upload_file.write(data, len) != len) {
                upload_failed = true;
                upload_file.close();
                return;
            }
            if (final && upload_file) upload_file.close();
        });

    // Static assets LAST: serveStatic("/") matches every path, so registering
    // it before the API routes makes it swallow /print and friends.
    //
    // no-cache, not max-age: the browser revalidates every load, so a fresh
    // `uploadfs` (UI or stickers.json edits) shows up immediately instead of
    // a day later. The files are a few KB, so the cost is nothing.
    server.serveStatic("/", LittleFS, "/www/")
          .setDefaultFile("index.html")
          .setCacheControl("no-cache");

    server.begin();
}
