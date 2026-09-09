#include "webui.h"

#include <Arduino.h>
#include <WiFi.h>
#include "esp_netif.h"
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

#include "config.h"
#include "job.h"
#include "stepper.h"

// Hard error rather than a warning: without credentials the board has no
// network at all, which looks like a WiFi bug later.
#if !__has_include("wifi_config.h")
  #error "src/wifi_config.h missing. Run: cp src/wifi_config.h.example src/wifi_config.h"
#endif
#include "wifi_config.h"

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
static String state_json() {
    float x = 0, y = 0;
    stepper_get_machine_position_mm(&x, &y);
    String s = "{\"state\":\"";
    s += state_name();
    s += "\",\"quality\":\"";
    s += job_quality();
    s += "\",\"motorsLocked\":";
    s += job_motors_locked() ? "true" : "false";
    s += ",\"done\":";
    s += job_lines_done();
    s += ",\"total\":";
    s += job_lines_total();
    s += ",\"x\":";
    s += String(x, 2);
    s += ",\"y\":";
    s += String(y, 2);
    s += "}";
    return s;
}

// Pushed to every connected browser on each state change, so a UI that
// didn't cause the change still sees it.
static void broadcast_state() {
    events.send(state_json().c_str(), "message", millis());
}

static void send_ok(AsyncWebServerRequest *req, bool ok) {
    req->send(ok ? 200 : 400, "application/json",
              ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// True once SLAAC has handed out a routable (non-link-local) address.
static bool have_global_ipv6(esp_ip6_addr_t *out) {
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    return sta && esp_netif_get_ip6_global(sta, out) == ESP_OK;
}

static void start_mdns() {
    if (MDNS.begin(MDNS_NAME)) {
        MDNS.addService("http", "tcp", 80);
    }
}

// ESP-IDF's mDNS responder only ever publishes the *link-local* fe80::
// address for IPv6 — restarting it doesn't help, that's how the component
// builds its AAAA record. A bare fe80:: is unusable to a browser because it
// carries no %iface scope, so permablot.local resolved to a dead address.
//
// Publish a second, delegated hostname whose address list we control, holding
// the SLAAC global address (and the IPv4 one). The board is then reachable at
// <MDNS_NAME>-v6.local with a routable AAAA record.
static void mdns_publish_global_ipv6() {
    static bool done = false;
    if (done) return;

    esp_ip6_addr_t g;
    if (!have_global_ipv6(&g)) return;
    done = true;

    static char host6[64];
    snprintf(host6, sizeof(host6), "%s-v6", MDNS_NAME);

    mdns_ip_addr_t a6 = {};
    a6.addr.type = ESP_IPADDR_TYPE_V6;
    memcpy(a6.addr.u_addr.ip6.addr, g.addr, sizeof(g.addr));
    a6.addr.u_addr.ip6.zone = 0; // global scope needs no zone
    a6.next = nullptr;

    if (mdns_delegate_hostname_add(host6, &a6) == ESP_OK) {
        mdns_service_add_for_host(nullptr, "_http", "_tcp", host6, 80, nullptr, 0);
    }
}

// Called from loop() until the connection resolves. Deliberately does NOT
// block setup(): the HTTP server and the plotter must come up regardless, so
// a missing router can't stop you printing over USB.
static void wifi_service() {
    static bool settled = false;
    static uint32_t deadline = 0;

    if (settled) return;

    if (deadline == 0) deadline = millis() + 20000;

    if (WiFi.status() == WL_CONNECTED) {
        settled = true;
        // Request a link-local + SLAAC IPv6 address. Without this the board is
        // IPv4-only, and an mDNS client that asks for AAAA first (glibc's
        // nss-mdns does) waits for that query to time out on every lookup —
        // which makes permablot.local look broken even though the A record
        // resolves fine.
        WiFi.enableIpV6();
        start_mdns();
        return;
    }

    // setAutoReconnect() keeps trying in the background; the plotter still
    // works over USB meanwhile.
    if ((int32_t)(millis() - deadline) >= 0) settled = true;
}

static void start_wifi() {
    WiFi.mode(WIFI_STA);
    // Start in modem-sleep. webui_service() turns it off for the duration
    // of a print (see below) — this is the idle state, where the radio
    // parking its receiver between beacons saves ~60-80mA and a matching
    // amount of self-heating.
    WiFi.setSleep(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    // Connection is finished in wifi_service(); see note above.
}

void webui_init() {
    start_wifi();

    // Push state to browsers whenever the job changes it on its own — a job
    // starting, pausing, finishing or progressing isn't tied to any request,
    // so without this the UI shows whatever it saw last.
    job_set_on_change(broadcast_state);

    events.onConnect([](AsyncEventSourceClient *c) {
        c->send(state_json().c_str(), "message", millis());
    });
    server.addHandler(&events);

    server.on("/state", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send(200, "application/json", state_json());
    });

    server.on("/print", HTTP_POST, [](AsyncWebServerRequest *req) {
        bool ok = job_start();
        send_ok(req, ok);
        broadcast_state();
    });

    server.on("/pause", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (job_state() == JOB_PAUSED) job_resume(); else job_pause();
        send_ok(req, true);
        broadcast_state();
    });

    server.on("/stop", HTTP_POST, [](AsyncWebServerRequest *req) {
        job_stop();
        send_ok(req, true);
        broadcast_state();
    });

    server.on("/quality", HTTP_POST, [](AsyncWebServerRequest *req) {
        if (!req->hasParam("q", true)) { send_ok(req, false); return; }
        String q = req->getParam("q", true)->value();
        if (q != "draft" && q != "poster") { send_ok(req, false); return; }
        job_set_quality(q.c_str());
        send_ok(req, true);
        broadcast_state();
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
                job_notify_new_file();   // clears the stale line count
            } else {
                job_notify_new_file();
            }
            upload_failed = false;
            req->send(200, "application/json",
                      ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"write failed\"}");
            broadcast_state();
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
    // it before the API routes makes it swallow /state, /print and friends.
    server.serveStatic("/", LittleFS, "/www/")
          .setDefaultFile("index.html")
          .setCacheControl("max-age=86400");

    server.begin();
}

// WiFi modem-sleep parks the receiver between AP beacons. That saves a lot of
// idle power, but it also adds tens of ms of latency — which matters during a
// print, because job_service() feeds one gcode line per loop() and the planner
// only buffers 16 blocks. Drain that and the plot stutters. So: radio fully
// awake while printing, asleep the rest of the time.
static void service_wifi_power(bool printing) {
    static bool awake = false;
    if (printing == awake) return;
    awake = printing;
    WiFi.setSleep(!printing);
}

void webui_service() {
    wifi_service();
    mdns_publish_global_ipv6();
    service_wifi_power(job_state() != JOB_IDLE);
}
