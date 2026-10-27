# Printerblot

A modified version of [Sam's updated Blot firmware](https://github.com/samdev-7/upgraded-blot), intended more for document/flier printing with speed and precision

It uses a Seeed XIAO ESP32-S3 instead of the RP2040, which allows you to control it from a web interface!

## Hardware

We're reusing the same hardware from [Blot](https://github.com/hackclub/blot), but there is a magnetic paper attachment with these [guide rails](https://cad.onshape.com/documents/e4d5e6d129eb20e428115785/w/b41a37e6be291bfad55ecd9b/e/578fac4d719cb84d5432b5e6?renderMode=0&uiState=6a8a94e45d478301f2f15755) that I've designed which helps for printing stability! A printable version is in [`Printerblot_rails_v1.stl`](Printerblot_rails_v1.stl)

## How it's improved

The webserver hosted on the ESP32-S3 boots as soon as you plug in the Blot. It announces itself as `permablot.local`, and a tiny proxy on my homelab makes it reachable over Tailscale at `http://alechomelab:8000`. You can upload a PDF there and it converts it to gcode in the browser, sending only the gcode to the Blot. That's stored in the device's flash, so you can reprint the same file without re-uploading it. The network runs on one of the S3's cores and printing on the other, so the page stays responsive mid-print and no connected computer is required — the web UI is the only way in (there's no USB input).

The firmware is fully event-driven and manages the hardware more carefully than the original — motors and servo go quiet, the CPU slows and the radio dozes whenever it's idle — so you can leave the blot plugged in all year.

## Setup

Make `src/wifi_config.h` and put in your config:

```c
#pragma once

#define WIFI_SSID  ""
#define WIFI_PASS  ""
```

```sh
pio run -t upload   # firmware
pio run -t uploadfs # web stuff
```

Then open `http://alechomelab:8000/` (tailnet) or `http://permablot.local/` (LAN, if your machine resolves mDNS) in your web browser!!
