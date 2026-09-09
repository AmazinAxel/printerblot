# Printerblot

A modified version of [Sam's updated Blot firmware](https://github.com/samdev-7/upgraded-blot), intended more for document/flier printing with speed and precision

It uses a Seeed XIAO ESP32-S3 instead of the RP2040, which allows you to control it from a web interface!

## Hardware

We're reusing the same hardware from [Blot](https://github.com/hackclub/blot), but there is a magnetic paper attachment with these [guide rails](https://cad.onshape.com/documents/e4d5e6d129eb20e428115785/w/b41a37e6be291bfad55ecd9b/e/578fac4d719cb84d5432b5e6?renderMode=0&uiState=6a8a94e45d478301f2f15755) that I've designed which helps for printing stability! A printable version is in [`Printerblot_rails_v1.stl`](Printerblot_rails_v1.stl)

## How it's improved

The webserver hosted on the ESP32-S3 boots as soon as you plug in the Blot. You can access it without remembering an IP, just go to `<your configured name>.local` in your web browser. You can upload a PDF there and it converts it to gcode in the browser, sending only the gcode to the Blot. That's stored in the device's flash, so you can reprint the same file without re-uploading it, and because the S3 has a second core you can upload one job while another prints so it's really fast and no connected computer is required.

The firmware also manages the hardware more carefully than the original with better motor handling, pen servo handling and networking radio which cut idle power and heat so you can leave the blot plugged in.

## Setup

Make `src/wifi_config.h` and put in your config:

```c
#pragma once

#define WIFI_SSID  ""
#define WIFI_PASS  ""
#define MDNS_NAME  "permablot"
```

```sh
pio run -t upload   # firmware
pio run -t uploadfs # web stuff
```

Then open `http://<MDNS name here>.local/` in your web browser!!
