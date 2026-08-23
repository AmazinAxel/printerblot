# Printerblot

A modified version of [Sam's updated Blot firmware](https://github.com/samdev-7/upgraded-blot), intended more for document/flier printing with speed and precision

## Hardware

We're reusing the same hardware from [Blot](https://github.com/hackclub/blot), but there is a magnetic paper attachment with these [guide rails](https://cad.onshape.com/documents/e4d5e6d129eb20e428115785/w/b41a37e6be291bfad55ecd9b/e/578fac4d719cb84d5432b5e6?renderMode=0&uiState=6a8a94e45d478301f2f15755) that I've designed which helps for printing stability. 

## Firmware flashing

Unlike the upstream firmware, you have to flash this with [PlatformIO](https://platformio.org/) instead of the Arduino IDE.

Plug in your Blot and hold the B button on the Xiao (you only need to do this if this is the first flash), then run `pio run -t upload` and you're done! You can use the utilities in `tools/` to send raw gcode (through `term.py`), or turn a pdf to gcode (`pdf2gcode.py`) and stream that (`stream.py`). If you're on Nix, you can use the devshell: `nix develop`

## On a Pi

You can optionally hook up a button to a Pi (on header pins 5 and 6) and install the Nix flake ((Permablot host))[https://github.com/AmazinAxel/flake] so you don't need a laptop! The Pi exposes a webserver where you can upload a PDF and adjust some printer settings and sleep the Pi. The gpio button can be used to start, stop, and pause prints and can even be used to cancel prints or shutdown the Pi. It runs well on a Zero W or a Zero 2W.

To install, clone my Nix flake and build an iso to flash to an SD card: `nixos-rebuild build-image --image-variant sd --flake .#permablot` (if you are installing it on a PC or a non-SD device, replace `sd` with `iso`).
