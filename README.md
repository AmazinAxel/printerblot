# Printerblot

A modified version of [Sam's updated Blot firmware](https://github.com/samdev-7/upgraded-blot), intended more for document/flier printing with speed and precision

## Firmware flashing

Unlike the upstream firmware, you have to flash this with [PlatformIO](https://platformio.org/) instead of the Arduino IDE.

Plug in your Blot and hold the B button on the Xiao (you only need to do this if this is the first flash), then run `pio run -t upload` and you're done! You can use the utilities in `tools/` to send raw gcode (through `term.py`), or turn a pdf to gcode (`pdf2gcode.py`) and stream that (`stream.py`). If you're on Nix, you can use the devshell: `nix develop`
