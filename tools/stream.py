#!/usr/bin/env python3
"""Minimal gcode streamer for the Blot pen plotter.

Usage:
    python3 tools/stream.py ~/blot-gcode/drawing.gcode
    python3 tools/stream.py ~/blot-gcode/drawing.gcode --port /dev/ttyACM0
"""
import argparse, sys, time, re
import serial


def open_port(port, baud):
    """Open the serial port and flush any buffered data."""
    s = serial.Serial(port, baud, timeout=0.2, dsrdtr=False, rtscts=False)
    s.reset_input_buffer()
    return s


def send_line(s, line):
    """Send one line and wait for ok/error. Returns status string."""
    s.write((line + "\n").encode())
    buf = ""
    start = time.time()
    while time.time() - start < 120:
        chunk = s.read(s.in_waiting or 1).decode(errors="replace")
        if chunk:
            buf += chunk
            if re.search(r"ok\r\n|error:\d+", buf):
                status = "ok" if "ok\r\n" in buf else buf.strip()
                return status
    return "timeout"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    with open(args.file) as f:
        lines = [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith(';')]

    s = open_port(args.port, args.baud)
    print(f"Streaming {len(lines)} lines from {args.file}...\n")

    for i, line in enumerate(lines):
        status = send_line(s, line)
        mark = "  " if status == "ok" else "!!"
        print(f"{mark} [{i + 1:3d}] {line:<40}  →  {status}")

        if "error" in status:
            print("\n  Stopping on error.")
            sys.exit(1)
        if status == "timeout":
            print("\n  Timed out waiting for ok.")
            sys.exit(1)

    print("\nDone. All lines accepted by firmware.")
    s.close()

if __name__ == "__main__":
    main()
