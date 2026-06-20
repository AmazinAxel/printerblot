#!/usr/bin/env python3
"""Blot terminal — interactive gcode sender with multi-line staging.

Keys:
  Enter             → newline (stage another command)
  Alt/Option+Enter  → send all staged lines
  ?                 → realtime status (when input buffer is empty)
  Ctrl+X            → soft reset (0x18)
  Ctrl+C / Ctrl+D   → quit
  Ctrl+L            → clear screen

Usage:
    python3 tools/term.py                      # default port
    python3 tools/term.py --port /dev/ttyACM0
"""
import argparse
import sys
import threading
import time

try:
    import serial
    from prompt_toolkit import PromptSession
    from prompt_toolkit.key_binding import KeyBindings
    from prompt_toolkit.patch_stdout import patch_stdout
    from prompt_toolkit.formatted_text import FormattedText
except ImportError as e:
    sys.stderr.write(f"Missing dependency: {e}\nMake sure you're in the nix devshell: nix develop\n")
    sys.exit(1)


DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200


def reader_thread(ser, stop_event):
    """Stream firmware output to stdout. patch_stdout() in main repaints the
    prompt around incoming lines so typing isn't clobbered."""
    buf = b""
    while not stop_event.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            print("[serial dropped]")
            break
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").rstrip("\r")
            if not text:
                continue
            # Colorize state-ful responses so they're easy to scan.
            if text.startswith("<"):
                prefix = "  "  # status report — MPos pings
            elif text.startswith("error"):
                prefix = "  ! "
            elif text.startswith("["):
                prefix = "  "  # $I, $G, $$ bracketed responses
            elif text == "ok":
                prefix = "    "  # de-emphasize ok
            else:
                prefix = "  "
            print(f"{prefix}{text}")
        # Drain partial-line tail only if it's been there a while; otherwise
        # wait for the rest.
        if buf and not buf.endswith(b"\r"):
            # Short-timeout flush for orphan fragments (rare — welcome often
            # doesn't end with \n).
            time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"Serial port (default: {DEFAULT_PORT})")
    ap.add_argument("--baud", type=int, default=BAUD)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.stderr.write(f"Can't open {args.port}: {e}\n")
        sys.exit(1)

    time.sleep(0.4)
    ser.reset_input_buffer()

    stop = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop), daemon=True)
    t.start()

    kb = KeyBindings()

    @kb.add("escape", "enter")
    def _submit(event):
        event.current_buffer.validate_and_handle()

    @kb.add("c-x")
    def _soft_reset(event):
        ser.write(bytes([0x18]))
        print("  [0x18 soft reset sent]")

    @kb.add("?")
    def _status_or_literal(event):
        # Only treat as realtime status if the input buffer is empty.
        b = event.current_buffer
        if not b.text:
            ser.write(b"?")
        else:
            b.insert_text("?")

    @kb.add("c-l")
    def _clear(event):
        # Clears visible terminal, keeps buffer.
        sys.stdout.write("\x1b[2J\x1b[H")
        sys.stdout.flush()

    def bottom_toolbar():
        return FormattedText([
            ("class:tb", f" {args.port} @ {args.baud}   "
                        "Alt+Enter: send   ?: status   Ctrl+X: reset   Ctrl+D: quit "),
        ])

    session = PromptSession(
        "",
        multiline=True,
        key_bindings=kb,
        bottom_toolbar=bottom_toolbar,
        prompt_continuation=lambda w, n, sw: "",
    )

    print(f"blot-term — connected to {args.port} @ {args.baud}")
    print("Type gcode. Enter = new line; Alt/Option+Enter = send. Ctrl+D to quit.\n")

    try:
        with patch_stdout():
            while True:
                try:
                    text = session.prompt("> ")
                except (EOFError, KeyboardInterrupt):
                    break
                if not text.strip():
                    continue
                for raw in text.splitlines():
                    line = raw.strip()
                    if not line:
                        continue
                    ser.write((line + "\n").encode())
                    # Small pacing; firmware line buffer + planner handle the rest.
                    time.sleep(0.005)
    finally:
        stop.set()
        try:
            ser.close()
        except Exception:
            pass
        print("\nbye.")


if __name__ == "__main__":
    main()
