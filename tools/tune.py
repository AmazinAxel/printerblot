#!/usr/bin/env python3
"""Blot firmware tuning helper — automates the procedure in TUNING.md.

Walks you through each of the four tests (C/D/A/B, in that order),
runs the test pattern at successively higher values, and asks whether
the pen returned cleanly to its starting mark. When you say no (or you
hear stalls), the script backs off to 80% of the last passing value
and saves it via the matching `$N=` so it persists to flash.

Tests run in accel-first order (C, D, A, B) — speed tests need high
accel to actually reach their commanded F.

Usage:
    python3 tune.py                        # coarse tune
    python3 tune.py --fine                 # fine tune (1.1× steps, 90% backoff)
    python3 tune.py --port /dev/ttyACM0
    python3 tune.py --tests CA             # axis accel + axis speed only
"""

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("Missing pyserial. Make sure you're in the nix devshell: nix develop\n")
    sys.exit(1)


DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200


def open_port(port, baud):
    """Open the serial port and flush any buffered data."""
    s = serial.Serial(port, baud, timeout=0.2, dsrdtr=False, rtscts=False)
    s.reset_input_buffer()
    return s


def read_until(s, pattern, timeout):
    buf = ""
    deadline = time.time() + timeout
    rx = re.compile(pattern)
    while time.time() < deadline:
        chunk = s.read(s.in_waiting or 1).decode(errors="replace")
        if chunk:
            buf += chunk
            if rx.search(buf):
                return buf
        else:
            time.sleep(0.01)
    return buf


def send_line(s, line, timeout=10.0):
    s.write((line + "\n").encode())
    buf = read_until(s, r"ok\r\n|error:\d+", timeout)
    if "ok\r\n" in buf: return "ok"
    m = re.search(r"error:(\d+)", buf)
    if m: return f"error:{m.group(1)}"
    return "timeout"


def status(s, timeout=0.6):
    s.reset_input_buffer()
    s.write(b"?")
    buf = read_until(s, r"<\w+\|MPos:[-\d.]+,[-\d.]+", timeout)
    m = re.search(r"<(\w+)\|MPos:([-\d.]+),([-\d.]+)", buf)
    if m:
        return m.group(1), float(m.group(2)), float(m.group(3))
    return "Unknown", 0.0, 0.0


def wait_idle(s, timeout=120.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        st, _, _ = status(s)
        if st == "Idle":
            return
        time.sleep(0.15)
    raise RuntimeError("Timed out waiting for Idle")


def stream(s, lines):
    for line in lines:
        line = line.strip()
        if not line: continue
        r = send_line(s, line)
        if r.startswith("error"):
            print(f"    ! [{line}] → {r}", file=sys.stderr)


def parse_all_settings(s):
    s.reset_input_buffer()
    s.write(b"$$\n")
    buf = read_until(s, r"ok\r\n", timeout=3.0)
    return {m.group(1): float(m.group(2))
            for m in re.finditer(r"(\$\d+)=([-\d.]+)", buf)}


def prompt(msg, valid=("y", "n", "r", "q")):
    while True:
        r = input(msg).strip().lower()
        if r in valid: return r
        print(f"  (choose one of: {', '.join(valid)})")


# ───────── test patterns ─────────

def pattern_a(speed_mm_min):
    return [
        f"G1 X100 F{int(speed_mm_min)}", "G1 X0",
        f"G1 X100 F{int(speed_mm_min)}", "G1 X0",
        f"G1 X100 F{int(speed_mm_min)}", "G1 X0",
    ]


def pattern_b(speed_mm_min):
    return [
        f"G1 X50 Y50 F{int(speed_mm_min)}", "G1 X0 Y0",
        f"G1 X50 Y50 F{int(speed_mm_min)}", "G1 X0 Y0",
        f"G1 X50 Y50 F{int(speed_mm_min)}", "G1 X0 Y0",
    ]


def pattern_c(_accel):
    return [
        "G1 X20 F999999", "G1 X0",
        "G1 X20 F999999", "G1 X0",
        "G1 X20 F999999", "G1 X0",
    ]


def pattern_d(_accel):
    return [
        "G1 X14 Y14 F999999", "G1 X0 Y0",
        "G1 X14 Y14 F999999", "G1 X0 Y0",
        "G1 X14 Y14 F999999", "G1 X0 Y0",
    ]


def test_block(pattern_lines):
    return (
        ["G21", "G90", "M17", "G92 X0 Y0", "M3", "G4 P100"]
        + pattern_lines
        + ["M5"]
    )


# ───────── the iterative search ─────────

def run_tuning(s, name, setting, pattern_fn, start, growth,
               backoff=0.8):
    print(f"\n=== {name} ({setting}) ===")
    print(f"Start {start}, step ×{growth}.\n")

    value = float(start)
    last_good = None

    while True:
        print(f"Setting {setting} = {int(value)}...")
        if send_line(s, f"{setting}={int(value)}") != "ok":
            print(f"  ! could not set {setting}, aborting test")
            break

        stream(s, test_block(pattern_fn(value)))
        try:
            wait_idle(s)
        except RuntimeError as e:
            print(f"  ! {e}")
            break

        _, mx, my = status(s)
        print(f"  MPos after test: ({mx:.3f}, {my:.3f})")
        r = prompt(
            "  Pen returned cleanly, no stalls? [y/n/r=retry/q=quit tuning]: "
        )
        if r == "q":
            print("  quitting this test")
            break
        if r == "r":
            continue
        if r == "y":
            last_good = value
            value *= growth
        else:  # "n"
            break

    if last_good is None:
        print(f"  ! No passing value found for {setting}.")
        return None
    final = int(last_good * backoff)
    print(f"  → Last passing {setting} = {int(last_good)}. "
          f"Saving {int(backoff * 100)}% → {final}.")
    send_line(s, f"{setting}={final}")
    return final


# ───────── main ─────────

# Minimum starting values (current tuned defaults)
MIN_ACCEL = 16606
MIN_SPEED = 16401

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"Serial port (default: {DEFAULT_PORT})")
    ap.add_argument("--tests", default="CDAB",
                    help="Which tests to run (any subset of ABCD; "
                         "default CDAB runs accel before speed)")
    ap.add_argument("--fine", action="store_true",
                    help="Fine-tune: 1.1× growth, 90%% backoff, starts "
                         "at 0.95× the currently-stored value.")
    args = ap.parse_args()

    if args.fine:
        growth  = 1.1
        backoff = 0.9
        start_frac = 0.95
    else:
        growth  = 1.5
        backoff = 0.8
        start_frac = 1.0

    s = open_port(args.port, BAUD)

    orig = parse_all_settings(s)
    print(f"\nCurrent persisted settings:")
    for k in ("$110", "$111", "$112", "$120", "$122"):
        if k in orig:
            print(f"  {k} = {orig[k]:.0f}")

    print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 Blot tuning

 PHYSICAL SETUP (do this before continuing):
   1. Put a pen in the holder with tip touching paper.
   2. Tape down paper so it doesn't slide.
   3. Move the carriage BY HAND to the origin you want. Leave at
      least 120 mm of room in +X and +Y.
   4. Close anything else that owns the USB port.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")
    input("Press ENTER when you're ready → ")

    print("\nMarking origin...")
    stream(s, ["M17", "G21", "G90", "G92 X0 Y0", "M3", "G4 P400", "M5"])
    wait_idle(s)
    print("Origin dot made. All subsequent tests should return to this point.\n")

    results = {}
    touched = set()

    def start_for(setting_key):
        stored = int(orig.get(setting_key, MIN_ACCEL))
        return max(stored, int(stored * start_frac))

    c_start = max(MIN_ACCEL, start_for("$120"))
    d_start = max(MIN_ACCEL, start_for("$122"))
    a_start = max(MIN_SPEED, start_for("$110"))
    b_start = max(MIN_SPEED, start_for("$112"))

    PUSH = 999999999  # uncapped — let firmware clamp internally

    try:
        tests = args.tests.upper()

        if "C" in tests:
            touched.update({"$110", "$112", "$122"})
            send_line(s, f"$110={PUSH}")
            send_line(s, f"$112={PUSH}")
            send_line(s, f"$122={PUSH}")
            results["$120"] = run_tuning(
                s, "Test C: axis-aligned accel", "$120",
                pattern_c, start=c_start, growth=growth, backoff=backoff,
            )

        if "D" in tests:
            touched.update({"$110", "$112", "$120"})
            send_line(s, f"$110={PUSH}")
            send_line(s, f"$112={PUSH}")
            send_line(s, f"$120={PUSH}")
            results["$122"] = run_tuning(
                s, "Test D: diagonal accel", "$122",
                pattern_d, start=d_start, growth=growth, backoff=backoff,
            )

        if "A" in tests:
            touched.update({"$120", "$122", "$112"})
            a_accel = results.get("$120") or int(orig.get("$120", MIN_ACCEL))
            d_accel = results.get("$122") or int(orig.get("$122", MIN_ACCEL))
            send_line(s, f"$120={a_accel}")
            send_line(s, f"$122={d_accel}")
            send_line(s, f"$112={PUSH}")
            results["$110"] = run_tuning(
                s, "Test A: axis-aligned speed", "$110",
                pattern_a, start=a_start, growth=growth, backoff=backoff,
            )

        if "B" in tests:
            touched.update({"$120", "$122", "$110"})
            a_accel = results.get("$120") or int(orig.get("$120", MIN_ACCEL))
            d_accel = results.get("$122") or int(orig.get("$122", MIN_ACCEL))
            send_line(s, f"$120={a_accel}")
            send_line(s, f"$122={d_accel}")
            send_line(s, f"$110={PUSH}")
            results["$112"] = run_tuning(
                s, "Test B: diagonal speed", "$112",
                pattern_b, start=b_start, growth=growth, backoff=backoff,
            )

    finally:
        for k in ("$110", "$112", "$120", "$122"):
            if k not in touched and k not in results:
                continue
            if results.get(k) is not None:
                desired = results[k]
            else:
                desired = orig.get(k)
            if desired is None:
                continue
            print(f"  writing {k} = {int(desired)}")
            send_line(s, f"{k}={int(desired)}")

        final_110 = results.get("$110") or orig.get("$110", MIN_SPEED)
        final_112 = results.get("$112") or orig.get("$112", MIN_SPEED)
        target_111 = min(final_112, max(final_110, orig.get("$111", MIN_SPEED)))
        send_line(s, f"$111={int(target_111)}")

        send_line(s, "M5")
        send_line(s, "M18")

    print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("Final persisted values:")
    final = parse_all_settings(s)
    for k in ("$110", "$111", "$112", "$120", "$122"):
        new = final.get(k)
        was = orig.get(k)
        if new is None: continue
        changed = "" if was == new else f"  (was {int(was) if was else '?'})"
        tuned = " [tuned]" if results.get(k) is not None else ""
        print(f"  {k:5} = {int(new)}{tuned}{changed}")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("\nDone. Tuning values persist across power cycles.")
    s.close()


if __name__ == "__main__":
    main()
