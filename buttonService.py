#!/usr/bin/env python3
import glob
import json
import os
import queue
import socket
import subprocess
import sys
import threading
import time
import serial
import lgpio

DATA_DIR = os.environ.get("BLOT_DATA_DIR", "/var/lib/blotd")
STATE_FILE = os.path.join(DATA_DIR, "state.json")
GCODE_FILE = os.path.join(DATA_DIR, "lastJob.gcode")
SOCKET_PATH = "/run/blot-socket/blot-socket.sock"

BAUD = 115200

BUTTON_PIN = 3
HOLD_TIME = 0.5

RETURN_RATE = 3000

QUALITY = {
    "draft":  {"$110": 16000, "$120": 16000, "$140": 0.05, "$141": 0.004},
    "poster": {"$110": 4000,  "$120": 1000,  "$140": 0.01, "$141": 0.002},
}

DEFAULT_STATE = {"state": "idle", "quality": "draft", "motorsLocked": False}


#def log(*a):
#    print(*a, flush=True)

class Controller:
    def __init__(self):
        self.q = queue.Queue()
        self.ser = None
        self.state = self._load_state()
        self.state["state"] = "idle" # never resume a half-finished stream
        self._origin = (0.0, 0.0) # job origin in MACHINE coords

    # state.json
    def _load_state(self):
        try:
            with open(STATE_FILE) as f:
                s = json.load(f)
            return {**DEFAULT_STATE, **s}
        except (OSError, ValueError):
            return dict(DEFAULT_STATE)

    def _save_state(self):
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.state, f)
        os.replace(tmp, STATE_FILE) # atomic rename

    def _set(self, **kw):
        self.state.update(kw)
        self._save_state()

    # serial
    def _open_serial(self):
        while True:
            ports = sorted(glob.glob("/dev/ttyACM*"))
            if ports:
                try:
                    self.ser = serial.Serial(ports[0], BAUD, timeout=0.5, dsrdtr=False, rtscts=False)
                    time.sleep(0.3)
                    self.ser.reset_input_buffer()
                    #log("serial open:", ports[0])
                    return
                except (serial.SerialException, OSError) as e:
                    #log("waiting for serial:", e)
            time.sleep(2)

    def _init_firmware(self):
        self._write(bytes([0x18]))
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self._send_line("M18")
        self._set(state="idle", motorsLocked=False)

    def _reopen(self):
        try:
            self.ser.close()
        except Exception:
            pass
        self._open_serial()
        self._init_firmware()

    def _write(self, data: bytes):
        self.ser.write(data)

    def _readline(self):
        return self.ser.readline().decode(errors="replace").strip()

    def _send_line(self, line, timeout=120):
        self._write((line + "\n").encode())
        deadline = time.time() + timeout
        while time.time() < deadline:
            ln = self._readline()
            if not ln:
                continue
            if ln == "ok":
                return "ok"
            if ln.startswith("error"):
                return ln
        return "timeout"

    def _status(self, timeout=2):
        self._write(b"?")
        deadline = time.time() + timeout
        while time.time() < deadline:
            ln = self._readline()
            if ln.startswith("<") and "MPos:" in ln:
                st = ln[1:].split("|", 1)[0]
                mpos = ln.split("MPos:", 1)[1].split("|", 1)[0]
                parts = mpos.split(",")
                x, y = float(parts[0]), float(parts[1])
                return st, x, y
        return "?", 0.0, 0.0

    def _wait_idle(self, timeout=120):
        deadline = time.time() + timeout
        while time.time() < deadline:
            st, _, _ = self._status()
            if st == "Idle":
                return True
            time.sleep(0.2)
        return False

    def _apply_quality(self, quality):
        for k, v in QUALITY[quality].items():
            self._send_line(f"{k}={v}")

    def _lock(self, locked):
        self._send_line("M17" if locked else "M18")
        self._set(motorsLocked=locked)

    def _poweroff(self):
        self._save_state()
        subprocess.run(["systemctl", "poweroff"])

    # print!
    def _run_print(self):
        if not os.path.isfile(GCODE_FILE):
            return
        try:
            with open(GCODE_FILE) as f:
                lines = [l.strip() for l in f
                         if l.strip() and not l.strip().startswith(";")]
        except OSError:
            return
        if not lines:
            return

        self._apply_quality(self.state["quality"])
        _, mx, my = self._status()
        self._origin = (mx, my)
        self._pen_down = False
        self._set(state="running")
        #log(f"printing {len(lines)} lines, origin=({mx:.3f},{my:.3f})")

        i = 0
        while i < len(lines):
            ev = self._drain()
            if ev == "STOP":
                self._stop_sequence()
                return
            if ev == "PAUSE":
                if self._pause_loop() == "STOP":
                    self._stop_sequence()
                    return
            
            
            # resumed!

            status = self._send_line(lines[i])
            if status != "ok":
                self._stop_sequence()
                return
            if lines[i].startswith("M3"):
                self._pen_down = True
            elif lines[i].startswith("M5"):
                self._pen_down = False
            i += 1

        self._wait_idle()
        self._send_line("M18")
        self._set(state="idle", motorsLocked=False)
        #log("print complete")

    def _pause_loop(self):
        self._wait_idle() # finish buffer then stop
        lifted = self._pen_down
        if lifted:
            self._send_line("M5") # pen up
            self._wait_idle()
        self._set(state="paused")
        while True:
            cmd = self.q.get() # block
            v = self._resolve(cmd)
            if v == "PAUSE": # press while paused = resume
                if lifted:
                    self._send_line("M3") # pen down
                    self._wait_idle()
                self._set(state="running")
                return "RESUME"
            if v == "STOP":
                return "STOP"
            if v == "QUALITY":
                self._set(quality=cmd[1]) # applies to the next job

    def _stop_sequence(self):
        self._write(b"!") # feed hold
        deadline = time.time() + 10
        while time.time() < deadline:
            st, _, _ = self._status()
            if st in ("Hold", "Idle"):
                break
            time.sleep(0.1)
        self._write(bytes([0x18])) # soft reset to flush planner and clear G92
        time.sleep(0.3)
        self.ser.reset_input_buffer() # drop annoying welcome banner
        mx, my = self._origin
        self._send_line("M5") # pen up
        self._send_line("G90")
        self._send_line(f"G1 X{mx:.3f} Y{my:.3f} F{RETURN_RATE}")
        self._wait_idle()
        self._send_line("M18") # disable motors
        self._set(state="idle", motorsLocked=False)
        #log("stopped, returned to origin")

    # commands from the website
    def _resolve(self, cmd):
        v = cmd[0]
        if v == "PRESS":
            return {"idle": "START", "running": "PAUSE", "paused": "PAUSE"}[self.state["state"]]
        if v == "HOLD":
            return "POWEROFF" if self.state["state"] == "idle" else "STOP"
        return v

    def _drain(self):
        result = None
        while True:
            try:
                cmd = self.q.get_nowait()
            except queue.Empty:
                return result
            v = self._resolve(cmd)
            if v in ("PAUSE", "STOP"):
                result = v
            elif v == "QUALITY":
                self._set(quality=cmd[1])

    def _dispatch(self, cmd):
        v = self._resolve(cmd)
        st = self.state["state"]
        if v == "START" and st == "idle":
            self._run_print()
        elif v == "POWEROFF" and st == "idle":
            self._poweroff()
        elif v == "QUALITY":
            self._set(quality=cmd[1])
        elif v == "LOCK" and st == "idle":
            self._lock(True)
        elif v == "UNLOCK" and st == "idle":
            self._lock(False)
        elif v == "SLEEP" and st == "idle":
            self._poweroff()
        # everything else ignored at idle

    def run(self):
        self._open_serial()
        self._init_firmware()

        while True:
            cmd = self.q.get()
            try:
                self._dispatch(cmd)
            except (serial.SerialException, OSError) as e:
                #log("serial error, reopening:", e)
                self._reopen()
                try:
                    self._dispatch(cmd)
                except (serial.SerialException, OSError) as e2:
                    #log("serial error after reopen, dropping command:", e2)
            except Exception as e:
                #log("controller error (continuing):", e)

## ACTUAL BUTTON SERVICE

def start_button(q):
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_alert(h, BUTTON_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
    lgpio.gpio_set_debounce_micros(h, BUTTON_PIN, 50000) # 50 ms debounce

    st = {"timer": None, "held": False}

    def fire_hold():
        st["held"] = True
        q.put(("HOLD",))

    def on_edge(chip, gpio, level, tstamp):
        # level: 0 = pressed, 1 = released, 2 = watchdog (ignored).
        if level == 0: # press
            st["held"] = False
            st["timer"] = threading.Timer(HOLD_TIME, fire_hold)
            st["timer"].daemon = True
            st["timer"].start()
        elif level == 1: # release
            if st["timer"]:
                st["timer"].cancel()
            if not st["held"]: # short press
                q.put(("PRESS",))

    cb = lgpio.callback(h, BUTTON_PIN, lgpio.BOTH_EDGES, on_edge)
    return (h, cb)


def start_socket(q):
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET_PATH)
    os.chmod(SOCKET_PATH, 0o666) # let web connect
    srv.listen(8)

    VERBS = {
        "lock": ("LOCK",), "unlock": ("UNLOCK",), "sleep": ("SLEEP",),
        "quality:draft": ("QUALITY", "draft"),
        "quality:poster": ("QUALITY", "poster"),
    }

    def serve():
        while True:
            try:
                conn, _ = srv.accept()
                msg = conn.recv(64).decode(errors="replace").strip()
                if msg in VERBS:
                    q.put(VERBS[msg])
                    conn.sendall(b"ok\n")
                else:
                    conn.sendall(b"err\n")
                conn.close()
            except Exception as e:
                #log("socket error:", e)

    threading.Thread(target=serve, daemon=True).start()

def main():
    ctl = Controller()
    _btn = start_button(ctl.q) # noqa: F841
    start_socket(ctl.q)
    ctl.run()

if __name__ == "__main__":
    main()
