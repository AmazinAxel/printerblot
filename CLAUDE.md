# Printerblot — working notes

Blot pen plotter firmware on a **Seeed XIAO ESP32-S3** (migrated from an
RP2040 XIAO; the D0-D10 aliases hit the same pads, so the carrier board is
unchanged). Hosts its own web UI — the **only** interface: no USB/serial
input, no daemon, no connected computer.

Full architecture is in [docs/FIRMWARE.md](docs/FIRMWARE.md). This file is the
short version plus the things that bite.

## Module map

| File | Role |
|------|------|
| `main.cpp` | `setup()`; `loop()` = sleep until woken, then service |
| `wake.cpp` | loop wake-ups (FreeRTOS task notification), ISR-safe |
| `gcode.cpp` | tokenizer + the supported subset (G0/G1/G21/G90/G92/M3/M5) |
| `planner.cpp` | 16-block ring, junction deviation, trapezoids |
| `stepper.cpp` | step ISR: Bresenham + velocity, **integer only**; pen blocks; drivers; CPU clock |
| `steptimer.cpp` | timer group 0 wrapper, IRAM |
| `penservo.cpp` | timer group 1, manual 50Hz pulse; travel/hold counted in frames |
| `job.cpp` | streams `/job.gcode` into the parser, web requests, idle release |
| `webui.cpp` | WiFi, mDNS `permablot` (A record only), async HTTP, SSE, upload |
| `settings.cpp` | one table of settings; EEPROM-emulation persistence |
| `kinematics.h` | CoreXY forward/inverse + `motor_load_for_unit_dir` |

PDF→gcode happens in the browser via `convert.js` served from
amazinaxel.com, **not** vendored here. Speed/accel calibration is to become
a web tool, specced in [docs/CALIBRATION.md](docs/CALIBRATION.md) (it
replaced USB `tune.py`).

## Hard constraints — violating these crashes or stutters the plot

1. **No floating point in the step ISR.** Xtensa FreeRTOS allocates the FPU
   lazily and doesn't save coprocessor state across interrupts → EXCCAUSE 4
   the moment another task owns the FPU. Hence the hand-rolled `isqrt()`.
   This crashed mid-print when the network stack was busy.
2. **No `digitalWrite` / `delayMicroseconds` / `gpio_set_level` in the ISR.**
   All live in flash; an `ESP_INTR_FLAG_IRAM` handler panics with "Cache
   disabled but cached memory region accessed" as soon as anything writes to
   flash. Use `isr_pin_write()` / `isr_delay_us_since()` from `config.h`.
3. **Everything the ISRs reach must be `IRAM_ATTR`** — including
   `plan_get_current_block()`, `plan_discard_current_block()` and
   `wake_loop_from_isr()`.
4. **The pen servo can't use PWM.** It's on D6 = GPIO43 (UART0); the LEDC
   matrix can't route there, so `Servo`/ESP32Servo hangs in `attach()`.
   `penservo.cpp` bit-bangs 50Hz from a hardware timer instead. There is
   deliberately **no servo library dependency**.
5. **CoreXY: pure X/Y are the *fastest* directions, ±45° the slowest** —
   inverse of a Cartesian machine. `$112`/`$122` are motor-space caps and
   bind first on diagonals (`$112`/√2).
6. **Event-driven: no polling, no delays, no software timers.** `loop()`
   sleeps in `wake_wait()`; anything that changes state the loop must act on
   wakes it (step ISR: slot freed / pen block / stopped; servo ISR: pen
   arrived; HTTP: requests). A new state change without a wake is a hang —
   the host sim's tests catch these. The one wait is the sub-2µs remainder
   of the STEP pulse inside the ISR.

## Design decisions worth not re-litigating

- **USB is gone** (Oct 2026): no serial protocol, no `$` commands, no
  realtime bytes, `ARDUINO_USB_CDC_ON_BOOT=0`. Printing is web upload only.
- **G-code is the generators' subset.** Unknown-but-harmless codes are
  skipped; anything that would misplace geometry (arcs, G20, G91, G28...) or
  an out-of-bounds move stops the print. Don't add codes without a consumer.
- **A line queues ≤ `MAX_BLOCKS_PER_LINE`** (2 pen + 1 move), and the feeder
  only parses with that many slots free, so nothing waits for a slot.
- **Settings persist**, written immediately by `settings_save()` and skipped
  when the blob is unchanged — flash is ~100k erase cycles and the UI
  re-sends the whole quality preset on every toggle. The quality name lives
  in former padding of the blob, so old blobs still load.
- **Motors**: enabled directly when a job starts, released directly once
  there's no job and nothing queued; a paused job keeps holding.
- **A stop preserves motor position** (the motors are physically wherever
  they are); the planner is re-anchored to it, not to a phantom 0,0.
- **`job_service()` fills the planner** rather than one line per wake —
  lookahead needs depth to carry speed through a corner.
- **No loop watchdog**: the loop sleeps indefinitely by design.
- **Reached via the homelab, not directly.** Clients don't resolve `.local`;
  `alechomelab:8000` (flake `hosts/alechomelab/services.nix`) is a
  socket-activated `systemd-socket-proxyd` to `permablot.local:80`, tailnet
  only. So the firmware publishes an IPv4 mDNS A record and nothing else —
  no IPv6, no DNS-SD. The hostname is hardcoded on both sides.
- **CPU runs 80MHz when fully idle**, 240MHz before any motion
  (`stepper_wake()`): the step ISR's timing assumes `F_CPU`. WiFi is in
  modem-sleep except while a page holds /events open (event-driven).
- **No `String` in the job feed path.** Tens of thousands of lines × 2 heap
  allocs, competing with WiFi for a ~320KB heap.

## Toolchain: why we're still on the legacy timer driver

`steptimer.cpp` and `penservo.cpp` use `driver/timer.h`, superseded by
`gptimer` in IDF 5.0. **This is a considered decision, not an oversight.**

- `platform = espressif32` pins Arduino core 2.0.17 / **IDF 4.4**, where
  `gptimer` does not exist. There's no in-place fix.
- Espressif's 5.x migration guide keeps the legacy driver available with **no
  announced removal date**, and none of the 12 timer functions this project
  calls are marked `__attribute__((deprecated))` — only `timer_spinlock_*`
  and `timer_group_intr_*` are, and we call neither. The build is warning-free.
- Migrating means the `pioarduino` fork (core 3.x / IDF 5.x). That fork is
  mature and widely used, but it drags in `EEPROM`→`Preferences` and
  ESPAsyncWebServer API drift *at the same time* as rewriting the step ISR —
  the code with the IRAM/no-FPU constraints above.

**Revisit when something pulls you**: needing a newer WiFi stack, a security
fix, or a library that requires core 3.x. Then do it as one motivated,
bench-tested migration. Pin an exact release tag, not `stable`.

## Known-unfixed

- **Feed hold decelerates across blocks but parks at pen blocks.** Job Stop
  uses it, so it decelerates before flushing the planner.
- **Soft limits are enforced** (`$20`, default on) in `plan_buffer_line()` —
  the single choke point all motion routes through. An out-of-range move
  stops the print rather than being skipped. Limits are on **machine**
  coordinates (0,0 = pen position at power-on), so a `G92` origin set far
  across the bed legitimately shrinks the drawable area.

## Build / verify

```sh
pio run              # compile (must be warning-free; -Wall on src/ only)
pio run -t upload    # firmware
pio run -t uploadfs  # web UI from data/ → LittleFS
nix shell nixpkgs#gcc -c test/host/run.sh   # host simulation, all tests
```

`src/wifi_config.h` is required and gitignored (hard `#error` if missing) —
the real one holds credentials, never overwrite it. `pio`: `nix shell
nixpkgs#platformio -c pio run`.

`test/host/` is the test suite: it compiles the real firmware modules against
stubs (`test/host/stub/`) and simulates time, both timers, pins, flash and
the loop's wake-ups, checking every pin edge and each job end to end. It
can't model real cycle timing inside an ISR, so bench-testing on hardware is
still the last word.
