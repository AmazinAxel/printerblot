# Printerblot — working notes

Blot pen plotter firmware on a **Seeed XIAO ESP32-S3** (migrated from an
RP2040 XIAO; the D0-D10 aliases hit the same pads, so the carrier board is
unchanged). Hosts its own web UI — no daemon, no connected computer.

Full architecture is in [docs/FIRMWARE.md](docs/FIRMWARE.md). This file is the
short version plus the things that bite.

## Module map

| File | Role |
|------|------|
| `main.cpp` | `setup()`/`loop()`, watchdog, idle `delay(2)` |
| `protocol.cpp` | USB line buffer + realtime bytes (`?` `!` `~` `0x18`) |
| `gcode.cpp` | tokenizer, modal state, `$` settings, M-codes |
| `arc.cpp` | G2/G3 → line segments (0.5mm floor) |
| `planner.cpp` | 16-block ring, junction deviation, trapezoids |
| `stepper.cpp` | step ISR: Bresenham + velocity, **integer only** |
| `steptimer.cpp` | timer group 0 wrapper, IRAM |
| `penservo.cpp` | timer group 1, manual 50Hz pulse (see below) |
| `job.cpp` | streams `/job.gcode` into the parser, power management |
| `webui.cpp` | WiFi, async HTTP, SSE, upload |
| `settings.cpp` | EEPROM-emulation persistence, debounced |
| `kinematics.h` | CoreXY forward/inverse + `motor_load_for_unit_dir` |

Host tooling: `tune.py` (speed/accel tuning over USB), `tools/pdf2gcode.py`
(offline PDF→gcode; the browser normally does this via `convert.js` served
from amazinaxel.com, **not** vendored here).

## Hard constraints — violating these crashes or stutters the plot

1. **No floating point in the step ISR.** Xtensa FreeRTOS allocates the FPU
   lazily and doesn't save coprocessor state across interrupts → EXCCAUSE 4
   the moment another task owns the FPU. Hence the hand-rolled `isqrt()`.
   This crashed mid-print when the network stack was busy.
2. **No `digitalWrite` / `delayMicroseconds` / `gpio_set_level` in the ISR.**
   All live in flash; an `ESP_INTR_FLAG_IRAM` handler panics with "Cache
   disabled but cached memory region accessed" as soon as anything writes to
   flash. Use `isr_pin_write()` / `isr_delay_us()` from `config.h`.
3. **Everything the step ISR reaches must be `IRAM_ATTR`** — including
   `plan_get_current_block()` and `plan_discard_current_block()`.
4. **The pen servo can't use PWM.** It's on D6 = GPIO43 (UART0); the LEDC
   matrix can't route there, so `Servo`/ESP32Servo hangs in `attach()`.
   `penservo.cpp` bit-bangs 50Hz from a hardware timer instead. There is
   deliberately **no servo library dependency**.
5. **CoreXY: pure X/Y are the *fastest* directions, ±45° the slowest** —
   inverse of a Cartesian machine. `$112`/`$122` are motor-space caps and
   bind first on diagonals (`$112`/√2).

## Design decisions worth not re-litigating

- **GRBL sender compatibility was deliberately removed** (Oct 2025). No
  welcome banner, no `$I`/`$G`/`$J`/`$X`/`$H`/`$C`/`$N`/`$SLP`, no `$0-$32`
  stubs, no jog-cancel, no `Z` field in `MPos`. The only consumer is
  `tune.py`, which needs exactly: `ok\r\n`, `error:<n>\r\n`, `$$` dump, and
  `<State|MPos:x,y|FS:f,0>`. Don't re-add GRBL surface without a consumer.
- **Settings persist**, flushed lazily (500ms debounce + stepper idle) and
  skipped when the blob is unchanged — flash is ~100k erase cycles and the UI
  re-sends the whole quality preset on every toggle.
- **Motors auto-release when idle**, auto-enable on motion. The enable is
  *queued through the planner*, never written directly — parsing runs far
  ahead of execution, so a direct write re-energises during an M18 drain.
- **Soft reset preserves motor position** (the motors are physically wherever
  they are); the planner is re-anchored to it, not to a phantom 0,0.
- **`job_service()` fills the planner** rather than one line per loop —
  lookahead needs depth to carry speed through a corner.
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
  the code with the IRAM/no-FPU constraints above — with no test suite and no
  runtime verification available here.

**Revisit when something pulls you**: needing a newer WiFi stack, a security
fix, or a library that requires core 3.x. Then do it as one motivated,
bench-tested migration. Pin an exact release tag, not `stable`.

## Known-unfixed

- **Feed hold decelerates across blocks but parks at sync blocks** (pen,
  dwell). Job Stop uses it, so it decelerates before flushing the planner.
- **Soft limits are enforced** (`$20`, default on) in `plan_buffer_line()` —
  the single choke point all motion routes through. Arcs are bounds-checked
  as a whole *before* any segment is queued, using the exact arc bounding box
  (endpoints + whichever axis crossings the sweep reaches), so a rejected arc
  draws nothing rather than stranding the pen mid-curve. Out-of-range moves
  return `error:15`; in a job that aborts the print rather than silently
  skipping lines. Limits are on **machine** coordinates, so a `G92` origin
  set far across the bed legitimately shrinks the drawable area.

## Build / verify

```sh
pio run              # compile (must be warning-free; -Wall on src/ only)
pio run -t upload    # firmware
pio run -t uploadfs  # web UI from data/ → LittleFS
```

`src/wifi_config.h` is required and gitignored (hard `#error` if missing).
There is **no test suite** — it was deleted along with the RP2040 port, so
`pio run` plus bench-testing on hardware is the only verification. No
`python3` on this NixOS box by default, so the Python tools can't be run here.
