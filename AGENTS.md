# AGENTS.md

Guide for coding agents working in this repository.

## What this repo is

The rack synth monorepo: **`rack-rp2350/`** — one unified RP2350 firmware
running seven live-switchable modules (Elements, Clouds, Plaits 98-engine,
Rings MiniElements, LIRA-8, Dronecaster, Benjolin) on the **NoLogo Mini USB RP2350** board (the
"RP2350 nano" — see [`docs/rp2350-board.md`](docs/rp2350-board.md)) +
PCM5102A I2S DAC,
with native USB MIDI device, DIN MIDI, an ESP32-C3 UI link
(**`rack-uimcu/`**) and a WS2812 status LED on GP22 — plus **`tools/`**
(browser control panels, SysEx converter, the flash/capture bench scripts)
and **`docs/`**. A **Pico SDK build** (**`rack-pico/`**, benjolin-first
scaffold) is replacing the Arduino/PlatformIO stack — see its README; the
engines are included from `rack-rp2350/src` (the single source of truth),
so the DSP work stays build-agnostic during the migration.

**The vendored DSP libs in `rack-rp2350/lib/` are the source of truth.** They
are edited directly. There is NO patch/sync layer: upstream Mutable
Instruments code is frozen (it will never update), the pristine upstream
trees and the old sync script were removed 2026-11-18 (git history keeps
everything — the pristine sources live in history if a diff is ever needed).
Do not regenerate or "sync" the libs, and do not reintroduce patches.

`RACK_BOOT_MODULE` picks the boot module (0 elements / 1 clouds / 2 plaits /
3 rings / 4 lira8 / 5 dronecaster / 6 benjolin — the env boots Benjolin);
new modules continue the ids (7, 8, ...): add the adapter to `src/`, extend the
registry/window tables,
bump the define in `platformio.ini`. Any module is reachable at runtime via
MIDI CC 32 or the serial console. The benjolin's parameter spec table lives
in `src/module_benjolin.h` (kCcSpecs — the exact norns benjolis.lua
ControlSpecs; per-parameter ranges/curves/defaults come from there, never
invented).

## PicoCollider submodule

`PicoCollider/` (git submodule, https://github.com/schrab/PicoCollider.git) is
the standalone library of reusable, reference-exact SuperCollider DSP building
blocks extracted from the Dronecaster porting work. It owns 26 header-only
kit headers and a 129-reference host gate (`cd PicoCollider/test && make test`).
Future Pico2 projects should consume it as the DSP foundation. The firmware and
host tests now include the submodule headers directly (see the `-I` roots in
`platformio.ini` and `rack-rp2350/test/Makefile`) — the only remaining headers
under `rack-rp2350/src/dronecaster/` are the dronecaster-specific engine, arena
and hot-code token.

## Document map

| Document | What it holds |
|---|---|
| [`docs/HANDOFF-pico-phase2-2026-11-23.md`](docs/HANDOFF-pico-phase2-2026-11-23.md) | **The Pico SDK migration handoff** — the rack firmware on the Pico SDK + CMake, all 7 modules ear-accepted; the bring-up root causes (the core-1 harness, the i2s autopull, the I2S format, the silent-after-swap, the staging-ring drain), the phase-2 rig plan, the machine facts |
| [`docs/HANDOFF-benjolin-2026-11-22.md`](docs/HANDOFF-benjolin-2026-11-22.md) | **The benjolin handoff** — what landed, the A/B rig usage, the device findings, the open items (the comparator/filter/budget/Pico migration), the gotchas |
| [`PicoCollider/`](PicoCollider/README.md) | **Reusable SC DSP kit** (git submodule): 26 header-only UGens + a host gate (141 refs). Source of truth for all `sc::*` ports; firmware/host include it via `-I` roots. |
| [`docs/ASSESSMENT-benjolin-port-2026-11-22.md`](docs/ASSESSMENT-benjolin-port-2026-11-22.md) | **The benjolin port assessment** — the pre-port audit (§1–§8) and the T6 device findings (§9: the port matches the NRT golden; norns is the live reference; the comparator/filter/budget plan) |
| [`docs/superpowers/plans/2026-11-22-benjolin-port.md`](docs/superpowers/plans/2026-11-22-benjolin-port.md) | **The benjolin port plan** — the 6-task implementation plan the benjolin work executed |
| [`.superpowers/sdd/2026-11-22-benjolin-port/progress.md`](.superpowers/sdd/2026-11-22-benjolin-port/progress.md) | **The benjolin ledger** — task status, commit hashes, rulings, and the golden-arbitration findings (the DPW/Filter2Pole next_a laws, the current-block filter freq, the CC-domain bugs). Read before debugging the benjolin |
| [`.superpowers/sdd/progress.md`](.superpowers/sdd/progress.md) | **The master ledger** — task status, commit hashes, and the authoritative investigation findings (build quirks, TinyUSB API facts, reverb root cause, FX chain law). Read this before debugging anything |
| [`docs/PORTING-HIGHLIGHTS.md`](docs/PORTING-HIGHLIGHTS.md) | Distilled porting experience: SRAM/XIP law, build rules, profiling pattern, acceptance checklist (its sync/patch sections are historical — that layer is gone) |
| [`docs/rp2350-board.md`](docs/rp2350-board.md) | **Board reference** — NoLogo Mini USB RP2350 pinout (edge/Qwiic/pads), peripheral inventory, crystal/VREF/flash facts, and how the rack uses each pin |
| [`rack-rp2350/README.md`](rack-rp2350/README.md) | **Rack firmware docs**: modules, chain mode, SRAM pool, wiring, MIDI/CC maps, console, soak checklist |
| [`rack-pico/README.md`](rack-pico/README.md) | **The Pico SDK build** (benjolin-first scaffold replacing Arduino/PlatformIO): CMake, the PIO I2S out, the pin map, the scope, the DSPi reference |
| [`rack-uimcu/README.md`](rack-uimcu/README.md) | ESP32-C3 UI firmware: pages, gestures, link protocol, build/flash |
| [`tools/rack_panel.html`](tools/rack_panel.html) | Browser control panel for the rack build: CC 32 module switching (7 tabs), all CC maps, mod-engine bench mux (CC 33/34/35), optional Web Serial console |
| [`tools/flash_rp2350.py`](tools/flash_rp2350.py) | **Windows-side RP2350 flash script**: the 1200-baud CDC touch → the BOOTSEL drive → the UF2 copy (the boards attach to Windows via usbipd; see the flash section) |
| [`tools/bjdump_capture.py`](tools/bjdump_capture.py) | The benjolin device-output capture (the CC injection + the CDC dump) for the device-vs-host comparison |
| [`tools/elements_panel.html`](tools/elements_panel.html) / [`tools/clouds_panel.html`](tools/clouds_panel.html) | Legacy single-module Web-MIDI panels (kept for reference) |
| [`tools/syx2array.py`](tools/syx2array.py) | SysEx → C array converter |
| `docs/superpowers/plans/` | Historical SDD plans (T1–T6, warps, combined, benjolin) — context for the ledger |

## Project layout

```
rack-rp2350/
├── platformio.ini        # ONE env: rack (RACK_BOOT_MODULE=6 → boots BENJOLIN)
├── memmap_rack.ld        # unified two-slot hot-code window + RACK_LMA region
├── src/
│   ├── main.cpp          # core 0 control + core 1 audio/swap/chain pipeline
│   ├── rack_arena.h      # unified 331,776 B SRAM pool + bump allocator
│   ├── rack_window.h     # flash -> SRAM hot-code window copy
│   ├── module_*.h        # seven module adapters (elements/clouds/plaits/rings/lira8/dronecaster/benjolin)
│   ├── benjolin/         # benjolin engine (reference-exact \benjolis port, host-testable)
│   ├── rack_dist.h       # shared 5-algo distortion (Rings + Elements pre-reverb insert; incl. the Swedish Chainsaw)
│   ├── lira8/            # hand-ported LIRA-8 engine (pure C++, host-testable)
│   ├── ui_link.h         # ESP32-C3 UI link (non-blocking TX ring)
│   ├── preset.h          # T4a presets / autosave (blob v6: 7 modules)
│   └── ...
├── lib/                  # THE SOURCE OF TRUTH — vendored MI libs, edited directly:
├── lib/                  # THE SOURCE OF TRUTH — vendored MI libs, edited directly:
│                         # elements (incl. warps) clouds plaits rings stmlib
│                         # cmsis_dsp lira8_engine pio_usb
└── test/                 # host test harness (plain g++, no MCU headers; run in WSL)

rack-pico/                # the Pico SDK build (benjolin-first scaffold replacing
                          # Arduino/PlatformIO; CMake, the PIO I2S out, USB audio
                          # capture is the next item — see rack-pico/README.md)
rack-uimcu/               # ESP32-C3 UI firmware (ESP-IDF v6.1; the boards attach
                          # to Windows via usbipd — see the flash section)
```

## Build, flash, monitor

### Build

```sh
# Rack (ACTIVE target, seven modules, one unified firmware)
pio run -d rack-rp2350
# → rack-rp2350/.pio/build/rack/firmware.uf2

# Pico SDK build (the benjolin-first scaffold replacing Arduino/PlatformIO)
export PATH=~/.local/bin:$PATH   # ninja (pip install --user ninja)
PICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk \
  cmake -B rack-pico/build -S rack-pico -DPICO_BOARD=pico2 && \
  cmake --build rack-pico/build
# → rack-pico/build/rack_benjolin.uf2

# Host tests (no g++ on Windows — run in WSL)
cd rack-rp2350/test && make test
# The benjolin gates (run the binary individually — see the ledger's canary note):
cd rack-rp2350/test && make test_benjolin && ./build/test_benjolin
```

### RP2350 flash — 1200-baud touch → BOOTSEL mass storage → UF2

**/dev/ttyACM* numbering is NOT stable.** Two USB devices can be attached:
the RP2350 app CDC and the ESP32-C3's USB-Serial-JTAG (the C3 can take
`ttyACM0` and push the RP2350 to `ttyACM1`, or vice versa). Always resolve
the board by `/dev/serial/by-id/`, never by number:

- RP2350: `usb-Raspberry_Pi_Rack_RP2350_<serial>-if00`
- C3: `usb-Espressif_USB_JTAG_serial_debug_unit_<mac>-if00`

```sh
# Run from the repo root.
RP=$(readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Rack_RP2350_*-if00)
echo "RP2350 port: $RP"

# 1) The 1200-baud touch wedges while ANY process holds DTR. Kill readers
#    first (a stray `cat` splits the byte stream and blocks the reboot).
fuser -v "$RP" 2>/dev/null || true

# 2) 1200-baud touch -> reboots into BOOTSEL (no button needed).
#    hupcl is REQUIRED: without it, close leaves DTR asserted, the app never
#    sees the 1200-baud DTR drop, and nothing reboots (silent no-op).
stty -F "$RP" 1200 raw -echo hupcl

# 3) Wait for the RP2350 mass-storage drive (label RP2350, usually sdb1)
for i in $(seq 1 20); do
  DEV=$(lsblk -no NAME,LABEL | awk '$2=="RP2350"{print "/dev/"$1; exit}')
  [ -n "$DEV" ] && break
  sleep 1
done
echo "BOOTSEL drive: $DEV"

# 4) Mount if not auto-mounted, copy the UF2, sync, unmount
MOUNT=$(lsblk -no MOUNTPOINT "$DEV")
if [ -z "$MOUNT" ]; then
  udisksctl mount -b "$DEV"
  MOUNT=$(lsblk -no MOUNTPOINT "$DEV")
fi
[ -n "$MOUNT" ] || { echo "mount failed"; exit 1; }
cp "rack-rp2350/.pio/build/rack/firmware.uf2" "$MOUNT"/
sync
udisksctl unmount -b "$DEV"

# 5) The board reboots into the app; the ACM number may change again —
#    re-resolve by-id for the console.
```

If `stty` does not reboot the board (seen when the CDC task is busy / the
port is stale), use the equivalent pyserial touch. The DTR drop must actually
reach the device — if the ioctl raises `[Errno 71] Protocol error`, check that
the device *re-enumerated* (device number / product string changes to
"RP2350 Boot") before assuming it worked:

```sh
python3 - <<'PY'
import glob, serial, time
p = glob.glob('/dev/serial/by-id/usb-Raspberry_Pi_Rack_RP2350_*-if00')[0]
s = serial.Serial(p, 1200, timeout=0)
time.sleep(0.2)
s.dtr = False          # 1200 baud + DTR drop = bootloader reset
time.sleep(0.2)
s.close()
PY
```

Rules:

- Copy **firmware.uf2** only — the BOOTSEL drive does not accept `.bin`.
- The boards attach to **Windows** via usbipd (`usbipd.exe attach --wsl
  --busid <id>` — the RP2350 must be Shared first; the C3's JTAG is
  pre-shared). The Windows-side flash script (COM ports + the drive letters)
  avoids the WSL port juggling entirely:
  `python \\wsl.localhost\Ubuntu\home\ubuntu\mi-rp2350\tools\flash_rp2350.py
  \\wsl.localhost\...\rack-rp2350\.pio\build\rack\firmware.uf2 COM15`
  (run from WSL via cmd.exe interop; see the script header).
- Do **not** flash the C3 by copying a UF2 to that drive; C3 updates are
  `idf.py` (Espressif USB-JTAG, the C3 attaches to WSL as /dev/ttyACM*):
  `cd rack-uimcu && . /home/ubuntu/esp-idf-v6.1/export.sh && idf.py -p
  /dev/ttyACM0 flash`. The sdkconfig is on esp-idf **v6.1**.
- After a BOOTSEL flash the first console command can be garbled by stale
  bytes; send a newline (or retry `status`) before trusting the reply.
- `stty` needs the port opened exclusively: one reader per port, ever.

### Monitor (RP2350 console)

```sh
RP=$(readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Rack_RP2350_*-if00)
stty -F "$RP" 115200 raw -echo
cat "$RP"     # boot log + 2 s report; kill stray readers with pkill cat
```

The 2 s report while standalone: `max Process: N us (budget N us) | i2s
stalls: T (+d) | in=… out=… notes=… | … mod=<module> | modeng=…`; while
chained it extends the module field with `bm=` (core-0 clouds-B cost), `c0=`
(core-0 loop max) and `u=` (B-not-ready underruns). Mount/umount events and
the re-enumeration watchdog print their own lines.

Desktop MIDI check (PipeWire holds raw ports, so avoid `amidi -p hw:…`): use
`aseqdump -l` to confirm "Rack RP2350" and the host controller appear.

### Bench MIDI send (2026-11-22)

- `aseqsend` silently no-ops on this bench (PipeWire holds the raw ports);
  send tests via `aplaymidi -p <port> file.mid` with a generated SMF. Port
  from `aplaymidi -l` / `aseqdump -l` ("Rack RP2350", e.g. `24:0`).
- SMF rules: exactly **one delta per event** (a delta never stands alone).
  A hold delta plus a per-event delta on the same note-off = double delta →
  aplaymidi fails with the misleading `invalid MIDI data (offset EOF+1)`
  (the offset is one past EOF; the real misalignment is earlier). Correct
  burst shape: `[ons…][vlq(hold)][80 n v][00 80 n2 v]…[00 FF 2F 00]` — the
  first off carries the hold delta, later offs delta 0. The MTrk length must
  include the 4-byte EOT.
- Verify receipt via the console `preset status` (`seen=NN CCs` counts only
  delivered CCs); the 2 s report's `notes=` accumulates note events.
- Report meter decode: `in=` is the **active module's output RMS ×1000**
  (`rack_meter_a`, written by each adapter's render), `out=` is the chain-B
  meter and is always 0 standalone — `out=0` does NOT mean "no audio".

## Hard-won constraints (details in the ledger)

- Init order in `setup()`: **USB host (PIO-USB) before `i2s.begin()`** —
  PIO-USB hard-claims all of PIO0 (3 SMs, 32 instruction slots, DMA0); the
  status LED then claims PIO2 SM0 and I2S lands on PIO2 SM1. Reordering panics
  at boot.
- f_cpu **must** be a multiple of 12 MHz (240/264/312 MHz) if PIO-USB host is
  enabled — there is a runtime guard that prints an error. Current build runs
  320 MHz with `RACK_ENABLE_USB_HOST=0` (VREG 1.20 V, see platformio.ini).
- The earlephilhower core appends `-Os` after project flags, silently defeating
  project `-O3`; the DSP libs get `-O3` via per-library `library.json` files
  in `lib/` (plain files — edit them directly).
- TinyUSB host MIDI API is **index-based** (`tuh_midi_itf_get_index`,
  `tuh_midi_packet_read(idx, …)`), packets are full 4-byte USB-MIDI event
  packets `[cable<<4|CIN, status, d1, d2]` — same entry point for device and
  host paths (`midiHandleEventPacket()`).
- TinyUSB gives up on enumeration after 3 failed control-transfer attempts and
  never retries; the re-enumeration watchdog + `main.cpp` force a reconnect
  when the bus is idle but nothing is mounted (controllers that power-cycle
  while attached).
- `Serial` is USB CDC and tolerates concurrent MIDI/host traffic — use it
  for all diagnostics. **Address it via `/dev/serial/by-id/`**: ttyACM
  numbering is not stable and the C3's USB-JTAG can take ACM0.
- Hot per-sample DSP must run from **SRAM, not flash**: XIP cache misses cost
  more than the entire block budget. Mark hot functions/data with
  `ELEMENTS_RAM_CODE(token)` / `PLAITS_RAM_CODE` / `WARPS_RAM_CODE`
  (`lib/*/…/ram_code.h`); each token must be unique (shared section names
  collide between inline/COMDAT and plain copies).
- `FxEngine<size>` (fx_engine.h) wraps delay accesses with `& (size-1)` — the
  size MUST be a power of two, or use `WrapAddress()` (in place since
  2026-11-17; the non-pow2 21632 reverb engine flanged before the fix).
- FX chain law (user): **the reverb is the LAST fx** — shapers run BEFORE it
  via `Part::set_pre_reverb_insert` (rack lib). Boot defaults: D DRIVE 0
  (zero-cost bypass), FX TYPE 0 = OFF.
- USB host socket wiring: D+→GP14, D−→GP15 (PIO-USB DPDM), GND→GND; VBUS→5V is
  optional (the SMC-PAD controller is self-powered and works without it).
- USB host zero-ACK fix: the JieLi/BR25 pad leaks bias on D− (~2.8 V idle) and
  enumeration dies with EP0 zero-ACK; cure = **15k pulldowns on D+ AND D−**
  (SMD pads at the socket). Without them the pad half-boots and crashes on bus
  resets/SOF. Related: core-0 stack overflow at attach was fixed separately via
  `core1_separate_stack=true`.

## Workflow

- Work happens on branch `rack-rp2350`; commits follow
  `[Add]/[Fix]/[Update]/[Remove] <scope>` style. Commit before risky work
  (bisect anchor).
- This is an SDD project: when a task lands, update the ledger
  (`.superpowers/sdd/progress.md`) with status, commit hash, and findings.
- The user tests on real hardware and reports mid-turn (e.g. "mounted", "no
  connection"); firmware acceptance requires their ears/eyes, not just a
  successful build. Never claim hardware-verified without their confirmation.
- Fix bugs by editing the code directly (lib or src). No patch files, no
  sync scripts — see "What this repo is".

## Agent behavior (user-verified preferences — apply on every machine)

The user moves between machines; the agent memory layer does not follow.
These rules were earned in this repo and bind any coding agent here:

- No time/schedule advice ("get some rest") and no anthropomorphizing
  hardware. Own failures plainly; strictly technical closings.
- Keep replies minimal and technical — the user's client may render them
  translated; never argue about rendering.
- Hardware misbehaving: run the isolating experiment first (flash
  last-known-good, retest) instead of code-reading hypotheses.
- Interactive hardware tests are armed only AFTER the user reports physical
  readiness (reboots kill armed patterns). **SAFETY: never instruct "touch X
  to GND" without checking X's current wiring** — RST is hard-tied to 3V3 on
  this hardware and a GND touch shorts the rail (caught once); reset pulses
  are only safe through the 10k series R.
- Never flash just to reboot a hung board — BOOTSEL + `picotool reboot -a`
  (bundled tool path) reboots with zero flash writes. A recurring hang gets
  instrumented ONCE (fault record + watchdog + breadcrumbs), then fixed in
  code — no multi-round diagnostic ceremony.
- Blank/dead peripheral: check the driver config math FIRST (window/offset/
  init sequence, interface mode, power domain) against the exact datasheet
  or a verbatim-fetched reference diff; accept the user's physical
  observation over measurement pattern-matching; never assume hardware
  provenance; read working reference code COMPLETELY, not summaries; commit
  diagnostic tooling as WIP.
- Crash/hang diagnostics recipe: watchdog SCRATCH-register breadcrumbs,
  per-core stage codes baked into the 2 s report, console `reboot`/`dma`.
- Stalled subagents: continue them via SendMessage when an agent id exists;
  if the id is gone, do the work directly rather than relaunching.
- `tools/` web panels: plain backgrounds, no decorative SVGs; "keyboard"
  means the computer keyboard — no on-screen pianos.
