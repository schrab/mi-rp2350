# AGENTS.md

Guide for coding agents working in this repository.

## What this repo is

The rack synth monorepo: **`rack-rp2350/`** — one unified RP2350 firmware
running five live-switchable modules (Elements, Clouds, Plaits 98-engine,
Rings MiniElements, LIRA-8) on the **NoLogo Mini USB RP2350** board (the
"RP2350 nano" — see [`docs/rp2350-board.md`](docs/rp2350-board.md)) +
PCM5102A I2S DAC,
with native USB MIDI device, DIN MIDI, an ESP32-C3 UI link
(**`rack-uimcu/`**) and a WS2812 status LED on GP22 — plus **`tools/`**
(browser control panels, SysEx converter) and **`docs/`**.

**The vendored DSP libs in `rack-rp2350/lib/` are the source of truth.** They
are edited directly. There is NO patch/sync layer: upstream Mutable
Instruments code is frozen (it will never update), the pristine upstream
trees and the old sync script were removed 2026-11-18 (git history keeps
everything — the pristine sources live in history if a diff is ever needed).
Do not regenerate or "sync" the libs, and do not reintroduce patches.

`RACK_BOOT_MODULE` picks the boot module (0 elements / 1 clouds / 2 plaits /
3 rings / 4 lira8 — the env boots LIRA-8); new modules continue the ids
(5, 6, ...): add the adapter to `src/`, extend the registry/window tables,
bump the define in `platformio.ini`. Any module is reachable at runtime via
MIDI CC 32 or the serial console.

## Document map

| Document | What it holds |
|---|---|
| [`.superpowers/sdd/progress.md`](.superpowers/sdd/progress.md) | **The ledger** — task status, commit hashes, and the authoritative investigation findings (build quirks, TinyUSB API facts, reverb root cause, FX chain law). Read this before debugging anything |
| [`docs/PORTING-HIGHLIGHTS.md`](docs/PORTING-HIGHLIGHTS.md) | Distilled porting experience: SRAM/XIP law, build rules, profiling pattern, acceptance checklist (its sync/patch sections are historical — that layer is gone) |
| [`docs/rp2350-board.md`](docs/rp2350-board.md) | **Board reference** — NoLogo Mini USB RP2350 pinout (edge/Qwiic/pads), peripheral inventory, crystal/VREF/flash facts, and how the rack uses each pin |
| [`rack-rp2350/README.md`](rack-rp2350/README.md) | **Rack firmware docs**: five modules, chain mode, SRAM pool, wiring, MIDI/CC maps, console, soak checklist |
| [`rack-uimcu/README.md`](rack-uimcu/README.md) | ESP32-C3 UI firmware: pages, gestures, link protocol, build/flash |
| [`tools/rack_panel.html`](tools/rack_panel.html) | Browser control panel for the rack build: CC 32 module switching (tabs), all CC maps, mod-engine bench mux (CC 33/34/35), optional Web Serial console |
| [`tools/elements_panel.html`](tools/elements_panel.html) / [`tools/clouds_panel.html`](tools/clouds_panel.html) | Legacy single-module Web-MIDI panels (kept for reference) |
| [`tools/syx2array.py`](tools/syx2array.py) | SysEx → C array converter |
| `docs/superpowers/plans/` | Historical SDD plans (T1–T6, warps, combined) — context for the ledger |

## Project layout

```
rack-rp2350/
├── platformio.ini        # ONE env: rack (RACK_BOOT_MODULE=4 → boots LIRA-8)
├── memmap_rack.ld        # unified two-slot hot-code window + RACK_LMA region
├── src/
│   ├── main.cpp          # core 0 control + core 1 audio/swap/chain pipeline
│   ├── rack_arena.h      # unified 331,776 B SRAM pool + bump allocator
│   ├── rack_window.h     # flash -> SRAM hot-code window copy
│   ├── module_*.h        # five module adapters (elements/clouds/plaits/rings/lira8)
│   ├── rings_dist.h      # shared 5-algo distortion (Rings + Elements pre-reverb insert)
│   ├── lira8/            # hand-ported LIRA-8 engine (pure C++, host-testable)
│   ├── ui_link.h         # ESP32-C3 UI link (non-blocking TX ring)
│   ├── preset.h          # T4a presets / autosave
│   └── ...
├── lib/                  # THE SOURCE OF TRUTH — vendored MI libs, edited directly:
│                         # elements (incl. warps) clouds plaits rings stmlib
│                         # cmsis_dsp lira8_engine pio_usb
└── test/                 # host test harness (plain g++, no MCU headers; run in WSL)

rack-uimcu/               # ESP32-C3 UI firmware (ESP-IDF, tools/build_uimcu.bat)
```

## Build, flash, monitor

### Build

```sh
# Rack (ACTIVE target, five modules, one unified firmware)
pio run -d rack-rp2350
# → rack-rp2350/.pio/build/rack/firmware.uf2

# Host tests (no g++ on Windows — run in WSL)
cd rack-rp2350/test && make test
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
- Do **not** flash the C3 by copying a UF2 to that drive; C3 updates are
  `idf.py` (Espressif USB-JTAG by-id):
  `cd rack-uimcu && . /home/schrab/esp/esp-idf-v5.5.4/export.sh && idf.py -p <C3-ACM> flash`.
  On Windows use `tools/build_uimcu.bat` (MSYSTEM/idf.py trap — see memory).
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
