# elements-rp2350

Standalone Mutable Instruments Elements synth voice on an RP2350 "nano" board
with a PCM5102A I2S DAC. PlatformIO project using the earlephilhower
arduino-pico core (core package version 1.60100.0).

Part of the plan in `../docs/superpowers/plans/2026-10-30-elements-rp2350.md`.

- Sample rate: 32000 Hz
- CPU: 250 MHz, `-O3`
- Core 0: USB MIDI (device via TinyUSB, host via PIO-USB on GP14/GP15)
- Core 1: `elements::Part::Process()` in 16-sample blocks -> I2S DMA

## Board ID

The plan's `board = pico2` is the Arduino core's board name; this PlatformIO
platform has no such ID. Use `board = rpipico2` (Raspberry Pi Pico 2, RP2350,
earlephilhower core default, picotool upload protocol).

## Building

```sh
pio run -d elements-rp2350
```

The first build downloads the platform and toolchain (several minutes).

## Library build strategy (verified 2026-10-30)

The ELEMENTS/STMLIB submodules double every `.cc` as an `.inc` compiled through
a single wrapper (`stmlib_all.cpp` / `elements_all.cpp`). Compiling both the
`.cc` files and the wrappers would duplicate every symbol, and the submodules
must not be modified.

**Primary strategy failed:** PlatformIO silently ignores `build_src_filter`
patterns outside the project's `src/` dir — `+<../STMLIB/src/stmlib_all.cpp>`
matched nothing (only `src/main.cpp` compiled, links failed with undefined
`elements::Part` symbols).

**Fallback strategy (chosen, working):** `../tools/sync_pio_libs.sh` copies
`STMLIB/src` -> `lib/stmlib/` and `ELEMENTS/src` -> `lib/elements/` with all
`.cc` files deleted (`.inc` duplicates, headers and the `*_all.cpp` wrappers
are kept). PlatformIO compiles the wrappers as regular project libraries via
LDF (`lib_ldf_mode = chain+`; main.cpp `#include <STMLIB.h>` /
`<ELEMENTS.h>` pulls them in). Re-run the script after submodule updates:

```sh
../tools/sync_pio_libs.sh
```

`-DUSE_TINYUSB` requires `#include <Adafruit_TinyUSB.h>` in the sketch and LDF
`chain+` (with `off`, the core maps `Serial` to the TinyUSB CDC whose symbols
live in the bundled Adafruit_TinyUSB_Arduino library that LDF then never
compiles -> undefined `Adafruit_USBD_CDC::begin` / `Serial` at link time).

`elements::Dsp::kSampleRate/kSrFactor/kIntervalCorrection` are declared as
class statics in `ELEMENTS/src/elements/dsp/dsp.h` but defined nowhere in the
library; `src/main.cpp` defines them (same definitions the plan's T3 uses).

## Uploading to /dev/ttyACM0

Verified on 2026-10-30 with the board running a factory ("Betaflight RP2350A")
firmware:

- `pio run -t upload` touches 1200 bps on the serial port and waits for a
  picotool device. The Betaflight firmware does not reboot to BOOTSEL on that
  touch, and picotool cannot open the running device (USB node is root:root,
  no picotool udev rules installed) -> upload fails with Error 249.
- The platform bundles `picotool` (2.3.0) at
  `~/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool`; no
  system package is needed. Raw USB access (used by picotool outside BOOTSEL
  and by `pio run -t upload` generally) currently requires root: add a udev
  rule granting write access to `2e8a:*` if command-line upload should work
  without sudo (optional).

**Procedure that always works (BOOTSEL / UF2 drag-drop):**

1. Build: `pio run -d elements-rp2350` -> produces
   `.pio/build/elements-rp2350/firmware.uf2`.
2. Unplug the USB-C cable.
3. Hold the board's **BOOTSEL** button, plug USB-C back in, release the button.
   The board enumerates as a USB mass-storage drive named `RP2350`.
4. Copy `firmware.uf2` onto that drive (file manager drag-drop, or
   `cp .pio/build/elements-rp2350/firmware.uf2 /run/media/$USER/RP2350/`).
   The board flashes and reboots into the new firmware.

Once our own firmware is flashed, the arduino-pico core supports
reboot-to-BOOTSEL (1200 bps touch + double-reset), so `pio run -t upload`
should work from then on — provided USB device-node write permission (udev
rule or sudo, see above).
