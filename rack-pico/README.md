# rack-pico — the active Pico SDK build

The Pico SDK/CMake build of the rack, replacing the Arduino/PlatformIO
stack. All eight modules are included from `../rack-rp2350/src` (the single
source of truth — nothing is copied). The platform layer (the I2S PIO, the
USB CDC console + MIDI + UAC1 capture, the clocks) is rewritten against the
Pico SDK. `RACK_BOOT_MODULE=7` boots the OpenRB-338 303/dist/PCF/comp chain
(module id 7, XIP); the seven original modules remain available by CC 32.
Use the system `arm-none-eabi-gcc` (13.2.1) — the PlatformIO GCC 16.1 cache
fails at link with unresolved `__retarget_lock_*`.

## Why

- Exact flag control: the earlephilhower core appends `-Os` after the
  project flags (the SRAM-law fight); CMake cannot. `-ffp-contract=off`
  matches the M33's no-FMA arithmetic for the device-vs-golden comparison.
- USB Audio becomes feasible: the DSPi project
  (github.com/WeebLabs/DSPi, GPL-3.0, Pico SDK/CMake) proves the loopback-
  capture pattern (~8.7 KB BSS: a second UAC1 function streaming the output
  slot back to the host — the device-vs-golden rig AND a user-facing
  recording feature).
- 307.2 MHz sys clock (the DSPi choice): the PIO instruction rate lands
  near-integer over the I2S bit clock at 44.1/48/96 kHz.

## Build

```sh
export PATH=~/.local/bin:$PATH   # ninja
cmake -B build-swd -S . -G Ninja -DPICO_BOARD=pico2 \
  -DPICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk \
  -DCMAKE_C_COMPILER=/usr/bin/arm-none-eabi-gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/arm-none-eabi-g++
cmake --build build-swd
# → build-swd/rack.uf2 (post-link RAM-placement gate must print VERDICT: PASS)
```

The post-link gate is `scripts/check_ram_placement.py`; host-only builds can
skip it with `-DRACK_SKIP_PLACEMENT_CHECK=ON`.

## Pin map (docs/rp2350-board.md)

| Function | GPIO |
| --- | --- |
| I2S DOUT | 8 |
| I2S BCK | 6 |
| I2S LRCLK | 7 |
| USB CDC console | the native USB |

## The boot state

The exact norns benjolis default state (amp 0 — the SILENT boot; raise amp
to hear it, like norns). The scaffold renders the default state and prints
the 2 s report (`max/avg µs`) over the USB CDC console.

## The deliberate scope (NOT in this scaffold)

USB MIDI, the module swap machinery, presets, the C3 UI link, the WS2812
LED, the USB audio loopback capture (the DSPi `DSPI_LOOPBACK` pattern is the
reference implementation), the comparator restoration (the ASSESSMENT §9.2
item 1 — the comparator/filter/arbitration work proceeds through the rig
once wired).

## The PIO I2S caveats (verify at hardware bring-up)

- The polarity/order: the program is the left-justified variant (the data
  MSB on the first BCK after the LR flip); the PCM5102A accepts LJ/I2S via
  its format strap — check the rack board's strap wiring.
- The fractional clkdiv (the instruction rate = FS × 70) is exact on
  average with ±1 sys-cycle jitter — fine for the PCM5102A; the integer-
  divider purity (the DSPi 307.2 MHz shape) is the later refinement.
