# rack-pico — the Pico SDK build (benjolin-first scaffold)

The Pico SDK/CMake build of the rack, replacing the Arduino/PlatformIO
stack. The DSP engines are included from `../rack-rp2350/src` (the single
source of truth — nothing is copied). The platform layer (the I2S PIO, the
USB CDC console, the clocks) is rewritten against the Pico SDK; the
remaining modules migrate here one at a time once this proves out.

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
# One-time: the SDK (pico_sdk_import.cmake clones it if PICO_SDK_PATH is
# unset; on this machine the PlatformIO-bundled SDK works):
PICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk \
  cmake -B build -S . -G Ninja
cmake --build build
# → build/rack_benjolin.uf2
```

The toolchain: `arm-none-eabi-gcc` (the PlatformIO one at
`~/.platformio/packages/toolchain-rp2040-earlephilhower/bin` is on PATH for
the pico-sdk's compiler checks if the system arm-none-eabi is absent).

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
