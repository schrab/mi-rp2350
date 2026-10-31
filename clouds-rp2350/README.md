# Clouds RP2350

Mutable Instruments **Clouds** (granular texture synthesizer) on an RP2350
"nano" board + PCM5102A I2S DAC, using the proven elements-rp2350
architecture. This is a permanent port — built to be combinable with the
other module ports (RINGS / PLAITS / ELEMENTS out → Clouds in via a block
FIFO; the hardware has no audio input jack, so standalone builds are fed by a
MIDI-triggered test exciter).

## Architecture (identical to elements-rp2350)

- Core 0: USB MIDI device (CDC serial + USB-MIDI), PIO-USB host MIDI on the
  USB-A socket, WS2812 status LED, 2 s diagnostics report.
- Core 1: audio render, 32-sample blocks @ 32 kHz (1 ms per block; the PIO-USB
  SOF alarm pool runs on core 1, isolated from core 0's I2S/TinyUSB IRQs).
- `bool core1_separate_stack = true` (see elements-rp2350 for the history).
- PIO-USB is vendored in `lib/pio_usb/`; lib/clouds + lib/stmlib are GENERATED
  by `tools/sync_pio_libs.sh` from the CLOUDS/STMLIB submodules — port deltas
  live ONLY in `tools/patches/clouds/*.patch` (currently `clouds-ram-hot.patch`),
  never hand-edited.
- **All hot DSP code and ALL lookup tables execute from SRAM** (`.time_critical`
  sections via `clouds/dsp/ram_code.h`): XIP flash cache misses cost more than
  the whole audio block budget (measured on Elements). Verified via nm: every
  hot symbol sits at 0x2000xxxx.

## Clouds specifics

- Memory partition mirrors the original F405 firmware (main SRAM + CCM):
  `large_buffer` 128 KB (mono/low-fidelity sample memory + FX workspace:
  diffuser, reverb, WSOLA correlator), `small_buffer` 64 KB (stereo 16-bit
  sample memory → 32768 samples/ch = **1.02 s @ 32 kHz**, same as hardware).
- Native rate 32 kHz = our I2S clock (no resampling in high-fidelity modes;
  low-fidelity modes run 16 kHz grains through the built-in SRC).
- `settings.cc` (STM32-flash preset persistence) is excluded from the build —
  not portable as-is, unused by the processor.
- Standalone input = test exciter: burst pluck (filtered noise, ~0.8 s decay)
  or gated sine tone; CC 11 = 0 mutes it (the chaining input slot).

## MIDI CC map

| CC | Parameter | Notes |
|----|-----------|-------|
| 14 | POSITION | |
| 15 | SIZE | |
| 16 | PITCH | |
| 17 | DENSITY | granular: overlap/deterministic-seed meta |
| 18 | TEXTURE | granular: window shape / blend meta |
| 19 | DRY/WET | |
| 20 | STEREO SPREAD | |
| 21 | FEEDBACK | |
| 22 | REVERB | |
| 23 | PLAYBACK MODE | 0 granular / 1 stretch / 2 looping delay / 3 spectral |
| 24 | QUALITY | 0 16-bit stereo, 1 16-bit mono, 2 8-bit stereo, 3 8-bit mono |
| 13 | EXCITER SOURCE | 0 burst pluck, ≥0.5 gated tone |
| 1  | EXCITER TONE | pluck brightness / tone breath |
| 11 | EXCITER LEVEL | 0 = mute input (chaining slot) |
| 65 | FREEZE | ≥64 held on, <64 released |
| 66 | TRIGGER | ≥64 = pulse (re-grab grains / looper tap) |

Notes: last-note priority stack (8 notes) like the Elements port; every
note-on also pulses TRIGGER; pitch bend rides the top note. Mode/quality/
freeze changes are applied between render blocks (pending slots).

## Diagnostics

Every 2 s, one line:

```
max Process: N us (budget 1000 us) | i2s stalls: T (+d) | host: conn=… st=… dev=… midi=… frame=…
```

Budget = 1000 µs (32-sample block @ 32 kHz). Must stay comfortably below it
(the all-SRAM build's numbers land in the ledger). Field semantics identical
to elements-rp2350.

## Build / flash / monitor

```sh
pio run -d clouds-rp2350          # → .pio/build/clouds-rp2350/firmware.uf2
```

Flash: hold **BOOTSEL** while plugging USB-C (the reliable path — the
1200-baud touch proved erratic; see the ledger), then copy the UF2 onto the
`RP2350` drive. Serial: `stty -F /dev/ttyACM0 115200 raw -echo && cat
/dev/ttyACM0`.

After submodule updates: `tools/sync_pio_libs.sh` (regenerates both projects'
libs and re-applies the patches; conflicts abort loudly).

## Status LED

Same behavior as Elements: dark idle, amber while the exciter is gated,
input/output envelopes blended in, 3 green blinks on host controller mount,
3 white self-test flashes at boot.
