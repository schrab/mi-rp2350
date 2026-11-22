# rp2350-porttest — OpenRB-338 DSP port on a bare Pico2

Standalone Pico SDK + CMake test firmware for porting the ReBirth RB-338 2.01
audio engine (https://github.com/pingw33n/OpenRB-338) to RP2350. This project
is deliberately separate from `rack-pico/` and `rack-rp2350/` — no rack files
are changed.

## What is ported

From `~/openrb-338/crates/orb-engine/src/`:

- **TB-303** (`tb303.rs`): both engines — Vintage (control every 4 samples,
  one resonance row) and ReBirth 2.0 (control per 64-sample block, 1/32
  coefficient ramp per sample pair, two-row interpolation). Note-mode
  `key_on`/`key_off` equivalent.
- **Distortion** (`dist.rs`): 5×5 gain grid interpolation, waveshaper lookup
  with linear interpolation, and the shape-0 hard clip.
- **PCF** (`pcf.rs`): LP/BP table filters, envelope, pattern data, correct
  zero-length-pattern fault behaviour (bus passes unchanged).
- **Compressor** (`comp.rs`): per-4-sample peak detector, attack/hold/release
  envelope, `x/env` gain and 4-step ramp.

All numerics follow the engine's law: f64 arithmetic, f32 rounding exactly
where the DLL stores a float, no FMA, original operand order. `pico_double` on
RP2350 Arm links DCP-backed `__aeabi_*` routines, so the f64 hot path is
hardware-assisted. No libm is needed at runtime.

Tables are the engine's bit-exact DLL dumps from
`orb-engine/tests/fixtures/tables/*.bin`, linked as `.rodata` with `.incbin`
(`tables.S.in`): level, 303 pitch/filter/smooth, distortion shaper, PCF LP/BP
and smoother, and the 4,336-byte PCF pattern blob.

Not ported (deliberate): delay (its 512 KiB line does not fit in internal
SRAM), song/`.rbs` loading, sequencer, MIDI/UI integration.

## Hardware

Bare Pico2 (RP2350) + PCM5102A, 3-wire I2S:

| Pico | PCM5102 board |
|---|---|
| GP19 (BCK) | BCK |
| GP20 (DIN) | DIN |
| GP21 (LRCK) | RCK |

Additional DAC strapping required: **board SCK pin → GND** (selects the
internal PLL from BCK), and the four solder bridges
**H1L→L, H2L→L, H3L→H, H4L→L** (FLT, DMP, XMT/unmute, FMT=I2S).

Debug: Raspberry Pi Debug Probe on SWD; the probe is attached to WSL with
`usbipd bind/attach --wsl --busid 2-1`. PyOCD lives in
`~/.local/share/pyocd-venv`.

## Build / flash / diagnose

```sh
cd ~/mi-rp2350/rp2350-porttest
PICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk \
  cmake -B build -S . -G Ninja -DPICO_BOARD=pico2
cmake --build build

# ALWAYS check the probe first — a detached probe makes pyocd hang the shell:
~/.local/share/pyocd-venv/bin/pyocd list
# (if absent: cmd.exe /c "usbipd attach --wsl --busid 2-1")
~/.local/share/pyocd-venv/bin/pyocd flash -t rp2350 build/porttest.elf
```

SWD-readable diagnostics (`volatile` globals, read with
`pyocd cmd -t rp2350 -c "read32 <addr>"`; get addresses with `arm-none-eabi-nm`):
`g_blocks`, `g_last_cyc`, `g_max_cyc`, `g_last_us`, `g_max_us`, `g_lost`
(DMA underruns), `g_out_peak` (post-chain DSP peak).

## Measured budget (full 303 → dist → PCF → comp chain)

| Clock / VREG | cycles/block | cycles/sample | wall time/block | core load |
|---|---|---|---|---|
| 300 MHz, default VREG | 85,468 | 1,335 | 285 µs | 19.6% |
| 320 MHz, 1.25 V, QMI 4/3 | 88,688 | 1,386 | 277 µs | 19.1% |
| 320 MHz, 1.20 V, QMI 4/3 | 90,349 | 1,412 | 282 µs | 19.5% |

64 samples per block at 44.1 kHz, deadline 1,452 µs, 0 DMA underruns in every
run (10 s soak at 320 MHz / 1.20 V included). The QMI CLKDIV 4 slows XIP, so
the cycle count rises slightly while wall time improves.

## Known issue

The DSP output is healthy on target (peak ~0.09–0.3 full scale) and the I2S
DMA runs at the correct rate with zero underruns, but no sound has been
confirmed from the PCM5102 on the bench. The I2S program and pin/timing
config follow the rack's ear-proven format. DAC output/wiring needs a scope
check; this is not known to be a DSP defect.
