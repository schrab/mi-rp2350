# Rings RP2350

Mutable Instruments **Rings** — the lylepmills **"MiniElements" fork**
(`EURORACK-STEREO/rings`, stock Rings DSP + internal Elements-style exciters +
a string-synth easter egg) — on an RP2350 "nano" board + PCM5102A I2S DAC,
using the proven plaits-rp2350 architecture. MIDI-driven synth — **no input
jack needed**: `internal_exciter = true` makes rings self-excite (pulse/pluck/
FM envelope/bow/blow/strike), so it is fully standalone like the Plaits port.

## Architecture (identical to plaits-rp2350)

- Core 0: USB MIDI device (CDC serial + USB-MIDI), PIO-USB host MIDI on the
  USB-A socket, WS2812 status LED, 2 s diagnostics report.
- Core 1: audio render, **24-sample blocks @ 32 kHz** (750 µs per block; the
  PIO-USB SOF alarm pool runs on core 1, isolated from core 0's I2S/TinyUSB
  IRQs).
- `bool core1_separate_stack = true` (see elements-rp2350 for the history).
- PIO-USB is vendored in `lib/pio_usb/`; lib/rings + lib/stmlib are GENERATED
  by `tools/sync_pio_libs.sh` from the EURORACK-STEREO/STMLIB submodules —
  port deltas live ONLY in `tools/patches/rings/*.patch`, never hand-edited.
- Two engines, exactly ONE renders per block (the original firmware
  contract, `rings.cc:88-127`): `rings::Part` (modes 0–3) and
  `rings::StringSynthPart` (mode 4 easter egg), both `Init()`-ed over one
  caller-owned 64 KB `reverb_buffer[32768]` — safe because the modes are
  mutually exclusive.
- The Strummer/OnsetDetector front-end of the original is dropped; note-ons
  drive `strum` directly (`internal_strum = false`).
- Native rate 32 kHz = our I2S clock (`rings/dsp/dsp.h kSampleRate` — a
  compile-time constant, this fork has no rate setter); `kMaxBlockSize = 24`;
  budget **750 µs/block** (multiplied-first in the report).
- Hot DSP code/LUTs move to SRAM via `.time_critical` sections (lib patches,
  T3) — XIP flash execution cost more than the whole block budget on
  Elements. The MiniElements blow/strike sample ROMs (~338 KB `smp_*` in
  resources.cc) stay flash-resident on purpose.

## Rings specifics

- **Per-block contract** (copied from `rings.cc:92-121`): every block builds
  a FRESH `Patch` + `PerformanceState` from the shared MIDI state, renders
  the mode-selected Part, then `Clip16(out/aux × 32768)` → I2S. Rings' parts
  run their own soft limiter internally (`part.cc:779`,
  `string_synth_part.cc:564`) but clipping to int16 is ours, exactly like
  the original firmware's codec write.
- **Pitch**: note stack (last-note priority, 8 deep) + pitch bend (±2
  semitones) → `performance_state.note` with `tonic = 0`, `fm = 0`
  (part.cc:701 sums `note + tonic + fm`; 69 = 440 Hz). Idle default 48 (C3).
- **Strum/gate**: every note-on holds `strum` high for exactly one render
  block (re-excite + voice rotation + easter-egg chord restart) and the gate
  stays on while any note is held (`strum_gate` sustains bow/blow via
  `EXCITER_FLAG_GATE`). Note-off releases only when the stack empties.
  **Velocity is ignored** — rings' strum is a bool, `PerformanceState` has
  no velocity field.
- **No settings persistence, no panel**: everything the CC map doesn't reach
  is fixed in `setup()`/the per-block fill (documented in `src/main.cpp`):
  `strum_hold_option = 0`, `fm_input_option = 0`, `frequency_locked = 0`,
  `locked_frequency_pot_value = 0`, input silence.

## MIDI CC map

| CC | Parameter | Notes |
|----|-----------|-------|
| 14 | STRUCTURE | 0–1 (clamped ≤ 0.9995 like the panel, cv_scaler.cc:187) |
| 15 | BRIGHTNESS | 0–1 (also the easter egg's drawbar registration) |
| 16 | DAMPING | 0–1 (also the easter egg's envelope decay/drone) |
| 17 | POSITION | 0–1; in modes 2/3 with WAVEFORM = blow/strike it becomes the exciter parameter instead (PositionRepurposed) |
| 18 | MODEL/FX | raw 0–5: resonator model in modes 0–3, FX type in mode 4 (table below) |
| 19 | POLYPHONY | CC 0–31 → 1, 32–63 → 2, 64–95 → 3, 96–127 → 4 |
| 20 | MODE | raw 0–4, clamped (table below) |
| 21 | WAVEFORM EXCITER | raw 0–2 (square/bow, impulse/blow, saw/strike — meaning per mode, table below) |
| 22 | CHORD TABLE | raw 0–2: Bryan / Jon / Joe |
| 23 | CHORD | raw chord index, clamped at fill time to the current table's size (Bryan 11, Jon 17, Joe 18 in the easter egg / 15·21·16·23 by polyphony otherwise) |

Notes: last-note priority stack (8 notes) like the other ports; every note-on
pulses strum (re-excites); pitch bend rides the top note. Model, polyphony,
mode, waveform, chord-table and chord changes are applied between render
blocks (pending slots) — never mid-`Process()`.

### MODE table (CC 20)

| # | PerformanceMode | What it does |
|---|-----------------|--------------|
| 0 | RINGS_STEREO | Classic Rings: resonator (model per CC 18), stereo out/aux |
| 1 | RINGS_WAVEFORM | Classic resonator on OUT (pickup crossfade) + the internal oscillator monitor on AUX: WAVEFORM 0/1/2 = square / impulse train / saw at the note pitch (part.cc:791-807); WAVEFORM has no effect on the resonator |
| 2 | MINI_ELEMENTS_STEREO | MiniElements: the internal exciter (WAVEFORM 0/1/2 = bow / blow / strike) drives the resonator; stereo out/aux |
| 3 | MINI_ELEMENTS_EXCITER | The raw exciter chain alone on AUX (bow / blow+tube+diffuser / strike), resonator crossfade on OUT (part.cc:808-811) |
| 4 | EASTER_EGG | String-synth chord machine (12 voices, polyphony-groups of chord notes; CC 18 selects the FX: formant / chorus / reverb / formant 2 / ensemble / reverb 2); BRIGHTNESS = registration, DAMPING = envelope decay (≥ CC 125 drones) |

### MODEL/FX table (CC 18)

Modes 0–3 (`ResonatorModel`): 0 modal · 1 sympathetic strings ·
2 string · 3 FM voice · 4 sympathetic strings, quantized chords (uses
CC 22/23) · 5 string + reverb.

Mode 4 (`FxType`): 0 formant filter · 1 chorus · 2 reverb · 3 formant
(shifted) · 4 ensemble · 5 reverb (long). The SAME byte drives both enums, so
switching modes keeps the selected index.

### WAVEFORM EXCITER overloads (CC 21)

| Mode | 0 | 1 | 2 |
|------|---|---|---|
| 0, 4 | unused | unused | unused |
| 1 | square monitor | impulse-train monitor | saw monitor |
| 2, 3 | bow (sustained) | blow (breath + tube + diffuser) | strike (sample player/particles) |

In modes 2/3 with WAVEFORM = blow or strike, **POSITION (CC 17) is
repurposed** as the exciter's tone/selection parameter by the Part itself
(`PositionRepurposed()`, part.h:133-137) — the resonator's position is forced
to a neutral value there. Strike mode reads the fork's sample ROMs (flash
resident by design).

## Diagnostics

Every 2 s, one line:

```
max Process: N us (budget 750 us) | i2s stalls: T (+d) | mode=M model=F out=X aux=Y notes=Z | host: conn=… st=… dev=… midi=… frame=…
```

Budget = 750 µs (24-sample block @ 32 kHz, computed
`1000000 × 24 / 32000` — multiply first). `mode` = applied PerformanceMode
index, `model` = applied CC 18 byte (resonator model in modes 0–3, FX type in
mode 4). `out`/`aux` are the OUT/AUX block RMS maxima since the last report
(×1000; in modes 1/3 `aux` tracks the exciter/waveform monitor). `notes` =
note-ons since boot. Field semantics otherwise identical to plaits-rp2350.
Boot log ends with `Rings engine up (…)`; `I2S up` confirms the PIO2 SM1
claim.

## Build / flash / monitor

```sh
pio run -d rings-rp2350           # → .pio/build/rings-rp2350/firmware.uf2
```

Flash: hold **BOOTSEL** while plugging USB-C (the reliable path — the
1200-baud touch proved erratic; see the ledger), then copy the UF2 onto the
`RP2350` drive. Serial: `stty -F /dev/ttyACM0 115200 raw -echo && cat
/dev/ttyACM0`.

After submodule updates: `tools/sync_pio_libs.sh` (regenerates the libs and
re-applies the patches; conflicts abort loudly).

## Known issues

- New port — per-mode CPU sweep (T3: SRAM placement of the hot resonator/
  exciter/FX code + `lut_sine`) still pending; per-mode `max Process`
  numbers land in the ledger after T3.

## Status LED

Same behavior as the other ports: dark idle, **amber while a note is held**,
OUT meter blended in turquoise, AUX meter blended in amber (max-since-report,
so the color holds through the 2 s report window), 3 green blinks on host
controller mount, 3 white self-test flashes at boot. The LED Poll takes
(gate, OUT meter, AUX meter) — rings has no exciter/resonator envelope split
(see `src/status_led.h` for the mapping note).
