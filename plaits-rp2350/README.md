# Plaits RP2350

Mutable Instruments **Plaits** (the Rubato "Plaits Lab" fork of pichenettes'
eurorack: **98 engines, all stereo-capable**) on an RP2350 "nano" board +
PCM5102A I2S DAC, using the proven clouds-rp2350 architecture. MIDI-driven
synth voice — no input jack, no test exciter needed.

## Architecture (identical to clouds-rp2350)

- Core 0: USB MIDI device (CDC serial + USB-MIDI), PIO-USB host MIDI on the
  USB-A socket, WS2812 status LED, 2 s diagnostics report.
- Core 1: audio render, **24-sample blocks @ 48 kHz** (500 µs per block; the
  PIO-USB SOF alarm pool runs on core 1, isolated from core 0's I2S/TinyUSB
  IRQs).
- `bool core1_separate_stack = true` (see elements-rp2350 for the history).
- PIO-USB is vendored in `lib/pio_usb/`; lib/plaits + lib/stmlib are GENERATED
  by `tools/sync_pio_libs.sh` from the EURORACK-STEREO/STMLIB submodules —
  port deltas live ONLY in `tools/patches/plaits/*.patch`, never hand-edited.
- `plaits::Voice` embeds all 98 engines (`PLAITS_ENGINE_MEMBERS`); their state
  RAM is carved from one shared 32 KB workspace pool via
  `stmlib::BufferAllocator` (freed before each engine Init — the pool must
  cover the largest single engine, 16.4 KB, not the sum). The `-D` flags that
  make this legal live in `platformio.ini`: `PLAITS_ENGINE_COUNT=98`,
  `PLAITS_CORRECTED_SAMPLE_RATE=48000.0f` (true-rate I2S → pitch shim = 0),
  `PLAITS_HAS_USER_DATA_BANK=1` (DX7 banks from resources.cc, no STM32 flash).
- Hot DSP code/LUTs move to SRAM via `.time_critical` sections (lib patches,
  T3) — XIP flash execution cost more than the whole block budget on Elements.
- Native rate 48 kHz = our I2S clock; `kMaxBlockSize = 24`; budget
  **500 µs/block** (multiplied-first in the report).

## Plaits specifics

- **Stereo convention**: `patch.aux_output_option = 1` (CC 24 default) + a
  stereo-capable engine → OUT = left, AUX = right through one linked limiter
  (`voice.cc` LimitStereoInPlace). `Frame { short out; short aux }` values are
  already clipped/limited inside `Voice` — they go to `i2s.write16()` raw, no
  double-clipping.
- **Engine selection**: CC 23 (raw value 0–97) → `patch.engine` →
  `engine_quantizer_.Process(patch.engine, 0)` (hysteresis quantizer,
  `model_cv_option = 1` so no CV offset) → registry index. Engine changes are
  applied between render blocks via a pending slot; `Voice` itself reloads
  user data and resets the engine on the index change.
- **Note/pitch**: note stack (last-note priority, 8 deep) + pitch bend →
  `patch.note` in semitones (the panel FREQUENCY pot's own domain; 69 =
  440 Hz). `modulations.note` (V/OCT CV add-on) stays unpatched at 0.
- **Trigger/velocity**: every note-on holds `modulations.trigger` high for one
  render block (0.5 ms) at `0.3 + 0.7 × velocity/127` — always above Voice's
  0.3 threshold, so even velocity 1 strikes. With the default
  `trig_response_option = 2` (velocity trigger) the latched pulse voltage
  becomes the accent: `accent = trigger_velocity × compressed_level`, and the
  LPG ping scales with it.
- **Level/sustain**: CC 25 → `modulations.level` (`level_patched = true`,
  default 0.8). A held note sustains through the LPG (`compressed_level =
  1.3·L/(0.3+|L|) × velocity`); note-off drops it and the vactrol releases
  with DECAY (CC 19). Drum/self-enveloped engines bypass the LPG and use the
  accent directly.
- **MACRO (TWIST)**: CC 17 is routed as the fork's fourth synthesis macro via
  the MODEL-CV option (`model_cv_option = 1`, `macro = 0.5 +
  modulations.engine`); CC 64 = exactly the engine-stock neutral 0.5. Engines
  that have no macro ignore it.
- **Modulation amounts** (CC 20/21/22) span −1..+1 like the panel attenuverter
  pots (CC 64 = off); the internal trigger envelope modulates note/timbre/morph
  by those amounts exactly as on hardware with TRIG patched.
- No settings persistence, no panel: every `patch.option_*` field the CC map
  doesn't reach is fixed in `setup()` (documented in `src/main.cpp`).

## MIDI CC map

| CC | Parameter | Notes |
|----|-----------|-------|
| 14 | TIMBRE | 0–1 |
| 15 | MORPH | 0–1 |
| 16 | HARMONICS | 0–1 |
| 17 | MACRO / TWIST | CC 64 = engine-stock neutral; fourth synthesis macro |
| 18 | LPG COLOUR | 0–1 |
| 19 | DECAY | 0–1 (also the release time after note-off) |
| 20 | FM AMOUNT | −1..+1, CC 64 = off (panel attenuverter range) |
| 21 | TIMBRE MOD AMT | −1..+1, CC 64 = off |
| 22 | MORPH MOD AMT | −1..+1, CC 64 = off |
| 23 | ENGINE | raw value = engine index 0–97 (table below) |
| 24 | AUX OPTION | raw 0 aux model / 1 stereo (default) / 2 suboscillator |
| 25 | LEVEL | 0–1, default 0.8; accent + LPG level |

Notes: last-note priority stack (8 notes) like the Elements/Clouds ports;
every note-on pulses TRIG with velocity as the pulse voltage; pitch bend rides
the top note; held notes sustain, note-off releases with DECAY. Engine and
aux-option changes are applied between render blocks (pending slots).

### Engine table (CC 23 value → engine)

Registry order = Plaits-Lab catalog order
(`alt_firmwares/plaits_lab_catalog/catalog.json`; 63 MI + 22 Rubato Lab +
13 Community; the two `retiredEngines` variants were replaced in-catalog):

| # | Engine (catalog id) | Name | Origin |
|---|---------------------|------|--------|
| 0 | virtual-analog | Virtual Analog | MI |
| 1 | virtual-analog-dual | Virtual Analog Dual | MI |
| 2 | virtual-analog-crossfade | Virtual Analog Crossfade | MI |
| 3 | waveshaping | Waveshaping | MI |
| 4 | two-op-fm | Two-op FM | MI |
| 5 | granular-formant | Granular Formant | MI |
| 6 | harmonic | Harmonic Oscillator | MI |
| 7 | wavetable | Wavetable | MI |
| 8 | chords | Chords | MI |
| 9 | speech | Speech | MI |
| 10 | formant-speech | Speech Sounds | MI |
| 11 | lpc-speech | LPC Words | MI |
| 12 | swarm | Swarm | MI |
| 13 | filtered-noise | Filtered Noise | MI |
| 14 | particle-noise | Particle Noise | MI |
| 15 | inharmonic-string | Inharmonic String | MI |
| 16 | modal-resonator | Modal Resonator | MI |
| 17 | analog-bass-drum | Analog Bass Drum | MI |
| 18 | analog-snare | Analog Snare | MI |
| 19 | analog-hi-hat | Analog Hi-hat | MI |
| 20 | virtual-analog-vcf | VA + VCF | MI |
| 21 | phase-distortion | Phase Distortion | MI |
| 22 | dx7-bank-a | 6-Op FM Bank A | MI |
| 23 | dx7-bank-b | 6-Op FM Bank B | MI |
| 24 | dx7-bank-c | 6-Op FM Bank C | MI |
| 25 | wave-terrain | Wave Terrain | MI |
| 26 | string-machine | String Machine | MI |
| 27 | chiptune | Chiptune | MI |
| 28 | natural-speech | Natural Speech | Rubato |
| 29 | glisson | Glisson | Rubato |
| 30 | gendy | GENDY | Rubato |
| 31 | scanned | Scanned | Rubato |
| 32 | pulsar | Pulsar | Rubato |
| 33 | loopback | Loopback | Rubato |
| 34 | lockstep | Lockstep | Rubato |
| 35 | tapfield | Tapfield | Rubato |
| 36 | phase-weave | Phase Weave | Rubato |
| 37 | sideband-bank | Sideband Bank | Rubato |
| 38 | attractor | Attractor | Rubato |
| 39 | undertow | Undertow | Rubato |
| 40 | reed-pipe | Reed Pipe | Rubato |
| 41 | phase-flock | Phase Flock | Rubato |
| 42 | rulefield | Rulefield | Rubato |
| 43 | spectral-spiral | Spectral Spiral | Rubato |
| 44 | z-filter | Z Filter | MI |
| 45 | toy | Toy | MI |
| 46 | csaw | CSaw | MI |
| 47 | ring-mod | Ring Mod | MI |
| 48 | fold | Fold | MI |
| 49 | buzz | Buzz | MI |
| 50 | dual-sync | Dual Sync | MI |
| 51 | granular-cloud | Granular Cloud | MI |
| 52 | morph | Morph | MI |
| 53 | noise-bank | Noise Bank | MI |
| 54 | particle-burst | Particle Burst | MI |
| 55 | saw-square | Saw Square | MI |
| 56 | saw-swarm | Saw Swarm | MI |
| 57 | vowel | Vowel | MI |
| 58 | harmonics | Harmonics | MI |
| 59 | vosim | VOSIM | MI |
| 60 | plucked | Plucked | MI |
| 61 | blown | Blown | MI |
| 62 | struck-bell | Struck Bell | MI |
| 63 | struck-drum | Struck Drum | MI |
| 64 | kick | Kick | MI |
| 65 | snare | Snare | MI |
| 66 | cymbal | Cymbal | MI |
| 67 | wave-scan | Wave Scan | MI |
| 68 | wave-paraphonic | Wave Paraphonic | MI |
| 69 | fluted | Fluted | MI |
| 70 | question-mark | Question Mark | MI |
| 71 | virtual-analog-variant | Virtual Analog Variant | MI |
| 72 | bowed | Bowed | MI |
| 73 | sub-oscillator | Sub Osc | MI |
| 74 | digital-modulation | Digital Modulation | MI |
| 75 | saw-comb | Saw Comb | MI |
| 76 | vowel-fof | Vowel FOF | MI |
| 77 | raw-fm | Raw FM | MI |
| 78 | triple | Triple | MI |
| 79 | bytebeat | Bytebeat | Community |
| 80 | diatonic-chord | Diatonic Chord | Community |
| 81 | scale-stack | Scale Stack | Community |
| 82 | wavetable-chord | Wavetable Diatonic Chord | Community |
| 83 | wavetable-scale-stack | Wavetable Scale Stack | Community |
| 84 | shakers | Shakers | Rubato |
| 85 | brass | Brass | Rubato |
| 86 | helix | Helix | Rubato |
| 87 | skins | Skins | Rubato |
| 88 | circuit-zaps | Circuit Zaps | Rubato |
| 89 | metalwork | Metalwork | Rubato |
| 90 | clap | Analog Clap | Community |
| 91 | analog-percussion | Analog Percussion | Community |
| 92 | freshets-formant | Freshets Formant | Community |
| 93 | bubbletime | BubbleTime | Community |
| 94 | zxphase48k | ZxPhase48k | Community |
| 95 | zxpulse48k | ZxPulse48k | Community |
| 96 | acid | Acid | Community |
| 97 | chords-supersaw | Supersaw Chords | Community |

## Diagnostics

Every 2 s, one line:

```
max Process: N us (budget 500 us) | i2s stalls: T (+d) | eng=E out=X aux=Y notes=Z | host: conn=… st=… dev=… midi=… frame=…
```

Budget = 500 µs (24-sample block @ 48 kHz, computed
`1000000 × 24 / 48000` — multiply first). `eng` = active engine index (CC 23
value). `out`/`aux` are the OUT/AUX block RMS maxima since the last report
(×1000). Field semantics otherwise identical to clouds-rp2350. Boot log ends
with `Plaits engine up (…)`; `I2S up` confirms the PIO2 SM1 claim.

## Build / flash / monitor

```sh
pio run -d plaits-rp2350          # → .pio/build/plaits-rp2350/firmware.uf2
```

Flash: hold **BOOTSEL** while plugging USB-C (the reliable path — the
1200-baud touch proved erratic; see the ledger), then copy the UF2 onto the
`RP2350` drive. Serial: `stty -F /dev/ttyACM0 115200 raw -echo && cat
/dev/ttyACM0`.

After submodule updates: `tools/sync_pio_libs.sh` (regenerates the libs and
re-applies the patches; conflicts abort loudly).

## Known issues

- New port — engine-by-engine CPU sweep (worst-case per the fork's own
  STM32F373 data ≈ 30% of budget) and SRAM placement (T3) still pending;
  per-engine `max Process` numbers land in the ledger after T3.

## Status LED

Same behavior as the other ports: dark idle, **amber while a note is held**,
OUT meter blended in turquoise, AUX meter blended in amber (max-since-report,
so the color holds through the 2 s report window), 3 green blinks on host
controller mount, 3 white self-test flashes at boot.
