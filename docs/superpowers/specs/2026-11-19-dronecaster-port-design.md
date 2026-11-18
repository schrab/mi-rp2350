# Dronecaster port — design spec (2026-11-19)

Port northern-information/dronecaster to the rack as **module id 5** (`RACK_MODULE_DRONECASTER`), standalone 48 kHz first, following the LIRA-8 methodology (hand-ported pure-C++ engine, TDD against golden renders).

## Source material

Dronecaster is a Monome norns script: a SuperCollider engine
(`engine/Engine_Dronecaster.sc`, `engine/Dronecaster_SynthSocket.sc`) + 37
drone definitions (`engine/drones/*.scd`, SynthDefs) + Lua UI. License
**GPL-3.0** — consistent with the MI-derived code already vendored here; keep
per-file attribution in the ported headers.

- The engine is **monophonic**: one drone synth at a time, swapped by the
  SynthSocket with an `Env.asr`-gated "down+up" transition (release old →
  silence gap → start new; explicitly NOT a crossfade).
- SynthDefs are wrapped by the socket with fixed args: `hz`, `amp`,
  `out`, `gate`, `attack`, `release` (amp envelope = `Env.asr(attack, 1,
  release)`); controls `hz`/`amp` reach the graph as `K2A.ar` signals.
- scsynth runs 48 kHz / 64-sample blocks — **the same grid as this port**, so
  renders align sample-for-sample.
- The socket allocates a 60 s stereo float "tape delay" buffer used by 5
  drones via Buf*/Phasor. 60 s does not exist on an RP2350 (see Memory).

### Audit (all 37 files inventoried, 2026-11-19)

- ~110 distinct UGen types; frequency-weighted top ~30 (SinOsc ×117, LFNoise0
  ×70, VarLag ×48, Impulse ×45, EnvGen ×40, …) cover the bulk of drones.
- 5 **tape-buffer** drones: UNREANTH, UNEABLIN, Shields, Takita, Belong.
- 3 **FFT/spectral** drones: Nautilus, UNREANTH, UNWEALNE (out of scope).
- 12 drones use LocalIn/LocalOut feedback graphs; 3 use MouseX/MouseY
  (→ mapped to a CC or removed per drone).
- Complexity spans 1 UGen (Sine.scd) to full generative compositions
  (Eno.scd: Demand-stream sequencing, 8-saw planes, CombL "piano", DynKlank,
  MoogLadder).

## Decisions (brainstorm 2026-11-19)

| Question | Decision |
|---|---|
| Drone scope / phasing | **Phased**: kit + ~15 curated → remaining ~19 non-tape → 4 tape drones with capped buffer (UNREANTH excluded). 3 FFT drones out of scope. |
| Verification | **scsynth NRT goldens** in WSL (the lira8_ref / headless-Pd pattern), seed-matched. |
| Pitch control | **MIDI note-on retunes hz** (running drone glides), note-off ignored; HZ CC = semitone offset on top. |
| Chain mode | **Standalone only** in v1; 32 kHz chain context is a later phase. |
| Architecture | **Approach A**: hand-ported per-drone C++ graphs over a shared UGen kit (B: runtime graph interpreter, C: sclang-eval→codegen — rejected). |

## Non-goals

- Animations / scenes (classic + mountain Lua) — never enter firmware.
- FFT drones (Nautilus, UNREANTH, UNWEALNE) — revisit only if asked; CMSIS
  `arm_rfft_fast_f32` is vendored if ever needed.
- norns glue (CroneEngine/OSC), the norns recorder, midicontrol.lua.
- Chain-A participation (later phase, after the engine is stable).

## Architecture

Engine lives in `src/dronecaster/` — pure C++, no MCU headers, host-compilable
by plain g++; `-O3` via a `lib/dronecaster_engine/` wrapper (mirror of
`lib/lira8_engine`). One adapter `src/module_dronecaster.h`.

```
src/dronecaster/
├── sc_rgen.h           # exact port of scsynth's RGen LCG (seed-matched goldens)
├── sc_defs.h           # rate struct (sr, kBlock=64), fold/clip/midicps, denormal flush
├── sc_kit_osc.h        # SinOsc (SC 8192-pt sine wavetable, linear interp),
│                       #   Saw/Pulse/VarSaw (SC 2-band-limit table scheme),
│                       #   LFSaw/LFPar/LFPulse, SinOscFB, PMOsc, SawDPW/PulseDPW,
│                       #   chaos: HenonC, LorenzL, FBSineN
├── sc_kit_noise.h      # LFNoise0/1/2, LFDNoise3, White/Pink/Brown, Dust/Dust2,
│                       #   Crackle, Impulse/Trig
├── sc_kit_filter.h     # LPF/HPF/BPF/RHPF (SC biquad SOS), RLPF (Resonz),
│                       #   Butterworth family (BLowPass/BPeakEQ/BHiShelf/BAllPass),
│                       #   OnePole, LeakDC, Lag/LagUD/VarLag
│                       #   phase 2: MoogLadder, MoogFF, DFM1, SVF, FreeVerb/GVerb/
│                       #   Greyhole, Compander, Limiter
├── sc_kit_env.h        # EnvGen (asr/perc/new, curve shapes), Linen, Decay2,
│                       #   Amplitude, Peak
├── sc_kit_delay.h      # DelayN/C/L, CombN/C/L, AllpassN/C, Pluck; phase 3: tape
│                       #   BufDelay*/BufRd/BufWr/Phasor over the int16 tape carve
├── sc_kit_logic.h      # Demand + Dseq/Dxrand/Dwrand, TDelay, TChoose, CoinGate,
│                       #   SetResetFF, Changed, Latch, Select/SelectX, Clip/Fold/
│                       #   InsideOut, LinExp/LinLin, A2K/K2A, DC
├── sc_kit_spatial.h    # Pan2, Balance2, Rotate2, Splay/SplayAz, XFade2
├── sc_kit_dist.h       # SineShaper, CrossoverDistortion, Decimator, DiodeRingMod
├── sc_arena.h          # bump allocator over the drone's SRAM carve (reset per voice)
├── drone.h             # DroneSpec interface: init(sr, seeds)/process(f32 L,R)/
│                       #   buffer_bytes() + the fixed 37-entry catalog (norns order)
├── drones/drone_<name>.h   # one struct per .scd — line-for-line C++ mirror
└── dronecaster_engine.h    # SynthSocket port: mono voice, Env.asr gate, down+up
                          #   swap; hz/amp/fade controls; per-block NaN guard
```

**Faithfulness law:** every kit UGen is ported from the scsynth source of that
UGen, including the internals that decide sample accuracy (8192-pt sine
wavetable, 2× oversampled osc tables, Paul Kellet pink filter, RGen's exact
LCG constants). `ar` UGens run per-sample; `kr` UGens once per 64-sample
internal block — exactly SC's rate semantics.

**Rates / burst:** standalone 48 kHz; host grid 16 samples; engine bursts
4×16 → 64 (the proven lira8-standalone pattern: I2S bank 8×16 ≈ 2.9 ms absorbs
the 4× burst). Reported budget = the 64-sample period = 1333 µs.

**LocalIn/LocalOut** (phase 2): per-voice feedback buffer — `LocalOut` writes
at end of internal block, `LocalIn` reads at the start of the next (exact SC
block-boundary semantics).

## Rack integration

- **Window/flash**: new `tc_dronecaster` overlay member in `memmap_rack.ld` +
  `__rack_flash_dronecaster_*` LMA symbols; `rack_window.h` section table +
  symbols; `RACK_MODULE_COUNT` 5→6; `RACK_BOOT_MODULE` untouched (currently 0
  = Elements). Hot code tagged with a `DRONECASTER_RAM_CODE()` macro with
  unique section names. Phase-1 hot code expected ~20–40 KB — under both the
  118 KB window and the 58 KB chain A-slot.
- **Adapter shape**: lira8 pattern — arena-placed state, pending slots for
  CCs/notes, `render()` = one 16-sample host block (engine bursts),
  float→int16 at output. Registry entry: `sample_rate=48000, block_size=16,
  budget_us=1333, i2s_buffers=8, i2s_words=16`.
- **CC map** (avoids rack-reserved 32 / 33-35 / 64):
  - CC 51 = drone select (catalog id 0–36, norns alphabetical order; a
    not-yet-ported id is ignored with a console note — the panel only offers
    ported ids)
  - CC 52 = amp (default 0.4)
  - CC 53 = fade — attack/release time (socket `setFadeTime`)
  - CC 54 = hz offset in semitones (±), on top of notes; base hz 55 if no
    note received yet (the norns default)
  - Notes: note-on → `hz = midi→hz` with a short lag (running drone glides);
    note-off does nothing
  - PLAY button (pseudo-CC 64) = recast: release + restart current drone
  - Mod destinations: continuous CC 52/53/54; no triggers
- **Presets**: autosave drone id + amp/fade/hz-offset via the existing
  `preset.h` path.
- **Panel** (`tools/rack_panel.html`): `dronecaster` tab — labeled drone
  dropdown (ported ids only), amp/fade/hz sliders, CC map, console link.

## Golden harness (`test/dronecaster_ref/`)

- Vendor the 37 `.scd` files into `test/dronecaster_ref/drones/` (GPL-3
  attribution note) — no external checkout dependency.
- WSL: `apt install supercollider` (sclang + scsynth).
- `gen_goldens.scd`: per drone — load `.scd`, wrap exactly like
  `SynthSocket.wrapDef` (hz/amp/out/gate/attack/release + `Env.asr`), fix the
  **sclang seed** (graph-construction randomness, e.g. Eno's `Rand(0.78,1.32)`)
  and the **server RGen seed** (`s.seed_`), NRT-render 4 s @ 48 kHz (defaults
  hz=55, amp=0.4) → WAV.
- `test/test_dronecaster.cc`: same seeds on the C++ side, render 4 s, compare
  max|Δ| + RMS-of-Δ — target **sample-exact** for deterministic drones (few-LSB
  int16 tolerance for float summation order), plus NaN/denormal/RMS smoke
  checks. A drone is "ported" only when its golden passes.
- Two-seed model: sclang-side randomness is fixed by the sclang seed;
  server-side UGen noise (`LFNoise*`, `Dust`, `TChoose`, `rrand`) is fixed by
  the server seed + the exact RGen port.

## Memory

When dronecaster is active it owns the 331,776 B SRAM pool: `sc_arena`
bump-allocates per voice and resets on drone swap. Phase-1 drones need only
small delay buffers (worst case FreeVerb ≈ 64 KB). Phase-3 tape cap: **1 s
stereo int16 = 192 KB** (vs SC's 60 s float) — the honest MCU ceiling; revisit
by ear in phase 3 (mono 2 s = fallback). `static_assert` carves like lira8.

## Error handling

- Per-internal-block NaN/inf sweep on voice output → clamp + one-shot console
  line (SC lets feedback drones explode audibly; we guard).
- Budget overrun → the existing stall detector; no new machinery.
- Unknown drone id (CC 51 out of ported range) → console note, keep current.
- Tape arena exhaustion → Init falls back to a shorter cap.

## Phases (each lands on hardware + ledger)

| Phase | Content | HW flash |
|---|---|---|
| T-D1 | Kit core (RGen, osc/noise/env/delay/logic/spatial basics) + goldens infra (WSL scsynth, gen script, comparison) + Sine/Square/Triangle **bit-exact** + minimal end-to-end module (adapter/window/CC/panel/presets) | ✅ — integration proven on the trivial trio |
| T-D2 | Mid-tier kit (biquads, Resonz, Butterworth, FreeVerb, Limiter) + ~12 curated drones chosen by greedy UGen-coverage over the 37 files | ✅ |
| T-D3 | Demand family, MoogLadder/MoogFF/DFM1/SVF, LocalIn/Out feedback, PitchShift, chaos UGens + remaining 19 non-tape drones | ✅ |
| T-D4 | Tape buffer (1 s stereo int16 carve) + the 4 in-scope tape drones (UNEABLIN, Shields, Takita, Belong — UNREANTH excluded with the FFT drones) | ✅ |

## Risks / open items

- RGen LCG constants + sine wavetable must be extracted from scsynth source —
  verify against the installed WSL version.
- sclang seed determinism for graph-construction randomness needs proving
  early (T-D1): if `Rand()` at build time can't be pinned, those drones get
  statistical (not sample-exact) goldens — acceptable, decided per drone.
- Hot-code size unknown until the first full build — window ASSERTs will catch
  a regression (expected headroom is large).
- MouseX/MouseY drones: gesture X → CC 55, gesture Y → CC 56 (these CCs are
  per-module, so they don't clash with lira8's sensor bank); done per drone
  during T-D2/T-D3.
- Eno-class drones are compute-heaviest; profile at T-D3 before flashing.
