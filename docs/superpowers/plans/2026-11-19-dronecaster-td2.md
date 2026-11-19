# Dronecaster T-D2 Implementation Plan — the full UGen kit

> **For agentic workers:** execute task-by-task with the T-D1 verification
> discipline. Each task = one kit family in `src/dronecaster/`, host-tested
> against the cited SC 3.14.1 sources AND sample-exact against per-UGen NRT
> reference renders (`test/dronecaster_ref/ugen_ref/`). Commit per family.

**Goal:** port EVERY UGen the 34 remaining drones need (T-D2 = kit only, no
drones), so the T-D3 drone ports become mechanical `.scd` mirrors.

**Source of truth:** `/tmp/sc_full` (shallow clone, tag Version-3.14.1) and
`/tmp/sc3p_full` (sc3-plugins main, pinned commit recorded in the ledger at
port time). Cite `file:line` in every header; re-read the cited source before
changing a port (the T-D1 rule: do not "fix" from memory).

**Verification law (per UGen):**
1. Host unit vectors transcribed from the source (edge cases: ctor priming,
   crossfades, calcspeed wraparound).
2. **Reference renders**: `gen_ugen_ref.scd` renders a canonical 1 s graph per
   UGen through NRT scsynth (48 kHz float32, seed-pinned with
   `RandSeed.kr(1, kGoldenSeed)` inside the def) into
   `ugen_ref/<Name>.expected.wav` (**48,000 frames** = 1 s at 48 kHz, trimmed
   like the drone goldens — the drone goldens themselves are 192,000 frames).
   `test_dronecaster.cc` compares the C++ port through the SAME
   gates (max_abs ≤ 1e-6, corr ≥ 0.999999). A UGen is DONE only when its
   reference render passes.
3. Noise/random UGens must additionally match `sc::RGen` sequences exactly
   (the taus88 is the single shared generator — NEVER libm rand).

**Rate law:** every port is block-rate driven like T-D1 (`ProcessBlock(out,n)`
with n = 64) and every time constant derives from `sr` at runtime. No libm in
per-sample paths except where the source itself calls exp/sin/pow per block
(e.g. filter coefficient updates) — cache per param change (T-D1 law).

**Licensing:** scsynth core = GPL-3.0 (same as the T-D1 kit); sc3-plugins
files carry their own headers — each SC3P port file must quote the original
header's license line. Classlib sugar (Splay/Mix/SelectX/TChoose/PMOsc/
DynKlank/Changed/Env) is GPL-3.0 class code → port its EXPANSION LAW as
documented helpers, never a re-invention.

## UGen inventory (from the corpus scan, 34 locked drones)

| Family | UGens | Source |
|---|---|---|
| noise | WhiteNoise, PinkNoise, BrownNoise, GrayNoise, Crackle, Dust, Dust2, LFNoise0/1/2, CoinGate, LFDNoise3 | CORE NoiseUGens.cpp (LFDNoise3: DynNoiseUGens.cpp) |
| lfo | LFSaw, LFPar, LFCub, LFPulse, VarSaw, SyncSaw, Impulse, Phasor, Line, XLine, Linen, DC, A2K, Clip, Fold | CORE LFUGens.cpp (Phasor: TriggerUGens.cpp) |
| trig | Trig, Trig1, TDelay, Latch, SetResetFF, Peak, Gate | CORE TriggerUGens.cpp |
| osc2 | Saw, SinOscFB, Select, Klank | CORE OscUGens.cpp |
| filters | LPF, HPF, BPF, RLPF, RHPF, BAllPass, BLowPass, BHiShelf, BPeakEQ, BLowShelf, MoogFF, OnePole, LeakDC, Slew, Integrator, Lag, LagUD, VarLag, Amplitude, Compander, Limiter | CORE FilterUGens.cpp |
| delays | DelayN/L/C, CombN/L/C, AllpassN/C, BufDelayN/C, BufAllpassC, Pluck, PitchShift | CORE DelayUGens.cpp |
| reverbs | FreeVerb, GVerb | CORE ReverbUGens.cpp |
| pan | Pan2, Balance2, Rotate2 | CORE PanUGens.cpp |
| demand | Demand, Dseq, Drand, Dxrand, Dseries, Dbrown, Dbufrd, Dbufwr | CORE DemandUGens.cpp |
| buffers | LocalBuf, BufRd, BufWr, BufFrames, BufRateScale, BufSampleRate, SampleRate, ControlRate | CORE DelayUGens.cpp |
| localio | LocalIn, LocalOut | CORE IOUGens.cpp |
| chaos | LorenzL, HenonC, FBSineN | CORE ChaosUGens.cpp |
| sc3p-filter | SVF, MoogLadder, DFM1 | SC3P BlackrainUGens.cpp / BhobFilt.cpp / TJUGens.cpp |
| sc3p-dyn | Decimator, CrossoverDistortion, SineShaper, DiodeRingMod, InsideOut | SC3P DistortionUGens.cpp / DiodeRingMod.cpp / MCLDDistortionUGens.cpp |
| sc3p-osc | SawDPW, SinGrain | SC3P MCLDOscUGens.cpp / JoshGrainUGens.cpp |
| sc3p-reverb | GreyholeRaw (Greyhole class wraps it) | SC3P DEINDUGens (faust) |
| sugar | Changed, PulseDPW, PMOsc, Splay, SplayAz, SelectX, TChoose, Mix, DynKlank, Env/Gaussian | classlib (expansion laws) |
| hw | MouseX, MouseY | CORE UIUGens.cpp — **port as CC-driven constants** (a drone port maps them to module params; NRT renders use the classes' defaults) |

Note: `SawDPW`/`PulseDPW` also appear under sc3p-osc / sugar above — those rows
are corpus cross-references. Both were ported once in Task 5 (`21fec7f`);
do not port them again.

## Tasks

1. **Reference-harness**: `test/dronecaster_ref/gen_ugen_ref.scd` +
   `run_ugen_ref.sh` — one canonical graph per UGen, float32 1 s renders,
   skip-existing + 48,000-frame trim (reuse the T-D1 script pattern). The
   graph table lives in the .scd; each entry pairs with a `test_dronecaster.cc`
   section.
2. **sc_kit_noise.h** (noise family + LFDNoise3) — pink/brown state filters,
   Dust's `fcoin`-biased scheduling, LFNoise interpolation laws, CoinGate.
3. **sc_kit_lfo.h** (lfo family) — LF oscillators read the SAME mPhase laws as
   LFTri (verified 3.14 ctor rollback!), VarSaw's crossfade, SyncSaw, Phasor.
4. **sc_kit_trig.h** (trig family) — edge laws from TriggerUGens (Trig/Trig1
   multi-trigger window, Latch, SetResetFF, Peak's decay, Gate).
5. **sc_kit_osc2.h** (osc2 + SawDPW + PulseDPW sugar law) — Saw's leaky
   integrator NOTE: SC's Saw is BLIT-style like Pulse (read it first), SinOscFB
   feedback path, Select's index law, Klank's resonator bank.
6. **sc_kit_filter.h** (core filters) — BEQ suite coefficient laws (LPF/HPF
   Bezout + RLPF/RHPF/B*), MoogFF, OnePole/LeakDC/Slew/Integrator/Lag/LagUD.
   Status: the 11 named filters are done (`0b90411`); BEQ suite, MoogFF and
   VarLag remain. Amplitude/Compander/Limiter landed in the separate
   **`sc_kit_core_dyn.h`** (`5449caa`) — do NOT use `sc_kit_dyn.h`, which is
   Task 14's filename.
7. **sc_kit_sc3p_filter.h** — SVF (Blackrain), MoogLadder (Bhob), DFM1 (TJ);
   quote their headers' licenses.
8. **sc_kit_delay.h** + **sc_kit_localio.h** — the delay core shared by
   DelayN/L/C, Comb*, Allpass*, BufDelay*, Pluck (Karplus-Strong), PitchShift
   (needs the pitch-shifter machinery — reuse the clouds correlator knowledge);
   LocalIn/Out as engine-level feedback buses (the C++ engine allocates a
   per-voice LocalBus area in the arena).
9. **sc_kit_reverb.h** — FreeVerb + GVerb; GreyholeRaw (faust) in a separate
   `sc_kit_greyhole.h` (larger; port after the core reverbs pass).
10. **sc_kit_pan.h** — Pan2/Balance2/Rotate2 + Splay/SplayAz expansion helpers.
11. **sc_kit_demand.h** — the Demand-rate machinery (a demand UGen is pulled
    by a triggering UGen; model as a per-voice DemandClock the drone process
    calls — the SC schedule applies one demand value per trigger).
12. **sc_kit_buf.h** — LocalBuf + BufRd/Wr/Frames/RateScale/SampleRate +
    SampleRate/ControlRate constants; buffer memory from the voice arena (the
    SynthSocket tape slot already exists in the socket contract).
13. **sc_kit_chaos.h** — LorenzL/HenonC/FBSineN (exact float laws from
    ChaosUGens).
14. **sc_kit_dyn.h** (sc3p dyn) — Decimator/CrossoverDistortion/SineShaper/
    DiodeRingMod/InsideOut (quote licenses).
15. **sc_sugar.h** — Changed, Mix (array sum), SelectX (interp select),
    TChoose (RGen-indexed select), PMOsc, DynKlank, Env helpers; document each
    expansion against the classlib source.
16. **MouseX/MouseY** — CC-driven constant sources (`MouseX.kr(lo,hi)` maps the
    drone's param 0-1 pot); NRT reference renders use the class defaults; the
    C++ defaults must match the class default arg (verify in the classlib).
17. **Full-suite + memory**: all families host-tested, `tc_dronecaster` slot
    measured, RAM headroom checked (the kit adds mostly per-voice state, the
    engine arena grows; 136 KiB carve may need +X KiB — re-derive and ASSERT).

## Constraints

- Host tests: `cd rack-rp2350/test && make test_dronecaster` (plain g++, no MCU
  headers — the portability proof).
- Instrument-rate renders: `cd rack-rp2350/test/dronecaster_ref && bash
  run_ugen_ref.sh` (cwd-relative ref paths, skip-existing, trim) — NOT a bare
  `sclang .../gen_ugen_ref.scd`, which breaks paths and skips the trim. System
  SC 3.14.1-6 + sc3-plugins 3.14.1-1 (pacman). Per-UGen references may use an
  optional 3rd entry tuple element as an engine override (Limiter renders via
  supernova because scsynth 3.14.1 segfaults on Limiter/Normalizer).
- Per-sample code paths get `DRONECASTER_TC` only via the -O3 wrapper — the kit
  headers stay inline; the wrapper's inlining pulls them into the SRAM slot
  (single token law — do not add per-function tokens).
- No libm in per-sample loops; exp/sin/pow only at segment/coefficient init.
- Denormals: FPSCR FZ+DN is global — zero software handling.
- Every SC3P file's original license line is quoted verbatim in the port.

## Verification checklist (per family)

- [ ] Header cites `file:line` for every law
- [ ] Unit vectors (ctor priming, edges) pass
- [ ] Reference render passes (max_abs ≤ 1e-6, corr ≥ 0.999999)
- [ ] No random source outside `sc::RGen`
- [ ] Commit `[Add] dronecaster: <family> kit (SC 3.14.1 port), reference-exact`
