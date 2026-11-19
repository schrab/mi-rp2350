# Dronecaster T-D2 handoff — "all UGens" kit build

**Read this first if you are continuing the T-D2 UGen-kit work without the
prior conversation.** It records what exists, the exact working method, the
environment quirks, the law-level findings, and what is left.

Plan contract: [`2026-11-19-dronecaster-td2.md`](2026-11-19-dronecaster-td2.md).
T-D1 (the trio engine) is CLOSED and user-accepted; ledger tail has its entry.

---

## 1. Mission

Port **every scsynth/sc3-plugins UGen the 34 remaining Dronecaster drones
need** into `rack-rp2350/src/dronecaster/`, each verified **sample-exact**
against an NRT scsynth reference render. No drone ports in T-D2 — drones come
after the kit is complete (T-D3).

## 2. Current state (2026-11-19, this session)

Commits on branch `rack-rp2350` (pushed):

| Commit | Content |
|---|---|
| `5cc7231` | noise kit + per-UGen NRT reference harness |
| `3bccf77` | LFO kit |
| `1d95fd9` | trigger kit |
| `21fec7f` | osc2 kit + filter kit (ports) |
| `0b90411` | filter references all passing |
| `887ec9b` | renderer simplified for the system sc3-plugins install |
| *(this commit)* | dynamics family — Amplitude/Compander/Limiter, 3/3 exact |

**Done and reference-exact (66 UGens + the trio):**

- `sc_kit_noise.h` — WhiteNoise, ClipNoise, GrayNoise, PinkNoise, BrownNoise,
  Crackle, Dust, Dust2, LFNoise0/1/2 (ar + kr), LFClipNoise, LFDNoise0,
  LFDNoise3, CoinGate + init-time Rand/IRand/TRand/TExpRand/NRand/LinRand/
  ExpRand helpers. **24/24 references exact** (max_abs ≤ 4.8e-7).
- `sc_kit_lfo.h` — LFSaw, LFPar, LFCub, LFPulse, VarSaw, Impulse, Phasor,
  Clip, Fold, DC, A2K (+ `sc_wrap` float/double). **15/15 exact.**
- `sc_kit_trig.h` — Trig1, Trig, TDelay, Latch, SetResetFF, Gate, Peak,
  Changed. **8/8 exact.**
- `sc_kit_osc2.h` — Saw (BLIT), SinOscFB, Select (+`SelectIndex`), Klank
  (law-tested only — see §7), SawDPW, PulseDPW. **5/5 exact.**
- `sc_kit_filter.h` — LPF/HPF/BPF/RLPF/RHPF (`Filter2Pole`), OnePole,
  Integrator, LeakDC, Slew, Lag, LagUD. **11/11 exact.**
- `sc_kit_dyn.h` — Amplitude, Compander, Limiter (lookahead ring). **3/3 exact.**
  Limiter's ref renders through **supernova** (scsynth 3.14.1 segfaults on
  Limiter/Normalizer — see §7); `gen_ugen_ref.scd` entries take an optional
  third element as an engine override.

`rack-rp2350/test/test_dronecaster.cc` runs the whole gate:
`cd rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster`
— expects `PASS (0 failures)` (67 refs as of the dynamics commit).

## 3. The working method (do exactly this per UGen/family)

1. **Read the law from source** — never from memory. Sources are cloned at
   `/tmp/sc_full` (supercollider, tag Version-3.14.1) and `/tmp/sc3p_full`
   (sc3-plugins main). If missing: shallow-clone both again.
   Cite `file:line` in the port header.
2. **Port into a family header** in `src/dronecaster/` (see existing headers
   for style: exact float/double types, ctor priming, CALCSLOPE laws).
3. **Add a canonical graph** to `test/dronecaster_ref/gen_ugen_ref.scd` —
   name + a SynthDef body. Params are part of the contract; put the same
   params in the C++ test.
4. **Render**: `cd rack-rp2350/test/dronecaster_ref && bash run_ugen_ref.sh`
   (skips refs that already exist; `DC_FORCE=1` forces; trims to 48000
   frames). ~12-14 s per new ref.
5. **Add a test section** in `test_dronecaster.cc` (use `ref_check`,
   `ref_ar_fill`, `ref_kr_fill`; the kr helper implements the SSE ramp —
   see §6).
6. **Iterate until `PASS`, then commit** `[Add] dronecaster: T-D2 <family>
   … — N/N references exact` and mention any law corrections in the message.
7. Update the ledger (`.superpowers/sdd/progress.md`) at family milestones.

## 4. Environment (system install — verified 2026-11-19)

- SuperCollider **3.14.1-6 AND sc3-plugins 3.14.1-1 are installed system-wide**
  (`pacman -Q sc3-plugins` → `3.14.1-1`), `sclang`/`scsynth` on PATH.
- The old `/tmp` wiring is **gone**: no `DC_SCSYNTH_FLAGS=-U /tmp/scplug`,
  no `sclang -l /tmp/sc_root/sc3p_conf.yaml`. `run_ugen_ref.sh` now calls
  plain `sclang gen_ugen_ref.scd` (mirroring `run_goldens.sh`). scsynth finds
  the sc3p `.so` files on its default plugin path (`/usr/lib/SuperCollider/
  plugins`) and sclang's default class lib resolves `SawDPW`/`PulseDPW`/`SVF`/
  `DFM1`/`GreyholeRaw`/`Decimator` — all probed OK.
- `gen_ugen_ref.scd` / `gen_goldens.scd` still read the `DC_SCSYNTH_FLAGS`
  and `DC_FORCE` env vars as generic hooks; they default to empty/off and
  need no setup.
- **Failure mode if a class is unresolved**: `SawDPW`/`SVF`/`DFM1` etc. fail at
  compile time → **sclang hangs silently after the welcome banner** (same trap
  as a late `var`). If a render "hangs", check that first.

## 5. What remains (queue, with anchors)

| Task | UGens | Where to read | Notes |
|---|---|---|---|
| ~~dynamics~~ | ~~Amplitude, Compander, Limiter~~ **DONE**; VarLag remains | `/tmp/sc_full/server/plugins/FilterUGens.cpp` | `sc_kit_dyn.h` 3/3 exact. VarLag is classlib (EnvGen-based) — port with the Env work. |
| moog | MoogFF | FilterUGens.cpp `MoogFF_next` | `wcD = 2 tan(T*pi*f)*sr; b0 = TwcD/(TwcD+2); a1 = (TwcD-2)/(TwcD+2)`; 4-state ladder (s1..s4) + reset input. |
| sc3p filters | SVF, MoogLadder, DFM1 | `/tmp/sc3p_full/source/BlackrainUGens/`, `BhobUGens/BhobFilt.cpp`, `TJUGens/Dfm1.cpp` (+ Dfm1Lut) | **Quote each file's license header verbatim.** DFM1 ships a lookup table (Dfm1Lut.h) — port it as a table constant. |
| delays | DelayN/L/C, CombN/L/C, AllpassN/C, BufDelayN/C, BufAllpassC, Pluck, PitchShift | DelayUGens.cpp | Common delay core; `maxdelaytime`-sized buffers belong in the voice arena (add a delay-buffer alloc helper). Pluck = Karplus-Strong. PitchShift = the clouds-style pitch shifter — reuse the correlator knowledge from `lib/clouds`. |
| localio | LocalIn, LocalOut | IOUGens.cpp | Engine-level feedback buses: the C++ engine needs a per-voice LocalBus area in the arena; `LocalIn` reads last block's `LocalOut` writes. |
| reverbs | FreeVerb, GVerb | ReverbUGens.cpp | FreeVerb = 8 comb + 4 allpass per channel. GVerb is larger (its own buffer layout). |
| sc3p reverb | GreyholeRaw (Greyhole class wraps it) | `sc3p/source/DEINDUGens/GreyholeRaw.cpp` (faust-generated) | Port after the core reverbs pass; it is large. |
| pan | Pan2, Balance2, Rotate2 | PanUGens.cpp | Pan2 = equal-power with CALCSLOPE on pos/level; Balance2 = the balance law; Rotate2 = rotation matrix. Splay/SplayAz/InsideOut are classlib/`MCLDDistortionUGens` — Splay is a classlib helper that pans an array. |
| demand | Demand, Dseq, Drand, Dxrand, Dseries, Dbrown, Dbufrd, Dbufwr | DemandUGens.cpp | Demand-rate machinery: a demand UGen is PULLED by a triggering UGen. Model as a per-voice demand clock the drone calls once per block; Dseq/Drand keep their own state. |
| buffers | LocalBuf, BufRd, BufWr, BufFrames, BufRateScale, BufSampleRate, SampleRate, ControlRate | DelayUGens.cpp (they live there) | LocalBuf allocates from the voice arena; BufRd's interpolation modes (1/2/4) matter. |
| chaos | LorenzL, HenonC, FBSineN | ChaosUGens.cpp | Exact float laws; LorenzL/FBSineN are per-sample integrators. |
| sc3p dyn | Decimator, CrossoverDistortion, SineShaper, DiodeRingMod, InsideOut | `sc3p/source/DistortionUGens/`, `DEINDUGens/DiodeRingMod.cpp`, `MCLDUGens/MCLDDistortionUGens.cpp` | Quote licenses. |
| sugar | PMOsc, DynKlank, Linen, Env, VarLag, Gaussian, Mix, Splay, SplayAz, SelectX, TChoose | classlib (`/tmp/sc_full/SCClassLibrary/`) | Port the EXPANSION LAWS as documented helpers (Mix = array sum; SelectX = interpolating select; TChoose = RGen-indexed; Env/EnvGen = the full EnvGen law — reuse/extend `sc_kit_env.h`; Linen = EnvGen with rise/sustain/decay). |
| hw | MouseX, MouseY | UIUGens.cpp | Map to drone params (CC-driven constants); NRT refs use the class defaults (check `MouseX.sc` defaults). |

## 6. Laws discovered (all load-bearing — do not re-derive differently)

- **RGen is shared per voice**: every noise UGen advances the same `sc::RGen`;
  **ctor priming draws are part of the sequence** (skipping one desyncs all
  later draws). Each port's `Init` replicates its source ctor exactly
  (prime then roll back state, or keep the primed output, per UGen).
- **kr → ar conversion (K2A / ControlRateInput / binary ops) RAMPS**: block k
  is `v(k-1) + i·slope`, and the x86 build's nova SSE `set_slope` computes
  lanes as `[s, s+sl, s+2sl, s+2sl+sl]` with a `4*sl` repeated-add group
  increment — NOT a plain accumulation. `ref_kr_fill` implements it; ported
  UGens expose `ctor_value()` where the K2A base matters (block 0 ramps from
  the ctor value).
- **Ctor priming**: SOS filters prime with `in[0]` (state y1 = in[0]); the
  prime must set the coefficient BASE **before** the block-slope deltas are
  computed (otherwise `b1` drifts to 2× and the recurrence explodes — this
  cost an hour once). Integrator is the exception (y1 stays 0). Triggers
  prime then roll back; their K2A block 0 is flat at `v(0)`.
- **Coefficient interpolation on change**: one slope step per **4-sample
  group**, `kSlope = 1/(n/4)` (= mFilterSlope law).
- **Float vs double is semantic**: e.g. `Saw`'s `m_cpstoinc` is double but the
  perform copies it to a float local; `Impulse`'s `mFreqMul` is a `float`
  member (a float product changes the wrap block at 10 Hz); `Crackle` is
  float math; `Pulse`/`Saw` `scale = 0.5/N` is a double division rounded to
  float. When a reference mismatches, diff the C types first.
- **Saw** (BLIT) is Pulse's structure with ONE lookup, `y1` primed to
  **−0.46**, and the `(numer*denom − 1)*scale` integrator; the bad-denominator
  branch outputs a raw `1.0` (added to `0.999*y1` without scale).
- **`SawDPW`** k-law: `scalefac = 1/(4·inc·(1−inc/sr))` with `inc = f·2/sr`;
  the ctor primes with the **A-rate** law (`sr/(8f(1−f/sr))`) and does NOT
  roll the phase back. **`PulseDPW`** = `SawDPW(f,0) − SawDPW(f, wrap(2·width,-1,1))`.
- **`Changed`** = classlib `HPZ1.kr(in).abs > threshold`; HPZ1 primes
  `prev = in(0)` → first output 0.
- **`Select`** ctor primes one kr sample → K2A block 0 flat at `v(0)`.
- **Compander gain is BLOCK-rate**: the `|control|` envelope runs over all n
  samples, then ONE gain `pow(prevmax/thresh, slope-1)` (zapgremlins below
  thresh) is derived from its FINAL value and CALCSLOPE-ramped across the
  block. Recomputing the gain per sample looks plausible and is badly wrong
  (this exact mistake gave max_abs 0.25 / corr 0.9988 before the fix).
- **Limiter rotation**: `m_xinbuf = table`, `m_xmidbuf = table+bufsize`,
  `m_xoutbuf = table+2*bufsize` (ZOFF=0 on x86). On flip,
  `out<-mid, mid<-in, in<-old out`. The ctor's one sample leaves `pos=1`,
  so the second flip lands on output sample `2*bufsize-1` (959 for
  dur=0.01 @ 48 kHz), not `2*bufsize`.

## 7. Pitfalls / environment hazards

- **Run the host suite from `rack-rp2350/test`** — the reference paths are
  cwd-relative; from elsewhere every check "fails" with missing files.
- **`scsynth -N` renders one block past the duration marker**; refs must be
  trimmed to 48000 frames (run_ugen_ref.sh does it; if you call sclang
  directly, trim afterwards or every layout check fails with `ref=48064f`).
- **Klank segfaults scsynth 3.14.1** (`Klank.ar` → rc 139) even though it is
  core. Its reference is SKIPPED by design; the port is verified by a law
  test (impulse rings, tail decays). Do not "fix" by guessing — if you need a
  real Klank reference, test whether the crash is the SIMD path with more
  partials, or render via a different engine.
- **Limiter AND Normalizer segfault scsynth 3.14.1-6** (rc 139, any input or
  `dur`) — same trap class as Klank, and a failed `systemCmd` leaves sclang
  hung after the error dump (the .scd's `0.exit` never runs). Fix: render
  these through **supernova**, which runs the same `FilterUGens.cpp` law
  cleanly and deterministically (data chunks byte-identical across runs,
  48,000 frames already trimmed, channel 1 silent). `gen_ugen_ref.scd` takes
  an optional 3rd tuple element as the engine command. Do not swap scsynth
  for supernova globally — only the affected UGens.
- **One reader per serial port** and the other RP2350 rules still apply if you
  touch hardware; not needed for T-D2.
- `sclang` silently hangs after the welcome banner on (a) a class that fails
  to resolve (missing sc3-plugins classes), (b) a `var` after statements,
  (c) top-level `( … )` instead of a called Function. The .scd files already
  follow the safe pattern — keep it.
- The reference renderer writes each ref with its own `scsynth -N` run;
  rendering 60+ refs takes ~10-15 min. The skip-existing logic keeps
  iteration to the new entries only.

## 8. Verification checklist (per family — from the T-D2 plan)

- [ ] Header cites `file:line` for every law
- [ ] Unit/edge vectors where ctor priming differs
- [ ] Reference render passes: `max_abs ≤ 1e-6`, `corr ≥ 0.999999`
- [ ] No random source outside `sc::RGen`
- [ ] sc3-plugins ports quote their license header
- [ ] Commit `[Add] dronecaster: T-D2 <family> … — N/N references exact`
- [ ] Ledger updated at milestones

Final T-D2 completion = the 34 drones' UGen lists fully covered (see the plan's
inventory table) + the full host suite green + the ledger note.
