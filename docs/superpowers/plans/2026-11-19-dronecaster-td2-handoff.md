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
| `5449caa` | dynamics family — Amplitude/Compander/Limiter, 3/3 exact |
| `50ea06c` | MoogFF + BEQ suite — 7/7 exact (completes plan Task 6 except VarLag) |
| `629847b` | SC3P filters complete: SVF + MoogLadder + DFM1 — 3/3 exact (DFM1 via pinned render epoch) |
| `f6852fd` | core delay family — DelayN/L/C, CombN/L/C, AllpassN/L/C, 9/9 exact |
| `2c3e95f` | Pluck (Karplus-Strong) — 1/1 exact |
| `53b5809` | Buf* delays (6) + PitchShift — 7/7 exact |
| `ce6b755` | LocalIn/LocalOut — 1/1 exact (Task 8 complete, 17/17) |
| `3f712ca` | FreeVerb — 1/1 exact |
| `be6ab28` | GVerb — 1/1 exact (core reverbs done; GreyholeRaw remains) |
| `3a45564` | Pan2/Balance2/Rotate2 — 3/3 exact (Task 10 core done) |
| *(this commit)* | GreyholeRaw — 1/1 exact (Task 9 complete: 3/3) |

**Done and reference-exact (100 UGens + the trio):**

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
- `sc_kit_core_dyn.h` — Amplitude, Compander, Limiter (lookahead ring). **3/3
  exact.** (`sc_kit_dyn.h` is reserved for plan Task 14, the SC3P dyn family.)
  Limiter's ref renders through **supernova** (scsynth 3.14.1 segfaults on
  Limiter/Normalizer — see §7); `gen_ugen_ref.scd` entries take an optional
  third element as an engine override.

- `sc_kit_filter.h` (continued) — MoogFF (4-state ladder) + BEQ suite
  (BLowPass/BHiPass/BPeakEQ/BAllPass/BLowShelf/BHiShelf, one shared biquad).
  **7/7 exact** (all max_abs = 0). Completes plan Task 6 except VarLag
  (classlib EnvGen expansion — deferred to the Env/sugar task).
- `sc_kit_sc3p_filter.h` — SVF, MoogLadder (verbatim source license headers).
  **2/2 exact.**
- `sc_kit_dfm1.h` + generated `sc_kit_dfm1_tables.h` — DFM1 (source license
  quoted, 1536-entry LUT + 3×128 Ziggurat tables). **1/1 exact** using the
  pinned render epoch (`ugen_ref/DFM1.seed.txt`).
- `sc_kit_delay.h` — DelayN/L/C, CombN/L/C, AllpassN/L/C share one power-of-two
  ring buffer (`ceil(maxdelay*sr+1)+64` rounded up). **9/9 exact.** The initial
  "checked" phase (zero output until the read phase is non-negative) is kept
  throughout — the source's switch to unchecked is only an optimization.
  `Pluck` (Karplus-Strong) joins the same header: **1/1 exact**. BufDelayN/L/C
  and BufAllpassN/L/C use `InitBuf` (external buffer, dsamp clipped to
  `bufSamples-1`): **6/6 exact**. `PitchShift` (4 crossfading read heads + the
  voice's shared RGen): **1/1 exact**.
- `sc_kit_localio.h` — LocalIn/LocalOut per-voice feedback bus: **1/1 exact**.
  The host-testable `LocalBus` carries the block counter / touched law; the
  firmware still needs it wired into the voice arena and audio loop.
- `sc_kit_reverb.h` — FreeVerb (8 comb + 4 allpass) and GVerb (4th-order FDN +
  diffusers) **2/2 exact**. GVerb traps: `zapgremlins` is `1e-15 < |x| < 1e15`
  (not 3.4e38), slopes clear every block, and ramped level locals are NOT
  written back (only the setter's target) — else a 1-ULP re-ramp repeats.
- `sc_kit_greyhole.h` — GreyholeRaw **1/1 exact**. The faust-generated `mydsp`
  class is embedded VERBATIM (renamed + a `SetParams` helper + stub `dsp`/`UI`/
  `Meta`/`primes`); do not hand-transcribe its ~450-line compute loop. It needs
  **scsynth** (supernova cannot load the UGen) and scsynth exits 139 at shutdown
  after writing a complete wav — the harness accepts rc 139 for this name only.
- `sc_kit_pan.h` — Pan2/Balance2/Rotate2 **3/3 exact**, using `KitTables::sine`
  (= `ft->mSine`) and CALCSLOPE on amplitude / rotation coefs.

`rack-rp2350/test/test_dronecaster.cc` runs the whole gate:
`cd rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster`
— expects `PASS (0 failures)` (100 active refs as of the GreyholeRaw commit;
`ugen_ref/Klankar.expected.wav` is a tracked orphan from an earlier attempt —
Klank is skipped, so no test reads it).

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
| ~~dynamics~~ | ~~Amplitude, Compander, Limiter~~ **DONE**; VarLag remains | `/tmp/sc_full/server/plugins/FilterUGens.cpp` | `sc_kit_core_dyn.h` 3/3 exact. VarLag is classlib (EnvGen-based) — port with the Env work. |
| ~~moog~~ | ~~MoogFF~~ **DONE** | FilterUGens.cpp `MoogFF_next` | in `sc_kit_filter.h`, 1/1 exact; uses FilterUGens.cpp:29's **float** `PI` macro (using double `pi` leaves a 7e-9 residual). |
| ~~beq~~ | ~~BAllPass/BLowPass/BHiShelf/BPeakEQ/BLowShelf~~ (+BHiPass) **DONE** | FilterUGens.cpp BEQSuite Ctors | `sc_kit_filter.h` `BEQFilter`, 6/6 exact; slope law = `1/(n/3)`, NOT `/4`. |
| ~~sc3p filters~~ | ~~SVF, MoogLadder, DFM1~~ **DONE** 3/3 | `/tmp/sc3p_full/source/BlackrainUGens/`, `BhobUGens/BhobFilt.cpp`, `TJUGens/Dfm1.cpp` (+ Dfm1Lut) | `sc_kit_sc3p_filter.h` quotes each license. DFM1's RNG is seeded from `time(NULL)`, so the renderer writes the epoch to `ugen_ref/DFM1.seed.txt` (retrying if a second boundary is crossed) and `Dfm1::Init` takes it as an explicit seed. Tables generated from pin `25eacaf7`. |
| ~~delays~~ | ~~DelayN/L/C, CombN/L/C, AllpassN/L/C, Pluck, BufDelayN/L/C, BufAllpassN/L/C, PitchShift~~ **DONE** 16/16 | DelayUGens.cpp | All in `sc_kit_delay.h`. Buf* use `InitBuf` (external power-of-two SndBuf, dsamp clipped to `bufSamples-1`); a real engine still needs arena-backed SndBuf plumbing (pair with Task 12). PitchShift uses the shared voice RGen and 4 crossfading heads. |
| ~~localio~~ | ~~LocalIn, LocalOut~~ **DONE** 1/1 | IOUGens.cpp | `sc_kit_localio.h` `LocalBus` holds the touched/block-counter law. Firmware still needs a per-voice LocalBus area in the arena and audio-loop wiring (engine task, not kit). |
| ~~reverbs~~ | ~~FreeVerb, GVerb, GreyholeRaw~~ **DONE** 3/3 | ReverbUGens.cpp / sc3p DEINDUGens | FreeVerb + GVerb in `sc_kit_reverb.h`; GreyholeRaw faust class embedded verbatim in `sc_kit_greyhole.h` (scsynth + rc-139 tolerance). |
| sc3p reverb | GreyholeRaw (Greyhole class wraps it) | `sc3p/source/DEINDUGens/GreyholeRaw.cpp` (faust-generated) | Port after the core reverbs pass; it is large. |
| ~~pan (core)~~ | ~~Pan2, Balance2, Rotate2~~ **DONE** 3/3; Splay/SplayAz remain (sugar) | PanUGens.cpp | `sc_kit_pan.h` uses `KitTables::sine` + CALCSLOPE. Splay/SplayAz are classlib helpers — port in the sugar task. |
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
- **BEQ slope law differs from Filter2Pole**: `mFilterLoops = n/3`,
  `mFilterSlope = 1/mFilterLoops` (n=64 → 21 groups of 3 samples + 1 remain),
  applied once per 3-sample group. `Filter2Pole` uses `/4` — do not copy that
  divisor into the BEQ code. VarLag is classlib `Env([start,in],[time],warp)`
  + `EnvGen` + `Changed` + `Impulse` (Filter.sc:113-145), not a CORE UGen —
  deferred to the Env/sugar task.
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
- **DFM1's render epoch is pinned, not guessed**: `DFM1_Ctor` →
  `NoiseGen::initialize` sets `w = 3*time(NULL)` (counter = 0 for the single
  instance in the def). The renderer captures `Date.getDate.rawSeconds`
  immediately before launch into `ugen_ref/DFM1.seed.txt` and retries up to
  5× if the render straddles a second boundary; `Dfm1::Init(epoch)` uses that
  value. Without this, renders 2 s apart differ by max_abs 1.5e-4 — 150× the
  gate. Never keep a DFM1 ref without its seed sidecar.
- **sclang silent-hang trap #2: bare `and`**. `a and b` in a `.scd` parses but
  hangs after the welcome banner (no error printed); `a.and(b)` or `a && b`
  work. Also `&&` with a Function operand and `while({ a and { b } }, …)` hang
  — prefer `N.do { … }` with a flag. The old failure mode (unresolved classes /
  late `var`) still applies.
- **Core delay UGens (DelayN/L/C, CombN/L/C, AllpassN/L/C) make scsynth
  3.14.1-6 exit 139 AFTER writing a complete, correct WAV** — the crash is at
  shutdown/`RTFree` of the delay line, not during rendering. The script's
  `rc != 0` check throws and leaves sclang hung. Supernova runs the same
  `DelayUGens.cpp` law cleanly (rc 0) and produced byte-identical channel data
  to scsynth's completed render. All nine delay refs therefore carry the
  `supernova` engine override. Do not delete them thinking they are corrupt.
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
