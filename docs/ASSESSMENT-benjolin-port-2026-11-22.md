# Benjolin port assessment — PD vs SC, UGen audit, and LocalIn/LocalOut decision

**Date:** 2026-11-22
**Status:** pre-port feasibility + correctness audit; reviewed and corrected
same day (SC porting confirmed; §2.1 comparator degeneration + §4.3 DFM1
mapping fixed — see the "review finding" boxes); no code changed
**Sources:** both upstream repos (cloned read-only to `/tmp/opencode/benjolin-pd` and
`/tmp/opencode/benjolis-sc`), this repo's vendored libs, `PicoCollider/`, the SDD
ledger, and the Lira8 / Dronecaster review documents.

---

## 1. Executive summary

**Recommendation: port the SuperCollider implementation
(`scazan/benjolis`, the Alejandro Olarte SynthDef) — not the PureData
implementation (`macumbista/benjolin`).**

The SC SynthDef is a single ~90-line graph whose entire UGen palette is already
ported reference-exactly in `PicoCollider/` (26 header-only UGens, 129 SC 3.14.1
NRT goldens, gate `max_abs ≤ 1e-6, corr ≥ 0.999999`). Assembly is largely
mechanical; the remaining work is a handful of nova-path math helpers, a full
SynthDef golden, and the firmware wiring for the rungler feedback.

The PD implementation is a 651-line deeply nested patch requiring nine external
libraries, MMB band-limited oscillators backed by **three 754,845-float tables
(~9 MB)** — physically impossible on RP2350's 520 KB SRAM / ~4 MB flash — and a
control-rate `env~ → > 50` "3-bit DAC" that would have to be reverse-engineered
before any port could start.

**Three items dominate the risk register** (details in §4–§6):

1. **LocalIn/LocalOut 2-channel audio-rate feedback** — correct host golden
   exists but only mono; the firmware arena wiring was never completed by the
   Dronecaster work; Benjolis cannot avoid it.
2. **DFM1's `time(NULL)` seed** — goldens require an epoch sidecar; no
   deterministic device-side policy exists.
3. **SC's `tanh`** — correct nova polynomial lives in a firmware-only header,
   is absent from PicoCollider, and had a documented sign bug fixed late with
   no golden covering it.

---

## 2. Why SC, why not PD

### 2.1 The SC implementation

`benjolis-sc/lib/Engine_Benjolis.sc` contains one SynthDef with ~90 lines of DSP:

- `LFTri` × 2, `PulseDPW` × 2
- `>` / `==` comparisons for PWM and the Rungler comparator (the comparator
  degenerates at build time — see the review-finding note below)
- 8-stage shift register: `Latch.ar` + `DelayN.ar` taps at `SampleDur.ir × 1…8`
- `LocalIn.ar(2)` / `LocalOut.ar` closing the chaotic feedback loop
- `RLPF` / `RHPF` / `SVF` / `DFM1` selected by `Select.ar`
- `tanh` soft clip, `DelayL` + `Lag` + `Clip` stereo pan, `LeakDC`

**REVIEW FINDING — the sh0 comparator degenerates at graph-build time.**
`sh8 = 1` (var initializer, `Engine_Benjolis.sc:27`) is referenced at line 47
*before* `sh8 = DelayN.ar(…)` assigns the register UGen at line 57 — sclang
evaluates the SynthDef function once at build, so the graph contains the
**constant 1**, not the register feedback. Since the inner comparisons output
exactly 0/1, the runtime law collapses to `sh0 = 1 − (osc1 > 0.5)` (truth
table: `(1>A)==(1<A)` is 1 only when A==1). The "XOR-ish" behavior was the
author's intent, not the shipped graph. The port must implement the degenerate
form — the full-SynthDef golden encodes it — and "restoring" the intended
sh8-XOR comparator is a deliberate deviation decision (closer to the original
Benjolin circuit, but it fails the golden). Add a comment in the engine so
nobody "fixes" it later.

`benjolis.lua` is norns GUI/params only — discard, as with the PD GUI.

**Every UGen in that list has a port** — see the audit table in §3. Dronecaster
already uses `PulseDPW` (`drone_sachiko.h`), `SVF` (`drone_fieldsteel.h`),
`LeakDC`, `Lag`, `NovaTanh`, etc., so there are working call patterns and an
`-O3` wrapper/arena template in `src/module_dronecaster.h`.

**Golden alignment is simpler than PD.** SC NRT renders use a 64-sample block
grid (`sc_defs.h:10`), matching the rack's engine burst (`module_dronecaster.h:41`
`kBlockSize = 16` host grid, engine bursts 64). PD Lira8 needed 32 kHz/64-sample
internal blocks vs the rack's 16-sample grid (4× accumulation) plus a battery of
headless probes to discover implicit scheduler semantics.

### 2.2 The PD implementation — three hard blockers

`benjolin-pd/benjolin.pd` (651 lines, nested `benjolinguts → rungler, O1, O2,
comparator, filter, output, limiter, switch`) declares:

```
mmb, creb, iemmatrix, maxlib, zexy, hexloader, flatgui, ggee, list-abs,
matrixctrl (custom GUI), mtx_*~
```

1. **Wavetable size.** `mmb/bl-tri.mmb~.pd` (150 objects) reads
   `tabread4~ $1-bl-tritable.mmb`; `mmb/bl-init.mmb.pd` (315,282 lines) defines
   **three arrays of 754,845 floats** (`bl-sawtable`, `bl-parabolatable`,
   `bl-tritable`) ≈ **9.06 MB total**. They cannot be stored on RP2350 and cannot
   be regenerated into SRAM. The only viable port replaces them with analytic
   band-limited oscillators — which is exactly what `PulseDPW` already is in
   PicoCollider. Also needed: `multimode.mmb~.pd` (213 lines) and
   `biquad.mmb~.pd` (216 lines).

2. **Control-rate hacks.** The 3-bit DAC in `benjolin.pd` is `env~ 2 → > 50` —
   a 64-sample Hann/RMS detector, not a digital bit. Every knob passes through
   `line~ … 50` smoothing; all `s~/r~` buses carry a one-block delay; a 64-sample
   `z~` delay is used in the limiter. These need the same forensic
   primitive-law derivation as `lira8_ref/PD-PRIMITIVES.md`, but with more
   primitives and less guarantee.

3. **Golden-rendering bootstrap.** Lira8's copyable harness
   (`gen_l8bridge.py`, `gen_golden_patch.py`, `drive_ref.py`, `scenarios.txt`,
   `compare_wav.py`) was built for a *vanilla-Pd* patch. This Benjolin patch
   needs the whole external stack just to render a golden, so that advantage
   evaporates.

Lira8 was tractable because it used vanilla Pd objects whose C source could be
quoted verbatim and probed headlessly. This patch is the opposite case.

### 2.3 What the Lira8 workflow teaches regardless of choice

Reusable patterns from `rack-rp2350/test/lira8_ref/` and
`docs/REVIEW-lira8-dsp-2026-11-21.md`:

- Spec-first with `file:line` provenance (`PORTING-SPEC.md`, 835 lines) and a
  primitive-law document (`PD-PRIMITIVES.md`), both with empirical correction
  boxes overriding earlier claims.
- Golden WAVs committed (`goldens/*.wav`) so `make test` needs neither Pd nor
  sclang; a shared scenario/param protocol drives both reference and host
  renderers.
- Plain-g++ host tests, no MCU headers, sanitizers (ASan/UBSan/TSan) as gates,
  source-text pinning for MCU-only adapter statements (`test_output_clip.cc`).
- Correlation retired as primary gate for chaotic scenarios; surviving gates =
  spectral exactness, RMS envelopes, ~1–2 dB spectral envelopes, and the user's
  ears.
- **A review against the source language's semantics is mandatory**: the
  2026-11-21 review found F1–F8, including two DSP-law bugs the goldens had
  enshrined (`powtodb` `100+10·log10` vs `100+20·log10`; expander slope ×2).
- Caveat: `make test` aggregate is not fully trustworthy — the unrelated
  Elements `test_reverb` canary fails nondeterministically (ASLR-sensitive
  uninitialized memory). Run module binaries individually.

---

## 3. UGen-by-UGen audit

Legend: **E** = reference-exact golden; **L** = law-only / unit-vector check;
**gap** = no golden for the exact rate/parameters Benjolis uses.
"Device proof" means a live drone instantiates it — see the caveat in §3.1.

| SynthDef UGen | Port location | Golden | Caveat | Device proof |
|---|---|---|---|---|
| `LFTri` | `sc_kit_osc.h:160-185` | L: `test_dronecaster.cc:1436-1466` | 3.14 **ctor-priming rollback**; no `iphase` arg | mixed |
| `PulseDPW` | `sc_kit_osc2.h:241-256` (two `SawDPW`) | E: `PulseDPWar` (w=0.3) | k-law; A-rate ctor prime, **no phase rollback**; no freq guard; w=0.5 gap | Sachiko (HW hang) |
| `>` / `==` | plain C++ operators | none | no comparison-specific documented issue; CALCSLOPE/reciprocal laws apply to arithmetic only | no exact use |
| `DelayN` ×8 (1–8 samples) | `sc_kit_delay.h:51-259` (`kind=0`) | E: `DelayNar` (supernova) | min 1 sample; checked-phase startup zeros; `InitExt` clamp `bufsize-4`; `Init()` heap-allocates | indirect only |
| `Latch` | `sc_kit_trig.h:96-106` | E: **`Latchkr` only** | rising edge `prev<=0 && trig>0`; `.ar` law **ungated**; call `Next()` per sample | Mt. Lion/Zion, mixed |
| `LocalIn`/`LocalOut` | `sc_kit_localio.h:18-62` | E: `LocalIO`, **1 channel only** | 1-block delay; accumulate-on-repeat; **firmware wiring never done**; 2-ch unproven | **none** |
| `SampleDur.ir` | constant `1/sr` | n/a | no port needed | n/a |
| `LinLin` | `sc_kit_lin.h:167-192` | E: `LinLinar`/`LinLinkr` | **no clipping** (classlib mul-add); double→float arg rounding | mixed |
| `.midicps` (ar) | `NovaMidi2Freq`, `drone_common.h:371-374` | via drone goldens | nova **truncated** `1/12` literal — not `sc::midicps` | mixed |
| `RLPF`/`RHPF` | `Filter2Pole` kind 3/4, `sc_kit_filter.h:29-155` | E: `RLPFar`/`RHPFar` | `qres ≥ 0.001`; `/4` coefficient ramp; ctor prime from **input's ctor value** | Sachiko (HW hang) |
| `SVF` | `sc_kit_sc3p_filter.h:33-98` | E: `SVFar` | ctor consumes one priming sample (`first_in`); cutoff `[20,sr]`, levels `[0,1]` | Fieldsteel (over budget) |
| `DFM1` | `sc_kit_dfm1.h:115-178` | E: `DFM1` **+ seed sidecar** | `time(NULL)` nondeterminism; freq/res block-rate via `SetParams` (which **lacks inputgain** — benjolis modulates gain); benjolis = **type 1 high-pass**, golden is lowpass; GPL-2+ | **none** |
| `Select` | `SelectIndex`, `sc_kit_osc2.h:258-264` | E: **`Selectkr` only** | `(int)idx+1` clipped `[1,n]`; `.ar` ungated; use generic helper, not 2-input `HkSelectIndex` | Hecker (no exact `.ar`) |
| `tanh` | `NovaTanh`, `drone_common.h:335-369` | **no kit golden** | nova cephes polynomial; **signed small-path** law; **not in PicoCollider** | Mt. Lion/Starlids/Twin Pks (hangs) |
| `DelayL` (pan) | `sc_kit_delay.h` `kind=1` | E: `DelayLar` (fixed delay) | per-block `dsamp` ramp correct for **kr** delaytime only; no production call site | none |
| `Lag` | `sc_kit_filter.h:266-285` | E: `Lagar` | for `Lag.kr` pass `sr/64`; `Prime(in0)` | Mt. Lion/Zion |
| `Clip` | `ClipTo`, `sc_kit_lfo.h:289-292` | E: **`Clipkr` only** | no `Clip.ar` golden; combined pan chain untested | Apparatus (HW OK) |
| `LeakDC` | `sc_kit_filter.h:229-247` | E: `LeakDCar` | `double` state (soft-float on M33); `Prime(in0)` → first output 0 | Apparatus (HW OK) |
| `Out.ar` | sink | golden wrapper | needs **ar** inputs; NRT renders one extra block (trim to 48,000) | every drone |

### 3.1 Important framing on "device proof"

The ledger states the 128-ref kit is **host-only and NOT linked into the audio
build** at that milestone (`.superpowers/sdd/progress.md:5352-5357`); only UGens
instantiated by the 14 live drones entered firmware. And device status is mixed
(`progress.md:5646-5659`):

- HW OK: Apparatus, Sine, Square, Supersaw, Harms Way
- over budget but running: Fieldsteel, Hecker, Mt. Zion, Thx, Triangle
- watchdog/hang: Mt. Lion, Sachiko, Starlids, Twin Pks

So "present in the firmware image" is not a clean performance guarantee. Budget
a per-module bench (`time_us_32()` per stage) and user ear acceptance.

---

## 4. The detailed issue register

### 4.1 `tanh` — not in PicoCollider, and the correct law is subtle

`NovaTanh` is at `rack-rp2350/src/dronecaster/drones/drone_common.h:335-369`
(nova `vec_tanh_float`, `vec_math.hpp:483-524`). Key law (`:335-339`, `:355-368`):
for `|x| < 0.625` the odd polynomial must be evaluated against the **signed**
`x` — using `abs_arg` "silently mirrored the negative half into the positive".
Found via Mt. Lion, whose comb output sits at `|x| ≈ 0.625`; never covered by a
reference. Ledger `progress.md:5449-5454`.

The ledger explicitly says to lift it into the kit and add a ref
(`progress.md:5453-5454`). **Do not use `tanhf`/`std::tanh`**, and do not confuse
it with Pd `ma.tanh~` (Padé `x(27+x²)/(27+9x²)`, clip ±3) or stmlib `SoftClip` —
those are different algorithms.

Caveat (review, 2026-11-22): the kit's nova-derived laws (`ScReciprocal`,
`NovaTanh`) have only ever been validated against nova *source*, never against
a rendered reference — the planned tanh ref is also where scsynth-3.14.1 vs
nova-polynomial equivalence gets *proven* (precedent: the `ScReciprocal`
ledger evidence suggests the harness reproduces nova semantics). Tie
`NovaTanh`'s adoption to that ref passing.

### 4.2 `.midicps` — three laws, pick by rate

- `sc::midicps` (`sc_defs.h:18`): `440*powf(2, (n-69)/12)` — glibc scalar
- `ScMidicps` (`drone_common.h:381-384`): SC `sc_midicps` kr `_1` path
- `NovaMidi2Freq` (`drone_common.h:371-374`): nova ar path, **truncated**
  `0.083333333333f` literal

`rungler.midicps` is audio-rate in Benjolis → use `NovaMidi2Freq`.

### 4.3 DFM1 — nondeterministic seed

`sc_kit_dfm1.h:24-26`: upstream `Dfm1_Ctor` seeds xorshift from `time(NULL)`.
With counter 0 the state is `x=0, y=1, z=2, w=3*epoch` (`:39-42`). Ledger
`progress.md:5333-5339`: renders 2 s apart differ by `max_abs 1.5e-4` — **150×
the 1e-6 gate**. `DFM1.seed.txt` holds `1795055191`; the renderer pins it just
before launch and retries if the render crosses a second boundary
(`gen_ugen_ref.scd:253-274`). `Dfm1::Init(sr, epoch, …)` takes it explicitly;
`rnor()` is consumed **three times per sample**, so drift desyncs everything.

REVIEW CORRECTION (2026-11-22, verified against upstream
`source/TJUGens/sc/TJUGens.sc` at the pinned commit 25eacaf7): the class
signature is `*ar { arg in, freq = 1000.0, res = 0.1, inputgain = 1.0,
type = 0.0, noiselevel = 0.0003, mul, add }`. Benjolis calls
`DFM1.ar(pwm, freq, q, gain, 1)` — positionally `inputgain = gain`
(CC-modulated) and **`type = 1` = high-pass mode**; `noiselevel` is omitted
and therefore takes the upstream default **0.0003 — exactly what the existing
golden used, so there is no noiselevel gap.** (0.00001 is the kit's `kNOISE`
floor constant, `sc_kit_dfm1.h:142`, not a class default — an earlier revision
of this section conflated the two.) The real new-reference need is the
**high-pass path (type=1) with modulated inputgain**; the existing golden is
lowpass (type=0), inputgain=1. Kit consequence: `Dfm1::SetParams(freq, res)`
covers only two params (`sc_kit_dfm1.h:158`) — live `gain` needs a SetParams
extension; never re-`Init` per change (that re-seeds the rnor stream). Push
freq/res/inputgain with `SetParams()` once per block (SC samples them at block
rate).

### 4.4 LocalIn/LocalOut — see §6

### 4.5 Version pinning

Goldens are SC **3.14.1**. Ledger `progress.md:5200-5227` records three
3.13→3.14 deltas: SinOsc `cpstoinc` float local, **LFTri ctor rollback**,
EnvGen ctor rollback. The ledger also contains an older 3.13 LFTri statement
(`:4960-4962`); trust the 3.14 one (`:5218`) and the test pins
(`test_dronecaster.cc:1455-1457`: sample0 = 0).

### 4.6 LinLin does not clip

`sc_kit_lin.h:4-5, 167-169`: neither UGen takes a clip argument — `clip=\minmax`
is a language-side default that never reaches the server graph.
`scale.linlin(0,1,0,127)` maps out-of-range inputs outside `[0,127]`. That's
faithful to SC; add an explicit `ClipTo` only if the design wants clamping.

### 4.7 Ctor primes must use the input's ctor value

Load-bearing Hecker law (`progress.md:5478-5481`): the
`Select→XFade2→LPF→BPF` prime chain had to be evaluated in `Init` to move corr
0.5664 → exact. Applies to Benjolis's `SVF.first_in` and `Filter2Pole` primes,
whose input is the `pwm` comparison chain. Do not blindly pass the first
rendered sample.

### 4.8 64-sample block law

`Filter2Pole` ramps coefficients one step per 4-sample group with `1/(n/4)`
(`sc_kit_filter.h:18-20`), **not** BEQ's `/3` (`progress.md:5330-5331`). Call
these APIs with `n = 64`; calling with `n = 16` changes intermediate trajectories
even though the ramp still completes at the block end. Use the
`module_dronecaster.h:41` pattern (16-sample host grid, 64-sample engine burst).

`DelayC` with an ar delaytime needs the separate per-sample `DelayCA`
(`drone_common.h:726`, ledger `:5512`) — **not** the kit's per-block ramp.
Benjolis's `DelayL` delaytime is kr (`pan` is a kr argument, `.lag` → `Lag.kr`),
so the kit path is correct here, but verify with a golden.

### 4.9 Operational rendering gotchas

- All nine delay refs require the `supernova` override
  (`gen_ugen_ref.scd:119-121`): scsynth 3.14.1-6 exits 139 *after* writing a
  correct WAV, so nonzero rc is not necessarily a failed render.
- `SVF`/`SawDPW`/`DFM1` unresolved at sclang load → silent hang after the
  welcome banner; use `-U <plugindir>` (`handoff.md:156-158`).
- Bare `a and b` hangs sclang silently; use `&&` (`:5342`).
- `Score:recordNRT` renders one block past the duration marker — trim to 48,000
  frames.

### 4.10 Golden gaps to fill

Existing goldens cover kr or different parameters. New checks needed:
`Latch.ar`, `Select.ar`, 2-channel `LocalIO`, `Clip.ar`, PulseDPW at width 0.5,
`tanh` (lift into kit), SVF/RLPF primes from the `pwm` chain, DFM1 high-pass
(`type=1`, `inputgain=gain`), and the full SynthDef.

### 4.11 Miscellaneous

- `SawDPW` divides by `freq` (`sc_kit_osc2.h:224`) → clamp `freq1/freq2 > 0`.
- `PulseDPW` golden uses width 0.3; the patch uses the default 0.5.
- `Select.ar` instantiates **all four** filter branches and runs them every
  block; only the selected output is audible. The engine may run just the
  selected filter per block (sonically exact — saves DFM1's 3 `rnor` draws per
  sample when unselected) but must still ctor-prime every branch at Init (§4.7).
- Benjolis scales the filters by `gain`: RLPF/RHPF via **mul**, SVF/DFM1 via
  their `gain`/`inputgain` args. The kit filter ports are UGen-only (no mul
  param — `Filter2Pole::ProcessBlock` has none) → apply `out *= gain`
  engine-side, once per block.
- `outSignal` actually selects **7** taps (0 tri1, 1 osc1, 2 tri2, 3 osc2,
  4 pwm, 5 comparator, 6 filt; default 6) — the file header comment says 6.
  Expose 0–6 in the CC map.
- License: DFM1 and SVF are GPL-2.0-or-later
  (`PicoCollider/LICENSE.md:10-14`), unlike the rest of the kit's GPL-3.0.
- SRAM/XIP law (`docs/PORTING-HIGHLIGHTS.md` §3): hot DSP must run from SRAM;
  per-function unique `-O3` wrapper token; verify with `nm` that symbols are at
  `0x2000xxxx`, not `0x1000xxxx`. Avoid per-sample libm — another reason to use
  the nova helpers rather than `powf`/`tanhf`.

---

## 5. What LocalIn/LocalOut is for, how fast it is, and whether hardware needs it

### 5.1 Purpose

SC graphs are acyclic; `LocalOut`/`LocalIn` is the in-SynthDef feedback bus. In
Benjolis it carries two signals:

| Channel | Written by `LocalOut` | Read as | Used for |
|---|---|---|---|
| 0 | `rungler = DAC.midicps` | `rungler` | FM on both oscillators: `LFTri/PulseDPW((rungler*rungler1)+freq1)` and `…rungler2+freq2` |
| 1 | `buf` = raw rungler DAC (0…0.996) | `buf` | LOOP switch: `osc1 = (buf*loop) + (osc1*(1-loop))` — recycles shift-register output as its own data input |

The loop is: **O2 clocks the rungler → DAC → O1/O2 frequencies + shift-register
data input → new rungler bits**. Not Benjolin-specific — 12 Dronecaster drones
use the same mechanism.

### 5.2 Speed

- **Latency: exactly one 64-sample block** = **1.33 ms @ 48 kHz** (1.45 ms @
  44.1 kHz). Kit law: `LocalIn` reads when `counter - touched` is 0 or 1;
  `LocalOut` copies on the first write of a block and accumulates thereafter
  (`sc_kit_localio.h:8-10, 28-60`).
- **Cost: trivial.** 2 ch × 64 floats = 128 floats = **512 B**; two
  `ProcessBlock` calls per block. CPU is not the issue; correctness is.
- **Relative to the rungler clock:** the shift register only changes on an O2
  rising edge. Default `freq2 = 4 Hz` → one latch every 250 ms, so 1.33 ms is
  ~0.5% of a period — negligible. At O2 = 100 Hz (10 ms period) the delay is
  ~13% of a clock period and starts to matter as timing jitter.
- **Real hardware:** the rungler is a CD4021 shift register + 3-bit resistor DAC
  wired directly to the oscillator FM inputs. Propagation is nanoseconds. The
  one-block delay is an SC scheduler artifact, not an instrument property.

### 5.3 Does hardware need it?

**No — not as a bus.** In C++ the same loop is two persistent floats plus the
shift-register state:

```cpp
float rungler_hz_;    // last DAC -> midicps result
float rungler_raw_;   // last raw DAC value (LOOP source)
// update on osc2 rising edge; O1/O2 read these directly
```

No bus, no block counter, no read/write ordering — simpler and *closer* to the
instrument than the SC version.

**But validation diverges.** The bit-exact SC golden encodes the one-block
delay. Folding to direct per-sample state changes the phase of a chaotic system
and can decorrelate the waveform completely — the same reason chaotic UGens are
law-only in this repo. Two options:

- **A — reference-exact:** keep the kit `LocalBus` (2 ch, 64-sample blocks,
  `LocalIn` at block start, `LocalOut` at block end, monotonic counter). ~512 B
  plus bookkeeping. Full-engine 1e-6 gate stays meaningful. Requires doing the
  firmware wiring Dronecaster never finished (§4.4).
- **B — hardware-faithful:** fold into direct state. Simpler and truer to the
  instrument, but the full-waveform golden no longer applies. Validate the
  rungler via scenario tests: 8-bit shift-register sequence, bit transitions vs
  O2 edges, DAC weight law (`1/256 … 1/2`), spectral envelope, RMS — plus ear
  check. This is the law-only + scenario approach Lira8 ended up using.

**Recommendation:** start with **A** so the SC golden gives a strong correctness
signal; expose **B** as a "hardware mode" once the engine is proven. Make this an
explicit design choice, not an accident.

---

## 6. Risk register (ranked)

1. **LocalIO 2-channel ar feedback** — mono-only golden, firmware wiring never
   completed (ledger `:5284`, `handoff.md:169`); four drones were deferred over
   exactly this. Required by Benjolis.
2. **DFM1 `time(NULL)` seed** — 150× gate drift without the epoch sidecar; no
   device policy; no production use.
3. **`tanh`** — nova law + sign fix + missing kit golden + firmware-only header.
4. **Version pinning** — LFTri rollback (3.13 vs 3.14).
5. **LinLin no clip** — out-of-range `scale` must be handled deliberately.
6. **Ctor-prime chain** — input's ctor value, not first rendered sample.
7. **Golden gaps** — `.ar` forms of Latch/Select/Clip, 2-ch LocalIO.
8. **Device performance** — several UGens only appear in HW-hang/over-budget
   drones; per-module bench required.

---

## 7. Proposed port plan

1. **Lift nova math helpers** (`NovaTanh`, `NovaExpTanh`, `NovaPow`,
   `NovaMidi2Freq`) into a PicoCollider header; add a `Tanh` golden ref and a
   `midicps` ar ref. Do not include `drone_common.h` wholesale.
2. **Engine** `rack-rp2350/src/benjolis/benjolis_engine.h` — header-only, no MCU
   headers, 16-sample host blocks accumulated into 64-sample internal blocks;
   assemble PicoCollider UGens; two feedback modes (A/B from §5.3); freq clamps.
3. **Goldens** — render the full SynthDef with SC 3.14.1 + sc3-plugins, capture
   the DFM1 epoch sidecar, use `supernova` for delay refs, trim to 48,000 frames.
4. **Host tests** — `rack-rp2350/test/test_benjolis.cc` + Makefile target:
   per-UGen goldens, the targeted `.ar` law checks from §4.10, shift-register
   sequence, ctor-prime chain, 64-sample ramp, mode A/B equivalence scope.
   Run the binary individually, not via aggregate `make test`.
5. **Adapter** — `rack-rp2350/src/module_benjolis.h` following
   `module_lira8.h`: arena-placed state, atomics/pending params, CC map,
   `Clip16` output, `-O3` wrapper with a unique SRAM token, window/registry
   tables, `RACK_BOOT_MODULE=6` (next free id), CC map in
   `tools/rack_panel.html`, presets/C3 page as needed.
6. **Acceptance** — `pio run`, `nm` SRAM check, flash + boot log + per-stage
   bench, raw-MIDI parameter sweep, user ear check + 30 min soak, and a ledger
   entry with commit hash and findings.

---

## 8. Bottom line

The SC Benjolis port is a **configuration-and-validation** job, not a DSP
invention job: every required UGen has a port, though only ~9 have exact
goldens at the exact rate/params Benjolis uses (the `E` rows of §3) — §4.10 is
the honest worklist.
The effort is concentrated in three places — the rungler feedback wiring, the
nova-math helpers that live in the wrong header, and a new full-engine golden
with its DFM1 epoch sidecar. None is a blocker; all are well-defined tasks with
precedent in the Lira8 and Dronecaster work.
