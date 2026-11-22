# Benjolin Port — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the SuperCollider Benjolin (`scazan/benjolis`, Alejandro Olarte SynthDef) to the rack as module id 6, reference-exact against an SC 3.14.1 full-SynthDef golden.

**Architecture:** Three layers, gated in order: (1) PicoCollider kit lift (nova tanh + DFM1 SetParams extension + 8 gap goldens), (2) a host-testable `benjolin_engine.h` assembling kit UGens on the 64-sample scsynth block grid with the 2-channel LocalIn/LocalOut feedback bus (mode A) plus a hardware-faithful explicit shift-register mode (mode B), (3) the `module_benjolin.h` adapter following the `module_dronecaster.h` 16→64 burst pattern.

**Tech Stack:** PicoCollider kit headers (submodule), plain-g++ host tests in `rack-rp2350/test/`, scsynth/sclang 3.14.1 + sc3-plugins (system-wide, `/usr/local/bin`), PlatformIO RP2350 firmware.

**Spec:** `docs/ASSESSMENT-benjolin-port-2026-11-22.md` (§4 issue register, §5 LocalIn/LocalOut decision, §7 plan) — including the 2026-11-22 review corrections (degenerate sh0 comparator §2.1; DFM1 `type=1` high-pass + `inputgain=gain` §4.3). The upstream SynthDef is `/tmp/opencode/benjolis-sc/lib/Engine_Benjolis.sc` (clone it again if `/tmp` was wiped: `git clone https://github.com/scazan/benjolis /tmp/opencode/benjolis-sc`).

## Global Constraints

- Golden gate everywhere: `max_abs ≤ 1e-6`, `corr ≥ 0.999999` (kit discipline). NRT renders are trimmed to exactly 48,000 frames (scsynth renders one block past the marker).
- SC version pin: **3.14.1** (LFTri ctor rollback law; never regress to 3.13 behavior).
- Engine internal grid = **64 samples** (scsynth `kBlock`, `sc_defs.h:10`); host/device grid = 16 samples, burst 4:1.
- Engine header is host-testable: **no MCU headers**, no `Serial`/`time_us_32`; device-only allocation paths go through the adapter, not the engine.
- No per-sample libm on device. `midicps` is evaluated **once per 64-block** in double precision (see Task 3 law note) — that is the sanctioned exception.
- Device DFM1 seed policy (decides §4.3's open question): seed `Dfm1::Init` with the golden epoch constant `1795055191` (`DFM1.seed.txt`) so device output is deterministic and golden-comparable. Never re-`Init` a live Dfm1 — re-seeding restarts the rnor stream.
- All filter `gain`/`mul` scaling is applied **engine-side per block** (kit filter ports are UGen-only).
- Module id **6** (`RACK_MODULE_BENJOLIN`); `kChainABufferBytes[6] = 0` (standalone-only, like dronecaster). Boot module set to 6 for acceptance, reverted at ledger time if the user prefers Elements boot.
- Hot DSP in SRAM: the device wrapper carries a unique `.tc_benjolin` section token; verify with `nm` that symbols sit at `0x2000xxxx`.
- PicoCollider is a **git submodule**: every kit change = commit inside `PicoCollider/`, then a pointer-bump commit in the superproject.
- Commits follow `[Add]/[Fix]/[Update]/[Remove] <scope>` on branch `rack-rp2350`.
- `make test` aggregate is not fully trustworthy (Elements `test_reverb` ASLR canary) — always run the relevant test binary individually.

## Review Focus

Failure modes the spec implies but that only a human ear or a specific input will expose — each is pinned by the named test:

1. **DFM1 rnor stream desync** — re-`Init`-per-param-change silently restarts the noise stream; expect live `gain` changes to move only tone, not noise. Pinned: Task 1 unit check (SetParams ≡ Init bit-exact).
2. **Comparator "fix" regression** — someone "restoring" the intended sh8-XOR changes the rungler bitstream entirely; expect the golden wave, not the intent. Pinned: Task 4 full-SynthDef golden (mode A).
3. **LocalBus channel order swap** — ch0 must carry `midicps(rungler)` (FM) and ch1 the raw DAC (LOOP); swapped channels still sound plausible but change the chaos. Pinned: Task 4 golden + Task 2 2-ch LocalIO ref.
4. **midicps float-vs-double law drift** — a float-pow implementation accumulates phase error far past the 1e-6 gate over 1 s. Pinned: Task 2 `Midicps` ref.
5. **DPW frequency ≤ 0** — `SawDPW` divides by freq; negative `rungler1`·rungler + freq1 can go through zero on device and produce inf/NaN. Expect clamped positive freq, no NaN blocks. Pinned: Task 3 unit check (clamp law) + adapter NaN sweep (Task 5).

---

### Task 1: PicoCollider lift — `sc_kit_nova.h` + `Dfm1::SetParams` extension

**Files:**
- Create: `PicoCollider/include/picocollider/sc_kit_nova.h`
- Modify: `PicoCollider/include/picocollider/sc_kit_dfm1.h` (SetParams only)
- Test: `PicoCollider/test/test_kit.cc` (append checks)

**Interfaces:**
- Consumes: nothing new (kit headers only).
- Produces: `float sc::NovaTanh(float)`, `float sc::NovaExpTanh(float)` (kit-grade, header-only); `Dfm1::SetParams(float freq, float res, float input_gain)` (2-arg overload retained). Task 3's engine consumes both.

- [ ] **Step 1: Create `sc_kit_nova.h`**

Header comment: nova vec math lifted from `rack-rp2350/src/dronecaster/drones/drone_common.h` (GPL-3.0, nova `vec_math.hpp`), with the signed-small-path tanh law. Copy **verbatim**:

- `NovaExpTanh` — copy the whole `inline float NovaExpTanh(float x)` block from `drone_common.h` (grep `-n 'NovaExpTanh'`; it is self-contained apart from `<cmath>`/`<cstdint>`/`<cstring>`). Include its doc comment.
- `NovaTanh` — this exact body (the sign law is the load-bearing part; keep both comments):

```cpp
// nova vec_tanh_float (cephes-style SSE polynomial; branches are lane masks).
// LAW (vec_math.hpp:483-524): the SMALL path (|x| < 0.625) evaluates the odd
// polynomial against the SIGNED arg (`* arg_sqr * arg + arg`) — dropping the
// sign there silently mirrors the negative half into the positive (found the
// hard way via Mt. Lion, whose comb output sits at |x| ≈ 0.625).
inline float NovaTanh(float x) {
  uint32_t xb;
  std::memcpy(&xb, &x, 4);
  const uint32_t sign = xb & 0x80000000u;
  const uint32_t absb = xb & 0x7FFFFFFFu;
  float abs_arg;
  std::memcpy(&abs_arg, &absb, 4);
  // medium values: 1 - 2/(exp(2|x|) + 1)
  const float medium = 1.0f - 2.0f / (NovaExpTanh(abs_arg + abs_arg) + 1.0f);
  // large values (|x| > 22): +-1
  float result_lm_abs = (abs_arg > 22.0f) ? 1.0f : medium;
  uint32_t lmb;
  std::memcpy(&lmb, &result_lm_abs, 4);
  lmb |= sign;
  std::memcpy(&result_lm_abs, &lmb, 4);
  // small values (|x| < 0.625): odd polynomial in the SIGNED x (nova
  // vec_math.hpp:516 — `* arg_sqr * arg + arg` — the sign must survive)
  const float f1 = -5.70498872745e-3f;
  const float f2 = 2.06390887954e-2f;
  const float f3 = -5.37397155531e-2f;
  const float f4 = 1.33314422036e-1f;
  const float f5 = -3.33332819422e-1f;
  const float arg_sqr = abs_arg * abs_arg;
  const float result_small =
      ((((f1 * arg_sqr + f2) * arg_sqr + f3) * arg_sqr + f4) * arg_sqr + f5) *
          arg_sqr * x +
      x;
  return (abs_arg < 0.625f) ? result_small : result_lm_abs;
}
```

Namespace `sc`, include guard `RACK_PICOCOLLIDER_SC_KIT_NOVA_H_` (kit style). **Do not** lift `NovaPow`/`NovaMidi2Freq` — benjolin uses the per-block double `midicps` law instead (Task 3; deviation from assessment §4.2, recorded there).

- [ ] **Step 2: Extend `Dfm1::SetParams` (kit)**

In `sc_kit_dfm1.h` replace:

```cpp
  void SetParams(float freq, float res) { freq_ = freq; res_ = res; }
```

with:

```cpp
  void SetParams(float freq, float res) { SetParams(freq, res, gain_); }
  // Benjolin modulates inputgain live; upstream samples it at block rate
  // alongside freq/res (TJUGens wrapper). Never re-Init to change gain —
  // that would re-seed the rnor stream.
  void SetParams(float freq, float res, float input_gain) {
    freq_ = freq; res_ = res; gain_ = input_gain;
  }
```

(`type`/`noise_level` stay Init-time — benjolin fixes them at 1 / 0.0003.)

- [ ] **Step 3: Write the failing unit checks (append to `test_kit.cc` main)**

```cpp
  // ---- nova tanh law (sc_kit_nova.h) ----
  {
    // odd function: sign must survive the small path (|x| < 0.625).
    if (!(std::abs(sc::NovaTanh(-0.5f) + sc::NovaTanh(0.5f)) < 1e-9f)) ++g_failures;
    // sign at the boundary: -0.63 must be NEGATIVE (the abs_arg bug made it ~+0.555).
    if (!(sc::NovaTanh(-0.63f) < -0.55f)) ++g_failures;
    // continuity across the 0.625 path switch (polynomial vs exp medium path).
    if (!(std::abs(sc::NovaTanh(0.625f) - sc::NovaTanh(0.6249f)) < 5e-3f)) ++g_failures;
    // saturation.
    if (!(sc::NovaTanh(30.0f) > 0.999999f && sc::NovaTanh(-30.0f) < -0.999999f)) ++g_failures;
  }
  // ---- Dfm1 SetParams(3-arg) ≡ Init (bit-exact) ----
  {
    sc::Dfm1 a, b;
    a.Init(48000.0, 1795055191, 1000.f, 0.1f, 1.f, 0.f, 0.0003f);
    b.Init(48000.0, 1795055191, 440.f, 0.2f, 2.f, 0.f, 0.0003f);
    b.SetParams(1000.f, 0.1f, 1.f);
    float ia[64], ib[64]; const float in[64] = {};  // zero input; noise drives diff
    a.ProcessBlock(in, ia, 64);
    b.ProcessBlock(in, ib, 64);
    for (int i = 0; i < 64; ++i)
      if (ia[i] != ib[i]) { ++g_failures; break; }
  }
```

- [ ] **Step 4: Run the kit gate**

Run: `cd PicoCollider/test && make test && ./build/test_kit` (run the binary individually, not just the aggregate).
Expected: all existing 133 golden refs still pass, new checks pass.

- [ ] **Step 5: Commit (submodule, then pointer bump)**

```bash
cd PicoCollider && git add include/picocollider/sc_kit_nova.h include/picocollider/sc_kit_dfm1.h test/test_kit.cc && git commit -m "[Add] nova tanh kit header + Dfm1 SetParams inputgain (benjolin prep)"
cd .. && git add PicoCollider && git commit -m "[Update] PicoCollider: nova tanh + Dfm1 SetParams (benjolin prep)"
```

---

### Task 2: Kit gap goldens — 8 new refs + kit checks (§4.10 list)

**Files:**
- Modify: `PicoCollider/test/ref/gen_ugen_ref.scd` (entries + DFM1 pin condition)
- Modify: `PicoCollider/test/test_kit.cc` (8 `ref_check`s)

**Interfaces:**
- Consumes: `sc::NovaTanh` (Task 1).
- Produces: golden WAVs `Tanh`, `Midicps`, `Latchar`, `Selectar`, `Clipar`, `LocalIO2`, `DFM1hp`, `PulseDPWar05` in `PicoCollider/test/ref/ugen_ref/`. Task 3/4 reuse the laws these pin.

- [ ] **Step 1: Add the 8 renderer entries to `gen_ugen_ref.scd`**

Append to `entries` (keep the file's one-line style; RandSeed pinning already wraps every synth):

```supercollider
        // ---- benjolin prep (2026-11-22): the .ar forms + params benjolin uses ----
        ["Tanh",         { Out.ar(0, (SinOsc.ar(220) * 3).tanh.dup) }],
        ["Midicps",      { Out.ar(0, K2A.ar(LFNoise0.kr(10).range(0, 127)).midicps.dup) }],
        ["Latchar",      { Out.ar(0, Latch.ar(SinOsc.ar(220), Impulse.ar(100)).dup) }],
        ["Selectar",     { Out.ar(0, Select.ar(LFNoise0.ar(400).range(0, 3),
                            [DC.ar(0.1), SinOsc.ar(220), LFSaw.ar(110), DC.ar(0.5)]).dup) }],
        ["Clipar",       { Out.ar(0, LFNoise1.ar(400).range(-2, 2).clip(-0.5, 0.5).dup) }],
        ["PulseDPWar05", { Out.ar(0, PulseDPW.ar(220, 0.5).dup) }],
        // DFM1 in the SHAPE benjolin uses: type=1 (high-pass), noiselevel =
        // upstream default 0.0003, inputgain=1 (benjolin's live gain is
        // covered by the full-SynthDef golden, not a kit ref).
        ["DFM1hp",       { Out.ar(0, DFM1.ar(Saw.ar(220), 1000, 0.1, 1, 1, 0.0003)) }],
        // 2-channel LocalIn/LocalOut feedback: ch0 * 0.5 + osc, ch1 * 0.25 + osc;
        // monitor = ch0 + ch1 (mono sum keeps the ref_check harness unchanged).
        ["LocalIO2",     { var in = LocalIn.ar(2, 0);
                           LocalOut.ar([in[0] * 0.5 + SinOsc.ar(220) * 0.1,
                                        in[1] * 0.25 + LFSaw.ar(110) * 0.1]);
                           Out.ar(0, (in[0] + in[1]).dup) }],
```

Then change the epoch-pinning condition so both DFM1 refs get a sidecar: replace `if (name == "DFM1") {` with `if (name.startsWith("DFM1")) {` (the `DFM1.seed.txt` write becomes `outDir ++ "/" ++ name ++ ".seed.txt"`).

- [ ] **Step 2: Render**

Run: `cd PicoCollider/test && bash ref/run_ugen_ref.sh`
Expected: 8 new WAVs in `ref/ugen_ref/` (+ `DFM1hp.seed.txt`); exit 139 from scsynth after a correct WAV is the known 3.14.1-6 behavior — verify WAV presence, not rc.

- [ ] **Step 3: Add the 8 kit checks to `test_kit.cc`**

Each mirrors an existing check's shape (`ref_check(name, out.data(), 48000)`). Input laws:

```cpp
  // ---- benjolin prep refs ----
  // Tanh: kit SinOsc(220) * 3 -> NovaTanh per sample.
  // Midicps: kit LFNoise0.kr(10) block-held, .range law x*63.5f + 63.5f,
  //   then the ENGINE law per sample: (float)(440.0 * pow(2.0, (n-69.0)/12.0)).
  //   (double pow then float cast — scsynth's ar midicps law; float powf
  //   drifts phase past the 1e-6 gate.)
  // Latchar: kit SinOsc(220) + kit Impulse(100); sc::Latch Next() per sample.
  // Selectar: kit LFNoise0.ar(400) * 1.5 + 1.5 (range law), sc::SelectIndex
  //   per sample over the 4 kit-rendered sources.
  // Clipar: kit LFNoise1.ar(400) range(-2,2) law, sc::ClipTo per sample.
  // PulseDPWar05: kit PulseDPW::Init(48000, 220, 0.5f, 0) Next() per sample.
  // DFM1hp: mirror the existing DFM1 check (kit Saw(220) input, epoch from
  //   DFM1hp.seed.txt) with type=1: Init(48000, epoch, 1000, 0.1, 1, 1, 0.0003).
  // LocalIO2: one sc::LocalBus(2); per 64-block: LocalIn x2 -> the two
  //   feedback laws -> LocalOut x2 -> out = ch0 + ch1 summed per sample.
```

Render the source UGens with the same kit APIs the existing passing checks use (grep the check names in `test_kit.cc` — SinOsc/LFNoise/Saw rendering already exists). Each block: build the input stream per the ref graph, apply the law, `ref_check`.

- [ ] **Step 4: Run**

Run: `cd PicoCollider/test && make test && ./build/test_kit`
Expected: 141 refs pass. If `Midicps` fails, the double-pow law assumption is wrong — re-derive from the WAV (do not loosen the gate).

- [ ] **Step 5: Commit (submodule + pointer bump)**

```bash
cd PicoCollider && git add -A test/ref test/test_kit.cc && git commit -m "[Add] benjolin-prep goldens: ar forms, 2-ch LocalIO, DFM1hp, midicps law"
cd .. && git add PicoCollider && git commit -m "[Update] PicoCollider: 8 benjolin-prep goldens"
```

---

### Task 3: Benjolin engine (host) — `benjolin_engine.h` + structural tests

**Files:**
- Create: `rack-rp2350/src/benjolin/benjolin_engine.h`
- Create: `rack-rp2350/test/test_benjolin.cc`
- Modify: `rack-rp2350/test/Makefile` (add the `test_benjolin` rule mirroring `test_dronecaster`'s)

**Interfaces:**
- Consumes: kit headers (`-I ../../PicoCollider/include/picocollider`), `sc::NovaTanh` (Task 1), `Dfm1::SetParams(freq,res,input_gain)` (Task 1).
- Produces (Task 4 + 5 consume): `benjolin::Params` (SynthDef defaults), `benjolin::Engine::Init(double sr, FeedbackMode, int dfm1_epoch, const Params&, float* ext_tap_rings)`, `SetParams(const Params&)` (applied at next block), `Process64(float* l, float* r)`, `Process16(float* l, float* r)` (16-grid burst 4:1).

- [ ] **Step 1: Write the engine header**

```cpp
// benjolin_engine.h — reference-exact port of the \benjolis SynthDef
// (Engine_Benjolis.sc, Alejandro Olarte / Scott Cazan) on the PicoCollider kit.
// Host-testable: no MCU headers. See docs/ASSESSMENT-benjolin-port-2026-11-22.md.
//
// Laws encoded here:
//  - 64-sample internal blocks = the scsynth grid (sc_defs.h kBlock).
//  - LocalIn/LocalOut 2-ch bus, 1-block latency (sc_kit_localio.h). ch0 =
//    midicps(rungler) [FM], ch1 = raw rungler DAC [LOOP].
//  - DEGENERATE COMPARATOR (review finding, do NOT "fix"): the SynthDef's
//    `sh8 = 1` var initializer is captured at graph build before the DelayN
//    assigns the register UGen, so the runtime comparator is
//      sh0 = 1 - ((1 > A) == (1 < A)),  A = (osc1 > 0.5)
//    which folds to 1 - (osc1 > 0.5). The full-SynthDef golden encodes this
//    form; restoring the intended sh8-XOR would be a deliberate deviation.
//  - rungler.midicps: the input is block-constant (LocalIn ch0), so it is
//    evaluated ONCE per 64-block in double precision and cast to float —
//    scsynth's ar midicps law. NovaMidi2Freq (float pow) drifts phase past
//    the 1e-6 gate over the 1 s golden (deviation from assessment §4.2).
//  - Filter gain: RLPF/RHPF mul and SVF/DFM1 inputgain are applied HERE per
//    block — the kit filter ports are UGen-only.
//  - DPW freq clamp: SawDPW divides by freq; SC would produce inf/NaN at
//    freq <= 0. Engine clamps to >= 1 Hz (documented device-safety deviation).
#ifndef RACK_SRC_BENJOLIN_BENJOLIN_ENGINE_H_
#define RACK_SRC_BENJOLIN_BENJOLIN_ENGINE_H_
#include <cmath>
#include <cstdint>

#include "sc_kit_osc.h"        // LFTri
#include "sc_kit_osc2.h"       // PulseDPW, SelectIndex
#include "sc_kit_trig.h"       // Latch
#include "sc_kit_delay.h"      // DelayLine (kind 0 taps, kind 1 pan)
#include "sc_kit_filter.h"     // Filter2Pole, Lag, LeakDC
#include "sc_kit_sc3p_filter.h"// SVF
#include "sc_kit_dfm1.h"       // Dfm1
#include "sc_kit_localio.h"    // LocalBus, LocalIn, LocalOut
#include "sc_kit_lfo.h"        // ClipTo
#include "sc_kit_nova.h"       // NovaTanh

namespace benjolin {

// SynthDef arg defaults, verbatim from Engine_Benjolis.sc:26 (amp=0 silence;
// the caller SetParams before the first render). pan is the SC arg — the
// norns command multiplies the UI value by 0.001; the adapter maps CC to
// SC units.
struct Params {
  float freq1 = 40.0f, freq2 = 4.0f, scale = 1.0f;
  float rungler1 = 0.16f, rungler2 = 0.0f, rungler_f = 9.0f;
  float loop = 0.0f, filt_freq = 40.0f, q = 0.82f, gain = 1.0f;
  float filter_type = 0.0f, out_signal = 6.0f, amp = 0.0f, pan = 0.0f;
};

enum class FeedbackMode { kReferenceSc = 0, kHardware = 1 };

// Benjolin 3-bit DAC weights (sh1/256 + ... + sh8/2, max 255/256 = 0.996).
// All powers of two, so multiply-by-reciprocal is bit-identical to SC's
// `x / 2.pow(k)` divisions.
inline float RunglerDac(const float* sh /* sh1..sh8 */) {
  return sh[0] * (1.0f / 256.0f) + sh[1] * (1.0f / 128.0f) +
         sh[2] * (1.0f / 64.0f) + sh[3] * (1.0f / 32.0f) +
         sh[4] * (1.0f / 16.0f) + sh[5] * (1.0f / 8.0f) +
         sh[6] * (1.0f / 4.0f) + sh[7] * 0.5f;
}

inline float Midicps(double n) {  // scsynth ar midicps law, once per block
  return static_cast<float>(440.0 * std::pow(2.0, (n - 69.0) / 12.0));
}
inline float DpwClamp(float f) { return f < 1.0f ? 1.0f : f; }

struct Engine {
  static constexpr int kBlock = 64;

  void Init(double sr, FeedbackMode mode, int dfm1_epoch, const Params& p,
            float* ext_tap_rings = nullptr) {
    sr_ = static_cast<float>(sr); mode_ = mode; p_ = p;
    block_ = 0; stage_ = 0;
    bus_.Init(2);
    // LFTri 3.14 ctor primes then rolls back -> first sample at phase 0.
    tri1_.Init(sr, p_.freq1); tri2_.Init(sr, p_.freq2);
    // PulseDPW default width 0.5 (SynthDef passes no width arg).
    osc1_.Init(sr, DpwClamp(p_.freq1), 0.5f, 0);
    osc2_.Init(sr, DpwClamp(p_.freq2), 0.5f, 0);
    // 8 register taps: DelayN.ar(in, 0.01, sr*k), k = 1..8. Constant delay.
    for (int k = 0; k < 8; ++k) {
      const float d = static_cast<float>(k + 1) / static_cast<float>(sr);
      if (ext_tap_rings != nullptr)
        tap_[k].InitExt(ext_tap_rings + k * 16, 16, sr, 0.01f, d, 0.f, 0);
      else
        tap_[k].Init(sr, 0.01f, d, 0.f, 0);
    }
    // Ctor primes (§4.7): the pwm chain's ctor value is 0.0f — both LFTri
    // outputs are 0 at phase 0, so (0 + 0) > 0 = 0. Prime each filter with
    // one n=1 block at ctor-time coefficients (freq = filt_freq, rq = 1-q).
    // NOTE: arbitrated by the Task 4 golden — if it shows a one-sample lead,
    // the kit's NaN-freq ctor already folds the prime; remove this block.
    rlpf_.Init(sr, 3); rhpf_.Init(sr, 4);
    const float z = 0.0f; float sink;
    rlpf_.ProcessBlock(&z, &sink, 1, p_.filt_freq, Rq());
    rhpf_.ProcessBlock(&z, &sink, 1, p_.filt_freq, Rq());
    svf_.Init(sr, p_.filt_freq, p_.q, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    // Benjolin DFM1: inputgain=gain, TYPE=1 (high-pass), noiselevel omitted
    // -> upstream default 0.0003 (see assessment §4.3 correction).
    dfm1_.Init(sr, dfm1_epoch, p_.filt_freq, p_.q, p_.gain, 1.0f, 0.0003f);
    // Lag.kr emulation (kit law: pass sr/64, call once per block).
    lag_l_.Init(sr / 64.0); lag_r_.Init(sr / 64.0);
    // DelayL: maxdelaytime 0.01 s; upstream clamps delaytime to it (CalcDelay
    // m_maxdelay) — done engine-side in Process64 (see DelayTimeR()).
    dl_l_.Init(sr, 0.01f, 0.0f, 0.f, 1);
    dl_r_.Init(sr, 0.01f, 0.0f, 0.f, 1);
    leak_l_.Init(); leak_l_.Prime(0.f);  // chain ctor value is 0.0
    leak_r_.Init(); leak_r_.Prime(0.f);
    rungler_hz_ = 0.0f; rungler_raw_ = 0.0f; buf_ = 0.0f;
    reg_ = 0;  // mode B explicit register
  }

  void SetParams(const Params& p) { pending_ = p; pending_set_ = true; }

  // One 16-sample host/device block; bursts 4:1 into the 64-grid.
  void Process16(float* l, float* r) {
    if (stage_ == 0) Process64(stage_l_, stage_r_);
    const int off = stage_ * 16;
    for (int i = 0; i < 16; ++i) { l[i] = stage_l_[off + i]; r[i] = stage_r_[off + i]; }
    stage_ = (stage_ + 1) & 3;
  }

  float Rq() const { const float rq = p_.q * -1.0f + 1.0f; return rq < 0.001f ? 0.001f : rq; }

  // RATE LAWS (self-review 2026-11-22):
  //  - rungler.midicps is PER-SAMPLE ar: the register taps stagger each edge
  //    over 8 samples, so the DAC sum (and the bus content) transitions
  //    per-sample; scsynth evaluates sc_midicps (double pow, float cast) on
  //    every sample. The bus ch0 read (FM) is therefore a per-sample
  //    trajectory too — tri/DPW freqs are SetFreq per sample.
  //  - RLPF/RHPF/SVF get ar freq -> upstream next_a recomputes coefficients
  //    immediately per sample (no kr ramp) -> kit call is n=1 per sample.
  //  - DFM1 samples its params at BLOCK rate (TJUGens IN0) and ramps the
  //    coefficient slopes across the block -> ONE ProcessBlock(64) call with
  //    SetParams(filt_freq[0], ...) beforehand. Per-sample n=1 calls would
  //    jump the ramps and desync the kit's trajectory law.
  void Process64(float* out_l, float* out_r) {
    if (pending_set_) { p_ = pending_; pending_set_ = false; }
    // LocalIn.ar(2, 0): ch0 = midicps(rungler) trajectory [FM],
    // ch1 = raw DAC trajectory [LOOP].
    float* inr[2] = { in0_, in1_ };
    const float defaults[2] = { 0.f, 0.f };
    if (mode_ == FeedbackMode::kReferenceSc) {
      lin_.ProcessBlock(bus_, inr, defaults, block_, kBlock);
    } else {
      for (int i = 0; i < kBlock; ++i) { in0_[i] = rungler_hz_; in1_[i] = buf_; }
    }
    const float loop_a = p_.loop, loop_b = p_.loop * -1.0f + 1.0f;

    // Phase 1 — per-sample sources (freqs set per sample from the bus).
    for (int i = 0; i < kBlock; ++i) {
      const float fm1 = in0_[i] * p_.rungler1 + p_.freq1;
      const float fm2 = in0_[i] * p_.rungler2 + p_.freq2;
      tri1_.SetFreq(fm1); tri2_.SetFreq(fm2);
      osc1_.SetFreq(DpwClamp(fm1)); osc2_.SetFreq(DpwClamp(fm2));
      tri_buf_[i] = tri1_.Process(); tri2_buf_[i] = tri2_.Process();
      o1_buf_[i] = osc1_.Next(); o2_buf_[i] = osc2_.Next();
      pwm_buf_[i] = ((tri_buf_[i] + tri2_buf_[i]) > 0.0f) ? 1.0f : 0.0f;
      osc1m_buf_[i] = in1_[i] * loop_a + o1_buf_[i] * loop_b;
      // DEGENERATE comparator — literal graph form (folds to 1-(x>0.5)).
      const float a = (osc1m_buf_[i] > 0.5f) ? 1.0f : 0.0f;
      const float gt = (1.0f > a) ? 1.0f : 0.0f;
      const float lt = (1.0f < a) ? 1.0f : 0.0f;
      const float sh0 = 1.0f - ((gt == lt) ? 1.0f : 0.0f);
      sh0_buf_[i] = sh0;
      // 8-stage register: Latch(prev stage, osc2) -> DelayN(sr*k).
      float latched = sh0;
      for (int k = 0; k < 8; ++k) {
        latched = latch_[k].Next(latched, o2_buf_[i]);
        tap_[k].ProcessBlock(&latched, &sh_buf_[k][i], 1, tap_delay_[k], 0.f);
      }
    }
    for (int i = 0; i < kBlock; ++i) {
      float sh[8];
      for (int k = 0; k < 8; ++k) sh[k] = sh_buf_[k][i];
      raw_buf_[i] = RunglerDac(sh);
      hz_buf_[i] = Midicps(static_cast<double>(raw_buf_[i] * 127.0f * p_.scale));
      // mode B: explicit 8-bit register shifted on the latch edge (hardware
      // law, §5.3 option B — scenario-gated, no golden).
      if (mode_ == FeedbackMode::kHardware && o2_buf_[i] > 0.0f && o2_prev_ <= 0.0f) {
        reg_ = ((reg_ << 1) | (static_cast<uint32_t>(sh0_buf_[i]) & 1u)) & 0xffu;
        rungler_raw_ = RunglerDacBits(reg_);
        buf_ = rungler_raw_;
        rungler_hz_ = Midicps(static_cast<double>(buf_ * 127.0f * p_.scale));
        in0_[i] = rungler_hz_; in1_[i] = buf_;  // hardware: immediate
      }
      o2_prev_ = o2_buf_[i];
      filt_freq_buf_[i] = in0_[i] * p_.rungler_f + p_.filt_freq;
    }

    // Phase 2 — selected filter branch (all primed at Init).
    const int type = sc::SelectIndex(p_.filter_type, 4);
    switch (type) {
      case 0:
        for (int i = 0; i < kBlock; ++i) { float fo;
          rlpf_.ProcessBlock(&pwm_buf_[i], &fo, 1, filt_freq_buf_[i], Rq());
          filt_buf_[i] = fo * p_.gain; }  // mul engine-side
        break;
      case 1:
        for (int i = 0; i < kBlock; ++i) { float fo;
          rhpf_.ProcessBlock(&pwm_buf_[i], &fo, 1, filt_freq_buf_[i], Rq());
          filt_buf_[i] = fo * p_.gain; }
        break;
      case 2:
        for (int i = 0; i < kBlock; ++i) {
          svf_.ProcessBlock(&pwm_buf_[i], &filt_buf_[i], 1, filt_freq_buf_[i],
                            p_.q, 1.f, 0.f, 0.f, 0.f, 0.f);
          filt_buf_[i] *= p_.gain; }  // gain = SVF mul (class sig verified)
        break;
      case 3:
        // DFM1: block-rate params, one n=64 call (coefficient ramp law).
        dfm1_.SetParams(filt_freq_buf_[0], p_.q, p_.gain);
        dfm1_.ProcessBlock(pwm_buf_, filt_buf_, kBlock);
        break;
    }

    // Phase 3 — outSignal select (7 taps) + tanh.
    for (int i = 0; i < kBlock; ++i) {
      const float* taps[7] = { &tri_buf_[i], &osc1m_buf_[i], &tri2_buf_[i],
                               &o2_buf_[i], &pwm_buf_[i], &sh0_buf_[i],
                               &filt_buf_[i] };
      const float shaped = sc::NovaTanh(*taps[sc::SelectIndex(p_.out_signal, 7)] * p_.amp);
      shaped_l_[i] = shaped; shaped_r_[i] = shaped;
    }

    // Phase 4 — feedback write + stereo pan chain.
    if (mode_ == FeedbackMode::kReferenceSc) {
      float* obr[2] = { hz_buf_, raw_buf_ };
      lout_.ProcessBlock(bus_, obr, block_, kBlock);
    } else {
      buf_ = rungler_raw_;  // mode B FM/DAC carry to the next block
    }
    const float dly_l = ClipTo(lag_l_.Next(ClipTo(p_.pan, 0.0f, 1.0f), 0.1f),
                               0.0f, 0.01f);  // upstream m_maxdelay clamp
    const float dly_r = ClipTo(lag_r_.Next(ClipTo(p_.pan * -1.0f, 0.0f, 1.0f), 0.1f),
                               0.0f, 0.01f);
    dl_l_.ProcessBlock(shaped_l_, out_l, kBlock, dly_l, 0.f);
    dl_r_.ProcessBlock(shaped_r_, out_r, kBlock, dly_r, 0.f);
    for (int i = 0; i < kBlock; ++i) {
      out_l[i] = leak_l_.Next(out_l[i], 0.995f);
      out_r[i] = leak_r_.Next(out_r[i], 0.995f);
    }
    ++block_;
  }

  // private state: float sr_; FeedbackMode mode_; Params p_, pending_;
  // bool pending_set_ = false; int block_ = 0, stage_ = 0;
  // sc::LocalBus bus_; sc::LocalIn lin_; sc::LocalOut lout_;
  // sc::LFTri tri1_, tri2_; sc::PulseDPW osc1_, osc2_; sc::Latch latch_[8];
  // sc::DelayLine tap_[8]; float tap_delay_[8];
  // sc::Filter2Pole rlpf_, rhpf_; sc::Svf svf_; sc::Dfm1 dfm1_;
  // sc::Lag lag_l_, lag_r_; sc::DelayLine dl_l_, dl_r_;
  // sc::LeakDC leak_l_, leak_r_;
  // float rungler_hz_, rungler_raw_, buf_, o2_prev_ = 0.f; uint32_t reg_ = 0;
  // float in0_[64], in1_[64], tri_buf_[64], tri2_buf_[64], o1_buf_[64],
  //   o2_buf_[64], pwm_buf_[64], osc1m_buf_[64], sh0_buf_[64],
  //   sh_buf_[8][64], raw_buf_[64], hz_buf_[64], filt_freq_buf_[64],
  //   filt_buf_[64], shaped_l_[64], shaped_r_[64], stage_l_[64], stage_r_[64];
  // plus: uint32_t RunglerDacBits(uint32_t reg) — bit k weight 2^{-(8-k)}
  // (bit 0 = sh1 = 1/256, bit 7 = sh8 = 1/2), and tap_delay_[k] = (k+1)/sr
  // filled in Init. reg() accessor for the Task 4 mode-B test.
};
}  // namespace benjolin
#endif
```

- [ ] **Step 2: Write failing structural tests (`test_benjolin.cc`)**

```cpp
// Structural gates before any golden exists (Task 4 adds golden compares).
#include <cmath>
#include <cstdio>
#include "benjolin_engine.h"
int g_failures = 0;
int main() {
  benjolin::Params p; p.amp = 0.5f;
  benjolin::Engine e; e.Init(48000.0, benjolin::FeedbackMode::kReferenceSc,
                             1795055191, p);
  // 1 s render: finite everywhere.
  float l[48000], r[48000];
  for (int i = 0; i < 3000; ++i) e.Process16(l + i * 16, r + i * 16);
  for (int i = 0; i < 48000; ++i)
    if (!std::isfinite(l[i]) || !std::isfinite(r[i])) { ++g_failures; break; }
  // Determinism: two engines, same seed/params -> bit-identical.
  benjolin::Engine a, b;
  a.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191, p);
  b.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191, p);
  bool same = true;
  for (int i = 0; i < 375; ++i) {
    float al[16], ar[16], bl[16], br[16];
    a.Process16(al, ar); b.Process16(bl, br);
    for (int k = 0; k < 16; ++k)
      if (al[k] != bl[k] || ar[k] != br[k]) { same = false; break; }
    if (!same) break;
  }
  if (!same) ++g_failures;
  // Process16 burst == Process64 stream, bit-exact.
  benjolin::Engine c, d;
  c.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191, p);
  d.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191, p);
  bool burst_ok = true;
  for (int i = 0; i < 64; ++i) {
    float l16[16], r16[16], l64[64], r64[64];
    c.Process16(l16, r16);
    if (i % 4 == 0) d.Process64(l64, r64);
    for (int k = 0; k < 16; ++k)
      if (l16[k] != l64[(i % 4) * 16 + k]) { burst_ok = false; break; }
    if (!burst_ok) break;
  }
  if (!burst_ok) ++g_failures;
  // Rungler DAC law: bits 0..7 -> weights 1/256 .. 1/2, max 255/256.
  {
    float all_on = benjolin::RunglerDacBits(0xffu);
    if (std::abs(all_on - 255.0f / 256.0f) > 1e-6f) ++g_failures;
    if (std::abs(benjolin::RunglerDacBits(0x01u) - 1.0f / 256.0f) > 1e-6f) ++g_failures;
  }
  printf(g_failures == 0 ? "benjolin structural: PASS\n" : "benjolin structural: FAIL %d\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
```

Add the Makefile rule by mirroring `test_dronecaster`'s (same `-I` set, add `-I../src/benjolin`); run: `cd rack-rp2350/test && make test_benjolin && ./build/test_benjolin`. Expected: PASS (fix compile errors first — this step is where the engine compiles).

- [ ] **Step 3: Commit**

```bash
git add rack-rp2350/src/benjolin/benjolin_engine.h rack-rp2350/test/test_benjolin.cc rack-rp2350/test/Makefile && git commit -m "[Add] benjolin engine: reference-exact SynthDef graph, dual feedback modes"
```

---

### Task 4: Full-SynthDef golden + engine golden/mode-B tests

**Files:**
- Create: `rack-rp2350/test/benjolin_ref/gen_benjolin_ref.scd`
- Create: `rack-rp2350/test/benjolin_ref/run_benjolin_ref.sh`
- Modify: `rack-rp2350/test/test_benjolin.cc` (golden + mode-B scenario checks)

**Interfaces:**
- Consumes: Task 3 `Engine::Process64/Process16`, `FeedbackMode`; the DFM1 epoch sidecar.
- Produces: `benjolin_ref/goldens/{default,loop1}.expected.wav` + `.seed.txt`; gates for Task 6 acceptance.

- [ ] **Step 1: Write the renderer**

`gen_benjolin_ref.scd`: copy the SynthDef **verbatim** from `Engine_Benjolis.sc:25-89` (strip the norns `CroneEngine` wrapper; keep every line of the graph including the `//pwm = tri1 > tri2` comment-adjacent code exactly as shipped). Replace `context` plumbing with `Synth(\benjolis, [\out, 0, <scenario args>])` in a `Score`/`recordNRT` harness **copied from `PicoCollider/test/ref/gen_ugen_ref.scd`** (same RandSeed pin, `writeOSCFile`, `scsynth -N ... 48000 WAV float`, DFM1 epoch pinning block `:253-274` copied verbatim, seed written to `goldens/benjolin.seed.txt`).
Scenarios: `default` = all SynthDef defaults + `amp: 0.5`; `loop1` = same + `loop: 0.9, rungler1: 0.3, freq2: 8` (loop closure exercised). Duration 1.0 s.

`run_benjolin_ref.sh`: copy `PicoCollider/test/ref/run_ugen_ref.sh` verbatim, retarget the glob to `goldens/*.expected.wav` and the seed file name. Run it; expect 2 WAVs trimmed to 48,000 frames.

- [ ] **Step 2: Golden compare tests (mode A)**

In `test_benjolin.cc` add: read each scenario's params, `Engine::Init(48000, kReferenceSc, <epoch from benjolin.seed.txt>, p)`, render 30,000 Process16 blocks → 48,000 frames, compare with the WAV (reuse the WAV reader + gate helper used by `test_dronecaster.cc`): `max_abs ≤ 1e-6`, `corr ≥ 0.999999`. If the golden shows a **one-sample lead**, remove the Task 3 filter-prime block and re-render nothing (the golden is the arbiter, per the Task 3 NOTE). If `corr` is high but `max_abs` large on `loop1` only, suspect the LocalBus channel order (Review Focus 3) — verify ch0 carries the FM value before touching anything else.

- [ ] **Step 3: Mode-B scenario checks (law-only, §5.3 option B)**

With `FeedbackMode::kHardware` at defaults (`freq2 = 4`, `loop = 0`): instrument the engine (test-only accessor or a debug callback — add `uint32_t reg() const { return reg_; }`) and verify over 1 s: (a) `reg_` changes **only** on `osc2` rising edges (count changes ≈ 4/s ±1); (b) reconstructed DAC from `reg_` matches `RunglerDacBits` bit law; (c) RMS within ±1.5 dB of the mode-A RMS at the same params; (d) no NaN blocks.

- [ ] **Step 4: Run**

Run: `cd rack-rp2350/test && make test_benjolin && ./build/test_benjolin`
Expected: golden PASS ×2, mode-B PASS. Chaos caveat: if `loop1` corr degrades with everything else exact, dump the first divergent sample index and bisect by muting stages (`out_signal` selects) before touching DSP.

- [ ] **Step 5: Commit**

```bash
git add rack-rp2350/test/benjolin_ref rack-rp2350/test/test_benjolin.cc && git commit -m "[Add] benjolin full-SynthDef goldens + mode-B scenario gates"
```

---

### Task 5: Firmware adapter + tables + panel

**Files:**
- Create: `rack-rp2350/src/module_benjolin.h`
- Modify: `rack-rp2350/src/rack_window.h` (enum id 6 + `kSections` entry), `rack-rp2350/src/main.cpp` (include, `kRackModules` entry, `kChainABufferBytes[6] = 0`), `rack-rp2350/memmap_rack.ld` (`.tc_benjolin` overlay + `__rack_flash_benjolin_*` LMA region — mirror the dronecaster blocks verbatim), `rack-rp2350/platformio.ini` (`-DRACK_BOOT_MODULE=6` + comment), `tools/rack_panel.html` (7th tab + CC rows)

**Interfaces:**
- Consumes: Task 3 `benjolin::Engine` (ext_tap_rings from the rack arena), Task 1 `SetParams(freq,res,input_gain)`.
- Produces: module id 6 reachable via CC 32 / serial console; CC map below.

- [ ] **Step 1: Write the adapter**

Mirror `module_dronecaster.h` structure exactly: `namespace rack_modules::mod_benjolin`; `kSampleRate = 48000`, `kBlockSize = 16`, `kBudgetUs = 1333` (the 64-sample period), `kI2sBuffers = 8`, `kI2sWords = 16`, `kStallUs = 900`; arena-placed `State` (engine + 8×16-float tap rings + pending params + atomics); `init()` places state in the arena, `Engine::Init(48000.0, kReferenceSc, 1795055191, p, arena_tap_rings)`; `render()` = pending-intent landing at block boundary → `process_block_benjolin_o3(s->eng, l, r, kBlockSize)` (the `-O3` wrapper, `__attribute__((noinline, section(".tc_benjolin_process")))`, unique token — mirror the dronecaster wrapper declaration) → NaN/inf sweep → `Clip16` → I2S. Engine params are applied via `SetParams` when pending floats changed (single-word atomic stores, core-0 writer / core-1 reader — the lira8/dronecaster discipline).

CC map (panel + registry + console must agree; 7-bit CCs, SC-unit scaling noted):

| CC | Param | Scaling CC→SC units |
|---|---|---|
| 60 | freq1 | 0–127 → 1–500 Hz (log) |
| 61 | freq2 | 0–127 → 0.1–50 Hz (log) |
| 62 | scale | 0–127 → 0–1 |
| 63 | rungler1 | 0–127 → 0–0.5 |
| 64 | rungler2 | 0–127 → 0–0.5 |
| 65 | runglerFilt | 0–127 → 0–24 |
| 66 | loop | 0–127 → 0–1 |
| 67 | filtFreq | 0–127 → 10–8000 Hz (log) |
| 68 | q | 0–127 → 0–0.99 |
| 69 | gain | 0–127 → 0–4 |
| 70 | filterType | 0–3 (RLPF/RHPF/SVF/DFM1) |
| 71 | outSignal | 0–6 (7 taps) |
| 72 | amp | 0–127 → 0–1 |
| 73 | pan | 0–127 → 0–0.127 (norns ×0.001 law) |

- [ ] **Step 2: Tables + linker + boot**

`rack_window.h`: add `RACK_MODULE_BENJOLIN = 6`, `RACK_MODULE_COUNT = 7`, `kSections` entry `{"benjolin", __tc_benjolin_start, __tc_benjolin_end, __rack_flash_benjolin_start, __rack_flash_benjolin_end}`. `main.cpp`: `#include "module_benjolin.h"` after the dronecaster include, registry entry (exact `RackModuleInfo` field order as shown at `main.cpp:250-320`), `kChainABufferBytes` gains a 7th `0`. `memmap_rack.ld`: duplicate the dronecaster `tc_*` OVERLAY + flash region blocks renamed `benjolin`. `platformio.ini`: boot module comment line gains `6 benjolin`; set `-DRACK_BOOT_MODULE=6`.

- [ ] **Step 3: Panel tab**

`tools/rack_panel.html`: copy the dronecaster tab block, rename benjolin, CC 32 value 6, add the 14 CC rows from the table above.

- [ ] **Step 4: Build + SRAM gate**

Run: `pio run -d rack-rp2350` then
`arm-none-eabi-nm .pio/build/rack/firmware.elf | grep -i benjolin` — every `tc_benjolin` symbol must sit at `0x2000xxxx` (SRAM), and `__tc_benjolin_start/end` must report the slot size (expect ≪ the 118,784 B window).
Expected: SUCCESS; RAM% within the existing headroom (engine state ≈ 2 KB + rings).

- [ ] **Step 5: Commit**

```bash
git add -A rack-rp2350 tools && git commit -m "[Add] benjolin module adapter: id 6, CC map, tc_benjolin window slot"
```

---

### Task 6: Device acceptance + ledger

**Files:**
- Modify: `.superpowers/sdd/progress.md` (T-B1 entry: status, commit hash, findings)

- [ ] **Step 1: Flash** — AGENTS.md procedure verbatim (1200-baud touch, kill DTR readers first, `by-id` paths only, firmware.uf2 only). First console command after boot: newline then `status` (stale-byte rule).
- [ ] **Step 2: Bench** — 2 s report: `max Process` ≤ 1333 µs budget, 0 i2s stalls; record the numbers in the ledger.
- [ ] **Step 3: MIDI sweep** — raw-MIDI sweep of CC 60–73 (browser panel or serial `cc`), CC 32 = 6 module swap in and out, verify no watchdog reset across all 14 CC boundaries.
- [ ] **Step 4: User acceptance** — the user's ears decide (all 7 `outSignal` taps audible, filter morph audibly matches SC/Norns character). 30 min soak with the 2 s report watched for stalls.
- [ ] **Step 5: Ledger + boot revert** — record commit hash + findings (including any Task 4 golden arbitration outcome); set `-DRACK_BOOT_MODULE` back to the user's preferred boot module if they want Elements boot; final `[Docs]` commit.

## Non-goals

- **C3 (ESP32-C3) UI page** for benjolin — the rack panel tab + CC 32 switching give full control; a C3 page is a separate `rack-uimcu` task after hardware acceptance.
- **Mode B via MIDI** — `kHardware` is compiled and host-tested but the device ships with `kReferenceSc` only; exposing B is a deliberate post-acceptance decision.
- Presets beyond the lira8-pattern autosave struct (fold into Task 5 only if it drops in cleanly; otherwise a follow-up commit).
