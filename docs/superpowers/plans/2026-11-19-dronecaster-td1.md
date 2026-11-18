# Dronecaster T-D1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land the first flashable Dronecaster rack module (id 5): seed-matched scsynth golden infrastructure, the kit core (RGen + SinOsc/LFTri/Pulse + EnvAsr), the SynthSocket port, and the end-to-end adapter with the Sine/Square/Triangle trio bit-exact.

**Architecture:** Hand-ported pure-C++ engine in `src/dronecaster/` (mirrors `src/lira8/`), verified against non-realtime scsynth renders with fixed seeds, wrapped by a `module_dronecaster.h` adapter following the lira8 adapter shape. Standalone 48 kHz, host grid 16 samples, engine bursts 4×16 → 64.

**Tech Stack:** Plain C++17 header-only engine (g++, no MCU headers), PlatformIO/earlephilhower RP2350 firmware, WSL SuperCollider (sclang + scsynth NRT) for goldens, Web-MIDI browser panel.

**Scope note:** This plan covers **T-D1 only** (spec §Phases). The spec's "kit basics" narrows here to RGen + osc + env + arena — exactly what the trio and the socket need (YAGNI); noise/delay/logic/spatial UGens land in T-D2 pulled per-drone. T-D2/T-D3/T-D4 get their own plans after this phase is accepted on hardware.

## Global Constraints (from the spec, verbatim values)

- Engine is **pure C++, no MCU headers** — host compilation is the portability proof.
- Rates: **48000 Hz / host grid 16 / internal block 64**; budget **1333 µs** (the 64-sample period); I2S bank **8 buffers × 16 words**; stall threshold 900 µs.
- **Denormals:** FPSCR FZ+DN already set on BOTH cores at boot — **zero** software denormal handling in the kit.
- **Sample-rate law:** every filter/time constant derived from `sr` at runtime — never baked 44.1 kHz constants.
- **No dot products** in the kit → no SMLAD; 4-accumulator float unroll only if a MAC-bound UGen appears (none in T-D1).
- Hot code: `DRONECASTER_TC(token)` sections `.tc_dronecaster.<token>`, **one unique token per function/data** (inline/COMDAT collision law).
- License: **GPL-3.0 attribution header** on every ported file (upstream: northern-information/dronecaster + supercollider/supercollider).
- `RACK_BOOT_MODULE` untouched (stays 0 = Elements).
- CC map: **51** drone select (catalog 0–36) · **52** amp (default 0.4) · **53** fade (attack=release) · **54** hz offset ±24 st · notes → hz · PLAY pseudo-CC 64 = recast. Mod destinations: continuous 52/53/54, no triggers.
- Report/prints: **no `%f`** (newlib-nano), dbg line buffer 560 B.
- Host tests: `cd rack-rp2350/test && make test` in **WSL** (no g++ on Windows).
- Commit style: `[Add]/[Fix]/[Update]/[Remove] <scope>`; work on branch `rack-rp2350`.
- Hardware acceptance requires the user's ears; never claim HW-verified without their confirmation.

---

### Task 1: Vendor the 37 drone sources

**Files:**
- Create: `rack-rp2350/test/dronecaster_ref/drones/*.scd` (37 files)
- Create: `rack-rp2350/test/dronecaster_ref/README.md`

**Interfaces:**
- Consumes: upstream repo `northern-information/dronecaster` (GPL-3.0).
- Produces: `test/dronecaster_ref/drones/` — the corpus the golden renderer and the catalog-order check read; Task 2 loads `drones/<Name>.scd`.

- [ ] **Step 1: Fetch the upstream archive into a temp dir**

Run (Git Bash):

```bash
mkdir -p /tmp/dc_audit && cd /tmp/dc_audit
curl -sL https://github.com/northern-information/dronecaster/archive/refs/heads/main.tar.gz -o dc.tgz
tar xzf dc.tgz
ls dronecaster-main/engine/drones/*.scd | wc -l   # expect: 37
```

- [ ] **Step 2: Copy the corpus + write the attribution README**

```bash
mkdir -p /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref/drones
cp /tmp/dc_audit/dronecaster-main/engine/drones/*.scd \
   /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref/drones/
```

Create `rack-rp2350/test/dronecaster_ref/README.md`:

```markdown
# dronecaster_ref — golden renders + drone corpus

The 37 drone SynthDefs in `drones/` are verbatim copies from
https://github.com/northern-information/dronecaster (GPL-3.0, engine/drones/),
vendored so the golden pipeline and the catalog-order check never depend on an
external checkout. Upstream attribution per file is kept in the original
comments. The C++ port of each drone lives in `src/dronecaster/drones/` and is
a re-implementation, not a copy of this directory.

`gen_goldens.scd` renders a drone through the SynthSocket wrap (hz/amp/gate +
Env.asr) with fixed seeds to `goldens/<Name>.expected.wav` (48 kHz, int16,
stereo). The catalog order is LC_ALL=C sort of the filenames (37 entries;
Sine=22, Square=23, Triangle=29 — asserted by test_dronecaster).
```

- [ ] **Step 3: Verify the corpus + catalog order**

```bash
cd /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref
ls drones/*.scd | wc -l                                   # expect: 37
ls drones/ | LC_ALL=C sort | head -40
```

Expected order (assert these three indices): `... Shields(21) Sine(22) Square(23) ... Thx(27) Toshiya(28) Triangle(29) ... harms way(36)`. If the sort disagrees with this order, the CATALOG in Task 6 uses the actual sort output — whatever it is, it is deterministic and both the C++ catalog and the panel use it.

- [ ] **Step 4: Commit**

```bash
git add rack-rp2350/test/dronecaster_ref/
git commit -m "[Add] test: vendor 37 dronecaster drone SynthDefs (GPL-3.0) for golden renders"
```

---

### Task 2: scsynth golden renderer (WSL)

**Files:**
- Create: `rack-rp2350/test/dronecaster_ref/gen_goldens.scd`
- Create: `rack-rp2350/test/dronecaster_ref/run_goldens.sh`
- Create: `rack-rp2350/test/dronecaster_ref/goldens/{Sine,Square,Triangle}.expected.wav` (rendered artifacts, committed)

**Interfaces:**
- Consumes: Task 1's `drones/*.scd`.
- Produces: `goldens/<Name>.expected.wav` — 192,000 frames, 48 kHz, stereo, int16 (4.0 s: voice starts at t = 0.001 s, gate off at t = 3.0 s, attack = release = 1.0 s, hz = 55.0, amp = 0.4). Task 7 compares against exactly this schedule.

- [ ] **Step 1: Install SuperCollider in WSL (skip if present)**

```bash
wsl bash -lc 'command -v sclang && command -v scsynth || { sudo apt update && sudo apt install -y supercollider; }'
wsl bash -lc 'sclang -v && scsynth -v'
```

Expected: both print version strings (3.13.x typical on Ubuntu 24.04).

- [ ] **Step 2: Write `gen_goldens.scd`**

```supercollider
// gen_goldens.scd — render drone SynthDefs through the SynthSocket wrap
// (hz/amp/K2A + Env.asr gate, doneAction 2) to 16-bit stereo WAV via
// non-realtime scsynth. Matches the C++ engine test schedule exactly:
// voice at t=0.001 s, gate off at t=3.0 s, attack=release=1.0 s, 4.0 s total.
(
var seed = 16rC0FFEE;
var drones = ["Sine", "Square", "Triangle"];
var dur = 4.0, start = 0.001, gateOff = 3.0;
var attack = 1.0, release = 1.0, hz = 55.0, amp = 0.4;
var dir = PathName(thisProcess.nowExecutingPath).pathOnly;
var outDir = dir ++ "goldens";

thisProcess.randSeed_(seed);          // sclang-side RNG (graph construction)
if (File.exists(outDir).not) { File.mkdir(outDir) };
drones.do { |name|
    var fn   = (dir ++ "drones/" ++ name ++ ".scd").load;   // evaluates to a Function
    var def  = SynthDef("dc_" ++ name, { |hz = 55.0, amp = 1.0, out = 0, gate = 1, attack = 1.0, release = 1.0|
        var snd = fn.value(K2A.ar(hz), K2A.ar(amp), 0);    // bufnum 0: no tape buffer in T-D1
        Out.ar(out, snd * Env.asr(attack, 1, release).ar(gate: gate));
    });
    var score = Score.new;
    var opts  = ServerOptions.new.numOutputBusChannels_(2).numInputBusChannels_(0).blockSize_(64);
    score.add([0,        [\d_recv, def.asBytes]]);
    score.add([start,    [\s_new, "dc_" ++ name, 1000, 0, 0, \hz, hz, \amp, amp]]);
    score.add([gateOff,  [\n_set, 1000, \gate, 0]]);
    score.add([dur,      [0]]);
    score.recordNRT("/tmp/dc_nrt.osc", outDir ++ "/" ++ name ++ ".expected.wav",
        nil, 48000, "WAV", "int16", opts, dur);
    File.delete("/tmp/dc_nrt.osc");
    ("rendered " ++ name).postln;
};
0.exit;
)
```

- [ ] **Step 3: Write `run_goldens.sh` (render + determinism gate)**

```bash
#!/usr/bin/env bash
# Renders the trio goldens and proves NRT determinism: two runs must produce
# byte-identical WAVs (the trio uses no noise UGens; sclang+server seeds are
# fixed regardless). Run from this directory, inside WSL.
set -euo pipefail
cd "$(dirname "$0")"
sclang gen_goldens.scd
mkdir -p /tmp/dc_det1 /tmp/dc_det2
cp goldens/*.expected.wav /tmp/dc_det1/
sclang gen_goldens.scd
cp goldens/*.expected.wav /tmp/dc_det2/
for f in Sine Square Triangle; do
  cmp "/tmp/dc_det1/$f.expected.wav" "/tmp/dc_det2/$f.expected.wav" \
    && echo "determinism: $f OK" || { echo "determinism: $f DIFFERS"; exit 1; }
done
rm -rf /tmp/dc_det1 /tmp/dc_det2
```

- [ ] **Step 4: Render + verify**

```bash
cd /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref && bash run_goldens.sh'
ls -la goldens/   # expect: 3 x ~768 KB WAVs
```

Expected: `determinism: Sine OK` ×3. If a determinism check fails, print `scsynth -h` and look for a seed flag (`-r`); re-run with it set in `run_goldens.sh`. The trio draws no randomness, so failure here means the render pipeline itself is nondeterministic — fix that before Task 7.

- [ ] **Step 5: Sanity-listen check (metadata only, no ears needed yet)**

```bash
python3 - <<'PY'
import wave
for n in ("Sine", "Square", "Triangle"):
    w = wave.open(f"goldens/{n}.expected.wav", "rb")
    print(n, w.getframerate(), w.getnchannels(), w.getnframes(), w.getsampwidth()*8, "bit")
PY
```

Expected: `48000 2 192000 16` for each.

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/test/dronecaster_ref/gen_goldens.scd rack-rp2350/test/dronecaster_ref/run_goldens.sh rack-rp2350/test/dronecaster_ref/goldens/
git commit -m "[Add] test: scsynth NRT golden renderer for dronecaster trio (seed-fixed, determinism-gated)"
```

---

### Task 3: Host comparator + Makefile wiring

**Files:**
- Create: `rack-rp2350/test/test_dronecaster.cc`
- Modify: `rack-rp2350/test/Makefile` (TEST_BINS + target)

**Interfaces:**
- Consumes: `wav_io.h` (`l8test::ReadWav`, `l8test::WriteWav16`).
- Produces: `golden_gates(ref_path, got_path, &report)` → bool — Task 7 uses it for the trio; the metrics struct is reused by every later golden test.

- [ ] **Step 1: Write the comparator with self-tests**

Create `rack-rp2350/test/test_dronecaster.cc`:

```cpp
// Dronecaster host tests (T-D1): comparator infrastructure + kit/engine
// unit tests + trio goldens (goldens land in Task 7; compare() is proven
// here against known synthetic cases so a later FAIL is trustworthy).
#include "../src/dronecaster/sc_defs.h"
#include "../src/dronecaster/sc_rgen.h"
#include "../src/dronecaster/sc_kit_osc.h"
#include "../src/dronecaster/sc_kit_env.h"
#include "../src/dronecaster/sc_arena.h"
#include "../src/dronecaster/dronecaster_engine.h"
#include "../src/dronecaster/drones/drone_trio.h"
#include "wav_io.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

static int g_failures = 0;
#define CHECK(cond, msg) do { if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); ++g_failures; } } while (0)

struct GoldenReport {
  long   frames_ref = 0, frames_got = 0;
  double max_abs = 0.0;      // max |delta| in int16 LSBs
  double rms_db = -999.0;    // 20*log10(rms(diff)/rms(ref))
  double corr = 0.0;
};

// Compare two WAVs in the int16 domain. Gates set by the caller.
static bool compare(const char* ref_path, const char* got_path,
                    double max_abs_gate, double corr_gate, GoldenReport* rep) {
  l8test::WavFile a = l8test::ReadWav(ref_path);
  l8test::WavFile b = l8test::ReadWav(got_path);
  rep->frames_ref = a.samples.size() / (a.channels ? a.channels : 1);
  rep->frames_got = b.samples.size() / (b.channels ? b.channels : 1);
  if (a.sample_rate != b.sample_rate || a.channels != b.channels ||
      a.samples.size() != b.samples.size()) {
    printf("  FAIL: layout ref=%ldf/%dch/%dHz got=%ldf/%dch/%dHz\n",
           rep->frames_ref, a.channels, a.sample_rate,
           rep->frames_got, b.channels, b.sample_rate);
    return false;
  }
  const long n = static_cast<long>(a.samples.size());
  double num = 0, den_a = 0, den_b = 0, diff_sq = 0, max_abs = 0;
  for (long i = 0; i < n; ++i) {
    const double d = a.samples[i] - b.samples[i];
    if (fabs(d) > max_abs) max_abs = fabs(d);
    diff_sq += d * d;
    num   += a.samples[i] * b.samples[i];
    den_a += a.samples[i] * a.samples[i];
    den_b += b.samples[i] * b.samples[i];
  }
  rep->max_abs = max_abs * 32768.0;                       // to LSBs
  rep->rms_db  = (diff_sq > 0 && den_a > 0)
      ? 20.0 * log10(sqrt(diff_sq / n) / sqrt(den_a / n)) : -999.0;
  rep->corr = (den_a > 0 && den_b > 0)
      ? num / sqrt(den_a * den_b) : 0.0;
  printf("  metrics: max=%.2f LSB rms=%.1f dB corr=%.6f\n",
         rep->max_abs, rep->rms_db, rep->corr);
  return rep->max_abs <= max_abs_gate && rep->corr >= corr_gate;
}

static void write_constant_wav(const char* path, float v, int frames) {
  std::vector<float> x(2 * frames, v);
  l8test::WriteWav16(path, x.data(), frames, 2, 48000);
}

int main() {
  printf("== comparator self-tests ==\n");
  {
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", 0.5f, 480);
    GoldenReport r;
    CHECK(compare("build/dc_a.wav", "build/dc_b.wav", 0.0, 1.0, &r),
          "identical files: max=0 corr=1");
    CHECK(r.max_abs == 0.0 && r.corr >= 0.999999, "identical metrics exact");
  }
  {
    // +1 LSB offset: ReadWav maps int16 s -> s/32768; write +1/32768.
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", 0.5f + 1.0f / 32768.0f, 480);
    GoldenReport r;
    CHECK(compare("build/dc_a.wav", "build/dc_b.wav", 2.0, 0.0, &r),
          "+1 LSB offset passes max<=2 gate");
    CHECK(fabs(r.max_abs - 1.0) < 1e-6, "+1 LSB measured as 1.00 LSB");
  }
  {
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", -0.5f, 480);
    GoldenReport r;
    CHECK(!compare("build/dc_a.wav", "build/dc_b.wav", 2.0, 0.9999, &r),
          "polarity flip rejected by corr gate");
  }
  printf("comparator self-tests: %s (%d failures)\n",
         g_failures ? "FAIL" : "PASS", g_failures);
  return g_failures ? 1 : 0;
}
```

(Task 5/6/7 append their test sections above the final `return`.)

- [ ] **Step 2: Wire the Makefile**

In `rack-rp2350/test/Makefile`, add `test_dronecaster` to `TEST_BINS` and add the target:

```make
test_dronecaster: test_dronecaster.cc ../src/dronecaster/sc_defs.h ../src/dronecaster/sc_rgen.h ../src/dronecaster/sc_kit_osc.h ../src/dronecaster/sc_kit_env.h ../src/dronecaster/sc_arena.h ../src/dronecaster/dronecaster_engine.h ../src/dronecaster/drones/drone_trio.h | $(BUILD)
	$(CXX) $(CXXFLAGS) -I../src/dronecaster -I. $< -o $(BUILD)$@
```

Until Tasks 4-6 create those headers the target won't compile — create the headers as **empty guards** now so this task is green in isolation:

For each of `sc_defs.h`, `sc_rgen.h`, `sc_kit_osc.h`, `sc_kit_env.h`, `sc_arena.h`, `dronecaster_engine.h` in `rack-rp2350/src/dronecaster/` and `drones/drone_trio.h` in `rack-rp2350/src/dronecaster/drones/`:

```cpp
// <name> — T-D1 scaffold; filled by the dronecaster plan tasks.
#ifndef RACK_SRC_DRONECASTER_<NAME>_H_
#define RACK_SRC_DRONECASTER_<NAME>_H_
#endif
```

- [ ] **Step 3: Run (WSL) and verify pass**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: `comparator self-tests: PASS (0 failures)`.

- [ ] **Step 4: Commit**

```bash
git add rack-rp2350/test/test_dronecaster.cc rack-rp2350/test/Makefile rack-rp2350/src/dronecaster/
git commit -m "[Add] test: dronecaster comparator + kit header scaffolds"
```

---

### Task 4: sc_defs.h + sc_rgen.h (RGen port)

**Files:**
- Modify: `rack-rp2350/src/dronecaster/sc_defs.h`
- Modify: `rack-rp2350/src/dronecaster/sc_rgen.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc` (append RGen tests)

**Interfaces:**
- Produces: `sc::kBlock` (=64), `sc::clamp`, `sc::midicps`, `sc::cpsmidi`, `sc::RGen` with `init(uint32 seed)`, `frand0/1/2()` — every kit UGen and the engine consume these exact names.

- [ ] **Step 1: Fetch the SC source references (verify anchors)**

```bash
wsl bash -lc 'mkdir -p /tmp/scsrc && cd /tmp/scsrc && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/server/scsynth/SC_RGen.h -o RGen.h && \
  grep -n "1664525\|1013904223\|frand0\|frand1\|frand2\|0x3F800000\|>> 9" RGen.h | head -20'
```

Expected: the LCG constants `1664525` / `1013904223` and the `(s >> 9) | 0x3F800000` float trick are present. Confirm which `sN` each of `frand0/1/2` uses (they use `mState.s1/s2/s3` respectively — mirror EXACTLY what the file shows, including any third generator). If the installed sclang version differs from 3.13.1, use its tag: `sclang -v` prints it; swap the URL tag.

- [ ] **Step 2: Implement `sc_rgen.h`**

```cpp
// sc_rgen.h — exact port of scsynth's RGen LCG (SC_RGen.h, GPL-3.0,
// supercollider/supercollider). Seed-matched so the C++ engine reproduces
// the server's noise/demand sequences sample-for-sample (the golden two-seed
// model: this port = server-side; the sclang seed = graph construction).
// VERIFY step already pinned: lcg = s*1664525 + 1013904223 (Numerical
// Recipes), float = bits((s>>9)|0x3F800000) - 1.0f.
#ifndef RACK_SRC_DRONECASTER_SC_RGEN_H_
#define RACK_SRC_DRONECASTER_SC_RGEN_H_
#include <cstdint>
namespace sc {

class RGen {
 public:
  void init(uint32_t seed) { s1_ = seed; s2_ = seed; s3_ = seed; }
  static uint32_t lcg(uint32_t s) { return s * 1664525u + 1013904223u; }
  static float frand(uint32_t& s) {
    s = lcg(s);
    union { uint32_t u; float f; } v;
    v.u = (s >> 9) | 0x3F800000u;
    return v.f - 1.0f;
  }
  float frand0() { return frand(s1_); }
  float frand1() { return frand(s2_); }
  float frand2() { return frand(s3_); }

 private:
  uint32_t s1_ = 0, s2_ = 0, s3_ = 0;
};

}  // namespace sc
#endif
```

If Step 1 showed a different index mapping (e.g. `frand0` on `s2`), correct it here — the port must match the source verbatim.

- [ ] **Step 3: Implement `sc_defs.h`**

```cpp
// sc_defs.h — shared constants + math helpers for the dronecaster engine
// (rate-parameterized: NO baked 44.1 kHz constants — the sample-rate law).
#ifndef RACK_SRC_DRONECASTER_SC_DEFS_H_
#define RACK_SRC_DRONECASTER_SC_DEFS_H_
#include <cmath>
namespace sc {

constexpr uint32_t kBlock = 64;          // scsynth block size, 48 kHz server

inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float midicps(float n) { return 440.0f * powf(2.0f, (n - 69.0f) / 12.0f); }
inline float cpsmidi(float f) { return 12.0f * log2f(f / 440.0f) + 69.0f; }

}  // namespace sc
#endif
```

- [ ] **Step 4: Append RGen tests to `test_dronecaster.cc`** (before the final `return`)

```cpp
  printf("== sc::RGen ==\n");
  {
    sc::RGen r;
    r.init(1);
    const float a = r.frand0(), b = r.frand0();
    CHECK(a != b, "sequence advances");
    CHECK(a >= 0.0f && a < 1.0f && b >= 0.0f && b < 1.0f, "values in [0,1)");
    sc::RGen r2; r2.init(1);
    CHECK(r2.frand0() == a, "deterministic per seed");
    // LCG law — exact values (computed mod 2^32: 1*1664525+1013904223, then again):
    uint32_t s = sc::RGen::lcg(1);
    CHECK(s == 1015568748u, "lcg(1) == 1015568748");
    s = sc::RGen::lcg(s);
    CHECK(s == 1586005467u, "lcg^2(1) == 1586005467");
    // frand bit trick — exact first values for seed 1:
    sc::RGen r4; r4.init(1);
    CHECK(fabsf(r4.frand0() - 0.23645544f) < 1e-6f, "frand0(seed1)[0] == 0.23645544");
    CHECK(fabsf(r4.frand0() - 0.36927056f) < 1e-6f, "frand0(seed1)[1] == 0.36927056");
    CHECK(fabsf(r4.frand0() - 0.50424194f) < 1e-6f, "frand0(seed1)[2] == 0.50424194");
    // independent streams (seed 7: frand0[0] vs frand1[0])
    sc::RGen r3; r3.init(7);
    const float f0 = r3.frand0(), f1 = r3.frand1();
    CHECK(fabsf(f0 - 0.23878074f) < 1e-6f, "frand0(seed7)[0] == 0.23878074");
    CHECK(fabsf(f1 - 0.91349316f) < 1e-6f, "frand1(seed7)[0] == 0.91349316");
  }
```

- [ ] **Step 5: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: comparator + RGen sections all `ok`, exit 0. If the LCG check fails, Step 1's fetch disagrees with the port — re-read RGen.h and fix.

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_defs.h rack-rp2350/src/dronecaster/sc_rgen.h rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: sc_defs + scsynth RGen LCG port (seed-matched), host-tested"
```

---

### Task 5: Kit oscillators — SinOsc, LFTri, Pulse

**Files:**
- Modify: `rack-rp2350/src/dronecaster/sc_kit_osc.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc` (append osc tests)

**Interfaces:**
- Consumes: `sc::RGen` (not used by these oscs; interface parity only), `sc_arena.h` for table storage (Task 6 wires arena; Task 5 tables live in the VoiceCtx-provided pointer — see Step 2).
- Produces: `sc::SinOsc`, `sc::LFTri`, `sc::Pulse` — each with `Init(ctx, freq)`, `SetFreq(float)`, `Process()` (one sample). Table storage: `sc::KitTables` (allocated from the engine arena; exact struct in Step 2). Later drone ports reuse the same three methods.

- [ ] **Step 1: Fetch the SC oscillator references**

```bash
wsl bash -lc 'cd /tmp/scsrc && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/server/plugins/OscUgens.cpp -o OscUgens.cpp && \
  grep -n "kSineSize\|gSineTable\|Sine_next\|LFTri_next\|Pulse_next\|gPulseTable\|gInvAmpChtable\|gInvAmpMchtable\|kPulseTableSize" OscUgens.cpp | head -30 && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/server/plugins/OscUGens.h -o OscUGens.h && \
  grep -n "InitializeWavetables\|gSineTable\|gPulseTable\|gInvAmp" OscUGens.h | head -20'
```

Read `Sine_next`, `LFTri_next`, `Pulse_next` and the wavetable initialization (`InitializeWavetables` lives in the plugin lib; if only declared in the header, fetch `server/plugins/OscUgens.cpp`'s `initialize`/"Wavetables" section — the tables are built once at plugin load). Pin: kSineSize value, sine table layout (guard point), LFTri's exact phase start + piecewise formula, Pulse's two-band-limit scheme (table sizes, amp-correction lookup, phase scaling).

- [ ] **Step 2: Implement `sc_kit_osc.h`**

Port each UGen from the fetched source. The structural skeleton (constants verified in Step 1 — adjust numbers to what the source shows):

```cpp
// sc_kit_osc.h — faithful ports of scsynth oscillators (OscUgens.cpp,
// GPL-3.0). NO software denormal handling (FPSCR FZ/DN is global on both
// cores). All time constants derive from the runtime sr (sample-rate law).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_OSC_H_
#define RACK_SRC_DRONECASTER_SC_KIT_OSC_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"
#include "sc_arena.h"

namespace sc {

// Kit tables, allocated from the engine's SRAM arena at Init (SRAM law —
// the tables are touched every sample). Sizes pinned from OscUgens.cpp in
// the Task 5 verify step; adjust here if the source disagrees.
struct KitTables {
  float sine[kSineSize + 1];       // WAVETABLE format: guard point == [0]
  // Pulse's band-limit scheme tables (sizes from the source):
  float pulse[kPulseTableSize + 1];
  float inv_amp_c[kPulseAmpTableSize];
  float inv_amp_m[kPulseAmpTableSize];
};

inline void InitKitTables(KitTables* t, sc::Arena* a) {
  // sine: t->sine[i] = sin(2*pi*i / kSineSize), guard = [0]  (source: InitializeWavetables)
  for (int i = 0; i < kSineSize; ++i)
    t->sine[i] = sinf(2.0f * static_cast<float>(M_PI) * i / kSineSize);
  t->sine[kSineSize] = t->sine[0];
  // pulse tables: port the source's two-band-limit construction verbatim here
  // (Pulse_next + its init; see OscUgens.cpp).
}

// SinOsc: sine wavetable, linear interp, phase in table samples.
struct SinOsc {
  void Init(const KitTables* t, float sr, float freq) {
    table_ = t; sr_ = sr; freq_ = freq; phase_ = 0.0f;
  }
  void SetFreq(float f) { freq_ = f; }
  float Process() {
    phase_ += freq_ * static_cast<float>(kSineSize) / sr_;
    if (phase_ >= static_cast<float>(kSineSize)) phase_ -= static_cast<float>(kSineSize);
    const int i = static_cast<int>(phase_);
    const float frac = phase_ - static_cast<float>(i);
    return table_->sine[i] + (table_->sine[i + 1] - table_->sine[i]) * frac;
  }
 private:
  const KitTables* table_ = nullptr;
  float sr_ = 48000.0f, freq_ = 0.0f, phase_ = 0.0f;
};

// LFTri: analytic triangle, phase in [0,1). Piecewise formula + phase start
// ported verbatim from LFTri_next.
struct LFTri {
  void Init(float sr, float freq) { sr_ = sr; freq_ = freq; phase_ = 0.0f; }
  void SetFreq(float f) { freq_ = f; }
  float Process() {
    phase_ += freq_ / sr_;
    if (phase_ >= 1.0f) phase_ -= 1.0f;
    return phase_ < 0.5f ? 4.0f * phase_ - 1.0f : 3.0f - 4.0f * phase_;
  }
 private:
  float sr_ = 48000.0f, freq_ = 0.0f, phase_ = 0.0f;
};

// Pulse: band-limited pulse via the two-band-limit tables (port Pulse_next
// verbatim incl. the amp-correction lookup).
struct Pulse {
  void Init(const KitTables* t, float sr, float freq, float width) {
    table_ = t; sr_ = sr; freq_ = freq; width_ = width; phase_ = 0.0f;
  }
  void SetFreq(float f) { freq_ = f; }
  void SetWidth(float w) { width_ = w; }
  float Process();   // body = verbatim port of Pulse_next's per-sample path
 private:
  const KitTables* table_ = nullptr;
  float sr_ = 48000.0f, freq_ = 0.0f, width_ = 0.5f, phase_ = 0.0f;
};

}  // namespace sc
#endif
```

`Pulse::Process` is filled from the fetched `Pulse_next` — the body is that function's arithmetic with `unit->mPhase/mFreq/mWidth` mapped to the members above and the table lookups pointed at `table_->pulse/inv_amp_c/inv_amp_m`. Keep the structure of the source (same interpolation call, same wrap arithmetic) — this is what buys bit-exactness.

- [ ] **Step 3: Also pin `sc_arena.h`** (needed by KitTables):

```cpp
// sc_arena.h — bump allocator over the engine's SRAM carve. Reset per voice
// swap (spec §Memory); no free, no fragmentation.
#ifndef RACK_SRC_DRONECASTER_SC_ARENA_H_
#define RACK_SRC_DRONECASTER_SC_ARENA_H_
#include <cstddef>
#include <cstdint>
namespace sc {

class Arena {
 public:
  void Init(uint8_t* base, size_t bytes) { base_ = base; cap_ = bytes; off_ = 0; }
  void* Alloc(size_t bytes, size_t align = 8) {
    off_ = (off_ + align - 1) & ~(align - 1);
    if (off_ + bytes > cap_) return nullptr;   // caller halts loudly
    void* p = base_ + off_;
    off_ += bytes;
    return p;
  }
  void Reset() { off_ = 0; }
  size_t used() const { return off_; }
 private:
  uint8_t* base_ = nullptr;
  size_t cap_ = 0, off_ = 0;
};

}  // namespace sc
#endif
```

- [ ] **Step 4: Append osc tests** (before the final `return`)

```cpp
  printf("== kit oscillators ==\n");
  {
    sc::Arena a; static uint8_t mem[64 * 1024]; a.Init(mem, sizeof(mem));
    sc::KitTables* t = static_cast<sc::KitTables*>(a.Alloc(sizeof(sc::KitTables)));
    sc::InitKitTables(t, &a);
    CHECK(fabsf(t->sine[0]) < 1e-6f, "sine[0] == 0");
    CHECK(fabsf(t->sine[kSineSize / 4] - 1.0f) < 1e-5f, "sine[kSineSize/4] == 1");
    CHECK(fabsf(t->sine[kSineSize]) < 1e-6f, "guard point == sine[0]");

    sc::SinOsc so; so.Init(t, 48000.0f, 55.0f);
    float pk = 0, dc = 0; int n = 48000;   // 1 s = 55 periods
    for (int i = 0; i < n; ++i) { float v = so.Process(); dc += v; if (fabsf(v) > pk) pk = fabsf(v); }
    CHECK(pk > 0.999f && pk < 1.001f, "SinOsc peak ~1");
    CHECK(fabsf(dc / n) < 1e-3f, "SinOsc DC ~0");

    sc::LFTri tri; tri.Init(48000.0f, 480.0f);
    float tmin = 2, tmax = -2; int zc = 0, prev_sign = 1;
    for (int i = 0; i < 4800; ++i) {          // 10 periods @ 480 Hz/48k
      float v = tri.Process();
      tmin = tmin < v ? tmin : v; tmax = tmax > v ? tmax : v;
      int s = v >= 0 ? 1 : -1; if (s != prev_sign) { ++zc; prev_sign = s; }
    }
    CHECK(tmin > -1.001f && tmin < -0.999f && tmax > 0.999f && tmax < 1.001f, "LFTri pp = 2");
    CHECK(zc == 10 || zc == 11, "LFTri 10 periods");

    sc::Pulse pu; pu.Init(t, 48000.0f, 55.0f, 0.5f);
    float pmax = 0, pmin = 0;
    for (int i = 0; i < 48000; ++i) { float v = pu.Process(); pmax = pmax > v ? pmax : v; pmin = pmin < v ? pmin : v; }
    CHECK(pmax > 0.9f && pmin < -0.9f, "Pulse(0.5) swing ~ +-1 (band-limited)");
  }
```

- [ ] **Step 5: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: all sections pass, exit 0. (`kSineSize`/`kPulseTableSize`/`kPulseAmpTableSize` constants must be defined in `sc_kit_osc.h` from the Step 1 source — expected `kSineSize = 8192`.)

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_kit_osc.h rack-rp2350/src/dronecaster/sc_arena.h rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: SinOsc/LFTri/Pulse kit ports + SRAM arena, host-tested"
```

---

### Task 6: EnvAsr + SynthSocket engine

**Files:**
- Modify: `rack-rp2350/src/dronecaster/sc_kit_env.h`
- Modify: `rack-rp2350/src/dronecaster/dronecaster_engine.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc` (append engine tests)

**Interfaces:**
- Consumes: Task 5's kit structs + `KitTables`, `sc::Arena`, `sc::RGen`.
- Produces (consumed by Task 7 tests and Task 8's adapter):
  - `sc::EnvAsr` — `Init(sr, attack, release)`, `Gate(bool)`, `Process()`, `done()`.
  - `sc::DroneSpec` — `{ const char* name; void (*init)(Ctx&); void (*process)(Ctx&, float*, float*, int); }`.
  - `sc::Ctx` — the graphfn argument carrier: `{ float sr; RGen* rgen; float hz, amp; KitTables* tables; Arena* arena; }`.
  - `sc::Engine` — `Init(float sr, uint8_t* buf_mem, size_t buf_bytes)`, `SetDrone(const DroneSpec*)`, `set_hz_target(float)`, `set_amp(float)`, `set_fade(float)`, `current_name()`, `Process(float* l, float* r, int n)` (n ≤ 64, internal 64-sample buffering — the lira8 pattern).
  - `sc::kCatalog` + `sc::kCatalogCount` (37; entries 22/23/29 wired, others nullptr in T-D1).

- [ ] **Step 1: Implement `sc_kit_env.h` (EnvAsr only — perc/new come with T-D2 drones)**

```cpp
// sc_kit_env.h — EnvGen subset: Env.asr with linear curves (the socket's
// amp envelope). attack/release derive from sr at runtime. gate->0 during
// attack drops into release from the current level (SC EnvGen law).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_ENV_H_
#define RACK_SRC_DRONECASTER_SC_KIT_ENV_H_
namespace sc {

class EnvAsr {
 public:
  void Init(float sr, float attack, float release) {
    sr_ = sr; atk_inc_ = attack > 0.0f ? 1.0f / (attack * sr) : 1.0f;
    rel_inc_ = release > 0.0f ? 1.0f / (release * sr) : 1.0f;
    level_ = 0.0f; stage_ = Stage::kIdle;
  }
  void Gate(bool on) {
    if (on) { if (stage_ == Stage::kIdle) stage_ = Stage::kAttack; }
    else if (stage_ == Stage::kAttack || stage_ == Stage::kPlay) stage_ = Stage::kRelease;
  }
  // setFadeTime law: updates the leg times WITHOUT resetting level/stage
  // (resetting level would click a playing drone).
  void set_times(float attack, float release) {
    atk_inc_ = attack > 0.0f ? 1.0f / (attack * sr_) : 1.0f;
    rel_inc_ = release > 0.0f ? 1.0f / (release * sr_) : 1.0f;
  }
  float Process() {
    switch (stage_) {
      case Stage::kIdle: return 0.0f;
      case Stage::kAttack:
        level_ += atk_inc_;
        if (level_ >= 1.0f) { level_ = 1.0f; stage_ = Stage::kPlay; }
        return level_;
      case Stage::kPlay: return 1.0f;
      case Stage::kRelease:
        level_ -= rel_inc_;
        if (level_ <= 0.0f) { level_ = 0.0f; stage_ = Stage::kIdle; }
        return level_;
    }
    return 0.0f;
  }
  bool done() const { return stage_ == Stage::kIdle; }
 private:
  enum class Stage { kIdle, kAttack, kPlay, kRelease };
  float sr_ = 48000.0f, atk_inc_ = 0.0f, rel_inc_ = 0.0f, level_ = 0.0f;
  Stage stage_ = Stage::kIdle;
};

}  // namespace sc
#endif
```

- [ ] **Step 2: Implement `dronecaster_engine.h`** (socket port: mono voice, down+up swap, hz slew, internal 64 buffering)

```cpp
// dronecaster_engine.h — the DroneCaster_SynthSocket port (GPL-3.0 upstream:
// northern-information/dronecaster engine/Dronecaster_SynthSocket.sc).
// Mono voice; "we don't want xfades, we want down+up": SetDrone releases the
// current synth (gate->0); when its Env.asr finishes, the cued synth starts.
// hz glides via a one-pole kr slew (~60 ms to 95% — the approved spec's
// "short lag" deviation from SC's stepped control bus).
#ifndef RACK_SRC_DRONECASTER_DRONECASTER_ENGINE_H_
#define RACK_SRC_DRONECASTER_DRONECASTER_ENGINE_H_
#include <cstring>
#include "sc_defs.h"
#include "sc_rgen.h"
#include "sc_arena.h"
#include "sc_kit_osc.h"
#include "sc_kit_env.h"

namespace sc {

struct Ctx {
  float sr = 48000.0f;
  RGen* rgen = nullptr;
  float hz = 55.0f, amp = 0.4f;
  KitTables* tables = nullptr;
  Arena* arena = nullptr;
  void* state = nullptr;     // per-voice DSP state, arena-owned (oscillator
                             // phase etc. MUST persist across blocks)
};

struct DroneSpec {
  const char* name;
  void (*init)(Ctx&);
  void (*process)(Ctx&, float* l, float* r, int n);
};

// Catalog order = LC_ALL=C sort of drones/*.scd (verified in Task 1).
// T-D1 wires the trio; every other entry stays nullptr until its phase.
constexpr int kCatalogCount = 37;
inline const DroneSpec* const* CatalogTable();   // defined below kCatalog

class Engine {
 public:
  void Init(float sr, uint8_t* buf_mem, size_t buf_bytes) {
    sr_ = sr;
    arena_.Init(buf_mem, buf_bytes);
    rgen_.init(16);                       // server RGen seed (golden parity)
    kit_ = static_cast<KitTables*>(arena_.Alloc(sizeof(KitTables)));
    InitKitTables(kit_, &arena_);
    ctx_.sr = sr; ctx_.rgen = &rgen_; ctx_.tables = kit_; ctx_.arena = &arena_;
    ctx_.hz = 55.0f; ctx_.amp = 0.4f;
    hz_cur_ = 55.0f; hz_target_ = 55.0f;
    fade_ = 1.0f;
    env_.Init(sr, fade_, fade_);
    phase_ = Phase::kIdle;
    current_ = nullptr; cued_ = nullptr;
    internal_pos_ = 0;
  }

  // down+up: release now; the cued voice starts when the env ends.
  // SetDrone(nullptr) = gate-off with no cue (the golden's gate path).
  void SetDrone(const DroneSpec* spec) {
    if (spec == nullptr) {
      if (phase_ == Phase::kPlaying) { env_.Gate(false); phase_ = Phase::kReleasing; cued_ = nullptr; }
      return;
    }
    if (spec == current_ && phase_ != Phase::kIdle) { cued_ = spec; return; }
    cued_ = spec;
    if (phase_ == Phase::kPlaying) { env_.Gate(false); phase_ = Phase::kReleasing; }
    else if (phase_ == Phase::kIdle) StartCued();
  }
  void Recast() {                       // PLAY button: release + restart
    if (phase_ == Phase::kPlaying) { cued_ = current_; env_.Gate(false); phase_ = Phase::kReleasing; }
  }

  void set_hz_target(float hz) { hz_target_ = hz; }
  void set_amp(float a) { ctx_.amp = a; }
  void set_fade(float t) {
    fade_ = t > 0.001f ? t : 0.001f;
    env_.set_times(fade_, fade_);       // SC setFadeTime: legs only, no reset
  }
  const char* current_name() const { return current_ ? current_->name : "--"; }

  // One host block (n <= 64). Serves from the internal 64-sample buffer,
  // rendering a fresh internal block whenever it drains (lira8 burst law).
  void Process(float* out_l, float* out_r, int n) {
    for (int i = 0; i < n; ++i) {
      if (internal_pos_ == 0) RenderInternal();
      out_l[i] = buf_l_[internal_pos_];
      out_r[i] = buf_r_[internal_pos_];
      internal_pos_ = (internal_pos_ + 1) & (kBlock - 1);
    }
  }

 private:
  enum class Phase { kIdle, kPlaying, kReleasing };

  void StartCued() {
    if (cued_ == nullptr) { phase_ = Phase::kIdle; return; }
    arena_.Reset();                      // frees the old voice's buffers
    kit_ = static_cast<KitTables*>(arena_.Alloc(sizeof(KitTables)));
    InitKitTables(kit_, &arena_);        // tables rebuilt per voice (cheap)
    ctx_.tables = kit_;
    ctx_.state = nullptr;                // fresh voice state
    ctx_.hz = hz_cur_;
    cued_->init(ctx_);
    env_.Init(sr_, fade_, fade_);
    env_.Gate(true);
    current_ = cued_; cued_ = nullptr;
    phase_ = Phase::kPlaying;
  }

  void RenderInternal() {
    // hz slew: one-pole per internal block — ~60 ms to 95% (the approved
    // "short glide"; SC's control bus is stepped, this is the spec deviation).
    hz_cur_ += (hz_target_ - hz_cur_) * (64.0f / (0.02f * sr_));
    ctx_.hz = hz_cur_;
    if (phase_ == Phase::kReleasing && env_.done()) StartCued();
    if (phase_ == Phase::kPlaying || phase_ == Phase::kReleasing) {
      current_->process(ctx_, buf_l_, buf_r_, kBlock);
      for (uint32_t i = 0; i < kBlock; ++i) {
        const float e = env_.Process();
        buf_l_[i] *= e; buf_r_[i] *= e;
      }
    } else {
      for (uint32_t i = 0; i < kBlock; ++i) { buf_l_[i] = 0.0f; buf_r_[i] = 0.0f; }
    }
  }

  float sr_ = 48000.0f;
  float buf_l_[64], buf_r_[64];
  uint32_t internal_pos_ = 0;
  Arena arena_;
  KitTables* kit_ = nullptr;
  RGen rgen_;
  Ctx ctx_;
  EnvAsr env_;
  Phase phase_ = Phase::kIdle;
  const DroneSpec* current_ = nullptr;
  const DroneSpec* cued_ = nullptr;
  float hz_cur_ = 55.0f, hz_target_ = 55.0f, fade_ = 1.0f;
};

}  // namespace sc
#endif
```

- [ ] **Step 3: Define the catalog + the three drones in `dronecaster_engine.h`'s companions**

Create `rack-rp2350/src/dronecaster/drones/drone_trio.h`:

```cpp
// drone_trio.h — T-D1 drones: line-for-line C++ mirrors of the .scd graphs.
//   Sine.scd:     { arg hz, amp; (SinOsc.ar(hz) * amp).dup; }
//   Square.scd:   { arg hz, amp; (Pulse.ar(hz, 0.5) * amp).dup; }
//   Triangle.scd: { arg hz, amp; (LFTri.ar(hz) * amp).dup; }
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_TRIO_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_TRIO_H_
#include "../sc_kit_osc.h"
#include "../dronecaster_engine.h"

namespace sc {
namespace drones {

// Voice state persists across blocks (oscillator phase is the whole point of
// a drone): allocated from the voice arena in *_Init, retrieved in Process.
// Each block re-sets the osc frequency from ctx (hz slew lands here).
struct SineState { SinOsc osc; };
struct SquareState { Pulse osc; };
struct TriState { LFTri osc; };

inline void Sine_Init(Ctx& c) {
  SineState* st = static_cast<SineState*>(c.arena->Alloc(sizeof(SineState)));
  st->osc.Init(c.tables, c.sr, c.hz);
  c.state = st;
}
inline void Sine_Process(Ctx& c, float* l, float* r, int n) {
  SineState* st = static_cast<SineState*>(c.state);
  st->osc.SetFreq(c.hz);
  for (int i = 0; i < n; ++i) { float v = st->osc.Process() * c.amp; l[i] = v; r[i] = v; }
}
inline void Square_Init(Ctx& c) {
  SquareState* st = static_cast<SquareState*>(c.arena->Alloc(sizeof(SquareState)));
  st->osc.Init(c.tables, c.sr, c.hz, 0.5f);
  c.state = st;
}
inline void Square_Process(Ctx& c, float* l, float* r, int n) {
  SquareState* st = static_cast<SquareState*>(c.state);
  st->osc.SetFreq(c.hz);
  for (int i = 0; i < n; ++i) { float v = st->osc.Process() * c.amp; l[i] = v; r[i] = v; }
}
inline void Triangle_Init(Ctx& c) {
  TriState* st = static_cast<TriState*>(c.arena->Alloc(sizeof(TriState)));
  st->osc.Init(c.sr, c.hz);
  c.state = st;
}
inline void Triangle_Process(Ctx& c, float* l, float* r, int n) {
  TriState* st = static_cast<TriState*>(c.state);
  st->osc.SetFreq(c.hz);
  for (int i = 0; i < n; ++i) { float v = st->osc.Process() * c.amp; l[i] = v; r[i] = v; }
}

}  // namespace drones

inline const DroneSpec kDroneSine     = {"Sine",     drones::Sine_Init,     drones::Sine_Process};
inline const DroneSpec kDroneSquare   = {"Square",   drones::Square_Init,   drones::Square_Process};
inline const DroneSpec kDroneTriangle = {"Triangle", drones::Triangle_Init, drones::Triangle_Process};

inline const DroneSpec* Catalog(int id) {
  static const DroneSpec* const kCatalog[kCatalogCount] = {
      /* 0 Apparatus .. 21 Shields */ nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr,
      /* 22 */ &kDroneSine, /* 23 */ &kDroneSquare,
      /* 24 .. 28 */ nullptr, nullptr, nullptr, nullptr, nullptr,
      /* 29 */ &kDroneTriangle,
      /* 30 .. 36 */ nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
  };
  if (id < 0 || id >= kCatalogCount) return nullptr;
  return kCatalog[id];
}

}  // namespace sc
#endif
```

(Count the nullptrs against the Task 1 sort output — the verify step in Task 1 prints the full order; if it disagrees with the 22/23/29 positions, fix the pointer positions here AND in the panel bytes of Task 9.)

- [ ] **Step 4: Append engine tests** (before the final `return`)

```cpp
  printf("== socket engine ==\n");
  {
    static uint8_t mem[96 * 1024];
    sc::Engine e; e.Init(48000.0f, mem, sizeof(mem));
    CHECK(strcmp(e.current_name(), "--") == 0, "idle: no current drone");

    e.SetDrone(sc::Catalog(22));          // Sine
    float l[16], r[16];
    e.Process(l, r, 16);
    CHECK(strcmp(e.current_name(), "Sine") == 0, "Sine starts immediately when idle");
    float pk = 0; for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    CHECK(pk < 0.05f, "attack ramp: near-silence at t=0");

    // 0.5 s in (block 1500): attack done (level 1). A 16-sample window at 55 Hz
    // can land near a zero crossing, so take the max over 100 blocks (~18 periods).
    for (int b = 0; b < 1500; ++b) e.Process(l, r, 16);
    pk = 0;
    for (int b = 0; b < 100; ++b) {
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    }
    CHECK(pk > 0.35f && pk < 0.45f, "sustain: peak ~amp 0.4");

    // down+up: gate off, quick fade -> silence gap -> not playing anymore.
    e.set_fade(0.01f);                    // 10 ms legs
    e.SetDrone(nullptr);                  // gate-off, no cue
    float gap_pk = 0;
    for (int b = 0; b < 400; ++b) {       // 0.21 s >> 10 ms release
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) gap_pk = gap_pk > fabsf(l[i]) ? gap_pk : fabsf(l[i]);
    }
    CHECK(strcmp(e.current_name(), "--") == 0, "gate-off: env ended, idle again");
    CHECK(gap_pk < 0.01f, "down+up: silence in the gap");

    // recast: PLAY releases + restarts the current drone.
    e.SetDrone(sc::Catalog(29));          // Triangle
    e.Process(l, r, 16);
    e.Recast();
    for (int b = 0; b < 400; ++b) e.Process(l, r, 16);    // >> 10 ms gap
    CHECK(strcmp(e.current_name(), "Triangle") == 0, "recast: same drone restarted");
    pk = 0;
    for (int b = 0; b < 100; ++b) {
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    }
    CHECK(pk > 0.1f, "recast: new voice sounding");
  }
  // (Adapter-level law, tested in Task 8 by construction: an unported CC 51
  //  id never reaches SetDrone — the adapter checks Catalog(id) != nullptr
  //  first, because SetDrone(nullptr) means gate-off.)
```

(Simplify while implementing: keep the phase/recast/sustain assertions, drop the half-baked zc loop — the golden in Task 7 is the exactness arbiter. Delete the trailing placeholder loop; the CHECK(true) line is scaffolding to remove.)

- [ ] **Step 5: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: all sections pass.

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_kit_env.h rack-rp2350/src/dronecaster/dronecaster_engine.h rack-rp2350/src/dronecaster/drones/drone_trio.h rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: EnvAsr + SynthSocket engine (down+up, hz slew) + trio drones"
```

---

### Task 7: Trio goldens pass (the exactness gate)

**Files:**
- Modify: `rack-rp2350/test/test_dronecaster.cc` (append golden tests)
- Create: `rack-rp2350/test/dronecaster_ref/goldens/*.expected.wav` (already from Task 2)

**Interfaces:**
- Consumes: Task 2's goldens (schedule: voice at 48-sample offset, gate off at sample 144,000, attack/release 1.0 s, hz 55, amp 0.4, 192,000 frames), Task 6's `Engine`.
- Produces: the `render_drone_golden(int catalog_id, const char* got_path)` helper — every future drone port reuses it unchanged.

- [ ] **Step 1: Append the golden harness** (before the final `return`)

```cpp
  printf("== trio goldens ==\n");
  {
    // Schedule mirrors gen_goldens.scd EXACTLY: the voice starts at sample
    // 48 (t = 0.001 s), gate off at sample 144000 (t = 3.0 s), 192000 frames
    // total, hz 55, amp 0.4, fade 1.0 s. Sample-precise anchors (==), not
    // windowed (>), so the two renders align exactly.
    struct GoldenCase { int id; const char* name; };
    const GoldenCase cases[] = {{22, "Sine"}, {23, "Square"}, {29, "Triangle"}};
    for (const GoldenCase& gc : cases) {
      static uint8_t mem[96 * 1024];
      sc::Engine e; e.Init(48000.0f, mem, sizeof(mem));
      e.set_amp(0.4f);
      e.set_fade(1.0f);
      e.set_hz_target(55.0f);
      const int total = 192000;
      const int start = 48, gate_off = 144000;
      std::vector<float> pcm; pcm.reserve(2 * static_cast<size_t>(total));
      float l[16], r[16];
      int written = 0;
      bool started = false;
      while (written < total) {
        if (!started && written == start) { e.SetDrone(sc::Catalog(gc.id)); started = true; }
        if (written == gate_off) e.SetDrone(nullptr);   // gate->0, no cue
        e.Process(l, r, 16);
        for (int i = 0; i < 16 && written < total; ++i, ++written) {
          pcm.push_back(l[i]); pcm.push_back(r[i]);
        }
      }
      char path[128];
      snprintf(path, sizeof(path), "build/dc_%s_got.wav", gc.name);
      l8test::WriteWav16(path, pcm.data(), total, 2, 48000);
      char ref[128];
      snprintf(ref, sizeof(ref), "dronecaster_ref/goldens/%s.expected.wav", gc.name);
      GoldenReport rep;
      const bool ok = compare(ref, path, /*max_abs_gate*/ 2.0, /*corr_gate*/ 0.9999, &rep);
      printf("  %s: %s\n", gc.name, ok ? "GOLDEN PASS" : "GOLDEN FAIL");
      if (!ok) ++g_failures;
    }
  }
```

- [ ] **Step 2: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: `Sine: GOLDEN PASS`, `Square: GOLDEN PASS`, `Triangle: GOLDEN PASS`, exit 0, max ≤ 2 LSB, corr ≥ 0.9999. If a golden fails: print the worst-sample index in `compare` (add `long worst_i` to the report loop), dump 64 samples around it from both files, and diff against the corresponding UGen source section — the bug is in the port, not the gate.

- [ ] **Step 3: Full suite green**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test'
```

Expected: every TEST_BINS binary exits 0.

- [ ] **Step 4: Commit**

```bash
git add rack-rp2350/test/test_dronecaster.cc rack-rp2350/src/dronecaster/dronecaster_engine.h
git commit -m "[Add] dronecaster: trio bit-exact vs scsynth goldens (Sine/Square/Triangle)"
```

---

### Task 8: Firmware integration — adapter, window, registry, presets

**Files:**
- Create: `rack-rp2350/src/dronecaster/dronecaster_tc.h`
- Create: `rack-rp2350/src/module_dronecaster.h`
- Create: `rack-rp2350/lib/dronecaster_engine/library.json`
- Create: `rack-rp2350/lib/dronecaster_engine/dronecaster_engine_o3.h`
- Create: `rack-rp2350/lib/dronecaster_engine/dronecaster_engine_o3.cpp`
- Modify: `rack-rp2350/src/rack_window.h` (enum + symbols + section table)
- Modify: `rack-rp2350/memmap_rack.ld` (tc_dronecaster OVERLAY member + LMA)
- Modify: `rack-rp2350/src/main.cpp` (include, kRackModules entry, boot guard, 2 s report)
- Modify: `rack-rp2350/src/preset.h` (version bump if the blob layout is module-count-driven)
- Modify: `rack-rp2350/src/module_registry.h` (comment only)

**Interfaces:**
- Consumes: `sc::Engine` (Task 6), the rack pool (`rack_mem_reset/alloc`), the registry hook contract.
- Produces: `rack_modules::mod_dronecaster::{init, render, note_on, note_off, control_change, mod_set_cc_float, mod_trigger_cc, state_size, buf_bytes, report_dc}` — the registry entry wires them.

- [ ] **Step 1: `dronecaster_tc.h` (SRAM law, lira8 pattern)**

```cpp
// Rack RP2350 — Dronecaster hot-code section placement (SRAM law).
// Same mechanism as LIRA8_TC: under RACK_RP2350_TC every hot function lands
// in a .tc_dronecaster.<token> input section -> memmap_rack.ld overlay
// (boot copy + live swap). Host builds compile them plain.
#ifndef RACK_SRC_DRONECASTER_DRONECASTER_TC_H_
#define RACK_SRC_DRONECASTER_DRONECASTER_TC_H_
#if defined(RACK_RP2350_TC)
#define DRONECASTER_TC(token) __attribute__((section(".tc_dronecaster." #token)))
#else
#define DRONECASTER_TC(token)
#endif
#endif
```

Tag `Engine::RenderInternal`, `Engine::Process`, and each drone's `*_Process` with `DRONECASTER_TC(<unique-token>)` (e.g. `DRONECASTER_TC(render_internal)`, `DRONECASTER_TC(sine_process)`, `DRONECASTER_TC(square_process)`, `DRONECASTER_TC(tri_process)`) — unique tokens, no shared names.

- [ ] **Step 2: `module_dronecaster.h`** (lira8 adapter shape, standalone-only)

```cpp
// Rack RP2350 — Dronecaster adapter (T-D1, 2026-11-19).
//
// Module id 5: hand-ported norns Dronecaster engine (src/dronecaster/, GPL-3.0
// upstream) following the LIRA-8 adapter shape. Standalone 48 kHz, host grid
// 16, engine bursts 4x16 -> 64 (the lira8-standalone pattern). Spec:
// docs/superpowers/specs/2026-11-19-dronecaster-port-design.md.
//
// AUDIO CONTRACT: identical to lira8's — Process() serves from an internal
// 64-sample buffer; the 8x16 I2S bank (~2.7 ms at 48 kHz) absorbs the burst.
// CC map (spec): 51 drone(0-36) / 52 amp / 53 fade / 54 hz-offset;
// notes = re-hz (note-off ignored); PLAY 64 = recast.
#pragma once
#include <new>
#include <cstring>
#include <cstdio>

#include "dronecaster/dronecaster_engine.h"
#include "dronecaster/drones/drone_trio.h"
#include "dronecaster/dronecaster_tc.h"

#include "rack_arena.h"
#include "module_registry.h"
#include "pico/time.h"

namespace rack_modules {
namespace mod_dronecaster {

static const char* const kName = "dronecaster";
static const uint32_t kSampleRate = 48000;
static constexpr uint32_t kBlockSize = 16;
static const uint32_t kBudgetUs = 1333;   // the 64-sample internal period
static const uint32_t kI2sBuffers = 8;    // 8 x 16 frames ~ 2.7 ms
static const uint32_t kI2sWords = 16;
static const uint32_t kStallUs = 900;     // >2x the 333 us write period

// Voice carve: kit tables (sine ~33 KB) + per-voice buffers. Phase-1 drones
// are osc-only (~0 extra); T-D2 grows this with its static_asserts.
static constexpr size_t kVoiceBytes = 64 * 1024;
static_assert(kVoiceBytes <= kRackPoolBytes, "dronecaster carve exceeds pool");

static const char* const kCcMap =
    "51 drone(0-36 catalog) 52 amp 53 fade(atk=rel) 54 hz-offset +-24st; "
    "notes = re-hz (glide ~60 ms); PLAY 64 = recast";

static const uint8_t kModContinuousCcs[] = {52, 53, 54};
static const RackModDestinations kModDestinations = {
    "dronecaster: continuous amp 52, fade 53, hz-offset 54 | no triggers; "
    "notes = drone re-hz",
    sizeof(kModContinuousCcs), kModContinuousCcs, 0, nullptr, nullptr,
};

struct State {
  sc::Engine eng;
  // core-0 writes -> core-1 pending slots (the lira8 discipline)
  volatile int16_t pending_drone;       // -1 = none
  volatile bool pending_recast;
  // report mirrors (core 1 -> core 0)
  volatile uint32_t proc_avg_us;
  explicit State(uint8_t* buf, size_t bytes) : pending_drone(-1), pending_recast(false), proc_avg_us(0) {
    eng.Init(static_cast<float>(kSampleRate), buf, bytes);
  }
};

static State* s = nullptr;
static size_t state_size() { return sizeof(State); }
static constexpr size_t buf_bytes() { return kVoiceBytes; }

void init_at(uint8_t* obj_base, uint8_t* buf_base) {
  memset(buf_base, 0, kVoiceBytes);
  s = new (obj_base) State(buf_base, kVoiceBytes);
  s->pending_drone = 22;   // boot default: Sine (catalog 22) — the module is
                           // not silent until the first panel touch; a preset
                           // replay overwrites this with the saved drone
}
void init() {
  rack_mem_reset();
  init_at(static_cast<uint8_t*>(rack_mem_alloc(state_size(), 8)),
          static_cast<uint8_t*>(rack_mem_alloc(buf_bytes(), 8)));
}

void note_on(uint8_t note, uint8_t vel) {
  (void)vel;
  s->eng.set_hz_target(sc::midicps(static_cast<float>(note)));
}
void note_off(uint8_t note) { (void)note; }   // drones do not stop

void control_change(uint8_t cc, uint8_t val) {
  if (cc == 64) { if (val >= 64) s->pending_recast = true; return; }
  if (cc == 51) { s->pending_drone = static_cast<int16_t>(val); return; }  // 0-36 catalog id
  if (cc == 52) { s->eng.set_amp(static_cast<float>(val) / 127.0f); return; }
  if (cc == 53) { const float v = static_cast<float>(val) / 127.0f;
                  s->eng.set_fade(16.0f * v * v); return; }                 // 0..16 s, ~1 s at 32/127
  if (cc == 54) { const float st = (static_cast<float>(val) - 64.0f) * (24.0f / 64.0f);
                  s->eng.set_hz_target(55.0f * sc::midicps(st + 69.0f) / sc::midicps(69.0f)); return; }
}

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
  if (cc == 52) { s->eng.set_amp(v); return true; }
  if (cc == 53) { s->eng.set_fade(16.0f * v * v); return true; }
  if (cc == 54) { const float st = (v * 2.0f - 1.0f) * 24.0f;
                  s->eng.set_hz_target(55.0f * sc::midicps(st + 69.0f) / sc::midicps(69.0f)); return true; }
  return false;
}
bool mod_trigger_cc(uint8_t cc, bool high) { (void)cc; (void)high; return false; }

void render() {
  // pending slots (core-0 writes land here at the block boundary)
  if (s->pending_drone >= 0) {
    const int id = s->pending_drone; s->pending_drone = -1;
    const sc::DroneSpec* spec = sc::Catalog(id);
    if (spec != nullptr) s->eng.SetDrone(spec);   // unported id: ignored (keep
                                                  // current) — NEVER pass
                                                  // nullptr (that is gate-off)
  }
  if (s->pending_recast) { s->pending_recast = false; s->eng.Recast(); }

  uint32_t t0 = time_us_32();
  float l[kBlockSize], r[kBlockSize];
  s->eng.Process(l, r, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  s->proc_avg_us = (s->proc_avg_us * 15 + dt) / 16;   // ewma for the report

  float acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) acc += l[i] * l[i];
  const float rms = sqrtf(acc / kBlockSize);
  if (rms > rack_meter_a) rack_meter_a = rms;

  const float gain = g_render_gain;
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out = static_cast<int16_t>(l[i] * gain * 32768.0f);
    i2s.write16(out, out);
  }
  if (time_us_32() - w0 > kStallUs) ++i2s_stall_count;
}

// 2 s report extra field (main.cpp prints when id == RACK_MODULE_DRONECASTER):
const char* report_dc(char* buf, size_t n) {
  snprintf(buf, n, "%s avg=%uu", s->eng.current_name(),
           static_cast<unsigned>(s->proc_avg_us));
  return buf;
}

}  // namespace mod_dronecaster
}  // namespace rack_modules
```

(`proc_us_max`, `rack_meter_a`, `g_render_gain`, `i2s_stall_count`, `i2s.write16` are the lira8 adapter's existing context — copy its exact surrounding usage when integrating; `report_dc`'s integer-only format respects the no-%f law.)

- [ ] **Step 3: `rack_window.h`** — enum `RACK_MODULE_DRONECASTER = 5`, `RACK_MODULE_COUNT = 6`; externs `__tc_dronecaster_start/end` + `__rack_flash_dronecaster_start/end`; append the section-table entry `{"dronecaster", __tc_dronecaster_start, __tc_dronecaster_end, __rack_flash_dronecaster_start, __rack_flash_dronecaster_end}`.

- [ ] **Step 4: `memmap_rack.ld`** — add the OVERLAY member after `tc_lira8` (mirror the `tc_rings` member at line ~409):

```
        tc_dronecaster
        {
            __tc_dronecaster_start = .;
            KEEP(*(.tc_dronecaster.*))
            __tc_dronecaster_end = .;
        }
```

and the LMA bookkeeping the other members get (the explicit `AT(ORIGIN(RACK_LMA) + SIZEOF(<previous members>))` accounting — follow how tc_lira8 slots in; the overlay ASSERTs then check the window/LMA fit and will fail loudly if T-D1's hot code overflows).

- [ ] **Step 5: `main.cpp`** — four edits:
  1. Include `#include "module_dronecaster.h"` beside the other adapter includes (before the `kRackModules` table).
  2. Boot guard (line ~165): `RACK_BOOT_MODULE > 4` → `> 5`, error text gains `5 (dronecaster)`.
  3. Append the 6th `kRackModules` entry (mirror the lira8 entry at lines 300-313 with `mod_dronecaster`'s constants/hooks).
  4. 2 s report: locate the `mod=` print (grep `"mod=%` or `modeng=`), extend: when `id == RACK_MODULE_DRONECASTER`, append ` dc=<report_dc buf>` — integer-only, ≤ 560 B buffer.

- [ ] **Step 6: `preset.h`** — grep the blob layout for the per-module CC cache (e.g. `seen[...]`, `cc_cache`): if the arrays are dimensioned by a literal 4 or `RACK_MODULE_COUNT`, they follow the registry automatically — bump `kPresetVersion` 4→5 (old blobs invalidate cleanly, the documented behavior). If a literal appears, switch it to `RACK_MODULE_COUNT` first, then bump.

- [ ] **Step 7: O3 wrapper library** — `rack-rp2350/lib/dronecaster_engine/`:

`library.json`:
```json
{
  "name": "dronecaster_engine",
  "version": "0.0.0",
  "build": { "flags": ["-O3"] }
}
```

`dronecaster_engine_o3.h`:
```cpp
// Declaration for the -O3 engine entry point (lib/dronecaster_engine) —
// the lira8 pattern: src/ includes this so the LDF links the library; the
// -O3 definition lives in the .cpp.
#ifndef RACK_LIB_DRONECASTER_ENGINE_DRONECASTER_ENGINE_O3_H_
#define RACK_LIB_DRONECASTER_ENGINE_DRONECASTER_ENGINE_O3_H_
namespace sc {
class Engine;
void process_block_o3(Engine& e, float* out_l, float* out_r, int n);
}  // namespace sc
#endif
```

`dronecaster_engine_o3.cpp`:
```cpp
#include "dronecaster_engine_o3.h"
#include "dronecaster/dronecaster_engine.h"
namespace sc {
void process_block_o3(Engine& e, float* out_l, float* out_r, int n) {
  e.Process(out_l, out_r, n);
}
}  // namespace sc
```

and `module_dronecaster.h`'s `render()` calls `sc::process_block_o3(s->eng, l, r, kBlockSize)` instead of the direct method.

- [ ] **Step 8: Build (Windows pio)**

```bash
cd /c/Users/schra/Developer/mi-rp2350 && pio run -d rack-rp2350
```

Expected: `Project build complete`. Verify NO `error:` in the output, and grep the map/size report for the tc_dronecaster slot size (expect well under the A-slot; note the number in the commit message). **Do not flash yet — Task 10 does that after the panel.**

- [ ] **Step 9: Commit**

```bash
git add rack-rp2350/src/dronecaster/dronecaster_tc.h rack-rp2350/src/module_dronecaster.h \
        rack-rp2350/lib/dronecaster_engine/ rack-rp2350/src/rack_window.h rack-rp2350/memmap_rack.ld \
        rack-rp2350/src/main.cpp rack-rp2350/src/preset.h rack-rp2350/src/module_registry.h
git commit -m "[Add] rack: dronecaster module id 5 (adapter+window+registry, trio catalog) — build green"
```

---

### Task 9: Panel tab

**Files:**
- Modify: `tools/rack_panel.html` (MODULES array + MOD_DESTS)

**Interfaces:**
- Consumes: the module-object schema (name/key/info/ccMap/knobs, `steps` selectors) and `MOD_DESTS`.
- Produces: the DRONECASTER tab — CC 51 as a labeled selector with the **ported ids only** (bytes = catalog indices 22/23/29), amp/fade/hz-offset knobs.

- [ ] **Step 1: Add the module object** (after the RINGS entry, before the array's `];`)

```javascript
{
  name: 'DRONECASTER', key: 'dronecaster',
  info: '48 kHz \u00B7 16-sample blocks \u00B7 budget 1333 \u00B5s \u00B7 norns Dronecaster port (T-D1 trio)',
  ccMap: '51 drone(0-36) \u00B7 52 amp \u00B7 53 fade(atk=rel) \u00B7 54 hz-offset \u00B7 notes = re-hz \u00B7 PLAY 64 = recast',
  knobs: [
    { cc:51, label:'DRONE', steps: { labels:['SINE','SQUARE','TRIANGLE'], bytes:[22,23,29] }, def: 22/127 },
    { cc:52, label:'AMP', variant:'white', def: 51/127 },
    { cc:53, label:'FADE', variant:'white', def: 32/127 },
    { cc:54, label:'HZ\u2009OFF', variant:'white', def:.5,
      fmt: c => { const st = (b127(c.v) - 64) * 24 / 64; return (st >= 0 ? '+' : '') + st.toFixed(1); } },
  ],
},
```

- [ ] **Step 2: Add the mod-destination table**

```javascript
dronecaster: { cont: [52,53,54], trig: [] },
```

- [ ] **Step 3: Verify the tab loads**

Open `tools/rack_panel.html` in the browser (Web MIDI needs Chrome/Edge + the device connected), confirm the DRONECASTER tab renders, the DRONE selector shows three labeled steps, and switching sends CC 51 bytes 22/23/29 (visible via the panel's activity log or `aseqdump -l` + `aseqdump -p "Rack RP2350"` on the host side in WSL).

- [ ] **Step 4: Commit**

```bash
git add tools/rack_panel.html
git commit -m "[Add] panel: dronecaster tab (CC 51 labeled drone selector, amp/fade/hz)"
```

---

### Task 10: Flash, hardware acceptance, docs, ledger

**Files:**
- Modify: `AGENTS.md` (module count/list), `rack-rp2350/README.md` (module table + CC map), `.superpowers/sdd/progress.md` (ledger entry)

- [ ] **Step 1: Flash via the documented BOOTSEL flow** (AGENTS.md §RP2350 flash — the 1200-baud touch → RP2350 drive → firmware.uf2 copy; resolve the port by `/dev/serial/by-id/`, never ttyACM number). The build from Task 8 is already verified — flash it unchanged.

- [ ] **Step 2: Console smoke** (WSL, by-id port, 115200): send `\n` then `status`; expect the boot log, no stall growth, and the report field `mod=dronecaster dc=Sine avg=...`. Send CC 51 = 23 (Square) / 29 (Triangle) / 22 (Sine) from the panel; play notes on a MIDI keyboard and hear the drone re-pitch (glide); PLAY button = recast.

- [ ] **Step 3: User acceptance** — the user judges the trio by ear (level, character, note tracking). Record their verdict verbatim in the ledger. Do NOT claim hardware-verified beyond what they confirm.

- [ ] **Step 4: Docs + ledger update** — AGENTS.md "five modules" → six (add dronecaster id 5 to the boot-list comment); rack-rp2350/README.md module list + the CC map + the SRAM pool line (dronecaster carve 64 KB); ledger entry: status, commit hash, findings (catalog order, golden gates used, tc_dronecaster slot size, proc avg observed).

- [ ] **Step 5: Commit + push**

```bash
git add AGENTS.md rack-rp2350/README.md .superpowers/sdd/progress.md
git commit -m "[Docs] dronecaster T-D1 landed: module id 5, trio golden-exact — ledger + docs"
git push
```

---

## Self-Review notes (fixes applied during review)

- Spec coverage: T-D1's spec row (kit core, goldens infra, trio bit-exact, end-to-end module, first HW flash) maps to Tasks 1-10; the kit-core narrowing (no noise/delay/logic/spatial in T-D1) is declared in the header and T-D2 pulls them per-drone.
- **Drone state persistence**: the first draft re-created oscillators per block (phase reset every 64 samples). Fixed — `Ctx.state` carries arena-allocated voice state; `*_Init` allocates, `*_Process` reuses (Task 6).
- **set_fade click**: `EnvAsr::set_times` updates leg increments without resetting `level_/stage_` (Task 6).
- **SetDrone(nullptr) semantics**: gate-off-with-no-cue (golden path only). The adapter MUST check `Catalog(id) != nullptr` before calling — an unported CC 51 id is ignored, never gate-off (Tasks 6 test note + Task 8 render).
- **RGen test values**: replaced a wrong hand-computed LCG sequence with exact mod-2^32 values (`lcg(1)=1015568748`, `lcg²(1)=1586005467`) and exact `frand0` floats for seeds 1 and 7 (computed offline, Task 4).
- **Golden anchors**: sample-precise (`written == 48` start, `written == 144000` gate-off) so both renders align exactly (Task 7).
- Type consistency: `sc::Engine::Process(float*, float*, int)`, `sc::Catalog(int)->const DroneSpec*`, `sc::KitTables`, `sc::Ctx` used identically in Tasks 5/6/7/8.
