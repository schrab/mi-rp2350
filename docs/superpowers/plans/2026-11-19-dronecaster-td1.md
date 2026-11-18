# Dronecaster T-D1 Implementation Plan (rev 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land the first flashable Dronecaster rack module (id 5): seed-matched scsynth float32 golden infrastructure, the kit core (RGen taus88 + SinOsc/LFTri/Pulse + Env.asr), the SynthSocket port, and the end-to-end adapter with the Sine/Square/Triangle trio bit-exact.

**Architecture:** Hand-ported pure-C++ engine in `src/dronecaster/` (mirrors `src/lira8/`), verified against non-realtime scsynth renders with fixed seeds, wrapped by a `module_dronecaster.h` adapter following the lira8 adapter shape. Standalone 48 kHz, host grid 16 samples, engine bursts 4×16 → 64.

**Tech Stack:** Plain C++17 header-only engine (g++, no MCU headers), PlatformIO/earlephilhower RP2350 firmware, WSL SuperCollider (sclang + scsynth NRT) for goldens, Web-MIDI browser panel.

**Scope note:** This plan covers **T-D1 only** (spec §Phases). The spec's "kit basics" narrows here to RGen + osc + env + arena — exactly what the trio and the socket need (YAGNI); noise/delay/logic/spatial UGens land in T-D2 pulled per-drone. T-D2/T-D3/T-D4 get their own plans after this phase is accepted on hardware.

**Rev 2 note:** The first revision described pre-3.8 SuperCollider internals (an LCG RGen, a wavetable Pulse, a linear per-sample EnvGen). Rev 2 is written against the actual **Version-3.13.1** sources; every ported UGen below cites the exact `file:line` it was taken from. Do not "fix" a port from memory — re-read the cited source.

## Global Constraints

- Engine is **pure C++, no MCU headers** — host compilation is the portability proof.
- Rates: **48000 Hz / host grid 16 / internal block 64**; budget **1333 µs** (the 64-sample period); I2S bank **8 buffers × 16 words**; stall threshold 900 µs.
- **Denormals:** FPSCR FZ+DN already set on BOTH cores at boot — **zero** software denormal handling in the kit.
- **Sample-rate law:** every filter/time constant derived from `sr` at runtime — never baked 44.1 kHz constants.
- **No dot products** in the kit → no SMLAD; 4-accumulator float unroll only if a MAC-bound UGen appears (none in T-D1).
- **SC reference is 3.13.x (verified against the Version-3.13.1 tag).** Ports cite `file:line`; the golden renderer runs the same distro's `scsynth`/`sclang` and the host test runs on the same WSL glibc, so double-precision `exp`/`sin` match bit-for-bit.
- **Goldens are float32 WAVs.** The host gate is `max_abs ≤ 1e-6` (absolute sample units) and `corr ≥ 0.999999`; do not quantize to int16 (that wastes half the tolerance on conversion differences).
- **Golden schedule is block-aligned:** voice at sample 0 (t = 0), gate off at sample 144000 (t = 3.0 s), 192000 frames, hz 55, amp 0.4, fade 1.0 s.
- **Hot code:** tag the `-O3` wrapper function `process_block_o3` with `DRONECASTER_TC(token)` (section `.tc_dronecaster.<token>`); do NOT tag inline callees (the attribute does not survive inlining). One unique token per emitted function.
- **Kit tables are 131,080 B** (`sine_wavetable[16384]` + `sine[8193]` + `cosecant[8193]` floats). The voice carve is **136 KiB**; kit tables live above an arena savepoint and survive voice swaps.
- License: **GPL-3.0 attribution header** on every ported file (upstream: northern-information/dronecaster + supercollider/supercollider).
- `RACK_BOOT_MODULE` untouched (stays 0 = Elements).
- CC map: **51** drone select (catalog 0–36) · **52** amp (default 0.4) · **53** fade (attack=release) · **54** hz offset ±24 st **on top of the current note** (base 55 if no note) · notes → hz · PLAY pseudo-CC 64 = recast.
- Report/prints: **no `%f`** (newlib-nano), dbg line buffer 560 B.
- Host tests: `cd rack-rp2350/test && make test` in **WSL** (no g++ on Windows).
- Commit style: `[Add]/[Fix]/[Update]/[Remove] <scope>`; work on branch `rack-rp2350`.
- Hardware acceptance requires the user's ears; never claim HW-verified without their confirmation.

## SC 3.13.1 reference facts (do not re-derive, do not contradict)

| Fact | Source |
|---|---|
| `RGen` is L'Ecuyer **taus88**, not an LCG. `init(seed)`: `seed = Hash(seed); s1 = 1243598713U ^ seed; s2 = 3093459404U ^ seed; s3 = 1821928721U ^ seed` with `s1<2/s2<8/s3<16` guards. `frand0()` ∈ [1,2), `frand2()` ∈ [-1,1), `frand()` ∈ [0,1), `frand8()`, `fcoin()`; **no `frand1`**. | `include/plugin_interface/SC_RGen.h:45-205`; `Hash.h:84` (Thomas Wang int hash) |
| `kSineSize = 8192`. `gSine`/`gInvSine` are 8193 floats; `gSineWavetable` is 16384 floats of interleaved pairs `{2a-b, b-a}` (`SignalAsWavetable`). `gInvSine` has `kBadValue` bands at 0, 4096, 8192 and ±8 around them. `kBadValue = 1e20f`. | `common/Samp.cpp:24-80`; `include/plugin_interface/SC_Complex.h:39`; `SC_Constants.h:85` |
| `lookupi1(table0, table1=table0+1, pphase, lomask)`: `pfrac = PhaseFrac1(pphase)` ∈ [1,2); `index = (pphase>>13) & lomask` (byte offset); `val1 + val2*pfrac`. `xlobits = 14`, `xlomask13 = 0x7FFC` (Pulse's float-table index = `(phase>>16)&8191`). | `include/plugin_interface/SC_SndBuf.h:158-215` |
| `SinOsc`: `m_cpstoinc`/`m_radtoinc` are **double**; phase is **int32**; the loop **reads at the current phase then increments** by `(int32)(cpstoinc*freq)`. The ctor's priming call is undone by `m_phase = initPhase` (0), so the first rendered sample is at phase 0. | `server/plugins/OscUGens.cpp:1209-1348` |
| `LFTri`: `mPhase` is **double**, `mFreqMul = (float)(4.0/sr)`, `iphase` default 0; `z = phase>1 ? 2-phase : phase`, wrap at 3 by `-=4`. The ctor calls `LFTri_next_k(unit,1)` and does **not** reset the phase → the first rendered sample is at phase = one step. | `server/plugins/LFUGens.cpp:636-670`; `SCClassLibrary/Common/Audio/Osc.sc:96-114` |
| `Pulse`: a BLIT-style pulse train — `N = int(sr*0.5/freq)`, `scale = 0.5/N`, `cpstoinc = kSineSize*SAMPLEDUR*65536*0.5`, `phaseoff = duty*(1L<<28)` with a per-sample `phaseoff_slope`, numerator from `ft->mSine`, denominator from `ft->mCosecant` with `kBadValue` fallback, an N-change **crossfade** branch, and the leaky integrator `y1 = (pul1-pul2)*scale + 0.999*y1`. **No pulse wavetables exist.** | `server/plugins/OscUGens.cpp:2510-2720` |
| `Pulse` class: `Pulse.ar(freq=440, width=0.5, ...)`. | `SCClassLibrary/Common/Audio/FSinOsc.sc:130-141` |
| `Env.asr(attack, sustain, release)` defaults to **curve -4.0** and levels `[0, sustain, 0]` → shape 5 (`shape_Curve`). `EnvGen.kr` runs at **control rate**: ONE envelope value per 64-sample block; `counter = dur*(sr/64)`; the curve law is `a1=(end-level)/(1-exp(curve)); a2=level+a1; b1=a1; grow=exp(curve/counter); out=a2-b1; b1*=grow`. The ctor primes one control sample. Gate-off starts the release from the current level over the full release time. | `SCClassLibrary/Common/Audio/Env.sc:198-204,352-400`; `server/plugins/LFUGens.cpp:2471-2830` (`EnvGen_Ctor` 2471, `check_gate` 2608, `EnvGen_next_k` 2798, curve setup 2670-2682) |
| scsynth applies score events at **block boundaries**. The world RGen is seeded from `server_timeseed()` (wall clock); `RandSeed.kr(trig, seed)` re-seeds it on a rising trigger and its ctor triggers once — this is the server-side seed hook. | `server/scsynth/SC_World.cpp:400`; `server/plugins/NoiseUGens.cpp:738-770` |
| `Score:recordNRT` is **asynchronous** (`unixCmd`); the signature is `(oscFilePath, outputFilePath, inputFilePath, sampleRate, headerFormat, sampleFormat, options, completionString, duration, action)`. `Score:write` writes entries in insertion order; an entry `[time, msg1, msg2]` is one bundle. | `SCClassLibrary/Common/Control/Score.sc:92-145` |
| Reference values used by the tests below were generated by a line-by-line transcription of the above (RGen seed vectors, table bits, first osc samples, env block values, pulse samples). | this plan, Task 4/5/6/7 test vectors |

---

### Task 1: Vendor the drone corpus + the SynthSocket, pinned

**Files:**
- Create: `rack-rp2350/test/dronecaster_ref/drones/*.scd` (37 files)
- Create: `rack-rp2350/test/dronecaster_ref/Dronecaster_SynthSocket.sc`
- Create: `rack-rp2350/test/dronecaster_ref/README.md`

**Interfaces:**
- Consumes: upstream repo `northern-information/dronecaster` (GPL-3.0).
- Produces: `test/dronecaster_ref/drones/` — the corpus the golden renderer and catalog check read; the vendored socket documents the exact wrap semantics the golden reproduces.

- [ ] **Step 1: Fetch the upstream archive and pin the commit**

```bash
mkdir -p /tmp/dc_audit && cd /tmp/dc_audit
git ls-remote https://github.com/northern-information/dronecaster refs/heads/main | tee upstream_commit.txt
curl -sL https://github.com/northern-information/dronecaster/archive/refs/heads/main.tar.gz -o dc.tgz
tar xzf dc.tgz
ls dronecaster-main/engine/drones/*.scd | wc -l   # expect: 37
```

Record the 40-hex commit from `upstream_commit.txt` — it goes into the README (Step 2) and the golden commit message.

- [ ] **Step 2: Copy the corpus + socket + write the attribution README**

```bash
mkdir -p /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref/drones
cp /tmp/dc_audit/dronecaster-main/engine/drones/*.scd \
   /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref/drones/
cp /tmp/dc_audit/dronecaster-main/engine/Dronecaster_SynthSocket.sc \
   /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref/
```

Create `rack-rp2350/test/dronecaster_ref/README.md`:

```markdown
# dronecaster_ref — golden renders + drone corpus

The 37 drone functions in `drones/` and `Dronecaster_SynthSocket.sc` are verbatim
copies from https://github.com/northern-information/dronecaster (GPL-3.0),
pinned at commit `<commit from Step 1>` (main, fetched 2026-11-19). They are
vendored so the golden pipeline and the catalog-order check never depend on an
external checkout. The C++ port of each drone lives in `src/dronecaster/drones/`
and is a re-implementation, not a copy of this directory.

`gen_goldens.scd` wraps a drone exactly like `Dronecaster_SynthSocket.sc:
wrapDef` — `fn.value(K2A.ar(hz), K2A.ar(amp), 0)`, `EnvGen.kr(Env.asr(attack,
1, release), gate, doneAction: 2)`, `Out.ar(out, snd * aenv)` — and renders it
with NRT scsynth: 48 kHz, float32 WAV, 4.0 s (voice at t = 0, gate off at
t = 3.0 s, attack = release = 1.0 s, hz = 55, amp = 0.4). Seeds: the sclang
seed fixes graph construction; `RandSeed.kr(1, 16rC0FFEE)` inside the wrapper
fixes the **server** RGen (scsynth's world RGen is otherwise time-seeded).

The catalog order is the byte-wise sort of the filenames (37 entries). Verified
positions: Shields=21, Sine=22, Square=23, Thx=27, Toshiya=28, Triangle=29,
harms way=36 (0-based). The upstream Lua sorts names with `table.sort`
(byte-wise), so this is also the norns order.
```

- [ ] **Step 3: Verify the corpus + catalog order**

```bash
cd /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref
ls drones/*.scd | wc -l                                   # expect: 37
ls drones/ | LC_ALL=C sort | cat -n | sed -n '20,31p'
```

Expected (0-based): `21 Shields, 22 Sine, 23 Square, ... 27 Thx, 28 Toshiya, 29 Triangle`. If the sort disagrees, Task 7's catalog ids change accordingly — the sort output is the single source of truth for ids.

- [ ] **Step 4: Commit**

```bash
git add rack-rp2350/test/dronecaster_ref/
git commit -m "[Add] test: vendor 37 dronecaster drones + SynthSocket (GPL-3.0, pinned) for goldens"
```

---

### Task 2: scsynth float32 golden renderer (WSL, synchronous)

**Files:**
- Create: `rack-rp2350/test/dronecaster_ref/gen_goldens.scd`
- Create: `rack-rp2350/test/dronecaster_ref/run_goldens.sh`
- Create: `rack-rp2350/test/dronecaster_ref/goldens/{Sine,Square,Triangle}.expected.wav` (rendered artifacts, committed)

**Interfaces:**
- Consumes: Task 1's `drones/*.scd` + the wrap semantics of `Dronecaster_SynthSocket.sc`.
- Produces: `goldens/<Name>.expected.wav` — 192,000 frames, 48 kHz, stereo, **float32** (voice starts at sample 0, gate off at sample 144000, attack = release = 1.0 s, hz = 55.0, amp = 0.4). Task 8 compares against exactly this schedule.

- [ ] **Step 1: Install SuperCollider in WSL (skip if present)**

```bash
wsl bash -lc 'command -v sclang && command -v scsynth || { sudo apt update && sudo apt install -y supercollider; }'
wsl bash -lc 'sclang -v && scsynth -v'
```

Expected: both print version strings. **Record the version** in the README commit message; the ports below are verified against 3.13.1, and the internals cited are unchanged across 3.13.x. If the distro ships a different major/minor, stop and re-read the cited sources before proceeding.

- [ ] **Step 2: Write `gen_goldens.scd`**

```supercollider
// gen_goldens.scd — render the T-D1 trio through the SynthSocket wrap to
// float32 WAVs via NRT scsynth. Schedule matches the C++ harness exactly:
// voice at t = 0 (block 0), gate off at t = 3.0 s (block 2250), 4.0 s total.
// Rendering is SYNCHRONOUS (Score:recordNRT uses unixCmd and would race the
// file copy): write the OSC score, then run scsynth -N with systemCmd.
(
var seed = 16rC0FFEE;
var drones = ["Sine", "Square", "Triangle"];
var dur = 4.0, gateOff = 3.0;
var attack = 1.0, release = 1.0, hz = 55.0, amp = 0.4;
var dir = PathName(thisProcess.nowExecutingPath).pathOnly;
var outDir = dir ++ "goldens";

thisProcess.randSeed_(seed);          // sclang-side RNG (graph construction)
if (File.exists(outDir).not) { File.mkdir(outDir) };

drones.do { |name|
    var fn = (dir ++ "drones/" ++ name ++ ".scd").load;   // a Function
    var def = SynthDef("dc_" ++ name, {
        |hz = 55.0, amp = 1.0, out = 0, gate = 1, attack = 1.0, release = 1.0|
        var snd, aenv;
        // Server RGen seed (the world RGen is time-seeded; see SC_World.cpp).
        // RandSeed's ctor triggers once, re-seeding the RGen at synth start.
        // The C++ engine calls sc::RGen::init(sc::kGoldenSeed) with this value.
        RandSeed.kr(1, 16rC0FFEE);
        snd = fn.value(K2A.ar(hz), K2A.ar(amp), 0);       // bufnum 0: no tape buffer
        aenv = EnvGen.kr(Env.asr(attack, 1, release), gate, doneAction: 2);
        Out.ar(out, snd * aenv);
    });
    var score = Score.new;
    var opts = ServerOptions.new.numOutputBusChannels_(2).numInputBusChannels_(0);
    var oscPath = outDir ++ "/" ++ name ++ ".osc";
    var wavPath = outDir ++ "/" ++ name ++ ".expected.wav";

    // ONE bundle at t = 0. Score:section sorts entries by time with an
    // unstable sort, so two separate t=0 entries could reorder; a single
    // bundle keeps \d_recv before \s_new (asRawOSC emits bundle messages in
    // array order) and both are performed in block 0.
    score.add([0, [\d_recv, def.asBytes],
                  [\s_new, "dc_" ++ name, 1000, 0, 0, \hz, hz, \amp, amp]]);
    score.add([gateOff, [\n_set, 1000, \gate, 0]]);
    score.add([dur, [0]]);                     // score-length marker
    score.writeOSCFile(oscPath, 0, dur);

    // Synchronous render (recordNRT's unixCmd is async — do not use it here).
    // Command fields mirror Score:recordNRT exactly.
    var cmd = Score.program ++ " -N " ++ oscPath.quote ++ " _ " ++ wavPath.quote
        ++ " 48000 WAV float" ++ opts.asOptionsString;
    var rc = cmd.systemCmd;
    ("rendered " ++ name ++ " rc=" ++ rc).postln;
    if (rc != 0) { Error("scsynth failed for " ++ name).throw };
    File.delete(oscPath);
};
0.exit;
)
```

Notes (do not change without re-reading the source):
- `float` sample format is what makes the host gate meaningful; `wav_io.h` learns fmt tag 3 in Task 3.
- The score events at t = 0 and t = 3.0 land exactly on block boundaries (0 and 2250). scsynth executes them at the start of those blocks; the C++ harness starts the drone before its first `Process` call.
- `RandSeed.kr` is a `WidthFirstUGen` with no output; calling it inside the SynthDef function is enough to instantiate it.

- [ ] **Step 3: Write `run_goldens.sh` (render + determinism gate)**

```bash
#!/usr/bin/env bash
# Renders the trio goldens twice and proves byte-determinism. The trio draws
# no randomness; the server seed is pinned by RandSeed regardless.
set -euo pipefail
cd "$(dirname "$0")"

sclang gen_goldens.scd
for f in Sine Square Triangle; do
  test -s "goldens/$f.expected.wav" || { echo "missing $f"; exit 1; }
done

mkdir -p /tmp/dc_det1
cp goldens/*.expected.wav /tmp/dc_det1/
sclang gen_goldens.scd

for f in Sine Square Triangle; do
  if cmp -s "/tmp/dc_det1/$f.expected.wav" "goldens/$f.expected.wav"; then
    echo "determinism: $f OK"
  else
    echo "determinism: $f DIFFERS"; exit 1
  fi
done
rm -rf /tmp/dc_det1
```

- [ ] **Step 4: Render + verify**

```bash
cd /c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref && bash run_goldens.sh'
ls -la goldens/   # expect: 3 x ~1.5 MB float32 WAVs
```

Expected: `determinism: Sine OK` ×3.

- [ ] **Step 5: Metadata check (float32, 192000 frames, stereo)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test/dronecaster_ref && python3 - <<PY
import wave
for n in ("Sine","Square","Triangle"):
    w = wave.open(f"goldens/{n}.expected.wav","rb")
    print(n, w.getframerate(), w.getnchannels(), w.getnframes(), w.getsampwidth()*8, "bit")
PY'
```

Expected: `48000 2 192000 32 bit` for each. (`wave` handles IEEE-float WAVs as 32-bit samples.)

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/test/dronecaster_ref/gen_goldens.scd \
        rack-rp2350/test/dronecaster_ref/run_goldens.sh \
        rack-rp2350/test/dronecaster_ref/goldens/
git commit -m "[Add] test: scsynth float32 NRT goldens for dronecaster trio (seed-pinned, block-aligned)"
```

---

### Task 3: Host comparator (float32) + wav_io.h extension + Makefile

**Files:**
- Modify: `rack-rp2350/test/wav_io.h` (fmt tag 3 read + `WriteWavF32`)
- Create: `rack-rp2350/test/test_dronecaster.cc`
- Modify: `rack-rp2350/test/Makefile` (TEST_BINS + target)

**Interfaces:**
- Consumes: `l8test::ReadWav` (extended), `l8test::WriteWavF32` (new).
- Produces: `compare(ref_path, got_path, max_abs_gate, corr_gate, &report)` → bool, and `GoldenReport` — Task 8 uses it for the trio; every later golden test reuses it unchanged.

- [ ] **Step 1: Extend `wav_io.h`**

Replace `ReadWav` and add `WriteWavF32` (leave `WriteWav16` untouched — other tests use it):

```cpp
// Float32 WAV (IEEE, fmt tag 3): little-endian host assumed (x86-64 host
// tests and the RP2350 M33 both are).
inline void WriteWavF32(const char* path, const float* interleaved,
                        int frames, int channels, int sample_rate) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  const int32_t data_bytes = frames * channels * 4;
  const uint8_t hdr[44] = {
      'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
      16, 0, 0, 0, 3, 0, static_cast<uint8_t>(channels), 0, 0, 0, 0, 0, 0, 0,
      0, 0, static_cast<uint8_t>(channels * 4), 0, 32, 0, 'd', 'a', 't', 'a',
      0, 0, 0, 0};
  uint8_t h[44];
  memcpy(h, hdr, 44);
  auto poke32 = [&](int off, int32_t v) { memcpy(h + off, &v, 4); };
  poke32(4, 36 + data_bytes);
  poke32(24, sample_rate);
  poke32(28, sample_rate * channels * 4);
  poke32(40, data_bytes);
  fwrite(h, 1, 44, f);
  fwrite(interleaved, 4, static_cast<size_t>(frames) * channels, f);
  fclose(f);
}

inline WavFile ReadWav(const char* path) {
  WavFile out;
  FILE* f = fopen(path, "rb");
  if (!f) return out;
  uint8_t h[44];
  if (fread(h, 1, 44, f) != 44) { fclose(f); return out; }
  const int format = h[20] | (h[21] << 8);   // 1 = PCM, 3 = IEEE float
  const int channels = h[22] | (h[23] << 8);
  const int sample_rate = h[24] | (h[25] << 8) | (h[26] << 16) | (h[27] << 24);
  const int data_bytes = h[40] | (h[41] << 8) | (h[42] << 16) | (h[43] << 24);
  const int bits = h[34] | (h[35] << 8);
  out.channels = channels;
  out.sample_rate = sample_rate;
  if (format == 3 && bits == 32) {
    std::vector<uint8_t> raw(static_cast<size_t>(data_bytes));
    const size_t got = fread(raw.data(), 1, static_cast<size_t>(data_bytes), f);
    const int n = static_cast<int>(got / 4);
    out.samples.resize(static_cast<size_t>(n));
    memcpy(out.samples.data(), raw.data(), static_cast<size_t>(n) * 4);
  } else if (bits == 16) {
    std::vector<uint8_t> raw(static_cast<size_t>(data_bytes));
    const size_t got = fread(raw.data(), 1, static_cast<size_t>(data_bytes), f);
    const int n = static_cast<int>(got / 2);
    out.samples.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const int16_t s = static_cast<int16_t>(raw[2 * i] | (raw[2 * i + 1] << 8));
      out.samples[i] = static_cast<float>(s) / 32768.0f;
    }
  }
  fclose(f);
  return out;
}
```

- [ ] **Step 2: Write the comparator + self-tests**

Create `rack-rp2350/test/test_dronecaster.cc`:

```cpp
// Dronecaster host tests (T-D1). Sections are appended by plan task;
// the comparator is proven here against synthetic float32 cases so a later
// golden FAIL is trustworthy.
#include "wav_io.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static int g_failures = 0;
#define CHECK(cond, msg) do { if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); ++g_failures; } } while (0)

struct GoldenReport {
  long   frames_ref = 0, frames_got = 0;
  double max_abs = 0.0;      // absolute sample units
  long   worst_i = 0;        // sample index of max |delta|
  double rms_db = -999.0;    // 20*log10(rms(diff)/rms(ref))
  double corr = 0.0;
};

// Compare two float32 WAVs in the float domain. Gates set by the caller.
static bool compare(const char* ref_path, const char* got_path,
                    double max_abs_gate, double corr_gate, GoldenReport* rep) {
  l8test::WavFile a = l8test::ReadWav(ref_path);
  l8test::WavFile b = l8test::ReadWav(got_path);
  rep->frames_ref = static_cast<long>(a.samples.size() / (a.channels ? a.channels : 1));
  rep->frames_got = static_cast<long>(b.samples.size() / (b.channels ? b.channels : 1));
  if (a.channels == 0 || b.channels == 0 || a.sample_rate != b.sample_rate ||
      a.channels != b.channels || a.samples.size() != b.samples.size()) {
    printf("  FAIL: layout ref=%ldf/%dch/%dHz got=%ldf/%dch/%dHz\n",
           rep->frames_ref, a.channels, a.sample_rate,
           rep->frames_got, b.channels, b.sample_rate);
    return false;
  }
  const long n = static_cast<long>(a.samples.size());
  double num = 0, den_a = 0, den_b = 0, diff_sq = 0, max_abs = 0;
  long worst_i = 0;
  for (long i = 0; i < n; ++i) {
    const double d = static_cast<double>(a.samples[i]) - b.samples[i];
    if (fabs(d) > max_abs) { max_abs = fabs(d); worst_i = i; }
    diff_sq += d * d;
    num   += static_cast<double>(a.samples[i]) * b.samples[i];
    den_a += static_cast<double>(a.samples[i]) * a.samples[i];
    den_b += static_cast<double>(b.samples[i]) * b.samples[i];
  }
  rep->max_abs = max_abs;
  rep->worst_i = worst_i;
  rep->rms_db = (diff_sq > 0 && den_a > 0)
      ? 20.0 * log10(sqrt(diff_sq / n) / sqrt(den_a / n)) : -999.0;
  rep->corr = (den_a > 0 && den_b > 0) ? num / sqrt(den_a * den_b) : 0.0;
  printf("  metrics: max=%.9f rms=%.1f dB corr=%.9f worst_i=%ld\n",
         rep->max_abs, rep->rms_db, rep->corr, rep->worst_i);
  return rep->max_abs <= max_abs_gate && rep->corr >= corr_gate;
}

static void write_constant_wav(const char* path, float v, int frames) {
  std::vector<float> x(static_cast<size_t>(2 * frames), v);
  l8test::WriteWavF32(path, x.data(), frames, 2, 48000);
}

int main() {
  printf("== comparator self-tests ==\n");
  {
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", 0.5f, 480);
    GoldenReport r;
    CHECK(compare("build/dc_a.wav", "build/dc_b.wav", 0.0, 1.0, &r),
          "identical files: max=0 corr=1");
    CHECK(r.max_abs == 0.0 && r.corr >= 0.999999999, "identical metrics exact");
  }
  {
    // +1 int16 LSB (1/32768) offset, written as floats.
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", 0.5f + 1.0f / 32768.0f, 480);
    GoldenReport r;
    CHECK(compare("build/dc_a.wav", "build/dc_b.wav", 1e-4, 0.0, &r),
          "+1 LSB offset passes the 1e-4 gate");
    CHECK(fabs(r.max_abs - 1.0 / 32768.0) < 1e-9, "+1 LSB measured exactly");
  }
  {
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", -0.5f, 480);
    GoldenReport r;
    CHECK(!compare("build/dc_a.wav", "build/dc_b.wav", 1e-4, 0.999999, &r),
          "polarity flip rejected by corr gate");
  }
  {
    // Float round-trip: values must survive ReadWav bit-for-bit.
    std::vector<float> x = {0.123456789f, -0.5f, 1.25f, -1.0f};
    l8test::WriteWavF32("build/dc_f32.wav", x.data(), 2, 2, 48000);
    l8test::WavFile w = l8test::ReadWav("build/dc_f32.wav");
    CHECK(w.samples.size() == x.size() && w.samples == x, "float32 round-trip exact");
  }
  printf("comparator self-tests: %s (%d failures)\n",
         g_failures ? "FAIL" : "PASS", g_failures);

  // task sections are inserted above this line

  printf("test_dronecaster: %s (%d failures)\n",
         g_failures ? "FAIL" : "PASS", g_failures);
  return g_failures ? 1 : 0;
}
```

- [ ] **Step 3: Wire the Makefile**

In `rack-rp2350/test/Makefile`, add `test_dronecaster` to `TEST_BINS`, then:

```make
# Dronecaster host tests: engine headers are header-only; the wildcard keeps
# the dependency list honest as tasks add files.
DC_HEADERS = $(wildcard ../src/dronecaster/*.h) \
             $(wildcard ../src/dronecaster/drones/*.h)

test_dronecaster: test_dronecaster.cc wav_io.h $(DC_HEADERS) | $(BUILD)
	$(CXX) $(CXXFLAGS) -I../src/dronecaster -I. $< -o $(BUILD)$@
```

- [ ] **Step 4: Run (WSL) and verify pass**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: `comparator self-tests: PASS (0 failures)` and `test_dronecaster: PASS (0 failures)`.

- [ ] **Step 5: Commit**

```bash
git add rack-rp2350/test/wav_io.h rack-rp2350/test/test_dronecaster.cc rack-rp2350/test/Makefile
git commit -m "[Add] test: dronecaster float32 golden comparator + wav_io float support"
```

---

### Task 4: sc_defs.h + sc_rgen.h (taus88 port)

**Files:**
- Modify: `rack-rp2350/src/dronecaster/sc_defs.h` (create the directory)
- Modify: `rack-rp2350/src/dronecaster/sc_rgen.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc` (append RGen tests)

**Interfaces:**
- Produces: `sc::kBlock` (64), `sc::kGoldenSeed` (0xC0FFEE), `sc::clampf`, `sc::midicps`, `sc::cpsmidi`, `sc::RGen` with `init(uint32_t)`, `trand()`, `frand()`, `frand0()`, `frand2()`, `frand8()`, `fcoin()` — every later task consumes these exact names. RGen's state layout (`s1,s2,s3`) is public so the tests can pin the seed vector.

- [ ] **Step 1: Fetch and read the two source files (verify anchors, do not guess)**

```bash
wsl bash -lc 'mkdir -p /tmp/scsrc131 && cd /tmp/scsrc131 && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/include/plugin_interface/SC_RGen.h -o SC_RGen.h && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/include/plugin_interface/Hash.h -o Hash.h && \
  grep -n "taus\|trand\|frand0\|frand2\|frand8\|fcoin\|1243598713\|3093459404\|1821928721" SC_RGen.h | head -30 && \
  sed -n "84,100p" Hash.h'
```

Expected: the taus88 update, the three XOR constants, and Thomas Wang's `Hash(int32)`. If the tag 404s (file moved), use GitHub's tree view for the checkout's `SCVersion.txt` and re-locate — do not substitute an older version.

- [ ] **Step 2: Implement `sc_defs.h`**

```cpp
// sc_defs.h — shared constants + math helpers for the dronecaster engine.
// Rate-parameterized: NO baked 44.1 kHz constants (the sample-rate law).
#ifndef RACK_SRC_DRONECASTER_SC_DEFS_H_
#define RACK_SRC_DRONECASTER_SC_DEFS_H_
#include <cmath>
#include <cstdint>

namespace sc {

constexpr uint32_t kBlock = 64;               // scsynth block size, 48 kHz
// The golden wrapper seeds the server RGen with RandSeed.kr(1, 16rC0FFEE);
// the engine calls RGen::init(kGoldenSeed) with the same value.
constexpr uint32_t kGoldenSeed = 0xC0FFEEu;

inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float midicps(float n) { return 440.0f * powf(2.0f, (n - 69.0f) / 12.0f); }
inline float cpsmidi(float f) { return 12.0f * log2f(f / 440.0f) + 69.0f; }

}  // namespace sc
#endif
```

- [ ] **Step 3: Implement `sc_rgen.h` (taus88, verbatim laws)**

```cpp
// sc_rgen.h — exact port of scsynth's RGen (include/plugin_interface/SC_RGen.h,
// GPL-3.0, supercollider/supercollider). SC 3.13.x uses L'Ecuyer's taus88
// three-component Tausworthe generator — NOT an LCG. Seed-matched so the C++
// engine reproduces the server's random sequences sample-for-sample.
#ifndef RACK_SRC_DRONECASTER_SC_RGEN_H_
#define RACK_SRC_DRONECASTER_SC_RGEN_H_
#include <cstdint>

namespace sc {

// Hash(int32) — Thomas Wang's integer hash (include/plugin_interface/Hash.h:84)
inline int32_t Hash(int32_t inKey) {
  uint32_t hash = static_cast<uint32_t>(inKey);
  hash += ~(hash << 15);
  hash ^= hash >> 10;
  hash += hash << 3;
  hash ^= hash >> 6;
  hash += ~(hash << 11);
  hash ^= hash >> 16;
  return static_cast<int32_t>(hash);
}

class RGen {
 public:
  // SC_RGen.h:63-77 — the seed is hashed, then the three states are XORed
  // with fixed constants and guarded away from zero.
  void init(uint32_t seed) {
    seed = static_cast<uint32_t>(Hash(static_cast<int32_t>(seed)));
    s1 = 1243598713u ^ seed;
    if (s1 < 2u) s1 = 1243598713u;
    s2 = 3093459404u ^ seed;
    if (s2 < 8u) s2 = 3093459404u;
    s3 = 1821928721u ^ seed;
    if (s3 < 16u) s3 = 1821928721u;
  }

  // SC_RGen.h:81-93
  uint32_t trand() {
    s1 = ((s1 & static_cast<uint32_t>(-2)) << 12) ^ (((s1 << 13) ^ s1) >> 19);
    s2 = ((s2 & static_cast<uint32_t>(-8)) << 4) ^ (((s2 << 2) ^ s2) >> 25);
    s3 = ((s3 & static_cast<uint32_t>(-16)) << 17) ^ (((s3 << 3) ^ s3) >> 11);
    return s1 ^ s2 ^ s3;
  }

  static float bits_to_float(uint32_t i) {
    union { uint32_t u; float f; } v;
    v.u = i;
    return v.f;
  }

  // SC_RGen.h:143-160 — note the ranges: frand0 = [1,2), frand2 = [-1,1).
  float frand()  { return bits_to_float(0x3F800000u | (trand() >> 9)) - 1.0f; }
  float frand0() { return bits_to_float(0x3F800000u | (trand() >> 9)); }
  float frand2() { return bits_to_float(0x40000000u | (trand() >> 9)) - 3.0f; }
  float frand8() { return bits_to_float(0x3E800000u | (trand() >> 9)) - 0.375f; }
  float fcoin()  { return bits_to_float(0x3F800000u | (0x80000000u & trand())); }

  uint32_t s1 = 0, s2 = 0, s3 = 0;
};

}  // namespace sc
#endif
```

- [ ] **Step 4: Append RGen tests** (above the `// task sections are inserted above this line` marker)

Add the includes at the top of `test_dronecaster.cc` (after `#include "wav_io.h"`):

```cpp
#include "../src/dronecaster/sc_defs.h"
#include "../src/dronecaster/sc_rgen.h"
```

The expected values below are exact outputs of the port above (integer truncation + float bit patterns); equality is intentional.

```cpp
  printf("== sc::RGen (taus88) ==\n");
  {
    sc::RGen r;
    r.init(1);
    CHECK(r.s1 == 681917145u && r.s2 == 3671622764u && r.s3 == 237148337u,
          "init(1) state vector");
    CHECK(r.trand() == 3477746156u, "trand(1)[0] == 3477746156");
    CHECK(r.trand() == 3423392677u, "trand(1)[1] == 3423392677");
    CHECK(r.trand() == 4088519076u, "trand(1)[2] == 4088519076");

    sc::RGen a; a.init(1);
    CHECK(a.frand0() == 1.809725760f, "frand0(1)[0] == 1.80972576");
    CHECK(a.frand0() == 1.797070620f, "frand0(1)[1] == 1.79707062");
    sc::RGen b; b.init(1);
    CHECK(b.frand2() == 0.619451523f, "frand2(1)[0] == 0.619451523");
    sc::RGen c; c.init(1);
    CHECK(c.frand() == 0.809725761f, "frand(1)[0] == 0.809725761");
    sc::RGen d; d.init(1);
    CHECK(d.fcoin() == -1.0f, "fcoin(1)[0] == -1");

    sc::RGen e; e.init(7);
    CHECK(e.s1 == 4158996277u && e.s2 == 93880704u && e.s3 == 3512900957u,
          "init(7) state vector");
    CHECK(e.frand0() == 1.190675500f, "frand0(7)[0] == 1.1906755");
    CHECK(e.frand0() == 1.194282050f, "frand0(7)[1] == 1.19428205");
    sc::RGen f; f.init(7);
    CHECK(f.frand2() == -0.618649006f, "frand2(7)[0] == -0.618649006");

    sc::RGen g; g.init(sc::kGoldenSeed);
    CHECK(g.s1 == 2304607986u && g.s2 == 2065709127u && g.s3 == 2950309018u,
          "kGoldenSeed state vector");
    CHECK(g.trand() == 592871963u, "kGoldenSeed trand[0] == 592871963");
  }
```

- [ ] **Step 5: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: comparator + RGen sections all `ok`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_defs.h rack-rp2350/src/dronecaster/sc_rgen.h \
        rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: sc_defs + scsynth taus88 RGen port (seed-matched), host-tested"
```

---

### Task 5: Kit tables + SinOsc + LFTri + arena

**Files:**
- Modify: `rack-rp2350/src/dronecaster/sc_kit_osc.h`
- Modify: `rack-rp2350/src/dronecaster/sc_arena.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `sc_defs.h` (`kBlock`, `kGoldenSeed`).
- Produces: `sc::kSineSize`, `sc::kBadValue`, `sc::KitTables` (131,080 B), `sc::InitKitTables(KitTables*)`, `sc::PhaseFrac/PhaseFrac1/lininterp/lookupi1`, `sc::SinOsc` and `sc::LFTri` (each `Init(...)`, `SetFreq(float)`, `Process()`, `ProcessBlock(float*, int)`), `sc::Arena` (`Init/Alloc/Mark/ResetTo/Reset/used`) — Tasks 6/7/9 consume these exactly.

- [ ] **Step 1: Fetch and read the osc source (verify anchors)**

```bash
wsl bash -lc 'cd /tmp/scsrc131 && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/server/plugins/OscUGens.cpp -o OscUGens.cpp && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/server/plugins/LFUGens.cpp -o LFUGens.cpp && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/common/Samp.cpp -o Samp.cpp && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/include/plugin_interface/SC_SndBuf.h -o SC_SndBuf.h && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/include/plugin_interface/SC_Constants.h -o SC_Constants.h && \
  sed -n "1209,1260p" OscUGens.cpp && sed -n "1320,1348p" OscUGens.cpp && \
  sed -n "636,672p" LFUGens.cpp && sed -n "20,80p" Samp.cpp && sed -n "158,215p" SC_SndBuf.h'
```

Read the whole blocks. Pin: the ctor reset at the end of `SinOsc_Ctor` (`unit->m_phase = initPhase;`), the read-before-increment loop, `LFTri_Ctor`'s un-reset phase advance, the interleaved wavetable construction, and the `kBadValue` bands.

- [ ] **Step 2: Implement `sc_kit_osc.h`**

```cpp
// sc_kit_osc.h — SC 3.13.1 wavetable + SinOsc + LFTri ports (GPL-3.0).
// Sources: common/Samp.cpp (FillTables/SignalAsWavetable),
// server/plugins/OscUGens.cpp:1209-1348 (SinOsc_Ctor, Osc_iai_perform),
// server/plugins/LFUGens.cpp:636-670 (LFTri_Ctor, LFTri_next_k),
// include/plugin_interface/SC_SndBuf.h:158-215 (PhaseFrac1/lookupi1).
// NO software denormal handling (FPSCR FZ/DN is global on both cores).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_OSC_H_
#define RACK_SRC_DRONECASTER_SC_KIT_OSC_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"

namespace sc {

constexpr int kSineSize = 8192;      // include/plugin_interface/SC_Complex.h:39
constexpr float kBadValue = 1e20f;   // include/plugin_interface/SC_Constants.h:85

// exact 131,080 B: 2*8192 + 8193 + 8193 floats
struct KitTables {
  float sine_wt[2 * kSineSize];   // SC gSineWavetable: pairs {2a-b, b-a}
  float sine[kSineSize + 1];      // SC gSine (Pulse numerator)
  float cosecant[kSineSize + 1];  // SC gInvSine (Pulse denominator)
};
static_assert(sizeof(KitTables) == (2 * kSineSize + 2 * (kSineSize + 1)) * 4,
              "KitTables layout/padding changed");

// SC_SndBuf.h:175-181
inline float PhaseFrac(uint32_t inPhase) {
  union { uint32_t itemp; float ftemp; } u;
  u.itemp = 0x3F800000u | (0x007FFF80u & (inPhase << 7));
  return u.ftemp - 1.0f;
}
inline float PhaseFrac1(uint32_t inPhase) {
  union { uint32_t itemp; float ftemp; } u;
  u.itemp = 0x3F800000u | (0x007FFF80u & (inPhase << 7));
  return u.ftemp;
}
inline float lininterp(float x, float a, float b) { return a + x * (b - a); }

// SC_SndBuf.h:206-213 (table1 must be table0 + 1 float)
inline float lookupi1(const float* table0, const float* table1,
                      uint32_t pphase, int32_t lomask) {
  const float pfrac = PhaseFrac1(pphase);
  const uint32_t index = ((pphase >> 13) & static_cast<uint32_t>(lomask));
  const float val1 = *reinterpret_cast<const float*>(
      reinterpret_cast<const char*>(table0) + index);
  const float val2 = *reinterpret_cast<const float*>(
      reinterpret_cast<const char*>(table1) + index);
  return val1 + val2 * pfrac;
}

inline void InitKitTables(KitTables* t) {
  const double twopi = 6.283185307179586476925286766559;
  const double sineIndexToPhase = twopi / kSineSize;
  for (int i = 0; i <= kSineSize; ++i) {
    const double phase = i * sineIndexToPhase;
    const float d = static_cast<float>(std::sin(phase));   // SC: float32 d = sin(phase)
    t->sine[i] = d;
    t->cosecant[i] = static_cast<float>(1.0 / static_cast<double>(d));
  }
  // SignalAsWavetable(gSine, gSineWavetable, kSineSize) — common/Samp.cpp:29-41
  float val1, val2;
  float* out = t->sine_wt - 1;
  for (int i = 0; i < kSineSize - 1; ++i) {
    val1 = t->sine[i];
    val2 = t->sine[i + 1];
    *++out = 2.0f * val1 - val2;
    *++out = val2 - val1;
  }
  val1 = t->sine[kSineSize - 1];
  val2 = t->sine[0];
  *++out = 2.0f * val1 - val2;
  *++out = val2 - val1;
  // kBadValue bands — common/Samp.cpp:72-78
  t->cosecant[0] = t->cosecant[kSineSize / 2] = t->cosecant[kSineSize] = kBadValue;
  const int sz = kSineSize;
  const int sz2 = sz >> 1;
  for (int i = 1; i <= 8; ++i) {
    t->cosecant[i] = t->cosecant[sz - i] = kBadValue;
    t->cosecant[sz2 - i] = t->cosecant[sz2 + i] = kBadValue;
  }
}

// SinOsc (OscUGens.cpp:1209-1348). a-rate freq via K2A, phase input 0.
// The ctor's priming call is UNDONE by `unit->m_phase = initPhase` — the
// first rendered sample is at phase 0. The loop reads, then increments.
struct SinOsc {
  void Init(const KitTables* t, double sr, float freq) {
    t_ = t;
    cpstoinc_ = static_cast<double>(kSineSize) * (1.0 / sr) * 65536.0;
    lomask_ = (kSineSize - 1) << 3;
    phase_ = 0;
    freq_ = freq;
  }
  void SetFreq(float f) { freq_ = f; }
  float Process() {
    const float z = lookupi1(t_->sine_wt, t_->sine_wt + 1, phase_, lomask_);
    // SC stores a wrapping int32 phase; use uint32 here so the wrap is
    // defined (same two's-complement bits) and UBSan stays clean.
    phase_ += static_cast<uint32_t>(static_cast<int32_t>(cpstoinc_ * freq_));
    return z;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Process();
  }
  uint32_t phase() const { return phase_; }

 private:
  const KitTables* t_ = nullptr;
  double cpstoinc_ = 0.0;
  int32_t lomask_ = 0;
  uint32_t phase_ = 0;
  float freq_ = 0.0f;
};

// LFTri (LFUGens.cpp:636-670). iphase default 0 (LFSaw class defaults);
// double phase; 4/sr phase rate; ctor advances one step WITHOUT resetting.
struct LFTri {
  void Init(double sr, float freq) {
    freqmul_ = static_cast<float>(4.0 * (1.0 / sr));
    phase_ = 0.0;
    freq_ = freq;
    // LFTri_Ctor calls LFTri_next_k(unit, 1): the z at phase 0 (0) is
    // discarded, the phase advances one step.
    phase_ += freq_ * freqmul_;
    if (phase_ >= 3.0) phase_ -= 4.0;
  }
  void SetFreq(float f) { freq_ = f; }
  float Process() {
    const float z = phase_ > 1.0 ? static_cast<float>(2.0 - phase_)
                                 : static_cast<float>(phase_);
    phase_ += freq_ * freqmul_;
    if (phase_ >= 3.0) phase_ -= 4.0;
    return z;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Process();
  }

 private:
  double phase_ = 0.0;
  float freqmul_ = 0.0f;
  float freq_ = 0.0f;
};

}  // namespace sc
#endif
```

- [ ] **Step 3: Implement `sc_arena.h` (with a savepoint)**

```cpp
// sc_arena.h — bump allocator over the engine's SRAM carve. Reset per voice,
// with a Mark/ResetTo savepoint so the kit tables (built once) survive swaps.
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
    if (off_ + bytes > cap_) return nullptr;   // caller asserts capacity
    void* p = base_ + off_;
    off_ += bytes;
    return p;
  }
  size_t Mark() const { return off_; }
  void ResetTo(size_t mark) { off_ = mark; }
  void Reset() { off_ = 0; }
  size_t used() const { return off_; }
  size_t capacity() const { return cap_; }

 private:
  uint8_t* base_ = nullptr;
  size_t cap_ = 0, off_ = 0;
};

}  // namespace sc
#endif
```

- [ ] **Step 4: Append tables/arena/SinOsc/LFTri tests**

Add the includes at the top of `test_dronecaster.cc` (after the RGen includes):

```cpp
#include "../src/dronecaster/sc_kit_osc.h"
#include "../src/dronecaster/sc_arena.h"
```

Expected values are the exact outputs of the port (the float bit patterns are pinned where the value is a table constant; sample values use a small tolerance).

```cpp
  printf("== kit tables + arena ==\n");
  {
    static sc::KitTables t;
    sc::InitKitTables(&t);
    CHECK(t.sine[0] == 0.0f, "sine[0] == 0");
    CHECK(t.sine[sc::kSineSize / 4] == 1.0f, "sine[2048] == 1");
    CHECK(t.sine[1024] == 0.707106769f, "sine[1024] exact");
    CHECK(t.sine[1] == 0.000766990299f, "sine[1] exact");
    CHECK(fabsf(t.sine[sc::kSineSize]) < 1e-6f, "sine guard ~0");
    CHECK(t.sine_wt[0] == -0.000766990299f && t.sine_wt[1] == 0.000766990299f,
          "wavetable pair[0] exact");
    CHECK(t.cosecant[0] == sc::kBadValue && t.cosecant[1] == sc::kBadValue &&
              t.cosecant[8] == sc::kBadValue,
          "cosecant sentinels (0 and +-8 bands)");
    CHECK(t.cosecant[9] == 144.867523f, "cosecant[9] exact (band ends at 8)");
    CHECK(fabsf(t.cosecant[100] - 1.0f / t.sine[100]) < 1e-3f, "cosecant = 1/sine");

    static uint8_t mem[64 * 1024];
    sc::Arena a;
    a.Init(mem, sizeof(mem));
    void* p1 = a.Alloc(16);
    const size_t mark = a.Mark();
    void* p2 = a.Alloc(64);
    CHECK(p1 != nullptr && p2 != nullptr && a.used() >= mark + 64, "arena alloc");
    a.ResetTo(mark);
    CHECK(a.used() == mark, "arena savepoint restores");
    a.Reset();
    CHECK(a.used() == 0, "arena reset");
  }

  printf("== SinOsc / LFTri ==\n");
  {
    static sc::KitTables t;
    sc::InitKitTables(&t);

    sc::SinOsc so;
    so.Init(&t, 48000.0, 55.0f);
    CHECK(so.Process() == 0.0f, "SinOsc sample0 == 0");
    CHECK(fabsf(so.Process() - 0.00719941361f) < 1e-7f, "SinOsc sample1 == 0.007199414");
    CHECK(fabsf(so.Process() - 0.0143984547f) < 1e-7f, "SinOsc sample2 == 0.014398455");
    CHECK(fabsf(so.Process() - 0.0215967484f) < 1e-7f, "SinOsc sample3 == 0.021596748");
    CHECK(so.phase() == 2460656, "SinOsc phase after 4 samples == 2460656");
    sc::SinOsc so2;
    so2.Init(&t, 48000.0, 55.0f);
    for (int i = 0; i < 48000; ++i) (void)so2.Process();
    CHECK(so2.phase() == 3758068224u, "SinOsc phase after 48000 samples exact");

    sc::LFTri tri;
    tri.Init(48000.0, 55.0f);
    CHECK(fabsf(tri.Process() - 0.00458333315f) < 1e-9f, "LFTri sample0 == 0.004583333");
    CHECK(fabsf(tri.Process() - 0.00916666631f) < 1e-9f, "LFTri sample1 == 0.009166666");
    float pk = 0.0f, dc = 0.0f;
    for (int i = 0; i < 48000; ++i) {
      const float v = tri.Process();
      dc += v;
      if (fabsf(v) > fabsf(pk)) pk = v;
    }
    CHECK(fabsf(pk) > 0.99f && fabsf(pk) <= 1.0f, "LFTri peak ~1");
    CHECK(fabsf(dc / 48000.0f) < 1e-3f, "LFTri DC ~0");
  }
```

- [ ] **Step 5: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: all sections `ok`, exit 0.

- [ ] **Step 6: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_kit_osc.h rack-rp2350/src/dronecaster/sc_arena.h \
        rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: SC sine wavetable + SinOsc/LFTri ports + arena savepoint, host-tested"
```

---

### Task 6: Pulse (sine/cosecant BLIT port)

**Files:**
- Create: `rack-rp2350/src/dronecaster/sc_kit_pulse.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `KitTables`, `PhaseFrac`, `lininterp`, `kSineSize`, `kBadValue` (Task 5).
- Produces: `sc::Pulse` — `Init(const KitTables*, double sr, float freq, float width)`, `SetFreq(float)`, `SetWidth(float)`, `ProcessBlock(float*, int)`. The block prelude is mandatory: frequency/N changes select the crossfade branch exactly as `Pulse_next` does.

- [ ] **Step 1: Read `Pulse_next` in full**

```bash
wsl bash -lc 'cd /tmp/scsrc131 && sed -n "2510,2725p" OscUGens.cpp'
```

Pin: `N = int(sr*0.5/freq)`, `scale = 0.5/N`, `freq = (int32)(cpstoinc*freqin)` (or `max(prev,new)` when crossfading), `N2 = 2N+1`, `phaseoff_slope = (int32)((next_phaseoff - phaseoff) * mSlopeFactor)` with `mSlopeFactor = 1/64`, the `kBadValue`/`|denom| < 0.0005` fallbacks (`rscale = 1/scale + 1` non-crossfade, `1.f` crossfade), `pul1 = numer*denom` in the normal branch (not `numer/denom`), and the two different output lines: crossfade `y1 = pul1 - pul2 + 0.999*y1`, non-crossfade `y1 = (pul1-pul2)*scale + 0.999*y1`.

- [ ] **Step 2: Implement `sc_kit_pulse.h`**

```cpp
// sc_kit_pulse.h — exact port of SC 3.13.1 Pulse (server/plugins/OscUGens.cpp:
// 2510-2720, GPL-3.0). A BLIT-style pulse train: sine/cosecant lookup with a
// duty phase offset, an N-change crossfade, and a 0.999 leaky integrator.
// There are NO pulse wavetables in SC 3.13 (that model is from SC 2.x).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_PULSE_H_
#define RACK_SRC_DRONECASTER_SC_KIT_PULSE_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"
#include "sc_kit_osc.h"

namespace sc {

class Pulse {
 public:
  void Init(const KitTables* t, double sr, float freq, float width) {
    t_ = t;
    sr_ = sr;
    freq_ = freq;
    width_ = width;
    freqin_ = freq;                       // ctor: unit->m_freqin = ZIN0(0)
    cpstoinc_ = static_cast<double>(kSineSize) * (1.0 / sr) * 65536.0 * 0.5;
    n_ = static_cast<int32_t>((sr * 0.5) / static_cast<double>(freq));
    scale_ = 0.5f / static_cast<float>(n_);
    phase_ = 0;
    phaseoff_ = 0;
    y1_ = 0.0f;
  }
  void SetFreq(float f) { freq_ = f; }
  void SetWidth(float w) { width_ = w; }
  void ProcessBlock(float* out, int n);

 private:
  // One half of the pulse difference at `phase` (the source computes pul1 at
  // `phase` and pul2 at `phase + phaseoff` with identical arithmetic).
  float PulseAt(uint32_t phase, int32_t N2, float scale, bool crossfade,
                int32_t prevN2, float prevscale, float xfade) const {
    const float* tbl = t_->cosecant + ((phase >> 16) & (kSineSize - 1));
    float t0 = tbl[0];
    float t1 = tbl[1];
    const float pfrac = PhaseFrac(phase);
    if (t0 == kBadValue || t1 == kBadValue) {
      tbl = t_->sine + ((phase >> 16) & (kSineSize - 1));
      t0 = tbl[0];
      t1 = tbl[1];
      const float denom = t0 + (t1 - t0) * pfrac;
      if (std::fabs(denom) < 0.0005f) {
        return crossfade ? 1.0f : (1.0f / scale + 1.0f);
      }
      const uint32_t rphase = phase * static_cast<uint32_t>(N2);
      const float numer = SineAt(rphase);
      if (!crossfade) return numer / denom;
      const uint32_t rphase_prev = phase * static_cast<uint32_t>(prevN2);
      const float n1 = (SineAt(rphase_prev) / denom - 1.0f) * prevscale;
      const float n2 = (numer / denom - 1.0f) * scale;
      return lininterp(xfade, n1, n2);
    }
    const float denom = t0 + (t1 - t0) * pfrac;
    const uint32_t rphase = phase * static_cast<uint32_t>(N2);
    const float numer = SineAt(rphase);
    if (!crossfade) return numer * denom;
    const uint32_t rphase_prev = phase * static_cast<uint32_t>(prevN2);
    const float n1 = (SineAt(rphase_prev) * denom - 1.0f) * prevscale;
    const float n2 = (numer * denom - 1.0f) * scale;
    return lininterp(xfade, n1, n2);
  }

  // lininterp(PhaseFrac(rphase), sine[i], sine[i+1]) — the source's
  // numtbl lookup with the (rphase>>14)&0x7FFC char offset.
  float SineAt(uint32_t rphase) const {
    const float* tbl = t_->sine + ((rphase >> 16) & (kSineSize - 1));
    return lininterp(PhaseFrac(rphase), tbl[0], tbl[1]);
  }

  const KitTables* t_ = nullptr;
  double sr_ = 48000.0;
  double cpstoinc_ = 0.0;
  uint32_t phase_ = 0, phaseoff_ = 0;
  int32_t n_ = 0;
  float freqin_ = 0.0f, freq_ = 0.0f, width_ = 0.5f;
  float scale_ = 0.0f, y1_ = 0.0f;
};

inline void Pulse::ProcessBlock(float* out, int n) {
  const float freqin = freq_;
  const float duty = width_;
  uint32_t phase = phase_;
  float y1 = y1_;

  int32_t freq, N, prevN = 0;
  float scale, prevscale = 0.0f;
  bool crossfade;

  // Pulse_next prelude (OscUGens.cpp:2530-2565)
  if (freqin != freqin_) {
    N = static_cast<int32_t>((sr_ * 0.5) / static_cast<double>(freqin));
    if (N != n_) {
      const float maxfreqin = freqin > freqin_ ? freqin : freqin_;
      freq = static_cast<int32_t>(cpstoinc_ * static_cast<double>(maxfreqin));
      crossfade = true;
    } else {
      freq = static_cast<int32_t>(cpstoinc_ * static_cast<double>(freqin));
      crossfade = false;
    }
    prevN = n_;
    prevscale = scale_;
    n_ = N;
    scale_ = scale = 0.5f / static_cast<float>(N);
  } else {
    N = n_;
    freq = static_cast<int32_t>(cpstoinc_ * static_cast<double>(freqin));
    scale = scale_;
    crossfade = false;
  }
  freqin_ = freqin;

  const int32_t N2 = 2 * N + 1;
  const int32_t phaseoff_start = static_cast<int32_t>(phaseoff_);
  const int32_t next_phaseoff = static_cast<int32_t>(duty * (1L << 28));
  const int32_t phaseoff_slope =
      static_cast<int32_t>((next_phaseoff - phaseoff_start) * (1.0 / 64.0));
  phaseoff_ = static_cast<uint32_t>(next_phaseoff);
  uint32_t phaseoff = static_cast<uint32_t>(phaseoff_start);

  if (crossfade) {
    const int32_t prevN2 = 2 * prevN + 1;
    float xfade = 0.0f;
    for (int i = 0; i < n; ++i) {
      const float pul1 = PulseAt(phase, N2, scale, true, prevN2, prevscale, xfade);
      const float pul2 = PulseAt(phase + phaseoff, N2, scale, true, prevN2,
                                 prevscale, xfade);
      out[i] = y1 = pul1 - pul2 + 0.999f * y1;
      phase += static_cast<uint32_t>(freq);
      phaseoff += static_cast<uint32_t>(phaseoff_slope);
      xfade += 1.0f / 64.0f;
    }
  } else {
    for (int i = 0; i < n; ++i) {
      const float pul1 = PulseAt(phase, N2, scale, false, 0, 0.0f, 0.0f);
      const float pul2 = PulseAt(phase + phaseoff, N2, scale, false, 0, 0.0f, 0.0f);
      out[i] = y1 = (pul1 - pul2) * scale + 0.999f * y1;
      phase += static_cast<uint32_t>(freq);
      phaseoff += static_cast<uint32_t>(phaseoff_slope);
    }
  }

  phase_ = phase;
  y1_ = y1;
}

}  // namespace sc
#endif
```

Implementation notes: `PulseAt` is called with the pre-increment `phaseoff` and the post-update `phaseoff_` is stored separately, exactly like the source's local `phaseoff` vs `unit->m_phaseoff`. The non-crossfade tiny-denom fallback recomputes the source's `rscale = 1.f/scale + 1.f` inside `PulseAt` (same expression, same float bits, since `scale` is block-constant).

- [ ] **Step 3: Append Pulse tests**

Add the include at the top of `test_dronecaster.cc` (after the osc includes):

```cpp
#include "../src/dronecaster/sc_kit_pulse.h"
```

The first eight samples below are exact outputs of the port (48 kHz, 55 Hz, width 0.5). The 1-second statistics come from the same reference run.

```cpp
  printf("== Pulse (BLIT) ==\n");
  {
    static sc::KitTables t;
    sc::InitKitTables(&t);
    sc::Pulse p;
    p.Init(&t, 48000.0, 55.0f, 0.5f);
    float buf[64];
    p.ProcessBlock(buf, 8);
    CHECK(buf[0] == 0.0f, "pulse sample0 == 0");
    CHECK(fabsf(buf[1] - 0.0214609262f) < 1e-7f, "pulse sample1 == 0.021460926");
    CHECK(fabsf(buf[2] - 0.0401634201f) < 1e-7f, "pulse sample2 == 0.040163420");
    CHECK(fabsf(buf[3] - 0.0533128008f) < 1e-7f, "pulse sample3 == 0.053312801");
    CHECK(fabsf(buf[4] - 0.0614881404f) < 1e-7f, "pulse sample4 == 0.061488140");
    CHECK(fabsf(buf[5] - 0.0637175143f) < 1e-7f, "pulse sample5 == 0.063717514");
    CHECK(fabsf(buf[6] - 0.0623465441f) < 1e-7f, "pulse sample6 == 0.062346544");
    CHECK(fabsf(buf[7] - 0.0577567518f) < 1e-7f, "pulse sample7 == 0.057756752");

    sc::Pulse q;
    q.Init(&t, 48000.0, 55.0f, 0.5f);
    float pk = 0.0f, prev = 0.0f;
    int zc = 0;
    for (int b = 0; b < 750; ++b) {                 // 1 s
      q.ProcessBlock(buf, 64);
      for (int i = 0; i < 64; ++i) {
        const float v = buf[i];
        if (fabsf(v) > fabsf(pk)) pk = v;
        if ((v >= 0.0f) != (prev >= 0.0f)) ++zc;
        prev = v;
      }
    }
    CHECK(fabsf(pk) > 1.0f && fabsf(pk) < 1.2f, "pulse 1 s peak ~1.08");
    CHECK(zc >= 110 && zc <= 116, "pulse 1 s zero crossings ~113");

    // Frequency-change path (crossfade): finite and bounded.
    sc::Pulse r;
    r.Init(&t, 48000.0, 55.0f, 0.5f);
    bool finite = true;
    float qpk = 0.0f;
    for (int b = 0; b < 100; ++b) {
      r.SetFreq(b < 50 ? 55.0f : 110.0f);
      r.ProcessBlock(buf, 64);
      for (int i = 0; i < 64; ++i) {
        if (!std::isfinite(buf[i])) finite = false;
        if (fabsf(buf[i]) > qpk) qpk = fabsf(buf[i]);
      }
    }
    CHECK(finite && qpk < 2.5f, "pulse crossfade path finite/bounded");
  }
```

- [ ] **Step 4: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: all sections `ok`, exit 0. If a sample value differs, diff `PulseAt` against `Pulse_next` line by line — do not relax the tolerance.

- [ ] **Step 5: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_kit_pulse.h rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: SC Pulse BLIT port (sine/cosecant, crossfade, leaky integrator), host-tested"
```

---

### Task 7: EnvAsr (Env.asr curve -4 at kr) + SynthSocket engine + trio drones

**Files:**
- Modify: `rack-rp2350/src/dronecaster/sc_kit_env.h`
- Modify: `rack-rp2350/src/dronecaster/dronecaster_engine.h`
- Create: `rack-rp2350/src/dronecaster/drones/drone_trio.h`
- Modify: `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: Tasks 5/6 kit structs, `sc::Arena`, `sc::RGen`.
- Produces (consumed by Task 8 tests and Task 9's adapter):
  - `sc::EnvAsr` — `Init(sr, attack, release)`, `Gate(bool)`, `set_times(attack, release)`, `Next()` (one **control sample** per 64-sample block), `done()`.
  - `sc::DroneSpec` — `{ const char* name; void (*init)(Ctx&); void (*process)(Ctx&, float*, float*, int); }`.
  - `sc::Ctx` — `{ float sr; RGen* rgen; float hz, amp; KitTables* tables; Arena* arena; void* state; }`.
  - `sc::Engine` — `Init(float sr, uint8_t* buf_mem, size_t buf_bytes)`, `SetDrone(const DroneSpec*)`, `Recast()`, `set_hz_target(float)`, `set_amp(float)`, `set_fade(float)`, `current_name()`, `swap_count()`, `Process(float* l, float* r, int n)`.
  - `sc::kCatalogCount` (37) + `sc::Catalog(int)` (entries 22/23/29 wired, others nullptr in T-D1).

- [ ] **Step 1: Read `Env.asr` + `EnvGen` (no porting from memory)**

```bash
wsl bash -lc 'cd /tmp/scfull 2>/dev/null || true; cd /tmp/scsrc131 && \
  curl -sL https://raw.githubusercontent.com/supercollider/supercollider/Version-3.13.1/SCClassLibrary/Common/Audio/Env.sc -o Env.sc && \
  sed -n "195,210p" Env.sc && sed -n "352,362p" Env.sc && \
  sed -n "2471,2500p" LFUGens.cpp && sed -n "2608,2642p" LFUGens.cpp && sed -n "2660,2690p" LFUGens.cpp && sed -n "2798,2830p" LFUGens.cpp'
```

Pin: `*asr`'s `curve = -4.0`; numeric curve → shape 5; `check_gate`'s gate-off stage jump; the segment init (`a1/a2/b1/grow`, `counter = max(1, dur*SAMPLERATE)` where SAMPLERATE is the kr rate = sr/64); the ctor's priming call.

- [ ] **Step 2: Implement `sc_kit_env.h`**

```cpp
// sc_kit_env.h — EnvGen subset: Env.asr with SC's default curve -4 at
// CONTROL rate (EnvGen.kr computes ONE value per 64-sample block; the graph
// multiplies the whole block by it). Source: server/plugins/LFUGens.cpp
// (EnvGen_Ctor 2471, check_gate 2608, next_segment/curve 2660-2690,
// EnvGen_next_k 2798) and SCClassLibrary/Common/Audio/Env.sc:198.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_ENV_H_
#define RACK_SRC_DRONECASTER_SC_KIT_ENV_H_
#include <cmath>
#include <cstdint>

namespace sc {

class EnvAsr {
 public:
  void Init(float sr, float attack, float release) {
    sr_ = sr;
    attack_ = attack;
    release_ = release;
    level_ = 0.0;
    a2_ = b1_ = 0.0;
    grow_ = 1.0;
    counter_ = 0;
    stage_ = Stage::kIdle;
  }
  // Gate(true) mirrors EnvGen_Ctor: starts the attack AND consumes the
  // ctor's priming control sample. The first rendered block is therefore
  // the second attack value (SC's EnvGen_Ctor calls EnvGen_next_k(unit,1)).
  void Gate(bool on) {
    if (on) {
      if (stage_ == Stage::kIdle) {
        stage_ = Stage::kAttack;
        StartSegment(1.0, attack_);
        Step();
      }
    } else if (stage_ == Stage::kAttack || stage_ == Stage::kPlay) {
      stage_ = Stage::kRelease;
      StartSegment(0.0, release_);
    }
  }
  // setFadeTime: sets the attack/release ARGS. SC initializes a segment when
  // it begins, so live segments keep their rates; the next segment uses the
  // new value (the release is initialized at gate-off).
  void set_times(float attack, float release) {
    attack_ = attack;
    release_ = release;
  }
  // One CONTROL sample (call once per 64-sample block).
  float Next() {
    switch (stage_) {
      case Stage::kIdle: return 0.0f;
      case Stage::kPlay: return 1.0f;
      case Stage::kAttack:
        if (counter_ <= 0) { stage_ = Stage::kPlay; level_ = 1.0; return 1.0f; }
        break;
      case Stage::kRelease:
        if (counter_ <= 0) { stage_ = Stage::kIdle; level_ = 0.0; return 0.0f; }
        break;
    }
    const float out = static_cast<float>(level_);
    Step();
    return out;
  }
  bool done() const { return stage_ == Stage::kIdle; }

 private:
  enum class Stage { kIdle, kAttack, kPlay, kRelease };

  void StartSegment(double end_level, float dur) {
    int32_t c = static_cast<int32_t>(dur * static_cast<float>(sr_ / 64.0f));
    if (c < 1) c = 1;
    const double curve = -4.0;                          // Env.asr default
    const double a1 = (end_level - level_) / (1.0 - std::exp(curve));
    a2_ = level_ + a1;
    b1_ = a1;
    grow_ = std::exp(curve / static_cast<double>(c));
    counter_ = c;
  }
  void Step() {
    b1_ *= grow_;
    level_ = a2_ - b1_;
    --counter_;
  }

  float sr_ = 48000.0f;
  float attack_ = 1.0f, release_ = 1.0f;
  double a2_ = 0.0, b1_ = 0.0, grow_ = 1.0, level_ = 0.0;
  int32_t counter_ = 0;
  Stage stage_ = Stage::kIdle;
};

}  // namespace sc
#endif
```

- [ ] **Step 3: Implement `dronecaster_engine.h`**

```cpp
// dronecaster_engine.h — the DroneCaster_SynthSocket port (GPL-3.0 upstream:
// northern-information/dronecaster engine/Dronecaster_SynthSocket.sc).
// Mono voice; "we don't want xfades, we want down+up": SetDrone releases the
// current synth (gate->0); when its Env.asr finishes, the cued synth starts.
// hz glides via a one-pole per-block slew (~60 ms to 95% — the approved
// spec's "short lag" deviation from SC's stepped control bus).
//
// Rate law: the host calls Process(n <= 64) on the 16-sample grid; a fresh
// internal 64-sample block is rendered whenever the buffer drains (lira8 law).
// EnvGen.kr semantics: ONE env value per internal block.
#ifndef RACK_SRC_DRONECASTER_DRONECASTER_ENGINE_H_
#define RACK_SRC_DRONECASTER_DRONECASTER_ENGINE_H_
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "sc_defs.h"
#include "sc_rgen.h"
#include "sc_arena.h"
#include "sc_kit_osc.h"
#include "sc_kit_pulse.h"
#include "sc_kit_env.h"

namespace sc {

struct Ctx {
  float sr = 48000.0f;
  RGen* rgen = nullptr;
  float hz = 55.0f, amp = 0.4f;
  KitTables* tables = nullptr;
  Arena* arena = nullptr;
  void* state = nullptr;     // per-voice DSP state, arena-owned
};

struct DroneSpec {
  const char* name;
  void (*init)(Ctx&);
  void (*process)(Ctx&, float* l, float* r, int n);
};

constexpr int kCatalogCount = 37;

class Engine {
 public:
  void Init(float sr, uint8_t* buf_mem, size_t buf_bytes) {
    sr_ = sr;
    arena_.Init(buf_mem, buf_bytes);
    rgen_.init(kGoldenSeed);                 // matches RandSeed.kr(1, seed)
    kit_ = static_cast<KitTables*>(arena_.Alloc(sizeof(KitTables)));
    if (kit_ != nullptr) InitKitTables(kit_); // capacity is static_asserted
    tables_mark_ = arena_.Mark();            // kit tables survive voice swaps
    ctx_.sr = sr;
    ctx_.rgen = &rgen_;
    ctx_.tables = kit_;
    ctx_.arena = &arena_;
    ctx_.hz = 55.0f;
    ctx_.amp = 0.4f;
    hz_cur_ = hz_target_ = 55.0f;
    fade_ = 1.0f;
    env_.Init(sr, fade_, fade_);
    phase_ = Phase::kIdle;
    current_ = nullptr;
    cued_ = nullptr;
    internal_pos_ = 0;
    swaps_ = 0;
  }

  // down+up: release now; the cued voice starts when the env ends.
  // SetDrone(nullptr) = gate-off with no cue (the golden's gate path).
  void SetDrone(const DroneSpec* spec) {
    if (spec == nullptr) {
      if (phase_ == Phase::kPlaying) {
        env_.Gate(false);
        phase_ = Phase::kReleasing;
        cued_ = nullptr;
      }
      return;
    }
    if (spec == current_ && phase_ != Phase::kIdle) return;  // already selected
    cued_ = spec;
    if (phase_ == Phase::kPlaying) {
      env_.Gate(false);
      phase_ = Phase::kReleasing;
    } else if (phase_ == Phase::kIdle) {
      StartCued();
    }
  }
  void Recast() {                          // PLAY button: release + restart
    if (phase_ == Phase::kPlaying) {
      cued_ = current_;
      env_.Gate(false);
      phase_ = Phase::kReleasing;
    }
  }

  void set_hz_target(float hz) { hz_target_ = hz; }
  void set_amp(float a) { ctx_.amp = a; }
  void set_fade(float t) {
    fade_ = t > 0.001f ? t : 0.001f;
    env_.set_times(fade_, fade_);          // SC setFadeTime: args only
  }
  const char* current_name() const { return current_ ? current_->name : "--"; }
  uint32_t swap_count() const { return swaps_; }

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
    if (cued_ == nullptr) {
      current_ = nullptr;
      phase_ = Phase::kIdle;
      return;
    }
    arena_.ResetTo(tables_mark_);          // frees the old voice, keeps tables
    ctx_.state = nullptr;
    ctx_.hz = hz_cur_;
    cued_->init(ctx_);
    env_.Init(sr_, fade_, fade_);
    env_.Gate(true);
    current_ = cued_;
    cued_ = nullptr;
    phase_ = Phase::kPlaying;
    ++swaps_;
  }

  void RenderInternal() {
    // hz slew: one-pole per internal block (~60 ms to 95%).
    hz_cur_ += (hz_target_ - hz_cur_) * (64.0f / (0.02f * sr_));
    ctx_.hz = hz_cur_;
    if (phase_ == Phase::kReleasing && env_.done()) StartCued();
    if (phase_ == Phase::kPlaying || phase_ == Phase::kReleasing) {
      current_->process(ctx_, buf_l_, buf_r_, kBlock);
      // EnvGen.kr: one value per internal block (Out.ar(snd * aenv)).
      const float e = env_.Next();
      for (uint32_t i = 0; i < kBlock; ++i) {
        buf_l_[i] *= e;
        buf_r_[i] *= e;
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
  size_t tables_mark_ = 0;
  RGen rgen_;
  Ctx ctx_;
  EnvAsr env_;
  Phase phase_ = Phase::kIdle;
  const DroneSpec* current_ = nullptr;
  const DroneSpec* cued_ = nullptr;
  float hz_cur_ = 55.0f, hz_target_ = 55.0f, fade_ = 1.0f;
  uint32_t swaps_ = 0;
};

}  // namespace sc
#endif
```

- [ ] **Step 4: Create `drones/drone_trio.h`**

```cpp
// drone_trio.h — T-D1 drones: line-for-line C++ mirrors of the .scd graphs.
//   Sine.scd:     { arg hz, amp; (SinOsc.ar(hz) * amp).dup; }
//   Square.scd:   { arg hz, amp; (Pulse.ar(hz, 0.5) * amp).dup; }
//   Triangle.scd: { arg hz, amp; (LFTri.ar(hz) * amp).dup; }
// Voice state is arena-allocated in *_Init and persists across blocks
// (oscillator phase is the whole point of a drone).
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_TRIO_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_TRIO_H_
#include <cstddef>
#include "../sc_kit_osc.h"
#include "../sc_kit_pulse.h"
#include "../dronecaster_engine.h"

namespace sc {
namespace drones {

struct SineState { SinOsc osc; };
struct SquareState { Pulse osc; };
struct TriState { LFTri osc; };

inline void Sine_Init(Ctx& c) {
  SineState* st = static_cast<SineState*>(c.arena->Alloc(sizeof(SineState)));
  st->osc.Init(c.tables, static_cast<double>(c.sr), c.hz);
  c.state = st;
}
inline void Sine_Process(Ctx& c, float* l, float* r, int n) {
  SineState* st = static_cast<SineState*>(c.state);
  st->osc.SetFreq(c.hz);
  st->osc.ProcessBlock(l, n);
  for (int i = 0; i < n; ++i) { const float v = l[i] * c.amp; l[i] = v; r[i] = v; }
}

inline void Square_Init(Ctx& c) {
  SquareState* st = static_cast<SquareState*>(c.arena->Alloc(sizeof(SquareState)));
  st->osc.Init(c.tables, static_cast<double>(c.sr), c.hz, 0.5f);
  c.state = st;
}
inline void Square_Process(Ctx& c, float* l, float* r, int n) {
  SquareState* st = static_cast<SquareState*>(c.state);
  st->osc.SetFreq(c.hz);
  st->osc.ProcessBlock(l, n);
  for (int i = 0; i < n; ++i) { const float v = l[i] * c.amp; l[i] = v; r[i] = v; }
}

inline void Triangle_Init(Ctx& c) {
  TriState* st = static_cast<TriState*>(c.arena->Alloc(sizeof(TriState)));
  st->osc.Init(static_cast<double>(c.sr), c.hz);
  c.state = st;
}
inline void Triangle_Process(Ctx& c, float* l, float* r, int n) {
  TriState* st = static_cast<TriState*>(c.state);
  st->osc.SetFreq(c.hz);
  st->osc.ProcessBlock(l, n);
  for (int i = 0; i < n; ++i) { const float v = l[i] * c.amp; l[i] = v; r[i] = v; }
}

}  // namespace drones

inline const DroneSpec kDroneSine     = {"Sine",     drones::Sine_Init,     drones::Sine_Process};
inline const DroneSpec kDroneSquare   = {"Square",   drones::Square_Init,   drones::Square_Process};
inline const DroneSpec kDroneTriangle = {"Triangle", drones::Triangle_Init, drones::Triangle_Process};

// Catalog order = byte-wise sort of drones/*.scd (verified in Task 1).
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

If Task 1's sort output disagrees with the 22/23/29 positions, fix the pointer positions here AND the panel bytes in Task 10.

- [ ] **Step 5: Append EnvAsr + engine tests**

Add the includes at the top of `test_dronecaster.cc` (after the Pulse include):

```cpp
#include "../src/dronecaster/sc_kit_env.h"
#include "../src/dronecaster/dronecaster_engine.h"
#include "../src/dronecaster/drones/drone_trio.h"
```

```cpp
  printf("== EnvAsr (Env.asr curve -4, kr) ==\n");
  {
    sc::EnvAsr e;
    e.Init(48000.0f, 1.0f, 1.0f);
    e.Gate(true);
    CHECK(fabsf(e.Next() - 0.00541837746f) < 1e-7f, "env block0 == 0.005418377");
    for (int i = 1; i < 375; ++i) (void)e.Next();
    CHECK(fabsf(e.Next() - 0.881530404f) < 1e-6f, "env 0.5 s == 0.881530404");
    for (int i = 376; i < 750; ++i) (void)e.Next();
    CHECK(e.Next() == 1.0f, "env sustain == 1");
    e.Gate(false);
    CHECK(e.Next() == 1.0f, "release first block == 1");
    CHECK(fabsf(e.Next() - 0.994581640f) < 1e-6f, "release second == 0.99458164");
    for (int i = 2; i < 748; ++i) (void)e.Next();
    CHECK(fabsf(e.Next() - 0.000200077018f) < 1e-7f, "release block749");
    CHECK(fabsf(e.Next() - 9.97717434e-05f) < 1e-8f, "release block750");
    CHECK(e.Next() == 0.0f, "release end == 0");
    CHECK(e.done(), "env done");
  }

  printf("== socket engine ==\n");
  {
    static uint8_t mem[160 * 1024];           // tables 131,080 B + voice
    sc::Engine e;
    e.Init(48000.0f, mem, sizeof(mem));
    CHECK(strcmp(e.current_name(), "--") == 0, "idle: no current drone");

    e.SetDrone(sc::Catalog(22));              // Sine
    float l[16], r[16];
    e.Process(l, r, 16);
    CHECK(strcmp(e.current_name(), "Sine") == 0, "Sine starts immediately when idle");
    float pk = 0;
    for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    CHECK(pk < 0.01f, "attack ramp: near-silence at t=0");

    // 1.5 s in (block 4500): attack done (env 1.0). Take the max over
    // 100 blocks (~18 periods at 55 Hz) so a zero crossing cannot mask it.
    for (int b = 0; b < 4500; ++b) e.Process(l, r, 16);
    pk = 0;
    for (int b = 0; b < 100; ++b) {
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    }
    CHECK(pk > 0.38f && pk < 0.42f, "sustain: peak ~amp 0.4");

    // down+up: gate off, quick fade -> silence gap -> idle, name cleared.
    // Skip the release tail (0.01 s = ~7 internal blocks) before measuring
    // the gap: the first release block legitimately outputs the full level.
    e.set_fade(0.01f);
    e.SetDrone(nullptr);
    for (int b = 0; b < 100; ++b) e.Process(l, r, 16);   // ~25 internal blocks
    float gap_pk = 0;
    for (int b = 0; b < 300; ++b) {
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) gap_pk = gap_pk > fabsf(l[i]) ? gap_pk : fabsf(l[i]);
    }
    CHECK(strcmp(e.current_name(), "--") == 0, "gate-off: env ended, idle again");
    CHECK(gap_pk < 1e-6f, "down+up: silence in the gap");

    // recast: PLAY releases + restarts the current drone. With fade 1 s the
    // release takes 3000 host blocks; measure the restart after it.
    e.set_fade(1.0f);
    e.SetDrone(sc::Catalog(29));              // Triangle
    e.Process(l, r, 16);
    e.Recast();
    pk = 0;
    for (int b = 0; b < 3100; ++b) {
      e.Process(l, r, 16);
      if (b >= 3000) {
        for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
      }
    }
    CHECK(strcmp(e.current_name(), "Triangle") == 0, "recast: same drone restarted");
    CHECK(pk > 0.01f, "recast: restarted voice sounding");
    CHECK(e.swap_count() >= 3, "swap counter advances");

    // SetDrone(nullptr) after a completed recast must clear the name.
    e.set_fade(0.01f);
    e.SetDrone(nullptr);
    for (int b = 0; b < 400; ++b) e.Process(l, r, 16);
    CHECK(strcmp(e.current_name(), "--") == 0, "second gate-off clears the name");
  }
```

Note: `mem[160*1024]` is larger than the adapter's 136 KiB carve to keep this test independent of the adapter constant; the tables are 131,080 B.

- [ ] **Step 6: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: all sections `ok`, exit 0. The env block values must match exactly (same glibc `exp` as the golden renderer); if they differ by >1e-7, the segment/counter law is wrong — re-read `EnvGen`.

- [ ] **Step 7: Commit**

```bash
git add rack-rp2350/src/dronecaster/sc_kit_env.h rack-rp2350/src/dronecaster/dronecaster_engine.h \
        rack-rp2350/src/dronecaster/drones/drone_trio.h rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: Env.asr curve-4 kr port + SynthSocket engine (down+up, hz slew) + trio drones"
```

---

### Task 8: Trio goldens pass (the exactness gate)

**Files:**
- Modify: `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: Task 2's float32 goldens, Task 7's `Engine`.
- Produces: `render_drone_golden(int catalog_id, const char* got_path)` — every future drone port reuses it unchanged.

- [ ] **Step 1: Append the golden harness**

```cpp
  printf("== trio goldens (float32) ==\n");
  {
    // Schedule mirrors gen_goldens.scd EXACTLY: voice before the first
    // Process call (SC's \s_new is in block 0), gate off at block 2250
    // (sample 144000), 192000 frames, hz 55, amp 0.4, fade 1.0 s.
    struct GoldenCase { int id; const char* name; };
    const GoldenCase cases[] = {{22, "Sine"}, {23, "Square"}, {29, "Triangle"}};
    for (const GoldenCase& gc : cases) {
      static uint8_t mem[160 * 1024];
      sc::Engine e;
      e.Init(48000.0f, mem, sizeof(mem));
      e.set_amp(0.4f);
      e.set_fade(1.0f);
      e.set_hz_target(55.0f);
      e.SetDrone(sc::Catalog(gc.id));         // SC: the synth exists at t = 0

      const int total = 192000;
      const int gate_off = 144000;
      std::vector<float> pcm;
      pcm.reserve(static_cast<size_t>(2 * total));
      float l[16], r[16];
      int written = 0;
      while (written < total) {
        if (written == gate_off) e.SetDrone(nullptr);
        e.Process(l, r, 16);
        for (int i = 0; i < 16 && written < total; ++i, ++written) {
          pcm.push_back(l[i]);
          pcm.push_back(r[i]);
        }
      }
      char path[128];
      snprintf(path, sizeof(path), "build/dc_%s_got.wav", gc.name);
      l8test::WriteWavF32(path, pcm.data(), total, 2, 48000);
      char ref[128];
      snprintf(ref, sizeof(ref), "dronecaster_ref/goldens/%s.expected.wav", gc.name);
      GoldenReport rep;
      const bool ok = compare(ref, path, 1e-6, 0.999999, &rep);
      printf("  %s: %s\n", gc.name, ok ? "GOLDEN PASS" : "GOLDEN FAIL");
      if (!ok) ++g_failures;
    }
  }
```

- [ ] **Step 2: Run (WSL)**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster'
```

Expected: `Sine: GOLDEN PASS`, `Square: GOLDEN PASS`, `Triangle: GOLDEN PASS`, exit 0, `max ≤ 1e-6`, `corr ≥ 0.999999`. If a golden fails: the report's `worst_i` points at the sample; dump 32 samples around it from both files and diff the corresponding UGen's cited source lines. **Do not relax the gate** — a 1e-6 failure is a port bug (likely phase accumulation, the env block value, or the Pulse prelude), not noise.

- [ ] **Step 3: Full suite green**

```bash
wsl bash -lc 'cd /mnt/c/Users/schra/Developer/mi-rp2350/rack-rp2350/test && make test'
```

Expected: every TEST_BINS binary exits 0.

- [ ] **Step 4: Commit**

```bash
git add rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: trio bit-exact vs scsynth float32 goldens (Sine/Square/Triangle)"
```

---

### Task 9: Firmware integration — adapter, window, linker, wrapper, presets, C3 UI

**Files:**
- Create: `rack-rp2350/src/dronecaster/dronecaster_tc.h`
- Create: `rack-rp2350/src/module_dronecaster.h`
- Create: `rack-rp2350/lib/dronecaster_engine/library.json`
- Create: `rack-rp2350/lib/dronecaster_engine/dronecaster_engine_o3.h`
- Create: `rack-rp2350/lib/dronecaster_engine/dronecaster_engine_o3.cpp`
- Modify: `rack-rp2350/src/rack_window.h` (enum + symbols + section table)
- Modify: `rack-rp2350/memmap_rack.ld` (OVERLAY member, LMA symbols, ASSERT, `__rack_flash_end`)
- Modify: `rack-rp2350/src/main.cpp` (include, boot guard, registry entry, 2 s report)
- Modify: `rack-rp2350/src/preset.h` (version + blob cap)
- Modify: `rack-rp2350/src/ui_link.h` (P1 page table + registry row + stale comments)
- Modify: `docs/ui-protocol.md` (TELEM module id range 0-5)
- Modify: `rack-rp2350/src/module_registry.h` (comment only)
- Modify: `rack-uimcu/main/RackLink.cpp` (module id 5, 6 slots)
- Modify: `rack-uimcu/main/PresetBank.hpp` (kMaxRp)
- Modify: `rack-uimcu/ui/RackUI.cpp` (module name/count)

**Interfaces:**
- Consumes: `sc::Engine` (Task 7), `rack_mem_reset/alloc`, the registry hook contract, the RP2350 UI page-table contract.
- Produces: `rack_modules::mod_dronecaster::{init, render, note_on, note_off, control_change, mod_set_cc_float, mod_trigger_cc, state_size, buf_bytes, report_dc}` and a `DRONECASTER` entry in the C3 UI module list.

- [ ] **Step 1: `dronecaster_tc.h`**

```cpp
// Rack RP2350 — Dronecaster hot-code section placement (SRAM law).
// Only the -O3 wrapper (lib/dronecaster_engine) carries the token: at -O3
// the engine inlines into it, and a section attribute on an inlined callee
// does not survive. One unique token per emitted function.
#ifndef RACK_SRC_DRONECASTER_DRONECASTER_TC_H_
#define RACK_SRC_DRONECASTER_DRONECASTER_TC_H_
#if defined(RACK_RP2350_TC)
#define DRONECASTER_TC(token) __attribute__((section(".tc_dronecaster." #token)))
#else
#define DRONECASTER_TC(token)
#endif
#endif
```

- [ ] **Step 2: `dronecaster_engine_o3.h` / `.cpp` / `library.json`**

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
// -O3 definition (and the SRAM token) lives in the .cpp.
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
// Dronecaster engine hot entry point — compiled at -O3 by this library's
// library.json. Mirrors lib/lira8_engine exactly: the whole inlined engine
// body lands in the .tc_dronecaster section (SRAM law). The include is
// RELATIVE (the lib must not depend on a src/ include root), matching the
// lira8 wrapper.
#include "../../src/dronecaster/dronecaster_engine.h"
#include "../../src/dronecaster/dronecaster_tc.h"
#include "dronecaster_engine_o3.h"

namespace sc {

DRONECASTER_TC(process_dronecaster_o3)
void process_block_o3(Engine& e, float* out_l, float* out_r, int n) {
  e.Process(out_l, out_r, n);
}

}  // namespace sc
```

- [ ] **Step 3: `module_dronecaster.h`**

```cpp
// Rack RP2350 — Dronecaster adapter (T-D1, 2026-11-19).
//
// Module id 5: hand-ported norns Dronecaster engine (src/dronecaster/, GPL-3.0
// upstream) following the LIRA-8 adapter shape. Standalone 48 kHz, host grid
// 16, engine bursts 4x16 -> 64. Spec:
// docs/superpowers/specs/2026-11-19-dronecaster-port-design.md.
//
// AUDIO CONTRACT: identical to lira8's — Process() serves from an internal
// 64-sample buffer; the 8x16 I2S bank (~2.7 ms at 48 kHz) absorbs the burst.
// CC map (spec): 51 drone(0-36) / 52 amp / 53 fade / 54 hz-offset;
// notes = re-hz (note-off ignored); PLAY 64 = recast.
#pragma once
#include <cmath>
#include <cstring>
#include <cstdio>
#include <new>

#include "dronecaster/dronecaster_engine.h"
#include "dronecaster/drones/drone_trio.h"
#include "dronecaster/dronecaster_tc.h"
#include "dronecaster_engine_o3.h"

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

// Voice carve: KitTables = 131,080 B (sine wavetable + gSine + gInvSine),
// resident above an arena savepoint; per-voice state sits above it.
static constexpr size_t kVoiceBytes = 136 * 1024;
static_assert(sizeof(sc::KitTables) <= kVoiceBytes,
              "kit tables exceed the dronecaster voice carve");
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
  // core-0 writes -> core-1 pending slots (block-aligned intents)
  volatile int16_t pending_drone;       // -1 = none
  volatile bool pending_recast;
  // report mirrors (core 1 -> core 0)
  volatile uint32_t proc_avg_us;
  volatile uint32_t nan_blocks;         // blocks zeroed by the NaN guard
  // core-0 pitch state (CC 54 is an offset on top of the current note)
  float note_hz;                        // last note-on pitch (default 55)
  float offset_st;                      // CC 54 offset in semitones
  int16_t last_bad_drone;
  explicit State(uint8_t* buf, size_t bytes)
      : pending_drone(22), pending_recast(false), proc_avg_us(0), nan_blocks(0),
        note_hz(55.0f), offset_st(0.0f), last_bad_drone(-1) {
    eng.Init(static_cast<float>(kSampleRate), buf, bytes);
  }
};

static State* s = nullptr;
static size_t state_size() { return sizeof(State); }
static constexpr size_t buf_bytes() { return kVoiceBytes; }

void init_at(uint8_t* obj_base, uint8_t* buf_base) {
  memset(buf_base, 0, kVoiceBytes);
  s = new (obj_base) State(buf_base, kVoiceBytes);
}
void init() {
  rack_mem_reset();
  init_at(static_cast<uint8_t*>(rack_mem_alloc(state_size(), 8)),
          static_cast<uint8_t*>(rack_mem_alloc(buf_bytes(), 8)));
}

// CC 54 is an OFFSET on top of the current note pitch (spec): base hz is the
// last note (55 until a note arrives), target = base * 2^(offset/12).
static void apply_pitch() {
  s->eng.set_hz_target(s->note_hz * powf(2.0f, s->offset_st / 12.0f));
}
void note_on(uint8_t note, uint8_t vel) {
  (void)vel;
  s->note_hz = sc::midicps(static_cast<float>(note));
  apply_pitch();
}
void note_off(uint8_t note) { (void)note; }   // drones do not stop

void control_change(uint8_t cc, uint8_t val) {
  if (cc == 64) { if (val >= 64) s->pending_recast = true; return; }
  if (cc == 51) {
    if (sc::Catalog(val) != nullptr) {
      s->pending_drone = static_cast<int16_t>(val);
    } else if (static_cast<int16_t>(val) != s->last_bad_drone) {
      s->last_bad_drone = static_cast<int16_t>(val);
      Serial.printf("dc: drone id %u not ported (keep current)\r\n",
                    static_cast<unsigned>(val));
    }
    return;
  }
  if (cc == 52) { s->eng.set_amp(static_cast<float>(val) / 127.0f); return; }
  if (cc == 53) { const float v = static_cast<float>(val) / 127.0f;
                  s->eng.set_fade(16.0f * v * v); return; }             // 0..16 s
  if (cc == 54) { s->offset_st = (static_cast<float>(val) - 64.0f) * (24.0f / 64.0f);
                  apply_pitch(); return; }
}

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  if (cc == 52) { s->eng.set_amp(v); return true; }
  if (cc == 53) { s->eng.set_fade(16.0f * v * v); return true; }
  if (cc == 54) { s->offset_st = (v * 2.0f - 1.0f) * 24.0f; apply_pitch(); return true; }
  return false;
}
bool mod_trigger_cc(uint8_t cc, bool high) { (void)cc; (void)high; return false; }

void render() {
  // Pending intents land at the block boundary (the lira8 discipline). The
  // three float params (amp/fade/hz) are single-word atomic stores written
  // by core 0 and read by core 1 — safe by construction, applied next block.
  if (s->pending_drone >= 0) {
    const int id = s->pending_drone;
    s->pending_drone = -1;
    const sc::DroneSpec* spec = sc::Catalog(id);
    if (spec != nullptr) s->eng.SetDrone(spec);  // unported id: ignored
  }
  if (s->pending_recast) { s->pending_recast = false; s->eng.Recast(); }

  uint32_t t0 = time_us_32();
  float l[kBlockSize], r[kBlockSize];
  sc::process_block_o3(s->eng, l, r, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  s->proc_avg_us = (s->proc_avg_us * 15 + dt) / 16;   // ewma for the report

  // Per-block NaN/inf sweep (spec §Error handling): clamp to silence and
  // count; the 2 s report prints the count (no printing from the audio path).
  bool bad = false;
  for (size_t i = 0; i < kBlockSize; ++i) {
    if (!std::isfinite(l[i])) { bad = true; break; }
  }
  if (bad) {
    ++s->nan_blocks;
    for (size_t i = 0; i < kBlockSize; ++i) l[i] = 0.0f;
  }

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

// 2 s report extra field (integer-only — no %f under newlib-nano).
const char* report_dc(char* buf, size_t n) {
  snprintf(buf, n, "%s avg=%uu sw=%lu nan=%lu", s->eng.current_name(),
           static_cast<unsigned>(s->proc_avg_us),
           static_cast<unsigned long>(s->eng.swap_count()),
           static_cast<unsigned long>(s->nan_blocks));
  return buf;
}

}  // namespace mod_dronecaster
}  // namespace rack_modules
```

(`proc_us_max`, `rack_meter_a`, `g_render_gain`, `i2s_stall_count`, `i2s.write16` are main.cpp's globals, visible exactly as in `module_lira8.h`.)

- [ ] **Step 4: `rack_window.h`**

```cpp
enum RackModuleId {
  RACK_MODULE_ELEMENTS = 0,
  RACK_MODULE_CLOUDS   = 1,
  RACK_MODULE_PLAITS   = 2,
  RACK_MODULE_RINGS    = 3,
  RACK_MODULE_LIRA8    = 4,
  RACK_MODULE_DRONECASTER = 5,
  RACK_MODULE_COUNT    = 6,
};
```

Add the externs and the table row:

```cpp
extern uint8_t __tc_dronecaster_start[], __tc_dronecaster_end[];
extern uint8_t __rack_flash_dronecaster_start[], __rack_flash_dronecaster_end[];
```

```cpp
      {"dronecaster", __tc_dronecaster_start, __tc_dronecaster_end,
                      __rack_flash_dronecaster_start, __rack_flash_dronecaster_end},
```

- [ ] **Step 5: `memmap_rack.ld`**

Add the OVERLAY member after `tc_lira8`:

```
        tc_dronecaster
        {
            __tc_dronecaster_start = .;
            KEEP(*(.tc_dronecaster.*))
            . = ALIGN(4);
            __tc_dronecaster_end = .;
        }
```

Add the LMA symbols after the lira8 pair:

```
    __rack_flash_dronecaster_start = LOADADDR(tc_dronecaster);
    __rack_flash_dronecaster_end   = LOADADDR(tc_dronecaster) + SIZEOF(tc_dronecaster);
```

**Move the end marker** (line ~440):

```
    __rack_flash_end = __rack_flash_dronecaster_end;
```

Add the window ASSERT after the lira8 one:

```
    ASSERT(SIZEOF(tc_dronecaster) <= LENGTH(RACK_WINDOW),
        "ERROR: dronecaster hot section exceeds the rack SRAM window")
```

(The OVERLAY has a single `AT(ORIGIN(RACK_LMA) + SIZEOF(tc_clouds))`; overlay members take sequential implicit LMAs — there is no per-member AT to add.)

- [ ] **Step 6: `main.cpp`** (five edits)

1. Comment at line ~15: `0 elements / 1 clouds / 2 plaits / 3 rings / 4 lira8 / 5 dronecaster`.
2. Boot guard at line ~165:

```cpp
#if !defined(RACK_BOOT_MODULE) || RACK_BOOT_MODULE < 0 || RACK_BOOT_MODULE > 5
#error "RACK_BOOT_MODULE must be defined in the env as 0 (elements), 1 (clouds), 2 (plaits), 3 (rings), 4 (lira8) or 5 (dronecaster)"
#endif
```

3. Include beside the other adapters (after `#include "module_lira8.h"`):

```cpp
#include "module_dronecaster.h"
```

4. Append the 6th `kRackModules` entry (mirror the lira8 entry):

```cpp
    {"dronecaster", RACK_MODULE_DRONECASTER,
     rack_modules::mod_dronecaster::kSampleRate,
     (uint32_t)rack_modules::mod_dronecaster::kBlockSize,
     rack_modules::mod_dronecaster::kBudgetUs,
     rack_modules::mod_dronecaster::kI2sBuffers,
     rack_modules::mod_dronecaster::kI2sWords,
     rack_modules::mod_dronecaster::kStallUs,
     rack_modules::mod_dronecaster::kCcMap,
     &rack_modules::mod_dronecaster::kModDestinations,
     rack_modules::mod_dronecaster::init,
     rack_modules::mod_dronecaster::render,
     rack_modules::mod_dronecaster::note_on,
     rack_modules::mod_dronecaster::note_off,
     rack_modules::mod_dronecaster::control_change},
```

Also add the sixth entry to `kChainABufferBytes` (line ~482) — dronecaster is standalone-only and never a chain A slot:

```cpp
static const size_t kChainABufferBytes[RACK_MODULE_COUNT] = {
    65536, 0, 32768, 65536,
    128016,
    0,        // dronecaster: standalone-only (not a chain A candidate)
};
```

5. 2 s report — add the tail beside `lira8_tail` and fold it into both `dbg_printf` variants with the same slot:

```cpp
      char dc_tail[80] = "";
      if (g_active_module == RACK_MODULE_DRONECASTER) {
        rack_modules::mod_dronecaster::report_dc(dc_tail, sizeof(dc_tail));
      }
```

In the `#else` variant, extend the ternary:

```cpp
                    g_active_module == RACK_MODULE_PLAITS ? plaits_tail
                        : (g_active_module == RACK_MODULE_LIRA8 ? lira8_tail
                        : (g_active_module == RACK_MODULE_DRONECASTER ? dc_tail : "")),
```

In the `#if RACK_ENABLE_USB_HOST` variant, replace the `plaits_tail` argument with:

```cpp
                    g_active_module == RACK_MODULE_PLAITS ? plaits_tail
                        : (g_active_module == RACK_MODULE_DRONECASTER ? dc_tail : ""),
```

(no format-string change needed — the slot is empty for all other modules).

- [ ] **Step 7: `preset.h`**

```cpp
static const uint8_t kPresetVersion = 5;   // v5: 6 modules (dronecaster id 5)
```

The layout comments and derived offsets must read 6 modules: `cc_seen[6][16]` (96 B), `cc_cache[6][128]` (768 B), `kPresetSize` 1215 B. Raise the cap:

```cpp
static const uint16_t kPresetBlobMax = 1216;   // v5 6-module blob = 1215 B
```

- [ ] **Step 8: C3 firmware edits**

`rack-uimcu/main/PresetBank.hpp`:

```cpp
            static constexpr size_t   kMaxRp = 1216;  // RP2350 part cap (v5 6-module blob = 1215 B)
```

`rack-uimcu/main/RackLink.cpp`:

```cpp
        bool RackLink::SendSwitch(uint8_t module_id) {
            if (module_id > 5) return false;   // 0-5: elements/clouds/plaits/rings/lira8/dronecaster
```

```cpp
        void RackLink::TelemLocalSysSwitch(uint8_t newMod) {
            // 6 module slots (elements/clouds/plaits/rings/lira8/dronecaster)
            // — must match kModNames/kModuleCount in RackUI.cpp.
            static constexpr uint8_t kModuleSlots = 6;
```

(Leave `SendChainRoute`'s `a > 4` cap: dronecaster is standalone-only and cannot be a chain A slot.)

`rack-uimcu/ui/RackUI.cpp`:

```cpp
        static const char *kModNames[6] = { "ELEMENTS", "CLOUDS", "PLAITS", "RINGS", "LIRA8", "DRONECSTR" };
        static const uint8_t kModuleCount = 6;
```

(Use a 10-character label — the C3's digi-slim font is 6 px; `"DRONECSTR"` is the established abbreviation style.)

- [ ] **Step 9: `ui_link.h`** (RP2350-side page table + stale comments)

Add the P1 table beside the other module tables:

```cpp
// dronecaster — P1: the four voice controls (CC 51-54).
static const UiPageCell kUiPageDronecasterP1[] = {
    {51, "DRONE",  kUiColorWhite},
    {52, "AMP",    kUiColorWhite},
    {53, "FADE",   kUiColorWhite},
    {54, "HZ OFF", kUiColorWhite},
};
```

Add the registry row:

```cpp
    { {kUiPageDronecasterP1, UI_PAGE_CELL_COUNT(kUiPageDronecasterP1)},
      {nullptr, 0},        {nullptr, 0} },                               // dronecaster
```

Update the three stale comments in `ui_link.h`: the SYS selector's "over 5 slots" → 6; the selector-law comment "(5 modules: 0/31/63/95/127)" → "(6 modules: 0/25/51/76/102/127)"; and the `g_ui_cc_cache` comment "5 modules x 128 CCs = 640 B" → 6 modules = 768 B. The selector arithmetic itself is generic (`RACK_MODULE_COUNT`).

Also update `docs/ui-protocol.md` (~line 130): the TELEM `module` field is the active module id **0-5** (add dronecaster to the list).

- [ ] **Step 10: Build (Windows pio)**

```bash
cd /c/Users/schra/Developer/mi-rp2350 && pio run -d rack-rp2350
```

Expected: `Project build complete`; verify no `error:` and grep the size/map output for the `tc_dronecaster` slot size (note the number in the commit message). Do **not** flash yet — Task 11 does that after the panel. Then build the C3 (Windows, MSYSTEM/idf.py trap — use the bat):

```bash
cd /c/Users/schra/Developer/mi-rp2350 && tools/build_uimcu.bat
```

Expected: the C3 firmware builds (the RP2350 side is unaffected by the C3 build).

- [ ] **Step 11: Commit**

```bash
git add rack-rp2350/src/dronecaster/dronecaster_tc.h rack-rp2350/src/module_dronecaster.h \
        rack-rp2350/lib/dronecaster_engine/ rack-rp2350/src/rack_window.h \
        rack-rp2350/memmap_rack.ld rack-rp2350/src/main.cpp rack-rp2350/src/preset.h \
        rack-rp2350/src/module_registry.h rack-rp2350/src/ui_link.h \
        docs/ui-protocol.md \
        rack-uimcu/main/RackLink.cpp rack-uimcu/main/PresetBank.hpp rack-uimcu/ui/RackUI.cpp
git commit -m "[Add] rack: dronecaster module id 5 (adapter/window/linker/wrapper/preset/C3 UI) — build green"
```

---

### Task 10: Panel tab

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
    { cc:51, label:'DRONE', steps: { labels:['SINE','SQUARE','TRIANGLE'], bytes:[22,23,29] }, def: 0 },
    { cc:52, label:'AMP', variant:'white', def: 51/127 },
    { cc:53, label:'FADE', variant:'white', def: 32/127 },
    { cc:54, label:'HZ\u2009OFF', variant:'white', def:.5,
      fmt: c => { const st = (b127(c.v) - 64) * 24 / 64; return (st >= 0 ? '+' : '') + st.toFixed(1); } },
  ],
},
```

`def: 0` is the **step index** for a `steps` knob (the panel sends `steps.bytes[round(v*(n-1))]`, rack_panel.html:575); `def: 22/127` would sit between steps at 17% travel.

- [ ] **Step 2: Add the mod-destination table**

```javascript
dronecaster: { cont: [52,53,54], trig: [] },
```

- [ ] **Step 3: Verify the tab loads**

Open `tools/rack_panel.html` in Chrome/Edge with the device connected; confirm the DRONECASTER tab renders, the DRONE selector shows three labeled steps (SINE at rest), and switching sends CC 51 bytes 22/23/29 (visible via the panel's activity log or `aseqdump` on the host side).

- [ ] **Step 4: Commit**

```bash
git add tools/rack_panel.html
git commit -m "[Add] panel: dronecaster tab (CC 51 labeled drone selector, amp/fade/hz)"
```

---

### Task 11: Flash, hardware acceptance, docs, ledger

**Files:**
- Modify: `AGENTS.md` (module count/list), `rack-rp2350/README.md` (module table + CC map), `.superpowers/sdd/progress.md` (ledger entry)

- [ ] **Step 1: Flash the RP2350 via the documented BOOTSEL flow** (AGENTS.md §RP2350 flash — 1200-baud touch → RP2350 drive → `firmware.uf2`; resolve the port by `/dev/serial/by-id/`, never ttyACM number). The Task 9 build is already verified — flash it unchanged.

- [ ] **Step 2: Flash the C3 (user flow)** — `cd rack-uimcu && idf.py -p <C3-ACM> flash` from the WSL ESP-IDF env, or the user's usual flow. The C3 image must be the Task 9 build (module 5 in `kModNames`), otherwise the hardware UI cannot select DRONECASTER.

- [ ] **Step 3: Console smoke** (WSL, by-id port, 115200): send `\n` then `status`; expect the boot log, no stall growth, and (after selecting the module) `mod=dronecaster` with a `dc=... avg=.. sw=.. nan=0` field. Send CC 51 = 23/29/22 from the panel or C3; play notes and hear the drone glide; PLAY = recast. Verify a not-yet-ported CC 51 id prints `dc: drone id N not ported` once and keeps the current drone.

- [ ] **Step 4: User acceptance** — the user judges the trio by ear (level, character, note tracking). Record their verdict verbatim in the ledger. Do NOT claim hardware-verified beyond what they confirm.

- [ ] **Step 5: Docs + ledger update** — AGENTS.md "five modules" → six (module list + boot comment); `rack-rp2350/README.md` module list + CC map + the SRAM pool line (dronecaster carve 136 KiB; kit tables 131,080 B); ledger entry: status, commit hash, findings (SC 3.13 taus88/BLIT/curve-4 facts, catalog order, float32 golden gates, `tc_dronecaster` slot size, proc avg observed, C3 changes).

- [ ] **Step 6: Commit + push**

```bash
git add AGENTS.md rack-rp2350/README.md .superpowers/sdd/progress.md
git commit -m "[Docs] dronecaster T-D1 landed: module id 5, trio float-golden-exact — ledger + docs"
git push
```

---

## Self-Review notes (rev 2)

- **Spec coverage:** T-D1's spec row (kit core, goldens infra, trio bit-exact, end-to-end module, first HW flash) maps to Tasks 1-11. The kit-core narrowing (no noise/delay/logic/spatial) is declared in the header; T-D2 pulls them per-drone.
- **RGen corrected:** taus88 + Hash seeding + the real `frand0/frand2/frand/frand8/fcoin` ranges; test vectors are exact port outputs. The spec's "LCG constants" line is fixed in the spec.
- **Pulse corrected:** the BLIT model (sine/cosecant, N, duty ramp, crossfade, leaky integrator) replaces the fictional wavetable model; the test vectors pin the first 8 samples.
- **Envelope corrected:** Env.asr curve -4 + EnvGen.kr block stepping + the ctor prime; the engine multiplies each internal block by one env value.
- **Golden alignment:** voice at sample 0 (the C++ engine starts before its first `Process`; scsynth's `\s_new` is in block 0), gate-off at 144000 (block 2250), float32 comparison with a 1e-6 gate.
- **Golden renderer:** synchronous (`writeOSCFile` + `systemCmd`), correct positional semantics, deterministic via RandSeed; no async race.
- **SRAM law:** only the `-O3` wrapper carries the section token; tables live above the arena savepoint; 131,080 B tables + 136 KiB carve with static_asserts.
- **Integration:** window enum/table, linker member + LMA symbols + ASSERT + `__rack_flash_end`, preset version/cap, C3 caps/names/selector, RP2350 `ui_link` page table, panel step index.
- **Observability:** unknown-id console note (core 0), NaN block counter in the report, swap counter; integer-only formatting.
- **Open decision recorded:** selecting the module boots the Sine default (`pending_drone = 22`), like the other adapters' factory defaults; a preset replay overrides it if CC 51 was ever seen.
