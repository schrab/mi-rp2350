So send~/receive~ is a plain buffer copy with NO inherent delay — the delay depends purely on DSP sort order. Pd's DSP scheduler sorts objects topologically; send~ and receive~ are NOT linked as dependencies in the graph (receive~ has no signal inlet), so Pd's sort places them... Pd's ugen sorting: objects with no signal inputs get sorted early (they're "roots"). receive~ has no signal inlets → it's a root → gets sorted FIRST (before send~). Therefore receive~ reads the PREVIOUS block's data → one block delay. That's the well-known behavior.

Actually Pd's d_ugen.c sorting: it does a topological sort where objects are visited from outlets... Pd's `ugen_doit`/`ugen_getsortno`. The sort starts from objects that are "sinks" (dac~, delwrite~?) Hmm. Pd sorts by starting at objects with no outlets? Let me recall: Pd's DSP sort (ugen_sort) does a depth-first traversal from all objects, marking... 

Empirically it's well established that `send~`/`receive~` in Pd introduce a one-block delay when used in a feedback loop, and in general receive~ is sorted before send~ (so always one block). The Pd help file for send~ says: "Note that send~/receive~ pairs incur a one-block delay." I'm confident enough. Also, importantly, for the LIRA-8 feedback paths a block delay is REQUIRED for causality anyway.

Decision: implement all `r~`/`s~` buses with a one-Pd-block delay (default 64 samples), which is faithful and causal. Document it. For non-feedback buses (del-tri, del-sqr) I'll also apply the block delay for consistency (it's a pure delay of 1.45ms on an LFO — inaudible, and faithful).

Now let me build. I'll set the engine's internal block size to 64 (Pd default) and let the host feed any size; the engine chunks internally. For ctag-tbd (32-frame callbacks), I'll allow configuring pdBlockSize=32 to halve latency, defaulting to 64 for fidelity. Actually simpler and better: engine processes in fixed 64-sample internal blocks, accumulating host samples. Latency = up to 64 samples extra (1.45ms) — ctag-tbd targets <1ms latency but for a drone synth this is fine. I'll make it configurable and default to 64 for the host, and document the ESP32 tradeoff. Let me default to 64 everywhere for fidelity, with a compile/runtime option.

Let me now write the code. This is a big implementation. Let me be systematic and write clean, well-documented C++17.

Key structures:

**pd_primitives.hpp**
- `Ramp` (line~): target, time in ms; per-sample increment. Pd's line~: when a new target+time arrives, it sets the ramp starting from current value, with `x_inc = (target - current)/(n_samples)` computed at block granularity... Actually Pd's line~ computes the increment per sample based on the remaining time, and it processes in blocks: it sets `x_biginc` and `x_inc` at block start. Let me be careful: Pd's line~ (d_ctl.c line_tilde_perform) uses:
```c
    if (x->x_retarget) { ... }
    ...
    t_sample f = x->x_value;
    t_sample inc = x->x_inc;
    t_sample biginc = x->x_biginc;
    ...
    for (i = 0; i < n; i++) { f += inc; *out++ = f; }
```
Hmm, Pd's line~ has the "biginc" thing for handling retargeting mid-block. Let me look at it since ramps are everywhere (23.22 ms). A simple per-sample linear ramp is behaviorally equivalent to within a sample. I'll implement per-sample linear ramp: on set(target, ms), compute inc = (target - current) / (ms * sr / 1000), then each sample current += inc until target reached. That's faithful to within a sample.

Note: Pd's line~ with time 0 → jumps immediately.

- `OnePoleLP` (lop~): coef = clamp(2π f/sr, 0, 1); y = coef*x + (1-coef)*y
- `OnePoleHP` (hip~): coef = clamp(1 - 2π f/sr, 0, 1); normal = 0.5*(1+coef); new = x + coef*last; out = normal*(new-last); last = new
- `RPole` (rpole~): y = coef*y + x
- `CosTable`: 2048+1 floats, cos~ with linear interp; osc~ = cos~(phase) with internal phasor.
- `SampHold`: samples in when control decreases.
- `Phasor`: double phase, out then increment.
- `DelayLine`: circular buffer with 4 extra samples guard, Pd's cubic read.
- `EnvFollow` (hv.envfollow): attack/release coeffs with block-delayed env comparison.
- `EnvRMS` (env~ 512): Hann-weighted overlapping sums, hop 256.
- `tanhApprox` (ma.tanh~): clip(x,-3,3); x*(27+x²)/(27+9x²)
- `mtof`, `ftom`, `dbtorms`, `powtodb`.

**Oscillator (os.triangle~)**
State: phasor, samphold, rlpf (rpole + coefficient calc).
Per sample:
```
f = abs(freqIn)
p = phasor.next(f)          // outputs phase then increments
dt = f / sr
pw = samphold(clip(shape,0,1), p)   // sample when p decreases
blep1 = polyblep(p, dt)
A = (2p-1) + blep1
p2 = wrap(p + pw)
B = (2p2-1) + polyblep(p2, dt)
pulse = A - B + 2*pw - 1
sq = 0.59 * pulse
fc = max(f/4, 100)
a = exp(-2π * min(fc, sr/2) / sr)
tri = 2.65 * rlpf(pulse, a) - 0.145
```
polyblep(phase, dt):
```
if (phase < dt) { t = phase/dt; return 2t - t² - 1 }   // wait, need to match patch
else if (phase > 1-dt) { t = (phase-1)/dt; return t² + 2t + 1 }
else 0
```
Let me derive from the patch's polyblep subpatch:
- [3] = 1/dt  (sig~1 /~ dt)
- [7] = phase * (1/dt) = phase/dt
- [11] = phase/dt - 1
- [5] = (phase/dt - 1)²
- [17] <=~ : phase <= dt → mask
- [14] = [5] * mask = (phase/dt - 1)² * (phase <= dt)
- [12] = 1/dt - 1
- [8] = phase/dt - (1/dt - 1) = (phase-1)/dt + 1
- [6] = [8]²
- [1] = 1 - dt
- [16] >=~ : phase >= (1-dt) → mask
- [15] = [6] * mask
- [0] = [14] - [15]
So polyblep = (phase/dt - 1)²·[phase ≤ dt] − ((phase−1)/dt + 1)²·[phase ≥ 1−dt]

Standard polyBLEP: for phase < dt: t=phase/dt; blep = 2t - t² - 1 = -(t-1)². ✓ matches (t-1)² negated? (phase/dt - 1)² = (t-1)². Hmm, standard is -(1-t)² = -(t-1)². But patch gives +(t-1)² for the first term and subtracts the second. Let's check the standard polyBLEP formula:

```
double poly_blep(double t, double dt) {
    if (t < dt) { t /= dt; return t+t - t*t - 1.0; }
    else if (t > 1.0 - dt) { t = (t - 1.0) / dt; return t*t + t+t + 1.0; }
    else return 0.0;
}
```
t+t-t²-1 = -(t²-2t+1) = -(t-1)². 
And second: t²+2t+1 = (t+1)².
So standard = -(t-1)² for t<dt, +(t+1)² for t>1-dt.
Patch = +(t-1)²·[t≤dt] − (t+1)²·[t≥1−dt] = −standard.

So the patch's polyblep is the NEGATIVE of the standard. Interesting. Then A = (2p-1) + blep_patch = saw − blep_standard. Hmm, that would be an incorrectly-signed BLEP... but wait, maybe the saw is inverted elsewhere. Let's see: the final pulse = A - B + 2pw - 1. With the negated blep, the correction sign flips. It might be a bug in the original patch, or intentional. Since we're porting faithfully, I replicate exactly: blep_patch = (t-1)²·[p≤dt] − (t+1)²·[p≥1−dt].

Hmm, let me double check the `<=~` and `>=~` subpatch semantics:
`<=~`: [6] min~(a, b) → m = min(a,b); [0] = m - b; [1] clip~(-1e-37, 0) → clamps to [-1e-37, 0]; [7] + 1e-37 → in [0, 1e-37]; [5] * 1e37 → [0,1].
  If a <= b: min = a → a - b <= 0 → clip gives (a-b) if a-b >= -1e-37 else -1e-37. Then +1e-37 → if a==b: 1e-37 → *1e37 = 1. If a < b by more than 1e-37: -1e-37+1e-37 = 0 → 0. 
  So `<=~` outputs 1 iff a == b (within 1e-37), i.e., a <= b. ✓ (with the min trick, when a<=b, min=a, a-b<=0; if a<b substantially → 0; if a==b → 1.) Wait: if a < b, min = a, a - b < 0 → clipped to -1e-37 (since a-b < -1e-37) → +1e-37 = 0 → output 0. That's wrong for a<b! Let me redo:
  clip~ -1e-37 0 means output = clamp(x, -1e-37, 0). For x = a-b = -0.5 → clamp → -1e-37. Then +1e-37 → 0 → *1e37 → 0. So a<b gives 0?! And a==b gives x=0 → clamp → 0 → +1e-37 → 1e-37 → *1e37 = 1.
  So `<=~` outputs 1 only when a == b exactly, else 0. It's effectively an EQUALITY test, not <=!
  
  Hmm, that's the heavy (hvcc) idiom for comparisons. Let me re-check: In heavy's `__lte~f`, the implementation is:
  ```
  min(a,b) - b  → if a<=b: a-b<=0 ... 
  ```
  Hmm. Let me reconsider: maybe it's `min~` with inputs (a, b) then subtract... Looking at the `<=~` subpatch again:
  ```
  [0] -~ : left = [6] min~, right = [3] inlet b   → min(a,b) - b
  [1] clip~ -1e-37 0
  [7] +~ 1e-37
  [5] *~ 1e+37
  ```
  For a < b: min = a; a - b = negative, e.g. -0.5. clip(-0.5, -1e-37, 0) = -1e-37. +1e-37 = 0. *1e37 = 0.
  For a == b: 0 → 0 → 1e-37 → 1.
  For a > b: min = b; b - b = 0 → 1.
  
  So output = 1 when a >= b, 0 when a < b. It's actually `>=~`!! The names are swapped in the patch, OR I mislabeled the inlets.
  
  Let me recheck the inlet order. In `<=~` subpatch:
  ```
  #X obj 12 12 inlet~;    → [2] = inlet a (left, x=12)
  #X obj 62 12 inlet~;    → [3] = inlet b (right, x=62)
  #X obj 41 38 min~;      → [6]
  connect 2 0 6 0   (a → min left)
  connect 2 0 0 1   (a → subtract RIGHT)
  connect 3 0 6 1   (b → min right)
  connect 6 0 0 0   (min → subtract left)
  ```
  Wait, I need to recheck. From the earlier dump of `<=~`:
  ```
  #X obj 41 60 -~;        [0]
  #X obj 41 81 clip~ -1e-37 0;  [1]
  #X obj 12 12 inlet~;    [2]
  #X obj 62 12 inlet~;    [3]
  #X obj 41 144 outlet~;  [4]
  #X obj 41 123 *~ 1e+37; [5]
  #X obj 41 38 min~;      [6]
  #X obj 41 102 +~ 1e-37; [7]
  connect 0 0 1 0;
  connect 1 0 7 0;
  connect 2 0 6 0;    a → min:0
  connect 2 0 0 1;    a → minus:1  (right)
  connect 3 0 6 1;    b → min:1
  connect 5 0 4 0;
  connect 6 0 0 0;    min → minus:0
  connect 7 0 5 0;
  ```
  So [0] = min(a,b) - a. 
  For a <= b: min = a → a - a = 0 → output 1. ✓
  For a > b: min = b → b - a < 0 → clip → -1e-37 → 0. ✓
  So `<=~` = (a <= b). Correct! I misread which inlet goes to the subtraction. 

  And `>=~`:
  ```
  connect 2 0 6 0;   a → min:0
  connect 3 0 6 1;   b → min:1
  connect 3 0 0 1;   b → minus:1
  connect 6 0 0 0;   min → minus:0
  ```
  [0] = min(a,b) - b → 1 iff b <= a i.e. a >= b ✓.

  Good, so my original reading was right: `<=~`(phase, dt) and `>=~`(phase, 1-dt).

  Now in polyblep: [17] `<=~` inputs: connect 9 0 17 0 (phase → a), connect 18 0 17 1 (dt → b). So mask1 = (phase <= dt) ✓.
  [16] `>=~` inputs: connect 9 0 16 0 (phase → a), connect 1 0 16 1 ((1-dt) → b). mask2 = (phase >= 1-dt) ✓.

  And [14] = [5] * mask1 where [5] = (phase/dt - 1)²; [15] = [6] * mask2 where [6] = [8]², [8] = phase/dt - (1/dt - 1).
  [0] = [14] - [15].
  
  So polyblep_patch = (p/dt - 1)²·[p≤dt] − ((p−1)/dt + 1)²·[p≥1−dt]. Confirmed = −standard_polyblep.

Hmm OK. Whether it's a sign bug in the original or not, faithful port = replicate. I'll note it in the docs as an observation (the BLEP correction has inverted polarity relative to the canonical Martin Finke formulation). Actually wait — let me reconsider. Maybe it's not a bug: the saw used is `2p-1` which rises from -1 to +1, and at the wrap (p: 1→0) the discontinuity is from +1 down to -1, a downward jump of 2. The standard polyBLEP adds a correction that... For a downward step at p=0, the BLEP should be: just before wrap (p near 1) add +(t+1)²... 

Standard saw polyBLEP: `saw = 2p-1; saw -= poly_blep(p, dt);` — yes! The canonical usage SUBTRACTS poly_blep from the saw. From Martin Finke's article:
```
double naiveWaveformForSaw(double phase, double phaseIncrement) {
    double value = (2*phase) - 1;
    value -= poly_blep(phase, phaseIncrement);
    return value;
}
```
Yes! So saw = (2p-1) − blep_standard = (2p-1) + blep_patch. ✓ The patch is CORRECT. 

Now the triangle: it's a filtered pulse, and the pulse is constructed as A - B + 2pw - 1 where A and B are BLEP-corrected saws. ✓ Correct.

Now let me write everything. Let me create the project.

One more: `wrap~` = x - floor(x). ✓

`min~ 1` etc. trivial.

`pow~ 31`: Pd's pow~ uses `powf`? For negative base and exponent 31 → C pow handles it. I'll use an exact integer power chain.

`dbtorms~`: signal version of dbtorms.

`clip~ 1 4`, `max~ 1.5`, `/~` etc trivial.

`noise~`: Pd's noise~ uses a 31-bit LCG: `x = x*435898247 + 382842987; output = ((double)(x & 0x7fffffff) - 1073741824) * (1/1073741824)` roughly giving [-1,1]. Let me check d_noise.c... I'll implement a similar LCG. Not critical (amplitude 0.001).

`random 1001` seeded with `$0-seed + $2`: the per-voice vibrato rate. In the port I'll use a deterministic seed derived from a user-settable seed + voice index, so it's reproducible. Pd's random uses a specific LCG; I'll use my own but document that vibrato rates differ per instance (they're random by design).

Let me now check: `osc~ 3` — the vibrato oscillator. osc~ uses the cos table with phase accumulation in double, conv = COSTABLESIZE/sr. Rate = 0.5 + 3*(rand/1000).

Also note: in lira.voice, the vibrato `line~` [28] smooths the vibrato ON/OFF param (0/1) over 23.22ms, multiplied by osc~ output. Then *0.025 → PWM, *0.005 → freq.

Now, the sensor envelope: `sel 1 0` on the sensor value (int). attack/release from FAST: fast → (100,100), slow → (200,8000). Note `v $0-attack` / `v $0-release` are shared per voice-pair (both sensors use the same). And `s $0-trigger` re-triggers the sensor when FAST changes.

Sensor: `i` (int) of the sensor param, `sel 1 0` → if 1: msg "1 $attack" → line~; if 0: msg "0 $release" → line~.

Note: line~ message "1 100" means ramp to 1 over 100 ms.

Alright. Also the `hold` param: prm.hold = (H/127)² smoothed 23.22ms → min~1 with e². 

Let me write the code now. I'll aim for a complete, compiling, tested implementation.

Let me set up directories and write files. I'll write substantial but clean code.

Let me start with the primitives header.

Regarding where to put the project: workspace root is .../workspace, containing LIRA-8 (cloned) and tools/. I'll create `lira8-cpp/`.

Let me write the files.

I need to be careful about size/time. Let me write efficiently but completely.

File 1: include/lira8/params.hpp — parameter definitions.
File 2: src/pd_primitives.hpp
File 3: src/oscillator.hpp
File 4: src/voice.hpp
File 5: src/lfo.hpp
File 6: src/delay.hpp
File 7: src/master.hpp
File 8: include/lira8/lira8.hpp + src/engine.cpp
File 9: host apps
File 10: esp32 plugin
File 11: tests
File 12: docs

Let me go. I'll write them in a few large write_file calls.

Actually — to keep it manageable and reduce risk of compile errors across many files, I'll use a header-only-ish core with a few .cpp files. Let me structure:

- `include/lira8/params.hpp` (param enum + metadata table)
- `include/lira8/lira8.hpp` (public Engine class, pimpl-free but with internal structs in namespace detail)
- `src/dsp/pd.hpp` (primitives)
- `src/dsp/osc.hpp`
- `src/dsp/voice.hpp`
- `src/dsp/lfo.hpp`
- `src/dsp/delay.hpp`
- `src/dsp/master.hpp`
- `src/engine.cpp`

For ESP32 simplicity, header-only core is attractive (single include). But engine.cpp is fine as a component source.

Let me make the core header-only except engine.cpp — actually let me just make everything header-only with an `LIRA8_IMPLEMENTATION` style? No, simpler: headers with inline functions + one engine.cpp. ESP-IDF CMake can compile it.

Memory: Engine contains 2 DelayLines whose buffers are externally provided (caller allocates, possibly PSRAM). API:
```cpp
struct Config { float sampleRate = 44100; int pdBlockSize = 64; float maxDelayMs = 5944.0f; };
static size_t delayBufferBytes(const Config&);
Engine(); 
void init(const Config&, float* delayMem1, float* delayMem2);
```
That gives full control on ESP32 (heap_caps_malloc SPIRAM).

Let me now write. I'll compute delay buffer size = ceil(maxDelayMs * sr / 1000) + 4 (XTRASAMPS) + blocksize safety.

Pd's delwrite~ size: `nsamps = (int)(ms * sr * 0.001) + XTRASAMPS`? Let me check sigdelwrite_check: 
```c
    int nsamps = (int)(x->x_msec * x->x_sr * 0.001) + XTRASAMPS;  // roughly
```
Let me not worry; I'll allocate ceil(ms*sr/1000) + 4 + pdBlockSize and use nsamps = that - 4.

Let me write the code.

For the read: limit = nsamps - n where n = pd block size. I'll implement per-sample within a block, tracking `fn` from n-1 down. Simpler: implement per-sample with p = write index, and read at delay D (in samples) with the cubic formula using indices relative to p:
  a = p - floor(D) + 1, b = p - floor(D), c = p - floor(D) - 1, d = p - floor(D) - 2, frac = D - floor(D)
with D clamped to [1.00001, nsamps - n].

That's exactly equivalent to Pd's formula with zerodel = n. 

I'll implement a circular buffer with `nsamps + 4` allocation and use modular indexing helper. To keep it fast, use a mask-free wrap with conditional adds. Since floor(D) can be up to nsamps-n, indices p - floor(D) - 2 can go below 0 → wrap by += nsamps. I'll store the buffer with 4 guard samples like Pd for speed: buffer of size nsamps+4, data region [4, nsamps+4), and when writing past the end, copy the last 4 samples to [0..3]. Then reads at index >= 2 are safe without wrapping (since p - floor(D) - 2 >= 4 - (nsamps - n) - 2 ... hmm could still be negative for large D). Pd does `if (bp < vp + XTRASAMPS) bp += nsamps;` — one conditional. I'll do the same: compute idx = p - floor(D); if (idx < 4) idx += nsamps; then a=buf[idx+1], b=buf[idx], c=buf[idx-1], d=buf[idx-2]. Since idx >= 4 after wrap, idx-2 >= 2 ≥ 0 ✓. And idx+1 <= nsamps+3 ✓ (p <= nsamps+3, floor(D)>=1 → idx <= nsamps+2, +1 = nsamps+3 ✓).

Good, exactly Pd's layout.

Now writing. Let me go file by file.

Let me first create the directory structure and params header.

For parameter enum, I'll use the standalone order (1..47) to match parameters.txt, with names from the plugin. Provide `ParamId` enum with kTune1... and a metadata table.

Let me define:
```cpp
enum Param : int {
  Tune1=0, Tune2, ..., Tune8,      // 0-7
  Fast12, Fast34, Fast56, Fast78,  // 8-11
  Sharp12, Sharp34, Sharp56, Sharp78, // 12-15
  Mod12, Mod34, Mod56, Mod78,      // 16-19
  Source12, Source34, Source56, Source78, // 20-23
  Pitch1234, Pitch5678,            // 24-25
  Hold1234, Hold5678,              // 26-27
  Switch, TotalFB, Vibrato,        // 28-30
  Drive, DistMix, Volume,          // 31-33
  LfoFreqA, LfoFreqB, LfoAndOr, LfoLink, // 34-37
  DelTime1, DelTime2, DelFeedback, DelMix, DelMod1, DelMod2, DelSource, DelWaveform, // 38-45
  Quantize,                        // 46
  kNumParams
};
```
This matches parameters.txt order exactly (1-based index = enum+1). 

Defaults: from parameters.txt (the shipped preset) AND from the plugin's -default values. I'll provide both: `defaultParams()` (plugin defaults) and `loadParametersTxt()` for the shipped preset. Let me use the plugin defaults as the canonical defaults, and ship the standalone's parameters.txt as a preset file.

Plugin defaults mapping:
Fast*=1, Tune1=32,Tune2=64,Tune3=32,Tune4=64,Tune5=32,Tune6=64,Tune7=32,Tune8=64,
Sharp*=0, Mod*=0, Source12=1(OFF),Source34=1,Source56=1,Source78=1,
Pitch1234=64,Pitch5678=64, Hold=0, Switch=0,TotalFB=0,Vibrato=0,
LfoFreqA=64,LfoFreqB=64,AndOr=0,Link=0,
DelTime1=64,DelTime2=64,Feedback=64,Mix=0,Mod1=0,Mod2=0,DelSource=2(LFO),Waveform=0(TRI),
Drive=64,DistMix=64,Volume=127,Quantize=0.

Good.

Now the code. Let me write it.

I'll write the primitives carefully.

Regarding `line~` (Ramp): Pd's line~ takes "$1 23.22" → target, ramp time ms. I'll implement:
```cpp
struct Ramp {
  float value=0, target=0, inc=0; int remaining=0;
  void set(float t, float ms, float sr) {
    target=t;
    if (ms<=0) { value=t; inc=0; remaining=0; }
    else { int n=(int)(ms*sr*0.001f); if(n<1)n=1; remaining=n; inc=(t-value)/n; }
  }
  float next() { if(remaining>0){ value+=inc; if(--remaining==0) value=target; } return value; }
};
```
Note: Pd's line~ outputs the value AFTER incrementing (first sample already moves). Fine.

Control-rate params: In Pd, params arrive as messages (from GUI) and are converted to floats then to line~ targets. Non-smoothed params (like `== 0` gates, `sel`, quantize) act immediately. I'll store raw param values (0..127 ints) and compute derived control values on change, then feed ramps.

For efficiency: recompute derived values only when a param changes (dirty flag), at block boundaries. That matches Pd (messages processed between blocks).

Now the per-sample smoothing: each Ramp advances per sample. Number of ramps: 
- per voice pair: sharp, mod, tune1, tune2, vibrato(shared per pair but Pd has one per pair), sensorA, sensorB → 7
- 4 pairs → 28
- hold: 2
- master: drv, mix, vol → 3
- lfo: freqA, freqB → 2
- delay: time1, time2, fb, mix, mod1, mod2 → 6
- compressor coeffs: 4 ramps (attack/release × 2 delays) with 10ms
Total ~45 ramps × ~6 cycles = 270 cycles/sample. OK.

Let me write now. Enough analysis.

For the ESP32 plugin I need to match ctag-tbd's API: `Process(const ProcessData&)` with interleaved stereo float buf, `Init(blockSize, blockPtr)`, data model JSON, etc. Writing a full ctagSoundProcessor subclass requires their generated data model boilerplate (knowYourself, pMapPar, etc.). Let me look at a small plugin's hpp/cpp fully to replicate the pattern correctly. Let me check ctagSoundProcessorFBDlyLine.hpp and the JSON model.

Given time, I'll write the ESP32 integration as:
1. A self-contained `Lira8Component` (ESP-IDF) that wraps the engine with I2S/WM8731 via ctag-tbd's drivers — but that's deep.
2. Better: a `ctagSoundProcessorLira8` plugin following their exact pattern.

Let me examine one plugin fully (a simple one) to copy the boilerplate. Let me look at the smallest: ctagSoundProcessorDust or MonoDelay.

Let me check MonoDelay (uses SPIRAM, good template).