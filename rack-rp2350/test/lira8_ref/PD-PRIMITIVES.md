# Pd primitive laws — verified for the rack-lira8 port (2026-11-15)

Ground truth for L2/L3 unit tests. Sources: pure-data/pure-data `master`
(d_ctl.c, d_osc.c, d_filter.c — quoted verbatim in-session) + empirical
probes (`probes/`, pd 0.57.0 at `-r 32000`, `-nogui -nodac -noadc` +
in-patch `[; pd dsp 1(` — see run_probes.sh header for the recipe gotchas).

## Scheduler facts (measured)

- Internal block = **64 samples** (`s~/r~` probe: offset exactly 2 ms @32k).
  Engine processes internal 64-sample blocks; rack 16-sample blocks
  accumulate 4×. Latency +2 ms — irrelevant for a drone machine.
- `send~`/`receive~` = **one-block delay** (receive sorted before send).
  All `r~`/`s~` buses (voice-pair buses, del-tri/del-sqr, total-feedback)
  get one internal block of delay. Causally REQUIRED for the feedback loops.
- `vd~` minimum read delay (zerodel) = **one block** (measured 2 ms @32k).
- pd honored `-r 32000` internally (env~ hop cadence 8 ms = 256 samples).

## hip~ (d_filter.c, verbatim)

```
coef = clamp(1 - f * 2*pi / sr, 0, 1)          // set at dsp time / freq change
normal = 0.5 * (1 + coef)                       // pd_compatibility > 43
new = x[n] + coef * last
y[n] = normal * (new - last)
last = new                                      // denormal flush to 0
// coef >= 1 (f <= 0): pure bypass
```
Probe fit: per-sample pole at f=20/sr32k = 0.996319 vs (1−2πf/sr) = 0.996073
(metro jitter); sim[64]=0.7759 vs measured 0.7789 ✓.

## lop~ (d_filter.c, verbatim)

```
coef = clamp(f * 2*pi / sr, 0, 1)
y[n] = coef * x[n] + (1 - coef) * y[n-1]        // y is the state
```
Probe: f=4000 @32k → coef 0.7854 → step passes in <1 ms (matches).

## rpole~ / rzero~ / samphold~ / clip~ / min~/max~ / wrap~

One-pole `y = coef·y + x` (rpole: coef from exp~ output as used in
os.triangle~'s rlpf); samphold samples input when control DECREASES;
wrap = x − floor(x). All standard.

## env~ (d_ctl.c, verbatim)

```
w[i]     = (1 - cos(2*pi*i / npoints)) / npoints      // Hann / npoints
period   = npoints / 2 (default)                      // 256 for env~ 512
realperiod = period rounded UP to block multiple      // 256 @ block 64
// MAXOVERLAP (=4) ring of partial sums:
each block: for (count = phase; count < npoints; count += realperiod)
                sumbuf[j] += w[count + i] * x[i]^2   // i over block, reversed
            sumbuf[last] = 0 after; phase -= n
on phase < 0: result = sumbuf[0]; shift ring; phase = realperiod - n
output (clock): powtodb(result) = (result < 1e-10 ? 0 : 100 + 10*log10(result))
```
`result` is Hann-weighted power. Half power gives about 96.9897 dB and a
sub-floor input gives 0 dB. At the primitive test's existing step alignment
(`kStep=19318`), consecutive reports are about 94.76835, 99.94455, then 100 dB.

## osc~ / cos~ (d_osc.c, verbatim)

- Table: `COSTABSIZE = 2048` (+1 guard), `t[i] = cos(2*pi*i/N)` with exact
  1/0/−1 pinned at quadrant points; **linear interpolation** between points.
- osc~: conv = COSTABSIZE/sr; phase in double, wrapped by the UNITBIT32
  trick (pure double wrap [0, N) is equivalent); output interpolates
  table[phase mod N].
- cos~: input phase signal, same table/interp.
- phasor~: conv = 1/sr, double phase, **output-then-increment**, wrap [0,1).

## noise~ (d_osc.c, verbatim)

```
static int init = 307;            // per-instance: init *= 1319 (first call)
val = val * 435898247 + 382842987; // int32 wrap
out = ((val & 0x7fffffff) - 0x40000000) / 0x40000000    // [-1, 1)
```
In the patch: noise~ * 0.001 into both delay inputs — deterministic LCG,
seed per instance (307·1319^k chain). Port: replicate exactly for goldens.

## line~ (d_ctl.c)

For `ms > 0`, the linear ramp uses `64 * max(1, floor(ms*sr/(1000*64)))`
samples. For `ms <= 0`, it jumps immediately. At 44.1/32 kHz, 23.22 ms
runs for 1024/704 samples and 11 ms runs for 448/320 samples.

The port's `Ramp::next()` retains increment-first indexing. Pd emits the
current value before incrementing.

## delay lines (d_delay.c + take-2)

- `delwrite~ NAME ms` — size arg is MILLISECONDS (the patch's 5944 =
  5.944 s max, confirming the time law `2^((T/127)·12)·1.45125 ms`).
  Buffer = ceil(ms·sr/1000) + 4 guard samples; write wraps with 4-sample
  guard copy.
- `vd~ NAME` 4-point cubic read at D samples: idx = p − floor(D) (wrap by
  += nsamps if < 4), a=buf[idx+1], b=buf[idx], c=buf[idx−1], d=buf[idx−2],
  frac = D − floor(D); D clamped to [1e-5, nsamps − blocksize].
- **RP2350 decision (user-approved)**: int16 storage @32 kHz, cap 1.45 s/line
  (delay time law compresses the top ~12% of the knob into the cap). The
  ESP32/ctag port keeps the full 5.944 s via PSRAM.
```
## Pd mtof / ftom (exact constants)

```
mtof(m) = 8.17579891564 * exp(0.0577622650 * m)   // m clamped [-1500, 1499]
ftom(f) = (f > 0 ? 17.3123405046 * log(0.12231220585 * f) : -1500)
dbtorms(d) = pow(10, (d - 100) / 20)   // 100 dB = 1.0
```

## pd quirk checklist (from take-1, to preserve in the port)

- **os.triangle~ rlpf is samplerate-AWARE** (connect-list verified
  2026-11-15): the `/~ 44100` divisor and `clip~ 0 22050` max are creation-arg
  fallbacks overridden by `samplerate~` (`6 1 22 1` wires sr into the divisor,
  `4 0 15 2` wires sr/2 into the clip max). No 32 kHz hazard. Coefficient:
  `a = exp(-2*pi*fc/sr)`, `fc = clip(max(f/4, 100), 0, sr/2)`; one-pole is
  input-normalized: `y += (1-a)*(x-y)`. (take-1 flagged these as hazards —
  correction recorded here.)
- compressor~ below-threshold gain = tanh(1) ≈ 0.7616 (ma.tanh~ applied to
  the clamped gain, not bypassed at unity).
- delay feedback law = `x·2^(2x)` up to 4.0 at F=127 (compander in the loop
  is what keeps it stable).
- 0.999/0.001 gating constants in prm.source mux (anti-denormal, no smoothing
  — *~ right-inlet floats jump at block boundaries).
- lop~ 4000 before the delay compander: delayed content is band-limited
  (relevant for the future 16 kHz delay variant).
- master drive amp = `10^((3^(2d+1)+3)/20)` → up to 31.6× before tanh.
- vibrato depth (not rate) is randomized per voice-pair: V = sin(2π·3t) ×
  (0.5 + 3·random(1001)/1000), seed = patch_seed + pair index (connect-list
  verified; take-1's "rate" wording corrected).
