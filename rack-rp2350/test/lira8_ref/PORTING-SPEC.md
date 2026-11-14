# LIRA-8 → C++ @ 32 kHz — Complete Porting Spec

> ## ⚠️ CORRECTIONS (2026-11-15, verified empirically AFTER this spec was written)
>
> Three claims in this spec were re-verified by headless probes / raw connect
> lists and CORRECTED. Where the body contradicts this box, this box wins:
>
> 1. **`delwrite~ 5944` IS MILLISECONDS (5.944 s buffer), not samples.**
>    Probe (probe_delms lineage): impulse into `delwrite~ bigbuf 5944`,
>    `sig~ 2000 → vd~` → step returned at exactly entry+2000 ms. pd's
>    documented msec semantics hold; the §2/"conflated samples with ms" claim
>    and the §8 `4312 samples` conversion are WRONG. The delay time law
>    `2^(12k/127)·1.45125 ms` (max 5944.32 ms ≈ buffer size) is real and
>    unclamped. RP2350 storage decision (user-approved): int16 @32 kHz,
>    **cap 1.45 s per line** (knob law compresses the top ~12% of the range
>    into the cap); ESP32/ctag keeps the full 5.944 s via PSRAM.
> 2. **The two delay buffers are PARALLEL with a SHARED write input, not
>    series.** Resolved dump (delay_master.txt): `[19] = inlet~ + noise*0.001`
>    feeds BOTH write summers (`[19]→[8]` and `[19]→[60]`); each bus's
>    feedback tap is its OWN expander output × the shared feedback gain
>    line~ (`[29]→[30]→[8]:1`, `[29]→[68]→[60]:1`). The two buffers diverge
>    only through their own feedback tails and their independent TIME/MOD
>    read values.
> 3. **The compressor~ multiplex quirk is behaviorally MOOT in this patch:**
>    the delay-path compressors run attack=release=2.5 ms (loadbang
>    `2.5 2.5 -12 5`), so whichever comparison the mux uses (signal-vs-env or
>    coeff-vs-coeff) both branches carry the SAME coefficient. Implement
>    take-1's law (`coeff = temp > env ? attack : release`).
>
> Also verified: vibrato depth draws are pd's default-PRNG constants
> (744/930/728/920 — the `$0-seed` receiver is dormant, nothing sends to it);
> the §14 "seed machinery" text about user seeds is aspirational only.

Source of truth: `patch/_LIRA-8.pd` + `patch/abs/*.pd` (verified byte-identical to
upstream `/home/schrab/Projects/LIRA-8/LIRA-8_Pd_Standalone/`). `graph.json`
(hvcc pd2hv dump) was used only as a cross-check; every wire below was confirmed
against the raw `.pd` text with a $-arg-resolving parser (`spec_pdlib.py`,
`spec_dump.py` in this directory — outputs in `buses.txt`, `voice.txt`,
`delay_master.txt`, `misc.txt`, `misc2.txt`, `tri.txt`, `comp.txt`, `xcheck.txt`).

Reference sample rate of the original: **Fs = 44100 Hz** (standalone). Target:
**Fs = 32000 Hz**. All times below are ms unless stated; all `line~` smoothing
ramps are **23.22 ms** (= 1024 samples @ 44.1 kHz) unless stated otherwise.

Patch-global $0 is rendered literally as `$0` (in the standalone it is `1001`).
In the port, `$0-…` names become per-patch-instance namespace prefixes (e.g.
`l8_…`).

---

## 1. Send / receive bus resolution

Scoping rules that were verified empirically in headless `pd` (see §14 item 1):

* Inside an abstraction instance, `$1…$9` substitute the instance's creation
  args recursively (`lira.voice $0 1 2` → its internal `r $1-s-sharp-$2$3`
  becomes `r $0-s-sharp-12`).
* **Abstraction inlet order is by x-position (leftmost = inlet 0), NOT creation
  order.** This was verified with a discriminating test patch in Pd
  (`/tmp/pdtest2/child4.pd`): a message sent to inlet 1 reached the *control*
  `[inlet]` created first but placed at larger x. For `lira.voice` this means:
  inlet 0 = `inlet~` at x=124 (I1 Mod Source), inlet 1 = `[inlet]` at x=364
  (I2 Pitch), inlet 2 = `inlet~` at x=377 (I3 Hold) — matching the on-canvas
  labels. (hvcc assigns the same indices, `graph.json` node `graph_apqcwQ3k`.)
* The 24 `v` (value) objects are hvcc-unsupported but trivial in the port:
  `$0-attack` / `$0-release` are two patch-global floats (all 4 voice instances
  share the same pair — see §10).

### 1.1 Signal buses (`s~` / `r~`)

| Bus name | Senders | Receivers |
|---|---|---|
| `$0-del-12` | `s~` in lira.voice(1,2) [obj 55, after hip~ 3] | `r~` ×2 in prm.source(3 4 · 1 2 · 1 2) |
| `$0-del-34` | `s~` in lira.voice(3,4) | `r~` in prm.source(1 2 · 3 4 · 7 8) [srcA] and prm.source(5 6 · 7 8 · 3 4) [srcB] |
| `$0-del-56` | `s~` in lira.voice(5,6) | `r~` ×2 in prm.source(7 8 · 5 6 · 5 6) |
| `$0-del-78` | `s~` in lira.voice(7,8) | `r~` in prm.source(1 2 · 3 4 · 7 8) [srcB] and prm.source(5 6 · 7 8 · 3 4) [srcA] |
| `$0-del-tri` | `s~` hyper-lfo/triangle-lfo [obj 11] | `r~` delay [obj 102] (LFO wave, TRI) |
| `$0-del-sqr` | `s~` hyper-lfo/square-lfo [obj 15] | `r~` delay [obj 103] (LFO wave, SQR) |
| `$0-sqr-lfo` | `s~` hyper-lfo/square-lfo [obj 19] | `r~` in all 4 prm.source (LFO-CV mode) |
| `$0-total-feedback` | `s~` master_output [obj 35, hip~ 3 tap] | `r~` in all 4 prm.source (LFO-CV alt) |
| `$0-env` | `s~` inside both delay compressor~/hv.envfollow | `r~` in the same two hv.envfollow (per-instance scoped by hvcc `$0`; in the port make it a per-compressor member) |

### 1.2 Control buses (`s` / `r`, all `$0`-scoped; senders are GUI atoms + MIDI)

| Receiver name | Producer (sender) | Consumer |
|---|---|---|
| `$0-s-tune-1..8` | 8 × `vsl 0..127` (gui 1,4,25–30) | `r` in prm.tune of matching voice |
| `$0-s-sharp-12/34/56/78` | 4 × `hsl 0..127` (gui 2,31,32,33) | `r` in prm.sharp of the pair |
| `$0-s-mod-12/34/56/78` | 4 × `hsl 0..127` (gui 3,34,35,36) | `r` in prm.mod of the pair |
| `$0-s-source-12/34/56/78` | 4 × `hradio 0..2` (gui 24,74,75,76) | `r` ×2 in prm.source of the pair |
| `$0-s-fast-12/34/56/78` | 4 × `tgl` (gui 5,37,38,39) | `r` in lira.voice [obj 45] |
| `$0-s-sensor-1..8` | 8 × `tgl` (gui 7–21) **and** 8 × notemap clone instances (MIDI notes 36–43) | `r` in prm.sensor of matching voice |
| `$0-s-pitch-1234 / -5678` | 2 × `hsl 0..127` (gui 22,23) | `r` in prm.pitch quad |
| `$0-s-hold-1234 / -5678` | 2 × `hsl 0..127` (gui 77,55) | `r` in prm.hold quad |
| `$0-s-switch` | `tgl` (gui 56) | `r` ×2 in each of the 4 prm.source |
| `$0-s-total-fb` | `tgl` (gui 57) | `r` ×2 in each of the 4 prm.source |
| `$0-s-vibrato` | `tgl` (gui 58) | `r` in each lira.voice [obj 29] |
| `$0-s-quantize` | `tgl` (gui 217) | `r` in all 8 prm.tune |
| `$0-s-drv` | `vsl 0..127` (gui 88) | `r` master_output [obj 5] |
| `$0-s-dst-mix` | `vsl 0..127` (gui 87) | `r` master_output [obj 17] |
| `$0-s-vol` | `vsl 0..127` (gui 86) | `r` master_output [obj 0] |
| `$0-s-f-a` / `-f-b` | 2 × `vsl 0..127` (gui 90,89) | `r` hyper-lfo [obj 7 / 16] |
| `$0-s-andor` | `vradio 0..1` (gui 91) | `r` ×2 in square-lfo |
| `$0-s-link` | `tgl` (gui 111) | `r` hyper-lfo [obj 4] |
| `$0-s-time-1 / -time-2` | 2 × `hsl 0..127` (gui 84,82) | `r` delay [obj 9 / 86] |
| `$0-s-feedback` | `hsl 0..127` (gui 174) | `r` delay [obj 21] |
| `$0-s-del-mix` | `hsl 0..127` (gui 173) | `r` delay [obj 91] |
| `$0-s-mod-1 / -mod-2` | 2 × `hsl 0..127` (gui 85,83) | `r` delay [obj 47 / 85] |
| `$0-s-del-mod` | `hradio 0..2` (gui 81) | `r` ×3 in delay [41,43,72] |
| `$0-s-lfo-wav` | `hradio 0..1` (gui 80) | `r` ×2 in delay [34,37] |
| `$0-trigger` | `s` in each lira.voice [obj 51] | `r` in all 16 prm.sensor |
| `$0-seed` | `s` gui [obj 181] (loadbang → realtime chain) | `r` in each lira.voice [obj 34] |
| `$0-attack` / `$0-release` | `v` write in each lira.voice [48/49] + each prm.sensor | `v` read in the same places (patch-global float) |
| `$0-r-sensor-1..8` | keyboard subpatch `s` (from key.tgl '1'..'8') and notemap clones | the sensor `tgl` GUI atoms (set their state; the tgl then emits `$0-s-sensor-N`) |
| `$0-r-led` | `s` square-lfo [11] (snapshot/change/sel 1 −1) | `bng` GUI blink only — cosmetic |
| `$0-s-dsp`, `$0-r-dsp`, `s pd` | gui/root DSP toggle plumbing | not needed in the port |
| `$0-s-cpu`, `$0-r-cpu` | cputime meter chain | cosmetic |
| `$0-loadbang-0/1`, `$0-write`, `$0-r-*` (label sync), `pd` | gui.link / labels GUI plumbing | **ignore entirely** (gui.link is the parameter persistence abstraction in `patch/abs/gui.link/gui.link.pd`) |

Everything is single-name → fan-out (Pd buses). No `throw~`/`catch~`, no
`send`/`receive` name collisions across scopes after substitution.

---

## 2. `delay` subpatch (120 objects; `_LIRA-8.pd` L674–919)

Two delay units in **series**, each with its own time/modulation/compression;
a shared monitor path with a tanh and wet/dry crossfade. Object indices below
are the parser indices of the `pd delay` canvas (see `delay_master.txt`).

### 2.1 Input and write chain (bus 1)

```
inlet~ [0] ──┬──> +~[19]:0 ──> +~[8]:0 ──────────────> delwrite~ $0-mod-delay-1, 5944 samples [11]
             │        ^               ^
             │        │               └── *~[30] <── expander~[17] <── compressor~[14] <── lop~ 4000 [32] <── hip~ 1 [31] <── vd~ $0-mod-delay-1 [12]
             │        └── noise~[20] → *~ 0.001 [104]      (feedback gain g, see 2.4)
             └──> *~[90]:0  (dry tap; see §2.5)
```

* Read position of `vd~[12]`: `+~[53] = line~[6] (time) + *~[46] (modulation)`.
* Time (control): `r $0-s-time-1 [9] → /127 [2] → *12 [10] → msg "2 {v}" [3]
  → pow [4] → * 1.45125 [5] → msg "{v} 23.22" [7] → line~ [6]`.
  So **requested time = 2^(12·k/127) · 1.45125 ms**, k = knob 0…127 →
  1.45125 ms … 5945.5 ms.
  **The author confused samples with ms**: the buffer is 5944 *samples*
  (134.74 ms @ 44.1 k), so Pd clamps `vd~` reads to 134.74 ms. Effective
  behavior: requested time is clamped to `[0, 134.74 ms]`; the knob reaches the
  full buffer at k ≈ 69 and stays clamped above that. (The saved default
  TIME-1 = 94.57 → 711 ms requested → fully clamped at 134.74 ms; TIME-2 =
  56.74 → 59.7 ms, unclamped.)
* Modulation of read time (added to `[53]`):
  * **SELF** (del-mod == 0): own delayed output `vd~[33] → /~ 2 [98] → lop~ 689
    [99] → *~[39]`, gated by `r $0-s-del-mod → == 0 [44]` (the `==` output 0/1
    feeds the `*~` right inlet directly — a hard, unsmoothed gate).
  * **LFO** (del-mod == 2): `r~ $0-del-tri [102] → *~[35]` (gated by
    `r $0-s-lfo-wav → == 0 [36]`), `r~ $0-del-sqr [103] → *~[38]` (gated by the
    raw lfo-wav value 0/1 [37]) → `+~[107] → *~[40]`, gated by
    `r $0-s-del-mod → == 2 [42]`. (Same hard gating.)
  * `+~[45] = SELF + LFO → *~[46]`, depth = `r $0-s-mod-1 [47] → /127 [48] →
    pow 2 [49] → * 10 [50] → line~ [52]` → **depth = (k/127)² · 10 ms**.
    (The depth scales BOTH self and LFO paths.)
  * del-mod == 1 (OFF): neither gate opens → static delay time.
* Feedback write sum: `+~[8] = (input + noise) + g · processed_read`.

### 2.2 Bus 2 (objects 54–101, 110) — identical structure

* Time: `r $0-s-time-2 [86] → /127 → *12 → "2 {v}" → pow → * 1.45125 → line~ [58]`.
* Mod: SELF = `vd~ $0-mod-delay-2 [82] → /~ 2 [97] → lop~ 689 [100] → *~[71]`
  (gated by the **same** `==0(del-mod)` [73] as bus 1); LFO = **`*~[40]` shared
  with bus 1** → `+~[74]` → `*~[75]` × depth from `r $0-s-mod-2 [85] → /127 →
  pow 2 → * 10 → line~ [80]` → `+~[81] = line~[58] + mod` → `vd~[84]`.
* **Bus 2's write input is bus 1's write sum**: `+~[19] → +~[60]:0` (plus `g ·
  processed_read` from `*~[68]`, plus nothing else — the 0.001 noise enters
  bus 1 only). So the two buffers are a series cascade:
  `mod-delay-2(t2) = [bus1_write_sum](t2) + g·proc2(bus2_read)`.

### 2.3 Per-bus compressor → expander (both buses identical)

```
vd~ → hip~ 1 Hz → lop~ 4000 Hz → compressor~ → expander~ ─┬→ *~ (×g) → write sum
                                                          └→ monitor sum
```

* `loadbang → msg "2.5 2.5 -12 5" [109/110] → unpack f f f f [15/64]` →
  compressor inlets: **attack = 2.5 ms, release = 2.5 ms, threshold = −12 dB,
  ratio = 5** (see §2.6 for the exact compressor math).
* `loadbang → msg "-60 5" [105/106] → unpack f f [18/67]` → expander inlets:
  **threshold = −60 dB, ratio = 5** (see §2.7).

### 2.4 Feedback gain (shared by both buses)

```
r $0-s-feedback [21] → /127 [22] → t f f [23]
    out0 → *2 [24] → msg "2 {v}" [25] → pow [26] → * [27] (× out1) → msg "{v} 23.22" [28] → line~ [29]
    out1 ────────────────────────────────────────────↗ (raw k/127)
```
**g = (k/127) · 2^(2·k/127)** → 0 at k=0, 4.0 at k=127, g=1.0 at k≈63.6,
default (k=54.04) g ≈ 0.77. The same `line~[29]` feeds `*~[30]:1` (bus 1 fb),
`*~[68]:1` (bus 2 fb) and the monitor limiter `max~ 1.5 [114]`.

### 2.5 Monitor / output

```
exp1[17] ─┐
          ├→ +~[87] → /~ 2 [101] → *~[113] → ma.tanh~[108] → *~[95] ─┐
exp2[66] ─┘              ↑        ↑                                  ├→ +~[96] → outlet~[1]
              sig~ 1 [111] → /~[112] (divisor = max(g, 1.5) [114])     │
dry: inlet~[0] → *~[90], gain = 1 − del-mix ([88] sig~ 1 − [89] line~) ─┘
```
* del-mix: `r $0-s-del-mix [91] → /127 [92] → line~ [94]`; feeds `*~[95]:1`
  (wet gain) and the dry subtraction `[89]` (so **dry = (1−mix)·in**, wet =
  `mix·tanh( (proc1+proc2)/2 / max(g,1.5) )`).
* `+~[96] = dry + wet → outlet~`.

### 2.6 compressor~ (abs, `compressor~.pd`; inlets: sig, attack ms, release ms, thr dB, ratio)

Envelope follower `hv.envfollow` **exactly as wired in the file** (the mux
compares coefficients, not signals — a quirk of the embedded heavylib copy; the
patch is what sounds, so reproduce the quirk):

```
tmp = |in|
c_attack = exp(−4.60517 / (attack_ms · Fs/1000))      [coeff subpatch: max 1 → "−4.60517 {t}" → /{t} → exp, smoothed by line~ 10 ms]
c_release = exp(−4.60517 / (release_ms · Fs/1000))
sel = (c_attack > c_release) ? tmp : env              [hv.multiplex: lhs=c_attack, rhs=c_release, a=tmp, b=env]
env(n) = tmp·(1 − sel) + sel·env(n−1)                 [rpole~ with input |in|·(1−sel), coefficient sel]
```

For the delay compressors **attack == release == 2.5 ms**, so `sel = env`
always and the follower reduces to:

```
env(n) = |in|·(1 − env(n−1)) + env(n−1)²
```

(tracks abs with a level-proportional time constant; decays quadratically).
The general branch (attack_ms > release_ms) selects `sel = tmp`, i.e.
`env = tmp·(1−tmp) + tmp·env` — an inverted follower; only reachable if a
future variant uses asymmetric times. Implement the mux exactly as above.

Gain law (rest of compressor~):

```
thr_rms = dbtorms(thr_db + 100) = 10^(thr_db/20)          [inlet → "+100" → dbtorms]
gain   = clip( ((env − thr_rms)/ratio + thr_rms) / env , 0, 1 )
out    = in · tanh(gain)                                   [ma.tanh~]
```

With thr = −12 dB → thr_rms = 0.2512, ratio 5. Note the tanh compresses even
at unity gain (tanh(1) = 0.7616) — this is part of the sound.

### 2.7 expander~ (abs, `expander~.pd`; inlets: sig, thr dB, ratio)

```
env_db = env~ 512 − 100                 (Pd [env~] outputs 100 + 20·log10(RMS); verified empirically: full-scale sine → 96.99)
slope  = (1 − 1/ratio) · (env_db − thr)      ([(1/r) → msg "1 {r}" → [−] → 1 − 1/r] then "* 2" with right inlet overridden; [−] with list "1 r" computes 1 − r — verified: prints −4)
slope  = min(slope, 0)
gain   = dbtorms~(100 + line~(slope, 11 ms)) = 10^(slope/20)
out    = in · gain
```
`env~ 512` = RMS over 512-sample blocks (11.6 ms @ 44.1 k). With thr −60 dB,
ratio 5: below −60 dB the envelope expands downward at slope 0.8; above, gain 1.

### 2.8 ma.tanh~ (abs)

```
x = clip(in, −3, +3)
y = x · (27 + x²) / (27 + 9·x²)        (Padé tanh, music-dsp #238; x·(x²+27)/(9x²+27))
```

### 2.9 Noise

`noise~ → *~ 0.001 [104] → +~[19]` — −60 dBFS white noise injected into the
bus-1 write sum only (therefore also flows into bus 2 via the cascade). Keeps
the compressor/expander alive and masks denormals. In the port: use a cheap
PRNG (e.g. LCG/xorshift) × 0.001, or a small dither if CPU-bound.

---

## 3. `master_output` (45 objects; `_LIRA-8.pd` L444–539)

```
inlet~[4] (from delay outlet)
  │
  ├───────────────────────────── dry path
  │
  ├→ *~ 0.1 [32] ────────────────────────┐
  │                                      ▼
  ├→ *~[15] (× drive) → hip~ 20 [31] → ma.tanh~[16] ─┬→ +~[28]:0 ─→ hip~ 10 [26] → *~[42] → +~[33]:1
  │                                                  └→ pow~ 31 [30] → /~ 4 [29] ─↗ (+~[28]:1)
  │                                *~[42]:1 = 1 / clip(drv_gain, 1..4)   (sig~1 [40] → /~[41], divisor from clip~ 1 4 [43])
  │
  │   +~[33] = 0.1·in + dist·makeup  → *~[21] (× dst-mix)  ─┐
  │   inlet~[4] → *~[24] (× (1 − dst-mix)) ─────────────────┴→ +~[25] → ma.tanh~[34] ─┬→ *~[3] (× vol) → outlet~[7]
  │                                                                                   └→ hip~ 3 [44] → s~ $0-total-feedback [35]
```

* **Drive**: `r $0-s-drv [5] → /127 [8] → *2 [37] → +1 [27] → msg "3 {v}" [36]
  → pow [9] (3^v) → +3 [38] → +100 [10] → dbtorms [11] → line~ [14]`.
  `v = 1 + 2k/127`, **gain = 10^((3^v + 3)/20)** → 2.0 (k=0) … 31.6 (k=127);
  default k=86 → v=2.354 → 3^v=13.28 → gain ≈ 6.5.
* **Distortion**: `dist = [ hip10( tanh(hip20(in·d)) + tanh(hip20(in·d))³¹/4 ) ]
  / clip(d, 1, 4)` — the `pow~ 31` adds an edge harmonically (odd-power waveshaper),
  `/~ 4` scales it; makeup `1/clip(d,1,4)` counteracts the drive (anti-blowup).
* **dst-mix**: `r $0-s-dst-mix [17] → /127 [18] → line~ [20]`; crossfades
  `0.1·in + dist` (wet) against plain `in` (dry).
* **Final**: everything passes `ma.tanh~[34]` (always), then `× (vol_k/127)²`
  (`r $0-s-vol → /127 → pow 2 → * 1 → line~ [2]`, line~ obj [2] is the smoothed
  gain; `*~[3]`), → `outlet~[7]` → gui `outlet~` → **root `dac~ 1 2`: both DAC
  channels get the same mono signal** (`#X connect 0 0 6 0; 0 0 6 1;`).
* **total-feedback tap**: `hip~ 3 Hz` of the master tanh output (pre-volume) →
  `s~ $0-total-feedback` → the four prm.source "LFO-CV" alt inputs.

---

## 4. `hyper-lfo` (28 objects) + `triangle-lfo` + `square-lfo`

### 4.1 Rates

```
r $0-s-f-a [7] → /127 [8] → pow 2 [19] → * 127 [20] → − 75 [9] → mtof [10] → line~ [11] → phasor~[0]
r $0-s-f-b [16] → /127 [12] → pow 2 [17] → * 127 [18] → − 75 [13] → mtof [14] → line~ [15] → phasor~[1]
```
**rate = mtof(127·(k/127)² − 75)** = mtof(k²/127 − 75): 0.107 Hz (k=0) …
164.8 Hz (k=127). Yes — this LFO reaches audio rate ("hyper"). Saved defaults:
F-A k=42 → 0.240 Hz, F-B k=64 → 0.692 Hz.

### 4.2 LINK (B's rate is FM'd by A's square)

```
squareA = clip( cos~(phasorA) · 1000 , −1, +1 )            [22 → 5 → 6; edges at phase 0.25/0.75]
phasorB freq input = fb_rate + squareA · fb_rate/2 · link  [15→1:0 direct; 15→2:1, 6→2:0, 2→/~2 [21] → *~[3]:0, *~[3]:1 ← r $0-s-link [4] → 1:0]
```
link = 0: independent rates. link = 1 (saved default): `rateB_effective =
fb·(1 ± 0.5)`.

### 4.3 triangle-lfo (sum of two triangle converters, one per phasor)

`tri(p) = (|p − 0.5|·4 − 1) · 0.5` ∈ [−0.5, +0.5]; both converted phasors sum
into `*~ 0.5 [10]` → **`s~ $0-del-tri` ∈ [−1, +1]** (obj 11).

### 4.4 square-lfo

```
SQR_B  = clip( cos~(phasorB) · 1000 , −1, +1 )                 [18 → 0(*~1000) → 1(clip)]
del-sqr = (squareA + SQR_B) / 2                                [13,1 → /~2 [9] → s~ $0-del-sqr [15]]  (always, independent of andor)
sqr-lfo = squareA·SQR_B·[andor==0] + (squareA+SQR_B)/2·andor   [*~2→*~3 (×==0(andor) [5]); /~2→*~6 (×andor [7]); +~[12] → s~ $0-sqr-lfo [19]]
```
andor vradio: **0 = OR mode (product of the two squares), 1 = AND mode
(average)** — labels "OR"/"AND" (text define `andor-label`: `AND -2 ; OR 2`,
label_pos flips; the vradio sends 0/1 directly).

### 4.5 LED

`bang~[16] → snapshot~[8] → change[10] → sel 1 −1 [17] → s $0-r-led` — blinks
the GUI bang each half period. Cosmetic; skip in the port.

---

## 5. `keyboard` (16 objects) + key.tgl + notemap

* `keyboard` = 8 × `key.tgl <keycode>` → `s $0-r-sensor-N`. Keycodes **49…56**
  = ASCII **'1' … '8'** → sensors 1…8.
* `key.tgl <code>`: `[key] → sel <code>`: key down → `t b b` → (msg "1" →
  `[change]` out) + (msg "stop" → del 52 right inlet, cancels pending); keyup →
  `sel <code> → del 52 (left inlet)` → after **52 ms** → msg "0" → `[change]`.
  Net: **press = 1 immediately, release = 0 after a 52 ms tail**. `[change]`
  debounces repeats.
* `notemap` (via `clone -s 1 notemap 8 $0`, obj gui[215]): 8 instances, $1 =
  1…8. Input: `[notein] → pack f f → list prepend all → list trim` (notes+bang).
  Inside: `loadbang → f $1 → + 35 → [route] (right inlet)` — instance k matches
  **MIDI note 35+k = 36…43 (C2…G2)**; `route → > 0 → s $2-r-sensor-$1` —
  note-on (vel > 0) sets sensor k **on**, note-off (vel 0) sets it **off**.
* Both keyboard and MIDI drive the same GUI `tgl` receive symbols, so the toggles
  show state and forward `$0-s-sensor-N`. Port: merge into one "sensor k state"
  byte with sources = MIDI notes 36–43 (+ optional UI).
* The GUI enumeration chain (gui objs 59–67, 78, 79, 64) and `gui.link` are
  parameter persistence only — ignore.

---

## 6. The four inter-voice nodes = `prm.source` instances (gui objs 187–190)

These are **not** unnamed mystery nodes: they are `prm.source` abstractions
(`prm.source.pd`, 33 objects) that select the **FM/mod source signal** fed to
each voice pair's `[inlet~]` (voice inlet 0). Args: `$1=$0, $2 $3 = destination
pair label, $4 $5 = srcA pair, $6 $7 = srcB pair`.

| Instance (gui obj / coords) | Args | Feeds voice pair | srcA (switch=0) | srcB (switch=1) |
|---|---|---|---|---|
| 187 @(405,407) | `$0 1 2 3 4 7 8` | voices 1-2 | `$0-del-34` | `$0-del-78` |
| 188 @(415,537) | `$0 3 4 1 2 1 2` | voices 3-4 | `$0-del-12` | `$0-del-12` |
| 189 @(595,407) | `$0 5 6 7 8 3 4` | voices 5-6 | `$0-del-78` | `$0-del-34` |
| 190 @(605,537) | `$0 7 8 5 6 5 6` | voices 7-8 | `$0-del-56` | `$0-del-56` |

Mix (all gates are soft: on = v·0.999 + 0.001, off = 0.001):

```
audio = r~ del-srcA · (switch==0)  +  r~ del-srcB · (switch)      [switch = raw 0/1 tgl]
lfoCV = ( r~ $0-sqr-lfo · (total-fb==0) + r~ $0-total-feedback · (total-fb) )
out   = audio·(source==0) + lfoCV·(source==2)
```

* source hradio: **0 = cross-feed audio from the "other" pair(s)** (34>56 /
  78>12 selection is via the SWITCH toggle), **1 = OFF** (only the 0.001
  leak), **2 = "LFO-CV"**: the square-LFO bus, or — if the TOTAL-FB toggle is
  on — the master-output feedback tap (`$0-total-feedback`, i.e. the whole mix
  through hip~ 3).
* So with defaults (source=0, switch=0): pair 1-2 is FM'd by the audio of pair
  3-4, pair 3-4 by pair 1-2, pair 5-6 by 7-8, pair 7-8 by 5-6. The modulation
  is a *frequency-multiplier* FM (see §7), not an index FM.

---

## 7. `prm.*` abstractions — full input→output math

### 7.1 prm.pitch (`prm.pitch.pd`; args `$0 q1 q2 q3 q4`)

```
r $0-s-pitch-<q1q2q3q4> → /127 → * 1.99 → + 0.01 → [outlet]  (control float)
```
**pitch_cv = 0.01 … 2.0** (k/127). One instance per quad; feeds both voice
inlets 1 of the quad (`t f f` fans out to both prm.tune inside each voice).

### 7.2 prm.tune (`prm.tune.pd`; args `$0 v`; v = 1…8 voice number)

Tuning table `text define $0-tune` (gui L433, `#A set`): 8 rows
`(semi, note2)` used as `(note at knob=0, note at knob=127)`:

| voice v | semi | note2 |
|---|---|---|
| 1 | −16 | 93 |
| 2 | −16 | 93 |
| 3 | 7 | 109 |
| 4 | 9 | 107 |
| 5 | 20 | 116.54 |
| 6 | 20 | 116.54 |
| 7 | 33 | 126.24 |
| 8 | 33 | 131.22 |

```
note   = semi + (tune_k/127)·(note2 − semi)          ["/127 → *" (× (note2−semi)) → "+" (semi) — both in MIDI-note space]
f0     = mtof(note) · pitch_cv                       ["* 28": mtof → left, pitch_cv → right (t b f retriggers)]
quantize on:  f0 = mtof( round( ftom(f0) + 0.5 ) )   [ftom → +0.5 → i → mtof; route picks by quantize flag]
out    = line~( f0 , 23.22 ms )                      [msg "{f} 23.22"]
```
`mtof(m) = 440·2^((m−69)/12)`, `ftom = 69 + 12·log2(f/440)`. Range examples:
voice 1 sweeps mtof(−16)=3.24 Hz … mtof(93)=1760 Hz (this is the
oscillator frequency; §7.6 applies the FM multiplier on top). Voice 8 tops out
at mtof(131.22) ≈ 16.0 kHz — at 32 kHz that already sits at Nyquist, and the
FM multiplier (up to ×3 with |src|≈1) wraps through it; the aliasing/wrap is
part of the original's character.

### 7.3 prm.sharp (inline `pd prm.sharp` inside lira.voice)

```
r $0-s-sharp-<vA><vB> → /127 → pow 2 → line~ 23.22 → out
```
**sharp = (k/127)²** ∈ [0,1]. One per voice pair (shared by both oscillators).

### 7.4 prm.mod (inline `pd prm.mod` inside lira.voice)

```
r $0-s-mod-<vA><vB> → /127 → pow 4 → * 2 → line~ 23.22 → out
```
**fm_depth = (k/127)⁴ · 2** ∈ [0,2].

### 7.5 prm.hold (inline `pd prm.hold` ×2 in gui; `r $0-s-hold-1234/5678`)

```
k/127 → pow 2 → line~ 23.22 → outlet~ (signal)
```
**hold = (k/127)²** ∈ [0,1], one signal per quad, wired to both voices' inlet 2
(hold). Inside the voice it is summed with the sensor gate (§10.2).

### 7.6 Frequency application inside lira.voice (per oscillator A=nA, B=nB)

```
f_smooth = prm.tune output                     (line~ 23.22 ms)
vib      = sin(2π·f_vib·t) · vibrato_depth     (osc~ [26]; depth = line~ of $0-s-vibrato 0/1)
f_add    = f_smooth + f_smooth·(0.005·vib)     [+~ 30/31, *~ 32/33, *~ 0.005 [52]]
fmd      = src · fm_depth                      [*~ 12: inlet~ src × prm.mod]
f_osc    = f_add + f_add·fmd                   [oscA: +~30 → osc:0 AND *~10 = 30·fmd → osc:0 (Pd sums) = f_add·(1 + fm·src)]
```
Osc B identical (`+~31`, `*~11 = fmd·f_addB`, `os.triangle~[6]`).
**This is thru-zero multiplier FM**: with `src` bipolar and depth 0…2 the
multiplier `(1 + fm·src)` sweeps through 0 into negative territory; the
oscillator's `abs~` folds negative frequencies. `src` is the prm.source output
(bipolar pair-audio or ±1 square). `vib` also modulates the PWM phase (§7.7).

### 7.7 os.triangle~ (abs; inlet 0 = freq Hz, inlet 1 = PWM phase; out0 = square, out1 = triangle)

```
pwm_in = 0.53125 + 0.025·vib          [sig~ 0.53125 [27] + *~ 0.025 [40] sum into inlet 1]
pwm    = clip(pwm_in, 0, 1)           [clip~ 0 1 [14], held once per cycle by samphold~ [13] clocked by the phasor]
p(t)   = phasor(|f|)                  [abs~ [21] first — negative FM folds to positive]
saw1   = 2p − 1 + pb(p, n)            [normalize = *2 −1; pb = polyblep correction, n = |f|/Fs (subpatch "1/Fs")]
saw2   = 2·wrap(p + pwm) − 1 + pb(wrap(p+pwm), n)
pulse  = (saw1 − saw2) + 2·pwm − 1    [+~2/7 → −~11 → +~12 (2·pwm via *~2 [15]) → −~1 [17]]  → bipolar square, duty = pwm
square_out  = pulse · 0.59                                   [*~ 0.59 [27] → outlet 0 [8]]
cutoff      = clip( max(|f|/4, 100), 0, Fs/2 )               [/~ 4 [22] → max~ 100 [29] → rlpf freq]
tri_out     = ( OnePoleLP(pulse, cutoff) − 0.145 ) · 2.65    [rlpf → *~ 2.65 [26] → −~ 0.145 [25] → outlet 1 [19]]
```

* **polyblep correction (patch-exact)**, per subpatch `polyblep` (t = phase
  0…1, n = |f|/Fs; `<=~`/`>=~` are signal comparators with a ±1e-37 denormal
  hack — in C++ just use `(a >= b) ? 1 : 0`):

  ```
  pb(t, n) = (t/n − 1)² · [t ≤ n]  −  ((t−1)/n + 1)² · [t ≥ 1 − n]
  ```

  (start correction positive, end correction negative; this is the sign
  convention that makes `saw1/saw2` continuous through the wrap — verified by
  derivation.)
* **rlpf** (one-pole LP, subpatch of os.triangle~): `coef = exp(−2π·fc/Fs)`;
  `y(n) = x(n)·(1 − |coef|) + coef·y(n−1)`. The `clip~ 0 22050` high limit and
  the `/~ 44100` divisor are **overridden at load time from `samplerate~`**
  (t b b → samplerate~ → t f f → clip inlet 2 and /~ right inlet), so the
  subpatch is sample-rate-correct at runtime despite the hardcoded args.
* The `msg reset` → samphold~ (obj 16) has no trigger — dead object.
* `samplerate~`/`1/Fs` subpatches are load-time constants.

### 7.8 Voice output and bus send (lira.voice)

```
oscA_sum = pulseA·sharp + triA·(1−sharp) + burstA        [*~1,2 → *~3:0 (sum); sharp from §7.3; 1−sharp = −~ [16] from loadbang msg "1"]
oscB_sum = pulseB·sharp + triB·(1−sharp) + burstB        [*~7,8 → *~9:1 (sum); *~9:0 = gate]
gateA    = min( hold + gateA², 1 )   [*~18; hold = inlet 2 signal, gateA² = prm.sensor outlet 0]
gateB    = min( hold + gateB², 1 )   [*~19]
voice_out = gateA·oscA_sum + gateB·oscB_sum              [*~3,9 → +~5]
outlet~   = voice_out                                    [*~5]
s~ $0-del-<nA><nB> = hip~ 3 Hz (voice_out)               [hip~ [57] → s~ [55]]
```
The `burst` terms (prm.sensor outlet 1) are added **inside** the gated sum.

---

## 8. Constants table (DSP path)

| Value | Role | Source (file:line or obj) |
|---|---|---|
| 5944 | delwrite~ buffer, samples (134.739 ms @ 44.1k) | `_LIRA-8.pd:686,758` |
| 1.45125 | delay time curve `2^(12k/127)·1.45125` ms (≈ 5944/4096) | `:680,732` |
| 12 | time curve exponent span | `:685,736` |
| 23.22 | all `line~` smoothing ramps, ms (= 1024 samples @ 44.1k) | every msg "{x} 23.22" (×30+) |
| 1 Hz | feedback-loop hip~ | `:706,744` |
| 4000 Hz | feedback-loop lop~ | `:707,745` |
| 689 Hz | SELF-mod lop~ (both buses) | `:774,775` |
| 2 | SELF-read divider (`/~ 2`) | `:772,773,776` |
| 10 | delay-mod depth ` (k/127)²·10 ` ms | `:725,753` |
| 0.001 | noise gain into write sum | `:779` |
| 2.5, 2.5, −12, 5 | compressor attack ms, release ms, thr dB, ratio (both buses) | msgs `:784,785` |
| −60, 5 | expander thr dB, ratio (both buses) | msgs `:780,781` |
| 512 | expander env~ window, samples | `expander~.pd:12` |
| 11 | expander line~ smoothing, ms | `expander~.pd:20` |
| −4.60517 | envfollow coeff = exp(−4.60517/t_samples) = 1% point | `compressor~.pd:28,46` |
| 10 | coeff line~ smoothing, ms | `compressor~.pd:25,43` |
| 27, 9, ±3 | ma.tanh Padé + input clip | `ma.tanh~.pd:3,9,11` |
| 10 Hz | master hip~ after distortion | `:471` |
| 20 Hz | master hip~ before drive tanh | `:476` |
| 31 | master pow~ exponent (tanh^31) | `:475` |
| 4 | master /~ 4 after pow~ ; also clip(d,1,4) | `:474,488` |
| 0.1 | master dry-into-distortion gain | `:477` |
| 1, 4 | master makeup clip bounds | `:488` |
| 3 Hz | voice→bus send hip~ ; master feedback tap hip~ | `lira.voice.pd:153`, `_LIRA-8.pd:489` |
| 0.166667 | 4-voice sum scaler (1/6) | `:1112` |
| 0.53125 | osc PWM base phase | `lira.voice.pd:90` |
| 0.025 | vibrato→PWM depth | `lira.voice.pd:103` |
| 0.005 | vibrato→pitch depth (relative) | `lira.voice.pd:148` |
| 3 Hz | vibrato osc creation arg (initial; overridden at load by the random value) | `lira.voice.pd:89` |
| 1001, /1000, ·3, +0.5 | vibrato rate = 0.5 + 3·random(1001)/1000 Hz | `lira.voice.pd:105–107,147` |
| 100, 100 | fast envelope attack/release ms | `lira.voice.pd:149` |
| 200, 8000 | slow envelope attack/release ms | `lira.voice.pd:150` |
| 52 | key.tgl release tail, ms | `key.tgl.pd:6` |
| 35 | notemap MIDI note offset (notes 36…43) | `notemap.pd:6` |
| 49…56 | keyboard keycodes '1'…'8' | `_LIRA-8.pd:262–269` |
| 0.999, 0.001 | prm.source soft gate on/off scalars | `prm.source.pd:16–27` |
| 1.99, 0.01 | prm.pitch: cv = 1.99·k/127 + 0.01 | `prm.pitch.pd:3–4` |
| −16/93, 7/109, 9/107, 20/116.54, 33/126.24, 33/131.22 | per-voice tune table (semi, note2) | `_LIRA-8.pd:434–435` |
| 127, −75, 2 | LFO rate = mtof(127(k/127)² − 75) | `_LIRA-8.pd hyper-lfo :634–637,626,627` |
| 1000, ±1 | cos·1000 hard clip → square | hyper-lfo `:622–623`, square-lfo `:579–580` |
| 0.5 | triangle-lfo output scaler (×2 converters summed) | `_LIRA-8.pd:564` |
| 4, 0.5, 1 | tri(p) = (|p−0.5|·4 − 1)·0.5 | `_LIRA-8.pd:558–559` |
| 2.65, −0.145, 0.59 | osc tri/scaling trims | `os.triangle~.pd:278–280` |
| −8, −6.28319 | rlpf: atan(1)·(−8) = −2π | `os.triangle~.pd:16,21` |
| 0 22050, 44100 | rlpf args (overridden by samplerate~ at load) | `os.triangle~.pd:15,22` |
| 100 | rlpf cutoff floor (`max~ 100`) | `os.triangle~.pd:282` |
| −1e-37, 1e-37, 1e+37 | hvcc signal-compare denormal hack (port: plain compare) | `os.triangle~.pd:99,105,106` etc. |
| 1001 | `$0` of the standalone (namespace only) | graph.json root |

---

## 9. 32 kHz hazard list

| # | Item | 44.1 kHz value | Correct 32 kHz equivalent | Notes |
|---|---|---|---|---|
| 1 | `delwrite~ $0-mod-delay-1/2` | 5944 samples = 134.739 ms | **4312 samples** (134.739·32 = 4311.65) | Time-preserving. Alternative (preserve samples → 185.75 ms) changes character — not recommended. Clamp vd reads to `[0, buf_ms]`; requested times >134.74 ms are already clamped at 44.1 so behavior matches. |
| 2 | rlpf hardcoded `clip 0 22050` + `/~ 44100` | overridden at load by `samplerate~` | recompute per sample: `coef = exp(−2π·min(fc, Fs/2)/Fs)`, fc ≤ 16000 | Only a hazard if ported from the literal args. |
| 3 | hv.envfollow coeff | `exp(−4.60517/(ms·Fs/1000))` | same formula with Fs = 32000 | Rate-correct in the original. |
| 4 | env~ 512 | 11.61 ms window | keep 512 (16 ms — slightly slower/smoother envelope) **or** 372 for time-preservation | Recommendation: 512 (matches hvcc, block-friendlier). Document the difference. |
| 5 | `23.22` ms line ramps | 1024 samples | keep 23.22 ms (743 samples) | Keep as ms; the 1024-sample alignment is not audible. |
| 6 | polyblep `1/Fs` | load-time 1/44100 | `n = f/32000` live | Widens the correction window; correct behavior. |
| 7 | lop~/hip~ (689, 4000, 1, 3, 10, 20 Hz) | one-pole coefs at 44.1k | `a = exp(−2πf/32000)` (lop), `a = exp(−2πf/32000)` (hip: y = x − x₁ + a·y₁) | Compute coefficients at runtime. |
| 8 | vd~ interpolation + latency | 4-point (cubic, tabread4-style) read; read cannot see data written later in the same Pd block (≤ 1 block @ 64 smp = 1.45 ms lag) | ring buffer, read = write − round(t·Fs/1000), cubic interp; per-sample port may read same-sample writes (≤ 1.45 ms earlier response than Pd — inaudible, but note) | Verify vd~ interpolation order by ear/recording if fidelity-critical. |
| 9 | osc max frequency | mtof(131.22) ≈ 16.0 kHz base, ×(1+fm·src) up to ×3 under modulation — wraps through Nyquist | at 32k Nyquist is 16 kHz: the top tune position already sits at Nyquist; FM wraps through it by design | phasor wrap + `abs()` semantics: fold `f` with `f = |f|` after the (1 + fm·src) multiply, allow wrap-through-zero aliasing to preserve character. |
| 10 | expander env~ dB ref | 100 + 20·log10(RMS) (verified) | identical (rate-free) | silence must clamp to −100 dB (env~ outputs 0). |
| 11 | key.tgl 52 ms tail, notemap note 36–43, all control curves | rate-free | unchanged | — |
| 12 | hyper-lfo mtof rate curve | rate-free | unchanged | max 164.8 Hz LFO is fine at 32k. |

---

## 10. Sensor / hold / seed semantics

### 10.1 Sensor state flow

```
sensor k state (0/1) sources:
  • GUI tgl $0-s-sensor-k (gui objs 7,9,…,21)
  • MIDI: clone notemap instance k ← notein: note 35+k, vel>0 → 1, vel 0 → 0  (sends to tgl's receive $0-r-sensor-k)
  • PC keyboard '1'…'8' via key.tgl (sends to the same receive symbols)
  → tgl emits $0-s-sensor-k → prm.sensor's `r $1-s-sensor-$2` → i (int, stores state)
```

### 10.2 prm.sensor envelope (16 instances = 2 per voice × 4 pairs; ×2 outlets)

```
r $0-s-sensor-k → [i] (stores 0/1) → sel 1 0
   1 → [v $0-attack]  → msg "1 {attack_ms}"  → line~ ┐
   0 → [v $0-release] → msg "0 {release_ms}" → line~ ┴→ gate (0…1 linear ramp)
r $0-trigger → [i] (bang → re-emit stored state → re-ramp with the CURRENT times)
gate_outlet0 = gate·gate        (*~ both inlets)
burst_outlet1 = cos(2π·(gate − 0.25)) / 2     (−~ 0.25 → cos~ → /~ 2; sweeps exactly one cycle per ramp)
```
* Voice osc A is gated by `min(hold + gateA², 1)`; osc B by `min(hold + gateB², 1)`
  — the **hold signal simply adds to the sensor gate** (a sustaining pedestal),
  soft-clipped at 1.
* The burst (one swept cosine cycle per attack/release ramp, peak ±0.5) is
  summed into the pre-gain oscillator mix — the percussive "sensor hit" body.
* `$0-trigger` is sent by **every** lira.voice after its fast/slow toggle
  changes the attack/release values, so toggling any FAST switch re-ramps **all
  16** envelopes with the new times.

### 10.3 $0-attack / $0-release (24 `v` objects, 2 global floats)

* Writers: each lira.voice's `r $0-s-fast-<vA><vB> → t b f → sel 1 0 →
  msg "100 100" / msg "200 8000" → unpack f f → v $0-attack, v $0-release`;
  also the 16 prm.sensor v-objects (same names, same values — they only
  re-store what they read).
* The float pair is **patch-global**: the last FAST toggle to change wins for
  all voices. FAST on → (100, 100) ms; off → (200, 8000) ms.
* Defaults at load: the `v` objects start at 0, but parameters.txt restores the
  fast toggles (all `1`) ≈500 ms after load (gui.link del 500), which fires
  `s-fast-*` → (100, 100) + trigger. Before that, a sensor hit ramps in 0 ms.
  Port default: (100, 100).

### 10.4 Seed

```
gui: loadbang → t b b → (realtime → * 1000 → i → s $0-seed) + (del 200 → realtime right inlet)
voice: r $0-seed → + (f $2 = first voice number nA) → msg "seed {n}" → [random 1001] left inlet ('seed' method)
       loadbang → t b b → (bang [random 1001] → /1000 → *3 → +0.5 → osc~ freq inlet) + (f $2 → + right operand)
```
* The `random 1001` is banged **once per oscillator at load** (8 draws total,
  in voice/osc creation order) with Pd's default PRNG seed; the `seed` message
  (= boot_seed + nA, 4 messages, one per voice instance) arrives after the
  draw and is never followed by another bang → the vibrato rates are
  **deterministic across boots**: `f_vib = 0.5 + 3·u`, u = k/1000. The whole
  `$0-seed` machinery is effectively dormant in the original.
* Port: either fix the 8 rates to the same deterministic sequence (replicate
  Pd's `random` LCG: seed 1234567, `x = (214013·x + 2531011) >> 16 & 0x7fff`
  — verify against Pd if exactness wanted) or simply use 8 arbitrary
  constants in [0.5, 3.5] Hz. Per-osc, not per-voice.

---

## 11. Signal flow summary

```
                        MIDI in (USB)                    PC keys '1'-'8'
                        notes 36..43 ──┐                        │
                                       ▼                        ▼
                    ┌────────── notemap ×8 / key.tgl ×8 ─────────────┐
                    │        (sets sensor toggles 1..8)              │
 LFO-A phasor ──► tri/sqr converters ──► s~ del-tri / del-sqr / sqr-lfo
 (rate mtof(k²/127−75),  LFO-B phasor ──┘   (link: B-rate FM'd ±50% by A-square)
  mtof curve)                     andor: sqr-lfo = A·B  |  (A+B)/2

 4 × lira.voice (pairs 1-2,3-4,5-6,7-8):
   prm.tune ─ freq ──(×(1+fm·src), thru-zero)──► os.triangle~ A ─┐ gateA=min(hold+gateA²,1)
   prm.pitch ─┘(both tunes)      pwm=0.53125+0.025·vib ──►│                 │
   prm.mod ─ fm_depth=(k/127)⁴·2 ─┐                        ├─(sq·s+tri·(1−s)+burst)─┤
   prm.source ─ src (other pair's │                        │                 │
     audio / sqr-lfo / total-fb) ─┘        os.triangle~ B ─┘ gateB=min(hold+gateB²,1)
   prm.sensor ×2 (gate² + cos burst; line~ attack/release from shared $0-attack/release)
   prm.hold (quad) ─ hold=(k/127)² ─► both gates
                    voice_out = gateA·A + gateB·B ──► hip~3 ──► s~ del-<pair>
                              └────────────────────────────► ×1/6 (sum of 4 voices)

                                        ┌────────────────────────────────────────┐
  Σvoices ──► *~ 0.166667 ──► DELAY     │                                        │
                                │       │ in+noise ─► (+g·proc) ─► delwrite 5944 │
                                │       │              ▲                          │
                                │       │ vd~(t + mod) ─ hip1 ─ lop4000 ─ comp ─ exp ─┬─ ×g ─► (fb)
                                │       │   t = 2^(12k/127)·1.45125 ms (clamp 134.7)   └─► monitor
                                │       │ mod = SELF(vd/2→lop689) + LFO(del-tri/sqr), ×(k/127)²·10 ms
                                │       │ bus2 write in = bus1 write sum (series), own t2/mod2
                                │       └────────────────────────────────────────┘
                                │            monitor: (e1+e2)/2 /max(g,1.5) → tanh → ×del-mix + dry·(1−del-mix)
                                ▼
                            MASTER: *drv → hip20 → tanh ─(+ tanh³¹/4)─► hip10 → /clip(d,1,4)
                                    mix with 0.1·dry by dst-mix → ma.tanh → ×vol² → outlet
                                    hip~3 tap ──► s~ $0-total-feedback ──► (prm.source LFO-CV alt)
                                ▼
                        gui outlet~ ──► dac~ 1 + 2 (mono, both channels)
```

---

## 12. CC candidate map (receiver name · range · curve · default)

All knobs are 0…127 unless noted. Defaults are the saved `parameters.txt`
values in gui.link order 1…47.

| # | Parameter | Receiver (set) name | Range | Curve to destination | Default |
|---|---|---|---|---|---|
| 1–8 | TUNE 1…8 | `$0-s-tune-1…8` | 0…127 | note = semi_v + (k/127)(note2_v − semi_v) → mtof (this is the osc freq; FM mult on top) | 0, 78, 74, 36, 27, 57, 60, 51 |
| 9–12 | FAST 12/34/56/78 | `$0-s-fast-12…78` | 0/1 | attack/release = 100/100 (1) or 200/8000 (0) ms; retriggers all sensors | 1, 1, 1, 1 |
| 13–16 | SHARP 12/34/56/78 | `$0-s-sharp-…` | 0…127 | (k/127)² square↔triangle crossfade per pair | 14.377, 43.132, 103.038, 71.887 |
| 17–20 | MOD 12/34/56/78 | `$0-s-mod-…` | 0…127 | (k/127)⁴·2 FM multiplier depth per pair | 0, 0, 0, 0 |
| 21–24 | SOURCE 12/34/56/78 | `$0-s-source-…` | 0/1/2 (hradio) | 0=cross-pair audio, 1=off, 2=LFO-CV | 0, 0, 0, 0 |
| 25–26 | PITCH 1234/5678 | `$0-s-pitch-…` | 0…127 | ×(1.99k/127 + 0.01) on both quads' tune freqs | 66, 65 |
| 27–28 | HOLD 1234/5678 | `$0-s-hold-…` | 0…127 | (k/127)² gate pedestal per quad | 0, 0 |
| 29 | SWITCH | `$0-s-switch` | 0/1 | cross-pair source A/B select | 0 |
| 30 | TOTAL-FB | `$0-s-total-fb` | 0/1 | LFO-CV source: sqr-lfo (0) / total-feedback (1) | 0 |
| 31 | VIBRATO | `$0-s-vibrato` | 0/1 | vibrato depth 0/1 | 1 |
| 32 | DRV | `$0-s-drv` | 0…127 | 10^((3^(1+2k/127)+3)/20) drive | 86 |
| 33 | DST MIX | `$0-s-dst-mix` | 0…127 | k/127 wet/dry into master tanh | 69 |
| 34 | VOL | `$0-s-vol` | 0…127 | (k/127)² | 127 |
| 35 | F-A | `$0-s-f-a` | 0…127 | mtof(127(k/127)²−75) LFO-A rate | 42 |
| 36 | F-B | `$0-s-f-b` | 0…127 | same, LFO-B | 64 |
| 37 | AND/OR | `$0-s-andor` | 0/1 (vradio) | sqr-lfo = product (0) / average (1) | 1 |
| 38 | LINK | `$0-s-link` | 0/1 | LFO-B rate FM by A-square | 1 |
| 39–40 | TIME-1 / TIME-2 | `$0-s-time-1/2` | 0…127 | 2^(12k/127)·1.45125 ms (clamp 134.74) | 94.574, 56.745 |
| 41 | FEEDBACK | `$0-s-feedback` | 0…127 | (k/127)·2^(2k/127) | 54.043 |
| 42 | DEL MIX | `$0-s-del-mix` | 0…127 | k/127 delay wet/dry | 67.553 |
| 43–44 | MOD-1 / MOD-2 | `$0-s-mod-1/2` | 0…127 | (k/127)²·10 ms delay-time mod depth | 40.736, 47.925 |
| 45 | DEL-MOD | `$0-s-del-mod` | 0/1/2 (hradio) | 0=SELF, 1=OFF, 2=LFO | 2 |
| 46 | LFO-WAV | `$0-s-lfo-wav` | 0/1 (hradio) | 0=TRI, 1=SQR | 0 |
| 47 | QUANTIZE | `$0-s-quantize` | 0/1 | tune semitone snap | 0 |
| — | SENSORS 1…8 | `$0-s-sensor-1…8` | 0/1 | gate per voice | 0 (toggles), MIDI notes 36–43 |
| — | CPU / DSP | `$0-s-cpu`, `$0-s-dsp` | — | cosmetic / Pd-internal | — |

Suggested MIDI CC grouping for the rack port (mirrors the gui.link numbering so
parameters.txt-style dumps stay meaningful): CC 1–8 tune, 9–12 fast, 13–16
sharp, 17–20 mod, 21–24 source, 25–26 pitch, 27–28 hold, 29 switch, 30
total-fb, 31 vibrato, 32 drv, 33 dst-mix, 34 vol, 35 f-a, 36 f-b, 37 andor, 38
link, 39–40 time, 41 feedback, 42 del-mix, 43–44 delay-mod depth, 45 del-mod,
46 lfo-wav, 47 quantize, 48–55 sensors 1–8.

---

## 13. hvcc / graph.json notes (where JSON must not be trusted)

* `gen_graph.py` (hvcc pd2hv) **drops** all GUI atom objects (vsl, hsl, tgl,
  hradio, vradio, bng, nbx, cnv — the entire control surface), `v` value
  objects (24, replaced by comment placeholders), `clone`, `notein`, `key`,
  `keyup`, `text define/get`, `realtime`, `makefilename`, `cputime`, and the
  gui.link abstraction (it lives in `patch/abs/gui.link/`, a directory). The
  raw `.pd` is the only source for all control plumbing (§1.2, §5, §10).
* hvcc **folds** the recurring `r → /127 → pow → msg "{x} 23.22" → line~`
  pattern into compound `__line~f` nodes (e.g. prm.hold parses to 4 objects)
  and turns message boxes into `sequence` nodes and control→signal converts
  into `var` nodes. Edge counts therefore differ from the raw graph
  (`xcheck.txt`); master_output matches exactly (49 edges both), delay/voices
  differ only by abstraction inlining.
* Abstraction inlet indices in graph.json agree with Pd's x-position rule
  (lira.voice: 0=mod~, 1=pitch, 2=hold~) — no swap there.
* `s~ $0-del-sqr 0` (square-lfo obj 15) carries a trailing `0` argument —
  ignored by Pd ([send~] takes only a name); not a second bus.

## 14. Remaining ambiguities and resolutions

1. **Abstraction inlet order (pitch/hold swap?)** — raw object order suggested
   hold=pitch swapped. Resolved empirically: Pd orders abstraction inlets by
   **x-position**; the labeled order (0=mod, 1=pitch, 2=hold) is correct and
   the patch loads without the "nonsignal inlet" error. hvcc agrees.
2. **Delay time units (5945 ms vs 5944 samples).** The time curve tops out at
   5945 *ms* while the buffer is 5944 *samples*; Pd clamps `vd~`. Spec'd the
   clamped behavior as ground truth (§2.1) and flagged the author's apparent
   samples-vs-ms confusion; at 32k use a time-preserving 4312-sample buffer.
3. **compressor~ envelope follower** — the embedded hv.envfollow compares the
   attack/release *coefficients* (not signal vs envelope) in its multiplex.
   With the delay's 2.5/2.5 ms this always selects the `env` branch, giving
   `env(n) = |in|(1−env) + env²`. Spec'd patch-exact (§2.6), not the
   music-dsp reference algorithm the comments describe.
4. **expander~ unit chain** — only meaningful if Pd's `[env~]` outputs
   `100 + 20·log10(RMS)`; verified empirically in headless Pd (sine → 96.99).
5. **List-argument order** for `[-]`, `[/]`, `[pow]` message forms (affects
   the expander slope `1 − 1/ratio`, the delay/drive `2^x`/`3^x` bases):
   verified in headless Pd — "1 5" → `[-]` = −4, `[/]` = 0.2, "2 10" →
   `[pow]` = 1024 (first element = left/hot).
6. **`$0-seed` machinery** — arrives after the one-shot vibrato-rate draw and
   is never followed by another bang; rates are deterministic. Spec'd both the
   faithful (Pd LCG sequence) and the pragmatic (8 constants) options (§10.4).
7. **vd~ interpolation order** — Pd uses 4-point interpolation (same family as
   tabread4~); flagged for listen-verification, not verifiable from the patch
   text alone.
8. **`s~ $0-del-sqr 0` trailing arg** — dead argument, ignored (§13).
9. **voice FM topology** — an initial reading suggested a spurious ×2
   multiplier (two connections into the oscillator frequency inlet). Re-checked
   against the parsed connections: the two connections are `f_add` and
   `f_add·(fm·src)` — the FM is a plain thru-zero multiplier `f·(1+fm·src)`
   with no base ×2 (§7.6).
10. **`graph.json` regenerated during analysis** — running `gen_graph.py`
    rewrote `graph.json` with the venv's current hvcc (1.95 MB, byte-identical
    to the git HEAD copy); the brief mentioned a 4.3 MB file. All structure
    cross-checks in `xcheck.txt` were performed against the committed version
    and are consistent with the raw `.pd`; no `graph.json` content was relied
    on where the `.pd` was available.
