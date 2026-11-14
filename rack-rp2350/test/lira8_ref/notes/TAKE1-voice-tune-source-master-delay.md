Now I have the complete picture. Let me nail down the voice math:

Let:
- `T1` = prm.tune(voice a) output (Hz, smoothed 23.22ms)
- `T2` = prm.tune(voice b)
- `V` = vibrato LFO = osc~(rate) * vibratoAmount(0/1 smoothed), rate = 0.5 + 3*rand(0..1000)/1000 ∈ [0.5, 3.5] Hz
- `M` = mod source signal (from prm.source)
- `mod` = (MOD/127)^4 * 2, smoothed 23.22ms
- `sharp` = (SHARP/127)^2, smoothed
- `S1` = sensor A env, `S2` = sensor B env (line~ 0..1, then squared → env^2 output at outlet~ 0; outlet~ 1 = cos((env-0.25)*2π)/2)

Wait, prm.sensor:
- [7] line~ output `e` (0..1 ramp with attack/release ms)
- [8] *~ e*e → outlet~ 0 = e²
- [9] -~ 0.25 → e - 0.25 ; [10] cos~ → cos(2π(e-0.25)) ; [13] /~ 2 → outlet~1 = cos(2π(e-0.25))/2

At e=0: cos(-π/2)/2 = 0. At e=0.5: cos(π/2)/2 = 0. At e=1: cos(1.5π)/2 = 0. Hmm, at e=0.25: cos(0)/2 = 0.5. At e=0.75: cos(π)/2 = -0.5.

So outlet1 = 0.5*cos(2π(e-0.25)) which goes 0 → +0.5 (e=0.25) → 0 (e=0.5) → -0.5 (e=0.75) → 0 (e=1).

This is the "hold crossfade" — interesting. In lira.voice:
- [16] prm.sensor A: outlet0 (e_A²) → [21] min~1 (with hold H) ; outlet1 (cosA) → [4] *~:0
- [55] prm.sensor B: outlet0 (e_B²) → [22] min~1 (with hold H); outlet1 (cosB) → [10] *~:1
- [20] hold H (0..1, pow2 smoothed) → both min~1
- [21] = min(e_A², H) → [4] *~:1
- [4] = cosA * min(e_A², H)
- [22] = min(e_B², H) → [10] *~:0
- [10] = min(e_B², H) * cosB

Then:
- [2] = square1 * sharp
- [3] = triangle1 * (1 - sharp)
- [4] = cosA * min(e_A², H)   ← this is a signal added into [4]?? 

Wait: connect 2 0 → 4 0; connect 3 0 → 4 0; connect 16 1 → 4 0; connect 21 0 → 4 1.

Hmm! [4] is `*~` with left inlet receiving THREE signals: [2] (square1*sharp), [3] (tri1*(1-sharp)), and [16]:1 (cosA). And right inlet from [21] = min(e_A², H).

So [4] = (square1*sharp + tri1*(1-sharp) + cosA) * min(e_A², H).

Interesting — the cosA term is added to the oscillator signal before multiplying by the envelope. That's a DC-ish offset/thump that gives the "sensor" its organic click/breath. Similarly [10] = (square2*sharp + tri2*(1-sharp) + cosB) * min(e_B², H).

Wait let me double check [10]: connect 8 0 → 10 1; connect 9 0 → 10 1; connect 55 1 → 10 1; connect 22 0 → 10 0.
So [10] left = [22] = min(e_B², H), right = [8]+[9]+[55]:1 = osc2mix + cosB. So [10] = min(e_B²,H) * (osc2mix + cosB). Same thing (multiplication commutes). ✓

And [4]: left = [2]+[3]+cosA = osc1mix + cosA, right = min(e_A²,H). ✓ Consistent.

Then [6] = [4] + [10] → voice pair output. Also → hip~3 → s~ del-XY (mod source bus).

FM path:
- [14] = M * mod
- [11] = [33] * [14]  where [33] = T1 + [35], [35] = T1 * (V*0.005)
  So [33] = T1 * (1 + 0.005*V)
- [11] = T1*(1+0.005V) * (M*mod)
- osc1 freq = [33] + [11] = T1*(1+0.005V) * (1 + M*mod)
- Similarly osc2 freq = [34] + [13] where [34] = T2*(1+0.005V), [13] = [14] * [34]
  osc2 freq = T2*(1+0.005V)*(1 + M*mod)

So both oscillators get the same exponential FM index. And PWM inlet = 0.53125 + 0.025*V.

Note: os.triangle~ takes abs(freq) so negative freq folds.

Now prm.tune:
- loadbang: idx = $2 - 1; text get $1-tune idx → row (two floats: lo, hi)
  tune table rows: 
  1: (-16, 93)
  2: (-16, 93)
  3: (7, 109)
  4: (9, 107)
  5: (20, 116.54)
  6: (20, 116.54)
  7: (33, 126.24)
  8: (33, 131.22)
- unpack f f → first = lo → [6] `-` : lo - hi? Let's see: [5] unpack f f: outlet0 → [6] `-`:0 (left), outlet1 → [13] t f f → [6]:1 (right) and [9]:1.
  So [6] = lo - hi (negative range), [13] gives hi to [9] `+`.
- [7] = tune_param/127 ; [8] = (tune/127) * (lo - hi) ; [9] = [8] + hi → MIDI note = hi + (tune/127)*(lo-hi) = hi - (tune/127)*(hi-lo). So tune slider 0 → hi, 127 → lo. 
- [10] mtof → Hz
- [28] `*` with pitch: [27] inlet pitch → [29] t b f → [28]:1; [28] = mtof_hz * pitch
  where pitch = prm.pitch = (P/127)*1.99 + 0.01 ∈ [0.01, 2.0]
- [18] t f f → [19] f (unquantized) and [15] ftom → [16] +0.5 → [17] i (round) → [26] mtof → [20] f (quantized)
- [21] quantize param → t f f → [25] sel 0 1 → bangs [19] (unquant) or [20] (quant); also sets [24] route's arg.
  Actually [21] t f f: outlet0 → [25] sel 0 1; outlet1 → [24] route:1 (set route argument).
  [22] msg "1 $1" ← from [20] (quantized) → [24] route (left inlet)
  [23] msg "0 $1" ← from [19] (unquantized) → [24] route
  route with arg = quantize value: if quantize==1, "1 x" matches → outputs x; if quantize==0, "0 x" matches → outputs x. Clever.
- [14] msg "$1 23.22" → line~ (23.22 ms smoothing) → outlet~

Important ordering: when quantize changes, [25] sel triggers the corresponding stored value. When tune changes, both [19] and [20] get updated (via t f f), and messages "0 x" and "1 x" are sent to route; only the matching one passes. Note the order: [18] t f f → outlet0 → [19] f → "0 x" → route; outlet1 → [15] ftom → ... → [20] f → "1 x" → route. Both fire; route passes only matching. Fine.

So: tune Hz = mtof(round(ftom(hz*pitch) )) if quantize else hz*pitch. Note ftom/mtof roundtrip: ftom(hz) = 69 + 12*log2(hz/440); rounding to nearest semitone. mtof(m) = 440*2^((m-69)/12).

Pd's mtof: `if (f <= -1500) return 0; else if (f > 1499) f = 1499; return 8.17579891564 * exp(0.0577622650 * f)`. Note Pd's mtof uses 8.17579891564 * exp(0.057762265 * m) which equals 440*2^((m-69)/12) approximately. And ftom: `return (f > 0 ? 17.3123405046 * log(.12231220585 * f) : -1500)`. Let me verify: .12231220585 * 440 = 53.817... log(53.817) = 3.9856; *17.3123 = 69.0 ✓.

For a faithful port I should use Pd's exact formulas (they differ in the last bits from the standard). I'll use Pd's constants.

Now prm.source (per pair):
args: $1=id, $2$3 = own pair (e.g. "12"), $4$5 = other pair A, $6$7 = other pair B
- in1 = r~ del-$4$5 (other pair's sum), gated by (switch==0)
- in2 = r~ del-$6$7 (own pair's sum? no...) 

Let's map: prm.source $0 1 2 3 4 7 8 → own=12, $4$5=34, $6$7=78.
- [32] r~ $1-del-$4$5 = del-34 → [7] *~ gate(switch==0)
- [31] r~ $1-del-$6$7 = del-78 → [9] *~ gate(source==0)
- [29] r~ sqr-lfo → [12] *~ gate(total-fb==0)
- [30] r~ total-feedback → [4] *~ gate(source==2)
- [7] and [9] → [1] *~ gate(source==0)... 

wait: [0] `== 0` ← [15] r source-$2$3. [16] *0.999 +0.001 → [1]:1. And [7]→[1]:0, [9]→[1]:0. So [1] = (del34*g_switch + del78*g_src0) * g_src0.

Hmm, that means del-78 gets multiplied by g_src0 twice (once in [9], once in [1]).
- [9] = del78 * (src==0 ? 0.999+0.001 : 0.001)
- [7] = del34 * (switch==0 ? 1.0 : 0.001)
- [1] = ([7] + [9]) * (src==0 ? 1.0 : 0.001)
- [12] = sqrlfo * (totalfb==0 ? 1.0 : 0.001)
- [4] = totalfeedback * (totalfb==0 ? 0.001 : 1.0)
- [2] = [4] * (src==2 ? 1.0 : 0.001)
- out = [1] + [2]

So:
- source==0 (other pair): out ≈ del34*g_switch + del78*1.0 ; and if switch==1, del34 nearly gone.
- source==1 (OFF): out ≈ 0.001*(del34*g_switch + del78*0.001) + 0.001*totalfb ≈ tiny
- source==2 (LFO:CV): out ≈ 0.001*(...) + totalfb*g_totalfb

Where g_totalfb = (total-fb==0 ? 0.001 : 1.0) for [4], and sqrlfo path [12] gated by (totalfb==0 → 1.0). But [12] output goes to [2]:0 along with [4]. So [2] = (sqrlfo*g_notfb + totalfb*g_fb) * g_src2.

So in LFO:CV mode: if TOTAL-FB is off → square LFO; if on → master total feedback. That matches the Lyra-8's "LFO/CV" with the total feedback switch.

And SWITCH: when switch==1, the cross-pair modulation del34 is suppressed (only del78 remains). Interesting: Lyra's "switch" changes the pairing.

Hmm wait, for pair 12: $4$5=34 and $6$7=78. For pair 34: $4$5=12, $6$7=12. For pair 56: $4$5=78, $6$7=34. For pair 78: $4$5=56, $6$7=56.

So pair 34's both are del-12. OK.

The 0.999/0.001 trick avoids exact zero (denormal/zipper avoidance) — I'll replicate as gain constants (no smoothing since they're control-rate multiplications by constants... in Pd these are float multiplications into *~ right inlet, which in Pd updates at block boundaries with no interpolation for *~ right inlet float? Actually *~ with a float right inlet: when a float arrives, Pd's *~ uses the "scalar" version and it jumps at block boundary. Yes, no smoothing.)

Now the master_output:
- in = delay output
- drv param D: d = D/127; e = 2*d + 1; p = 3^e (msg "3 $1" → pow) → wait [36] msg "3 $1" → [9] pow: pow(3, e). Then [38] + 3 → pow(3,e)+3; [10] + 100 → +100; [11] dbtorms → amp = 10^((pow(3, 2d+1)+3+100-100)/20) = 10^((3^(2d+1)+3)/20).
  At d=0: 3^1=3, +3=6 dB → amp=1.995. At d=1: 3^3=27, +3=30 dB → amp=31.6.
- drive gain G = that amp, smoothed 23.22ms
- [15] = in * G
- [31] hip~ 20 (DC blocker)
- [16] ma.tanh~ → t = tanh_approx(highpass20(in*G))
- [30] pow~ 31 → t^31 ; [29] /~ 4 → t^31/4 ; [28] = t + t^31/4
- [26] hip~ 10
- [43] clip~ 1 4 of G → [41] 1/clip(G,1,4) → [42] = hip10(t + t^31/4) / clip(G,1,4)
- [32] = in * 0.1 (dry-ish path)
- [33] = in*0.1 + [42]
- [21] = [33] * mix   (mix = dst-mix/127, smoothed)
- [24] = in * (1 - mix)
- [25] = [24] + [21]
- [34] ma.tanh~ → out_pre
- [3] = out_pre * vol where vol = (V/127)^2 * 1, smoothed
- [44] hip~ 3 → s~ total-feedback

Note: pow~ 31 with negative base: Pd's pow~ computes pow(x,y) — for negative x and integer-valued y=31 it's fine (returns negative). Actually Pd's pow~ uses `powf`? In Pd, pow~ performs `pow(x, y)` via C pow which handles negative base with integer exponent correctly. Yes.

Now the delay:
- input = sum of 4 voice pairs * 0.166667
- noise: noise~ * 0.001 added
- [19] = in + noise*0.001 → feeds both delay lines' input summing nodes
- Delay 1: 
  - time param t1: x = (T/127)*12; ms = 2^x * 1.45125 → at T=0: 1.45125 ms; at T=127: 2^12*1.45125 = 5944.32 ms. delwrite~ size 5944 ms. ✓
  - mod source: 
    - [33] vd~ mod-delay-1 (unconnected input → 0 delay → effectively 1-sample) → [98] /~2 → [39] *~ (del-mod==0 ? 1:0) → [100] lop~ 689 → [45] +~
    - [82] vd~ mod-delay-2 (unconnected → 1 sample) → [97] /~2 → [99] lop~ 689 → [71] *~ (del-mod==0) → [74] +~ (for delay 2)
    
    Wait: [100] lop~689 → [45]+~:0 (delay1's mod sum) and [99] lop~689 → [71]*~:0 → [74]+~:1 (delay2's mod sum). So delay1 self-mod reads delay1, delay2 self-mod reads delay2. ✓ ("SELF")
    - LFO: [102] r~ del-tri * (lfo-wav==0) + [103] r~ del-sqr * (lfo-wav==1) → [107] +~ → [40] *~ (del-mod==2) → [45]+~:1 (delay1) and [74]+~:0 (delay2)
  - [45] = selfmod1 + lfomod ; [46] = [45] * mod1amt where mod1amt = (MOD1/127)^2 * 10 (ms)
  - [53] = time1_ms + [46] → vd~ mod-delay-1 (4-point interpolating read)
  - [31] hip~ 1 → [32] lop~ 4000 → compressor~ (2.5ms attack, 2.5ms release, -12dB threshold, ratio 5) → expander~ (-60, 5) → [30] *~ feedbackGain
  - feedback = (F/127)^2 * ... : [22] x=F/127 → t f f → [24] *2 → [25] "2 $1" → [26] pow → 2^(2x); [27] * with x → x * 2^(2x). At F=127: 1*4 = 4. At F=64: 0.504*2^1.008=1.016. So feedback can exceed 1 (up to 4) — that's why the compressor is there!
  - [8] = [19] + [30] → delwrite~ mod-delay-1
- Delay 2: same with time2, mod2.
- Wet: [87] = expander1out + expander2out → [101] /~2 → [113] *~ (1/max(feedback,1.5)) → [108] ma.tanh~ → [95] *~ mix
- Dry: [90] = in * (1-mix)
- out = dry + wet

Note the wet is taken BEFORE the feedback gain multiply (post compressor/expander). Good.

compressor~:
- hv.envfollow with attack/release ms: coeff = exp(-4.60517 / (Fs * time_ms/1000))... let me re-read: 
  [15] / 1000: samplerate/1000. Then coeff subpatch: inlet = time_ms; [6] max 1 → [5] msg "-4.60517 $1" → [4] `/` → -4.60517/time_ms → [1] exp → exp(-4.60517/time_ms) → line~ 10ms → outlet.
  
  Hmm wait, that gives coeff = exp(-4.60517/time_ms) which doesn't involve Fs. But the comment says coeff = exp(-1/(Fs*time_ms)). Let me re-read the connections in hv.envfollow:
  - [3] loadbang → [4] samplerate~ → [15] / 1000 → [5]:1 and [8]:1
  - [23] inlet attack → [5] `*` : attack_ms * (Fs/1000) = attack in samples
  - [24] inlet release → [8] `*` : release_ms * Fs/1000
  - [5] → [20] pd coeff (inlet = time in SAMPLES) → coeff = exp(-4.60517/samples) ✓ = exp(-ln(100)/ (Fs*t)) → 99% settle time (4.60517 = ln(100)).
  - [8] → [19] pd coeff → release coeff
  - [18] abs~ of input → temp
  - [22] hv.multiplex(a=attackCoeff, b=releaseCoeff, lhs=temp, rhs=env): out = (temp>env) ? attackCoeff : releaseCoeff
    Actually multiplex: input-a * (lhs>rhs) + input-b * !(lhs>rhs). With input-a=[20] attack coeff, input-b=[19] release coeff, lhs=temp, rhs=env(from r~ $0-env).
    So coeff = temp > env ? attackCoeff : releaseCoeff ✓
  - [0] = 1 - coeff ; [6] = temp * (1-coeff) ; [2] rpole~ with coeff → env = temp*(1-c) + c*env_prev ✓
  - s~ $0-env feedback (one block delay via send~/receive~)
- Then compressor: 
  - [2] = envfollow_out - threshold_amp (dbtorms(threshold+100))
  - [6] = [2] * (1/ratio)
  - [10] = [6] + threshold_amp
  - [11] = [10] / envfollow_out
  - [12] clip~ 0 1
  - [22] ma.tanh~ → gain
  - [13] = in * gain
  
  Note: threshold_amp = dbtorms(-12+100) = dbtorms(88) = 10^((88-100)/20) = 0.2512.
  Wait, dbtorms in Pd: `dbtorms` converts dB to RMS where 100dB = 1.0. So dbtorms(88) = 10^((88-100)/20) = 0.2512 ✓.
  
  So gain = tanh(clip((thresh + (env - thresh)/ratio)/env, 0, 1)). When env <= thresh → ratio value > 1 → clipped to 1 → gain = tanh(1) = 0.7616. Hmm! That means even below threshold the gain is 0.7616, not 1. That's a quirk of this design (ma.tanh~ of 1 = 0.7616). Faithful port must keep it.

expander~:
- [10] env~ 512 (RMS in dB, Pd's env~ outputs dB where 1.0 = 100dB)
- [11] - 100 → dB relative to unity
- [14] `- -20` → x - (-20) = x + 20 (threshold -20 dB)
- [13] * 2 → (x+20)*2  (ratio 2? actually expansion factor)
- [12] min 0
- [18] msg "$1 11" → line~ (11 ms)
- [16] +~ 100 → back to Pd dB
- [17] dbtorms~ → linear gain
- [5] = in * gain

Wait: [1] inlet (second) → [14]:1 (the threshold, -20); [4] inlet (third) → [9] max 1 → [2] "1 $1" → [3] / → 1/ratio → [7] "1 $1" → [8] `-` → 1 - 1/ratio... then [8] → [13]:1.

Hmm: [13] `* 2` with right inlet from [8] = 1 - 1/ratio. With ratio=5: 1-0.2 = 0.8. So [13] = (x+20)*0.8. Then min 0. So it's a downward expander with slope 0.8 below -20dB.

Hmm, but the creation arg `* 2` is overridden by the connection. Yes → 0.8.

So expander gain_dB = min(0, (envdB - thresh)*0.8) where envdB = env~512 output - 100.

env~ 512: Pd's env~ computes RMS over a window of 512 samples with... Pd's env~ uses a Hanning window? Let me recall: Pd's env~ computes the mean square over the analysis period with overlapping? Pd's env~ implementation (d_filter.c sigenv_perform): it accumulates x² over blocks, with window size npoints, and uses a "Hanning window"? 

Actually Pd's env~: "env~ - envelope follower. The output is in dB (100 = unity gain), computed over a window of the given size (default 1024). The analysis is done every window/2 samples (50% overlap)." And it applies a Hanning window? Let me check Pd source: 

```c
static t_int *sigenv_perform(t_int *w) {
    ...
    for (i = 0; i < n; i++) {
        t_sample f = in[i];
        sum += f * f;
    }
    ...
}
```
Hmm, I recall Pd's env~ does use a Hanning window in newer versions. Let me look at actual Pd source to be accurate. In d_filter.c:

```c
typedef struct sigenv {
    t_object x_obj;
    t_float x_f;
    t_sample *x_buf;
    int x_phase;
    int x_npoints;
    int x_allocforvs;
    t_outlet *x_outlet;
    t_float x_result;
    ...
} t_sigenv;

static t_int *sigenv_perform(t_int *w)
{
    ...
    t_sample *in = ...;
    ...
    if (x->x_phase) { ... }
    ...
}
```

I'm not 100% sure. This level of detail matters for exactness but the expander is a subtle gain effect. I'll implement env~ as RMS over 512 samples with 50% overlap (hop 256) and Hanning weighting per Pd's actual implementation — I should verify. Let me check Pd's source online.

Actually — I should be pragmatic. The user asked for a C++ port. Perfect bit-exactness with Pd's env~ internals is a nice-to-have; I should aim for faithful behavior and document deviations. But let me try to get env~ right since it's cheap to check.

Let me now decide on asking the user. I have a thorough understanding. Key ambiguities:
1. Deliverable form: (a) portable C++17 DSP core + offline WAV render CLI (no deps), (b) real-time standalone app (needs audio lib), (c) JUCE audio plugin, (d) something else (Daisy/embedded).
2. Whether they want the GUI too.
3. Fidelity: block-accurate Pd emulation vs clean per-sample.

I'll ask 3 questions. Given the effort, asking is right. Let me ask concisely with good defaults.

Actually, let me reconsider — maybe just 2-3 questions max, and offer a recommended default. Let me ask:

Q1: What should the C++ port target? options: core lib + WAV CLI (recommended), realtime standalone (RtAudio/miniaudio), JUCE plugin (VST3/AU/standalone), embedded/Daisy.
Q2: Fidelity: faithful Pd emulation (64-sample blocks, Pd line~/env~/filter quirks) vs modernized per-sample clean DSP.
Q3: GUI/controls needed? options: no GUI, params via API/CLI + MIDI; simple terminal/keyboard control; full GUI later.

Hmm, Q3 might be overkill. Let me do Q1, Q2, and Q3 about parameter/preset handling maybe. I'll keep 3.

Let me ask now.