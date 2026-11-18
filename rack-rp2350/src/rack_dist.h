// Rack distortion stage — shared by Rings (post-resonator) and Elements
// (pre-reverb insert). Origin: the four fast algorithms from ctag-tbd's
// TBDings/Elements distortion stage (commits 2166c84c, 4f086145, a62ad652),
// plus the Swedish Chainsaw (FX 5 "SWEDE") from Barabas5532/SwedishChainsaw.
//
// Ported into a pure C++ header (no MCU headers) so it is host-testable and
// can run in the adapters' flash-resident render loops.
//   type 0 tube  : SoftClip(k*x)/SoftClip(k) — punch-preserving glue
//   type 1 fold  : Vult/Befaco wavefolder Fold_do()
//   type 2 fuzz  : biased asymmetric SoftClip with bounded normalization
//   type 3 shape : TBD03's seeded braids SignatureWaveshaper curve
// A 1-pole tone LP and dry/wet blend follow the shaper. drive == 0 is an
// exact zero-cost bypass. The 257-entry int32 shape table is FILE-STATIC
// (function-local static), never in the arena-placed Rings State — the rack
// pool has only 128 B spare in the worst chain.
#ifndef RACK_SRC_RACK_DIST_H_
#define RACK_SRC_RACK_DIST_H_

#include <stddef.h>
#include <stdint.h>

#include <cmath>

// q15 dual-MAC for the speaker FIR (the M33 DSP extension — Yiu ch.19.4):
// the FIR accumulates in 64 BIT (SMLALD/SMLALDX on the board, plain C on
// the host — bit-exact, so the host tests validate the board's path).
// 64-bit accumulate = NO sum|ir16| cap: the coefficients use FULL 15-bit
// scale (peak at 32767) and the accumulator cannot overflow
// (sum|ir16| x 32767 ~= 1.2e10 << 2^63). The int32-SMLAD experiment
// (scale so sum|ir16| <= 32700) squeezed the coefficients to ~12 bits —
// audible grit on sustained tones; the 64-bit form cuts the coefficient
// quantization noise ~11x. SMLALD = two MACs accumulating into a
// register PAIR, SMLALDX = the same with Xm's halves swapped, PKHBT =
// pack (top from Rn, bottom from Rm). GAS (2.4x) notes: SMLAD/SMLADX
// parse ONLY in the 4-operand form ("bad arguments" otherwise, Windows
// 2.43.1 AND Linux 2.47); the no-shift `pkhtb` mnemonic canonicalizes to
// PKHBT with its operands SWAPPED — spell `pkhbt` for the unshifted pack.
static inline int64_t rack_smlald_acc(int64_t acc, int32_t a, int32_t b) {
  const uint32_t ua = (uint32_t)a, ub = (uint32_t)b;
  const int16_t al = (int16_t)(ua & 0xffffu);
  const int16_t ah = (int16_t)((ua >> 16) & 0xffffu);
  const int16_t bl = (int16_t)(ub & 0xffffu);
  const int16_t bh = (int16_t)((ub >> 16) & 0xffffu);
  return acc + ((int64_t)al * bl + (int64_t)ah * bh);
}
static inline int64_t rack_smlaldx_acc(int64_t acc, int32_t a, int32_t b) {
  const uint32_t ua = (uint32_t)a, ub = (uint32_t)b;
  const int16_t al = (int16_t)(ua & 0xffffu);
  const int16_t ah = (int16_t)((ua >> 16) & 0xffffu);
  const int16_t bl = (int16_t)(ub & 0xffffu);
  const int16_t bh = (int16_t)((ub >> 16) & 0xffffu);
  return acc + ((int64_t)al * bh + (int64_t)ah * bl);
}
#if defined(__ARM_FEATURE_DSP) && (defined(__arm__) || defined(__thumb__))
static inline uint32_t rack_pkhtb(uint32_t hi_src, uint32_t lo_src) {
  uint32_t r;
  __asm__("pkhbt %0, %1, %2" : "=l"(r) : "l"(hi_src), "l"(lo_src));
  return r;
}
#else
static inline uint32_t rack_pkhtb(uint32_t hi_src, uint32_t lo_src) {
  return (hi_src & 0xffff0000u) | (lo_src & 0xffffu);
}
#endif
#define RACK_SMLALD_ACC(acc, a, b) rack_smlald_acc((acc), (a), (b))
#define RACK_SMLALDX_ACC(acc, a, b) rack_smlaldx_acc((acc), (a), (b))
#define RACK_PKHTB(hi_src, lo_src) rack_pkhtb((hi_src), (lo_src))

// Per-TU SRAM-placement hook: the including adapter may define
// RACK_DIST_MARK(token) (-> ELEMENTS_RAM_CODE) BEFORE including this header
// to land the hot dispatch + cab IR in its tc_* SRAM window. The header is
// included by TWO TUs (elements + rings adapters); a shared section name
// across both would collide (see ram_code.h's token rule), so the default is
// a no-op — the rings TU's copies stay in flash (rings never selects swede,
// and its four fast algos keep their own placement decision).
#ifndef RACK_DIST_MARK
#define RACK_DIST_MARK(token)
#endif

#include "speaker_ir.h"  // swede cab IR (rodata; see tools/ir2array.py)

namespace rackdist {

enum Type : uint8_t {
  kTube = 0, kFold = 1, kFuzz = 2, kShape = 3,
  kSwedish = 4,  // Swedish Chainsaw: HM-2 EQ + cascaded tanh + FMV B/M/T stack
  kWarps = 5     // Elements-only: the T9 stage inside elements::Part::Process
                 // (carrier = main, mod = raw exciter). This adapter skips
                 // rackdist::process(); Rings never selects it (no window room).
};

struct State {
  float lp[2];  // one-pole tone state, per channel
};

struct Params {
  uint8_t type;  // 0..4 (kWarps never reaches this adapter)
  float drive;   // 0..1; 0 = zero-cost bypass
  float bias;    // 0..1; fuzz asymmetry floor (UI removed — stays 0)
  float tone;    // 0..1; 1 = open
  float bass;    // 0..1; swede FMV bass (plugin default 0.05)
  float mid;     // 0..1; swede FMV middle (plugin default 0.5)
  float treble;  // 0..1; swede FMV treble (plugin default 0.5)
  // No wet/dry mix: drive == 0 is a zero-cost bypass, so DRIVE alone is the
  // distortion-amount control. The stage is always fully wet when engaged.
};

static inline float clamp01(float x) {
  return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

// stmlib::SoftClip — 1 divide, no libm (lib/stmlib/stmlib/dsp/dsp.h).
static inline float softclip(float x) {
  if (x < -3.0f) return -1.0f;
  if (x > 3.0f) return 1.0f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// ---- Swedish Chainsaw (Barabas5532/SwedishChainsaw, GPLv3) ---------------
// The Entombed/Dismember "buzzsaw" is the Boss HM-2 graphic EQ with all four
// bands maxed: three fixed peaking biquads (MATLAB coefficients from the
// plugin's HM2Filter.cpp) wrapped in two saturation stages, then the JCM800
// FMV bass/middle/treble tonestack (the plugin's FMVDesign.cpp, ported here
// verbatim-in-math) and the speaker cab convolution. The clippers are
// stmlib::SoftClip — the classic tanh approximation (~2.5% knee deviation at
// moderate drive, identical saturation beyond |x| = 2.5; the upstream's
// waveshape() is std::tanh). Not ported: 4x oversampling (aliasing vs CPU)
// and the TIGHT toggle (upstream default OFF == the fixed 60 Hz HP below).
//
// The HM-2 bands are REDESIGNED at the rack's 32 kHz: the plugin's fixed
// digital coefficients were designed at 44.1 kHz — solving (f0, gain, Q) from
// them gives 86 Hz +20.0 dB Q1.073 / 900 Hz +11.1 dB Q1.789 / 1400 Hz +19.1 dB
// Q0.980 (round-trip-exact RBJ), and running them at 32 kHz pitched every
// band 0.726x low (62/653/1016 Hz). Designing at 32 kHz restores the true
// band centers.
struct SwedishState {
  float hp_y[2];        // input high-pass output state
  float hp_x[2];        // input high-pass input state
  float bq[2][3][2];    // per-channel biquad z1/z2 for the 3 HM2 bands
  float pre_lp[2];      // 7 kHz post-HM2 one-pole
  float amp_lp[2];      // 7 kHz pre-amp one-pole (the upstream BUG-faithful
                        // stage, see swedish_process)
  float fmv_z[2][3];    // per-channel FMV tonestack z1/z2/z3 (3rd-order DF2T)
  // The dB gains are cached: the powf pair is libm from XIP (~us-scale per
  // call) and the knobs move rarely — recompute on drive change only.
  float last_drive = -1.0f;
  float pedal_gain = 0.0f;
  float amp_gain = 0.0f;
};
static inline SwedishState& swedish_state() {
  static SwedishState s{};
  return s;
}

struct SwedishBiquad { float b0, b1, b2, a1, a2; };

// RBJ peaking-EQ design (compute-once statics below).
static inline SwedishBiquad swedish_peaking(float f0, float gain_db, float Q,
                                            float fs) {
  const float A = powf(10.0f, gain_db / 40.0f);
  const float w0 = 6.2831853f * f0 / fs;
  const float alpha = sinf(w0) / (2.0f * Q);
  const float a0 = 1.0f + alpha / A;
  SwedishBiquad c;
  c.b0 = (1.0f + alpha * A) / a0;
  c.b1 = (-2.0f * cosf(w0)) / a0;
  c.b2 = (1.0f - alpha * A) / a0;
  c.a1 = c.b1;  // RBJ peaking: b1 == a1
  c.a2 = (1.0f - alpha / A) / a0;
  return c;
}
static inline const SwedishBiquad* swedish_bands() {
  static const SwedishBiquad k[3] = {
      swedish_peaking(86.0f, 20.0f, 1.073f, 32000.0f),
      swedish_peaking(900.0f, 11.13f, 1.788f, 32000.0f),
      swedish_peaking(1400.0f, 19.08f, 0.980f, 32000.0f),
  };
  return k;
}

// Fixed HM2 bands: b0,b1,b2,a1,a2 (a0 = 1 in all three).
static inline void swedish_biquad(float* z, float x,
                                  const SwedishBiquad& c, float& y) {
  y = c.b0 * x + z[0];
  z[0] = c.b1 * x - c.a1 * y + z[1];
  z[1] = c.b2 * x - c.a2 * y;
}

// ---- JCM800 FMV tonestack (Barabas5532/SwedishChainsaw FMVDesign.cpp,
// GPLv3, (C) 2019 Barabas Raffai) — math ported verbatim -------------------
// The analog 3rd-order transfer function of the JCM800 bass/middle/treble
// network (R1 220k, R2 1M, R3 22k, R4 33k, C1 470pF, C2 C3 22nF); the knobs
// scale RC time-constant products. Bilinear transform at the rack rate,
// normalized by A0 for direct-form use.
struct SwedishFmvCoefs { float b0, b1, b2, b3, a1, a2, a3; };
static inline void swedish_fmv_design(float l, float m, float t, float fs,
                                      SwedishFmvCoefs* out) {
  const float R1 = 220e3f, R2 = 1e6f, R3 = 22e3f, R4 = 33e3f;
  const float C1 = 470e-12f, C2 = 22e-9f, C3 = 22e-9f;

  const float b1 = t * C1 * R1 + m * C3 * R3 + l * (C1 * R2 + C2 * R2) +
      (C1 * R3 + C2 * R3);

  const float b2 = t * (C1 * C2 * R1 * R4 + C1 * C3 * R1 * R4)
      - m * m * (C1 * C3 * R3 * R3 + C2 * C3 * R3 * R3)
      + m * (C1 * C3 * R1 * R3 + C1 * C3 * R3 * R3 + C2 * C3 * R3 * R3)
      + l * (C1 * C2 * R1 * R2 + C1 * C2 * R2 * R4 + C1 * C3 * R2 * R4)
      + l * m * (C1 * C3 * R2 * R3 + C2 * C3 * R2 * R3)
      + (C1 * C2 * R1 * R3 + C1 * C2 * R3 * R4 + C1 * C3 * R3 * R4);

  const float b3 = l * m * (C1 * C2 * C3 * R1 * R2 * R3 +
                            C1 * C2 * C3 * R2 * R3 * R4)
      - m * m * (C1 * C2 * C3 * R1 * R3 * R3 + C1 * C2 * C3 * R3 * R3 * R4)
      + m * (C1 * C2 * C3 * R1 * R3 * R3 + C1 * C2 * C3 * R3 * R3 * R4)
      + t * C1 * C2 * C3 * R1 * R3 * R4 - t * m * C1 * C2 * C3 * R1 * R3 * R4
      + t * l * C1 * C2 * C3 * R1 * R2 * R4;

  const float a1 = (C1 * R1 + C1 * R3 + C2 * R3 + C2 * R4 + C3 * R4)
      + m * C3 * R3 + l * (C1 * R2 + C2 * R2);

  const float a2 = m * (C1 * C3 * R1 * R3 - C2 * C3 * R3 * R4 +
                        C1 * C3 * R3 * R3 + C2 * C3 * R3 * R3)
      + l * m * (C1 * C3 * R2 * R3 + C2 * C3 * R2 * R3)
      - m * m * (C1 * C3 * R3 * R3 + C2 * C3 * R3 * R3)
      + l * (C1 * C2 * R2 * R4 + C1 * C2 * R1 * R2 + C1 * C3 * R2 * R4 +
             C2 * C3 * R2 * R4)
      + (C1 * C2 * R1 * R4 + C1 * C3 * R1 * R4 + C1 * C2 * R3 * R4 +
         C1 * C2 * R1 * R3 + C1 * C3 * R3 * R4 + C2 * C3 * R3 * R4);

  const float a3 = l * m * (C1 * C2 * C3 * R1 * R2 * R3 +
                            C1 * C2 * C3 * R2 * R3 * R4)
      - m * m * (C1 * C2 * C3 * R1 * R3 * R3 + C1 * C2 * C3 * R3 * R3 * R4)
      + m * (C1 * C2 * C3 * R3 * R3 * R4 + C1 * C2 * C3 * R1 * R3 * R3 -
             C1 * C2 * C3 * R1 * R3 * R4)
      + l * C1 * C2 * C3 * R1 * R2 * R4 + C1 * C2 * C3 * R1 * R3 * R4;

  // bilinear transform (c = 2*Fs), A0-normalized.
  const float c = 2.0f * fs, c2 = c * c, c3 = c2 * c;
  const float B0 = -b1 * c - b2 * c2 - b3 * c3;
  const float B1 = -b1 * c + b2 * c2 + 3.0f * b3 * c3;
  const float B2 = b1 * c + b2 * c2 - 3.0f * b3 * c3;
  const float B3 = b1 * c - b2 * c2 + b3 * c3;
  const float A0 = -1.0f - a1 * c - a2 * c2 - a3 * c3;
  const float A1 = -3.0f - a1 * c + a2 * c2 + 3.0f * a3 * c3;
  const float A2 = -3.0f + a1 * c + a2 * c2 - 3.0f * a3 * c3;
  const float A3 = -1.0f + a1 * c - a2 * c2 + a3 * c3;
  out->b0 = B0 / A0;
  out->b1 = B1 / A0;
  out->b2 = B2 / A0;
  out->b3 = B3 / A0;
  out->a1 = A1 / A0;
  out->a2 = A2 / A0;
  out->a3 = A3 / A0;
}

// 3rd-order transposed direct-form II section (a0-normalized).
static inline void swedish_fmv_biquad3(float* z, float x,
                                       const SwedishFmvCoefs& c, float& y) {
  y = c.b0 * x + z[0];
  z[0] = c.b1 * x - c.a1 * y + z[1];
  z[1] = c.b2 * x - c.a2 * y + z[2];
  z[2] = c.b3 * x - c.a3 * y;
}

// One-pole high-pass / low-pass (RC at 32 kHz Rings rate).
static inline float swedish_hp(float& y, float& xp, float x, float a) {
  y = a * (y + x - xp);
  xp = x;
  return y;
}
static inline float swedish_lp(float& y, float x, float a) {
  y += a * (x - y);
  return y;
}

// One stereo block through the pedal core. `drive` = pedal gain;
// bass/mid/treble = the JCM800 FMV tonestack knobs (plugin defaults 0.05 /
// 0.5 / 0.5). The input HP is the fixed 60 Hz circuit-measured one (the
// plugin's "tight" 700 Hz switch is UI-only and was removed with D BIAS).
// State is a singleton: there is exactly one Rings adapter instance, and
// keeping it static avoids growing the arena-placed State (chain rings+clouds
// has only ~128 B spare).
RACK_DIST_MARK(dist_swedish)
static inline void swedish_process(float* out, float* aux, size_t n,
                                   float drive, float bass, float mid,
                                   float treble) {
  SwedishState& st = swedish_state();
  const float a_hp = 1.0f / (1.0f + 6.2831853f * 60.0f / 32000.0f);
  const float a_lp = 6.2831853f * 7000.0f / 32000.0f;
  // Upstream-faithful drive ranges (PluginProcessor.cpp): pedalGain 20-40 dB,
  // ampGain 0-30 dB (defaults 30/15 land at drive 0.5 on the diagonal the
  // single DRIVE knob sweeps). Upstream applies them as juce gain dB values.
  if (drive != st.last_drive) {
    st.pedal_gain = powf(10.0f, (20.0f + 20.0f * clamp01(drive)) / 20.0f);
    st.amp_gain = powf(10.0f, 30.0f * clamp01(drive) / 20.0f);
    st.last_drive = drive;
  }
  const float pedal_gain = st.pedal_gain;
  const float amp_gain = st.amp_gain;
  SwedishFmvCoefs fmv;
  swedish_fmv_design(clamp01(bass), clamp01(mid), clamp01(treble), 32000.0f,
                     &fmv);
  const SwedishBiquad* bands = swedish_bands();
  float* chans[2] = {out, aux};
  for (int c = 0; c < 2; ++c) {
    for (size_t i = 0; i < n; ++i) {
      float x = swedish_hp(st.hp_y[c], st.hp_x[c], chans[c][i], a_hp);
      x = softclip(x * pedal_gain);                    // pedal clip
      float y;
      swedish_biquad(st.bq[c][0], x, bands[0], y);
      swedish_biquad(st.bq[c][1], y, bands[1], y);
      swedish_biquad(st.bq[c][2], y, bands[2], y);
      y = swedish_lp(st.pre_lp[c], y, a_lp);
      // BUG-FAITHFUL to the plugin's AUDIBLE chain: its prepare() intends a
      // 600 Hz amp-input high-pass but the assignment is aimed at
      // ampInputFilter with makeLowPass(7000) (copy-paste slip), overwriting
      // the HP — while ampLowpassFilter (the "removes harshness" LP) never
      // gets coefficients = identity. Net effect: the HM-2 output is
      // band-limited at 7 kHz TWICE into the amp clip. Ported as heard.
      y = swedish_lp(st.amp_lp[c], y, a_lp);
      y = softclip(y * amp_gain);                      // amp clip
      swedish_fmv_biquad3(st.fmv_z[c], y, fmv, y);     // JCM800 B/M/T stack
      // makeup: the HM2 bands boost hard at max settings
      chans[c][i] = y * 0.35f;
    }
  }
}

// ---- Speaker cab convolution (swede only) ---------------------------------
// Mono-summed cab FIR (src/speaker_ir.h — the Hesu 2x12 V30 IR, resampled to
// 16 kHz, pre-delay compacted, peak-normalized; see tools/ir2array.py), run
// at the DECIMATED rate: the IR is band-limited to 8 kHz (0.42% energy
// above), so a 31-tap halfband decimation -> 1024-tap FIR @ 16 kHz (64 ms) ->
// halfband interpolation halves the MAC count of a 1024-tap 32 kHz FIR while
// keeping the full 64 ms body (99.9% of the compacted energy; the 512-tap
// window held 98.8%). The Barabas5532 plugin convolves a speaker IR at the
// same chain point (after the amp + tonestack).
//   Per 16-sample block: dec 16x16 + conv 8x1024 + interp 8x14 ~= 8.5k MACs
//   (the dual-MAC kernel does that in 8 instructions per 4 MACs).
// Adds ~67 ms of latency — inaudible, the swede path is fully wet.
// The halfband (Blackman-windowed sinc, DC-normalized, center 0.5): the
// decimator runs the full 31 taps on odd 32 kHz samples; the interpolator's
// passthrough phase is the delayed v (free), the filtered phase is the
// halfband's 14 nonzero even-index taps x2 (their DC sum is 0.5).
static const size_t kSpeakerTaps = 1024;   // @ 16 kHz = 64 ms
static const float kSpeakerConvGain = 0.25f;   // makeup for the normalized IR

// RMS-matching gain for the pre-conv bypass: sqrt(sum ir^2) * conv gain, so
// a bypass block (mono pre-cab signal) has the same loudness as a conv block.
// Precomputed by ir2array.py as kSpeakerIrRms (the float IR table itself was
// dropped — its only consumer was this energy sum, and the 1024-tap q15 set
// would not have fit the tc_elements A-slot alongside it).
static inline float speaker_bypass_gain() {
  return kSpeakerIrRms * kSpeakerConvGain;
}

// int64 -> float without the soft __aeabi_l2f (~100 cycles per call, 8 calls
// per block): acc = hi*2^32 + lo, each half converts exactly (|hi| < 2^31,
// lo < 2^31 as unsigned), the float sum differs from the direct conversion
// by <= 1 ULP of the 24-bit mantissa (both round the same way to first
// order; 1 ULP ~= -144 dB here).
static inline float acc64_to_f(int64_t a) {
  return (float)(int32_t)(a >> 32) * 4294967296.0f + (float)(uint32_t)a;
}
static const float kHalfbandDec[31] = {   // DC-normalized (sum = 1)
    2.94498535e-19f, 7.81558784e-20f, 0.000410322871f, -7.83790506e-19f,
    -0.00223028552f, 2.53383519e-18f, 0.00710085713f, -5.64234343e-18f,
    -0.0179170304f, 9.93628146e-18f, 0.0401074177f, -1.4544273e-17f,
    -0.0901069221f, 1.81325764e-17f, 0.312633322f, 0.500004637f,
    0.312633322f, 1.81325764e-17f, -0.0901069221f, -1.4544273e-17f,
    0.0401074177f, 9.93628146e-18f, -0.0179170304f, -5.64234343e-18f,
    0.00710085713f, 2.53383519e-18f, -0.00223028552f, -7.83790506e-19f,
    0.000410322871f, 7.81558784e-20f, 2.94498535e-19f};
// The interpolator's filtered phase: the halfband's even-index taps x2
// (causal indices 28..2, descending to pair with v[m-14..m-1]; index 15 is
// the passthrough center; the symmetric filter makes either order valid).
static const float kHalfbandInt[14] = {
    0.000820645742f, -0.00446057104f, 0.0142017143f, -0.0358340608f,
    0.0802148354f, -0.180213844f, 0.625266644f, 0.625266644f,
    -0.180213844f, 0.0802148354f, -0.0358340608f, 0.0142017143f,
    -0.00446057104f, 0.000820645742f};

struct SpeakerConv {
  // 32 kHz decimator: the last 31 mono samples (oldest first).
  float dec_[31] = {0};
  // 16 kHz conv history: kTaps-1 past v16 samples + up to 12 new per block.
  // 4-aligned so the shift and the Ws pack below can move it word-at-a-time
  // (a 4-byte __builtin_memcpy per pair; element loops are the odd-n16
  // fallback).
  alignas(4) int16_t hist16_[kSpeakerTaps - 1 + 12] = {0};
  // 16 kHz conv-output history for the interpolator (float, 14 taps).
  float y16h_[14] = {0};
  // 16 kHz v-history for the interpolator's passthrough (7-deep delay).
  float vhist_[14] = {0};
  RACK_DIST_MARK(dist_speaker_conv)
  void Process(float* l, float* r, size_t n) {
    if (n > 24u) n = 24u;
    // 1) mono-sum into the decimator window tail (31 history + n new)
    float decw[31 + 24];
    for (size_t i = 0; i < 31; ++i) decw[i] = dec_[i];
    for (size_t i = 0; i < n; ++i) decw[31 + i] = 0.5f * (l[i] + r[i]);
    // 2) decimate: one 16 kHz sample per odd 32 kHz sample, full 31-tap dot
    //    anchored at the center tap (causal index 15): v[j] over the window.
    const size_t n16 = n / 2;
    float v[12];
    for (size_t j = 0; j < n16; ++j) {
      const float* w = decw + 30 + 2 * j;   // newest sample of this pair
      float acc = 0.0f;
      for (size_t k = 0; k < 31; ++k) {
        acc += kHalfbandDec[k] * w[-k];
      }
      v[j] = acc;
    }
    // 3) shift the decimator history: keep the last 31 mono samples
    for (size_t i = 0; i < 31; ++i) {
      dec_[i] = decw[n + i];
    }
    // 4) conv at 16 kHz, q15 int16 dual-MAC (SMLALD/SMLALDX, M33 DSP
    //    extension — Yiu ch.19.4): quantize v to int16, append to hist16,
    //    then the packed dual-MAC FIR below (two outputs per iteration,
    //    64-bit accumulate, full 15-bit coefficients). Host fallback: the
    //    same dual-MAC math in portable C (bit-exact vs the hardware).
    for (size_t j = 0; j < n16; ++j) {
      float q = v[j] * 32767.0f;
      if (q > 32767.0f) q = 32767.0f;
      if (q < -32768.0f) q = -32768.0f;
      hist16_[kSpeakerTaps - 1 + j] = (int16_t)q;
    }
    // The packed FIR. hist16_ (kTaps-1 old + n16 new v16 samples, 4-aligned)
    // is read as uint32 words W[k] = v16[2k] | v16[2k+1]<<16 — packed into
    // Ws first with one 4-byte memcpy per word (gcc emits a single ldr+str;
    // the element-wise orr loop cost ~10 us per block). Output j's window
    // ends at m = kSpeakerTaps-1+j; its t-th tap pair is (v16[m-2t],
    // v16[m-2t-1]) — DESCENDING consecutive samples, so per output parity:
    //   m odd (j even): the pair = (W[k].hi, W[k].lo), k = kTaps/2-1+r-t —
    //     SMLALDX's Xm-half swap realigns it with (ir16[2t], ir16[2t+1]).
    //   m even (j odd): the pair straddles two adjacent words —
    //     (W[k].lo, W[k-1].hi), k = kTaps/2+r-t — PKHBT re-packs it
    //     (top from the older word, bottom from the newer); plain SMLALD.
    // Both outputs share the same ir word and the same two W words (one
    // descending pointer pair). Splitting the parity RIGHT is what the
    // first SMLADX-everywhere attempt got wrong (crossed tap pairs, err
    // 0.33 — the naive law caught it). Per pair of outputs this is 8
    // instructions for 4 MACs.
    uint32_t Ws[(kSpeakerTaps + 14) / 2];
    const size_t ns16 = kSpeakerTaps - 1 + n16;   // valid v16 samples
    if (n16 & 1) {
      // n16 odd (never with the 16-sample block): the element-wise build.
      for (size_t k = 0; k < (ns16 + 1) / 2; ++k) {
        const uint32_t lo = (uint16_t)hist16_[2 * k];
        const uint32_t hi =
            2 * k + 1 < ns16 ? (uint16_t)hist16_[2 * k + 1] : 0u;
        Ws[k] = lo | (hi << 16);
      }
    } else {
      // Word-at-a-time pack (hist16_ is 4-aligned, n16 even). ns16 is odd
      // here (kTaps-1 is odd), so the last word's hi half comes from stale
      // history — mask it (it is never read, but keep the build
      // deterministic).
      for (size_t k = 0; k < (ns16 + 1) / 2; ++k) {
        __builtin_memcpy(&Ws[k], &hist16_[2 * k], 4);
      }
      Ws[ns16 >> 1] &= 0x0000ffffu;
    }
    for (size_t r = 0; r < n16 / 2; ++r) {
      int64_t acc_a = 0, acc_b = 0;
#if defined(__ARM_FEATURE_DSP) && (defined(__arm__) || defined(__thumb__))
      // Hand-written kernel (the CMSIS q15-FIR pattern): gcc spilled one of
      // the two descending pointers to the stack every iteration and burned
      // ~6 movs shuffling the inline-asm operands (~15 instructions per 4
      // MACs). Register variables pin every operand — no allocator, no
      // spill. Per iteration: ldr ir, ldr b (post-dec), ldr a (plain),
      // smlaldx (output 2r), pkhbt + smlald (output 2r+1) = 8 instructions
      // for 4 MACs, 64-bit accumulate in two register pairs each.
      {
        register size_t cnt __asm__("r3") = kSpeakerTaps / 2;
        register int32_t aa_lo __asm__("r4") = 0;
        register int32_t aa_hi __asm__("r5") = 0;
        register int32_t ab_lo __asm__("r8") = 0;
        register int32_t ab_hi __asm__("r9") = 0;
        // Two SEPARATE loads, stride one word per pointer: the window pairs
        // OVERLAP between iterations (the pairs are (W[511+r-t],
        // W[512+r-t]) then (W[510+r-t], W[511+r-t])) so a single ldmdb of
        // two regs — which slides the base by 8 — drifts one extra word
        // per iteration and misaligns every FIR tap after the first (the
        // constant-noise bug this file's first HW build produced; warps
        // stays clean because it bypasses the conv).
        register const uint32_t* bp __asm__("r6") =
            Ws + (kSpeakerTaps / 2 - 1 + r) + 1;      // &W[512+r] (b of t=0)
        register const uint32_t* irp __asm__("r7") = kSpeakerIrQ15;
        register uint32_t iw __asm__("r0");
        register uint32_t a __asm__("r1");
        register uint32_t b __asm__("r2");
        register uint32_t x __asm__("r10");
        __asm__ __volatile__(
            "1:\n"
            "ldr.w   %[iw], [%[irp]], #4\n"
            "ldr.w   %[b], [%[bp]], #-4\n"
            "ldr.w   %[a], [%[bp]]\n"
            "smlaldx %[al], %[ah], %[iw], %[a]\n"
            "pkhbt   %[x], %[a], %[b]\n"
            "smlald  %[bl], %[bh], %[iw], %[x]\n"
            "subs    %[cnt], #1\n"
            "bne.n   1b\n"
            : [al] "+r"(aa_lo), [ah] "+r"(aa_hi), [bl] "+r"(ab_lo),
              [bh] "+r"(ab_hi), [bp] "+r"(bp), [irp] "+r"(irp),
              [cnt] "+r"(cnt), [iw] "=&r"(iw), [a] "=&r"(a), [b] "=&r"(b),
              [x] "=&r"(x)
            :
            : "cc", "memory");
        acc_a = ((int64_t)(uint32_t)aa_hi << 32) | (uint32_t)aa_lo;
        acc_b = ((int64_t)(uint32_t)ab_hi << 32) | (uint32_t)ab_lo;
      }
#else
      {
        const uint32_t* pa = Ws + (kSpeakerTaps / 2 - 1 + r);  // W[511+r] ↓
        const uint32_t* pb = pa + 1;                           // W[512+r] ↓
        const uint32_t* irp = kSpeakerIrQ15;
        for (size_t t = 0; t < kSpeakerTaps / 2; ++t) {
          const uint32_t iw = *irp++;
          const uint32_t a = *pa--;                // W[511+r-t] (older)
          const uint32_t b = *pb--;                // W[512+r-t] (newer)
          acc_a = RACK_SMLALDX_ACC(acc_a, iw, a);  // output 2r   (m odd)
          acc_b = RACK_SMLALD_ACC(acc_b, iw,
                                  RACK_PKHTB(a, b));  // output 2r+1 (m even)
        }
      }
#endif
      v[2 * r] = acc64_to_f(acc_a) * kSpeakerQ15Norm * kSpeakerConvGain;
      v[2 * r + 1] = acc64_to_f(acc_b) * kSpeakerQ15Norm * kSpeakerConvGain;
    }
    if (n16 & 1) {   // never with the 16-sample rack block; kept correct
      const size_t j = n16 - 1;
      const int16_t* xp = hist16_ + (kSpeakerTaps - 1 + (int)j);
      int64_t acc = 0;
      for (size_t t = 0; t < kSpeakerTaps / 2; ++t) {
        // (ir16[2t], ir16[2t+1]) x (v16[m-2t], v16[m-2t-1]) — plain halves
        const uint32_t xw = (uint32_t)(uint16_t)xp[-2 * (int)t]
                          | ((uint32_t)(uint16_t)xp[-2 * (int)t - 1] << 16);
        acc = RACK_SMLALD_ACC(acc, kSpeakerIrQ15[t], xw);
      }
      v[j] = acc64_to_f(acc) * kSpeakerQ15Norm * kSpeakerConvGain;
    }
    // 5) shift the conv history (the element loop cost ~8.5 us/block at
    //    1023 halfwords; word-at-a-time with n16 even is ~2 us). kTaps-1 is
    //    odd: full words + one halfword tail.
    if (n16 & 1) {
      for (size_t i = 0; i < kSpeakerTaps - 1; ++i) {
        hist16_[i] = hist16_[n16 + i];
      }
    } else {
      for (size_t k = 0; k < (kSpeakerTaps - 1) / 2; ++k) {
        __builtin_memcpy(&hist16_[2 * k], &hist16_[n16 + 2 * k], 4);
      }
      hist16_[kSpeakerTaps - 2] = hist16_[n16 + kSpeakerTaps - 2];
    }
    // 5) interpolate: per v sample emit the pair (passthrough = the delayed
    //    conv output, filtered = the 14-tap even-phase sum x2), advancing
    //    both histories with the new conv output.
    for (size_t j = 0; j < n16; ++j) {
      float acc = 0.0f;
      for (size_t k = 0; k < 14; ++k) {
        acc += kHalfbandInt[k] * y16h_[k];
      }
      for (size_t k = 0; k < 13; ++k) y16h_[k] = y16h_[k + 1];
      y16h_[13] = v[j];
      const float pass = y16h_[6];    // conv output delayed 7 (the center)
      l[2 * j] = acc;
      r[2 * j] = acc;
      l[2 * j + 1] = pass;
      r[2 * j + 1] = pass;
    }
  }
};
static inline SpeakerConv& speaker_conv() {
  static SpeakerConv s{};   // zero-initialized history
  return s;
}

// Retroactor's Vult-generated Befaco wavefolder, from
// ctag-tbd/components/ctagSoundProcessor/vult/vult_formantor.cpp.
static inline float fold(float signal, float level) {
  const float sign = signal > 0.0f ? 1.0f : -1.0f;
  const float amp = std::fabs(signal) * (1.0f + 8.0f * level);
  const float base = std::floor(amp);
  const float delta = amp - base;
  const bool odd = ((int32_t)base & 1) != 0;
  return (odd ? (1.0f - delta) : delta) * sign;
}

// braids' per-unit sine table, copied verbatim from
// eurorack@5642bf4:braids/resources.cc (512 B of read-only data). Using the
// exact table keeps the seeded curve bit-identical to ctag-tbd and avoids a
// per-sample sinf().
static const int16_t kWavSine[257] = {
  -32512, -32502, -32473, -32423,
  -32356, -32265, -32160, -32031,
  -31885, -31719, -31533, -31331,
  -31106, -30864, -30605, -30324,
  -30028, -29712, -29379, -29026,
  -28658, -28272, -27868, -27449,
  -27011, -26558, -26089, -25604,
  -25103, -24588, -24056, -23512,
  -22953, -22378, -21793, -21191,
  -20579, -19954, -19316, -18667,
  -18006, -17334, -16654, -15960,
  -15259, -14548, -13828, -13100,
  -12363, -11620, -10868, -10112,
   -9347,  -8578,  -7805,  -7023,
   -6241,  -5453,  -4662,  -3868,
   -3073,  -2274,  -1474,   -674,
     126,    929,   1729,   2527,
    3326,   4123,   4916,   5707,
    6495,   7278,   8057,   8833,
    9601,  10366,  11122,  11874,
   12618,  13353,  14082,  14802,
   15512,  16216,  16906,  17589,
   18260,  18922,  19569,  20207,
   20834,  21446,  22045,  22634,
   23206,  23765,  24311,  24842,
   25357,  25858,  26343,  26812,
   27266,  27701,  28123,  28526,
   28912,  29281,  29632,  29966,
   30281,  30579,  30859,  31118,
   31361,  31583,  31788,  31973,
   32139,  32286,  32412,  32521,
   32608,  32679,  32725,  32757,
   32766,  32757,  32725,  32679,
   32608,  32521,  32412,  32286,
   32139,  31973,  31788,  31583,
   31361,  31118,  30859,  30579,
   30281,  29966,  29632,  29281,
   28912,  28526,  28123,  27701,
   27266,  26812,  26343,  25858,
   25357,  24842,  24311,  23765,
   23206,  22634,  22045,  21446,
   20834,  20207,  19569,  18922,
   18260,  17589,  16906,  16216,
   15512,  14802,  14082,  13353,
   12618,  11874,  11122,  10366,
    9601,   8833,   8057,   7278,
    6495,   5707,   4916,   4123,
    3326,   2527,   1729,    929,
     126,   -674,  -1474,  -2274,
   -3073,  -3868,  -4662,  -5453,
   -6241,  -7023,  -7805,  -8578,
   -9347, -10112, -10868, -11620,
  -12363, -13100, -13828, -14548,
  -15259, -15960, -16654, -17334,
  -18006, -18667, -19316, -19954,
  -20579, -21191, -21793, -22378,
  -22953, -23512, -24056, -24588,
  -25103, -25604, -26089, -26558,
  -27011, -27449, -27868, -28272,
  -28658, -29026, -29379, -29712,
  -30028, -30324, -30605, -30864,
  -31106, -31331, -31533, -31719,
  -31885, -32031, -32160, -32265,
  -32356, -32423, -32473, -32502,
  -32512,
};

static inline int16_t mix16(int16_t a, int16_t b, uint16_t balance) {
  return (int16_t)(((int32_t)a * (int32_t)(65535u - balance) +
                    (int32_t)b * (int32_t)balance) >> 16);
}

struct ShapeTable {
  int32_t v[257];
  bool built;
};

// TBD03's seed 0xcafe curve — braids SignatureWaveshaper math verbatim.
static inline const int32_t* shape_table() {
  static ShapeTable t{};
  if (t.built) return t.v;
  uint32_t seed = 0xcafeu;
  int32_t skew = (int32_t)(seed & 15u);          seed >>= 4;
  int32_t sigmoid_strength = (int32_t)(seed & 31u); seed >>= 5;
  int32_t bumplets_frequency = (int32_t)(seed & 3u); seed >>= 2;
  bumplets_frequency += 3;
  int32_t bumplets_width = (int32_t)(seed & 7u);  seed >>= 3;
  bumplets_width += 1;
  bumplets_width <<= 7;
  bumplets_width *= bumplets_width;
  for (int i = 0; i < 256; ++i) {
    int16_t x = (int16_t)((i - 128) << 8);
    int16_t x_skew = (int16_t)(i * i - 32768);
    x = mix16(x, x_skew, (uint16_t)(skew << 11));
    const int32_t ax = x < 0 ? -(int32_t)x : (int32_t)x;
    const int32_t num = (int32_t)x * (8192 + (sigmoid_strength << 10));
    const int32_t den = 8192 + ((sigmoid_strength * ax) >> 5);
    int16_t sigmoid = (int16_t)(num / den);
    int16_t bumplets = kWavSine[(i * bumplets_frequency) & 255];
    uint16_t bumplet_gain = (uint16_t)((int32_t)x * (int32_t)x /
                                       bumplets_width + 16);
    bumplet_gain = (uint16_t)(32768 * 128 / (128 + (int32_t)bumplet_gain));
    t.v[i] = mix16(sigmoid, bumplets, bumplet_gain);
  }
  t.v[256] = t.v[255];
  t.built = true;
  return t.v;
}

// Shape the stereo pair in place. State carries the per-channel tone LP
// between blocks. drive == 0 returns immediately (no reads, no writes).
// RACK_DIST_MARK: the whole dispatcher (4 fast algos + swede chain incl. the
// cab FIR) lives in the including adapter's SRAM window when provided.
RACK_DIST_MARK(dist_process)
static inline void process(State* st, const Params& p,
                           float* out, float* aux, size_t n) {
  if (p.drive <= 0.0f) return;
  const float d = p.drive;
  const float lp_k = 0.04f + 0.96f * clamp01(p.tone);
  const int type = (int)(p.type % 5u);
  if (type == kSwedish) {
    // Block-oriented pedal core, then the cab IR and the common tone LP.
    // Rings blocks <= 24.
    const size_t m = n < 64u ? n : 64u;
    swedish_process(out, aux, m, d, p.bass, p.mid, p.treble);
    // The cab convolution moved OFF the audio core (2026-11-19): the adapter
    // publishes the post-FMV mono block to core 0, which runs speaker_conv()
    // and hands the result back one block later (see module_elements.h).
    float* chans[2] = {out, aux};
    for (int c = 0; c < 2; ++c) {
      float lp = st->lp[c];
      for (size_t i = 0; i < m; ++i) {
        lp += lp_k * (chans[c][i] - lp);
        chans[c][i] = lp;
      }
      st->lp[c] = lp;
    }
    return;
  }
  float k = 0.0f, k_norm = 1.0f;
  float bias = 0.0f, bias_sub = 0.0f, bias_norm = 1.0f;
  float ws_gain = 1.0f;
  const int32_t* ws = nullptr;
  if (type == kTube) {
    k = 1.0f + 9.0f * d * d;
    k_norm = 1.0f / softclip(k);
  } else if (type == kFuzz) {
    k = 1.0f + 9.0f * d * d;
    // Fuzz MUST be asymmetric even with the bias knob at zero — otherwise
    // (bias_sub=0, bias_norm=1) it is bit-identical to the tube algorithm.
    bias = 0.25f + 0.75f * clamp01(p.bias);
    bias_sub = softclip(bias);
    bias_norm = 1.0f / (1.0f + bias_sub);
  } else if (type == kShape) {
    ws_gain = 1.0f + 7.0f * d;
    ws = shape_table();
  }
  float* chans[2] = {out, aux};
  for (int c = 0; c < 2; ++c) {
    float lp = st->lp[c];
    for (size_t i = 0; i < n; ++i) {
      const float dry = chans[c][i];
      float wet;
      if (type == kFold) {
        wet = fold(dry, d);
      } else if (type == kFuzz) {
        wet = (softclip(k * dry + bias) - bias_sub) * bias_norm;
      } else if (type == kShape) {
        float xg = dry * ws_gain;
        if (xg > 1.0f) xg = 1.0f;
        if (xg < -1.0f) xg = -1.0f;
        const uint16_t idx = (uint16_t)((int16_t)(xg * 32767.0f) + 32768);
        const int32_t a = ws[idx >> 8];
        const int32_t b = ws[(idx >> 8) + 1];
        wet = (float)(a + ((b - a) * (int32_t)(idx & 0xffu) >> 8)) *
              (1.0f / 32768.0f);
      } else {
        wet = softclip(k * dry) * k_norm;
      }
      lp += lp_k * (wet - lp);
      chans[c][i] = lp;
    }
    st->lp[c] = lp;
  }
}

}  // namespace rackdist
#endif  // RACK_SRC_RINGS_DIST_H_
