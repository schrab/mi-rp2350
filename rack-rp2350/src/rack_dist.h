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
#ifndef RACK_SRC_RINGS_DIST_H_
#define RACK_SRC_RINGS_DIST_H_

#include <stddef.h>
#include <stdint.h>

#include <cmath>

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
// plugin's HM2Filter.cpp) wrapped in two tanh saturation stages, then the
// JCM800 FMV bass/middle/treble tonestack (the plugin's FMVDesign.cpp, ported
// here verbatim-in-math). Not ported: 4x oversampling and the speaker IR.
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
  float fmv_z[2][3];    // per-channel FMV tonestack z1/z2/z3 (3rd-order DF2T)
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
static inline void swedish_process(float* out, float* aux, size_t n,
                                   float drive, float bass, float mid,
                                   float treble) {
  SwedishState& st = swedish_state();
  const float a_hp = 1.0f / (1.0f + 6.2831853f * 60.0f / 32000.0f);
  const float a_lp = 6.2831853f * 7000.0f / 32000.0f;
  const float pedal_gain = 1.0f + 19.0f * clamp01(drive);
  const float amp_gain = 1.0f + 9.0f * clamp01(drive);
  SwedishFmvCoefs fmv;
  swedish_fmv_design(clamp01(bass), clamp01(mid), clamp01(treble), 32000.0f,
                     &fmv);
  const SwedishBiquad* bands = swedish_bands();
  float* chans[2] = {out, aux};
  for (int c = 0; c < 2; ++c) {
    for (size_t i = 0; i < n; ++i) {
      float x = swedish_hp(st.hp_y[c], st.hp_x[c], chans[c][i], a_hp);
      x = softclip(x * pedal_gain);                    // pedal tanh
      float y;
      swedish_biquad(st.bq[c][0], x, bands[0], y);
      swedish_biquad(st.bq[c][1], y, bands[1], y);
      swedish_biquad(st.bq[c][2], y, bands[2], y);
      y = swedish_lp(st.pre_lp[c], y, a_lp);
      y = softclip(y * amp_gain);                      // amp tanh
      swedish_fmv_biquad3(st.fmv_z[c], y, fmv, y);     // JCM800 B/M/T stack
      // makeup: the HM2 bands boost hard at max settings
      chans[c][i] = y * 0.35f;
    }
  }
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
static inline void process(State* st, const Params& p,
                           float* out, float* aux, size_t n) {
  if (p.drive <= 0.0f) return;
  const float d = p.drive;
  const float lp_k = 0.04f + 0.96f * clamp01(p.tone);
  const int type = (int)(p.type % 5u);
  if (type == kSwedish) {
    // Block-oriented pedal core, then the common tone LP. Rings blocks <= 24.
    const size_t m = n < 64u ? n : 64u;
    swedish_process(out, aux, m, d, p.bass, p.mid, p.treble);
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
