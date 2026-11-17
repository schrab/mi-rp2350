// Rings post-resonator distortion — the four fast algorithms from ctag-tbd's
// TBDings/Elements distortion stage (commits 2166c84c, 4f086145, a62ad652).
//
// Ported verbatim into a pure C++ header (no MCU headers) so it is host-
// testable and can run in the RINGS adapter's flash-resident per-sample loop.
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

namespace ringdist {

enum Type : uint8_t {
  kTube = 0, kFold = 1, kFuzz = 2, kShape = 3,
  kSwedish = 4  // Swedish Chainsaw: HM-2 EQ + cascaded tanh
};

struct State {
  float lp[2];  // one-pole tone state, per channel
};

struct Params {
  uint8_t type;  // 0..3
  float drive;   // 0..1; 0 = zero-cost bypass
  float bias;    // 0..1; fuzz bias / Swedish "tight" HP toggle
  float tone;    // 0..1; 1 = open
  float mix;     // 0..1; 0 = fully dry
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
// plugin's HM2Filter.cpp) wrapped in two tanh saturation stages. The plugin
// also does 4x oversampling, an amp input HP and a JCM800 FMV tonestack; we
// keep the pedal's defining HM2+tanh core at 32 kHz and reuse the common
// tone-LP/dry-wet stage below. Coefficients are fixed-digital (the JUCE
// prepare() ignores sampleRate), so at 32 kHz the bands sit ~0.73x lower.
struct SwedishState {
  float hp_y[2];        // input high-pass output state
  float hp_x[2];        // input high-pass input state
  float bq[2][3][2];    // per-channel biquad z1/z2 for the 3 HM2 bands
  float pre_lp[2];      // 7 kHz post-HM2 one-pole
};
static inline SwedishState& swedish_state() {
  static SwedishState s{};
  return s;
}

// Fixed HM2 bands: b0,b1,b2,a1,a2 (a0 = 1 in all three).
static inline void swedish_biquad(float* z, float x,
                                  float b0, float b1, float b2,
                                  float a1, float a2, float& y) {
  y = b0 * x + z[0];
  z[0] = b1 * x - a1 * y + z[1];
  z[1] = b2 * x - a2 * y;
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

// One stereo block through the pedal core. `drive` = pedal gain, `bias`
// >= 0.5 selects the tighter 700 Hz input HP. State is a singleton: there is
// exactly one Rings adapter instance, and keeping it static avoids growing
// the arena-placed State (chain rings+clouds has only ~128 B spare).
static inline void swedish_process(float* out, float* aux, size_t n,
                                   float drive, float bias) {
  SwedishState& st = swedish_state();
  const float a_hp = 1.0f / (1.0f + 6.2831853f * (bias >= 0.5f ? 700.0f
                                                              : 60.0f)
                                            / 32000.0f);
  const float a_lp = 6.2831853f * 7000.0f / 32000.0f;
  const float pedal_gain = 1.0f + 19.0f * clamp01(drive);
  const float amp_gain = 1.0f + 9.0f * clamp01(drive);
  float* chans[2] = {out, aux};
  for (int c = 0; c < 2; ++c) {
    for (size_t i = 0; i < n; ++i) {
      float x = swedish_hp(st.hp_y[c], st.hp_x[c], chans[c][i], a_hp);
      x = softclip(x * pedal_gain);                    // pedal tanh
      float y;
      swedish_biquad(st.bq[c][0], x,
                     1.016224171805848f, -1.996244766867333f,
                     0.980170456681741f, -1.996244766867333f,
                     0.996394628487589f, y);
      swedish_biquad(st.bq[c][1], y,
                     1.048098514125369f, -1.946884930731663f,
                     0.914902628855116f, -1.946884930731663f,
                     0.963001142980485f, y);
      swedish_biquad(st.bq[c][2], y,
                     1.260798129602192f, -1.896438187817481f,
                     0.674002337997260f, -1.896438187817481f,
                     0.934800467599452f, y);
      y = swedish_lp(st.pre_lp[c], y, a_lp);
      y = softclip(y * amp_gain);                      // amp tanh
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
  const float mix = clamp01(p.mix);
  const float lp_k = 0.04f + 0.96f * clamp01(p.tone);
  const int type = (int)(p.type % 5u);
  if (type == kSwedish) {
    // Block-oriented pedal core: save dry, shape, then reuse the common
    // tone LP + dry/wet blend. Rings blocks are <= 24 samples.
    float dry[2][64];
    const size_t m = n < 64u ? n : 64u;
    for (size_t i = 0; i < m; ++i) {
      dry[0][i] = out[i];
      dry[1][i] = aux[i];
    }
    swedish_process(out, aux, m, d, p.bias);
    float* chans[2] = {out, aux};
    for (int c = 0; c < 2; ++c) {
      float lp = st->lp[c];
      for (size_t i = 0; i < m; ++i) {
        lp += lp_k * (chans[c][i] - lp);
        chans[c][i] = dry[c][i] + (lp - dry[c][i]) * mix;
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
      chans[c][i] = dry + (lp - dry) * mix;
    }
    st->lp[c] = lp;
  }
}

}  // namespace ringdist
#endif  // RACK_SRC_RINGS_DIST_H_
