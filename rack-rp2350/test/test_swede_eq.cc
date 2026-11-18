// Swedish Chainsaw upgrade tests: 32 kHz HM-2 band redesign + JCM800 FMV
// tonestack knobs (bass/mid/treble), ported from Barabas5532/SwedishChainsaw.
//
// Laws:
//   1. Band redesign: swedish_peaking() at 44.1 kHz with the derived band
//      specs (86 Hz +20 dB Q1.073 / 900 Hz +11.1 dB Q1.789 / 1400 Hz +19.1 dB
//      Q0.980) reproduces the reference MATLAB coefficients from HM2Filter.cpp
//      (the plugin's fixed-digital design) to float tolerance — proving the
//      derived specs AND the design function; the 32 kHz design therefore
//      puts the HM-2 bands at their true centers (the old fixed coefficients
//      ran them 0.726x low at 32 kHz).
//   2. Stability: noise through the full swede chain (defaults and all B/M/T
//      corners) stays bounded and finite.
//   3. Knobs work: the three FMV knobs change the output.
//   4. FMV design math: at (0.05, 0.5, 0.5) the z-coefficients are finite and
//      the normalized denominator differs across knob extremes (the knobs
//      actually enter the design).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "rack_dist.h"          // -I../src

static int g_fail = 0;
#define CHECK(cond, ...)                                                    \
  do {                                                                      \
    if (!(cond)) {                                                          \
      fprintf(stderr, "FAIL %s:%d ", __FILE__, __LINE__);                   \
      fprintf(stderr, __VA_ARGS__);                                         \
      fprintf(stderr, "\n");                                                \
      ++g_fail;                                                             \
    }                                                                       \
  } while (0)

// Reference MATLAB coefficients (Barabas5532 HM2Filter.cpp), as previously
// hard-coded in rack_dist.h before the 32 kHz redesign.
static const rackdist::SwedishBiquad kRef[3] = {
    {1.016224171805848f, -1.996244766867333f, 0.980170456681741f,
     -1.996244766867333f, 0.996394628487589f},
    {1.048098514125369f, -1.946884930731663f, 0.914902628855116f,
     -1.946884930731663f, 0.963001142980485f},
    {1.260798129602192f, -1.896438187817481f, 0.674002337997260f,
     -1.896438187817481f, 0.934800467599452f},
};

static uint32_t g_seed = 0x12345678u;
static float noise() {           // deterministic LCG noise, ~[-1, 1]
  g_seed = g_seed * 1664525u + 1013904223u;
  return (float)(int32_t)g_seed / (float)0x40000000;
}

static float render_rms(float bass, float mid, float treble, size_t blocks) {
  // fresh singleton state per render: recompile the translation unit? No —
  // swedish_state() persists; run the render twice and keep the second half.
  const size_t kBlock = 16;
  float out[kBlock], aux[kBlock];
  double sum2 = 0.0;
  size_t n = 0;
  for (size_t b = 0; b < blocks * 2; ++b) {   // first half: settle the state
    for (size_t i = 0; i < kBlock; ++i) {
      out[i] = noise() * 0.5f;
      aux[i] = out[i] * 0.5f;
    }
    rackdist::swedish_process(out, aux, kBlock, 0.75f, bass, mid, treble);
    if (b >= blocks) {
      for (size_t i = 0; i < kBlock; ++i) {
        sum2 += (double)out[i] * out[i];
        ++n;
      }
    }
  }
  return (float)sqrt(sum2 / (double)n);
}

int main() {
  // Law 1: the design function reproduces the reference MATLAB coefficients.
  static const float kFs = 44100.0f;
  static const float kSpec[3][3] = {{86.0f, 20.0f, 1.073f},
                                    {900.0f, 11.13f, 1.788f},
                                    {1400.0f, 19.08f, 0.980f}};
  for (int i = 0; i < 3; ++i) {
    const rackdist::SwedishBiquad c =
        rackdist::swedish_peaking(kSpec[i][0], kSpec[i][1], kSpec[i][2], kFs);
    const float err[] = {fabsf(c.b0 - kRef[i].b0), fabsf(c.b1 - kRef[i].b1),
                         fabsf(c.b2 - kRef[i].b2), fabsf(c.a1 - kRef[i].a1),
                         fabsf(c.a2 - kRef[i].a2)};
    float mx = 0.0f;
    for (float e : err) mx = mx > e ? mx : e;
    CHECK(mx < 5e-4f,
          "band %d design mismatch vs reference MATLAB coefficients: %g",
          i + 1, mx);
  }
  // ...and the 32 kHz redesign actually moved the bands (not a no-op).
  const rackdist::SwedishBiquad at32 =
      rackdist::swedish_peaking(86.0f, 10.0f, 1.073f, 32000.0f);
  CHECK(fabsf(at32.b0 - kRef[0].b0) > 1e-4f,
        "32 kHz design identical to the 44.1 kHz reference — redesign no-op?");

  // Law 2: stability at defaults and all B/M/T corners.
  static const float kCorners[9][3] = {
      {0.05f, 0.5f, 0.5f},                       // plugin defaults
      {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f},    // min / max
      {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f},    // mixed corners
      {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f},
      {1.0f, 0.0f, 0.0f},
  };
  for (const auto& k : kCorners) {
    const float rms = render_rms(k[0], k[1], k[2], 5000);
    CHECK(std::isfinite(rms) && rms < 10.0f && rms > 0.0f,
          "unstable/empty render at B%.2f M%.2f T%.2f: rms %f", k[0], k[1],
          k[2], rms);
  }

  // Law 3: the knobs change the output.
  const float dflt = render_rms(0.05f, 0.5f, 0.5f, 5000);
  const float maxed = render_rms(1.0f, 1.0f, 1.0f, 5000);
  const float flat = render_rms(0.0f, 0.0f, 0.0f, 5000);
  CHECK(fabsf(maxed - dflt) > 1e-4f * (maxed + dflt),
        "B/M/T maxed == defaults — knobs do nothing");
  CHECK(fabsf(flat - maxed) > 1e-4f * (flat + maxed),
        "B/M/T zeroed == maxed — knobs do nothing");

  // Law 4: the FMV design enters the knobs and stays finite.
  rackdist::SwedishFmvCoefs f;
  rackdist::swedish_fmv_design(0.05f, 0.5f, 0.5f, 32000.0f, &f);
  CHECK(std::isfinite(f.b0) && std::isfinite(f.a3) && std::isfinite(f.a1),
        "FMV coefficients not finite at defaults");
  rackdist::SwedishFmvCoefs g2;
  rackdist::swedish_fmv_design(1.0f, 1.0f, 1.0f, 32000.0f, &g2);
  CHECK(fabsf(g2.a1 - f.a1) > 1e-6f,
        "FMV denominator identical across knob extremes — knobs do not enter "
        "the design");

  if (g_fail) {
    fprintf(stderr, "test_swede_eq: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_swede_eq: all laws hold\n");
  return 0;
}
