// Speaker cab convolution tests (swede FX, src/speaker_ir.h + SpeakerConv).
//
// The conv runs DECIEMATED: 31-tap halfband dec (32k -> 16k), 512-tap FIR at
// 16 kHz (the IR is band-limited to 8 kHz), 31-tap halfband interpolation
// (16k -> 32k). Laws:
//   1. REFERENCE LAW: the block implementation matches an independently
//      written naive implementation of the full decimated chain (per the
//      definitions in rack_dist.h: v[m] = halfband at mono 2m+1,
//      y[m] = sum ir[k]*v[m-k], pairs (filt, pass) per v), sample-exact
//      within float tolerance across many consecutive blocks (history
//      carry-over + the dec/interp phase alignment exercised).
//   2. Stability: the full swede chain at defaults + IR, driven with noise,
//      stays bounded.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "rack_dist.h"          // -I../src

using namespace rackdist;

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

static uint32_t g_seed = 0xfeedbeefu;
static float noise() {
  g_seed = g_seed * 1664525u + 1013904223u;
  return (float)(int32_t)g_seed / (float)0x40000000;
}

// Naive reference of the full decimated chain, straight from the
// definitions (v[m] = halfband at mono 2m-1; the conv runs in q15:
// v16[m] = clamp(round(v[m]*32767)), y16[m] = (sum ir16[k]*v16[m-k]) *
// kSpeakerQ15Norm * conv gain — the int64 reference is bit-exact vs the
// hardware dual-MAC since sum|ir16| <= 32700 keeps the int32 accumulator
// in range; the interpolator runs over the y16 stream: output pair per v:
// (filt[m] = 14-tap even-phase sum over y16[m-14..m-1], pass[m] = y16[m-7])).
struct NaiveChain {
  std::vector<float> mono, v, y16;
  std::vector<int16_t> v16;
  // halfband value at mono index 2m-1 (zero-padded history; the impl's
  // decimator anchors its center tap there — the first v is at mono -1)
  float halfband(size_t m) const {
    float acc = 0.0f;
    for (size_t k = 0; k < 31; ++k) {
      const long long idx = (long long)(2 * m - 1) - (long long)k;
      if (idx >= 0 && idx < (long long)mono.size()) {
        acc += kHalfbandDec[k] * mono[(size_t)idx];
      }
    }
    return acc;
  }
  static int16_t quant15(float v) {
    float q = v * 32767.0f;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int16_t)(q >= 0.0f ? q + 0.5f : q - 0.5f);
  }
  float conv(size_t m) const {
    int64_t acc = 0;
    for (size_t k = 0; k < kSpeakerTaps; ++k) {
      const long long idx = (long long)m - (long long)k;
      const int16_t x = idx >= 0 && idx < (long long)v16.size()
                            ? v16[(size_t)idx]
                            : (int16_t)0;
      const int16_t lo = (int16_t)(kSpeakerIrQ15[k >> 1] & 0xffffu);
      const int16_t hi = (int16_t)((kSpeakerIrQ15[k >> 1] >> 16) & 0xffffu);
      acc += (int64_t)(k & 1u ? hi : lo) * (int64_t)x;
    }
    return (float)((double)acc * (double)kSpeakerQ15Norm *
                   (double)kSpeakerConvGain);
  }
  float filt(size_t m) const {   // the interpolator's filtered phase
    float acc = 0.0f;
    for (size_t k = 0; k < 14; ++k) {
      const long long idx = (long long)m - (long long)(14 - k);
      if (idx >= 0 && idx < (long long)y16.size()) {
        acc += kHalfbandInt[k] * y16[(size_t)idx];
      }
    }
    return acc;
  }
  float pass(size_t m) const {   // the delayed passthrough phase
    const long long idx = (long long)m - 7;
    return idx >= 0 && idx < (long long)y16.size() ? y16[(size_t)idx] : 0.0f;
  }
};

int main() {
  const size_t kBlock = 16;
  const size_t kBlocks = 60;

  SpeakerConv conv;
  NaiveChain naive;
  std::vector<float> got, want;
  for (size_t b = 0; b < kBlocks; ++b) {
    float l[kBlock], r[kBlock];
    for (size_t i = 0; i < kBlock; ++i) {
      l[i] = noise() * 0.4f;
      r[i] = noise() * 0.4f;
    }
    // naive: mono append, then the v's this block produces, then the pairs
    for (size_t i = 0; i < kBlock; ++i) {
      naive.mono.push_back(0.5f * (l[i] + r[i]));
    }
    const size_t v0 = naive.v.size();
    for (size_t j = 0; j < kBlock / 2; ++j) {
      naive.v.push_back(naive.halfband(v0 + j));
      naive.v16.push_back(naive.quant15(naive.v.back()));
    }
    for (size_t j = 0; j < kBlock / 2; ++j) {
      const size_t m = v0 + j;
      naive.y16.push_back(naive.conv(m));     // the interp runs over y16
    }
    for (size_t j = 0; j < kBlock / 2; ++j) {
      const size_t m = v0 + j;
      want.push_back(naive.filt(m));          // even output
      want.push_back(naive.pass(m));          // odd output
    }
    // implementation
    conv.Process(l, r, kBlock);
    for (size_t i = 0; i < kBlock; ++i) got.push_back(l[i]);
  }

  CHECK(got.size() == want.size(),
        "output count mismatch: %zu vs %zu", got.size(), want.size());
  float max_err = 0.0f;
  const size_t n = got.size() < want.size() ? got.size() : want.size();
  for (size_t i = 0; i < n; ++i) {
    const float e = fabsf(got[i] - want[i]);
    if (e > max_err) max_err = e;
  }
  CHECK(max_err < 5e-5f,
        "block impl vs naive decimated chain: max err %g", max_err);

  // stability: full swede chain at defaults + IR, driven with noise.
  rackdist::State st;
  memset(&st, 0, sizeof(st));
  rackdist::Params p{};
  p.type = kSwedish;
  p.drive = 0.75f;
  p.bass = 0.05f;
  p.mid = 0.5f;
  p.treble = 0.5f;
  p.tone = 1.0f;
  float peak = 0.0f;
  for (size_t b = 0; b < 4000; ++b) {
    float l[kBlock], r[kBlock];
    for (size_t i = 0; i < kBlock; ++i) {
      l[i] = noise() * 0.4f;
      r[i] = noise() * 0.4f;
    }
    rackdist::process(&st, p, l, r, kBlock);
    for (size_t i = 0; i < kBlock; ++i) {
      const float a = fabsf(l[i]);
      if (a > peak) peak = a;
    }
  }
  CHECK(std::isfinite(peak) && peak < 4.0f,
        "swede + cab IR render unbounded: peak %f", peak);
  printf("swede+cab peak at defaults: %.3f (conv gain 0.25)\n", peak);

  if (g_fail) {
    fprintf(stderr, "test_speaker_ir: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_speaker_ir: all laws hold\n");
  return 0;
}
