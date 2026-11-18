// Speaker cab convolution tests (swede FX, src/speaker_ir.h + SpeakerConv).
//
// Laws:
//   1. IMPULSE LAW: a single 1.0 sample into the conv produces exactly the
//      chronological kSpeakerIr array at the output (a cab conv's response
//      to an impulse IS the IR) — verifies delay bookkeeping and the MAC
//      order end-to-end.
//   2. REFERENCE LAW: the block implementation matches a naive O(T*n)
//      time-domain convolution of the mono-summed input, sample-exact within
//      float tolerance, across several consecutive blocks (history carry-over
//      between blocks is exercised).
//   3. Stability: full swede chain at defaults + IR, driven with noise,
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

int main() {
  // Law 1: impulse -> the IR emerges verbatim.
  SpeakerConv conv;
  const size_t kBlock = 16;
  std::vector<float> out_l, out_r;
  for (size_t b = 0; b < (kSpeakerTaps / kBlock) + 4; ++b) {
    float l[kBlock], r[kBlock];
    for (size_t i = 0; i < kBlock; ++i) {
      const size_t t = b * kBlock + i;
      l[i] = (t == 5) ? 1.0f : 0.0f;   // impulse 5 samples in
      r[i] = (t == 5) ? 1.0f : 0.0f;
    }
    conv.Process(l, r, kBlock);
    for (size_t i = 0; i < kBlock; ++i) {
      out_l.push_back(l[i]);
      out_r.push_back(r[i]);
    }
  }
  // mono-sum: impulse 1.0 + 1.0 -> 0.5*(1+1) = 1.0 into the history, so the
  // response is exactly the (peak-normalized) IR scaled by the conv gain.
  for (size_t i = 0; i < kSpeakerTaps; ++i) {
    const size_t t = 5 + i;
    const float want = kSpeakerIr[i] * kSpeakerConvGain;
    CHECK(t < out_l.size() && fabsf(out_l[t] - want) < 1e-6f &&
              fabsf(out_r[t] - want) < 1e-6f,
          "impulse response mismatch at %zu: %.9f vs %.9f", i, out_l[t],
          want);
    if (g_fail) break;
  }

  // Law 2: block impl == naive time-domain convolution of the mono sum.
  const size_t kBlocks = 40;
  SpeakerConv conv2;
  std::vector<float> mono;   // the mono-summed signal, chronological
  std::vector<float> got_l;
  for (size_t b = 0; b < kBlocks; ++b) {
    float l[kBlock], r[kBlock];
    for (size_t i = 0; i < kBlock; ++i) {
      l[i] = noise() * 0.4f;
      r[i] = noise() * 0.4f;
      mono.push_back(0.5f * (l[i] + r[i]));
    }
    conv2.Process(l, r, kBlock);
    for (size_t i = 0; i < kBlock; ++i) got_l.push_back(l[i]);
  }
  float max_err = 0.0f;
  for (size_t i = 0; i < mono.size(); ++i) {
    // direct definition: y[i] = sum_k ir[k] * x[i-k]
    float ref = 0.0f;
    for (size_t k = 0; k < kSpeakerTaps && k <= i; ++k) {
      ref += kSpeakerIr[k] * mono[i - k];
    }
    ref *= 0.25f;
    const float e = fabsf(got_l[i] - ref);
    if (e > max_err) max_err = e;
  }
  CHECK(max_err < 2e-5f, "block impl vs naive convolution: max err %g",
        max_err);

  // Law 3: full swede chain at defaults + IR stays bounded.
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
