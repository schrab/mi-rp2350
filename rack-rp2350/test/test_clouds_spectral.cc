// Clouds spectral-mode host test (STFT sanity + the workspace-overrun bug
// class that ASan caught on device once: Prepare() escaping its carve).
//
// Laws:
//   1. DRY PLUMBING: dry_wet = 0 -> the output IS the input (sample-exact
//      through the processor's block path).
//   2. SPECTRAL IDENTITY: dry_wet = 1, pitch = center (0 st) -> the phase
//      vocoder passes a steady sine through with comparable RMS, bounded
//      peaks, and no NaN (garbage here = a wrong FFT bin layout / a
//      workspace overrun, the things ears hear as "sounding not right").
//   3. STABILITY: 6 s of noise in spectral mode stays bounded.
// Compile plain for the laws, with -fsanitize=address for the overrun trip.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "clouds/dsp/granular_processor.h"

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

static const size_t kLarge = 131072;
static const size_t kSmall = 65536;
static const size_t kBlock = 32;
static const float kSr = 32000.0f;

static uint32_t g_seed = 0x1fff7a10u;
static float noise() {
  g_seed = g_seed * 1664525u + 1013904223u;
  return (float)(int32_t)g_seed / (float)0x40000000;
}

struct Rig {
  uint8_t large[kLarge];
  uint8_t small[kSmall];
  clouds::GranularProcessor processor;
  float in_l[kBlock], in_r[kBlock];
  float out_l[kBlock], out_r[kBlock];
  float last_l[kBlock];

  void init() {
    memset(large, 0, sizeof(large));
    memset(small, 0, sizeof(small));
    processor.Init(&large[0], kLarge, &small[0], kSmall);
    processor.set_sample_rate(kSr);
    processor.set_playback_mode(clouds::PLAYBACK_MODE_SPECTRAL);
    processor.set_quality(0);
    clouds::Parameters* p = processor.mutable_parameters();
    // The rack's factory defaults (module_clouds.h init_at).
    p->position = 0.5f;
    p->size = 0.5f;
    p->pitch = 0.5f;
    p->density = 0.692f;
    p->texture = 0.666f;
    p->dry_wet = 0.773f;
    p->stereo_spread = 0.690f;
    p->feedback = 0.097f;
    p->reverb = 0.0f;
    p->freeze = false;
    p->trigger = false;
    p->note = 48.0f;
    processor.Prepare();
  }

  void set_dry_wet(float v) { processor.mutable_parameters()->dry_wet = v; }

  void process() {
    memcpy(last_l, out_l, sizeof(last_l));
    processor.Process(processor.mutable_parameters(),
                      in_l, in_r, out_l, out_r, kBlock);
  }
};

static float block_rms(const float* x, size_t n) {
  float e = 0.0f;
  for (size_t i = 0; i < n; ++i) e += x[i] * x[i];
  return sqrtf(e / (float)n);
}

int main() {
  // 1) DRY PLUMBING: dry_wet 0, a sine in, the same sine out.
  {
    Rig rig;
    rig.init();
    rig.set_dry_wet(0.0f);
    const double w = 2.0 * 3.141592653589793 * 440.0 / (double)kSr;
    double phase = 0.0;
    for (size_t b = 0; b < 200; ++b) {
      for (size_t i = 0; i < kBlock; ++i) {
        phase += w;
        rig.in_l[i] = rig.in_r[i] = 0.5f * (float)sin(phase);
      }
      rig.process();
      for (size_t i = 0; i < kBlock; ++i) {
        if (fabsf(rig.out_l[i] - rig.in_l[i]) > 1e-4f) {
          CHECK(false, "dry out[%zu] = %.6f, want %.6f (block %zu)",
                i, rig.out_l[i], rig.in_l[i], b);
          b = 999;
          break;
        }
      }
    }
    printf("dry plumbing: sample-exact\n");
  }

  // 2) SPECTRAL IDENTITY: dry_wet 1, pitch center. After ~0.5 s warmup the
  //    wet path must pass the 440 Hz sine with a comparable RMS, bounded
  //    and finite.
  float wet_rms = 0.0f, max_out = 0.0f;
  {
    Rig rig;
    rig.init();
    rig.set_dry_wet(1.0f);
    const double w = 2.0 * 3.141592653589793 * 440.0 / (double)kSr;
    double phase = 0.0;
    int nan_seen = 0;
    double rms_acc = 0.0;
    size_t rms_n = 0;
    for (size_t b = 0; b < 1500; ++b) {   // ~1.5 s
      for (size_t i = 0; i < kBlock; ++i) {
        phase += w;
        rig.in_l[i] = rig.in_r[i] = 0.5f * (float)sin(phase);
      }
      rig.process();
      for (size_t i = 0; i < kBlock; ++i) {
        const float o = rig.out_l[i];
        if (!std::isfinite(o)) ++nan_seen;
        if (fabsf(o) > max_out) max_out = fabsf(o);
      }
      if (b > 500) {   // past the STFT warmup: measure the tail
        rms_acc += (double)block_rms(rig.last_l, kBlock);
        ++rms_n;
      }
    }
    wet_rms = (float)(rms_acc / (double)(rms_n ? rms_n : 1));
    CHECK(nan_seen == 0, "%d non-finite spectral output samples", nan_seen);
    CHECK(max_out < 4.0f, "spectral output unbounded: peak %f", max_out);
    CHECK(wet_rms > 0.15f && wet_rms < 1.2f,
          "spectral sine RMS %.4f out of the plausible band (0.15..1.2) — "
          "garbled STFT?", wet_rms);
    printf("spectral identity: rms %.3f, peak %.3f, nan %d\n",
           wet_rms, max_out, nan_seen);
  }

  // 3) STABILITY: noise through spectral mode stays bounded.
  {
    Rig rig;
    rig.init();
    rig.set_dry_wet(1.0f);
    float peak = 0.0f;
    for (size_t b = 0; b < 6000; ++b) {   // 6 s
      for (size_t i = 0; i < kBlock; ++i) {
        rig.in_l[i] = rig.in_r[i] = noise() * 0.5f;
      }
      rig.process();
      for (size_t i = 0; i < kBlock; ++i) {
        const float a = fabsf(rig.out_l[i]);
        if (!std::isfinite(a)) peak = 1e9f;
        if (a > peak) peak = a;
      }
    }
    CHECK(peak < 4.0f, "spectral + noise unbounded: peak %f", peak);
    printf("stability: noise peak %.3f\n", peak);
  }

  if (g_fail) {
    fprintf(stderr, "test_clouds_spectral: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_clouds_spectral: all laws hold\n");
  return 0;
}
