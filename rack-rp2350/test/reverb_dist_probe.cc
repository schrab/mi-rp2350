// Probe: what does the always-on post-reverb rackdist stage do to the
// Elements reverb tail? Renders the SAME musical phrase twice through the
// real Part (strike exciter, modal resonator, reverb at space 0.95) —
//   A) raw Part output (pre-rackdist; the true upstream reverb tail)
//   B) firmware path: rackdist tube @ boot defaults (drive 0.55, tone 1.0)
// Writes A/B WAVs and prints tail metrics (peak/RMS/crest + clip fraction).
// The stage is the ONLY delta vs the pre-FX-session sound, so A = "how it
// sounded all the time before", B = what the user hears now.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "elements/dsp/part.h"
#include "rack_dist.h"          // -I../src

// Mirror lib/elements/elements_all.cpp: single-TU compilation of the port.
#include "elements/dsp/exciter.inc"
#include "elements/dsp/multistage_envelope.inc"
#include "elements/dsp/ominous_voice.inc"
#include "elements/dsp/part.inc"
#include "elements/dsp/resonator.inc"
#include "elements/dsp/string.inc"
#include "elements/dsp/tube.inc"
#include "elements/dsp/voice.inc"
#include "elements/resources.inc"

#include "stmlib/dsp/atan.inc"
#include "stmlib/dsp/units.inc"
#include "stmlib/utils/random.inc"

#include "wav_io.h"

using namespace elements;

float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

static const size_t kBlock = 16;
static const size_t kReverbFrames = 21632;

struct Rig {
  uint16_t reverb[kReverbFrames];
  Part part;
  PerformanceState perf;
  float blow[kBlock];
  float strike[kBlock];
  float main[kBlock];
  float aux[kBlock];
};

int main() {
  Rig rig_a, rig_b;
  // identical init for both arms (rack init_at sequence, defaults)
  for (Rig* r : {&rig_a, &rig_b}) {
    memset(r->reverb, 0, sizeof(r->reverb));
    r->part.Init(r->reverb, nullptr);        // nullptr warps: stage off
    uint32_t seed[3] = {0x1fff7a10, 0x1fff7a10, 0x1fff7a10};
    r->part.Seed(seed, 3);
    r->part.set_easter_egg(false);
    r->part.set_resonator_model(RESONATOR_MODEL_MODAL);
    Patch* p = r->part.mutable_patch();
    p->exciter_strike_level = 0.8f;
    p->resonator_damping = 0.25f;
    p->space = 0.95f;                        // the user's high-space case
    p->warps_algorithm = 0.0f;               // no warps stage (nullptr Init)
    r->perf.strength = 0.8f;
    r->perf.modulation = 0.0f;
    memset(r->blow, 0, sizeof(r->blow));
    memset(r->strike, 0, sizeof(r->strike));
  }

  // Phrase: 8 s; gate 250 ms on / 250 ms off (8000-sample period), note 48.
  const size_t kBlocks = 16000;              // 16000*16 = 256000 samples = 8 s
  std::vector<float> raw, dist;              // interleaved L,R
  raw.reserve(kBlocks * kBlock * 2);
  dist.reserve(kBlocks * kBlock * 2);

  rackdist::State dist_state;
  memset(&dist_state, 0, sizeof(dist_state));
  rackdist::Params dp{};
  dp.type = 0;                               // kTube (boot default fx_type=0)
  dp.drive = 0.55f;                          // boot default
  dp.bias = 0.0f;
  dp.tone = 1.0f;                            // open — boot default

  float peak_raw = 0, peak_dist = 0, sum2_raw = 0, sum2_dist = 0;
  size_t clip_raw = 0, clip_dist = 0, n = 0;
  for (size_t b = 0; b < kBlocks; ++b) {
    const size_t t = b * kBlock;
    bool gate = ((t % 8000) < 8000 / 2);     // 250 ms on / 250 ms off
    for (Rig* r : {&rig_a, &rig_b}) {
      r->perf.gate = gate;
      r->perf.note = 48.0f;
      r->part.Process(r->perf, r->blow, r->strike, r->main, r->aux, kBlock);
    }
    rackdist::process(&dist_state, dp, rig_b.main, rig_b.aux, kBlock);
    for (size_t i = 0; i < kBlock; ++i) {
      raw.push_back(rig_a.main[i]);  raw.push_back(rig_a.aux[i]);
      dist.push_back(rig_b.main[i]); dist.push_back(rig_b.aux[i]);
      float a = fabsf(rig_a.main[i]), b2 = fabsf(rig_b.main[i]);
      if (a > peak_raw) peak_raw = a;
      if (b2 > peak_dist) peak_dist = b2;
      if (a > 1.0f) ++clip_raw;
      if (b2 > 1.0f) ++clip_dist;
      sum2_raw += a * a; sum2_dist += b2 * b2;
      ++n;
    }
  }
  l8test::WriteWav16("build/tail_raw.wav", raw.data(),
                     (int)(raw.size() / 2), 2, 32000);
  l8test::WriteWav16("build/tail_distorted.wav", dist.data(),
                     (int)(dist.size() / 2), 2, 32000);
  float rms_raw = sqrtf(sum2_raw / n), rms_dist = sqrtf(sum2_dist / n);
  printf("samples: %zu (%.1f s)\n", n, (float)n / 32000.0f);
  printf("RAW   (pre-rackdist, upstream tail): peak %.3f  rms %.4f  crest %.2f  over-1.0 %.3f%%\n",
         peak_raw, rms_raw, peak_raw / (rms_raw + 1e-9f), 100.0f * clip_raw / n);
  printf("DIST  (tube d=0.55, boot default):    peak %.3f  rms %.4f  crest %.2f  over-1.0 %.3f%%\n",
         peak_dist, rms_dist, peak_dist / (rms_dist + 1e-9f), 100.0f * clip_dist / n);
  return 0;
}
