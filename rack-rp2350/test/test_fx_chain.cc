// T9 chain-law tests: REVERB IS THE LAST FX in the Elements chain.
//
// Laws (user directive 2026-11-17: "reverb should be last fx in the chain.
// distortion and warps should be BEFORE!"):
//   1. STRUCTURAL — the adapter insert runs on the PRE-reverb mixdown, not on
//      the tail. Proof: during gate-off stretches the insert's input is
//      ~silent (voices render nothing) while Process's OUTPUT still rings
//      (the reverb tail) — impossible if the insert sat after reverb_.
//   2. Bypass law: an inert insert (drive 0, rack_dist's zero-cost bypass)
//      and a capture-only insert both render BIT-EXACT vs no insert.
//   3. Active law: drive 0.55 pre-reverb differs from the raw render (the
//      shaper really is in the path) and stays bounded. Informational: the
//      reverb OF a shaped signal has a denser tail (deep-tail crest 2.0 vs
//      3.8 clean on this rig) — that is the musical meaning of fuzz-into-
//      reverb, not tail corruption; nothing shapes the tail itself.
//
// Same render rig as test_warps_stage.cc: real Part (strike exciter, modal
// resonator, space 0.95), 16-sample blocks.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "elements/dsp/part.h"
#include "rack_dist.h"          // -I../src (the adapter's shaper)

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

using namespace elements;

float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

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

// --- adapter inserts (exactly what module_elements.h installs) ---
static rackdist::Params g_params;            // set before each render

static void shaper_insert(float* main, float* aux, size_t size, void* user) {
  rackdist::process(static_cast<rackdist::State*>(user), g_params, main, aux,
                    size);
}

// Capture-only insert: records its PRE-reverb input, touches nothing.
static std::vector<float> g_insert_main;     // pre-reverb mixdown, per sample
static std::vector<float> g_output_main;     // post-Process, per sample
static bool g_capture_output = false;        // record output only during the
                                             // capture rig's render
static void capture_insert(float* main, float* aux, size_t size, void*) {
  (void)aux;
  g_insert_main.insert(g_insert_main.end(), main, main + size);
}

static void init_rig(Rig* r, Part::PreReverbInsert insert, void* user) {
  // Part::Init does not zero panic_ (upstream leaves it to the caller's
  // zeroed arena in firmware); the stack test rig must zero it or the first
  // block may take the panic/bypass path and skip the insert entirely.
  memset(r, 0, sizeof(*r));
  memset(r->reverb, 0, sizeof(r->reverb));
  r->part.Init(r->reverb, nullptr);          // no warps modulator
  uint32_t seed[3] = {0x1fff7a10, 0x1fff7a10, 0x1fff7a10};
  r->part.Seed(seed, 3);
  r->part.set_easter_egg(false);
  r->part.set_resonator_model(RESONATOR_MODEL_MODAL);
  Patch* p = r->part.mutable_patch();
  p->exciter_strike_level = 0.8f;
  p->resonator_damping = 0.25f;
  p->space = 0.95f;                          // the user's high-space case
  p->warps_algorithm = 0.0f;
  r->perf.strength = 0.8f;
  r->perf.modulation = 0.0f;
  memset(r->blow, 0, sizeof(r->blow));
  memset(r->strike, 0, sizeof(r->strike));
  r->part.set_pre_reverb_insert(insert, user);
}

static void render(Rig* r, std::vector<float>* out) {
  const size_t kBlocks = 16000;              // 256000 samples = 8 s
  for (size_t b = 0; b < kBlocks; ++b) {
    const size_t t = b * kBlock;
    r->perf.gate = ((t % 8000) < 4000);      // 250 ms on / 250 ms off
    r->perf.note = 48.0f;
    r->part.Process(r->perf, r->blow, r->strike, r->main, r->aux, kBlock);
    for (size_t i = 0; i < kBlock; ++i) {
      out->push_back(r->main[i]);
      out->push_back(r->aux[i]);
    }
    if (g_capture_output)
      g_output_main.insert(g_output_main.end(), r->main, r->main + kBlock);
  }
}

int main() {
  Rig raw_rig, bypass_rig, shaped_rig, capture_rig;
  std::vector<float> raw, bypass, shaped;

  init_rig(&raw_rig, nullptr, nullptr);
  render(&raw_rig, &raw);

  // Law 1: capture-only insert — structurally pre-reverb, bit-exact output.
  init_rig(&capture_rig, capture_insert, nullptr);
  g_capture_output = true;
  render(&capture_rig, &bypass);             // reuse buffer name: inert too
  g_capture_output = false;

  // Law 2: inert shaper (drive 0) — bit-exact vs no insert.
  static rackdist::State bypass_dist;
  memset(&bypass_dist, 0, sizeof(bypass_dist));
  g_params = {};
  g_params.type = 0;
  g_params.drive = 0.0f;
  g_params.tone = 1.0f;
  init_rig(&bypass_rig, shaper_insert, &bypass_dist);
  std::vector<float> bypass_out;
  render(&bypass_rig, &bypass_out);

  // Law 3: active shaper (drive 0.55) pre-reverb.
  static rackdist::State shaped_dist;
  memset(&shaped_dist, 0, sizeof(shaped_dist));
  g_params = {};
  g_params.type = 0;                         // kTube
  g_params.drive = 0.55f;
  g_params.bias = 0.0f;
  g_params.tone = 1.0f;
  init_rig(&shaped_rig, shaper_insert, &shaped_dist);
  render(&shaped_rig, &shaped);

  // Law 2: both inert inserts are bit-exact vs no insert.
  CHECK(raw == bypass, "capture-only insert changed the output");
  CHECK(raw == bypass_out,
        "drive-0 insert is NOT bit-exact vs no insert — the bypass leaks");

  // Law 1 (structural): during gate-off stretches the insert's input is
  // ~silent while the Process output still rings the reverb tail.
  CHECK(g_insert_main.size() == g_output_main.size(),
        "insert/output capture lengths differ: %zu vs %zu",
        g_insert_main.size(), g_output_main.size());
  // Gate-off = samples [4000, 8000) of each 8000-sample period. Give the
  // exciter 1000 samples to fully die after gate-off, then require a window
  // where the pre-reverb input is tiny and the output is a live tail.
  size_t proven = 0;
  const float kInsertSilent = 0.001f;        // pre-reverb mixdown ~0
  const float kTailAlive = 0.01f;            // reverb output clearly ringing
  for (size_t t = 0; t + 1 < g_output_main.size(); ++t) {
    const size_t phase = t % 8000;
    if (phase >= 5000 && phase < 8000 - 1) {
      if (fabsf(g_insert_main[t]) < kInsertSilent &&
          fabsf(g_output_main[t]) > kTailAlive) {
        ++proven;
      }
    }
  }
  printf("structural proof samples (insert silent while tail rings): %zu\n",
         proven);
  // Single 8 s render = 32 gate-off windows. ~6k proof samples is ample;
  // the old 13,780 figure counted the same evidence four times because
  // g_output_main was accidentally accumulated across all four renders.
  CHECK(proven > 4000,
        "insert does not behave as PRE-reverb: only %zu samples show a "
        "silent insert input under a live tail", proven);

  // Law 3: active shaper is in the path and stays bounded.
  CHECK(!(shaped == raw),
        "drive-0.55 pre-reverb insert rendered identical to raw — "
        "the shaper did not run");
  float peak = 0.0f;
  for (size_t i = 0; i < shaped.size(); i += 2) {
    const float a = fabsf(shaped[i]);
    if (a > peak) peak = a;
  }
  CHECK(peak < 2.0f, "shaped render peak %.3f out of range", peak);

  // Informational: reverb-of-shaped-signal tail density (NOT a law — this is
  // the correct musical consequence of fuzz-into-reverb).
  auto deep_tail_crest = [](const std::vector<float>& v) {
    const size_t kTail = 64000;              // last 2 s interleaved stereo
    const size_t start = v.size() / 2 - kTail;
    float pk = 0.0f, sum2 = 0.0f;
    for (size_t i = start; i < v.size(); i += 2) {
      const float a = fabsf(v[i]);
      if (a > pk) pk = a;
      sum2 += a * a;
    }
    return pk / (sqrtf(sum2 / (kTail / 2)) + 1e-9f);
  };
  printf("deep-tail crest: raw %.2f | shaped-pre-reverb %.2f "
         "(informational — reverb of a shaped signal is denser by design)\n",
         deep_tail_crest(raw), deep_tail_crest(shaped));

  if (g_fail) {
    fprintf(stderr, "test_fx_chain: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_fx_chain: all laws hold (reverb is the last FX)\n");
  return 0;
}
