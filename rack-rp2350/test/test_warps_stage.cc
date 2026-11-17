// T9 TDD unit tests for the warps cross-modulation stage inside
// elements::Part (rack build; compiled with -DRACK_RP2350_TC).
//
// Laws under test (docs/superpowers/plans/2026-11-17-elements-warps.md):
//   - Part::Init(reverb_buffer, warps::Modulator*) — pool-placed modulator,
//     nullptr = stage off (standalone-compatible path stays 1-arg);
//   - warps_algorithm == 0 -> stage bypass with ZERO observable effect
//     (bit-exact main/aux vs a nullptr-warp Part);
//   - the stage touches ONLY main (aux passes through untouched);
//   - the algorithm is clamped to 0.625 (pure comparator): alg 1.0 must be
//     indistinguishable from alg 0.625 (the vocoder zone is excluded);
//   - with the modulator level at 0 the warps output is exactly 0, so
//     mix=0.5 renders main == 0.5 * bypass-main exactly (w = 0 exactly;
//     x - x*0.5 == x*0.5 in IEEE).
//
// The reverb is at space = 0 for every scenario: with reverb_amount = 0 the
// reverb is a bit-exact pass-through, so post-stage state never feeds back
// into later blocks and cross-run comparisons stay valid.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <new>

#include "elements/dsp/part.h"

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

// stmlib's class-static LUT tables (as the firmware wrapper compiles them).
#include "stmlib/dsp/atan.inc"
#include "stmlib/dsp/units.inc"
#include "stmlib/utils/random.inc"

#include "warps/dsp/modulator.h"

using namespace elements;

// Firmware-side definitions of the elements::Dsp statics — declared in
// elements/dsp/dsp.h, defined nowhere in the library (module_elements.h
// defines them the same way; the out-of-class definition is the firmware's
// established pattern).
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

static const size_t kBlock = 16;       // elements kMaxBlockSize (rack)
static const size_t kReverbFrames = 21632;

struct Rig {
  uint16_t reverb[kReverbFrames];
  elements::Part part;
  elements::PerformanceState perf;
  float blow[kBlock];
  float strike[kBlock];
  float main[kBlock];
  float aux[kBlock];
};

static const uint32_t kSeed[3] = {0x1fff7a10, 0x1fff7a10, 0x1fff7a10};

// Fresh rig: factory Part init + rack seed + space 0 (pass-through reverb) +
// strike 0.8 (excited), warps stage attached only when warps_mod != nullptr.
static void rig_init(Rig* r, warps::Modulator* warps_mod) {
  memset(r->reverb, 0, sizeof(r->reverb));
  memset(r->blow, 0, sizeof(r->blow));
  memset(r->strike, 0, sizeof(r->strike));
  r->part.Init(r->reverb, warps_mod);
  uint32_t seed[3] = {kSeed[0], kSeed[1], kSeed[2]};
  r->part.Seed(seed, 3);
  elements::Patch* p = r->part.mutable_patch();
  p->space = 0.0f;
  p->warps_algorithm = 0.0f;
  p->warps_timbre = 0.0f;
  p->warps_mod_level = 0.0f;
  p->warps_car_level = 0.0f;
  p->warps_mix = 0.0f;
  r->perf.gate = true;
  r->perf.note = 48.0f;
  r->perf.strength = 0.8f;
  r->perf.modulation = 0.0f;
}

// 64 blocks, mallet retriggers every 8 blocks (a single impulse leaves the
// resonator near-silent — the stage scenarios need sustained material).
static void rig_run(Rig* r, int blocks) {
  for (int b = 0; b < blocks; ++b) {
    for (size_t i = 0; i < kBlock; ++i) {
      r->strike[i] = (b % 8 == 1 && i == 0) ? 0.9f : 0.0f;
    }
    r->part.Process(r->perf, r->blow, r->strike, r->main, r->aux, kBlock);
  }
}

static float block_peak(const float* x, size_t n) {
  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) peak = peak > fabsf(x[i]) ? peak : fabsf(x[i]);
  return peak;
}

static void test_bypass_is_bit_exact() {
  static Rig base;                 // nullptr warps: the standalone path
  static Rig staged;               // warps attached, algorithm 0 = bypass
  static uint8_t wstorage[sizeof(warps::Modulator)];
  warps::Modulator* wm = new (wstorage) warps::Modulator();

  rig_init(&base, nullptr);
  rig_init(&staged, wm);
  staged.part.mutable_patch()->warps_mix = 1.0f;   // even at full mix
  rig_run(&base, 64);
  rig_run(&staged, 64);

  CHECK(memcmp(base.main, staged.main, sizeof(base.main)) == 0,
        "bypass (alg 0) main differs from the warps-less Part");
  CHECK(memcmp(base.aux, staged.aux, sizeof(base.aux)) == 0,
        "bypass (alg 0) aux differs from the warps-less Part");
}

static void test_stage_modifies_main_only() {
  static Rig base;
  static Rig staged;
  static uint8_t wstorage[sizeof(warps::Modulator)];
  warps::Modulator* wm = new (wstorage) warps::Modulator();

  rig_init(&base, nullptr);
  rig_init(&staged, wm);
  elements::Patch* pb = base.part.mutable_patch();
  elements::Patch* ps = staged.part.mutable_patch();
  pb->warps_algorithm = 0.0f;
  ps->warps_algorithm = 0.25f;      // diode ring modulation zone
  ps->warps_mod_level = 1.0f;
  ps->warps_car_level = 1.0f;
  ps->warps_mix = 1.0f;
  rig_run(&base, 64);
  rig_run(&staged, 64);

  float base_peak = block_peak(base.main, kBlock);
  CHECK(base_peak > 0.05f,
        "scenario sanity: bypass main is near-silent (peak %.6f) — the "
        "strike did not excite the resonator, 'differs' asserts are vacuous",
        base_peak);
  CHECK(memcmp(base.main, staged.main, sizeof(base.main)) != 0,
        "warps at alg 0.25 / mix 1 must transform main");
  // Warps mode consumes the raw exciter as the modulator and forces aux to
  // the clean stereo resonator (no raw-exciter bleed into reverb/distortion),
  // so aux MUST differ from the warps-less Part at space=0.
  CHECK(memcmp(base.aux, staged.aux, sizeof(base.aux)) != 0,
        "warps mode must keep the raw modulator out of aux");
  for (size_t i = 0; i < kBlock; ++i) {
    CHECK(std::isfinite(staged.main[i]), "non-finite staged main[%zu]", i);
  }
}

static void test_algorithm_clamped_to_comparator() {
  static Rig a;
  static Rig b;
  static uint8_t wstorage[sizeof(warps::Modulator)];
  warps::Modulator* wm = new (wstorage) warps::Modulator();

  rig_init(&a, wm);
  a.part.mutable_patch()->warps_algorithm = 1.0f;    // deep in the vocoder zone
  a.part.mutable_patch()->warps_mod_level = 1.0f;
  a.part.mutable_patch()->warps_car_level = 1.0f;
  a.part.mutable_patch()->warps_mix = 1.0f;
  rig_run(&a, 64);

  rig_init(&b, wm);
  b.part.mutable_patch()->warps_algorithm = 0.625f;  // pure comparator (clamp)
  b.part.mutable_patch()->warps_mod_level = 1.0f;
  b.part.mutable_patch()->warps_car_level = 1.0f;
  b.part.mutable_patch()->warps_mix = 1.0f;
  rig_run(&b, 64);

  CHECK(memcmp(a.main, b.main, sizeof(a.main)) == 0,
        "alg 1.0 must render exactly like the 0.625 clamp");
}

static void test_silent_modulator_halves_main_exactly() {
  static Rig base;
  static Rig staged;
  static uint8_t wstorage[sizeof(warps::Modulator)];
  warps::Modulator* wm = new (wstorage) warps::Modulator();

  rig_init(&base, nullptr);
  rig_init(&staged, wm);
  elements::Patch* ps = staged.part.mutable_patch();
  ps->warps_algorithm = 0.375f;     // digital ring mod: ring = 4*x1*x2/(1+|..|)
  ps->warps_mod_level = 0.0f;       // modulator path silent -> warps out = 0
  ps->warps_car_level = 1.0f;
  ps->warps_mix = 0.5f;
  rig_run(&base, 64);
  rig_run(&staged, 64);

  for (size_t i = 0; i < kBlock; ++i) {
    if (staged.main[i] != base.main[i] * 0.5f) {
      CHECK(false, "staged main[%zu] = %.9g, expected exactly %.9g "
                   "(zero warps output at mix 0.5)", i, staged.main[i],
            base.main[i] * 0.5f);
      break;
    }
  }
}

int main() {
  test_bypass_is_bit_exact();
  test_stage_modifies_main_only();
  test_algorithm_clamped_to_comparator();
  test_silent_modulator_halves_main_exactly();
  if (g_fail) {
    fprintf(stderr, "test_warps_stage: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_warps_stage: all checks passed\n");
  return 0;
}
