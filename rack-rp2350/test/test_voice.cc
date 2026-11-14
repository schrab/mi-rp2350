// TDD unit tests for the LIRA-8 voice layer (LiraOsc + VoicePair).
// Every assert encodes math verified against the patch (take-1/TAKE1 notes,
// connect lists) — see test/lira8_ref/PORTING-SPEC.md + PD-PRIMITIVES.md.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "lira_osc.h"
#include "lira_voice.h"

using namespace l8pd;

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

static bool near(float a, float b, float tol) {
  return std::fabs(a - b) <= tol;
}

// rising zero crossings per second ~= frequency
static double measure_freq_sq(LiraOsc& o, float freq, float pwm, float seconds,
                              float sr) {
  const int n = static_cast<int>(seconds * sr);
  int crossings = 0;
  float sq, tri, prev = 0.f;
  for (int i = 0; i < n; ++i) {
    o.process(freq, pwm, sq, tri);
    if (prev <= 0.f && sq > 0.f) ++crossings;
    prev = sq;
  }
  return crossings / seconds;
}

static void test_lira_osc() {
  const float sr = 32000.f;
  LiraOsc o(sr);
  // frequency tracking (square, zero crossings), skip 50 ms settle
  LiraOsc o1(sr);
  measure_freq_sq(o1, 220.f, 0.53125f, 0.05f, sr);  // warmup
  double f = measure_freq_sq(o1, 220.f, 0.53125f, 1.0f, sr);
  CHECK(std::fabs(f - 220.0) < 3.0, "osc freq ~220 (got %.2f)", f);

  LiraOsc o2(sr);
  measure_freq_sq(o2, 110.f, 0.5f, 0.05f, sr);
  f = measure_freq_sq(o2, 110.f, 0.5f, 1.0f, sr);
  CHECK(std::fabs(f - 110.0) < 2.0, "osc freq ~110 (got %.2f)", f);

  // PWM duty: mean of the square tracks (2*pw - 1) roughly (before 0.59)
  LiraOsc o3(sr);
  double acc = 0.0;
  int n = static_cast<int>(0.5 * sr);
  for (int i = 0; i < n; ++i) {
    float sq, tri;
    o3.process(220.f, 0.9f, sq, tri);
    acc += sq;
  }
  const double mean_pw09 = acc / n / 0.59;
  // pulse duty 0.9: mean ~ (2*0.9-1) - small blep/pwm-sample offsets
  CHECK(mean_pw09 > 0.5, "PWM 0.9 raises square mean (got %.3f)", mean_pw09);

  // bounds: blep keeps the square in [-0.59*3, 0.59*3] worst case
  LiraOsc o4(sr);
  float worst = 0.f;
  for (int i = 0; i < static_cast<int>(0.2 * sr); ++i) {
    float sq, tri;
    o4.process(1000.f, 0.5f, sq, tri);
    worst = std::max(worst, std::fabs(sq));
  }
  CHECK(worst < 0.59f * 3.0f, "square bounded (got %.3f)", worst);

  // triangle is smoothed: sample-to-sample delta of tri << pulse's
  LiraOsc o5(sr);
  float max_dt = 0.f;
  float ptri = 0.f;
  for (int i = 0; i < static_cast<int>(0.2 * sr); ++i) {
    float sq, tri;
    o5.process(500.f, 0.5f, sq, tri);
    max_dt = std::max(max_dt, std::fabs(tri - ptri));
    ptri = tri;
  }
  CHECK(max_dt < 0.3f, "triangle smooth (max delta %.4f)", max_dt);
}

static void test_sensor_env() {
  const float sr = 32000.f;
  SensorEnv e(sr);
  e.set_fast(true);           // attack 100 ms
  e.set(true);
  float e2 = 0.f, cos_t = 0.f;
  for (int i = 0; i < static_cast<int>(0.05 * sr); ++i) e.process(e2, cos_t);
  // 50 ms into a 100 ms linear attack: e = 0.5 -> e^2 = 0.25
  CHECK(near(e2, 0.25f, 0.02f), "sensor e^2 at 50ms/100ms attack (got %.4f)",
        e2);
  // cos morph: e=0.5 -> cos(2*pi*0.25)/2 = 0
  CHECK(std::fabs(cos_t) < 0.05f, "cos term zero at e=0.5 (got %.4f)", cos_t);
  // finish the attack to e = 1
  for (int i = 0; i < static_cast<int>(0.06 * sr); ++i) e.process(e2, cos_t);
  // release 8000 ms (slow): after 100 ms of release, e = 1 - 0.1/8 = 0.9875
  e.set_fast(false);          // switch to slow: release_ms 8000
  e.set(false);
  for (int i = 0; i < static_cast<int>(0.1 * sr); ++i) e.process(e2, cos_t);
  CHECK(e2 > 0.94f, "slow release holds e^2 (got %.4f)", e2);
  // fast release 100 ms: after 100 ms, e = 0
  e.set_fast(true);
  e.set(true);
  for (int i = 0; i < static_cast<int>(0.15 * sr); ++i) e.process(e2, cos_t);
  e.set(false);  // release 100 ms
  for (int i = 0; i < static_cast<int>(0.15 * sr); ++i) e.process(e2, cos_t);
  CHECK(e2 < 0.01f, "fast release decays (got %.4f)", e2);
}

static void test_voice_pair() {
  const float sr = 32000.f;
  VoicePair vp(sr, 0, 0u);
  // defaults: sensors off -> silence
  float buf[64], bus[64];
  float mod_sig[64];
  for (int i = 0; i < 64; ++i) mod_sig[i] = 0.f;
  double acc = 0.0;
  for (int b = 0; b < 20; ++b) {
    vp.process(mod_sig, 64, buf, bus);
    for (int i = 0; i < 64; ++i) acc += std::fabs(buf[i]);
  }
  CHECK(acc == 0.0, "sensors off -> silent pair (sum %f)", acc);

  // hold=0 gates the pair even with sensors on
  VoicePair vp2(sr, 0, 0u);
  vp2.set_hold(0);
  vp2.set_sensor(0, true);
  vp2.set_sensor(1, true);
  acc = 0.0;
  for (int b = 0; b < 10; ++b) {
    vp2.process(mod_sig, 64, buf, bus);
    for (int i = 0; i < 64; ++i) acc += std::fabs(buf[i]);
  }
  CHECK(acc == 0.0, "hold=0 gates pair (sum %f)", acc);

  // sensors on + hold open + sharp=1 -> square content, audible
  VoicePair vp3(sr, 0, 0u);
  vp3.set_hold(127);          // hold = 1
  vp3.set_sharp(127);         // sharp = 1 -> pure square
  vp3.set_sensor(0, true);
  vp3.set_sensor(1, true);
  vp3.set_tune(0, 64);
  vp3.set_tune(1, 64);
  vp3.set_pitch(64);
  vp3.refresh_tune();
  acc = 0.0;
  for (int b = 0; b < 100; ++b) {   // 200 ms: attack settles
    vp3.process(mod_sig, 64, buf, bus);
    for (int i = 0; i < 64; ++i) acc += std::fabs(buf[i]);
  }
  CHECK(acc > 100.0, "open pair renders audio (sum %f)", acc);

  // bus_send = hip~3(pair): DC-free
  double bus_mean = 0.0;
  int cnt = 0;
  for (int b = 0; b < 50; ++b) {
    vp3.process(mod_sig, 64, buf, bus);
    for (int i = 0; i < 64; ++i) {
      bus_mean += bus[i];
      ++cnt;
    }
  }
  CHECK(std::fabs(bus_mean / cnt) < 0.05, "bus DC-blocked (mean %f)",
        bus_mean / cnt);

  // tune table: voice 1 (pair 0 osc 0) tune=0 -> note 93 -> mtof(93) ~ 627 Hz
  VoicePair vp4(sr, 0, 0u);
  vp4.set_tune(0, 0);
  vp4.set_tune(1, 0);         // both oscs at note 93 -> same freq
  vp4.set_pitch(64);          // pitch ~ 1.0134
  vp4.refresh_tune();
  // measure via freq of the rendered square
  vp4.set_hold(127);
  vp4.set_sharp(127);
  vp4.set_sensor(0, true);
  vp4.set_vibrato(0);
  // warm up the 23.22 ms tune line
  for (int b = 0; b < 20; ++b) vp4.process(mod_sig, 64, buf, bus);
  // count square zero crossings via bus (hip~3 keeps edges at 220 Hz)
  int crossings = 0;
  float prev = 0.f;
  const double secs = 1.0;
  for (int b = 0; b < static_cast<int>(secs * sr / 64); ++b) {
    vp4.process(mod_sig, 64, buf, bus);
    for (int i = 0; i < 64; ++i) {
      if (prev <= 0.f && buf[i] > 0.f) ++crossings;
      prev = buf[i];
    }
  }
  // identical oscs (note 93 -> ~1761 Hz * pitch 1.0134 ~ 1785): the sum of
  // two in-phase squares crosses once per cycle each -> ~1785 crossings/s
  const double fmeas = crossings / secs;
  CHECK(fmeas > 1750.0 && fmeas < 1820.0, "tune table freq ~1785 (got %.1f)",
        fmeas);
}

static void test_pd_random() {
  // pd_random LCG: determinism + range (kept for future reseed support)
  uint32_t st = 0u + 1u;
  int r = pd_random(&st, 1001);
  CHECK(r >= 0 && r < 1001, "pd_random in range (%d)", r);
  uint32_t st2 = 0u + 1u;
  CHECK(pd_random(&st2, 1001) == r, "pd_random deterministic");
  // vibrato depths: the patch's $0-seed receiver is DORMANT, so the four
  // [random 1001] draws are pd's measured default-PRNG constants
  // (744/930/728/920, probed 2026-11-15) -> depths 0.5 + 3*draw/1000.
  const float expect[4] = {2.732f, 3.290f, 2.684f, 3.260f};
  for (int p = 0; p < 4; ++p) {
    VoicePair vp(32000.f, p, 0u);
    (void)vp;
    float d = 0.5f + 3.0f * kVibDraws[p] / 1000.0f;
    CHECK(near(d, expect[p], 1e-3f), "vib depth pair %d (got %.4f)", p, d);
  }
}

int main() {
  test_lira_osc();
  test_sensor_env();
  test_voice_pair();
  test_pd_random();
  if (g_fail) {
    fprintf(stderr, "%d FAILURES\n", g_fail);
    return 1;
  }
  printf("test_voice: all green\n");
  return 0;
}
