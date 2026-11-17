// Host tests for the Rings VARG-chord adapter + the 4 ctag-tbd distortion
// algos. Both are pure headers (no MCU headers) — same portability discipline
// as varg.h / lira_harmony.h.
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../src/rings_chord.h"
#include "../src/rings_dist.h"

static int g_fail = 0;

static void expect_int(const char* what, long got, long want) {
  if (got != want) {
    std::printf("FAIL %s: %ld != %ld\n", what, got, want);
    ++g_fail;
  }
}
static void expect_true(const char* what, bool cond) {
  if (!cond) {
    std::printf("FAIL %s\n", what);
    ++g_fail;
  }
}
static void expect_near(const char* what, float got, float want, float eps) {
  if (std::fabs(got - want) > eps) {
    std::printf("FAIL %s: %f != %f (eps %f)\n", what, got, want, eps);
    ++g_fail;
  }
}

static void test_chord_from_pool() {
  {
    const int8_t pool[] = {0, 3, 7, 12};  // A minor triad
    ringchord::Chord c = ringchord::from_pool(pool, 4, 9, 0, 8);
    expect_int("triad n", c.n, 4);
    expect_near("triad t0", c.tone[0], 45.0f, 0.01f);  // A2
    expect_near("triad t1", c.tone[1], 48.0f, 0.01f);  // C3
    expect_near("triad t2", c.tone[2], 52.0f, 0.01f);  // E3
    expect_near("triad t3", c.tone[3], 57.0f, 0.01f);  // A3
  }
  {
    const int8_t pool[] = {0, 7, 12};  // power chord
    ringchord::Chord c = ringchord::from_pool(pool, 3, 9, 1, 8);
    expect_int("power n", c.n, 3);
    expect_near("power oct t0", c.tone[0], 57.0f, 0.01f);
    expect_near("power oct t2", c.tone[2], 69.0f, 0.01f);
  }
  {
    // pool bigger than voices: keep root/third/fifth, drop color tones
    const int8_t pool[] = {0, 1, 3, 6, 7, 12};
    ringchord::Chord c = ringchord::from_pool(pool, 6, 9, 0, 4);
    expect_int("trunc n", c.n, 4);
    expect_near("trunc t0", c.tone[0], 45.0f, 0.01f);
    expect_near("trunc t3", c.tone[3], 51.0f, 0.01f);
  }
  {
    // snapshot GroupCtx: root = 48 + pc, scale_st = pool
    l8harmony::GroupCtx g{};
    g.root = 57;  // A
    g.nsteps = 3;
    g.scale_st[0] = 0; g.scale_st[1] = 6; g.scale_st[2] = 7;  // tritone
    ringchord::Chord c = ringchord::from_ctx(g, 0, 8);
    expect_int("ctx n", c.n, 3);
    expect_near("ctx t0", c.tone[0], 45.0f, 0.01f);
    expect_near("ctx t1", c.tone[1], 51.0f, 0.01f);
  }
  {
    // all slots clamped to a valid MIDI range even at silly octaves
    const int8_t pool[] = {0, 7, 12};
    ringchord::Chord c = ringchord::from_pool(pool, 3, 11, 12, 8);
    expect_true("clamp hi", c.tone[0] <= 127.0f);
    ringchord::Chord d = ringchord::from_pool(pool, 3, 0, -12, 8);
    expect_true("clamp lo", d.tone[0] >= 0.0f);
  }
}

static void test_shaper_curves() {
  expect_near("softclip 0", ringdist::softclip(0.0f), 0.0f, 1e-6f);
  expect_near("softclip sat+", ringdist::softclip(4.0f), 1.0f, 1e-6f);
  expect_near("softclip sat-", ringdist::softclip(-4.0f), -1.0f, 1e-6f);
  expect_true("softclip bounded", std::fabs(ringdist::softclip(2.0f)) < 1.0f);

  // Vult/Befaco Fold_do reference values
  expect_near("fold even", ringdist::fold(0.5f, 0.0f), 0.5f, 1e-5f);
  expect_near("fold odd+", ringdist::fold(0.25f, 0.5f), 0.75f, 1e-5f);
  expect_near("fold odd-", ringdist::fold(-0.25f, 0.5f), -0.75f, 1e-5f);

  // seeded curve is deterministic and spans the register
  const int32_t* w = ringdist::shape_table();
  expect_true("shape built", w[255] > 0 && w[0] < 0);
  expect_int("shape sentinel", w[256], w[255]);
  expect_true("shape same pointer", ringdist::shape_table() == w);
}

static void test_process_bypass_and_types() {
  float out[8], aux[8];
  for (int i = 0; i < 8; ++i) { out[i] = 0.1f * i - 0.3f; aux[i] = -out[i]; }
  float keep[8], keep_aux[8];
  std::memcpy(keep, out, sizeof(out));
  std::memcpy(keep_aux, aux, sizeof(aux));
  ringdist::State st{{0.0f, 0.0f}};
  ringdist::Params p{};
  p.drive = 0.0f;
  ringdist::process(&st, p, out, aux, 8);
  expect_true("bypass out", std::memcmp(out, keep, sizeof(out)) == 0);
  expect_true("bypass aux", std::memcmp(aux, keep_aux, sizeof(aux)) == 0);
  expect_near("bypass lp", st.lp[0], 0.0f, 1e-9f);

  for (int type = 0; type < 4; ++type) {
    for (int i = 0; i < 8; ++i) { out[i] = 0.2f * i - 0.6f; aux[i] = -out[i]; }
    ringdist::State s2{{0.0f, 0.0f}};
    ringdist::Params q{};
    q.type = (uint8_t)type;
    q.drive = 0.7f;
    q.bias = 0.3f;
    q.tone = 0.5f;
    q.mix = 1.0f;
    ringdist::process(&s2, q, out, aux, 8);
    bool finite = true, bounded = true;
    for (int i = 0; i < 8; ++i) {
      if (!std::isfinite(out[i]) || !std::isfinite(aux[i])) finite = false;
      if (std::fabs(out[i]) > 4.0f || std::fabs(aux[i]) > 4.0f) bounded = false;
    }
    expect_true("type finite", finite);
    expect_true("type bounded", bounded);
    expect_true("type lp moved", std::fabs(s2.lp[0]) > 0.0f);
  }
}

static void test_process_mix_and_state() {
  float out[4] = {0.5f, -0.5f, 0.25f, -0.25f};
  float aux[4] = {-0.5f, 0.5f, -0.25f, 0.25f};
  float keep[4];
  std::memcpy(keep, out, sizeof(out));
  ringdist::State st{{0.0f, 0.0f}};
  ringdist::Params p{};
  p.type = ringdist::kTube;
  p.drive = 0.5f;
  p.tone = 1.0f;
  p.mix = 0.0f;  // fully dry
  ringdist::process(&st, p, out, aux, 4);
  for (int i = 0; i < 4; ++i) expect_near("dry mix", out[i], keep[i], 1e-6f);

  // lp state persists across calls
  float o2[2] = {1.0f, 1.0f}, a2[2] = {1.0f, 1.0f};
  p.mix = 1.0f;
  ringdist::process(&st, p, o2, a2, 2);
  const float lp0 = st.lp[0];
  ringdist::process(&st, p, o2, a2, 2);
  expect_true("lp persists", std::fabs(st.lp[0] - lp0) >= 0.0f);
}

int main() {
  test_chord_from_pool();
  test_shaper_curves();
  test_process_bypass_and_types();
  test_process_mix_and_state();
  if (g_fail == 0) std::printf("test_rings_chord: ALL PASS\n");
  else std::printf("test_rings_chord: %d FAILURES\n", g_fail);
  return g_fail ? 1 : 0;
}
