// VARG pure-engine host tests (pools, note calc, lira8 ctx; kernels + riff
// arrive in later tasks). Pure-header target: src/varg.h must stay
// MCU-header free (ctag-tbd portability discipline).
#include <cstdio>
#include <cstring>

#include "../src/varg.h"
#include "../src/lira8/lira_harmony.h"

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

using namespace varg;

static Inputs make_inputs() {
  Inputs in{};
  in.riff = 16; in.prog = 6; in.voice = 17; in.diss = 0;
  in.range = 2; in.root_mode = 0; in.neo_op = 0;
  in.scale_id = 6; in.key_root_pc = 9;  // A minor (Aeolian)
  return in;
}

static void expect_pool(const char* what, const State& st,
                        const int8_t* want, uint8_t n) {
  if (st.pool_n != n) {
    std::printf("FAIL %s: pool_n %u != %u\n", what, st.pool_n, n);
    ++g_fail;
    return;
  }
  for (uint8_t i = 0; i < n; ++i) {
    if (st.pool[i] != want[i]) {
      std::printf("FAIL %s: pool[%u]=%d want %d\n", what, i, st.pool[i],
                  want[i]);
      ++g_fail;
    }
  }
}

static void test_pools() {
  State st{};
  Inputs in = make_inputs();

  in.voice = 0;    // POWER (0*6/101 = 0)
  st.root_pc = 9; st.quality = kQMinor;
  build_pool(&st, in);
  { static const int8_t w[] = { 0, 7, 12 }; expect_pool("power", st, w, 3); }

  in.voice = 17;   // TRIAD (17*6/101 = 1)
  st.quality = kQMinor; build_pool(&st, in);
  { static const int8_t w[] = { 0, 3, 7, 12 }; expect_pool("triad-min", st, w, 4); }
  st.quality = kQMajor; build_pool(&st, in);
  { static const int8_t w[] = { 0, 4, 7, 12 }; expect_pool("triad-maj", st, w, 4); }
  st.quality = kQDim; build_pool(&st, in);
  { static const int8_t w[] = { 0, 3, 6, 12 }; expect_pool("triad-dim", st, w, 4); }
  st.quality = kQAug; build_pool(&st, in);
  { static const int8_t w[] = { 0, 4, 8, 12 }; expect_pool("triad-aug", st, w, 4); }

  in.voice = 34;   // TRITONE (34*6/101 = 2)
  st.quality = kQMinor; build_pool(&st, in);
  { static const int8_t w[] = { 0, 6, 7, 12 }; expect_pool("tritone", st, w, 4); }

  in.voice = 51;   // DIMAUG (51*6/101 = 3)
  st.quality = kQDim; build_pool(&st, in);
  { static const int8_t w[] = { 0, 3, 6, 9 }; expect_pool("dim7", st, w, 4); }
  st.quality = kQMajor; build_pool(&st, in);
  { static const int8_t w[] = { 0, 4, 8, 12 }; expect_pool("aug", st, w, 4); }
  st.quality = kQMinor; build_pool(&st, in);
  { static const int8_t w[] = { 0, 3, 6, 12 }; expect_pool("dim-flavored-min", st, w, 4); }

  in.voice = 68;   // ADD9 (68*6/101 = 4)
  st.quality = kQMinor; build_pool(&st, in);
  { static const int8_t w[] = { 0, 3, 7, 14 }; expect_pool("add9", st, w, 4); }

  in.voice = 85;   // PEDAL (85*6/101 = 5)
  st.root_pc = 4; build_pool(&st, in);   // E chord in A key -> F pedal
  { static const int8_t w[] = { 0, 3, 5, 7, 12 }; expect_pool("pedal", st, w, 5); }
  st.root_pc = 9; build_pool(&st, in);   // tonic chord -> octave pedal (dedupe)
  { static const int8_t w[] = { 0, 3, 7, 12 }; expect_pool("pedal-tonic", st, w, 4); }

  in.voice = 17; st.root_pc = 9; st.quality = kQMinor;
  in.diss = 30; build_pool(&st, in);
  { static const int8_t w[] = { 0, 1, 3, 7, 12 }; expect_pool("diss30-b2", st, w, 5); }
  in.diss = 65; build_pool(&st, in);
  { static const int8_t w[] = { 0, 1, 3, 6, 7, 12 }; expect_pool("diss65-tt", st, w, 6); }
}

static void test_calc_note() {
  State st{};
  st.pool_n = 4;
  const int8_t p[] = { 0, 3, 7, 12 };
  for (uint8_t i = 0; i < 4; ++i) st.pool[i] = p[i];
  st.root_pc = 9;
  Step s{};
  s.off = 0; s.oct = 0;
  expect_int("calc root", calc_note(st, s), 45);         // A2
  s.off = 2; expect_int("calc fifth", calc_note(st, s), 52);
  s.off = 3; expect_int("calc octave", calc_note(st, s), 57);
  s.off = 4; expect_int("calc oct+1 wrap", calc_note(st, s), 57);
  s.off = -1; expect_int("calc neg floor wrap", calc_note(st, s), 45);
  // fold >96 down an octave, pitch class preserved (36+11+14+36 = 97 -> 85)
  st.root_pc = 11; st.pool[3] = 14;
  s.off = 3; s.oct = 3;
  expect_int("calc fold", calc_note(st, s), 85);
}

static void test_lira_ctx() {
  // The engine fills l8harmony::GroupCtx from a VARG pool exactly the way
  // the publish branch does; the voicing law must yield chord tones.
  State st{};
  Inputs in = make_inputs();
  in.voice = 17; in.diss = 0;
  st.root_pc = 9; st.quality = kQMinor;
  build_pool(&st, in);
  l8harmony::GroupCtx g{};
  g.root = l8harmony::fold_root((uint8_t)(36u + st.root_pc));
  g.nsteps = st.pool_n;
  for (uint8_t i = 0; i < 12; ++i) g.scale_st[i] = 0;
  for (uint8_t i = 0; i < st.pool_n; ++i) g.scale_st[i] = (uint8_t)st.pool[i];
  g.chord_deg = 0;
  g.chord_nt = (st.pool_n >= 4) ? 4u : 3u;
  int8_t lo[4], hi[4];
  l8harmony::group_offsets(g, 0, lo);
  l8harmony::group_offsets(g, 1, hi);
  expect_int("lira lo0", lo[0], -12);   // perfect bass stack law unchanged
  expect_int("lira lo3", lo[3], 7);
  for (int i = 0; i < 4; ++i) {         // high group = minor chord tones
    const int pc = ((hi[i] % 12) + 12) % 12;
    expect_true("lira hi chord tone", pc == 0 || pc == 3 || pc == 7);
  }
}

int main() {
  test_pools();
  test_calc_note();
  test_lira_ctx();
  if (g_fail == 0) std::printf("test_varg: ALL PASS\n");
  else std::printf("test_varg: %d FAILURES\n", g_fail);
  return g_fail ? 1 : 0;
}
