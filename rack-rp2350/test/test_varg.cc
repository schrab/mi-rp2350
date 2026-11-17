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

// ---- Task 2: harmony kernels -------------------------------------------

static bool state_eq(const State& a, const State& b) {
  return a.prog_idx == b.prog_idx && a.chord_idx == b.chord_idx &&
         a.prev_deg == b.prev_deg && a.root_pc == b.root_pc &&
         a.quality == b.quality && a.is_minor == b.is_minor &&
         a.anchor_pc == b.anchor_pc && a.bars == b.bars &&
         a.key_shift == b.key_shift;
}

static Inputs inputs_for(uint8_t prog_idx) {
  Inputs in = make_inputs();
  in.prog = (uint8_t)(prog_idx * 13u + 6u);  // prog_of() lands exactly here
  return in;
}

static void test_advance_determinism() {
  for (uint8_t prog = 0; prog < kProgCount; ++prog) {
    Inputs in = inputs_for(prog);
    in.diss = 40;
    State a{}, b{};
    init(&a, 1234u, in);
    init(&b, 1234u, in);
    for (uint8_t bar = 0; bar < 16u; ++bar) advance(&a, in);
    for (uint8_t bar = 0; bar < 16u; ++bar) advance(&b, in);
    expect_true("advance deterministic", state_eq(a, b));
  }
}

// Rel of root_pc to the CURRENT tonic (section shifts move it mid-run).
static uint8_t rel_to_tonic(const State& st) {
  const uint8_t tonic = (uint8_t)((9u + st.key_shift) % 12u);
  return (uint8_t)((st.root_pc + 12u - tonic) % 12u);
}

static void test_aeolian_ostinato() {
  // A Aeolian scale steps {0,2,3,5,7,8,10}: degrees {0,5,6} -> semis {0,8,10}
  // (i, bVI, bVII); degrees {0,2,5,6} -> semis {0,3,8,10} (i, bIII, bVI, bVII).
  Inputs in = inputs_for(kProgAeolian);
  State st{};
  init(&st, 77u, in);
  bool ok = true;
  for (uint8_t bar = 0; bar < 16u; ++bar) {
    advance(&st, in);
    const uint8_t rel = rel_to_tonic(st);
    if (!(rel == 0 || rel == 8 || rel == 10) || st.quality != kQMinor) ok = false;
    if (!(st.prev_deg == 0 || st.prev_deg == 5 || st.prev_deg == 6)) ok = false;
  }
  expect_true("aeolian degrees+quality", ok);

  Inputs in2 = inputs_for(kProgOstinato);
  init(&st, 78u, in2);
  ok = true;
  for (uint8_t bar = 0; bar < 16u; ++bar) {
    advance(&st, in2);
    const uint8_t rel = rel_to_tonic(st);
    if (!(rel == 0 || rel == 3 || rel == 8 || rel == 10) ||
        st.quality != kQMinor) ok = false;
  }
  expect_true("ostinato degrees+quality", ok);
}

static void test_weitzmann_region() {
  Inputs in = inputs_for(kProgWeitzmann);
  in.diss = 100;   // mutations stay INSIDE the region by construction
  State st{};
  init(&st, 99u, in);
  const uint8_t anchor = 8u;  // tonic A(9) - 1
  bool ok = true;
  for (uint8_t bar = 0; bar < 32u; ++bar) {
    advance(&st, in);
    const uint8_t anch = (uint8_t)((anchor + st.key_shift) % 12u);
    const uint8_t rel = (uint8_t)((st.root_pc + 12u - anch) % 12u);
    const bool minor_ok = st.is_minor && (rel == 1 || rel == 5 || rel == 9);
    const bool major_ok = !st.is_minor && (rel == 0 || rel == 4 || rel == 8);
    if (!minor_ok && !major_ok) ok = false;
  }
  expect_true("weitzmann region membership", ok);
}

static void test_mediant() {
  Inputs in = inputs_for(kProgMediant);
  State st{};
  init(&st, 5u, in);
  bool ok = true;
  uint8_t prev = st.root_pc;
  for (uint8_t bar = 0; bar < 16u; ++bar) {
    advance(&st, in);
    const uint8_t d = (uint8_t)((st.root_pc + 12u - prev) % 12u);
    if (!(d == 3 || d == 8) || st.quality != kQMinor) ok = false;
    prev = st.root_pc;
  }
  expect_true("mediant +3/-4 minor", ok);
}

static void test_add9dim_seed() {
  Inputs in = inputs_for(kProgAdd9Dim);
  in.diss = 0;
  State st{};
  init(&st, 1u, in);
  // golden cycle relative to tonic A=9: +6 dim, +11 min, +0 maj, +0 add9.
  // Exactly 4 advances (bars < 8): the section-shift roll can never fire.
  static const uint8_t kRoot[4] = { 3, 8, 9, 9 };
  static const uint8_t kQual[4] = { kQDim, kQMinor, kQMajor, kQAdd9 };
  for (uint8_t bar = 0; bar < 4u; ++bar) {
    advance(&st, in);
    expect_int("add9dim root", st.root_pc, kRoot[bar & 3u]);
    expect_int("add9dim quality", st.quality, kQual[bar & 3u]);
  }
}

static void test_halftrn_b2b5() {
  Inputs in = inputs_for(kProgHalfTrn);
  State st{};
  init(&st, 2u, in);
  bool ok = true;
  uint8_t prev = st.root_pc;
  for (uint8_t bar = 0; bar < 32u; ++bar) {
    advance(&st, in);
    const uint8_t d = (uint8_t)((st.root_pc + 12u - prev) % 12u);
    if (!(d == 1 || d == 6 || d == 11) || st.quality != kQMinor) ok = false;
    prev = st.root_pc;
  }
  expect_true("halftrn +-1/+-6 minor", ok);

  Inputs in2 = inputs_for(kProgB2b5);
  in2.diss = 0;
  init(&st, 3u, in2);
  ok = true;
  for (uint8_t bar = 0; bar < 32u; ++bar) {
    advance(&st, in2);
    const uint8_t rel = rel_to_tonic(st);
    if (!(rel == 0 || rel == 1 || rel == 3 || rel == 5 || rel == 6 ||
          rel == 11) || st.quality != kQPower) ok = false;
  }
  expect_true("b2b5 pool+power", ok);
}

static void test_neofree_and_kernels() {
  Inputs in = inputs_for(kProgNeoFree);
  in.neo_op = 1;  // pinned P: root NEVER moves, quality flips per transform
  in.diss = 0;
  State st{};
  init(&st, 7u, in);
  const uint8_t root0 = st.root_pc;
  bool flipped = false;
  for (uint8_t bar = 0; bar < 32u; ++bar) {
    advance(&st, in);
    if (st.root_pc != root0) { expect_true("neofree pinned P root", false); break; }
    flipped = flipped || (st.is_minor == 0);
  }
  expect_true("neofree pinned P flips quality", flipped);

  // ROOT=W7 forces the diatonic fallback on NEOFREE (degrees {0,2,5,6} ->
  // Aeolian semitone offsets {0,3,8,10} above the current tonic)
  in.root_mode = 2;
  in.neo_op = 0;
  init(&st, 8u, in);
  bool ok = true;
  for (uint8_t bar = 0; bar < 16u; ++bar) {
    advance(&st, in);
    const uint8_t rel = rel_to_tonic(st);
    if (!(rel == 0 || rel == 3 || rel == 8 || rel == 10) ||
        st.quality != kQMinor) ok = false;
  }
  expect_true("w7 fallback on neofree", ok);

  // ROOT=NEO forces the NEO walk on AEOLIAN
  in.prog = 6;  // AEOLIAN
  in.root_mode = 3;
  init(&st, 9u, in);
  ok = true;
  for (uint8_t bar = 0; bar < 16u; ++bar) {
    advance(&st, in);
    if (st.quality != (st.is_minor ? kQMinor : kQMajor)) ok = false;
  }
  expect_true("neo kernel quality mirror", ok);
}

static void test_diss_substitution_and_sections() {
  Inputs in = inputs_for(kProgAeolian);
  in.diss = 100;
  State st{};
  init(&st, 11u, in);
  bool left_diatonic = false;
  for (uint8_t bar = 0; bar < 64u; ++bar) {
    advance(&st, in);
    // diatonic set is {0,8,10} above the CURRENT tonic (shift-aware)
    const uint8_t rel = rel_to_tonic(st);
    if (!(rel == 0 || rel == 8 || rel == 10)) left_diatonic = true;
  }
  expect_true("diss substitution leaves diatonic", left_diatonic);

  Inputs in2 = inputs_for(kProgWeitzmann);
  in2.diss = 100;
  init(&st, 12u, in2);
  bool shifted = false;
  for (uint16_t bar = 0; bar < 400u; ++bar) {
    advance(&st, in2);
    if (st.key_shift != 0u) shifted = true;
    if (st.key_shift > 11u) { expect_true("key_shift bound", false); break; }
  }
  expect_true("section shift occurs", shifted);
}

int main() {
  test_pools();
  test_calc_note();
  test_lira_ctx();
  test_advance_determinism();
  test_aeolian_ostinato();
  test_weitzmann_region();
  test_mediant();
  test_add9dim_seed();
  test_halftrn_b2b5();
  test_neofree_and_kernels();
  test_diss_substitution_and_sections();
  if (g_fail == 0) std::printf("test_varg: ALL PASS\n");
  else std::printf("test_varg: %d FAILURES\n", g_fail);
  return g_fail ? 1 : 0;
}
