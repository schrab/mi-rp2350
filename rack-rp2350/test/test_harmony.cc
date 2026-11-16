// T8b — lira8 ARP-driven voice pitch: voicing math host tests.
// Pure-header target: src/lira8/lira_harmony.h must stay MCU-header free
// (part of the LIRA-8 port's ctag-tbd portability proof).
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../src/lira8/lira_harmony.h"

static int g_fail = 0;

static void expect_offsets(const char* what, const l8harmony::GroupCtx& g,
                           int group, const int8_t want[4]) {
  int8_t got[4];
  l8harmony::group_offsets(g, group, got);
  for (int i = 0; i < 4; ++i) {
    if (got[i] != want[i]) {
      std::printf("FAIL %s: offset[%d] = %d, want %d\n", what, i, got[i],
                  want[i]);
      ++g_fail;
    }
  }
}

static void expect_int(const char* what, int got, int want) {
  if (got != want) {
    std::printf("FAIL %s: %d != %d\n", what, got, want);
    ++g_fail;
  }
}

static void expect_hz(const char* what, float got, float want) {
  if (std::fabs(got - want) > 0.01f) {
    std::printf("FAIL %s: %.4f != %.4f\n", what, got, want);
    ++g_fail;
  }
}

static l8harmony::GroupCtx make_ctx(uint8_t root, const uint8_t* scale,
                                    uint8_t nsteps, int8_t deg) {
  l8harmony::GroupCtx g{};
  g.root = root;
  g.nsteps = nsteps;
  g.chord_deg = deg;
  g.chord_nt = (nsteps >= 6) ? 4 : 3;
  for (uint8_t i = 0; i < nsteps; ++i) g.scale_st[i] = scale[i];
  return g;
}

int main() {
  const uint8_t kMajor[7] = {0, 2, 4, 5, 7, 9, 11};
  const uint8_t kAeolian[7] = {0, 2, 3, 5, 7, 8, 10};
  const uint8_t kPentMaj[5] = {0, 2, 4, 7, 9};

  // Low group is always the perfect bass stack (root, fifth over two octaves).
  const int8_t kLow[4] = {-12, -5, 0, 7};
  {
    const auto g = make_ctx(48, kMajor, 7, 0);
    expect_offsets("Cmaj low", g, 0, kLow);
    expect_offsets("Amin-deg low", make_ctx(48, kMajor, 7, 5), 0, kLow);
    expect_offsets("pent low", make_ctx(48, kPentMaj, 5, 0), 0, kLow);
  }

  // Cmaj7 (deg 0): third +4, fifth +7, seventh +11 -> E4 G4 B4 C5 over C3.
  {
    const int8_t kWant[4] = {16, 19, 23, 24};
    expect_offsets("Cmaj7 high", make_ctx(48, kMajor, 7, 0), 1, kWant);
  }

  // Degree 5 of C major = A minor: third +3, fifth +7, seventh +10.
  {
    const int8_t kWant[4] = {15, 19, 22, 24};
    expect_offsets("Amin high", make_ctx(48, kMajor, 7, 5), 1, kWant);
  }

  // NEO minor triad (aeolian scale rebuild): minor third, perfect fifth.
  {
    const int8_t kWant[4] = {15, 19, 22, 24};
    expect_offsets("NEO minor high", make_ctx(48, kAeolian, 7, 0), 1, kWant);
  }

  // Pentatonic (nsteps 5 -> triad): third +4, "fifth" = 4 scale steps +9,
  // triad top = third+24, root+24.
  {
    const int8_t kWant[4] = {16, 21, 28, 24};
    expect_offsets("pent high", make_ctx(48, kPentMaj, 5, 0), 1, kWant);
  }

  // Root octave folding (harmony mode keeps C3..B3; mapped roots are exact).
  expect_int("fold C5", l8harmony::fold_root(72), 48);
  expect_int("fold B4", l8harmony::fold_root(71), 59);
  expect_int("fold C3", l8harmony::fold_root(48), 48);

  // Full Cmaj7 voicing on C3: voices 1-8 = C2 G2 C3 G3 E4 G4 B4 C5.
  {
    const auto g = make_ctx(48, kMajor, 7, 0);
    int8_t off[4];
    const int expect_low[4] = {36, 43, 48, 55};
    l8harmony::group_offsets(g, 0, off);
    for (int i = 0; i < 4; ++i) {
      expect_int("Cmaj7 voicing low", 48 + off[i], expect_low[i]);
    }
    const int expect_high[4] = {64, 67, 71, 72};
    l8harmony::group_offsets(g, 1, off);
    for (int i = 0; i < 4; ++i) {
      expect_int("Cmaj7 voicing high", 48 + off[i], expect_high[i]);
    }
  }

  // Hz law (the engine's mtof).
  expect_hz("A4", l8harmony::note_hz(69), 440.0f);
  expect_hz("A2", l8harmony::note_hz(45), 110.0f);
  expect_hz("C3", l8harmony::note_hz(48), 130.8128f);

  if (g_fail == 0) {
    std::printf("test_harmony: all tests passed\n");
    return 0;
  }
  std::printf("test_harmony: %d FAILURES\n", g_fail);
  return 1;
}
