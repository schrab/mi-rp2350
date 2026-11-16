// T9 TDD unit test for the rack reverb engine size (Task 2).
//
// Law: the rack build (RACK_RP2350_TC) instantiates FxEngine<21632> — the
// Dattorro Reserve chain in reverb.h sums to exactly 21,617 frames
// (150+214+319+527+2182+2690+4501+2525+2197+6312) — so Part's reverb carve
// drops 65,536 -> 43,264 B and the freed pool room hosts the warps state.
// FxEngine::Init zero-fills EXACTLY the declared template size, so the
// caller's buffer must match: this test hands the engine a 21,632-frame
// buffer and guards frames [21632, 21632+64) with canaries. The upstream
// 32,768 instantiation (standalone) smashes the canaries -> RED; the rack
// instantiation must leave them intact -> GREEN.
//
// Built with -DRACK_RP2350_TC (see Makefile) — on host the RAM_CODE markers
// no-op (no ARDUINO_ARCH_RP2040).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>

#include "elements/dsp/fx/reverb.h"

using namespace elements;

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

static const size_t kFrames = 21632;   // the rack carve (43,264 B)
static const size_t kCanary = 64;
// Oversized allocation: the UPSTREAM arm (FxEngine<32768>) Init-writes 32,768
// frames — the block must hold that without faulting, while the canary zone
// at [kFrames, kFrames+kCanary) sits INSIDE the upstream write range (so the
// pristine arm zeroes it -> RED via CHECK, not SIGSEGV).
static const size_t kAlloc = 32768 + kCanary;

static bool canaries_intact(const uint16_t* canary) {
  for (size_t i = 0; i < kCanary; ++i) {
    if (canary[i] != 0xA5A5u) return false;
  }
  return true;
}

static void test_init_fills_exactly_the_carve() {
  static uint16_t mem[kAlloc];
  uint16_t* canary = mem + kFrames;
  memset(mem, 0, sizeof(mem));
  for (size_t i = 0; i < kCanary; ++i) canary[i] = 0xA5A5u;

  Reverb reverb;
  reverb.Init(mem);

  CHECK(canaries_intact(canary),
        "Reverb::Init wrote past frame %zu — the rack instantiation is not "
        "FxEngine<21632> (or the caller carve and the engine disagree)",
        kFrames);
  for (size_t i = 0; i < kFrames; ++i) {
    if (mem[i] != 0) {
      CHECK(false, "working frame %zu not zero-filled by Init", i);
      break;
    }
  }
}

static void test_render_smoke_and_decay() {
  static uint16_t mem[kAlloc];
  Reverb reverb;
  reverb.Init(mem);
  reverb.set_amount(1.0f);
  reverb.set_time(0.8f);
  reverb.set_diffusion(0.625f);
  reverb.set_lp(0.7f);
  reverb.set_input_gain(0.2f);

  float main[16], aux[16];
  float peak_first = 0.0f, peak_last = 0.0f;
  for (int block = 0; block < 400; ++block) {
    for (size_t i = 0; i < 16; ++i) {
      main[i] = (block == 0 && i == 0) ? 0.9f : 0.0f;
      aux[i] = 0.0f;
    }
    reverb.Process(main, aux, 16);
    for (size_t i = 0; i < 16; ++i) {
      CHECK(std::isfinite(main[i]) && std::isfinite(aux[i]),
            "non-finite reverb output at block %d", block);
      if (block < 20) {
        peak_first = peak_first > fabsf(main[i]) ? peak_first : fabsf(main[i]);
      }
      if (block >= 390) {
        peak_last = peak_last > fabsf(main[i]) ? peak_last : fabsf(main[i]);
      }
    }
  }
  CHECK(peak_last < peak_first,
        "impulse response does not decay: first-20-block peak %.6f vs "
        "last-10-block peak %.6f", peak_first, peak_last);
  CHECK(peak_first > 0.01f, "reverb produced no audible wet signal (%.6f)",
        peak_first);
}

int main() {
  test_init_fills_exactly_the_carve();
  test_render_smoke_and_decay();
  if (g_fail) {
    fprintf(stderr, "test_reverb: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_reverb: all checks passed\n");
  return 0;
}
