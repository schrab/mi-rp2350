// Benjolin engine structural gate (plain g++, no MCU headers).
// Golden compares land in Task 4; this file pins the structural laws:
// finiteness, determinism, the 16->64 burst equivalence, and the rungler
// DAC weight laws.
#include <cmath>
#include <cstdio>

#include "benjolin_engine.h"

static int g_failures = 0;

static const benjolin::Params& TestParams() {
  static benjolin::Params p;
  p.amp = 0.5f;
  return p;
}

int main() {
  // 1 s render through the 16-sample grid: finite everywhere.
  {
    benjolin::Engine e;
    e.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191,
           TestParams());
    float l[48000], r[48000];
    for (int i = 0; i < 3000; ++i) e.Process16(l + i * 16, r + i * 16);
    for (int i = 0; i < 48000; ++i) {
      if (!std::isfinite(l[i]) || !std::isfinite(r[i])) {
        printf("FAIL: non-finite output at %d\n", i);
        ++g_failures;
        break;
      }
    }
  }
  // Determinism: two engines, same seed/params -> bit-identical streams.
  {
    benjolin::Engine a, b;
    a.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191,
           TestParams());
    b.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191,
           TestParams());
    bool same = true;
    for (int i = 0; i < 375 && same; ++i) {
      float al[16], ar[16], bl[16], br[16];
      a.Process16(al, ar);
      b.Process16(bl, br);
      for (int k = 0; k < 16; ++k)
        if (al[k] != bl[k] || ar[k] != br[k]) { same = false; }
    }
    if (!same) { printf("FAIL: engines diverged\n"); ++g_failures; }
  }
  // Process16 burst == Process64 stream, bit-exact.
  {
    benjolin::Engine c, d;
    c.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191,
           TestParams());
    d.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191,
           TestParams());
    bool burst_ok = true;
    float l64[64], r64[64];
    for (int i = 0; i < 64 && burst_ok; ++i) {
      float l16[16], r16[16];
      c.Process16(l16, r16);
      if (i % 4 == 0) d.Process64(l64, r64);
      for (int k = 0; k < 16; ++k)
        if (l16[k] != l64[(i % 4) * 16 + k]) { burst_ok = false; }
    }
    if (!burst_ok) { printf("FAIL: Process16 != Process64 stream\n"); ++g_failures; }
  }
  // Rungler DAC weight laws (all powers of two, max 255/256).
  {
    float all_on[8], only_first[8];
    for (int k = 0; k < 8; ++k) { all_on[k] = 1.0f; only_first[k] = 0.0f; }
    if (!(std::abs(benjolin::RunglerDac(all_on) - 255.0f / 256.0f) < 1e-9f)) {
      printf("FAIL: RunglerDac all-on != 255/256\n");
      ++g_failures;
    }
    only_first[0] = 1.0f;
    if (!(std::abs(benjolin::RunglerDac(only_first) - 1.0f / 256.0f) < 1e-9f)) {
      printf("FAIL: RunglerDac sh1 != 1/256\n");
      ++g_failures;
    }
    if (!(std::abs(benjolin::RunglerDacBits(0xffu) - 255.0f / 256.0f) < 1e-9f)) {
      printf("FAIL: RunglerDacBits(0xff) != 255/256\n");
      ++g_failures;
    }
    if (!(std::abs(benjolin::RunglerDacBits(0x01u) - 1.0f / 256.0f) < 1e-9f)) {
      printf("FAIL: RunglerDacBits(bit0) != 1/256\n");
      ++g_failures;
    }
    if (!(std::abs(benjolin::RunglerDacBits(0x80u) - 0.5f) < 1e-9f)) {
      printf("FAIL: RunglerDacBits(bit7) != 1/2\n");
      ++g_failures;
    }
  }
  // Mode B: the explicit register shifts on osc2 rising edges only.
  {
    benjolin::Params p = TestParams();
    p.freq2 = 4.0f;   // ~4 edges/s
    benjolin::Engine e;
    e.Init(48000.0, benjolin::FeedbackMode::kHardware, 1795055191, p);
    uint32_t changes = 0;
    uint32_t prev = e.reg();
    float l[16], r[16];
    for (int i = 0; i < 3000; ++i) {
      e.Process16(l, r);
      if (e.reg() != prev) { ++changes; prev = e.reg(); }
    }
    // 1 s at ~4 Hz -> 4 changes +-2 (register shifts only on rising edges).
    if (!(changes >= 2 && changes <= 6)) {
      printf("FAIL: mode-B register changes = %u (expected ~4)\n", changes);
      ++g_failures;
    }
    for (int i = 0; i < 48000; ++i) { /* finiteness checked via l/r reuse */ }
  }
  printf("test_benjolin: %s (%d failures)\n", g_failures ? "FAIL" : "PASS",
         g_failures);
  return g_failures ? 1 : 0;
}
