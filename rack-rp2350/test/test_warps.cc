// T9 TDD unit tests for the vendored warps Modulator (rack deltas).
//
// Law (docs/superpowers/plans/2026-11-17-elements-warps.md): the rack copy
//   - strips the vocoder + easter egg (memory law: sizeof(Modulator) must fit
//     the pool carve; the vocoder's two FilterBanks alone are ~62 KB),
//   - shrinks kMaxBlockSize to the Part block (16),
//   - renames lut_xfade_in/out (clouds keeps the originals),
//   - stays host-compilable with plain g++ (the RAM_CODE marker no-ops
//     off-Arduino — that property is the portability proof, as for lira8).
//
// The easter egg's SRC divisibility trap (FilterBank silently drops blocks
// whose size is not a multiple of the band ratios) is why small blocks were
// unsafe with the vocoder — documented in the plan, irrelevant once stripped.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <new>
#include <type_traits>

#include "warps/dsp/modulator.h"

// stmlib's class-static LUT tables (units/random/atan) are defined in .inc
// files compiled by the firmware's single-TU wrapper — mirror it here.
#include "stmlib/dsp/atan.inc"
#include "stmlib/dsp/units.inc"
#include "stmlib/utils/random.inc"

using namespace warps;

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

// SFINAE detector: the easter-egg surface must be gone from the rack copy.
template <typename T>
static auto probe_set_easter_egg(int)
    -> decltype(std::declval<T&>().set_easter_egg(false), std::true_type{}) {
  return {};
}
template <typename T>
static auto probe_set_easter_egg(long) -> std::false_type {
  return {};
}

// Adapter-parity construction: placement-new value-init over max-aligned
// raw storage (zeroes level_ etc. — SaturatingAmplifier::Init only resets
// drive_), then Init(32000). A fresh object per call: the Modulator is all
// POD (trivial dtor), so storage reuse between tests is safe.
static Modulator* make_modulator() {
  static uint8_t* storage =
      static_cast<uint8_t*>(::operator new(sizeof(Modulator)));
  Modulator* m = new (storage) Modulator();
  m->Init(32000.0f);
  return m;
}

static void set_ext_carrier(Parameters* p, float algorithm, float timbre) {
  p->carrier_shape = 0;              // external carrier, always (rack law)
  p->channel_drive[0] = 1.0f;
  p->channel_drive[1] = 1.0f;
  p->modulation_algorithm = algorithm;
  p->modulation_parameter = timbre;
}

static void test_rack_deltas() {
  CHECK(warps::kMaxBlockSize == 16,
        "kMaxBlockSize must equal the Part block (16), got %zu",
        warps::kMaxBlockSize);
  CHECK(sizeof(Modulator) <= 4096,
        "sizeof(Modulator) = %zu B — exceeds the 4 KiB pool-carve budget; "
        "the vocoder/easter-egg strip is missing or something grew back",
        sizeof(Modulator));
  CHECK(!decltype(probe_set_easter_egg<Modulator>(0))::value,
        "easter-egg surface (set_easter_egg) must be stripped");
}

static void test_bypass_copies_input() {
  Modulator* m = make_modulator();
  m->set_bypass(true);
  ShortFrame in[16], out[16];
  for (size_t i = 0; i < 16; ++i) {
    in[i].l = static_cast<short>(i * 1000 - 8000);
    in[i].r = static_cast<short>(-i * 500 + 3000);
  }
  m->Process(in, out, 16);
  for (size_t i = 0; i < 16; ++i) {
    CHECK(out[i].l == in[i].l && out[i].r == in[i].r,
          "bypass frame %zu differs: (%d,%d) -> (%d,%d)", i, in[i].l, in[i].r,
          out[i].l, out[i].r);
  }
}

static void test_silent_at_zero_drive() {
  Modulator* m = make_modulator();
  Parameters* p = m->mutable_parameters();
  set_ext_carrier(p, 0.375f, 0.0f);   // position 3 = pure digital ring mod
  p->channel_drive[0] = 0.0f;
  p->channel_drive[1] = 0.0f;
  ShortFrame in[16], out[16];
  for (size_t i = 0; i < 16; ++i) { in[i].l = 8000; in[i].r = 4000; }
  m->Process(in, out, 16);
  m->Process(in, out, 16);            // let the level followers settle
  for (size_t i = 0; i < 16; ++i) {
    CHECK(out[i].l == 0 && out[i].r == 0,
          "zero drive must be silent at frame %zu: (%d,%d)", i, out[i].l,
          out[i].r);
  }
}

static void test_digital_ringmod_zero_products() {
  // ALGORITHM_DIGITAL_RING_MODULATION (position 3, alg = 3/8 = 0.375):
  // ring = 4*x1*x2 / (1 + |4*x1*x2|) -> exactly 0 whenever either input is 0,
  // regardless of the other channel and of the amplifier's drive curve.
  Modulator* m = make_modulator();
  Parameters* p = m->mutable_parameters();
  set_ext_carrier(p, 0.375f, 0.0f);
  ShortFrame in[16], out[16];
  for (size_t i = 0; i < 16; ++i) { in[i].l = 10000; in[i].r = 0; }
  m->Process(in, out, 16);
  m->Process(in, out, 16);
  for (size_t i = 0; i < 16; ++i) {
    CHECK(out[i].l == 0, "silent carrier must ring to 0 at frame %zu: %d", i,
          out[i].l);
  }
  for (size_t i = 0; i < 16; ++i) { in[i].l = 0; in[i].r = 10000; }
  m->Process(in, out, 16);
  m->Process(in, out, 16);
  for (size_t i = 0; i < 16; ++i) {
    CHECK(out[i].l == 0, "silent modulator must ring to 0 at frame %zu: %d", i,
          out[i].l);
  }
}

int main() {
  test_rack_deltas();
  test_bypass_copies_input();
  test_silent_at_zero_drive();
  test_digital_ringmod_zero_products();
  if (g_fail) {
    fprintf(stderr, "test_warps: %d check(s) FAILED\n", g_fail);
    return 1;
  }
  printf("test_warps: all checks passed\n");
  return 0;
}
