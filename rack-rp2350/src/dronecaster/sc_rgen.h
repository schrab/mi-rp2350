// sc_rgen.h — exact port of scsynth's RGen (include/plugin_interface/SC_RGen.h,
// GPL-3.0, supercollider/supercollider). SC 3.13.x/3.14.x use L'Ecuyer's
// taus88 three-component Tausworthe generator — NOT an LCG. Seed-matched so
// the C++ engine reproduces the server's random sequences sample-for-sample.
// Verified against the Version-3.14.1 tag (brace-only diff vs 3.13.1).
#ifndef RACK_SRC_DRONECASTER_SC_RGEN_H_
#define RACK_SRC_DRONECASTER_SC_RGEN_H_
#include <cmath>
#include <cstdint>

namespace sc {

// Hash(int32) — Thomas Wang's integer hash (include/plugin_interface/Hash.h)
inline int32_t Hash(int32_t inKey) {
  uint32_t hash = static_cast<uint32_t>(inKey);
  hash += ~(hash << 15);
  hash ^= hash >> 10;
  hash += hash << 3;
  hash ^= hash >> 6;
  hash += ~(hash << 11);
  hash ^= hash >> 16;
  return static_cast<int32_t>(hash);
}

class RGen {
 public:
  // SC_RGen.h RGen::init — the seed is hashed, then the three states are
  // XORed with fixed constants and guarded away from zero.
  void init(uint32_t seed) {
    seed = static_cast<uint32_t>(Hash(static_cast<int32_t>(seed)));
    s1 = 1243598713u ^ seed;
    if (s1 < 2u) s1 = 1243598713u;
    s2 = 3093459404u ^ seed;
    if (s2 < 8u) s2 = 3093459404u;
    s3 = 1821928721u ^ seed;
    if (s3 < 16u) s3 = 1821928721u;
  }

  // SC_RGen.h ::trand
  uint32_t trand() {
    s1 = ((s1 & static_cast<uint32_t>(-2)) << 12) ^ (((s1 << 13) ^ s1) >> 19);
    s2 = ((s2 & static_cast<uint32_t>(-8)) << 4) ^ (((s2 << 2) ^ s2) >> 25);
    s3 = ((s3 & static_cast<uint32_t>(-16)) << 17) ^ (((s3 << 3) ^ s3) >> 11);
    return s1 ^ s2 ^ s3;
  }

  static float bits_to_float(uint32_t i) {
    union { uint32_t u; float f; } v;
    v.u = i;
    return v.f;
  }

  // Ranges: frand = [0,1), frand0 = [1,2), frand2 = [-1,1),
  // frand8 = [-0.125,0.125), fcoin = {-1,+1}.
  float frand()  { return bits_to_float(0x3F800000u | (trand() >> 9)) - 1.0f; }
  float frand0() { return bits_to_float(0x3F800000u | (trand() >> 9)); }
  float frand2() { return bits_to_float(0x40000000u | (trand() >> 9)) - 3.0f; }
  float frand8() { return bits_to_float(0x3E800000u | (trand() >> 9)) - 0.375f; }
  float fcoin()  { return bits_to_float(0x3F800000u | (0x80000000u & trand())); }

  // SC_RGen.h drand(): a double in [0,1) from one trand (the hi word is
  // 0x41300000 = 2^20..2^21). Little-endian word order as in the source
  // (the RP2350 M33 and every host we build on are LE).
  double drand() {
    union { uint32_t w[2]; double d; } du;
    du.w[0] = trand();          // lo word first (little-endian, as the source)
    du.w[1] = 0x41300000u;      // hi word
    return du.d - 1048576.0;
  }
  int32_t irand(int32_t scale) { return static_cast<int32_t>(floor(scale * drand())); }
  double drand2(double scale) { return 2.0 * scale * drand() - scale; }

  uint32_t s1 = 0, s2 = 0, s3 = 0;
};

}  // namespace sc
#endif
