// sc_defs.h — shared constants + math helpers for the dronecaster engine.
// Rate-parameterized: NO baked 44.1 kHz constants (the sample-rate law).
#ifndef RACK_SRC_DRONECASTER_SC_DEFS_H_
#define RACK_SRC_DRONECASTER_SC_DEFS_H_
#include <cmath>
#include <cstdint>

namespace sc {

constexpr uint32_t kBlock = 64;               // scsynth block size, 48 kHz
// The golden wrapper seeds the server RGen with RandSeed.kr(1, 16rC0FFEE);
// the engine calls RGen::init(kGoldenSeed) with the same value.
constexpr uint32_t kGoldenSeed = 0xC0FFEEu;

inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float midicps(float n) { return 440.0f * powf(2.0f, (n - 69.0f) / 12.0f); }
inline float cpsmidi(float f) { return 12.0f * log2f(f / 440.0f) + 69.0f; }

}  // namespace sc
#endif
