// Rings VARG chord adapter — pure C++ (no MCU headers).
//
// Converts a VARG chord (root pitch class + ascending chord-tone pool) into a
// set of ABSOLUTE MIDI notes for rings::StringSynthPart's simultaneous-chord
// renderer. The easter-egg engine renders every chord tone at once; this is
// what makes the VARG mode a chord bed rather than an arpeggio.
//
// Design: the generated black-metal doc (docs/Black-Metal-Chords-Scales-
// Harmonies-and-Progressions.md) — full minor triads, power/tritone/dim/aug/
// add9 voicings, Weitzmann/mediant motion. The pool semantics come from
// src/varg.h; the snapshot re-published by mod_engine.h is a l8harmony
// GroupCtx whose scale_st[] IS the VARG pool.
#ifndef RACK_SRC_RINGS_CHORD_H_
#define RACK_SRC_RINGS_CHORD_H_

#include <stdint.h>

#include "varg.h"
#include "lira8/lira_harmony.h"

namespace ringchord {

// Up to kMaxChordSize (8) simultaneous tones. n = number of valid slots.
struct Chord {
  float tone[8];
  uint8_t n;
  uint8_t root_pc;
};

static inline uint8_t clamp_u8(int32_t v, int32_t lo, int32_t hi) {
  return (uint8_t)(v < lo ? lo : (v > hi ? hi : v));
}

// Pool -> absolute MIDI notes. `octave` is a 12-semitone shift; `max_slots`
// is the engine's chord size (12 / polyphony, capped at 8). The pool is
// ascending and deduped, so the first `n` slots always carry root, third,
// fifth, ... — the exact chord tones the research calls for. When the pool
// is larger than the available voices the upper tones are dropped (they are
// color extensions: b2 / b9 / tritone), keeping the triad intact.
static inline Chord from_pool(const int8_t* pool, uint8_t pool_n,
                              uint8_t root_pc, int8_t octave,
                              uint8_t max_slots) {
  Chord c{};
  const uint8_t root = (uint8_t)(root_pc % 12u);
  const uint8_t slots = max_slots ? (max_slots > 8u ? 8u : max_slots) : 1u;
  const uint8_t n = (uint8_t)(pool_n < slots ? pool_n : slots);
  const int32_t base = 36 + (int32_t)root + 12 * (int32_t)octave;
  for (uint8_t i = 0; i < n; ++i) {
    int32_t note = base + (int32_t)pool[i];
    if (note < 0) note = 0;
    if (note > 127) note = 127;
    c.tone[i] = (float)note;
  }
  c.n = n;
  c.root_pc = root;
  return c;
}

static inline Chord from_varg(const varg::State& v, int8_t octave,
                              uint8_t max_slots) {
  return from_pool(v.pool, v.pool_n, v.root_pc, octave, max_slots);
}

// The published snapshot stores root = 48 + root_pc and scale_st[] = pool.
static inline Chord from_ctx(const l8harmony::GroupCtx& g, int8_t octave,
                             uint8_t max_slots) {
  const uint8_t root = (uint8_t)((g.root + 12u - 48u) % 12u);
  // The publisher may add the shared ARP transpose to root (48 + pc +/- 24st).
  // fold_root() keeps only the pitch class, so carry the octave delta here.
  const int16_t shift = (int16_t)((int16_t)g.root - (int16_t)(48u + root));
  int8_t pool[8];
  const uint8_t n = g.nsteps < 8u ? g.nsteps : 8u;
  for (uint8_t i = 0; i < n; ++i) pool[i] = (int8_t)g.scale_st[i];
  Chord c = from_pool(pool, n, root, octave, max_slots);
  for (uint8_t i = 0; i < c.n; ++i) {
    int32_t note = (int32_t)c.tone[i] + shift;
    if (note < 0) note = 0;
    if (note > 127) note = 127;
    c.tone[i] = (float)note;
  }
  return c;
}

}  // namespace ringchord
#endif  // RACK_SRC_RINGS_CHORD_H_
