// VARG — black-metal generative harmony + riff engine (6th MOD-ARP algo).
//
// Pure C++ (gen_tables.h only — NO MCU headers) so the host harness tests it
// directly; same portability discipline as src/lira8/lira_harmony.h. Design
// spec: docs/superpowers/specs/2026-11-17-varg-black-metal-generative-
// sequencer-design.md
//
// Coordinate space: every bar, advance() picks a chord (root pitch class +
// quality) through a W7 or Neo-Riemannian kernel, and build_bar() derives
// the chord-tone POOL (absolute semitone offsets from the chord root) plus
// a 16-step riff whose steps INDEX the pool. calc_note() renders
// 36 + root_pc + pool[off] + 12*oct and folds into the 28..96 window.
#ifndef RACK_SRC_VARG_H_
#define RACK_SRC_VARG_H_

#include <stdint.h>
#include "gen_tables.h"

namespace varg {

// Chord quality ids (State::quality).
enum Quality : uint8_t {
  kQMinor = 0, kQMajor, kQPower, kQDim, kQAug, kQAdd9, kQCount
};

// PROG archetypes (p2 -> (p2 * 8) / 101).
enum Prog : uint8_t {
  kProgAeolian = 0, kProgOstinato, kProgWeitzmann, kProgMediant,
  kProgAdd9Dim, kProgHalfTrn, kProgB2b5, kProgNeoFree, kProgCount
};

// VOICE voicings (p3 -> (p3 * 6) / 101).
enum Voicing : uint8_t {
  kVoxPower = 0, kVoxTriad, kVoxTritone, kVoxDimAug, kVoxAdd9, kVoxPedal,
  kVoxCount
};

// RIFF zones (p1 -> (p1 * 6) / 101).
enum RiffZone : uint8_t {
  kRiffSustain = 0, kRiffHalf, kRiff8th, kRiff16th, kRiffGallop, kRiffBlast,
  kRiffZoneCount
};

// Step flags — same bit layout as the engine's ModArpStep.
static const uint8_t kFlagGate   = 1u;
static const uint8_t kFlagAccent = 2u;
static const uint8_t kFlagSlide  = 4u;

struct Step {
  uint8_t flags;  // kFlagGate | kFlagAccent | kFlagSlide
  int8_t  off;    // index into State::pool
  int8_t  oct;    // extra octave offset
};

struct State {
  uint8_t  prog_idx;    // 0..7 archetype
  uint8_t  chord_idx;   // position in the archetype's cycle / seed
  uint8_t  prev_deg;    // last W7 (pseudo-)degree
  uint8_t  root_pc;     // current chord root pitch class 0..11
  uint8_t  quality;     // Quality
  uint8_t  is_minor;    // NEO triad quality (0 maj / 1 min)
  uint8_t  anchor_pc;   // Weitzmann augmented anchor
  uint8_t  bars;        // bars since the last section shift
  uint8_t  key_shift;   // section tonal-center shift 0..11 (semitones)
  uint8_t  pool_n;      // 3..6 chord-tone pool size
  int8_t   pool[6];     // ascending semitone offsets from the chord root
  uint8_t  last_key;    // key_root_pc the progression was seeded under
  uint8_t  last_scale;  // scale_id it was seeded under
  uint32_t rng;         // LCG state
};

struct Inputs {
  uint8_t riff;         // p1 0..100 (RIFF zone + density)
  uint8_t prog;         // p2 0..100 (PROG archetype)
  uint8_t voice;        // p3 0..100 (VOICE voicing)
  uint8_t diss;         // p4 0..100 (NEO subs, planing, passing, mutation)
  uint8_t range;        // 1..3 octave spread
  uint8_t root_mode;    // shared ROOT: 0 midi / 1 paral / 2 w7 / 3 neo
  uint8_t neo_op;       // shared TRANS: 0 rand, 1..6 pinned, 7 PLR
  uint8_t scale_id;     // shared SCALE 0..19
  uint8_t key_root_pc;  // tonic pitch class (shared ROOT note % 12)
};

// ---- helpers -----------------------------------------------------------

static inline uint32_t v_next(State* st) {
  st->rng = st->rng * 1664525u + 1013904223u;
  return st->rng;
}
// 0..n-1
static inline uint8_t v_rand(State* st, uint8_t n) {
  return n ? (uint8_t)(((v_next(st) >> 16) * (uint32_t)n) >> 16) : 0u;
}
// 0..99
static inline uint8_t v_roll100(State* st) {
  return (uint8_t)(((v_next(st) >> 16) * 100u) >> 16);
}

static inline uint8_t prog_of(const Inputs& in) {
  const uint8_t p = (uint8_t)(((uint16_t)in.prog * kProgCount) / 101u);
  return p < kProgCount ? p : (uint8_t)(kProgCount - 1u);
}
static inline uint8_t voicing_of(const Inputs& in) {
  const uint8_t v = (uint8_t)(((uint16_t)in.voice * kVoxCount) / 101u);
  return v < kVoxCount ? v : (uint8_t)(kVoxCount - 1u);
}
static inline uint8_t zone_of(const Inputs& in) {
  const uint8_t z = (uint8_t)(((uint16_t)in.riff * kRiffZoneCount) / 101u);
  return z < kRiffZoneCount ? z : (uint8_t)(kRiffZoneCount - 1u);
}

// ---- chord-tone pool ---------------------------------------------------

// Insert `semi` ascending, deduped; the pool caps at 6 entries.
static inline void pool_push(State* st, int8_t semi) {
  for (uint8_t i = 0; i < st->pool_n; ++i) {
    if (st->pool[i] == semi) return;
  }
  if (st->pool_n >= 6u) return;
  uint8_t k = st->pool_n++;
  while (k > 0u && st->pool[k - 1u] > semi) {
    st->pool[k] = st->pool[k - 1u];
    --k;
  }
  st->pool[k] = semi;
}

// Build the pool for the current chord (root_pc + quality) under the VOICE
// voicing. DISS >= 30 adds the b2 as a passing color, DISS >= 65 also the
// tritone (deduped — voicings that already contain them don't grow).
static inline void build_pool(State* st, const Inputs& in) {
  st->pool_n = 0;
  const bool minor_third = (st->quality == kQMinor || st->quality == kQAdd9 ||
                            st->quality == kQDim);
  const int8_t third = minor_third ? 3 : 4;
  switch (voicing_of(in)) {
    case kVoxPower:
      pool_push(st, 0); pool_push(st, 7); pool_push(st, 12);
      break;
    case kVoxTritone:
      pool_push(st, 0); pool_push(st, 6); pool_push(st, 7); pool_push(st, 12);
      break;
    case kVoxDimAug:
      if (st->quality == kQDim) {
        pool_push(st, 0); pool_push(st, 3); pool_push(st, 6); pool_push(st, 9);
      } else if (st->quality == kQAug || st->quality == kQMajor) {
        pool_push(st, 0); pool_push(st, 4); pool_push(st, 8); pool_push(st, 12);
      } else {  // minor/power/add9: dim-flavored minor (b5 color)
        pool_push(st, 0); pool_push(st, 3); pool_push(st, 6); pool_push(st, 12);
      }
      break;
    case kVoxAdd9:
      pool_push(st, 0); pool_push(st, third); pool_push(st, 7);
      pool_push(st, 14);
      break;
    case kVoxPedal: {
      pool_push(st, 0); pool_push(st, third); pool_push(st, 7);
      pool_push(st, 12);
      // tonic pedal note; when the chord IS the tonic, pedal the octave
      const int8_t pedal = (int8_t)((in.key_root_pc + 12u - st->root_pc) % 12u);
      pool_push(st, pedal == 0 ? (int8_t)12 : pedal);
      break;
    }
    case kVoxTriad:
    default:
      pool_push(st, 0); pool_push(st, third);
      if (st->quality == kQDim)      pool_push(st, 6);
      else if (st->quality == kQAug) pool_push(st, 8);
      else                           pool_push(st, 7);
      pool_push(st, 12);
      break;
  }
  if (in.diss >= 30u) pool_push(st, 1);   // b2 passing color
  if (in.diss >= 65u) pool_push(st, 6);   // tritone passing color
}

// Index of `semi` in the pool (0 when absent — callers re-check).
static inline int8_t pool_idx(const State* st, int8_t semi) {
  for (uint8_t i = 0; i < st->pool_n; ++i) {
    if (st->pool[i] == semi) return (int8_t)i;
  }
  return 0;
}
// Like pool_idx but falls back to the TOP pool entry when `semi` is absent —
// used for fifth/octave slots in sparse pools (dim/aug have no P5/P8).
static inline int8_t pool_top_of(const State* st, int8_t semi) {
  const int8_t i = pool_idx(st, semi);
  return st->pool[i] == semi ? i : (int8_t)(st->pool_n - 1u);
}

// ---- note rendering ----------------------------------------------------

// 36 + root_pc + pool[off] + 12*(oct + off/pool_n), folded into 28..96 by
// octave shifts (pitch class preserved).
static inline uint8_t calc_note(const State& st, const Step& s) {
  const int16_t n = st.pool_n > 0 ? (int16_t)st.pool_n : 1;
  const int16_t oct = (int16_t)s.oct + floor_div_i16((int16_t)s.off, n);
  const int16_t semi = st.pool[floor_mod_i16((int16_t)s.off, n)];
  int16_t note = 36 + (int16_t)st.root_pc + semi + 12 * oct;
  while (note > 96) note -= 12;
  while (note < 28) note += 12;
  return (uint8_t)note;
}

// ---- lifecycle ---------------------------------------------------------

// (Re-)seed the progression. Called by the engine when the algorithm is
// selected or the shared key/scale changed.
static inline void init(State* st, uint32_t seed, const Inputs& in) {
  st->prog_idx   = prog_of(in);
  st->chord_idx  = 0;
  st->prev_deg   = 0;
  st->root_pc    = in.key_root_pc;
  st->quality    = kQMinor;
  st->is_minor   = 1;
  st->anchor_pc  = (uint8_t)((in.key_root_pc + 11u) % 12u);  // tonic - 1
  st->bars       = 0;
  st->key_shift  = 0;
  st->rng        = seed ? seed : 0x1badb002u;
  st->last_key   = in.key_root_pc;
  st->last_scale = in.scale_id;
  build_pool(st, in);
}

}  // namespace varg
#endif  // RACK_SRC_VARG_H_
