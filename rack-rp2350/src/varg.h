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

// ---- harmony kernels (advance) -----------------------------------------

enum Kernel : uint8_t { kKernW7 = 0, kKernNeo = 1 };

// Archetype -> default transition kernel (shared ROOT=2/3 overrides it).
static const uint8_t kProgKernel[kProgCount] = {
  kKernW7,   // Aeolian
  kKernW7,   // Ostinato
  kKernNeo,  // Weitzmann
  kKernNeo,  // Mediant
  kKernW7,   // Add9Dim
  kKernNeo,  // HalfTrn
  kKernW7,   // B2B5
  kKernNeo   // NeoFree
};

// Tonic pitch class including the section-level key shift.
static inline uint8_t tonic_pc(const State* st, const Inputs& in) {
  return (uint8_t)((in.key_root_pc + st->key_shift) % 12u);
}

// Scale degree -> chord root pitch class inside the shared scale.
static inline uint8_t degree_root_pc(const State* st, const Inputs& in,
                                     uint8_t deg) {
  uint8_t steps[12];
  const uint8_t n =
      expand_scale_mask(SCALE_BITMASKS[in.scale_id % SCALE_COUNT], steps);
  return (uint8_t)((tonic_pc(st, in) + steps[deg % n]) % 12u);
}

// NEO operator selection. Shared TRANS (neo_op): 0 = weighted RAND
// (N/R/L common, P/S mid, H rare — black-metal bias), 1..6 pinned, 7 = PLR.
static inline uint8_t pick_neo_op(State* st, const Inputs& in) {
  const uint8_t sel = in.neo_op & 7u;
  if (sel == 7u) return v_rand(st, 3);              // PLR: P/L/R
  if (sel != 0u) return (uint8_t)(sel - 1u);        // pinned P/L/R/N/S/H
  static const uint8_t kWeighted[8] = { 3, 2, 1, 3, 0, 4, 2, 1 };
  return kWeighted[v_rand(st, 8)];
}

// Restricted W7 CDF over n candidates. deg[i] = W7 pseudo-degree of
// candidate i; the current degree gets zero weight (n > 1). Returns the
// picked candidate INDEX.
static inline uint8_t w7_pick(State* st, const uint8_t* deg, uint8_t n) {
  uint32_t w[7];
  uint32_t sum = 0;
  const uint8_t prev = (uint8_t)(st->prev_deg % 7u);
  for (uint8_t i = 0; i < n; ++i) {
    const uint8_t d = (uint8_t)(deg[i] % 7u);
    w[i] = (d == prev && n > 1u)
        ? 0u
        : (uint32_t)DIATONIC_W7_WEIGHTS[d] *
          (uint32_t)CHORD_MARKOV_7X7[prev][d];
    sum += w[i];
  }
  if (sum == 0u) return (uint8_t)((st->chord_idx + 1u) % n);
  uint32_t pick = (v_next(st) >> 16) % sum;
  for (uint8_t i = 0; i < n; ++i) {
    if (pick < w[i]) return i;
    pick -= w[i];
  }
  return 0u;
}

// Weitzmann region of the augmented anchor a: minors root on a+1/a+5/a+9,
// majors on a/a+4/a+8 (verified against the research's {Bb,D,F#} anchor:
// Bm/Ebm/Gm + D/Gb/Bb). member 0..2 minor, 3..5 major.
static const uint8_t kWzSemi[6]  = { 1, 5, 9, 0, 4, 8 };
static const uint8_t kWzMinor[6] = { 1, 1, 1, 0, 0, 0 };
// Documented cycle Ebm -> Bm -> Gm -> Bb for anchor {Bb,D,F#}:
// m(a+1) -> m(a+9) -> m(a+5) -> M(a+8).
static const uint8_t kWzCycle[4] = { 0, 2, 1, 5 };

// One bar: advance the progression, then (maybe) shift the section.
static inline void advance(State* st, const Inputs& in) {
  st->bars++;
  // Section shift every 8 bars, probability scales with DISS: tonal center
  // moves (the research's section-level key shifts) and the phrase resets.
  if ((st->bars % 8u) == 0u &&
      v_roll100(st) < (uint8_t)(10u + ((uint16_t)in.diss * 40u) / 100u)) {
    static const int8_t kShifts[7] = { 2, 3, 5, -2, -3, 1, -1 };
    const int16_t s = (int16_t)st->key_shift + kShifts[v_rand(st, 7)];
    st->key_shift = (uint8_t)(((s % 12) + 12) % 12);
  }

  const uint8_t prog = st->prog_idx;
  const uint8_t kernel = (in.root_mode == 2u) ? kKernW7
                       : (in.root_mode == 3u) ? kKernNeo
                       : kProgKernel[prog];

  if (kernel == kKernNeo &&
      (prog == kProgAeolian || prog == kProgOstinato ||
       prog == kProgAdd9Dim || prog == kProgB2b5)) {
    // ROOT=NEO forces the NEO walk for the W7 archetypes (TRANS honored);
    // stasis keeps minor triads dominant: transform with prob 40 + diss/2 %.
    if (v_roll100(st) < (uint8_t)(40u + (uint16_t)in.diss / 2u)) {
      apply_neo_riemannian(&st->root_pc, &st->is_minor,
                           (neo_riemannian_id_t)pick_neo_op(st, in));
      st->quality = st->is_minor ? kQMinor : kQMajor;
    }
    return;
  }
  if (kernel == kKernW7 &&
      (prog == kProgWeitzmann || prog == kProgMediant ||
       prog == kProgHalfTrn || prog == kProgNeoFree)) {
    // ROOT=W7 forces the diatonic fallback for the chromatic archetypes
    // (minor planing over {i, bIII, bVI, bVII}).
    static const uint8_t kFallback[4] = { 0, 2, 5, 6 };
    const uint8_t i = w7_pick(st, kFallback, 4);
    st->prev_deg = kFallback[i];
    st->root_pc = degree_root_pc(st, in, st->prev_deg);
    st->quality = kQMinor;
    st->is_minor = 1;
    return;
  }

  switch (prog) {
    case kProgWeitzmann: {
      st->chord_idx = (uint8_t)((st->chord_idx + 1u) & 3u);
      uint8_t member = kWzCycle[st->chord_idx];
      if (v_roll100(st) < (uint8_t)(in.diss / 2u)) {
        member = v_rand(st, 6);              // NEO pivot within the region
      }
      st->root_pc =
          (uint8_t)((st->anchor_pc + st->key_shift + kWzSemi[member]) % 12u);
      st->is_minor = kWzMinor[member];
      st->quality = st->is_minor ? kQMinor : kQMajor;
      st->prev_deg = 0;
      break;
    }
    case kProgMediant: {
      // minor <-> minor chromatic mediant as a NEO pair:
      // R->P = root +3 (minor third), L->P = root +8 (= -4, major third).
      const uint8_t op = (v_rand(st, 2) == 0) ? (uint8_t)NR_TRANSFORM_R
                                              : (uint8_t)NR_TRANSFORM_L;
      apply_neo_riemannian(&st->root_pc, &st->is_minor,
                           (neo_riemannian_id_t)op);
      apply_neo_riemannian(&st->root_pc, &st->is_minor, NR_TRANSFORM_P);
      st->quality = kQMinor;
      break;
    }
    case kProgAdd9Dim: {
      // Research seed: Am(add9) -> D#dim7(add6) -> G#m -> A (relative to i).
      static const uint8_t kSeedRoot[4] = { 0, 6, 11, 0 };
      static const uint8_t kSeedQual[4] = { kQAdd9, kQDim, kQMinor, kQMajor };
      st->chord_idx = (uint8_t)((st->chord_idx + 1u) & 3u);
      st->root_pc =
          (uint8_t)((tonic_pc(st, in) + kSeedRoot[st->chord_idx]) % 12u);
      st->quality = kSeedQual[st->chord_idx];
      st->is_minor = (st->quality == kQMajor) ? 0u : 1u;
      st->prev_deg = (uint8_t)((kSeedRoot[st->chord_idx] + 3u) % 7u);
      break;
    }
    case kProgHalfTrn: {
      // Half-step + tritone darkness: root motion +-1 / +-6 (Ebm-Em-Bbm).
      static const int8_t kMotion[4] = { 1, -1, 6, -6 };
      const int16_t r = (int16_t)st->root_pc + kMotion[v_rand(st, 4)];
      st->root_pc = (uint8_t)((r + 12) % 12);
      st->quality = kQMinor;
      st->is_minor = 1;
      break;
    }
    case kProgB2b5: {
      // Phrygian power-chord pool (E5 F5 G5 A5 A#5 D#5): W7-weighted over
      // root offsets {0, +1, +3, +5, +6, +11} mapped to W7 pseudo-degrees.
      static const uint8_t kOff[6]  = { 0, 1, 3, 5, 6, 11 };
      static const uint8_t kPdeg[6] = { 0, 1, 2, 3, 6, 6 };
      const uint8_t i = w7_pick(st, kPdeg, 6);
      st->prev_deg = kPdeg[i];
      st->root_pc = (uint8_t)((tonic_pc(st, in) + kOff[i]) % 12u);
      st->quality = kQPower;
      st->is_minor = 1;
      break;
    }
    case kProgNeoFree: {
      // Pure NEO walk (stasis keeps minor triads dominant).
      if (v_roll100(st) < (uint8_t)(40u + (uint16_t)in.diss / 2u)) {
        apply_neo_riemannian(&st->root_pc, &st->is_minor,
                             (neo_riemannian_id_t)pick_neo_op(st, in));
        st->quality = st->is_minor ? kQMinor : kQMajor;
      }
      break;
    }
    case kProgAeolian: {
      // i - bVII / i - bVI - bVII vamps: W7 x CHORD_MARKOV over {0,5,6}.
      static const uint8_t kDegs[3] = { 0, 5, 6 };
      const uint8_t i = w7_pick(st, kDegs, 3);
      st->prev_deg = kDegs[i];
      st->root_pc = degree_root_pc(st, in, st->prev_deg);
      st->quality = kQMinor;
      st->is_minor = 1;
      break;
    }
    case kProgOstinato:
    default: {
      // Dungeon-synth ostinato i - bIII - bVI - bVII: same kernel, {0,2,5,6}.
      static const uint8_t kDegs[4] = { 0, 2, 5, 6 };
      const uint8_t i = w7_pick(st, kDegs, 4);
      st->prev_deg = kDegs[i];
      st->root_pc = degree_root_pc(st, in, st->prev_deg);
      st->quality = kQMinor;
      st->is_minor = 1;
      break;
    }
  }

  // DISS NEO substitution on the diatonic/seed archetypes: chromatic
  // mutation of the picked chord (a substituted bar plays a plain triad).
  if ((prog == kProgAeolian || prog == kProgOstinato ||
       prog == kProgAdd9Dim || prog == kProgB2b5) &&
      in.diss > 0u && v_roll100(st) < (uint8_t)(in.diss / 2u)) {
    apply_neo_riemannian(&st->root_pc, &st->is_minor,
                         (neo_riemannian_id_t)pick_neo_op(st, in));
    st->quality = st->is_minor ? kQMinor : kQMajor;
  }
}

// ---- riff (build_bar) --------------------------------------------------

static const uint8_t kZoneLo[kRiffZoneCount] = { 0, 17, 34, 51, 68, 85 };

// 55..100 across the zone's span; downbeats always survive thinning.
static inline uint8_t riff_density(const Inputs& in) {
  const uint8_t frac = (uint8_t)(in.riff - kZoneLo[zone_of(in)]);
  const uint16_t d = 55u + ((uint16_t)frac * 45u) / 16u;
  return d > 100u ? 100u : (uint8_t)d;
}

// Build the 16-step riff for the current bar (pool + zone + DISS).
static inline void build_bar(State* st, const Inputs& in, Step out[16]) {
  build_pool(st, in);
  const uint8_t zone = zone_of(in);
  const uint8_t dens = riff_density(in);
  const int8_t i_third = pool_top_of(
      st, (st->quality == kQMinor || st->quality == kQAdd9) ? 3 : 4);
  const int8_t i_fifth = pool_top_of(st, 7);
  const int8_t i_high  = pool_top_of(st, 12);
  const int8_t i_pass  = (in.diss >= 65u) ? pool_top_of(st, 6)
                       : (in.diss >= 30u) ? pool_top_of(st, 1) : 0;
  // Riff-cell rotation (mutation), DISS-driven.
  const uint8_t rot =
      (in.diss >= 25u && v_roll100(st) < (uint8_t)(in.diss / 4u))
          ? v_rand(st, 4) : 0u;

  for (uint8_t s = 0; s < 16u; ++s) {
    Step& t = out[s];
    const uint8_t beat = (uint8_t)(s & 3u);
    bool gate = false, acc = false, slide = false;
    int8_t off = 0, oct = 0;
    switch (zone) {
      case kRiffSustain: {
        gate = (s == 0u || s == 8u);
        slide = gate;
        off = (s == 8u) ? i_fifth : 0;
        acc = (s == 0u);
        break;
      }
      case kRiffHalf: {
        gate = (beat == 0u);
        off = (s == 0u || s == 8u) ? 0 : i_fifth;
        acc = (s == 0u || s == 8u);
        break;
      }
      case kRiff8th: {
        // 4-step cell: root, fifth, root, third (rotated by mutation).
        static const uint8_t kCell[4] = { 0, 2, 0, 1 };
        gate = ((s & 1u) == 0u);
        const uint8_t c = kCell[(uint8_t)(((s >> 1) + rot) & 3u)];
        off = (c == 0u) ? 0
            : (c == 1u) ? i_third
            : (c == 2u) ? i_fifth : i_high;
        acc = (s == 0u || s == 8u);
        break;
      }
      case kRiff16th: {
        gate = true;
        off = (int8_t)((s + rot) % st->pool_n);
        acc = (beat == 0u);
        break;
      }
      case kRiffGallop: {
        // 3+1 gallop cell: root, root, fifth, rest.
        static const uint8_t kGal[3] = { 0, 0, 2 };
        gate = (beat != 3u);
        const uint8_t c = kGal[beat % 3u];
        off = (c == 2u) ? i_fifth : 0;
        acc = (beat == 0u);
        break;
      }
      case kRiffBlast:
      default: {
        gate = true;
        off = (s & 1u) ? i_high : 0;
        acc = (beat == 0u);
        break;
      }
    }
    // Density thinning: downbeats always survive.
    if (gate && beat != 0u && dens < 100u) {
      gate = (v_roll100(st) < dens);
    }
    // DISS passing tones on odd 16ths in the tremolo zones.
    if (gate && (s & 1u) == 1u && zone >= kRiff16th && in.diss > 0u &&
        v_roll100(st) < (uint8_t)(in.diss / 3u)) {
      off = i_pass;
      acc = false;
    }
    // Octave displacement in the tremolo zones (uses the RANGE octaves).
    if (gate && zone >= kRiff8th && in.range > 1u && (s & 7u) == 5u &&
        v_roll100(st) < 40u) {
      oct = (int8_t)(1u + v_rand(st, (uint8_t)(in.range - 1u)));
    }
    t.flags = (uint8_t)((gate ? kFlagGate : 0u) | (acc ? kFlagAccent : 0u) |
                        (slide ? kFlagSlide : 0u));
    t.off = off;
    t.oct = oct;
  }
}

}  // namespace varg
#endif  // RACK_SRC_VARG_H_
