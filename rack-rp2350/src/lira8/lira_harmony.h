// LIRA-8 ARP-driven voice pitch — voicing math + the cross-module snapshot.
//
// The MOD-ARP engine (mod_engine.h) is the rack's generative harmony source:
// every ARP instance always computes its root, scale, current chord degree
// and current step note, even when its destination is unrouted. Two consumers
// live in this header:
//
//   1) ARP HARM switch (lira8 P3, CC 54): all 8 voices are retuned from the
//      ARP's harmonic state — low group (voices 1-4 / pairs 0-1) to a perfect
//      bass stack, high group (voices 5-8 / pairs 2-3) to the current scale
//      chord (3rd / 5th / 7th / root).
//   2) ARP -> PITCH 1234 / PITCH 5678 (CC 25/26) destinations: the mapped
//      ARP's exact step note becomes that group's root (same voicing law).
//
// Both paths reduce to: a group ROOT (MIDI note) + 4 semitone offsets.
// Pure C++ (no MCU headers) so the host harness tests it directly — this
// property is part of the LIRA-8 port's ctag-tbd portability proof.
#ifndef RACK_SRC_LIRA8_LIRA_HARMONY_H_
#define RACK_SRC_LIRA8_LIRA_HARMONY_H_

#include <cmath>
#include <cstdint>

namespace l8harmony {

static const int kGroupVoices = 4;

// One group's harmonic context (as published by the mod engine).
struct GroupCtx {
  uint8_t root;         // MIDI note the group's chord is built on
  uint8_t nsteps;       // scale size (0 = invalid context)
  uint8_t scale_st[12]; // ascending semitone offsets of the scale
  int8_t  chord_deg;    // chord root as a scale degree (0..nsteps-1)
  uint8_t chord_nt;     // 3 = triad, 4 = seventh chord
};

// Cross-module snapshot. Written by the mod engine on core 1 BEFORE the
// module render and read by the lira8 adapter right after — same core, no
// locking. `seq` increments only when a published value changes, so the
// adapter recomputes the eight voice frequencies only on chord/route moves.
struct Snapshot {
  uint8_t  harmony_valid; // an ARP context exists (harmony/ctx usable)
  GroupCtx harmony;       // root folded to C3..B3 (stable register)
  uint8_t  lo_mapped;     // an ARP is routed to CC 25 (exact-note mode)
  GroupCtx lo_ctx;
  uint8_t  hi_mapped;     // an ARP is routed to CC 26 (exact-note mode)
  GroupCtx hi_ctx;
  uint32_t seq;
};

// The one instance (inline: header-only firmware TU + independent host tests).
inline Snapshot g_lira_harmony{};

// Fold any MIDI note to the C3..B3 octave (pitch class only) so the harmony
// mode keeps a stable register regardless of the ARP root's octave. Mapped
// groups (CC 25/26) keep the ARP's EXACT note — no folding there.
static inline uint8_t fold_root(uint8_t note) {
  return static_cast<uint8_t>(48u + (note % 12u));
}

// Semitone interval of scale degree (chord_deg + 2k) above the chord root,
// measured inside the scale — the same chord-tone law the ARP uses to pick
// its melodic steps (every other scale degree), so the pads always play
// material that belongs to the arpeggio's own harmony.
static inline int scale_chord_interval(const GroupCtx& g, int k) {
  const int n = g.nsteps ? g.nsteps : 7;
  const int deg = (g.chord_deg % n + n) % n;
  const int total = deg + 2 * k;
  const int oct = total / n;
  const int idx = total % n;
  return static_cast<int>(g.scale_st[idx]) + 12 * oct -
         static_cast<int>(g.scale_st[deg]);
}

// Fills a group's 4 voice offsets relative to its root note.
//   group 0 (voices 1-4 / low):  -12, -5, 0, +7
//     root + fifth over two octaves — perfect intervals, consonant in every
//     scale (scale-stacking this low can put a 6th/7th in the bass).
//   group 1 (voices 5-8 / high): third+12, fifth+12,
//     (seventh+12 | third+24 on a triad), +24
//     the scale's color tones, so NEO's major/minor flip and W7's chord drift
//     are heard where they belong.
static inline void group_offsets(const GroupCtx& g, int group, int8_t out[4]) {
  if (group == 0) {
    out[0] = -12;
    out[1] = -5;
    out[2] = 0;
    out[3] = 7;
    return;
  }
  const int third = scale_chord_interval(g, 1);
  const int fifth = scale_chord_interval(g, 2);
  out[0] = static_cast<int8_t>(third + 12);
  out[1] = static_cast<int8_t>(fifth + 12);
  out[2] = static_cast<int8_t>((g.chord_nt >= 4)
                                   ? scale_chord_interval(g, 3) + 12
                                   : third + 24);
  out[3] = 24;
}

// Equal-tempered MIDI note -> Hz (the engine's mtof law).
static inline float note_hz(int note) {
  return 440.0f * std::pow(2.0f, (static_cast<float>(note) - 69.0f) / 12.0f);
}

}  // namespace l8harmony

#endif  // RACK_SRC_LIRA8_LIRA_HARMONY_H_
