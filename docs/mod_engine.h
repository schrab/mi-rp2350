// Rack RP2350 — T3 persistent modulation engine + clock bus + T5/Arp sources.
//
// Persistent sources (MOD_SOURCE_COUNT = 12):
//   0..3  : lfo1..4   (MOD_LFO)
//   4..5  : eu1..2    (MOD_EUCLID)
//   6..7  : gate1..2  (MOD_GATE — untouched)
//   8..9  : adsr1..2  (MOD_ADSR — untouched)
//   10..11: arp1..2   (MOD_ARP — switchable 3-algo generative melody/gate)
//
// Note on GPL-3.0: The HYP-1 algorithm logic in MOD_ARP Algo 0 is re-implemented
// from the mathematical specifications in hyp1-mono-arp-algorithms.md.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#include "pico/time.h"           // time_us_32 (static inline, pico-sdk 2.x)

#include "module_registry.h"     // RackModuleInfo / RackModDestinations / ids

extern volatile uint8_t g_active_module;   // defined in main.cpp (non-static since T3)

// T6 chain-mode grid override
extern volatile uint8_t g_mod_chain_grid;       // 0 = off, else block size
extern volatile uint8_t g_mod_chain_rate_khz;   // 0 = off, else rate/1000

// ===================== source taxonomy =====================
enum ModSourceType : uint8_t {
  MOD_LFO = 0, MOD_EUCLID = 1, MOD_GATE = 2, MOD_ADSR = 3, MOD_ARP = 4,
};
static const uint8_t MOD_TYPE_COUNT = 5;

enum ModWave : uint8_t {
  MOD_WAVE_SINE = 0, MOD_WAVE_TRI = 1, MOD_WAVE_SAW = 2, MOD_WAVE_SQR = 3,
  MOD_WAVE_SH = 4,
};
static const uint8_t MOD_WAVE_SH_COUNT = 5;

enum ModClockSel : uint8_t {
  MOD_CLK_GLOBAL = 0,   // follow the global bus source (bpm/clock commands)
  MOD_CLK_INTERNAL = 1, // per-source override: internal accumulator
  MOD_CLK_MIDI = 2,     // per-source override: external MIDI clock
};

enum ModAdsrState : uint8_t {
  MOD_ENV_IDLE = 0, MOD_ENV_A = 1, MOD_ENV_D = 2, MOD_ENV_S = 3, MOD_ENV_R = 4,
};

enum ModArpAlgo : uint8_t {
  MOD_ARP_HYP1 = 0,     // HYP-1 Acid / W7 Chord-Drift + Mutate + Fill Bar
  MOD_ARP_BARLOW = 1,   // Barlow Indispensability + Sioros-Guedes Syncopation
  MOD_ARP_GRIDS = 2,    // 2D Topographic Trance-Gate (Grids) + Nibble Arp
};

// Source ids (CC 33 values):
// 0-3 LFO1-4, 4-5 EU1-2, 6-7 GATE1-2, 8-9 ADSR1-2, 10-11 ARP1-2.
static const uint8_t MOD_SOURCE_COUNT = 12;
static const uint8_t MOD_SRC_EU_FIRST = 4;    // odometer source for telemetry
static const uint8_t MOD_SRC_ARP_FIRST = 10;  // arp1 = 10, arp2 = 11
static const uint8_t MOD_ARP_COUNT = 2;
static const uint8_t MOD_DEST_VOICE = 127;    // universal voice pitch+gate pseudo-CC

// Clock divisions: PPQ ticks per division cycle (24 PPQ = quarter note).
struct ModDivision { const char* name; uint8_t ppq; };
static const ModDivision kModDivisions[10] = {
    {"1/1", 96}, {"1/2", 48}, {"1/4", 24}, {"1/8", 12}, {"1/16", 6},
    {"1/32", 3}, {"1/4T", 16}, {"1/8T", 8}, {"1/16T", 4}, {"1/32T", 2},
};
static const uint8_t MOD_DIVISION_COUNT = 10;

// Per-source parameter names/counts — the CC 34 param indexes and the console
// `mod <src> <param> <value>` names use the SAME tables. The last three
// params of every type are uniformly clk / div / dest.
static const char* const kModParamNames[MOD_TYPE_COUNT][8] = {
    {"mode", "rate", "wave", "phase", "level", "clk", "div", "dest"},   // LFO
    {"steps", "pulses", "rot", "clk", "div", "dest"},                   // EUCLID
    {"len", "mask", "glen", "clk", "div", "dest"},                      // GATE
    {"mode", "atk", "dec", "sus", "rel", "clk", "div", "dest"},         // ADSR
    {"algo", "root", "scale", "dens", "groove", "clk", "div", "dest"},  // ARP
};
static const uint8_t kModParamCount[MOD_TYPE_COUNT] = {8, 6, 6, 8, 8};

// Compact 16-step pattern + harmony state for arp1 and arp2 (64 B per instance).
struct ModArpStep {
  uint8_t flags;   // bit 0: gate, bit 1: accent, bit 2: slide
  int8_t  off;     // scale steps above current chord degree
  int8_t  oct;     // octave offset (0..range-1)
};

struct ModArpExt {
  ModArpStep steps[16];
  uint8_t scale_st[12];
  uint8_t nsteps;
  uint8_t chord_nt;
  int8_t  deg;
  uint8_t bar_count;
  uint8_t dd_fill;
};

struct ModSource {
  // common
  uint8_t type;          // ModSourceType
  uint8_t clock_sel;     // ModClockSel
  uint8_t division;      // index into kModDivisions
  uint8_t dest_cc;       // destination CC (0 = un-routed)
  float   clk_phase;     // 0..1 division-cycle phase (LFO wave phase when clocked)
  float   out;           // last output (0..1) — telemetry
  uint8_t was_high;      // last trigger-source level (edge detect)
  uint32_t rng;          // per-source LCG state (S&H / generative arp)
  float   sh_value;      // S&H held value
  float   sh_acc;        // S&H 1 Hz re-trigger accumulator (Hz mode)
  union {
    struct { float rate_hz, level, phase_off; uint8_t wave, hz_mode; } lfo;
    struct { uint8_t steps, pulses, rot; uint32_t mask; uint8_t step;
             uint32_t ticks; uint8_t level; } eu;
    struct { uint8_t len; uint32_t mask; float gate_len; uint8_t step;
             uint32_t ticks; uint8_t level; } gate;
    struct { float a_s, d_s, s_lvl, r_s; uint8_t retrig, env_state; float env;
             uint32_t ticks; } adsr;
    struct {
      uint8_t algo;           // 0=HYP1, 1=BARLOW, 2=GRIDS
      uint8_t range;          // 1..3 octaves
      uint8_t root_param;     // 0 = MIDI follow, 24..84 = fixed root MIDI note
      uint8_t midi_root;      // last captured external MIDI note (default 48 = C3)
      uint8_t last_out_note;  // last note written to shared_note by this arp
      uint8_t scale_idx;      // 0..15 (low 3 bits = mode 0..7, bit 3 = chord flavor)
      uint8_t density;        // 0..100
      uint8_t groove;         // 0..100 (syncopation / morph / mutate+slide)
      uint8_t step;           // 0..15 current step
      uint8_t cur_note;       // active step MIDI note (28..96)
      uint8_t level;          // 1 if current step is gated
      uint8_t cur_acc;        // 1 if current step is accented
      uint8_t cur_hold;       // 1 if current step has slide (full-step gate)
      uint8_t retrig_edge;    // 1 if step should re-fire trigger (not a slide)
      uint8_t held;           // 1 if previous step left voice sounding
      uint8_t ext_idx;        // 0 for arp1, 1 for arp2
      uint32_t ticks;         // step odometer
    } arp;
  } p;
};

// ===================== clock-bus stamp ring =====================
static const uint32_t kModStampRingSize = 64;    // power of two
enum ModStampStatus : uint8_t {
  MOD_STAMP_TICK = 0xF8, MOD_STAMP_START = 0xFA,
  MOD_STAMP_CONT = 0xFB, MOD_STAMP_STOP = 0xFC,
};
struct ModStamp {
  uint64_t pos;    // render sample position at arrival (sub-block estimated)
  uint8_t status;  // ModStampStatus
  uint8_t pad[7];
};

struct ModEngine {
  // ---- clock bus ----
  float    bpm;               // 20-300, quantized to 0.1
  uint8_t  global_clock;      // 0 internal / 1 midi (the `clock` command)
  float    int_phase;         // internal PPQ-tick accumulator (fractional)
  // MIDI transport + measured tempo
  uint8_t  midi_running;      // FA/FB set, FC clears
  uint32_t midi_tick_count;   // total F8 consumed (odometer, wraps ~57 days)
  float    midi_bpm_meas;     // EMA of incoming tempo (0 = unknown yet)
  uint64_t last_tick_pos;     // last consumed F8 stamp position
  uint8_t  last_tick_valid;
  // ---- block-position exports (core 1 writes, core 0 reads when stamping) --
  volatile uint64_t sample_counter;   // first sample index of the CURRENT block
  volatile uint32_t block_start_us;   // time_us_32 at the current block's top
  volatile uint32_t block_period_us;  // current block period (us)
  volatile uint32_t block_size_now;   // current block size (samples)
  // ---- stamp ring ----
  volatile uint32_t stamp_head;       // core 0 writes
  volatile uint32_t stamp_tail;       // core 1 writes
  volatile uint32_t stamp_overflow;   // dropped stamps (ring full)
  ModStamp ring[kModStampRingSize];
  // ---- sources ----
  ModSource src[MOD_SOURCE_COUNT];
  ModArpExt arp_ext[MOD_ARP_COUNT];
  // ---- telemetry (core 1 writes, core 0 prints + resets each 2 s report) ---
  volatile uint32_t mod_us_max;       // engine block cost, max since report
  volatile uint32_t telem_midi_ticks; // F8 consumed since last report
};
extern ModEngine g_mod_engine;   // DEFINED in main.cpp — plain .bss, NOT arenas

// ===================== small helpers =====================
static inline float mod_clamp01(float v) {
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static inline uint32_t mod_u32_rand(ModSource& s) {
  s.rng = s.rng * 1664525u + 1013904223u;
  return s.rng;
}

static inline float mod_rand01(ModSource& s) {
  return (float)(mod_u32_rand(s) >> 8) * (1.0f / 16777216.0f);
}

static inline uint8_t mod_irand(ModSource& s, uint8_t n) {
  return n > 0 ? (uint8_t)(((mod_u32_rand(s) >> 16) * n) >> 16) : 0;
}

// LFO wave, UNIPOLAR 0..1
static inline float mod_wave(uint8_t wave, float ph, float sh_value) {
  switch (wave) {
    case MOD_WAVE_SINE: return 0.5f - 0.5f * cosf(6.2831853f * ph);
    case MOD_WAVE_TRI:  return ph < 0.5f ? 2.0f * ph : 2.0f - 2.0f * ph;
    case MOD_WAVE_SAW:  return ph;
    case MOD_WAVE_SQR:  return ph < 0.5f ? 0.0f : 1.0f;
    default:            return sh_value;                 // S&H
  }
}

// Euclidean mask
static uint32_t mod_euclid_mask(uint8_t pulses, uint8_t steps) {
  if (steps == 0 || steps > 32 || pulses == 0) return 0;
  if (pulses >= steps) {
    return (steps >= 32) ? 0xFFFFFFFFu : ((1u << steps) - 1u);
  }
  uint32_t mask = 0;
  uint32_t bucket = 0;
  for (uint8_t i = 0; i < steps; ++i) {
    bucket += pulses;
    if (bucket >= steps) { bucket -= steps; mask |= (1u << i); }
  }
  uint8_t shift = 0;
  while (shift < steps && !(mask & (1u << shift))) ++shift;
  if (shift && shift < steps) {
    if (steps >= 32) {
      mask = (mask >> shift) | (mask << (32u - shift));
    } else {
      mask = ((mask >> shift) | (mask << (steps - shift))) & ((1u << steps) - 1u);
    }
  }
  return mask;
}

static uint32_t mod_rotate_mask(uint32_t mask, uint8_t steps, uint8_t rot) {
  if (steps <= 1 || rot == 0) return mask;
  const uint8_t r = rot % steps;
  if (r == 0) return mask;
  if (steps >= 32) return (mask >> r) | (mask << (32u - r));
  const uint32_t m = mask & ((1u << steps) - 1u);
  return ((m >> r) | (m << (steps - r))) & ((1u << steps) - 1u);
}

static void mod_eu_recompute(ModSource& s) {
  if (s.p.eu.steps < 1) s.p.eu.steps = 1;
  if (s.p.eu.steps > 32) s.p.eu.steps = 32;
  if (s.p.eu.pulses > s.p.eu.steps) s.p.eu.pulses = s.p.eu.steps;
  s.p.eu.mask = mod_rotate_mask(
      mod_euclid_mask(s.p.eu.pulses, s.p.eu.steps), s.p.eu.steps, s.p.eu.rot);
  s.p.eu.step %= s.p.eu.steps;
}

// ===================== MOD_ARP: 3-Algo Melody & Trance-Gate Engine ===========
// Modes: Ionian, Dorian, Phrygian, Lydian, Mixolydian, Aeolian, PentMaj, PentMin
static const uint16_t kArpModePools[8] = {
    0x0AB5u, 0x06ADu, 0x06B5u, 0x0ABDu, 0x0AADu, 0x05ADu, 0x0125u, 0x04A9u
};

// Barlow 16-step 4/4 Metrical Indispensability weights (Step 0..15)
static const uint8_t kBarlow16[16] = {
    15, 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7
};
static const uint8_t kBarlowSyncOrder[8] = {0, 4, 8, 12, 2, 6, 10, 14};

// 2D Topographic Trance-Gate Nodes (Mutable Instruments Grids style, 4 corners)
// Node 00: Offbeat/Alternate | Node 10: Forward Gallop (3+1)
// Node 01: Tresillo / Clave  | Node 11: Vanguard Anthem Chopper
static const uint8_t kGridsNode00[16] = {220, 40, 255, 60, 180, 20, 240, 80, 200, 30, 250, 50, 170, 10, 230, 110};
static const uint8_t kGridsNode10[16] = {255, 190, 210, 20, 240, 170, 200, 30, 250, 185, 205, 15, 235, 175, 220, 90};
static const uint8_t kGridsNode01[16] = {255, 20, 60, 230, 40, 80, 240, 30, 140, 210, 50, 90, 225, 35, 160, 195};
static const uint8_t kGridsNode11[16] = {255, 50, 200, 175, 30, 225, 60, 190, 40, 235, 70, 180, 245, 160, 80, 210};

static inline uint8_t mod_u8_mix(uint8_t a, uint8_t b, uint8_t bal) {
  return (uint8_t)(((uint16_t)a * (255u - bal) + (uint16_t)b * bal) >> 8);
}

static void mod_arp_rebuild_scale(ModSource& s) {
  ModArpExt& ext = g_mod_engine.arp_ext[s.p.arp.ext_idx];
  const uint16_t pool = kArpModePools[s.p.arp.scale_idx & 0x07u] | 1u;
  uint8_t m = 0;
  for (uint8_t i = 0; i < 12; ++i) {
    if ((pool >> i) & 1u) ext.scale_st[m++] = i;
  }
  ext.nsteps = m ? m : 7;
  if (ext.deg >= (int8_t)ext.nsteps) ext.deg = 0;
  ext.chord_nt = (ext.nsteps >= 6) ? 4 : 3;
}

// HYP-1 W7 weighted random bar-line chord progression
static void mod_arp_next_chord(ModSource& s) {
  ModArpExt& ext = g_mod_engine.arp_ext[s.p.arp.ext_idx];
  static const uint8_t kW7[7] = {40, 20, 10, 30, 30, 30, 4};
  uint32_t w[12], sum = 0;
  for (uint8_t i = 0; i < ext.nsteps; ++i) {
    w[i] = (i == (uint8_t)ext.deg) ? 0u
         : (ext.nsteps == 7) ? kW7[i]
         : (i == 0) ? 30u : 15u;
    sum += w[i];
  }
  uint32_t r = mod_irand(s, (uint8_t)(sum ? sum : 1));
  uint8_t next_deg = 0;
  for (uint8_t i = 0; i < ext.nsteps; ++i) {
    if (r < w[i] && w[i] > 0) { next_deg = i; break; }
    r -= w[i];
  }
  if (next_deg == (uint8_t)ext.deg) {
    next_deg = (ext.deg == 0 && ext.nsteps > 1) ? 1 : 0;
  }
  ext.deg = (int8_t)next_deg;
}

// Pick chord-tone offset (even scale steps) or passing tone
static int8_t mod_arp_pick_offset(ModSource& s, const ModArpExt& ext, bool strong_beat) {
  const bool extended_chord = (s.p.arp.scale_idx & 0x08u) != 0;
  if (strong_beat || mod_irand(s, 100) < 80) {
    uint8_t idx = mod_irand(s, ext.chord_nt);
    int8_t off = (int8_t)(2 * idx);
    if (extended_chord && idx == 2) off = 6; // 9th/Sus flavor skips 5th for 7th/9th
    if (mod_irand(s, 100) < 15) off = (int8_t)(off + ext.nsteps);
    return off;
  }
  return (int8_t)mod_irand(s, ext.nsteps);
}

// Roll a single step for Algo 0 (HYP-1 ac_roll)
static void mod_arp_roll_hyp1_step(ModSource& s, ModArpExt& ext, uint8_t st) {
  ModArpStep& step = ext.steps[st];
  const uint8_t dens = s.p.arp.density;
  const uint8_t eff_dens = (st == 0) ? (dens + 30 > 100 ? 100 : dens + 30) : dens;
  const bool gate = mod_irand(s, 100) < eff_dens;
  const bool acc  = mod_irand(s, 100) < 30;
  const bool sld  = mod_irand(s, 100) < ((uint16_t)s.p.arp.groove * 70 / 100);
  step.flags = (gate ? 1u : 0u) | (acc ? 2u : 0u) | (sld ? 4u : 0u);
  step.off = mod_arp_pick_offset(s, ext, (st & 3u) == 0u);
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;
  step.oct = (mod_irand(s, 100) < 55) ? 0 : (int8_t)mod_irand(s, rng_oct);
}

// Rebuild all 16 steps for the currently selected algorithm (0=HYP1, 1=BARLOW, 2=GRIDS)
static void mod_arp_rebuild_pattern(ModSource& s) {
  ModArpExt& ext = g_mod_engine.arp_ext[s.p.arp.ext_idx];
  mod_arp_rebuild_scale(s);
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;

  if (s.p.arp.algo == MOD_ARP_HYP1) {
    for (uint8_t st = 0; st < 16; ++st) {
      mod_arp_roll_hyp1_step(s, ext, st);
    }
    return;
  }

  if (s.p.arp.algo == MOD_ARP_BARLOW) {
    // 1. Deterministic Barlow 4/4 metrical skeleton
    const uint8_t active_steps = (uint8_t)((uint16_t)s.p.arp.density * 16 / 100);
    const uint8_t threshold = (active_steps >= 16) ? 0 : (16 - active_steps);
    uint8_t on[16], sld[16];
    for (uint8_t i = 0; i < 16; ++i) {
      on[i] = (kBarlow16[i] >= threshold) ? 1u : 0u;
      sld[i] = 0u;
    }
    // 2. Sioros-Guedes weak-strong left-shift anticipation with anti-offset rule
    const uint8_t sync_prob = (uint8_t)((uint16_t)s.p.arp.groove * 220 / 100);
    const uint8_t tie_prob  = (uint8_t)((uint16_t)s.p.arp.groove * 140 / 100);
    bool prev_shifted = false;
    for (uint8_t c = 0; c < 8; ++c) {
      const uint8_t i = kBarlowSyncOrder[c];
      const uint8_t prev_i = (i + 15u) & 0x0Fu;
      if (on[i] && !on[prev_i] && !prev_shifted) {
        if (mod_irand(s, 255) < sync_prob) {
          on[prev_i] = 1u;
          if (mod_irand(s, 255) < tie_prob) {
            sld[prev_i] = 1u; // anticipated pickup glides into strong beat
          } else {
            on[i] = 0u;       // true syncopation (strong beat muted)
          }
          prev_shifted = true;
        } else {
          prev_shifted = false;
        }
      } else {
        prev_shifted = false;
      }
    }
    // 3. Assign pitch offsets by metrical indispensability
    static const int8_t kChordWalk[4] = {0, 4, 2, 6};
    for (uint8_t st = 0; st < 16; ++st) {
      const uint8_t w = kBarlow16[st];
      const bool acc = (w >= 13) || (sld[st] != 0);
      ext.steps[st].flags = (on[st] ? 1u : 0u) | (acc ? 2u : 0u) | (sld[st] ? 4u : 0u);
      if (w >= 12) {
        ext.steps[st].off = kChordWalk[(st >> 2) & 3u]; // anchor beats on root/5th/3rd
        ext.steps[st].oct = 0;
      } else if (w >= 8) {
        ext.steps[st].off = (int8_t)(2 * ((st >> 1) % ext.chord_nt)); // offbeats on chord tones
        ext.steps[st].oct = (rng_oct > 1 && (st & 4u)) ? 1 : 0;
      } else {
        ext.steps[st].off = mod_arp_pick_offset(s, ext, false);
        ext.steps[st].oct = (rng_oct > 1) ? (int8_t)mod_irand(s, rng_oct) : 0;
      }
    }
    return;
  }

  // MOD_ARP_GRIDS: 2D Topographic Trance-Gate + Call-and-Response Nibble Phrasing
  const uint8_t g = s.p.arp.groove;
  const uint8_t morph_x = (g <= 50) ? (uint8_t)(g * 5) : (uint8_t)(255 - (g - 50) * 3);
  const uint8_t morph_y = (uint8_t)((uint16_t)g * 255 / 100);
  const uint8_t chaos   = (g > 65) ? (uint8_t)((g - 65) * 4) : 0;
  const uint8_t fill    = (uint8_t)((uint16_t)s.p.arp.density * 255 / 100);
  const uint8_t cutoff  = 255u - fill;

  // ABAC call-and-response chord-tone motifs (4-step nibbles)
  static const int8_t kMotifA[4] = {0, 2, 4, 2};
  static const int8_t kMotifB[4] = {2, 4, 6, 4};
  static const int8_t kMotifC[4] = {0, 4, 6, 7};

  for (uint8_t st = 0; st < 16; ++st) {
    const uint8_t top = mod_u8_mix(kGridsNode00[st], kGridsNode10[st], morph_x);
    const uint8_t bot = mod_u8_mix(kGridsNode01[st], kGridsNode11[st], morph_x);
    int16_t val = mod_u8_mix(top, bot, morph_y);
    if (chaos > 0) {
      int16_t jit = ((((int16_t)mod_irand(s, 255)) - 128) * chaos) >> 8;
      val += jit;
      if (val < 0) val = 0;
      if (val > 255) val = 255;
    }
    const bool gate = ((uint8_t)val > cutoff);
    const bool acc  = gate && (val >= 192);
    const bool sld  = gate && (val >= 225) && ((st & 3u) != 3u);
    ext.steps[st].flags = (gate ? 1u : 0u) | (acc ? 2u : 0u) | (sld ? 4u : 0u);

    const uint8_t beat = st >> 2;
    const uint8_t sub  = st & 3u;
    const int8_t* motif = (beat == 0 || beat == 2) ? kMotifA : (beat == 1 ? kMotifB : kMotifC);
    ext.steps[st].off = motif[sub];
    ext.steps[st].oct = (acc && rng_oct > 1) ? (int8_t)((st & 1u) % rng_oct) : 0;
  }
}

// Compute MIDI note (28..96) for a step
static uint8_t mod_arp_calc_note(const ModSource& s, const ModArpExt& ext,
                                 const ModArpStep& step) {
  const uint8_t base_root = (s.p.arp.root_param == 0)
      ? (s.p.arp.midi_root ? s.p.arp.midi_root : 48u)
      : s.p.arp.root_param;
  const int d = (int)ext.deg + (int)step.off;
  const int nst = (int)(ext.nsteps ? ext.nsteps : 7);
  const int deg_mod = ((d % nst) + nst) % nst;
  const int oct_div = d >= 0 ? (d / nst) : -1;
  int note = (int)base_root + (int)ext.scale_st[deg_mod] + 12 * (oct_div + (int)step.oct);
  while (note > 96) note -= 12;
  while (note < 28) note += 12;
  return (uint8_t)note;
}

// ===================== engine -> adapter dispatch =====================
static bool mod_write_cc(uint8_t module_id, uint8_t cc, float v) {
  switch (module_id) {
    case RACK_MODULE_ELEMENTS:
      return rack_modules::mod_elements::mod_set_cc_float(cc, v);
    case RACK_MODULE_CLOUDS:
      return rack_modules::mod_clouds::mod_set_cc_float(cc, v);
    case RACK_MODULE_PLAITS:
      return rack_modules::mod_plaits::mod_set_cc_float(cc, v);
    default:
      return rack_modules::mod_rings::mod_set_cc_float(cc, v);
  }
}

static void mod_fire_trigger(uint8_t module_id, uint8_t cc, bool high) {
  switch (module_id) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::mod_trigger_cc(cc, high); break;
    case RACK_MODULE_CLOUDS:
      rack_modules::mod_clouds::mod_trigger_cc(cc, high); break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::mod_trigger_cc(cc, high); break;
    default:
      rack_modules::mod_rings::mod_trigger_cc(cc, high); break;
  }
}

static int8_t mod_trigger_kind(uint8_t module_id, uint8_t cc) {
  if (cc == MOD_DEST_VOICE) return RACK_TRIG_PULSE; // universal voice pitch+gate target
  const RackModDestinations* d = kRackModules[module_id].mod_destinations;
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == cc) return static_cast<int8_t>(d->trigger_kind[i]);
  }
  return -1;
}

// ===================== defaults (.bss is zero; these are the bench states) ==
void mod_engine_init() {
  ModEngine& e = g_mod_engine;
  memset(&e, 0, sizeof(e));
  e.bpm = 120.0f;
  e.global_clock = 0;              // internal
  e.midi_running = 0;
  e.block_period_us = 0;
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    s.type = (i < 4) ? MOD_LFO : (i < 6) ? MOD_EUCLID
           : (i < 8) ? MOD_GATE : (i < 10) ? MOD_ADSR : MOD_ARP;
    s.clock_sel = MOD_CLK_GLOBAL;
    s.division = 2;                // 1/4 default (overridden below)
    s.dest_cc = 0;                 // nothing routes until the bench routes it
    s.rng = 0x1fff7a10u + i * 2654435761u;
    if (s.type == MOD_LFO) {
      s.p.lfo.hz_mode = 1;
      s.p.lfo.level = 0.8f;
      s.p.lfo.wave = i;            // sine / tri / saw / square across the four
      static const float kRates[4] = {1.0f, 0.5f, 2.0f, 0.25f};
      s.p.lfo.rate_hz = kRates[i];
    } else if (s.type == MOD_EUCLID) {
      s.division = 4;              // 1/16: E(4,16) = four-on-the-floor per bar
      s.p.eu.steps = 16;
      s.p.eu.pulses = 4;
      s.p.eu.rot = 0;
      mod_eu_recompute(s);
    } else if (s.type == MOD_GATE) {
      s.division = 3;              // 1/8 (untouched)
      s.p.gate.len = 8;
      s.p.gate.mask = 0x55u;
      s.p.gate.gate_len = 0.5f;
    } else if (s.type == MOD_ADSR) {
      s.p.adsr.a_s = 0.005f;
      s.p.adsr.d_s = 0.100f;
      s.p.adsr.s_lvl = 0.5f;
      s.p.adsr.r_s = 0.200f;
      s.p.adsr.retrig = 0;
    } else {                       // MOD_ARP (arp1 = 10, arp2 = 11)
      s.division = 4;              // 1/16 default
      s.p.arp.ext_idx = (uint8_t)(i - MOD_SRC_ARP_FIRST);
      s.p.arp.algo = (i == MOD_SRC_ARP_FIRST) ? MOD_ARP_HYP1 : MOD_ARP_BARLOW;
      s.p.arp.range = 2;
      s.p.arp.root_param = 0;      // 0 = MIDI root follow by default
      s.p.arp.midi_root = 48;      // C3 fallback before first MIDI note
      s.p.arp.scale_idx = 5;       // Aeolian (natural minor)
      s.p.arp.density = 62;
      s.p.arp.groove = 40;
      s.p.arp.cur_note = 48;
      mod_arp_rebuild_pattern(s);
    }
  }
}

// FA re-phase: MIDI-clocked sources only (internal clock keeps running).
static void mod_rephase_midi_sources() {
  ModEngine& e = g_mod_engine;
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    const bool use_midi = (s.clock_sel == MOD_CLK_MIDI) ||
        (s.clock_sel == MOD_CLK_GLOBAL && e.global_clock == 1u);
    if (!use_midi) continue;
    s.clk_phase = 0.0f;
    if (s.type == MOD_EUCLID) {
      s.p.eu.step = 0;
      s.p.eu.level = (s.p.eu.mask & 1u) ? 1u : 0u;
    } else if (s.type == MOD_GATE) {
      s.p.gate.step = 0;
      s.p.gate.level = (s.p.gate.mask & 1u) ? 1u : 0u;
    } else if (s.type == MOD_ADSR && s.p.adsr.retrig) {
      s.p.adsr.env = 0.0f;
      s.p.adsr.env_state = MOD_ENV_IDLE;
      s.p.adsr.ticks = 0;
    } else if (s.type == MOD_ARP) {
      s.p.arp.step = 0;
      s.p.arp.held = 0;
    }
  }
}

// ===================== core 1: the per-block engine pass =====================
void mod_engine_process_block(uint8_t active_module) {
  const uint32_t t0 = time_us_32();
  ModEngine& e = g_mod_engine;
  const RackModuleInfo& m = kRackModules[active_module];
  const uint32_t bs = g_mod_chain_grid ? g_mod_chain_grid : m.block_size;
  const float sr = g_mod_chain_rate_khz ? g_mod_chain_rate_khz * 1000.0f
                                        : (float)m.sample_rate;
  const float dt = (float)bs / sr;

  e.block_start_us = time_us_32();
  e.block_size_now = bs;
  e.block_period_us = (uint32_t)(dt * 1000000.0f + 0.5f);

  // ---- 1) drain the stamp ring (core 0 -> core 1), in order ----
  uint32_t midi_ticks = 0;
  while (e.stamp_tail != e.stamp_head) {
    const ModStamp st = e.ring[e.stamp_tail];
    e.stamp_tail = (e.stamp_tail + 1u) & (kModStampRingSize - 1u);
    if (st.status == MOD_STAMP_TICK) {
      ++e.midi_tick_count;
      ++e.telem_midi_ticks;
      if (e.midi_running) {
        ++midi_ticks;
        if (e.last_tick_valid) {
          const uint64_t d = st.pos - e.last_tick_pos;
          if (d && d < (uint64_t)(4.0f * sr)) {
            const float inst = 60.0f * sr / (24.0f * (float)d);
            if (inst >= 20.0f && inst <= 300.0f) {
              e.midi_bpm_meas = e.midi_bpm_meas
                  ? e.midi_bpm_meas + 0.2f * (inst - e.midi_bpm_meas)
                  : inst;
            }
          }
        }
        e.last_tick_pos = st.pos;
        e.last_tick_valid = 1;
      } else {
        e.last_tick_valid = 0;
      }
    } else if (st.status == MOD_STAMP_START) {
      e.midi_running = 1;
      e.midi_tick_count = 0;
      midi_ticks = 0;
      e.midi_bpm_meas = 0.0f;
      e.last_tick_valid = 0;
      mod_rephase_midi_sources();
    } else if (st.status == MOD_STAMP_CONT) {
      e.midi_running = 1;
    } else {
      e.midi_running = 0;
      e.last_tick_valid = 0;
    }
  }

  // ---- 2) internal clock accumulator (always runs; fractional) ----
  const float int_before = e.int_phase;
  e.int_phase += 24.0f * (e.bpm / 60.0f) * dt;
  const float int_delta = e.int_phase - int_before;
  if (e.int_phase > 65536.0f) e.int_phase -= 65536.0f;

  // ---- 3) sources ----
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    const bool use_midi = (s.clock_sel == MOD_CLK_MIDI) ||
        (s.clock_sel == MOD_CLK_GLOBAL && e.global_clock == 1u);
    const bool clock_running = !use_midi || e.midi_running;
    const float div_ppq = (float)kModDivisions[s.division].ppq;
    uint32_t wraps = 0;

    if (s.type == MOD_LFO && s.p.lfo.hz_mode) {
      s.clk_phase += s.p.lfo.rate_hz * dt;
      if (s.clk_phase >= 1.0f) s.clk_phase -= floorf(s.clk_phase);
      s.sh_acc += dt;
      if (s.sh_acc >= 1.0f) { s.sh_acc -= 1.0f; s.sh_value = mod_rand01(s); }
      float pw = s.clk_phase + s.p.lfo.phase_off;
      if (pw >= 1.0f) pw -= floorf(pw);
      s.out = mod_wave(s.p.lfo.wave, pw, s.sh_value) * s.p.lfo.level;
    } else {
      const float adv = use_midi ? ((float)midi_ticks / div_ppq)
                                 : (int_delta / div_ppq);
      const float prev = s.clk_phase;
      s.clk_phase += adv;
      if (s.clk_phase >= 1.0f) s.clk_phase -= floorf(s.clk_phase);
      wraps = (uint32_t)(prev + adv);

      if (s.type == MOD_LFO) {
        if (use_midi && e.midi_bpm_meas > 0.0f && clock_running) {
          s.clk_phase = prev + dt * (0.4f * e.midi_bpm_meas) / div_ppq;
          if (s.clk_phase >= 1.0f) s.clk_phase -= floorf(s.clk_phase);
        }
        if (s.p.lfo.wave == MOD_WAVE_SH &&
            (wraps || (use_midi && midi_ticks))) {
          s.sh_value = mod_rand01(s);
        }
      } else if (s.type == MOD_EUCLID) {
        if (wraps) {
          s.p.eu.step = (uint8_t)((s.p.eu.step + wraps) % s.p.eu.steps);
          s.p.eu.ticks += wraps;
          s.p.eu.level = (s.p.eu.mask >> s.p.eu.step) & 1u;
        }
        s.out = s.p.eu.level ? 1.0f : 0.0f;
      } else if (s.type == MOD_GATE) {
        if (wraps) {
          s.p.gate.step = (uint8_t)((s.p.gate.step + wraps) % s.p.gate.len);
          s.p.gate.ticks += wraps;
          s.p.gate.level = (s.p.gate.mask >> s.p.gate.step) & 1u;
        }
        const bool high = s.p.gate.level && (s.clk_phase < s.p.gate.gate_len);
        s.out = high ? 1.0f : 0.0f;
      } else if (s.type == MOD_ADSR) {
        bool gate;
        if (s.p.adsr.retrig) {
          if (wraps) s.p.adsr.ticks += wraps;
          gate = clock_running && s.p.adsr.ticks;
        } else {
          gate = shared_gate;
        }
        if (gate) {
          if (s.p.adsr.env_state == MOD_ENV_IDLE ||
              s.p.adsr.env_state == MOD_ENV_R) {
            s.p.adsr.env_state = MOD_ENV_A;
          }
          if (s.p.adsr.env_state == MOD_ENV_A) {
            s.p.adsr.env += dt / (s.p.adsr.a_s > 1e-4f ? s.p.adsr.a_s : 1e-4f);
            if (s.p.adsr.env >= 1.0f) {
              s.p.adsr.env = 1.0f;
              s.p.adsr.env_state = MOD_ENV_D;
            }
          } else if (s.p.adsr.env_state == MOD_ENV_D) {
            s.p.adsr.env -= dt * (1.0f - s.p.adsr.s_lvl) /
                (s.p.adsr.d_s > 1e-4f ? s.p.adsr.d_s : 1e-4f);
            if (s.p.adsr.env <= s.p.adsr.s_lvl) {
              s.p.adsr.env = s.p.adsr.s_lvl;
              s.p.adsr.env_state = MOD_ENV_S;
            }
          } else {
            s.p.adsr.env = s.p.adsr.s_lvl;
          }
        } else if (s.p.adsr.env_state != MOD_ENV_IDLE) {
          s.p.adsr.env_state = MOD_ENV_R;
          s.p.adsr.env -= dt / (s.p.adsr.r_s > 1e-4f ? s.p.adsr.r_s : 1e-4f);
          if (s.p.adsr.env <= 0.0f) {
            s.p.adsr.env = 0.0f;
            s.p.adsr.env_state = MOD_ENV_IDLE;
          }
        }
        s.out = s.p.adsr.env;
      } else {  // MOD_ARP (Switchable 3-Algo Melody & Gate Generator)
        ModArpExt& ext = e.arp_ext[s.p.arp.ext_idx];
        // Capture external MIDI note-ons when in MIDI Follow mode (root_param == 0)
        const uint8_t sn = shared_note;
        if (sn >= 12 && sn <= 108 && sn != s.p.arp.last_out_note) {
          s.p.arp.midi_root = sn;
        }
        if (wraps && clock_running) {
          const uint8_t prev_st = s.p.arp.step;
          s.p.arp.step = (uint8_t)((s.p.arp.step + wraps) & 0x0Fu);
          s.p.arp.ticks += wraps;
          const uint8_t st = s.p.arp.step;

          // Bar-line wrap (step 0): chord drift, mutation, and fill-bar state
          if (st < prev_st || (s.p.arp.ticks & 0x0Fu) == 0u) {
            ext.bar_count++;
            const uint8_t drift_bars = (s.p.arp.groove >= 60) ? 4 : 8;
            if (ext.bar_count >= drift_bars) {
              ext.bar_count = 0;
              if (s.p.arp.groove > 15) mod_arp_next_chord(s);
              ext.dd_fill = 0;
            } else if (ext.bar_count == drift_bars - 1 && s.p.arp.groove >= 50) {
              ext.dd_fill = 1; // Fill bar right before chord/pattern change
            }
            if (s.p.arp.algo == MOD_ARP_HYP1) {
              // HYP-1 ac_mutate: in-place step mutation on loop wrap
              const uint8_t mut_prob = (uint8_t)((uint16_t)s.p.arp.groove * 25 / 100);
              for (uint8_t k = 0; k < 16; ++k) {
                if (mod_irand(s, 100) < mut_prob) mod_arp_roll_hyp1_step(s, ext, k);
              }
            } else {
              // Barlow / Grids: refresh syncopation / chaos variations on the bar
              mod_arp_rebuild_pattern(s);
            }
          }

          // Evaluate step (or rising chord-tone fill run on steps 8..15 of fill bar)
          ModArpStep cur_step = ext.steps[st];
          const bool in_fill = (ext.dd_fill != 0) && (st >= 8) && (s.p.arp.algo == MOD_ARP_HYP1);
          if (in_fill) {
            cur_step.flags = 1u | ((st >= 12) ? 2u : 0u); // gate on, accent on 12..15
            cur_step.off = (int8_t)((st - 8u) * 2u);      // rising chord run 0..14
            cur_step.oct = 0;
          }

          if (cur_step.flags & 1u) {
            const ModArpStep& prev_step = ext.steps[prev_st];
            const bool prev_slid = !in_fill && ((prev_step.flags & 5u) == 5u) && s.p.arp.held;
            const uint8_t next_note = mod_arp_calc_note(s, ext, cur_step);
            s.p.arp.retrig_edge = !(prev_slid && next_note != s.p.arp.cur_note) ? 1u : 0u;
            s.p.arp.cur_note = next_note;
            s.p.arp.cur_acc  = (cur_step.flags & 2u) ? 1u : 0u;
            s.p.arp.cur_hold = (cur_step.flags & 4u) ? 1u : 0u;
            s.p.arp.level    = 1u;
            s.p.arp.held     = 1u;
          } else {
            s.p.arp.level       = 0u;
            s.p.arp.retrig_edge = 0u;
            s.p.arp.held        = 0u;
          }
        }

        // Gate window: 100% if slide/hold bit is set, else 55% (HYP-1 303 gate)
        const float g_win = s.p.arp.cur_hold ? 1.0f : 0.55f;
        const bool gate_open = clock_running && s.p.arp.level && (s.clk_phase < g_win);
        const float target = gate_open ? (s.p.arp.cur_acc ? 1.0f : 0.72f) : 0.0f;
        // Click-free asymmetric slew (~2 ms attack, ~12 ms release) for continuous CCs
        const float slew = (target > s.out) ? (dt / 0.002f) : (dt / 0.012f);
        s.out += (target - s.out) * (slew > 1.0f ? 1.0f : slew);
      }

      if (s.type == MOD_LFO) {
        float pw = s.clk_phase + s.p.lfo.phase_off;
        if (pw >= 1.0f) pw -= floorf(pw);
        s.out = mod_wave(s.p.lfo.wave, pw, s.sh_value) * s.p.lfo.level;
      }
    }

    // ---- 4) destination routing (resolved against the ACTIVE module) ----
    const uint8_t dest = s.dest_cc;
    if (dest == 0) { s.was_high = 0; continue; }

    if (s.type == MOD_LFO || s.type == MOD_ADSR) {
      mod_write_cc(active_module, dest, s.out);
    } else if (s.type == MOD_ARP) {
      // MOD_ARP can drive the Voice (dest == 127 or module trigger CC 64/65/66)
      // OR modulate any Continuous CC on the active module as a slewed trance gate!
      const int8_t trig_kind = mod_trigger_kind(active_module, dest);
      if (trig_kind >= 0) {
        const float g_win = s.p.arp.cur_hold ? 1.0f : 0.55f;
        const bool high = clock_running && s.p.arp.level && (s.clk_phase < g_win);
        if (high) {
          s.p.arp.last_out_note = s.p.arp.cur_note;
          shared_note = s.p.arp.cur_note;
          shared_gate = 1;
        } else if (s.was_high) {
          shared_gate = 0;
        }
        if (dest != MOD_DEST_VOICE) {
          if (trig_kind == RACK_TRIG_LATCH) {
            if (high != (bool)s.was_high) mod_fire_trigger(active_module, dest, high);
          } else if (wraps && s.p.arp.level && s.p.arp.retrig_edge) {
            // Fire pulse on every new non-slid gated step (legato slides skip retrigger)
            mod_fire_trigger(active_module, dest, true);
          }
        }
        s.was_high = high ? 1u : 0u;
      } else {
        // Routed to a continuous CC: write slewed trance-gate / accent envelope
        mod_write_cc(active_module, dest, s.out);
      }
    } else {
      const bool high = (s.type == MOD_EUCLID) ? (s.p.eu.level != 0)
                                               : (s.out > 0.5f);
      const int8_t kind = mod_trigger_kind(active_module, dest);
      if (kind < 0) { s.was_high = 0; continue; }
      if (kind == RACK_TRIG_LATCH) {
        if (high != (bool)s.was_high) {
          mod_fire_trigger(active_module, dest, high);
          s.was_high = high ? 1u : 0u;
        }
      } else {
        if (s.type == MOD_EUCLID) {
          if (wraps && s.p.eu.level) mod_fire_trigger(active_module, dest, true);
        } else if (high && !s.was_high) {
          mod_fire_trigger(active_module, dest, true);
        }
        s.was_high = high ? 1u : 0u;
      }
    }
  }

  e.sample_counter += bs;

  const uint32_t dt_us = time_us_32() - t0;
  if (dt_us > e.mod_us_max) e.mod_us_max = dt_us;
}

// ===================== core 0: stamping funnel =====================
static uint64_t mod_read_sample_counter() {
  const volatile uint64_t* p = &g_mod_engine.sample_counter;
  for (;;) {
    const uint32_t hi1 = (uint32_t)(*p >> 32);
    const uint32_t lo = (uint32_t)*p;
    const uint32_t hi2 = (uint32_t)(*p >> 32);
    if (hi1 == hi2) return ((uint64_t)hi1 << 32) | lo;
  }
}

void rack_mod_clock_realtime(uint8_t status) {
  if (status != MOD_STAMP_TICK && status != MOD_STAMP_START &&
      status != MOD_STAMP_CONT && status != MOD_STAMP_STOP) {
    return;
  }
  ModEngine& e = g_mod_engine;
  uint64_t pos = mod_read_sample_counter();
  const uint32_t period = e.block_period_us;
  if (period) {
    const uint32_t bs = e.block_size_now;
    uint32_t elapsed = time_us_32() - e.block_start_us;
    if (elapsed >= period) elapsed = period - 1u;
    pos += ((uint64_t)elapsed * bs) / period;
  }
  const uint32_t head = e.stamp_head;
  const uint32_t next = (head + 1u) & (kModStampRingSize - 1u);
  if (next == e.stamp_tail) { e.stamp_overflow++; return; }
  e.ring[head].pos = pos;
  e.ring[head].status = status;
  e.stamp_head = next;
}

// ===================== core 0: bench CC 33/34/35 mux =====================
static uint8_t mod_param_count_of(uint8_t type) {
  return kModParamCount[type < MOD_TYPE_COUNT ? type : 0];
}

static const char* mod_src_name(uint8_t idx) {
  static const char* const kNames[MOD_SOURCE_COUNT] = {
      "lfo1", "lfo2", "lfo3", "lfo4",
      "eu1", "eu2",
      "gate1", "gate2",
      "adsr1", "adsr2",
      "arp1", "arp2",
  };
  return kNames[idx < MOD_SOURCE_COUNT ? idx : 0];
}

static void mod_set_dest(uint8_t src_idx, uint8_t cc) {
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  s.dest_cc = cc;
  s.was_high = 0;
  if (cc == 0) {
    Serial.printf("mod: %s destination cleared (panel regains the parameter)\r\n",
                  mod_src_name(src_idx));
    return;
  }
  const RackModuleInfo& m = kRackModules[g_active_module];
  const RackModDestinations* d = m.mod_destinations;
  bool continuous_ok = false, trigger_ok = (cc == MOD_DEST_VOICE && s.type == MOD_ARP);
  for (uint8_t i = 0; i < d->continuous_count; ++i) {
    if (d->continuous[i] == cc) { continuous_ok = true; break; }
  }
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == cc) { trigger_ok = true; break; }
  }
  if (s.type == MOD_ARP && (trigger_ok || continuous_ok)) {
    Serial.printf("mod: %s -> %s CC %u (%s)\r\n", mod_src_name(src_idx), m.name,
                  cc, trigger_ok ? "voice pitch+gate+trig" : "slewed gate CV");
    return;
  }
  const bool want_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR);
  if (want_continuous && continuous_ok) {
    Serial.printf("mod: %s -> %s CC %u (continuous): the panel's static value "
                  "is overridden while routed\r\n",
                  mod_src_name(src_idx), m.name, cc);
  } else if (!want_continuous && trigger_ok) {
    Serial.printf("mod: %s -> %s CC %u (trigger)\r\n",
                  mod_src_name(src_idx), m.name, cc);
  } else {
    Serial.printf("mod: WARNING: %s -> CC %u is INERT on %s (%s source wants "
                  "%s; see 'module')\r\n", mod_src_name(src_idx), cc, m.name,
                  want_continuous ? "continuous" : "trigger",
                  want_continuous ? "a continuous CC" : "a trigger input");
  }
}

static inline uint8_t mod_clk_from_cc(uint8_t v) {
  return v < 43 ? MOD_CLK_GLOBAL : (v < 85 ? MOD_CLK_INTERNAL : MOD_CLK_MIDI);
}

static inline float mod_adsr_s_from_cc(uint8_t v) {
  return powf(10.0f, -3.0f + 4.0f * (float)v / 127.0f);
}

static void mod_apply_cc_value(uint8_t src_idx, uint8_t param, uint8_t val) {
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  const uint8_t n = mod_param_count_of(s.type);
  if (param >= n) return;
  const uint8_t tail = n - 3;
  if (param == tail) {
    s.clock_sel = mod_clk_from_cc(val);
    return;
  }
  if (param == tail + 1) {
    const uint8_t d = (uint16_t)val * MOD_DIVISION_COUNT / 128;
    s.division = d < MOD_DIVISION_COUNT ? d : MOD_DIVISION_COUNT - 1;
    return;
  }
  if (param == tail + 2) { mod_set_dest(src_idx, val); return; }
  switch (s.type) {
    case MOD_LFO:
      switch (param) {
        case 0: s.p.lfo.hz_mode = val < 64 ? 1 : 0; break;
        case 1: s.p.lfo.rate_hz = 0.02f * powf(1000.0f, val / 127.0f); break;
        case 2: {
          const uint8_t w = (uint16_t)val * MOD_WAVE_SH_COUNT / 128;
          s.p.lfo.wave = w < MOD_WAVE_SH_COUNT ? w : MOD_WAVE_SH_COUNT - 1;
          break;
        }
        case 3: s.p.lfo.phase_off = (float)val / 127.0f; break;
        case 4: s.p.lfo.level = val / 127.0f; break;
        default: break;
      }
      break;
    case MOD_EUCLID:
      switch (param) {
        case 0:
          s.p.eu.steps = (uint8_t)(1 + (uint16_t)val * 31 / 127);
          mod_eu_recompute(s);
          break;
        case 1:
          s.p.eu.pulses = (uint8_t)((uint16_t)val * 32 / 127);
          mod_eu_recompute(s);
          break;
        case 2:
          s.p.eu.rot = (uint8_t)((uint16_t)val * 31 / 127);
          mod_eu_recompute(s);
          break;
        default: break;
      }
      break;
    case MOD_GATE:
      switch (param) {
        case 0: {
          s.p.gate.len = (uint8_t)(1 + (uint16_t)val * 31 / 127);
          const uint32_t keep = (s.p.gate.len >= 32)
              ? 0xFFFFFFFFu : ((1u << s.p.gate.len) - 1u);
          s.p.gate.mask &= keep;
          break;
        }
        case 1: {
          const uint8_t k = (uint8_t)(((uint16_t)val * s.p.gate.len) / 127);
          s.p.gate.mask = mod_euclid_mask(k, s.p.gate.len);
          break;
        }
        case 2: s.p.gate.gate_len = (1.0f + (float)val) / 128.0f; break;
        default: break;
      }
      break;
    case MOD_ADSR:
      switch (param) {
        case 0: s.p.adsr.retrig = val < 64 ? 0 : 1; break;
        case 1: s.p.adsr.a_s = mod_adsr_s_from_cc(val); break;
        case 2: s.p.adsr.d_s = mod_adsr_s_from_cc(val); break;
        case 3: s.p.adsr.s_lvl = val / 127.0f; break;
        case 4: s.p.adsr.r_s = mod_adsr_s_from_cc(val); break;
        default: break;
      }
      break;
    default:  // MOD_ARP
      switch (param) {
        case 0: // algo (0..42=HYP1, 43..85=BARLOW, 86..127=GRIDS; sub-range sets 1..3 oct)
          s.p.arp.algo = (val < 43) ? MOD_ARP_HYP1
                       : (val < 86) ? MOD_ARP_BARLOW : MOD_ARP_GRIDS;
          s.p.arp.range = (uint8_t)(1 + ((val % 43) * 3) / 43);
          mod_arp_rebuild_pattern(s);
          break;
        case 1: // root (0 = MIDI Follow, 1..127 -> MIDI note 24..84)
          s.p.arp.root_param = (val == 0) ? 0 : (uint8_t)(24 + ((uint16_t)(val - 1) * 60) / 126);
          break;
        case 2: // scale (0..127 -> 0..15 scale+chord pool)
          s.p.arp.scale_idx = (uint8_t)(((uint16_t)val * 16) / 128);
          mod_arp_rebuild_pattern(s);
          break;
        case 3: // dens (0..127 -> 0..100%)
          s.p.arp.density = (uint8_t)(((uint16_t)val * 100) / 127);
          mod_arp_rebuild_pattern(s);
          break;
        case 4: // groove (0..127 -> 0..100%)
          s.p.arp.groove = (uint8_t)(((uint16_t)val * 100) / 127);
          mod_arp_rebuild_pattern(s);
          break;
        default: break;
      }
      break;
  }
}

void rack_mod_bench_cc(uint8_t cc, uint8_t val) {
  static uint8_t sel_source = 0;
  static uint8_t sel_param = 0;
  if (cc == kModBenchCcSource) {
    if (val < MOD_SOURCE_COUNT) sel_source = val;
    return;
  }
  if (cc == kModBenchCcParam) {
    sel_param = val;
    return;
  }
  mod_apply_cc_value(sel_source, sel_param, val);
}

// ===================== T5 stage 1: UI-link MOD_SET entry =====================
void mod_engine_apply_ui_set(uint8_t src, uint8_t param, uint8_t val) {
  if (src >= MOD_SOURCE_COUNT) {
    Serial.printf("mod: ui set REJECTED (source %u >= %u)\r\n", src,
                  MOD_SOURCE_COUNT);
    return;
  }
  if (param >= mod_param_count_of(g_mod_engine.src[src].type)) {
    Serial.printf("mod: ui set REJECTED (%s has no param %u; see 'mod?')\r\n",
                  mod_src_name(src), param);
    return;
  }
  Serial.printf("mod: ui set %s param %u <- %u\r\n", mod_src_name(src), param,
                val);
  mod_apply_cc_value(src, param, val);
}

// ===================== core 0: serial console =====================
static bool mod_next_token(const char*& p, char* out, size_t n) {
  while (*p == ' ') ++p;
  if (*p == '\0') return false;
  size_t i = 0;
  while (*p && *p != ' ') {
    if (i + 1 < n) out[i++] = *p;
    ++p;
  }
  out[i] = '\0';
  return true;
}

static int8_t mod_parse_source(const char* tok) {
  if (tok[0] >= '0' && tok[0] <= '9') {
    const long v = atol(tok);
    if (v >= 0 && v < MOD_SOURCE_COUNT) return (int8_t)v;
    return -1;
  }
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    if (strcasecmp(tok, mod_src_name(i)) == 0) return (int8_t)i;
  }
  return -1;
}

static int8_t mod_parse_div(const char* tok) {
  for (uint8_t i = 0; i < MOD_DIVISION_COUNT; ++i) {
    if (strcasecmp(tok, kModDivisions[i].name) == 0) return (int8_t)i;
  }
  if (tok[0] >= '0' && tok[0] <= '9') {
    const long v = atol(tok);
    if (v >= 0 && v < MOD_DIVISION_COUNT) return (int8_t)v;
  }
  return -1;
}

static int8_t mod_parse_wave(const char* tok) {
  static const char* const kNames[MOD_WAVE_SH_COUNT] = {
      "sine", "tri", "saw", "square", "sh",
  };
  for (uint8_t i = 0; i < MOD_WAVE_SH_COUNT; ++i) {
    if (strcasecmp(tok, kNames[i]) == 0) return (int8_t)i;
  }
  if (strcasecmp(tok, "s&h") == 0) return MOD_WAVE_SH;
  if (strcasecmp(tok, "sq") == 0) return MOD_WAVE_SQR;
  if (tok[0] >= '0' && tok[0] <= '9') {
    const long v = atol(tok);
    if (v >= 0 && v < MOD_WAVE_SH_COUNT) return (int8_t)v;
  }
  return -1;
}

static int8_t mod_parse_clk(const char* tok) {
  if (!strcasecmp(tok, "global") || !strcasecmp(tok, "g")) return MOD_CLK_GLOBAL;
  if (!strcasecmp(tok, "internal") || !strcasecmp(tok, "int") ||
      !strcasecmp(tok, "i")) return MOD_CLK_INTERNAL;
  if (!strcasecmp(tok, "midi") || !strcasecmp(tok, "m")) return MOD_CLK_MIDI;
  if (tok[0] >= '0' && tok[0] <= '9') {
    const long v = atol(tok);
    if (v >= 0 && v <= 2) return (int8_t)v;
  }
  return -1;
}

static bool mod_set_param_console(uint8_t src_idx, uint8_t param,
                                  const char* tok) {
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  const uint8_t n = mod_param_count_of(s.type);
  if (param >= n) return false;
  const uint8_t tail = n - 3;
  if (param == tail) {
    const int8_t c = mod_parse_clk(tok);
    if (c < 0) return false;
    s.clock_sel = (uint8_t)c;
    return true;
  }
  if (param == tail + 1) {
    const int8_t d = mod_parse_div(tok);
    if (d < 0) return false;
    s.division = (uint8_t)d;
    return true;
  }
  if (param == tail + 2) {
    if (!strcasecmp(tok, "off")) { mod_set_dest(src_idx, 0); return true; }
    if (!strcasecmp(tok, "voice")) { mod_set_dest(src_idx, MOD_DEST_VOICE); return true; }
    if (tok[0] >= '0' && tok[0] <= '9') {
      const long v = atol(tok);
      if (v >= 0 && v <= 127) { mod_set_dest(src_idx, (uint8_t)v); return true; }
    }
    return false;
  }
  const float v = (float)atof(tok);
  switch (s.type) {
    case MOD_LFO:
      switch (param) {
        case 0:
          if (!strcasecmp(tok, "hz") || !strcasecmp(tok, "free")) {
            s.p.lfo.hz_mode = 1; return true;
          }
          if (!strcasecmp(tok, "clock") || !strcasecmp(tok, "clocked") ||
              !strcasecmp(tok, "clk")) {
            s.p.lfo.hz_mode = 0; return true;
          }
          s.p.lfo.hz_mode = v < 0.5f ? 1 : 0;
          return true;
        case 1:
          s.p.lfo.rate_hz = v;
          if (s.p.lfo.rate_hz < 0.01f) s.p.lfo.rate_hz = 0.01f;
          if (s.p.lfo.rate_hz > 50.0f) s.p.lfo.rate_hz = 50.0f;
          return true;
        case 2: {
          const int8_t w = mod_parse_wave(tok);
          if (w < 0) return false;
          s.p.lfo.wave = (uint8_t)w;
          return true;
        }
        case 3:
          s.p.lfo.phase_off = mod_clamp01(v / 360.0f);
          return true;
        case 4:
          s.p.lfo.level = mod_clamp01(v);
          return true;
        default: return false;
      }
    case MOD_EUCLID:
      switch (param) {
        case 0: s.p.eu.steps = (uint8_t)(v < 1 ? 1 : (v > 32 ? 32 : v));
                mod_eu_recompute(s); return true;
        case 1: s.p.eu.pulses = (uint8_t)(v < 0 ? 0 : (v > 32 ? 32 : v));
                mod_eu_recompute(s); return true;
        case 2: s.p.eu.rot = (uint8_t)(v < 0 ? 0 : (v > 31 ? 31 : v));
                mod_eu_recompute(s); return true;
        default: return false;
      }
    case MOD_GATE:
      switch (param) {
        case 0: {
          uint8_t len = (uint8_t)(v < 1 ? 1 : (v > 32 ? 32 : v));
          s.p.gate.len = len;
          const uint32_t keep = (len >= 32) ? 0xFFFFFFFFu : ((1u << len) - 1u);
          s.p.gate.mask &= keep;
          return true;
        }
        case 1:
          s.p.gate.mask = (uint32_t)strtoul(tok, nullptr, 0);
          return true;
        case 2:
          s.p.gate.gate_len = mod_clamp01(v);
          return true;
        default: return false;
      }
    case MOD_ADSR:
      switch (param) {
        case 0:
          if (!strcasecmp(tok, "gate") || !strcasecmp(tok, "follow")) {
            s.p.adsr.retrig = 0; return true;
          }
          if (!strcasecmp(tok, "retrig") || !strcasecmp(tok, "clock")) {
            s.p.adsr.retrig = 1; return true;
          }
          s.p.adsr.retrig = v >= 0.5f ? 1 : 0;
          return true;
        case 1: s.p.adsr.a_s = v / 1000.0f;
                if (s.p.adsr.a_s < 0.0005f) s.p.adsr.a_s = 0.0005f;
                if (s.p.adsr.a_s > 10.0f) s.p.adsr.a_s = 10.0f;
                return true;
        case 2: s.p.adsr.d_s = v / 1000.0f;
                if (s.p.adsr.d_s < 0.0005f) s.p.adsr.d_s = 0.0005f;
                if (s.p.adsr.d_s > 10.0f) s.p.adsr.d_s = 10.0f;
                return true;
        case 3: s.p.adsr.s_lvl = mod_clamp01(v); return true;
        case 4: s.p.adsr.r_s = v / 1000.0f;
                if (s.p.adsr.r_s < 0.0005f) s.p.adsr.r_s = 0.0005f;
                if (s.p.adsr.r_s > 10.0f) s.p.adsr.r_s = 10.0f;
                return true;
        default: return false;
      }
    default:  // MOD_ARP
      switch (param) {
        case 0: // algo
          if (!strcasecmp(tok, "hyp1") || !strcasecmp(tok, "acid")) s.p.arp.algo = MOD_ARP_HYP1;
          else if (!strcasecmp(tok, "barlow") || !strcasecmp(tok, "sync")) s.p.arp.algo = MOD_ARP_BARLOW;
          else if (!strcasecmp(tok, "grids") || !strcasecmp(tok, "trance")) s.p.arp.algo = MOD_ARP_GRIDS;
          else s.p.arp.algo = (uint8_t)(v < 0 ? 0 : (v > 2 ? 2 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        case 1: // root
          if (!strcasecmp(tok, "midi") || !strcasecmp(tok, "follow") || !strcasecmp(tok, "auto")) {
            s.p.arp.root_param = 0; return true;
          }
          s.p.arp.root_param = (uint8_t)(v <= 0 ? 0 : (v < 24 ? 24 : (v > 84 ? 84 : v)));
          return true;
        case 2: { // scale
          static const char* const kScNames[8] = {
              "ionian", "dorian", "phrygian", "lydian", "mixo", "aeolian", "pentmaj", "pentmin"
          };
          for (uint8_t i = 0; i < 8; ++i) {
            if (!strcasecmp(tok, kScNames[i])) {
              s.p.arp.scale_idx = i; mod_arp_rebuild_pattern(s); return true;
            }
          }
          s.p.arp.scale_idx = (uint8_t)(v < 0 ? 0 : (v > 15 ? 15 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        }
        case 3: // dens
          s.p.arp.density = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        case 4: // groove
          s.p.arp.groove = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        default: return false;
      }
  }
}

static int8_t mod_parse_param(uint8_t src_idx, const char* tok) {
  ModEngine& e = g_mod_engine;
  const ModSource& s = e.src[src_idx];
  const uint8_t n = mod_param_count_of(s.type);
  for (uint8_t i = 0; i < n; ++i) {
    if (strcasecmp(tok, kModParamNames[s.type][i]) == 0) return (int8_t)i;
  }
  if (tok[0] >= '0' && tok[0] <= '9') {
    const long v = atol(tok);
    if (v >= 0 && v < n) return (int8_t)v;
  }
  return -1;
}

void mod_engine_cmd_mod(const char* args) {
  const char* p = args;
  char tok[32];
  if (!mod_next_token(p, tok, sizeof(tok))) {
    Serial.println("usage: mod <0-11|lfo1-4|eu1-2|gate1-2|adsr1-2|arp1-2> <param> <value>");
    return;
  }
  const int8_t src = mod_parse_source(tok);
  if (src < 0) {
    Serial.println("mod: unknown source (0-11 or lfo1-4 / eu1-2 / gate1-2 / adsr1-2 / arp1-2)");
    return;
  }
  if (!mod_next_token(p, tok, sizeof(tok))) {
    Serial.println("mod: missing param (mod? lists every param per source)");
    return;
  }
  const int8_t param = mod_parse_param((uint8_t)src, tok);
  if (param < 0) {
    Serial.printf("mod: unknown param for this source (try: %s..)\r\n",
                  kModParamNames[g_mod_engine.src[src].type][0]);
    return;
  }
  if (!mod_next_token(p, tok, sizeof(tok))) {
    Serial.println("mod: missing value");
    return;
  }
  if (!mod_set_param_console((uint8_t)src, (uint8_t)param, tok)) {
    Serial.printf("mod: bad value '%s' for %s\r\n", tok,
                  kModParamNames[g_mod_engine.src[src].type][param]);
  }
}

static void mod_print_dest_state(uint8_t src_idx) {
  ModEngine& e = g_mod_engine;
  const ModSource& s = e.src[src_idx];
  if (s.dest_cc == 0) {
    Serial.print("dest=0(off)");
    return;
  }
  const RackModuleInfo& m = kRackModules[g_active_module];
  if (s.type == MOD_ARP && s.dest_cc == MOD_DEST_VOICE) {
    Serial.printf("dest=127(voice on %s)", m.name);
    return;
  }
  const RackModDestinations* d = m.mod_destinations;
  const bool want_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR);
  bool cont = false, trig = false;
  for (uint8_t i = 0; i < d->continuous_count; ++i) {
    if (d->continuous[i] == s.dest_cc) { cont = true; break; }
  }
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == s.dest_cc) { trig = true; break; }
  }
  char tag[12];
  if (s.type == MOD_ARP && trig) snprintf(tag, sizeof(tag), "voice+trig");
  else if (s.type == MOD_ARP && cont) snprintf(tag, sizeof(tag), "gate-cv");
  else if (want_continuous && cont) snprintf(tag, sizeof(tag), "cont");
  else if (!want_continuous && trig) snprintf(tag, sizeof(tag), "trig");
  else snprintf(tag, sizeof(tag), "INERT!");
  Serial.printf("dest=%u(%s on %s)", s.dest_cc, tag, m.name);
}

static const char* mod_print_fixed(char* buf, size_t n, float v) {
  if (v < 0.0f) v = 0.0f;
  const uint32_t scaled = (uint32_t)(v * 100.0f + 0.5f);
  snprintf(buf, n, "%lu.%02lu", (unsigned long)(scaled / 100u),
           (unsigned long)(scaled % 100u));
  return buf;
}

void mod_engine_cmd_modq(const char* args) {
  (void)args;
  ModEngine& e = g_mod_engine;
  char b1[16], b2[16], b3[16], b4[16], b5[16];
  const uint32_t bpm10 = (uint32_t)(e.bpm * 10.0f + 0.5f);
  Serial.printf("mod: clock global=%s bpm=%lu.%lu | midi running=%u measured=%s bpm ticks=%lu stamps_dropped=%lu\r\n",
                e.global_clock ? "midi" : "internal",
                (unsigned long)(bpm10 / 10), (unsigned long)(bpm10 % 10),
                (unsigned)e.midi_running,
                mod_print_fixed(b1, sizeof(b1), e.midi_bpm_meas),
                (unsigned long)e.midi_tick_count,
                (unsigned long)e.stamp_overflow);
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    const ModSource& s = e.src[i];
    static const char* const kClk[3] = {"global", "internal", "midi"};
    Serial.printf("mod: %2u %-5s clk=%-8s div=%-5s ", i,
                  s.type == MOD_LFO ? "LFO" : s.type == MOD_EUCLID ? "EU"
                  : s.type == MOD_GATE ? "GATE" : s.type == MOD_ADSR ? "ADSR" : "ARP",
                  kClk[s.clock_sel], kModDivisions[s.division].name);
    switch (s.type) {
      case MOD_LFO: {
        static const char* const kWave[MOD_WAVE_SH_COUNT] = {
            "sine", "tri", "saw", "square", "sh"};
        Serial.printf("mode=%-7s rate=%sHz wave=%-6s phase=%lu level=%s out=%s",
                      s.p.lfo.hz_mode ? "hz" : "clocked",
                      mod_print_fixed(b1, sizeof(b1), s.p.lfo.rate_hz),
                      kWave[s.p.lfo.wave],
                      (unsigned long)(uint32_t)(s.p.lfo.phase_off * 359.0f + 0.5f),
                      mod_print_fixed(b2, sizeof(b2), s.p.lfo.level),
                      mod_print_fixed(b3, sizeof(b3), s.out));
        break;
      }
      case MOD_EUCLID:
        Serial.printf("steps=%u pulses=%u rot=%u mask=0x%08lx step=%u ticks=%lu",
                      s.p.eu.steps, s.p.eu.pulses, s.p.eu.rot,
                      (unsigned long)s.p.eu.mask, s.p.eu.step,
                      (unsigned long)s.p.eu.ticks);
        break;
      case MOD_GATE:
        Serial.printf("len=%u mask=0x%08lx glen=%s step=%u ticks=%lu out=%s",
                      s.p.gate.len, (unsigned long)s.p.gate.mask,
                      mod_print_fixed(b1, sizeof(b1), s.p.gate.gate_len),
                      s.p.gate.step, (unsigned long)s.p.gate.ticks,
                      mod_print_fixed(b2, sizeof(b2), s.out));
        break;
      case MOD_ADSR:
        Serial.printf("mode=%s atk=%sms dec=%sms sus=%s rel=%sms out=%s",
                      s.p.adsr.retrig ? "retrig" : "gate",
                      mod_print_fixed(b1, sizeof(b1), s.p.adsr.a_s * 1000.0f),
                      mod_print_fixed(b2, sizeof(b2), s.p.adsr.d_s * 1000.0f),
                      mod_print_fixed(b3, sizeof(b3), s.p.adsr.s_lvl),
                      mod_print_fixed(b4, sizeof(b4), s.p.adsr.r_s * 1000.0f),
                      mod_print_fixed(b5, sizeof(b5), s.out));
        break;
      default: { // MOD_ARP
        static const char* const kAlgo[3] = {"hyp1", "barlow", "grids"};
        static const char* const kSc[8] = {
            "ionian", "dorian", "phryg", "lydian", "mixo", "aeolian", "pentmaj", "pentmin"};
        const ModArpExt& ext = e.arp_ext[s.p.arp.ext_idx];
        Serial.printf("algo=%s(%uoct) root=%s(%u) scale=%s dens=%u%% groove=%u%% deg=%d note=%u step=%u out=%s",
                      kAlgo[s.p.arp.algo % 3], s.p.arp.range,
                      s.p.arp.root_param == 0 ? "midi" : "fix",
                      s.p.arp.root_param == 0 ? s.p.arp.midi_root : s.p.arp.root_param,
                      kSc[s.p.arp.scale_idx & 7u],
                      s.p.arp.density, s.p.arp.groove, (int)ext.deg,
                      s.p.arp.cur_note, s.p.arp.step,
                      mod_print_fixed(b1, sizeof(b1), s.out));
        break;
      }
    }
    Serial.print(' ');
    mod_print_dest_state(i);
    Serial.print("\r\n");
  }
}

void mod_engine_cmd_bpm(const char* args) {
  const char* p = args;
  char tok[16];
  if (!mod_next_token(p, tok, sizeof(tok))) {
    Serial.println("usage: bpm <20-300> (0.1 steps accepted)");
    return;
  }
  const float v = (float)atof(tok);
  if (v < 20.0f || v > 300.0f) {
    Serial.println("mod: bpm out of range (20-300)");
    return;
  }
  g_mod_engine.bpm = roundf(v * 10.0f) / 10.0f;
  Serial.println("mod: bpm set (internal phase kept; no re-phase)");
}

void mod_engine_cmd_clock(const char* args) {
  const char* p = args;
  char tok[24];
  if (!mod_next_token(p, tok, sizeof(tok))) {
    Serial.println("usage: clock <internal|midi> [source]  (source = 0-11 or name; "
                   "clear overrides with 'mod <src> clk global')");
    return;
  }
  int8_t g = -1;
  if (!strcasecmp(tok, "internal") || !strcasecmp(tok, "i")) g = 0;
  else if (!strcasecmp(tok, "midi") || !strcasecmp(tok, "m")) g = 1;
  if (g < 0) {
    Serial.println("mod: clock source must be internal|midi");
    return;
  }
  if (!mod_next_token(p, tok, sizeof(tok))) {
    g_mod_engine.global_clock = (uint8_t)g;
    Serial.printf("mod: GLOBAL clock = %s (per-source overrides unchanged)\r\n",
                  g ? "midi" : "internal");
    return;
  }
  const int8_t src = mod_parse_source(tok);
  if (src < 0) {
    Serial.println("mod: unknown source for per-source override");
    return;
  }
  g_mod_engine.src[src].clock_sel = g ? MOD_CLK_MIDI : MOD_CLK_INTERNAL;
  Serial.printf("mod: source %s clock override = %s\r\n",
                mod_src_name((uint8_t)src), g ? "midi" : "internal");
}

void mod_engine_report_tail(char* buf, size_t n) {
  ModEngine& e = g_mod_engine;
  char states[MOD_SOURCE_COUNT + 1];
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    const ModSource& s = e.src[i];
    const bool use_midi = (s.clock_sel == MOD_CLK_MIDI) ||
        (s.clock_sel == MOD_CLK_GLOBAL && e.global_clock == 1u);
    const bool running = !use_midi || e.midi_running;
    char c = '.';
    switch (s.type) {
      case MOD_LFO: c = s.p.lfo.hz_mode ? 'h' : (running ? 'L' : '.'); break;
      case MOD_EUCLID: c = running ? 'e' : '.'; break;
      case MOD_GATE: c = running ? 'g' : '.'; break;
      case MOD_ADSR: c = (s.p.adsr.env > 0.01f) ? 'A' : 'a'; break;
      default: c = running ? 'M' : '.'; break; // MOD_ARP
    }
    states[i] = c;
  }
  states[MOD_SOURCE_COUNT] = '\0';
  const uint8_t phase = (uint8_t)(mod_clamp01(e.src[0].clk_phase) * 255.0f);
  if (e.global_clock == 0) {
    const uint32_t bpm10 = (uint32_t)(e.bpm * 10.0f + 0.5f);
    snprintf(buf, n, "modeng=i:%lu.%lu e:%lu p:%02x u:%lu [%s]",
             (unsigned long)(bpm10 / 10), (unsigned long)(bpm10 % 10),
             (unsigned long)e.src[MOD_SRC_EU_FIRST].p.eu.ticks,
             (unsigned)phase, (unsigned long)e.mod_us_max, states);
  } else if (e.midi_running) {
    snprintf(buf, n, "modeng=m:%lu e:%lu p:%02x u:%lu [%s]",
             (unsigned long)(e.telem_midi_ticks / 2u),
             (unsigned long)e.src[MOD_SRC_EU_FIRST].p.eu.ticks,
             (unsigned)phase, (unsigned long)e.mod_us_max, states);
  } else {
    snprintf(buf, n, "modeng=m:x e:%lu p:%02x u:%lu [%s]",
             (unsigned long)e.src[MOD_SRC_EU_FIRST].p.eu.ticks,
             (unsigned)phase, (unsigned long)e.mod_us_max, states);
  }
}