// Rack RP2350 — T3 persistent modulation engine + clock bus.
//
// ONE plain-.bss object (g_mod_engine, DEFINED in main.cpp — OUTSIDE the T2
// arenas, never touched by the swap path) holding:
//   - the CLOCK BUS: a 24-PPQ phase accumulator fed by the INTERNAL clock
//     (BPM 20-300, 0.1 steps) or by EXTERNAL MIDI clock; every source selects
//     its effective clock (global / internal override / midi override) and a
//     clock division (1/1..1/32 + triplets);
//   - 12 persistent SOURCES: 4x LFO (free-Hz or clocked-division; sine/tri/
//     saw/square/S&H; phase offset; level), 4x Euclidean trigger (steps
//     1-32, pulses, rotation -> uint32 mask), 2x rhythmic gate (length 1-32,
//     uint32 mask, gate length per step), 2x ADSR (gate-follow from the
//     shared note stack OR clock-retrigger; 1 ms..10 s A/D/R);
//   - the DESTINATION ROUTER: per source one destination CC, resolved against
//     the ACTIVE module's registry entry (kRackModules[].mod_destinations)
//     every block — continuous sources write the module's shared-float/patch
//     machinery via the adapter's mod_set_cc_float hook, trigger sources fire
//     the adapter's trigger map via mod_trigger_cc (plaits trig = pseudo-CC
//     64, rings strum = pseudo-CC 64, clouds freeze = 65 (latch) / trig = 66
//     (pulse); elements has no trigger inputs).
//
// CORE SPLIT + SWAP DISCIPLINE (the load-bearing invariants):
//   - The engine's per-block pass runs on CORE 1 at loop-top, BEFORE the
//     active module's render, at the module's block size (loop1 in main.cpp).
//     It is skipped on swap calls exactly like render (loop1 returns before
//     both on a pending swap) — the engine never touches module state while
//     it aliases a fresh arena placement. Its phase/state survive swaps
//     untouched (nothing in do_module_swap references the engine).
//   - CORE 0 is the only writer of incoming clock stamps: the MIDI funnel
//     (midi.h -> rack_mod_clock_realtime, one entry for the USB device path,
//     the PIO-USB host path and T4's future DIN path) stamps each F8/FA/FB/FC
//     with the current render sample position and pushes it into a 64-deep
//     SPSC ring (core 0 = head, core 1 = tail). Core 1 drains the ring at the
//     next block top.
//
// CLOCK PRECISION STORY (bench-verifiable):
//   - sample_counter counts the FIRST sample index of the block currently
//     rendering; core 1 publishes block_start_us (time_us_32 at the engine
//     top) and the block period, then increments the counter at the END of
//     the pass — so during a whole block core 0 reads a CONSISTENT
//     (counter, start, period) triple and stamps pos = counter +
//     (us since block start) / period * block_size.
//   - Stamp precision is therefore the funnel's own latency (core 0 loop
//     iteration: USB tasks + console + LED poll; typically well under 1 ms,
//     worst ~1-2 ms during LED/serial activity) — the systematic error of
//     "which block did the tick land in" is gone; what remains is the
//     consumer granularity: division/step events are applied at the first
//     block boundary after their stamped position (0..1 block, 0.75-1 ms at
//     the rack's rates — far below the 1/32-note = ~26 ms division grid the
//     sources run on). The stamped positions themselves are kept on the
//     timeline (tick-to-tick deltas feed the measured-tempo EMA), so a future
//     sample-accurate consumer needs no protocol change.
//   - Transport: FA resets the MIDI tick count and re-phases every MIDI-
//     clocked source (clocked LFOs re-phase to 0, euclid/gate steps to 0,
//     retrigger ADSRs clear); FB continues without reset; FC freezes (no
//     ticks -> all MIDI-clocked sources hold their phase). Internal-clocked
//     sources are unaffected by the MIDI transport. Euclid tick ODOMETERS are
//     never reset (swap/phase-survival telemetry).
//   - MIDI-clocked clocked-LFOs ramp at the MEASURED incoming tempo (EMA of
//     the sample-accurate tick intervals) instead of stair-stepping per tick,
//     and re-phase on FA; MIDI-clocked step sources (euclid/gate/retrig-ADSR)
//     advance on the integer ticks (they are edge machines, no ramp needed).
//
// BENCH CONTROL (T3 scope): multiplexed CC 33/34/35 (source/param/value, see
// midi.h) + serial console `mod`, `mod?`, `bpm`, `clock` — the C3 MOD page
// (T5) will write the same state. Routing to a CC the active module does not
// accept is INERT and logged once at set time (core 0; the engine itself
// never prints from core 1).
//
// STATE BUDGET: ~1.9 KB .bss (12 sources x ~56 B + 64x16 B stamp ring +
// scalars) — inside the plan's "mod engine ≈ 8 KB" envelope.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "pico/time.h"           // time_us_32 (static inline, pico-sdk 2.x)

// Forward declaration of ui_link helper to restore cached parameter values on mod unassign.
void ui_link_restore_cached_cc(uint8_t mod, uint8_t cc);
// T4a preset autosave dirty hook (defined in ui_link.h — every persistent
// state change marks the blob dirty; a core-0 debounce then pushes it to the
// C3's flash bank). Defined AFTER this header in main.cpp's include order.
void preset_mark_dirty();

#include "module_registry.h"     // RackModuleInfo / RackModDestinations / ids

// The four adapters (included by main.cpp BEFORE this header) provide, per
// module: mod_set_cc_float(cc, v) / mod_trigger_cc(cc, high). midi.h
// (included earlier) declares rack_mod_clock_realtime / rack_mod_bench_cc —
// defined below. shared_note / shared_gate come from midi.h.

extern volatile uint8_t g_active_module;   // defined in main.cpp (non-static since T3)

// T6 chain-mode grid override (ADDITIVE — the engine's core logic is
// untouched; these two reads replace the registry reads at the top of
// mod_engine_process_block when nonzero). While a chain is engaged the
// render loop runs the CHAIN grid (32 kHz / 24-sample blocks = 750 us)
// instead of the A slot's registry block/rate, and g_active_module IS the A
// slot — so destinations bind to A (decision 4) with no further changes.
// Both are 0 whenever no chain is engaged (main.cpp keeps them in sync with
// g_chain.active). Rate is carried as kHz (32/48 fits a byte).
extern volatile uint8_t g_mod_chain_grid;       // 0 = off, else block size
extern volatile uint8_t g_mod_chain_rate_khz;   // 0 = off, else rate/1000

#include "gen_tables.h"

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
  MOD_ARP_HYP1   = 0,   // HYP-1 Acid / 303 W7 Chord-Drift + Mutate + Turnarounds
  MOD_ARP_MOTIF  = 1,   // Phrase-Grammar Motif Transformer (ABAC/AABC)
  MOD_ARP_MARKOV = 2,   // Tonal-Gravity Markov Walker + 16 Melodic Contours
  MOD_ARP_METRO  = 3,   // Metropolix 8-Stage Accumulator & Ratchets
  MOD_ARP_DEJAVU = 4,   // Marbles 3-Zone Deja-Vu Shift Register + Salience
};

// Source ids (CC 33 values): 0-3 LFO1-4, 4-5 EU1-2, 6-7 GATE1-2, 8-9 ADSR1-2, 10-11 ARP1-2.
// 4+2+2+2+2 = 12 sources total.
static const uint8_t MOD_SOURCE_COUNT = 12;
static const uint8_t MOD_SRC_EU_FIRST = 4;    // odometer source for telemetry
static const uint8_t MOD_SRC_ARP_FIRST = 10;  // arp1 = 10, arp2 = 11
static const uint8_t MOD_ARP_COUNT = 2;
static const uint8_t MOD_DEST_VOICE = 127;    // universal voice pitch+gate pseudo-CC

// Clock divisions: PPQ ticks per division cycle (24 PPQ = quarter note).
// All divisors are integers in 24 PPQ (triplets included).
struct ModDivision { const char* name; uint16_t ppq; };
static const ModDivision kModDivisions[13] = {
    {"8/1", 768}, {"4/1", 384}, {"2/1", 192},
    {"1/1",  96}, {"1/2",  48}, {"1/4",  24}, {"1/8", 12}, {"1/16", 6},
    {"1/32",  3}, {"1/4T", 16}, {"1/8T",  8}, {"1/16T", 4}, {"1/32T", 2},
};
static const uint8_t MOD_DIVISION_COUNT = 13;

// Per-source parameter names/counts — the CC 34 param indexes and the console
// `mod <src> <param> <value>` names use the SAME tables. The last three
// params of every type are uniformly clk / div / dest.
static const char* const kModParamNames[5][8] = {
    {"mode", "rate", "wave", "phase", "level", "clk", "div", "dest"},  // LFO
    {"steps", "pulses", "rot", "clk", "div", "dest"},                  // EUCLID
    {"mode", "dens", "sync", "glen", "clk", "div", "dest"},            // GATE
    {"mode", "atk", "dec", "sus", "rel", "clk", "div", "dest"},        // ADSR
    {"algo", "p1", "p2", "p3", "p4", "clk", "div", "dest"},            // ARP
};
static const uint8_t kModParamCount[5] = {8, 6, 7, 8, 8};

// Compact 16-step pattern + harmony state for arp1 and arp2 (112 B per instance).
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
  uint8_t is_minor_triad;
  uint8_t bar_count;
  uint8_t dd_fill;
  uint8_t turn_type;

  // Algorithm-specific persistent generator state
  union {
    struct {
      int8_t  seed_deg[4];
      int8_t  seed_oct[4];
      uint8_t seed_flags[4];
      uint8_t form;
    } motif;
    struct {
      uint8_t prev_deg;
      int8_t  prev_oct;
      uint8_t contour_id;
    } markov;
    struct {
      int8_t  accum_val[8];
      int8_t  accum_dir[8];
      uint8_t cur_stage;
      uint8_t cur_pulse;
    } metro;
    struct {
      uint8_t u_pitch[16];
      uint8_t u_gate[16];
    } dejavu;
  } st;

  // 303 DSP Articulator state (Q16.16)
  int32_t current_pitch_q16;
  int32_t target_pitch_q16;
  int32_t accent_cap_q16;
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
    struct { uint8_t mode, density, syncopation, len; uint32_t mask;
             float gate_len; uint8_t step; uint32_t ticks; uint8_t level;
             uint32_t rng_state; uint8_t cur_preset; } gate;
    struct { float a_s, d_s, s_lvl, r_s; uint8_t retrig, env_state; float env;
             uint32_t ticks; } adsr;
    struct {
      uint8_t algo;           // 0..4 (HYP1, MOTIF, MARKOV, METRO, DEJAVU)
      uint8_t range;          // 1..3 octaves
      uint8_t root_param;     // 0 = MIDI follow, 1=PARAL, 2=W7, 3=NEO, 24..84 = fixed root MIDI note
      uint8_t midi_root;      // last captured external MIDI note (default 48 = C3)
      uint8_t last_out_note;  // last note written to shared_note by this arp
      uint8_t scale_idx;      // 0..19 (SCALE_COUNT)
      uint8_t p[4];           // 4 algorithm-specific parameters (0..100)
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
// SPSC: core 0 produces (head), core 1 consumes (tail). Entries are 16 B
// (aligned; single-writer per index, same ordering discipline as the T2 swap
// handshake volatiles).
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

// Check if any modulation source is currently routed to destination cc.
bool mod_is_dest_routed(uint8_t cc) {
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    if (g_mod_engine.src[i].dest_cc == cc) return true;
  }
  return false;
}

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

// LFO wave, UNIPOLAR 0..1 (the routed destination value is this x level;
// all waves start at 0 at phase 0 so a routed param sweeps up from its floor).
static inline float mod_wave(uint8_t wave, float ph, float sh_value) {
  switch (wave) {
    case MOD_WAVE_SINE: return 0.5f - 0.5f * cosf(6.2831853f * ph);
    case MOD_WAVE_TRI:  return ph < 0.5f ? 2.0f * ph : 2.0f - 2.0f * ph;
    case MOD_WAVE_SAW:  return ph;
    case MOD_WAVE_SQR:  return ph < 0.5f ? 0.0f : 1.0f;
    default:            return sh_value;                 // S&H
  }
}

// Euclidean mask — maximally-even distribution of `pulses` over `steps`, bit i
// = step i, NORMALIZED so the first pulse sits at step 0. Equal to Toussaint's
// canonical E(k,n) UP TO ROTATION (verified exact for E(4,16)={0,4,8,12},
// E(5,8)={0,2,3,5,6}, E(5,16)={0,3,6,9,12}; e.g. E(3,8) lands on the {0,3,5}
// rotation of the {0,3,6} canon — the rot param explores every rotation).
// Host-test-verified invariants: pulse count, first-pulse-at-0, interval
// multiset == the canonical's. Computed ONLY on parameter change (bucket/
// bresenham accumulator + first-bit rotation), never per tick.
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

// Rotate a `steps`-bit mask RIGHT by rot (the euclid "rotation" param).
static uint32_t mod_rotate_mask(uint32_t mask, uint8_t steps, uint8_t rot) {
  if (steps <= 1 || rot == 0) return mask;
  const uint8_t r = rot % steps;
  if (r == 0) return mask;
  if (steps >= 32) return (mask >> r) | (mask << (32u - r));
  const uint32_t m = mask & ((1u << steps) - 1u);
  return ((m >> r) | (m << (steps - r))) & ((1u << steps) - 1u);
}

// Recompute the euclid mask cache + clamp the step index. Called on any
// steps/pulses/rot change (core 0 bench CC / console) — core 1 reads the
// resulting uint32 atomically.
static void mod_eu_recompute(ModSource& s) {
  if (s.p.eu.steps < 1) s.p.eu.steps = 1;
  if (s.p.eu.steps > 32) s.p.eu.steps = 32;
  if (s.p.eu.pulses > s.p.eu.steps) s.p.eu.pulses = s.p.eu.steps;
  s.p.eu.mask = mod_rotate_mask(
      mod_euclid_mask(s.p.eu.pulses, s.p.eu.steps), s.p.eu.steps, s.p.eu.rot);
  s.p.eu.step %= s.p.eu.steps;
}

// 16 Canonical 16-Step Trance Gate Presets (MSB = Step 1, 1 = 'o', 0 = 'x')
static const uint16_t kTranceGatePresets[16] = {
    0xAAAA, // 01: o x o x o x o x o x o x o x o x (Classic 1/8th Alternate)
    0x5555, // 02: x o x o x o x o x o x o x o x o (Classic Trance Off-Beat)
    0x7777, // 03: x o o o x o o o x o o o x o o o (4-on-the-Floor Ducking Pump)
    0x3333, // 04: x x o o x x o o x x o o x x o o (Deep / Prog Late Pump)
    0xEEEE, // 05: o o o x o o o x o o o x o o o x (Forward Gallop 3+1)
    0xBBBB, // 06: o x o o o x o o o x o o o x o o (Reverse Gallop Pickup)
    0x5657, // 07: x o x o x o o x x o x o x o o o (Uplifting Rolling Gate)
    0xB5B5, // 08: o x o o x o x o o x o o x o x o (Vanguard Anthem Gate)
    0x9249, // 09: o x x o x x o x x o x x o x x o (Dotted 1/8th Cross-Grid)
    0x9292, // 10: o x x o x x o x o x x o x x o x (Double Tresillo 3-3-2)
    0x9228, // 11: o x x o x x o x x x o x o x x x (Son Clave 3-2 Timeline)
    0x9129, // 12: o x x o x x x o x x o x o x x o (Rumba / Funk Syncopator)
    0xC0C2, // 13: o o x x x x x x o o x x x x o x (Punctuated Chord Stabs)
    0xB6B5, // 14: o x o o x o o x o x o o x o x o (Prog Call & Response)
    0xA55A, // 15: o x o x x o x o x o x o o x o x (Breakbeat / Electro Chop)
    0x88AF  // 16: o x x x o x x x o x o x o o o o (Accelerating Riser Build)
};

static const char* const kTrancePresetNames[19] = {
    "GEN",  "ALT8", "OFF8", "PUMP", "LATE", "GALL", "RGAL", "ROLL", "ANTH",
    "DOT8", "TRES", "CLAV", "RUMB", "STAB", "PROG", "CHOP", "RISE",
    "TMR",  "MRKV"
};

// Clarence Barlow's 4/4 16-step metric indispensability table (15 = strongest, 0 = weakest)
static const uint8_t kBarlow16[16] = {
    15, 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7
};

// Sioros-Guedes syncopation candidate order (strong quarter notes first, then eighths)
static const uint8_t kSyncCandidates[8] = { 0, 4, 8, 12, 2, 6, 10, 14 };

// Normalized Hamming distance between two 16-bit patterns
static inline uint8_t mod_hamming_dist(uint16_t a, uint16_t b) {
  return (uint8_t)__builtin_popcount((uint32_t)(a ^ b));
}

// Architecture 4: Hamming-Distance Markov Transition across the 16 Presets
static uint8_t mod_gate_markov_next(uint8_t current_preset, uint8_t temp, uint32_t& rng) {
  uint8_t d_min = 16;
  uint8_t dists[16];
  for (uint8_t j = 0; j < 16; ++j) {
    if (j == current_preset) {
      dists[j] = 0;
      continue;
    }
    dists[j] = mod_hamming_dist(kTranceGatePresets[current_preset], kTranceGatePresets[j]);
    if (dists[j] < d_min) d_min = dists[j];
  }

  uint16_t weights[16];
  uint16_t total_weight = 0;
  for (uint8_t j = 0; j < 16; ++j) {
    if (j == current_preset) {
      weights[j] = 0;
      continue;
    }
    uint8_t delta = dists[j] - d_min;
    // Scale delta attenuation: at temp=0, strong drop; at temp=255, uniform (shift=0)
    uint32_t shift = ((uint32_t)delta * (255u - (uint32_t)temp) * 3u) / 255u;
    uint16_t w = (shift >= 12) ? 0 : (uint16_t)(512u >> shift);
    if (w == 0 && temp > 128) w = 1;
    weights[j] = w;
    total_weight += w;
  }

  if (total_weight == 0) {
    for (uint8_t j = 0; j < 16; ++j) {
      if (j != current_preset && dists[j] == d_min) return j;
    }
    return (current_preset + 1) % 16;
  }

  rng = rng * 1664525UL + 1013904223UL;
  uint16_t pick = (uint16_t)(((rng >> 16) * total_weight) >> 16);
  uint16_t running = 0;
  for (uint8_t j = 0; j < 16; ++j) {
    running += weights[j];
    if (pick < running && weights[j] > 0) return j;
  }
  return (current_preset + 1) % 16;
}

// Recompute the rhythmic trance gate mask.
static void mod_gate_recompute(ModSource& s) {
  s.p.gate.len = 16;
  uint8_t step_on[16];

  if (s.p.gate.mode == 17) {
    // Mode 17: TMR (Architecture 4 Shift-Register / Turing Machine Mutation)
    // Seed register based on density threshold
    const uint8_t thresh = (s.p.gate.density >= 16) ? 0 : (uint8_t)(16 - s.p.gate.density);
    for (int i = 0; i < 16; ++i) {
      step_on[i] = (kBarlow16[i] >= thresh) ? 1 : 0;
    }
    uint16_t mask = 0;
    for (int i = 0; i < 16; ++i) {
      if (step_on[i]) mask |= (1u << (15 - i));
    }
    s.p.gate.mask = mask;
    s.p.gate.step %= s.p.gate.len;
    s.p.gate.level = (s.p.gate.mask >> (15 - s.p.gate.step)) & 1u;
    return;
  }

  if (s.p.gate.mode == 18) {
    // Mode 18: MRKV (Architecture 4 Hamming-Distance Markov Preset Transitions)
    // DENS selects starting seed preset (1..16 -> index 0..15)
    uint8_t p_idx = (s.p.gate.density >= 1 && s.p.gate.density <= 16)
                        ? (uint8_t)(s.p.gate.density - 1) : 0;
    s.p.gate.cur_preset = p_idx;
    s.p.gate.mask = kTranceGatePresets[p_idx];
    s.p.gate.step %= s.p.gate.len;
    s.p.gate.level = (s.p.gate.mask >> (15 - s.p.gate.step)) & 1u;
    return;
  }

  if (s.p.gate.mode > 0 && s.p.gate.mode <= 16) {
    // Canonical preset mode (1..16)
    const uint16_t preset = kTranceGatePresets[s.p.gate.mode - 1];
    for (int i = 0; i < 16; ++i) {
      step_on[i] = (preset >> (15 - i)) & 1u;
    }
    // Density trim if density < 16
    if (s.p.gate.density < 16) {
      const uint8_t thresh = (uint8_t)(16 - s.p.gate.density);
      for (int i = 0; i < 16; ++i) {
        if (kBarlow16[i] < thresh) step_on[i] = 0;
      }
    }
  } else {
    // Mode 0: Barlow metric indispensability generative engine
    const uint8_t thresh = (s.p.gate.density >= 16) ? 0 : (uint8_t)(16 - s.p.gate.density);
    for (int i = 0; i < 16; ++i) {
      step_on[i] = (kBarlow16[i] >= thresh) ? 1 : 0;
    }
  }

  // Sioros-Guedes Left-Shift Anticipation with Anti-Offset Protection
  if (s.p.gate.syncopation > 0) {
    // Deterministic PRNG seed per (mode, density, syncopation) for 100% reproducible grooves
    uint32_t rng = 0x1337cafeu ^ ((uint32_t)s.p.gate.density * 1013904223u) ^
                   ((uint32_t)s.p.gate.syncopation * 1664525u) ^
                   ((uint32_t)s.p.gate.mode * 2654435761u);
    bool prev_was_shifted = false;
    for (int c = 0; c < 8; ++c) {
      const uint8_t i = kSyncCandidates[c];
      const uint8_t prev_i = (i + 15) & 0x0Fu;
      if (step_on[i] == 1 && step_on[prev_i] == 0 && !prev_was_shifted) {
        rng = rng * 1664525u + 1013904223u;
        const uint8_t rand_sync = (uint8_t)((rng >> 24) & 0xFFu);
        if (rand_sync < s.p.gate.syncopation) {
          step_on[prev_i] = 1; // Anticipate into weaker pickup step
          step_on[i] = 0;      // True syncopation (mute the strong beat)
          prev_was_shifted = true;
        } else {
          prev_was_shifted = false;
        }
      } else {
        prev_was_shifted = false;
      }
    }
  }

  uint16_t mask = 0;
  for (int i = 0; i < 16; ++i) {
    if (step_on[i]) mask |= (1u << (15 - i));
  }
  s.p.gate.mask = mask;
  s.p.gate.step %= s.p.gate.len;
  s.p.gate.level = (s.p.gate.mask >> (15 - s.p.gate.step)) & 1u;
}

// ===================== MOD_ARP: 5-Algo Generative Melody & Acid Engine ===========

static void mod_arp_rebuild_scale(ModSource& s) {
  ModArpExt& ext = g_mod_engine.arp_ext[s.p.arp.ext_idx];
  const uint8_t sc_id = (s.p.arp.scale_idx & 0x3Fu) % SCALE_COUNT;
  const uint16_t pool = SCALE_BITMASKS[sc_id];
  ext.nsteps = expand_scale_mask(pool, ext.scale_st);
  if (ext.deg >= (int8_t)ext.nsteps) ext.deg = 0;
  ext.chord_nt = (ext.nsteps >= 6) ? 4 : 3;
}

// Bar-line chord progression (Neo-Riemannian, Diatonic Markov, or W7 drift)
static void mod_arp_next_chord(ModSource& s) {
  ModArpExt& ext = g_mod_engine.arp_ext[s.p.arp.ext_idx];
  if (s.p.arp.root_param == 3) {
    // Neo-Riemannian Triad drift (semitone shift + major/minor quality flip)
    apply_neo_riemannian(&s.p.arp.midi_root, &ext.is_minor_triad,
                         (neo_riemannian_id_t)(mod_irand(s, NR_TRANSFORM_COUNT)));
    ext.deg = 0;
    for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
      g_mod_engine.src[MOD_SRC_ARP_FIRST + k].p.arp.midi_root = s.p.arp.midi_root;
      g_mod_engine.arp_ext[k].is_minor_triad = ext.is_minor_triad;
      g_mod_engine.arp_ext[k].deg = 0;
    }
    return;
  }
  // Diatonic Functional Markov or W7 drift
  uint32_t w[12], sum = 0;
  const uint8_t cur_d = (uint8_t)((ext.deg >= 0 && ext.deg < ext.nsteps) ? ext.deg : 0);
  for (uint8_t i = 0; i < ext.nsteps; ++i) {
    if (i == cur_d) {
      w[i] = 0u;
    } else if (ext.nsteps == 7) {
      w[i] = (s.p.arp.root_param == 2) ? DIATONIC_W7_WEIGHTS[i] : CHORD_MARKOV_7X7[cur_d][i];
    } else {
      w[i] = (i == 0) ? 30u : 15u;
    }
    sum += w[i];
  }
  uint32_t r = mod_irand(s, (uint8_t)(sum ? sum : 1));
  uint8_t next_deg = 0;
  for (uint8_t i = 0; i < ext.nsteps; ++i) {
    if (r < w[i] && w[i] > 0) { next_deg = i; break; }
    r -= w[i];
  }
  if (next_deg == cur_d) {
    next_deg = (cur_d == 0 && ext.nsteps > 1) ? 1 : 0;
  }
  ext.deg = (int8_t)next_deg;
  for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
    g_mod_engine.arp_ext[k].deg = ext.deg;
  }
}

// Pick chord-tone offset (even scale steps) or passing tone
static int8_t mod_arp_pick_offset(ModSource& s, const ModArpExt& ext, bool strong_beat) {
  const bool extended_chord = (s.p.arp.scale_idx >= 64) || ((s.p.arp.scale_idx & 0x08u) != 0);
  if (strong_beat || mod_irand(s, 100) < 80) {
    uint8_t idx = mod_irand(s, ext.chord_nt);
    int8_t off = (int8_t)(2 * idx);
    if (extended_chord && idx == 2) off = 6; // 9th/Sus flavor skips 5th for 7th/9th
    if (mod_irand(s, 100) < 15) off = (int8_t)(off + ext.nsteps);
    return off;
  }
  return (int8_t)mod_irand(s, ext.nsteps);
}

// Roll a single step for Algo 0 (HYP-1 Acid ac_roll)
// p[0]=DENS (0..100), p[1]=SLIDE (0..100), p[2]=ACCENT (0..100), p[3]=MUTATE (0..100)
static void mod_arp_roll_hyp1_step(ModSource& s, ModArpExt& ext, uint8_t st) {
  ModArpStep& step = ext.steps[st];
  const uint8_t dens = s.p.arp.p[0];
  const uint8_t eff_dens = (st == 0) ? (dens + 30 > 100 ? 100 : dens + 30) : dens;
  const bool gate = mod_irand(s, 100) < eff_dens;
  const bool acc  = mod_irand(s, 100) < s.p.arp.p[2];
  const bool sld  = mod_irand(s, 100) < s.p.arp.p[1];
  step.flags = (gate ? 1u : 0u) | (acc ? 2u : 0u) | (sld ? 4u : 0u);
  step.off = mod_arp_pick_offset(s, ext, (st & 3u) == 0u);
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;
  step.oct = (mod_irand(s, 100) < 55) ? 0 : (int8_t)mod_irand(s, rng_oct);
}

// Algo 1: Phrase-Grammar Motif Transformer (ABAC / AABC / ABBC / AABA)
// p[0]=DENS (0..100), p[1]=FORM (0..3), p[2]=XFORM (0..3), p[3]=MUTATE (0..100)
static void mod_arp_build_motif(ModSource& s, ModArpExt& ext) {
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;
  const uint8_t form = (s.p.arp.p[1] * 4) / 101; // 0=ABAC, 1=AABC, 2=ABBC, 3=AABA
  const uint8_t xform = (s.p.arp.p[2] * 4) / 101; // 0=INV, 1=RETRO, 2=TRANS, 3=DISP
  const uint8_t mutate = s.p.arp.p[3];
  ext.st.motif.form = form;

  // Generate / refresh Seed A (4 steps = 1 beat)
  for (uint8_t i = 0; i < 4; ++i) {
    if (ext.st.motif.seed_flags[i] == 0 || (mutate > 0 && mod_irand(s, 100) < (mutate >> 1))) {
      uint8_t gate_thresh = (i == 0) ? (s.p.arp.p[0] + 25 > 100 ? 100 : s.p.arp.p[0] + 25) : s.p.arp.p[0];
      bool gate = mod_irand(s, 100) < gate_thresh;
      int8_t deg;
      if (i == 0) {
        deg = (mod_irand(s, 2) == 0) ? 0 : 4; // Tonic or 5th
      } else if (mod_irand(s, 100) < 80) {
        deg = (int8_t)(2 * mod_irand(s, ext.chord_nt)); // Chord tone
      } else {
        deg = (int8_t)mod_irand(s, ext.nsteps);          // Passing tone
      }
      ext.st.motif.seed_deg[i] = deg;
      ext.st.motif.seed_oct[i] = (mod_irand(s, 100) < 80) ? 0 : (int8_t)mod_irand(s, rng_oct);
      bool acc = mod_irand(s, 100) < 30;
      bool sld = mod_irand(s, 100) < 25;
      ext.st.motif.seed_flags[i] = (gate ? 1u : 0u) | (acc ? 2u : 0u) | (sld ? 4u : 0u);
    }
  }
  // Guarantee step 0 active tonic anchor
  ext.st.motif.seed_flags[0] |= 1u;
  ext.st.motif.seed_deg[0] = 0;

  // Transform B according to xform
  int8_t b_deg[4], b_oct[4];
  uint8_t b_flags[4];
  int8_t pivot = ext.st.motif.seed_deg[0] + 2;
  for (uint8_t i = 0; i < 4; ++i) {
    if (xform == 0) { // Inversion
      b_deg[i] = (int8_t)((2 * pivot) - ext.st.motif.seed_deg[i]);
      b_oct[i] = ext.st.motif.seed_oct[i];
      b_flags[i] = ext.st.motif.seed_flags[i];
    } else if (xform == 1) { // Retrograde
      b_deg[i] = ext.st.motif.seed_deg[3 - i];
      b_oct[i] = ext.st.motif.seed_oct[3 - i];
      b_flags[i] = ext.st.motif.seed_flags[3 - i];
    } else if (xform == 2) { // Transpose +5th
      b_deg[i] = (int8_t)(ext.st.motif.seed_deg[i] + 4);
      b_oct[i] = ext.st.motif.seed_oct[i];
      b_flags[i] = ext.st.motif.seed_flags[i];
    } else { // Rhythmic displacement (rotate right by 1)
      uint8_t prev_i = (i + 3) & 3u;
      b_deg[i] = ext.st.motif.seed_deg[prev_i];
      b_oct[i] = ext.st.motif.seed_oct[prev_i];
      b_flags[i] = ext.st.motif.seed_flags[prev_i];
    }
  }

  // Transform C: Retrograde with Rhythmic Displacement and Cadential Pickup on step 3 (bar step 15)
  int8_t c_deg[4], c_oct[4];
  uint8_t c_flags[4];
  for (uint8_t i = 0; i < 4; ++i) {
    c_deg[i] = ext.st.motif.seed_deg[3 - i];
    c_oct[i] = ext.st.motif.seed_oct[3 - i];
    c_flags[i] = ext.st.motif.seed_flags[3 - i];
  }
  // Enforce cadential pickup on step 15 (slurs into beat 1)
  c_flags[3] = 1u | 2u | 4u; // Gate, accent, slide
  c_deg[3] = -1;             // Leading tone resolving to tonic
  c_oct[3] = 0;

  // Compile 16 steps into bar
  const int8_t* deg_ptrs[4];
  const int8_t* oct_ptrs[4];
  const uint8_t* flag_ptrs[4];
  if (form == 0) { // ABAC
    deg_ptrs[0] = ext.st.motif.seed_deg; deg_ptrs[1] = b_deg;
    deg_ptrs[2] = ext.st.motif.seed_deg; deg_ptrs[3] = c_deg;
    oct_ptrs[0] = ext.st.motif.seed_oct; oct_ptrs[1] = b_oct;
    oct_ptrs[2] = ext.st.motif.seed_oct; oct_ptrs[3] = c_oct;
    flag_ptrs[0] = ext.st.motif.seed_flags; flag_ptrs[1] = b_flags;
    flag_ptrs[2] = ext.st.motif.seed_flags; flag_ptrs[3] = c_flags;
  } else if (form == 1) { // AABC
    deg_ptrs[0] = ext.st.motif.seed_deg; deg_ptrs[1] = ext.st.motif.seed_deg;
    deg_ptrs[2] = b_deg;                 deg_ptrs[3] = c_deg;
    oct_ptrs[0] = ext.st.motif.seed_oct; oct_ptrs[1] = ext.st.motif.seed_oct;
    oct_ptrs[2] = b_oct;                 oct_ptrs[3] = c_oct;
    flag_ptrs[0] = ext.st.motif.seed_flags; flag_ptrs[1] = ext.st.motif.seed_flags;
    flag_ptrs[2] = b_flags;                 flag_ptrs[3] = c_flags;
  } else if (form == 2) { // ABBC
    deg_ptrs[0] = ext.st.motif.seed_deg; deg_ptrs[1] = b_deg;
    deg_ptrs[2] = b_deg;                 deg_ptrs[3] = c_deg;
    oct_ptrs[0] = ext.st.motif.seed_oct; oct_ptrs[1] = b_oct;
    oct_ptrs[2] = b_oct;                 oct_ptrs[3] = c_oct;
    flag_ptrs[0] = ext.st.motif.seed_flags; flag_ptrs[1] = b_flags;
    flag_ptrs[2] = b_flags;                 flag_ptrs[3] = c_flags;
  } else { // AABA
    deg_ptrs[0] = ext.st.motif.seed_deg; deg_ptrs[1] = ext.st.motif.seed_deg;
    deg_ptrs[2] = b_deg;                 deg_ptrs[3] = ext.st.motif.seed_deg;
    oct_ptrs[0] = ext.st.motif.seed_oct; oct_ptrs[1] = ext.st.motif.seed_oct;
    oct_ptrs[2] = b_oct;                 oct_ptrs[3] = ext.st.motif.seed_oct;
    flag_ptrs[0] = ext.st.motif.seed_flags; flag_ptrs[1] = ext.st.motif.seed_flags;
    flag_ptrs[2] = b_flags;                 flag_ptrs[3] = ext.st.motif.seed_flags;
  }

  for (uint8_t b = 0; b < 4; ++b) {
    for (uint8_t sub = 0; sub < 4; ++sub) {
      uint8_t st = (b << 2) | sub;
      ext.steps[st].off   = deg_ptrs[b][sub];
      ext.steps[st].oct   = oct_ptrs[b][sub];
      ext.steps[st].flags = flag_ptrs[b][sub];
    }
  }
}

// Algo 2: Tonal-Gravity Markov Walker & 16 Melodic Contours
// p[0]=DENS (0..100), p[1]=CONTOUR (0..15), p[2]=PEDAL (0..3), p[3]=GRAVTY (0..100)
static void mod_arp_build_markov(ModSource& s, ModArpExt& ext) {
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;
  const uint8_t cid = (s.p.arp.p[1] * 16) / 101;
  const uint8_t pedal_mode = (s.p.arp.p[2] * 4) / 101; // 0=OFF, 1=ROOT, 2=5TH, 3=BOUNCE
  const uint8_t grav_weight = s.p.arp.p[3]; // 0..100
  ext.st.markov.contour_id = cid;
  uint8_t cur_d = ext.st.markov.prev_deg % 7u;

  for (uint8_t st = 0; st < 16; ++st) {
    uint8_t beat_tier = ((st & 3u) == 0) ? 0 : (((st & 1u) == 0) ? 1 : 2);
    uint16_t eff_dens = s.p.arp.p[0];
    if (beat_tier == 0) eff_dens += 20;
    else if (beat_tier == 1) eff_dens += 10;
    if (eff_dens > 100) eff_dens = 100;

    bool gate = mod_irand(s, 100) < eff_dens;
    bool is_pedal = false;
    int8_t deg = 0, oct = 0;
    bool acc = false, sld = false;

    if (pedal_mode == 1 && beat_tier == 0) { // ROOT on quarters
      is_pedal = true; deg = 0;
    } else if (pedal_mode == 2 && beat_tier == 0) { // 5TH on quarters
      is_pedal = true; deg = 4;
    } else if (pedal_mode == 3 && (st == 0 || st == 8)) { // BOUNCE on half notes
      is_pedal = true; deg = (st == 0) ? 0 : 4;
    }

    if (is_pedal) {
      acc = (st == 0);
    } else {
      // Evaluate Markov + Metrical Gravity + Contour Proximity
      int8_t target_deg = MELODIC_CONTOURS_DEG[cid][st];
      int8_t target_mod = floor_mod_i16(target_deg, 7);

      uint32_t cdf[7];
      uint32_t total_w = 0;
      for (uint8_t d = 0; d < 7; ++d) {
        uint32_t w_markov = DEGREE_MARKOV_7X7[cur_d][d];
        uint32_t w_metric = DEGREE_METRICAL_GRAVITY[beat_tier][d];
        uint32_t w = (w_markov * (100u - grav_weight) + w_metric * grav_weight) >> 4;
        int8_t dist = (int8_t)d - target_mod;
        if (dist < 0) dist = (int8_t)(-dist);
        if (dist > 3) dist = (int8_t)(7 - dist);
        w += (uint32_t)(4 - dist) * ((grav_weight * 15u) / 100u);
        if (w == 0) w = 1;
        total_w += w;
        cdf[d] = total_w;
      }
      uint32_t pick = mod_irand(s, (uint8_t)(total_w > 255 ? 255 : total_w));
      for (uint8_t d = 0; d < 7; ++d) {
        if (pick < cdf[d]) { cur_d = d; break; }
      }
      deg = (int8_t)cur_d;
      oct = (target_deg >= 7 && rng_oct > 1) ? 1 : ((target_deg < 0) ? -1 : 0);
      acc = (beat_tier == 2 && mod_irand(s, 100) < 35);
      sld = (beat_tier == 2 && mod_irand(s, 100) < 30);
    }

    ext.steps[st].off   = deg;
    ext.steps[st].oct   = oct;
    ext.steps[st].flags = (gate ? 1u : 0u) | (acc ? 2u : 0u) | (sld ? 4u : 0u);
  }
  ext.st.markov.prev_deg = cur_d;
}

// Algo 3: Metropolix Multi-Stage Accumulator
// p[0]=PROB (0..100), p[1]=PULSES (1..8), p[2]=RATCHET (0..3), p[3]=ACCUM (0..3)
static void mod_arp_build_metro(ModSource& s, ModArpExt& ext) {
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;
  static const int8_t kInitDegs[8]   = { 0, 2, 4, 6, 4, 2, 1, 4 };
  const uint8_t max_pulses = 1 + (s.p.arp.p[1] * 7) / 100; // 1..8
  const uint8_t ratchet_mode = (s.p.arp.p[2] * 4) / 101; // 0=OFF, 1=2X, 2=3X, 3=4X
  const uint8_t accum_mode = (s.p.arp.p[3] * 4) / 101;   // 0=PENDLM, 1=WRAP, 2=RANDOM, 3=HOLD
  static const int8_t kInitAccum[8]  = { 0, 1, 0, 2, 0, -1, 1, 2 };

  uint8_t st = 0;
  for (uint8_t stage = 0; stage < 8; ++stage) {
    uint8_t pulses = (max_pulses <= 2) ? max_pulses : (uint8_t)(1 + (stage % max_pulses));
    int8_t base_d = kInitDegs[stage];
    int8_t acc = ext.st.metro.accum_val[stage];
    bool gate_stage = mod_irand(s, 100) < s.p.arp.p[0];

    for (uint8_t p = 0; p < pulses && st < 16; ++p, ++st) {
      bool gate = gate_stage && (stage == 0 || p == 0 || mod_irand(s, 100) < 70);
      bool is_hold = (stage == 0 && p < pulses - 1);
      bool is_acc = (stage == 0 || stage == 3 || stage == 6);
      bool is_sld = is_hold || (stage == 2 || stage == 5);
      if (ratchet_mode > 0 && is_acc) is_sld = true; // Ratchet triggers slide/hold legato
      ext.steps[st].off   = (int8_t)(base_d + acc);
      ext.steps[st].oct   = (stage == 3 || stage == 7) && (rng_oct > 1) ? 1 : 0;
      ext.steps[st].flags = (gate ? 1u : 0u) | (is_acc ? 2u : 0u) | (is_sld ? 4u : 0u);
    }

    // Accumulation advance according to accum_mode
    if (accum_mode == 0) { // PENDULUM
      int8_t step_amt = kInitAccum[stage];
      if (ext.st.metro.accum_dir[stage] == 0) ext.st.metro.accum_dir[stage] = 1;
      int16_t next = (int16_t)ext.st.metro.accum_val[stage] + (int16_t)(step_amt * ext.st.metro.accum_dir[stage]);
      if (next > 7) { next = 7 - (next - 7); ext.st.metro.accum_dir[stage] = -1; }
      if (next < -7) { next = -7 + (-7 - next); ext.st.metro.accum_dir[stage] = 1; }
      ext.st.metro.accum_val[stage] = (int8_t)next;
    } else if (accum_mode == 1) { // WRAP (0..7)
      int8_t step_amt = kInitAccum[stage];
      int16_t next = (int16_t)ext.st.metro.accum_val[stage] + step_amt;
      if (next > 7) next = 0;
      if (next < 0) next = 7;
      ext.st.metro.accum_val[stage] = (int8_t)next;
    } else if (accum_mode == 2) { // RANDOM step drift
      if (mod_irand(s, 100) < 30) {
        ext.st.metro.accum_val[stage] = (int8_t)(mod_irand(s, 7) - 3);
      }
    } // accum_mode == 3: HOLD (no advance)
  }
}

// Algo 4: Marbles 3-Zone Deja-Vu Shift Register
// p[0]=DENS (0..100), p[1]=DEJAVU (0..100), p[2]=STEPS (0..3), p[3]=SPREAD (0..100)
static void mod_arp_build_dejavu(ModSource& s, ModArpExt& ext) {
  const uint8_t rng_oct = s.p.arp.range ? s.p.arp.range : 1;
  const uint8_t dj = s.p.arp.p[1]; // 0..100 -> Zone 1 (0..45), Zone 2 (46..55), Zone 3 (56..100)
  const uint8_t steps_mode = (s.p.arp.p[2] * 4) / 101; // 0=ALL, 1=PENT, 2=TRIAD, 3=ROOT
  const uint8_t spread = s.p.arp.p[3]; // 0..100

  // 1. Zone 1: Metrically biased mutation (weak beats mutate first)
  if (dj < 46) {
    uint16_t mut_drive = (46u - dj) * 5u; // 0..230
    for (uint8_t i = 0; i < 16; ++i) {
      uint16_t sens = METRIC_MUTATE_SENSITIVITY[i];
      uint16_t prob = (mut_drive * sens) >> 8;
      if (mod_irand(s, 255) < prob) {
        ext.st.dejavu.u_pitch[i] = (uint8_t)mod_irand(s, 255);
        ext.st.dejavu.u_gate[i]  = (uint8_t)mod_irand(s, 255);
      }
    }
  }

  // 2. Progressive STEPS Salience Filtering:
  for (uint8_t st = 0; st < 16; ++st) {
    uint8_t r_ix = st;
    // Zone 3: Permutation / jump read pointer
    if (dj > 55) {
      uint8_t jump_prob = (uint8_t)((dj - 55u) * 2u);
      if (mod_irand(s, 100) < jump_prob) {
        r_ix = (uint8_t)mod_irand(s, 16);
      }
    }

    uint8_t raw_p = ext.st.dejavu.u_pitch[r_ix];
    uint8_t raw_g = ext.st.dejavu.u_gate[r_ix];
    uint16_t g_thresh = 255u - (uint16_t)s.p.arp.p[0] * 255u / 100u;
    if (st == 0 && g_thresh > 40) g_thresh -= 40;

    bool gate = (raw_g >= g_thresh);
    int16_t deg = (int16_t)(((int32_t)raw_p - 128) * 10 / 256);
    int16_t base_d = floor_mod_i16(deg, 7);
    int8_t oct = (int8_t)floor_div_i16(deg, 7);
    if (spread > 0 && mod_irand(s, 100) < spread) {
      oct = (int8_t)mod_irand(s, rng_oct);
    }
    if (oct < 0) oct = 0;
    if (oct >= rng_oct) oct = (int8_t)(rng_oct - 1);

    // STEPS salience filtering
    if (steps_mode == 3) {
      // ROOT lock
      base_d = 0;
    } else if (steps_mode == 2) {
      // Triad only
      static const int8_t kTriadSnap[7] = { 0, 0, 2, 2, 4, 4, 0 };
      base_d = kTriadSnap[base_d];
    } else if (steps_mode == 1) {
      // Pentatonic subset (strip 4th and 7th)
      if (base_d == 3) base_d = 2;
      if (base_d == 6) base_d = 4;
    } // steps_mode == 0: ALL

    bool acc = (raw_g > 200);
    bool sld = ((raw_g & 0x07u) == 0x07u);
    ext.steps[st].off   = (int8_t)base_d;
    ext.steps[st].oct   = oct;
    ext.steps[st].flags = (gate ? 1u : 0u) | (acc ? 2u : 0u) | (sld ? 4u : 0u);
  }
}

// Rebuild all 16 steps for the currently selected algorithm (0..4)
static void mod_arp_rebuild_pattern(ModSource& s) {
  ModArpExt& ext = g_mod_engine.arp_ext[s.p.arp.ext_idx];
  mod_arp_rebuild_scale(s);

  switch (s.p.arp.algo) {
    case MOD_ARP_HYP1:
      for (uint8_t st = 0; st < 16; ++st) mod_arp_roll_hyp1_step(s, ext, st);
      break;
    case MOD_ARP_MOTIF:
      mod_arp_build_motif(s, ext);
      break;
    case MOD_ARP_MARKOV:
      mod_arp_build_markov(s, ext);
      break;
    case MOD_ARP_METRO:
      mod_arp_build_metro(s, ext);
      break;
    case MOD_ARP_DEJAVU:
    default:
      mod_arp_build_dejavu(s, ext);
      break;
  }
}

// Shared ROOT and SCALE across both ARP instances for 100% harmonic lock
static void mod_arp_set_shared_root(uint8_t root_param) {
  for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
    ModSource& s = g_mod_engine.src[MOD_SRC_ARP_FIRST + k];
    s.p.arp.root_param = root_param;
    mod_arp_rebuild_scale(s);
    mod_arp_rebuild_pattern(s);
  }
}

static void mod_arp_set_shared_scale(uint8_t scale_idx) {
  for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
    ModSource& s = g_mod_engine.src[MOD_SRC_ARP_FIRST + k];
    s.p.arp.scale_idx = scale_idx;
    mod_arp_rebuild_scale(s);
    mod_arp_rebuild_pattern(s);
  }
}

// Intercepts step output on final bar of phrase cycle to inject cadential turnaround fills
static void mod_arp_apply_turnaround(ModSource& s, ModArpExt& ext, uint8_t st, ModArpStep& io_step) {
  if (ext.dd_fill == 0) return;
  const uint8_t fill_type = ext.turn_type & 3u;
  if (fill_type == 0 && st >= 8) {
    // Type 0: Ascending Chord-Tone Run (Steps 8..15)
    static const int8_t kRun[8] = { 0, 2, 4, 6, 8, 10, 12, 14 };
    io_step.flags = 1u | ((st >= 12) ? 2u : 0u) | ((st == 11 || st == 14) ? 4u : 0u);
    io_step.off = kRun[st - 8];
    io_step.oct = 0;
  } else if (fill_type == 1 && st >= 12) {
    // Type 1: Acid Descending Slid Cascade (Steps 12..15)
    static const int8_t kDesc[4] = { 7, 5, 4, 1 };
    io_step.flags = 1u | 2u | ((st != 15) ? 4u : 0u);
    io_step.off = kDesc[st - 12];
    io_step.oct = 0;
  } else if (fill_type == 2 && st >= 12) {
    // Type 2: Accelerating Octave Ratchet / Pedal Build
    io_step.flags = 1u | ((st >= 14) ? 2u : 0u);
    io_step.off = ((st & 1u) == 0) ? 0 : 7;
    io_step.oct = 0;
  } else if (fill_type == 3 && st >= 12) {
    // Type 3: Dominant Tension Tie -> Leading-Tone Pickup
    io_step.flags = 1u | ((st == 12 || st == 15) ? 2u : 0u) | 4u;
    io_step.off = (st < 15) ? 4 : -1;
    io_step.oct = 0;
  }
}

// Compute MIDI note (28..96) for a step
static uint8_t mod_arp_calc_note(const ModSource& s, const ModArpExt& ext,
                                 const ModArpStep& step) {
  const uint8_t base_root = (s.p.arp.root_param == 0)
      ? (s.p.arp.midi_root ? s.p.arp.midi_root : 48u)
      : (s.p.arp.root_param == 1 || s.p.arp.root_param == 2 || s.p.arp.root_param == 3)
      ? (s.p.arp.midi_root ? s.p.arp.midi_root : 48u)
      : s.p.arp.root_param;
  return degree_to_midi_folded(base_root, ext.deg, step.off, step.oct, ext.scale_st, ext.nsteps);
}

// ===================== engine -> adapter dispatch =====================
// Continuous destination write (active module only — its adapter state is the
// live one). Returns false when the CC is not a continuous destination there.
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

// Trigger-kind of a destination CC on a module (-1 = not a trigger there).
static int8_t mod_trigger_kind(uint8_t module_id, uint8_t cc) {
  if (cc == MOD_DEST_VOICE) return RACK_TRIG_PULSE; // universal voice pitch+gate target
  const RackModDestinations* d = kRackModules[module_id].mod_destinations;
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == cc) return static_cast<int8_t>(d->trigger_kind[i]);
  }
  return -1;
}

// Check if a destination CC is continuous on a module.
static bool mod_is_continuous(uint8_t module_id, uint8_t cc) {
  const RackModDestinations* d = kRackModules[module_id].mod_destinations;
  for (uint8_t i = 0; i < d->continuous_count; ++i) {
    if (d->continuous[i] == cc) return true;
  }
  return false;
}

// ===================== defaults (.bss is zero; these are the bench states) ==
void mod_engine_init() {
  ModEngine& e = g_mod_engine;
  memset(&e, 0, sizeof(e));
  e.bpm = 120.0f;
  e.global_clock = 0;              // internal
  e.midi_running = 0;
  e.block_period_us = 0;           // no block published yet (stamps skip the
                                   // sub-block estimate until audio runs)
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    s.type = (i < 4) ? MOD_LFO : (i < 6) ? MOD_EUCLID
           : (i < 8) ? MOD_GATE : (i < 10) ? MOD_ADSR : MOD_ARP;
    s.clock_sel = MOD_CLK_GLOBAL;
    s.division = 2;                // 1/4 default (gates override below)
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
      mod_eu_recompute(s);         // E(4,16) = 0x00001111
    } else if (s.type == MOD_GATE) {
      s.division = 4;              // 1/16 (division 4 = 1/16)
      s.p.gate.len = 16;
      s.p.gate.mode = 1;           // 01: Classic 1/8th Alternate (0xAAAA)
      s.p.gate.density = 8;
      s.p.gate.syncopation = 0;
      s.p.gate.gate_len = 0.5f;
      s.p.gate.step = 0;
      s.p.gate.ticks = 0;
      s.p.gate.rng_state = 0x12345678u + ((uint32_t)i * 0x9e3779b9u);
      s.p.gate.cur_preset = 0;
      mod_gate_recompute(s);
    } else if (s.type == MOD_ADSR) {
      s.p.adsr.a_s = 0.005f;       // 5 ms
      s.p.adsr.d_s = 0.100f;       // 100 ms
      s.p.adsr.s_lvl = 0.5f;
      s.p.adsr.r_s = 0.200f;       // 200 ms
      s.p.adsr.retrig = 0;         // gate-follow from the note stack
    } else {                       // MOD_ARP (arp1 = 10, arp2 = 11)
      s.division = 4;              // 1/16 default
      s.p.arp.ext_idx = (uint8_t)(i - MOD_SRC_ARP_FIRST);
      s.p.arp.algo = (i == MOD_SRC_ARP_FIRST) ? MOD_ARP_HYP1 : MOD_ARP_MOTIF;
      s.p.arp.range = 2;
      s.p.arp.root_param = 0;      // 0 = MIDI root follow by default
      s.p.arp.midi_root = 48;      // C3 fallback before first MIDI note
      s.p.arp.scale_idx = 6;       // Aeolian (natural minor in 20-scale table)
      s.p.arp.p[0] = 62; // p1: DENS / PROB
      s.p.arp.p[1] = 40; // p2: SLID / FORM / CNTR / PULS / DJVU
      s.p.arp.p[2] = 50; // p3: ACC / XFRM / PEDL / RTCH / STPS
      s.p.arp.p[3] = 0;  // p4: MUTE / MUTE / GRAV / ACUM / SPRD
      s.p.arp.cur_note = 48;
      ModArpExt& ext = e.arp_ext[s.p.arp.ext_idx];
      for (uint8_t k = 0; k < 16; ++k) {
        ext.st.dejavu.u_pitch[k] = (uint8_t)(128 + k * 8);
        ext.st.dejavu.u_gate[k]  = (uint8_t)(160 + (k & 3) * 20);
      }
      ext.current_pitch_q16 = 48 << 16;
      ext.target_pitch_q16  = 48 << 16;
      ext.accent_cap_q16    = 0;
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
    s.clk_phase = 0.0f;            // clocked LFOs re-phase on START (plan)
    if (s.type == MOD_EUCLID) {
      s.p.eu.step = 0;
      s.p.eu.level = (s.p.eu.mask & 1u) ? 1u : 0u;
    } else if (s.type == MOD_GATE) {
      s.p.gate.step = 0;
      s.p.gate.level = (s.p.gate.mask >> 15) & 1u;
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
// Called from loop1 BEFORE the active module's render, every non-swap block,
// at the module's block size. Skipped on swap calls (loop1 returns before
// this on a pending swap) — engine state is untouched by the swap path.
void mod_engine_process_block(uint8_t active_module) {
  const uint32_t t0 = time_us_32();
  ModEngine& e = g_mod_engine;
  const RackModuleInfo& m = kRackModules[active_module];
  // T6: the chain grid overrides the registry block/rate while engaged
  // (see the g_mod_chain_* note above); otherwise byte-identical to T3.
  const uint32_t bs = g_mod_chain_grid ? g_mod_chain_grid : m.block_size;
  const float sr = g_mod_chain_rate_khz ? g_mod_chain_rate_khz * 1000.0f
                                        : (float)m.sample_rate;
  const float dt = (float)bs / sr;                  // seconds per block

  // Publish the block position for core 0's stamping. ORDER MATTERS: the
  // counter must hold the CURRENT block's first sample for the WHOLE block
  // (the i2s.write16 loop paces the block after this pass), so start/period/
  // size go first and the counter increments last — core 0 can then read the
  // triple in any order without a torn (start, counter) pair.
  e.block_start_us = time_us_32();
  e.block_size_now = bs;
  e.block_period_us = (uint32_t)(dt * 1000000.0f + 0.5f);

  // ---- 1) drain the stamp ring (core 0 -> core 1), in order ----
  uint32_t midi_ticks = 0;          // F8s applying to THIS block
  while (e.stamp_tail != e.stamp_head) {
    const ModStamp st = e.ring[e.stamp_tail];
    e.stamp_tail = (e.stamp_tail + 1u) & (kModStampRingSize - 1u);
    if (st.status == MOD_STAMP_TICK) {
      ++e.midi_tick_count;
      ++e.telem_midi_ticks;
      if (e.midi_running) {
        ++midi_ticks;
        if (e.last_tick_valid) {
          const uint64_t d = st.pos - e.last_tick_pos;   // sample-accurate gap
          if (d && d < (uint64_t)(4.0f * sr)) {          // sanity: > 0.25 BPM
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
        e.last_tick_valid = 0;      // tick while stopped: no interval datum
      }
    } else if (st.status == MOD_STAMP_START) {
      e.midi_running = 1;
      e.midi_tick_count = 0;
      midi_ticks = 0;               // ticks before FA belonged to the old run
      e.midi_bpm_meas = 0.0f;
      e.last_tick_valid = 0;
      mod_rephase_midi_sources();
    } else if (st.status == MOD_STAMP_CONT) {
      e.midi_running = 1;           // continue WITHOUT reset
    } else {                        // MOD_STAMP_STOP
      e.midi_running = 0;           // phase frozen: no ticks reach sources
      e.last_tick_valid = 0;
    }
  }

  // ---- 2) internal clock accumulator (always runs; fractional) ----
  const float int_before = e.int_phase;
  e.int_phase += 24.0f * (e.bpm / 60.0f) * dt;
  const float int_delta = e.int_phase - int_before; // fractional PPQ ticks
  if (e.int_phase > 65536.0f) e.int_phase -= 65536.0f;  // keep float precision

  // ---- 3) sources ----
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    const bool use_midi = (s.clock_sel == MOD_CLK_MIDI) ||
        (s.clock_sel == MOD_CLK_GLOBAL && e.global_clock == 1u);
    const bool clock_running = !use_midi || e.midi_running;
    const float div_ppq = (float)kModDivisions[s.division].ppq;
    uint32_t wraps = 0;

    if (s.type == MOD_LFO && s.p.lfo.hz_mode) {
      // Free-running Hz mode: clk_phase is the LFO phase; the S&H wave
      // re-samples at a fixed 1 Hz (documented choice for Hz mode).
      s.clk_phase += s.p.lfo.rate_hz * dt;
      if (s.clk_phase >= 1.0f) s.clk_phase -= floorf(s.clk_phase);
      s.sh_acc += dt;
      if (s.sh_acc >= 1.0f) { s.sh_acc -= 1.0f; s.sh_value = mod_rand01(s); }
      float pw = s.clk_phase + s.p.lfo.phase_off;
      if (pw >= 1.0f) pw -= floorf(pw);
      s.out = mod_wave(s.p.lfo.wave, pw, s.sh_value) * s.p.lfo.level;
    } else {
      // Clocked sources: advance the division phase. Internal = the exact
      // fractional accumulator; MIDI = the integer ticks drained this block.
      const float adv = use_midi ? ((float)midi_ticks / div_ppq)
                                 : (int_delta / div_ppq);
      const float prev = s.clk_phase;
      s.clk_phase += adv;
      if (s.clk_phase >= 1.0f) s.clk_phase -= floorf(s.clk_phase);
      wraps = (uint32_t)(prev + adv);   // completed division cycles (adv < 1)

      if (s.type == MOD_LFO) {
        // Clocked-division mode: one LFO cycle per division period. MIDI-
        // clocked LFOs ramp at the measured tempo (EMA) instead of
        // stair-stepping per tick; internal uses the fractional accumulator.
        if (use_midi && e.midi_bpm_meas > 0.0f && clock_running) {
          s.clk_phase = prev + dt * (0.4f * e.midi_bpm_meas) / div_ppq;
          if (s.clk_phase >= 1.0f) s.clk_phase -= floorf(s.clk_phase);
        }
        if (s.p.lfo.wave == MOD_WAVE_SH &&
            (wraps || (use_midi && midi_ticks))) {
          s.sh_value = mod_rand01(s);   // re-sample on the clock edge
        }
      } else if (s.type == MOD_EUCLID) {
        if (wraps) {
          s.p.eu.step = (uint8_t)((s.p.eu.step + wraps) % s.p.eu.steps);
          s.p.eu.ticks += wraps;        // odometer: never reset (telemetry)
          s.p.eu.level = (s.p.eu.mask >> s.p.eu.step) & 1u;
        }
        s.out = s.p.eu.level ? 1.0f : 0.0f;
      } else if (s.type == MOD_GATE) {
        if (wraps) {
          const uint8_t prev_step = s.p.gate.step;
          s.p.gate.step = (uint8_t)((s.p.gate.step + wraps) % s.p.gate.len);
          s.p.gate.ticks += wraps;

          // Architecture 4: Mode 17 TMR (Turing Machine Shift-Register Mutation)
          if (s.p.gate.mode == 17 && s.p.gate.syncopation > 0) {
            const uint8_t s_idx = s.p.gate.step;
            const uint8_t w = (uint8_t)(15 - kBarlow16[s_idx]); // inverse Barlow indispensability
            const uint32_t prob = ((uint32_t)s.p.gate.syncopation * (uint32_t)w) / 15u;
            s.p.gate.rng_state = s.p.gate.rng_state * 1664525UL + 1013904223UL;
            const uint8_t r = (uint8_t)((s.p.gate.rng_state >> 24) & 0xFFu);
            if (r < prob) {
              s.p.gate.mask ^= (1u << (15 - s_idx)); // flip bit
            }
          }

          // Architecture 4: Mode 18 MRKV (Whole-Bar Hamming Markov Preset Transitions at bar boundaries)
          if (s.p.gate.mode == 18 && s.p.gate.step == 0 && prev_step > 0 && s.p.gate.syncopation > 0) {
            s.p.gate.rng_state = s.p.gate.rng_state * 1664525UL + 1013904223UL;
            const uint8_t r = (uint8_t)((s.p.gate.rng_state >> 24) & 0xFFu);
            if (r < s.p.gate.syncopation) {
              s.p.gate.cur_preset = mod_gate_markov_next(s.p.gate.cur_preset, s.p.gate.syncopation, s.p.gate.rng_state);
              s.p.gate.mask = kTranceGatePresets[s.p.gate.cur_preset];
            }
          }

          s.p.gate.level = (s.p.gate.mask >> (15 - s.p.gate.step)) & 1u;
        }
        // Gate length: the step's level holds only for the first gate_len
        // fraction of the division cycle (clk_phase is the in-step fraction).
        const bool high = s.p.gate.level && (s.clk_phase < s.p.gate.gate_len);
        const float target = high ? 1.0f : 0.0f;
        // Asymmetric one-pole slew filter for click-free continuous gating (~1 ms attack, ~3 ms release)
        if (target > s.out) {
          s.out += 0.4f * (target - s.out);
          if (s.out > 0.99f) s.out = 1.0f;
        } else {
          s.out += 0.2f * (target - s.out);
          if (s.out < 0.01f) s.out = 0.0f;
        }
      } else if (s.type == MOD_ADSR) {
        bool gate;
        if (s.p.adsr.retrig) {
          // Clock-retrigger mode: the envelope re-fires on every division
          // tick and holds the gate until the next tick (full-step gate: set
          // sustain 0 for an AD pluck loop). Freezes on STOP.
          if (wraps) s.p.adsr.ticks += wraps;
          gate = clock_running && s.p.adsr.ticks;
        } else {
          gate = shared_gate;           // gate-follow from the shared note stack
        }
        if (gate) {
          if (s.p.adsr.env_state == MOD_ENV_IDLE ||
              s.p.adsr.env_state == MOD_ENV_R) {
            s.p.adsr.env_state = MOD_ENV_A;   // re-gate restarts the attack
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
      } else {  // MOD_ARP (Switchable 5-Algo Generative Melody & Gate Generator)
        ModArpExt& ext = e.arp_ext[s.p.arp.ext_idx];
        // Capture external MIDI note-ons when in MIDI Follow mode (root_param == 0 or 1)
        const uint8_t sn = (uint8_t)(shared_note + 0.5f);
        if (sn >= 12 && sn <= 108 && sn != s.p.arp.last_out_note) {
          if (s.p.arp.root_param == 0) {
            // Diatonic Harmonizer: find nearest scale degree in active key
            uint8_t pc = sn % 12u;
            uint8_t base_pc = s.p.arp.midi_root % 12u;
            uint8_t rel_semi = (uint8_t)((pc + 12u - base_pc) % 12u);
            uint8_t best_deg = 0;
            uint8_t min_err = 12;
            for (uint8_t k = 0; k < ext.nsteps; ++k) {
              uint8_t err = (rel_semi >= ext.scale_st[k]) ? (rel_semi - ext.scale_st[k]) : (ext.scale_st[k] - rel_semi);
              if (err < min_err) { min_err = err; best_deg = k; }
            }
            ext.deg = (int8_t)best_deg;
            for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
              e.arp_ext[k].deg = (int8_t)best_deg;
            }
          } else if (s.p.arp.root_param == 1) {
            // Berlin School parallel chromatic transpose
            for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
              e.src[MOD_SRC_ARP_FIRST + k].p.arp.midi_root = sn;
              e.arp_ext[k].deg = 0;
            }
          } else {
            for (uint8_t k = 0; k < MOD_ARP_COUNT; ++k) {
              e.src[MOD_SRC_ARP_FIRST + k].p.arp.midi_root = sn;
            }
          }
        }
        if (wraps && clock_running) {
          const uint8_t prev_st = s.p.arp.step;
          s.p.arp.step = (uint8_t)((s.p.arp.step + wraps) & 0x0Fu);
          s.p.arp.ticks += wraps;
          const uint8_t st = s.p.arp.step;

          // Bar-line wrap (step 0): chord drift, mutation, and fill-bar state
          if (st < prev_st || (s.p.arp.ticks & 0x0Fu) == 0u) {
            ext.bar_count++;
            const uint8_t drift_bars = (s.p.arp.p[3] >= 60) ? 4 : 8;
            if (ext.bar_count >= drift_bars) {
              ext.bar_count = 0;
              if (s.p.arp.p[3] > 15 || s.p.arp.root_param >= 2) mod_arp_next_chord(s);
              ext.dd_fill = 0;
              ext.turn_type = (uint8_t)((s.p.arp.ticks >> 4) & 3u);
            } else if (ext.bar_count == drift_bars - 1 && s.p.arp.p[3] >= 40) {
              ext.dd_fill = 1; // Fill bar right before chord/pattern change
              ext.turn_type = (uint8_t)((s.p.arp.ticks >> 4) & 3u);
            }

            if (s.p.arp.algo == MOD_ARP_HYP1) {
              // HYP-1 ac_mutate: in-place step mutation on loop wrap (driven by p[3])
              const uint8_t mut_prob = (uint8_t)((uint16_t)s.p.arp.p[3] * 50 / 100);
              for (uint8_t k = 0; k < 16; ++k) {
                if (mod_irand(s, 100) < mut_prob) mod_arp_roll_hyp1_step(s, ext, k);
              }
            } else {
              // Refresh variations on the bar
              mod_arp_rebuild_pattern(s);
            }
          }

          // Evaluate step (or cadential turnaround fill on fill bars)
          ModArpStep cur_step = ext.steps[st];
          if (ext.dd_fill != 0) {
            mod_arp_apply_turnaround(s, ext, st, cur_step);
          }

          if (cur_step.flags & 1u) {
            const ModArpStep& prev_step = ext.steps[prev_st];
            const bool prev_slid = ((prev_step.flags & 5u) == 5u) && s.p.arp.held;
            const uint8_t next_note = mod_arp_calc_note(s, ext, cur_step);
            // Retrigger edge fires on fresh attack, suppressed on legato slide
            s.p.arp.retrig_edge = prev_slid ? 0u : 1u;
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

        // 303 Constant-Time Pitch Glide (~55ms exponential slew)
        const int32_t target_q16 = ((int32_t)s.p.arp.cur_note) << 16;
        if (s.p.arp.retrig_edge || !s.p.arp.held) {
          ext.current_pitch_q16 = target_q16;
          ext.target_pitch_q16  = target_q16;
        } else {
          ext.target_pitch_q16 = target_q16;
          int32_t diff = ext.target_pitch_q16 - ext.current_pitch_q16;
          if (diff != 0) {
            // Slew alpha per block (~32 samples at 32kHz -> alpha ~= 0.038 ~= 2480 / 65536)
            ext.current_pitch_q16 += (int32_t)(((int64_t)diff * 2480) >> 16);
          }
        }

        // C13 Accent Capacitor Stacking simulation
        if (s.p.arp.cur_acc && s.p.arp.level) {
          ext.accent_cap_q16 += 16384;
          if (ext.accent_cap_q16 > 65536) ext.accent_cap_q16 = 65536;
        }
        ext.accent_cap_q16 = (int32_t)(((int64_t)ext.accent_cap_q16 * 63000) >> 16);

        // Gate window: 100% if slide/hold bit is set, else 55% (HYP-1 303 gate)
        const float g_win = s.p.arp.cur_hold ? 1.0f : 0.55f;
        const bool gate_open = clock_running && s.p.arp.level && (s.clk_phase < g_win);
        const float acc_mod = (float)ext.accent_cap_q16 / 131072.0f;
        const float base_target = gate_open ? (s.p.arp.cur_acc ? 1.0f : 0.72f) : 0.0f;
        const float target = mod_clamp01(base_target + (gate_open ? acc_mod : 0.0f));
        // Click-free asymmetric slew (~2 ms attack, ~12 ms release) for continuous CCs
        const float slew = (target > s.out) ? (dt / 0.002f) : (dt / 0.012f);
        s.out += (target - s.out) * (slew > 1.0f ? 1.0f : slew);
      }

      // Clocked-LFO wave output (the other types wrote s.out in their case).
      if (s.type == MOD_LFO) {
        float pw = s.clk_phase + s.p.lfo.phase_off;
        if (pw >= 1.0f) pw -= floorf(pw);
        s.out = mod_wave(s.p.lfo.wave, pw, s.sh_value) * s.p.lfo.level;
      }
    }

    // ---- 4) destination routing (resolved against the ACTIVE module) ----
    const uint8_t dest = s.dest_cc;
    if (dest == 0) { s.was_high = 0; continue; }
    const bool is_cont = mod_is_continuous(active_module, dest);
    const int8_t kind = mod_trigger_kind(active_module, dest);

    if (s.type == MOD_LFO || s.type == MOD_ADSR) {
      if (is_cont) {
        mod_write_cc(active_module, dest, s.out);     // continuous, float-level
      } else {
        s.was_high = 0;
      }
    } else if (s.type == MOD_ARP) {
      ModArpExt& ext = e.arp_ext[s.p.arp.ext_idx];
      const int8_t trig_kind = mod_trigger_kind(active_module, dest);
      const bool is_voice_pitch = (dest == 1 || dest == 42 || dest == MOD_DEST_VOICE);
      if (trig_kind >= 0 || is_voice_pitch) {
        const float g_win = s.p.arp.cur_hold ? 1.0f : 0.55f;
        const bool high = clock_running && s.p.arp.level && (s.clk_phase < g_win);
        if (high) {
          s.p.arp.last_out_note = s.p.arp.cur_note;
          const float slewed_note = (float)ext.current_pitch_q16 / 65536.0f;
          shared_note = (slewed_note >= 12.0f && slewed_note <= 108.0f) ? slewed_note : (float)s.p.arp.cur_note;
          shared_gate = 1;
        } else if (s.was_high) {
          shared_gate = 0;
        }
        if (trig_kind == RACK_TRIG_LATCH) {
          if (high != (bool)s.was_high) mod_fire_trigger(active_module, dest, high);
        } else if (wraps && s.p.arp.level && s.p.arp.retrig_edge) {
          // Fire module trigger (CC 64 on Plaits/Rings/Elements, CC 66 on Clouds)
          const uint8_t trig_cc = (trig_kind >= 0 && dest != MOD_DEST_VOICE) ? dest
                                : (active_module == RACK_MODULE_CLOUDS ? 66 : 64);
          mod_fire_trigger(active_module, trig_cc, true);
        }
        s.was_high = high ? 1u : 0u;
      } else {
        mod_write_cc(active_module, dest, s.out);
      }
    } else if (s.type == MOD_GATE) {
      if (is_cont) {
        mod_write_cc(active_module, dest, s.out);     // rhythmic gate level (0.0 / 1.0, smoothed)
      } else if (kind >= 0) {
        const bool high = (s.p.gate.level && (s.clk_phase < s.p.gate.gate_len));
        if (kind == RACK_TRIG_LATCH) {
          if (high != (bool)s.was_high) {
            mod_fire_trigger(active_module, dest, high);
          }
        } else {
          if (high && !s.was_high) {
            mod_fire_trigger(active_module, dest, true);
          }
        }
        s.was_high = high ? 1u : 0u;
      } else {
        s.was_high = 0;
      }
    } else if (s.type == MOD_EUCLID) {
      if (is_cont) {
        mod_write_cc(active_module, dest, s.out);     // euclidean level (0.0 / 1.0)
      } else if (kind >= 0) {
        if (kind == RACK_TRIG_LATCH) {
          const bool high = (s.p.eu.level != 0);
          if (high != (bool)s.was_high) {
            mod_fire_trigger(active_module, dest, high);
          }
          s.was_high = high ? 1u : 0u;
        } else {
          if (wraps && s.p.eu.level) {
            mod_fire_trigger(active_module, dest, true);
          }
          s.was_high = s.p.eu.level ? 1u : 0u;
        }
      } else {
        s.was_high = 0;
      }
    }
  }

  // Counter publish LAST (see the ordering note at the top).
  e.sample_counter += bs;

  const uint32_t dt_us = time_us_32() - t0;
  if (dt_us > e.mod_us_max) e.mod_us_max = dt_us;   // the 50 us gate metric
}

// ===================== core 0: stamping funnel =====================
// ONE entry for every incoming clock byte (device readPacket + PIO-USB host
// poll + T4's DIN parser all funnel through midiHandleEventPacket -> here).
// Runs on core 0; writes only the stamp ring + reads the published triple.
static uint64_t mod_read_sample_counter() {   // tear-safe 64-bit read
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
  // Stamp = current render sample position + sub-block estimate from the us
  // delta since core 1 published the block start (see the precision story in
  // the header). Before audio runs (period 0) the estimate is skipped.
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
// Sticky selection: CC 33 latches the source, CC 34 latches the parameter,
// CC 35 applies the value to (source, param). All engine scalars written here
// are aligned 32-bit atomics read by core 1 (same benign-race class as the
// existing panel CC writes). Works during swaps (engine state is swap-safe).
static uint8_t mod_param_count_of(uint8_t type) {
  return kModParamCount[type < MOD_TYPE_COUNT ? type : 0];
}

// Canonical source names (also used by the dest ack + `mod <src>` dumps).
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

// Destination set + one-shot validation log against the ACTIVE module.
static void mod_set_dest(uint8_t src_idx, uint8_t cc) {
  preset_mark_dirty();            // T4a: routing is persistent state
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  const uint8_t old_cc = s.dest_cc;
  s.dest_cc = cc;
  s.was_high = 0;                 // no stale edge into the new destination

  // If old destination was routed and no other source is driving it,
  // restore panel's static value from cache.
  if (old_cc != 0 && old_cc != cc) {
    if (s.type == MOD_ARP) shared_gate = 0;
    bool other_routed = false;
    for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
      if (i != src_idx && e.src[i].dest_cc == old_cc) { other_routed = true; break; }
    }
    if (!other_routed) {
      ui_link_restore_cached_cc(g_active_module, old_cc);
    }
  }

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
  const bool is_voice_pitch = (cc == 1 || cc == 42 || cc == MOD_DEST_VOICE);
  if (s.type == MOD_ARP && (trigger_ok || is_voice_pitch || continuous_ok)) {
    Serial.printf("mod: %s -> %s CC %u (%s)\r\n", mod_src_name(src_idx), m.name,
                  cc, (trigger_ok || is_voice_pitch) ? "voice pitch+gate+trig" : "slewed gate CV");
    return;
  }
  const bool can_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR ||
                              s.type == MOD_GATE || s.type == MOD_EUCLID);
  const bool can_trigger = (s.type == MOD_EUCLID || s.type == MOD_GATE);

  if (can_continuous && continuous_ok) {
    Serial.printf("mod: %s -> %s CC %u (continuous): the panel's static value "
                  "is overridden while routed\r\n",
                  mod_src_name(src_idx), m.name, cc);
  } else if (can_trigger && trigger_ok) {
    Serial.printf("mod: %s -> %s CC %u (trigger)\r\n",
                  mod_src_name(src_idx), m.name, cc);
  } else {
    Serial.printf("mod: WARNING: %s -> CC %u is INERT on %s (%s source cannot drive %s)\r\n",
                  mod_src_name(src_idx), cc, m.name, mod_src_name(src_idx),
                  (continuous_ok ? "continuous CC" : (trigger_ok ? "trigger input" : "unknown CC")));
  }
}

static inline uint8_t mod_clk_from_cc(uint8_t v) {
  return v < 43 ? MOD_CLK_GLOBAL : (v < 85 ? MOD_CLK_INTERNAL : MOD_CLK_MIDI);
}

static inline float mod_adsr_s_from_cc(uint8_t v) {   // 1 ms .. 10 s, exponential
  return powf(10.0f, -3.0f + 4.0f * (float)v / 127.0f);
}

// Apply a bench CC value (0-127) to (source, param). Silent per move (a knob
// sweep would spam); `mod?` shows the resulting state, dest changes ack.
static void mod_apply_cc_value(uint8_t src_idx, uint8_t param, uint8_t val) {
  preset_mark_dirty();            // T4a: persistent state changed
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  if (s.type == MOD_ARP && (param == 10 || param == 11)) {
    if (param == 10) {
      uint8_t r = (val <= 3) ? val : (uint8_t)(24 + ((uint16_t)(val - 4) * 60) / 123);
      mod_arp_set_shared_root(r);
    } else {
      uint8_t sc = (val < 64) ? (uint8_t)(((uint16_t)val * SCALE_COUNT) / 64)
                              : (uint8_t)(0x40u | (((uint16_t)(val - 64) * SCALE_COUNT) / 64));
      mod_arp_set_shared_scale(sc);
    }
    return;
  }
  const uint8_t n = mod_param_count_of(s.type);
  if (param >= n) return;                       // CC 34 beyond the table
  const uint8_t tail = n - 3;                   // uniform clk/div/dest tail
  if (param == tail) {                          // clk
    s.clock_sel = mod_clk_from_cc(val);
    return;
  }
  if (param == tail + 1) {                      // div
    const uint8_t d = (uint16_t)val * MOD_DIVISION_COUNT / 128;
    s.division = d < MOD_DIVISION_COUNT ? d : MOD_DIVISION_COUNT - 1;
    return;
  }
  if (param == tail + 2) { mod_set_dest(src_idx, val); return; }
  switch (s.type) {
    case MOD_LFO:
      switch (param) {
        case 0: s.p.lfo.hz_mode = val < 64 ? 1 : 0; break;   // 0-63 Hz / 64+ clocked
        case 1: s.p.lfo.rate_hz = 0.02f * powf(1000.0f, val / 127.0f); break;
        case 2: {
          const uint8_t w = (uint16_t)val * MOD_WAVE_SH_COUNT / 128;
          s.p.lfo.wave = w < MOD_WAVE_SH_COUNT ? w : MOD_WAVE_SH_COUNT - 1;
          break;
        }
        case 3: s.p.lfo.phase_off = (float)val / 127.0f; break;  // 0..360 deg
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
        case 0: { // mode: 0 = GEN, 1..16 = PRESETS, 17 = TMR, 18 = MRKV
          uint8_t m = (uint8_t)(((uint16_t)val * 19) / 128);
          s.p.gate.mode = m <= 18 ? m : 18;
          mod_gate_recompute(s);
          break;
        }
        case 1: { // dens: 1..16
          s.p.gate.density = (uint8_t)(1 + ((uint16_t)val * 15) / 127);
          mod_gate_recompute(s);
          break;
        }
        case 2: { // sync: 0..255 (0..100%)
          s.p.gate.syncopation = (uint8_t)(((uint16_t)val * 255) / 127);
          mod_gate_recompute(s);
          break;
        }
        case 3: { // glen: 0..1.0
          s.p.gate.gate_len = (1.0f + (float)val) / 128.0f;
          break;
        }
        default: break;
      }
      break;
    case MOD_ADSR:
      switch (param) {
        case 0: s.p.adsr.retrig = val < 64 ? 0 : 1; break;   // gate / retrig
        case 1: s.p.adsr.a_s = mod_adsr_s_from_cc(val); break;
        case 2: s.p.adsr.d_s = mod_adsr_s_from_cc(val); break;
        case 3: s.p.adsr.s_lvl = val / 127.0f; break;
        case 4: s.p.adsr.r_s = mod_adsr_s_from_cc(val); break;
        default: break;
      }
      break;
    default:  // MOD_ARP
      switch (param) {
        case 0: { // algo (0..14 choices = 5 algos x 3 octave ranges)
          uint8_t choice = (uint8_t)(((uint16_t)val * 15) / 128);
          if (choice > 14) choice = 14;
          s.p.arp.algo = choice / 3;
          s.p.arp.range = (uint8_t)(1 + (choice % 3));
          mod_arp_rebuild_pattern(s);
          break;
        }
        case 1: // p1 (0..100)
          s.p.arp.p[0] = (uint8_t)(((uint16_t)val * 100 + 63) / 127);
          mod_arp_rebuild_pattern(s);
          break;
        case 2: // p2 (0..100)
          s.p.arp.p[1] = (uint8_t)(((uint16_t)val * 100 + 63) / 127);
          mod_arp_rebuild_pattern(s);
          break;
        case 3: // p3 (0..100)
          s.p.arp.p[2] = (uint8_t)(((uint16_t)val * 100 + 63) / 127);
          mod_arp_rebuild_pattern(s);
          break;
        case 4: // p4 (0..100)
          s.p.arp.p[3] = (uint8_t)(((uint16_t)val * 100 + 63) / 127);
          mod_arp_rebuild_pattern(s);
          break;
        default: break;
      }
      break;
  }
}

void rack_mod_bench_cc(uint8_t cc, uint8_t val) {
  static uint8_t sel_source = 0;   // sticky bench selection (core 0 only)
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
// Called by ui_link.h (main.cpp TU) for 0x03 MOD_SET frames [src:u8][param:u8]
// [value:u8]. The param id IS the kModParamNames index — the SAME tables the
// console `mod` command names and the CC 34 mux uses — and the value is
// applied through mod_apply_cc_value, i.e. the bench-mux validation and
// 0-127 quantization verbatim (dest changes still ack through mod_set_dest's
// one-shot validation log). One log line per set: the C3 gesture rate is low
// and the acks are the stage-1 bring-up evidence.
void mod_engine_apply_ui_set(uint8_t src, uint8_t param, uint8_t val) {
  if (src >= MOD_SOURCE_COUNT) {
    Serial.printf("mod: ui set REJECTED (source %u >= %u)\r\n", src,
                  MOD_SOURCE_COUNT);
    return;
  }
  if (g_mod_engine.src[src].type == MOD_ARP && (param == 10 || param == 11)) {
    Serial.printf("mod: ui set %s %s <- %u\r\n", mod_src_name(src),
                  param == 10 ? "shared root" : "shared scale", val);
    mod_apply_cc_value(src, param, val);
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
// mod <src> <param> <value> | mod? | bpm <20-300> | clock <internal|midi> [src]
// All core-0 Serial prints (the established console discipline).

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

// Console-side param application (natural units; see the mod? dump + help).
// Returns false + prints when the value doesn't parse.
static bool mod_set_param_console(uint8_t src_idx, uint8_t param,
                                  const char* tok) {
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  const uint8_t n = mod_param_count_of(s.type);
  if (param >= n && param != 10 && param != 11 && param != 99) return false;
  preset_mark_dirty();            // T4a: persistent state changed
  if (param < n) {
    const uint8_t tail = n - 3;
    if (param == tail) {                                   // clk
      const int8_t c = mod_parse_clk(tok);
      if (c < 0) return false;
      s.clock_sel = (uint8_t)c;
      return true;
    }
    if (param == tail + 1) {                               // div
      const int8_t d = mod_parse_div(tok);
      if (d < 0) return false;
      s.division = (uint8_t)d;
      return true;
    }
    if (param == tail + 2) {                               // dest
      if (!strcasecmp(tok, "off")) { mod_set_dest(src_idx, 0); return true; }
      if (!strcasecmp(tok, "voice")) { mod_set_dest(src_idx, MOD_DEST_VOICE); return true; }
      if (tok[0] >= '0' && tok[0] <= '9') {
        const long v = atol(tok);
        if (v >= 0 && v <= 127) { mod_set_dest(src_idx, (uint8_t)v); return true; }
      }
      return false;
    }
  }
  const float v = (float)atof(tok);                      // numeric fallback
  switch (s.type) {
    case MOD_LFO:
      switch (param) {
        case 0:                                          // mode
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
          s.p.lfo.rate_hz = v;                     // Hz, clamped below
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
          s.p.lfo.phase_off = mod_clamp01(v / 360.0f);   // degrees in
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
      if (param == 99) { // raw mask alias
        s.p.gate.mask = (uint32_t)strtoul(tok, nullptr, 0);
        s.p.gate.mode = 0;
        return true;
      }
      switch (param) {
        case 0: { // mode
          if (!strcasecmp(tok, "gen") || !strcasecmp(tok, "barl")) {
            s.p.gate.mode = 0;
          } else if (!strcasecmp(tok, "tmr") || !strcasecmp(tok, "shft")) {
            s.p.gate.mode = 17;
          } else if (!strcasecmp(tok, "mrkv") || !strcasecmp(tok, "markov")) {
            s.p.gate.mode = 18;
          } else {
            int m = -1;
            for (uint8_t p = 0; p < 16; ++p) {
              if (!strcasecmp(tok, kTrancePresetNames[p + 1])) { m = p + 1; break; }
            }
            if (m >= 0) s.p.gate.mode = (uint8_t)m;
            else s.p.gate.mode = (uint8_t)(v < 0 ? 0 : (v > 18 ? 18 : v));
          }
          mod_gate_recompute(s);
          return true;
        }
        case 1: { // dens: 1..16
          s.p.gate.density = (uint8_t)(v < 1 ? 1 : (v > 16 ? 16 : v));
          mod_gate_recompute(s);
          return true;
        }
        case 2: { // sync: 0..100 or 0..255
          s.p.gate.syncopation = (uint8_t)(v <= 100.0f ? (uint8_t)(v * 2.55f + 0.5f) : (v > 255 ? 255 : v));
          mod_gate_recompute(s);
          return true;
        }
        case 3: // glen: 0.0..1.0
          s.p.gate.gate_len = mod_clamp01(v);
          return true;
        default: return false;
      }
    case MOD_ADSR:
      switch (param) {
        case 0:                                          // mode
          if (!strcasecmp(tok, "gate") || !strcasecmp(tok, "follow")) {
            s.p.adsr.retrig = 0; return true;
          }
          if (!strcasecmp(tok, "retrig") || !strcasecmp(tok, "clock")) {
            s.p.adsr.retrig = 1; return true;
          }
          s.p.adsr.retrig = v >= 0.5f ? 1 : 0;
          return true;
        case 1: s.p.adsr.a_s = v / 1000.0f;              // ms in
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
      if (param == 10) { // shared root
        if (!strcasecmp(tok, "midi") || !strcasecmp(tok, "follow") || !strcasecmp(tok, "harm")) {
          mod_arp_set_shared_root(0); return true;
        }
        if (!strcasecmp(tok, "parallel") || !strcasecmp(tok, "paral")) {
          mod_arp_set_shared_root(1); return true;
        }
        if (!strcasecmp(tok, "w7") || !strcasecmp(tok, "drift")) {
          mod_arp_set_shared_root(2); return true;
        }
        if (!strcasecmp(tok, "neo") || !strcasecmp(tok, "plr")) {
          mod_arp_set_shared_root(3); return true;
        }
        uint8_t r = (uint8_t)(v <= 3 ? v : (v < 24 ? 24 : (v > 84 ? 84 : v)));
        mod_arp_set_shared_root(r);
        return true;
      }
      if (param == 11) { // shared scale
        static const char* const kScNames[SCALE_COUNT] = {
            "chrom", "ionian", "dorian", "phrygian", "lydian", "mixo", "aeolian", "locrian",
            "pentmaj", "pentmin", "blues", "harmmin", "melmin", "phrygdom", "wholetone",
            "octatonic", "hirajoshi", "insen", "doubleharm", "acidpool"
        };
        for (uint8_t i = 0; i < SCALE_COUNT; ++i) {
          if (!strcasecmp(tok, kScNames[i])) {
            mod_arp_set_shared_scale(i); return true;
          }
        }
        uint8_t sc = (uint8_t)(v < 0 ? 0 : (v >= SCALE_COUNT ? (SCALE_COUNT - 1) : v));
        mod_arp_set_shared_scale(sc);
        return true;
      }
      switch (param) {
        case 0: // algo
          if (!strcasecmp(tok, "hyp1") || !strcasecmp(tok, "acid")) s.p.arp.algo = MOD_ARP_HYP1;
          else if (!strcasecmp(tok, "motif") || !strcasecmp(tok, "abac") || !strcasecmp(tok, "phrase")) s.p.arp.algo = MOD_ARP_MOTIF;
          else if (!strcasecmp(tok, "markov") || !strcasecmp(tok, "gravity") || !strcasecmp(tok, "walk")) s.p.arp.algo = MOD_ARP_MARKOV;
          else if (!strcasecmp(tok, "metro") || !strcasecmp(tok, "m185") || !strcasecmp(tok, "accum")) s.p.arp.algo = MOD_ARP_METRO;
          else if (!strcasecmp(tok, "dejavu") || !strcasecmp(tok, "marbles") || !strcasecmp(tok, "shift")) s.p.arp.algo = MOD_ARP_DEJAVU;
          else s.p.arp.algo = (uint8_t)(v < 0 ? 0 : (v > 4 ? 4 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        case 1: // p1 (0..100)
          s.p.arp.p[0] = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        case 2: // p2 (0..100)
          s.p.arp.p[1] = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        case 3: // p3 (0..100)
          s.p.arp.p[2] = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
          mod_arp_rebuild_pattern(s);
          return true;
        case 4: // p4 (0..100)
          s.p.arp.p[3] = (uint8_t)(v < 0 ? 0 : (v > 100 ? 100 : v));
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
  if (s.type == MOD_GATE && !strcasecmp(tok, "mask")) return 99; // raw mask alias
  if (s.type == MOD_ARP) {
    if (!strcasecmp(tok, "root")) return 10;
    if (!strcasecmp(tok, "scale") || !strcasecmp(tok, "sc")) return 11;
    if (!strcasecmp(tok, "dens") || !strcasecmp(tok, "prob")) return 1; // p1
    if (!strcasecmp(tok, "slide") || !strcasecmp(tok, "form") || !strcasecmp(tok, "contour") ||
        !strcasecmp(tok, "pulses") || !strcasecmp(tok, "dejavu")) return 2; // p2
    if (!strcasecmp(tok, "accent") || !strcasecmp(tok, "acc") || !strcasecmp(tok, "xform") ||
        !strcasecmp(tok, "pedal") || !strcasecmp(tok, "ratchet") || !strcasecmp(tok, "steps")) return 3; // p3
    if (!strcasecmp(tok, "mutate") || !strcasecmp(tok, "mut") || !strcasecmp(tok, "grav") ||
        !strcasecmp(tok, "accum") || !strcasecmp(tok, "spread")) return 4; // p4
  }
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
    const char* pname = (param < mod_param_count_of(g_mod_engine.src[src].type))
        ? kModParamNames[g_mod_engine.src[src].type][param]
        : (param == 10 ? "root" : (param == 11 ? "scale" : "mask"));
    Serial.printf("mod: bad value '%s' for %s\r\n", tok, pname);
  }
}

// mod? — the full engine dump (clock bus + all 12 sources).
static void mod_print_dest_state(uint8_t src_idx) {
  ModEngine& e = g_mod_engine;
  const ModSource& s = e.src[src_idx];
  if (s.dest_cc == 0) {
    Serial.print("dest=0(off)");
    return;
  }
  const RackModuleInfo& m = kRackModules[g_active_module];
  const RackModDestinations* d = m.mod_destinations;
  if (s.dest_cc == MOD_DEST_VOICE) {
    Serial.printf("dest=127(voice on %s)", m.name);
    return;
  }
  const bool is_voice_pitch = (s.dest_cc == 1 || s.dest_cc == 42);
  const bool can_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR ||
                              s.type == MOD_GATE || s.type == MOD_EUCLID ||
                              s.type == MOD_ARP);
  const bool can_trigger = (s.type == MOD_EUCLID || s.type == MOD_GATE ||
                            s.type == MOD_ARP);
  bool cont = false, trig = false;
  for (uint8_t i = 0; i < d->continuous_count; ++i) {
    if (d->continuous[i] == s.dest_cc) { cont = true; break; }
  }
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == s.dest_cc) { trig = true; break; }
  }
  char tag[16];
  if (s.type == MOD_ARP && (is_voice_pitch || trig)) snprintf(tag, sizeof(tag), "voice+trig");
  else if (s.type == MOD_ARP && cont) snprintf(tag, sizeof(tag), "gate-cv");
  else if (can_continuous && cont) snprintf(tag, sizeof(tag), "cont");
  else if (can_trigger && trig) snprintf(tag, sizeof(tag), "trig");
  else snprintf(tag, sizeof(tag), "INERT!");
  Serial.printf("dest=%u(%s on %s)", s.dest_cc, tag, m.name);
}

static const char* mod_print_fixed(char* buf, size_t n, float v) {  // "x.yy"
  if (v < 0.0f) v = 0.0f;
  const uint32_t scaled = (uint32_t)(v * 100.0f + 0.5f);
  snprintf(buf, n, "%lu.%02lu", (unsigned long)(scaled / 100u),
           (unsigned long)(scaled % 100u));
  return buf;
}

void mod_engine_cmd_modq(const char* args) {
  (void)args;
  ModEngine& e = g_mod_engine;
  char b1[16], b2[16], b3[16], b4[16], b5[16];  // distinct: snprintf evaluates
                                                // every arg BEFORE printing
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
        Serial.printf("mode=%-4s dens=%2u sync=%3u%% mask=0x%04x glen=%s step=%u ticks=%lu out=%s",
                      kTrancePresetNames[s.p.gate.mode <= 18 ? s.p.gate.mode : 0],
                      s.p.gate.density,
                      (unsigned int)((s.p.gate.syncopation * 100 + 127) / 255),
                      (unsigned int)s.p.gate.mask,
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
        static const char* const kAlgo[5] = {"hyp1", "motif", "markov", "metro", "dejavu"};
        static const char* const kSc[SCALE_COUNT] = {
            "chrom", "ionian", "dorian", "phryg", "lydian", "mixo", "aeol", "locr",
            "pmaj", "pmin", "blues", "hmin", "mmin", "pdom", "wtone",
            "oct", "hira", "insen", "dharm", "acid"
        };
        const ModArpExt& ext = e.arp_ext[s.p.arp.ext_idx];
        const char* root_str = (s.p.arp.root_param == 0) ? "harm"
                             : (s.p.arp.root_param == 1) ? "paral"
                             : (s.p.arp.root_param == 2) ? "w7"
                             : (s.p.arp.root_param == 3) ? "neo" : "fix";
        Serial.printf("algo=%s(%uoct) root=%s(%u) scale=%s p1=%u p2=%u p3=%u p4=%u deg=%d note=%u step=%u out=%s",
                      kAlgo[s.p.arp.algo % 5], s.p.arp.range,
                      root_str,
                      (s.p.arp.root_param <= 3) ? s.p.arp.midi_root : s.p.arp.root_param,
                      kSc[(s.p.arp.scale_idx & 0x3Fu) % SCALE_COUNT],
                      s.p.arp.p[0], s.p.arp.p[1], s.p.arp.p[2], s.p.arp.p[3], (int)ext.deg,
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
  preset_mark_dirty();            // T4a: clock settings are persistent state
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
    preset_mark_dirty();          // T4a: clock settings are persistent state
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
  preset_mark_dirty();            // T4a: per-source clock override persists
  Serial.printf("mod: source %s clock override = %s\r\n",
                mod_src_name((uint8_t)src), g ? "midi" : "internal");
}

// ===================== telemetry: the 2 s report tail =====================
// Appended to main.cpp's report line: clock source + BPM (internal) or the
// measured MIDI tick rate (midi; 'x' = transport stopped), the EU1 division-
// tick ODOMETER (never reset — swap/phase survival), LFO1's phase byte
// (0-255 around its cycle), the engine's own block-cost max (the 50 us gate)
// and one state char per source:
//   LFO:  h = free-Hz running, L = clocked+clock running, . = clock stopped
//   EU:   e = clock running, . = stopped     GATE: g / .
//   ADSR: A = envelope active, a = idle
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
             (unsigned long)(e.telem_midi_ticks / 2u),   // ticks/s over ~2 s
             (unsigned long)e.src[MOD_SRC_EU_FIRST].p.eu.ticks,
             (unsigned)phase, (unsigned long)e.mod_us_max, states);
  } else {
    snprintf(buf, n, "modeng=m:x e:%lu p:%02x u:%lu [%s]",
             (unsigned long)e.src[MOD_SRC_EU_FIRST].p.eu.ticks,
             (unsigned)phase, (unsigned long)e.mod_us_max, states);
  }
}

// ===================== T4a preset serialization (clock bus + sources) ====
// The preset blob holds this engine's PERSISTENT state only — never the
// per-block transients (phases, envelope states, odometers, S&H values).
// Layout per source record (28 B): [type u8][clock_sel u8][division u8]
// [dest_cc u8] + a 24-byte type payload written field-by-field (LE for the
// u32 mask, memcpy for floats — ARM LE both ends, and ONLY the RP2350 ever
// parses the blob; the C3 stores it opaquely). preset.h owns the blob
// framing around these records; both sides must stay in lockstep.
static const uint8_t kModPresetSrcRecord = 28;

// Append clock bus + all source records at the cursor. Returns bytes written.
static uint16_t mod_engine_preset_write(uint8_t* p) {
  uint8_t* w = p;
  ModEngine& e = g_mod_engine;
  *w++ = e.global_clock;
  const float bpm = e.bpm;
  memcpy(w, &bpm, 4); w += 4;                      // f32 LE
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    const ModSource& s = e.src[i];
    *w++ = s.type; *w++ = s.clock_sel; *w++ = s.division; *w++ = s.dest_cc;
    uint8_t pl[24];
    memset(pl, 0, sizeof(pl));
    switch (s.type) {
      case MOD_LFO:
        memcpy(pl + 0, &s.p.lfo.rate_hz, 4);
        memcpy(pl + 4, &s.p.lfo.level, 4);
        memcpy(pl + 8, &s.p.lfo.phase_off, 4);
        pl[12] = s.p.lfo.wave; pl[13] = s.p.lfo.hz_mode;
        break;
      case MOD_EUCLID:
        pl[0] = s.p.eu.steps; pl[1] = s.p.eu.pulses; pl[2] = s.p.eu.rot;
        memcpy(pl + 4, &s.p.eu.mask, 4);
        pl[8] = s.p.eu.level;
        break;
      case MOD_GATE:
        pl[0] = s.p.gate.mode; pl[1] = s.p.gate.density;
        pl[2] = s.p.gate.syncopation; pl[3] = s.p.gate.len;
        memcpy(pl + 4, &s.p.gate.mask, 4);
        memcpy(pl + 8, &s.p.gate.gate_len, 4);
        pl[12] = s.p.gate.cur_preset;
        break;
      case MOD_ADSR:
        memcpy(pl + 0, &s.p.adsr.a_s, 4);
        memcpy(pl + 4, &s.p.adsr.d_s, 4);
        memcpy(pl + 8, &s.p.adsr.s_lvl, 4);
        memcpy(pl + 12, &s.p.adsr.r_s, 4);
        pl[16] = s.p.adsr.retrig;
        break;
      default: // MOD_ARP
        pl[0] = s.p.arp.algo;
        pl[1] = s.p.arp.range;
        pl[2] = s.p.arp.root_param;
        pl[3] = s.p.arp.scale_idx;
        pl[4] = s.p.arp.p[0];
        pl[5] = s.p.arp.p[1];
        pl[6] = s.p.arp.p[2];
        pl[7] = s.p.arp.p[3];
        break;
    }
    memcpy(w, pl, 24); w += 24;
  }
  return (uint16_t)(w - p);
}

// Read clock bus + all source records from the cursor (preset.h has already
// validated the blob header + crc — bounds-sane). Clamps everything to the
// engine's live invariants, then recomputes the derived caches (euclid
// masks, gate masks). Field defaults replace an unknown record type so a
// future/newer blob still loads the parts it understands.
static uint16_t mod_engine_preset_read(const uint8_t* p) {
  const uint8_t* r = p;
  ModEngine& e = g_mod_engine;
  e.global_clock = (*r < 2) ? *r : 0;
  ++r;
  float bpm;
  memcpy(&bpm, r, 4); r += 4;
  if (bpm < 20.0f) bpm = 20.0f;
  if (bpm > 300.0f) bpm = 300.0f;
  e.bpm = bpm;
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    const uint8_t type = (*r < 5) ? *r : 0xFF;
    const uint8_t clock_sel = r[1] < 3 ? r[1] : 0;
    const uint8_t division = r[2] < MOD_DIVISION_COUNT ? r[2] : 3;  // 1/8
    const uint8_t dest_cc = r[3];
    r += 4;
    const uint8_t* pl = r;
    r += 24;
    if (type != s.type) {
      // Type changed under us (blob from an older layout): record the
      // common fields, give the new type sane defaults.
      s.type = type;
      memset(&s.p, 0, sizeof(s.p));
      switch (type) {
        case MOD_LFO:
          s.p.lfo.rate_hz = 1.0f; s.p.lfo.level = 0.8f; s.p.lfo.hz_mode = 1;
          break;
        case MOD_EUCLID: s.p.eu.steps = 16; s.p.eu.pulses = 4; break;
        case MOD_GATE: s.p.gate.mode = 0; s.p.gate.density = 8;
                       s.p.gate.len = 16; s.p.gate.gate_len = 0.5f; break;
        case MOD_ADSR: s.p.adsr.a_s = 0.01f; s.p.adsr.d_s = 0.2f;
                       s.p.adsr.s_lvl = 0.7f; s.p.adsr.r_s = 0.3f; break;
        default: // MOD_ARP
          s.p.arp.algo = (i == MOD_SRC_ARP_FIRST) ? MOD_ARP_HYP1 : MOD_ARP_MOTIF;
          s.p.arp.range = (i == MOD_SRC_ARP_FIRST) ? 2 : 1;
          s.p.arp.root_param = 0; // MIDI follow
          s.p.arp.scale_idx = 6;  // Aeolian
          s.p.arp.p[0] = (i == MOD_SRC_ARP_FIRST) ? 65 : 40;
          s.p.arp.p[1] = 40;
          s.p.arp.p[2] = 30;
          s.p.arp.p[3] = 20;
          s.p.arp.ext_idx = (uint8_t)((i >= MOD_SRC_ARP_FIRST) ? (i - MOD_SRC_ARP_FIRST) : 0);
          break;
      }
    }
    s.clock_sel = clock_sel;
    s.division = division;
    s.dest_cc = dest_cc;
    switch (s.type) {
      case MOD_LFO: {
        float v;
        memcpy(&v, pl + 0, 4);
        s.p.lfo.rate_hz = (v >= 0.02f && v <= 20.0f) ? v : 1.0f;
        memcpy(&v, pl + 4, 4);
        s.p.lfo.level = (v >= 0.0f && v <= 1.0f) ? v : 0.8f;
        memcpy(&v, pl + 8, 4);
        s.p.lfo.phase_off = (v >= 0.0f && v <= 1.0f) ? v : 0.0f;
        s.p.lfo.wave = pl[12] < MOD_WAVE_SH_COUNT ? pl[12] : 0;
        s.p.lfo.hz_mode = pl[13] ? 1 : 0;
        break;
      }
      case MOD_EUCLID:
        s.p.eu.steps = pl[0]; s.p.eu.pulses = pl[1]; s.p.eu.rot = pl[2];
        memcpy(&s.p.eu.mask, pl + 4, 4);   // recomputed below anyway
        s.p.eu.level = pl[8];
        mod_eu_recompute(s);
        break;
      case MOD_GATE:
        s.p.gate.mode = pl[0]; s.p.gate.density = pl[1];
        s.p.gate.syncopation = pl[2]; s.p.gate.len = pl[3];
        memcpy(&s.p.gate.mask, pl + 4, 4); // recomputed below anyway
        {
          float v;
          memcpy(&v, pl + 8, 4);
          s.p.gate.gate_len = (v > 0.0f && v <= 1.0f) ? v : 0.5f;
        }
        s.p.gate.cur_preset = pl[12];
        mod_gate_recompute(s);
        break;
      case MOD_ADSR: {
        float v;
        memcpy(&v, pl + 0, 4);
        s.p.adsr.a_s = (v > 0.0f && v <= 10.0f) ? v : 0.01f;
        memcpy(&v, pl + 4, 4);
        s.p.adsr.d_s = (v > 0.0f && v <= 10.0f) ? v : 0.2f;
        memcpy(&v, pl + 8, 4);
        s.p.adsr.s_lvl = (v >= 0.0f && v <= 1.0f) ? v : 0.7f;
        memcpy(&v, pl + 12, 4);
        s.p.adsr.r_s = (v > 0.0f && v <= 10.0f) ? v : 0.3f;
        s.p.adsr.retrig = pl[16] ? 1 : 0;
        break;
      }
      default: {  // MOD_ARP
        s.p.arp.algo = pl[0] <= 4 ? pl[0] : 0;
        s.p.arp.range = (pl[1] >= 1 && pl[1] <= 3) ? pl[1] : 1;
        s.p.arp.root_param = pl[2] <= 84 ? pl[2] : 0;
        s.p.arp.scale_idx = pl[3] <= 127 ? pl[3] : 6;
        s.p.arp.p[0] = pl[4] <= 100 ? pl[4] : 50;
        s.p.arp.p[1] = pl[5] <= 100 ? pl[5] : 40;
        s.p.arp.p[2] = pl[6] <= 100 ? pl[6] : 30;
        s.p.arp.p[3] = pl[7] <= 100 ? pl[7] : 20;
        s.p.arp.ext_idx = (uint8_t)((i >= MOD_SRC_ARP_FIRST) ? (i - MOD_SRC_ARP_FIRST) : 0);
        mod_arp_rebuild_pattern(s);
        break;
      }
    }
    s.was_high = 0;                // no stale edges into the restored state
  }
  return (uint16_t)(r - p);
}
