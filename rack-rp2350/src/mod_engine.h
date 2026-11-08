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

// ===================== source taxonomy =====================
enum ModSourceType : uint8_t {
  MOD_LFO = 0, MOD_EUCLID = 1, MOD_GATE = 2, MOD_ADSR = 3,
};
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

// Source ids (CC 33 values): 0-3 LFO1-4, 4-7 EU1-4, 8-9 GATE1-2, 10-11 ADSR1-2.
// NOTE: the plan's Sources section header says "(8)" but its verbatim list is
// 4+4+2+2 = 12 and the acceptance gate requires all four types audible — the
// full 12-source list is implemented; CC 33 = 0-7 keeps the semantics the T3
// bench decision assumed, 8-11 reach the gate/ADSR pair. (Reported deviation.)
static const uint8_t MOD_SOURCE_COUNT = 12;
static const uint8_t MOD_SRC_EU_FIRST = 4;    // odometer source for telemetry

// Clock divisions: PPQ ticks per division cycle (24 PPQ = quarter note).
// All divisors are integers in 24 PPQ (triplets included).
struct ModDivision { const char* name; uint8_t ppq; };
static const ModDivision kModDivisions[10] = {
    {"1/1", 96}, {"1/2", 48}, {"1/4", 24}, {"1/8", 12}, {"1/16", 6},
    {"1/32", 3}, {"1/4T", 16}, {"1/8T", 8}, {"1/16T", 4}, {"1/32T", 2},
};
static const uint8_t MOD_DIVISION_COUNT = 10;

// Per-source parameter names/counts — the CC 34 param indexes and the console
// `mod <src> <param> <value>` names use the SAME tables. The last three
// params of every type are uniformly clk / div / dest.
static const char* const kModParamNames[4][8] = {
    {"mode", "rate", "wave", "phase", "level", "clk", "div", "dest"},  // LFO
    {"steps", "pulses", "rot", "clk", "div", "dest"},                  // EUCLID
    {"len", "mask", "glen", "clk", "div", "dest"},                     // GATE
    {"mode", "atk", "dec", "sus", "rel", "clk", "div", "dest"},        // ADSR
};
static const uint8_t kModParamCount[4] = {8, 6, 6, 8};

struct ModSource {
  // common
  uint8_t type;          // ModSourceType
  uint8_t clock_sel;     // ModClockSel
  uint8_t division;      // index into kModDivisions
  uint8_t dest_cc;       // destination CC (0 = un-routed)
  float   clk_phase;     // 0..1 division-cycle phase (LFO wave phase when clocked)
  float   out;           // last output (0..1) — telemetry
  uint8_t was_high;      // last trigger-source level (edge detect)
  uint32_t rng;          // per-source LCG state (S&H)
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
  // ---- telemetry (core 1 writes, core 0 prints + resets each 2 s report) ---
  volatile uint32_t mod_us_max;       // engine block cost, max since report
  volatile uint32_t telem_midi_ticks; // F8 consumed since last report
};
extern ModEngine g_mod_engine;   // DEFINED in main.cpp — plain .bss, NOT arenas

// ===================== small helpers =====================
static inline float mod_clamp01(float v) {
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

static inline float mod_rand01(ModSource& s) {
  s.rng = s.rng * 1664525u + 1013904223u;
  return (float)(s.rng >> 8) * (1.0f / 16777216.0f);
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
    case RACK_MODULE_ELEMENTS: break;   // no trigger inputs (documented)
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
  e.block_period_us = 0;           // no block published yet (stamps skip the
                                   // sub-block estimate until audio runs)
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    ModSource& s = e.src[i];
    s.type = (i < 4) ? MOD_LFO : (i < 8) ? MOD_EUCLID
           : (i < 10) ? MOD_GATE : MOD_ADSR;
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
      s.division = 3;              // 1/8
      s.p.gate.len = 8;
      s.p.gate.mask = 0x55u;       // alternating, starts on step 0
      s.p.gate.gate_len = 0.5f;
    } else {                       // MOD_ADSR
      s.p.adsr.a_s = 0.005f;       // 5 ms
      s.p.adsr.d_s = 0.100f;       // 100 ms
      s.p.adsr.s_lvl = 0.5f;
      s.p.adsr.r_s = 0.200f;       // 200 ms
      s.p.adsr.retrig = 0;         // gate-follow from the note stack
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
      s.p.gate.level = (s.p.gate.mask & 1u) ? 1u : 0u;
    } else if (s.type == MOD_ADSR && s.p.adsr.retrig) {
      s.p.adsr.env = 0.0f;
      s.p.adsr.env_state = MOD_ENV_IDLE;
      s.p.adsr.ticks = 0;
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
          s.p.gate.step = (uint8_t)((s.p.gate.step + wraps) % s.p.gate.len);
          s.p.gate.ticks += wraps;
          s.p.gate.level = (s.p.gate.mask >> s.p.gate.step) & 1u;
        }
        // Gate length: the step's level holds only for the first gate_len
        // fraction of the division cycle (clk_phase is the in-step fraction).
        const bool high = s.p.gate.level && (s.clk_phase < s.p.gate.gate_len);
        s.out = high ? 1.0f : 0.0f;
      } else {  // MOD_ADSR
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
    if (s.type == MOD_LFO || s.type == MOD_ADSR) {
      mod_write_cc(active_module, dest, s.out);     // continuous, float-level
    } else {
      const bool high = (s.type == MOD_EUCLID) ? (s.p.eu.level != 0)
                                               : (s.out > 0.5f);
      const int8_t kind = mod_trigger_kind(active_module, dest);
      if (kind < 0) { s.was_high = 0; continue; }   // inert (logged at set)
      if (kind == RACK_TRIG_LATCH) {
        if (high != (bool)s.was_high) {             // freeze follows the level
          mod_fire_trigger(active_module, dest, high);
          s.was_high = high ? 1u : 0u;
        }
      } else {                                      // PULSE destinations
        if (s.type == MOD_EUCLID) {
          // Every masked step re-fires (consecutive hits each get an edge).
          if (wraps && s.p.eu.level) mod_fire_trigger(active_module, dest, true);
        } else if (high && !s.was_high) {           // gate: rising edges only
          mod_fire_trigger(active_module, dest, true);
        }
        s.was_high = high ? 1u : 0u;
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
  return kModParamCount[type < 4 ? type : 0];
}

// Canonical source names (also used by the dest ack + `mod <src>` dumps).
static const char* mod_src_name(uint8_t idx) {
  static const char* const kNames[MOD_SOURCE_COUNT] = {
      "lfo1", "lfo2", "lfo3", "lfo4",
      "eu1", "eu2", "eu3", "eu4",
      "gate1", "gate2", "adsr1", "adsr2",
  };
  return kNames[idx < MOD_SOURCE_COUNT ? idx : 0];
}

// Destination set + one-shot validation log against the ACTIVE module.
static void mod_set_dest(uint8_t src_idx, uint8_t cc) {
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
  s.dest_cc = cc;
  s.was_high = 0;                 // no stale edge into the new destination
  if (cc == 0) {
    Serial.printf("mod: %s destination cleared (panel regains the parameter)\r\n",
                  mod_src_name(src_idx));
    return;
  }
  const RackModuleInfo& m = kRackModules[g_active_module];
  const RackModDestinations* d = m.mod_destinations;
  bool continuous_ok = false, trigger_ok = false;
  for (uint8_t i = 0; i < d->continuous_count; ++i) {
    if (d->continuous[i] == cc) { continuous_ok = true; break; }
  }
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == cc) { trigger_ok = true; break; }
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

static inline float mod_adsr_s_from_cc(uint8_t v) {   // 1 ms .. 10 s, exponential
  return powf(10.0f, -3.0f + 4.0f * (float)v / 127.0f);
}

// Apply a bench CC value (0-127) to (source, param). Silent per move (a knob
// sweep would spam); `mod?` shows the resulting state, dest changes ack.
static void mod_apply_cc_value(uint8_t src_idx, uint8_t param, uint8_t val) {
  ModEngine& e = g_mod_engine;
  ModSource& s = e.src[src_idx];
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
        case 0: {
          s.p.gate.len = (uint8_t)(1 + (uint16_t)val * 31 / 127);
          const uint32_t keep = (s.p.gate.len >= 32)
              ? 0xFFFFFFFFu : ((1u << s.p.gate.len) - 1u);
          s.p.gate.mask &= keep;               // bits beyond len are ignored
          break;
        }
        // CC writes a DENSITY FILL (evenly spread k-of-len pattern so one
        // knob is musical); the console takes the raw mask (`mod .. mask`).
        case 1: {
          const uint8_t k = (uint8_t)(((uint16_t)val * s.p.gate.len) / 127);
          s.p.gate.mask = mod_euclid_mask(k, s.p.gate.len);
          break;
        }
        case 2: s.p.gate.gate_len = (1.0f + (float)val) / 128.0f; break;
        default: break;
      }
      break;
    default:  // MOD_ADSR
      switch (param) {
        case 0: s.p.adsr.retrig = val < 64 ? 0 : 1; break;   // gate / retrig
        case 1: s.p.adsr.a_s = mod_adsr_s_from_cc(val); break;
        case 2: s.p.adsr.d_s = mod_adsr_s_from_cc(val); break;
        case 3: s.p.adsr.s_lvl = val / 127.0f; break;
        case 4: s.p.adsr.r_s = mod_adsr_s_from_cc(val); break;
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
  if (param >= n) return false;
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
    if (tok[0] >= '0' && tok[0] <= '9') {
      const long v = atol(tok);
      if (v >= 0 && v <= 127) { mod_set_dest(src_idx, (uint8_t)v); return true; }
    }
    return false;
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
      switch (param) {
        case 0: {
          uint8_t len = (uint8_t)(v < 1 ? 1 : (v > 32 ? 32 : v));
          s.p.gate.len = len;
          const uint32_t keep = (len >= 32) ? 0xFFFFFFFFu : ((1u << len) - 1u);
          s.p.gate.mask &= keep;
          return true;
        }
        case 1:                                          // raw mask, hex ok
          s.p.gate.mask = (uint32_t)strtoul(tok, nullptr, 0);
          return true;
        case 2:
          s.p.gate.gate_len = mod_clamp01(v);
          return true;
        default: return false;
      }
    default:  // MOD_ADSR
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
    Serial.println("usage: mod <0-11|lfo1-4|eu1-4|gate1-2|adsr1-2> <param> <value>");
    return;
  }
  const int8_t src = mod_parse_source(tok);
  if (src < 0) {
    Serial.println("mod: unknown source (0-11 or lfo1-4 / eu1-4 / gate1-2 / adsr1-2)");
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
  const bool want_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR);
  bool cont = false, trig = false;
  for (uint8_t i = 0; i < d->continuous_count; ++i) {
    if (d->continuous[i] == s.dest_cc) { cont = true; break; }
  }
  for (uint8_t i = 0; i < d->trigger_count; ++i) {
    if (d->trigger_cc[i] == s.dest_cc) { trig = true; break; }
  }
  char tag[12];
  if (want_continuous && cont) snprintf(tag, sizeof(tag), "cont");
  else if (!want_continuous && trig) snprintf(tag, sizeof(tag), "trig");
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
                  : s.type == MOD_GATE ? "GATE" : "ADSR",
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
      default:
        Serial.printf("mode=%s atk=%sms dec=%sms sus=%s rel=%sms out=%s",
                      s.p.adsr.retrig ? "retrig" : "gate",
                      mod_print_fixed(b1, sizeof(b1), s.p.adsr.a_s * 1000.0f),
                      mod_print_fixed(b2, sizeof(b2), s.p.adsr.d_s * 1000.0f),
                      mod_print_fixed(b3, sizeof(b3), s.p.adsr.s_lvl),
                      mod_print_fixed(b4, sizeof(b4), s.p.adsr.r_s * 1000.0f),
                      mod_print_fixed(b5, sizeof(b5), s.out));
        break;
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
      default: c = (s.p.adsr.env > 0.01f) ? 'A' : 'a'; break;
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
