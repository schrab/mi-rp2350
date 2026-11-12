// Rack RP2350 — PLAITS adapter.
//
// Lifted VERBATIM from plaits-rp2350/src/main.cpp + midi.h: the exact
// Patch/Modulations per-block fill of the original firmware main (documented
// inline below), velocity-scaled one-block TRIG pulses, MACRO via
// modulations.engine under model_cv_option = 1, stereo aux via
// aux_output_option = 1, raw write16 (Voice clips/limits internally).
//
// T2: compiles in EVERY rack env (state is arena-placed, see rack_arena.h);
// the 32 KiB BufferAllocator workspace carves from the BUFFER arena.
#pragma once

// Module-library includes UNCONDITIONAL (see module_elements.h): all four
// module libs are linked in every rack env.
#include <STMLIB.h>
#include <PLAITS.h>

#include <new>                   // placement-new into the arenas

#include "plaits/dsp/voice.h"    // plaits::Voice, Patch, Modulations
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"
#include "module_registry.h"     // RackModDestinations (T3 mod routing table)

// Forward declaration of mod_engine destination query (defined in mod_engine.h).
bool mod_is_dest_routed(uint8_t cc);

namespace rack_modules {
namespace mod_plaits {

// The MIDI CC map bounds engine selection to the Plaits-Lab catalog size;
// this must equal the registry the lib was generated with
// (-DPLAITS_ENGINE_COUNT=98 reaches src/ and lib/ alike in every rack env).
static const uint8_t kPlaitsEngineCount = 98;
static_assert(plaits::kMaxEngines == kPlaitsEngineCount,
              "module_plaits catalog size must match PLAITS_ENGINE_COUNT");

static const char* const kName = "plaits";
static const uint32_t kSampleRate = 48000;               // = plaits::kSampleRate
static const size_t kBlockSize = plaits::kMaxBlockSize;  // 24 samples
static const uint32_t kBudgetUs =
    (uint32_t)(1000000UL * kBlockSize / kSampleRate);  // 500
static const uint32_t kI2sBuffers = 48;   // 48 x 16 words = 768 frames
static const uint32_t kI2sWords = 16;     // = 16 ms bank at 48 kHz
static const uint32_t kStallUs = 900;     // >2x the 500 us block period

static const char* const kCcMap =
    "7 vca, 14 timbre, 15 morph, 16 harmonics, 17 macro/model-cv, 18 lpg-colour, "
    "1 pitch(-24..+24st), 7 vca/level, 14 timbre, 15 morph, 16 harmonics, "
    "17 macro, 18 lpg-colour, 19 decay, 23 engine(0-97), 24 aux-option, "
    "25 level, 64 trig";
// T3 mod destinations: continuous 1, 7, 14-19, 25 (17 keeps macro mapping;
// 1 = pitch -24..+24 st). 23/24 excluded (selectors). Pseudo-CC 64 = TRIG.
static const uint8_t kModContinuousCcs[] = {1, 7, 14, 15, 16, 17, 18, 19, 25};
static const uint8_t kModTriggerCcs[] = {64};
static const uint8_t kModTriggerKinds[] = {RACK_TRIG_PULSE};
static const RackModDestinations kModDestinations = {
    "plaits: continuous 1,7,14-19,25 (23/24 excluded: engine/aux selectors) | "
    "triggers: 64 trig (pulse, full velocity)",
    9, kModContinuousCcs,
    1, kModTriggerCcs, kModTriggerKinds,
};

// --- engine state: arena-placed (was file-scope before T2) ---
struct State {
  // plaits::Voice embeds all 98 engines; their state RAM is carved per engine
  // from one shared pool by stmlib::BufferAllocator (voice.cc:105-113 frees
  // before every engine Init) — 32 KB covers the largest single engine
  // (16.4 KB) with margin.
  plaits::Voice voice;
  plaits::Patch patch;
  plaits::Modulations mod;
  plaits::Voice::Frame frames[kBlockSize];  // interleaved stereo shorts

  // Panel parameters (core 0 writes, core 1 copies into the Patch/Modulations).
  volatile float shared_pitch;       // CC 1: PITCH (-24..+24 st)
  volatile float shared_timbre;      // CC 14
  volatile float shared_morph;       // CC 15
  volatile float shared_harmonics;   // CC 16
  volatile float shared_macro_cv;    // CC 17: (val-64)/127
  volatile float shared_lpg_colour;  // CC 18
  volatile float shared_decay;       // CC 19
  volatile float shared_fm_amount;   // CC 20 (unused, neutral 0.0)
  volatile float shared_timbre_mod;  // CC 21 (unused, neutral 0.0)
  volatile float shared_morph_mod;   // CC 22 (unused, neutral 0.0)
  volatile float shared_level;       // CC 25 / CC 7

  // TRIG (CC 64): latched gate switch + pending edge
  volatile bool  trig_latched;       // CC 64 pot: false (0) = unpatched, true (127) = patched
  volatile bool  pending_trig_pulse; // pulse trigger on rising edge (from CC 64 or mod)

  volatile int8_t   pending_engine;      // -1 none, 0..97 engine
  volatile int8_t   pending_aux_option;  // -1 none, 0/1/2 aux option
  volatile float    pending_trigger;     // <0 none, else pulse voltage

  // Active engine mirror for the 2 s report (core 1 publishes, core 0 prints).
  volatile uint32_t active_engine_report;
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

// T6 chain hooks (see module_elements.h): state_size()/init_at() keep the
// single-module path; init_chain_a() re-Inits the slot at the CHAIN rate
// context (plaits renders at 32 kHz while chained, 48 kHz switched).
static size_t state_size() { return sizeof(State); }

void init_at(uint8_t* base) {
  s = new (base) State();
  // Buffer carve: uint32_t workspace[8192] = 32 KB from the buffer arena.
  // The arena is alignas(8): Allocate<T> does not adjust alignment.
  memset(g_rack_buffer_arena, 0, 32768);
  stmlib::BufferAllocator allocator(
      reinterpret_cast<void*>(g_rack_buffer_arena), 32768);
  // Rate context (08-plaits-runtime-rate.patch: runtime kSampleRate): the
  // native SWITCHED context is 48 kHz. Engines capture the rate inside their
  // Init() — the setter MUST run before voice.Init(). (Before T6 this was
  // implicit: the compile-time 48 kHz constant.)
  plaits::SetSampleRate(kSampleRate);   // 48000
  s->voice.Init(&allocator);

  s->pending_engine = -1;
  s->pending_aux_option = -1;
  s->pending_trigger = -1.0f;
  s->active_engine_report = 0;
  s->trig_latched = false;
  s->pending_trig_pulse = false;
  s->shared_pitch = 0.0f;
  s->shared_timbre = 0.5f;
  s->shared_morph = 0.5f;
  s->shared_harmonics = 0.5f;
  s->shared_macro_cv = 0.0f;
  s->shared_lpg_colour = 0.5f;
  s->shared_decay = 0.5f;
  s->shared_fm_amount = 0.0f;
  s->shared_timbre_mod = 0.0f;
  s->shared_morph_mod = 0.0f;
  s->shared_level = 1.0f;

  // Patch option fields the CC map does not reach (voice.h), set to the
  // MIDI-sensible defaults the per-block fill relies on:
  s->patch.engine = 0;                      // CC 23 from here on
  s->patch.aux_output_option = 1;           // stereo OUT/AUX (CC 24)
  s->patch.aux_subosc_option = 0;           // square, no octave shift (option 2)
  s->patch.trig_response_option = 2;        // velocity trigger
  s->patch.model_cv_option = 1;             // MODEL CV = fourth macro (CC 17)
  s->patch.locked_frequency_pot_option = 0; // no pitch lock: DECAY is decay
  s->patch.level_cv_option = 0;             // LEVEL stays level
  s->patch.hold_on_trigger_option = 0;      // no parameter hold on trigger
  s->patch.chord_set_option = 0;            // first chord table (chord engines)
  s->patch.attenuverter_mode = plaits::ATTENUVERTER_MODE_STOCK;  // 0
  s->patch.freqlock_param = 0.5f;           // unused while locked_frequency=0
}

void init() { init_at(g_rack_object_arena); }

// T6: chain A-slot init — the SAME factory state as init_at, but at the
// CHAIN rate context (32 kHz). Decision 3: entering/leaving a chain is a
// rate-context event; the Voice re-Init re-captures the rate in every engine
// (engines bake rate-derived coefficients in Init()). NOT reachable in this
// build (the pair table rejects plaits as chain A on window capacity — see
// main.cpp) but kept wired so the runtime-rate mechanism is exercised
// end-to-end by the state machine the moment the window grows.
void init_chain_a(uint8_t* base) {
  init_at(base);
  // Re-Init at the chain rate: the whole Init sequence runs again with the
  // 32 kHz context (BufferAllocator state is rebuilt by voice.Init).
  memset(g_rack_buffer_arena, 0, 32768);
  stmlib::BufferAllocator allocator(
      reinterpret_cast<void*>(g_rack_buffer_arena), 32768);
  plaits::SetSampleRate(32000.0f);
  s->voice.Init(&allocator);
}

// T6: chain A-slot render — exactly n samples (n <= 24 = kMaxBlockSize) into
// caller buffers as ±1.0 floats (frames are already int16-clipped by Voice;
// /32768 is the same normalization the I2S write path uses), NO I2S write,
// NO proc_us_max update (the chain loop times A + route + B together).
void chain_render_a(float* out_l, float* out_r, size_t n) {
  if (s->pending_engine >= 0) {
    int engine = s->pending_engine;
    if (engine > plaits::kMaxEngines - 1) engine = plaits::kMaxEngines - 1;
    s->patch.engine = engine;
    s->pending_engine = -1;
  }
  if (s->pending_aux_option >= 0) {
    s->patch.aux_output_option = static_cast<uint8_t>(s->pending_aux_option);
    s->pending_aux_option = -1;
  }

  if (s->pending_trig_pulse) {
    s->pending_trig_pulse = false;
    s->pending_trigger = 1.0f;
  }

  const bool trig_routed = ::mod_is_dest_routed(64);
  const bool level_routed = ::mod_is_dest_routed(25) || ::mod_is_dest_routed(7);

  const bool trig_patched = s->trig_latched || trig_routed;
  const bool level_patched = level_routed;

  if (s->pending_trigger > -0.5f) {
    s->mod.trigger = s->pending_trigger;
    s->pending_trigger = -1.0f;
  } else {
    s->mod.trigger = 0.0f;
  }

  s->patch.note = shared_note + s->shared_pitch;
  s->patch.harmonics = s->shared_harmonics;
  s->patch.timbre = s->shared_timbre;
  s->patch.morph = s->shared_morph;
  s->patch.frequency_modulation_amount = s->shared_fm_amount;
  s->patch.timbre_modulation_amount = s->shared_timbre_mod;
  s->patch.morph_modulation_amount = s->shared_morph_mod;
  s->patch.decay = s->shared_decay;
  s->patch.lpg_colour = s->shared_lpg_colour;

  s->mod.engine = s->shared_macro_cv;
  s->mod.note = 0.0f;
  s->mod.frequency = 0.0f;
  s->mod.harmonics = 0.0f;
  s->mod.timbre = 0.0f;
  s->mod.morph = 0.0f;
  s->mod.level = level_patched ? s->shared_level : 0.0f;
  s->mod.hard_sync = 0;
  for (size_t i = 0; i < n; ++i) {
    s->mod.frequency_audio[i] = 0.0f;
  }
  s->mod.frequency_audio_rate = false;
  s->mod.frequency_patched = false;
  s->mod.timbre_patched = false;
  s->mod.morph_patched = false;
  s->mod.trigger_patched = trig_patched;
  s->mod.level_patched = level_patched;

  s->voice.Render(s->patch, s->mod, s->frames, n);
  const float vca = level_patched ? 1.0f : s->shared_level;
  for (size_t i = 0; i < n; ++i) {
    out_l[i] = (static_cast<float>(s->frames[i].out) / 32768.0f) * vca;
    out_r[i] = (static_cast<float>(s->frames[i].aux) / 32768.0f) * vca;
  }
}

void render() {
  // Apply pending changes strictly between two Render() calls. Engine
  // changes go through patch.engine; Voice reloads/resets the engine itself
  // at the top of Render (voice.cc:382-489).
  if (s->pending_engine >= 0) {
    int engine = s->pending_engine;
    if (engine > plaits::kMaxEngines - 1) engine = plaits::kMaxEngines - 1;
    s->patch.engine = engine;
    s->pending_engine = -1;
  }
  if (s->pending_aux_option >= 0) {
    s->patch.aux_output_option = static_cast<uint8_t>(s->pending_aux_option);
    s->pending_aux_option = -1;
  }

  if (s->pending_trig_pulse) {
    s->pending_trig_pulse = false;
    s->pending_trigger = 1.0f;
  }

  const bool trig_routed = ::mod_is_dest_routed(64);
  const bool level_routed = ::mod_is_dest_routed(25) || ::mod_is_dest_routed(7);

  const bool trig_patched = s->trig_latched || trig_routed;
  const bool level_patched = level_routed;

  // TRIG pulse: one block high at the note-on/pulse velocity, then low so
  // trigger_state_ clears. Rapid re-notes each get their own edge.
  if (s->pending_trigger > -0.5f) {
    s->mod.trigger = s->pending_trigger;
    s->pending_trigger = -1.0f;
  } else {
    s->mod.trigger = 0.0f;
  }

  // --- Patch fill (units documented in plaits-rp2350/src/main.cpp) ---
  s->patch.note = shared_note + s->shared_pitch;
  s->patch.harmonics = s->shared_harmonics;
  s->patch.timbre = s->shared_timbre;
  s->patch.morph = s->shared_morph;
  s->patch.frequency_modulation_amount = s->shared_fm_amount;
  s->patch.timbre_modulation_amount = s->shared_timbre_mod;
  s->patch.morph_modulation_amount = s->shared_morph_mod;
  s->patch.decay = s->shared_decay;
  s->patch.lpg_colour = s->shared_lpg_colour;

  // --- Modulations fill: TRIG + LEVEL reflect patch state, everything else unpatched ---
  s->mod.engine = s->shared_macro_cv;  // fourth macro under model_cv_option=1
  s->mod.note = 0.0f;
  s->mod.frequency = 0.0f;
  s->mod.harmonics = 0.0f;
  s->mod.timbre = 0.0f;
  s->mod.morph = 0.0f;
  s->mod.level = level_patched ? s->shared_level : 0.0f;
  s->mod.hard_sync = 0;
  for (size_t i = 0; i < kBlockSize; ++i) {
    s->mod.frequency_audio[i] = 0.0f;  // unused: PLAITS_BUILD_FREQUENCY_OFFSET_FM=0
  }
  s->mod.frequency_audio_rate = false;
  s->mod.frequency_patched = false;
  s->mod.timbre_patched = false;
  s->mod.morph_patched = false;
  s->mod.trigger_patched = trig_patched;
  s->mod.level_patched = level_patched;

  uint32_t t0 = time_us_32();
  // Render() every block (the original firmware contract: plaits.cc:254).
  // Output Frames are already limited/clipped to int16 inside Voice — write
  // them raw, do NOT double-clip. g_render_gain is the swap-mute ramp
  // (1.0 in normal render).
  const float gain = g_render_gain;
  s->voice.Render(s->patch, s->mod, s->frames, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  s->active_engine_report = (uint32_t)s->voice.active_engine();

  // Master VCA: when LEVEL is unpatched (LPG bypass or internal ping), scale
  // output directly by shared_level; when LEVEL is patched by mod, LPG already
  // applies compressed_level.
  const float vca = level_patched ? 1.0f : s->shared_level;

  // Meters: OUT/AUX block RMS, max since the last 2 s report.
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    const float l = static_cast<float>(s->frames[i].out) * gain * vca / 32768.0f;
    const float r = static_cast<float>(s->frames[i].aux) * gain * vca / 32768.0f;
    out_acc += l * l;
    aux_acc += r * r;
  }
  const float out_rms = sqrtf(out_acc / kBlockSize);
  const float aux_rms = sqrtf(aux_acc / kBlockSize);
  if (out_rms > rack_meter_a) rack_meter_a = out_rms;
  if (aux_rms > rack_meter_b) rack_meter_b = aux_rms;

  // The writes pace the render loop: one iteration = one 24-frame DMA block,
  // blocks free every 500 us; > 900 us = stalled.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out = static_cast<int16_t>(s->frames[i].out * gain * vca);
    const int16_t aux = static_cast<int16_t>(s->frames[i].aux * gain * vca);
    i2s.write16(out, aux);
  }
  if (time_us_32() - w0 > kStallUs) {
    ++i2s_stall_count;
  }
}

void note_on(uint8_t note, uint8_t vel) {
  (void)note;
  // TRIG pulse voltage: velocity scaled to always cross Voice's 0.3 trigger
  // threshold — 0.3 + 0.7 * v — latched as the accent velocity on the
  // (delayed) rising edge. Consumed by core 1 on the next block.
  s->pending_trigger = 0.3f + 0.7f * (float)vel / 127.0f;
}

void note_off(uint8_t note) {
  (void)note;   // gate -> LEVEL release handled by the shared note stack
}

void control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 1:
      // CC 1 = PITCH: -24..+24 semitones around standard pitch (val 64 = 0)
      s->shared_pitch = (val < 64)
          ? (float)(val - 64) * (24.0f / 64.0f)
          : (float)(val - 64) * (24.0f / 63.0f);
      break;
    case 14: s->shared_timbre     = v; break;
    case 15: s->shared_morph      = v; break;
    case 16: s->shared_harmonics  = v; break;
    // MACRO/TWIST as the MODEL CV (patch.model_cv_option = 1): macro = 0.5 +
    // modulations.engine. (val-64)/127 puts the exact engine-stock neutral
    // (0.5) at CC 64.
    case 17: s->shared_macro_cv = ((float)val - 64.0f) / 127.0f; break;
    case 18: s->shared_lpg_colour = v; break;
    case 19: s->shared_decay      = v; break;
    // ENGINE: raw value = registry index (0-97). Values beyond the catalog
    // are ignored; applied between render blocks.
    case 23:
      if (val < kPlaitsEngineCount) s->pending_engine = static_cast<int8_t>(val);
      break;
    // AUX OPTION: 0 regular aux model / 1 stereo (OUT=L, AUX=R on
    // stereo-capable engines) / 2 suboscillator.
    case 24:
      if (val <= 2) s->pending_aux_option = static_cast<int8_t>(val);
      break;
    case 7:
    case 25: s->shared_level = v; break;
    // TRIG pseudo-CC (64): val >= 64 patches TRIG and starts envelope pulse;
    // val < 64 unpatches TRIG (restores LPG bypass if LEVEL is unpatched).
    case 64:
      s->trig_latched = (val >= 64);
      if (val >= 64) s->pending_trig_pulse = true;
      break;
    default: break;
  }
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Same between-blocks discipline as the pending slots in render(); the mod
// engine only runs while not swapping, so `s` is always the live state.

// Continuous destination write (0..1 virtual CC position, float precision).
// CCs 1/17 apply the SAME range mappings control_change applies to a
// CC byte (macro centered at v=64/127; pitch -24..+24 st), so a routed
// LFO sweeps the full panel range at level 1.0. 23/24 excluded (selectors).
bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  switch (cc) {
    case 1:  s->shared_pitch      = (v - 0.5f) * 48.0f; break; // CC 1 = PITCH: -24..+24 st
    case 7:
    case 25: s->shared_level      = v; break;
    case 14: s->shared_timbre     = v; break;
    case 15: s->shared_morph      = v; break;
    case 16: s->shared_harmonics  = v; break;
    case 17: s->shared_macro_cv   = (v * 127.0f - 64.0f) / 127.0f; break;
    case 18: s->shared_lpg_colour = v; break;
    case 19: s->shared_decay      = v; break;
    default: return false;
  }
  return true;
}

// Trigger destination write: pseudo-CC 64 = the TRIG input. Rising edges fire
// a full-velocity one-block pulse (the same pending_trigger path note-on
// uses; 1.0 = accent ceiling of the 0.3..1.0 pulse voltage).
bool mod_trigger_cc(uint8_t cc, bool high) {
  if (cc == 64) {
    if (high) s->pending_trig_pulse = true;
    return true;
  }
  return false;
}

}  // namespace mod_plaits
}  // namespace rack_modules
