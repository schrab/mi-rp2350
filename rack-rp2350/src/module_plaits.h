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
    "14 timbre, 15 morph, 16 harmonics, 17 macro/model-cv, 18 lpg-colour, "
    "19 decay, 20 fm-amount, 21 timbre-mod, 22 morph-mod, 23 engine(0-97), "
    "24 aux-option, 25 level";
// T3 mod destinations: the patch/modulation parameters 14-22 + 25 accept
// continuous modulation (17/20/21/22 keep their control_change range mappings,
// applied at float precision). 23 (engine catalog index) and 24 (aux output
// option) are EXCLUDED — catalog/option selectors must not be modulated; the
// engine change also re-Inits the engine mid-render, so a per-block write
// would retrigger it every block. Pseudo-CC 64 (sustain pedal — verified
// unused by all four CC maps) = the TRIG input: pulse-kind, full velocity.
static const uint8_t kModContinuousCcs[] = {14, 15, 16, 17, 18, 19, 20, 21,
                                            22, 25};
static const uint8_t kModTriggerCcs[] = {64};
static const uint8_t kModTriggerKinds[] = {RACK_TRIG_PULSE};
static const RackModDestinations kModDestinations = {
    "plaits: continuous 14-22,25 (23/24 excluded: engine/aux selectors) | "
    "triggers: 64 trig (pulse, full velocity)",
    10, kModContinuousCcs,
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
  volatile float shared_timbre;      // CC 14
  volatile float shared_morph;       // CC 15
  volatile float shared_harmonics;   // CC 16
  volatile float shared_macro_cv;    // CC 17: (val-64)/127
  volatile float shared_lpg_colour;  // CC 18
  volatile float shared_decay;       // CC 19
  volatile float shared_fm_amount;   // CC 20: v*2-1
  volatile float shared_timbre_mod;  // CC 21: v*2-1
  volatile float shared_morph_mod;   // CC 22: v*2-1
  volatile float shared_level;       // CC 25

  volatile int8_t   pending_engine;      // -1 none, 0..97 engine
  volatile int8_t   pending_aux_option;  // -1 none, 0/1/2 aux option
  volatile float    pending_trigger;     // <0 none, else pulse voltage

  // Active engine mirror for the 2 s report (core 1 publishes, core 0 prints).
  volatile uint32_t active_engine_report;
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

void init() {
  s = new (g_rack_object_arena) State();
  // Buffer carve: uint32_t workspace[8192] = 32 KB from the buffer arena.
  // The arena is alignas(8): Allocate<T> does not adjust alignment.
  memset(g_rack_buffer_arena, 0, 32768);
  stmlib::BufferAllocator allocator(
      reinterpret_cast<void*>(g_rack_buffer_arena), 32768);
  s->voice.Init(&allocator);

  s->pending_engine = -1;
  s->pending_aux_option = -1;
  s->pending_trigger = -1.0f;
  s->active_engine_report = 0;
  s->shared_timbre = 0.5f;
  s->shared_morph = 0.5f;
  s->shared_harmonics = 0.5f;
  s->shared_macro_cv = 0.0f;
  s->shared_lpg_colour = 0.5f;
  s->shared_decay = 0.5f;
  s->shared_fm_amount = 0.0f;
  s->shared_timbre_mod = 0.0f;
  s->shared_morph_mod = 0.0f;
  s->shared_level = 0.8f;

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

  // TRIG pulse: one block high at the note-on velocity, then low so
  // trigger_state_ clears. Rapid re-notes each get their own edge.
  if (s->pending_trigger > -0.5f) {
    s->mod.trigger = s->pending_trigger;
    s->pending_trigger = -1.0f;
  } else {
    s->mod.trigger = 0.0f;
  }

  // --- Patch fill (units documented in plaits-rp2350/src/main.cpp) ---
  s->patch.note = shared_note;
  s->patch.harmonics = s->shared_harmonics;
  s->patch.timbre = s->shared_timbre;
  s->patch.morph = s->shared_morph;
  s->patch.frequency_modulation_amount = s->shared_fm_amount;
  s->patch.timbre_modulation_amount = s->shared_timbre_mod;
  s->patch.morph_modulation_amount = s->shared_morph_mod;
  s->patch.decay = s->shared_decay;
  s->patch.lpg_colour = s->shared_lpg_colour;

  // --- Modulations fill: TRIG + LEVEL patched, everything else unpatched ---
  s->mod.engine = s->shared_macro_cv;  // fourth macro under model_cv_option=1
  s->mod.note = 0.0f;
  s->mod.frequency = 0.0f;
  s->mod.harmonics = 0.0f;
  s->mod.timbre = 0.0f;
  s->mod.morph = 0.0f;
  s->mod.level = shared_gate ? s->shared_level : 0.0f;  // held = sustain, off = release
  s->mod.hard_sync = 0;
  for (size_t i = 0; i < kBlockSize; ++i) {
    s->mod.frequency_audio[i] = 0.0f;  // unused: PLAITS_BUILD_FREQUENCY_OFFSET_FM=0
  }
  s->mod.frequency_audio_rate = false;
  s->mod.frequency_patched = false;
  s->mod.timbre_patched = false;
  s->mod.morph_patched = false;
  s->mod.trigger_patched = true;
  s->mod.level_patched = true;

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

  // Meters: OUT/AUX block RMS, max since the last 2 s report.
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    const float l = static_cast<float>(s->frames[i].out) * gain / 32768.0f;
    const float r = static_cast<float>(s->frames[i].aux) * gain / 32768.0f;
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
    const int16_t out = static_cast<int16_t>(s->frames[i].out * gain);
    const int16_t aux = static_cast<int16_t>(s->frames[i].aux * gain);
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
    case 14: s->shared_timbre     = v; break;
    case 15: s->shared_morph      = v; break;
    case 16: s->shared_harmonics  = v; break;
    // MACRO/TWIST as the MODEL CV (patch.model_cv_option = 1): macro = 0.5 +
    // modulations.engine. (val-64)/127 puts the exact engine-stock neutral
    // (0.5) at CC 64.
    case 17: s->shared_macro_cv = ((float)val - 64.0f) / 127.0f; break;
    case 18: s->shared_lpg_colour = v; break;
    case 19: s->shared_decay      = v; break;
    // Modulation amounts span [-1, +1] like the panel pots; CC 64 = off.
    case 20: s->shared_fm_amount  = v * 2.0f - 1.0f; break;
    case 21: s->shared_timbre_mod = v * 2.0f - 1.0f; break;
    case 22: s->shared_morph_mod  = v * 2.0f - 1.0f; break;
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
    case 25: s->shared_level = v; break;
    default: break;
  }
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Same between-blocks discipline as the pending slots in render(); the mod
// engine only runs while not swapping, so `s` is always the live state.

// Continuous destination write (0..1 virtual CC position, float precision).
// CCs 17/20/21/22 apply the SAME range mappings control_change applies to a
// CC byte (macro centered at v=64/127; the ±1 modulation amounts), so a routed
// LFO sweeps the full panel range at level 1.0. 23/24 excluded (selectors).
bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  switch (cc) {
    case 14: s->shared_timbre     = v; break;
    case 15: s->shared_morph      = v; break;
    case 16: s->shared_harmonics  = v; break;
    case 17: s->shared_macro_cv   = (v * 127.0f - 64.0f) / 127.0f; break;
    case 18: s->shared_lpg_colour = v; break;
    case 19: s->shared_decay      = v; break;
    case 20: s->shared_fm_amount  = v * 2.0f - 1.0f; break;
    case 21: s->shared_timbre_mod = v * 2.0f - 1.0f; break;
    case 22: s->shared_morph_mod  = v * 2.0f - 1.0f; break;
    case 25: s->shared_level      = v; break;
    default: return false;
  }
  return true;
}

// Trigger destination write: pseudo-CC 64 = the TRIG input. Rising edges fire
// a full-velocity one-block pulse (the same pending_trigger path note-on
// uses; 1.0 = accent ceiling of the 0.3..1.0 pulse voltage).
bool mod_trigger_cc(uint8_t cc, bool high) {
  if (cc == 64) {
    if (high) s->pending_trigger = 1.0f;
    return true;
  }
  return false;
}

}  // namespace mod_plaits
}  // namespace rack_modules
