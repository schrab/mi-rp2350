// Rack RP2350 — PLAITS adapter (boot-select via RACK_BOOT_MODULE == 2).
//
// Lifted VERBATIM from plaits-rp2350/src/main.cpp + midi.h: the exact
// Patch/Modulations per-block fill of the original firmware main (documented
// inline below), velocity-scaled one-block TRIG pulses, MACRO via
// modulations.engine under model_cv_option = 1, stereo aux via
// aux_output_option = 1, raw write16 (Voice clips/limits internally).
// Only the SELECTED adapter is compiled; plaits runs at its native 48 kHz /
// 24-sample blocks when selected.
#pragma once

// Module-library includes UNCONDITIONAL (see module_elements.h): all four
// module libs are linked in every rack env; only the state + hook bodies
// below are compile-time guarded.
#include <STMLIB.h>
#include <PLAITS.h>

#include "plaits/dsp/voice.h"    // plaits::Voice, Patch, Modulations
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

// The MIDI CC map bounds engine selection to the Plaits-Lab catalog size;
// this must equal the registry the lib was generated with
// (-DPLAITS_ENGINE_COUNT=98 reaches src/ and lib/ alike in every rack env).
static const uint8_t kPlaitsEngineCount = 98;
static_assert(plaits::kMaxEngines == kPlaitsEngineCount,
              "module_plaits catalog size must match PLAITS_ENGINE_COUNT");

#if RACK_BOOT_MODULE == 2  // ---- active only in [env:rack-plaits] ----

static const char* const kModuleName = "plaits";
static const uint32_t kModuleSampleRate = 48000;               // = plaits::kSampleRate
static const size_t kModuleBlockSize = plaits::kMaxBlockSize;  // 24 samples
static const unsigned kModuleBudgetUs =
    (unsigned)(1000000UL * kModuleBlockSize / kModuleSampleRate);  // 500
static const uint32_t kModuleI2sBuffers = 48;   // 48 x 16 words = 768 frames
static const uint32_t kModuleI2sWords = 16;     // = 16 ms bank at 48 kHz
static const uint32_t kModuleStallUs = 900;     // >2x the 500 us block period

// --- engine state (identical sizes/objects to the standalone main.cpp) ---
// plaits::Voice embeds all 98 engines; their state RAM is carved per engine
// from one shared pool by stmlib::BufferAllocator (voice.cc:105-113 frees
// before every engine Init) — 32 KB covers the largest single engine
// (16.4 KB) with margin. aligned(8): Allocate<T> does not adjust alignment.
static uint32_t workspace[8192] __attribute__((aligned(8)));   // 32 KB

static plaits::Voice voice;
static plaits::Voice::Frame frames[kModuleBlockSize];  // interleaved stereo shorts

// Patch/Modulations are filled per block by core 1. Defaults mirror a
// centered panel at boot; amounts are exactly neutral (0 = no modulation).
static plaits::Patch patch;
static plaits::Modulations mod;

// Panel parameters (core 0 writes, core 1 copies into the Patch/Modulations).
static volatile float shared_timbre = 0.5f;      // CC 14
static volatile float shared_morph = 0.5f;       // CC 15
static volatile float shared_harmonics = 0.5f;   // CC 16
static volatile float shared_macro_cv = 0.0f;    // CC 17: (val-64)/127
static volatile float shared_lpg_colour = 0.5f;  // CC 18
static volatile float shared_decay = 0.5f;       // CC 19
static volatile float shared_fm_amount = 0.0f;   // CC 20: v*2-1
static volatile float shared_timbre_mod = 0.0f;  // CC 21: v*2-1
static volatile float shared_morph_mod = 0.0f;   // CC 22: v*2-1
static volatile float shared_level = 0.8f;       // CC 25

static volatile int8_t   pending_engine = -1;      // -1 none, 0..97 engine
static volatile int8_t   pending_aux_option = -1;  // -1 none, 0/1/2 aux option
static volatile float    pending_trigger = -1.0f;  // <0 none, else pulse voltage

// Active engine mirror for the 2 s report (core 1 publishes, core 0 prints).
static volatile uint32_t active_engine_report = 0;

void module_init() {
  // One shared workspace pool carved per engine; the corrected-sample-rate
  // define makes the fork's module-clock pitch shim exactly zero (I2S at
  // true 48 kHz).
  memset(workspace, 0, sizeof(workspace));
  stmlib::BufferAllocator allocator(workspace, sizeof(workspace));
  voice.Init(&allocator);

  // Patch option fields the CC map does not reach (voice.h), set to the
  // MIDI-sensible defaults the per-block fill relies on:
  patch.engine = 0;                      // CC 23 from here on
  patch.aux_output_option = 1;           // stereo OUT/AUX (CC 24)
  patch.aux_subosc_option = 0;           // square, no octave shift (option 2)
  patch.trig_response_option = 2;        // velocity trigger
  patch.model_cv_option = 1;             // MODEL CV = fourth macro (CC 17)
  patch.locked_frequency_pot_option = 0; // no pitch lock: DECAY is decay
  patch.level_cv_option = 0;             // LEVEL stays level
  patch.hold_on_trigger_option = 0;      // no parameter hold on trigger
  patch.chord_set_option = 0;            // first chord table (chord engines)
  patch.attenuverter_mode = plaits::ATTENUVERTER_MODE_STOCK;  // 0
  patch.freqlock_param = 0.5f;           // unused while locked_frequency=0
}

void module_render() {
  // Apply pending changes strictly between two Render() calls. Engine
  // changes go through patch.engine; Voice reloads/resets the engine itself
  // at the top of Render (voice.cc:382-489).
  if (pending_engine >= 0) {
    int engine = pending_engine;
    if (engine > plaits::kMaxEngines - 1) engine = plaits::kMaxEngines - 1;
    patch.engine = engine;
    pending_engine = -1;
  }
  if (pending_aux_option >= 0) {
    patch.aux_output_option = static_cast<uint8_t>(pending_aux_option);
    pending_aux_option = -1;
  }

  // TRIG pulse: one block high at the note-on velocity, then low so
  // trigger_state_ clears. Rapid re-notes each get their own edge.
  if (pending_trigger > -0.5f) {
    mod.trigger = pending_trigger;
    pending_trigger = -1.0f;
  } else {
    mod.trigger = 0.0f;
  }

  // --- Patch fill (units documented in plaits-rp2350/src/main.cpp) ---
  patch.note = shared_note;
  patch.harmonics = shared_harmonics;
  patch.timbre = shared_timbre;
  patch.morph = shared_morph;
  patch.frequency_modulation_amount = shared_fm_amount;
  patch.timbre_modulation_amount = shared_timbre_mod;
  patch.morph_modulation_amount = shared_morph_mod;
  patch.decay = shared_decay;
  patch.lpg_colour = shared_lpg_colour;

  // --- Modulations fill: TRIG + LEVEL patched, everything else unpatched ---
  mod.engine = shared_macro_cv;        // fourth macro under model_cv_option=1
  mod.note = 0.0f;
  mod.frequency = 0.0f;
  mod.harmonics = 0.0f;
  mod.timbre = 0.0f;
  mod.morph = 0.0f;
  mod.level = shared_gate ? shared_level : 0.0f;  // held = sustain, off = release
  mod.hard_sync = 0;
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    mod.frequency_audio[i] = 0.0f;     // unused: PLAITS_BUILD_FREQUENCY_OFFSET_FM=0
  }
  mod.frequency_audio_rate = false;
  mod.frequency_patched = false;
  mod.timbre_patched = false;
  mod.morph_patched = false;
  mod.trigger_patched = true;
  mod.level_patched = true;

  uint32_t t0 = time_us_32();
  // Render() every block (the original firmware contract: plaits.cc:254).
  // Output Frames are already limited/clipped to int16 inside Voice — write
  // them raw, do NOT double-clip.
  voice.Render(patch, mod, frames, kModuleBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  active_engine_report = (uint32_t)voice.active_engine();

  // Meters: OUT/AUX block RMS, max since the last 2 s report.
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    const float l = static_cast<float>(frames[i].out) / 32768.0f;
    const float r = static_cast<float>(frames[i].aux) / 32768.0f;
    out_acc += l * l;
    aux_acc += r * r;
  }
  const float out_rms = sqrtf(out_acc / kModuleBlockSize);
  const float aux_rms = sqrtf(aux_acc / kModuleBlockSize);
  if (out_rms > rack_meter_a) rack_meter_a = out_rms;
  if (aux_rms > rack_meter_b) rack_meter_b = aux_rms;

  // The writes pace the render loop: one iteration = one 24-frame DMA block,
  // blocks free every 500 us; > 900 us = stalled.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    i2s.write16(frames[i].out, frames[i].aux);
  }
  if (time_us_32() - w0 > kModuleStallUs) {
    ++i2s_stall_count;
  }
}

void module_note_on(uint8_t note, uint8_t vel) {
  (void)note;
  // TRIG pulse voltage: velocity scaled to always cross Voice's 0.3 trigger
  // threshold — 0.3 + 0.7 * v — latched as the accent velocity on the
  // (delayed) rising edge. Consumed by core 1 on the next block.
  pending_trigger = 0.3f + 0.7f * (float)vel / 127.0f;
}

void module_note_off(uint8_t note) {
  (void)note;   // gate -> LEVEL release handled by the shared note stack
}

void module_control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 14: shared_timbre     = v; break;
    case 15: shared_morph      = v; break;
    case 16: shared_harmonics  = v; break;
    // MACRO/TWIST as the MODEL CV (patch.model_cv_option = 1): macro = 0.5 +
    // modulations.engine. (val-64)/127 puts the exact engine-stock neutral
    // (0.5) at CC 64.
    case 17: shared_macro_cv = ((float)val - 64.0f) / 127.0f; break;
    case 18: shared_lpg_colour = v; break;
    case 19: shared_decay      = v; break;
    // Modulation amounts span [-1, +1] like the panel pots; CC 64 = off.
    case 20: shared_fm_amount  = v * 2.0f - 1.0f; break;
    case 21: shared_timbre_mod = v * 2.0f - 1.0f; break;
    case 22: shared_morph_mod  = v * 2.0f - 1.0f; break;
    // ENGINE: raw value = registry index (0-97). Values beyond the catalog
    // are ignored; applied between render blocks.
    case 23:
      if (val < kPlaitsEngineCount) pending_engine = static_cast<int8_t>(val);
      break;
    // AUX OPTION: 0 regular aux model / 1 stereo (OUT=L, AUX=R on
    // stereo-capable engines) / 2 suboscillator.
    case 24:
      if (val <= 2) pending_aux_option = static_cast<int8_t>(val);
      break;
    case 25: shared_level = v; break;
    default: break;
  }
}

#endif  // RACK_BOOT_MODULE == 2
