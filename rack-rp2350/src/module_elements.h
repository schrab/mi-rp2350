// Rack RP2350 — ELEMENTS adapter (boot-select via RACK_BOOT_MODULE == 0).
//
// Everything here is lifted VERBATIM from elements-rp2350/src/main.cpp +
// midi.h (state sizes, init sequence, per-block call contract, CC map,
// pending-slot discipline, meters): the rack module must sound and behave
// exactly like the accepted standalone build. Only the SELECTED adapter is
// compiled (guard below), so its state is the only module state in RAM.
#pragma once

// The module-library includes are UNCONDITIONAL (outside the RACK_BOOT_MODULE
// guard below): PlatformIO's library dependency finder evaluates the
// preprocessor, so guarded includes would drop the other three module libs
// from the build — but the rack links ALL FOUR modules in every env (their
// .tc_* hot sections fill the tc_* overlay members / the reserved flash
// region, see memmap_rack.ld); only the state + hook bodies below are
// compile-time guarded to the selected module.
#include <STMLIB.h>
#include <ELEMENTS.h>

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (not pulled in by the wrappers)
#include "elements/dsp/dsp.h"
#include "elements/dsp/part.h"
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

// Firmware-side definitions of the elements::Dsp statics. They are declared
// as class statics in elements/dsp/dsp.h but defined nowhere in the library
// (the standalone elements-rp2350/src/main.cpp defines them the same way).
// UNCONDITIONAL (outside the guard below): the ELEMENTS lib is linked in
// every rack env, so every env must provide these definitions.
float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

#if RACK_BOOT_MODULE == 0  // ---- active only in [env:rack-elements] ----

static const char* const kModuleName = "elements";
static const uint32_t kModuleSampleRate = 32000;
static const size_t kModuleBlockSize = elements::kMaxBlockSize;
// Budget = 496 us: the standalone elements report's divide-first grouping
// (1000000/32000 = 31, x16 = 496) — kept for bench-number comparability.
static const unsigned kModuleBudgetUs =
    (unsigned)(1000000UL / kModuleSampleRate * kModuleBlockSize);
static const uint32_t kModuleI2sBuffers = 8;    // 8 x 16 words = 128 frames (~4 ms)
static const uint32_t kModuleI2sWords = 16;
static const uint32_t kModuleStallUs = 900;     // >2x the 500 us block period

// --- engine state (identical sizes/objects to the standalone main.cpp) ---
static uint16_t reverb_buffer[32768];   // 64 KB, verified FxEngine<32768>
static elements::Part part;
static elements::Patch* patch;
static elements::PerformanceState perf_state;

static float blow_in[kModuleBlockSize];
static float strike_in[kModuleBlockSize];
static float main_out[kModuleBlockSize];
static float aux_out[kModuleBlockSize];

// CC hand-off slots (core 0 writes via midiControlChange, core 1 applies
// between render blocks): resonator model (CC 28) + easter egg (CC 29).
static volatile int8_t pending_resonator_model = -1;
static volatile int8_t pending_easter_egg = -1;

// CC 1 / CC 11 shared values (standalone midi.h pattern).
static volatile float shared_modulation = 0.0f;
static volatile float shared_strength = 0.8f;

void module_init() {
  elements::Dsp::setSr(kModuleSampleRate);

  memset(reverb_buffer, 0, sizeof(reverb_buffer));
  memset(blow_in, 0, sizeof(blow_in));
  memset(strike_in, 0, sizeof(strike_in));

  part.Init(reverb_buffer);
  // Seed word count matches the field-proven Scarp sketch, but as a real
  // 3-word array — Seed() hashes all `size` words it is given.
  uint32_t seed[3] = {0x1fff7a10, 0x1fff7a10, 0x1fff7a10};
  part.Seed(seed, 3);
  part.set_easter_egg(false);
  part.set_resonator_model(elements::RESONATOR_MODEL_STRING);

  patch = part.mutable_patch();
  // sensible default: strike/mallet voice
  patch->exciter_envelope_shape = 0.1f;
  patch->exciter_bow_level = 0.0f;
  patch->exciter_blow_level = 0.0f;
  patch->exciter_strike_level = 0.8f;
  patch->exciter_blow_meta = 0.5f;
  patch->exciter_strike_meta = 0.5f;
  patch->exciter_bow_timbre = 0.5f;
  patch->exciter_blow_timbre = 0.5f;
  patch->exciter_strike_timbre = 0.5f;
  patch->exciter_signature = 0.0f;
  patch->resonator_geometry = 0.25f;
  patch->resonator_brightness = 0.4f;
  patch->resonator_damping = 0.7f;
  patch->resonator_position = 0.3f;
  patch->space = 0.4f;
}

void module_render() {
  // Apply at most one pending easter-egg / resonator-model change per block
  // (strictly between two Process() calls; runtime switching is inherently
  // safe — see the standalone main.cpp notes).
  int8_t egg = pending_easter_egg;
  if (egg >= 0) {
    pending_easter_egg = -1;
    part.set_easter_egg(egg != 0);
  }

  int8_t m = pending_resonator_model;
  if (m >= 0) {
    pending_resonator_model = -1;
    part.set_resonator_model(static_cast<elements::ResonatorModel>(m));
  }

  perf_state.gate = shared_gate;
  perf_state.note = shared_note;
  perf_state.strength = shared_strength;
  perf_state.modulation = shared_modulation;

  uint32_t t0 = time_us_32();
  part.Process(perf_state, blow_in, strike_in, main_out, aux_out,
               kModuleBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters: Part's meters carry the original fast-attack / slow-release
  // smoothing (~31 ms exciter / ~62 ms resonator tails) — status LED + report.
  rack_meter_a = part.exciter_level();
  rack_meter_b = part.resonator_level();

  // The writes pace the render loop: one iteration = exactly one 16-word DMA
  // buffer, buffers free every 500 us; > 900 us means no buffer freed for ~2
  // block periods -> I2S output stalled.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    int16_t l = stmlib::Clip16(static_cast<int32_t>(main_out[i] * 32768.0f));
    int16_t r = stmlib::Clip16(static_cast<int32_t>(aux_out[i] * 32768.0f));
    i2s.write16(l, r);   // blocking write, one L/R frame; paces loop at 32 kHz
  }
  if (time_us_32() - w0 > kModuleStallUs) {
    ++i2s_stall_count;
  }
}

void module_note_on(uint8_t note, uint8_t vel) {
  (void)note;
  shared_strength = 0.4f + 0.6f * (static_cast<float>(vel) / 127.0f);
}

void module_note_off(uint8_t note) {
  (void)note;   // gate handled by the shared note stack
}

void module_control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  elements::Patch* p = part.mutable_patch();
  switch (cc) {
    case 1:  shared_modulation = v; break;
    case 11: shared_strength = v; break;
    case 14: p->exciter_envelope_shape = v; break;
    case 15: p->exciter_bow_level = v; break;
    case 16: p->exciter_bow_timbre = v; break;
    case 17: p->exciter_blow_level = v; break;
    case 18: p->exciter_blow_meta = v; break;
    case 19: p->exciter_blow_timbre = v; break;
    case 20: p->exciter_strike_level = v; break;
    case 21: p->exciter_strike_meta = v; break;
    case 22: p->exciter_strike_timbre = v; break;
    case 23: p->resonator_geometry = v; break;
    case 24: p->resonator_brightness = v; break;
    case 25: p->resonator_damping = v; break;
    case 26: p->resonator_position = v; break;
    case 27: p->space = v; break;
    // Resonator model selector: 0 modal / 1 string / 2 strings (values > 2
    // ignored). Handed to core 1 via pending_resonator_model (applied between
    // render blocks).
    case 28:
      if (val <= 2) {
        pending_resonator_model = static_cast<int8_t>(val);
      }
      break;
    // Ominous-voice (easter egg) selector: 0 = normal, 1..127 = ominous.
    // Absolute values, not a toggle — a toggle would desync knob vs engine.
    case 29:
      pending_easter_egg = val ? 1 : 0;
      break;
    default: break;
  }
}

#endif  // RACK_BOOT_MODULE == 0
