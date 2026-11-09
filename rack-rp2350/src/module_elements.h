// Rack RP2350 — ELEMENTS adapter.
//
// Everything here is lifted VERBATIM from elements-rp2350/src/main.cpp +
// midi.h (state sizes, init sequence, per-block call contract, CC map,
// pending-slot discipline, meters): the rack module must sound and behave
// exactly like the accepted standalone build.
//
// T2: this adapter compiles in EVERY rack env (no RACK_BOOT_MODULE guard —
// all four modules link and provide hooks; the registry in main.cpp dispatches
// to the ACTIVE one). Its state lives in the T2 ARENAS (rack_arena.h):
// placement-new'd at Init time, file-scope state objects are gone.
#pragma once

// The module-library includes are UNCONDITIONAL: PlatformIO's library
// dependency finder evaluates the preprocessor, so guarded includes would
// drop the other three module libs from the build — the rack links ALL FOUR
// modules in every env (their .tc_* hot sections fill the tc_* OVERLAY
// members, see memmap_rack.ld).
#include <STMLIB.h>
#include <ELEMENTS.h>

#include <new>                   // placement-new into the arenas

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (not pulled in by the wrappers)
#include "elements/dsp/dsp.h"
#include "elements/dsp/part.h"
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"
#include "module_registry.h"     // RackModDestinations (T3 mod routing table)

// Firmware-side definitions of the elements::Dsp statics. They are declared
// as class statics in elements/dsp/dsp.h but defined nowhere in the library
// (the standalone elements-rp2350/src/main.cpp defines them the same way).
// The ELEMENTS lib is linked in every rack env, so every env provides these.
float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

namespace rack_modules {
namespace mod_elements {

static const char* const kName = "elements";
static const uint32_t kSampleRate = 32000;
static const size_t kBlockSize = elements::kMaxBlockSize;
// Budget = 496 us: the standalone elements report's divide-first grouping
// (1000000/32000 = 31, x16 = 496) — kept for bench-number comparability.
static const uint32_t kBudgetUs =
    (uint32_t)(1000000UL / kSampleRate * kBlockSize);
static const uint32_t kI2sBuffers = 8;    // 8 x 16 words = 128 frames (~4 ms)
static const uint32_t kI2sWords = 16;
static const uint32_t kStallUs = 900;     // >2x the 500 us block period

// Registry strings (see module_registry.h).
static const char* const kCcMap =
    "1 pitch, 11 strength, 14 env-shape, 15-16 bow, 17-19 blow, 20-22 strike, "
    "23 geometry, 24 brightness, 25 damping, 26 position, 27 space, "
    "28 model, 29 easter-egg";
// T3 mod destinations: every float-parameter CC of the map accepts continuous
// modulation (1/11 included — note that velocity-scaled strength from note-on
// is overridden while a source is routed to CC 11). Elements has NO trigger
// inputs (excitation is envelope/gate driven): routing a trigger source
// (euclid/gate) to any elements destination is inert. Selectors 28/29 are
// excluded (int model/easter-egg switches — not modulable).
static const uint8_t kModContinuousCcs[] = {1, 11, 14, 15, 16, 17, 18, 19,
                                            20, 21, 22, 23, 24, 25, 26, 27};
static const RackModDestinations kModDestinations = {
    "elements: continuous 1,11,14-27 | triggers: none (no trigger inputs)",
    16, kModContinuousCcs,
    0, 0, 0,
};

// --- engine state: arena-placed (was file-scope before T2) ---
struct State {
  elements::Part part;
  elements::PerformanceState perf_state;
  elements::Patch* patch;
  float blow_in[kBlockSize];
  float strike_in[kBlockSize];
  float main_out[kBlockSize];
  float aux_out[kBlockSize];

  // CC hand-off slots (core 0 writes via midiControlChange, core 1 applies
  // between render blocks): resonator model (CC 28) + easter egg (CC 29).
  volatile int8_t pending_resonator_model;
  volatile int8_t pending_easter_egg;

  // CC 1 (PITCH: -24..+24 st) / CC 11 shared values (standalone midi.h pattern).
  volatile float shared_pitch;
  volatile float shared_strength;
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

// T6 chain hooks (see main.cpp's chain state machine): state_size() +
// init_at() let the chain place TWO module states side by side in the OBJECT
// arena (the arena is a union — chain mode needs the A state and the
// clouds-B state live at once). init() stays the single-module path
// (placement at arena offset 0, exactly as before T6).
static size_t state_size() { return sizeof(State); }

void init_at(uint8_t* base) {
  s = new (base) State();
  // Buffer carve: uint16_t reverb[32768] = 64 KB from the buffer arena.
  uint16_t* reverb_buffer = reinterpret_cast<uint16_t*>(g_rack_buffer_arena);
  memset(reverb_buffer, 0, 32768 * sizeof(uint16_t));
  memset(s->blow_in, 0, sizeof(s->blow_in));
  memset(s->strike_in, 0, sizeof(s->strike_in));
  s->pending_resonator_model = -1;
  s->pending_easter_egg = -1;
  s->shared_pitch = 0.0f;
  s->shared_strength = 0.8f;

  elements::Dsp::setSr(kSampleRate);
  s->part.Init(reverb_buffer);
  // Seed word count matches the field-proven Scarp sketch, but as a real
  // 3-word array — Seed() hashes all `size` words it is given.
  uint32_t seed[3] = {0x1fff7a10, 0x1fff7a10, 0x1fff7a10};
  s->part.Seed(seed, 3);
  s->part.set_easter_egg(false);
  s->part.set_resonator_model(elements::RESONATOR_MODEL_STRING);

  s->patch = s->part.mutable_patch();
  // sensible default: strike/mallet voice
  elements::Patch* p = s->patch;
  p->exciter_envelope_shape = 0.1f;
  p->exciter_bow_level = 0.0f;
  p->exciter_blow_level = 0.0f;
  p->exciter_strike_level = 0.8f;
  p->exciter_blow_meta = 0.5f;
  p->exciter_strike_meta = 0.5f;
  p->exciter_bow_timbre = 0.5f;
  p->exciter_blow_timbre = 0.5f;
  p->exciter_strike_timbre = 0.5f;
  p->exciter_signature = 0.0f;
  p->resonator_geometry = 0.25f;
  p->resonator_brightness = 0.4f;
  p->resonator_damping = 0.7f;
  p->resonator_position = 0.3f;
  p->space = 0.4f;
}

void init() { init_at(g_rack_object_arena); }

// T6 chain A-slot render: exactly n samples (n <= 24, the chain grid) into
// caller buffers, NO I2S write, NO proc_us_max update (the chain loop times
// A + route + B together, per decision 6). Part::Process is fully size-
// parameterized but its internal scratch is kMaxBlockSize = 16, so any n is
// rendered as <= 16-sample sub-calls (24 = 16 + 8). The pending-slot apply
// and perf_state fill are identical to render() — core-0 CCs keep landing on
// the chain A slot through the registry dispatch while chained. Meters are
// NOT written here: while chained, rack_meter_a/b report the B slot's
// (clouds) input/output RMS — the LED + report meters become the effect
// stage's meters.
void chain_render_a(float* out_l, float* out_r, size_t n) {
  int8_t egg = s->pending_easter_egg;
  if (egg >= 0) {
    s->pending_easter_egg = -1;
    s->part.set_easter_egg(egg != 0);
  }
  int8_t m = s->pending_resonator_model;
  if (m >= 0) {
    s->pending_resonator_model = -1;
    s->part.set_resonator_model(static_cast<elements::ResonatorModel>(m));
  }

  s->perf_state.gate = shared_gate;
  s->perf_state.note = shared_note;
  s->perf_state.strength = s->shared_strength;
  s->perf_state.modulation = s->shared_pitch;

  size_t done = 0;
  while (done < n) {
    const size_t chunk =
        (n - done > kBlockSize) ? (size_t)kBlockSize : (n - done);
    s->part.Process(s->perf_state, s->blow_in, s->strike_in, s->main_out,
                    s->aux_out, chunk);
    for (size_t i = 0; i < chunk; ++i) {
      out_l[done + i] = s->main_out[i];
      out_r[done + i] = s->aux_out[i];
    }
    done += chunk;
  }
}

void render() {
  // Apply at most one pending easter-egg / resonator-model change per block
  // (strictly between two Process() calls; runtime switching is inherently
  // safe — see the standalone main.cpp notes).
  int8_t egg = s->pending_easter_egg;
  if (egg >= 0) {
    s->pending_easter_egg = -1;
    s->part.set_easter_egg(egg != 0);
  }

  int8_t m = s->pending_resonator_model;
  if (m >= 0) {
    s->pending_resonator_model = -1;
    s->part.set_resonator_model(static_cast<elements::ResonatorModel>(m));
  }

  s->perf_state.gate = shared_gate;
  s->perf_state.note = shared_note;
  s->perf_state.strength = s->shared_strength;
  s->perf_state.modulation = s->shared_pitch;

  uint32_t t0 = time_us_32();
  s->part.Process(s->perf_state, s->blow_in, s->strike_in, s->main_out,
                  s->aux_out, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters: Part's meters carry the original fast-attack / slow-release
  // smoothing (~31 ms exciter / ~62 ms resonator tails) — status LED + report.
  rack_meter_a = s->part.exciter_level();
  rack_meter_b = s->part.resonator_level();

  // The writes pace the render loop: one iteration = exactly one 16-word DMA
  // buffer, buffers free every 500 us; > 900 us means no buffer freed for ~2
  // block periods -> I2S output stalled. g_render_gain is the swap-mute ramp
  // (1.0 in normal render; 0..1 ramped by the core-1 swap sequence).
  uint32_t w0 = time_us_32();
  const float gain = g_render_gain;
  for (size_t i = 0; i < kBlockSize; ++i) {
    int16_t l = stmlib::Clip16(
        static_cast<int32_t>(s->main_out[i] * 32768.0f * gain));
    int16_t r = stmlib::Clip16(
        static_cast<int32_t>(s->aux_out[i] * 32768.0f * gain));
    i2s.write16(l, r);   // blocking write, one L/R frame; paces loop at 32 kHz
  }
  if (time_us_32() - w0 > kStallUs) {
    ++i2s_stall_count;
  }
}

void note_on(uint8_t note, uint8_t vel) {
  (void)note;
  s->shared_strength = 0.4f + 0.6f * (static_cast<float>(vel) / 127.0f);
}

void note_off(uint8_t note) {
  (void)note;   // gate handled by the shared note stack
}

void control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  elements::Patch* p = s->part.mutable_patch();
  switch (cc) {
    case 1:  s->shared_pitch = (v - 0.5f) * 48.0f; break; // CC 1 = PITCH: -24..+24 semitones
    case 11: s->shared_strength = v; break;
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
        s->pending_resonator_model = static_cast<int8_t>(val);
      }
      break;
    // Ominous-voice (easter egg) selector: 0 = normal, 1..127 = ominous.
    // Absolute values, not a toggle — a toggle would desync knob vs engine.
    case 29:
      s->pending_easter_egg = val ? 1 : 0;
      break;
    default: break;
  }
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Called by the persistent mod engine (mod_engine.h) at the loop-top, BEFORE
// render — the same between-blocks discipline as the pending slots above. The
// engine runs only while not swapping, so `s` always points at the live state.

// Continuous destination write: v is a 0..1 "virtual CC position" (clamped
// here) applied with the EXACT mapping control_change uses for a CC byte, but
// at float precision (no 7-bit quantization of the modulated value). While a
// source is routed, it fully owns the parameter — the static CC value is
// overridden each block (standard mod-matrix behavior; un-route with dest=0
// to hand the parameter back to the panel). Returns false for CCs that are
// not continuous destinations (selectors 28/29, unknown CCs) — the engine
// treats those routings as inert.
bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  elements::Patch* p = s->part.mutable_patch();
  switch (cc) {
    case 1:  s->shared_pitch = (v - 0.5f) * 48.0f; break; // CC 1 = PITCH: -24..+24 semitones
    case 11: s->shared_strength = v; break;
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
    default: return false;    // 28/29 selectors + unknown CCs: not modulable
  }
  return true;
}

// Trigger destination hook: elements has no trigger inputs (see the
// kModDestinations note) — always inert.
bool mod_trigger_cc(uint8_t cc, bool high) {
  (void)cc; (void)high;
  return false;
}

}  // namespace mod_elements
}  // namespace rack_modules
