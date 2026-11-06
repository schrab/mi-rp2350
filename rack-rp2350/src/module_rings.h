// Rack RP2350 — RINGS adapter.
//
// Lifted VERBATIM from rings-rp2350/src/main.cpp + midi.h: fresh
// Patch/PerformanceState locals every block (rings.cc contract), one caller-
// owned reverb buffer shared by both Parts (mutually exclusive by mode),
// internal_exciter = true (self-exciting synth, no input jack), strum pulses
// from MIDI note-ons, the clamped chord index (Joe table's per-polyphony
// quirk included) and the ODD/EVEN out/aux convention with our Clip16.
// NOTE: the fork has NO Dsp::setSr — kSampleRate 32000 is a compile-time
// constant baked into the lib (the standalone main.cpp defines no Dsp
// statics; none needed here).
//
// T2: compiles in EVERY rack env (state is arena-placed, see rack_arena.h);
// rings is the OBJECT arena's sizing module (Part 123,352 B +
// StringSynthPart 2,004 B) and carves its 64 KiB reverb buffer from the
// BUFFER arena.
#pragma once

// Module-library includes UNCONDITIONAL (see module_elements.h): all four
// module libs are linked in every rack env.
#include <STMLIB.h>
#include <RINGS.h>               // rings::Part + rings::StringSynthPart

#include <new>                   // placement-new into the arenas

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (rings has no internal clipper)
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"
#include "module_registry.h"     // RackModDestinations (T3 mod routing table)

namespace rack_modules {
namespace mod_rings {

static const char* const kName = "rings";
static const uint32_t kSampleRate = 32000;               // = rings::kSampleRate
static const size_t kBlockSize = rings::kMaxBlockSize;   // 24 samples
static const uint32_t kBudgetUs =
    (uint32_t)(1000000UL * kBlockSize / kSampleRate);  // 750
static const uint32_t kI2sBuffers = 32;   // 32 x 16 words = 512 frames
static const uint32_t kI2sWords = 16;     // = 16 ms bank at 32 kHz
static const uint32_t kStallUs = 1400;    // ~1.8x the 750 us block period

static const char* const kCcMap =
    "14 structure, 15 brightness, 16 damping, 17 position, 18 model/fx, "
    "19 polyphony, 20 mode, 21 waveform, 22 chord-table, 23 chord";
// T3 mod destinations: the four float patch parameters 14-17 accept continuous
// modulation; CC 23 (chord index) is included as a STEPPED destination (the
// value is rounded into the pending_chord int8 and clamped against the active
// chord table at fill time — modulating it sweeps chords). 18-22 are EXCLUDED
// (model/fx, polyphony, mode, waveform, chord-table selectors — per-block
// selector writes would spam the between-blocks apply path with no musical
// meaning). Pseudo-CC 64 (verified unused by all four maps) = the STRUM
// input: pulse-kind, one block per rising edge (same path as MIDI note-on).
static const uint8_t kModContinuousCcs[] = {14, 15, 16, 17, 23};
static const uint8_t kModTriggerCcs[] = {64};
static const uint8_t kModTriggerKinds[] = {RACK_TRIG_PULSE};
static const RackModDestinations kModDestinations = {
    "rings: continuous 14-17,23 (18-22 excluded: model/poly/mode/wave/"
    "chord-table selectors) | triggers: 64 strum (pulse)",
    5, kModContinuousCcs,
    1, kModTriggerCcs, kModTriggerKinds,
};

// --- engine state: arena-placed (was file-scope before T2) ---
struct State {
  // Both Parts over the one shared reverb buffer (rings.cc:43 uses the same
  // single buffer for part.Init + string_synth.Init). Safe because the
  // firmware contract renders exactly ONE part per block (mode-selected).
  rings::Part part;
  rings::StringSynthPart string_synth;

  float in[kBlockSize];    // no input jack: fed with silence
  float out[kBlockSize];
  float aux[kBlockSize];

  // Panel parameters (core 0 writes, core 1 copies into the fresh per-block
  // Patch — same shared-state pattern as the standalone port).
  volatile float shared_structure;   // CC 14 (clamped <= 0.9995)
  volatile float shared_brightness;  // CC 15
  volatile float shared_damping;     // CC 16
  volatile float shared_position;    // CC 17

  volatile int8_t  pending_model;         // -1 none, 0..5 model/FxType
  volatile int8_t  pending_polyphony;     // -1 none, 1..4
  volatile int8_t  pending_mode;          // -1 none, 0..4 mode
  volatile int8_t  pending_waveform;      // -1 none, 0..2 waveform
  volatile int8_t  pending_chord_table;   // -1 none, 0..2 ChordTable
  volatile int8_t  pending_chord;         // -1 none, 0..127 raw
  volatile bool    pending_strum;         // one-block strum pulse

  // Applied-state mirrors for the 2 s report (model is the APPLIED CC 18
  // byte, clamped 0-5: ResonatorModel index in modes 0-3, FxType index in
  // mode 4).
  volatile uint32_t active_mode_report;
  volatile uint32_t active_model_report;

  // --- applied settings (core 1 only; applied between blocks) ---
  int32_t applied_mode;
  int32_t applied_model;
  int32_t applied_polyphony;
  int32_t applied_waveform;
  int32_t applied_chord_table;
  int32_t applied_chord;
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

// Number of valid chord indexes for the current table/mode/polyphony — a
// mirror of CvScaler::NumChords (cv_scaler.cc:289-309). CHORD_TABLE_JOE is
// the quirk: 18 chords in the easter egg, otherwise the count depends on
// polyphony (part.cc:292-...).
static inline int32_t NumChordsFor(int32_t table, int32_t mode, int32_t polyphony) {
  using namespace rings;  // ChordTable/PerformanceMode enums + kNum*Chords
  switch (table) {
    case CHORD_TABLE_BRYAN:
      return kNumBryanChords;             // 11
    case CHORD_TABLE_JON:
      return kNumJonChords;               // 17
    default:                              // CHORD_TABLE_JOE
      if (mode == MODE_EASTER_EGG) return kNumJoeEasterEggChords;  // 18
      if (polyphony == 1) return 15;
      if (polyphony == 2) return 21;
      if (polyphony == 3) return 16;
      return kMaxNumJoeChords;            // 23
  }
}

// T6 chain hooks (see module_elements.h): state_size()/init_at() keep the
// single-module path. Rings is 32k-native — no rate-context hook needed
// (unlike plaits, whose init_chain_a re-captures the runtime rate).
static size_t state_size() { return sizeof(State); }

void init_at(uint8_t* base) {
  s = new (base) State();
  // Buffer carve: uint16_t reverb[32768] = 64 KB from the buffer arena.
  uint16_t* reverb_buffer = reinterpret_cast<uint16_t*>(g_rack_buffer_arena);
  memset(reverb_buffer, 0, 32768 * sizeof(uint16_t));

  s->shared_structure = 0.5f;
  s->shared_brightness = 0.5f;
  s->shared_damping = 0.5f;
  s->shared_position = 0.5f;
  s->pending_model = -1;
  s->pending_polyphony = -1;
  s->pending_mode = -1;
  s->pending_waveform = -1;
  s->pending_chord_table = -1;
  s->pending_chord = -1;
  s->pending_strum = false;
  s->active_mode_report = 0;
  s->active_model_report = 0;

  // Both Parts over the one shared reverb buffer. Defaults match Part::Init
  // (MODAL, polyphony 1) and StringSynthPart::Init; the model byte 0 also
  // becomes FX_FORMANT so CC 18 means the same index in every mode.
  s->part.Init(reverb_buffer);
  s->string_synth.Init(reverb_buffer);
  s->part.set_polyphony(1);
  s->part.set_model(rings::RESONATOR_MODEL_MODAL);
  s->string_synth.set_polyphony(1);
  s->string_synth.set_fx(rings::FX_FORMANT);
  s->applied_mode = rings::MODE_RINGS_STEREO;        // 0
  s->applied_model = 0;                              // RESONATOR_MODEL_MODAL
  s->applied_polyphony = 1;
  s->applied_waveform = 0;                           // square / bow
  s->applied_chord_table = rings::CHORD_TABLE_BRYAN; // 0
  s->applied_chord = 0;                              // raw, clamped at fill
}

void init() { init_at(g_rack_object_arena); }

// T6: chain A-slot render — exactly n samples (n <= 24 = kMaxBlockSize) into
// caller buffers (out/aux = the ODD/EVEN stereo convention), NO I2S write,
// NO proc_us_max update. NOT reachable in this build (the pair table rejects
// rings as chain A on window capacity — see main.cpp) but kept wired so
// enabling the pair later is a linker-numbers change, not new code.
void chain_render_a(float* out_l, float* out_r, size_t n) {
  // --- apply pending changes strictly between two Process() calls ---
  if (s->pending_mode >= 0) {
    s->applied_mode = s->pending_mode <= rings::MODE_EASTER_EGG
        ? s->pending_mode : rings::MODE_EASTER_EGG;
    s->pending_mode = -1;
  }
  if (s->pending_model >= 0) {
    s->applied_model = s->pending_model <= 5 ? s->pending_model : 5;
    s->part.set_model(static_cast<rings::ResonatorModel>(s->applied_model));
    s->string_synth.set_fx(static_cast<rings::FxType>(s->applied_model));
    s->pending_model = -1;
  }
  if (s->pending_polyphony >= 0) {
    int32_t p = s->pending_polyphony;
    if (p < 1) p = 1;
    if (p > rings::kMaxPolyphony) p = rings::kMaxPolyphony;
    s->applied_polyphony = p;
    s->part.set_polyphony(p);
    s->string_synth.set_polyphony(p);
    s->pending_polyphony = -1;
  }
  if (s->pending_waveform >= 0) {
    s->applied_waveform = s->pending_waveform <= 2 ? s->pending_waveform : 2;
    s->pending_waveform = -1;
  }
  if (s->pending_chord_table >= 0) {
    s->applied_chord_table = s->pending_chord_table <= rings::CHORD_TABLE_JOE
        ? s->pending_chord_table : rings::CHORD_TABLE_JOE;
    s->pending_chord_table = -1;
  }
  if (s->pending_chord >= 0) {
    s->applied_chord = s->pending_chord;
    s->pending_chord = -1;
  }

  const bool strum_pulse = s->pending_strum;
  s->pending_strum = false;

  rings::PerformanceState perf;
  rings::Patch patch;

  perf.internal_exciter = true;
  perf.internal_strum = false;
  perf.internal_note = false;
  perf.mode = static_cast<rings::PerformanceMode>(s->applied_mode);
  perf.chord_table = static_cast<rings::ChordTable>(s->applied_chord_table);
  perf.waveform_exciter = static_cast<uint8_t>(s->applied_waveform);
  perf.strum_hold_option = 0;
  perf.fm_input_option = 0;
  perf.frequency_locked = 0;
  perf.locked_frequency_pot_value = 0.0f;
  perf.tonic = 0.0f;
  perf.fm = 0.0f;
  perf.note = shared_note;
  perf.strum = strum_pulse;
  perf.strum_gate = shared_gate;
  perf.chord = s->applied_chord;
  {
    const int32_t num_chords =
        NumChordsFor(s->applied_chord_table, s->applied_mode, s->applied_polyphony);
    if (perf.chord > num_chords - 1) perf.chord = num_chords - 1;
    if (perf.chord < 0) perf.chord = 0;
  }

  patch.structure = s->shared_structure;
  if (patch.structure > 0.9995f) patch.structure = 0.9995f;
  if (patch.structure < 0.0f) patch.structure = 0.0f;
  patch.brightness = s->shared_brightness;
  patch.damping = s->shared_damping;
  patch.position = s->shared_position;

  memset(s->in, 0, sizeof(s->in));

  if (s->applied_mode == rings::MODE_EASTER_EGG) {
    s->string_synth.Process(perf, patch, s->in, s->out, s->aux, n);
  } else {
    s->part.Process(perf, patch, s->in, s->out, s->aux, n);
  }
  for (size_t i = 0; i < n; ++i) {
    out_l[i] = s->out[i];
    out_r[i] = s->aux[i];
  }
}

void render() {
  // --- apply pending changes strictly between two Process() calls ---
  if (s->pending_mode >= 0) {
    s->applied_mode = s->pending_mode <= rings::MODE_EASTER_EGG
        ? s->pending_mode : rings::MODE_EASTER_EGG;
    s->pending_mode = -1;
  }
  if (s->pending_model >= 0) {
    s->applied_model = s->pending_model <= 5 ? s->pending_model : 5;
    // The SAME byte drives the resonator model (modes 0-3) and the easter
    // egg's FxType (mode 4); apply to both so a mode flip lands on the
    // already-selected model/FX.
    s->part.set_model(static_cast<rings::ResonatorModel>(s->applied_model));
    s->string_synth.set_fx(static_cast<rings::FxType>(s->applied_model));
    s->pending_model = -1;
  }
  if (s->pending_polyphony >= 0) {
    int32_t p = s->pending_polyphony;
    if (p < 1) p = 1;
    if (p > rings::kMaxPolyphony) p = rings::kMaxPolyphony;
    s->applied_polyphony = p;
    s->part.set_polyphony(p);            // Part clamps too (part.h:89-96)
    s->string_synth.set_polyphony(p);    // (string_synth_part.h:87-96)
    s->pending_polyphony = -1;
  }
  if (s->pending_waveform >= 0) {
    s->applied_waveform = s->pending_waveform <= 2 ? s->pending_waveform : 2;
    s->pending_waveform = -1;
  }
  if (s->pending_chord_table >= 0) {
    s->applied_chord_table = s->pending_chord_table <= rings::CHORD_TABLE_JOE
        ? s->pending_chord_table : rings::CHORD_TABLE_JOE;
    s->pending_chord_table = -1;
  }
  if (s->pending_chord >= 0) {
    s->applied_chord = s->pending_chord;    // raw 0..127, clamped at fill time
    s->pending_chord = -1;
  }

  // Strum pulse: exactly one block high per note-on.
  const bool strum_pulse = s->pending_strum;
  s->pending_strum = false;

  // --- fresh Patch + PerformanceState locals EVERY block (rings.cc:92-96) ---
  rings::PerformanceState perf;
  rings::Patch patch;

  perf.internal_exciter = true;
  perf.internal_strum = false;
  perf.internal_note = false;
  perf.mode = static_cast<rings::PerformanceMode>(s->applied_mode);
  perf.chord_table = static_cast<rings::ChordTable>(s->applied_chord_table);
  perf.waveform_exciter = static_cast<uint8_t>(s->applied_waveform);
  perf.strum_hold_option = 0;
  perf.fm_input_option = 0;
  perf.frequency_locked = 0;
  perf.locked_frequency_pot_value = 0.0f;
  perf.tonic = 0.0f;
  perf.fm = 0.0f;
  perf.note = shared_note;
  perf.strum = strum_pulse;
  perf.strum_gate = shared_gate;
  perf.chord = s->applied_chord;
  {
    const int32_t num_chords =
        NumChordsFor(s->applied_chord_table, s->applied_mode, s->applied_polyphony);
    if (perf.chord > num_chords - 1) perf.chord = num_chords - 1;
    if (perf.chord < 0) perf.chord = 0;
  }

  patch.structure = s->shared_structure;
  if (patch.structure > 0.9995f) patch.structure = 0.9995f;  // cv_scaler.cc:187
  if (patch.structure < 0.0f) patch.structure = 0.0f;
  patch.brightness = s->shared_brightness;
  patch.damping = s->shared_damping;
  patch.position = s->shared_position;

  // No input jack: silence in. 0.0f is all-zero bytes, so memset is an exact
  // float-zero fill.
  memset(s->in, 0, sizeof(s->in));

  uint32_t t0 = time_us_32();
  // ONE part renders per block (rings.cc:98-116): modes 0-3 -> Part, mode 4
  // (easter egg) -> StringSynthPart.
  if (s->applied_mode == rings::MODE_EASTER_EGG) {
    s->string_synth.Process(perf, patch, s->in, s->out, s->aux, kBlockSize);
  } else {
    s->part.Process(perf, patch, s->in, s->out, s->aux, kBlockSize);
  }
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  s->active_mode_report = (uint32_t)s->applied_mode;
  s->active_model_report = (uint32_t)s->applied_model;

  // Meters: OUT/AUX block RMS, max since the last 2 s report.
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    out_acc += s->out[i] * s->out[i];
    aux_acc += s->aux[i] * s->aux[i];
  }
  const float out_rms = sqrtf(out_acc / kBlockSize);
  const float aux_rms = sqrtf(aux_acc / kBlockSize);
  if (out_rms > rack_meter_a) rack_meter_a = out_rms;
  if (aux_rms > rack_meter_b) rack_meter_b = aux_rms;

  // The writes pace the render loop: one iteration = one 24-frame DMA block,
  // blocks free every 750 us; > 1400 us (~1.8x, the convention the other
  // ports use) = stalled. Clip16 is OURS (rings clips at the codec write).
  // g_render_gain is the swap-mute ramp (1.0 in normal render).
  uint32_t w0 = time_us_32();
  const float gain = g_render_gain;
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t l = static_cast<int16_t>(
        stmlib::Clip16(static_cast<int32_t>(s->out[i] * 32768.0f * gain)));
    const int16_t r = static_cast<int16_t>(
        stmlib::Clip16(static_cast<int32_t>(s->aux[i] * 32768.0f * gain)));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > kStallUs) {
    ++i2s_stall_count;
  }
}

void note_on(uint8_t note, uint8_t vel) {
  (void)note; (void)vel;            // velocity unused: rings' strum is a bool
  // One-block strum pulse: re-excites the resonator and starts the easter
  // egg's next chord.
  s->pending_strum = true;
}

void note_off(uint8_t note) {
  (void)note;   // gate -> strum_gate handled by the shared note stack
}

void control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 14: s->shared_structure = v; break;
    case 15: s->shared_brightness = v; break;
    case 16: s->shared_damping    = v; break;
    case 17: s->shared_position   = v; break;
    // MODEL/FX: raw byte = ResonatorModel index (modes 0-3) or FxType index
    // (mode 4). Values beyond 5 are meaningless in both enums — ignored.
    case 18:
      if (val <= 5) s->pending_model = static_cast<int8_t>(val);
      break;
    // POLYPHONY: CC 0-127 -> 1 + val/32 (1/2/3/4 ranges of 32); core 1
    // clamps again against kMaxPolyphony.
    case 19: s->pending_polyphony = static_cast<int8_t>(1 + val / 32); break;
    // MODE: raw PerformanceMode index 0-4, clamped to 4.
    case 20: s->pending_mode = static_cast<int8_t>(val > 4 ? 4 : val); break;
    // WAVEFORM EXCITER: 0 square/bow, 1 impulse/blow, 2 saw/strike.
    case 21:
      if (val <= 2) s->pending_waveform = static_cast<int8_t>(val);
      break;
    // CHORD TABLE: 0 Bryan, 1 Jon, 2 Joe.
    case 22:
      if (val <= 2) s->pending_chord_table = static_cast<int8_t>(val);
      break;
    // CHORD index: raw value kept (0..127), clamped at fill time.
    case 23: s->pending_chord = static_cast<int8_t>(val); break;
    default: break;
  }
}

// ===================== T6b: chained-budget clamp (core 0) =====================
// While rings is the chain A slot, the PAIR budget is 750 us (24-sample
// blocks @ 32 kHz) shared as rings + route + clouds-B (~180-195 us chained)
// + mod engine (<= 50 us). The standalone rings sweep (T5a, 19 combos, all
// under the 750 us SINGLE-module budget) measured the sympathetic-string
// models as the worst: model 1 (RESONATOR_MODEL_SYMPATHETIC_STRING) = 511 us
// ALONE -> 511 + 195 + 50 = 756 > 750, over the pair budget. Model 4
// (SYMPATHETIC_STRING_QUANTIZED) runs the same string bank (the sweep's
// string-family range 461-511 tops out there). Both are REJECTED while
// chained; every other combination measured keeps >= 44 us of pair headroom
// (string-family floor 461 + 195 + 50 = 706 <= 750; modal 279, MiniElements
// modes 271-274, easter egg 300-320).
//   - CC 18 (model/fx) is clamped only in the Part modes 0-3; in mode 4 the
//     same CC selects the easter-egg FxType (measured 300-320 us across
//     fx 1-5 — no clamp).
//   - CC 20 (mode) is clamped when it would ENTER a Part mode (0-3) while a
//     sympathetic model byte is already applied (in mode 4 the byte is an
//     FxType index — the same byte drives both enums, so entering mode 0-3
//     with fx 1/4 set would silently become a sympathetic model).
//   - Polyphony (CC 19) is NOT clamped: the sweep's poly-4 rows never
//     exceeded its poly-1 sympathetic worst (511), so no measured
//     combination puts poly > 2 over the pair budget beyond what the model
//     clamp already removes.
// Single-module rings is fully unclamped (this hook is only called while
// g_chain.active with rings as A — see main.cpp rack_module_control_change).
// Benign cross-core read: applied_mode/applied_model are core-1-written
// aligned int32s; the load is atomic and at most one block stale.
bool chain_rejects_cc(uint8_t cc, uint8_t val) {
  // T6d re-measurement (hardware): the sympathetic PAIR costs 692 us of 750
  // (92%) — the T6b blanket model clamp was over-conservative and is REMOVED
  // (sympathetic models are allowed while chained). The one corner still
  // unmeasured is sympathetic + high POLYphony (the T5a sweep only bounded
  // poly-4 by poly-1 ALONE; the pair adds clouds+mod), so polyphony is
  // capped at 2 while a sympathetic model is live and chained — measured-
  // conservative, single-module rings unaffected.
  if (!s) return false;
  const bool part_mode = s->applied_mode != rings::MODE_EASTER_EGG;
  const bool sympathetic =
      part_mode && (s->applied_model == 1 || s->applied_model == 4);
  if (cc == 19 && sympathetic && (1 + val / 32) > 2) {
    Serial.printf("rack: chain: rings poly %u rejected while chained "
                  "(sympathetic pair = 692 us at poly 1; budget 750 us "
                  "incl. clouds+mod)\r\n", (unsigned)(1 + val / 32));
    return true;
  }
  return false;
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Same between-blocks discipline as the pending slots in render(); the mod
// engine only runs while not swapping, so `s` is always the live state.

// Continuous destination write (0..1 virtual CC position, float precision).
// 14-17 are the float patch parameters; 23 is stepped (rounded into the
// pending_chord int8, clamped against the table at fill). 18-22 excluded.
bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  switch (cc) {
    case 14: s->shared_structure = v; break;
    case 15: s->shared_brightness = v; break;
    case 16: s->shared_damping    = v; break;
    case 17: s->shared_position   = v; break;
    case 23:
      s->pending_chord = static_cast<int8_t>(v * 127.0f + 0.5f);
      break;
    default: return false;
  }
  return true;
}

// Trigger destination write: pseudo-CC 64 = STRUM. Rising edges pulse for
// exactly one block (re-excites the resonator / advances the easter egg's
// chord) — the same pending_strum path MIDI note-on uses.
bool mod_trigger_cc(uint8_t cc, bool high) {
  if (cc == 64) {
    if (high) s->pending_strum = true;
    return true;
  }
  return false;
}

}  // namespace mod_rings
}  // namespace rack_modules
