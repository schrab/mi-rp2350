// Rack RP2350 — RINGS adapter (boot-select via RACK_BOOT_MODULE == 3).
//
// Lifted VERBATIM from rings-rp2350/src/main.cpp + midi.h: fresh
// Patch/PerformanceState locals every block (rings.cc contract), one caller-
// owned reverb buffer shared by both Parts (mutually exclusive by mode),
// internal_exciter = true (self-exciting synth, no input jack), strum pulses
// from MIDI note-ons, the clamped chord index (Joe table's per-polyphony
// quirk included) and the ODD/EVEN out/aux convention with our Clip16.
// Only the SELECTED adapter is compiled. NOTE: the fork has NO Dsp::setSr —
// kSampleRate 32000 is a compile-time constant baked into the lib (the
// standalone main.cpp defines no Dsp statics; none needed here).
#pragma once

// Module-library includes UNCONDITIONAL (see module_elements.h): all four
// module libs are linked in every rack env; only the state + hook bodies
// below are compile-time guarded.
#include <STMLIB.h>
#include <RINGS.h>               // rings::Part + rings::StringSynthPart

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (rings has no internal clipper)
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#if RACK_BOOT_MODULE == 3  // ---- active only in [env:rack-rings] ----

static const char* const kModuleName = "rings";
static const uint32_t kModuleSampleRate = 32000;               // = rings::kSampleRate
static const size_t kModuleBlockSize = rings::kMaxBlockSize;   // 24 samples
static const unsigned kModuleBudgetUs =
    (unsigned)(1000000UL * kModuleBlockSize / kModuleSampleRate);  // 750
static const uint32_t kModuleI2sBuffers = 32;   // 32 x 16 words = 512 frames
static const uint32_t kModuleI2sWords = 16;     // = 16 ms bank at 32 kHz
static const uint32_t kModuleStallUs = 1400;    // ~1.8x the 750 us block period

// --- engine state (identical sizes/objects to the standalone main.cpp) ---
// One caller-owned reverb/chorus/ensemble workspace shared by both Parts
// (rings.cc:43 uses the same single buffer for part.Init +
// string_synth.Init). Safe because the firmware contract renders exactly ONE
// part per block (mode-selected).
static uint16_t reverb_buffer[32768];   // 64 KB

static rings::Part part;
static rings::StringSynthPart string_synth;

static float in[kModuleBlockSize];    // no input jack: fed with silence
static float out[kModuleBlockSize];
static float aux[kModuleBlockSize];

// Panel parameters (core 0 writes, core 1 copies into the fresh per-block
// Patch — same shared-state pattern as the standalone port).
static volatile float shared_structure = 0.5f;   // CC 14 (clamped <= 0.9995)
static volatile float shared_brightness = 0.5f;  // CC 15
static volatile float shared_damping = 0.5f;     // CC 16
static volatile float shared_position = 0.5f;    // CC 17

static volatile int8_t  pending_model = -1;         // -1 none, 0..5 model/FxType
static volatile int8_t  pending_polyphony = -1;     // -1 none, 1..4
static volatile int8_t  pending_mode = -1;          // -1 none, 0..4 mode
static volatile int8_t  pending_waveform = -1;      // -1 none, 0..2 waveform
static volatile int8_t  pending_chord_table = -1;   // -1 none, 0..2 ChordTable
static volatile int8_t  pending_chord = -1;         // -1 none, 0..127 raw
static volatile bool    pending_strum = false;      // one-block strum pulse

// Applied-state mirrors for the 2 s report (model is the APPLIED CC 18 byte,
// clamped 0-5: ResonatorModel index in modes 0-3, FxType index in mode 4).
static volatile uint32_t active_mode_report = 0;
static volatile uint32_t active_model_report = 0;

// --- applied settings (core 1 only; applied between blocks) ---
static int32_t applied_mode = rings::MODE_RINGS_STEREO;        // 0
static int32_t applied_model = 0;                       // RESONATOR_MODEL_MODAL
static int32_t applied_polyphony = 1;
static int32_t applied_waveform = 0;                    // square / bow
static int32_t applied_chord_table = rings::CHORD_TABLE_BRYAN;  // 0
static int32_t applied_chord = 0;                       // raw, clamped at fill

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

void module_init() {
  // Both Parts over the one shared reverb buffer. Defaults match Part::Init
  // (MODAL, polyphony 1) and StringSynthPart::Init; the model byte 0 also
  // becomes FX_FORMANT so CC 18 means the same index in every mode.
  memset(reverb_buffer, 0, sizeof(reverb_buffer));
  part.Init(reverb_buffer);
  string_synth.Init(reverb_buffer);
  part.set_polyphony(1);
  part.set_model(rings::RESONATOR_MODEL_MODAL);
  string_synth.set_polyphony(1);
  string_synth.set_fx(rings::FX_FORMANT);
}

void module_render() {
  // --- apply pending changes strictly between two Process() calls ---
  if (pending_mode >= 0) {
    applied_mode = pending_mode <= rings::MODE_EASTER_EGG
        ? pending_mode : rings::MODE_EASTER_EGG;
    pending_mode = -1;
  }
  if (pending_model >= 0) {
    applied_model = pending_model <= 5 ? pending_model : 5;
    // The SAME byte drives the resonator model (modes 0-3) and the easter
    // egg's FxType (mode 4); apply to both so a mode flip lands on the
    // already-selected model/FX.
    part.set_model(static_cast<rings::ResonatorModel>(applied_model));
    string_synth.set_fx(static_cast<rings::FxType>(applied_model));
    pending_model = -1;
  }
  if (pending_polyphony >= 0) {
    int32_t p = pending_polyphony;
    if (p < 1) p = 1;
    if (p > rings::kMaxPolyphony) p = rings::kMaxPolyphony;
    applied_polyphony = p;
    part.set_polyphony(p);            // Part clamps too (part.h:89-96)
    string_synth.set_polyphony(p);    // (string_synth_part.h:87-96)
    pending_polyphony = -1;
  }
  if (pending_waveform >= 0) {
    applied_waveform = pending_waveform <= 2 ? pending_waveform : 2;
    pending_waveform = -1;
  }
  if (pending_chord_table >= 0) {
    applied_chord_table = pending_chord_table <= rings::CHORD_TABLE_JOE
        ? pending_chord_table : rings::CHORD_TABLE_JOE;
    pending_chord_table = -1;
  }
  if (pending_chord >= 0) {
    applied_chord = pending_chord;    // raw 0..127, clamped at fill time
    pending_chord = -1;
  }

  // Strum pulse: exactly one block high per note-on.
  const bool strum_pulse = pending_strum;
  pending_strum = false;

  // --- fresh Patch + PerformanceState locals EVERY block (rings.cc:92-96) ---
  rings::PerformanceState perf;
  rings::Patch patch;

  perf.internal_exciter = true;
  perf.internal_strum = false;
  perf.internal_note = false;
  perf.mode = static_cast<rings::PerformanceMode>(applied_mode);
  perf.chord_table = static_cast<rings::ChordTable>(applied_chord_table);
  perf.waveform_exciter = static_cast<uint8_t>(applied_waveform);
  perf.strum_hold_option = 0;
  perf.fm_input_option = 0;
  perf.frequency_locked = 0;
  perf.locked_frequency_pot_value = 0.0f;
  perf.tonic = 0.0f;
  perf.fm = 0.0f;
  perf.note = shared_note;
  perf.strum = strum_pulse;
  perf.strum_gate = shared_gate;
  perf.chord = applied_chord;
  {
    const int32_t num_chords =
        NumChordsFor(applied_chord_table, applied_mode, applied_polyphony);
    if (perf.chord > num_chords - 1) perf.chord = num_chords - 1;
    if (perf.chord < 0) perf.chord = 0;
  }

  patch.structure = shared_structure;
  if (patch.structure > 0.9995f) patch.structure = 0.9995f;  // cv_scaler.cc:187
  if (patch.structure < 0.0f) patch.structure = 0.0f;
  patch.brightness = shared_brightness;
  patch.damping = shared_damping;
  patch.position = shared_position;

  // No input jack: silence in. 0.0f is all-zero bytes, so memset is an exact
  // float-zero fill.
  memset(in, 0, sizeof(in));

  uint32_t t0 = time_us_32();
  // ONE part renders per block (rings.cc:98-116): modes 0-3 -> Part, mode 4
  // (easter egg) -> StringSynthPart.
  if (applied_mode == rings::MODE_EASTER_EGG) {
    string_synth.Process(perf, patch, in, out, aux, kModuleBlockSize);
  } else {
    part.Process(perf, patch, in, out, aux, kModuleBlockSize);
  }
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  active_mode_report = (uint32_t)applied_mode;
  active_model_report = (uint32_t)applied_model;

  // Meters: OUT/AUX block RMS, max since the last 2 s report.
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    out_acc += out[i] * out[i];
    aux_acc += aux[i] * aux[i];
  }
  const float out_rms = sqrtf(out_acc / kModuleBlockSize);
  const float aux_rms = sqrtf(aux_acc / kModuleBlockSize);
  if (out_rms > rack_meter_a) rack_meter_a = out_rms;
  if (aux_rms > rack_meter_b) rack_meter_b = aux_rms;

  // The writes pace the render loop: one iteration = one 24-frame DMA block,
  // blocks free every 750 us; > 1400 us (~1.8x, the convention the other
  // ports use) = stalled. Clip16 is OURS (rings clips at the codec write).
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    const int16_t l = static_cast<int16_t>(
        stmlib::Clip16(static_cast<int32_t>(out[i] * 32768.0f)));
    const int16_t r = static_cast<int16_t>(
        stmlib::Clip16(static_cast<int32_t>(aux[i] * 32768.0f)));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > kModuleStallUs) {
    ++i2s_stall_count;
  }
}

void module_note_on(uint8_t note, uint8_t vel) {
  (void)note; (void)vel;            // velocity unused: rings' strum is a bool
  // One-block strum pulse: re-excites the resonator and starts the easter
  // egg's next chord.
  pending_strum = true;
}

void module_note_off(uint8_t note) {
  (void)note;   // gate -> strum_gate handled by the shared note stack
}

void module_control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 14: shared_structure = v; break;
    case 15: shared_brightness = v; break;
    case 16: shared_damping    = v; break;
    case 17: shared_position   = v; break;
    // MODEL/FX: raw byte = ResonatorModel index (modes 0-3) or FxType index
    // (mode 4). Values beyond 5 are meaningless in both enums — ignored.
    case 18:
      if (val <= 5) pending_model = static_cast<int8_t>(val);
      break;
    // POLYPHONY: CC 0-127 -> 1 + val/32 (1/2/3/4 ranges of 32); core 1
    // clamps again against kMaxPolyphony.
    case 19: pending_polyphony = static_cast<int8_t>(1 + val / 32); break;
    // MODE: raw PerformanceMode index 0-4, clamped to 4.
    case 20: pending_mode = static_cast<int8_t>(val > 4 ? 4 : val); break;
    // WAVEFORM EXCITER: 0 square/bow, 1 impulse/blow, 2 saw/strike.
    case 21:
      if (val <= 2) pending_waveform = static_cast<int8_t>(val);
      break;
    // CHORD TABLE: 0 Bryan, 1 Jon, 2 Joe.
    case 22:
      if (val <= 2) pending_chord_table = static_cast<int8_t>(val);
      break;
    // CHORD index: raw value kept (0..127), clamped at fill time.
    case 23: pending_chord = static_cast<int8_t>(val); break;
    default: break;
  }
}

#endif  // RACK_BOOT_MODULE == 3
