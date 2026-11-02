// Shared MIDI parser + state for the Plaits RP2350 firmware.
// Same packet semantics as clouds-rp2350/src/midi.h (verified against the
// bundled Adafruit TinyUSB Library v3.7.7): 4-byte USB-MIDI event packets from
// BOTH the device port (Adafruit_USBD_MIDI::readPacket) and the PIO-USB host
// port (tuh_midi_packet_read) route through midiHandleEventPacket().
//
// Note handling: last-note priority with an 8-deep note stack (same as the
// Elements/Clouds ports). The top note (+ pitch bend) becomes patch.note —
// in the Plaits contract that field is a NOTE NUMBER IN SEMITONES, the same
// domain the original panel's FREQUENCY pot writes (ui_poll.cc:192
// `patch_->note = 60.0f + transposition_ * 48.0f`; plaits/dsp/engine/engine.h
// NoteToFrequency maps 69 -> 440 Hz). modulations.note, the V/OCT CV add-on
// (voice.cc:704 `p.note = ApplyModulations(patch.note + note, ...)`), stays
// unpatched at 0.0f.
//
// Note-ons also pulse TRIG: modulations.trigger is the calibrated TRIG CV in
// 0..1; Voice delays it 5 blocks (kTriggerDelay, voice.cc:154-156), detects a
// rising edge above 0.3 and latches trigger_velocity_ = pulse value
// (voice.cc:199-202). With patch.trig_response_option = 2 ("velocity trigger")
// that latch scales accent and the LPG ping (voice.cc:589-593, 879-884) — so
// the pulse VOLTAGE is the MIDI velocity. The pulse rides at 0.3 + 0.7 *
// velocity so even velocity 1 crosses the 0.3 trigger threshold; see main.cpp.
//
// CC map (Plaits panel knobs; 0-127 normalized unless noted):
//   14 TIMBRE     15 MORPH      16 HARMONICS
//   17 MACRO      (the fork's fourth synthesis macro/TWIST; routed as the
//                  MODEL CV input: patch.model_cv_option = 1 -> voice.cc
//                  322-326/510-514 macro = 0.5 + modulations.engine; we send
//                  (val-64)/127 so CC 64 = exactly the engine-stock 0.5)
//   18 LPG COLOUR 19 DECAY
//   20 FM AMOUNT  (patch.frequency_modulation_amount = v*2-1: the panel pot
//                  spans [-1, +1], ui.cc:163-164 pot scale 2.0 offset -1.0;
//                  CC 64 = off, CC 0 = -1, CC 127 = +1)
//   21 TIMBRE MOD AMT (patch.timbre_modulation_amount = v*2-1, ui.cc:161-162)
//   22 MORPH MOD AMT  (patch.morph_modulation_amount = v*2-1, ui.cc:165-166)
//   23 ENGINE     (raw value = engine index 0-97 into the Plaits-Lab catalog
//                  registry, NOT normalized; handed to core 1 via a pending
//                  slot — voice.cc:382 reloads + resets the engine between
//                  blocks when patch.engine changes)
//   24 AUX OPTION (raw 0 = regular aux model, 1 = stereo OUT/AUX L/R,
//                  2 = suboscillator; patch.aux_output_option, voice.h:262-266)
//   25 LEVEL      (modulations.level with level_patched = true; voice.cc:574
//                  compresses it to 1.3*L/(0.3+|L|) for accent and LPG level)
//
// Engine and aux-option changes are handed to core 1 via pending_* slots
// (applied strictly between two Render() calls — a mid-Render engine change
// would race the engine reload in voice.cc:382-489). Held-note LEVEL follows
// the gate: main.cpp zeroes it on note-off so the LPG releases with DECAY.
#pragma once
#include <stdint.h>

#include <Arduino.h>             // millis() for the stuck-gate watchdog

// The Plaits-Lab catalog registry size this firmware is built against
// (alt_firmwares/plaits_lab_catalog/catalog.json, 98 engines). main.cpp
// static_asserts this against plaits::kMaxEngines (= PLAITS_ENGINE_COUNT).
static const uint8_t kPlaitsEngineCount = 98;

// --- shared MIDI state (core0 writes, core1 reads; DEFINED in main.cpp) ---
extern volatile float shared_note;       // top note + bend, in semitones
extern volatile bool  shared_gate;       // note stack non-empty
// Plaits panel parameters (core 1 copies them into the Patch between render
// blocks — same shared-state pattern as the Elements/Clouds ports).
extern volatile float shared_timbre;          // CC 14, 0..1 (default 0.5)
extern volatile float shared_morph;           // CC 15, 0..1 (default 0.5)
extern volatile float shared_harmonics;       // CC 16, 0..1 (default 0.5)
extern volatile float shared_macro_cv;        // CC 17, (val-64)/127, 0.0 neutral
extern volatile float shared_lpg_colour;      // CC 18, 0..1 (default 0.5)
extern volatile float shared_decay;           // CC 19, 0..1 (default 0.5)
extern volatile float shared_fm_amount;       // CC 20, v*2-1 (default 0 = off)
extern volatile float shared_timbre_mod;      // CC 21, v*2-1 (default 0 = off)
extern volatile float shared_morph_mod;       // CC 22, v*2-1 (default 0 = off)
extern volatile float shared_level;           // CC 25, 0..1 (default 0.8)

// MIDI delivery counter (DEFINED in main.cpp): incremented per note-on,
// printed in the 2 s report — separates "MIDI reached the firmware" from
// "the DSP made sound".
extern volatile uint32_t note_on_count;

// Pending parameter changes (DEFINED in main.cpp): core 0 writes, core 1
// applies at the top of loop1() (strictly between two Render() calls) and
// resets the slot to -1 / negative.
extern volatile int8_t  pending_engine;       // -1 = none, 0..97 = engine index
extern volatile int8_t  pending_aux_option;   // -1 = none, 0/1/2 = aux option
extern volatile float   pending_trigger;      // <0 = none, else pulse voltage

// LED meters (core 1 writes after each block, core 0 reads in statusLed.Poll):
// OUT (left) and AUX (right) block RMS, max since the last 2 s report.
extern volatile float shared_out_meter;
extern volatile float shared_aux_meter;

// --- note stack: last-note priority (single TU: main.cpp -> static storage) ---
struct NoteStack {
  static const uint8_t kMaxNotes = 8;
  uint8_t notes[kMaxNotes];      // notes[0] = oldest ... notes[size-1] = newest
  uint8_t size = 0;

  void clear() { size = 0; }

  int find(uint8_t note) const {
    for (uint8_t i = 0; i < size; ++i) {
      if (notes[i] == note) return static_cast<int>(i);
    }
    return -1;
  }

  uint8_t top() const { return notes[size - 1]; }

  // Note-on: a re-pressed held note moves to the top (no duplicate); a new
  // note is pushed to the top, dropping the oldest when the stack is full.
  void push(uint8_t note) {
    int i = find(note);
    if (i >= 0) {
      for (int j = i; j < size - 1; ++j) notes[j] = notes[j + 1];
      notes[size - 1] = note;
      return;
    }
    if (size >= kMaxNotes) {
      for (uint8_t j = 0; j < size - 1; ++j) notes[j] = notes[j + 1];
      --size;
    }
    notes[size++] = note;
  }

  // Note-off: remove the note if it is held (no-op otherwise).
  void remove(uint8_t note) {
    int i = find(note);
    if (i < 0) return;
    for (int j = i; j < size - 1; ++j) notes[j] = notes[j + 1];
    --size;
  }
};

static NoteStack note_stack;
static int16_t bend_offset = 0;          // -8192..8191, applied to the top note
static uint32_t last_note_event_ms = 0;  // stuck-gate watchdog input (core 0)

// shared_note = newest held note (or 48.0 when idle) + current bend offset.
static void applyNote() {
  float base = note_stack.size ? static_cast<float>(note_stack.top()) : 48.0f;
  shared_note = base + static_cast<float>(bend_offset) * (2.0f / 8192.0f);
}

inline void midiNoteOn(uint8_t note, uint8_t vel) {
  note_stack.push(note);          // retrigger: moves to top, no duplicate
  applyNote();
  shared_gate = true;
  // TRIG pulse voltage: velocity scaled to always cross Voice's 0.3 trigger
  // threshold (voice.cc:156) — 0.3 + 0.7 * v — and latched as the accent
  // velocity on the (delayed) rising edge. Consumed by core 1 on the next
  // block; the pulse lasts exactly one render block.
  pending_trigger = 0.3f + 0.7f * (float)vel / 127.0f;
  ++note_on_count;
  last_note_event_ms = millis();
}

inline void midiNoteOff(uint8_t note) {
  note_stack.remove(note);
  if (note_stack.size == 0) {
    shared_gate = false;          // gate only drops when the stack empties
  } else {
    applyNote();                  // released note was on top: fall back to the
  }                               // previous held note
  last_note_event_ms = millis();
}

inline void midiPitchBend(int16_t bend14) {  // -8192..8191, 0 = centered
  bend_offset = bend14;
  applyNote();                    // bend always applies to the current top note
}

inline void midiControlChange(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 14: shared_timbre     = v; break;
    case 15: shared_morph      = v; break;
    case 16: shared_harmonics  = v; break;
    // MACRO/TWIST as the MODEL CV (patch.model_cv_option = 1): macro = 0.5 +
    // modulations.engine (voice.cc:322-326, 510-514). (val-64)/127 puts the
    // exact engine-stock neutral (0.5) at CC 64.
    case 17: shared_macro_cv = ((float)val - 64.0f) / 127.0f; break;
    case 18: shared_lpg_colour = v; break;
    case 19: shared_decay      = v; break;
    // Modulation amounts span [-1, +1] like the panel pots (ui.cc:161-166:
    // scale 2.0, offset -1.0); CC 64 = off.
    case 20: shared_fm_amount  = v * 2.0f - 1.0f; break;
    case 21: shared_timbre_mod = v * 2.0f - 1.0f; break;
    case 22: shared_morph_mod  = v * 2.0f - 1.0f; break;
    // ENGINE: raw value = registry index (0-97). Values beyond the catalog
    // are ignored; handed to core 1 via the pending slot (applied between
    // render blocks — voice reloads/resets the engine there).
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

// USB-MIDI event packet (4 bytes) -> channel voice messages.
// CIN 0x8..0xE map 1:1 onto the MIDI status nibble (single cable).
inline void midiHandleEventPacket(const uint8_t pkt[4]) {
  const uint8_t cin = pkt[0] & 0x0F;
  if (cin < 0x8 || cin > 0xE) return;      // ignore system / SYSEX / malformed
  const uint8_t d1 = pkt[2] & 0x7F;        // note number / cc# / bend LSB
  const uint8_t d2 = pkt[3] & 0x7F;        // velocity / value / bend MSB
  switch (cin) {
    case 0x8: midiNoteOff(d1); break;
    case 0x9: if (d2) { midiNoteOn(d1, d2); } else { midiNoteOff(d1); } break;
    case 0xB: midiControlChange(d1, d2); break;
    case 0xE: midiPitchBend((int16_t)((d2 << 7) | d1) - 8192); break;
    default: break;                        // 0xC pgm / 0xD pressure: unused
  }
}
