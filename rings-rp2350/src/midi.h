// Shared MIDI parser + state for the Rings RP2350 firmware.
// Same packet semantics as clouds-rp2350/src/midi.h (verified against the
// bundled Adafruit TinyUSB Library v3.7.7): 4-byte USB-MIDI event packets from
// BOTH the device port (Adafruit_USBD_MIDI::readPacket) and the PIO-USB host
// port (tuh_midi_packet_read) route through midiHandleEventPacket().
//
// Note handling: last-note priority with an 8-deep note stack (verbatim from
// the Elements/Clouds/Plaits ports). The top note (+ pitch bend) becomes
// performance_state.note — in the rings contract that field is a NOTE NUMBER
// IN SEMITONES summed with performance_state.tonic and .fm (part.cc:701
// `note = note_[voice] + performance_state.tonic + fm`, then
// SemitonesToRatio(note - 69) * a3, so 69 = 440 Hz). We send tonic = 0 and
// fm = 0 from main.cpp, so the MIDI note is the exact pitch.
//
// Note-ons also fire rings' strum: pending_strum is a one-render-block pulse
// consumed by core 1 (the original firmware's Strummer generates the same
// pulse from the TRIG jack — we drive it directly; internal_strum = false).
// The strum pulse re-excites (EXCITER_FLAG_RISING_EDGE, part.cc:825-827;
// part.cc:665-674 voice rotation; easter egg chord restart,
// string_synth_part.cc:435-441) and shared_gate rides strum_gate
// (EXCITER_FLAG_GATE, part.cc:828-830) so bow/blow sustain while held.
//
// VELOCITY IS IGNORED: rings' strum is a bool and PerformanceState has no
// velocity/accent field (dsp/performance_state.h) — there is nothing to map
// it to. (The MiniElements exciters' timbre/tone comes from the patch floats
// instead.)
//
// CC map (rings panel knobs; 0-127 normalized unless noted):
//   14 STRUCTURE  (patch.structure, 0..1, clamped <= 0.9995 at fill —
//                  cv_scaler.cc:187; also the chord CV on hardware, but our
//                  chord comes from CC 23)
//   15 BRIGHTNESS (patch.brightness, 0..1)
//   16 DAMPING    (patch.damping, 0..1; also the easter egg's envelope
//                  decay/drone shape, string_synth_part.cc:345-373)
//   17 POSITION   (patch.position, 0..1; in the MiniElements modes with
//                  waveform_exciter 1/2 the Part repurposes it as the
//                  blow/strike exciter parameter — PositionRepurposed(),
//                  part.h:133-137)
//   18 MODEL/FX   (raw 0..5: ResonatorModel modal/sympathetic/string/
//                  fm-voice/sympathetic-quantized/string+reverb in modes
//                  0-3, FxType formant/chorus/reverb/formant2/ensemble/
//                  reverb2 in mode 4; values > 5 ignored; pending slot —
//                  set_model/set_fx applied between blocks)
//   19 POLYPHONY  (mapped 1 + val/32: CC 0-31 -> 1, 32-63 -> 2, 64-95 -> 3,
//                  96-127 -> 4; pending slot)
//   20 MODE       (raw 0..4 PerformanceMode, clamped to 4; pending slot)
//   21 WAVEFORM EXCITER (raw 0..2, values > 2 ignored; per-mode meaning:
//                  mode 1: square/impulse/saw monitor oscillator;
//                  modes 2-3: bow/blow/strike exciter; modes 0/4: unused)
//   22 CHORD TABLE (raw 0..2 ChordTable Bryan/Jon/Joe, values > 2 ignored;
//                  pending slot)
//   23 CHORD      (raw 0..127 chord index; clamped at fill time to the
//                  current table's NumChords — Bryan 11, Jon 17, Joe 18 in
//                  the easter egg / 15-21-16-23 by polyphony otherwise,
//                  cv_scaler.cc:289-309)
//
// Engine/option changes are handed to core 1 via pending_* slots (applied
// strictly between two Process() calls — Part::set_model marks the
// resonators dirty and re-Inits them at the top of the next Process, which
// must not happen mid-block).
#pragma once
#include <stdint.h>

#include <Arduino.h>             // millis() for the stuck-gate watchdog

// --- shared MIDI state (core0 writes, core1 reads; DEFINED in main.cpp) ---
extern volatile float shared_note;       // top note + bend, in semitones
extern volatile bool  shared_gate;       // note stack non-empty
// Rings panel parameters (core 1 copies them into the fresh per-block Patch —
// same shared-state pattern as the Elements/Clouds/Plaits ports).
extern volatile float shared_structure;   // CC 14, 0..1 (default 0.5)
extern volatile float shared_brightness;  // CC 15, 0..1 (default 0.5)
extern volatile float shared_damping;     // CC 16, 0..1 (default 0.5)
extern volatile float shared_position;    // CC 17, 0..1 (default 0.5)

// MIDI delivery counter (DEFINED in main.cpp): incremented per note-on,
// printed in the 2 s report — separates "MIDI reached the firmware" from
// "the DSP made sound".
extern volatile uint32_t note_on_count;

// Pending parameter changes (DEFINED in main.cpp): core 0 writes, core 1
// applies at the top of loop1() (strictly between two Process() calls) and
// resets the slot to -1 / false.
extern volatile int8_t  pending_model;        // -1 none, 0..5 model / FxType
extern volatile int8_t  pending_polyphony;    // -1 none, 1..4
extern volatile int8_t  pending_mode;         // -1 none, 0..4 PerformanceMode
extern volatile int8_t  pending_waveform;     // -1 none, 0..2 waveform_exciter
extern volatile int8_t  pending_chord_table;  // -1 none, 0..2 ChordTable
extern volatile int8_t  pending_chord;        // -1 none, 0..127 raw index
extern volatile bool    pending_strum;        // one-block pulse per note-on

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
// Bend: standard 14-bit wheel centered at 8192, mapped to a +/-2 semitone
// offset (bend_offset * 2/8192) — the same convention as the Plaits port.
static void applyNote() {
  float base = note_stack.size ? static_cast<float>(note_stack.top()) : 48.0f;
  shared_note = base + static_cast<float>(bend_offset) * (2.0f / 8192.0f);
}

inline void midiNoteOn(uint8_t note, uint8_t vel) {
  (void)vel;                      // velocity unused: rings' strum is a bool
  note_stack.push(note);          // retrigger: moves to top, no duplicate
  applyNote();
  shared_gate = true;
  // One-block strum pulse (core 1 consumes it at the top of the next render
  // block): re-excites the resonator and starts the easter egg's next chord.
  pending_strum = true;
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
    case 14: shared_structure = v; break;
    case 15: shared_brightness = v; break;
    case 16: shared_damping    = v; break;
    case 17: shared_position   = v; break;
    // MODEL/FX: raw byte = ResonatorModel index (modes 0-3) or FxType index
    // (mode 4). Values beyond 5 are meaningless in both enums — ignored.
    case 18:
      if (val <= 5) pending_model = static_cast<int8_t>(val);
      break;
    // POLYPHONY: CC 0-127 -> 1 + val/32 (1/2/3/4 ranges of 32), documented
    // in the map above; core 1 clamps again against kMaxPolyphony.
    case 19: pending_polyphony = static_cast<int8_t>(1 + val / 32); break;
    // MODE: raw PerformanceMode index 0-4, clamped to 4 (out-of-range CCs
    // land on the easter egg rather than being dropped — this is a mode
    // selector, a stray value should not be silently ignored).
    case 20: pending_mode = static_cast<int8_t>(val > 4 ? 4 : val); break;
    // WAVEFORM EXCITER: 0 square/bow, 1 impulse/blow, 2 saw/strike.
    case 21:
      if (val <= 2) pending_waveform = static_cast<int8_t>(val);
      break;
    // CHORD TABLE: 0 Bryan, 1 Jon, 2 Joe.
    case 22:
      if (val <= 2) pending_chord_table = static_cast<int8_t>(val);
      break;
    // CHORD index: raw value kept (0..127), clamped at fill time to the
    // CURRENT table's NumChords (cv_scaler.cc:289-309) — the Joe table's
    // count depends on mode + polyphony, which core 1 owns.
    case 23: pending_chord = static_cast<int8_t>(val); break;
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
