// Shared MIDI parser + state for the Elements RP2350 firmware.
// Used by core 0 (main.cpp): 4-byte USB-MIDI event packets from BOTH the
// device port (Adafruit_USBD_MIDI::readPacket) and the PIO-USB host port
// (tuh_midi_packet_read) route through midiHandleEventPacket().
//
// Packet semantics VERIFIED against the bundled Adafruit TinyUSB Library
// v3.7.7 (framework-arduinopico 1.60100.0):
//   Adafruit_USBD_MIDI::readPacket(uint8_t packet[4]) fills the FULL 4-byte
//   USB-MIDI event packet via tud_midi_packet_read():
//     pkt[0] = (cable_number << 4) | CIN   (single cable => cable nibble 0)
//     pkt[1] = MIDI status byte (e.g. 0x9n)
//     pkt[2] = data1 (note/cc#/bend LSB)
//     pkt[3] = data2 (velocity/cc value/bend MSB; 0 for 2-byte messages)
//   CIN 0x8..0xE are the channel voice messages; CIN 0xF is system/SYSEX.
//   (The original plan indexed the packet as [CIN, d1, d2] over 3 bytes —
//   corrected here: the status byte occupies pkt[1] and data starts at pkt[2].)
//
// Note handling (T6): last-note priority with an 8-deep note stack.
// shared_note always tracks the NEWEST held note; the gate only drops when
// the stack empties. A re-pressed held note moves to the top instead of
// duplicating; pitch bend is remembered and applied on top of the current
// top note.
//
// Resonator model (T6): CC 28 stores a pending model (0 modal / 1 string /
// 2 strings) in pending_resonator_model; core 1's loop1() applies it between
// render blocks, so the change can never land inside a running Process().
#pragma once
#include <stdint.h>

#include <Arduino.h>             // millis() for the stuck-gate watchdog

#include <ELEMENTS.h>

// --- shared MIDI state (core0 writes, core1 reads; DEFINED in main.cpp) ---
extern volatile float shared_note;
extern volatile float shared_modulation;
extern volatile float shared_strength;
extern volatile bool  shared_gate;
extern elements::Part part;

// Pending resonator-model change (DEFINED in main.cpp): core 0 writes 0..2
// from CC 28; core 1 applies part.set_resonator_model() at the top of loop1()
// (strictly between two Process() calls) and resets the slot to -1. At most
// one change is applied per block.
extern volatile int8_t pending_resonator_model;   // -1 = none, 0..2 = model

// Pending easter-egg (ominous voice) change (DEFINED in main.cpp): core 0
// writes 0/1 from CC 29 (0 = normal, >=1 = ominous); core 1 applies
// part.set_easter_egg() at the top of loop1() (strictly between two
// Process() calls) and resets the slot to -1. Runtime switching is safe:
// easter_egg_ is only branched on per block inside Part::Process() and the
// OminousVoice instances are Init()ed alongside the normal ones.
extern volatile int8_t pending_easter_egg;        // -1 = none, 0/1 = mode

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
  shared_strength = 0.4f + 0.6f * (static_cast<float>(vel) / 127.0f);
  shared_gate = true;
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
    // Resonator model selector (T6): 0 modal / 1 string / 2 strings. Values
    // > 2 are ignored; re-sending a value re-selects that model. Handed to
    // core 1 via pending_resonator_model (applied between render blocks).
    case 28:
      if (val <= 2) {
        pending_resonator_model = static_cast<int8_t>(val);
      }
      break;
    // Ominous-voice (easter egg) selector: 0 = normal, 1..127 = ominous.
    // Absolute values, not a toggle — the panel cannot read the synth state,
    // so a toggle would desync knob vs engine. Handed to core 1 via
    // pending_easter_egg (applied between render blocks). While ominous, the
    // patch fields are reinterpreted as FM op amounts and CC 28 has no effect.
    case 29:
      pending_easter_egg = val ? 1 : 0;
      break;
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
