// Shared MIDI parser + state for the Clouds RP2350 firmware.
// Same packet semantics as elements-rp2350/src/midi.h (verified against the
// bundled Adafruit TinyUSB Library v3.7.7): 4-byte USB-MIDI event packets from
// BOTH the device port (Adafruit_USBD_MIDI::readPacket) and the PIO-USB host
// port (tuh_midi_packet_read) route through midiHandleEventPacket().
//
// Note handling: last-note priority with an 8-deep note stack (same as the
// Elements port). The note pitch drives the test exciter and the granular
// pitch parameter; note-ons also pulse the Clouds TRIGGER input.
//
// CC map (Clouds panel knobs, one CC per original knob/sidechain):
//   14 POSITION   15 SIZE     16 PITCH    17 DENSITY   18 TEXTURE
//   19 DRY/WET    20 STEREO SPREAD  21 FEEDBACK  22 REVERB
//   23 PLAYBACK MODE (0 granular / 1 stretch / 2 looping delay / 3 spectral)
//   24 QUALITY (0 = 16-bit stereo ... 3 = 8-bit mono low-fidelity)
//   13 EXCITER SOURCE (0 = burst pluck, >=0.5 = gated tone)
//    1 EXCITER TONE   11 EXCITER LEVEL (0 mutes the test input entirely —
//      used when Clouds is chained after another module's output later)
//   65 FREEZE (>=64 held on, <64 released — absolute, not a toggle)
//   66 TRIGGER (>=64 = pulse re-triggers grain grabbing / looper grab)
//
// Mode/quality/freeze changes are handed to core 1 via pending_* slots
// (applied strictly between two Process() calls, same discipline as the
// Elements port's pending_resonator_model). Freeze is a held bool, so it is
// also applied between blocks via its slot.
#pragma once
#include <stdint.h>

#include <Arduino.h>             // millis() for the stuck-gate watchdog

// --- shared MIDI state (core0 writes, core1 reads; DEFINED in main.cpp) ---
extern volatile float shared_note;
extern volatile float shared_exciter_tone;
extern volatile float shared_exciter_level;
extern volatile float shared_exciter_mode;
extern volatile bool  shared_gate;
// Clouds panel parameters (core 1 copies them into mutable_parameters()
// between render blocks — same shared-state pattern as the Elements port).
extern volatile float shared_position;
extern volatile float shared_size;
extern volatile float shared_pitch;
extern volatile float shared_density;
extern volatile float shared_texture;
extern volatile float shared_dry_wet;
extern volatile float shared_stereo_spread;
extern volatile float shared_feedback;
extern volatile float shared_reverb;

// MIDI delivery counter (DEFINED in main.cpp): incremented per note-on,
// printed in the 2 s report — separates "MIDI reached the firmware" from
// "the DSP made sound".
extern volatile uint32_t note_on_count;

// Pending parameter changes (DEFINED in main.cpp): core 0 writes, core 1
// applies at the top of loop1() (strictly between two Process() calls) and
// resets the slot to -1 / false.
extern volatile int8_t  pending_playback_mode;   // -1 = none, 0..3 = mode
extern volatile int8_t  pending_quality;         // -1 = none, 0..3 = quality
extern volatile int8_t  pending_freeze;          // -1 = none, 0/1 = freeze
extern volatile bool    pending_trigger;         // one-shot pulse

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
  pending_trigger = true;         // Clouds TRIGGER input pulses on every note-on
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
    case 1:  shared_exciter_tone = v; break;
    case 11: shared_exciter_level = v; break;
    case 13: shared_exciter_mode = v; break;
    case 14: shared_position     = v; break;
    case 15: shared_size         = v; break;
    case 16: shared_pitch        = v; break;
    case 17: shared_density      = v; break;
    case 18: shared_texture      = v; break;
    case 19: shared_dry_wet      = v; break;
    case 20: shared_stereo_spread = v; break;
    case 21: shared_feedback     = v; break;
    case 22: shared_reverb       = v; break;
    // Playback mode selector: 0 granular / 1 stretch / 2 looping delay /
    // 3 spectral. Values > 3 are ignored; handed to core 1 via the pending
    // slot (applied between render blocks — a mid-Process change would race).
    case 23:
      if (val <= 3) pending_playback_mode = static_cast<int8_t>(val);
      break;
    // Quality selector: 0 = 16-bit stereo, 1 = 16-bit mono, 2 = 8-bit stereo,
    // 3 = 8-bit mono (low fidelity). Handed to core 1 via the pending slot;
    // changing quality resets the recording buffers inside Prepare().
    case 24:
      if (val <= 3) pending_quality = static_cast<int8_t>(val);
      break;
    // FREEZE: absolute held value (the panel knows the switch state), applied
    // between render blocks via the pending slot.
    case 65: pending_freeze = val >= 64 ? 1 : 0; break;
    // TRIGGER: one-shot pulse consumed by core 1 on the next block.
    case 66: if (val >= 64) pending_trigger = true; break;
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
