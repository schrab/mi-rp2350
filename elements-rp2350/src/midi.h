// Task 4 — shared MIDI parser + state for the Elements RP2350 firmware.
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
#pragma once
#include <stdint.h>

#include <ELEMENTS.h>

// --- shared MIDI state (core0 writes, core1 reads; DEFINED in main.cpp) ---
extern volatile float shared_note;
extern volatile float shared_modulation;
extern volatile float shared_strength;
extern volatile bool  shared_gate;
extern elements::Part part;

// Last note-on pitch; per-TU internal linkage (single TU today: main.cpp).
static int16_t base_note = 48;

inline void midiNoteOn(uint8_t note, uint8_t vel) {
  base_note = note;
  shared_note = (float)note;
  shared_strength = 0.4f + 0.6f * ((float)vel / 127.0f);
  shared_gate = true;
}

inline void midiNoteOff(uint8_t note) {
  if ((int16_t)note == base_note) shared_gate = false;
}

inline void midiPitchBend(int16_t bend14) {  // -8192..8191, 0 = centered
  shared_note = (float)base_note + (float)bend14 * (2.0f / 8192.0f);
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
    // NOTE: maps resonator_modulation_frequency for now; CC 28 becomes the
    // resonator MODEL selector in T6 (set_resonator_model() reallocates
    // voice state — needs a core-1-side pending-model gate first).
    case 28: p->resonator_modulation_frequency = v; break;
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
