// Shared MIDI parser + state for the Rack RP2350 firmware — module-agnostic.
// The per-module CC maps / note handling live in the module adapters
// (src/module_*.h — ALL FOUR compile since T2); dispatch goes through the
// module registry (module_registry.h, defined in main.cpp).
//
// Same 4-byte USB-MIDI event packet contract as the four standalone ports
// (elements/clouds/plaits/rings-rp2350/src/midi.h), verified against the
// bundled Adafruit TinyUSB Library v3.7.7:
//   pkt[0] = (cable_number << 4) | CIN
//   pkt[1] = MIDI status byte (e.g. 0x9n)
//   pkt[2] = data1 (note/cc#/bend LSB)
//   pkt[3] = data2 (velocity/cc value/bend MSB; 0 for 2-byte messages)
// Device (readPacket) and PIO-USB host (tuh_midi_packet_read) packets take
// the same midiHandleEventPacket() entry point; T4 adds DIN/BLE to the same
// funnel.
//
// Note handling: last-note priority with an 8-deep note stack (proven
// pattern from the standalone ports). shared_note always tracks the NEWEST
// held note; the gate only drops when the stack empties. Pitch bend is
// remembered and applied on top of the current top note (+/- 2 semitones).
// The note stack + bend are PLATFORM state: they survive module swaps (the
// swapped-in module sees the currently held note; its CC values re-Init to
// factory defaults — documented rack behavior, web panels re-sync on
// reconnect).
#pragma once
#include <stdint.h>

#include <Arduino.h>             // millis() for the stuck-gate watchdog
#include <I2S.h>                 // extern I2S i2s below

// --- shared MIDI state (core0 writes, core1 reads; DEFINED in main.cpp) ---
extern volatile float shared_note;         // top note + bend, semitones
extern volatile bool  shared_gate;         // note stack non-empty
// Firmware-side note-on counter — printed in the 2 s report; separates
// "MIDI reached the firmware" from "the DSP made sound".
extern volatile uint32_t note_on_count;

// Per-module meters for the status LED + the 2 s report (core 1 writes after
// each block, core 0 reads). Max-since-report, x1000 in the print.
// elements: exciter level / resonator level (the original smoothed meters);
// clouds:   input RMS / output RMS;
// plaits:   OUT RMS / AUX RMS;
// rings:    OUT RMS / AUX RMS.
extern volatile float rack_meter_a;
extern volatile float rack_meter_b;

// --- perf telemetry (core1 writes, core0 prints; DEFINED in main.cpp) ---
extern volatile uint32_t proc_us_max;      // worst module Process since report
extern volatile uint32_t i2s_stall_count;  // slow I2S write-block loops

// Swap-mute ramp multiplier (core 1 writes, core 1 reads in the adapters'
// write loops): 1.0 in normal render; the live-swap sequence ramps it down
// to 0 before muting (click-free) and back up after the swap. DEFINED in
// main.cpp.
extern volatile float g_render_gain;

extern I2S i2s;                            // defined in main.cpp (OUTPUT)

// --- module registry dispatch (DEFINED in main.cpp; the registry routes to
// the ACTIVE adapter's hooks — all four adapters compile in every env) ---
void rack_module_note_on(uint8_t note, uint8_t vel);
void rack_module_note_off(uint8_t note);
void rack_module_control_change(uint8_t cc, uint8_t val);

// Dedicated switch CC: BANK SELECT (CC 32), value 0-3 = module id. Verified
// UNUSED by all four adapters' CC maps (elements 1,11,14-29; clouds
// 1,11,13-24,65,66; plaits 14-25; rings 14-23) and by this parser. Values
// > 3 and == active module are ignored. Core 0 writes the pending slot;
// core 1 consumes it at the top of loop1 (max one swap per call).
static const uint8_t kRackSwitchCc = 32;
void rack_request_switch(uint8_t module_id);

// --- T3 mod-engine bench CCs (multiplexed select/param/value, core 0) ---
// CC 33 = source select (0-11: 0-3 LFO1-4, 4-7 EU1-4, 8-9 GATE1-2,
// 10-11 ADSR1-2), CC 34 = parameter select (per-source param index), CC 35 =
// value (0-127, mapped per param). Verified UNUSED by all four adapters' CC
// maps and by this parser (grep: no `case 3[0-9]` outside CC 32). Both
// handlers run on CORE 0 (called from midiControlChange below); they touch
// only engine scalars (atomic aligned 32-bit writes) — the swap path is not
// involved and keeps working during swaps.
static const uint8_t kModBenchCcSource = 33;
static const uint8_t kModBenchCcParam  = 34;
static const uint8_t kModBenchCcValue  = 35;
// Defined in mod_engine.h (included by main.cpp after the adapters).
void rack_mod_bench_cc(uint8_t cc, uint8_t val);

// --- clouds grain-parameter ALIAS BANK (rack v2) --------------------------
// CC 40-48 addresses clouds' grain parameters 14-22 (position..reverb)
// whenever clouds is the TARGET — single-module clouds, or the B slot while
// chained. Rationale: while chained, clouds' native 14-22 collide with every
// chain-A module's map (elements 14-22 are env/bow/blow/strike, plaits 14-22
// timbre..morph-mod, rings 14-17 structure..position), so the chain
// intercept cannot hand the native band to clouds-B without stealing A's
// panel. The alias band lives in the free range (36-63 is unused by all four
// maps and the rack system CCs) and is ALWAYS ON: the web panel sends 40-48
// in both modes, so its clouds view needs no chain-state knowledge. Native
// wire CCs 14-22 keep their existing meaning everywhere (A's panel params
// while chained; clouds' own params in single-module mode).
static const uint8_t kCloudsAliasCcBase = 40;   // 40-48 -> clouds 14-22
// DEFINED in main.cpp: true when the alias band should take the registry
// path (single-module clouds). While chained the band is grabbed by
// rack_chain_wants_cc below and translated inside rack_chain_control_change.
bool rack_clouds_alias_active();

// T6 chain-mode CC intercept (DEFINED in main.cpp). While a chain is
// engaged, CCs 23/24/65/66 AND the clouds grain alias bank 40-48 (see
// kCloudsAliasCcBase) belong to the B slot (clouds) and are handled by the
// chain machinery (mode clamp / quality / freeze / trig / grain params);
// every other CC goes to the A slot (the sound source) through the normal
// registry dispatch. CC 23 is dual-mapped in every module map (elements
// geometry / plaits engine / rings chord) — while chained it means clouds-B
// mode, so the A slot loses CC 23/24 to the clamp (the mod engine can still
// drive A's parameters — destinations bind to A). 65/66 are unused by all
// four maps; 40-48 is unused everywhere (the alias bank's free range).
bool rack_chain_wants_cc(uint8_t cc);
void rack_chain_control_change(uint8_t cc, uint8_t val);

// T3 clock-bus realtime funnel: F8 (timing clock) / FA (start) / FB
// (continue) / FC (stop), stamped with the render-loop sample position by
// CORE 0 at this single entry point. Both USB paths (device readPacket +
// PIO-USB host poll) call midiHandleEventPacket below, so the stamping is
// automatically live on both; T4's DIN parser needs exactly one call to
// midiHandleEventPacket to join. Defined in mod_engine.h.
void rack_mod_clock_realtime(uint8_t status);

// T5 UI-link CC-value observer (defined in ui_link.h, main.cpp TU): records
// the wire CC under the active module for the TELEM value cache and, when
// the CC is a cell of the C3's active page, emits a throttled TELEM delta.
// PURE OBSERVATION — called at the top of midiControlChange before any
// routing decision; dispatch semantics are untouched.
void ui_link_observe_cc(uint8_t cc, uint8_t val);

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
  ++note_on_count;
  last_note_event_ms = millis();
  rack_module_note_on(note, vel);  // registry -> active adapter
}

inline void midiNoteOff(uint8_t note) {
  note_stack.remove(note);
  if (note_stack.size == 0) {
    shared_gate = false;          // gate only drops when the stack empties
  } else {
    applyNote();                  // released note was on top: fall back to the
  }                               // previous held note
  last_note_event_ms = millis();
  rack_module_note_off(note);
}

inline void midiPitchBend(int16_t bend14) {  // -8192..8191, 0 = centered
  bend_offset = bend14;
  applyNote();                    // bend always applies to the current top note
}

inline void midiControlChange(uint8_t cc, uint8_t val) {
  ui_link_observe_cc(cc, val);   // T5: TELEM value cache (pure observation)
  if (cc == kRackSwitchCc) {
    rack_request_switch(val);     // CC 32 = live module switch / chain-slot
                                  // semantics while chained (T6 decision 5)
    return;
  }
  if (cc >= kModBenchCcSource && cc <= kModBenchCcValue) {
    rack_mod_bench_cc(cc, val);   // CC 33/34/35 = mod-engine bench mux (T3)
    return;
  }
  // Clouds grain alias bank, single-module path (see kCloudsAliasCcBase):
  // only when clouds itself is the active module — other actives leave the
  // band untouched (unused CCs in their maps). While chained the band is
  // intercepted by rack_chain_wants_cc below and routed to the B slot.
  if (cc >= kCloudsAliasCcBase && cc <= kCloudsAliasCcBase + 8 &&
      rack_clouds_alias_active()) {
    rack_module_control_change(static_cast<uint8_t>(cc - kCloudsAliasCcBase + 14), val);
    return;
  }
  if (rack_chain_wants_cc(cc)) {
    rack_chain_control_change(cc, val);   // T6: B-slot CCs while chained
    return;
  }
  rack_module_control_change(cc, val);  // registry -> active adapter CC map
}

// USB-MIDI event packet (4 bytes) -> channel voice messages.
// CIN 0x8..0xE map 1:1 onto the MIDI status nibble (single cable).
// CIN 0xF = single-byte SYSTEM REALTIME: F8/FA/FB/FC are the MIDI-clock
// transport and feed the mod engine's clock bus (stamped with the render
// sample position, see mod_engine.h). CIN 0x5 (single-byte system common) is
// also accepted for status >= 0xF8 — some marginal sources send realtime
// bytes with CIN 0x5; the status check makes the two unambiguous.
inline void midiHandleEventPacket(const uint8_t pkt[4]) {
  const uint8_t cin = pkt[0] & 0x0F;
  if (cin == 0xF || cin == 0x5) {
    const uint8_t st = pkt[1];
    if (st >= 0xF8) rack_mod_clock_realtime(st);  // F8 clock / FA-FB-FC transport
    return;
  }
  if (cin < 0x8 || cin > 0xE) return;      // ignore system common / SYSEX
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
