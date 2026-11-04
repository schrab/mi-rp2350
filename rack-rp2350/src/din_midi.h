// Rack RP2350 — T4 DIN MIDI byte-stream parser (MIDI 1.0 -> USB-MIDI packets).
//
// PURE header: stdint/stdio only, no Arduino/SDK types — the SAME file is
// compiled into the firmware (main.cpp) and into the T4 host unit test
// (throwaway, /tmp; results in the task report + ledger). All state lives in
// the DinMidiParser struct; the UART plumbing (UART1, IRQ ring, funnel call)
// lives in main.cpp — this file never touches hardware.
//
// OUTPUT CONTRACT (identical to the device + PIO-USB host paths — see
// midi.h): 4-byte USB-MIDI event packets [cable<<4|CIN, status, d1, d2] fed
// to midiHandleEventPacket(). ONE funnel, zero special cases downstream: the
// mod engine's sample-stamped clock (rack_mod_clock_realtime) automatically
// sees DIN F8/FA/FB/FC because midiHandleEventPacket routes CIN 0xF single-
// byte system realtime there (midi.h). CIN assignment per the USB-MIDI spec:
//   0x8..0xE  channel voice messages (status high nibble)
//   0xF       single-byte system realtime (F8-FF)
//   0x5       single-byte system common (F6 tune request)
//   0x2       2-byte system common (F1 MTC quarter frame / F3 song select)
//   0x3       3-byte system common (F2 song position pointer)
// NOTE the rack funnel currently consumes only CIN 0x8-0xE (module MIDI) and
// 0xF with status >= F8 (clock transport); system common (0x2/0x3/0x5) is
// parsed and counted but has NO consumer downstream — exactly like the USB
// paths, which never emit it. FE (active sense) reaches the funnel and is
// filtered by rack_mod_clock_realtime's status check.
//
// STATE MACHINE (per byte):
//   IDLE    no running status; data bytes here are ORPHANS (dropped++)
//   READY   running status held (run_status); next data byte starts a new
//           message under it (MIDI 1.0 running status)
//   COLLECT pend_status set, `need` data bytes still expected (channel
//           message or system common F1/F2/F3)
//   SYSEX   F0 received; body bytes DISCARDED (counted once per message)
//
// POLICIES (documented decisions, all MIDI 1.0 + bench-driven):
//   - System REALTIME (F8-FF) is emitted IMMEDIATELY, anywhere — idle,
//     mid-message, mid-sysex — and NEVER disturbs running status, an
//     in-flight message, or sysex state (MIDI 1.0 spec; this is what makes
//     "clock inside a note-on" work).
//   - RUNNING STATUS: a completed channel message returns to READY under the
//     same status. Any SYSTEM COMMON byte (F1-F7) CLEARS running status.
//     Realtime does not touch it.
//   - SYSEX: DISCARDED from the F0 (T4 scope: no downstream consumer). One
//     dropped++ per discarded message, materializing at the END of the
//     sysex: its F7, an aborting non-realtime status byte, or a second F0.
//     A stray F7 (no F0 in flight) is ignored and NOT counted (it ends
//     nothing we counted). Body bytes are silent discards (no per-byte
//     counter noise).
//   - ABORTED INCOMPLETE messages: a non-realtime status byte arriving while
//     COLLECT is mid-message drops the fragment (dropped++, per spec "the
//     incomplete message is lost") and the new message starts normally.
//   - UNDEFINED statuses F4/F5: dropped++ (one per byte), state untouched.
//   - Data bytes are passed through AS RECEIVED (the funnel masks d1/d2 with
//     &0x7F, identical to the USB paths). Parser-side validation stops at
//     the status/data class boundary.
//   - OVERFLOW: the parser is byte-at-a-time with NO message buffer, so it
//     cannot overflow. The only overflow point is the UART RX ring in
//     main.cpp (256 B, IRQ producer); that count is folded into `dropped`
//     by the RX layer (Serial2.overflow()).
//
// COUNTERS (T4 telemetry; surfaced by the console `status`/`din` commands —
// deliberately NOT in the 2 s report line, which must stay byte-compatible):
//   rx_bytes / messages / realtime / dropped + last packet snapshot.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// DIN is cable 0 on the rack's virtual USB-MIDI device (device/host paths
// carry whatever cable the sender used; 0 is also what the panels use).
static const uint8_t kDinMidiCable = 0;

// The `dintest` canned stream (shared firmware/host-test byte-for-byte so
// the bench expectations and the unit test can never drift apart):
//   1) explicit-status note on C4 vel 100        -> CIN 9 packet
//   2) note OFF via running status (vel 0)       -> CIN 9 vel 0 (funnel ->
//      midiNoteOff), running status continues
//   3) CC 7 = 100                                -> CIN B
//   4) pitch bend center                         -> CIN E
//   5) clock byte F8 INSIDE a note-on            -> F8 packet first, then the
//      completed note-on (realtime passes anywhere)
//   6) running status survived the realtime byte: note on D2... 0x43 = G2,
//      plus its vel-0 note off
//   7) transport: FA + one full beat of 24 PPQ clock + FC
//   8) a sysex (GM-enable-shaped)                -> discarded, dropped +1
//   9) an orphan data byte (broken stream)       -> dropped +1
// Totals: 52 bytes -> 34 packets (27 realtime: 25x F8 + FA + FC), 2 dropped.
// Funnel effects: note_on_count +3 (60, 64, 67); clock bus sees FA + 25 F8 +
// FC (midi_tick_count +25 once core 1 drains the stamp ring).
static const uint8_t kDinTestStream[] = {
    0x90, 0x3C, 0x64,                                     // note on 60
    0x3C, 0x00,                                           // note off (running)
    0xB0, 0x07, 0x64,                                     // CC 7 = 100
    0xE0, 0x00, 0x40,                                     // bend center
    0x90, 0x40, 0xF8, 0x7F,                               // F8 inside note on
    0x43, 0x40,                                           // note on 67 (running)
    0x43, 0x00,                                           // note off 67 (running)
    0xFA,                                                 // START
    0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,       // 24 PPQ clock
    0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,
    0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8,
    0xFC,                                                 // STOP
    0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7,                   // sysex -> discard
    0x55,                                                 // orphan data byte
};
static const uint32_t kDinTestStreamLen =
    sizeof(kDinTestStream) / sizeof(kDinTestStream[0]);

struct DinMidiParser {
  // ---- telemetry (public; single core-0 writer in the firmware) ----
  uint32_t rx_bytes = 0;    // every byte handed to parse()
  uint32_t messages = 0;    // 4-byte packets emitted
  uint32_t realtime = 0;    // CIN 0xF packets (F8-FF)
  uint32_t dropped = 0;     // sysex discarded + aborted + orphan + undefined
                            // (+ UART ring overflow, folded in by main.cpp)
  uint8_t last_pkt[4] = {0, 0, 0, 0};
  bool have_last = false;

  // ---- state ----
  enum State : uint8_t { ST_IDLE = 0, ST_READY = 1, ST_COLLECT = 2, ST_SYSEX = 3 };
  uint8_t state = ST_IDLE;
  uint8_t run_status = 0;    // running status (0 = none)
  uint8_t pend_status = 0;   // status of the message being collected
  uint8_t need = 0;          // data bytes still expected (1 or 2 in COLLECT)
  uint8_t d1 = 0;            // first data byte collected

  void reset_state() {
    state = ST_IDLE;
    run_status = pend_status = need = d1 = 0;
  }

  // Data bytes required after a channel status (C/D take 1, others 2).
  static uint8_t chan_need(uint8_t st) {
    const uint8_t t = st & 0xF0;
    return (t == 0xC0 || t == 0xD0) ? 1u : 2u;
  }

  // Parse ONE byte; returns true and fills pkt[4] when a complete USB-MIDI
  // event packet is emitted.
  bool parse(uint8_t b, uint8_t pkt[4]) {
    ++rx_bytes;
    if (b >= 0xF8) {                       // system realtime: inline anywhere
      emit(pkt, 0xF, b, 0, 0);
      ++realtime;
      return true;
    }
    if (b & 0x80) return parse_status(b, pkt);
    return parse_data(b, pkt);
  }

  // ---- internal: emit + bookkeeping ----
  void emit(uint8_t pkt[4], uint8_t cin, uint8_t st, uint8_t d1_, uint8_t d2_) {
    pkt[0] = (uint8_t)((kDinMidiCable << 4) | (cin & 0x0F));
    pkt[1] = st;
    pkt[2] = d1_;
    pkt[3] = d2_;
    memcpy(last_pkt, pkt, 4);
    have_last = true;
    ++messages;
  }

  // Non-realtime status byte (F0-F7 system common/sysex, 80-EF channel).
  bool parse_status(uint8_t b, uint8_t pkt[4]) {
    if (b == 0xF7) {                       // EOX
      if (state == ST_SYSEX) {             // the discard materializes here
        ++dropped;
        state = ST_IDLE;
      }
      run_status = 0;                      // system common clears running status
      return false;
    }
    // Any other non-realtime status aborts an in-flight sysex or incomplete
    // message (one dropped each, per the policies in the header).
    if (state == ST_SYSEX || state == ST_COLLECT) ++dropped;
    if (b == 0xF0) {                       // sysex: discard from here on
      state = ST_SYSEX;
      run_status = 0;
      return false;
    }
    run_status = 0;                        // F1-F7 + all channel statuses
                                           // clear running status (channel
                                           // statuses re-set it below)
    if (b >= 0xF1 && b <= 0xF6) {          // system common
      switch (b) {
        case 0xF1:                         // MTC quarter frame: 1 data byte
        case 0xF3:                         // song select: 1 data byte
          pend_status = b;
          need = 1;
          state = ST_COLLECT;
          return false;
        case 0xF2:                         // song position: 2 data bytes
          pend_status = b;
          need = 2;
          state = ST_COLLECT;
          return false;
        case 0xF6:                         // tune request: no data bytes
          emit(pkt, 0x5, b, 0, 0);
          state = ST_IDLE;
          return true;
        default:                           // F4/F5 undefined: drop, no state
          ++dropped;
          state = ST_IDLE;
          return false;
      }
    }
    // Channel voice message 0x80..0xEF: start collecting under running status.
    pend_status = b;
    run_status = b;
    need = chan_need(b);
    state = ST_COLLECT;
    return false;
  }

  // Data byte (00-7F).
  bool parse_data(uint8_t b, uint8_t pkt[4]) {
    if (state == ST_SYSEX) return false;   // body discarded silently
    if (state == ST_IDLE) {                // no running status: orphan
      ++dropped;
      return false;
    }
    if (state == ST_READY) {               // start a message under run_status
      pend_status = run_status;
      need = chan_need(run_status);
      state = ST_COLLECT;
    }
    // ST_COLLECT: pend_status is channel (80-EF, run_status set) or system
    // common (F1/F2/F3, run_status 0 — the abort rules above already
    // guaranteed a data byte cannot land here under a cleared system-common
    // collect: F4-F7 reset the state).
    if (need == 2) {
      d1 = b;
      need = 1;
      return false;
    }
    // need == 1: this byte completes the message. Two shapes: 2-data-byte
    // messages (d1 was stored, b is the second byte) and 1-data-byte
    // messages (b IS the only data byte — d1 is stale here and must not be
    // used; the unit test caught exactly that).
    if (pend_status >= 0xF0) {             // system common completion
      if (pend_status == 0xF2) {
        emit(pkt, 0x3, pend_status, d1, b);
      } else {                             // F1 / F3: single data byte
        emit(pkt, 0x2, pend_status, b, 0);
      }
      state = ST_IDLE;                     // system common sets no running status
      return true;
    }
    if (chan_need(pend_status) == 2) {     // 8/9/A/B/E: d1 + b
      emit(pkt, (uint8_t)(pend_status >> 4), pend_status, d1, b);
    } else {                               // C/D: single data byte, d2 = 0
      emit(pkt, (uint8_t)(pend_status >> 4), pend_status, b, 0);
    }
    state = ST_READY;                      // running status continues
    return true;
  }
};

// Human-readable decode of one 4-byte packet (console `din` + `dintest`;
// pure snprintf so the host test can assert on it too).
static void din_describe(const uint8_t pkt[4], char* out, size_t n) {
  const uint8_t cin = pkt[0] & 0x0F;
  switch (cin) {
    case 0x8:
      snprintf(out, n, "note off %u", pkt[2] & 0x7F);
      return;
    case 0x9:
      if (pkt[3] & 0x7F) snprintf(out, n, "note on %u vel %u", pkt[2] & 0x7F, pkt[3] & 0x7F);
      else snprintf(out, n, "note off %u (vel 0)", pkt[2] & 0x7F);
      return;
    case 0xA:
      snprintf(out, n, "poly press %u = %u", pkt[2] & 0x7F, pkt[3] & 0x7F);
      return;
    case 0xB:
      snprintf(out, n, "cc %u = %u", pkt[2] & 0x7F, pkt[3] & 0x7F);
      return;
    case 0xC:
      snprintf(out, n, "program %u", pkt[2] & 0x7F);
      return;
    case 0xD:
      snprintf(out, n, "chan press %u", pkt[2] & 0x7F);
      return;
    case 0xE:
      snprintf(out, n, "bend %d", (int)(((pkt[3] & 0x7F) << 7) | (pkt[2] & 0x7F)) - 8192);
      return;
    case 0xF: {
      switch (pkt[1]) {
        case 0xF8: snprintf(out, n, "realtime F8 (clock)"); return;
        case 0xFA: snprintf(out, n, "realtime FA (start)"); return;
        case 0xFB: snprintf(out, n, "realtime FB (continue)"); return;
        case 0xFC: snprintf(out, n, "realtime FC (stop)"); return;
        case 0xFE: snprintf(out, n, "realtime FE (active sense)"); return;
        case 0xFF: snprintf(out, n, "realtime FF (reset)"); return;
        default: snprintf(out, n, "realtime %02x", pkt[1]); return;
      }
    }
    case 0x2:
      snprintf(out, n, "sys common %02x %02x", pkt[1], pkt[2]);
      return;
    case 0x3:
      snprintf(out, n, "sys common %02x %02x %02x", pkt[1], pkt[2], pkt[3]);
      return;
    case 0x5:
      snprintf(out, n, "sys common %02x", pkt[1]);
      return;
    default:
      snprintf(out, n, "cin %u (no consumer)", cin);
      return;
  }
}
