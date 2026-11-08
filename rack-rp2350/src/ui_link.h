// T5 STAGE 1 — UI-MCU link (ESP32-C3 <-> RP2350, 921600 8N1 framed UART).
//
// The C3 (rack-uimcu, seeded from ctag-tbd/uimcu) drives the rack's UI: it
// sends MIDI (its pots/BLE), module switches, mod-engine sets, chain routes
// and UI commands; the RP2350 answers with TELEM page snapshots. Protocol
// contract (BYTE-EXACT on both ends, full details in docs/ui-protocol.md):
//
//   frame  = [0x7E][len:u8][type:u8][payload... len bytes][crc8]
//   len    = payload length 0-255 (the 0x7E mark may occur INSIDE a payload;
//            no byte stuffing — len is known up front)
//   crc8   = CRC-8/ATM (poly 0x07, init 0x00, MSB-first, no reflection,
//            no xorout) computed over [type, then payload bytes]
//
// C3 -> RP2350 types: 0x01 MIDI (payload = 4-byte USB-MIDI event packet ->
// midiHandleEventPacket, the SAME funnel USB device/host/DIN use — zero new
// dispatch semantics), 0x02 SWITCH (module id -> the console `switch` path),
// 0x03 MOD_SET ([src][param][value] -> the mod engine's bench setters),
// 0x04 UI_CMD ([action][arg][arg2]: 1 = page request -> TELEM snapshot,
// 2 = echo -> echo back + RTT, 3 = sys toggle -> log-only in stage 1),
// 0x05 CHAIN_ROUTE ([a][b][engage] -> the console `chain` path). Types
// 0x11/0x12/0x13 (PRESET_REQ/PART/APPLY) are RESERVED for T4a — received
// frames are logged only. RP2350 -> C3: 0x10 TELEM.
//
// UART choice: Serial1 = HW UART0, TX = GP16, RX = GP17. Pin validity was
// verified against the earlephilhower core's SerialUART tables (RP2350A
// branch, SerialUART.cpp): UART0 TX valid = {0,2,12,14,16,18,28}, RX valid =
// {1,3,13,15,17,19,29} — GP16/GP17 are the pair (the brief's TX=17/RX=16
// guess was REVERSED; both pins are free: used pins are 4/5 DIN, 6/7/8 I2S,
// 14/15 PIO-USB host, 22 LED, 24/25 native USB). Hardware UART claims no
// PIO/alarm/DMA, so the init position is unconstrained by the PIO-USB/I2S
// order law (same argument as DIN UART1). NOTE: no internal loopback exists
// on the RP2350's PL011-style UART (no LBM bit in UARTLCR_H — checked the
// pico-sdk register headers), so `uitest` runs the IN-MEMORY codec test;
// wire verification is the C3's echo/hw-loopback self-test plus the bench.
//
// Ingest (core 0): the core's SerialUART IRQ keeps a 512 B SW ring fed; the
// loop-side pump drains it into the frame parser (same pattern as the DIN
// pump). With the link absent/unwired the pump costs one ring-empty check.
//
// TELEM value source: a per-module CC-value cache (uint8_t x 128 per module,
// 512 B .bss) observed at the single control_change dispatch point
// (midiControlChange in midi.h calls ui_link_observe_cc — pure observation,
// no dispatch change). Cache starts zeroed; values appear once a controller
// (panel/BLE/UI) sends the CC. Routed mod sources override the audio value
// but not the cached panel value — the TELEM flags bit0 (mod-routed) marks
// those cells instead.
//
// Registry page tables (the design's single source of truth): per module,
// P1 = the primary cells {cc, label, color class} in pot-row order (6/4/6),
// built from the four adapters' CC maps + the knob labels/colors of
// tools/rack_panel.html (the rack's current recolor; tools/elements_panel.html
// has the same trio). Color classes: 0 = white (standard params), 1 = red
// #e6295e group, 2 = teal #17a3bd group; the panels' yellow #f2d21f is a
// STATE accent, never a param class. P2: elements = CC 28/29 selectors +
// the alt-layout duplicates of its acoustic params; clouds/plaits/rings =
// duplicate of P1 (page id differs). MOD-A/B/C and SYS report count=0 in
// stage 1 (stage 4 fills them from mod_engine.h / chain state).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <Arduino.h>             // Serial1, millis, Serial
#include "pico/time.h"           // time_us_32 (echo turnaround timing)

// ===================== protocol constants =====================
static const uint8_t kUiFrameMark = 0x7E;

// Frame types (u8).
static const uint8_t kUiTypeMidi        = 0x01;  // 4-byte USB-MIDI event packet
static const uint8_t kUiTypeSwitch      = 0x02;  // module id
static const uint8_t kUiTypeModSet      = 0x03;  // [src][param][value]
static const uint8_t kUiTypeUiCmd       = 0x04;  // [action][arg][arg2]
static const uint8_t kUiTypeChainRoute  = 0x05;  // [a][b][engage]
static const uint8_t kUiTypeTelem       = 0x10;  // RP2350 -> C3 only
static const uint8_t kUiTypePresetReq   = 0x11;  // reserved, T4a
static const uint8_t kUiTypePresetPart  = 0x12;  // reserved, T4a
static const uint8_t kUiTypePresetApply = 0x13;  // reserved, T4a

// UI_CMD actions.
static const uint8_t kUiCmdPageRequest = 1;      // arg = page id
static const uint8_t kUiCmdEcho        = 2;      // arg/arg2 = 16-bit seq
static const uint8_t kUiCmdSysToggle   = 3;      // log-only in stage 1

// Page ids.
static const uint8_t kUiPageP1   = 0;
static const uint8_t kUiPageP2   = 1;
static const uint8_t kUiPageModA = 2;
static const uint8_t kUiPageModB = 3;
static const uint8_t kUiPageModC = 4;
static const uint8_t kUiPageSys  = 5;
static const uint8_t kUiPageCount = 6;

// TELEM modes: 0 = full snapshot, 1 = single-cell delta (same format).
static const uint8_t kUiTelemFull  = 0;
static const uint8_t kUiTelemDelta = 1;

// TELEM cell flags.
static const uint8_t kUiFlagModRouted = 1 << 0;  // a mod source drives this CC
static const uint8_t kUiFlagChainB    = 1 << 1;  // cell belongs to the chain B slot
static const uint8_t kUiFlagInert     = 1 << 2;  // routing exists but is inert/rejected

// Param color classes (tools/rack_panel.html trio).
static const uint8_t kUiColorWhite = 0;
static const uint8_t kUiColorRed   = 1;
static const uint8_t kUiColorTeal  = 2;

// ===================== CRC-8/ATM (poly 0x07, init 0x00, MSB-first) ==========
static uint8_t ui_crc8(const uint8_t* data, size_t n) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ===================== registry page tables =====================
// P1 = 16 pot cells in physical order (top row 1-6, middle row 7-10, bottom
// row 11-16) — the C3 mirrors the same 6/4/6 layout. Only non-empty cells
// are stored; TELEM sends `count` cells. Labels <= 10 chars (digi-slim 6 px).
struct UiPageCell {
  uint8_t cc;
  const char* label;
  uint8_t color;      // kUiColor*
};
struct UiPageTable {
  const UiPageCell* cells;
  uint8_t count;
};

// elements — P1: perf modulators (red), acoustic excitation (teal), resonator
// + envelope (white); colors per tools/rack_panel.html.
static const UiPageCell kUiPageElementsP1[] = {
    {1,  "MOD",        kUiColorRed},
    {11, "STRENGTH",   kUiColorRed},
    {14, "CONTOUR",    kUiColorWhite},
    {15, "BOW",        kUiColorWhite},
    {16, "TIMBRE B",   kUiColorWhite},
    {17, "BLOW",       kUiColorTeal},
    {18, "FLOW",       kUiColorTeal},
    {19, "TIMBRE Bl",  kUiColorTeal},
    {20, "STRIKE",     kUiColorTeal},
    {21, "MALLET",     kUiColorTeal},
    {22, "TIMBRE S",   kUiColorTeal},
    {23, "GEOMETRY",   kUiColorWhite},
    {24, "BRIGHTNESS", kUiColorWhite},
    {25, "DAMPING",    kUiColorWhite},
    {26, "POSITION",   kUiColorWhite},
    {27, "SPACE",      kUiColorWhite},
};
// elements — P2: the two selectors (CC 28/29) + the alt-layout duplicates of
// the acoustic params (the panel's per-group rows without the perf mods).
static const UiPageCell kUiPageElementsP2[] = {
    {28, "MODEL",      kUiColorWhite},
    {29, "OMINOUS",    kUiColorWhite},
    {14, "CONTOUR",    kUiColorWhite},
    {15, "BOW",        kUiColorWhite},
    {16, "TIMBRE B",   kUiColorWhite},
    {17, "BLOW",       kUiColorTeal},
    {18, "FLOW",       kUiColorTeal},
    {19, "TIMBRE Bl",  kUiColorTeal},
    {20, "STRIKE",     kUiColorTeal},
    {21, "MALLET",     kUiColorTeal},
    {22, "TIMBRE S",   kUiColorTeal},
    {23, "GEOMETRY",   kUiColorWhite},
    {24, "BRIGHTNESS", kUiColorWhite},
    {25, "DAMPING",    kUiColorWhite},
    {26, "POSITION",   kUiColorWhite},
    {27, "SPACE",      kUiColorWhite},
};
// clouds — P1: the rack panel's clouds view. Grain params travel the alias
// bank CC 40-48 (kCloudsAliasCcBase — works single-module AND chained);
// exciter 1/11/13, selectors 23/24, freeze/trig 65/66. All white in the panel.
static const UiPageCell kUiPageCloudsP1[] = {
    {1,  "EX TONE",    kUiColorWhite},
    {11, "EX LEVEL",   kUiColorWhite},
    {13, "EX SRC",     kUiColorWhite},
    {40, "POSITION",   kUiColorWhite},
    {41, "SIZE",       kUiColorWhite},
    {42, "PITCH",      kUiColorWhite},
    {43, "DENSITY",    kUiColorWhite},
    {44, "TEXTURE",    kUiColorWhite},
    {45, "DRY/WET",    kUiColorWhite},
    {46, "SPREAD",     kUiColorWhite},
    {47, "FEEDBACK",   kUiColorWhite},
    {48, "REVERB",     kUiColorWhite},
    {23, "MODE",       kUiColorWhite},
    {24, "QUALITY",    kUiColorWhite},
    {65, "FREEZE",     kUiColorWhite},
    {66, "TRIG",       kUiColorWhite},
};
// plaits — P1: panel colors (MACRO red; FM/TIM-MOD/MORPH-MOD teal). CC 64 is
// the TRIG pseudo-CC (mod-engine trigger input). 13 cells (3 pots empty).
static const UiPageCell kUiPagePlaitsP1[] = {
    {14, "TIMBRE",     kUiColorWhite},
    {15, "MORPH",      kUiColorWhite},
    {16, "HARMONICS",  kUiColorWhite},
    {17, "MACRO",      kUiColorRed},
    {18, "LPG",        kUiColorWhite},
    {19, "DECAY",      kUiColorWhite},
    {20, "FM AMT",     kUiColorTeal},
    {21, "TIM MOD",    kUiColorTeal},
    {22, "MORPH MOD",  kUiColorTeal},
    {25, "LEVEL",      kUiColorWhite},
    {23, "ENGINE",     kUiColorWhite},
    {24, "AUX",        kUiColorWhite},
    {64, "TRIG",       kUiColorWhite},
};
// rings — P1: panel colors (all white). CC 64 = STRUM pseudo-CC. 11 cells.
static const UiPageCell kUiPageRingsP1[] = {
    {14, "STRUCTURE",  kUiColorWhite},
    {15, "BRIGHTNESS", kUiColorWhite},
    {16, "DAMPING",    kUiColorWhite},
    {17, "POSITION",   kUiColorWhite},
    {18, "MODEL/FX",   kUiColorWhite},
    {19, "POLY",       kUiColorWhite},
    {20, "MODE",       kUiColorWhite},
    {21, "WAVE",       kUiColorWhite},
    {22, "CHORD TB",   kUiColorWhite},
    {23, "CHORD",      kUiColorWhite},
    {64, "STRUM",      kUiColorWhite},
};

// [module][0 = P1, 1 = P2]. P2 duplicates P1 for clouds/plaits/rings (the
// design's "most-used 16", page id differs).
static const UiPageTable kUiPages[RACK_MODULE_COUNT][2] = {
    { {kUiPageElementsP1, 16}, {kUiPageElementsP2, 16} },  // elements
    { {kUiPageCloudsP1, 16},   {kUiPageCloudsP1, 16} },    // clouds
    { {kUiPagePlaitsP1, 13},   {kUiPagePlaitsP1, 13} },    // plaits
    { {kUiPageRingsP1, 11},    {kUiPageRingsP1, 11} },     // rings
};
static const UiPageTable kUiPageEmpty = { nullptr, 0 };   // MOD-A/B/C, SYS

// ===================== CC-value cache (TELEM source) =====================
// Observed at the single dispatch point (midiControlChange -> the hook below).
// 4 modules x 128 CCs = 512 B .bss. The full-CC-range span was chosen over a
// remapped 32-CC window to cover CC 64/65/66 (trig/strum/freeze) and the
// clouds alias bank 40-48 without index arithmetic.
static uint8_t g_ui_cc_cache[RACK_MODULE_COUNT][128];

// Last page the C3 requested (delta policy target; default P1).
static uint8_t g_ui_active_page = kUiPageP1;

// Link stats (for `ui`; counters are core-0-only, no cross-core races).
static volatile uint32_t g_ui_tx_frames = 0;
static volatile uint32_t g_ui_rx_frames = 0;
static volatile uint32_t g_ui_crc_errors = 0;
static volatile uint32_t g_ui_rx_overflow = 0;
static volatile uint32_t g_ui_last_rtt_us = 0;

// ===================== UART (Serial1 = HW UART0) =====================
static const uint8_t kPinUiTx = 16;   // UART0 TX — valid per the core's
                                      // RP2350A table {0,2,12,14,16,18,28}
static const uint8_t kPinUiRx = 17;   // UART0 RX — valid {1,3,13,15,17,19,29}

static void ui_link_uart_init() {
  // ORDER (all three before begin, same as the DIN init): FIFO size + pinout
  // apply only while _running == false. 512 B SW ring: at 921600 baud max
  // (~92 KB/s) it rides out ~5.6 ms between drains — far beyond any loop()
  // stall (swap prints, LED poll ~1 ms, one TELEM write ~2.6 ms).
  Serial1.setFIFOSize(512);
  Serial1.setPinout(kPinUiTx, kPinUiRx);
  Serial1.begin(921600, SERIAL_8N1);
}

// ===================== TX =====================
static uint8_t g_ui_tx_buf[3 + 255 + 1];  // hdr+type already counted: 0x7E,
                                          // len, type, payload(255), crc

// Build a complete frame into `out` (needs len + 4 bytes). Returns the frame
// length in bytes. Separate from the UART write so `uitest` can round-trip
// frames through the parser in memory.
static size_t ui_link_build_frame(uint8_t type, const uint8_t* payload,
                                  uint8_t len, uint8_t* out) {
  out[0] = kUiFrameMark;
  out[1] = len;
  out[2] = type;
  if (len) memcpy(&out[3], payload, len);
  // CRC-8/ATM over [type, then payload bytes].
  out[3 + len] = ui_crc8(&out[2], (size_t)len + 1);
  return (size_t)len + 4;   // mark + len + type + payload + crc
}

// Encode + transmit one frame (blocking into the SerialUART TX ring).
static void ui_link_send_frame(uint8_t type, const uint8_t* payload,
                               uint8_t len) {
  const size_t n = ui_link_build_frame(type, payload, len, g_ui_tx_buf);
  Serial1.write(g_ui_tx_buf, n);
  ++g_ui_tx_frames;
}

// TELEM flags for one cell (see kUiFlag*).
static uint8_t ui_link_cell_flags(uint8_t cc, uint8_t mod) {
  uint8_t flags = 0;
  bool routed = false, inert = false;
  const RackModDestinations* d = kRackModules[mod].mod_destinations;
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    const ModSource& s = g_mod_engine.src[i];
    if (s.dest_cc != cc) continue;
    routed = true;
    const bool want_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR);
    bool ok = false;
    if (want_continuous) {
      for (uint8_t k = 0; k < d->continuous_count; ++k) {
        if (d->continuous[k] == cc) { ok = true; break; }
      }
    } else {
      for (uint8_t k = 0; k < d->trigger_count; ++k) {
        if (d->trigger_cc[k] == cc) { ok = true; break; }
      }
    }
    if (!ok) inert = true;
  }
  if (routed) flags |= kUiFlagModRouted;
  if (g_chain.active && mod == g_chain.b) flags |= kUiFlagChainB;
  if (inert) flags |= kUiFlagInert;
  return flags;
}

// Full TELEM snapshot (mode 0) or a 1-cell delta (mode 1, delta_cc = the cell).
// Format: [page][module][mode][count][count x (cc,value,flags)][count x
// NUL-terminated labels in cell order]. Max payload: 4 + 16*3 + labels ~110
// = ~162 B — comfortably inside the 255-byte len.
static void ui_link_send_telem(uint8_t page, uint8_t mode, uint8_t delta_cc) {
  const uint8_t mod = g_active_module;
  const UiPageTable* t = &kUiPageEmpty;
  if (page <= kUiPageP2) t = &kUiPages[mod][page];
  const UiPageCell* cells = t->cells;
  uint8_t count = t->count;

  uint8_t pl[256];
  size_t n = 0;
  if (mode == kUiTelemFull) {
    pl[n++] = page;
    pl[n++] = mod;
    pl[n++] = mode;
    pl[n++] = count;
    for (uint8_t i = 0; i < count; ++i) {
      pl[n++] = cells[i].cc;
      pl[n++] = g_ui_cc_cache[mod][cells[i].cc];
      pl[n++] = ui_link_cell_flags(cells[i].cc, mod);
    }
    for (uint8_t i = 0; i < count; ++i) {
      const char* lab = cells[i].label;
      size_t l = strlen(lab) + 1;               // include the NUL
      memcpy(&pl[n], lab, l);
      n += l;
    }
  } else {
    // Delta: reuse the full format with count = 1 (documented in
    // docs/ui-protocol.md) — the C3 parser is identical for both modes.
    int8_t idx = -1;
    for (uint8_t i = 0; i < count; ++i) {
      if (cells[i].cc == delta_cc) { idx = (int8_t)i; break; }
    }
    if (idx < 0) return;                        // not a cell of this page
    pl[n++] = page;
    pl[n++] = mod;
    pl[n++] = mode;
    pl[n++] = 1;
    pl[n++] = cells[idx].cc;
    pl[n++] = g_ui_cc_cache[mod][delta_cc];
    pl[n++] = ui_link_cell_flags(delta_cc, mod);
    const char* lab = cells[idx].label;
    size_t l = strlen(lab) + 1;
    memcpy(&pl[n], lab, l);
    n += l;
  }
  if (n > 255) return;                          // cannot happen; guard anyway
  ui_link_send_frame(kUiTypeTelem, pl, (uint8_t)n);
}

// ===================== CC-value observation hook =====================
// Declared in midi.h; called at the TOP of midiControlChange — the single
// place every wire CC passes through (USB device/host/DIN/this link). Pure
// observation: no dispatch behavior change.
void ui_link_observe_cc(uint8_t cc, uint8_t val) {
  const uint8_t mod = g_active_module;
  g_ui_cc_cache[mod][cc] = val;
  // Throttled delta after control_change on the active module (>= 50 ms
  // spacing). Chain-B CCs (23/24/65/66 + the clouds grain bank while
  // chained) are B-slot writes — not cells of the A slot's page, skip.
  if (rack_chain_wants_cc(cc)) return;
  if (g_ui_active_page > kUiPageP2) return;     // MOD/SYS pages: no CC cells (stage 1)
  static uint32_t last_delta_ms = 0;
  const uint32_t now = millis();
  if (now - last_delta_ms < 50) return;
  const UiPageTable& t = kUiPages[mod][g_ui_active_page];
  for (uint8_t i = 0; i < t.count; ++i) {
    if (t.cells[i].cc == cc) {
      last_delta_ms = now;
      ui_link_send_telem(g_ui_active_page, kUiTelemDelta, cc);
      return;
    }
  }
}

// ===================== RX parser (state machine) =====================
enum UiRxState : uint8_t { UI_RX_HUNT = 0, UI_RX_LEN, UI_RX_BODY, UI_RX_CRC };
static uint8_t g_ui_rx_state = UI_RX_HUNT;
static uint8_t g_ui_rx_len = 0;
// uint16_t: the body of a len=255 frame is type+payload = 256 bytes — a
// uint8_t counter wraps at 255 and the parser stalls mid-frame (bench-caught
// 2026-11-08: uitest cases 4-6 all failed downstream of the wrap).
static uint16_t g_ui_rx_idx = 0;
static uint8_t g_ui_rx_buf[256];   // [0] = type, [1..len] = payload (len <= 255)
static uint8_t g_ui_rx_crc = 0;
static bool g_ui_rx_crc_ok = false;

// One byte into the parser. Returns true when a COMPLETE frame is assembled
// (CRC verified into g_ui_rx_crc_ok; buf = [type][payload...]; state reset to
// HUNT — on CRC failure the parser resyncs at the next 0x7E by design).
static bool ui_link_step(uint8_t b) {
  switch (g_ui_rx_state) {
    case UI_RX_HUNT:
      if (b == kUiFrameMark) g_ui_rx_state = UI_RX_LEN;
      return false;
    case UI_RX_LEN:
      g_ui_rx_len = b;
      g_ui_rx_idx = 0;
      g_ui_rx_state = UI_RX_BODY;
      return false;
    case UI_RX_BODY:
      g_ui_rx_buf[g_ui_rx_idx++] = b;
      if (g_ui_rx_idx >= (uint16_t)g_ui_rx_len + 1) g_ui_rx_state = UI_RX_CRC;
      return false;
    default:  // UI_RX_CRC
      g_ui_rx_crc = b;
      g_ui_rx_crc_ok = (ui_crc8(g_ui_rx_buf, (size_t)g_ui_rx_len + 1) == b);
      if (!g_ui_rx_crc_ok) ++g_ui_crc_errors;
      g_ui_rx_state = UI_RX_HUNT;
      return true;
  }
}

// ===================== RX dispatch =====================
// Declared later in main.cpp (the data-driven console table's forward-decl
// block): the SWITCH / CHAIN_ROUTE frames reuse the console paths verbatim
// — one behavior, two entrances.
static void ui_link_dispatch() {
  const uint8_t type = g_ui_rx_buf[0];
  const uint8_t* pl = &g_ui_rx_buf[1];
  switch (type) {
    case kUiTypeMidi:
      // 4-byte USB-MIDI event packet -> THE funnel (same as USB device/host/
      // DIN). Zero new dispatch semantics.
      if (g_ui_rx_len == 4) {
        midiHandleEventPacket(pl);
      } else {
        Serial.printf("ui: MIDI frame bad len %u (want 4) - dropped\r\n",
                      g_ui_rx_len);
      }
      break;
    case kUiTypeSwitch: {
      // Module id -> the existing pending-swap path EXACTLY like the console
      // `switch` command (acks, chain semantics, self-swap guard included).
      if (g_ui_rx_len == 1 && pl[0] < RACK_MODULE_COUNT) {
        char id[2] = { (char)('0' + pl[0]), '\0' };
        cmd_switch(id);
      } else {
        Serial.printf("ui: SWITCH frame bad (len=%u id=%u)\r\n",
                      g_ui_rx_len, g_ui_rx_len ? pl[0] : 0);
      }
      break;
    }
    case kUiTypeModSet:
      // [src][param][value] -> the mod engine's bench setters with the same
      // validation/quantization as the console `mod` command (see
      // mod_engine_apply_ui_set in mod_engine.h).
      if (g_ui_rx_len == 3) {
        mod_engine_apply_ui_set(pl[0], pl[1], pl[2]);
      } else {
        Serial.printf("ui: MOD_SET frame bad len %u (want 3)\r\n", g_ui_rx_len);
      }
      break;
    case kUiTypeUiCmd: {
      if (g_ui_rx_len != 3) {
        Serial.printf("ui: UI_CMD frame bad len %u (want 3)\r\n", g_ui_rx_len);
        break;
      }
      const uint8_t action = pl[0], arg = pl[1], arg2 = pl[2];
      if (action == kUiCmdPageRequest) {
        if (arg < kUiPageCount) {
          g_ui_active_page = arg;
          ui_link_send_telem(arg, kUiTelemFull, 0);   // full snapshot reply
        } else {
          Serial.printf("ui: page request %u out of range (0-%u)\r\n",
                        arg, kUiPageCount - 1);
        }
      } else if (action == kUiCmdEcho) {
        // Echo the frame back verbatim + record the turnaround time (rx
        // complete -> echo written). The C3 measures the full wire RTT; the
        // RP2350-side number is its echo turnaround.
        const uint32_t t0 = time_us_32();
        const uint8_t p[3] = { kUiCmdEcho, arg, arg2 };
        ui_link_send_frame(kUiTypeUiCmd, p, 3);
        g_ui_last_rtt_us = time_us_32() - t0;
      } else if (action == kUiCmdSysToggle) {
        // Log-only in stage 1 (SYS toggles are wired to real state in a
        // later stage).
        Serial.printf("ui: SYS toggle arg=%u arg2=%u (log-only in stage 1)\r\n",
                      arg, arg2);
      } else {
        Serial.printf("ui: UI_CMD unknown action %u\r\n", action);
      }
      break;
    }
    case kUiTypeChainRoute: {
      // [a][b][engage] -> the existing console chain path (validations,
      // window math, acks, queueing — one behavior, two entrances).
      if (g_ui_rx_len == 3) {
        char args[16];
        if (pl[2] == 1) {
          if (pl[0] < RACK_MODULE_COUNT && pl[1] < RACK_MODULE_COUNT) {
            snprintf(args, sizeof(args), "%u %u", pl[0], pl[1]);
            cmd_chain(args);
          } else {
            Serial.println("ui: CHAIN_ROUTE module ids out of range");
          }
        } else if (pl[2] == 0) {
          cmd_chain("off");
        } else {
          Serial.printf("ui: CHAIN_ROUTE engage=%u (want 0|1)\r\n", pl[2]);
        }
      } else {
        Serial.printf("ui: CHAIN_ROUTE frame bad len %u (want 3)\r\n",
                      g_ui_rx_len);
      }
      break;
    }
    case kUiTypeTelem:
      Serial.printf("ui: TELEM received (len=%u) - RP2350->C3 only, ignored\r\n",
                    g_ui_rx_len);
      break;
    case kUiTypePresetReq:
    case kUiTypePresetPart:
    case kUiTypePresetApply:
      // Reserved for T4a (preset bank on the C3): receive-and-log.
      Serial.printf("ui: PRESET frame type 0x%02x len=%u - reserved (T4a), "
                    "logged only\r\n", type, g_ui_rx_len);
      break;
    default:
      Serial.printf("ui: unknown frame type 0x%02x len=%u\r\n", type,
                    g_ui_rx_len);
      break;
  }
}

// One byte from the UART -> parser -> dispatch (on a verified frame).
static void ui_link_feed(uint8_t b) {
  if (!ui_link_step(b)) return;
  if (!g_ui_rx_crc_ok) {
    Serial.printf("ui: CRC FAIL (type=0x%02x len=%u) - frame dropped, "
                  "resyncing at the next 0x7E\r\n",
                  g_ui_rx_buf[0], g_ui_rx_len);
    return;
  }
  ++g_ui_rx_frames;
  ui_link_dispatch();
}

// Loop-side pump (core 0, next to the DIN pump): drain the IRQ-fed SW ring.
// With the link absent this costs one ring-empty check per loop().
static void ui_link_pump() {
  if (Serial1.overflow()) ++g_ui_rx_overflow;   // SW ring overflow (IRQ side)
  while (Serial1.available() > 0) {
    const int c = Serial1.read();
    if (c < 0) break;
    ui_link_feed((uint8_t)c);
  }
}
