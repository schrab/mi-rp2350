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
// 2 = echo -> echo back + RTT, 3 = sys toggle -> arg = item id (0 telemetry
// deltas gate, 1 full-snapshot gate, 2 global clock source toggle; semantics
// = TOGGLE, arg2 reserved 0), 4 = ASSIGN_CYCLE [cell 0-15][dir 1 up/0 down]
// -> cycles that ACTIVE-page cell's mod source through [off, src0..src11]
// via mod_set_dest (the RP2350 owns routing + validation; invalid targets
// are inert and show as TELEM flags bit2) and replies a full TELEM snapshot),
// 0x05 CHAIN_ROUTE ([a][b][engage] -> the console `chain` path). Types
// 0x11/0x12/0x13 (PRESET_REQ/PART/APPLY) are RESERVED for T4a — PRESET_REQ
// carries [slot:u8] for the T4a bank, all are received-and-logged.
// RP2350 -> C3: 0x10 TELEM.
//
// UART choice: Serial1 = HW UART0, TX = GP0, RX = GP1 (2026-11-08 user
// call: the RP2350 nano board only populates GPIO 0-5 + 26-29; GP16/17 —
// the first pick after correcting the brief's TX=17/RX=16 pad swap — are
// not on the header). Validity per the earlephilhower core's RP2350A
// SerialUART tables: TX = {0,2,12,14,16,18,28}, RX = {1,3,13,15,17,19,29};
// from the populated set GP0/GP1 is the clean pair (GP2/3 also valid;
// GP28/29 kept free as the only ADC-capable pins). Free pins: 4/5 DIN,
// 6/7/8 I2S, 14/15 PIO-USB host, 22 LED, 24/25 native USB. Hardware UART
// claims no
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
// duplicate of P1 (page id differs). MOD-A/B/C: real cells from the mod
// engine (stage 4) — the C3's pot turns on these pages arrive as MOD_SET
// [src][param][value] with the SAME (src, param) map as the tables below
// (docs/ui-protocol.md §6). SYS: 8 cells (4 modules, chain pair/engage,
// telem gates); the C3's preset slot is C3-local (T4a).
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
static const uint8_t kUiCmdSysToggle   = 3;      // arg = sys item id (below)
static const uint8_t kUiCmdAssignCycle = 4;      // arg = cell 0-15, arg2 = dir
static const uint8_t kUiCmdSetGlobal   = 5;      // arg = global item, arg2 = val

// Sys toggle item ids (UI_CMD action 3; semantics = TOGGLE, arg2 reserved 0).
static const uint8_t kUiSysTelemDelta  = 0;  // throttled CC-delta stream gate
static const uint8_t kUiSysTelemFull   = 1;  // RP2350-pushed full snapshots gate
static const uint8_t kUiSysClockToggle = 2;  // global clock internal <-> midi

// SET_GLOBAL item ids (UI_CMD action 5, [item][value 0-127]).
static const uint8_t kUiGlobalBpm = 0;       // value -> 20 + v*280/127 BPM

// Page ids.
static const uint8_t kUiPageP1        = 0;
static const uint8_t kUiPageP2        = 1;
static const uint8_t kUiPageModLfo12  = 2;
static const uint8_t kUiPageModLfo34  = 3;
static const uint8_t kUiPageModTrig   = 4;
static const uint8_t kUiPageModGate   = 5;
static const uint8_t kUiPageModEnv    = 6;
static const uint8_t kUiPageSys       = 7;
// Chain-B panel (T5 UX): while a chain is engaged the C3 gains a page
// showing the B slot's (clouds) parameters — the cells are the CLOUDS P1
// registry table (the alias bank CCs 40-48 + 23/24/65/66 are exactly the
// B-directed CCs the chain dispatcher accepts while engaged).
static const uint8_t kUiPageChainB    = 8;
static const uint8_t kUiPageCount     = 9;

// TELEM modes: 0 = full snapshot, 1 = single-cell delta (same format).
static const uint8_t kUiTelemFull  = 0;
static const uint8_t kUiTelemDelta = 1;

// TELEM cell flags.
static const uint8_t kUiFlagModRouted = 1 << 0;  // a mod source drives this CC
static const uint8_t kUiFlagChainB    = 1 << 1;  // cell belongs to the chain B slot
static const uint8_t kUiFlagInert     = 1 << 2;  // routing exists but is inert/rejected
// Flags bits 3-6 = the FIRST mod source routed to this CC, +1 (0 = unrouted;
// 1-10 = source 0-9). The mod-assign gesture needs to know WHICH source
// drives the cell; the summary bit0 stays "any route exists" (multiple
// sources may share a destination — the cycle tracks the first).
static const uint8_t kUiFlagSrcShift  = 3;
static const uint8_t kUiFlagSrcMask   = 0x0F << kUiFlagSrcShift;  // bit7 reserved 0

// MOD/SYS TELEM cell ids: the `cc` field of MOD-A/B/C and SYS cells is an
// OPAQUE cell id = kUiModCellBase + cell index (they are not MIDI CCs). The
// C3 uses the CELL INDEX for rendering and MOD_SET addressing; the cc byte
// is informational (and lets a delta name its cell unambiguously).
static const uint8_t kUiModCellBase = 200;

// Param color classes (white, pink, teal, yellow).
static const uint8_t kUiColorWhite  = 0;
static const uint8_t kUiColorPink   = 1;
static const uint8_t kUiColorRed    = 1;  // backward-compat alias
static const uint8_t kUiColorTeal   = 2;
static const uint8_t kUiColorYellow = 3;

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

// elements — P1: physical front-panel order (Row 0: exciter levels + pitch/strength/contour;
// Row 1: flow/mallet + geometry/brightness; Row 2: timbres + damping/position/space).
static const UiPageCell kUiPageElementsP1[] = {
    // Row 0 (pots 0..5):
    {15, "BOW",        kUiColorWhite},
    {17, "BLOW",       kUiColorPink},
    {20, "STRIKE",     kUiColorTeal},
    {1,  "PITCH",      kUiColorWhite},
    {11, "STRENGTH",   kUiColorWhite},
    {14, "CONTOUR",    kUiColorWhite},
    // Row 1 (pots 6..9):
    {18, "FLOW",       kUiColorPink},
    {21, "MALLET",     kUiColorTeal},
    {23, "GEOMETRY",   kUiColorWhite},
    {24, "BRIGHTNESS", kUiColorWhite},
    // Row 2 (pots 10..15):
    {16, "TIMBRE",     kUiColorWhite},
    {19, "TIMBRE",     kUiColorPink},
    {22, "TIMBRE",     kUiColorTeal},
    {25, "DAMPING",    kUiColorWhite},
    {26, "POSITION",   kUiColorWhite},
    {27, "SPACE",      kUiColorWhite},
};
// elements — P2: selectors (CC 28) + PLAY trigger (CC 64) + VCA (CC 7).
static const UiPageCell kUiPageElementsP2[] = {
    {28, "MODEL",      kUiColorWhite},
    {64, "PLAY",       kUiColorWhite},
    {7,  "VCA",        kUiColorWhite},
};
// clouds — P1: SIZE/TEXTURE teal, POSITION/DENSITY pink, rest white.
static const UiPageCell kUiPageCloudsP1[] = {
    {1,  "EX TONE",    kUiColorWhite},
    {11, "EX LEVEL",   kUiColorWhite},
    {13, "EX SRC",     kUiColorWhite},
    {40, "POSITION",   kUiColorPink},
    {41, "SIZE",       kUiColorTeal},
    {42, "PITCH",      kUiColorWhite},
    {43, "DENSITY",    kUiColorPink},
    {44, "TEXTURE",    kUiColorTeal},
    {45, "DRY/WET",    kUiColorWhite},
    {46, "SPREAD",     kUiColorWhite},
    {47, "FEEDBACK",   kUiColorWhite},
    {48, "REVERB",     kUiColorWhite},
    {23, "MODE",       kUiColorWhite},
    {24, "QUALITY",    kUiColorWhite},
    {65, "FREEZE",     kUiColorWhite},
    {66, "TRIG",       kUiColorWhite},
};
// plaits — P1: 11 cells.
static const UiPageCell kUiPagePlaitsP1[] = {
    {1,  "PITCH",      kUiColorWhite},
    {14, "TIMBRE",     kUiColorWhite},
    {15, "MORPH",      kUiColorWhite},
    {16, "HARMONICS",  kUiColorWhite},
    {17, "MACRO",      kUiColorWhite},
    {18, "LPG",        kUiColorWhite},
    {19, "DECAY",      kUiColorWhite},
    {25, "LEVEL",      kUiColorWhite},
    {23, "ENGINE",     kUiColorWhite},
    {24, "AUX",        kUiColorWhite},
    {64, "TRIG",       kUiColorWhite},
};
// rings — P1: panel colors (all white). CC 1 = PITCH (FREQUENCY), CC 64 = STRUM pseudo-CC. 13 cells.
static const UiPageCell kUiPageRingsP1[] = {
    {1,  "PITCH",      kUiColorWhite},
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
    {7,  "VCA",        kUiColorWhite},
};

// [module][0 = P1, 1 = P2]. Elements has 3 unique params on P2 (MODEL CC 28,
// PLAY CC 64, VCA CC 7). Clouds, Plaits, and Rings have all parameters on P1, so P2
// has count = 0 (no duplicate params).
static const UiPageTable kUiPages[RACK_MODULE_COUNT][2] = {
    { {kUiPageElementsP1, 16}, {kUiPageElementsP2, 3} },   // elements
    { {kUiPageCloudsP1, 16},   {nullptr, 0} },             // clouds
    { {kUiPagePlaitsP1, 11},   {nullptr, 0} },             // plaits
    { {kUiPageRingsP1, 13},    {nullptr, 0} },             // rings
};
static const UiPageTable kUiPageEmpty = { nullptr, 0 };   // unused fallback

// ===================== Dedicated Modulation & SYS aux cells =====================
// Cell order = physical pot order (the C3's MOD-page pot map sends MOD_SET
// with exactly these (src, param) pairs — MUST MATCH the C3's kModPotMap).
// `cc` in TELEM = kUiModCellBase + index. Special src 0xFF = a SYS-style
// selector in `param` (values read from chain/telem/clock state, not the
// mod engine): 0-3 module-id cells (value 0; the active module is the TELEM
// header), 4 chain pair index, 5 chain engaged, 6 telem-delta gate,
// 7 telem-full gate, 8 BPM pot, 9 global clock source.
struct UiAuxCell {
  const char* label;
  uint8_t color;      // kUiColor*
  uint8_t src;        // mod source 0-9, or 0xFF = selector (see above)
  uint8_t param;      // kModParamNames index, or the selector id
};
static const uint8_t kUiAuxSelModule    = 0;   // param selectors for src 0xFF
static const uint8_t kUiAuxSelChainPair = 4;
static const uint8_t kUiAuxSelChainOn   = 5;
static const uint8_t kUiAuxSelBpm       = 6;
static const uint8_t kUiAuxSelClkSrc    = 7;
static const uint8_t kUiAuxSelTelemD    = 14;
static const uint8_t kUiAuxSelTelemF    = 15;

// MOD-LFO 1/2: 2x LFO with full 6 params (Row 0 = LFO 1, Row 2 = LFO 2)
static const UiAuxCell kUiModLfo12Cells[16] = {
    // Row 0: LFO 1 (pots 0..5)
    {"L1 RATE",  kUiColorYellow, 0, 1},
    {"L1 WAVE",  kUiColorYellow, 0, 2},
    {"L1 PHASE", kUiColorYellow, 0, 3},
    {"L1 LEV",   kUiColorYellow, 0, 4},
    {"L1 MODE",  kUiColorYellow, 0, 0},
    {"L1 DIV",   kUiColorYellow, 0, 6},
    // Row 1: empty (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: LFO 2 (pots 10..15)
    {"L2 RATE",  kUiColorYellow, 1, 1},
    {"L2 WAVE",  kUiColorYellow, 1, 2},
    {"L2 PHASE", kUiColorYellow, 1, 3},
    {"L2 LEV",   kUiColorYellow, 1, 4},
    {"L2 MODE",  kUiColorYellow, 1, 0},
    {"L2 DIV",   kUiColorYellow, 1, 6},
};

// MOD-LFO 3/4: 2x LFO with full 6 params (Row 0 = LFO 3, Row 2 = LFO 4)
static const UiAuxCell kUiModLfo34Cells[16] = {
    // Row 0: LFO 3 (pots 0..5)
    {"L3 RATE",  kUiColorYellow, 2, 1},
    {"L3 WAVE",  kUiColorYellow, 2, 2},
    {"L3 PHASE", kUiColorYellow, 2, 3},
    {"L3 LEV",   kUiColorYellow, 2, 4},
    {"L3 MODE",  kUiColorYellow, 2, 0},
    {"L3 DIV",   kUiColorYellow, 2, 6},
    // Row 1: empty (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: LFO 4 (pots 10..15)
    {"L4 RATE",  kUiColorYellow, 3, 1},
    {"L4 WAVE",  kUiColorYellow, 3, 2},
    {"L4 PHASE", kUiColorYellow, 3, 3},
    {"L4 LEV",   kUiColorYellow, 3, 4},
    {"L4 MODE",  kUiColorYellow, 3, 0},
    {"L4 DIV",   kUiColorYellow, 3, 6},
};

// MOD-TRIG: 2x Euclidean (Row 0 = EUC 1, Row 2 = EUC 2)
static const UiAuxCell kUiModTrigCells[16] = {
    // Row 0: EUCLID 1 (pots 0..4)
    {"E1 STPS", kUiColorTeal, 4, 0},
    {"E1 PULS", kUiColorTeal, 4, 1},
    {"E1 ROT",  kUiColorTeal, 4, 2},
    {"E1 CLK",  kUiColorTeal, 4, 3},
    {"E1 DIV",  kUiColorTeal, 4, 4},
    {"",        kUiColorWhite, 0xFF, 0xFF},
    // Row 1: empty (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: EUCLID 2 (pots 10..14)
    {"E2 STPS", kUiColorTeal, 5, 0},
    {"E2 PULS", kUiColorTeal, 5, 1},
    {"E2 ROT",  kUiColorTeal, 5, 2},
    {"E2 CLK",  kUiColorTeal, 5, 3},
    {"E2 DIV",  kUiColorTeal, 5, 4},
    {"",        kUiColorWhite, 0xFF, 0xFF},
};

// MOD-GATE: 2x Gate pattern generators (Row 0 = GATE 1, Row 2 = GATE 2)
static const UiAuxCell kUiModGateCells[16] = {
    // Row 0: GATE 1 (pots 0..5)
    {"G1 MODE", kUiColorTeal, 6, 0},
    {"G1 DENS", kUiColorTeal, 6, 1},
    {"G1 SYNC", kUiColorTeal, 6, 2},
    {"G1 GLEN", kUiColorTeal, 6, 3},
    {"G1 CLK",  kUiColorTeal, 6, 4},
    {"G1 DIV",  kUiColorTeal, 6, 5},
    // Row 1: empty / spacer (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: GATE 2 (pots 10..15)
    {"G2 MODE", kUiColorTeal, 7, 0},
    {"G2 DENS", kUiColorTeal, 7, 1},
    {"G2 SYNC", kUiColorTeal, 7, 2},
    {"G2 GLEN", kUiColorTeal, 7, 3},
    {"G2 CLK",  kUiColorTeal, 7, 4},
    {"G2 DIV",  kUiColorTeal, 7, 5},
};

// MOD-ENV: 2x ADSR envelopes (Row 0 = ADSR 1, Row 2 = ADSR 2)
static const UiAuxCell kUiModEnvCells[16] = {
    // Row 0: ADSR 1 (pots 0..5)
    {"A1 ATK",  kUiColorTeal, 8, 1},
    {"A1 DEC",  kUiColorTeal, 8, 2},
    {"A1 SUS",  kUiColorTeal, 8, 3},
    {"A1 REL",  kUiColorTeal, 8, 4},
    {"A1 MODE", kUiColorTeal, 8, 0},
    {"A1 DIV",  kUiColorTeal, 8, 6},
    // Row 1: empty (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: ADSR 2 (pots 10..15)
    {"A2 ATK",  kUiColorTeal, 9, 1},
    {"A2 DEC",  kUiColorTeal, 9, 2},
    {"A2 SUS",  kUiColorTeal, 9, 3},
    {"A2 REL",  kUiColorTeal, 9, 4},
    {"A2 MODE", kUiColorTeal, 9, 0},
    {"A2 DIV",  kUiColorTeal, 9, 6},
};

// SYS: ONE module selector pot (dual-purpose: module select when no chain,
// chain-pair select while engaged — the C3 renders text names), chain
// engage, master BPM pot, master clock source toggle, preset slot,
// telemetry gates. Cells 1-4 are blank (the old 4-pot module radio and the
// separate CHAIN PR pot are folded into cell 0).
static const UiAuxCell kUiSysCells[16] = {
    {"MODULE",   kUiColorWhite, 0xFF, kUiAuxSelModule},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"CHAIN ON", kUiColorWhite, 0xFF, kUiAuxSelChainOn},
    {"BPM",      kUiColorWhite, 0xFF, kUiAuxSelBpm},
    {"CLKSRC",   kUiColorWhite, 0xFF, kUiAuxSelClkSrc},
    {"PRESET",   kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"",         kUiColorWhite, 0xFF, 0xFF},
    {"TELEM D",  kUiColorWhite, 0xFF, kUiAuxSelTelemD},
    {"TELEM F",  kUiColorWhite, 0xFF, kUiAuxSelTelemF},
};

// ===================== telemetry gates (sys toggles 0/1) =====================
// Defaults ON (stage-1 behavior preserved). Toggled by UI_CMD action 3.
static uint8_t g_ui_telem_deltas_en = 1;   // throttled CC-delta stream
static uint8_t g_ui_telem_full_en = 1;     // RP2350-pushed full snapshots
                                           // (module/chain change events)
// Last verified RX frame (for the status tail's link bit) — core 0 only.
static uint32_t g_ui_last_rx_ms = 0;

// ===================== CC-value cache (TELEM source) =====================
// Observed at the single dispatch point (midiControlChange -> the hook below).
// 4 modules x 128 CCs = 512 B .bss. The full-CC-range span was chosen over a
// remapped 32-CC window to cover CC 64/65/66 (trig/strum/freeze) and the
// clouds alias bank 40-48 without index arithmetic.
static uint8_t g_ui_cc_cache[RACK_MODULE_COUNT][128];

// T4a "seen" bitmap: which CCs were actually RECEIVED per module (cache
// zeros are ambiguous — "never sent" vs "explicitly 0"). 8 u16 x 4 = 64 B.
// preset replay/restore only touches seen CCs so factory defaults survive
// for parameters no controller has moved yet. Seeded with the boot defaults
// below (VCA/PITCH/plaits level), filled at the single observation point.
static uint16_t g_ui_cc_seen[RACK_MODULE_COUNT][8];

// T4a autosave dirty flag: any persistent state change (CC, mod param/dest,
// module switch, chain, bpm/clock) bumps this; core 0's loop pushes the
// serialized blob to the C3 flash bank once the state has settled (3 s).
static volatile uint32_t g_preset_dirty_ms = 0;
static volatile bool g_preset_dirty = false;
void preset_mark_dirty() {
  g_preset_dirty = true;
  g_preset_dirty_ms = millis();
}

// T4a handlers live in preset.h (included by main.cpp after this header —
// same single-TU discipline as midi.h; the definitions below see ui_link's
// statics, preset.h sees both).
void ui_preset_req_reply(uint8_t slot);              // PRESET_REQ -> PART burst
void ui_preset_apply_chunk(const uint8_t* pl, uint8_t len);  // APPLY reassembly
void preset_autosave_poll();                         // core-0 loop debounce push

// Restore cached parameter value to module (used by mod_set_dest on unassign)
void ui_link_restore_cached_cc(uint8_t mod, uint8_t cc) {
  if (mod < RACK_MODULE_COUNT && cc < 128) {
    kRackModules[mod].control_change(cc, g_ui_cc_cache[mod][cc]);
  }
}

// Last page the C3 requested (delta policy target; default P1).
static uint8_t g_ui_active_page = kUiPageP1;

// Link stats (for `ui`; counters are core-0-only, no cross-core races).
static volatile uint32_t g_ui_tx_frames = 0;
static volatile uint32_t g_ui_rx_frames = 0;
static volatile uint32_t g_ui_crc_errors = 0;
static volatile uint32_t g_ui_rx_overflow = 0;
static volatile uint32_t g_ui_last_rtt_us = 0;

// ===================== UART (Serial1 = HW UART0) =====================
// 2026-11-08 user call: the RP2350 nano board only POPULATES GPIO 0-5 +
// 26-29 — GP16/17 (the first choice) don't exist on the header. From the
// populated set the valid UART0 pads are TX {0,2,28} / RX {1,3,29}; GP0/GP1
// is the clean adjacent pair (GP2/3 also valid; GP28/29 kept free as the
// only ADC-capable pins). GP0/1 are unused by the rack (DIN 4/5, I2S 6/7/8,
// host 14/15, LED 22).
static const uint8_t kPinUiTx = 0;    // UART0 TX — valid {0,2,12,14,16,18,28}
static const uint8_t kPinUiRx = 1;    // UART0 RX — valid {1,3,13,15,17,19,29}

static void ui_link_uart_init() {
  // ORDER (all three before begin, same as the DIN init): FIFO size + pinout
  // apply only while _running == false. 512 B SW ring: at 921600 baud max
  // (~92 KB/s) it rides out ~5.6 ms between drains — far beyond any loop()
  // stall (swap prints, LED poll ~1 ms, one TELEM write ~2.6 ms).
  Serial1.setFIFOSize(512);
  Serial1.setPinout(kPinUiTx, kPinUiRx);
  Serial1.begin(921600, SERIAL_8N1);
  for (uint8_t m = 0; m < RACK_MODULE_COUNT; ++m) {
    g_ui_cc_cache[m][7] = 127;   // VCA default full open (1.0)
    g_ui_cc_cache[m][1] = 64;    // PITCH default center (0 st)
    g_ui_cc_seen[m][0] = 0x0083; // seen bits: CC 1 (0x0002) + CC 7 (0x0080)
  }
  g_ui_cc_cache[RACK_MODULE_PLAITS][25] = 102; // Plaits default shared_level = 0.8 (~102/127)
  g_ui_cc_seen[RACK_MODULE_PLAITS][1] = 0x0200;  // seen bit: CC 25 (25-16=9)
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

// TELEM flags for one cell (see kUiFlag*). Bits 3-6 carry the FIRST routed
// source id + 1 so the C3's assign gesture knows what it is cycling from.
static uint8_t ui_link_cell_flags(uint8_t cc, uint8_t mod) {
  uint8_t flags = 0;
  bool routed = false, inert = false;
  int8_t first_src = -1;
  const RackModDestinations* d = kRackModules[mod].mod_destinations;
  for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
    const ModSource& s = g_mod_engine.src[i];
    if (s.dest_cc != cc) continue;
    if (first_src < 0) first_src = (int8_t)i;
    routed = true;
    const bool can_continuous = (s.type == MOD_LFO || s.type == MOD_ADSR ||
                                s.type == MOD_GATE || s.type == MOD_EUCLID);
    const bool can_trigger = (s.type == MOD_EUCLID || s.type == MOD_GATE);
    bool ok = false;
    if (can_continuous) {
      for (uint8_t k = 0; k < d->continuous_count; ++k) {
        if (d->continuous[k] == cc) { ok = true; break; }
      }
    }
    if (!ok && can_trigger) {
      for (uint8_t k = 0; k < d->trigger_count; ++k) {
        if (d->trigger_cc[k] == cc) { ok = true; break; }
      }
    }
    if (!ok) inert = true;
  }
  if (routed) flags |= kUiFlagModRouted;
  if (first_src >= 0) {
    flags |= (uint8_t)((first_src + 1) << kUiFlagSrcShift) & kUiFlagSrcMask;
  }
  if (g_chain.active && mod == g_chain.b) flags |= kUiFlagChainB;
  if (inert) flags |= kUiFlagInert;
  return flags;
}

// ===================== aux cell value readers =====================
// Inverse of mod_apply_cc_value's quantization (mod_engine.h) — a MOD_SET
// round-trip of a displayed value is stable. Rounding is nearest; the
// quantizer truncates, so re-applying a round-tripped value can land one
// count below the exact float (documented; imperceptible on pots).

// log-domain helpers without libm round-trips.
static uint8_t ui_mod_lfo_rate_val(float hz) {   // 0.02 Hz .. 20 Hz exp
  if (hz < 0.02f) hz = 0.02f;
  if (hz > 20.0f) hz = 20.0f;
  // v = 127 * log(hz/0.02)/log(1000), log(1000) = 3*ln10
  float v = 127.0f * logf(hz / 0.02f) / (3.0f * logf(10.0f));
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  return (uint8_t)(v + 0.5f);
}
static uint8_t ui_mod_adsr_s_val(float s) {      // 1 ms .. 10 s exponential
  if (s < 1e-3f) s = 1e-3f;
  if (s > 10.0f) s = 10.0f;
  // mod_adsr_s_from_cc: s = 10^(-3 + 4v/127) -> v = 127*(log10(s)+3)/4
  float v = 127.0f * (log10f(s) + 3.0f) / 4.0f;
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  return (uint8_t)(v + 0.5f);
}
// ceil(a/b) for positive ints — inverts the quantizer's floor().
static inline uint8_t ui_ceil_div(uint16_t a, uint16_t b) {
  return (uint8_t)((a + b - 1) / b);
}

// Value byte for one aux cell (0-127). The C3 renders the number; it never
// interprets beyond the documented per-cell semantics (docs §6).
static uint8_t ui_aux_cell_value(const UiAuxCell& c) {
  if (c.src != 0xFF) {
    const ModSource& s = g_mod_engine.src[c.src % MOD_SOURCE_COUNT];
    switch (s.type) {
      case MOD_LFO:
        switch (c.param) {
          case 0: return s.p.lfo.hz_mode ? 0 : 127;
          case 1: return ui_mod_lfo_rate_val(s.p.lfo.rate_hz);
          case 2: return ui_ceil_div((uint16_t)s.p.lfo.wave * 128u, 5u);
          case 3: return (uint8_t)(s.p.lfo.phase_off * 127.0f + 0.5f);
          case 4: return (uint8_t)(s.p.lfo.level * 127.0f + 0.5f);
          case 5: return s.clock_sel == MOD_CLK_GLOBAL ? 0 : (s.clock_sel == MOD_CLK_INTERNAL ? 64 : 127);
          case 6: return (uint8_t)(((uint16_t)s.division * 127) / (MOD_DIVISION_COUNT - 1));
          default: return 0;
        }
      case MOD_EUCLID:
        switch (c.param) {
          case 0: return (uint8_t)(((uint16_t)(s.p.eu.steps - 1) * 127) / 31);
          case 1: return (uint8_t)(((uint16_t)s.p.eu.pulses * 127) / 32);
          case 2: return (uint8_t)(((uint16_t)s.p.eu.rot * 127) / 31);
          case 3: return s.clock_sel == MOD_CLK_GLOBAL ? 0 : (s.clock_sel == MOD_CLK_INTERNAL ? 64 : 127);
          case 4: return (uint8_t)(((uint16_t)s.division * 127) / (MOD_DIVISION_COUNT - 1));
          default: return 0;
        }
      case MOD_GATE:
        switch (c.param) {
          case 0: return (uint8_t)(((uint16_t)s.p.gate.mode * 127) / 18);
          case 1: return (uint8_t)(((uint16_t)(s.p.gate.density - 1) * 127) / 15);
          case 2: return (uint8_t)(((uint16_t)s.p.gate.syncopation * 127) / 255);
          case 3: {
            int v = (int)(s.p.gate.gate_len * 128.0f) - 1;
            if (v < 0) v = 0;
            if (v > 127) v = 127;
            return (uint8_t)v;
          }
          case 4: return s.clock_sel == MOD_CLK_GLOBAL ? 0 : (s.clock_sel == MOD_CLK_INTERNAL ? 64 : 127);
          case 5: return (uint8_t)(((uint16_t)s.division * 127) / (MOD_DIVISION_COUNT - 1));
          default: return 0;
        }
      default:  // MOD_ADSR
        switch (c.param) {
          case 0: return s.p.adsr.retrig ? 127 : 0;
          case 1: return ui_mod_adsr_s_val(s.p.adsr.a_s);
          case 2: return ui_mod_adsr_s_val(s.p.adsr.d_s);
          case 3: return (uint8_t)(s.p.adsr.s_lvl * 127.0f + 0.5f);
          case 4: return ui_mod_adsr_s_val(s.p.adsr.r_s);
          case 5: return s.clock_sel == MOD_CLK_GLOBAL ? 0 : (s.clock_sel == MOD_CLK_INTERNAL ? 64 : 127);
          case 6: return (uint8_t)(((uint16_t)s.division * 127) / (MOD_DIVISION_COUNT - 1));
          default: return 0;
        }
    }
  }
  // SYS-style selectors (src 0xFF).
  switch (c.param) {
    case kUiAuxSelModule:
      // ONE selector pot, dual-purpose: not chained = the ACTIVE module
      // over 4 slots (0/32/64/96); chained = the engaged PAIR over 3
      // slots (0/64/127 — the C3 renders E>C / P>C / R>C from the chain
      // tail and picks the pair with the same pot).
      if (g_chain.active) {
        if (g_chain.a == RACK_MODULE_PLAITS) return 64;
        if (g_chain.a == RACK_MODULE_RINGS) return 127;
        return 0;
      }
      return (uint8_t)(g_active_module * 32);
    case kUiAuxSelChainPair:
      // Configured pair index (1 elements->clouds / 2 plaits->clouds /
      // 3 rings->clouds) — shown even while disengaged so the C3 knows what
      // the next engage will do.
      if (g_chain.a == RACK_MODULE_ELEMENTS) return 1;
      if (g_chain.a == RACK_MODULE_PLAITS) return 2;
      if (g_chain.a == RACK_MODULE_RINGS) return 3;
      return 0;
    case kUiAuxSelChainOn: return g_chain.active ? 127 : 0;
    case kUiAuxSelTelemD: return g_ui_telem_deltas_en ? 127 : 0;
    case kUiAuxSelTelemF: return g_ui_telem_full_en ? 127 : 0;
    case kUiAuxSelBpm: {
      // 20-300 BPM -> 0-127.
      float b = g_mod_engine.bpm;
      if (b < 20.0f) b = 20.0f;
      if (b > 300.0f) b = 300.0f;
      return (uint8_t)((b - 20.0f) * 127.0f / 280.0f + 0.5f);
    }
    case kUiAuxSelClkSrc: return g_mod_engine.global_clock ? 127 : 0;
    default: return 0;   // module-id cells: value 0 (active = TELEM header)
  }
}

// Status tail (full snapshots, mode 0 only): [bpm][clocksrc][chain][link].
static void ui_link_status_tail(uint8_t* out) {
  // bpm: internal BPM 20-300 rounded to 1 BPM; 0 = external MIDI clock.
  if (g_mod_engine.global_clock == 1) {
    out[0] = 0;
    out[1] = 1;
  } else {
    float b = g_mod_engine.bpm;
    if (b < 20.0f) b = 20.0f;
    if (b > 300.0f) b = 300.0f;
    const uint16_t b_int = (uint16_t)(b + 0.5f);
    out[0] = (uint8_t)(b_int & 0xFF);
    out[1] = (b_int >= 256) ? 0x80 : 0x00;
  }
  // chain: 0 off / 1-3 engaged pair index (elements/plaits/rings -> clouds).
  if (!g_chain.active) {
    out[2] = 0;
  } else if (g_chain.a == RACK_MODULE_ELEMENTS) {
    out[2] = 1;
  } else if (g_chain.a == RACK_MODULE_PLAITS) {
    out[2] = 2;
  } else if (g_chain.a == RACK_MODULE_RINGS) {
    out[2] = 3;
  } else {
    out[2] = 0;                      // not a table pair (cannot happen)
  }
  // link: bit0 = the DIN parser has seen bytes since boot; bit1 = this UART
  // link verified a frame within the last 3 s.
  out[3] = (uint8_t)((din_parser.rx_bytes > 0 ? 0x01 : 0x00) |
                     ((millis() - g_ui_last_rx_ms < 3000) ? 0x02 : 0x00));
}

// Full TELEM snapshot (mode 0) or a 1-cell delta (mode 1, delta_cc = the
// cell id). Format (docs/ui-protocol.md §5 — the C3 parser mirrors it):
//   [page][module][mode][count]
//   [count x (cc:u8, value:u8, flags:u8, color:u8)]      <- cell = 4 bytes
//   [count x NUL-terminated label strings, in cell order]
//   mode 0 only: [bpm:u8][clocksrc:u8][chain:u8][link:u8]  (status tail)
// Color = the registry class byte (0 white / 1 red / 2 teal) — the C3 does
// NOT duplicate the registry. Max payload: 4 + 16*4 + labels + 4 < 255.
static void ui_link_send_telem(uint8_t page, uint8_t mode, uint8_t delta_cc) {
  const uint8_t mod = g_active_module;
  uint8_t pl[256];
  size_t n = 0;

  if (mode == kUiTelemFull) {
    uint8_t count = 0;
    // Cell pass: build the (cc,value,flags,color) quads for the page kind.
    uint8_t cc[16], val[16], flg[16], col[16];
    if (page == kUiPageChainB && g_chain.active) {
      // Chain-B panel: the CLOUDS P1 registry table, values from the
      // CLOUDS cache (B-directed CCs land there via the observe hook
      // while chained). The cells' CCs are exactly what the chain
      // dispatcher accepts for B (23/24/65/66 + the alias bank 40-48),
      // so C3 pot writes on this page just work through that path.
      // Cells 0..2 (EX TONE, EX LEVEL, EX SRC) are blanked while chained
      // because clouds receives module A audio and its exciter is bypassed.
      const UiPageTable& t = kUiPages[RACK_MODULE_CLOUDS][kUiPageP1];
      count = t.count;
      for (uint8_t i = 0; i < count; ++i) {
        if (i < 3) {
          cc[i] = 0;
          val[i] = 0;
          flg[i] = 0;
          col[i] = 0;
        } else {
          cc[i] = t.cells[i].cc;
          val[i] = g_ui_cc_cache[RACK_MODULE_CLOUDS][t.cells[i].cc];
          flg[i] = 0;
          col[i] = t.cells[i].color;
        }
      }
    } else if (page <= kUiPageP2) {
      const UiPageTable& t = kUiPages[mod][page];
      count = t.count;
      for (uint8_t i = 0; i < count; ++i) {
        cc[i] = t.cells[i].cc;
        val[i] = g_ui_cc_cache[mod][t.cells[i].cc];
        flg[i] = ui_link_cell_flags(cc[i], mod);
        col[i] = t.cells[i].color;
      }
    } else {
      const UiAuxCell* a = nullptr;
      if (page == kUiPageModLfo12) { a = kUiModLfo12Cells; count = 16; }
      else if (page == kUiPageModLfo34) { a = kUiModLfo34Cells; count = 16; }
      else if (page == kUiPageModTrig) { a = kUiModTrigCells; count = 16; }
      else if (page == kUiPageModGate) { a = kUiModGateCells; count = 16; }
      else if (page == kUiPageModEnv) { a = kUiModEnvCells; count = 16; }
      else if (page == kUiPageSys) { a = kUiSysCells; count = 16; }
      if (a != nullptr) {
        for (uint8_t i = 0; i < count; ++i) {
          cc[i] = (uint8_t)(kUiModCellBase + i);   // opaque MOD/SYS cell id
          val[i] = ui_aux_cell_value(a[i]);
          flg[i] = 0;   // routing/chain flags are CC-cell concepts
          col[i] = a[i].color;
        }
      }
    }
    pl[n++] = page;
    pl[n++] = mod;
    pl[n++] = mode;
    pl[n++] = count;
    for (uint8_t i = 0; i < count; ++i) {
      pl[n++] = cc[i]; pl[n++] = val[i]; pl[n++] = flg[i]; pl[n++] = col[i];
    }
    // Labels in cell order.
    for (uint8_t i = 0; i < count; ++i) {
      const char* lab = (page == kUiPageChainB)
          ? ((i < 3) ? ""
                     : (i < kUiPages[RACK_MODULE_CLOUDS][kUiPageP1].count
                            ? kUiPages[RACK_MODULE_CLOUDS][kUiPageP1].cells[i].label : ""))
          : (page <= kUiPageP2)
          ? (i < kUiPages[mod][page].count ? kUiPages[mod][page].cells[i].label : "")
          : (page == kUiPageModLfo12 ? kUiModLfo12Cells[i].label
         : page == kUiPageModLfo34 ? kUiModLfo34Cells[i].label
         : page == kUiPageModTrig ? kUiModTrigCells[i].label
         : page == kUiPageModGate ? kUiModGateCells[i].label
         : page == kUiPageModEnv ? kUiModEnvCells[i].label
                                 : kUiSysCells[i].label);
      size_t l = strlen(lab) + 1;                 // include the NUL
      memcpy(&pl[n], lab, l);
      n += l;
    }
    // Status tail (mode 0 only): the C3 status strip renders from this.
    uint8_t tail[4];
    ui_link_status_tail(tail);
    memcpy(&pl[n], tail, 4);
    n += 4;
  } else {
    // Delta: reuse the cell format with count = 1, NO status tail (the C3
    // keeps its last tail). delta_cc = the cell's cc byte (a real CC on
    // P1/P2, kUiModCellBase+index on MOD/SYS pages).
    const char* lab = nullptr;
    uint8_t c = 0, v = 0, f = 0, k = 0;
    if (page == kUiPageChainB && g_chain.active) {
      // Chain-B deltas: delta_cc = the B cell's real CC (alias bank /
      // 23/24/65/66) — value straight from the CLOUDS cache.
      const UiPageTable& t = kUiPages[RACK_MODULE_CLOUDS][kUiPageP1];
      int8_t idx = -1;
      for (uint8_t i = 3; i < t.count; ++i) {
        if (t.cells[i].cc == delta_cc) { idx = (int8_t)i; break; }
      }
      if (idx < 0) return;                        // not a cell of this page
      c = t.cells[idx].cc;
      v = g_ui_cc_cache[RACK_MODULE_CLOUDS][c];
      f = 0;
      k = t.cells[idx].color;
      lab = t.cells[idx].label;
    } else if (page <= kUiPageP2) {
      const UiPageTable& t = kUiPages[mod][page];
      int8_t idx = -1;
      for (uint8_t i = 0; i < t.count; ++i) {
        if (t.cells[i].cc == delta_cc) { idx = (int8_t)i; break; }
      }
      if (idx < 0) return;                        // not a cell of this page
      c = t.cells[idx].cc;
      v = g_ui_cc_cache[mod][c];
      f = ui_link_cell_flags(c, mod);
      k = t.cells[idx].color;
      lab = t.cells[idx].label;
    } else {
      const UiAuxCell* a = nullptr; uint8_t cnt = 0;
      if (page == kUiPageModLfo12) { a = kUiModLfo12Cells; cnt = 16; }
      else if (page == kUiPageModLfo34) { a = kUiModLfo34Cells; cnt = 16; }
      else if (page == kUiPageModTrig) { a = kUiModTrigCells; cnt = 16; }
      else if (page == kUiPageModGate) { a = kUiModGateCells; cnt = 16; }
      else if (page == kUiPageModEnv) { a = kUiModEnvCells; cnt = 16; }
      else if (page == kUiPageSys) { a = kUiSysCells; cnt = 16; }
      if (a == nullptr) return;
      // delta_cc names the cell index (kUiModCellBase + i).
      if (delta_cc < kUiModCellBase || delta_cc - kUiModCellBase >= cnt) return;
      const uint8_t idx = (uint8_t)(delta_cc - kUiModCellBase);
      c = delta_cc;
      v = ui_aux_cell_value(a[idx]);
      f = 0;
      k = a[idx].color;
      lab = a[idx].label;
    }
    pl[n++] = page;
    pl[n++] = mod;
    pl[n++] = mode;
    pl[n++] = 1;
    pl[n++] = c; pl[n++] = v; pl[n++] = f; pl[n++] = k;
    size_t l = strlen(lab) + 1;
    memcpy(&pl[n], lab, l);
    n += l;
  }
  if (n > 255) return;                          // cannot happen; guard anyway
  ui_link_send_frame(kUiTypeTelem, pl, (uint8_t)n);
}

// MOD/SYS delta after a MOD_SET: find the ACTIVE page's cell for (src, param)
// and emit a single-cell TELEM delta so the C3 renders the applied result.
// Called from the MOD_SET dispatch below (mod_engine.h cannot call back into
// this header — it is included earlier in the TU).
static void ui_link_send_mod_delta(uint8_t src, uint8_t param) {
  const uint8_t page = g_ui_active_page;
  if (page < kUiPageModLfo12 || page > kUiPageModEnv) return;
  const UiAuxCell* a = nullptr; uint8_t cnt = 0;
  if (page == kUiPageModLfo12) { a = kUiModLfo12Cells; cnt = 16; }
  else if (page == kUiPageModLfo34) { a = kUiModLfo34Cells; cnt = 16; }
  else if (page == kUiPageModTrig) { a = kUiModTrigCells; cnt = 16; }
  else if (page == kUiPageModGate) { a = kUiModGateCells; cnt = 16; }
  else if (page == kUiPageModEnv) { a = kUiModEnvCells; cnt = 16; }
  else return;                                  // SYS cells are not mod params
  for (uint8_t i = 0; i < cnt; ++i) {
    if (a[i].src == src && a[i].param == param) {
      ui_link_send_telem(page, kUiTelemDelta,
                         (uint8_t)(kUiModCellBase + i));
      return;
    }
  }
}

// ===================== CC-value observation hook =====================
// Declared in midi.h; called at the TOP of midiControlChange — the single
// place every wire CC passes through (USB device/host/DIN/this link). Pure
// observation: no dispatch behavior change.
void ui_link_observe_cc(uint8_t cc, uint8_t val) {
  const uint8_t mod = g_active_module;
  g_ui_cc_cache[mod][cc] = val;
  g_ui_cc_seen[mod][cc >> 4] |= (uint16_t)(1u << (cc & 15));
  // Chain-B observation: while engaged, the B-directed CCs (23/24/65/66 +
  // the clouds alias bank 40-48) ALSO land in the CLOUDS cache so the
  // chain-B page's TELEM reads live values.
  if (g_chain.active && rack_chain_wants_cc(cc)) {
    g_ui_cc_cache[RACK_MODULE_CLOUDS][cc] = val;
    g_ui_cc_seen[RACK_MODULE_CLOUDS][cc >> 4] |=
        (uint16_t)(1u << (cc & 15));
  }
  preset_mark_dirty();             // T4a: a received CC is persistent state
  // Throttled (>= 50 ms) single-cell delta, gated by the C3's
  // telemetry-deltas toggle (UI_CMD sys id 0). Target page: the chain-B
  // panel when the changed CC is a B-directed one and the C3 is on that
  // page; otherwise the active module's P1/P2 page (MOD/SYS deltas come
  // from the MOD_SET / gesture ack paths instead).
  if (!g_ui_telem_deltas_en) return;
  static uint32_t last_delta_ms = 0;
  const uint32_t now = millis();
  if (g_chain.active && g_ui_active_page == kUiPageChainB &&
      rack_chain_wants_cc(cc)) {
    if (now - last_delta_ms < 50) return;
    last_delta_ms = now;
    ui_link_send_telem(kUiPageChainB, kUiTelemDelta, cc);
    return;
  }
  if (rack_chain_wants_cc(cc)) return;
  if (g_ui_active_page > kUiPageP2) return;     // MOD/SYS deltas come from the
                                                // MOD_SET / gesture ack paths
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
      // mod_engine_apply_ui_set in mod_engine.h). After applying, a
      // single-cell TELEM delta for the matching ACTIVE-page cell (if any)
      // shows the C3 the result ("the firmware acks drive the UI").
      if (g_ui_rx_len == 3) {
        mod_engine_apply_ui_set(pl[0], pl[1], pl[2]);
        ui_link_send_mod_delta(pl[0], pl[1]);
        const uint8_t s = pl[0];
        if (s < MOD_SOURCE_COUNT) {
          const uint8_t n = mod_param_count_of(g_mod_engine.src[s].type);
          if (pl[1] == n - 1) { // destination routing changed: send full snapshot
            ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
          }
        }
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
        // Sys toggles (semantics = TOGGLE; arg2 reserved 0). Wired: 0 =
        // telemetry-deltas gate, 1 = full-snapshot gate, 2 = global clock
        // source. Other ids log-only (reserved for future sys items).
        bool handled = true;
        switch (arg) {
          case kUiSysTelemDelta:
            g_ui_telem_deltas_en = !g_ui_telem_deltas_en;
            Serial.printf("ui: telemetry deltas %s\r\n",
                          g_ui_telem_deltas_en ? "ON" : "OFF");
            break;
          case kUiSysTelemFull:
            g_ui_telem_full_en = !g_ui_telem_full_en;
            Serial.printf("ui: pushed full snapshots %s\r\n",
                          g_ui_telem_full_en ? "ON" : "OFF");
            break;
          case kUiSysClockToggle:
            g_mod_engine.global_clock = g_mod_engine.global_clock ? 0 : 1;
            Serial.printf("ui: global clock = %s\r\n",
                          g_mod_engine.global_clock ? "midi" : "internal");
            break;
          default:
            Serial.printf("ui: SYS toggle arg=%u arg2=%u (id reserved, "
                          "log-only)\r\n", arg, arg2);
            handled = false;
            break;
        }
        // Echo the result when the toggled state is on the C3's active page
        // (SYS cells 6/7/11) — acks drive the UI.
        if (handled && g_ui_active_page == kUiPageSys) {
          ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
        }
      } else if (action == kUiCmdAssignCycle) {
        // [cell 0-15][dir 1 up / 0 down]: cycle the ACTIVE page cell's mod
        // source through [off, src0..src11]. The RP2350 owns the routing
        // table + validation ("the C3 never enforces") — the C3 gesture
        // would otherwise need off-protocol knowledge of which source drives
        // each cell. Invalid targets pass through mod_set_dest's one-shot
        // validation (inert + TELEM flags bit2) and the full-snapshot reply
        // re-renders the result on the C3.
        if (arg2 > 1) {
          Serial.printf("ui: ASSIGN_CYCLE dir=%u (want 0|1)\r\n", arg2);
          break;
        }
        if (g_ui_active_page > kUiPageP2) {
          Serial.printf("ui: ASSIGN_CYCLE inert on page %u (no CC cells)\r\n",
                        g_ui_active_page);
          break;
        }
        const UiPageTable& t = kUiPages[g_active_module][g_ui_active_page];
        if (arg >= t.count) {
          Serial.printf("ui: ASSIGN_CYCLE cell %u >= count %u (empty pot)\r\n",
                        arg, t.count);
          break;
        }
        const uint8_t cc = t.cells[arg].cc;
        int8_t cur = -1;                       // first source routing this CC
        for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
          if (g_mod_engine.src[i].dest_cc == cc) { cur = (int8_t)i; break; }
        }
        // Cycle position: 0 = off, 1-12 = src 0-11.
        const uint8_t pos = (cur < 0) ? 0 : (uint8_t)(cur + 1);
        const uint8_t np =
            (uint8_t)((arg2 ? pos + 1 : pos + MOD_SOURCE_COUNT) %
                      (MOD_SOURCE_COUNT + 1));
        if (np == 0) {
          if (cur >= 0) {
            mod_set_dest((uint8_t)cur, 0);     // unassign (ack log inside)
          } else {
            Serial.printf("ui: ASSIGN_CYCLE cell %u (CC %u): already off\r\n",
                          arg, cc);
          }
        } else {
          mod_set_dest(np - 1, cc);            // validation + ack log inside
        }
        ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
      } else if (action == kUiCmdSetGlobal) {
        // [item][value 0-127]: global state the C3 pots address but MOD_SET
        // cannot express (MOD_SET is per-source). Wired: item 0 = BPM
        // (20-300, the same quantization family as the `bpm` console cmd).
        // Other items log-only. Ack: full TELEM when the affected cell is
        // on the active page (MOD-C BPM / SYS).
        if (arg == kUiGlobalBpm) {
          const float bpm = 20.0f + (float)arg2 * 280.0f / 127.0f;
          g_mod_engine.bpm = roundf(bpm * 10.0f) / 10.0f;
          Serial.printf("ui: bpm <- %.1f (SET_GLOBAL)\r\n", (double)bpm);
          // Omit sending 200-byte full snapshot on every pot step to prevent UART queue backlog!
        } else {
          Serial.printf("ui: SET_GLOBAL item %u (reserved, log-only)\r\n", arg);
        }
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
      // T4a: [slot u8] — "send me your current state part" (the C3 appends
      // its own part and writes its flash bank; slot 0xFF = autosave pull).
      if (g_ui_rx_len == 1) {
        ui_preset_req_reply(pl[0]);
      } else {
        Serial.printf("ui: PRESET_REQ bad len %u (want 1)\r\n", g_ui_rx_len);
      }
      break;
    case kUiTypePresetPart:
      // RP2350->C3 direction only — a C3 never has a part we want here.
      Serial.printf("ui: PRESET_PART received (len=%u) - ignored\r\n",
                    g_ui_rx_len);
      break;
    case kUiTypePresetApply:
      // T4a: [seq][total][data...] reassembled by preset.h; on the last
      // chunk the blob is validated (magic/version/crc) and applied, then
      // a full TELEM snapshot acks to the C3.
      ui_preset_apply_chunk(pl, g_ui_rx_len);
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
  g_ui_last_rx_ms = millis();   // status tail link bit (3 s freshness window)
  ui_link_dispatch();
}

// Loop-side pump (core 0, next to the DIN pump): drain the IRQ-fed SW ring.
// With the link absent this costs one ring-empty check per loop().
static void ui_link_pump() {
  if (Serial1.overflow()) ++g_ui_rx_overflow;   // SW ring overflow (IRQ side)
  // Full-snapshot push on module/chain changes, gated by the C3's
  // full-snapshot toggle (UI_CMD sys id 1): the C3 re-renders without a
  // page request ("the firmware acks drive the UI"). Fires once per change
  // (also once at boot — last_* start at 0xFF).
  static uint8_t last_mod = 0xFF;
  static uint8_t last_chain = 0xFF;
  const uint8_t chain_now =
      (uint8_t)((g_chain.active ? 0x10 : 0x00) | g_chain.a);
  if (g_ui_telem_full_en &&
      (g_active_module != last_mod || chain_now != last_chain)) {
    last_mod = g_active_module;
    last_chain = chain_now;
    ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
  }
  while (Serial1.available() > 0) {
    const int c = Serial1.read();
    if (c < 0) break;
    ui_link_feed((uint8_t)c);
  }
}
