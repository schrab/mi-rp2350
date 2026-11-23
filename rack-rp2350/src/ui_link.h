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
// built from the five adapters' CC maps + the knob labels/colors of
// tools/rack_panel.html (the rack's current recolor; tools/elements_panel.html
// has the same trio). Color classes: 0 = white (standard params), 1 = red
// #e6295e group, 2 = teal #17a3bd group; the panels' yellow #f2d21f is a
// STATE accent, never a param class. P2: elements = CC 28/29 selectors +
// the alt-layout duplicates of its acoustic params; clouds/plaits/rings =
// duplicate of P1 (page id differs); lira8 = real P2/P3 banks (47 params do
// not fit one 16-cell page). MOD-A/B/C: real
// cells from the mod
// [src][param][value] with the SAME (src, param) map as the tables below
// (docs/ui-protocol.md §6). SYS: 8 cells (one module selector over 6 slots,
// chain pair/engage, telem gates); the C3's preset slot is C3-local (T4a).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#if defined(RACK_PICO_BUILD)
#include "pico/time.h"           // time_us_32 (echo turnaround timing)
// SDK-native UART0 link transport (implementations in rack-pico/main.cpp):
// an IRQ-fed 512 B SW RX ring + a 512 B SW TX ring drained lazily into the
// 32-deep HW FIFO — the earlephilhower SerialUART behavior, re-implemented.
int ui_uart_available();
int ui_uart_read();              // -1 when empty
bool ui_uart_overflow();         // true once per RX ring overflow event
int ui_uart_available_for_write();  // free bytes in the SW TX ring
void ui_uart_write_bytes(const uint8_t* buf, int n);  // non-blocking push
void ui_uart_sdk_init();         // uart0 @ 921600 8N1 + RX IRQ
void ui_uart_tx_service();       // drain the SW TX ring into the HW FIFO
#else
#include <Arduino.h>             // Serial1 (HW UART0 link), millis, Serial
// Arduino-build shims for the shared transport call sites below: the
// RACK_PICO_BUILD arm declares SDK implementations (rack-pico/main.cpp);
// here they map 1:1 onto the core's SerialUART.
static inline int ui_uart_available() { return Serial1.available() > 0; }
static inline int ui_uart_read() { return Serial1.read(); }
static inline bool ui_uart_overflow() { return Serial1.overflow(); }
static inline int ui_uart_available_for_write() {
  return Serial1.availableForWrite();
}
static inline void ui_uart_write_bytes(const uint8_t* buf, int n) {
  Serial1.write(buf, n);
}
static inline void ui_uart_sdk_init() { }
static inline void ui_uart_tx_service() { }  // Serial1.write self-drains
#endif

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

// Page ids (kUiPageModArp inserted at 7 so MOD pages stay contiguous 2..7).
static const uint8_t kUiPageP1        = 0;
static const uint8_t kUiPageP2        = 1;
static const uint8_t kUiPageModLfo12  = 2;
static const uint8_t kUiPageModLfo34  = 3;
static const uint8_t kUiPageModTrig   = 4;
static const uint8_t kUiPageModGate   = 5;
static const uint8_t kUiPageModEnv    = 6;
static const uint8_t kUiPageModArp    = 7;
static const uint8_t kUiPageSys       = 8;
// Chain-B panel (T5 UX): while a chain is engaged the C3 gains a page
// showing the B slot's (clouds) parameters — the cells are the CLOUDS P1
// registry table (the alias bank CCs 40-48 + 23/24/65/66 are exactly the
// B-directed CCs the chain dispatcher accepts while engaged).
static const uint8_t kUiPageChainB    = 9;
// Per-module THIRD parameter page (rack-lira8: 47 params > 2 x 16 cells).
// Id 10 keeps the MOD/SYS/chain ids stable; the C3 lists it only for the
// modules whose registry row has a P3 table.
static const uint8_t kUiPageP3        = 10;
static const uint8_t kUiPageCount     = 11;

// Module-page helpers: a page id that indexes kUiPages[module][0..2].
static inline bool ui_page_is_module(uint8_t page) {
  return page == kUiPageP1 || page == kUiPageP2 || page == kUiPageP3;
}
static inline uint8_t ui_module_page_idx(uint8_t page) {
  return (page == kUiPageP3) ? 2 : page;
}

// TELEM modes: 0 = full snapshot, 1 = single-cell delta (same format).
static const uint8_t kUiTelemFull  = 0;
static const uint8_t kUiTelemDelta = 1;

// TELEM cell flags.
static const uint8_t kUiFlagModRouted = 1 << 0;  // a mod source drives this CC
static const uint8_t kUiFlagChainB    = 1 << 1;  // cell belongs to the chain B slot
static const uint8_t kUiFlagInert     = 1 << 2;  // routing exists but is inert/rejected
// Flags bits 3-6 = the FIRST mod source routed to this CC, +1 (0 = unrouted;
// 1-12 = source 0-11). Fits in the 4-bit mask (0..15).
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
// elements — P2: selectors (CC 28) + PLAY trigger (CC 64) + VCA (CC 7)
// + FX. CC 42 selects distortion (0-4) or the T9 Warps stage (5). The 5 FX
// cells are mode-dependent (both DSPs compiled in, one active per block).
static const UiPageCell kUiPageElementsP2[] = {
    {28, "MODEL",      kUiColorWhite},
    {64, "PLAY",       kUiColorWhite},
    {7,  "VCA",        kUiColorWhite},
    {42, "FX TYPE",    kUiColorWhite},
    {43, "D DRIVE",    kUiColorWhite},
    // D BIAS removed (2026-11-18, user): inaudible. CC 44 is now the swede
    // FMV BASS (shown on the SWEDE page variant below).
    {45, "D TONE",     kUiColorWhite},
    {0,  "",           kUiColorWhite}, // dist MIX removed: drive is the amount
};
// Swede-mode P2 (CC 42 = 5): the Swedish Chainsaw FMV tonestack knobs
// (Barabas5532 plugin defaults; bass is fine-grained — most of its change
// is 0..0.1). CC 46/47 were unused on this module.
static const UiPageCell kUiPageElementsP2Swede[] = {
    {28, "MODEL",      kUiColorWhite},
    {64, "PLAY",       kUiColorWhite},
    {7,  "VCA",        kUiColorWhite},
    {42, "FX TYPE",    kUiColorWhite},
    {43, "D DRIVE",    kUiColorWhite},
    {44, "D BASS",     kUiColorWhite},
    {46, "D MID",      kUiColorWhite},
    {47, "D TREBLE",   kUiColorWhite},
    {45, "D TONE",     kUiColorWhite},
};
// Warps-mode P2 (CC 42 = 5): the original T9 controls on CC 36-40. All five
// are on the C3 now — W MIX (CC 40) makes the stage audible, W CAR (CC 39) is
// the carrier level (the user asked for it on the UI). 16 pots per page, so
// the extra cell at index 8 costs nothing.
static const UiPageCell kUiPageElementsP2Warps[] = {
    {28, "MODEL",      kUiColorWhite},
    {64, "PLAY",       kUiColorWhite},
    {7,  "VCA",        kUiColorWhite},
    {42, "FX TYPE",    kUiColorWhite},
    {36, "W ALGO",     kUiColorWhite},
    {37, "W TIMBRE",   kUiColorWhite},
    {38, "W MOD",      kUiColorWhite},
    {40, "W MIX",      kUiColorWhite},
    {39, "W CAR",      kUiColorWhite},
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
    {49, "MODE",       kUiColorWhite},
    {50, "QUALITY",    kUiColorWhite},
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
// rings — P1: physical front-panel order (16 slots: Row 0 has 3 cells + 3 empty;
// Row 1 has 4 cells: PITCH, STRUCTURE, STRUM, VCA;
// Row 2 has 6 cells: MODEL/FX, CHORD TB, CHORD, MODE, WAVE, POLY).
static const UiPageCell kUiPageRingsP1[16] = {
    // Row 0 (pots 0..5 / physical p1..p6):
    {15, "BRIGHTNESS", kUiColorWhite}, // p1
    {16, "DAMPING",    kUiColorWhite}, // p2
    {17, "POSITION",   kUiColorWhite}, // p3
    {24, "DIST DRIVE", kUiColorWhite}, // p4 (post-render distortion drive)
    {27, "DIST TYPE",  kUiColorWhite}, // p5 (0 tube / 1 fold / 2 fuzz / 3 shape / 4 swede)
    {0,  "",           kUiColorWhite}, // p6 (dist MIX removed: drive is the amount)
    // Row 1 (pots 6..9 / physical p7..p10):
    {1,  "PITCH",      kUiColorWhite}, // p7
    {14, "STRUCTURE",  kUiColorWhite}, // p8
    {64, "STRUM",      kUiColorWhite}, // p9
    {7,  "VCA",        kUiColorWhite}, // p10
    // Row 2 (pots 10..15 / physical p11..p16):
    {18, "MODEL/FX",   kUiColorWhite}, // p11
    {22, "CHORD TB",   kUiColorWhite}, // p12
    {23, "CHORD",      kUiColorWhite}, // p13
    {20, "MODE",       kUiColorWhite}, // p14
    {21, "WAVE",       kUiColorWhite}, // p15
    {19, "POLY",       kUiColorWhite}, // p16
};

// rack-lira8 pages P1/P2/P3 — pot mapping by the user (2026-11-15), control
// semantics from the LYRA-8 USER MANUAL V2.1.
// P1 = group 1234, P2 = group 5678: row0 = the pair's FAST + FM-source
// switches (cells 3/4 spare), middle row = the four TUNEs, bottom row =
// SHARP/MOD of both pairs with the group's PITCH + HOLD between them.
// P3 = MOD DELAY (DEL MODE, then MOD/TIME per line, FB), DISTORTION (MIX,
// DRIVE, DIST MIX, VOL), HYPER LFO (FREQ A/B) + the global switches
// TOTAL FB, VIBRATO, FM STRUCT.
// CC 51 is the COMBINED delay-mode selector: 0 SELF, 1 LFO-triangle,
// 2 LFO-square (the manual's SELF/LFO switch and the delay waveform switch
// merged — the LFO shape is irrelevant in SELF). CC 52 stays a direct MIDI
// override of the waveform. NOT on the pots: AND/OR (CC 42) and LINK
// (CC 43) — see the P3 row2; QUANTIZE (CC 53, a Pd-patch extra) sits at P3
// pot 14 as an ON/OFF cell, ARP HARM (CC 54, T8b) at pot 15.
// Sensors = MIDI notes 24-31 (the patch's notemap shifted one octave down),
// triggers 55-62.
static const UiPageCell kUiPageLira8P1[16] = {
    // Row 0 (pots 1..6): pair switches + this group's DELAY line
    // (MOD 1 = pot 3, TIME 1 = pot 4 — user)
    {9,  "FAST 12",    kUiColorWhite}, // p1
    {21, "FM SRC 12",  kUiColorWhite}, // p2
    {87, "MOD 1",      kUiColorWhite}, // p3 (delay line 1)
    {83, "TIME 1",     kUiColorWhite}, // p4 (delay line 1)
    {10, "FAST 34",    kUiColorWhite}, // p5
    {22, "FM SRC 34",  kUiColorWhite}, // p6
    // Row 1 (pots 7..10): the four voice tunings
    {1,  "TUNE 1",     kUiColorWhite}, // p7
    {2,  "TUNE 2",     kUiColorWhite}, // p8
    {3,  "TUNE 3",     kUiColorWhite}, // p9
    {4,  "TUNE 4",     kUiColorWhite}, // p10
    // Row 2 (pots 11..16): tone + the group controls
    {13, "SHARP 12",   kUiColorWhite}, // p11
    {17, "MOD 12",     kUiColorWhite}, // p12
    {25, "PITCH 1234", kUiColorWhite}, // p13
    {27, "HOLD 1234",  kUiColorWhite}, // p14
    {14, "SHARP 34",   kUiColorWhite}, // p15
    {18, "MOD 34",     kUiColorWhite}, // p16
};

static const UiPageCell kUiPageLira8P2[16] = {
    // Row 0 (pots 1..6): pair switches + this group's DELAY line
    // (MOD 2 = pot 3, TIME 2 = pot 4 — user)
    {11, "FAST 56",    kUiColorWhite}, // p1
    {23, "FM SRC 56",  kUiColorWhite}, // p2
    {88, "MOD 2",      kUiColorWhite}, // p3 (delay line 2)
    {84, "TIME 2",     kUiColorWhite}, // p4 (delay line 2)
    {12, "FAST 78",    kUiColorWhite}, // p5
    {24, "FM SRC 78",  kUiColorWhite}, // p6
    // Row 1 (pots 7..10): the four voice tunings
    {5,  "TUNE 5",     kUiColorWhite}, // p7
    {6,  "TUNE 6",     kUiColorWhite}, // p8
    {7,  "TUNE 7",     kUiColorWhite}, // p9
    {8,  "TUNE 8",     kUiColorWhite}, // p10
    // Row 2 (pots 11..16): tone + the group controls
    {15, "SHARP 56",   kUiColorWhite}, // p11
    {19, "MOD 56",     kUiColorWhite}, // p12
    {26, "PITCH 5678", kUiColorWhite}, // p13
    {28, "HOLD 5678",  kUiColorWhite}, // p14
    {16, "SHARP 78",   kUiColorWhite}, // p15
    {20, "MOD 78",     kUiColorWhite}, // p16
};

// P3: the delay's SHARED controls and the distortion (row 0), the hyper-LFO
// and global switches (rows 1-2). Per-line MOD/TIME live on the group pages.
static const UiPageCell kUiPageLira8P3[16] = {
    // Row 0 (pots 1..6): MOD DELAY + DISTORTION
    {51, "DEL MODE",   kUiColorWhite}, // p1 (self / lfo-tri / lfo-sqr)
    {85, "FB",         kUiColorWhite}, // p2
    {86, "MIX",        kUiColorWhite}, // p3
    {36, "DRIVE",      kUiColorWhite}, // p4
    {37, "DIST MIX",   kUiColorWhite}, // p5
    {38, "VOL",        kUiColorWhite}, // p6
    // Row 1 (pots 7..10): HYPER LFO + the global voice switches
    {39, "FREQ A",     kUiColorWhite}, // p7
    {80, "FREQ B",     kUiColorWhite}, // p8
    {30, "TOTAL FB",   kUiColorWhite}, // p9
    {31, "VIBRATO",    kUiColorWhite}, // p10
    // Row 2 (pots 11..16): the remaining switches (+ the patch-only QUANTIZE)
    {29, "FM STRUCT",  kUiColorWhite}, // p11
    {81, "AND/OR",     kUiColorWhite}, // p12
    {82, "LINK",       kUiColorWhite}, // p13
    {53, "QUANTIZE",   kUiColorWhite}, // p14 (Pd-patch extra, OFF/ON)
    {54, "ARP HARM",   kUiColorWhite}, // p15 (T8b: voices follow the ARP)
    {0,  "",           kUiColorWhite}, // p16 (spare)
};

// dronecaster — P1: the four voice controls (CC 51-54). CC 51 selects one of
// the 14 live catalog ids {0,7,10,14,15,20,22,23,24,25,27,29,30,36}; the C3
// pot and the browser panel both send those raw ids. Over-budget drones are
// flagged, not blocked — a core-1 hang is caught by the heartbeat watchdog.
// CC 52 amp, CC 53 fade (attack=release, 0..16 s),
// CC 54 a +-24 st offset on top of the current note pitch (base 55 Hz until
// a note arrives; notes = re-hz, PLAY 64 = recast).
static const UiPageCell kUiPageDronecasterP1[] = {
    {51, "DRONE",      kUiColorWhite},
    {52, "AMP",        kUiColorWhite},
    {53, "FADE",       kUiColorWhite},
    {54, "HZ OFF",     kUiColorWhite},
};

// benjolin — P1: all 14 parameters (CC 60-73; the plan's CC table). CC 70
// selects the filter (RLPF/RHPF/SVF/DFM1), CC 71 the output tap (tri1/osc1/
// tri2/osc2/pwm/comparator/filter). Defaults = the golden scenario (amp
// 0.5, osc1 40 Hz, osc2 4 Hz, RLPF 113 Hz).
static const UiPageCell kUiPageBenjolinP1[] = {
    {60, "FREQ1",      kUiColorWhite},
    {61, "FREQ2",      kUiColorWhite},
    {62, "SCALE",      kUiColorWhite},
    {63, "RUNGLER1",   kUiColorWhite},
    {64, "RUNGLER2",   kUiColorWhite},
    {65, "FILT FM",    kUiColorWhite},
    {66, "LOOP",       kUiColorWhite},
    {67, "FILT FREQ",  kUiColorWhite},
    {68, "Q",          kUiColorWhite},
    {69, "GAIN",       kUiColorWhite},
    {70, "FILTER",     kUiColorWhite},
    {71, "OUTPUT",     kUiColorWhite},
    {72, "AMP",        kUiColorWhite},
    {0,  "",           kUiColorWhite}, // p14 (spare)
    {0,  "",           kUiColorWhite}, // p15 (spare)
    {0,  "",           kUiColorWhite}, // p16 (spare)
};

// [module][0 = P1, 1 = P2, 2 = P3]. Elements' P2 = MODEL/PLAY/VCA + the T9
// warps cells (CC 36-40). Clouds, Plaits, and Rings have all parameters on
// P1, so P2/P3 have count = 0 (no duplicate params). lira8's P2/P3 are REAL
// banks (its 47 params cannot fit one page).
//
// Counts are DERIVED from the tables (T9 lesson: a hand-written literal
// drifted from its array and TELEM sent only the first N cells — the C3
// renders what it is told).
#define UI_PAGE_CELL_COUNT(t) (sizeof(t) / sizeof((t)[0]))
static constexpr UiPageTable kUiPages[RACK_MODULE_COUNT][3] = {
    { {kUiPageElementsP1, UI_PAGE_CELL_COUNT(kUiPageElementsP1)},
      {kUiPageElementsP2, UI_PAGE_CELL_COUNT(kUiPageElementsP2)},
      {nullptr, 0} },                                                    // elements
    { {kUiPageCloudsP1,   UI_PAGE_CELL_COUNT(kUiPageCloudsP1)},
      {nullptr, 0},        {nullptr, 0} },                               // clouds
    { {kUiPagePlaitsP1,   UI_PAGE_CELL_COUNT(kUiPagePlaitsP1)},
      {nullptr, 0},        {nullptr, 0} },                               // plaits
    { {kUiPageRingsP1,    UI_PAGE_CELL_COUNT(kUiPageRingsP1)},
      {nullptr, 0},        {nullptr, 0} },                               // rings
    { {kUiPageLira8P1,    UI_PAGE_CELL_COUNT(kUiPageLira8P1)},
      {kUiPageLira8P2,     UI_PAGE_CELL_COUNT(kUiPageLira8P2)},
      {kUiPageLira8P3,     UI_PAGE_CELL_COUNT(kUiPageLira8P3)} },        // lira8
    { {kUiPageDronecasterP1, UI_PAGE_CELL_COUNT(kUiPageDronecasterP1)},
      {nullptr, 0},        {nullptr, 0} },                               // dronecaster
    { {kUiPageBenjolinP1,    UI_PAGE_CELL_COUNT(kUiPageBenjolinP1)},
      {nullptr, 0},        {nullptr, 0} },                               // benjolin
};

// ===================== Dedicated Modulation & SYS aux cells =====================
// Cell order = physical pot order (the C3's MOD-page pot map sends MOD_SET
// with exactly these (src, param) pairs — MUST MATCH the C3's kModPotMap).
// `cc` in TELEM = kUiModCellBase + index. Special src 0xFF = a SYS-style
// selector in `param` (values read from chain/telem/clock state, not the
// mod engine): 0 = the MODULE selector (value = the ACTIVE module scaled
// 0..127 over RACK_MODULE_COUNT slots, or the chain pair while chained),
// 4 chain pair index, 5 chain engaged, 6 telem-delta gate,
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
    {"E1 STPS", kUiColorYellow, 4, 0},
    {"E1 PULS", kUiColorYellow, 4, 1},
    {"E1 ROT",  kUiColorYellow, 4, 2},
    {"E1 CLK",  kUiColorYellow, 4, 3},
    {"E1 DIV",  kUiColorYellow, 4, 4},
    {"",        kUiColorWhite, 0xFF, 0xFF},
    // Row 1: empty (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: EUCLID 2 (pots 10..14)
    {"E2 STPS", kUiColorYellow, 5, 0},
    {"E2 PULS", kUiColorYellow, 5, 1},
    {"E2 ROT",  kUiColorYellow, 5, 2},
    {"E2 CLK",  kUiColorYellow, 5, 3},
    {"E2 DIV",  kUiColorYellow, 5, 4},
    {"",        kUiColorWhite, 0xFF, 0xFF},
};

// MOD-GATE: 2x Gate pattern generators (Row 0 = GATE 1, Row 2 = GATE 2)
static const UiAuxCell kUiModGateCells[16] = {
    // Row 0: GATE 1 (pots 0..5)
    {"G1 MODE", kUiColorYellow, 6, 0},
    {"G1 DENS", kUiColorYellow, 6, 1},
    {"G1 SYNC", kUiColorYellow, 6, 2},
    {"G1 GLEN", kUiColorYellow, 6, 3},
    {"G1 CLK",  kUiColorYellow, 6, 4},
    {"G1 DIV",  kUiColorYellow, 6, 5},
    // Row 1: empty / spacer (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: GATE 2 (pots 10..15)
    {"G2 MODE", kUiColorYellow, 7, 0},
    {"G2 DENS", kUiColorYellow, 7, 1},
    {"G2 SYNC", kUiColorYellow, 7, 2},
    {"G2 GLEN", kUiColorYellow, 7, 3},
    {"G2 CLK",  kUiColorYellow, 7, 4},
    {"G2 DIV",  kUiColorYellow, 7, 5},
};

// MOD-ENV: 2x ADSR envelopes (Row 0 = ADSR 1, Row 2 = ADSR 2)
static const UiAuxCell kUiModEnvCells[16] = {
    // Row 0: ADSR 1 (pots 0..5)
    {"A1 ATK",  kUiColorYellow, 8, 1},
    {"A1 DEC",  kUiColorYellow, 8, 2},
    {"A1 SUS",  kUiColorYellow, 8, 3},
    {"A1 REL",  kUiColorYellow, 8, 4},
    {"A1 MODE", kUiColorYellow, 8, 0},
    {"A1 DIV",  kUiColorYellow, 8, 6},
    // Row 1: empty (pots 6..9)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: ADSR 2 (pots 10..15)
    {"A2 ATK",  kUiColorYellow, 9, 1},
    {"A2 DEC",  kUiColorYellow, 9, 2},
    {"A2 SUS",  kUiColorYellow, 9, 3},
    {"A2 REL",  kUiColorYellow, 9, 4},
    {"A2 MODE", kUiColorYellow, 9, 0},
    {"A2 DIV",  kUiColorYellow, 9, 6},
};

// MOD-ARP: 2x Switchable 5-Algo Melody/Arp Generators (Row 0 = ARP 1, Row 2 = ARP 2)
// Row 1 Pots 9/10 (slots 8/9) host shared ROOT and SCALE for 100% tonal sync.
// Params per ARP: 0=algo, 1=p1, 2=p2, 3=p3, 4=p4, 6=div
static const UiAuxCell kUiModArpCells[16] = {
    // Row 0: ARP 1 (pots 0..5 / physical 1..6)
    {"P1 ALGO",  kUiColorYellow, 10, 0},
    {"P1 DENS",  kUiColorYellow, 10, 1},
    {"P1 SLID",  kUiColorYellow, 10, 2},
    {"P1 ACC",   kUiColorYellow, 10, 3},
    {"P1 MUTE",  kUiColorYellow, 10, 4},
    {"P1 DIV",   kUiColorYellow, 10, 6},
    // Row 1: shared TRANSPOSE (slot 6), TRANS (7), ROOT (8), SCALE (9)
    {"OCTAVE",    kUiColorYellow, 10, 13},
    {"TRANS",    kUiColorYellow, 10, 12},
    {"ROOT",     kUiColorYellow, 10, 10},
    {"SCALE",    kUiColorYellow, 10, 11},
    // Row 2: ARP 2 (pots 10..15 / physical 11..16)
    {"P2 ALGO",  kUiColorYellow, 11, 0},
    {"P2 DENS",  kUiColorYellow, 11, 1},
    {"P2 FORM",  kUiColorYellow, 11, 2},
    {"P2 XFRM",  kUiColorYellow, 11, 3},
    {"P2 MUTE",  kUiColorYellow, 11, 4},
    {"P2 DIV",   kUiColorYellow, 11, 6},
};

static const char* const kArp1Labels[6][4] = {
    { "P1 DENS", "P1 SLID", "P1 ACC",  "P1 MUTE" }, // HYP-1
    { "P1 DENS", "P1 FORM", "P1 XFRM", "P1 MUTE" }, // MOTIF
    { "P1 DENS", "P1 CNTR", "P1 PEDL", "P1 GRAV" }, // MARKOV
    { "P1 PROB", "P1 PULS", "P1 RTCH", "P1 ACCM" }, // METRO
    { "P1 DENS", "P1 DJVU", "P1 STEP", "P1 SPRD" }, // DEJAVU
    { "P1 RIFF", "P1 PROG", "P1 VOIC", "P1 DISS" }, // VARG
};

static const char* const kArp2Labels[6][4] = {
    { "P2 DENS", "P2 SLID", "P2 ACC",  "P2 MUTE" }, // HYP-1
    { "P2 DENS", "P2 FORM", "P2 XFRM", "P2 MUTE" }, // MOTIF
    { "P2 DENS", "P2 CNTR", "P2 PEDL", "P2 GRAV" }, // MARKOV
    { "P2 PROB", "P2 PULS", "P2 RTCH", "P2 ACCM" }, // METRO
    { "P2 DENS", "P2 DJVU", "P2 STEP", "P2 SPRD" }, // DEJAVU
    { "P2 RIFF", "P2 PROG", "P2 VOIC", "P2 DISS" }, // VARG
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

static const char* ui_aux_cell_label(uint8_t page, uint8_t idx) {
  if (page == kUiPageModArp) {
    if (idx == 0) return "P1 ALGO";
    if (idx >= 1 && idx <= 4) {
      uint8_t a = g_mod_engine.src[10].p.arp.algo % MOD_ARP_ALGO_COUNT;
      return kArp1Labels[a][idx - 1];
    }
    if (idx == 5) return "P1 DIV";
    if (idx == 6) return "OCTAVE";
    if (idx == 7) {
      return (g_mod_engine.src[10].p.arp.root_param == 3) ? "TRANS" : "";
    }
    if (idx == 8) return "ROOT";
    if (idx == 9) return "SCALE";
    if (idx == 10) return "P2 ALGO";
    if (idx >= 11 && idx <= 14) {
      uint8_t a = g_mod_engine.src[11].p.arp.algo % MOD_ARP_ALGO_COUNT;
      return kArp2Labels[a][idx - 11];
    }
    if (idx == 15) return "P2 DIV";
    return "";
  }
  if (page == kUiPageModLfo12) return kUiModLfo12Cells[idx].label;
  if (page == kUiPageModLfo34) return kUiModLfo34Cells[idx].label;
  if (page == kUiPageModTrig)  return kUiModTrigCells[idx].label;
  if (page == kUiPageModGate)  return kUiModGateCells[idx].label;
  if (page == kUiPageModEnv)   return kUiModEnvCells[idx].label;
  return kUiSysCells[idx].label;
}

// ===================== telemetry gates (sys toggles 0/1) =====================
// Defaults ON (stage-1 behavior preserved). Toggled by UI_CMD action 3.
static uint8_t g_ui_telem_deltas_en = 1;   // throttled CC-delta stream
static uint8_t g_ui_telem_full_en = 1;     // RP2350-pushed full snapshots
                                           // (module/chain change events)
// Last verified RX frame (for the status tail's link bit) — core 0 only.
static uint32_t g_ui_last_rx_ms = 0;

// ===================== CC-value cache (TELEM source) =====================
// Observed at the single dispatch point (midiControlChange -> the hook below).
// 6 modules x 128 CCs = 768 B .bss. The full-CC-range span was chosen over a
// remapped 32-CC window to cover CC 64/65/66 (trig/strum/freeze) and the
// clouds alias bank 40-48 without index arithmetic.
static uint8_t g_ui_cc_cache[RACK_MODULE_COUNT][128];

// Elements P2 is mode-dependent: CC 42 = 5 selects the Swedish Chainsaw FMV
// knobs, 6 the T9 Warps stage. The cache above already tracks CC 42 (single
// observation point), so no extra state is needed. Every TELEM / pot lookup
// goes through this selector.
static inline const UiPageTable& ui_module_page(uint8_t mod, uint8_t page) {
  // FX TYPE cache: 0 OFF / 1 tube / 2 fold / 3 fuzz / 4 shape / 5 swede /
  // 6 warps.
  if (mod == RACK_MODULE_ELEMENTS && page == kUiPageP2) {
    const uint8_t fx = g_ui_cc_cache[RACK_MODULE_ELEMENTS][42];
    if (fx == 6u) {
      static const UiPageTable kElementsP2Warps = {
          kUiPageElementsP2Warps, UI_PAGE_CELL_COUNT(kUiPageElementsP2Warps)};
      return kElementsP2Warps;
    }
    if (fx == 5u) {
      static const UiPageTable kElementsP2Swede = {
          kUiPageElementsP2Swede, UI_PAGE_CELL_COUNT(kUiPageElementsP2Swede)};
      return kElementsP2Swede;
    }
  }
  return kUiPages[mod][ui_module_page_idx(page)];
}

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
  g_preset_dirty_ms = rack_now_ms();
}

// T4a handlers live in preset.h (included by main.cpp after this header —
// same single-TU discipline as midi.h; the definitions below see ui_link's
// statics, preset.h sees both).
void ui_preset_req_reply(uint8_t slot);              // PRESET_REQ -> PART burst
void ui_preset_apply_chunk(const uint8_t* pl, uint8_t len);  // APPLY reassembly
void preset_autosave_poll();                         // core-0 loop debounce push

// Restore cached parameter value to module (used by mod_set_dest on unassign).
// Only a value the panel actually SET may be restored: an unseen CC's cache is
// 0, and writing it would overwrite a module default (e.g. the lira8 pitch
// multiplier slamming to 0.01 when an ARP is unassigned from PITCH — T8b).
void ui_link_restore_cached_cc(uint8_t mod, uint8_t cc) {
  if (mod >= RACK_MODULE_COUNT || cc >= 128) return;
  if (!(g_ui_cc_seen[mod][cc >> 4] & (uint16_t)(1u << (cc & 15)))) return;
  kRackModules[mod].control_change(cc, g_ui_cc_cache[mod][cc]);
}

// Last page the C3 requested (delta policy target; default P1).
static uint8_t g_ui_active_page = kUiPageP1;

// Link stats (for `ui`; counters are core-0-only, no cross-core races).
static volatile uint32_t g_ui_tx_frames = 0;
static volatile uint32_t g_ui_tx_dropped = 0;
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

static inline uint8_t ui_link_cached_cc(uint8_t mod, uint8_t cc) {
  return (mod < RACK_MODULE_COUNT && cc < 128) ? g_ui_cc_cache[mod][cc] : 0;
}

static inline void ui_link_set_cached_cc(uint8_t mod, uint8_t cc, uint8_t val) {
  if (mod < RACK_MODULE_COUNT && cc < 128) {
    g_ui_cc_cache[mod][cc] = val;
    g_ui_cc_seen[mod][cc >> 4] |= (uint16_t)(1u << (cc & 15));
  }
}

static void ui_link_init_defaults() {
  memset(g_ui_cc_cache, 0, sizeof(g_ui_cc_cache));
  memset(g_ui_cc_seen, 0, sizeof(g_ui_cc_seen));

  // Global baseline for all modules: pitch 0 st, VCA full open
  for (uint8_t m = 0; m < RACK_MODULE_COUNT; ++m) {
    ui_link_set_cached_cc(m, 1, 64);   // PITCH center (0 st)
    ui_link_set_cached_cc(m, 7, 127);  // VCA full open (1.0f)
  }

  // 1. Elements (from ctag-tbd mp-Elements.jsn)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 11, 102); // STRENGTH (0.80f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 14, 64);  // CONTOUR (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 15, 0);   // BOW (0.0f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 16, 64);  // BOW TIMBRE (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 17, 0);   // BLOW (0.0f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 18, 64);  // FLOW (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 19, 64);  // BLOW TIMBRE (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 20, 102); // STRIKE (3276/4095 = 0.80f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 21, 64);  // MALLET (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 22, 64);  // STRIKE TIMBRE (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 23, 25);  // GEOMETRY (819/4095 = 0.20f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 24, 64);  // BRIGHTNESS (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 25, 32);  // DAMPING (1024/4095 = 0.25f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 26, 38);  // POSITION (1229/4095 = 0.30f)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 27, 0);   // SPACE / Reverb = 0
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 28, 0);   // MODEL (Modal)
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 29, 0);   // EASTER EGG = 0
  ui_link_set_cached_cc(RACK_MODULE_ELEMENTS, 64, 0);   // PLAY trigger = off

  // 2. Clouds (from ctag-tbd mp-Claude.jsn)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 1, 64);    // EX TONE (0.50f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 11, 102);  // EX LEVEL (0.80f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 13, 0);    // EX SRC (0)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 14, 64);   // POSITION (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 40, 64);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 15, 64);   // SIZE (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 41, 64);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 16, 64);   // PITCH (0 st center)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 42, 64);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 17, 88);   // DENSITY (2835/4095 = 0.692f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 43, 88);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 18, 85);   // TEXTURE (2729/4095 = 0.666f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 44, 85);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 19, 98);   // DRY/WET (3167/4095 = 0.773f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 45, 98);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 20, 88);   // SPREAD (2827/4095 = 0.690f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 46, 88);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 21, 12);   // FEEDBACK (399/4095 = 0.097f)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 47, 12);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 22, 0);    // REVERB = 0
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 48, 0);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 23, 0);    // MODE (Granular)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 49, 0);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 24, 0);    // QUALITY (16-bit stereo)
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 50, 0);
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 65, 0);    // FREEZE = 0
  ui_link_set_cached_cc(RACK_MODULE_CLOUDS, 66, 0);    // TRIG = 0

  // 3. Plaits (from ctag-tbd mp-TBDaits.jsn)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 14, 64);   // TIMBRE (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 15, 64);   // MORPH (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 16, 64);   // HARMONICS (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 17, 64);   // MACRO (0.0f center)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 18, 64);   // LPG (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 19, 64);   // DECAY (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 23, 0);    // ENGINE (0 = Virtual Analog)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 24, 1);    // AUX (1 = stereo OUT/AUX)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 25, 127);  // LEVEL = 127 (1.0f)
  ui_link_set_cached_cc(RACK_MODULE_PLAITS, 64, 0);    // TRIG = 0

  // 4. Rings (from ctag-tbd mp-TBDings.jsn)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 14, 64);   // STRUCTURE (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 15, 64);   // BRIGHTNESS (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 16, 64);   // DAMPING (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 17, 64);   // POSITION (2047/4095 = 0.50f)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 18, 0);    // MODEL/FX (0 = Modal)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 19, 0);    // POLY (0 = 1 voice)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 20, 0);    // MODE (0 = Rings Stereo)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 21, 0);    // WAVE (0 = Square)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 22, 0);    // CHORD TB (0 = Bryan)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 23, 0);    // CHORD (0)
  ui_link_set_cached_cc(RACK_MODULE_RINGS, 64, 0);    // STRUM = 0

  // 7. Benjolin — kCcSeeds from module_benjolin.h: the norns benjolis
  // DEFAULT mapped to raw CC bytes through the exact ControlSpecs. Without
  // these the UI cache is all zeros while the engine boots at the spec
  // defaults (LOOP/GAIN/OUTPUT = 127), so LOOP never reflected startup
  // settings and every value looked range-mismatched.
  static const uint8_t kBjCc[14] = {
      60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73};
  static const uint8_t kBjSeed[14] = {
      24, 40, 127, 20, 0, 14, 11, 13, 2, 127, 0, 127, 0, 64};
  for (uint8_t i = 0; i < 14; ++i) {
    ui_link_set_cached_cc(RACK_MODULE_BENJOLIN, kBjCc[i], kBjSeed[i]);
  }
}

static void ui_link_uart_init() {
  // ORDER (all three before begin, same as the DIN init): FIFO size + pinout
  // apply only while _running == false. 512 B SW ring: at 921600 baud max
  // (~92 KB/s) it rides out ~5.6 ms between drains — far beyond any loop()
  // stall (swap prints, LED poll ~1 ms, one TELEM write ~2.6 ms).
#if defined(RACK_PICO_BUILD)
  ui_uart_sdk_init();
#else
  Serial1.setFIFOSize(512);
  Serial1.setPinout(kPinUiTx, kPinUiRx);
  Serial1.begin(921600, SERIAL_8N1);
#endif
  ui_link_init_defaults();
}

// ===================== TX =====================
static uint8_t g_ui_tx_buf[3 + 255 + 1];  // hdr+type already counted: 0x7E,
                                          // len, type, payload(255), crc

// Non-blocking TX ring (T8 hardening): the old direct Serial1.write blocked
// core 0 for up to ~10 ms on a 1,071 B preset PART burst (and ~2.6 ms on a
// TELEM snapshot). The I2S DMA IRQ lives on core 0 and must service the
// two-channel ping-pong chain every ~500 us, so a blocking UART write caused
// audible chain dropouts (user: "gritty at high dry/wet" after a pot move
// triggered autosave). Frames are enqueued here and drained by
// ui_link_tx_pump() in the loop: it writes ONLY what the UART TX ring can
// take immediately, so core 0 never spins on the wire.
static const uint16_t kUiTxRingSize = 4096;   // covers a full 1,071 B burst
static uint8_t g_ui_tx_ring[kUiTxRingSize];
static uint16_t g_ui_tx_head = 0;   // write index
static uint16_t g_ui_tx_tail = 0;   // read index
static uint16_t g_ui_tx_count = 0;

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

// Encode + enqueue one frame. Returns false if the ring is full (counted; a
// dropped TELEM/delta is harmless, a dropped PRESET chunk would corrupt the
// blob — the 4 KiB ring holds a full burst with >2.9 KiB to spare, and TX
// drains at 92 KB/s, so this only fires under pathological flooding).
static bool ui_link_send_frame(uint8_t type, const uint8_t* payload,
                               uint8_t len) {
  const size_t n = ui_link_build_frame(type, payload, len, g_ui_tx_buf);
  if (g_ui_tx_count + n > kUiTxRingSize) {
    ++g_ui_tx_dropped;
    return false;
  }
  for (size_t i = 0; i < n; ++i) {
    g_ui_tx_ring[g_ui_tx_head] = g_ui_tx_buf[i];
    g_ui_tx_head = (uint16_t)((g_ui_tx_head + 1) % kUiTxRingSize);
  }
  g_ui_tx_count = (uint16_t)(g_ui_tx_count + n);
  ++g_ui_tx_frames;
  return true;
}

// Drain the TX ring into the UART TX ring WITHOUT blocking: write only as
// many bytes as Serial1.availableForWrite() promises. Called every core-0
// loop iteration (ui_link_pump); 92 KB/s of wire traffic needs only a few
// hundred bytes per millisecond and the ring holds 4 KiB.
static void ui_link_tx_pump() {
  // Service the staging ring FIRST, unconditionally: the app ring can be
  // empty while the staging ring still holds a frame's stranded tail (the
  // bench-caught page-update lag, 2026-11-23).
  ui_uart_tx_service();
  while (g_ui_tx_count > 0) {
    int space = ui_uart_available_for_write();
    if (space <= 0) return;
    // Leave a small margin: the core's TX ring can be drained concurrently
    // by its IRQ, but never write past what it reported as free.
    if (space > 8) space -= 8;
    uint16_t run = (g_ui_tx_tail + g_ui_tx_count > kUiTxRingSize)
        ? (uint16_t)(kUiTxRingSize - g_ui_tx_tail)
        : g_ui_tx_count;
    uint16_t n = (run < (uint16_t)space) ? run : (uint16_t)space;
    if (n > g_ui_tx_count) n = g_ui_tx_count;
    ui_uart_write_bytes(&g_ui_tx_ring[g_ui_tx_tail], n);
    g_ui_tx_tail = (uint16_t)((g_ui_tx_tail + n) % kUiTxRingSize);
    g_ui_tx_count = (uint16_t)(g_ui_tx_count - n);
  }
}

// TELEM flags for one cell (see kUiFlag*). Bits 3-6 carry the FIRST routed
// source id + 1 so the C3's assign gesture knows what it is cycling from.
static uint8_t ui_link_cell_flags(uint8_t cc, uint8_t mod) {
  if (cc == 0) return 0;
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
                                 s.type == MOD_GATE || s.type == MOD_EUCLID ||
                                 s.type == MOD_ARP);
    const bool can_trigger = (s.type == MOD_EUCLID || s.type == MOD_GATE ||
                              s.type == MOD_ARP);
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
      case MOD_ADSR:
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
      default:  // MOD_ARP
        if (c.param == 13) { // shared ARP octave (-2..+2) -> 5 steps
          const uint8_t steps = (uint8_t)(g_mod_engine.arp_octave + 2);
          return (uint8_t)(((uint16_t)steps * 128 + 12) / 5);
        }
        if (c.param == 12) { // shared TRANS / NEO-OP (0..7)
          return (uint8_t)(((uint16_t)g_mod_engine.neo_op * 128 + 64) / 8);
        }
        if (c.param == 10) { // shared ROOT (0..15)
          const uint8_t r = s.p.arp.root_param <= 15 ? s.p.arp.root_param : 0;
          return (uint8_t)(((uint16_t)r * 128 + 64) / 16);
        }
        if (c.param == 11) { // shared SCALE (0..19)
          const uint8_t sc = (s.p.arp.scale_idx & 0x3Fu) % SCALE_COUNT;
          return (uint8_t)(((uint16_t)sc * 128 + 64) / SCALE_COUNT);
        }
        switch (c.param) {
          case 0: {
            const uint8_t choice = (uint8_t)((s.p.arp.algo % MOD_ARP_ALGO_COUNT) * 3 + ((s.p.arp.range >= 1 && s.p.arp.range <= 3) ? (s.p.arp.range - 1) : 0));
            return (uint8_t)(((uint16_t)choice * 128 + 64) / (MOD_ARP_ALGO_COUNT * 3u));
          }
          case 1:  // p1: RIFF zone-centered echo (VARG) / percent
            if (s.p.arp.algo == MOD_ARP_VARG) {
              const uint8_t z = (uint8_t)(((uint16_t)s.p.arp.p[0] * 6) / 101);
              return (uint8_t)(((uint16_t)z * 128 + 64) / 6);
            }
            return (uint8_t)(((uint16_t)s.p.arp.p[0] * 127 + 50) / 100);
          case 2:  // p2: PROG archetype-centered echo (VARG) / percent
            if (s.p.arp.algo == MOD_ARP_VARG) {
              const uint8_t z = (uint8_t)(((uint16_t)s.p.arp.p[1] * 8) / 101);
              return (uint8_t)(((uint16_t)z * 128 + 64) / 8);
            }
            return (uint8_t)(((uint16_t)s.p.arp.p[1] * 127 + 50) / 100);
          case 3:  // p3: VOICE voicing-centered echo (VARG) / percent
            if (s.p.arp.algo == MOD_ARP_VARG) {
              const uint8_t z = (uint8_t)(((uint16_t)s.p.arp.p[2] * 6) / 101);
              return (uint8_t)(((uint16_t)z * 128 + 64) / 6);
            }
            return (uint8_t)(((uint16_t)s.p.arp.p[2] * 127 + 50) / 100);
          case 4: return (uint8_t)(((uint16_t)s.p.arp.p[3] * 127 + 50) / 100);
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
      // over RACK_MODULE_COUNT slots, scaled 0..127 so every module lands
      // in its own pot-space bucket (6 modules: 0/25/51/76/102/127). The old
      // `* 32` spacing wrapped module 4 to 0 in a uint8_t and put rings in
      // the plaits bucket — the C3 selector showed no LIRA8 and jumped.
      // chained = the engaged PAIR over 4 slots (0/42/85/127 — the C3
      // renders E>C / P>C / R>C / L>C from the chain tail and picks the
      // pair with the same pot).
      if (g_chain.active) {
        if (g_chain.a == RACK_MODULE_PLAITS) return 42;
        if (g_chain.a == RACK_MODULE_RINGS) return 85;
        if (g_chain.a == RACK_MODULE_LIRA8) return 127;
        return 0;   // elements->clouds
      }
      return (uint8_t)((uint16_t)g_active_module * 127 /
                       (RACK_MODULE_COUNT - 1));
    case kUiAuxSelChainPair:
      // Configured pair index (1 elements->clouds / 2 plaits->clouds /
      // 3 rings->clouds / 4 lira8->clouds) — shown even while disengaged so
      // the C3 knows what the next engage will do.
      if (g_chain.a == RACK_MODULE_ELEMENTS) return 1;
      if (g_chain.a == RACK_MODULE_PLAITS) return 2;
      if (g_chain.a == RACK_MODULE_RINGS) return 3;
      if (g_chain.a == RACK_MODULE_LIRA8) return 4;
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
  // chain: 0 off / 1-4 engaged pair index (elements/plaits/rings/lira8 ->
  // clouds).
  if (!g_chain.active) {
    out[2] = 0;
  } else if (g_chain.a == RACK_MODULE_ELEMENTS) {
    out[2] = 1;
  } else if (g_chain.a == RACK_MODULE_PLAITS) {
    out[2] = 2;
  } else if (g_chain.a == RACK_MODULE_RINGS) {
    out[2] = 3;
  } else if (g_chain.a == RACK_MODULE_LIRA8) {
    out[2] = 4;
  } else {
    out[2] = 0;                      // not a table pair (cannot happen)
  }
  // link: bit0 = the DIN parser has seen bytes since boot; bit1 = this UART
  // link verified a frame within the last 3 s.
  out[3] = (uint8_t)((din_parser.rx_bytes > 0 ? 0x01 : 0x00) |
                     ((rack_now_ms() - g_ui_last_rx_ms < 3000) ? 0x02 : 0x00));
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
          // T8c: routed/inert/source bits so the C3 shows B-slot mappings
          // (strip the chain-B dim bit — this IS the B page).
          flg[i] = (uint8_t)(ui_link_cell_flags(t.cells[i].cc,
                                                RACK_MODULE_CLOUDS) &
                             (uint8_t)~kUiFlagChainB);
          col[i] = t.cells[i].color;
        }
      }
    } else if (ui_page_is_module(page)) {
      const UiPageTable& t = ui_module_page(mod, page);
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
      else if (page == kUiPageModArp) { a = kUiModArpCells; count = 16; }
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
          : (ui_page_is_module(page))
          ? (i < ui_module_page(mod, page).count
                 ? ui_module_page(mod, page).cells[i].label : "")
          : (page == kUiPageModLfo12 ? kUiModLfo12Cells[i].label
         : page == kUiPageModLfo34 ? kUiModLfo34Cells[i].label
         : page == kUiPageModTrig ? kUiModTrigCells[i].label
         : page == kUiPageModGate ? kUiModGateCells[i].label
         : page == kUiPageModEnv ? kUiModEnvCells[i].label
         : page == kUiPageModArp ? ui_aux_cell_label(page, i)
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
    } else if (ui_page_is_module(page)) {
      const UiPageTable& t = ui_module_page(mod, page);
      int8_t idx = -1;
      for (uint8_t i = 0; i < t.count; ++i) {
        if (t.cells[i].cc != 0 && t.cells[i].cc == delta_cc) { idx = (int8_t)i; break; }
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
      else if (page == kUiPageModArp) { a = kUiModArpCells; cnt = 16; }
      else if (page == kUiPageSys) { a = kUiSysCells; cnt = 16; }
      if (a == nullptr) return;
      // delta_cc names the cell index (kUiModCellBase + i).
      if (delta_cc < kUiModCellBase || delta_cc - kUiModCellBase >= cnt) return;
      const uint8_t idx = (uint8_t)(delta_cc - kUiModCellBase);
      c = delta_cc;
      v = ui_aux_cell_value(a[idx]);
      f = 0;
      k = a[idx].color;
      lab = (page == kUiPageModArp) ? ui_aux_cell_label(page, idx) : a[idx].label;
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
  if (page < kUiPageModLfo12 || page > kUiPageModArp) return;
  const UiAuxCell* a = nullptr; uint8_t cnt = 0;
  if (page == kUiPageModLfo12) { a = kUiModLfo12Cells; cnt = 16; }
  else if (page == kUiPageModLfo34) { a = kUiModLfo34Cells; cnt = 16; }
  else if (page == kUiPageModTrig) { a = kUiModTrigCells; cnt = 16; }
  else if (page == kUiPageModGate) { a = kUiModGateCells; cnt = 16; }
  else if (page == kUiPageModEnv) { a = kUiModEnvCells; cnt = 16; }
  else if (page == kUiPageModArp) { a = kUiModArpCells; cnt = 16; }
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
  // Sync clouds alias 49/50 with native 23/24 in cache for standalone mode
  if (mod == RACK_MODULE_CLOUDS) {
    if (cc == 23) {
      g_ui_cc_cache[RACK_MODULE_CLOUDS][49] = val;
      g_ui_cc_seen[RACK_MODULE_CLOUDS][49 >> 4] |= (uint16_t)(1u << (49 & 15));
    } else if (cc == 24) {
      g_ui_cc_cache[RACK_MODULE_CLOUDS][50] = val;
      g_ui_cc_seen[RACK_MODULE_CLOUDS][50 >> 4] |= (uint16_t)(1u << (50 & 15));
    } else if (cc == 49) {
      g_ui_cc_cache[RACK_MODULE_CLOUDS][23] = val;
      g_ui_cc_seen[RACK_MODULE_CLOUDS][23 >> 4] |= (uint16_t)(1u << (23 & 15));
    } else if (cc == 50) {
      g_ui_cc_cache[RACK_MODULE_CLOUDS][24] = val;
      g_ui_cc_seen[RACK_MODULE_CLOUDS][24 >> 4] |= (uint16_t)(1u << (24 & 15));
    }
  }
  // Chain-B observation: while engaged, the B-directed CCs (clouds alias bank
  // 40-50 + 65/66) ALSO land in the CLOUDS cache so the chain-B page's TELEM reads
  // live values.
  if (g_chain.active && rack_chain_wants_cc(cc)) {
    g_ui_cc_cache[RACK_MODULE_CLOUDS][cc] = val;
    g_ui_cc_seen[RACK_MODULE_CLOUDS][cc >> 4] |=
        (uint16_t)(1u << (cc & 15));
    if (cc == 49) {
      g_ui_cc_cache[RACK_MODULE_CLOUDS][23] = val;
      g_ui_cc_seen[RACK_MODULE_CLOUDS][23 >> 4] |= (uint16_t)(1u << (23 & 15));
    } else if (cc == 50) {
      g_ui_cc_cache[RACK_MODULE_CLOUDS][24] = val;
      g_ui_cc_seen[RACK_MODULE_CLOUDS][24 >> 4] |= (uint16_t)(1u << (24 & 15));
    }
  }
  preset_mark_dirty();             // T4a: a received CC is persistent state
  // Throttled (>= 50 ms) single-cell delta, gated by the C3's
  // telemetry-deltas toggle (UI_CMD sys id 0). Target page: the chain-B
  // panel when the changed CC is a B-directed one and the C3 is on that
  // page; otherwise the active module's P1/P2 page (MOD/SYS deltas come
  // from the MOD_SET / gesture ack paths instead).
  if (!g_ui_telem_deltas_en) return;
  static uint32_t last_delta_ms = 0;
  const uint32_t now = rack_now_ms();
  if (g_chain.active && g_ui_active_page == kUiPageChainB &&
      rack_chain_wants_cc(cc)) {
    if (now - last_delta_ms < 50) return;
    last_delta_ms = now;
    ui_link_send_telem(kUiPageChainB, kUiTelemDelta, cc);
    return;
  }
  if (rack_chain_wants_cc(cc)) return;
  if (!ui_page_is_module(g_ui_active_page)) return;  // MOD/SYS deltas come from the
                                                     // MOD_SET / gesture ack paths
  if (now - last_delta_ms < 50) return;
  const UiPageTable& t = ui_module_page(mod, g_ui_active_page);
  for (uint8_t i = 0; i < t.count; ++i) {
    if (t.cells[i].cc != 0 && t.cells[i].cc == cc) {
      last_delta_ms = now;
      // Elements FX type (CC 42) swaps the P2 cell set, so the C3 needs a
      // FULL snapshot — a delta carries the value but no relabeled cells.
      if (mod == RACK_MODULE_ELEMENTS && cc == 42)
        ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
      else
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
// EVERY diagnostic below goes through dbg_printf (the non-blocking 2 KiB ring)
// and never through Serial.printf: this handler runs on core 0 from the C3's
// UART traffic, and an unconditional CDC write with no console reader attached
// BLOCKS FOREVER — the board then looks crashed (audio stops, console dead,
// UI link dead) while USB stays enumerated. That is the page-switch hang: a
// C3 frame (page request / preset chunk / mode CC) hit a blocking print.
// Console-command output (cmd_*, preset load/status) may use Serial — a reader
// is by definition attached there.
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
        dbg_printf("ui: MIDI frame bad len %u (want 4) - dropped\r\n",
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
        dbg_printf("ui: SWITCH frame bad (len=%u id=%u)\r\n",
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
        const uint8_t s = pl[0];
        const uint8_t prev_algo = (s < MOD_SOURCE_COUNT && g_mod_engine.src[s].type == MOD_ARP)
            ? g_mod_engine.src[s].p.arp.algo : 0;
        mod_engine_apply_ui_set(pl[0], pl[1], pl[2]);
        if (s < MOD_SOURCE_COUNT) {
          const uint8_t n = mod_param_count_of(g_mod_engine.src[s].type);
          if (pl[1] == n - 1) { // destination routing changed: send full snapshot
            ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
          } else if (g_mod_engine.src[s].type == MOD_ARP) {
            if (pl[1] == 0) {
              if (g_mod_engine.src[s].p.arp.algo != prev_algo) {
                // Algo family changed: push full snapshot so dynamic labels update
                ui_link_send_telem(kUiPageModArp, kUiTelemFull, 0);
              } else {
                ui_link_send_mod_delta(pl[0], pl[1]);
              }
            } else if (pl[1] == 10) {
              // Shared root changed: send full snapshot so TRANS label/value update immediately
              ui_link_send_telem(kUiPageModArp, kUiTelemFull, 0);
            } else if (pl[1] == 11 || pl[1] == 12) {
              // Shared scale / neo-op: push deltas for slots 7, 8, 9
              ui_link_send_telem(kUiPageModArp, kUiTelemDelta, (uint8_t)(kUiModCellBase + 7));
              ui_link_send_telem(kUiPageModArp, kUiTelemDelta, (uint8_t)(kUiModCellBase + 8));
              ui_link_send_telem(kUiPageModArp, kUiTelemDelta, (uint8_t)(kUiModCellBase + 9));
            } else {
              ui_link_send_mod_delta(pl[0], pl[1]);
            }
          } else {
            ui_link_send_mod_delta(pl[0], pl[1]);
          }
        }
      } else {
        dbg_printf("ui: MOD_SET frame bad len %u (want 3)\r\n", g_ui_rx_len);
      }
      break;
    case kUiTypeUiCmd: {
      if (g_ui_rx_len != 3) {
        dbg_printf("ui: UI_CMD frame bad len %u (want 3)\r\n", g_ui_rx_len);
        break;
      }
      const uint8_t action = pl[0], arg = pl[1], arg2 = pl[2];
      if (action == kUiCmdPageRequest) {
        if (arg < kUiPageCount) {
          dbg_printf("ui: page request %u -> full snapshot\r\n", arg);
          g_ui_active_page = arg;
          ui_link_send_telem(arg, kUiTelemFull, 0);   // full snapshot reply
        } else {
          dbg_printf("ui: page request %u out of range (0-%u)\r\n",
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
            dbg_printf("ui: telemetry deltas %s\r\n",
                          g_ui_telem_deltas_en ? "ON" : "OFF");
            break;
          case kUiSysTelemFull:
            g_ui_telem_full_en = !g_ui_telem_full_en;
            dbg_printf("ui: pushed full snapshots %s\r\n",
                          g_ui_telem_full_en ? "ON" : "OFF");
            break;
          case kUiSysClockToggle:
            g_mod_engine.global_clock = g_mod_engine.global_clock ? 0 : 1;
            dbg_printf("ui: global clock = %s\r\n",
                          g_mod_engine.global_clock ? "midi" : "internal");
            break;
          default:
            dbg_printf("ui: SYS toggle arg=%u arg2=%u (id reserved, "
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
          dbg_printf("ui: ASSIGN_CYCLE dir=%u (want 0|1)\r\n", arg2);
          break;
        }
        if (!ui_page_is_module(g_ui_active_page)) {
          dbg_printf("ui: ASSIGN_CYCLE inert on page %u (no CC cells)\r\n",
                        g_ui_active_page);
          break;
        }
        const UiPageTable& t =
            ui_module_page(g_active_module, g_ui_active_page);
        if (arg >= t.count) {
          dbg_printf("ui: ASSIGN_CYCLE cell %u >= count %u (empty pot)\r\n",
                        arg, t.count);
          break;
        }
        const uint8_t cc = t.cells[arg].cc;
        if (cc == 0) {
          dbg_printf("ui: ASSIGN_CYCLE cell %u is empty\r\n", arg);
          break;
        }
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
            dbg_printf("ui: ASSIGN_CYCLE cell %u (CC %u): already off\r\n",
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
          dbg_printf("ui: bpm <- %.1f (SET_GLOBAL)\r\n", (double)bpm);
          // Omit sending 200-byte full snapshot on every pot step to prevent UART queue backlog!
        } else {
          dbg_printf("ui: SET_GLOBAL item %u (reserved, log-only)\r\n", arg);
        }
      } else {
        dbg_printf("ui: UI_CMD unknown action %u\r\n", action);
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
            dbg_printf("ui: CHAIN_ROUTE module ids out of range\r\n");
          }
        } else if (pl[2] == 0) {
          cmd_chain("off");
        } else {
          dbg_printf("ui: CHAIN_ROUTE engage=%u (want 0|1)\r\n", pl[2]);
        }
      } else {
        dbg_printf("ui: CHAIN_ROUTE frame bad len %u (want 3)\r\n",
                      g_ui_rx_len);
      }
      break;
    }
    case kUiTypeTelem:
      dbg_printf("ui: TELEM received (len=%u) - RP2350->C3 only, ignored\r\n",
                    g_ui_rx_len);
      break;
    case kUiTypePresetReq:
      // T4a: [slot u8] — "send me your current state part" (the C3 appends
      // its own part and writes its flash bank; slot 0xFF = autosave pull).
      if (g_ui_rx_len == 1) {
        ui_preset_req_reply(pl[0]);
      } else {
        dbg_printf("ui: PRESET_REQ bad len %u (want 1)\r\n", g_ui_rx_len);
      }
      break;
    case kUiTypePresetPart:
      // RP2350->C3 direction only — a C3 never has a part we want here.
      dbg_printf("ui: PRESET_PART received (len=%u) - ignored\r\n",
                    g_ui_rx_len);
      break;
    case kUiTypePresetApply:
      // T4a: [seq][total][data...] reassembled by preset.h; on the last
      // chunk the blob is validated (magic/version/crc) and applied, then
      // a full TELEM snapshot acks to the C3.
      ui_preset_apply_chunk(pl, g_ui_rx_len);
      break;
    default:
      dbg_printf("ui: unknown frame type 0x%02x len=%u\r\n", type,
                    g_ui_rx_len);
      break;
  }
}

// One byte from the UART -> parser -> dispatch (on a verified frame).
static void ui_link_feed(uint8_t b) {
  if (!ui_link_step(b)) return;
  if (!g_ui_rx_crc_ok) {
    dbg_printf("ui: CRC FAIL (type=0x%02x len=%u) - frame dropped, "
                  "resyncing at the next 0x7E\r\n",
                  g_ui_rx_buf[0], g_ui_rx_len);
    return;
  }
  ++g_ui_rx_frames;
  g_ui_last_rx_ms = rack_now_ms();   // status tail link bit (3 s freshness window)
  ui_link_dispatch();
}

// Loop-side pump (core 0, next to the DIN pump): drain the IRQ-fed SW ring.
// With the link absent this costs one ring-empty check per loop().
static void ui_link_pump() {
  ui_link_tx_pump();                            // drain queued TX, never blocks
  if (ui_uart_overflow()) ++g_ui_rx_overflow;   // SW ring overflow (IRQ side)
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
  while (ui_uart_available() > 0) {
    const int c = ui_uart_read();
    if (c < 0) break;
    ui_link_feed((uint8_t)c);
  }
}
