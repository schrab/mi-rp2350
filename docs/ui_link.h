// T5 STAGE 1 — UI-MCU link (ESP32-C3 <-> RP2350, 921600 8N1 framed UART).
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
static const uint8_t kUiPageChainB    = 9;
static const uint8_t kUiPageCount     = 10;

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
struct UiPageCell {
  uint8_t cc;
  const char* label;
  uint8_t color;      // kUiColor*
};
struct UiPageTable {
  const UiPageCell* cells;
  uint8_t count;
};

// elements — P1
static const UiPageCell kUiPageElementsP1[] = {
    {15, "BOW",        kUiColorWhite},
    {17, "BLOW",       kUiColorPink},
    {20, "STRIKE",     kUiColorTeal},
    {1,  "PITCH",      kUiColorWhite},
    {11, "STRENGTH",   kUiColorWhite},
    {14, "CONTOUR",    kUiColorWhite},
    {18, "FLOW",       kUiColorPink},
    {21, "MALLET",     kUiColorTeal},
    {23, "GEOMETRY",   kUiColorWhite},
    {24, "BRIGHTNESS", kUiColorWhite},
    {16, "TIMBRE",     kUiColorWhite},
    {19, "TIMBRE",     kUiColorPink},
    {22, "TIMBRE",     kUiColorTeal},
    {25, "DAMPING",    kUiColorWhite},
    {26, "POSITION",   kUiColorWhite},
    {27, "SPACE",      kUiColorWhite},
};
// elements — P2
static const UiPageCell kUiPageElementsP2[] = {
    {28, "MODEL",      kUiColorWhite},
    {64, "PLAY",       kUiColorWhite},
    {7,  "VCA",        kUiColorWhite},
};
// clouds — P1
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
// plaits — P1
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
// rings — P1
static const UiPageCell kUiPageRingsP1[16] = {
    {15, "BRIGHTNESS", kUiColorWhite},
    {16, "DAMPING",    kUiColorWhite},
    {17, "POSITION",   kUiColorWhite},
    {0,  "",           kUiColorWhite},
    {0,  "",           kUiColorWhite},
    {0,  "",           kUiColorWhite},
    {1,  "PITCH",      kUiColorWhite},
    {14, "STRUCTURE",  kUiColorWhite},
    {64, "STRUM",      kUiColorWhite},
    {7,  "VCA",        kUiColorWhite},
    {18, "MODEL/FX",   kUiColorWhite},
    {22, "CHORD TB",   kUiColorWhite},
    {23, "CHORD",      kUiColorWhite},
    {20, "MODE",       kUiColorWhite},
    {21, "WAVE",       kUiColorWhite},
    {19, "POLY",       kUiColorWhite},
};

static const UiPageTable kUiPages[RACK_MODULE_COUNT][2] = {
    { {kUiPageElementsP1, 16}, {kUiPageElementsP2, 3} },   // elements
    { {kUiPageCloudsP1, 16},   {nullptr, 0} },             // clouds
    { {kUiPagePlaitsP1, 11},   {nullptr, 0} },             // plaits
    { {kUiPageRingsP1, 16},    {nullptr, 0} },             // rings
};
static const UiPageTable kUiPageEmpty = { nullptr, 0 };

// ===================== Dedicated Modulation & SYS aux cells =====================
struct UiAuxCell {
  const char* label;
  uint8_t color;      // kUiColor*
  uint8_t src;        // mod source 0-11, or 0xFF = selector
  uint8_t param;      // kModParamNames index, or the selector id
};
static const uint8_t kUiAuxSelModule    = 0;
static const uint8_t kUiAuxSelChainPair = 4;
static const uint8_t kUiAuxSelChainOn   = 5;
static const uint8_t kUiAuxSelBpm       = 6;
static const uint8_t kUiAuxSelClkSrc    = 7;
static const uint8_t kUiAuxSelTelemD    = 14;
static const uint8_t kUiAuxSelTelemF    = 15;

// MOD-LFO 1/2
static const UiAuxCell kUiModLfo12Cells[16] = {
    {"L1 RATE",  kUiColorYellow, 0, 1},
    {"L1 WAVE",  kUiColorYellow, 0, 2},
    {"L1 PHASE", kUiColorYellow, 0, 3},
    {"L1 LEV",   kUiColorYellow, 0, 4},
    {"L1 MODE",  kUiColorYellow, 0, 0},
    {"L1 DIV",   kUiColorYellow, 0, 6},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"L2 RATE",  kUiColorYellow, 1, 1},
    {"L2 WAVE",  kUiColorYellow, 1, 2},
    {"L2 PHASE", kUiColorYellow, 1, 3},
    {"L2 LEV",   kUiColorYellow, 1, 4},
    {"L2 MODE",  kUiColorYellow, 1, 0},
    {"L2 DIV",   kUiColorYellow, 1, 6},
};

// MOD-LFO 3/4
static const UiAuxCell kUiModLfo34Cells[16] = {
    {"L3 RATE",  kUiColorYellow, 2, 1},
    {"L3 WAVE",  kUiColorYellow, 2, 2},
    {"L3 PHASE", kUiColorYellow, 2, 3},
    {"L3 LEV",   kUiColorYellow, 2, 4},
    {"L3 MODE",  kUiColorYellow, 2, 0},
    {"L3 DIV",   kUiColorYellow, 2, 6},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"L4 RATE",  kUiColorYellow, 3, 1},
    {"L4 WAVE",  kUiColorYellow, 3, 2},
    {"L4 PHASE", kUiColorYellow, 3, 3},
    {"L4 LEV",   kUiColorYellow, 3, 4},
    {"L4 MODE",  kUiColorYellow, 3, 0},
    {"L4 DIV",   kUiColorYellow, 3, 6},
};

// MOD-TRIG: 2x Euclidean (Row 0 = EUC 1, Row 2 = EUC 2)
static const UiAuxCell kUiModTrigCells[16] = {
    {"E1 STPS", kUiColorTeal, 4, 0},
    {"E1 PULS", kUiColorTeal, 4, 1},
    {"E1 ROT",  kUiColorTeal, 4, 2},
    {"E1 CLK",  kUiColorTeal, 4, 3},
    {"E1 DIV",  kUiColorTeal, 4, 4},
    {"",        kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"E2 STPS", kUiColorTeal, 5, 0},
    {"E2 PULS", kUiColorTeal, 5, 1},
    {"E2 ROT",  kUiColorTeal, 5, 2},
    {"E2 CLK",  kUiColorTeal, 5, 3},
    {"E2 DIV",  kUiColorTeal, 5, 4},
    {"",        kUiColorWhite, 0xFF, 0xFF},
};

// MOD-GATE: 2x Gate pattern generators (Row 0 = GATE 1, Row 2 = GATE 2) — UNTOUCHED
static const UiAuxCell kUiModGateCells[16] = {
    {"G1 MODE", kUiColorTeal, 6, 0},
    {"G1 DENS", kUiColorTeal, 6, 1},
    {"G1 SYNC", kUiColorTeal, 6, 2},
    {"G1 GLEN", kUiColorTeal, 6, 3},
    {"G1 CLK",  kUiColorTeal, 6, 4},
    {"G1 DIV",  kUiColorTeal, 6, 5},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"G2 MODE", kUiColorTeal, 7, 0},
    {"G2 DENS", kUiColorTeal, 7, 1},
    {"G2 SYNC", kUiColorTeal, 7, 2},
    {"G2 GLEN", kUiColorTeal, 7, 3},
    {"G2 CLK",  kUiColorTeal, 7, 4},
    {"G2 DIV",  kUiColorTeal, 7, 5},
};

// MOD-ENV: 2x ADSR envelopes (Row 0 = ADSR 1, Row 2 = ADSR 2)
static const UiAuxCell kUiModEnvCells[16] = {
    {"A1 ATK",  kUiColorTeal, 8, 1},
    {"A1 DEC",  kUiColorTeal, 8, 2},
    {"A1 SUS",  kUiColorTeal, 8, 3},
    {"A1 REL",  kUiColorTeal, 8, 4},
    {"A1 MODE", kUiColorTeal, 8, 0},
    {"A1 DIV",  kUiColorTeal, 8, 6},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"A2 ATK",  kUiColorTeal, 9, 1},
    {"A2 DEC",  kUiColorTeal, 9, 2},
    {"A2 SUS",  kUiColorTeal, 9, 3},
    {"A2 REL",  kUiColorTeal, 9, 4},
    {"A2 MODE", kUiColorTeal, 9, 0},
    {"A2 DIV",  kUiColorTeal, 9, 6},
};

// MOD-ARP: 2x Switchable 3-Algo Melody/Arp Generators (Row 0 = ARP 1, Row 2 = ARP 2)
// Params: 0=algo(HYP1/BARLOW/GRIDS), 1=root(0=MIDI,1..127=Note), 2=scale, 3=dens, 4=groove, 5=clk, 6=div
static const UiAuxCell kUiModArpCells[16] = {
    // Row 0: ARP 1 (pots 0..5)
    {"P1 ALGO", kUiColorPink, 10, 0},
    {"P1 ROOT", kUiColorPink, 10, 1},
    {"P1 SCAL", kUiColorPink, 10, 2},
    {"P1 DENS", kUiColorPink, 10, 3},
    {"P1 GRV",  kUiColorPink, 10, 4},
    {"P1 DIV",  kUiColorPink, 10, 6},
    // Row 1: empty (pots 6..9 — matches LFO/TRIG/GATE/ENV layout)
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    {"", kUiColorWhite, 0xFF, 0xFF},
    // Row 2: ARP 2 (pots 10..15)
    {"P2 ALGO", kUiColorPink, 11, 0},
    {"P2 ROOT", kUiColorPink, 11, 1},
    {"P2 SCAL", kUiColorPink, 11, 2},
    {"P2 DENS", kUiColorPink, 11, 3},
    {"P2 GRV",  kUiColorPink, 11, 4},
    {"P2 DIV",  kUiColorPink, 11, 6},
};

// SYS
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
static uint8_t g_ui_telem_deltas_en = 1;
static uint8_t g_ui_telem_full_en = 1;
static uint32_t g_ui_last_rx_ms = 0;

// ===================== CC-value cache (TELEM source) =====================
static uint8_t g_ui_cc_cache[RACK_MODULE_COUNT][128];
static uint16_t g_ui_cc_seen[RACK_MODULE_COUNT][8];

static volatile uint32_t g_preset_dirty_ms = 0;
static volatile bool g_preset_dirty = false;
void preset_mark_dirty() {
  g_preset_dirty = true;
  g_preset_dirty_ms = millis();
}

void ui_preset_req_reply(uint8_t slot);
void ui_preset_apply_chunk(const uint8_t* pl, uint8_t len);
void preset_autosave_poll();

void ui_link_restore_cached_cc(uint8_t mod, uint8_t cc) {
  if (mod < RACK_MODULE_COUNT && cc < 128) {
    kRackModules[mod].control_change(cc, g_ui_cc_cache[mod][cc]);
  }
}

static uint8_t g_ui_active_page = kUiPageP1;

static volatile uint32_t g_ui_tx_frames = 0;
static volatile uint32_t g_ui_rx_frames = 0;
static volatile uint32_t g_ui_crc_errors = 0;
static volatile uint32_t g_ui_rx_overflow = 0;
static volatile uint32_t g_ui_last_rtt_us = 0;

// ===================== UART (Serial1 = HW UART0) =====================
static const uint8_t kPinUiTx = 0;
static const uint8_t kPinUiRx = 1;

static void ui_link_uart_init() {
  Serial1.setFIFOSize(512);
  Serial1.setPinout(kPinUiTx, kPinUiRx);
  Serial1.begin(921600, SERIAL_8N1);
  for (uint8_t m = 0; m < RACK_MODULE_COUNT; ++m) {
    g_ui_cc_cache[m][7] = 127;
    g_ui_cc_cache[m][1] = 64;
    g_ui_cc_seen[m][0] = 0x0083;
  }
  g_ui_cc_cache[RACK_MODULE_PLAITS][25] = 102;
  g_ui_cc_seen[RACK_MODULE_PLAITS][1] = 0x0200;
}

// ===================== TX =====================
static uint8_t g_ui_tx_buf[3 + 255 + 1];

static size_t ui_link_build_frame(uint8_t type, const uint8_t* payload,
                                  uint8_t len, uint8_t* out) {
  out[0] = kUiFrameMark;
  out[1] = len;
  out[2] = type;
  if (len) memcpy(&out[3], payload, len);
  out[3 + len] = ui_crc8(&out[2], (size_t)len + 1);
  return (size_t)len + 4;
}

static void ui_link_send_frame(uint8_t type, const uint8_t* payload,
                               uint8_t len) {
  const size_t n = ui_link_build_frame(type, payload, len, g_ui_tx_buf);
  Serial1.write(g_ui_tx_buf, n);
  ++g_ui_tx_frames;
}

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
static uint8_t ui_mod_lfo_rate_val(float hz) {
  if (hz < 0.02f) hz = 0.02f;
  if (hz > 20.0f) hz = 20.0f;
  float v = 127.0f * logf(hz / 0.02f) / (3.0f * logf(10.0f));
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  return (uint8_t)(v + 0.5f);
}
static uint8_t ui_mod_adsr_s_val(float s) {
  if (s < 1e-3f) s = 1e-3f;
  if (s > 10.0f) s = 10.0f;
  float v = 127.0f * (log10f(s) + 3.0f) / 4.0f;
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  return (uint8_t)(v + 0.5f);
}
static inline uint8_t ui_ceil_div(uint16_t a, uint16_t b) {
  return (uint8_t)((a + b - 1) / b);
}

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
        switch (c.param) {
          case 0: {
            const uint8_t base = (s.p.arp.algo >= 2) ? 86 : (s.p.arp.algo == 1 ? 43 : 0);
            const uint8_t rng = (s.p.arp.range > 1) ? (uint8_t)((s.p.arp.range - 1) * 15) : 0;
            return (uint8_t)(base + rng);
          }
          case 1:
            if (s.p.arp.root_param == 0) return 0;
            return (uint8_t)(1 + (((uint16_t)(s.p.arp.root_param - 24) * 126 + 30) / 60));
          case 2: return ui_ceil_div((uint16_t)s.p.arp.scale_idx * 128u, 16u);
          case 3: return (uint8_t)(((uint16_t)s.p.arp.density * 127 + 50) / 100);
          case 4: return (uint8_t)(((uint16_t)s.p.arp.groove * 127 + 50) / 100);
          case 5: return s.clock_sel == MOD_CLK_GLOBAL ? 0 : (s.clock_sel == MOD_CLK_INTERNAL ? 64 : 127);
          case 6: return (uint8_t)(((uint16_t)s.division * 127) / (MOD_DIVISION_COUNT - 1));
          default: return 0;
        }
    }
  }
  // SYS-style selectors (src 0xFF).
  switch (c.param) {
    case kUiAuxSelModule:
      if (g_chain.active) {
        if (g_chain.a == RACK_MODULE_PLAITS) return 64;
        if (g_chain.a == RACK_MODULE_RINGS) return 127;
        return 0;
      }
      return (uint8_t)(g_active_module * 32);
    case kUiAuxSelChainPair:
      if (g_chain.a == RACK_MODULE_ELEMENTS) return 1;
      if (g_chain.a == RACK_MODULE_PLAITS) return 2;
      if (g_chain.a == RACK_MODULE_RINGS) return 3;
      return 0;
    case kUiAuxSelChainOn: return g_chain.active ? 127 : 0;
    case kUiAuxSelTelemD: return g_ui_telem_deltas_en ? 127 : 0;
    case kUiAuxSelTelemF: return g_ui_telem_full_en ? 127 : 0;
    case kUiAuxSelBpm: {
      float b = g_mod_engine.bpm;
      if (b < 20.0f) b = 20.0f;
      if (b > 300.0f) b = 300.0f;
      return (uint8_t)((b - 20.0f) * 127.0f / 280.0f + 0.5f);
    }
    case kUiAuxSelClkSrc: return g_mod_engine.global_clock ? 127 : 0;
    default: return 0;
  }
}

// Status tail (full snapshots, mode 0 only): [bpm][clocksrc][chain][link].
static void ui_link_status_tail(uint8_t* out) {
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
  if (!g_chain.active) {
    out[2] = 0;
  } else if (g_chain.a == RACK_MODULE_ELEMENTS) {
    out[2] = 1;
  } else if (g_chain.a == RACK_MODULE_PLAITS) {
    out[2] = 2;
  } else if (g_chain.a == RACK_MODULE_RINGS) {
    out[2] = 3;
  } else {
    out[2] = 0;
  }
  out[3] = (uint8_t)((din_parser.rx_bytes > 0 ? 0x01 : 0x00) |
                     ((millis() - g_ui_last_rx_ms < 3000) ? 0x02 : 0x00));
}

static void ui_link_send_telem(uint8_t page, uint8_t mode, uint8_t delta_cc) {
  const uint8_t mod = g_active_module;
  uint8_t pl[256];
  size_t n = 0;

  if (mode == kUiTelemFull) {
    uint8_t count = 0;
    uint8_t cc[16], val[16], flg[16], col[16];
    if (page == kUiPageChainB && g_chain.active) {
      const UiPageTable& t = kUiPages[RACK_MODULE_CLOUDS][kUiPageP1];
      count = t.count;
      for (uint8_t i = 0; i < count; ++i) {
        if (i < 3) {
          cc[i] = 0; val[i] = 0; flg[i] = 0; col[i] = 0;
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
      else if (page == kUiPageModArp) { a = kUiModArpCells; count = 16; }
      else if (page == kUiPageSys) { a = kUiSysCells; count = 16; }
      if (a != nullptr) {
        for (uint8_t i = 0; i < count; ++i) {
          cc[i] = (uint8_t)(kUiModCellBase + i);
          val[i] = ui_aux_cell_value(a[i]);
          flg[i] = 0;
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
         : page == kUiPageModArp ? kUiModArpCells[i].label
                                 : kUiSysCells[i].label);
      size_t l = strlen(lab) + 1;
      memcpy(&pl[n], lab, l);
      n += l;
    }
    uint8_t tail[4];
    ui_link_status_tail(tail);
    memcpy(&pl[n], tail, 4);
    n += 4;
  } else {
    const char* lab = nullptr;
    uint8_t c = 0, v = 0, f = 0, k = 0;
    if (page == kUiPageChainB && g_chain.active) {
      const UiPageTable& t = kUiPages[RACK_MODULE_CLOUDS][kUiPageP1];
      int8_t idx = -1;
      for (uint8_t i = 3; i < t.count; ++i) {
        if (t.cells[i].cc == delta_cc) { idx = (int8_t)i; break; }
      }
      if (idx < 0) return;
      c = t.cells[idx].cc;
      v = g_ui_cc_cache[RACK_MODULE_CLOUDS][c];
      f = 0;
      k = t.cells[idx].color;
      lab = t.cells[idx].label;
    } else if (page <= kUiPageP2) {
      const UiPageTable& t = kUiPages[mod][page];
      int8_t idx = -1;
      for (uint8_t i = 0; i < t.count; ++i) {
        if (t.cells[i].cc != 0 && t.cells[i].cc == delta_cc) { idx = (int8_t)i; break; }
      }
      if (idx < 0) return;
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
  if (n > 255) return;
  ui_link_send_frame(kUiTypeTelem, pl, (uint8_t)n);
}

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
  else return;
  for (uint8_t i = 0; i < cnt; ++i) {
    if (a[i].src == src && a[i].param == param) {
      ui_link_send_telem(page, kUiTelemDelta,
                         (uint8_t)(kUiModCellBase + i));
      return;
    }
  }
}

// ===================== CC-value observation hook =====================
void ui_link_observe_cc(uint8_t cc, uint8_t val) {
  const uint8_t mod = g_active_module;
  g_ui_cc_cache[mod][cc] = val;
  g_ui_cc_seen[mod][cc >> 4] |= (uint16_t)(1u << (cc & 15));
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
  preset_mark_dirty();
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
  if (g_ui_active_page > kUiPageP2) return;
  if (now - last_delta_ms < 50) return;
  const UiPageTable& t = kUiPages[mod][g_ui_active_page];
  for (uint8_t i = 0; i < t.count; ++i) {
    if (t.cells[i].cc != 0 && t.cells[i].cc == cc) {
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
static uint16_t g_ui_rx_idx = 0;
static uint8_t g_ui_rx_buf[256];
static uint8_t g_ui_rx_crc = 0;
static bool g_ui_rx_crc_ok = false;

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
static void ui_link_dispatch() {
  const uint8_t type = g_ui_rx_buf[0];
  const uint8_t* pl = &g_ui_rx_buf[1];
  switch (type) {
    case kUiTypeMidi:
      if (g_ui_rx_len == 4) {
        midiHandleEventPacket(pl);
      } else {
        Serial.printf("ui: MIDI frame bad len %u (want 4) - dropped\r\n",
                      g_ui_rx_len);
      }
      break;
    case kUiTypeSwitch: {
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
      if (g_ui_rx_len == 3) {
        mod_engine_apply_ui_set(pl[0], pl[1], pl[2]);
        ui_link_send_mod_delta(pl[0], pl[1]);
        const uint8_t s = pl[0];
        if (s < MOD_SOURCE_COUNT) {
          const uint8_t n = mod_param_count_of(g_mod_engine.src[s].type);
          if (pl[1] == n - 1) {
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
          ui_link_send_telem(arg, kUiTelemFull, 0);
        } else {
          Serial.printf("ui: page request %u out of range (0-%u)\r\n",
                        arg, kUiPageCount - 1);
        }
      } else if (action == kUiCmdEcho) {
        const uint32_t t0 = time_us_32();
        const uint8_t p[3] = { kUiCmdEcho, arg, arg2 };
        ui_link_send_frame(kUiTypeUiCmd, p, 3);
        g_ui_last_rtt_us = time_us_32() - t0;
      } else if (action == kUiCmdSysToggle) {
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
        if (handled && g_ui_active_page == kUiPageSys) {
          ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
        }
      } else if (action == kUiCmdAssignCycle) {
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
        if (cc == 0) {
          Serial.printf("ui: ASSIGN_CYCLE cell %u is empty\r\n", arg);
          break;
        }
        int8_t cur = -1;
        for (uint8_t i = 0; i < MOD_SOURCE_COUNT; ++i) {
          if (g_mod_engine.src[i].dest_cc == cc) { cur = (int8_t)i; break; }
        }
        const uint8_t pos = (cur < 0) ? 0 : (uint8_t)(cur + 1);
        const uint8_t np =
            (uint8_t)((arg2 ? pos + 1 : pos + MOD_SOURCE_COUNT) %
                      (MOD_SOURCE_COUNT + 1));
        if (np == 0) {
          if (cur >= 0) {
            mod_set_dest((uint8_t)cur, 0);
          } else {
            Serial.printf("ui: ASSIGN_CYCLE cell %u (CC %u): already off\r\n",
                          arg, cc);
          }
        } else {
          mod_set_dest(np - 1, cc);
        }
        ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
      } else if (action == kUiCmdSetGlobal) {
        if (arg == kUiGlobalBpm) {
          const float bpm = 20.0f + (float)arg2 * 280.0f / 127.0f;
          g_mod_engine.bpm = roundf(bpm * 10.0f) / 10.0f;
          Serial.printf("ui: bpm <- %.1f (SET_GLOBAL)\r\n", (double)bpm);
        } else {
          Serial.printf("ui: SET_GLOBAL item %u (reserved, log-only)\r\n", arg);
        }
      } else {
        Serial.printf("ui: UI_CMD unknown action %u\r\n", action);
      }
      break;
    }
    case kUiTypeChainRoute: {
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
      if (g_ui_rx_len == 1) {
        ui_preset_req_reply(pl[0]);
      } else {
        Serial.printf("ui: PRESET_REQ bad len %u (want 1)\r\n", g_ui_rx_len);
      }
      break;
    case kUiTypePresetPart:
      Serial.printf("ui: PRESET_PART received (len=%u) - ignored\r\n",
                    g_ui_rx_len);
      break;
    case kUiTypePresetApply:
      ui_preset_apply_chunk(pl, g_ui_rx_len);
      break;
    default:
      Serial.printf("ui: unknown frame type 0x%02x len=%u\r\n", type,
                    g_ui_rx_len);
      break;
  }
}

static void ui_link_feed(uint8_t b) {
  if (!ui_link_step(b)) return;
  if (!g_ui_rx_crc_ok) {
    Serial.printf("ui: CRC FAIL (type=0x%02x len=%u) - frame dropped, "
                  "resyncing at the next 0x7E\r\n",
                  g_ui_rx_buf[0], g_ui_rx_len);
    return;
  }
  ++g_ui_rx_frames;
  g_ui_last_rx_ms = millis();
  ui_link_dispatch();
}

static void ui_link_pump() {
  if (Serial1.overflow()) ++g_ui_rx_overflow;
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