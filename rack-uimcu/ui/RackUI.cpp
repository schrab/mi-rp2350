/***************
Rack UI controller — fixed-page renderer + gestures (see RackUI.hpp).
Renders ONLY from the RackLink TELEM cache + local UI state ("the firmware
acks drive the UI" — the C3 never invents values; missing TELEM = dashes).
***************/

#include "RackUI.hpp"

#include "Display.hpp"
#include "RackLink.hpp"
#include "RackPots.hpp"
#include "input/UserInput.hpp"
#include "BtMidiReceiver.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>

static const char *TAG = "RackUI";

namespace CTAG {
    namespace CTRL {

        using DRIVERS::Display;

        // ===================== MOD-page pot maps =====================
        // (src, param) per pot; {0xFF, 0xFF} = no MOD_SET target. MUST MATCH
        // the RP2350's kUiModA/B/CCells cell order (docs/ui-protocol.md §6)
        // — the pot index IS the cell index on these pages.
        struct SrcParam { uint8_t src, param; };
        static const SrcParam kNoSet = { 0xFF, 0xFF };
        static const SrcParam kModPotMap[5][16] = {
            {   // MOD-LFO 1/2 (page 2): LFO 1 (pots 0..5), LFO 2 (pots 10..15)
                {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 0}, {0, 6},
                kNoSet, kNoSet, kNoSet, kNoSet,
                {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 0}, {1, 6},
            },
            {   // MOD-LFO 3/4 (page 3): LFO 3 (pots 0..5), LFO 4 (pots 10..15)
                {2, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 0}, {2, 6},
                kNoSet, kNoSet, kNoSet, kNoSet,
                {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 0}, {3, 6},
            },
            {   // MOD-TRIG (page 4): EUCLID 1 (pots 0..4), EUCLID 2 (pots 10..14)
                {4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}, kNoSet,
                kNoSet, kNoSet, kNoSet, kNoSet,
                {5, 0}, {5, 1}, {5, 2}, {5, 3}, {5, 4}, kNoSet,
            },
            {   // MOD-GATE (page 5): GATE 1 (pots 0..4), GATE 2 (pots 10..14)
                {6, 0}, {6, 1}, {6, 2}, {6, 3}, {6, 4}, kNoSet,
                kNoSet, kNoSet, kNoSet, kNoSet,
                {7, 0}, {7, 1}, {7, 2}, {7, 3}, {7, 4}, kNoSet,
            },
            {   // MOD-ENV (page 6): ADSR 1 (pots 0..5), ADSR 2 (pots 10..15)
                {8, 1}, {8, 2}, {8, 3}, {8, 4}, {8, 0}, {8, 6},
                kNoSet, kNoSet, kNoSet, kNoSet,
                {9, 1}, {9, 2}, {9, 3}, {9, 4}, {9, 0}, {9, 6},
            },
        };

        // ===================== layout =====================
        // Status strip y 0..11; 3 cell rows at y 12 / 33 / 54, each:
        // label (6) + value (6) + bar (2) + spacing.
        static constexpr int STRIP_H = 12;
        static constexpr int ROW_Y[3] = { 12, 33, 54 };
        static constexpr int LABEL_DY = 0;
        static constexpr int VALUE_DY = 7;
        static constexpr int BAR_DY = 14;    // 2 px tall

        // Cell rects: rows 1/3 have 6 cells (x = 2, 49, 96, 143, 190, 237);
        // row 2 has 4 cells staggered halfway between cols 0-1, 1-2, 3-4, 4-5
        // (matching docs/UI_sketch.png).
        static int cellX(int slot) {         // slot 0-15 in pot order
            if (slot < 6) return 2 + slot * 47;
            if (slot == 6) return 25;        // between col 0 (2) and col 1 (49)
            if (slot == 7) return 72;        // between col 1 (49) and col 2 (96)
            if (slot == 8) return 166;       // between col 3 (143) and col 4 (190)
            if (slot == 9) return 213;       // between col 4 (190) and col 5 (237)
            return 2 + (slot - 10) * 47;
        }
        static int cellW(int slot) {
            (void)slot;
            return 46;
        }
        static int cellY(int slot) {
            return ROW_Y[slot < 6 ? 0 : (slot < 10 ? 1 : 2)];
        }

        // Class byte -> RGB565.
        static uint16_t classColor(uint8_t cls) {
            switch (cls) {
                case 1: return Display::kPink;
                case 2: return Display::kTeal;
                case 3: return Display::kYellow;
                default: return Display::kWhite;
            }
        }

        // ===================== state =====================
        static uint8_t s_page = 0;            // 0-7
        static bool s_assignArmed = false;    // push held (assign/overlay)
        static bool s_overlay = false;        // module-select overlay live
        static uint8_t s_overlaySel = 0;      // 0-3
        static bool s_fullArrived = false;    // TELEM snapshot for s_page
        static uint8_t s_presetSlot = 0;      // SYS: C3-local selection
        static uint8_t s_sysRing = 0;         // SYS push-ring position
        static bool s_blinkPhase = false;     // 2 Hz inert-cell blink
        static uint8_t s_activePot = 0xFF;    // last moved pot (0-15)
        static uint8_t s_assignPot = 0xFF;    // pot being mapped while held
        static uint8_t s_assignSrc = 0;       // 0 = OFF, 1..10 = LFO1..ADS2
        static uint16_t s_lastWireSent[16];   // dedup discrete wire sends
        static int64_t s_pendingRefreshUs = 0;
        static bool s_bootModCleaned = false;

        static RackTelemPage s_cur;          // last TELM copy (renderer)
        static bool s_stripDirty = true;
        static bool s_pageDirty = true;

        // Rendered-cell diff snapshot (skip unchanged cells). pick: the
        // soft-takeover arrow state (0 none / 1 '<' left of value / 2 '>'
        // right of value) — tracked so arrow flips redraw even without any
        // TELEM traffic (alignment crossings come from the pot task).
        struct CellSnap {
            uint8_t value, flags, color;
            uint8_t pick;
            char label[16];
            bool present;
        };
        static CellSnap s_rendered[16];

        static const char *kPageNames[8] = {
            "P1", "P2", "MOD-LFO 1/2", "MOD-LFO 3/4", "MOD-TRIG", "MOD-GATE", "MOD-ENV", "SYS",
        };
        static const char *kModNames[4] = { "ELEMENTS", "CLOUDS", "PLAITS", "RINGS" };
        static const char *kSrcNames[11] = {
            "OFF",  "LFO1", "LFO2", "LFO3", "LFO4",
            "EUC1", "EUC2",
            "GAT1", "GAT2", "ADS1", "ADS2",
        };

        static inline bool ModuleHasP2(uint8_t mod) {
            return (mod & 3) == 0; // Only Elements (module 0) has P2
        }

        // Return discrete option count N (2..98) if (page, mod, slot, cc) is
        // a stepped selector on P1/P2 whose wire byte is 0..N-1 (or 0/127 for
        // 2-state triggers/toggles); 0 if continuous 0..127.
        static uint8_t p1p2DiscreteCount(uint8_t mod, uint8_t cc,
                                         bool &wireIsBool127) {
            wireIsBool127 = false;
            switch (mod & 3) {
                case 0: // Elements
                    if (cc == 28) return 3;   // MODEL: 0..2
                    if (cc == 29) return 2;   // OMINOUS: 0..1
                    break;
                case 1: // Clouds
                    if (cc == 23) return 3;   // MODE: 0..2
                    if (cc == 24) return 4;   // QUALITY: 0..3
                    if (cc == 65 || cc == 66) {
                        wireIsBool127 = true;
                        return 2;             // FREEZE / TRIG: <64 vs >=64
                    }
                    break;
                case 2: // Plaits
                    if (cc == 23) return 98;  // ENGINE: 0..97
                    if (cc == 24) return 3;   // AUX: 0..2
                    if (cc == 64) {
                        wireIsBool127 = true;
                        return 2;             // TRIG
                    }
                    break;
                case 3: // Rings
                    if (cc == 18) return 6;   // MODEL/FX: 0..5
                    if (cc == 20) return 5;   // MODE: 0..4
                    if (cc == 21) return 3;   // WAVE: 0..2
                    if (cc == 22) return 3;   // CHORD TB: 0..2
                    if (cc == 23) return 11;  // CHORD: 0..10
                    if (cc == 64) {
                        wireIsBool127 = true;
                        return 2;             // STRUM
                    }
                    break;
            }
            return 0;
        }

        uint8_t RackUI::WireToPotValue(uint8_t page, uint8_t mod,
                                       uint8_t slot, uint8_t cc,
                                       uint8_t wireVal) {
            if (page <= 1) {
                bool wireBool = false;
                const uint8_t n = p1p2DiscreteCount(mod, cc, wireBool);
                if (n > 1) {
                    if (wireBool) return wireVal >= 64 ? 127 : 0;
                    const uint8_t clamped = wireVal < n ? wireVal : (uint8_t)(n - 1);
                    return (uint8_t)(((uint16_t)clamped * 127 + (n - 1) / 2) / (n - 1));
                }
                return wireVal;
            }
            if ((page == 2 || page == 3 || page == 6) && (slot == 4 || slot == 14)) {
                // LFO/ENV MODE: 0 or 127
                return wireVal >= 64 ? 127 : 0;
            }
            if (page == 7) {
                if (slot < 4) return (mod == slot) ? 127 : 0;
                if (slot == 4) {
                    // CHAIN PR: wire is 1..3
                    if (wireVal <= 1) return 0;
                    if (wireVal == 2) return 64;
                    return 127;
                }
                if (slot == 5 || slot == 7 || slot == 14 || slot == 15) {
                    return wireVal ? 127 : 0;
                }
                if (slot == 6) {
                    return wireVal;
                }
            }
            return wireVal;
        }

        uint8_t RackUI::PotToWireValue(uint8_t page, uint8_t mod,
                                       uint8_t slot, uint8_t cc,
                                       uint8_t potVal) {
            (void)slot;
            if (page <= 1) {
                bool wireBool = false;
                const uint8_t n = p1p2DiscreteCount(mod, cc, wireBool);
                if (n > 1) {
                    if (wireBool) return potVal >= 64 ? 127 : 0;
                    uint8_t idx = (uint8_t)(((uint16_t)potVal * n) / 128);
                    if (idx >= n) idx = (uint8_t)(n - 1);
                    return idx;
                }
            }
            return potVal;
        }

        // Convert 0-127 pot-space value into human-readable display number
        // and bar fill (barVal / barMax) so every stepped/range parameter
        // uses 100% of the pot turn and shows its true range.
        static int potToDisplayValue(uint8_t page, uint8_t mod, int slot,
                                     uint8_t cc, uint8_t potVal,
                                     int &barVal, int &barMax) {
            barVal = potVal;
            barMax = 127;
            if (page <= 1) {
                bool wireBool = false;
                const uint8_t n = p1p2DiscreteCount(mod, cc, wireBool);
                if (n > 1) {
                    int idx = ((int)potVal * n) / 128;
                    if (idx >= n) idx = n - 1;
                    barVal = idx;
                    barMax = n - 1;
                    return idx;
                }
                if ((mod & 3) == 0 && cc == 1) {
                    // Elements PITCH: -24..+24 st (potVal 64 = 0)
                    int v = ((int)potVal * 48 + 63) / 127 - 24;
                    barVal = potVal;
                    barMax = 127;
                    return v;
                }
                if ((mod & 3) == 1 && cc == 13) {
                    // Clouds EX SRC: 5 exciter modes (0..4)
                    int idx = ((int)potVal * 5) / 128;
                    if (idx > 4) idx = 4;
                    barVal = idx;
                    barMax = 4;
                    return idx;
                }
                if ((mod & 3) == 3 && cc == 19) {
                    // Rings POLY: 1..4
                    int idx = potVal / 32;
                    if (idx > 3) idx = 3;
                    barVal = idx;
                    barMax = 3;
                    return 1 + idx;
                }
                return potVal;
            }
            if (page == 2 || page == 3) {
                // MOD-LFO: WAVE (1, 11) -> 5 waves (0..4)
                if (slot == 1 || slot == 11) {
                    int idx = ((int)potVal * 5) / 128;
                    if (idx > 4) idx = 4;
                    barVal = idx;
                    barMax = 4;
                    return idx;
                }
                // MODE (4, 14) -> 0..1 (Hz vs Clocked)
                if (slot == 4 || slot == 14) {
                    int v = potVal >= 64 ? 1 : 0;
                    barVal = v;
                    barMax = 1;
                    return v;
                }
                // DIV (5, 15) -> 10 divisions (0..9)
                if (slot == 5 || slot == 15) {
                    int v = ((int)potVal * 10) / 128;
                    if (v > 9) v = 9;
                    barVal = v;
                    barMax = 9;
                    return v;
                }
                return potVal;
            }
            if (page == 4) {
                // MOD-TRIG: E1/E2 STPS (0, 10) -> 1..32
                if (slot == 0 || slot == 10) {
                    int v = 1 + ((int)potVal * 31 + 63) / 127;
                    barVal = v - 1;
                    barMax = 31;
                    return v;
                }
                // E1/E2 PULS (1, 11) -> 0..32
                if (slot == 1 || slot == 11) {
                    int v = ((int)potVal * 32 + 63) / 127;
                    barVal = v;
                    barMax = 32;
                    return v;
                }
                // E1/E2 ROT (2, 12) -> 0..31
                if (slot == 2 || slot == 12) {
                    int v = ((int)potVal * 31 + 63) / 127;
                    barVal = v;
                    barMax = 31;
                    return v;
                }
                // E1/E2 CLK (3, 13) -> 0..2
                if (slot == 3 || slot == 13) {
                    int v = ((int)potVal * 3) / 128;
                    if (v > 2) v = 2;
                    barVal = v;
                    barMax = 2;
                    return v;
                }
                // E1/E2 DIV (4, 14) -> 0..9
                if (slot == 4 || slot == 14) {
                    int v = ((int)potVal * 10) / 128;
                    if (v > 9) v = 9;
                    barVal = v;
                    barMax = 9;
                    return v;
                }
                return potVal;
            }
            if (page == 5) {
                // MOD-GATE: G1/G2 LEN (0, 10) -> 1..32
                if (slot == 0 || slot == 10) {
                    int v = 1 + ((int)potVal * 31 + 63) / 127;
                    barVal = v - 1;
                    barMax = 31;
                    return v;
                }
                // G1/G2 MASK (1, 11) -> 0..32
                if (slot == 1 || slot == 11) {
                    int v = ((int)potVal * 32 + 63) / 127;
                    barVal = v;
                    barMax = 32;
                    return v;
                }
                // G1/G2 CLK (3, 13) -> 0..2
                if (slot == 3 || slot == 13) {
                    int v = ((int)potVal * 3) / 128;
                    if (v > 2) v = 2;
                    barVal = v;
                    barMax = 2;
                    return v;
                }
                // G1/G2 DIV (4, 14) -> 0..9
                if (slot == 4 || slot == 14) {
                    int v = ((int)potVal * 10) / 128;
                    if (v > 9) v = 9;
                    barVal = v;
                    barMax = 9;
                    return v;
                }
                return potVal;
            }
            if (page == 6) {
                // MOD-ENV: A1/A2 MODE (4, 14) -> 0..1 (gate / retrig)
                if (slot == 4 || slot == 14) {
                    int v = potVal >= 64 ? 1 : 0;
                    barVal = v;
                    barMax = 1;
                    return v;
                }
                // A1/A2 DIV (5, 15) -> 0..9
                if (slot == 5 || slot == 15) {
                    int v = ((int)potVal * 10) / 128;
                    if (v > 9) v = 9;
                    barVal = v;
                    barMax = 9;
                    return v;
                }
                return potVal;
            }
            if (page == 7) {
                // SYS: 0..3 modules (0..1), 4 CHAIN PR (1..3), 5 CHAIN ON (0..1),
                // 6 BPM (20..300), 7 CLKSRC (0..1), 8 PRESET (0..3),
                // 14 TELEM D (0..1), 15 TELEM F (0..1)
                if (slot == 4) {
                    int v = 1 + ((int)potVal * 3) / 128;
                    if (v > 3) v = 3;
                    barVal = v - 1;
                    barMax = 2;
                    return v;
                }
                if (slot == 6) {
                    int v = 20 + ((int)potVal * 280 + 63) / 127;
                    barVal = potVal;
                    barMax = 127;
                    return v;
                }
                if (slot == 7) {
                    int v = potVal >= 64 ? 1 : 0;
                    barVal = v;
                    barMax = 1;
                    return v;
                }
                if (slot == 8) {
                    int v = ((int)potVal * 4) / 128;
                    if (v > 3) v = 3;
                    barVal = v;
                    barMax = 3;
                    return v;
                }
                int v = potVal >= 64 ? 1 : 0;
                barVal = v;
                barMax = 1;
                return v;
            }
            return potVal;
        }

        // ===================== rendering =====================
        static void renderStrip() {
            Display::FillRect(0, 0, Display::W, STRIP_H, Display::kBlack);
            int x = 2;
            // module + page (TELEM header; dashes before the first snapshot; SYS is global)
            char buf[24];
            if (s_page == 7) {
                snprintf(buf, sizeof(buf), "SYSTEM");
            } else if (s_page >= 2 && s_page <= 6) {
                snprintf(buf, sizeof(buf), "%s", kPageNames[s_page]);
            } else if (s_cur.valid) {
                snprintf(buf, sizeof(buf), "%s %s",
                         kModNames[s_cur.module & 3], kPageNames[s_page]);
            } else {
                snprintf(buf, sizeof(buf), "--- %s", kPageNames[s_page]);
            }
            Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                Display::kWhite);
            x += Display::StringWidth(buf, Display::FONT_VALUE) + 6;
            // clock source / BPM from the status tail (0 = external MIDI).
            if (s_cur.tail_valid) {
                if (s_cur.clocksrc == 1) {
                    snprintf(buf, sizeof(buf), "MIDI");
                } else {
                    snprintf(buf, sizeof(buf), "INT %u", (unsigned)s_cur.bpm);
                }
                Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                    Display::kTeal);
                x += Display::StringWidth(buf, Display::FONT_VALUE) + 6;
                // chain state: off / pair index.
                const char *pairs[4] = { "OFF", "E>C", "P>C", "R>C" };
                snprintf(buf, sizeof(buf), "CHN %s", pairs[s_cur.chain & 3]);
                Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                    s_cur.chain ? Display::kYellow
                                                : Display::kWhite);
                x += Display::StringWidth(buf, Display::FONT_VALUE) + 6;
            }
            // assign-mode / overlay / active-mod-source indicator
            if (s_overlay) {
                snprintf(buf, sizeof(buf), ">%s<", kModNames[s_overlaySel & 3]);
                Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                    Display::kYellow);
            } else if (s_assignArmed) {
                if (s_assignPot < 16) {
                    snprintf(buf, sizeof(buf), ">MOD %s<",
                             kSrcNames[s_assignSrc <= 10 ? s_assignSrc : 0]);
                } else {
                    snprintf(buf, sizeof(buf), ">MOD<");
                }
                Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                    Display::kYellow);
            } else if (s_page <= 1 && s_activePot < 16 && s_fullArrived) {
                const uint8_t rsrc = (uint8_t)(
                    (s_cur.cell[s_activePot].flags & RackLink::kFlagSrcMask) >>
                    RackLink::kFlagSrcShift);
                if (rsrc > 0 && rsrc <= 10) {
                    snprintf(buf, sizeof(buf), "MOD:%s", kSrcNames[rsrc]);
                    Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                        Display::kYellow);
                }
            }
            // BLE + MIDI activity dot at the right edge.
            if (DRIVERS::BtMidiReceiver::IsConnected()) {
                Display::DrawString(Display::W - 24, 1, "BLE",
                                    Display::FONT_VALUE, Display::kTeal);
            }
            const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            if (RackLink::LastMidiMs() != 0 && now - RackLink::LastMidiMs() < 250) {
                Display::FillRect(Display::W - 4, 4, 3, 3, Display::kYellow);
            }
            s_stripDirty = false;
        }

        // One pot slot: black clear + label/value/bar.
        static void renderCell(int slot) {
            const int x = cellX(slot), y = cellY(slot), w = cellW(slot);
            Display::FillRect(x, y, w, 19, Display::kBlack);
            CellSnap &snap = s_rendered[slot];
            if (!snap.present) return;          // empty pot: black
            const RackTelemCell &c = s_cur.cell[slot];
            const bool dim = (c.flags & RackLink::kFlagChainB) != 0;
            const int8_t routedSrc =
                (int8_t)((c.flags & RackLink::kFlagSrcMask) >> RackLink::kFlagSrcShift);
            const uint16_t base = classColor(c.color);
            const uint16_t col = dim ? Display::Dim(base) : base;
            const bool blinkOff = (c.flags & RackLink::kFlagInert) != 0 &&
                                  !s_blinkPhase;
            int barVal = 0, barMax = 127;
            const int dispVal = potToDisplayValue(
                s_page, s_cur.module, slot, c.cc, snap.value, barVal, barMax);
            if (!blinkOff) {
                if (s_fullArrived) {
                    // Mod-routed cells read YELLOW as a whole (label too) —
                    // the state accent marks "this pot rides the source's
                    // speed param, it does not write this value".
                    Display::DrawString(x, y + LABEL_DY, snap.label,
                                        Display::FONT_LABEL,
                                        routedSrc ? Display::kYellow : col);
                } else {
                    // No TELEM snapshot yet: dashes, never guesses.
                    Display::DrawString(x, y + LABEL_DY, "---",
                                        Display::FONT_LABEL,
                                        Display::Dim(Display::kWhite));
                }
                char val[10];
                if (s_fullArrived) {
                    int8_t pdir = 0;
                    if (RackPots::PickupPending(slot, pdir)) {
                        if (pdir > 0) {
                            snprintf(val, sizeof(val), "%d>", dispVal);
                        } else {
                            snprintf(val, sizeof(val), "<%d", dispVal);
                        }
                    } else {
                        snprintf(val, sizeof(val), "%d", dispVal);
                    }
                } else {
                    snprintf(val, sizeof(val), "---");
                }
                // Yellow = STATE accent: a mod-routed cell's value text.
                const uint16_t vcol =
                    routedSrc ? Display::kYellow : col;
                Display::DrawString(x, y + VALUE_DY, val,
                                    Display::FONT_VALUE, vcol);
            }
            // 2 px underline bar (scaled across barMax); routed cells in yellow.
            const uint16_t barCol =
                routedSrc ? Display::kYellow
                          : (dim ? Display::Dim(base) : base);
            Display::FillRect(x, y + BAR_DY, w - 2, 2, Display::kBlack);
            if (s_fullArrived && barVal > 0 && barMax > 0) {
                const int fill = (barVal * (w - 2)) / barMax;
                if (fill > 0) {
                    Display::FillRect(x, y + BAR_DY, fill, 2, barCol);
                }
            }
        }

        static void renderPage() {
            for (int slot = 0; slot < 16; ++slot) {
                // SYS pot 8 = the C3-LOCAL preset slot cell (no TELEM cell
                // exists — the RP2350 owns no slot state until T4a).
                if (s_page == 7 && slot == 8 && s_fullArrived) {
                    const uint8_t presetPotVal =
                        (uint8_t)(((uint16_t)s_presetSlot * 127) / 3);
                    int8_t pdir = 0;
                    const uint8_t pick = RackPots::PickupPending(slot, pdir)
                                             ? (uint8_t)(pdir > 0 ? 2 : 1)
                                             : 0;
                    CellSnap &ps = s_rendered[slot];
                    if (s_pageDirty || !ps.present ||
                        ps.value != presetPotVal || ps.pick != pick ||
                        strcmp(ps.label, "PRESET") != 0) {
                        ps.present = true;
                        ps.value = presetPotVal;
                        ps.flags = 0;
                        ps.color = 2;               // teal class
                        ps.pick = pick;
                        snprintf(ps.label, sizeof(ps.label), "PRESET");
                        renderCell(slot);
                    }
                    continue;
                }
                bool present;
                if (!s_fullArrived) {
                    present = true;         // dashes placeholder (no data)
                } else if (slot < s_cur.count) {
                    present = strlen(s_cur.cell[slot].label) > 0;
                } else {
                    present = false;        // real empty pot (count < 16)
                }
                CellSnap &snap = s_rendered[slot];
                const RackTelemCell &c = s_cur.cell[slot];
                int8_t pdir = 0;
                const uint8_t pick = RackPots::PickupPending(slot, pdir)
                                         ? (uint8_t)(pdir > 0 ? 2 : 1)
                                         : 0;
                const bool changed =
                    snap.present != present ||
                    snap.value != c.value || snap.flags != c.flags ||
                    snap.color != c.color || snap.pick != pick ||
                    strcmp(snap.label, c.label) != 0;
                if (!s_pageDirty && !changed) continue;
                snap.present = present;
                snap.value = c.value;
                snap.flags = c.flags;
                snap.color = c.color;
                snap.pick = pick;
                snprintf(snap.label, sizeof(snap.label), "%s", c.label);
                renderCell(slot);
            }
            // SYS page: the push-ring marker (yellow 1-px underline under
            // the cell the NEXT short push will act on).
            if (s_page == 7 && s_fullArrived) {
                for (int slot = 0; slot < 16; ++slot) {
                    // ring: 0 chain-pair(cell 4) 1 engage(5) 2 preset(8,
                    // C3-local) 3 telem-d(14) 4 telem-f(15)
                    static const int ringCell[5] = { 4, 5, 8, 14, 15 };
                    const bool marked = ringCell[s_sysRing] == slot;
                    const int x = cellX(slot), y = cellY(slot);
                    Display::FillRect(x, y + BAR_DY + 3, cellW(slot) - 2, 1,
                                      marked ? Display::kYellow
                                             : Display::kBlack);
                }
            }
            s_pageDirty = false;
        }

        // ===================== page / gesture actions =====================
        static const uint8_t kPairA[4] = { 0, 0, 2, 3 };  // 1=elem 2=plaits 3=rings
        static constexpr uint8_t kChainB = 1;             // clouds in all pairs

        static void sysRingAction() {
            // Engaged flag + configured pair from the cached SYS cells.
            const bool engaged = s_fullArrived && s_cur.count > 5 &&
                                 s_cur.cell[5].value >= 64;
            uint8_t pair = s_fullArrived && s_cur.count > 4
                               ? (uint8_t)(1 + ((uint16_t)s_cur.cell[4].value * 3) / 128)
                               : 1;
            if (pair < 1 || pair > 3) pair = 1;
            switch (s_sysRing) {
                case 0: {  // chain pair cycle (engage state preserved)
                    const uint8_t next = (uint8_t)(pair % 3 + 1);
                    if (engaged) {
                        RackLink::SendChainRoute(kPairA[next], kChainB, 0);
                        RackLink::SendChainRoute(kPairA[next], kChainB, 1);
                    } else {
                        RackLink::SendChainRoute(kPairA[next], kChainB, 0);
                    }
                    break;
                }
                case 1:  // chain engage toggle
                    RackLink::SendChainRoute(kPairA[pair], kChainB,
                                             engaged ? 0 : 1);
                    break;
                case 2:  // preset slot cycle + PRESET_REQ (T4a stub)
                    s_presetSlot = (uint8_t)((s_presetSlot + 1) & 3);
                    RackLink::SendPresetReq(s_presetSlot);
                    s_pageDirty = true;
                    break;
                case 3: RackLink::SendSysToggle(RackLink::kSysTelemDelta); break;
                case 4: RackLink::SendSysToggle(RackLink::kSysTelemFull); break;
            }
            s_sysRing = (uint8_t)((s_sysRing + 1) % 5);
            s_pendingRefreshUs = esp_timer_get_time() + 45000;
        }

        void RackUI::GetPotBucketBounds(uint8_t pot, const PotTarget &t,
                                        uint8_t &lo, uint8_t &hi) {
            lo = t.value;
            hi = t.value;
            uint8_t n = 0;
            if (s_page <= 1) {
                bool wireBool = false;
                n = p1p2DiscreteCount(s_cur.module, t.cc, wireBool);
                if (n == 0 && (s_cur.module & 3) == 1 && t.cc == 13) n = 5;
                if (n == 0 && (s_cur.module & 3) == 3 && t.cc == 19) n = 4;
                if (n > 12) n = 0; // 98-engine selector behaves as continuous
            } else if (s_page == 2 || s_page == 3) {
                if (pot == 1 || pot == 11) n = 5;    // LFO WAVE (0..4)
                if (pot == 4 || pot == 14) n = 2;    // LFO MODE (0..1)
                if (pot == 5 || pot == 15) n = 10;   // LFO DIV (0..9)
            } else if (s_page == 4 || s_page == 5) {
                if (pot == 3 || pot == 13) n = 3;    // CLK (0..2)
                if (pot == 4 || pot == 14) n = 10;   // DIV (0..9)
            } else if (s_page == 6) {
                if (pot == 4 || pot == 14) n = 2;    // ADSR MODE (0..1)
                if (pot == 5 || pot == 15) n = 10;   // ADSR DIV (0..9)
            } else if (s_page == 7) {
                if (pot == 4) n = 3;          // CHAIN PR (1..3)
                else if (pot == 8) n = 4;     // PRESET (0..3)
                else if (pot < 4 || pot == 5 || pot == 7 || pot == 14 || pot == 15) n = 2; // Modules 0..3, CHAIN ON 5, CLKSRC 7, TELEM D 14, TELEM F 15
            }
            if (n >= 2 && n <= 12) {
                int k = ((int)t.value * n) / 128;
                if (k >= n) k = n - 1;
                int bLo = (k * 128 + n - 1) / n;
                int bHi = ((k + 1) * 128 - 1) / n;
                if (k > 0 && bLo + 2 <= bHi) bLo += 2;
                if (k < n - 1 && bHi - 2 >= bLo) bHi -= 2;
                lo = (uint8_t)(bLo < 0 ? 0 : bLo);
                hi = (uint8_t)(bHi > 127 ? 127 : bHi);
            }
        }

        uint8_t RackUI::GetPage() { return s_page; }
        uint8_t RackUI::GetModule() { return s_cur.module; }

        bool RackUI::AssignModeActive() { return s_assignArmed; }

        void RackUI::NoteActivePot(uint8_t pot) {
            if (pot < 16 && s_activePot != pot) {
                s_activePot = pot;
                s_stripDirty = true;
            }
        }

        void RackUI::NoteAssignPot(uint8_t pot, uint8_t srcIdxPlus1) {
            s_assignPot = pot;
            s_assignSrc = srcIdxPlus1;
            s_activePot = pot;
            s_stripDirty = true;
        }

        void RackUI::SetPage(uint8_t page) {
            if (page > 7 || page == s_page) return;
            s_page = page;
            s_fullArrived = false;      // dashes until the snapshot lands
            s_activePot = 0xFF;
            s_assignPot = 0xFF;
            for (int i = 0; i < 16; ++i) s_lastWireSent[i] = 0xFFFF;
            s_pageDirty = true;
            s_stripDirty = true;
            RackLink::SendPageRequest(page);
        }

        bool RackUI::GetPotTarget(uint8_t pot, PotTarget &out) {
            out.kind = PotTarget::kNone;
            out.value = 0;
            out.routedSrc = 0;
            if (pot > 15) return false;
            if (s_page <= 1) {
                // P1/P2: the ACTIVE page cell's CC (TELEM-driven). routedSrc
                // rides the TELEM flags bits 3-6 (source id + 1).
                if (!s_fullArrived || pot >= s_cur.count) return false;
                out.kind = PotTarget::kCC;
                out.cc = s_cur.cell[pot].cc;
                out.value = s_cur.cell[pot].value;
                out.routedSrc = (uint8_t)(
                    (s_cur.cell[pot].flags & RackLink::kFlagSrcMask) >>
                    RackLink::kFlagSrcShift);
                return true;
            }
            if (s_page >= 2 && s_page <= 6) {
                const SrcParam &sp = kModPotMap[s_page - 2][pot];
                if (sp.src == 0xFF) return false;
                out.kind = PotTarget::kMod;
                out.src = sp.src;
                out.param = sp.param;
                out.value = s_fullArrived && pot < s_cur.count
                                ? s_cur.cell[pot].value : 0;
                return true;
            }
            if (s_page == 7) {
                if (!s_fullArrived) return false;
                if (pot < 6 && pot < s_cur.count) {
                    out.kind = PotTarget::kSys;
                    out.cc = pot;
                    out.value = s_cur.cell[pot].value;
                    return true;
                }
                if (pot == 6 && pot < s_cur.count) {
                    out.kind = PotTarget::kGlobal;
                    out.cc = 0;             // item 0 = BPM (kUiGlobalBpm)
                    out.value = s_cur.cell[pot].value;
                    return true;
                }
                if (pot == 7 && pot < s_cur.count) {
                    out.kind = PotTarget::kSys;
                    out.cc = 7;            // CLKSRC on SYS
                    out.value = s_cur.cell[pot].value;
                    return true;
                }
                if (pot == 8) {
                    out.kind = PotTarget::kSys;
                    out.cc = 8;
                    out.value = (uint8_t)(((uint16_t)s_presetSlot * 127) / 3);
                    return true;
                }
                if ((pot == 14 || pot == 15) && pot < s_cur.count) {
                    out.kind = PotTarget::kSys;
                    out.cc = pot;          // TELEM D (14), TELEM F (15)
                    out.value = s_cur.cell[pot].value;
                    return true;
                }
            }
            return false;
        }

        void RackUI::ApplyPotAction(uint8_t pot, const PotTarget &t,
                                    uint8_t potVal) {
            NoteActivePot(pot);
            switch (t.kind) {
                case PotTarget::kCC: {
                    const uint8_t wireVal =
                        PotToWireValue(s_page, s_cur.module, pot, t.cc, potVal);
                    RackLink::TelemLocalCellWrite(pot, potVal);
                    if (s_lastWireSent[pot] != wireVal) {
                        s_lastWireSent[pot] = wireVal;
                        const uint8_t pkt[4] = { 0x0B, 0xB0, t.cc, wireVal };
                        RackLink::SendMidi(pkt);
                    }
                    break;
                }
                case PotTarget::kMod: {
                    RackLink::TelemLocalCellWrite(pot, potVal);
                    RackLink::SendModSet(t.src, t.param, potVal);
                    break;
                }
                case PotTarget::kGlobal: {
                    RackLink::TelemLocalCellWrite(pot, potVal);
                    RackLink::SendUiCmd(RackLink::kCmdSetGlobal, t.cc, potVal);
                    s_stripDirty = true;
                    break;
                }
                case PotTarget::kSys: {
                    if (s_page == 7 && pot == 7) {
                        // SYS CLKSRC (0 = INT, 1 = MIDI)
                        const uint8_t wantClk = potVal >= 64 ? 1 : 0;
                        RackLink::TelemLocalCellWrite(pot, potVal);
                        if (s_lastWireSent[pot] != wantClk) {
                            s_lastWireSent[pot] = wantClk;
                            if ((s_cur.clocksrc != 0) != (wantClk != 0)) {
                                RackLink::SendSysToggle(
                                    RackLink::kSysClockToggle);
                                s_pendingRefreshUs =
                                    esp_timer_get_time() + 45000;
                            }
                        }
                        break;
                    }
                    if (s_page == 7) {
                        if (pot < 4) {
                            // Module selector radio buttons (0..3): turning
                            // any module pot >= 64 activates that module;
                            // turning the already-active module pot down does
                            // not deactivate it (one module is always active).
                            const uint8_t wantOn = potVal >= 64 ? 1 : 0;
                            if (wantOn) {
                                if (s_lastWireSent[pot] != 1) {
                                    s_lastWireSent[pot] = 1;
                                    if (s_cur.module != pot) {
                                        RackLink::TelemLocalSysSwitch(pot);
                                        RackLink::SendSwitch(pot);
                                        s_pendingRefreshUs =
                                            esp_timer_get_time() + 45000;
                                    } else {
                                        RackLink::TelemLocalCellWrite(pot, 127);
                                    }
                                }
                            } else {
                                s_lastWireSent[pot] = 0;
                                // Keep active module cell at 127, inactive at 0
                                RackLink::TelemLocalCellWrite(
                                    pot, (s_cur.module == pot) ? 127 : 0);
                            }
                        } else if (pot == 4) {
                            uint8_t pair =
                                (uint8_t)(1 + ((uint16_t)potVal * 3) / 128);
                            if (pair > 3) pair = 3;
                            RackLink::TelemLocalCellWrite(pot, potVal);
                            if (s_lastWireSent[pot] != pair) {
                                s_lastWireSent[pot] = pair;
                                const bool engaged =
                                    s_cur.count > 5 &&
                                    s_cur.cell[5].value >= 64;
                                if (engaged) {
                                    RackLink::SendChainRoute(
                                        kPairA[pair], kChainB, 0);
                                    RackLink::SendChainRoute(
                                        kPairA[pair], kChainB, 1);
                                } else {
                                    RackLink::SendChainRoute(
                                        kPairA[pair], kChainB, 0);
                                }
                                s_pendingRefreshUs =
                                    esp_timer_get_time() + 45000;
                            }
                        } else if (pot == 5) {
                            const uint8_t wantOn = potVal >= 64 ? 1 : 0;
                            RackLink::TelemLocalCellWrite(pot, potVal);
                            if (s_lastWireSent[pot] != wantOn) {
                                s_lastWireSent[pot] = wantOn;
                                uint8_t pair = (uint8_t)(
                                    1 + ((uint16_t)s_cur.cell[4].value * 3) / 128);
                                if (pair > 3) pair = 3;
                                RackLink::SendChainRoute(
                                    kPairA[pair], kChainB, wantOn);
                                s_pendingRefreshUs =
                                    esp_timer_get_time() + 45000;
                            }
                        } else if (pot == 14) {
                            const uint8_t wantOn = potVal >= 64 ? 1 : 0;
                            const uint8_t curOn =
                                s_cur.cell[14].value >= 64 ? 1 : 0;
                            RackLink::TelemLocalCellWrite(pot, potVal);
                            if (s_lastWireSent[pot] != wantOn) {
                                s_lastWireSent[pot] = wantOn;
                                if (wantOn != curOn) {
                                    RackLink::SendSysToggle(
                                        RackLink::kSysTelemDelta);
                                    s_pendingRefreshUs =
                                        esp_timer_get_time() + 45000;
                                }
                            }
                        } else if (pot == 15) {
                            const uint8_t wantOn = potVal >= 64 ? 1 : 0;
                            const uint8_t curOn =
                                s_cur.cell[15].value >= 64 ? 1 : 0;
                            RackLink::TelemLocalCellWrite(pot, potVal);
                            if (s_lastWireSent[pot] != wantOn) {
                                s_lastWireSent[pot] = wantOn;
                                if (wantOn != curOn) {
                                    RackLink::SendSysToggle(
                                        RackLink::kSysTelemFull);
                                    s_pendingRefreshUs =
                                        esp_timer_get_time() + 45000;
                                }
                            }
                        } else if (pot == 8) {
                            uint8_t slotIdx =
                                (uint8_t)(((uint16_t)potVal * 4) / 128);
                            if (slotIdx > 3) slotIdx = 3;
                            if (s_presetSlot != slotIdx) {
                                s_presetSlot = slotIdx;
                                RackLink::SendPresetReq(s_presetSlot);
                                s_pageDirty = true;
                            }
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }

        void RackUI::DebugState() {
            if (!s_fullArrived) {
                ESP_LOGI(TAG, "page %s: no TELEM snapshot yet (dashes)",
                         kPageNames[s_page]);
                return;
            }
            ESP_LOGI(TAG, "page %s mod=%u cells=%u tail(bpm=%u clk=%u "
                          "chain=%u link=%02x)", kPageNames[s_page],
                     s_cur.module, s_cur.count, (unsigned)s_cur.bpm,
                     s_cur.clocksrc, s_cur.chain, s_cur.link);
            for (uint8_t i = 0; i < s_cur.count && i < 16; ++i) {
                ESP_LOGI(TAG, "  [%2u] cc=%3u val=%3u flg=%02x col=%u %s",
                         i, s_cur.cell[i].cc, s_cur.cell[i].value,
                         s_cur.cell[i].flags, s_cur.cell[i].color,
                         s_cur.cell[i].label);
            }
        }

        void RackUI::Init() {
            memset(&s_cur, 0, sizeof(s_cur));
            memset(s_rendered, 0, sizeof(s_rendered));
            for (int i = 0; i < 16; ++i) s_lastWireSent[i] = 0xFFFF;
            Display::Clear(Display::kBlack);
            renderStrip();
            renderPage();
            Display::Flush();
            // First snapshot: ask for P1 (the link task also probes while
            // the link is unconfirmed — this is the UI's own request).
            RackLink::SendPageRequest(s_page);
        }

        void RackUI::TaskFunction(void *) {
            while (true) {
                // ---- input ----
                InputEvent ev;
                while (UserInput::GetEvent(ev, 0)) {
                    switch (ev.type) {
                        case InputEvent::ENC_DELTA:
                            if (s_assignArmed) {
                                // hold + rotate = module-select overlay
                                s_overlay = true;
                                s_overlaySel = (uint8_t)(
                                    (s_overlaySel + ev.delta + 4) & 3);
                                s_stripDirty = true;
                            } else {
                                int p = (int)s_page;
                                int steps = ev.delta;
                                while (steps > 0) {
                                    p++;
                                    if (p == 1 && !ModuleHasP2(s_cur.module)) {
                                        p++; // skip empty P2
                                    }
                                    if (p > 7) { p = 7; break; }
                                    steps--;
                                }
                                while (steps < 0) {
                                    p--;
                                    if (p == 1 && !ModuleHasP2(s_cur.module)) {
                                        p--; // skip empty P2
                                    }
                                    if (p < 0) { p = 0; break; }
                                    steps++;
                                }
                                if (p != (int)s_page) {
                                    SetPage((uint8_t)p);
                                }
                            }
                            break;
                        case InputEvent::PUSH_HOLD:
                            s_assignArmed = true;
                            s_assignPot = 0xFF;
                            s_stripDirty = true;
                            break;
                        case InputEvent::PUSH_SHORT:
                            if (s_page == 7) {
                                sysRingAction();
                            }
                            break;
                        case InputEvent::PUSH_RELEASE:
                            if (s_overlay) {
                                // Release CONFIRMS the selected module.
                                RackLink::TelemLocalSysSwitch(s_overlaySel);
                                RackLink::SendSwitch(s_overlaySel);
                                s_pendingRefreshUs =
                                    esp_timer_get_time() + 45000;
                                s_overlay = false;
                                s_assignArmed = false;
                                if (s_page == 1 && !ModuleHasP2(s_overlaySel)) {
                                    SetPage(0);
                                }
                            } else {
                                s_assignArmed = false;   // release cancels
                            }
                            s_assignPot = 0xFF;
                            s_stripDirty = true;
                            break;
                    }
                }

                if (s_pendingRefreshUs != 0 &&
                    esp_timer_get_time() >= s_pendingRefreshUs) {
                    s_pendingRefreshUs = 0;
                    RackLink::SendPageRequest(s_page);
                }

                // ---- render (30-60 Hz; test patterns own the screen) ----
                if (!Display::TestPatternActive()) {
                    // Poll the TELEM cache.
                    RackTelemPage tmp;
                    if (RackLink::TelemGet(tmp) && tmp.page == s_page) {
                        if (!s_bootModCleaned) {
                            s_bootModCleaned = true;
                            RackPots::ClearAllModSources();
                            s_pendingRefreshUs = esp_timer_get_time() + 30000;
                        }
                        const bool wasFull =
                            tmp.mode == RackLink::kTelemFull;
                        if (wasFull && s_page <= 1) {
                            for (uint8_t i = 0; i < 16 && i < tmp.count; ++i) {
                                const uint8_t rsrc = (uint8_t)(
                                    (tmp.cell[i].flags & RackLink::kFlagSrcMask) >>
                                    RackLink::kFlagSrcShift);
                                if (rsrc > 0) {
                                    RackPots::ClearResidualUnmap(i, rsrc);
                                }
                            }
                        }
                        const bool pageOrModChanged =
                            !s_fullArrived || tmp.page != s_cur.page ||
                            (s_page <= 1 && tmp.module != s_cur.module);
                        if (pageOrModChanged) {
                            s_cur = tmp;
                            if (s_page == 1 && !ModuleHasP2(s_cur.module)) {
                                SetPage(0);
                            }
                            if (wasFull) s_fullArrived = true;
                            s_pageDirty = true;
                            // Fresh page state: recompute pot pickup for
                            // every pot (page switch / module change).
                            RackPots::PickupResync();
                        } else {
                            // Delta or periodic same-page refresh: only
                            // changed cells matter; our own active pot turns
                            // never arm pending (RackPots.hpp).
                            for (uint8_t i = 0; i < 16 && i < tmp.count; ++i) {
                                if (tmp.cell[i].value != s_cur.cell[i].value ||
                                    tmp.cell[i].flags != s_cur.cell[i].flags) {
                                    RackPots::PickupCellNote(
                                        i, tmp.cell[i].value);
                                }
                            }
                            s_cur = tmp;
                            if (wasFull) s_fullArrived = true;
                            s_pageDirty = true;
                        }
                        s_stripDirty = true;
                        RackLink::TelemClearDirty();
                    }
                    // Pickup arrows also flip from the pot task (alignment
                    // crossing) with no TELEM traffic — per-pass diff.
                    for (uint8_t slot = 0; slot < 16; ++slot) {
                        if (!s_rendered[slot].present) continue;
                        int8_t pdir = 0;
                        const uint8_t pick =
                            RackPots::PickupPending(slot, pdir)
                                ? (uint8_t)(pdir > 0 ? 2 : 1)
                                : 0;
                        if (s_rendered[slot].pick != pick) {
                            s_rendered[slot].pick = pick;
                            renderCell(slot);
                        }
                    }
                    // 2 Hz blink phase for inert cells.
                    const bool phase =
                        ((esp_timer_get_time() / 500000) & 1) != 0;
                    if (phase != s_blinkPhase) {
                        s_blinkPhase = phase;
                        s_pageDirty = true;
                    }
                    if (s_pageDirty) renderPage();
                    if (s_stripDirty) renderStrip();
                    Display::Flush();
                }

                vTaskDelay(pdMS_TO_TICKS(16));    // ~60 Hz
            }
        }

    } // namespace CTRL
} // namespace CTAG
