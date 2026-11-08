/***************
Rack UI controller — fixed-page renderer + gestures (see RackUI.hpp).
Renders ONLY from the RackLink TELEM cache + local UI state ("the firmware
acks drive the UI" — the C3 never invents values; missing TELEM = dashes).
***************/

#include "RackUI.hpp"

#include "Display.hpp"
#include "RackLink.hpp"
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
        static const SrcParam kModPotMap[3][16] = {
            {   // MOD-A: 4x LFO (rate, level, wave, phase per source)
                {0, 1}, {0, 4}, {0, 2}, {0, 3},
                {1, 1}, {1, 4}, {1, 2}, {1, 3},
                {2, 1}, {2, 4}, {2, 2}, {2, 3},
                {3, 1}, {3, 4}, {3, 2}, {3, 3},
            },
            {   // MOD-B: 4x EUclid + gate1 len/glen + adsr1 mode + div (EU1)
                {4, 0}, {4, 1}, {4, 2},
                {5, 0}, {5, 1}, {5, 2},
                {6, 0}, {6, 1}, {6, 2},
                {7, 0}, {7, 1}, {7, 2},
                {8, 0}, {8, 2}, {10, 0}, {4, 4},
            },
            {   // MOD-C: gate2 + ADSR1/2 + BPM pot (UI_CMD SET_GLOBAL) +
                // CLKSRC (push toggles, pot inert)
                {9, 0}, {9, 1}, {9, 2},
                {10, 1}, {10, 2}, {10, 3}, {10, 4},
                {11, 1}, {11, 2}, {11, 3}, {11, 4},
                kNoSet /* BPM -> kGlobal */, kNoSet /* CLKSRC: push only */,
                kNoSet, kNoSet, kNoSet,
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

        // Cell rects: rows 1/3 have 6 cells (~46 px), row 2 has 4 (~70 px).
        static int cellX(int slot) {         // slot 0-15 in pot order
            if (slot < 6) return 2 + slot * 47;
            if (slot < 10) return 1 + (slot - 6) * 71;
            return 2 + (slot - 10) * 47;
        }
        static int cellW(int slot) {
            return (slot >= 6 && slot < 10) ? 70 : 46;
        }
        static int cellY(int slot) {
            return ROW_Y[slot < 6 ? 0 : (slot < 10 ? 1 : 2)];
        }

        // Class byte -> RGB565.
        static uint16_t classColor(uint8_t cls) {
            switch (cls) {
                case 1: return Display::kRed;
                case 2: return Display::kTeal;
                default: return Display::kWhite;
            }
        }

        // ===================== state =====================
        static uint8_t s_page = 0;            // 0-5
        static bool s_assignArmed = false;    // push held (assign/overlay)
        static bool s_overlay = false;        // module-select overlay live
        static uint8_t s_overlaySel = 0;      // 0-3
        static bool s_fullArrived = false;    // TELEM snapshot for s_page
        static uint8_t s_presetSlot = 0;      // SYS: C3-local selection
        static uint8_t s_sysRing = 0;         // SYS push-ring position
        static bool s_blinkPhase = false;     // 2 Hz inert-cell blink

        static RackTelemPage s_cur;          // last TELM copy (renderer)
        static bool s_stripDirty = true;
        static bool s_pageDirty = true;

        // Rendered-cell diff snapshot (skip unchanged cells).
        struct CellSnap {
            uint8_t value, flags, color;
            char label[16];
            bool present;
        };
        static CellSnap s_rendered[16];

        static const char *kPageNames[6] = {
            "P1", "P2", "MOD-A", "MOD-B", "MOD-C", "SYS",
        };
        static const char *kModNames[4] = { "ELEM", "CLDS", "PLTS", "RNGS" };

        // ===================== rendering =====================
        static void renderStrip() {
            Display::FillRect(0, 0, Display::W, STRIP_H, Display::kBlack);
            int x = 2;
            // module + page (TELEM header; dashes before the first snapshot)
            char buf[24];
            if (s_cur.valid) {
                snprintf(buf, sizeof(buf), "%s %s",
                         kModNames[s_cur.module & 3], kPageNames[s_page]);
            } else {
                snprintf(buf, sizeof(buf), "--- %s", kPageNames[s_page]);
            }
            Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                Display::kWhite);
            x += 44;
            // clock source / BPM from the status tail (0 = external MIDI).
            if (s_cur.tail_valid) {
                if (s_cur.clocksrc == 1) {
                    Display::DrawString(x, 1, "MIDI", Display::FONT_VALUE,
                                        Display::kTeal);
                    x += 26;
                } else {
                    snprintf(buf, sizeof(buf), "INT %u", s_cur.bpm);
                    Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                        Display::kTeal);
                    x += 42;
                }
                // chain state: off / pair index.
                const char *pairs[4] = { "OFF", "E>C", "P>C", "R>C" };
                snprintf(buf, sizeof(buf), "CHN %s", pairs[s_cur.chain & 3]);
                Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                    s_cur.chain ? Display::kYellow
                                                : Display::kWhite);
                x += 34;
            }
            // assign-mode / overlay indicator (the yellow STATE accent).
            if (s_overlay) {
                snprintf(buf, sizeof(buf), ">%s<", kModNames[s_overlaySel & 3]);
                Display::DrawString(x, 1, buf, Display::FONT_VALUE,
                                    Display::kYellow);
            } else if (s_assignArmed) {
                Display::DrawString(x, 1, ">MOD<", Display::FONT_VALUE,
                                    Display::kYellow);
            }
            // BLE + MIDI activity dot at the right edge.
            if (DRIVERS::BtMidiReceiver::IsConnected()) {
                Display::DrawString(Display::W - 30, 1, "BLE",
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
            if (!blinkOff) {
                if (s_fullArrived) {
                    Display::DrawString(x, y + LABEL_DY, snap.label,
                                        Display::FONT_LABEL, col);
                } else {
                    // No TELEM snapshot yet: dashes, never guesses.
                    Display::DrawString(x, y + LABEL_DY, "---",
                                        Display::FONT_LABEL,
                                        Display::Dim(Display::kWhite));
                }
                char val[8];
                if (s_fullArrived) {
                    snprintf(val, sizeof(val), "%u", snap.value);
                } else {
                    snprintf(val, sizeof(val), "---");
                }
                // Yellow = STATE accent: a mod-routed cell's value text.
                const uint16_t vcol =
                    routedSrc ? Display::kYellow : col;
                Display::DrawString(x, y + VALUE_DY, val,
                                    Display::FONT_VALUE, vcol);
            }
            // 2 px underline bar (0-127 fill); routed cells bar in yellow.
            const uint16_t barCol =
                routedSrc ? Display::kYellow
                          : (dim ? Display::Dim(base) : base);
            Display::FillRect(x, y + BAR_DY, w - 2, 2, Display::kBlack);
            if (s_fullArrived && snap.value > 0) {
                const int fill = (snap.value * (w - 2)) / 127;
                if (fill > 0) {
                    Display::FillRect(x, y + BAR_DY, fill, 2, barCol);
                }
            }
        }

        static void renderPage() {
            for (int slot = 0; slot < 16; ++slot) {
                // SYS pot 8 = the C3-LOCAL preset slot cell (no TELEM cell
                // exists — the RP2350 owns no slot state until T4a).
                if (s_page == 5 && slot == 8 && s_fullArrived) {
                    CellSnap &ps = s_rendered[slot];
                    if (!ps.present || ps.value != s_presetSlot ||
                        strcmp(ps.label, "PRESET") != 0) {
                        ps.present = true;
                        ps.value = s_presetSlot;    // shown as the slot number
                        ps.flags = 0;
                        ps.color = 2;               // teal class
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
                const bool changed =
                    snap.present != present ||
                    snap.value != c.value || snap.flags != c.flags ||
                    snap.color != c.color ||
                    strcmp(snap.label, c.label) != 0;
                if (!s_pageDirty && !changed) continue;
                snap.present = present;
                snap.value = c.value;
                snap.flags = c.flags;
                snap.color = c.color;
                snprintf(snap.label, sizeof(snap.label), "%s", c.label);
                renderCell(slot);
            }
            // SYS page: the push-ring marker (yellow 1-px underline under
            // the cell the NEXT short push will act on).
            if (s_page == 5 && s_fullArrived) {
                for (int slot = 0; slot < 16; ++slot) {
                    // ring: 0 chain-pair(cell 4) 1 engage(5) 2 preset(8,
                    // C3-local) 3 telem-d(6) 4 telem-f(7)
                    static const int ringCell[5] = { 4, 5, 8, 6, 7 };
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
                                 s_cur.cell[5].value != 0;
            uint8_t pair = s_fullArrived && s_cur.count > 4
                               ? s_cur.cell[4].value : 1;
            if (pair < 1 || pair > 3) pair = 1;
            switch (s_sysRing) {
                case 0: {  // chain pair cycle (engage state preserved)
                    const uint8_t next = (uint8_t)(pair % 3 + 1);
                    if (engaged) {
                        // Live pair switch: disengage, then engage the new
                        // pair — the console path validates + acks both.
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
        }

        uint8_t RackUI::GetPage() { return s_page; }

        bool RackUI::AssignModeActive() { return s_assignArmed; }

        void RackUI::SetPage(uint8_t page) {
            if (page > 5 || page == s_page) return;
            s_page = page;
            s_fullArrived = false;      // dashes until the snapshot lands
            s_pageDirty = true;
            s_stripDirty = true;
            RackLink::SendPageRequest(page);
        }

        bool RackUI::GetPotTarget(uint8_t pot, PotTarget &out) {
            out.kind = PotTarget::kNone;
            if (pot > 15) return false;
            if (s_page <= 1) {
                // P1/P2: the ACTIVE page cell's CC (TELEM-driven).
                if (!s_fullArrived || pot >= s_cur.count) return false;
                out.kind = PotTarget::kCC;
                out.cc = s_cur.cell[pot].cc;
                return true;
            }
            if (s_page >= 2 && s_page <= 4) {
                // MOD pages: the fixed (src, param) map. BPM pot -> the
                // SET_GLOBAL intent (UI_CMD action 5, item BPM); CLKSRC pot
                // is push-toggled, not turned.
                const SrcParam &sp = kModPotMap[s_page - 2][pot];
                if (s_page == 4 && pot == 11) {
                    out.kind = PotTarget::kGlobal;
                    out.cc = 0;             // item 0 = BPM (kUiGlobalBpm)
                    return true;
                }
                if (sp.src == 0xFF) return false;
                out.kind = PotTarget::kMod;
                out.src = sp.src;
                out.param = sp.param;
                return true;
            }
            return false;                   // SYS: pots inert
        }

        void RackUI::DebugState() {
            const char *names[6] = { "P1", "P2", "MOD-A", "MOD-B", "MOD-C",
                                     "SYS" };
            if (!s_fullArrived) {
                ESP_LOGI(TAG, "page %s: no TELEM snapshot yet (dashes)",
                         names[s_page]);
                return;
            }
            ESP_LOGI(TAG, "page %s mod=%u cells=%u tail(bpm=%u clk=%u "
                          "chain=%u link=%02x)", names[s_page],
                     s_cur.module, s_cur.count, s_cur.bpm, s_cur.clocksrc,
                     s_cur.chain, s_cur.link);
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
                                int p = (int)s_page + ev.delta;
                                while (p < 0) p += 6;
                                SetPage((uint8_t)(p % 6));
                            }
                            break;
                        case InputEvent::PUSH_HOLD:
                            s_assignArmed = true;
                            s_stripDirty = true;
                            break;
                        case InputEvent::PUSH_SHORT:
                            if (s_page == 4) {
                                // MOD-C: toggle the global clock source.
                                RackLink::SendSysToggle(
                                    RackLink::kSysClockToggle);
                            } else if (s_page == 5) {
                                sysRingAction();
                            }
                            // P1/P2/MOD-A/MOD-B: no short-push action.
                            break;
                        case InputEvent::PUSH_RELEASE:
                            if (s_overlay) {
                                // Release CONFIRMS the selected module.
                                RackLink::SendSwitch(s_overlaySel);
                                s_overlay = false;
                                s_assignArmed = false;
                            } else {
                                s_assignArmed = false;   // release cancels
                            }
                            s_stripDirty = true;
                            break;
                    }
                }

                // ---- render (30-60 Hz; test patterns own the screen) ----
                if (!Display::TestPatternActive()) {
                    // Poll the TELEM cache.
                    RackTelemPage tmp;
                    if (RackLink::TelemGet(tmp) && tmp.page == s_page) {
                        const bool wasFull =
                            tmp.mode == RackLink::kTelemFull;
                        if (wasFull || !s_fullArrived) {
                            s_cur = tmp;
                            if (wasFull) s_fullArrived = true;
                            s_pageDirty = true;
                        } else {
                            // Delta: RackLink already patched the cache —
                            // take the whole copy (cheap, consistent).
                            s_cur = tmp;
                            s_pageDirty = true;
                        }
                        s_stripDirty = true;
                        RackLink::TelemClearDirty();
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
