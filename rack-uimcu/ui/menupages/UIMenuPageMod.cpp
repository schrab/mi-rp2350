#include "UIMenuPageMod.hpp"
#include "Display.hpp"
#include "DspProxy.hpp"
#include "CVSlotNames.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace CTAG::DRIVERS;

namespace CTAG {
    namespace CTRL {

        // ---- label tables ----
        static const char* tModelNames[7] = {
            "Coins", "Cluster", "Drums", "Indep", "Divider", "3State", "Markov"
        };
        static const char* rangeNames[3] = {"0.25x", "1x", "4x"};
        static const char* cvRangeNames[3] = {"2oct", "5oct", "10oct"};
        static const char* ctlNames[3] = {"Ident", "Bump", "Tilt"};
        static const char* dvNames[3] = {"Off", "On", "Lock"};
        static const char* xyClkNames[5] = {"T123", "T1", "T2", "T3", "MIDI"};
        static const char* yDivNames[12] = {
            "1/64", "1/48", "1/32", "1/24", "1/16", "1/12",
            "1/8", "1/6", "1/4", "1/3", "1/2", "1/1"
        };
        static const char* scaleNames[6] = {
            "Major", "Minor", "Penta", "Pelog", "Bhairav", "Shri"
        };

        // Live output indication: T1/T2/T3 dots (filled = gate high) and
        // X1/X2/X3/Y center-fill bars (frameless, normalized to the active
        // octave range). Seven equal 18px cells across the line.
        void UIMenuPageMod::drawOutputLine() {
            uint32_t s = DspProxy::GetOutputSnapshot();
            for (int i = 0; i < 3; i++) {
                Display::DrawRect(1 + i * 18 + 6, 57, 6, 6, (s >> i) & 1);
            }
            const int shifts[4] = {3, 10, 17, 24};
            for (int i = 0; i < 4; i++) {
                int x = 1 + (i + 3) * 18;
                int cx = x + 9;              // zero boundary of the cell
                int q = (s >> shifts[i]) & 0x7f;
                float n = (q - 64) / 63.5f;
                int w = (int)(fabsf(n) * 9.0f + 0.5f);
                if (w > 9) w = 9;
                Display::DrawVLine(cx, 57, 6);   // zero tick
                if (w > 0) {
                    Display::DrawRect(n >= 0 ? cx : cx - w, 57, w, 6, true);
                }
            }
            lastDrawnSnapshot = s;
        }

        void UIMenuPageMod::onTick() {
            // Full redraws are gated on output changes: the display driver
            // flushes per dirty page, so unchanged frames cost nothing.
            if (DspProxy::GetOutputSnapshot() != lastDrawnSnapshot) {
                doRedraw();
            }
        }

        bool UIMenuPageMod::wantsTick() const { return true; }

        void UIMenuPageMod::init() {
            subPage = SP_MAIN;
            cursor = 0;
            ccEditSlot = 0;
            editing = false;
            lastTapTick = 0;
            tapHistoryCount = 0;
        }

        void UIMenuPageMod::deinit() {
            if (dirty) { DspProxy::SaveConfig(); dirty = false; }
        }

        // ---- row model: one flat list per subpage ----

        int UIMenuPageMod::rowCount() const {
            switch (subPage) {
                case SP_MAIN: return 6;
                case SP_PANEL: return 9;  // the module's 8 knobs + DV length button
                case SP_CLOCK: return 7;  // Clock, BPM, [Tap], Fine, Range, XClock, [Reset]
                case SP_T: return 8;      // Model, Range, PW, PWv, DV, T1..T3 slots
                case SP_X: return 9;      // Scale, Rng, Ctl, Reg, RVal, DV, X1..X3 slots
                case SP_Y: return 6;      // Rng, Sprd, Bias, Step, DV, Y slot
                case SP_CC_SLOTS: return 8;
                case SP_CC_EDIT: return 3;
                default: return 1;
            }
        }

        bool UIMenuPageMod::rowIsAction(int row) const {
            if (subPage == SP_CLOCK && (row == 2 || row == 6)) return true;  // [Tap], [Reset]
            if (subPage == SP_CC_EDIT && row == 2) return true; // [Learn]
            return false;
        }

        void UIMenuPageMod::activateRow(int row) {
            if (subPage == SP_CLOCK && row == 2) {
                onTapTempo();
            } else if (subPage == SP_CLOCK && row == 6) {
                DspProxy::RequestReset();
            } else if (subPage == SP_CC_EDIT && row == 2) {
                if (DspProxy::IsLearning()) {
                    DspProxy::StopLearn();
                } else {
                    DspProxy::StartLearn();
                }
            }
        }

        // ---- internal clock helpers (BPM <-> marbles semitone rate) ----
        // One t-clock cycle = one beat, so BPM = Hz * 60. BPM entry is
        // clamped to 20..480 (musical range, reachable in all three ranges);
        // the fine Rate row keeps the full +-60 semitone span.
        static float MasterBaseHz() {
            float base = 2.0f;
            if (DspProxy::GetTRange() == 0) base = 0.5f;
            else if (DspProxy::GetTRange() == 2) base = 8.0f;
            return base;
        }

        static float MasterHz() {
            return MasterBaseHz() * powf(2.0f, DspProxy::GetTRate() / 12.0f);
        }

        static float MasterBPM() {
            return MasterHz() * 60.0f;
        }

        static void SetMasterBPM(float bpm) {
            if (bpm < 20.0f) bpm = 20.0f;
            if (bpm > 480.0f) bpm = 480.0f;
            float st = 12.0f * log2f((bpm / 60.0f) / MasterBaseHz());
            DspProxy::SetTRate(st);  // engine clamps to +-60 st
        }

        // ---- tap tempo: rolling average (max 4) of stored tap intervals ----
        void UIMenuPageMod::onTapTempo() {
            uint32_t now = xTaskGetTickCount();
            // A reference tap is valid for 3 s; older -> start a new sequence.
            bool haveRef = lastTapTick != 0 && (now - lastTapTick) <= pdMS_TO_TICKS(3000);
            if (!haveRef) tapHistoryCount = 0;
            if (haveRef) {
                uint32_t interval = now - lastTapTick;
                if (interval >= pdMS_TO_TICKS(125)) {  // < 480 BPM; ignore bounce
                    for (int i = 3; i > 0; i--) tapIntervalHistory[i] = tapIntervalHistory[i - 1];
                    tapIntervalHistory[0] = interval;
                    if (tapHistoryCount < 4) tapHistoryCount++;
                    uint32_t sum = 0;
                    for (int i = 0; i < tapHistoryCount; i++) sum += tapIntervalHistory[i];
                    float avgMs = (float)sum * portTICK_PERIOD_MS / tapHistoryCount;
                    SetMasterBPM(60000.0f / avgMs);
                    dirty = true;
                }
            }
            lastTapTick = now;
        }

        void UIMenuPageMod::adjustRow(int row, int delta) {
            switch (subPage) {
                case SP_PANEL:
                    // the hardware module's 8 knobs, in panel order
                    switch (row) {
                        case 0: DspProxy::SetDejaVuAmount(DspProxy::GetDejaVuAmount() + delta * 0.01f); break;
                        case 1: DspProxy::SetDejaVuLength(DspProxy::GetDejaVuLength() + delta); break;
                        case 2: SetMasterBPM(MasterBPM() + delta); break;          // T RATE
                        case 3: DspProxy::SetTBias(DspProxy::GetTBias() + delta * 0.01f); break;
                        case 4: DspProxy::SetTJitter(DspProxy::GetTJitter() + delta * 0.01f); break;
                        case 5: DspProxy::SetXSpread(DspProxy::GetXSpread() + delta * 0.01f); break;
                        case 6: DspProxy::SetXBias(DspProxy::GetXBias() + delta * 0.01f); break;
                        case 7: DspProxy::SetXSteps(DspProxy::GetXSteps() + delta * 0.01f); break;
                        case 8: DspProxy::SetYDivider(DspProxy::GetYDivider() + delta); break; // Y RATE
                    }
                    break;
                case SP_CLOCK:
                    switch (row) {
                        case 0: DspProxy::SetTExternal(!DspProxy::GetTExternal()); break;
                        case 1: SetMasterBPM(MasterBPM() + delta); break;
                        case 3: DspProxy::SetTRate(DspProxy::GetTRate() + delta); break;
                        case 4: DspProxy::SetTRange(DspProxy::GetTRange() + delta); break;
                        case 5: DspProxy::SetXYClockSource(DspProxy::GetXYClockSource() + delta); break;
                    }
                    break;
                case SP_T:
                    switch (row) {
                        case 0: DspProxy::SetTModel(DspProxy::GetTModel() + delta); break;
                        case 1: DspProxy::SetTRange(DspProxy::GetTRange() + delta); break;
                        case 2: DspProxy::SetTPwMean(DspProxy::GetTPwMean() + delta * 5); break;
                        case 3: DspProxy::SetTPwStd(DspProxy::GetTPwStd() + delta * 5); break;
                        case 4: DspProxy::SetTDejaVu(DspProxy::GetTDejaVu() + delta); break;
                        case 5: DspProxy::SetTSlot(0, DspProxy::GetTSlot(0) + delta); break;
                        case 6: DspProxy::SetTSlot(1, DspProxy::GetTSlot(1) + delta); break;
                        case 7: DspProxy::SetTSlot(2, DspProxy::GetTSlot(2) + delta); break;
                    }
                    break;
                case SP_X:
                    switch (row) {
                        case 0: DspProxy::SetXScale(DspProxy::GetXScale() + delta); break;
                        case 1: DspProxy::SetXRange(DspProxy::GetXRange() + delta); break;
                        case 2: DspProxy::SetXControl(DspProxy::GetXControl() + delta); break;
                        case 3: DspProxy::SetXRegister(!DspProxy::GetXRegister()); break;
                        case 4: DspProxy::SetXRegisterValue(DspProxy::GetXRegisterValue() + delta * 0.01f); break;
                        case 5: DspProxy::SetXDejaVu(DspProxy::GetXDejaVu() + delta); break;
                        case 6: DspProxy::SetXSlot(0, DspProxy::GetXSlot(0) + delta); break;
                        case 7: DspProxy::SetXSlot(1, DspProxy::GetXSlot(1) + delta); break;
                        case 8: DspProxy::SetXSlot(2, DspProxy::GetXSlot(2) + delta); break;
                    }
                    break;
                case SP_Y:
                    switch (row) {
                        case 0: DspProxy::SetYRange(DspProxy::GetYRange() + delta); break;
                        case 1: DspProxy::SetYSpread(DspProxy::GetYSpread() + delta * 0.01f); break;
                        case 2: DspProxy::SetYBias(DspProxy::GetYBias() + delta * 0.01f); break;
                        case 3: DspProxy::SetYSteps(DspProxy::GetYSteps() + delta * 0.01f); break;
                        case 4: DspProxy::SetYDejaVu(DspProxy::GetYDejaVu() + delta); break;
                        case 5: DspProxy::SetYSlot(DspProxy::GetYSlot() + delta); break;
                    }
                    break;
                case SP_CC_EDIT:
                    if (row == 0) {
                        DspProxy::SetDynamicSlotCC(ccEditSlot, DspProxy::GetDynamicSlotCC(ccEditSlot) + delta);
                    } else if (row == 1) {
                        DspProxy::SetDynamicSlotChan(ccEditSlot, DspProxy::GetDynamicSlotChan(ccEditSlot) + delta);
                    }
                    break;
                default:
                    break;
            }
            dirty = true;
        }

        void UIMenuPageMod::formatRow(int row, char *buf, int buflen) {
            switch (subPage) {
                case SP_PANEL:
                    // the hardware module's 8 knobs + DV length button, in panel order
                    switch (row) {
                        case 0:
                            snprintf(buf, buflen, " DejaVu:%.2f", DspProxy::GetDejaVuAmount());
                            break;
                        case 1:
                            snprintf(buf, buflen, " DejaVuLn:%d", DspProxy::GetDejaVuLength());
                            break;
                        case 2:
                            snprintf(buf, buflen, " T.Rate:%.0fbpm", MasterBPM());
                            break;
                        case 3:
                            snprintf(buf, buflen, " T.Bias:%.2f", DspProxy::GetTBias());
                            break;
                        case 4:
                            snprintf(buf, buflen, " T.Jitter:%.2f", DspProxy::GetTJitter());
                            break;
                        case 5:
                            snprintf(buf, buflen, " X.Spread:%.2f", DspProxy::GetXSpread());
                            break;
                        case 6:
                            snprintf(buf, buflen, " X.Bias:%.2f", DspProxy::GetXBias());
                            break;
                        case 7:
                            snprintf(buf, buflen, " X.Steps:%.2f", DspProxy::GetXSteps());
                            break;
                        case 8:
                            snprintf(buf, buflen, " Y.Rate:%s", yDivNames[DspProxy::GetYDivider()]);
                            break;
                    }
                    break;
                case SP_CLOCK:
                    switch (row) {
                        case 0:
                            snprintf(buf, buflen, " Clock:%s", DspProxy::GetTExternal() ? "MIDI" : "Int");
                            break;
                        case 1:
                            snprintf(buf, buflen, " BPM:%.1f", MasterBPM());
                            break;
                        case 3:
                            snprintf(buf, buflen, " Fine:%.2fHz", MasterHz());
                            break;
                        case 4:
                            snprintf(buf, buflen, " Range:%s", rangeNames[DspProxy::GetTRange()]);
                            break;
                        case 5:
                            snprintf(buf, buflen, " XClock:%s", xyClkNames[DspProxy::GetXYClockSource()]);
                            break;
                        case 2:
                            snprintf(buf, buflen, tapHistoryCount > 0 ? " [Tap] %.0fbpm" : " [Tap]", MasterBPM());
                            break;
                        case 6:
                            snprintf(buf, buflen, " [Reset]");
                            break;
                    }
                    break;
                case SP_T:
                    switch (row) {
                        case 0:
                            snprintf(buf, buflen, " Model:%s", tModelNames[DspProxy::GetTModel()]);
                            break;
                        case 1:
                            snprintf(buf, buflen, " Range:%s", rangeNames[DspProxy::GetTRange()]);
                            break;
                        case 2:
                            snprintf(buf, buflen, " Width:%d%%", DspProxy::GetTPwMean() * 100 / 255);
                            break;
                        case 3:
                            snprintf(buf, buflen, " WidthVar:%d%%", DspProxy::GetTPwStd() * 100 / 255);
                            break;
                        case 4:
                            snprintf(buf, buflen, " DejaVu:%s", dvNames[DspProxy::GetTDejaVu()]);
                            break;
                        case 5:
                        case 6:
                        case 7: {
                            int s = DspProxy::GetTSlot(row - 5);
                            snprintf(buf, buflen, " T%d out:%s", row - 4, TrigSlotName(s));
                            break;
                        }
                    }
                    break;
                case SP_X:
                    switch (row) {
                        case 0:
                            snprintf(buf, buflen, " Scale:%s", scaleNames[DspProxy::GetXScale()]);
                            break;
                        case 1:
                            snprintf(buf, buflen, " Range:%s", cvRangeNames[DspProxy::GetXRange()]);
                            break;
                        case 2:
                            snprintf(buf, buflen, " CtlMode:%s", ctlNames[DspProxy::GetXControl()]);
                            break;
                        case 3:
                            snprintf(buf, buflen, " Register:%s", DspProxy::GetXRegister() ? "On" : "Off");
                            break;
                        case 4:
                            snprintf(buf, buflen, " RegValue:%.2f", DspProxy::GetXRegisterValue());
                            break;
                        case 5:
                            snprintf(buf, buflen, " DejaVu:%s", dvNames[DspProxy::GetXDejaVu()]);
                            break;
                        case 6:
                        case 7:
                        case 8: {
                            int s = DspProxy::GetXSlot(row - 6);
                            if (s < 0) snprintf(buf, buflen, " X%d out: none", row - 5);
                            else snprintf(buf, buflen, " X%d out:%s", row - 5, cvSlotDisplayNames[s]);
                            break;
                        }
                    }
                    break;
                case SP_Y:
                    switch (row) {
                        case 0:
                            snprintf(buf, buflen, " Range:%s", cvRangeNames[DspProxy::GetYRange()]);
                            break;
                        case 1:
                            snprintf(buf, buflen, " Spread:%.2f", DspProxy::GetYSpread());
                            break;
                        case 2:
                            snprintf(buf, buflen, " Bias:%.2f", DspProxy::GetYBias());
                            break;
                        case 3:
                            snprintf(buf, buflen, " Steps:%.2f", DspProxy::GetYSteps());
                            break;
                        case 4:
                            snprintf(buf, buflen, " DejaVu:%s", dvNames[DspProxy::GetYDejaVu()]);
                            break;
                        case 5: {
                            int s = DspProxy::GetYSlot();
                            if (s < 0) snprintf(buf, buflen, " Y out: none");
                            else snprintf(buf, buflen, " Y out:%s", cvSlotDisplayNames[s]);
                            break;
                        }
                    }
                    break;
                case SP_CC_EDIT:
                    if (row == 0) {
                        int cc = DspProxy::GetDynamicSlotCC(ccEditSlot);
                        if (cc < 0) snprintf(buf, buflen, " CC: --");
                        else snprintf(buf, buflen, " CC: %d", cc);
                    } else if (row == 1) {
                        snprintf(buf, buflen, " Chan: %d", DspProxy::GetDynamicSlotChan(ccEditSlot));
                    } else {
                        snprintf(buf, buflen, " [%s]", DspProxy::IsLearning() ? "StopLearn" : "Learn");
                    }
                    break;
                default:
                    buf[0] = 0;
                    break;
            }
        }

        // ---- event handling ----

        void UIMenuPageMod::onEncoder(int delta) {
            if (subPage == SP_MAIN) {
                cursor += delta;
                if (cursor < 0) cursor = 0;
                if (cursor > 5) cursor = 5;
            } else if (editing) {
                adjustRow(cursor, delta);
            } else {
                cursor += delta;
                if (cursor < 0) cursor = 0;
                if (cursor > rowCount() - 1) cursor = rowCount() - 1;
            }
            doRedraw();
        }

        void UIMenuPageMod::onButton(int btnId, bool longPress) {
            if (btnId != 2 || longPress) {
                doRedraw();
                return;
            }
            if (subPage == SP_MAIN) {
                SubPage target = SP_MAIN;
                switch (cursor) {
                    case 0: target = SP_PANEL; break;
                    case 1: target = SP_CLOCK; break;
                    case 2: target = SP_T; break;
                    case 3: target = SP_X; break;
                    case 4: target = SP_Y; break;
                    case 5: target = SP_CC_SLOTS; break;
                }
                if (target != SP_MAIN) {
                    subPage = target;
                    cursor = 0;
                    editing = false;
                }
            } else if (subPage == SP_CC_SLOTS) {
                ccEditSlot = cursor;
                subPage = SP_CC_EDIT;
                cursor = 0;
                editing = false;
            } else if (rowIsAction(cursor)) {
                activateRow(cursor);
            } else {
                editing = !editing;
            }
            doRedraw();
        }

        bool UIMenuPageMod::onBack() {
            if (editing) {
                editing = false;
                doRedraw();
                return true;
            }
            switch (subPage) {
                case SP_MAIN:
                    return false;
                case SP_CC_EDIT:
                    subPage = SP_CC_SLOTS;
                    cursor = ccEditSlot;
                    doRedraw();
                    return true;
                case SP_PANEL: cursor = 0; break;
                case SP_CLOCK: cursor = 1; break;
                case SP_T: cursor = 2; break;
                case SP_X: cursor = 3; break;
                case SP_Y: cursor = 4; break;
                case SP_CC_SLOTS: cursor = 5; break;
                default: cursor = 0; break;
            }
            subPage = SP_MAIN;
            doRedraw();
            return true;
        }

        // ---- rendering ----

        void UIMenuPageMod::doRedraw() {
            switch (subPage) {
                case SP_MAIN: redrawMain(); break;
                case SP_PANEL: redrawRows("Knobs"); break;
                case SP_CLOCK: redrawRows("Clock"); break;
                case SP_T: redrawRows("T Setup"); break;
                case SP_X: redrawRows("X Setup"); break;
                case SP_Y: redrawRows("Y Setup"); break;
                case SP_CC_SLOTS: redrawCCSlots(); break;
                case SP_CC_EDIT: redrawRows(nullptr); break;
                default: break;
            }
        }

        void UIMenuPageMod::redrawMain() {
            Display::Clear();
            char buf[32];
            const int TOTAL = 6;
            for (int i = 0; i < TOTAL; i++) {
                int y = ROW(i);
                switch (i) {
                    case 0:
                        snprintf(buf, sizeof(buf), " Knobs");
                        break;
                    case 1:
                        if (DspProxy::GetTExternal()) {
                            snprintf(buf, sizeof(buf), " Clk:MIDI");
                        } else {
                            snprintf(buf, sizeof(buf), " Clock:Int %.0fbpm", MasterBPM());
                        }
                        break;
                    case 2:
                        snprintf(buf, sizeof(buf), " T Setup:%s", tModelNames[DspProxy::GetTModel()]);
                        break;
                    case 3:
                        snprintf(buf, sizeof(buf), " X Setup:%s", scaleNames[DspProxy::GetXScale()]);
                        break;
                    case 4:
                        snprintf(buf, sizeof(buf), " Y Setup:%s", yDivNames[DspProxy::GetYDivider()]);
                        break;
                    default:
                        snprintf(buf, sizeof(buf), " CC Slots");
                        break;
                }
                Display::DrawString(0, y, buf, Display::FONT_5X7);
            }
            int cy = ROW(cursor);
            Display::InvertRect(0, cy, 128, 8);
            drawOutputLine();
            Display::Flush();
        }

        // Generic scrolling parameter list used by all Marbles subpages
        // (and CC Edit). nullptr title renders the CC-slot header.
        void UIMenuPageMod::redrawRows(const char *title) {
            Display::Clear();
            char buf[32];
            if (title != nullptr) {
                Display::DrawString(0, ROW(0), title, Display::FONT_5X7);
    } else {
        snprintf(buf, sizeof(buf), "CC Slot %d", ccEditSlot + 1);
        Display::DrawString(0, ROW(0), buf, Display::FONT_5X7);
    }

            const int TOTAL = rowCount();
            const int VIS = VISIBLE_ITEMS_HDR - 1;  // bottom row = output strip
            int scrollOff = (cursor > VIS - 1) ? cursor - (VIS - 1) : 0;
            int visible = TOTAL - scrollOff;
            if (visible > VIS) visible = VIS;
            for (int i = 0; i < visible; i++) {
                int idx = scrollOff + i;
                int y = ROW_HDR(i);
                formatRow(idx, buf, sizeof(buf));
                Display::DrawString(0, y, buf, Display::FONT_5X7);
                if (editing && cursor == idx) {
                    // invert the value region (after " Lll:")
                    int vstart = 1;
                    while (buf[vstart] && buf[vstart] != ':') vstart++;
                    vstart++;
                    int x = vstart * 6 + 1;
                    if (x < 127) Display::InvertRect(x, y, 128 - x, 8);
                }
            }
            if (!editing) {
                int cy = ROW_HDR(cursor - scrollOff);
                Display::InvertRect(0, cy, 128, 8);
            }
            if (TOTAL > VIS) {
                Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0_HDR, VIS * LINE_H, TOTAL, cursor);
            }
            drawOutputLine();
            Display::Flush();
        }

        void UIMenuPageMod::redrawCCSlots() {
            Display::Clear();
            Display::DrawString(0, ROW(0), "CC Slots", Display::FONT_5X7);
            constexpr int TOTAL = 8;
            const int VIS = VISIBLE_ITEMS_HDR - 1;  // bottom row = output strip
            int scrollOff = (cursor > VIS - 1) ? cursor - (VIS - 1) : 0;
            int visible = TOTAL - scrollOff;
            if (visible > VIS) visible = VIS;
            for (int i = 0; i < visible; i++) {
                int idx = scrollOff + i;
                int y = ROW_HDR(i);
                int cc = DspProxy::GetDynamicSlotCC(idx);
                int ch = DspProxy::GetDynamicSlotChan(idx);
                char buf[48];
                if (cc < 0) {
                    snprintf(buf, sizeof(buf), " CC%d: --", idx + 1);
                } else {
                    snprintf(buf, sizeof(buf), " CC%d: CC%d Ch%d", idx + 1, cc, ch);
                }
                Display::DrawString(0, y, buf, Display::FONT_5X7);
            }
            int cy = ROW_HDR(cursor - scrollOff);
            Display::InvertRect(0, cy, 128, 8);
            if (TOTAL > VIS)
                Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0_HDR, VIS * LINE_H, TOTAL, cursor);
            drawOutputLine();
            Display::Flush();
        }
    }
}
