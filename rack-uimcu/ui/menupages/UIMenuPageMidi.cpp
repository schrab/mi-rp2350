/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

***************/

#include "UIMenuPageMidi.hpp"
#include "../Display.hpp"
#include "../DspProxy.hpp"
#include "BtMidiReceiver.hpp"
#include <cstdio>

using namespace CTAG::DRIVERS;

namespace CTAG {
    namespace CTRL {
        // Layout constants come from Display.hpp (ROW/LINE_H/ITEM_Y0/
        // VISIBLE_ITEMS/SCROLLBAR_X/ClampScroll). No local redefinitions —
        // Display.hpp's ROW is a macro and collides with anything named ROW.
        void UIMenuPageMidi::init() {
            subPage = SP_MAIN;
            cursor = 0;
            scrollOffset = 0;
            // Pull the DSP's current enables (reply updates the cache and
            // the next redraw reflects it).
            DspProxy::RefreshMidiFlags();
        }

        void UIMenuPageMidi::deinit() {}

        void UIMenuPageMidi::onEncoder(int delta) {
            if (subPage == SP_MAIN) {
                cursor += delta;
                if (cursor < 0) cursor = 0;
                if (cursor > 4) cursor = 4;
            } else if (subPage == SP_SCAN) {
                int n = BtMidiReceiver::GetDeviceCount();
                int newCursor = cursor + delta;
                if (newCursor < 0) newCursor = 0;
                if (newCursor >= n) newCursor = (n > 0) ? n - 1 : 0;
                cursor = newCursor;
                ClampScroll(cursor, scrollOffset, n, VISIBLE_ITEMS);
            }
            doRedraw();
        }

        void UIMenuPageMidi::onButton(int btnId, bool longPress) {
            if (subPage == SP_MAIN) {
                if (btnId == 2 && longPress) {
                    if (cursor == 1 && BtMidiReceiver::IsConnected())
                        BtMidiReceiver::Disconnect();
                } else if (btnId == 2 && !longPress) {
                    if (cursor == 0) {
                        BtMidiReceiver::StartScan();
                        subPage = SP_SCAN;
                        cursor = 0;
                        scrollOffset = 0;
                    } else if (cursor == 1) {
                        // Disconnect answers a plain OK press (long-press
                        // also works — kept for A1S muscle memory).
                        if (BtMidiReceiver::IsConnected())
                            BtMidiReceiver::Disconnect();
                    } else if (cursor == 3) {
                        DspProxy::SetMidiFlags(!DspProxy::IsMidiUartEnabled(),
                                               DspProxy::IsMidiBleEnabled());
                    } else if (cursor == 4) {
                        DspProxy::SetMidiFlags(DspProxy::IsMidiUartEnabled(),
                                               !DspProxy::IsMidiBleEnabled());
                    }
                }
            } else if (subPage == SP_SCAN) {
                if (btnId == 2 && !longPress) {
                    int n = BtMidiReceiver::GetDeviceCount();
                    if (n > 0 && cursor < n) {
                        BtMidiReceiver::Connect(cursor);
                        subPage = SP_MAIN;
                        cursor = 0;
                        doRedraw();
                        return;
                    }
                }
                if (btnId == 1 && longPress) {
                    if (BtMidiReceiver::IsScanning()) BtMidiReceiver::StopScan();
                    subPage = SP_MAIN;
                    cursor = 0;
                    doRedraw();
                    return;
                }
            }
            doRedraw();
        }

        bool UIMenuPageMidi::onBack() {
            if (subPage == SP_SCAN) {
                if (BtMidiReceiver::IsScanning()) BtMidiReceiver::StopScan();
                subPage = SP_MAIN;
                cursor = 0;
                doRedraw();
                return true;
            }
            return false;
        }

        void UIMenuPageMidi::doRedraw() {
            if (subPage == SP_MAIN) redrawMain();
            else if (subPage == SP_SCAN) redrawScan();
        }

        void UIMenuPageMidi::redrawMain() {
            Display::Clear();
            char buf[32];
            int y = ITEM_Y0;

            snprintf(buf, sizeof(buf), "SCAN%s",
                     BtMidiReceiver::IsScanning() ? " (running)" : "");
            Display::DrawString(0, y, buf, Display::FONT_5X7);
            y += LINE_H;

            snprintf(buf, sizeof(buf), "Disconnect%s",
                     BtMidiReceiver::IsConnected() ? "" : " (none)");
            Display::DrawString(0, y, buf, Display::FONT_5X7);
            y += LINE_H;

            snprintf(buf, sizeof(buf), "Status: %s",
                     BtMidiReceiver::IsConnected() ? "CONNECTED" : "IDLE");
            Display::DrawString(0, y, buf, Display::FONT_5X7);
            y += LINE_H;

            snprintf(buf, sizeof(buf), "TRS In [%s]",
                     DspProxy::IsMidiUartEnabled() ? "ON" : "OFF");
            Display::DrawString(0, y, buf, Display::FONT_5X7);
            y += LINE_H;

            snprintf(buf, sizeof(buf), "BLE  In [%s]",
                     DspProxy::IsMidiBleEnabled() ? "ON" : "OFF");
            Display::DrawString(0, y, buf, Display::FONT_5X7);

            Display::InvertRect(0, ROW(cursor), 128, 8);
            Display::Flush();
        }

        void UIMenuPageMidi::redrawScan() {
            Display::Clear();
            int n = BtMidiReceiver::GetDeviceCount();

            if (BtMidiReceiver::IsScanning()) {
                Display::DrawString(0, 5, "Scanning...", Display::FONT_5X7);
            } else {
                Display::DrawString(0, 5, "Scan complete", Display::FONT_5X7);
            }

            if (n == 0) {
                Display::DrawString(0, 24, "No devices found", Display::FONT_5X7);
            } else {
                int visible = ClampVisible(n, scrollOffset, VISIBLE_ITEMS);
                for (int i = 0; i < visible; i++) {
                    int idx = scrollOffset + i;
                    const BtDeviceInfo *d = BtMidiReceiver::GetDevice(idx);
                    if (!d) continue;
                    Display::DrawString(0, ROW(i),
                                        d->name[0] ? d->name : "(unnamed)",
                                        Display::FONT_5X7);
                }
                int cy = ROW(cursor - scrollOffset);
                Display::InvertRect(0, cy, 128, 8);
                if (n > VISIBLE_ITEMS)
                    Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0, VISIBLE_ITEMS * LINE_H,
                                           n, cursor);
            }

            Display::Flush();
        }

        // Live scan updates: while the scan screen is up the task pumps
        // onTick() every 20 ms; redraw only when something changed (found
        // count or scanning state) so the SPI bus isn't hammered.
        void UIMenuPageMidi::onTick() {
            static int lastCount = -1;
            static bool lastScanning = false;
            static SubPage lastPage = SP_MAIN;
            int count = BtMidiReceiver::GetDeviceCount();
            bool scanning = BtMidiReceiver::IsScanning();
            if (count != lastCount || scanning != lastScanning || subPage != lastPage) {
                lastCount = count;
                lastScanning = scanning;
                lastPage = subPage;
                doRedraw();
            }
        }

        bool UIMenuPageMidi::wantsTick() const {
            // Pump while the scan screen is up (or BLE is connecting) so
            // found devices and status appear immediately.
            return subPage == SP_SCAN || BtMidiReceiver::IsConnected() ||
                   BtMidiReceiver::IsScanning();
        }

        void UIMenuPageMidi::DebugStateJson(std::string &out) {
            out += "{\"midiUart\":";
            out += DspProxy::IsMidiUartEnabled() ? "1" : "0";
            out += ",\"midiBle\":";
            out += DspProxy::IsMidiBleEnabled() ? "1" : "0";
            out += ",\"bleConnected\":";
            out += BtMidiReceiver::IsConnected() ? "1" : "0";
            out += ",\"bleDevices\":";
            out += std::to_string(BtMidiReceiver::GetDeviceCount());
            out += ",\"cursor\":";
            out += std::to_string(cursor);
            out += "}";
        }
    }
}
