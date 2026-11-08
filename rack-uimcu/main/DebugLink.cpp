/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

The CTAG TBD hardware design is released under the Creative Commons
Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0).
Details here: https://creativecommons.org/licenses/by-nc-sa/4.0/

CTAG TBD is provided "as is" without any express or implied warranties.

License and copyright details for specific submodules are included in their
respective component folders / files if different from this license.
***************/

// USB-Serial-JTAG bench channel. Framing and reply shapes match the A1S
// DebugUI byte-for-byte so existing host scripts only change their port.
// Commands:
//   {"cmd":"/debug/getUiState"}
//   {"cmd":"/debug/getDisplayFramebuffer"}
//   {"cmd":"/debug/injectEvent","action":"ok"|"mod"|"back"|"enc"|"enc2",
//                   "delta":n,"count":n}
//   {"cmd":"/debug/setActivePlugin","ch":0,"id":"..."}   (stub state)
//   {"cmd":"/debug/getProxyState"}                        (DspProxy stub dump)
//
// Transport: the usb_serial_jtag DRIVER (own ISR + ring buffers), not the
// console VFS. The VFS path proved unreliable on the bench: replies only
// flowed while the boot-log burst kept the wire busy, then stalled to a
// ~600 ms one-reply-per-second cadence. This mirrors the A1S DebugUI
// solution (install the driver on the console peripheral, then hand ESP_LOG
// to the driver so logging keeps flowing on the same wire).
//
// Console note (SDD §2.1): the C3 console is USB-Serial-JTAG only — UART0
// pins belong to the future link. ESP_LOG output shares the wire; the STX/
// ETX decoder ignores non-framed bytes, same as the A1S bench.

#include "DebugLink.hpp"
#include "Display.hpp"
#include "UIMenu.hpp"
#include "input/UserInput.hpp"
#include "DspProxy.hpp"
#include "LinkClient.hpp"
#include "ctag/link/protocol.hpp"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rapidjson/document.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <unistd.h>

using namespace std;
using namespace rapidjson;
using namespace CTAG::LINK;

static const char *TAG = "DebugLink";

namespace CTAG {
    namespace CTRL {
        static TaskHandle_t hDebugTask = nullptr;

        static const char *panelName(int panel) {
            switch (panel) {
                case 0: return "MIX";
                case 1: return "TAPE";
                case 2: return "HOME";
                case 3: return "MOD";
                case 4: return "PARAMS";
                default: return "?";
            }
        }

        // After the driver owns the USJ, the ROM/vfs console cannot TX
        // reliably; hand ESP_LOG to the driver like DebugUI does on the A1S.
        static int usjLogWrite(const char *format, va_list args) {
            static char line[256];
            int len = vsnprintf(line, sizeof(line), format, args);
            if (len > 0)
                usb_serial_jtag_write_bytes(line, (size_t)len, pdMS_TO_TICKS(100));
            return len;
        }

        static void writeFramed(const string &payload) {
            string frame;
            AppendFrame(payload.data(), payload.size(), frame);
            usb_serial_jtag_write_bytes(frame.data(), frame.size(), pdMS_TO_TICKS(100));
        }

        static bool handleCommandImpl(const string &doc, string &response);

        // Public dispatch: parses the request and, when it carries an integer
        // "seq", echoes it back in the reply so a host script can correlate
        // responses with commands even when replies lag or interleave on the
        // shared console wire.
        static bool handleCommand(const string &doc, string &response) {
            if (!handleCommandImpl(doc, response)) return false;
            Document s;
            s.Parse(doc);
            if (!s.HasParseError() && s.IsObject() && s.HasMember("seq") &&
                s["seq"].IsInt()) {
                // splice ,"seq":N before the closing brace
                size_t close = response.rfind('}');
                if (close != string::npos) {
                    response.insert(close, ",\"seq\":" + to_string(s["seq"].GetInt()));
                }
            }
            return true;
        }

        static bool handleCommandImpl(const string &doc, string &response) {
            Document d;
            d.Parse(doc);
            if (d.HasParseError() || !d.IsObject() || !d.HasMember("cmd")) return false;

            string s(d["cmd"].GetString());
            if (s.rfind("/debug/", 0) != 0) return false;

            if (s == "/debug/getUiState") {
                int nav = 0, panel = 0;
                UIMenu::GetDebugState(nav, panel);
                response = "{\"nav\":" + to_string(nav)
                           + ",\"panel\":" + to_string(panel)
                           + ",\"panelName\":\"" + panelName(panel) + "\""
                           + ",\"stackFree\":" + to_string(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t))
                           + "}";
                return true;
            }

            if (s == "/debug/getDisplayFramebuffer") {
                const int fbsz = DRIVERS::Display::FRAMEBUFFER_SIZE;
                static uint8_t fbBuf[DRIVERS::Display::FRAMEBUFFER_SIZE];
                if (!UIMenu::CopyFramebufferShot(fbBuf, 500)) {
                    response = "{\"error\":\"fb timeout\"}";
                    return true;
                }
                static const char *hexDigits = "0123456789abcdef";
                string out;
                out.reserve(fbsz * 2 + 64);
                out = "{\"screenshot\":1,\"w\":128,\"h\":64,\"data\":\"";
                for (int i = 0; i < fbsz; i++) {
                    out.push_back(hexDigits[fbBuf[i] >> 4]);
                    out.push_back(hexDigits[fbBuf[i] & 0x0f]);
                }
                out += "\"}";
                response = out.c_str();
                return true;
            }

            if (s == "/debug/injectEvent") {
                if (!d.HasMember("action") || !d["action"].IsString()) {
                    response = "{\"error\":\"missing action\"}";
                    return true;
                }
                const char *action = d["action"].GetString();
                int16_t delta = 0;
                int count = 1;
                if (d.HasMember("delta") && d["delta"].IsInt()) delta = (int16_t)d["delta"].GetInt();
                if (d.HasMember("count") && d["count"].IsInt()) count = d["count"].GetInt();
                if (count < 1) count = 1;
                if (count > 64) count = 64;

                InputEvent::Type type;
                if (strcmp(action, "ok") == 0) type = InputEvent::BTN2_SHORT;
                else if (strcmp(action, "mod") == 0) type = InputEvent::BTN2_LONG;
                else if (strcmp(action, "back") == 0) type = InputEvent::BTN1_SHORT;
                else if (strcmp(action, "enc") == 0) type = InputEvent::ENC_DELTA;
                else if (strcmp(action, "enc2") == 0) type = InputEvent::ENC2_DELTA;
                else {
                    response = "{\"error\":\"unknown action\"}";
                    return true;
                }

                int sent = 0;
                for (int i = 0; i < count; i++) {
                    if (!UserInput::InjectEvent(type, delta)) break;
                    sent++;
                }
                response = "{\"sent\":" + to_string(sent) + "}";
                return true;
            }

            if (s == "/debug/displayCmd") {
                // One raw SSD1309 command byte. 0xA5 = force ALL pixels on
                // (ignores RAM) — the power/reset wiring test; 0xA4 = back.
                if (!d.HasMember("value") || !d["value"].IsInt()) {
                    response = "{\"error\":\"missing value\"}";
                    return true;
                }
                DRIVERS::Display::RawCommand((uint8_t)d["value"].GetInt());
                response = "{\"ok\":1}";
                return true;
            }

            if (s == "/debug/displayScan") {
                // Bit-banged init + all-on through one pin permutation.
                int mode = d.HasMember("mode") && d["mode"].IsInt() ? d["mode"].GetInt() : 0;
                if (mode < 0 || mode > 3) {
                    response = "{\"error\":\"bad mode\"}";
                    return true;
                }
                DRIVERS::Display::BenchScan(mode);
                response = "{\"ok\":1,\"mode\":" + to_string(mode) + "}";
                return true;
            }

            if (s == "/debug/setScreensaver") {
                bool en = d.HasMember("enable") && d["enable"].IsInt() && d["enable"].GetInt() != 0;
                UIMenu::SetScreensaverEnabled(en);
                response = string("{\"ok\":1,\"enabled\":") + (en ? "1}" : "0}");
                return true;
            }

            if (s == "/debug/displayPins") {
                int cs = d.HasMember("cs") && d["cs"].IsInt() ? d["cs"].GetInt() : 2;
                int dc = d.HasMember("dc") && d["dc"].IsInt() ? d["dc"].GetInt() : 8;
                if (cs < 0 || cs > 21 || dc < 0 || dc > 21) {
                    response = "{\"error\":\"bad pins\"}";
                    return true;
                }
                DRIVERS::Display::BenchPins(cs, dc);
                response = "{\"ok\":1,\"cs\":" + to_string(cs) + ",\"dc\":" + to_string(dc) + "}";
                return true;
            }

            if (s == "/debug/displayReinit") {
                DRIVERS::Display::Reinit();
                response = "{\"ok\":1}";
                return true;
            }

            if (s == "/debug/getBtnAdc") {
                // Button-ladder calibration aid: averaged raw ADC1_CH0
                // (GPIO0). Press BTN_A / BTN_B / both in turn and read this.
                extern int UserInputDebugBtnAdc();
                response = string("{\"adc\":") + std::to_string(UserInputDebugBtnAdc()) + "}";
                return true;
            }

            if (s == "/debug/setLinkBaud") {
                // Debug-only link UART retune, e.g.
                // {"cmd":"/debug/setLinkBaud","value":460800} — baud sweep
                // for the handshake hunt (LinkClient owns UART0).
                if (!d.HasMember("value") || !d["value"].IsInt()) {
                    response = "{\"error\":\"missing value\"}";
                    return true;
                }
                response = LinkClient::SetBaud(d["value"].GetInt())
                               ? "{\"ok\":1}"
                               : "{\"error\":\"bad baud\"}";
                return true;
            }

            if (s == "/debug/setActivePlugin") {
                if (!d.HasMember("id") || !d["id"].IsString()) {
                    response = "{\"error\":\"missing id\"}";
                    return true;
                }
                int ch = 0;
                if (d.HasMember("ch") && d["ch"].IsInt()) ch = d["ch"].GetInt();
                if (ch < 0 || ch > 1) {
                    response = "{\"error\":\"bad channel\"}";
                    return true;
                }
                DspProxy::SetSoundProcessorChannel(ch, d["id"].GetString());
                response = "{\"ok\":1}";
                return true;
            }

            if (s == "/debug/getProxyState") {
                response = string("{\"active0\":\"") + DspProxy::GetStringID(0)
                           + "\",\"active1\":\"" + DspProxy::GetStringID(1)
                           + "\",\"fav\":" + to_string((long long)DspProxy::GetActiveFav())
                           + ",\"sdMounted\":" + (DspProxy::IsSDMounted() ? "1" : "0")
                           + ",\"proto\":" + to_string(kProtoMajor) + "." + to_string(kProtoMinor)
                           + "}";
                return true;
            }

            response = "{\"error\":\"unknown debug command\"}";
            return true;
        }

        static void debugTask(void *) {
            string cmd;
            bool receiving = false;
            uint8_t rx[64];
            // The framebuffer reply is ~2.1 KB; guard against a runaway host.
            constexpr size_t MAX_DOC = 4096;

            while (true) {
                // Driver read with a 0-tick timeout: genuinely non-blocking,
                // served from the driver's RX ring buffer.
                int got = usb_serial_jtag_read_bytes(rx, sizeof(rx), 0);
                if (got <= 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }

                for (int i = 0; i < got; i++) {
                    char c = (char)rx[i];
                    if (c == (char)kStx) {
                        receiving = true;
                        cmd.clear();
                    } else if (c == (char)kEtx) {
                        if (receiving && cmd.size() <= MAX_DOC) {
                            string response;
                            if (handleCommand(cmd, response)) {
                                writeFramed(response);
                            }
                        }
                        receiving = false;
                        cmd.clear();
                    } else if (receiving) {
                        if (cmd.size() < MAX_DOC) cmd.push_back(c);
                    }
                }
            }
        }

        void DebugLink::Start() {
            if (hDebugTask != nullptr) return;

            // Both buffers must be > 0; TX must hold the ~2.1 KB
            // framebuffer reply in one piece.
            usb_serial_jtag_driver_config_t cfg = {
                .tx_buffer_size = 4096,
                .rx_buffer_size = 4096,
            };
            esp_err_t err = usb_serial_jtag_driver_install(&cfg);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "usj driver install: %s", esp_err_to_name(err));
                return;
            }
            esp_log_set_vprintf(usjLogWrite);

            if (xTaskCreate(debugTask, "debug_link", 6144, nullptr,
                            tskIDLE_PRIORITY + 1, &hDebugTask) != pdPASS) {
                ESP_LOGW(TAG, "could not start debug link task");
                hDebugTask = nullptr;
                return;
            }
            ESP_LOGI(TAG, "bench channel on USB console (STX/ETX JSON)");
        }
    }
}
