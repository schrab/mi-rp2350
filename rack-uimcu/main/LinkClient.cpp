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

#include "LinkClient.hpp"
#include "DspProxy.hpp"
#include "version.hpp"
#include "ctag/link/protocol.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <cstring>
#include <string>

using namespace std;
using namespace CTAG::LINK;

static const char *TAG = "LinkCli";

namespace CTAG {
    namespace CTRL {

        namespace {
            constexpr uart_port_t kUart = UART_NUM_0;
            constexpr int kLinkBaud = 921600;
            // RX ring must ride out the boot-metadata burst (params JSON can
            // be tens of KB) while the task drains continuously.
            constexpr size_t kRxRing = 16384;
            constexpr size_t kTxRing = 8192;
            constexpr uint32_t kOfflineAfterMs = 3000;
            constexpr uint32_t kHelloPeriodMs = 1000;
            constexpr uint32_t kHeartbeatPeriodMs = 1000;

            TaskHandle_t hLinkTask = nullptr;
            volatile bool s_online = false;
            uint32_t lastDspRxMs = 0;
            uint32_t lastHelloMs = 0;
            uint32_t lastHeartbeatMs = 0;
            uint32_t heartbeatSeq = 0;
            uint32_t fsSeq = 0;

            // Pending fsList round trip (only one at a time; the UIMenu task
            // is the only caller and it is user-paced).
            SemaphoreHandle_t fsDone = nullptr;
            volatile bool fsWaiting = false;
            uint32_t fsWaitingSeq = 0;
            string fsResult;

            uint32_t nowMs() {
                return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            }

            void sendFrame(const char *p, size_t len) {
                uart_write_bytes(kUart, "\x02", 1);
                uart_write_bytes(kUart, p, len);
                uart_write_bytes(kUart, "\x03", 1);
            }

            void linkTask(void *) {
                string payload;
                bool receiving = false;
                uint8_t rx[256];

                while (true) {
                    uint32_t now = nowMs();

                    int got = uart_read_bytes(kUart, rx, sizeof(rx), 0);
                    if (got > 0) {
                        lastDspRxMs = now;
                        for (int i = 0; i < got; i++) {
                            char c = (char)rx[i];
                            if (c == (char)kStx) {
                                receiving = true;
                                payload.clear();
                            } else if (c == (char)kEtx) {
                                if (receiving && payload.size() < 128 * 1024) {
                                    // The DSP's hello_ack opens the session;
                                    // everything else belongs to the cache.
                                    HelloReply hello = ParseHandshake(payload.data(), payload.size());
                                    if (hello.valid && hello.isAck && hello.role == Role::kDsp) {
                                        if (hello.accepted) {
                                            if (!s_online)
                                                ESP_LOGI(TAG, "DSP link online (proto %u.%u)",
                                                         hello.major, hello.minor);
                                            s_online = true;
                                            lastHelloMs = now;
                                            lastHeartbeatMs = now;
                                        } else {
                                            ESP_LOGE(TAG, "DSP refused handshake");
                                            s_online = false;
                                        }
                                    } else {
                                        DspProxy::OnLinkFrame(payload.data(), payload.size());
                                    }
                                }
                                receiving = false;
                                payload.clear();
                            } else if (receiving) {
                                payload.push_back(c);
                            }
                        }
                    }

                    // Offline watchdog.
                    if (s_online && (now - lastDspRxMs) > kOfflineAfterMs) {
                        s_online = false;
                        ESP_LOGW(TAG, "DSP went quiet — link offline");
                    }

                    // Handshake retry while offline; heartbeat while online.
                    if (!s_online) {
                        if (now - lastHelloMs >= kHelloPeriodMs) {
                            lastHelloMs = now;
                            string hello = MakeHello(Role::kPanel, TBD_FW_VERSION.c_str());
                            sendFrame(hello.data(), hello.size());
                            ESP_LOGI(TAG, "hello retry (online=0)");
                        }
                    } else if (now - lastHeartbeatMs >= kHeartbeatPeriodMs) {
                        lastHeartbeatMs = now;
                        string hb = MakeHeartbeat(now, ++heartbeatSeq);
                        sendFrame(hb.data(), hb.size());
                    }

                    // One FULL tick — pdMS_TO_TICKS(5) rounds to zero at
                    // FREERTOS_HZ=100 and busy-spins (see A1S LinkEndpoint).
                    vTaskDelay(1);
                }
            }
        }

        bool LinkClient::IsOnline() { return s_online; }

        bool LinkClient::SetBaud(int baud) {
            if (baud < 1200 || baud > 5000000) return false;
            if (uart_set_baudrate(kUart, baud) != ESP_OK) return false;
            ESP_LOGI(TAG, "link baud retuned to %d", baud);
            return true;
        }

        void LinkClient::Start() {
            if (hLinkTask != nullptr) return;

            uart_config_t uc = {};
            uc.baud_rate = kLinkBaud;
            uc.data_bits = UART_DATA_8_BITS;
            uc.parity = UART_PARITY_DISABLE;
            uc.stop_bits = UART_STOP_BITS_1;
            uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            uc.rx_flow_ctrl_thresh = 0;
            uc.source_clk = UART_SCLK_DEFAULT;
            // SDD §2.1: link on UART0 default pins, TX=GPIO21, RX=GPIO20.
            ESP_ERROR_CHECK(uart_set_pin(kUart, 21, 20, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_param_config(kUart, &uc));
            ESP_ERROR_CHECK(uart_driver_install(kUart, kRxRing, kTxRing, 0, nullptr, 0));

            fsDone = xSemaphoreCreateBinary();

            if (xTaskCreate(linkTask, "link_cli", 8192, nullptr,
                            tskIDLE_PRIORITY + 2, &hLinkTask) != pdPASS) {
                ESP_LOGW(TAG, "could not start link client task");
                hLinkTask = nullptr;
                return;
            }
            ESP_LOGI(TAG, "link client on UART0 GPIO20/21 @%d", kLinkBaud);
        }

        bool LinkClient::SendCommand(const string &payload) {
            sendFrame(payload.data(), payload.size());
            return true;
        }

        bool LinkClient::RequestFsList(const char *path, string &entries,
                                       int timeoutMs) {
            if (!s_online) return false;
            fsSeq++;
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "{\"cmd\":\"/link/fsList\",\"path\":\"%s\",\"seq\":%u}",
                     path, (unsigned)fsSeq);
            fsWaitingSeq = fsSeq;
            fsResult.clear();
            fsWaiting = true;
            xSemaphoreTake(fsDone, 0); // drain a stale token
            sendFrame(buf, strlen(buf));

            if (xSemaphoreTake(fsDone, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
                fsWaiting = false;
                return false;
            }
            entries = fsResult;
            return true;
        }

        // Called by DspProxy::OnLinkFrame when an fsList reply arrives.
        void LinkClient_NotifyFsList(uint32_t seq, const char *entriesJson) {
            if (fsWaiting && seq == fsWaitingSeq) {
                fsResult = entriesJson ? entriesJson : "";
                fsWaiting = false;
                xSemaphoreGive(fsDone);
            }
        }
    }
}
