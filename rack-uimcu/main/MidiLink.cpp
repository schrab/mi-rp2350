/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

***************/

#include "MidiLink.hpp"
#include "LinkClient.hpp"
#include "BtMidiReceiver.hpp"
#include "DspProxy.hpp"
#include "ctag/link/protocol.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "MidiLink";

namespace CTAG {
    namespace CTRL {

        using namespace CTAG::LINK;
        using CTAG::DRIVERS::BtMidiReceiver;

        namespace {
            constexpr uart_port_t kUart = UART_NUM_1;
            constexpr int kMidiBaud = 31250;
            // TRS jacks (RP-054 Type A): TX=GPIO9 (OUT; remapped from 1 —
            // GPIO9 is a strap pin, and an idle-high UART TX reads it high
            // at reset; GPIO1 went to the button ladder), RX=GPIO10 (IN,
            // through the opto). UART pins are fully matrixed on the C3.
            constexpr int kPinTx = 9;
            constexpr int kPinRx = 10;
            constexpr size_t kRxRing = 1024;
            constexpr size_t kTxRing = 512;
            // Batch window: one relay frame per read; the read blocks for
            // one tick so quiet periods cost one empty task round.
            constexpr size_t kBatchMax = 64;

            TaskHandle_t hMidiTask = nullptr;

            void midiTask(void *) {
                uint8_t raw[kBatchMax];
                char frame[64 + 8 * kBatchMax];
                auto relay = [&frame](const uint8_t *bytes, size_t n) {
                    if (n == 0 || !LinkClient::IsOnline()) return;
                    static uint32_t pkts = 0;   // relay probe (temporary)
                    pkts++;
                    if (pkts <= 10 || (pkts % 32) == 0)
                        ESP_LOGI(TAG, "relay #%u (%u B): %02x %02x %02x %02x",
                                 (unsigned) pkts, (unsigned) n,
                                 bytes[0], n > 1 ? bytes[1] : 0,
                                 n > 2 ? bytes[2] : 0, n > 3 ? bytes[3] : 0);
                    int used = snprintf(frame, sizeof(frame),
                                        "{\"cmd\":\"%s\",\"d\":[", kMsgMidiIn);
                    for (size_t i = 0; i < n && used < (int) sizeof(frame) - 8; i++)
                        used += snprintf(frame + used, sizeof(frame) - used,
                                         "%s%u", i ? "," : "", (unsigned) bytes[i]);
                    snprintf(frame + used, sizeof(frame) - used, "]}");
                    LinkClient::SendCommand(frame);
                };

                while (true) {
                    // P4: persistent BLE peer — retry the stored device
                    // while idle (no-op once connected or scanning).
                    BtMidiReceiver::TickReconnect();
                    // P4: BLE-MIDI keepalive — the SMC-PAD only starts its
                    // notification stream once the client writes to it
                    // (the Ableton session paces LED writes constantly).
                    // A real-time clock tick every 250 ms keeps it talking.
                    static uint32_t lastKa = 0;
                    if (BtMidiReceiver::IsConnected() && DspProxy::IsMidiBleEnabled() &&
                        xTaskGetTickCount() - lastKa > pdMS_TO_TICKS(250)) {
                        lastKa = xTaskGetTickCount();
                        uint8_t ka[3] = {0x80, 0x00, 0xF8};   // ts header + clock tick
                        BtMidiReceiver::SendKeepalive(ka, sizeof(ka));
                    }
                    // P4: BLE-MIDI relay (the central receiver runs here;
                    // the DSP-side BLE flag gates the drain).
                    if (DspProxy::IsMidiBleEnabled()) {
                        uint8_t bbuf[64];
                        uint32_t bn = 0;
                        BtMidiReceiver::Read(bbuf, &bn);
                        relay(bbuf, bn);
                    }
                    int got = uart_read_bytes(kUart, raw, sizeof(raw),
                                              pdMS_TO_TICKS(10));
                    if (got <= 0) continue;
                    relay(raw, (size_t) got);
                }
            }
        }

        void MidiLink::Start() {
            if (hMidiTask != nullptr) return;
            uart_config_t uc = {};
            uc.baud_rate = kMidiBaud;
            uc.data_bits = UART_DATA_8_BITS;
            uc.parity = UART_PARITY_DISABLE;
            uc.stop_bits = UART_STOP_BITS_1;
            uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            uc.rx_flow_ctrl_thresh = 0;
            uc.source_clk = UART_SCLK_DEFAULT;
            ESP_ERROR_CHECK(uart_set_pin(kUart, kPinTx, kPinRx,
                                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_param_config(kUart, &uc));
            ESP_ERROR_CHECK(uart_driver_install(kUart, kRxRing, kTxRing, 0,
                                                nullptr, 0));
            if (xTaskCreate(midiTask, "midi_in", 3072, nullptr,
                            tskIDLE_PRIORITY + 2, &hMidiTask) != pdPASS) {
                ESP_LOGW(TAG, "could not start midi relay task");
                hMidiTask = nullptr;
                return;
            }
            ESP_LOGI(TAG, "TRS MIDI on UART1 TX%d/RX%d @%d",
                     kPinTx, kPinRx, kMidiBaud);
        }

        void MidiLink::OnMidiOut(const uint8_t *data, size_t n) {
            if (n == 0) return;
            uart_write_bytes(kUart, data, n);
        }
    }
}
