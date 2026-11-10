/***************
Rack UI controller — USB-Serial-JTAG line console implementation (see the
header). The transport pattern is the seed DebugLink's (usj driver +
esp_log vprintf hand-off); the command set is the rack's.
***************/

#include "RackConsole.hpp"

#include "PresetStore.hpp"
#include "RackLink.hpp"
#include "RackPots.hpp"
#include "RackUI.hpp"
#include "BtMidiReceiver.hpp"
#include "Display.hpp"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

static const char *TAG = "Console";

namespace CTAG {
    namespace CTRL {

        namespace {

            // After the driver owns the USJ, the ROM/vfs console cannot TX
            // reliably; hand ESP_LOG to the driver like the seed DebugUI.
            int usjLogWrite(const char *format, va_list args) {
                static char line[256];
                int len = vsnprintf(line, sizeof(line), format, args);
                if (len > 0)
                    usb_serial_jtag_write_bytes(line, (size_t)len,
                                                pdMS_TO_TICKS(100));
                return len;
            }

            TaskHandle_t hConsoleTask = nullptr;

            void printHelp() {
                printf("rack console commands:\r\n");
                printf("  ui            link + page state\r\n");
                printf("  page <0-5>    switch page (0 P1, 1 P2, 2 MOD-A, "
                       "3 MOD-B, 4 MOD-C, 5 SYS)\r\n");
                printf("  pots          raw + mapped HC4067 pot values\r\n");
                printf("  calib         8 s pot min/max sweep -> NVS\r\n");
                printf("  calib clear   reset pot calibration\r\n");
                printf("  testpat <0-4> display bring-up patterns "
                       "(1 white, 2 black, 3 hues, 4 border+ruler)\r\n");
                printf("  heap          free-heap report\r\n");
                printf("  preset        T4a bank status (save <0-15> / "
                       "load <0-15> / clear)\r\n");
            }

            void consoleTask(void *) {
                static char line[64];
                size_t len = 0;
                uint8_t rx[32];
                printf("rack console ready (type 'help')\r\n");
                while (true) {
                    const int got =
                        usb_serial_jtag_read_bytes(rx, sizeof(rx), 0);
                    if (got <= 0) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                        continue;
                    }
                    for (int i = 0; i < got; ++i) {
                        const char c = (char)rx[i];
                        if (c == '\r' || c == '\n') {
                            if (len == 0) continue;    // CR-LF pairs
                            line[len] = '\0';
                            len = 0;
                            // ---- dispatch ----
                            if (!strcasecmp(line, "help")) {
                                printHelp();
                            } else if (!strcasecmp(line, "ui")) {
                                RackUI::DebugState();
                                uint32_t tx = 0, rx = 0, crc = 0;
                                RackLink::Stats(&tx, &rx, &crc);
                                printf("link: %s | frames tx=%lu rx=%lu "
                                       "crc_err=%lu | ble=%u msgs=%lu "
                                       "drops=%lu\r\n",
                                       RackLink::IsOnline() ? "online"
                                                            : "offline",
                                       (unsigned long)tx, (unsigned long)rx,
                                       (unsigned long)crc,
                                       CTAG::DRIVERS::BtMidiReceiver::
                                           IsConnected() ? 1 : 0,
                                       (unsigned long)CTAG::DRIVERS::
                                           BtMidiReceiver::MsgCount(),
                                       (unsigned long)CTAG::DRIVERS::
                                           BtMidiReceiver::DropCount());
                            } else if (!strncasecmp(line, "page ", 5)) {
                                const int p = atoi(line + 5);
                                if (p < 0 || p > 5) {
                                    printf("page 0-5\r\n");
                                } else {
                                    RackUI::SetPage((uint8_t)p);
                                    printf("page -> %d (dashes until the "
                                           "TELEM snapshot)\r\n", p);
                                }
                            } else if (!strcasecmp(line, "pots")) {
                                printf("pot raw mapped\r\n");
                                for (int p = 0; p < 16; ++p) {
                                    printf("%3d %4d %4d\r\n", p,
                                           RackPots::DebugRaw((uint8_t)p),
                                           RackPots::DebugValue((uint8_t)p));
                                }
                            } else if (!strcasecmp(line, "calib")) {
                                RackPots::BeginCalibration();
                            } else if (!strcasecmp(line, "calib clear")) {
                                RackPots::ClearCalibration();
                            } else if (!strncasecmp(line, "testpat ", 8)) {
                                const int m = atoi(line + 8);
                                if (m < 0 || m > 4) {
                                    printf("testpat 0-4\r\n");
                                } else {
                                    CTAG::DRIVERS::Display::TestPattern(m);
                                    printf("testpat %d (0 = back to the UI)\r\n",
                                           m);
                                }
                            } else if (!strcasecmp(line, "spiflood")) {
                                printf("spiflood: minimal init + color flood "
                                       "via SPI, ~90 s\r\n");
                                CTAG::DRIVERS::Display::SpiFlood();
                                printf("spiflood done\r\n");
                            } else if (!strcasecmp(line, "osweep")) {
                                printf("osweep: 16 digits, 2.5 s each - read "
                                       "them\r\n");
                                CTAG::DRIVERS::Display::OrientationSweep();
                                printf("osweep done - REBOOT before the SPI "
                                       "display path works again\r\n");
                            } else if (!strncasecmp(line, "preset", 6)) {
                                PresetStore::ConsoleCmd(line + 6);
                            } else if (!strcasecmp(line, "heap")) {
                                printf("heap free=%u largest=%u (internal "
                                       "8-bit; FB=%u B)\r\n",
                                       (unsigned) heap_caps_get_free_size(
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                                       (unsigned) heap_caps_get_largest_free_block(
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                                       (unsigned) CTAG::DRIVERS::Display::
                                           FRAMEBUFFER_SIZE);
                            } else {
                                printf("unknown: %s (help)\r\n", line);
                            }
                        } else if (len < sizeof(line) - 1) {
                            line[len++] = c;
                        } else {
                            len = 0;               // overflow: drop the line
                        }
                    }
                }
            }

        } // namespace

        void RackConsole::Start() {
            if (hConsoleTask != nullptr) return;
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
            if (xTaskCreate(consoleTask, "rack_con", 4096, nullptr,
                            tskIDLE_PRIORITY + 1, &hConsoleTask) != pdPASS) {
                ESP_LOGW(TAG, "could not start console task");
                hConsoleTask = nullptr;
                return;
            }
            ESP_LOGI(TAG, "rack console on USB-Serial-JTAG (line-based)");
        }

    } // namespace CTRL
} // namespace CTAG
