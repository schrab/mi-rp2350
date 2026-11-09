/***************
Rack UI controller (ESP32-C3) — T5 stages 2-6 entry point. Seeded from
ctag-tbd/uimcu (the ctag menu/DspProxy/net stack was excluded from the
build — see main/CMakeLists.txt); the rack UI owns this flow:

  1. Display (ST7789P3 284x76, SPI2 SCK=0 MOSI=1 DC=2)
  2. RackLink (RP2350 framed UART, GPIO4/5 @921600, boot self-tests)
  3. RackConsole (USB-Serial-JTAG line console: ui/page/pots/calib/testpat)
  4. RackUI render task (fixed pages, TELEM-driven, ~60 Hz)
  5. RackPots (HC4067 scan task, send-on-move -> MIDI/MOD_SET/ASSIGN)
  6. BtMidiReceiver (NimBLE PERIPHERAL — BLE-MIDI -> RackLink::SendMidi)

Ordering notes kept from the seed: the UI task starts before NimBLE so the
render task's stack + the framebuffer come off the fattest heap first; the
console's USJ driver takes over the boot log wire early. Heap before/after
NimBLE is logged (README bring-up: NimBLE + 43 KB FB + link rings in the
C3's ~320 KB).
***************/

#include "Display.hpp"
#include "RackUI.hpp"
#include "RackLink.hpp"
#include "RackPots.hpp"
#include "RackConsole.hpp"
#include "BtMidiReceiver.hpp"
#include "input/UserInput.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "uimcu";

extern "C" void app_main() {
    ESP_LOGI(TAG, "rack ui controller boot (C3, T5 stages 2-6)");

    // 0. PANEL BUS HOUSEKEEPING (2026-11-09): hold the display bus quiet
    // from the first instruction — data pads LOW, CS deasserted (high)
    // until the SPI driver takes it over in Init(). Pins: SCK=10, MOSI=20,
    // DC=21, CS=7; RES is a hardware 10k pull-up (not driven).
    {
        gpio_config_t io = {};
        io.pin_bit_mask = (1ULL << 10) | (1ULL << 20) | (1ULL << 21);
        io.mode = GPIO_MODE_OUTPUT;
        io.pull_down_en = GPIO_PULLDOWN_ENABLE;
        gpio_config(&io);
        gpio_set_level((gpio_num_t)10, 0);  // SCK idle low (SPI mode 0)
        gpio_set_level((gpio_num_t)20, 0);  // MOSI low
        gpio_set_level((gpio_num_t)21, 0);  // DC low (command)
        io.pin_bit_mask = 1ULL << 7;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io);
        gpio_set_level((gpio_num_t)7, 1);   // CS deasserted
    }

    // 1. Display backend (ST7789P3 bar; init + first flush inside).
    CTAG::DRIVERS::Display::Init();

    // 2. The RP2350 link (stage 1, extended): UART0 GPIO4/5 @921600, boot
    // self-tests (codec + uart_set_loop_back) log once; the task probes P1
    // page requests every 5 s until the cross-board link is confirmed.
    CTAG::CTRL::RackLink::Start();

    // 3. Console (USJ driver + log hand-off) — bring-up commands live here.
    CTAG::CTRL::RackConsole::Start();

    // 4. Encoder/push input (queue + gpio + poll task) BEFORE the UI task —
    // RackUI::TaskFunction's first statement polls GetEvent; the only seed
    // call site of UserInput::Init was in the EXCLUDED UIMenu framework, so
    // the queue was never created and the first poll assert-crashed
    // (bench-caught 2026-11-08: xQueueReceive(pxQueue) boot loop).
    CTAG::CTRL::UserInput::Init();

    // UI task FIRST (before NimBLE eats the heap; the seed's ordering
    // lesson): Init draws the dash placeholder page + status strip and
    // sends the first P1 page request; the task serves gestures + TELEM
    // renders at ~60 Hz.
    CTAG::CTRL::RackUI::Init();
    if (xTaskCreatePinnedToCore(CTAG::CTRL::RackUI::TaskFunction, "rack_ui",
                                8192, nullptr, tskIDLE_PRIORITY + 3, nullptr,
                                0) != pdPASS) {
        ESP_LOGE(TAG, "could not create rack_ui task");
    }

    // 5. HC4067 pot scan (ADC1_CH0 + 4 addr lines; NVS calibration).
    CTAG::CTRL::RackPots::Start();

    // 6. BLE-MIDI peripheral (NimBLE) LAST — it takes the big heap chunk.
    ESP_LOGI(TAG, "heap before NimBLE: free=%u largest=%u",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL |
                                                MALLOC_CAP_8BIT),
             (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL |
                                                         MALLOC_CAP_8BIT));
    CTAG::DRIVERS::BtMidiReceiver::Init();
    // WiFi/REST: OFF (same verdict as the seed — NimBLE + FB + tasks do not
    // leave room for the AP; BLE-MIDI is the requirement).
}
