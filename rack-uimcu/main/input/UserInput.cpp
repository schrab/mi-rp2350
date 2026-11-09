/***************
Rack UI controller — T5 input implementation. The quadrature half is the
seed UserInput approach verbatim (GPIO-ISR detent table + poll-task drain,
2 quarter-cycles per detent); the push button is a 10 ms polled GPIO with
20 ms debounce and the 600 ms hold threshold (the rack gesture timing,
rack-uimcu/README.md).

Strapping-pin notes (GPIO8/9): both carry external pull-ups on the carrier;
the encoder must never be held in a low state at reset (GPIO8 low = bad
strap, GPIO9 held at power-on = download mode). Internal pull-ups are also
enabled here as belt-and-braces.
***************/

#include "UserInput.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define DEBOUNCE_MS 20
#define HOLD_MS 350          // responsive hold threshold (was 600)
#define POLL_MS 10

static const char *TAG = "UserInput";

namespace CTAG {
    namespace CTRL {

        namespace {
            QueueHandle_t evQueue = nullptr;
            volatile bool s_pressed = false;
            volatile bool s_pushRaw = false;   // fed by RackPots (mux-scan
                                               // push detection, GPIO3)

            // ---- encoder hardware map (rack GPIO allocation, README) ----
            // 2026-11-09 final map: the display took GPIO10/20/21 (the SPI
            // path) and CS=6/RST=7 — the encoder lands on the freed RTC
            // pads 0/1 (inputs work fine there) + push on GPIO3 SHARED with
            // the mux SIG line via a 4.7k series resistor (RackPots detects
            // the press as an all-channels dip and injects the events).
            constexpr gpio_num_t ENC_A = GPIO_NUM_0;
            constexpr gpio_num_t ENC_B = GPIO_NUM_1;
            constexpr gpio_num_t ENC_PUSH = GPIO_NUM_3;  // shared w/ mux SIG

            // Quadrature detent transitions: index (prev<<2)|cur of the 2-bit
            // AB state. Full detents move two steps of the gray code, so the
            // count is halved when drained (one event per detent). Invalid
            // (double-transition) pairs emit 0 — this is what rejects bounce.
            // (Seed UserInput table, verbatim.)
            const int8_t kQuadTable[16] = {
                0,  // 00 -> 00
                +1, // 00 -> 01
                -1, // 00 -> 10
                0,  // 00 -> 11 (bounce)
                -1, // 01 -> 00
                0,  // 01 -> 01
                0,  // 01 -> 10 (bounce)
                +1, // 01 -> 11
                +1, // 10 -> 00
                0,  // 10 -> 01 (bounce)
                0,  // 10 -> 10
                -1, // 10 -> 11
                0,  // 11 -> 00 (bounce)
                -1, // 11 -> 01
                +1, // 11 -> 10
                0,  // 11 -> 11
            };

            struct QuadEncoder {
                gpio_num_t pinA;
                gpio_num_t pinB;
                volatile int32_t count = 0; // raw quarter-cycles, signed
                uint8_t state = 0;
            };
            QuadEncoder s_enc{ENC_A, ENC_B};
            // Guards the count drain against the ISR (single core: the
            // critical section masks interrupts).
            portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

            uint8_t readAB(QuadEncoder &e) {
                return (uint8_t)((gpio_get_level(e.pinA) << 1) | gpio_get_level(e.pinB));
            }

            void encoderIsr(void *arg) {
                QuadEncoder *e = (QuadEncoder *)arg;
                uint8_t cur = (uint8_t)((gpio_get_level(e->pinA) << 1) | gpio_get_level(e->pinB));
                e->count += kQuadTable[(e->state << 2) | cur];
                e->state = cur;
            }

            void encoderInit(QuadEncoder &e) {
                gpio_config_t io = {};
                uint64_t mask = (1ULL << e.pinA) | (1ULL << e.pinB);
                io.pin_bit_mask = mask;
                io.mode = GPIO_MODE_INPUT;
                io.pull_up_en = GPIO_PULLUP_ENABLE;  // external pull-ups expected too
                io.pull_down_en = GPIO_PULLDOWN_DISABLE;
                io.intr_type = GPIO_INTR_ANYEDGE;
                gpio_config(&io);
                e.state = readAB(e);
                ESP_ERROR_CHECK(gpio_isr_handler_add(e.pinA, encoderIsr, &e));
                ESP_ERROR_CHECK(gpio_isr_handler_add(e.pinB, encoderIsr, &e));
            }

            void inputTask(void *) {
                uint32_t pressTs = 0;      // tick the press started
                bool holdEmitted = false;
                bool debouncing = false;
                uint32_t lastEdgeTs = 0;
                int32_t carry = 0;

                while (true) {
                    const uint32_t now = xTaskGetTickCount();

                    // Drain encoder quarter-cycles under a critical section
                    // so no ISR increment is lost; two quarters = one detent.
                    taskENTER_CRITICAL(&encMux);
                    int32_t q = s_enc.count;
                    s_enc.count = 0;
                    taskEXIT_CRITICAL(&encMux);
                    carry += q;
                    int d = carry / 2;
                    carry %= 2;
                    if (d != 0) {
                        InputEvent ev;
                        ev.type = InputEvent::ENC_DELTA;
                        ev.delta = (int16_t)d;
                        xQueueSend(evQueue, &ev, 0);
                    }

                    // Push button: active low, fed by RackPots's mux-scan
                    // press detection (GPIO3 shared with the mux SIG).
                    const bool raw = s_pushRaw;
                    if (raw != s_pressed && !debouncing) {
                        debouncing = true;
                        lastEdgeTs = now;
                    } else if (debouncing &&
                               (now - lastEdgeTs) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
                        debouncing = false;
                        if (raw != s_pressed) {
                            s_pressed = raw;
                            if (raw) {
                                pressTs = now;
                                holdEmitted = false;
                            } else {
                                // Release: short push, or the confirm/cancel
                                // release after a hold.
                                InputEvent ev;
                                ev.delta = 0;
                                ev.type = holdEmitted ? InputEvent::PUSH_RELEASE
                                                      : InputEvent::PUSH_SHORT;
                                xQueueSend(evQueue, &ev, 0);
                            }
                        }
                    }
                    // Hold threshold fires ONCE while the button stays down.
                    if (s_pressed && !holdEmitted && !debouncing &&
                        (now - pressTs) >= pdMS_TO_TICKS(HOLD_MS)) {
                        holdEmitted = true;
                        InputEvent ev;
                        ev.type = InputEvent::PUSH_HOLD;
                        ev.delta = 0;
                        xQueueSend(evQueue, &ev, 0);
                    }

                    vTaskDelay(pdMS_TO_TICKS(POLL_MS));
                }
            }
        }

        bool UserInput::IsPressed() { return s_pressed; }

        void UserInput::SetPushRaw(bool pressed) { s_pushRaw = pressed; }

        void UserInput::Init() {
            evQueue = xQueueCreate(64, sizeof(InputEvent));

            // GPIO ISR service for the quadrature encoder
            ESP_ERROR_CHECK(gpio_install_isr_service(0));
            encoderInit(s_enc);

            // The push button lives on GPIO3 SHARED with the mux SIG line
            // (through a 4.7k series resistor) — RackPots's scan detects
            // the press (an all-channels dip signature) and feeds the raw
            // state here via SetPushRaw; this task keeps the debounce +
            // hold/short timing. NO gpio_config here: GPIO3 is the ADC pad.
            s_pushRaw = false;

            if (xTaskCreate(inputTask, "input_task", 2048, nullptr,
                            tskIDLE_PRIORITY + 3, nullptr) != pdPASS) {
                ESP_LOGE(TAG, "could not create input task");
            }
        }

        bool UserInput::GetEvent(InputEvent &ev, uint32_t timeoutMs) {
            return xQueueReceive(evQueue, &ev, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
        }

        bool UserInput::PeekEvent(InputEvent &ev) {
            return xQueuePeek(evQueue, &ev, 0) == pdTRUE;
        }

        bool UserInput::InjectEvent(InputEvent::Type type, int16_t delta) {
            if (evQueue == nullptr) return false;
            InputEvent ev;
            ev.type = type;
            ev.delta = delta;
            return xQueueSend(evQueue, &ev, 0) == pdTRUE;
        }
    }
}
