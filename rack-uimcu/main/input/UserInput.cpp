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

// C3 panel input implementation. Encoders: GPIO-ISR quadrature (the C3 has
// no PCNT) with the detent state table rejecting bounce; accumulated counts
// are drained by the poll task like the A1S PCNT ReadDelta(). Buttons: the
// A1S ADC-ladder state machine (UserInput.cpp readADCState) copied verbatim
// — BTN_A emits BTN2_* (OK/MOD), BTN_B emits BTN1_* (BACK), events on
// release, 500 ms long-press threshold.
//
// Bring-up notes (SDD §10): E1 sits on strapping pins GPIO8/9 — the board
// needs external pull-ups on both encoder pins and must not be held at
// reset; power-on with encoders in random states is a P2 test. The ADC
// thresholds below mirror the A1S BTN ladder topology; re-measure against
// the actual resistor values when the panel is wired.

#include "UserInput.hpp"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define DEBOUNCE_MS 20
#define LONG_PRESS_MS 500
#define POLL_MS 10

static const char *TAG = "UserInput";

namespace CTAG {
    namespace CTRL {

        namespace {
            QueueHandle_t evQueue = nullptr;

            // ---- encoder hardware map (E1 remapped off strap pins 8/9:
            // a detent shorted both low → invalid boot strap; E1 now owns
            // the freed BTN-ADC pin GPIO0 + OLED-DC's old GPIO3) ----
            constexpr gpio_num_t ENC1_A = GPIO_NUM_0;
            constexpr gpio_num_t ENC1_B = GPIO_NUM_3;
            constexpr gpio_num_t ENC2_A = GPIO_NUM_4;
            constexpr gpio_num_t ENC2_B = GPIO_NUM_5;

            // Quadrature detent transitions: index (prev<<2)|cur of the 2-bit
            // AB state. Full detents move two steps of the gray code, so the
            // count is halved when drained (matches one event per detent).
            // Invalid (double-transition) pairs emit 0, which is what rejects
            // contact bounce.
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
            QuadEncoder enc1{ENC1_A, ENC1_B};
            QuadEncoder enc2{ENC2_A, ENC2_B};
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
                io.pull_up_en = GPIO_PULLUP_ENABLE; // external pull-ups expected too
                io.pull_down_en = GPIO_PULLDOWN_DISABLE;
                io.intr_type = GPIO_INTR_ANYEDGE;
                gpio_config(&io);
                e.state = readAB(e);
                ESP_ERROR_CHECK(gpio_isr_handler_add(e.pinA, encoderIsr, &e));
                ESP_ERROR_CHECK(gpio_isr_handler_add(e.pinB, encoderIsr, &e));
            }

            // ---- button ADC ladder (A1S UserInput.cpp:41–66 model) ----
            enum BtnADCState { BTN_NONE, BTN_SW1, BTN_SW2, BTN_BOTH };

            BtnADCState currentADCState = BTN_NONE;

            // Thresholds measured 2026-10-28 on the panel ladder via
            // /debug/getBtnAdc: idle 4095, BTN_A ~877, BTN_B ~415, both
            // ~280 (stable within ±10 counts). Hysteresis windows per
            // state; SW1=BTN_A(OK/MOD), SW2=BTN_B(BACK).
            BtnADCState readADCState(BtnADCState prev, int adc) {
                switch (prev) {
                    case BTN_NONE:
                        if (adc < 350) return BTN_BOTH;
                        if (adc < 650) return BTN_SW2;
                        if (adc < 1500) return BTN_SW1;
                        return BTN_NONE;
                    case BTN_SW1:
                        if (adc > 2000) return BTN_NONE;
                        if (adc < 350) return BTN_BOTH;
                        if (adc < 650) return BTN_SW2;
                        return BTN_SW1;
                    case BTN_SW2:
                        if (adc > 2000) return BTN_NONE;
                        if (adc < 350) return BTN_BOTH;
                        if (adc > 700) return BTN_SW1;
                        return BTN_SW2;
                    case BTN_BOTH:
                        if (adc > 2000) return BTN_NONE;
                        if (adc > 550) return BTN_SW1;
                        if (adc > 350) return BTN_SW2;
                        return BTN_BOTH;
                }
                return BTN_NONE;
            }

            void inputTask(void *) {
                uint32_t lastDebounceTs = 0;
                uint32_t sw1PressTs = 0;
                uint32_t sw2PressTs = 0;
                int32_t enc1Carry = 0;
                int32_t enc2Carry = 0;

                while (true) {
                    uint32_t now = xTaskGetTickCount();

                    // Drain encoder quarter-cycles under a critical section so
                    // no ISR increment is lost; two quarters = one detent.
                    // E1 -> ENC_DELTA (in-page value), E2 -> ENC2_DELTA (panel).
                    taskENTER_CRITICAL(&encMux);
                    int32_t q1 = enc1.count;
                    enc1.count = 0;
                    int32_t q2 = enc2.count;
                    enc2.count = 0;
                    taskEXIT_CRITICAL(&encMux);
                    enc1Carry += q1;
                    enc2Carry += q2;
                    int d1 = enc1Carry / 2;
                    enc1Carry %= 2;
                    int d2 = enc2Carry / 2;
                    enc2Carry %= 2;
                    if (d1 != 0) {
                        InputEvent ev;
                        ev.type = InputEvent::ENC_DELTA;
                        ev.delta = (int16_t)d1;
                        xQueueSend(evQueue, &ev, 0);
                    }
                    if (d2 != 0) {
                        InputEvent ev;
                        ev.type = InputEvent::ENC2_DELTA;
                        ev.delta = (int16_t)d2;
                        xQueueSend(evQueue, &ev, 0);
                    }

                    // read ADC buttons with 4-sample averaging
                    int sum = 0;
                    for (int i = 0; i < 4; i++) {
                        sum += adc1_get_raw(ADC1_CHANNEL_1);
                    }
                    int adc = sum / 4;
                    BtnADCState newState = readADCState(currentADCState, adc);

                    if (newState != currentADCState && (now - lastDebounceTs) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
                        BtnADCState old = currentADCState;
                        InputEvent ev;
                        ev.delta = 0;

                        // Both -> None: no events
                        if (old == BTN_BOTH && newState == BTN_NONE) {
                            // nothing
                        }
                        // Both -> SW1: fresh timer for remaining SW1
                        else if (old == BTN_BOTH && newState == BTN_SW1) {
                            sw1PressTs = now;
                        }
                        // Both -> SW2: fresh timer for remaining SW2
                        else if (old == BTN_BOTH && newState == BTN_SW2) {
                            sw2PressTs = now;
                        }
                        // -> Both: no events, fresh timers
                        else if (newState == BTN_BOTH) {
                            if (old == BTN_SW1) {
                                sw2PressTs = now;
                            } else if (old == BTN_SW2) {
                                sw1PressTs = now;
                            } else {
                                sw1PressTs = now;
                                sw2PressTs = now;
                            }
                        }
                        // SW1 released (physical OK / BTN_A)
                        else if (old == BTN_SW1 && newState == BTN_NONE) {
                            ev.type = (now - sw1PressTs >= pdMS_TO_TICKS(LONG_PRESS_MS))
                                ? InputEvent::BTN2_LONG : InputEvent::BTN2_SHORT;
                            xQueueSend(evQueue, &ev, 0);
                        }
                        // SW1 pressed (physical OK / BTN_A)
                        else if (old == BTN_NONE && newState == BTN_SW1) {
                            sw1PressTs = now;
                        }
                        // SW1 -> SW2 (release OK, press BACK)
                        else if (old == BTN_SW1 && newState == BTN_SW2) {
                            ev.type = (now - sw1PressTs >= pdMS_TO_TICKS(LONG_PRESS_MS))
                                ? InputEvent::BTN2_LONG : InputEvent::BTN2_SHORT;
                            xQueueSend(evQueue, &ev, 0);
                            sw2PressTs = now;
                        }
                        // SW2 released (physical BACK / BTN_B)
                        else if (old == BTN_SW2 && newState == BTN_NONE) {
                            ev.type = (now - sw2PressTs >= pdMS_TO_TICKS(LONG_PRESS_MS))
                                ? InputEvent::BTN1_LONG : InputEvent::BTN1_SHORT;
                            ESP_LOGI(TAG, "BTN_B release %s adc=%d",
                                     ev.type == InputEvent::BTN1_LONG ? "LONG" : "SHORT", adc);
                            xQueueSend(evQueue, &ev, 0);
                        }
                        // SW2 pressed (physical BACK / BTN_B)
                        else if (old == BTN_NONE && newState == BTN_SW2) {
                            sw2PressTs = now;
                        }
                        // SW2 -> SW1 (release BACK, press OK)
                        else if (old == BTN_SW2 && newState == BTN_SW1) {
                            ev.type = (now - sw2PressTs >= pdMS_TO_TICKS(LONG_PRESS_MS))
                                ? InputEvent::BTN1_LONG : InputEvent::BTN1_SHORT;
                            xQueueSend(evQueue, &ev, 0);
                            sw1PressTs = now;
                        }

                        currentADCState = newState;
                        lastDebounceTs = now;
                    }

                    vTaskDelay(pdMS_TO_TICKS(POLL_MS));
                }
            }
        }

        // Button-ladder calibration aid (bench channel reads this).
        int UserInputDebugBtnAdc() {
            int sum = 0;
            for (int i = 0; i < 8; i++)
                sum += adc1_get_raw(ADC1_CHANNEL_1);
            return sum / 8;
        }

        void UserInput::Init() {
            evQueue = xQueueCreate(64, sizeof(InputEvent));

            // GPIO ISR service for the two quadrature encoders
            ESP_ERROR_CHECK(gpio_install_isr_service(0));
            encoderInit(enc1);
            encoderInit(enc2);

            // Configure ADC1 channel 1 (GPIO1, remapped from GPIO0 — that
            // pin went to encoder E1) for button sensing. The weak
            // internal pull-up idles the pin at BTN_NONE when nothing is
            // wired (a bare board floats otherwise and the ADC state machine
            // emits garbage button events); the wired ladder overdrives it.
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
            gpio_pullup_en(GPIO_NUM_1);

            if (xTaskCreate(inputTask, "input_task", 2048, nullptr, tskIDLE_PRIORITY + 3, nullptr) != pdPASS) {
                ESP_LOGE(TAG, "could not create input task");
            }
        }

        // ISRs are installed in Init() on the C3 (GPIO path, no audio-critical
        // startup ordering to respect like the A1S).
        void UserInput::EnableISR() {}

        bool UserInput::GetEvent(InputEvent& ev, uint32_t timeoutMs) {
            return xQueueReceive(evQueue, &ev, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
        }

        bool UserInput::PeekEvent(InputEvent& ev) {
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
