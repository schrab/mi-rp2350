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

#include "UIMenu.hpp"
#include "input/UserInput.hpp"
#include "Display.hpp"
#include "DspProxy.hpp"
#include "menupages/UIMenuPageHome.hpp"
#include "menupages/UIMenuPageMix.hpp"
#include "menupages/UIMenuPageParams.hpp"
#include "menupages/UIMenuPageMidi.hpp"
#include "menupages/UIMenuPageTape.hpp"
#include "menupages/UIMenuPageMod.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char *TAG = "UIMenu";

using namespace CTAG::DRIVERS;

namespace CTAG {
    namespace CTRL {
        UIMenu::Panel UIMenu::currentPanel = UIMenu::PANEL_HOME;
        UIMenuPage *UIMenu::pages[6] = {nullptr, nullptr, nullptr, nullptr, nullptr};
        UIMenu::NavState UIMenu::navState = UIMenu::ROOT;
        bool UIMenu::redrawNeeded = true;
        int UIMenu::panelBarTimer = 0;
        int UIMenu::screensaverTimer = 0;
        int UIMenu::pageTickCounter = 0;
        volatile bool UIMenu::fbShotPending = false;
        volatile bool UIMenu::fbShotReady = false;
        uint8_t UIMenu::fbShotBuf[CTAG::DRIVERS::Display::FRAMEBUFFER_SIZE];
        UIMenu::DisplayState UIMenu::displayState = UIMenu::AWAKE;
        bool UIMenu::screensaverEnabled = false; // bring-up default; see TaskFunction

        void UIMenu::Init() {
            ESP_LOGI(TAG, "Init: creating pages...");
            pages[PANEL_HOME] = new UIMenuPageHome();
            pages[PANEL_MIX] = new UIMenuPageMix();
            pages[PANEL_TAPE] = new UIMenuPageTape();
            pages[PANEL_MOD] = new UIMenuPageMod();
            pages[PANEL_PARAMS] = new UIMenuPageParams();
            pages[PANEL_MIDI] = new UIMenuPageMidi();
            ESP_LOGI(TAG, "Init: calling init on pages...");
            for (int i = 0; i < PANEL_COUNT; i++) pages[i]->init();
            navState = ROOT;
            redrawNeeded = true;
            panelBarTimer = 50; // ~1s
            // Do initial redraw before task starts so display clears splash immediately
            pages[PANEL_HOME]->doRedraw();
            drawPanelBar();
            ESP_LOGI(TAG, "Init: done");
            // task created in main.cpp
            // UserInput::EnableISR() called after task creation in main.cpp
        }

        void UIMenu::GetDebugState(int &navOut, int &panelOut) {
            navOut = (int)navState;
            panelOut = (int)currentPanel;
        }

        void UIMenu::RequestFramebufferShot() {
            fbShotReady = false;
            fbShotPending = true;
        }

        bool UIMenu::CopyFramebufferShot(uint8_t *dst, uint32_t timeoutMs) {
            if (dst == nullptr) return false;
            RequestFramebufferShot();
            uint32_t start = xTaskGetTickCount();
            while (!fbShotReady) {
                if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeoutMs)) return false;
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            memcpy(dst, fbShotBuf, CTAG::DRIVERS::Display::FRAMEBUFFER_SIZE);
            return true;
        }

        void UIMenu::drawPanelBar() {
            if (panelBarTimer > 0) {
                // Norns-style indicator bar at top: 20px wide rectangles
                int segWidth = 128 / PANEL_COUNT;
                int rectW = 20;
                for (int i = 0; i < PANEL_COUNT; i++) {
                    int rx = i * segWidth + (segWidth - rectW) / 2;
                    if (i == currentPanel) {
                        // Active: filled rectangle
                        Display::DrawRect(rx, 0, rectW, 3, true, true);
                    } else {
                        // Inactive: 1px border rectangle
                        Display::DrawRect(rx, 0, rectW, 3, false, true);
                    }
                }
                panelBarTimer--;
            }

            // CPU overload indicator at top-right corner (unconditional)
            if (CTAG::CTRL::DspProxy::GetCPUOverload()) {
                Display::DrawRect(124, 0, 3, 3, true, true);
            } else {
                Display::DrawRect(124, 0, 3, 3, true, false);
            }
            Display::Flush();
        }

        void UIMenu::TaskFunction(void *) {
            ESP_LOGI(TAG, "TaskFunction: starting main loop");
            UserInput::Init();
            while (1) {
                InputEvent ev;
                bool gotEv = UserInput::GetEvent(ev, 20);
                uint32_t now = xTaskGetTickCount();

                if (gotEv) {
                    if (displayState == ASLEEP) {
                        Display::Wake();
                        displayState = AWAKE;
                        screensaverTimer = 0;
                        panelBarTimer = 50;
                        redrawNeeded = true;
                    } else {
                        screensaverTimer = 0;
                        if (navState == ROOT) {
                            // Panel switching: E2 (ENC2_DELTA) per the C3 input
                            // split (SDD §5.1). E1 (ENC_DELTA) is also accepted
                            // here so host scripts written for the A1S single-
                            // encoder bench keep working.
                            if (ev.type == InputEvent::ENC_DELTA || ev.type == InputEvent::ENC2_DELTA) {
                                int steps = ev.delta;
                                if (steps > 1) steps /= 2;
                                if (steps < -1) steps /= 2;
                                if (steps == 0) steps = (ev.delta > 0) ? 1 : -1;
                                int prev = currentPanel;
                                currentPanel = (Panel)((currentPanel + steps + PANEL_COUNT) % PANEL_COUNT);
                                if (currentPanel != prev) {
                                    pages[prev]->deinit();
                                    pages[currentPanel]->init();
                                }
                                panelBarTimer = 50;
                                redrawNeeded = true;
                            } else if (ev.type == InputEvent::BTN2_SHORT) {
                                navState = PANEL_IN;
                                redrawNeeded = true;
                            }
                        } else {
                            // Two-encoder split (physical row under the OLED:
                            // BACK, OK, then encoders left→right). Hardware
                            // naming: E1 "(value)" is the RIGHT encoder,
                            // E2 "(panel)" the LEFT one:
                            // ENC2_DELTA = LEFT encoder  (E2) = list scroll
                            // ENC_DELTA  = RIGHT encoder (E1) = live value edit
                            if (ev.type == InputEvent::ENC2_DELTA) {
                                pages[currentPanel]->onEncoder(ev.delta);
                            } else if (ev.type == InputEvent::ENC_DELTA) {
                                pages[currentPanel]->onValueEncoder(ev.delta);
                            } else if (ev.type == InputEvent::BTN1_SHORT || ev.type == InputEvent::BTN1_LONG) {
                                if (!pages[currentPanel]->onBack()) {
                                    navState = ROOT;
                                    redrawNeeded = true;
                                    panelBarTimer = 50;
                                }
                            } else if (ev.type == InputEvent::BTN2_SHORT) {
                                pages[currentPanel]->onButton(2, false);
                            } else if (ev.type == InputEvent::BTN2_LONG) {
                                pages[currentPanel]->onButton(2, true);
                            }
                        }
                    }
                } else {
                    screensaverTimer++;
                    // Bring-up: the screensaver is disabled by default — it
                    // kept masquerading as a dead panel. Re-enable via the
                    // bench once the hardware is proven.
                    if (screensaverEnabled && displayState == AWAKE &&
                        screensaverTimer >= SCREENSAVER_TIMEOUT) {
                        Display::Sleep();
                        displayState = ASLEEP;
                    }
                }

                // Redraw if needed
                if (displayState != ASLEEP && redrawNeeded) {
                    if (navState == ROOT) {
                        pages[currentPanel]->doRedraw();
                        drawPanelBar();
                    } else {
                        pages[currentPanel]->doRedraw();
                    }
                    redrawNeeded = false;
                }

                // Decrement panel bar timer each tick when visible
                if (navState == ROOT && panelBarTimer > 0) {
                    panelBarTimer--;
                }

                // Periodic page refresh while a panel is open. Pages opt in via
                // wantsTick() (MIX VU meters today, tape timecode later).
                if (navState == PANEL_IN && displayState == AWAKE &&
                    pages[currentPanel]->wantsTick()) {
                    if (++pageTickCounter >= TICK_DIVIDER) {
                        pageTickCounter = 0;
                        pages[currentPanel]->onTick();
                    }
                } else {
                    pageTickCounter = 0;
                }

                // Framebuffer snapshot request (debug screenshot command).
                // Runs unconditionally every iteration, not just when an input
                // event arrived, so a request is serviced within one loop.
                if (fbShotPending) {
                    Display::CopyFramebuffer(fbShotBuf);
                    fbShotPending = false;
                    fbShotReady = true;
                }

                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }
}
