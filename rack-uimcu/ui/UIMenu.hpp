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

#pragma once

#include "UIMenuPage.hpp"
#include "Display.hpp"
#include "input/UserInput.hpp"
#include <array>

namespace CTAG {
    namespace CTRL {
        // C3 panel controller (SDD §5.1). Five panels; the MIDI page returns
        // with the P4 BtMidiReceiver port, then becomes PANEL 5 again.
        class UIMenu final {
        public:
            static void Init();
            static void TaskFunction(void *);

            // Debug support: grab the current OLED framebuffer. The copy is
            // taken inside the UIMenu task so it can never be torn by a
            // half-finished redraw. RequestFramebufferShot() is async;
            // CopyFramebufferShot() blocks until the task has serviced it.
            static void RequestFramebufferShot();
            static bool CopyFramebufferShot(uint8_t *dst, uint32_t timeoutMs = 500);

            // Debug introspection so host-side test scripts can assert on real
            // state instead of inferring it from pixels.
            // navState: 0=ROOT 1=PANEL_IN. panel: Panel enum value.
            static void GetDebugState(int &navState, int &panel);

            // Bring-up control: the screensaver blanks the panel 15 s after
            // the last input, which looks exactly like a dead display.
            static void SetScreensaverEnabled(bool en) { screensaverEnabled = en; }

            // Debug: reach a page instance so host scripts can assert on its
            // internal state. Caller owns the returned pointer.
            static UIMenuPage *GetPage(int panel) {
                if (panel < 0 || panel >= PANEL_COUNT) return nullptr;
                return pages[panel];
            }

            enum Panel : uint8_t {
                PANEL_MIX = 0, PANEL_TAPE = 1, PANEL_HOME = 2, PANEL_MOD = 3,
                PANEL_PARAMS = 4, PANEL_MIDI = 5
            };

        private:
            enum NavState : uint8_t { ROOT, PANEL_IN };

            static constexpr int PANEL_COUNT = 6;
            static UIMenuPage *pages[6];

            static Panel currentPanel;
            static NavState navState;
            static bool redrawNeeded;
            static int panelBarTimer;
            static int screensaverTimer;
            // Periodic page refresh divider: task runs at ~50 Hz, a full redraw
            // every 5th iteration gives ~4 Hz which is fast enough for VU meters.
            static int pageTickCounter;
            static constexpr int TICK_DIVIDER = 5;
            // framebuffer snapshot handshake (see RequestFramebufferShot)
            static volatile bool fbShotPending;
            static volatile bool fbShotReady;
            static uint8_t fbShotBuf[CTAG::DRIVERS::Display::FRAMEBUFFER_SIZE];
            enum DisplayState { AWAKE, ASLEEP };
            static DisplayState displayState;
            static bool screensaverEnabled; // bring-up: off (bench can enable)
            static constexpr int SCREENSAVER_TIMEOUT = 1500;

            static void drawPanelBar();
        };
    }
}
