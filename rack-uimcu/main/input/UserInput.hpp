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

// C3 panel input (SDD §2.1/§5.1). Same queue + semantic-event API as the
// A1S UserInput so UIMenu and host scripts work unchanged, with two changes:
//   - a second encoder (ENC2_DELTA) — the panel switch in ROOT state;
//   - buttons on one ADC ladder pin instead of the A1S GPIO36 design.
// Hardware: E1 A=GPIO8 B=GPIO9 (strapping pins — external pull-ups, never
// hold low at reset), E2 A=GPIO4 B=GPIO5, BTN_A+BTN_B resistor ladder on
// GPIO0 (ADC1_CH0).

#pragma once
#include <cstdint>

int UserInputDebugBtnAdc();

namespace CTAG {
    namespace CTRL {
        struct InputEvent {
            enum Type : uint8_t {
                ENC_DELTA = 0,
                BTN1_SHORT = 1,
                BTN1_LONG = 2,
                BTN2_SHORT = 3,
                BTN2_LONG = 4,
                BTN1_DOUBLE = 5,
                ENC2_DELTA = 6,
            };
            Type type;
            int16_t delta; // only meaningful for ENC*_DELTA
        };

        class UserInput {
        public:
            static void Init();
            static void EnableISR();
            static bool GetEvent(InputEvent& ev, uint32_t timeoutMs = 10);
            static bool PeekEvent(InputEvent& ev);

            // Debug support: push a synthetic event straight into the same
            // queue the hardware path writes to. This deliberately bypasses
            // debounce/long-press detection, so a host script must pace its
            // own actions the way a human would. Events are the semantic UI
            // actions (BTN2_SHORT = OK), not raw button transitions.
            static bool InjectEvent(InputEvent::Type type, int16_t delta = 0);
        };
    }
}
