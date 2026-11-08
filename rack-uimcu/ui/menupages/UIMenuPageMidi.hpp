/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

***************/

// P4: MIDI panel page — BLE scan/connect (receiver runs on the panel) and
// the source enables (TRS DIN via link, BLE relay drain). Flags live on
// the DSP and are mirrored in the DspProxy cache (see /link/midiFlags).

#pragma once

#include "../UIMenuPage.hpp"

namespace CTAG {
    namespace CTRL {
        class UIMenuPageMidi final : public UIMenuPage {
        public:
            void init() override;
            void deinit() override;
            void doRedraw() override;
            void onEncoder(int delta) override;
            void onButton(int btnId, bool longPress) override;
            bool onBack() override;
            void onTick() override;
            bool wantsTick() const override;
            void DebugStateJson(std::string &out) override;

        private:
            enum SubPage : uint8_t { SP_MAIN, SP_SCAN };
            SubPage subPage = SP_MAIN;
            int cursor = 0;
            int scrollOffset = 0;
            void redrawMain();
            void redrawScan();
        };
    }
}
