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

#include <string>

namespace CTAG {
    namespace CTRL {
        class UIMenuPage {
        public:
            virtual ~UIMenuPage() = default;
            virtual void init() = 0;
            virtual void deinit() = 0;
            virtual void doRedraw() = 0;
            virtual void onEncoder(int delta) {}
            // Second (right-hand) encoder: live value edit for the item under
            // the cursor. Default: the page has no value dimension.
            virtual void onValueEncoder(int delta) {}
            virtual void onButton(int btnId, bool longPress) {}
            virtual bool onBack() { return false; } // true=handled, false=return to ROOT
            // Called from UIMenu::TaskFunction at ~4 Hz while PANEL_IN and the
            // page has requested periodic refresh (VU meters, tape timecode).
            // Redraws itself; UIMenu's redraw flag is not involved.
            virtual void onTick() {}
            // Pages returning true get onTick() pumped every task iteration
            // (20 ms). Only useful for live metering — it costs a full redraw.
            virtual bool wantsTick() const { return false; }
            // Debug introspection for host-side test scripts (no RTTI in this
            // build, so pages publish their state as JSON themselves).
            virtual void DebugStateJson(std::string &out) {}
        };
    }
}
