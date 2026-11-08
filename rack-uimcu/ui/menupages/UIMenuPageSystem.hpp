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

namespace CTAG {
    namespace CTRL {
        class UIMenuPageSystem final : public UIMenuPage {
        public:
            void init() override;
            void deinit() override;
            void doRedraw() override;
            void onEncoder(int delta) override;
            void onButton(int btnId, bool longPress) override;
            bool onBack() override;

        private:
            int cursor;
            int scrollOffset;
            bool editMode;
            bool dirty; // staged values differ from the persisted config

            struct ConfigItem {
                char id[24];
                char name[24];
                char type[12];
                char value[16];
                int valInt;
                int min;
                int max;
                bool deferred;
                // enum-style: comma-separated options
                char options[64];
            };
            static const int MAX_ITEMS = 21;
            ConfigItem items[MAX_ITEMS];
            int itemCount;

            void parseConfig();
            void redrawMain();
            void previewCodecLevels();
            void applyCurrent();
        };
    }
}
