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
#include <cstdint>

namespace CTAG {
    namespace CTRL {
        class UIMenuPageParams final : public UIMenuPage {
        public:
            void init() override;
            void deinit() override;
            void doRedraw() override;
            void onEncoder(int delta) override;      // left encoder: list scroll
            void onValueEncoder(int delta) override; // right encoder: live value edit
            void onButton(int btnId, bool longPress) override;
            bool onBack() override;
            void onTick() override;
            bool wantsTick() const override;

        private:
            struct ParamInfo {
                char id[24];
                char name[24];
                char type[24];
                int min;
                int max;
                int current;
                int cv;
                int trig;
                bool canCv;    // param accepts a CV source (numeric params)
                bool canTrig;  // param accepts a gate source (bool params)
            };

            struct GroupInfo {
                char name[24];
                int firstParamIdx;
                int paramCount;
            };

            struct PresetInfo {
                char name[24];
                int number;
            };

            enum Mode { MODE_SELECT, MODE_GROUP, MODE_EDIT, MODE_MAP, MODE_PSET, MODE_PRESETS };
            int cursor;
            int scrollOffset;
            Mode mode;
            int paramCount;
            static const int MAX_PARAMS = 256;
            ParamInfo *params;

            int groupCount;
            int currentGroup;
            static const int MAX_GROUPS = 32;
            GroupInfo *groups;

            static const int MAX_PRESETS = 64;
            PresetInfo presets[MAX_PRESETS];
            int presetCount;
            int presetChan; // channel we're editing presets for

            int mapParamIdx;
            // MODE_MAP: one unified, capability-filtered source list per param
            // (CV sources for numeric params, gate sources for bool params).
            int mapPos;      // cursor into the ordered source list
            bool mapIsCv;    // true: param maps a CV source, false: gate source
            bool mapLearning;    // waiting for a MIDI CC to capture
            bool mapLearnFull;   // all 8 dynamic CC slots taken
            uint32_t mapHintUntil = 0;  // ticks: show "hold param=map" hint

            int chan;
            bool hasDualCh;
            int encoderAccel = 0;
            int lastEncDir = 0;
            uint32_t lastEncTick = 0;

            void parseParams();
            void parsePresets(int chan);
            void enterParamList(bool withMapHint);
            void redrawSelect();
            void redrawGroup();
            void redrawEdit();
            void redrawMap();
            void redrawPresets();
            int paramIndexToScreen(int idx) const;
            int groupEditEnd() const;
            int sourceCount() const;          // size of the active source list
            int sourceSlotAt(int pos) const;  // slot index (or -2 = learn row) at list position
            int sourcePosOf(int slot) const;  // list position of a slot (-1 if absent)
        };
    }
}
