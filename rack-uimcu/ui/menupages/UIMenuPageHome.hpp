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
        class UIMenuPageSystem;

        class UIMenuPageHome final : public UIMenuPage {
        public:
            void init() override;
            void deinit() override;
            void doRedraw() override;
            void onEncoder(int delta) override;
            void onButton(int btnId, bool longPress) override;
            bool onBack() override;

        private:
            // plugin browser items
            struct PluginEntry {
                char id[32];
                char name[32];
                bool isStereo;
            };

            // favorites items
            struct FavEntry {
                char name[24];
            };

            enum SubPage { SP_MAIN, SP_SELECT, SP_SELECT_CH, SP_SYSTEM, SP_FAVORITES, SP_SD_CARD };

            SubPage subPage;
            int cursor;
            int scrollOffset;
            int pluginCount;
            static const int MAX_PLUGINS = 64;
            PluginEntry plugins[MAX_PLUGINS];

            static const int MAX_FAVORITES = 10;
            FavEntry favorites[MAX_FAVORITES];
            int favActiveId;

            static const int VISIBLE_LINES = 7;
            static const int MAX_SD_FILES = 32;
            char sdEntries[MAX_SD_FILES][32];
            bool sdIsDir[MAX_SD_FILES];
            int sdFileCount;
            char sdCurrentPath[64];

            int selChan;
            int selectedPlugin; // index of plugin being channel-assigned

            UIMenuPageSystem *systemPage;

            void parsePlugins();
            void parseFavorites();
            void redrawMain();
            void redrawSelect();
            void redrawSelectCh();
            void redrawFavorites();
            void scanSdFiles();
            void goUpSdDir();
            void redrawSdCard();
        };
    }
}
