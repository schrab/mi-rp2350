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

#include "UIMenuPageTape.hpp"
#include "Display.hpp"
#include "DspProxy.hpp"
#include <cstring>
#include <cstdio>

using namespace CTAG::DRIVERS;

namespace CTAG {
    namespace CTRL {
        void UIMenuPageTape::init() {
            subPage = SP_MAIN;
            cursor = 0;
            scrollOffset = 0;
            fileCount = 0;
        }

        void UIMenuPageTape::deinit() {}

        void UIMenuPageTape::scanFiles() {
            fileCount = 0;
            if (!DspProxy::IsSDMounted()) return;
            // Listing comes over the link (SD proxy, SDD §4.2); filter wavs.
            char names[MAX_FILES][32];
            bool isDir[MAX_FILES];
            int n = DspProxy::ListSdDir("/sd", names, isDir, MAX_FILES);
            for (int i = 0; i < n; i++) {
                const char *name = names[i];
                int len = strlen(name);
                if (len > 4 && (strcasecmp(name + len - 4, ".wav") == 0)) {
                    size_t copyLen = len < 31 ? len : 31;
                    memcpy(fileNames[fileCount], name, copyLen);
                    fileNames[fileCount][copyLen] = '\0';
                    fileCount++;
                }
            }
        }

        void UIMenuPageTape::onEncoder(int delta) {
            if (subPage == SP_MAIN) {
                cursor += delta;
                if (cursor < 0) cursor = 0;
                if (cursor > 2) cursor = 2;
            } else if (subPage == SP_FILELIST) {
                int nc = cursor + delta;
                if (nc < 0) nc = 0;
                if (nc >= fileCount) nc = fileCount - 1;
                cursor = nc;
                ClampScroll(cursor, scrollOffset, fileCount, VISIBLE_ITEMS);
            } else if (subPage == SP_RECORD) {
                cursor += delta;
                if (cursor < 0) cursor = 0;
                if (cursor > 1) cursor = 1;
            }
            doRedraw();
        }

        void UIMenuPageTape::onButton(int btnId, bool longPress) {
            if (subPage == SP_MAIN) {
                if (btnId == 2 && !longPress) {
                    if (cursor == 0) {
                        // Playback: browse files
                        scanFiles();
                        subPage = SP_FILELIST;
                        cursor = 0;
                        scrollOffset = 0;
                    } else if (cursor == 1) {
                        // Record
                        subPage = SP_RECORD;
                        cursor = 0;
                    } else if (cursor == 2) {
                        // Stop
                        DspProxy::SdStop();
                    }
                }
            } else if (subPage == SP_FILELIST) {
                if (btnId == 2 && !longPress && fileCount > 0 && cursor < fileCount) {
                    char path[64];
                    snprintf(path, sizeof(path), "/sd/%s", fileNames[cursor]);
                    DspProxy::SdStartPlayback(path);
                    subPage = SP_MAIN;
                    cursor = 0;
                }
            } else if (subPage == SP_RECORD) {
                if (btnId == 2 && !longPress) {
                    if (cursor == 0) {
                        DspProxy::SdStartRecording("/sd/record.wav");
                    } else {
                        DspProxy::SdStop();
                    }
                    subPage = SP_MAIN;
                    cursor = 0;
                }
            }
            doRedraw();
        }

        bool UIMenuPageTape::onBack() {
            if (subPage == SP_RECORD) {
                subPage = SP_MAIN;
                cursor = 0;
                doRedraw();
                return true;
            }
            if (subPage == SP_FILELIST) {
                subPage = SP_MAIN;
                cursor = 0;
                doRedraw();
                return true;
            }
            return false;
        }

        void UIMenuPageTape::doRedraw() {
            if (subPage == SP_MAIN) redrawMain();
            else if (subPage == SP_FILELIST) redrawFileList();
            else if (subPage == SP_RECORD) redrawRecord();
        }

        void UIMenuPageTape::redrawMain() {
            Display::Clear();
            int y = ITEM_Y0;
            if (DspProxy::SdIsPlaying()) {
                Display::DrawString(0, y, "PLAYING", Display::FONT_5X7);
                if (DspProxy::SdGetTotalFrames() > 0) {
                    char buf[24];
                    uint32_t sec = DspProxy::SdGetPositionFrames() / 44100;
                    uint32_t total = DspProxy::SdGetTotalFrames() / 44100;
                    snprintf(buf, sizeof(buf), "%02u:%02u / %02u:%02u",
                             (unsigned)(sec / 60), (unsigned)(sec % 60), (unsigned)(total / 60), (unsigned)(total % 60));
                    y += LINE_H;
                    Display::DrawString(0, y, buf, Display::FONT_5X7);
                }
            } else if (DspProxy::SdIsRecording()) {
                Display::DrawString(0, y, "RECORDING", Display::FONT_5X7);
                char buf[24];
                uint32_t sec = DspProxy::SdGetPositionFrames() / 44100;
                snprintf(buf, sizeof(buf), "%02u:%02u", (unsigned)(sec / 60), (unsigned)(sec % 60));
                y += LINE_H;
                Display::DrawString(0, y, buf, Display::FONT_5X7);
            } else {
                Display::DrawString(0, y, "PLAY FILE", Display::FONT_5X7);
                y += LINE_H;
                Display::DrawString(0, y, "RECORD", Display::FONT_5X7);
                y += LINE_H;
                Display::DrawString(0, y, "STOP", Display::FONT_5X7);
                y += LINE_H;
            }
            int highlightY = ROW(cursor);
            Display::InvertRect(0, highlightY, 128, 8);
            Display::Flush();
        }

        void UIMenuPageTape::redrawFileList() {
            Display::Clear();
            if (!DspProxy::IsSDMounted()) {
                Display::DrawString(0, 24, "No SD card", Display::FONT_5X7);
                Display::Flush();
                return;
            }
            if (fileCount == 0) {
                Display::DrawString(0, 24, "No WAV files", Display::FONT_5X7);
                Display::Flush();
                return;
            }
            int visible = ClampVisible(fileCount, scrollOffset, VISIBLE_ITEMS);
            for (int i = 0; i < visible; i++) {
                int idx = scrollOffset + i;
                Display::DrawString(0, ROW(i), fileNames[idx], Display::FONT_5X7);
            }
            int cy = ROW(cursor - scrollOffset);
            Display::InvertRect(0, cy, 128, 8);
            if (fileCount > VISIBLE_ITEMS) Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0, VISIBLE_ITEMS * LINE_H, fileCount, cursor);
            Display::Flush();
        }

        void UIMenuPageTape::redrawRecord() {
            Display::Clear();
            Display::DrawString(0, ROW(0), "START REC", Display::FONT_5X7);
            Display::DrawString(0, ROW(1), "CANCEL", Display::FONT_5X7);
            Display::InvertRect(0, ROW(cursor), 128, 8);
            Display::Flush();
        }
    }
}
