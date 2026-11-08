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

#include "UIMenuPageParams.hpp"
#include "Display.hpp"
#include "DspProxy.hpp"
#include "CVSlotNames.hpp"
#include "rapidjson/document.h"
#include <cstdlib>
#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace CTAG::DRIVERS;
using namespace rapidjson;

namespace CTAG {
    namespace CTRL {

        // ---- ordered modulation-source lists for the mapping page ----
        // Most-useful first: [Learn] (CV only, sentinel -2) -> None -> Marbles
        // outputs -> learned CC slots -> fixed MIDI-mapped slots. A param sees
        // exactly one of the two lists, chosen by its own capability.
        namespace {
            constexpr int kSourceLearn = -2;  // [Learn CC] action row

            const int kCvSourceOrder[] = {
                kSourceLearn, -1,
                98, 99, 100, 101,                       // MB.X1..X3, MB.Y
                90, 91, 92, 93, 94, 95, 96, 97,         // learned CC1..CC8
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
                40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
                60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
                80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
            };
            const int kTrigSourceOrder[] = {
                -1,
                40, 41, 42,                              // MB.T1..T3
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
            };

            void FormatSourceName(int slot, bool isCv, char *buf, int buflen) {
                if (slot == kSourceLearn) {
                    snprintf(buf, buflen, "[Learn CC]");
                } else if (slot < 0) {
                    snprintf(buf, buflen, "None");
                } else if (isCv && slot < (int)(sizeof(cvSlotDisplayNames) / sizeof(cvSlotDisplayNames[0]))) {
                    snprintf(buf, buflen, "%s", cvSlotDisplayNames[slot]);
                } else {
                    snprintf(buf, buflen, "%s", TrigSlotName(slot));
                }
            }
        }

        void UIMenuPageParams::init() {
            if (!params) {
                params = (ParamInfo *)malloc(MAX_PARAMS * sizeof(ParamInfo));
                groups = (GroupInfo *)malloc(MAX_GROUPS * sizeof(GroupInfo));
            }
            cursor = 0;
            scrollOffset = 0;
            mode = MODE_SELECT;
            paramCount = 0;
            groupCount = 0;
            currentGroup = -1;
            mapParamIdx = 0;
            mapPos = 0;
            mapIsCv = true;
            mapLearning = false;
            mapLearnFull = false;
            mapHintUntil = 0;
            presetCount = 0;
            presetChan = 0;
            chan = 0;
            string id0 = DspProxy::GetStringID(0);
            string id1 = DspProxy::GetStringID(1);
            bool stereoCh0 = DspProxy::IsPluginStereo(id0);
            hasDualCh = !stereoCh0 && !id0.empty() && !id1.empty() && id0 != id1;
            parseParams();
        }

        void UIMenuPageParams::deinit() {
            if (params) { free(params); params = nullptr; }
            if (groups) { free(groups); groups = nullptr; }
        }

        void UIMenuPageParams::parseParams() {
            paramCount = 0;
            groupCount = 0;
            currentGroup = -1;
            const char *json = DspProxy::GetCStrJSONActivePluginParams(chan);
            if (!json) return;

            Document doc;
            doc.Parse(json);
            if (!doc.HasMember("params") || !doc["params"].IsArray()) return;

            const Value &arr = doc["params"];

            // First pass: standalone leaf params (not in groups)
            int standaloneStart = paramCount;
            for (SizeType i = 0; i < arr.Size() && paramCount < MAX_PARAMS; i++) {
                const Value &p = arr[i];
                if (!p.HasMember("id") || !p.HasMember("name") || !p.HasMember("type")) continue;
                if (strcmp(p["type"].GetString(), "group") == 0) continue;
                ParamInfo &pi = params[paramCount];
                snprintf(pi.id, sizeof(pi.id), "%s", p["id"].GetString());
                snprintf(pi.name, sizeof(pi.name), "%s", p["name"].GetString());
                snprintf(pi.type, sizeof(pi.type), "%s", p["type"].GetString());
                pi.min = p.HasMember("min") ? p["min"].GetInt() : 0;
                pi.max = p.HasMember("max") ? p["max"].GetInt() : 1;
                pi.current = p.HasMember("current") ? p["current"].GetInt() : 0;
                pi.cv = p.HasMember("cv") ? p["cv"].GetInt() : -1;
                pi.trig = p.HasMember("trig") ? p["trig"].GetInt() : -1;
                pi.canCv = p.HasMember("cv");
                pi.canTrig = p.HasMember("trig");
                paramCount++;
            }
            int standaloneCount = paramCount - standaloneStart;
            if (standaloneCount > 0 && groupCount < MAX_GROUPS) {
                GroupInfo &g = groups[groupCount++];
                snprintf(g.name, sizeof(g.name), "General");
                g.firstParamIdx = standaloneStart;
                g.paramCount = standaloneCount;
            }

            // Second pass: recurse into groups
            for (SizeType i = 0; i < arr.Size(); i++) {
                const Value &p = arr[i];
                if (!p.HasMember("type") || strcmp(p["type"].GetString(), "group") != 0) continue;
                if (!p.HasMember("params") || !p["params"].IsArray()) continue;
                if (groupCount >= MAX_GROUPS) break;

                GroupInfo &g = groups[groupCount];
                snprintf(g.name, sizeof(g.name), "%s", p["name"].GetString());
                g.firstParamIdx = paramCount;

                const Value &garr = p["params"];
                int count = 0;
                for (SizeType j = 0; j < garr.Size() && paramCount < MAX_PARAMS; j++) {
                    const Value &leaf = garr[j];
                    if (!leaf.HasMember("id") || !leaf.HasMember("name") || !leaf.HasMember("type")) continue;
                    ParamInfo &pi = params[paramCount];
                    snprintf(pi.id, sizeof(pi.id), "%s", leaf["id"].GetString());
                    snprintf(pi.name, sizeof(pi.name), "%s", leaf["name"].GetString());
                    snprintf(pi.type, sizeof(pi.type), "%s", leaf["type"].GetString());
                    pi.min = leaf.HasMember("min") ? leaf["min"].GetInt() : 0;
                    pi.max = leaf.HasMember("max") ? leaf["max"].GetInt() : 1;
                    pi.current = leaf.HasMember("current") ? leaf["current"].GetInt() : 0;
                    pi.cv = leaf.HasMember("cv") ? leaf["cv"].GetInt() : -1;
                    pi.trig = leaf.HasMember("trig") ? leaf["trig"].GetInt() : -1;
                    pi.canCv = leaf.HasMember("cv");
                    pi.canTrig = leaf.HasMember("trig");
                    paramCount++;
                    count++;
                }
                g.paramCount = count;
                groupCount++;
            }
        }

        void UIMenuPageParams::parsePresets(int chan) {
            presetCount = 0;
            presetChan = chan;
            const char *json = DspProxy::GetCStrJSONGetPresets(chan);
            if (!json) return;

            Document doc;
            doc.Parse(json);
            if (!doc.HasMember("presets") || !doc["presets"].IsArray()) return;

            const Value &arr = doc["presets"];
            for (SizeType i = 0; i < arr.Size() && presetCount < MAX_PRESETS; i++) {
                const Value &p = arr[i];
                if (!p.IsObject() || !p.HasMember("name") || !p["name"].IsString()) continue;
                PresetInfo &pi = presets[presetCount];
                snprintf(pi.name, sizeof(pi.name), "%s", p["name"].GetString());
                pi.number = p.HasMember("number") ? p["number"].GetInt() : i;
                presetCount++;
            }
        }

        int UIMenuPageParams::paramIndexToScreen(int idx) const {
            return idx - scrollOffset;
        }

        int UIMenuPageParams::groupEditEnd() const {
            return (currentGroup >= 0)
                ? groups[currentGroup].firstParamIdx + groups[currentGroup].paramCount
                : paramCount;
        }

        void UIMenuPageParams::onEncoder(int delta) {
            if (mode == MODE_SELECT) {
                int newCursor = cursor + delta;
                if (newCursor < 0) newCursor = 0;
                if (newCursor > 3) newCursor = 3;
                if (newCursor != cursor) { encoderAccel = 0; lastEncDir = 0; }
                cursor = newCursor;
            } else if (mode == MODE_GROUP) {
                if (groupCount == 0) return;
                int newCursor = cursor + delta;
                if (newCursor < 0) newCursor = 0;
                if (newCursor >= groupCount) newCursor = groupCount - 1;
                if (newCursor != cursor) { encoderAccel = 0; lastEncDir = 0; }
                cursor = newCursor;
                ClampScroll(cursor, scrollOffset, groupCount, VISIBLE_ITEMS);
            } else if (mode == MODE_EDIT) {
                if (paramCount == 0) return;
                int endIdx = groupEditEnd();
                int newCursor = cursor + delta;
                int startIdx = (currentGroup >= 0) ? groups[currentGroup].firstParamIdx : 0;
                if (newCursor < startIdx) newCursor = startIdx;
                if (newCursor >= endIdx) newCursor = endIdx - 1;
                if (newCursor != cursor) { encoderAccel = 0; lastEncDir = 0; }
                cursor = newCursor;

                // auto-scroll
                int scr = paramIndexToScreen(cursor);
                if (scr < 0) scrollOffset += scr;
                if (scr >= VISIBLE_ITEMS) scrollOffset += (scr - (VISIBLE_ITEMS - 1));
                if (scrollOffset > endIdx - VISIBLE_ITEMS) scrollOffset = endIdx - VISIBLE_ITEMS;
                if (scrollOffset < startIdx) scrollOffset = startIdx;
            } else if (mode == MODE_MAP) {
                if (paramCount == 0) return;
                mapPos += delta;
                if (mapPos < 0) mapPos = 0;
                if (mapPos > sourceCount() - 1) mapPos = sourceCount() - 1;
                mapLearnFull = false;
            } else if (mode == MODE_PRESETS) {
                if (presetCount == 0) return;
                int newCursor = cursor + delta;
                if (newCursor < 0) newCursor = 0;
                if (newCursor >= presetCount) newCursor = presetCount - 1;
                if (newCursor != cursor) { encoderAccel = 0; lastEncDir = 0; }
                cursor = newCursor;
                // auto-scroll
                ClampScroll(cursor, scrollOffset, presetCount, VISIBLE_ITEMS);
            }
            doRedraw();
        }

        // Right encoder: live value edit for the param under the cursor
        // (no confirm step — the change goes straight to the DSP). The
        // momentary "play" param has no value dimension; pulse it via OK.
        void UIMenuPageParams::onValueEncoder(int delta) {
            if (mode != MODE_EDIT || paramCount == 0 || cursor >= paramCount) return;
            ParamInfo &pi = params[cursor];
            if (strcmp(pi.id, "play") == 0) return;

            // Encoder acceleration: build momentum on consecutive same-direction turns
            uint32_t now = xTaskGetTickCount();
            if (now - lastEncTick > pdMS_TO_TICKS(100)) encoderAccel = 0;
            lastEncTick = now;

            int dir = delta > 0 ? 1 : (delta < 0 ? -1 : 0);
            if (dir != lastEncDir) encoderAccel = 0;
            else if (encoderAccel < 10) encoderAccel++;
            lastEncDir = dir;

            int step = delta * (1 + encoderAccel * encoderAccel);
            if (step > 50) step = 50;
            if (step < -50) step = -50;

            int val = pi.current + step;
            if (val < pi.min) val = pi.min;
            if (val > pi.max) val = pi.max;
            if (val == pi.current) return;
            pi.current = val;
            DspProxy::SetChannelParamValue(chan, pi.id, "current", val);
            doRedraw();
        }

        void UIMenuPageParams::onButton(int btnId, bool longPress) {
            if (mode == MODE_SELECT) {
                if (btnId == 2 && !longPress) {
                    // enter sub-mode
                    if (cursor == 0) {
                        chan = (hasDualCh ? 0 : 0);
                        enterParamList(false);
                    }
                    else if (cursor == 1) {
                        if (hasDualCh) {
                            chan = 1;
                            enterParamList(false);
                        } else {
                            enterParamList(true);
                        }
                    }
                    else if (hasDualCh && cursor == 2) {
                        // "MAP" in the dual-channel layout: pick the param on
                        // ch0 first (long-press maps it); previously this fell
                        // through to MODE_PSET.
                        chan = 0;
                        enterParamList(true);
                    }
                    else if (!hasDualCh && cursor == 2) { mode = MODE_PSET; }
                    else if (cursor == 3) {
                        mode = MODE_PRESETS;
                        cursor = 0;
                        scrollOffset = 0;
                        parsePresets(chan);
                    }
                }
            } else if (mode == MODE_GROUP) {
                if (btnId == 2 && !longPress && groupCount > 0) {
                    currentGroup = cursor;
                    cursor = groups[currentGroup].firstParamIdx;
                    scrollOffset = groups[currentGroup].firstParamIdx;
                    mode = MODE_EDIT;
                }
            } else if (mode == MODE_EDIT) {
                if (btnId == 2 && paramCount > 0 && cursor < paramCount) {
                    // Momentary trigger for "play" param: pulse instead of mapping.
                    if (strcmp(params[cursor].id, "play") == 0) {
                        DspProxy::SetChannelParamValue(chan, "play", "current", 1);
                        doRedraw();
                        return;
                    }
                    // OK = modulation mapping for mappable params (short or
                    // long — the value dimension lives on the right encoder).
                    if (params[cursor].canCv || params[cursor].canTrig) {
                        mapParamIdx = cursor;
                        mapIsCv = params[cursor].canCv || !params[cursor].canTrig;
                        mapPos = sourcePosOf(mapIsCv ? params[cursor].cv : params[cursor].trig);
                        if (mapPos < 0) mapPos = 0;
                        mapLearning = false;
                        mapLearnFull = false;
                        mode = MODE_MAP;
                    }
                }
            } else if (mode == MODE_MAP) {
                if (btnId == 2 && !longPress) {
                    int slot = sourceSlotAt(mapPos);
                    if (slot == kSourceLearn) {
                        // Bind the next incoming MIDI CC directly to this
                        // param: allocate a free dynamic slot on capture.
                        bool free = false;
                        for (int i = 0; i < 8; i++) {
                            if (DspProxy::GetDynamicSlotCC(i) == -1) { free = true; break; }
                        }
                        if (free) {
                            DspProxy::StartLearn();
                            mapLearning = true;
                            mapLearnFull = false;
                        } else {
                            mapLearnFull = true;
                            mapLearning = false;
                        }
                    } else if (slot >= 0) {
                        ParamInfo &pi = params[mapParamIdx];
                        if (mapIsCv) {
                            pi.cv = slot;
                            DspProxy::SetChannelParamValue(chan, pi.id, "cv", slot);
                        } else {
                            pi.trig = slot;
                            DspProxy::SetChannelParamValue(chan, pi.id, "trig", slot);
                        }
                        mode = MODE_EDIT;
                    }
                }
            } else if (mode == MODE_PRESETS) {
                if (btnId == 2 && !longPress) {
                    // load selected preset
                    if (presetCount > 0 && cursor >= 0 && cursor < presetCount) {
                        DspProxy::ChannelLoadPreset(presetChan, presets[cursor].number);
                        mode = MODE_SELECT;
                        cursor = 0;
                    }
                } else if (btnId == 2 && longPress) {
                    // save current as new preset
                    if (presetCount > 0) {
                        int nextNum = presets[presetCount - 1].number + 1;
                        DspProxy::ChannelSavePreset(presetChan, "User", nextNum);
                        // re-parse to show updated list
                        parsePresets(presetChan);
                    }
                }
            }
            doRedraw();
        }

        bool UIMenuPageParams::onBack() {
            if (mode == MODE_EDIT) {
                if (currentGroup >= 0) {
                    mode = MODE_GROUP;
                    cursor = currentGroup;
                    scrollOffset = 0;
                    currentGroup = -1;
                } else {
                    mode = MODE_SELECT;
                    cursor = 0;
                }
                doRedraw();
                return true;
            }
            if (mode == MODE_GROUP) {
                mode = MODE_SELECT;
                cursor = 0;
                doRedraw();
                return true;
            }
            if (mode == MODE_MAP) {
                if (mapLearning) {
                    DspProxy::StopLearn();
                    mapLearning = false;
                }
                mode = MODE_EDIT;
                doRedraw();
                return true;
            }
            if (mode == MODE_PSET) {
                mode = MODE_SELECT;
                cursor = 0;
                doRedraw();
                return true;
            }
            if (mode == MODE_PRESETS) {
                mode = MODE_SELECT;
                cursor = 0;
                doRedraw();
                return true;
            }
            return false;
        }

        void UIMenuPageParams::doRedraw() {
            if (mode == MODE_SELECT) {
                redrawSelect();
            } else if (mode == MODE_GROUP) {
                redrawGroup();
            } else if (mode == MODE_EDIT) {
                redrawEdit();
            } else if (mode == MODE_MAP) {
                redrawMap();
            } else if (mode == MODE_PRESETS) {
                redrawPresets();
            } else {
                // PSET — simple placeholder
                Display::Clear();
                Display::DrawString(0, 16, "Not implemented", Display::FONT_5X7);
                Display::Flush();
            }
        }

        void UIMenuPageParams::redrawSelect() {
            Display::Clear();
            if (hasDualCh) {
                string id0 = DspProxy::GetStringID(0);
                string id1 = DspProxy::GetStringID(1);
                char buf[26];
                snprintf(buf, sizeof(buf), "Ch0:%.*s", 21, id0.c_str());
                Display::DrawString(0, ROW(0), buf, Display::FONT_5X7);
                snprintf(buf, sizeof(buf), "Ch1:%.*s", 21, id1.c_str());
                Display::DrawString(0, ROW(1), buf, Display::FONT_5X7);
                Display::DrawString(0, ROW(2), "MAP", Display::FONT_5X7);
                Display::DrawString(0, ROW(3), "PRESETS", Display::FONT_5X7);
            } else {
                Display::DrawString(0, ROW(0), "EDIT", Display::FONT_5X7);
                Display::DrawString(0, ROW(1), "MAP", Display::FONT_5X7);
                Display::DrawString(0, ROW(2), "PSET", Display::FONT_5X7);
                Display::DrawString(0, ROW(3), "PRESETS", Display::FONT_5X7);
            }
            Display::InvertRect(0, ROW(cursor), 128, 8);
            Display::Flush();
        }

        void UIMenuPageParams::redrawGroup() {
            Display::Clear();
            if (groupCount == 0) {
                Display::DrawString(0, 24, "No groups", Display::FONT_5X7);
                Display::Flush();
                return;
            }
            int visible = ClampVisible(groupCount, scrollOffset, VISIBLE_ITEMS);
            for (int i = 0; i < visible; i++) {
                int idx = scrollOffset + i;
                const GroupInfo &g = groups[idx];
                int y = ROW(i);
                char buf[32];
                snprintf(buf, sizeof(buf), "%s (%d)", g.name, g.paramCount);
                Display::DrawString(0, y, buf, Display::FONT_5X7);
            }
            int cy = ROW(cursor - scrollOffset);
            Display::InvertRect(0, cy, 128, 8);
            if (groupCount > VISIBLE_ITEMS)
                Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0, VISIBLE_ITEMS * LINE_H, groupCount, cursor);
            Display::Flush();
        }

        void UIMenuPageParams::redrawEdit() {
            Display::Clear();
            if (paramCount == 0) {
                Display::DrawString(0, 24, "No params", Display::FONT_5X7);
                Display::Flush();
                return;
            }
            int endIdx = groupEditEnd();
            int startIdx = (currentGroup >= 0) ? groups[currentGroup].firstParamIdx : 0;
            int count = endIdx - startIdx;
            // While the transient hint is up, keep the bottom row free.
            bool hint = mapHintUntil != 0 &&
                (int32_t)(xTaskGetTickCount() - mapHintUntil) < 0;
            int maxVisible = hint ? VISIBLE_ITEMS - 1 : VISIBLE_ITEMS;
            int visible = ClampVisible(count, scrollOffset - startIdx, maxVisible);

            for (int i = 0; i < visible; i++) {
                int idx = scrollOffset + i;
                if (idx >= endIdx) break;
                const ParamInfo &pi = params[idx];
                int y = ROW(i);
                Display::DrawString(0, y, pi.name, Display::FONT_5X7);
                char valBuf[16];
                if (strcmp(pi.type, "bool") == 0) {
                    strcpy(valBuf, pi.current ? "ON" : "OFF");
                } else if (strcmp(pi.type, "enum") == 0) {
                    snprintf(valBuf, sizeof(valBuf), "%d", pi.current);
                } else {
                    snprintf(valBuf, sizeof(valBuf), "%d", pi.current);
                }
                Display::DrawStringRight(127, y, valBuf, Display::FONT_5X7);
            }
            int cursorY = ROW(paramIndexToScreen(cursor));
            Display::InvertRect(0, cursorY, 128, 8);
            if (hint) {
                Display::DrawString(0, ROW(maxVisible), " OK = map source", Display::FONT_5X7);
            }
            if (count > VISIBLE_ITEMS)
                Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0, VISIBLE_ITEMS * LINE_H, count, cursor);
            Display::Flush();
        }

        // Pumped every task iteration while a MIDI-learn capture or the map
        // hint is pending. Redraws only on state change (capture landed /
        // hint expired), not per tick.
        void UIMenuPageParams::onTick() {
            bool changed = false;
            if (mode == MODE_MAP && mapLearning && !DspProxy::IsLearning()) {
                mapLearning = false;
                changed = true;
                int slot = DspProxy::GetLastLearnedSlot();
                if (slot >= 0 && mapParamIdx < paramCount) {
                    ParamInfo &pi = params[mapParamIdx];
                    pi.cv = slot;
                    DspProxy::SetChannelParamValue(chan, pi.id, "cv", slot);
                    mapPos = sourcePosOf(slot);
                    if (mapPos < 0) mapPos = 0;
                }
            }
            if (mapHintUntil != 0 &&
                (int32_t)(xTaskGetTickCount() - mapHintUntil) >= 0) {
                mapHintUntil = 0;
                changed = true;
            }
            if (changed) doRedraw();
        }

        bool UIMenuPageParams::wantsTick() const {
            return (mode == MODE_MAP && mapLearning) || mapHintUntil != 0;
        }

        // Shared "open the parameter list" path for the EDIT and MAP menu
        // items. withMapHint shows a transient "hold param=map" footer.
        void UIMenuPageParams::enterParamList(bool withMapHint) {
            parseParams();
            if (groupCount == 1) {
                currentGroup = 0;
                cursor = groups[0].firstParamIdx;
                scrollOffset = groups[0].firstParamIdx;
                mode = MODE_EDIT;
            } else if (groupCount > 1) {
                mode = MODE_GROUP;
                cursor = 0;
                scrollOffset = 0;
            } else {
                mode = MODE_EDIT;
                cursor = 0;
                scrollOffset = 0;
                currentGroup = -1;
            }
            if (withMapHint) {
                mapHintUntil = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
            }
        }

        int UIMenuPageParams::sourceCount() const {
            return mapIsCv ? (int)(sizeof(kCvSourceOrder) / sizeof(kCvSourceOrder[0]))
                           : (int)(sizeof(kTrigSourceOrder) / sizeof(kTrigSourceOrder[0]));
        }

        int UIMenuPageParams::sourceSlotAt(int pos) const {
            const int *order = mapIsCv ? kCvSourceOrder : kTrigSourceOrder;
            int n = sourceCount();
            return (pos >= 0 && pos < n) ? order[pos] : -1;
        }

        int UIMenuPageParams::sourcePosOf(int slot) const {
            const int *order = mapIsCv ? kCvSourceOrder : kTrigSourceOrder;
            int n = sourceCount();
            for (int i = 0; i < n; i++) {
                if (order[i] == slot) return i;
            }
            return -1;
        }

        void UIMenuPageParams::redrawMap() {
            Display::Clear();
            if (paramCount == 0) return;
            const ParamInfo &pi = params[mapParamIdx];
            Display::DrawString(0, ROW(0), "MOD SOURCE", Display::FONT_5X7);
            Display::DrawString(0, ROW_HDR(0), pi.name, Display::FONT_5X7);
            char buf[40];

            // selected source (inverted), with the param's saved source noted
            int cur = mapIsCv ? pi.cv : pi.trig;
            char selName[16], curName[16];
            FormatSourceName(sourceSlotAt(mapPos), mapIsCv, selName, sizeof(selName));
            FormatSourceName(cur, mapIsCv, curName, sizeof(curName));
            snprintf(buf, sizeof(buf), " %s  was:%.8s", selName, curName);
            Display::DrawString(0, ROW_HDR(1), buf, Display::FONT_5X7);
            Display::InvertRect(0, ROW_HDR(1), 128, 8);

            // context line: position in list / learn state
            if (mapLearning) {
                snprintf(buf, sizeof(buf), " Send CC now...");
            } else if (mapLearnFull) {
                snprintf(buf, sizeof(buf), " All 8 CC slots full!");
            } else {
                snprintf(buf, sizeof(buf), " %d/%d  OK=sel BK=exit",
                         mapPos + 1, sourceCount());
            }
            Display::DrawString(0, ROW_HDR(2), buf, Display::FONT_5X7);
            Display::Flush();
        }

        void UIMenuPageParams::redrawPresets() {
            Display::Clear();
            if (presetCount == 0) {
                Display::DrawString(0, 24, "No presets", Display::FONT_5X7);
                Display::Flush();
                return;
            }
            int visible = ClampVisible(presetCount, scrollOffset, VISIBLE_ITEMS);
            for (int i = 0; i < visible; i++) {
                int idx = scrollOffset + i;
                const PresetInfo &pi = presets[idx];
                int y = ROW(i);
                char buf[64];
                snprintf(buf, sizeof(buf), "%s", pi.name);
                Display::DrawString(0, y, buf, Display::FONT_5X7);
            }
            int cy = ROW(cursor - scrollOffset);
            Display::InvertRect(0, cy, 128, 8);
            if (presetCount > VISIBLE_ITEMS)
                Display::DrawScrollbar(SCROLLBAR_X, ITEM_Y0, VISIBLE_ITEMS * LINE_H, presetCount, cursor);
            Display::Flush();
        }
    }
}
