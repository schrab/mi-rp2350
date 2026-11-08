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

// Stub implementation (CMake option UIMCU_DSP_STUB): realistic fake state
// served locally, so the panel hardware and the whole menu port can be
// brought up with no DSP and no link. Shapes of the JSON blobs match the
// real A1S serializers (plugin list, param specs, config, presets,
// favorites) so the pages are exercised exactly as they would be on live
// data. The production build uses DspProxyLink.cpp instead.

#include "DspProxy.hpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std;

namespace CTAG {
    namespace CTRL {

        namespace {
            // Slot bounds mirror the BBA DSP slot space (top-level CMakeLists:
            // N_CVS=102, N_TRIGS=43). The link protocol keeps these in sync.
            constexpr int kNCvs = 102;
            constexpr int kNTrigs = 43;

            int clampI(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
            float clampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

            // --- fake plugin bundle (shaped like SPManagerDataModel output) ---
            const char *kPluginListJson =
                "[{\"id\":\"ctagSoundProcessorElements\",\"name\":\"Elements\",\"isStereo\":true},"
                "{\"id\":\"TestDelay\",\"name\":\"Delay\",\"isStereo\":true},"
                "{\"id\":\"TestMonoSynth\",\"name\":\"MonoSynth\",\"isStereo\":false},"
                "{\"id\":\"TestFilter\",\"name\":\"Filter\",\"isStereo\":false},"
                "{\"id\":\"TestChorus\",\"name\":\"Chorus\",\"isStereo\":true},"
                "{\"id\":\"TestDrive\",\"name\":\"Drive\",\"isStereo\":false}]";

            // Params for the fake ch0 plugin: exercises bool, enum, ranged int,
            // a group, and the momentary "play" param (Params page paths).
            const char *kParamsMonoJson =
                "{\"params\":["
                "{\"id\":\"tune\",\"name\":\"Tune\",\"type\":\"int\",\"min\":-12,\"max\":12,\"current\":0,\"cv\":-1},"
                "{\"id\":\"decay\",\"name\":\"Decay\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":42,\"cv\":-1},"
                "{\"id\":\"type\",\"name\":\"Type\",\"type\":\"enum\",\"min\":0,\"max\":3,\"current\":1},"
                "{\"id\":\"freeze\",\"name\":\"Freeze\",\"type\":\"bool\",\"min\":0,\"max\":1,\"current\":0,\"trig\":-1},"
                "{\"id\":\"group\",\"name\":\"Envelope\",\"type\":\"group\",\"params\":["
                "{\"id\":\"attack\",\"name\":\"Attack\",\"type\":\"int\",\"min\":0,\"max\":255,\"current\":10,\"cv\":-1},"
                "{\"id\":\"release\",\"name\":\"Release\",\"type\":\"int\",\"min\":0,\"max\":255,\"current\":200,\"cv\":-1}]}"
                "]}";

            const char *kParamsElementsJson =
                "{\"params\":["
                "{\"id\":\"strength\",\"name\":\"Strength\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":50,\"cv\":-1},"
                "{\"id\":\"contour\",\"name\":\"Contour\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":30,\"cv\":-1},"
                "{\"id\":\"play\",\"name\":\"Play\",\"type\":\"bool\",\"min\":0,\"max\":1,\"current\":0,\"trig\":-1},"
                "{\"id\":\"group\",\"name\":\"Resonator\",\"type\":\"group\",\"params\":["
                "{\"id\":\"structure\",\"name\":\"Structure\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":64,\"cv\":-1},"
                "{\"id\":\"brightness\",\"name\":\"Brightness\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":50,\"cv\":-1},"
                "{\"id\":\"damping\",\"name\":\"Damping\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":50,\"cv\":-1},"
                "{\"id\":\"position\",\"name\":\"Position\",\"type\":\"int\",\"min\":0,\"max\":100,\"current\":35,\"cv\":-1}]}"
                "]}";

            const char *kParamsDefaultJson = "{\"params\":[]}";

            const char *kPresetsJson =
                "{\"presets\":["
                "{\"name\":\"Init\",\"number\":0},"
                "{\"name\":\"User 1\",\"number\":1},"
                "{\"name\":\"User 2\",\"number\":2}]}";

            // The full config as the System page and MIX page expect to find it
            // (same keys as the A1S stored model).
            const char *kConfigJson =
                "{\"ng_config\":\"dual\",\"ch01_daisy\":\"off\",\"ch0_toStereo\":\"off\","
                "\"ch1_toStereo\":\"off\",\"ch0_outputSoftClip\":\"off\",\"ch1_outputSoftClip\":\"off\","
                "\"ch0_codecLvlOut\":\"33\",\"ch1_codecLvlOut\":\"33\",\"input_source\":\"mic\","
                "\"input_gain\":\"0\",\"output_source\":\"all\",\"mixer_mode\":\"dac\","
                "\"oled_brightness\":\"255\"}";

            const char *kFavoritesJson =
                "[{\"name\":\"Fav 0\",\"plug_0\":\"Elements\",\"plug_1\":\"\"},"
                "{\"name\":\"Fav 1\",\"plug_0\":\"TestDelay\",\"plug_1\":\"TestDelay\"},"
                "{\"name\":\"Fav 2\",\"plug_0\":\"TestMonoSynth\",\"plug_1\":\"TestMonoSynth\"},"
                "{\"name\":\"Fav 3\",\"plug_0\":\"\",\"plug_1\":\"TestFilter\"}]";

            // Stub SD tree: enough to exercise the HOME file browser (enter a
            // subdirectory, come back with ..) and the TAPE wav list.
            const char *kSdRoot[] = {"drums", "take1.wav", "take2.wav", "piano.wav"};
            const bool kSdRootDir[] = {true, false, false, false};
            const char *kSdDrums[] = {"kick.wav", "snare.wav", "hat.wav"};
            const bool kSdDrumsDir[] = {false, false, false};

            // --- mutable stub state ---
            char configBuf[1024];
            char activeId[2][32] = {"ctagSoundProcessorElements", ""};
            int32_t activeFav = -1;
            bool metering = false;
            bool sdPlaying = false, sdRecording = false;
            uint32_t sdTotalFrames = 0, sdPosFrames = 0;

            struct MarblesState {
                int tModel = 0, tRange = 1, xyClockSource = 0;
                int xScale = 0, xRange = 1, xControl = 0, xDejaVu = 0;
                int yRange = 1, yDivider = 8, yDejaVu = 0;
                int tDejaVu = 0, tpwMean = 128, tpwStd = 64;
                int dejaVuLength = 4;
                float tRate = 0, tBias = 0.5f, tJitter = 0.1f;
                float xRegisterValue = 0.5f, xSpread = 0.5f, xBias = 0.5f, xSteps = 0.5f;
                float ySpread = 0.5f, yBias = 0.5f, ySteps = 0.5f;
                float dejaVuAmount = 0.5f;
                bool tExternal = false, xRegister = false;
                int tSlot[3] = {40, 41, 42};
                int xSlot[3] = {98, 99, 100};
                int ySlot = 101;
                int ccSlot[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
                int ccChan[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                bool learning = false;
                int lastLearned = -1;
            };
            MarblesState mb;

            // Animated fake output snapshot so the MOD page's live strip
            // visibly moves during bring-up (the real engine pushes this at
            // 10 Hz from the audio task).
            uint32_t FakeSnapshot() {
                uint32_t t = (uint32_t)(xTaskGetTickCount() / 5); // ~2 Hz at 100 Hz tick
                uint32_t s = 0;
                s |= ((t % 3) == 0) ? 0x1 : 0;                          // T1
                s |= ((t % 4) == 0) ? 0x2 : 0;                          // T2
                s |= ((t % 5) == 0) ? 0x4 : 0;                          // T3
                for (int i = 0; i < 4; i++) {
                    int q = 64 + (int)((t * (i + 2)) % 40) - 20;        // wobble around 0
                    s |= ((uint32_t)q & 0x7f) << (3 + i * 7);
                }
                return s;
            }
        }

        void DspProxy::Init() {
            snprintf(configBuf, sizeof(configBuf), "%s", kConfigJson);
        }

        bool DspProxy::IsOnline() { return true; }

        const char *DspProxy::GetCStrJSONSoundProcessors() { return kPluginListJson; }

        const char *DspProxy::GetCStrJSONActivePluginParams(int chan) {
            if (chan < 0 || chan > 1) return kParamsDefaultJson;
            if (activeId[chan][0] == '\0') return kParamsDefaultJson;
            if (strcmp(activeId[chan], "ctagSoundProcessorElements") == 0) return kParamsElementsJson;
            return kParamsMonoJson;
        }

        const char *DspProxy::GetCStrJSONGetPresets(int) { return kPresetsJson; }

        const char *DspProxy::GetCStrJSONConfiguration() { return configBuf; }

        void DspProxy::SetConfigurationFromJSON(const std::string &json) {
            // Stub: overlay the keys the pages manage so a re-read shows the
            // commit took. (P3: send the full config as one link command.)
            snprintf(configBuf, sizeof(configBuf), "%s", json.c_str());
        }

        std::string DspProxy::GetStringID(int chan) {
            if (chan < 0 || chan > 1) return "";
            return activeId[chan];
        }

        bool DspProxy::IsPluginStereo(const std::string &id) {
            // Mirrors the fake bundle above.
            return id == "ctagSoundProcessorElements" || id == "TestDelay" ||
                   id == "TestChorus";
        }

        void DspProxy::SetSoundProcessorChannel(int chan, const std::string &id) {
            if (chan < 0 || chan > 1) return;
            snprintf(activeId[chan], sizeof(activeId[chan]), "%s", id.c_str());
        }

        void DspProxy::SetChannelParamValue(int, const std::string &, const std::string &, int) {
            // Stub: values live in the page's parsed copy; nothing to do.
        }

        void DspProxy::ChannelSavePreset(int, const std::string &, int) {}

        void DspProxy::ChannelLoadPreset(int, int) {}

        void DspProxy::SetMeteringEnabled(bool on) { metering = on; }

        bool DspProxy::GetCPUOverload() { return false; }

        uint32_t DspProxy::GetVUPeak(int channel) {
            if (!metering) return 0;
            // Fake meters so the MIX strips move during bring-up.
            uint32_t t = (uint32_t)xTaskGetTickCount();
            uint32_t v = (t * (37u + channel * 11u)) % 700u;
            return v;
        }

        void DspProxy::SetInputGain(int) {}

        void DspProxy::SetOutputLevels(int, int) {}

        void DspProxy::SdStartPlayback(const char *path) {
            (void)path;
            sdPlaying = true;
            sdRecording = false;
            sdTotalFrames = 44100u * 154u; // 2:34
            sdPosFrames = 0;
        }

        void DspProxy::SdStartRecording(const char *) {
            sdRecording = true;
            sdPlaying = false;
            sdPosFrames = 0;
        }

        void DspProxy::SdStop() {
            sdPlaying = false;
            sdRecording = false;
        }

        bool DspProxy::SdIsPlaying() { return sdPlaying; }

        bool DspProxy::SdIsRecording() { return sdRecording; }

        uint32_t DspProxy::SdGetTotalFrames() { return sdTotalFrames; }

        uint32_t DspProxy::SdGetPositionFrames() {
            if (sdPlaying || sdRecording) {
                sdPosFrames += 4410u; // ~4 Hz tick * 44100 Hz / 10
            }
            return sdPosFrames;
        }

        std::string DspProxy::GetAllFavorites() { return kFavoritesJson; }

        int32_t DspProxy::GetActiveFav() { return activeFav; }

        void DspProxy::ActivateFavorite(int id) { activeFav = id; }

        bool DspProxy::IsSDMounted() { return true; }

        int DspProxy::ListSdDir(const char *path, char names[][32], bool *isDir, int max) {
            const char **src;
            const bool *dirFlags;
            int n;
            if (strcmp(path, "/sd/drums") == 0) {
                src = kSdDrums;
                dirFlags = kSdDrumsDir;
                n = (int)(sizeof(kSdDrums) / sizeof(kSdDrums[0]));
            } else {
                src = kSdRoot;
                dirFlags = kSdRootDir;
                n = (int)(sizeof(kSdRoot) / sizeof(kSdRoot[0]));
            }
            int count = 0;
            for (int i = 0; i < n && count < max; i++) {
                snprintf(names[count], 32, "%s", src[i]);
                isDir[count] = dirFlags[i];
                count++;
            }
            return count;
        }

        void DspProxy::RequestReset() {}

        uint32_t DspProxy::GetOutputSnapshot() { return FakeSnapshot(); }

        int DspProxy::GetTModel() { return mb.tModel; }
        void DspProxy::SetTModel(int v) { mb.tModel = clampI(v, 0, 6); }
        int DspProxy::GetTRange() { return mb.tRange; }
        void DspProxy::SetTRange(int v) { mb.tRange = clampI(v, 0, 2); }
        float DspProxy::GetTRate() { return mb.tRate; }
        void DspProxy::SetTRate(float v) { mb.tRate = clampF(v, -60.0f, 60.0f); }
        float DspProxy::GetTBias() { return mb.tBias; }
        void DspProxy::SetTBias(float v) { mb.tBias = clampF(v, 0.0f, 1.0f); }
        float DspProxy::GetTJitter() { return mb.tJitter; }
        void DspProxy::SetTJitter(float v) { mb.tJitter = clampF(v, 0.0f, 1.0f); }
        int DspProxy::GetTPwMean() { return mb.tpwMean; }
        void DspProxy::SetTPwMean(int v) { mb.tpwMean = clampI(v, 0, 255); }
        int DspProxy::GetTPwStd() { return mb.tpwStd; }
        void DspProxy::SetTPwStd(int v) { mb.tpwStd = clampI(v, 0, 255); }
        int DspProxy::GetTDejaVu() { return mb.tDejaVu; }
        void DspProxy::SetTDejaVu(int v) { mb.tDejaVu = clampI(v, 0, 2); }
        bool DspProxy::GetTExternal() { return mb.tExternal; }
        void DspProxy::SetTExternal(bool v) { mb.tExternal = v; }
        int DspProxy::GetXYClockSource() { return mb.xyClockSource; }
        void DspProxy::SetXYClockSource(int v) { mb.xyClockSource = clampI(v, 0, 4); }
        int DspProxy::GetXScale() { return mb.xScale; }
        void DspProxy::SetXScale(int v) { mb.xScale = clampI(v, 0, 5); }
        int DspProxy::GetXRange() { return mb.xRange; }
        void DspProxy::SetXRange(int v) { mb.xRange = clampI(v, 0, 2); }
        int DspProxy::GetXControl() { return mb.xControl; }
        void DspProxy::SetXControl(int v) { mb.xControl = clampI(v, 0, 2); }
        bool DspProxy::GetXRegister() { return mb.xRegister; }
        void DspProxy::SetXRegister(bool v) { mb.xRegister = v; }
        float DspProxy::GetXRegisterValue() { return mb.xRegisterValue; }
        void DspProxy::SetXRegisterValue(float v) { mb.xRegisterValue = clampF(v, 0.0f, 1.0f); }
        float DspProxy::GetXSpread() { return mb.xSpread; }
        void DspProxy::SetXSpread(float v) { mb.xSpread = clampF(v, 0.0f, 1.0f); }
        float DspProxy::GetXBias() { return mb.xBias; }
        void DspProxy::SetXBias(float v) { mb.xBias = clampF(v, 0.0f, 1.0f); }
        float DspProxy::GetXSteps() { return mb.xSteps; }
        void DspProxy::SetXSteps(float v) { mb.xSteps = clampF(v, 0.0f, 1.0f); }
        int DspProxy::GetXDejaVu() { return mb.xDejaVu; }
        void DspProxy::SetXDejaVu(int v) { mb.xDejaVu = clampI(v, 0, 2); }
        int DspProxy::GetYRange() { return mb.yRange; }
        void DspProxy::SetYRange(int v) { mb.yRange = clampI(v, 0, 2); }
        float DspProxy::GetYSpread() { return mb.ySpread; }
        void DspProxy::SetYSpread(float v) { mb.ySpread = clampF(v, 0.0f, 1.0f); }
        float DspProxy::GetYBias() { return mb.yBias; }
        void DspProxy::SetYBias(float v) { mb.yBias = clampF(v, 0.0f, 1.0f); }
        float DspProxy::GetYSteps() { return mb.ySteps; }
        void DspProxy::SetYSteps(float v) { mb.ySteps = clampF(v, 0.0f, 1.0f); }
        int DspProxy::GetYDivider() { return mb.yDivider; }
        void DspProxy::SetYDivider(int v) { mb.yDivider = clampI(v, 0, 11); }
        int DspProxy::GetYDejaVu() { return mb.yDejaVu; }
        void DspProxy::SetYDejaVu(int v) { mb.yDejaVu = clampI(v, 0, 2); }
        float DspProxy::GetDejaVuAmount() { return mb.dejaVuAmount; }
        void DspProxy::SetDejaVuAmount(float v) { mb.dejaVuAmount = clampF(v, 0.0f, 1.0f); }
        int DspProxy::GetDejaVuLength() { return mb.dejaVuLength; }
        void DspProxy::SetDejaVuLength(int v) { mb.dejaVuLength = clampI(v, 1, 16); }
        int DspProxy::GetTSlot(int i) { return (i >= 0 && i < 3) ? mb.tSlot[i] : -1; }
        void DspProxy::SetTSlot(int i, int slot) {
            if (i >= 0 && i < 3) mb.tSlot[i] = clampI(slot, -1, kNTrigs - 1);
        }
        int DspProxy::GetXSlot(int i) { return (i >= 0 && i < 3) ? mb.xSlot[i] : -1; }
        void DspProxy::SetXSlot(int i, int slot) {
            if (i >= 0 && i < 3) mb.xSlot[i] = clampI(slot, -1, kNCvs - 1);
        }
        int DspProxy::GetYSlot() { return mb.ySlot; }
        void DspProxy::SetYSlot(int slot) { mb.ySlot = clampI(slot, -1, kNCvs - 1); }

        void DspProxy::StartLearn() { mb.learning = true; }

        void DspProxy::StopLearn() { mb.learning = false; }

        bool DspProxy::IsLearning() { return mb.learning; }

        int DspProxy::GetLastLearnedSlot() {
            // Stub learn: landing the capture frees the UI immediately.
            if (mb.learning) return -1;
            return mb.lastLearned;
        }

        int DspProxy::GetDynamicSlotCC(int slot_idx) {
            return (slot_idx >= 0 && slot_idx < 8) ? mb.ccSlot[slot_idx] : -1;
        }

        void DspProxy::SetDynamicSlotCC(int slot_idx, int cc) {
            if (slot_idx >= 0 && slot_idx < 8) mb.ccSlot[slot_idx] = clampI(cc, -1, 127);
        }

        void DspProxy::SetDynamicSlotChan(int slot_idx, int chan) {
            if (slot_idx >= 0 && slot_idx < 8) mb.ccChan[slot_idx] = clampI(chan, 0, 15);
        }

        int DspProxy::GetDynamicSlotChan(int slot_idx) {
            return (slot_idx >= 0 && slot_idx < 8) ? mb.ccChan[slot_idx] : 0;
        }

        void DspProxy::SaveConfig() {}
    }
}
