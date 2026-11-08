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

// DspProxy: the panel-side stand-in for every DSP-module call the menu
// pages make (the SDD §6 catalog: SPManager, MarblesEngine, Codec, SDAudio,
// Favorites, SD listing). Pages call ONLY this class — the P2 stub serves
// fake state locally; from P3 the same signatures are backed by the UART
// link (reads from the C3-local cache per SDD §4.3, writes as link
// commands, invalidated by DSP pushes).
//
// Method names mirror the A1S originals so the ported pages read the same;
// SDAudio methods are prefixed Sd* because the names are too generic to
// widen into the proxy namespace.

#pragma once

#include <cstdint>
#include <string>
#include <algorithm>

namespace CTAG {
    namespace CTRL {

        // The ported pages were written against SPManager.hpp's transitive
        // `using namespace std` (ctagSoundProcessor.hpp). These keep them
        // compiling unchanged without that wholesale import.
        using std::string;
        using std::to_string;
        using std::max;

        class DspProxy final {
        public:
            DspProxy() = delete;

            // Seed the stub state (UIMCU_DSP_STUB build) / open the link
            // session (link build). Must run before UIMenu::Init().
            static void Init();
            // True between the version handshake and the offline watchdog
            // (link build); always true on the stub.
            static bool IsOnline();
            // Link build only: entry point for every complete frame the
            // link client receives (pushes, stream, fsList replies).
            static void OnLinkFrame(const char *payload, size_t len);

            // ---- SPManager surface (plugin list, params, config) ----
            // All Get* return pointers into the local cache; the cache only
            // changes on pushes/acks, never mid-redraw.
            static const char *GetCStrJSONSoundProcessors();
            static const char *GetCStrJSONActivePluginParams(int chan);
            static const char *GetCStrJSONGetPresets(int chan);
            static const char *GetCStrJSONConfiguration();
            static void SetConfigurationFromJSON(const std::string &json);
            static std::string GetStringID(int chan);
            static bool IsPluginStereo(const std::string &id);
            static void SetSoundProcessorChannel(int chan, const std::string &id);
            // key is "current", "cv" or "trig" (SDD §6).
            static void SetChannelParamValue(int chan, const std::string &id,
                                             const std::string &key, int value);
            static void ChannelSavePreset(int chan, const std::string &name, int number);
            static void ChannelLoadPreset(int chan, int number);
            static void SetMeteringEnabled(bool on);
            static bool GetCPUOverload();

            // ---- metering (from the 10 Hz stream, SDD §4.2) ----
            // 0=inL 1=inR 2=outL 3=outR, scaled 0..1000.
            static uint32_t GetVUPeak(int channel);

            // ---- Codec surface ----
            static constexpr int MAX_OUTPUT_LEVEL = 33;     // es8388 OUTPUT_VOLUME_MAX
            static constexpr int DEFAULT_OUTPUT_LEVEL = 33; // 0 dB
            static void SetInputGain(int gain);
            static void SetOutputLevels(int left, int right);

            // ---- SDAudio surface ----
            static void SdStartPlayback(const char *path);
            static void SdStartRecording(const char *path);
            static void SdStop();
            static bool SdIsPlaying();
            static bool SdIsRecording();
            static uint32_t SdGetTotalFrames();
            static uint32_t SdGetPositionFrames();

            // ---- Favorites surface ----
            static std::string GetAllFavorites();
            static int32_t GetActiveFav();
            static void ActivateFavorite(int id);

            // ---- MIDI source enables (P4, ground truth on the DSP) ----
            static bool IsMidiUartEnabled();
            static bool IsMidiBleEnabled();
            // Optimistic cache update + /link/midiFlags set frame.
            static void SetMidiFlags(bool uart, bool ble);
            // Ask the DSP for current flags (reply updates the cache).
            static void RefreshMidiFlags();

            // ---- filesystem proxy (replaces direct /sd opendir) ----
            static bool IsSDMounted();
            // List one directory of the DSP-side SD card. names/isDir are
            // caller arrays of `max` entries; returns the entry count (no
            // "." or ".." entries; the pages add ".." themselves).
            static int ListSdDir(const char *path, char names[][32], bool *isDir, int max);

            // ---- MarblesEngine surface (full static mirror) ----
            static void RequestReset();
            static uint32_t GetOutputSnapshot();

            static int GetTModel();    static void SetTModel(int v);
            static int GetTRange();    static void SetTRange(int v);
            static float GetTRate();   static void SetTRate(float v);
            static float GetTBias();   static void SetTBias(float v);
            static float GetTJitter(); static void SetTJitter(float v);
            static int GetTPwMean();   static void SetTPwMean(int v);
            static int GetTPwStd();    static void SetTPwStd(int v);
            static int GetTDejaVu();   static void SetTDejaVu(int v);
            static bool GetTExternal(); static void SetTExternal(bool v);
            static int GetXYClockSource(); static void SetXYClockSource(int v);
            static int GetXScale();    static void SetXScale(int v);
            static int GetXRange();    static void SetXRange(int v);
            static int GetXControl();  static void SetXControl(int v);
            static bool GetXRegister(); static void SetXRegister(bool v);
            static float GetXRegisterValue(); static void SetXRegisterValue(float v);
            static float GetXSpread(); static void SetXSpread(float v);
            static float GetXBias();   static void SetXBias(float v);
            static float GetXSteps();  static void SetXSteps(float v);
            static int GetXDejaVu();   static void SetXDejaVu(int v);
            static int GetYRange();    static void SetYRange(int v);
            static float GetYSpread(); static void SetYSpread(float v);
            static float GetYBias();   static void SetYBias(float v);
            static float GetYSteps();  static void SetYSteps(float v);
            static int GetYDivider();  static void SetYDivider(int v);
            static int GetYDejaVu();   static void SetYDejaVu(int v);
            static float GetDejaVuAmount(); static void SetDejaVuAmount(float v);
            static int GetDejaVuLength();   static void SetDejaVuLength(int v);
            static int GetTSlot(int i);     static void SetTSlot(int i, int slot);
            static int GetXSlot(int i);     static void SetXSlot(int i, int slot);
            static int GetYSlot();          static void SetYSlot(int slot);
            static void StartLearn();
            static void StopLearn();
            static bool IsLearning();
            static int GetLastLearnedSlot();
            static int GetDynamicSlotCC(int slot_idx);
            static void SetDynamicSlotCC(int slot_idx, int cc);
            static void SetDynamicSlotChan(int slot_idx, int chan);
            static int GetDynamicSlotChan(int slot_idx);
            static void SaveConfig();
        };
    }
}
