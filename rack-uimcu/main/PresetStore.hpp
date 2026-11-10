/***************
Rack UI controller — T4a preset flows (bank <-> link <-> UI orchestration).

SAVE  (SYS ring "SAVE" / console `preset save <slot>`):
  SendPresetReq(slot) -> the RP2350 bursts its part as PRESET_PART chunks
  -> OnPresetPart reassembles -> PresetBank::SaveSlot(slot, rp, c3part).
  The C3 part (12 B, written below) rides along: version, last page,
  active-module mirror, slot. RP2350 audio is untouched (flash writes are
  HERE, on the C3).

LOAD  (SYS ring "LOAD" / console `preset load <slot>`):
  PresetBank::LoadSlot -> apply the C3 part locally (page) -> send the RP2350
  part back as chunked PRESET_APPLY frames -> the RP2350 validates
  (magic/version/crc) + applies via its pending machinery and acks a full
  TELEM snapshot (the renderer just follows the acks).

AUTOSAVE: the RP2350 marks its state dirty on every persistent change and
pushes a PRESET_PART burst with slot 0xFF once the state has settled (3 s
debounce, its side); OnPresetPart(0xFF) writes the 4-sector rotating
autosave. BootRestore() fires once when the link first comes online and
replays the autosave through the same LOAD path — "power-cycle restores
exactly where you were" (the plan's preset decision).

The only C3-side knowledge of the RP2350 part is its byte 5 (active module
id, kPresetSize layout) — used to validate the stored page for that module.
***************/

#pragma once

#include <cstdint>

namespace CTAG {
    namespace CTRL {

        class PresetStore final {
        public:
            PresetStore() = delete;

            // PresetBank::Init + read the autosave seq (boot telemetry).
            static void Init();

            // ---- flows ----
            static void SaveTo(uint8_t slot);    // 0-15
            static void LoadFrom(uint8_t slot);  // 0-15
            static void ClearBank();             // bench wipe (slots+autosave)

            // RackLink dispatch hook for PRESET_PART frames (reassembles;
            // completes a save or an autosave write).
            static void OnPresetPart(const uint8_t *pl, uint8_t len);

            // Call from the UI task loop: collect timeout + one-shot boot
            // restore when the link first comes online.
            static void Poll();

            // Console `preset` command (info/save/load/clear).
            static void ConsoleCmd(const char *args);

            static bool BootRestored();
        };

    } // namespace CTRL
} // namespace CTAG
