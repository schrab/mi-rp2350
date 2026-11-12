/***************
Rack UI controller — T4a preset bank (raw flash, esp_partition).

The C3 OWNS the preset bank (plan "Presets": the C3's own 4 MB flash —
writes never touch the RP2350's XIP audio path). 16 user slots + a separate
"last state" autosave, one 4 KB flash sector per slot and 4 rotating sectors
for the autosave:

  partition "presets" (data, 0x99, 0x3E0000, 0x20000 = 32 sectors):
    sector 0..15   slot 0..15   "RKPB" records
    sector 16..19  autosave     "RKPA" records, highest valid seq wins
    sectors 20..31 spare (future wear-spreading if a slot ever wears)

Record: [magic4][slot u8][rpLen u16 LE][c3Len u16 LE][rp part][c3 part]
        [crc16 LE over rp+c3]. The RP2350 part is OPAQUE to this code
        (serialized + parsed only by rack-rp2350/src/preset.h, ~879 B);
        the C3 part is PresetStore's own 12-byte blob. rpLen+c3Len <= 1016.

Sector writes: erase (4 KB) then write the record + 0xFF padding — the
presets are saved by explicit gesture (SAVE ring action) or the debounced
autosave (>= 3 s after the last state change, rotating 4 sectors), so the
erase budget (100k cycles/sector) is a non-issue even for daily autosave.

MUST MATCH rack-rp2350/src/preset.h kPresetSize (the rp part's expected
max) — the RP2350 side validates its own blob on apply; this bank only
enforces capacity.
***************/

#pragma once

#include <cstdint>
#include <cstddef>

namespace CTAG {
    namespace CTRL {

        class PresetBank final {
        public:
            PresetBank() = delete;

            static constexpr uint8_t  kSlots       = 16;
            static constexpr uint8_t  kAutosaveSectors = 4;
            // First sector of the autosave region (after the 16 slots).
            static constexpr uint32_t kAutosaveBaseSector = kSlots;
            static constexpr size_t   kMaxRp = 1024;  // RP2350 part cap (covers 12 mod sources, 927 B)
            static constexpr size_t   kMaxC3 = 64;    // C3 part cap
            static constexpr uint16_t kC3PartVersion = 1;

            // Find the "presets" partition; false if absent (bank disabled,
            // save/load report it and nothing else breaks).
            static bool Init();

            static bool SaveSlot(uint8_t slot, const uint8_t *rp,
                                 uint16_t rpLen, const uint8_t *c3,
                                 uint16_t c3Len);
            // Loads into the caller's buffers; *rpLen/*c3Len set on success.
            static bool LoadSlot(uint8_t slot, uint8_t *rp, uint16_t *rpLen,
                                 uint8_t *c3, uint16_t *c3Len);

            // Autosave (same record shape, "RKPA" magic, rotating sectors).
            static bool SaveAutosave(const uint8_t *rp, uint16_t rpLen,
                                     const uint8_t *c3, uint16_t c3Len);
            static bool LoadAutosave(uint8_t *rp, uint16_t *rpLen,
                                     uint8_t *c3, uint16_t *c3Len);

            // Erase slots + autosave (bench "preset clear").
            static void ClearAll();

            // True when the partition was found (console `preset` status).
            static bool Ready();
            // Last autosave seq (wear telemetry).
            static uint16_t AutosaveSeq();
        };

    } // namespace CTRL
} // namespace CTAG
