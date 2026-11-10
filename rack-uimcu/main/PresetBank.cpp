/***************
Rack UI controller — T4a preset bank implementation (see the header).
Raw esp_partition access; no filesystem component (the littlefs `storage`
partition stays reserved/unused).
***************/

#include "PresetBank.hpp"

#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"

static const char *TAG = "PresetBank";

namespace CTAG {
    namespace CTRL {

        namespace {

            constexpr uint32_t kSectorSize = 4096;
            constexpr const char *kPartName = "presets";
            constexpr uint8_t kMagicSlot[4] = {'R', 'K', 'P', 'B'};
            constexpr uint8_t kMagicAuto[4] = {'R', 'K', 'P', 'A'};
            // Header: magic4 + slot/seq u16 + rpLen u16 + c3Len u16.
            constexpr size_t kHdrLen = 12;
            constexpr size_t kMaxRecord =
                kHdrLen + PresetBank::kMaxRp + PresetBank::kMaxC3 + 2;

            const esp_partition_t *s_part = nullptr;
            uint16_t s_autosave_seq = 0;

            // CRC-16/CCITT (poly 0x1021, init 0xFFFF, MSB-first) — the same
            // family the RP2350 blob uses; self-consistency is what matters.
            uint16_t crc16(const uint8_t *d, size_t n) {
                uint16_t c = 0xFFFF;
                for (size_t i = 0; i < n; ++i) {
                    c ^= (uint16_t)((uint16_t)d[i] << 8);
                    for (int b = 0; b < 8; ++b)
                        c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021)
                                         : (uint16_t)(c << 1);
                }
                return c;
            }

            // Parse a raw sector into (rpLen, c3Len, rp*, c3*, crc). Returns
            // false for erased/foreign/corrupt sectors.
            bool parseRecord(const uint8_t *sec, const uint8_t magic[4],
                             uint16_t *rpLen, uint16_t *c3Len,
                             const uint8_t **rp, const uint8_t **c3) {
                if (memcmp(sec, magic, 4) != 0) return false;
                const uint16_t rl =
                    (uint16_t)(sec[6] | (uint16_t)sec[7] << 8);
                const uint16_t cl =
                    (uint16_t)(sec[8] | (uint16_t)sec[9] << 8);
                if (rl == 0 || rl > PresetBank::kMaxRp || cl > PresetBank::kMaxC3)
                    return false;
                if (kHdrLen + rl + cl + 2 > kSectorSize) return false;
                uint16_t crc = (uint16_t)(sec[kHdrLen + rl + cl] |
                                          (uint16_t)sec[kHdrLen + rl + cl + 1]
                                              << 8);
                if (crc != crc16(sec + kHdrLen, rl + cl)) return false;
                *rpLen = rl;
                *c3Len = cl;
                *rp = sec + kHdrLen;
                *c3 = sec + kHdrLen + rl;
                return true;
            }

            esp_err_t writeRecord(uint32_t sector, const uint8_t magic[4],
                                  uint16_t slotOrSeq, const uint8_t *rp,
                                  uint16_t rpLen, const uint8_t *c3,
                                  uint16_t c3Len) {
                static uint8_t sec[kSectorSize];  // static: not on a task stack
                memset(sec, 0xFF, sizeof(sec));
                memcpy(sec, magic, 4);
                sec[4] = (uint8_t)(slotOrSeq & 0xFF);
                sec[5] = (uint8_t)(slotOrSeq >> 8);
                sec[6] = (uint8_t)(rpLen & 0xFF);
                sec[7] = (uint8_t)(rpLen >> 8);
                sec[8] = (uint8_t)(c3Len & 0xFF);
                sec[9] = (uint8_t)(c3Len >> 8);
                sec[10] = sec[11] = 0;
                memcpy(sec + kHdrLen, rp, rpLen);
                memcpy(sec + kHdrLen + rpLen, c3, c3Len);
                const uint16_t crc = crc16(sec + kHdrLen, rpLen + c3Len);
                sec[kHdrLen + rpLen + c3Len] = (uint8_t)(crc & 0xFF);
                sec[kHdrLen + rpLen + c3Len + 1] = (uint8_t)(crc >> 8);

                const uint32_t off = sector * kSectorSize;
                esp_err_t err = esp_partition_erase_range(s_part, off,
                                                          kSectorSize);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "erase sector %lu: %s",
                             (unsigned long)sector, esp_err_to_name(err));
                    return err;
                }
                err = esp_partition_write(s_part, off, sec, kSectorSize);
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "write sector %lu: %s",
                             (unsigned long)sector, esp_err_to_name(err));
                return err;
            }

        } // namespace

        bool PresetBank::Init() {
            if (s_part != nullptr) return true;
            s_part = esp_partition_find_first(
                ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x99,
                kPartName);
            if (s_part == nullptr) {
                ESP_LOGW(TAG, "partition '%s' not found - preset bank "
                              "disabled (reflash with the new partition "
                              "table)", kPartName);
                return false;
            }
            ESP_LOGI(TAG, "bank ready: %s @0x%lx %lu KB (%u slots + %u "
                          "autosave sectors)",
                     s_part->label, (unsigned long)s_part->address,
                     (unsigned long)(s_part->size / 1024), kSlots,
                     kAutosaveSectors);
            return true;
        }

        bool PresetBank::Ready() { return s_part != nullptr; }

        uint16_t PresetBank::AutosaveSeq() { return s_autosave_seq; }

        bool PresetBank::SaveSlot(uint8_t slot, const uint8_t *rp,
                                  uint16_t rpLen, const uint8_t *c3,
                                  uint16_t c3Len) {
            if (s_part == nullptr) return false;
            if (slot >= kSlots || rpLen == 0 || rpLen > kMaxRp ||
                c3Len > kMaxC3 || kHdrLen + rpLen + c3Len + 2 > kSectorSize)
                return false;
            return writeRecord(slot, kMagicSlot, slot, rp, rpLen, c3,
                               c3Len) == ESP_OK;
        }

        bool PresetBank::LoadSlot(uint8_t slot, uint8_t *rp, uint16_t *rpLen,
                                  uint8_t *c3, uint16_t *c3Len) {
            if (s_part == nullptr || slot >= kSlots) return false;
            static uint8_t sec[kSectorSize];  // static: not on a task stack
            if (esp_partition_read(s_part, slot * kSectorSize, sec,
                                   kSectorSize) != ESP_OK)
                return false;
            const uint8_t *rpP, *c3P;
            uint16_t rl, cl;
            if (!parseRecord(sec, kMagicSlot, &rl, &cl, &rpP, &c3P))
                return false;
            memcpy(rp, rpP, rl);
            memcpy(c3, c3P, cl);
            *rpLen = rl;
            *c3Len = cl;
            return true;
        }

        bool PresetBank::SaveAutosave(const uint8_t *rp, uint16_t rpLen,
                                      const uint8_t *c3, uint16_t c3Len) {
            if (s_part == nullptr) return false;
            if (rpLen == 0 || rpLen > kMaxRp || c3Len > kMaxC3 ||
                kHdrLen + rpLen + c3Len + 2 > kSectorSize)
                return false;
            s_autosave_seq = (uint16_t)(s_autosave_seq + 1);
            return writeRecord(kAutosaveBaseSector + (s_autosave_seq %
                                                          kAutosaveSectors),
                               kMagicAuto, s_autosave_seq, rp, rpLen, c3,
                               c3Len) == ESP_OK;
        }

        bool PresetBank::LoadAutosave(uint8_t *rp, uint16_t *rpLen,
                                      uint8_t *c3, uint16_t *c3Len) {
            if (s_part == nullptr) return false;
            static uint8_t sec[kSectorSize];  // static: not on a task stack
            uint16_t best_seq = 0;
            uint32_t best_sector = 0xFFFFFFFF;
            for (uint32_t i = 0; i < kAutosaveSectors; ++i) {
                if (esp_partition_read(s_part,
                                       (kAutosaveBaseSector + i) * kSectorSize,
                                       sec, kSectorSize) != ESP_OK)
                    continue;
                const uint16_t seq =
                    (uint16_t)(sec[4] | (uint16_t)sec[5] << 8);
                const uint8_t *rpP, *c3P;
                uint16_t rl, cl;
                if (seq == 0 || seq == 0xFFFF) continue;
                if (!parseRecord(sec, kMagicAuto, &rl, &cl, &rpP, &c3P))
                    continue;
                if (seq >= best_seq) {
                    best_seq = seq;
                    best_sector = i;
                }
            }
            if (best_sector == 0xFFFFFFFF) return false;
            if (esp_partition_read(s_part,
                                   (kAutosaveBaseSector + best_sector) *
                                       kSectorSize,
                                   sec, kSectorSize) != ESP_OK)
                return false;
            const uint8_t *rpP, *c3P;
            uint16_t rl, cl;
            if (!parseRecord(sec, kMagicAuto, &rl, &cl, &rpP, &c3P))
                return false;
            memcpy(rp, rpP, rl);
            memcpy(c3, c3P, cl);
            *rpLen = rl;
            *c3Len = cl;
            s_autosave_seq = best_seq;
            return true;
        }

        void PresetBank::ClearAll() {
            if (s_part == nullptr) return;
            esp_err_t err = esp_partition_erase_range(s_part, 0, s_part->size);
            ESP_LOGI(TAG, "clear all: %s", esp_err_to_name(err));
        }

    } // namespace CTRL
} // namespace CTAG
