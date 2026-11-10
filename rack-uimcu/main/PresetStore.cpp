/***************
Rack UI controller — T4a preset flows implementation (see the header).
***************/

#include "PresetStore.hpp"

#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

#include "PresetBank.hpp"
#include "RackLink.hpp"
#include "RackUI.hpp"

static const char *TAG = "PresetStore";

namespace CTAG {
    namespace CTRL {

        namespace {

            // C3 part: [version][page][module][slot][pad x8] = 12 B.
            constexpr size_t kC3Len = 12;
            constexpr uint8_t kAutosaveSlot = 0xFF;
            constexpr int64_t kCollectTimeoutUs = 3000000;  // 3 s

            // PRESET_PART reassembly: slot 0xFF = autosave push, 0-15 = a
            // save-flow collect. One transfer at a time (the flows never
            // overlap: SAVE waits, autosave push is debounce-serialized on
            // the RP2350 side).
            struct Collect {
                bool active = false;
                uint8_t slot = 0;
                uint8_t total = 0;
                uint8_t have = 0;
                int64_t start_us = 0;
                uint8_t rp[PresetBank::kMaxRp];
                uint16_t rpLen = 0;
            };
            Collect s_collect;

            bool s_boot_restored = false;

            void fillC3Part(uint8_t *c3, uint8_t slot) {
                memset(c3, 0, kC3Len);
                c3[0] = PresetBank::kC3PartVersion;
                RackTelemPage t;
                c3[1] = RackLink::TelemGet(t) ? t.page : 0;  // last page
                c3[2] = t.valid ? t.module : 0;              // module mirror
                c3[3] = slot;
            }

            void applyC3Part(const uint8_t *c3) {
                if (c3[0] != PresetBank::kC3PartVersion) {
                    ESP_LOGW(TAG, "c3 part version %u (want %u) - page not "
                                  "restored", c3[0],
                             PresetBank::kC3PartVersion);
                    return;
                }
                uint8_t page = c3[1] < RackLink::kPageCount ? c3[1] : 0;
                // P2 only exists for modules with secondary parameters (the
                // RP2350's byte-5 module mirror; page validity re-checked).
                if (page == RackLink::kPageP2) {
                    const uint8_t mod =
                        c3[2] < 4 ? c3[2] : 0;
                    if (mod != 0) page = 0;  // only elements has a P2 today
                }
                RackUI::SetPage(page);
            }

            void sendApply(const uint8_t *rp, uint16_t rpLen) {
                if (!RackLink::SendPresetApply(rp, rpLen)) {
                    ESP_LOGW(TAG, "PRESET_APPLY send failed");
                }
            }

        } // namespace

        void PresetStore::Init() { PresetBank::Init(); }

        void PresetStore::SaveTo(uint8_t slot) {
            if (slot >= PresetBank::kSlots) {
                ESP_LOGW(TAG, "save: slot %u out of range", slot);
                return;
            }
            if (!PresetBank::Ready()) {
                ESP_LOGW(TAG, "save: bank not ready (no presets partition)");
                return;
            }
            if (s_collect.active) {
                ESP_LOGW(TAG, "save: another collect in flight");
                return;
            }
            s_collect = Collect{};
            s_collect.active = true;
            s_collect.slot = slot;
            s_collect.start_us = esp_timer_get_time();
            RackLink::SendPresetReq(slot);
            ESP_LOGI(TAG, "save: requested the RP2350 part for slot %u "
                          "(awaiting PRESET_PART chunks)", slot);
        }

        void PresetStore::LoadFrom(uint8_t slot) {
            if (slot >= PresetBank::kSlots) {
                ESP_LOGW(TAG, "load: slot %u out of range", slot);
                return;
            }
            if (!PresetBank::Ready()) {
                ESP_LOGW(TAG, "load: bank not ready");
                return;
            }
            static uint8_t rp[PresetBank::kMaxRp];   // static: not task stack
            static uint8_t c3[PresetBank::kMaxC3];
            uint16_t rl = 0, cl = 0;
            if (!PresetBank::LoadSlot(slot, rp, &rl, c3, &cl)) {
                ESP_LOGW(TAG, "load: slot %u empty or corrupt", slot);
                return;
            }
            ESP_LOGI(TAG, "load: slot %u (%u B rp part) -> PRESET_APPLY",
                     slot, rl);
            applyC3Part(c3);
            sendApply(rp, rl);
        }

        void PresetStore::ClearBank() {
            PresetBank::ClearAll();
            s_collect.active = false;
            ESP_LOGI(TAG, "bank cleared (16 slots + autosave)");
        }

        void PresetStore::OnPresetPart(const uint8_t *pl, uint8_t len) {
            // [slot][seq][total][data...]
            if (len < 3) {
                ESP_LOGW(TAG, "PART chunk too short (%u)", len);
                return;
            }
            const uint8_t slot = pl[0];
            const uint8_t seq = pl[1];
            const uint8_t total = pl[2];
            const uint8_t take = (uint8_t)(len - 3);

            const bool autosave = (slot == kAutosaveSlot);
            if (total == 0 || total > 8 || seq >= total) {
                ESP_LOGW(TAG, "PART bad seq/total %u/%u - dropped", seq,
                         total);
                s_collect.active = false;
                return;
            }
            if (!autosave) {
                if (!s_collect.active || s_collect.slot != slot) {
                    ESP_LOGW(TAG, "PART slot %u without a save request - "
                                  "dropped", slot);
                    return;
                }
            } else if (!s_collect.active) {
                // Autosave pushes arrive unsolicited: start an implicit
                // collect (the RP2350 debounce means they never overlap).
                s_collect = Collect{};
                s_collect.active = true;
                s_collect.slot = kAutosaveSlot;
                s_collect.start_us = esp_timer_get_time();  // fresh window!
            }
            Collect &c = s_collect;
            if (total != c.total) {
                c.total = total;
                c.have = 0;
                c.rpLen = 0;
            }
            if (seq != c.have) {
                ESP_LOGW(TAG, "PART hole (want %u got %u) - restart", c.have,
                         seq);
                c.have = 0;
                if (seq != 0) return;
            }
            const uint32_t off = (uint32_t)seq * 252;
            if (off + take > PresetBank::kMaxRp) {
                ESP_LOGW(TAG, "PART overflow - dropped");
                c.active = false;
                return;
            }
            memcpy(c.rp + off, pl + 3, take);
            ++c.have;
            c.rpLen = (uint16_t)(off + take);
            if (c.have != c.total) return;

            // Complete burst -> write the bank (RP2350 audio untouched).
            static uint8_t c3[PresetBank::kMaxC3];
            fillC3Part(c3, c.slot);
            if (c.slot == kAutosaveSlot) {
                const bool ok =
                    PresetBank::SaveAutosave(c.rp, c.rpLen, c3, kC3Len);
                ESP_LOGI(TAG, "autosave written (%u B rp part, seq %u): %s",
                         c.rpLen, PresetBank::AutosaveSeq(),
                         ok ? "OK" : "FAILED");
            } else {
                const bool ok =
                    PresetBank::SaveSlot(c.slot, c.rp, c.rpLen, c3, kC3Len);
                ESP_LOGI(TAG, "slot %u saved (%u B rp part + %u B c3 part): "
                              "%s", c.slot, c.rpLen, kC3Len,
                         ok ? "OK" : "FAILED");
            }
            c.active = false;
        }

        bool PresetStore::BootRestored() { return s_boot_restored; }

        void PresetStore::Poll() {
            // Collect timeout (RP2350 absent / link dropped mid-save).
            if (s_collect.active &&
                esp_timer_get_time() - s_collect.start_us >
                    kCollectTimeoutUs) {
                ESP_LOGW(TAG, "collect timed out (slot 0x%02x, %u/%u chunks)",
                         s_collect.slot, s_collect.have, s_collect.total);
                s_collect.active = false;
            }

            // One-shot boot restore: the link online + the first TELEM
            // landed (the RP2350 is up and answering). The autosave holds
            // "exactly where you were" across the power cycle.
            if (!s_boot_restored) {
                RackTelemPage t;
                if (RackLink::IsOnline() && RackLink::TelemGet(t) &&
                    t.valid) {
                    s_boot_restored = true;   // even if nothing valid below
                    static uint8_t rp[PresetBank::kMaxRp];
                    static uint8_t c3[PresetBank::kMaxC3];
                    uint16_t rl = 0, cl = 0;
                    if (PresetBank::LoadAutosave(rp, &rl, c3, &cl)) {
                        ESP_LOGI(TAG, "boot restore: autosave found (%u B "
                                      "rp part, seq %u) - applying", rl,
                                 PresetBank::AutosaveSeq());
                        applyC3Part(c3);
                        sendApply(rp, rl);
                    } else {
                        ESP_LOGI(TAG, "boot restore: no autosave (factory "
                                      "state stands)");
                    }
                }
            }
        }

        void PresetStore::ConsoleCmd(const char *args) {
            while (args != nullptr && *args == ' ') ++args;
            if (args == nullptr || *args == '\0' ||
                strcasecmp(args, "info") == 0) {
                RackTelemPage t;
                const bool tv = RackLink::TelemGet(t);
                printf("preset: bank %s | autosave seq %u | boot-restored %u "
                       "| page %u module %u | save <0-15> / load <0-15> / "
                       "clear\r\n",
                       PresetBank::Ready() ? "ready" : "ABSENT",
                       PresetBank::AutosaveSeq(),
                       s_boot_restored ? 1 : 0,
                       tv ? t.page : 0, tv ? t.module : 0);
                return;
            }
            if (strcasecmp(args, "clear") == 0) {
                PresetStore::ClearBank();
                return;
            }
            if (strncasecmp(args, "save ", 5) == 0) {
                const int s = atoi(args + 5);
                if (s < 0 || s >= PresetBank::kSlots) {
                    printf("preset: slot 0-%u\r\n", PresetBank::kSlots - 1);
                } else {
                    PresetStore::SaveTo((uint8_t)s);
                    printf("preset: save %d requested\r\n", s);
                }
                return;
            }
            if (strncasecmp(args, "load ", 5) == 0) {
                const int s = atoi(args + 5);
                if (s < 0 || s >= PresetBank::kSlots) {
                    printf("preset: slot 0-%u\r\n", PresetBank::kSlots - 1);
                } else {
                    PresetStore::LoadFrom((uint8_t)s);
                    printf("preset: load %d sent\r\n", s);
                }
                return;
            }
            printf("preset: usage [info|save <0-15>|load <0-15>|clear]\r\n");
        }

    } // namespace CTRL
} // namespace CTAG
