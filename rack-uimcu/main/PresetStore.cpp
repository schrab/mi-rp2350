/***************
Rack UI controller — T4a preset flows implementation (see the header).
***************/

#include "PresetStore.hpp"

#include <cstdio>
#include <cstdlib>
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
            // save-flow collect. An unsolicited autosave can already be in
            // flight when a manual request is sent. Keep them separate.
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
            Collect s_autoCollect;

            bool s_boot_restored = false;

            void fillC3Part(uint8_t *c3, uint8_t slot) {
                memset(c3, 0, kC3Len);
                c3[0] = PresetBank::kC3PartVersion;
                RackTelemPage t;
                c3[1] = RackLink::TelemGet(t) ? t.page : 0;  // last page
                c3[2] = t.valid ? t.module : 0;              // module mirror
                c3[3] = slot;
            }

            bool applyC3Part(const uint8_t *c3) {
                if (c3[0] != PresetBank::kC3PartVersion) {
                    ESP_LOGW(TAG, "c3 part version %u (want %u) - page not "
                                  "restored", c3[0],
                             PresetBank::kC3PartVersion);
                    return false;
                }
                uint8_t page = c3[1] < RackLink::kPageCount ? c3[1] : 0;
                // The page must exist for the preset's module. P2 belongs to
                // Elements/Lira8; P3 is Lira8-only.
                const uint8_t mod = c3[2] < 8 ? c3[2] : 0;
                if (page == RackLink::kPageP2 && mod != 0 && mod != 4)
                    page = RackLink::kPageP1;
                if (page == 10 && mod != 4) page = RackLink::kPageP1;
                RackUI::SetPage(page);
                return true;
            }

            bool sendApply(const uint8_t *rp, uint16_t rpLen) {
                if (!RackLink::SendPresetApply(rp, rpLen)) {
                    ESP_LOGW(TAG, "PRESET_APPLY send failed");
                    return false;
                }
                return true;
            }

            void showSlotBanner(const char *message, uint8_t slot) {
                char buf[24];
                snprintf(buf, sizeof(buf), "%s P%u", message, slot);
                RackUI::ShowBanner(buf, 2500);
            }

        } // namespace

        void PresetStore::Init() { PresetBank::Init(); }

        bool PresetStore::SaveTo(uint8_t slot) {
            if (slot >= PresetBank::kSlots) {
                ESP_LOGW(TAG, "save: slot %u out of range", slot);
                showSlotBanner("SAVE FAILED", slot);
                return false;
            }
            if (!PresetBank::Ready()) {
                ESP_LOGW(TAG, "save: bank not ready (no presets partition)");
                RackUI::ShowBanner("BANK UNAVAILABLE");
                return false;
            }
            if (s_collect.active) {
                ESP_LOGW(TAG, "save: another collect in flight");
                RackUI::ShowBanner("SAVE BUSY");
                return false;
            }
            if (!RackLink::IsOnline()) {
                RackUI::ShowBanner("RACK OFFLINE");
                return false;
            }
            s_collect = Collect{};
            s_collect.active = true;
            s_collect.slot = slot;
            s_collect.start_us = esp_timer_get_time();
            showSlotBanner("SAVING", slot);
            if (!RackLink::SendPresetReq(slot)) {
                s_collect.active = false;
                showSlotBanner("SAVE FAILED", slot);
                return false;
            }
            ESP_LOGI(TAG, "save: requested the RP2350 part for slot %u "
                          "(awaiting PRESET_PART chunks)", slot);
            return true;
        }

        bool PresetStore::LoadFrom(uint8_t slot) {
            if (slot >= PresetBank::kSlots) {
                ESP_LOGW(TAG, "load: slot %u out of range", slot);
                showSlotBanner("LOAD FAILED", slot);
                return false;
            }
            if (!PresetBank::Ready()) {
                ESP_LOGW(TAG, "load: bank not ready");
                RackUI::ShowBanner("BANK UNAVAILABLE");
                return false;
            }
            if (!RackLink::IsOnline()) {
                RackUI::ShowBanner("RACK OFFLINE");
                return false;
            }
            static uint8_t rp[PresetBank::kMaxRp];   // static: not task stack
            static uint8_t c3[PresetBank::kMaxC3];
            uint16_t rl = 0, cl = 0;
            if (!PresetBank::LoadSlot(slot, rp, &rl, c3, &cl)) {
                ESP_LOGW(TAG, "load: slot %u empty or corrupt", slot);
                showSlotBanner("EMPTY/BAD", slot);
                return false;
            }
            if (cl != kC3Len || c3[0] != PresetBank::kC3PartVersion) {
                ESP_LOGW(TAG, "load: slot %u has an invalid C3 page record", slot);
                showSlotBanner("LOAD FAILED", slot);
                return false;
            }
            ESP_LOGI(TAG, "load: slot %u (%u B rp part) -> PRESET_APPLY",
                     slot, rl);
            if (!sendApply(rp, rl)) {
                showSlotBanner("LOAD FAILED", slot);
                return false;
            }
            applyC3Part(c3);
            // A queued UART write isn't proof the RP2350 applied the preset.
            showSlotBanner("LOAD SENT", slot);
            return true;
        }

        void PresetStore::ClearBank() {
            PresetBank::ClearAll();
            s_collect.active = false;
            s_autoCollect.active = false;
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
            Collect &c = autosave ? s_autoCollect : s_collect;
            if (!autosave && (!c.active || c.slot != slot)) {
                ESP_LOGW(TAG, "PART slot %u without a save request - dropped", slot);
                return;
            }
            if (total == 0 || total > 8 || seq >= total || take == 0 ||
                (seq + 1 < total && take != 252)) {
                ESP_LOGW(TAG, "PART bad chunk %u/%u (%u B) - dropped", seq,
                         total, take);
                c.active = false;
                if (!autosave) showSlotBanner("SAVE FAILED", slot);
                return;
            }
            if (autosave && seq == 0) {
                c = Collect{};
                c.active = true;
                c.slot = kAutosaveSlot;
                c.start_us = esp_timer_get_time();
            }
            if (!c.active) return;
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
                if (!autosave) showSlotBanner("SAVE FAILED", slot);
                return;
            }
            memcpy(c.rp + off, pl + 3, take);
            ++c.have;
            c.rpLen = (uint16_t)(off + take);
            if (c.have != c.total) return;

            // Complete burst -> write the bank (RP2350 audio untouched).
            c.start_us = 0;  // no longer waiting for chunks; keep slot busy during flash write
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
                showSlotBanner(ok ? "SAVED" : "SAVE FAILED", slot);
            }
            c.active = false;
        }

        bool PresetStore::BootRestored() { return s_boot_restored; }

        void PresetStore::Poll() {
            // Collect timeout (RP2350 absent / link dropped mid-save).
            const int64_t now = esp_timer_get_time();
            Collect *const channels[] = {&s_collect, &s_autoCollect};
            for (Collect *c : channels) {
                if (c->active && c->start_us != 0 &&
                    now - c->start_us > kCollectTimeoutUs) {
                    ESP_LOGW(TAG, "collect timed out (slot 0x%02x, %u/%u chunks)",
                             c->slot, c->have, c->total);
                    c->active = false;
                    if (c == &s_collect) showSlotBanner("SAVE TIMEOUT", c->slot);
                }
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
                        if (cl == kC3Len &&
                            c3[0] == PresetBank::kC3PartVersion &&
                            applyC3Part(c3)) {
                            sendApply(rp, rl);
                        }
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
                    const bool sent = PresetStore::SaveTo((uint8_t)s);
                    printf("preset: save %d %s\r\n", s, sent ? "requested" : "failed");
                }
                return;
            }
            if (strncasecmp(args, "load ", 5) == 0) {
                const int s = atoi(args + 5);
                if (s < 0 || s >= PresetBank::kSlots) {
                    printf("preset: slot 0-%u\r\n", PresetBank::kSlots - 1);
                } else {
                    const bool sent = PresetStore::LoadFrom((uint8_t)s);
                    printf("preset: load %d %s\r\n", s, sent ? "sent" : "failed");
                }
                return;
            }
            printf("preset: usage [info|save <0-15>|load <0-15>|clear]\r\n");
        }

    } // namespace CTRL
} // namespace CTAG
