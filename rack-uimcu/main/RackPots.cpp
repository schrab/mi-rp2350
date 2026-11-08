/***************
Rack UI controller — HC4067 pot scan implementation (see RackPots.hpp).

Send target resolution lives with the page state: RackUI::GetPotTarget()
maps a pot index to the ACTIVE page's cell (CC number, or (src, param) for
MOD pages) — RackPots stays a dumb scanner/sender.
***************/

#include "RackPots.hpp"

#include "RackLink.hpp"
#include "RackUI.hpp"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include <cstring>

static const char *TAG = "RackPots";

namespace CTAG {
    namespace CTRL {

        namespace {

            constexpr int PIN_ADC = 0;    // ADC1_CH0 (mux COM)
            constexpr int PIN_S0 = 1;
            constexpr int PIN_S1 = 2;     // strapping pin (external pull-up)
            constexpr int PIN_S2 = 3;
            constexpr int PIN_S3 = 6;

            constexpr uint8_t NPOTS = 16;
            constexpr int DEADBAND = 2;            // counts vs last SENT
            constexpr int PICK_BAND = 1;           // pickup alignment window
            constexpr int CAL_WINDOW_MS = 8000;    // `calib` sweep window
            // Rate limit: <= 200 msgs/s aggregate = one send per 5 ms.
            constexpr int64_t SEND_PERIOD_US = 5000;
            // Own-echo window: a TELEM delta arriving within this after OUR
            // send of the same value is the firmware's echo, never an
            // external change (deltas are throttled to ~50 ms RP2350-side).
            constexpr int64_t ECHO_WINDOW_US = 400000;

            struct PotCal { uint16_t min, max; };
            PotCal cal[NPOTS];                     // RAM copy (full range if none)
            bool calStored = false;                // NVS has a stored blob
            bool calRunning = false;

            uint16_t lastSent[NPOTS];              // last SENT 0-127 value
            uint16_t lastValue[NPOTS];             // last mapped value
            int64_t lastSendPotUs[NPOTS];          // per-pot last SEND time
            int64_t lastSendUs = 0;
            int64_t calStartUs = 0;

            // ---- soft takeover state (UI task writes, scan task reads;
            // single-byte fields, benign races cost one scan cycle) ----
            volatile bool pickPending[NPOTS];
            volatile uint8_t pickTarget[NPOTS];    // cell value to align to

            // The per-source SPEED param a pot rides on a mod-routed cell
            // (user spec 2026-11-08: "like for LFO - its changing LFO rate").
            // MUST MATCH the kModParamNames indexes in
            // rack-rp2350/src/mod_engine.h: LFO 1 = rate, EU 4 = div,
            // GATE 4 = div, ADSR 6 = div. (Gate-follow ADSR ignores div —
            // the ride is a no-op there; retrigger mode rides the speed.)
            uint8_t rideParam(uint8_t src) {
                if (src <= 3) return 1;            // LFO: rate
                if (src <= 7) return 4;            // EUclid: clock division
                if (src <= 9) return 4;            // rhythmic gate: division
                return 6;                          // ADSR: division
            }

            void addrSet(uint8_t ch) {
                gpio_set_level((gpio_num_t)PIN_S0, ch & 1);
                gpio_set_level((gpio_num_t)PIN_S1, (ch >> 1) & 1);
                gpio_set_level((gpio_num_t)PIN_S2, (ch >> 2) & 1);
                gpio_set_level((gpio_num_t)PIN_S3, (ch >> 3) & 1);
            }

            void loadCal() {
                for (int i = 0; i < NPOTS; ++i) {
                    cal[i].min = 0;
                    cal[i].max = 4095;
                }
                nvs_handle_t h;
                if (nvs_open("rpots", NVS_READONLY, &h) != ESP_OK) return;
                size_t sz = sizeof(cal);
                if (nvs_get_blob(h, "cal", cal, &sz) == ESP_OK &&
                    sz == sizeof(cal)) {
                    calStored = true;
                    ESP_LOGI(TAG, "NVS calibration loaded");
                }
                nvs_close(h);
            }

            void saveCal() {
                nvs_handle_t h;
                if (nvs_open("rpots", NVS_READWRITE, &h) != ESP_OK) {
                    ESP_LOGW(TAG, "nvs open failed - calibration not stored");
                    return;
                }
                nvs_set_blob(h, "cal", cal, sizeof(cal));
                nvs_commit(h);
                nvs_close(h);
                calStored = true;
            }

            // Raw ADC -> 0-127 through the per-pot window.
            uint8_t mapValue(uint8_t pot, int raw) {
                int lo = cal[pot].min, hi = cal[pot].max;
                if (hi - lo < 64) return 0;        // dead/shorted pot guard
                int v = (raw - lo) * 127 / (hi - lo);
                if (v < 0) v = 0;
                if (v > 127) v = 127;
                return (uint8_t)v;
            }

            void sendTarget(uint8_t pot, uint8_t value) {
                RackUI::PotTarget t;
                if (!RackUI::GetPotTarget(pot, t)) return;
                const uint8_t pkt[4] = {
                    0x0B, 0xB0, t.cc, value        // CIN 0xB (CC), cable 0
                };
                switch (t.kind) {
                    case RackUI::PotTarget::kCC:
                        RackLink::SendMidi(pkt);
                        break;
                    case RackUI::PotTarget::kMod:
                        RackLink::SendModSet(t.src, t.param, value);
                        break;
                    case RackUI::PotTarget::kGlobal:
                        // Global state MOD_SET cannot express (the MOD-C BPM
                        // pot): UI_CMD action 5 [item][value].
                        RackLink::SendUiCmd(RackLink::kCmdSetGlobal, t.cc,
                                            value);
                        break;
                    default:
                        break;                     // SYS page: pots inert
                }
            }

            void sendAssign(uint8_t pot, bool up) {
                RackLink::SendAssignCycle(pot, up ? 1 : 0);
            }

            void potTask(void *) {
                uint8_t ch = 0;
                while (true) {
                    // Address settle: the HC4067 switches in ~ns; a short
                    // delay + the ADC's own sampling window is plenty.
                    addrSet(ch);
                    esp_rom_delay_us(8);
                    const int raw = adc1_get_raw(ADC1_CHANNEL_0);
                    const int64_t now = esp_timer_get_time();

                    if (calRunning) {
                        // Calibration sweep: widen the per-pot window; end
                        // the window after CAL_WINDOW_MS and store.
                        if (raw < (int)cal[ch].min) cal[ch].min = (uint16_t)raw;
                        if (raw > (int)cal[ch].max) cal[ch].max = (uint16_t)raw;
                        if (now - calStartUs >= CAL_WINDOW_MS * 1000) {
                            calRunning = false;
                            saveCal();
                            for (int i = 0; i < NPOTS; ++i) {
                                ESP_LOGI(TAG, "cal pot %2d: %u..%u", i,
                                         cal[i].min, cal[i].max);
                                lastSent[i] = 0xFFFF;   // re-arm send-on-move
                            }
                            ESP_LOGI(TAG, "calibration stored (NVS rpots/cal)");
                        }
                    } else {
                        const uint8_t prev = lastValue[ch];
                        const uint8_t v = mapValue(ch, raw);
                        lastValue[ch] = v;
                        if (pickPending[ch]) {
                            // Soft takeover: the pot is not aligned with its
                            // cell's value — send nothing until the position
                            // crosses the value (the renderer shows the
                            // < / > hint). Picking up does not send: the
                            // parameter is already AT the pick target; the
                            // next DEADBAND move writes from there.
                            const int d = (int)v - (int)pickTarget[ch];
                            if (d <= PICK_BAND && d >= -PICK_BAND) {
                                pickPending[ch] = false;
                                lastSent[ch] = v;
                                lastSendPotUs[ch] = now;
                            }
                        } else {
                            const int delta = (int)v - (int)lastSent[ch];
                            if (delta >= DEADBAND || delta <= -DEADBAND) {
                                if (now - lastSendUs >= SEND_PERIOD_US) {
                                    lastSendUs = now;
                                    lastSent[ch] = v;
                                    lastSendPotUs[ch] = now;
                                    if (RackUI::AssignModeActive()) {
                                        // Assign gesture: the pot move cycles
                                        // the cell's mod source; direction =
                                        // the move.
                                        sendAssign(ch, v > prev);
                                    }
                                    RackUI::PotTarget t;
                                    if (RackUI::GetPotTarget(ch, t) &&
                                        t.kind == RackUI::PotTarget::kCC &&
                                        t.routedSrc) {
                                        // Mod-routed cell: the pot RIDES the
                                        // routed source's speed param (user
                                        // spec 2026-11-08); the cell's own CC
                                        // is untouched (the source owns it).
                                        const uint8_t src =
                                            (uint8_t)(t.routedSrc - 1);
                                        RackLink::SendModSet(
                                            src, rideParam(src), v);
                                    } else {
                                        sendTarget(ch, v);
                                    }
                                }
                            }
                        }
                    }

                    ch = (uint8_t)((ch + 1) & 15);

                    // ~700 sweeps/s: 16 x (8 us settle + ~90 us ADC read)
                    // ~= 1.4 ms/sweep; the yield keeps the idle task fed.
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }

            TaskHandle_t hPotTask = nullptr;

        } // namespace

        bool RackPots::BeginCalibration() {
            if (calRunning) return false;
            // Start from the FULL raw range so the sweep can only widen.
            for (int i = 0; i < NPOTS; ++i) {
                cal[i].min = 4095;
                cal[i].max = 0;
            }
            calStartUs = esp_timer_get_time();
            calRunning = true;
            ESP_LOGI(TAG, "calibration window open (%d ms): sweep every pot "
                          "end to end once or twice", CAL_WINDOW_MS);
            return true;
        }

        bool RackPots::CalibrationRunning() { return calRunning; }

        void RackPots::ClearCalibration() {
            for (int i = 0; i < NPOTS; ++i) {
                cal[i].min = 0;
                cal[i].max = 4095;
            }
            calStored = false;
            nvs_handle_t h;
            if (nvs_open("rpots", NVS_READWRITE, &h) == ESP_OK) {
                nvs_erase_key(h, "cal");
                nvs_commit(h);
                nvs_close(h);
            }
            ESP_LOGI(TAG, "calibration cleared (full raw range)");
        }

        int RackPots::DebugRaw(uint8_t pot) {
            if (pot >= NPOTS) return -1;
            addrSet(pot);
            esp_rom_delay_us(8);
            return adc1_get_raw(ADC1_CHANNEL_0);
        }

        int RackPots::DebugValue(uint8_t pot) {
            if (pot >= NPOTS) return -1;
            return lastValue[pot];
        }

        // ---- soft takeover (RackPots.hpp has the policy) ----

        void RackPots::PickupResync() {
            for (uint8_t i = 0; i < NPOTS; ++i) {
                pickPending[i] = false;
                RackUI::PotTarget t;
                if (!RackUI::GetPotTarget(i, t)) continue;   // no cell: live
                if (t.kind == RackUI::PotTarget::kCC && t.routedSrc) {
                    continue;    // ride cell: the pot writes the source, not
                                 // this value — nothing to pick up
                }
                const int d = (int)lastValue[i] - (int)t.value;
                if (d > PICK_BAND || d < -PICK_BAND) {
                    pickPending[i] = true;
                    pickTarget[i] = t.value;
                }
            }
        }

        void RackPots::PickupCellNote(uint8_t pot, uint8_t value) {
            if (pot >= NPOTS) return;
            if (pickPending[pot]) {
                // An external change landing exactly on our position (or a
                // stale pick target the pot has since crossed) — re-arm
                // against the new value; potTask clears on the crossing.
                pickTarget[pot] = value;
                const int d = (int)lastValue[pot] - (int)value;
                if (d <= PICK_BAND && d >= -PICK_BAND) {
                    pickPending[pot] = false;
                    lastSent[pot] = value;
                    lastSendPotUs[pot] = esp_timer_get_time();
                }
                return;
            }
            // Live pot: our own echo (same value right after our send) never
            // arms pending — mid-turn echo lag must not stall a turn.
            if ((int)value == (int)lastSent[pot] &&
                esp_timer_get_time() - lastSendPotUs[pot] < ECHO_WINDOW_US) {
                return;
            }
            RackUI::PotTarget t;
            if (!RackUI::GetPotTarget(pot, t)) return;
            if (t.kind == RackUI::PotTarget::kCC && t.routedSrc) return;
            const int d = (int)lastValue[pot] - (int)value;
            if (d > PICK_BAND || d < -PICK_BAND) {
                pickPending[pot] = true;
                pickTarget[pot] = value;
            }
        }

        bool RackPots::PickupPending(uint8_t pot, int8_t &dir) {
            if (pot >= NPOTS || !pickPending[pot]) return false;
            // Position below the value: turn UP to pick it ('>' right of the
            // value); above: turn DOWN ('<').
            dir = ((int)lastValue[pot] < (int)pickTarget[pot]) ? +1 : -1;
            return true;
        }

        void RackPots::Start() {
            if (hPotTask != nullptr) return;

            // Address lines (outputs; GPIO2 strapping note in RackPots.hpp).
            gpio_config_t io = {};
            io.pin_bit_mask = (1ULL << PIN_S0) | (1ULL << PIN_S1) |
                              (1ULL << PIN_S2) | (1ULL << PIN_S3);
            io.mode = GPIO_MODE_OUTPUT;
            gpio_config(&io);
            addrSet(0);

            // ADC1_CH0, 12 bit, 11 dB atten (full 0..~3.3 V swing).
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

            loadCal();
            for (int i = 0; i < NPOTS; ++i) {
                lastSent[i] = 0xFFFF;          // force a first send on move
                lastValue[i] = 0;
                lastSendPotUs[i] = 0;
                pickPending[i] = false;
                pickTarget[i] = 0;
            }

            if (xTaskCreate(potTask, "rack_pots", 3072, nullptr,
                            tskIDLE_PRIORITY + 3, &hPotTask) != pdPASS) {
                ESP_LOGE(TAG, "could not start pot scan task");
                hPotTask = nullptr;
                return;
            }
            ESP_LOGI(TAG, "HC4067 scan on GPIO0 (ADC1_CH0), addr %d/%d/%d/%d",
                     PIN_S0, PIN_S1, PIN_S2, PIN_S3);
        }

    } // namespace CTRL
} // namespace CTAG
