/***************
Rack UI controller — HC4067 pot scan implementation (see RackPots.hpp).

Send target resolution lives with the page state: RackUI::GetPotTarget()
maps a pot index to the ACTIVE page's cell (CC number, or (src, param) for
MOD pages) — RackPots stays a dumb scanner/sender.
***************/

#include "RackPots.hpp"

#include "RackLink.hpp"
#include "RackUI.hpp"
#include "input/UserInput.hpp"
#include "esp_adc/adc_oneshot.h"
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

            // 2026-11-08 user call: display onto GPIO0/1/2 (the 0-4 header);
            // the mux SIG (common pin, datasheet Z) takes GPIO3 — the last
            // free ADC1 pin (ADC1 = only GPIO0-4, GPIO4 is the UART TX) —
            // and the address lines take the freed GPIO10/20/21. Strapping
            // pins 2 (display DC, floats at reset: no SCK edges, panel
            // idles) and 8 (S3, don't-care in normal boot) carry outputs
            // that are safe there; a pot on SIG (GPIO3, JTAG-source strap)
            // is high-impedance at reset and the ROM pull-up holds the
            // strap (worst case, a pot at min weakly pulls the JTAG-source
            // strap low — USB console + flashing are unaffected, only
            // OpenOCD-over-USB-JTAG would, which we never use).
            constexpr int PIN_ADC = 3;    // ADC1_CH3 (mux SIG, common) —
                                          // also the ENCODER PUSH line (4.7k
                                          // series): pressed pins the pad
                                          // low, detected in the scan task
            constexpr adc_channel_t ADC_CH = ADC_CHANNEL_3;
            // esp_adc oneshot handle (the legacy adc1_get_raw hung on its
            // done-flag poll at task priority — bench-caught 2026-11-08:
            // rack_pots starved CPU 0, the TWDT fired every 5 s and every
            // lower-priority task — link, console, UI — never ran).
            adc_oneshot_unit_handle_t s_adc = nullptr;
            constexpr int PIN_S0 = 8;
            constexpr int PIN_S1 = 9;
            constexpr int PIN_S2 = 2;     // strapping pin — floats harmlessly
                                          // at boot (mux picks an arbitrary
                                          // channel; nothing reads it)
            constexpr int PIN_S3 = 6;     // ex display-CS (display CS is
                                          // now GPIO7; RES = hardware
                                          // pull-up)

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
                bool pushActive = false;
                static uint16_t prevSweep[NPOTS];
                static bool prevValid = false;
                while (true) {
                    // Address settle: the HC4067 switches in ~ns; a short
                    // delay + the ADC's own sampling window is plenty.
                    addrSet(ch);
                    esp_rom_delay_us(8);
                    int raw = 0;
                    // Oneshot read blocks for the conversion (~us) — the
                    // scan rate stays ~700 sweeps/s.
                    adc_oneshot_read(s_adc, ADC_CH, &raw);
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
                                        // the move. The readings during the
                                        // press are 0.6x-scaled by the push's
                                        // divider — valid for DIRECTION,
                                        // never sent as parameter values.
                                        sendAssign(ch, v > prev);
                                    } else if (pushActive) {
                                        // Pressed but not armed: the readings
                                        // are scaled — suppress parameters.
                                    } else {
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
                    }

                    ch = (uint8_t)((ch + 1) & 15);

                    // ENCODER PUSH detection (GPIO3 shared with the SIG line
                    // through the 220R divider): a press scales EVERY
                    // channel's reading to ~0.6x simultaneously (the divider:
                    // pad = SIG x 220/(220+mux-Ron)) while a real pot turn
                    // moves ONE channel — a full-sweep comparison detects it.
                    // pushActive then suppresses parameter sends (scaled
                    // readings are never parameter-safe) while keeping the
                    // assign-direction tracking live.
                    if (ch == 0) {
                        int dropped = 0, alive = 0;
                        for (int i = 0; i < NPOTS; ++i) {
                            if (prevSweep[i] > 40) {
                                ++alive;
                                if (lastValue[i] < prevSweep[i] -
                                                   prevSweep[i] / 4)
                                    ++dropped;
                            }
                            prevSweep[i] = lastValue[i];
                        }
                        prevValid = true;
                        pushActive = alive >= 10 && dropped >= alive * 3 / 4;
                        UserInput::SetPushRaw(pushActive);
                    }

                    // One FULL tick per complete 16-channel sweep. TRAP
                    // (bench-caught 2026-11-08): CONFIG_FREERTOS_HZ=100
                    // makes pdMS_TO_TICKS(1) round to ZERO — vTaskDelay(0)
                    // free-runs this prio-3 task, starving IDLE (the TWDT
                    // fired every 5 s) AND the prio-2 link task (silent
                    // wire, no probes, dead console). The seed's LinkClient
                    // carries the same lesson in its comments. ~2 ms of
                    // scan per 10 ms tick ~= 100 sweeps/s; each pot is
                    // sampled every ~12 ms — well inside the send-on-move
                    // latency budget.
                    if (ch == 0) vTaskDelay(1);
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
            if (pot >= NPOTS || s_adc == nullptr) return -1;
            addrSet(pot);
            esp_rom_delay_us(8);
            int raw = 0;
            adc_oneshot_read(s_adc, ADC_CH, &raw);
            return raw;
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

            // ADC1 oneshot unit, CH3 (mux SIG), 12 bit, 12 dB atten
            // (full 0..~3.3 V swing; DB_12 == the old DB_11 value range).
            adc_oneshot_unit_init_cfg_t unit = {};
            unit.unit_id = ADC_UNIT_1;
            unit.ulp_mode = ADC_ULP_MODE_DISABLE;
            ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));
            adc_oneshot_chan_cfg_t chan = {};
            chan.atten = ADC_ATTEN_DB_12;
            chan.bitwidth = ADC_BITWIDTH_12;
            ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH, &chan));

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
