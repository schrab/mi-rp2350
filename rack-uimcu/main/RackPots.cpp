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
            constexpr int RAW_HYST = 38;           // ~1.2 MIDI steps in 12-bit raw space
            constexpr int PICK_BAND = 2;           // pickup alignment window (0-127)
            constexpr int CAL_WINDOW_MS = 8000;    // `calib` sweep window
            // Own-echo window: any TELEM arriving for a pot within this window
            // after OUR send is in-flight echo traffic from our own turn,
            // never an external change.
            constexpr int64_t ECHO_WINDOW_US = 500000;

            struct PotCal { uint16_t min, max; };
            PotCal cal[NPOTS];                     // RAM copy (full range if none)
            bool calStored = false;                // NVS has a stored blob
            bool calRunning = false;

            uint16_t lastSent[NPOTS];              // last SENT 0-127 value
            int lastSentRaw[NPOTS];                // raw ADC at last send (hysteresis anchor)
            int filtRaw[NPOTS];                    // adaptive EMA-filtered 12-bit raw
            uint16_t lastValue[NPOTS];             // last mapped 0-127 value
            int64_t lastSendPotUs[NPOTS];          // per-pot last SEND time
            int64_t calStartUs = 0;

            // ---- soft takeover state (UI task writes, scan task reads;
            // single-byte fields, benign races cost one scan cycle) ----
            volatile bool pickPending[NPOTS];
            volatile uint8_t pickTarget[NPOTS];    // cell value to align to

            // ---- per-pot modulation assign tracking ----
            uint8_t s_potModSrc[NPOTS];            // 0 = OFF, 1..12 = LFO1..ADS2
            volatile bool s_potWantUnmap[NPOTS];   // true after user sets MOD OFF
            int64_t s_lastAssignUs[NPOTS];         // echo guard for cell flags

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

            // Per-source `dest` param index in rack-rp2350/src/mod_engine.h:
            // LFO(0..3) = 7, EUCLID(4..7) = 5, GATE(8..9) = 5, ADSR(10..11) = 7.
            uint8_t destParam(uint8_t src) {
                if (src <= 3) return 7;
                if (src <= 9) return 5;
                return 7;
            }

            // Linearize the pressed ADC reading (0..~3560 while the 470R push
            // button pulls SIG toward GND) into a 0..127 pot angle so turning
            // a pot while holding the button spans 0..127 evenly.
            uint8_t pressedPosFromRaw(int raw) {
                static const int kKnots[13] = {
                    0, 55, 105, 145, 180, 215, 260, 315, 395, 515, 730, 1200, 3200
                };
                if (raw <= kKnots[0]) return 0;
                if (raw >= kKnots[12]) return 127;
                for (int i = 0; i < 12; ++i) {
                    if (raw <= kKnots[i + 1]) {
                        const int span = kKnots[i + 1] - kKnots[i];
                        const int frac = ((raw - kKnots[i]) * 256) / (span > 0 ? span : 1);
                        const int pos = ((i * 256 + frac) * 127) / (12 * 256);
                        return (uint8_t)(pos < 0 ? 0 : (pos > 127 ? 127 : pos));
                    }
                }
                return 127;
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
                RackUI::ApplyPotAction(pot, t, value);
            }

            void applyModAssign(uint8_t pot, uint8_t modChoice) {
                RackUI::PotTarget t;
                if (!RackUI::GetPotTarget(pot, t) ||
                    t.kind != RackUI::PotTarget::kCC) {
                    return;
                }
                const uint8_t curSrc = t.routedSrc; // 0 = OFF, 1..12 = LFO1..ADS2
                const uint8_t localSrc = s_potModSrc[pot];
                s_lastAssignUs[pot] = esp_timer_get_time();
                RackUI::NoteAssignPot(pot, modChoice);
                RackLink::TelemLocalAssignWrite(pot, modChoice);

                if (modChoice == 0) {
                    s_potWantUnmap[pot] = true;
                    if (localSrc >= 1 && localSrc <= 12) {
                        const uint8_t oldIdx = (uint8_t)(localSrc - 1);
                        RackLink::SendModSet(oldIdx, destParam(oldIdx), 0);
                    }
                    if (curSrc >= 1 && curSrc <= 12 && curSrc != localSrc) {
                        const uint8_t prevIdx = (uint8_t)(curSrc - 1);
                        RackLink::SendModSet(prevIdx, destParam(prevIdx), 0);
                    }
                    s_potModSrc[pot] = 0;
                    RackLink::SendPageRequest(RackUI::GetPage());
                    return;
                }

                s_potWantUnmap[pot] = false;
                if (modChoice == localSrc && modChoice == curSrc) return;

                if (localSrc >= 1 && localSrc <= 12 && localSrc != modChoice) {
                    const uint8_t oldIdx = (uint8_t)(localSrc - 1);
                    RackLink::SendModSet(oldIdx, destParam(oldIdx), 0);
                }
                if (curSrc >= 1 && curSrc <= 12 && curSrc != modChoice &&
                    curSrc != localSrc) {
                    const uint8_t prevIdx = (uint8_t)(curSrc - 1);
                    RackLink::SendModSet(prevIdx, destParam(prevIdx), 0);
                }
                s_potModSrc[pot] = modChoice;
                const uint8_t newIdx = (uint8_t)(modChoice - 1);
                RackLink::SendModSet(newIdx, destParam(newIdx), t.cc);
                RackLink::SendPageRequest(RackUI::GetPage());
            }

            int readChannelRaw(uint8_t ch) {
                addrSet(ch);
                esp_rom_delay_us(25);
                // 2 dummy reads charge the SAR S/H cap to the new mux channel;
                // then 10 reads with min/max outlier rejection (trimmed mean of
                // middle 8) reject BLE RF / tick-wakeup spikes on ch=0.
                int dummy = 0;
                adc_oneshot_read(s_adc, ADC_CH, &dummy);
                adc_oneshot_read(s_adc, ADC_CH, &dummy);
                int acc = 0;
                int minR = 4096, maxR = -1;
                for (int s = 0; s < 10; ++s) {
                    int r = 0;
                    adc_oneshot_read(s_adc, ADC_CH, &r);
                    acc += r;
                    if (r < minR) minR = r;
                    if (r > maxR) maxR = r;
                }
                acc -= (minR + maxR);
                return (acc + 4) >> 3;
            }

            void potTask(void *) {
                bool pushActive = false;
                static int baselineRaw[NPOTS];
                static int assignAnchorRaw[NPOTS];
                static int assignFiltRaw[NPOTS];
                static uint8_t assignAnchorPos[NPOTS];
                static bool prevValid = false;
                static bool assignSessionArmed = false;
                static int64_t assignArmTimeUs = 0;
                static int64_t lastPushUs = 0;
                static int8_t assignPotLocked = -1;
                static int8_t assignCandidatePot = -1;
                static uint8_t assignCandidateCount = 0;
                static uint8_t assignLastChoice = 0xFF;
                static uint8_t assignStableChoice = 0xFF;
                static uint8_t assignStableCount = 0;
                static uint16_t assignTurnedMask = 0;

                constexpr int64_t POST_RELEASE_GUARD_US = 250000; // 250 ms

                while (true) {
                    // Warm up SAR ADC reference after 10 ms tick sleep
                    (void)readChannelRaw(0);

                    int rawSweep[NPOTS];
                    for (uint8_t ch = 0; ch < NPOTS; ++ch) {
                        rawSweep[ch] = readChannelRaw(ch);
                    }
                    const int64_t now = esp_timer_get_time();

                    if (!prevValid) {
                        for (int i = 0; i < NPOTS; ++i) {
                            baselineRaw[i] = rawSweep[i];
                            filtRaw[i] = rawSweep[i];
                            lastSentRaw[i] = rawSweep[i];
                        }
                        prevValid = true;
                    }

                    // 1) Detect encoder push BEFORE touching filtRaw[0..15]:
                    //    Pressing the 470R button on GPIO3 (SIG) pulls all
                    //    non-zero mux channels down simultaneously.
                    int alive = 0, dropped = 0;
                    for (int i = 0; i < NPOTS; ++i) {
                        if (baselineRaw[i] > 200) {
                            ++alive;
                            const int drop = baselineRaw[i] - rawSweep[i];
                            if (drop > 80 && rawSweep[i] < baselineRaw[i] - (baselineRaw[i] / 8)) {
                                ++dropped;
                            }
                        }
                    }
                    const bool pushDetected =
                        (alive >= 2 && dropped >= 2 && dropped * 2 >= alive) ||
                        (alive == 1 && dropped == 1 &&
                         (baselineRaw[0] - rawSweep[0] > 300 ||
                          rawSweep[0] < baselineRaw[0] / 2));

                    if (pushDetected) {
                        pushActive = true;
                        lastPushUs = now;
                        UserInput::SetPushRaw(true);

                        // While button is held, NEVER update filtRaw[] or send
                        // normal pot values. Only run modulation assignment
                        // once AssignModeActive() has been armed + settled.
                        if (RackUI::AssignModeActive() && RackUI::GetPage() <= 1) {
                            if (!assignSessionArmed) {
                                if (assignArmTimeUs == 0) {
                                    assignArmTimeUs = now;
                                } else if (now - assignArmTimeUs >= 60000) {
                                    for (int i = 0; i < NPOTS; ++i) {
                                        assignAnchorRaw[i] = rawSweep[i];
                                        assignFiltRaw[i] = rawSweep[i];
                                        assignAnchorPos[i] = pressedPosFromRaw(rawSweep[i]);
                                        RackUI::PotTarget t;
                                        if (RackUI::GetPotTarget((uint8_t)i, t) &&
                                            t.kind == RackUI::PotTarget::kCC) {
                                            s_potModSrc[i] = t.routedSrc;
                                        }
                                    }
                                    assignPotLocked = -1;
                                    assignCandidatePot = -1;
                                    assignCandidateCount = 0;
                                    assignLastChoice = 0xFF;
                                    assignStableChoice = 0xFF;
                                    assignStableCount = 0;
                                    assignSessionArmed = true;
                                }
                            } else {
                                // Guard against mid-sweep button release: if
                                // the other 15 pots are rising together or any
                                // alive pot is no longer dropped, the button
                                // contact is opening — freeze assign state!
                                int otherRiseSum = 0;
                                for (int i = 0; i < NPOTS; ++i) {
                                    if (i == assignPotLocked) continue;
                                    const int diff = rawSweep[i] - assignFiltRaw[i];
                                    if (diff > 0) otherRiseSum += diff;
                                }
                                const bool releasingNow =
                                    (dropped < alive) || (otherRiseSum > 140);

                                if (!releasingNow) {
                                    for (int i = 0; i < NPOTS; ++i) {
                                        assignFiltRaw[i] =
                                            (assignFiltRaw[i] * 3 + rawSweep[i] + 2) >> 2;
                                    }
                                    if (assignPotLocked < 0) {
                                        // Find if a SINGLE pot is being deliberately turned
                                        int movingCount = 0;
                                        int bestPot = -1;
                                        int bestPosDelta = 0;
                                        int ccwUnmapPot = -1;
                                        for (int i = 0; i < NPOTS; ++i) {
                                            const int curPos =
                                                (int)pressedPosFromRaw(assignFiltRaw[i]);
                                            int dPos = curPos - (int)assignAnchorPos[i];
                                            if (dPos < 0) dPos = -dPos;
                                            int dRaw = assignFiltRaw[i] - assignAnchorRaw[i];
                                            if (dRaw < 0) dRaw = -dRaw;
                                            if (dPos >= 8 && dRaw >= 20) {
                                                ++movingCount;
                                                if (dPos > bestPosDelta) {
                                                    bestPosDelta = dPos;
                                                    bestPot = i;
                                                }
                                            }
                                            // Also allow easy CCW unmap when a
                                            // modulated pot was already near CCW:
                                            RackUI::PotTarget t;
                                            if (RackUI::GetPotTarget((uint8_t)i, t) &&
                                                t.kind == RackUI::PotTarget::kCC &&
                                                (t.routedSrc > 0 || s_potModSrc[i] > 0)) {
                                                if (assignFiltRaw[i] <= 115 &&
                                                    (assignFiltRaw[i] <= assignAnchorRaw[i] - 10 ||
                                                     assignFiltRaw[i] <= 45)) {
                                                    ccwUnmapPot = i;
                                                }
                                            }
                                        }
                                        int candidate = -1;
                                        if (movingCount == 1 && bestPot >= 0 &&
                                            bestPosDelta >= 10) {
                                            candidate = bestPot;
                                        } else if (movingCount == 0 && ccwUnmapPot >= 0) {
                                            candidate = ccwUnmapPot;
                                        }
                                        if (candidate >= 0) {
                                            if (assignCandidatePot == candidate) {
                                                if (++assignCandidateCount >= 2) {
                                                    assignPotLocked = (int8_t)candidate;
                                                    assignTurnedMask |=
                                                        (uint16_t)(1u << candidate);
                                                }
                                            } else {
                                                assignCandidatePot = (int8_t)candidate;
                                                assignCandidateCount = 1;
                                            }
                                        } else {
                                            assignCandidateCount = 0;
                                        }
                                    }
                                    if (assignPotLocked >= 0) {
                                        const uint8_t p = (uint8_t)assignPotLocked;
                                        const int r = assignFiltRaw[p];
                                        const uint8_t pos = pressedPosFromRaw(r);
                                        // Generous CCW OFF zone with hysteresis:
                                        // enter OFF at raw <= 115 (pos <= 18),
                                        // stay OFF until raw > 140 (pos > 22).
                                        uint8_t choice = 0;
                                        const bool stayOff =
                                            (assignLastChoice == 0) && (r <= 140);
                                        if (!stayOff && r > 115 && pos > 18) {
                                            choice = (uint8_t)(
                                                1 + ((int)(pos - 19) * 12) / 109);
                                            if (choice > 12) choice = 12;
                                        }
                                        // Update top bar live so user sees the
                                        // source name immediately while turning;
                                        // commit to RP2350 once stable for 3 sweeps.
                                        RackUI::NoteAssignPot(p, choice);
                                        if (choice == assignStableChoice) {
                                            if (assignStableCount < 10) {
                                                ++assignStableCount;
                                            }
                                            if (assignStableCount >= 3 &&
                                                choice != assignLastChoice) {
                                                assignLastChoice = choice;
                                                applyModAssign(p, choice);
                                            }
                                        } else {
                                            assignStableChoice = choice;
                                            assignStableCount = 1;
                                        }
                                    }
                                }
                            }
                        }
                        vTaskDelay(1);
                        continue;
                    }

                    // Button is not pressed
                    if (pushActive) {
                        pushActive = false;
                        UserInput::SetPushRaw(false);
                    }
                    assignSessionArmed = false;
                    assignArmTimeUs = 0;
                    assignPotLocked = -1;
                    assignCandidatePot = -1;
                    assignCandidateCount = 0;
                    assignLastChoice = 0xFF;
                    assignStableChoice = 0xFF;
                    assignStableCount = 0;

                    // Post-release guard: for 250 ms after releasing the button,
                    // re-seed filtRaw/lastSentRaw to the unpressed voltage and
                    // suppress all sends so release recovery never triggers
                    // bogus parameter or modulation changes.
                    if (lastPushUs != 0 && (now - lastPushUs) < POST_RELEASE_GUARD_US) {
                        for (uint8_t ch = 0; ch < NPOTS; ++ch) {
                            filtRaw[ch] = rawSweep[ch];
                            lastSentRaw[ch] = rawSweep[ch];
                            baselineRaw[ch] = rawSweep[ch];
                            lastValue[ch] = mapValue(ch, rawSweep[ch]);
                            lastSent[ch] = lastValue[ch];
                        }
                        if (assignTurnedMask != 0) {
                            assignTurnedMask = 0;
                            RackPots::PickupResync();
                        }
                        vTaskDelay(1);
                        continue;
                    }

                    // Normal unpressed pot processing across all 16 channels
                    for (uint8_t ch = 0; ch < NPOTS; ++ch) {
                        const int raw = rawSweep[ch];
                        const int diff = raw - filtRaw[ch];
                        if (diff > 96 || diff < -96) {
                            filtRaw[ch] = (filtRaw[ch] + raw) >> 1;
                        } else {
                            filtRaw[ch] = (filtRaw[ch] * 7 + raw + 4) >> 3;
                        }
                        baselineRaw[ch] = filtRaw[ch];

                        if (calRunning) {
                            if (filtRaw[ch] < (int)cal[ch].min) cal[ch].min = (uint16_t)filtRaw[ch];
                            if (filtRaw[ch] > (int)cal[ch].max) cal[ch].max = (uint16_t)filtRaw[ch];
                            if (now - calStartUs >= CAL_WINDOW_MS * 1000) {
                                calRunning = false;
                                saveCal();
                                for (int i = 0; i < NPOTS; ++i) {
                                    ESP_LOGI(TAG, "cal pot %2d: %u..%u", i,
                                             cal[i].min, cal[i].max);
                                    lastSent[i] = 0xFFFF;
                                }
                                ESP_LOGI(TAG, "calibration stored (NVS rpots/cal)");
                            }
                        } else {
                            const uint8_t prev = (uint8_t)lastValue[ch];
                            const uint8_t v = mapValue(ch, filtRaw[ch]);
                            if (pickPending[ch]) {
                                lastValue[ch] = v;
                                const int target = (int)pickTarget[ch];
                                const int d = (int)v - target;
                                const bool crossed =
                                    prevValid &&
                                    (((int)prev < target && (int)v >= target) ||
                                     ((int)prev > target && (int)v <= target));
                                if (crossed || (d <= PICK_BAND && d >= -PICK_BAND)) {
                                    pickPending[ch] = false;
                                    lastSent[ch] = (uint8_t)target;
                                    lastSentRaw[ch] = filtRaw[ch];
                                    lastValue[ch] = (uint8_t)target;
                                    lastSendPotUs[ch] = now;
                                    RackUI::NoteActivePot(ch);
                                }
                            }
                            if (!pickPending[ch]) {
                                const int rawDelta = filtRaw[ch] - lastSentRaw[ch];
                                const bool atEnd =
                                    (v == 0 || v == 127) && (v != lastSent[ch]);
                                const bool movedEnough =
                                    atEnd || rawDelta >= RAW_HYST ||
                                    rawDelta <= -RAW_HYST;
                                if (movedEnough && v != lastSent[ch]) {
                                    lastValue[ch] = v;
                                    lastSent[ch] = v;
                                    lastSentRaw[ch] = filtRaw[ch];
                                    lastSendPotUs[ch] = now;
                                    RackUI::PotTarget t;
                                    if (RackUI::GetPotTarget(ch, t) &&
                                        t.kind == RackUI::PotTarget::kCC &&
                                        t.routedSrc) {
                                        // Mod-routed cell: ride the routed
                                        // source's speed param, and show the
                                        // source name in the top bar.
                                        RackUI::NoteActivePot(ch);
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

                    vTaskDelay(1);
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
            return filtRaw[pot];
        }

        int RackPots::DebugValue(uint8_t pot) {
            if (pot >= NPOTS) return -1;
            return lastValue[pot];
        }

        // ---- soft takeover (RackPots.hpp has the policy) ----

        void RackPots::PickupResync() {
            const int64_t now = esp_timer_get_time();
            for (uint8_t i = 0; i < NPOTS; ++i) {
                pickPending[i] = false;
                s_potWantUnmap[i] = false;
                if (lastSendPotUs[i] != 0 &&
                    now - lastSendPotUs[i] < ECHO_WINDOW_US) {
                    continue;    // pot is actively being turned right now
                }
                RackUI::PotTarget t;
                if (!RackUI::GetPotTarget(i, t)) continue;   // no cell: live
                if (t.kind == RackUI::PotTarget::kCC) {
                    s_potModSrc[i] = t.routedSrc;
                    if (t.routedSrc) {
                        continue; // ride cell: pot writes the mod source speed
                    }
                }
                uint8_t bLo = t.value, bHi = t.value;
                RackUI::GetPotBucketBounds(i, t, bLo, bHi);
                const int cur = (int)lastValue[i];
                if (cur < (int)bLo - PICK_BAND) {
                    pickPending[i] = true;
                    pickTarget[i] = bLo;
                } else if (cur > (int)bHi + PICK_BAND) {
                    pickPending[i] = true;
                    pickTarget[i] = bHi;
                } else {
                    lastSent[i] = lastValue[i];
                    lastSentRaw[i] = filtRaw[i];
                }
            }
        }

        void RackPots::PickupCellNote(uint8_t pot, uint8_t value) {
            if (pot >= NPOTS) return;
            RackUI::PotTarget t;
            if (!RackUI::GetPotTarget(pot, t)) return;
            t.value = value;
            if (t.kind == RackUI::PotTarget::kCC) {
                if (!AssignRecentlyChanged(pot)) {
                    s_potModSrc[pot] = t.routedSrc;
                }
                if (t.routedSrc) return;
            }
            uint8_t bLo = value, bHi = value;
            RackUI::GetPotBucketBounds(pot, t, bLo, bHi);
            const int cur = (int)lastValue[pot];

            if (pickPending[pot]) {
                if (cur >= (int)bLo - PICK_BAND && cur <= (int)bHi + PICK_BAND) {
                    pickPending[pot] = false;
                    lastSent[pot] = lastValue[pot];
                    lastSentRaw[pot] = filtRaw[pot];
                    lastSendPotUs[pot] = esp_timer_get_time();
                } else {
                    pickTarget[pot] = (cur < (int)bLo) ? bLo : bHi;
                }
                return;
            }
            if (lastSendPotUs[pot] != 0 &&
                esp_timer_get_time() - lastSendPotUs[pot] < ECHO_WINDOW_US) {
                return;
            }
            if (cur < (int)bLo - PICK_BAND) {
                pickPending[pot] = true;
                pickTarget[pot] = bLo;
            } else if (cur > (int)bHi + PICK_BAND) {
                pickPending[pot] = true;
                pickTarget[pot] = bHi;
            }
        }

        bool RackPots::RecentlyMoved(uint8_t pot) {
            if (pot >= NPOTS || lastSendPotUs[pot] == 0) return false;
            return (esp_timer_get_time() - lastSendPotUs[pot]) < ECHO_WINDOW_US;
        }

        bool RackPots::AssignRecentlyChanged(uint8_t pot) {
            if (pot >= NPOTS || s_lastAssignUs[pot] == 0) return false;
            return (esp_timer_get_time() - s_lastAssignUs[pot]) < ECHO_WINDOW_US;
        }

        void RackPots::ClearResidualUnmap(uint8_t pot, uint8_t routedSrc) {
            if (pot >= NPOTS || !s_potWantUnmap[pot] || routedSrc == 0 ||
                routedSrc > 12) {
                return;
            }
            const uint8_t srcIdx = (uint8_t)(routedSrc - 1);
            RackLink::SendModSet(srcIdx, destParam(srcIdx), 0);
            RackLink::TelemLocalAssignWrite(pot, 0);
            RackLink::SendPageRequest(RackUI::GetPage());
        }

        void RackPots::ClearAllModSources() {
            for (uint8_t s = 0; s < 12; ++s) {
                RackLink::SendModSet(s, destParam(s), 0);
            }
            for (uint8_t i = 0; i < NPOTS; ++i) {
                s_potModSrc[i] = 0;
                s_potWantUnmap[i] = false;
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
                const int r = readChannelRaw((uint8_t)i);
                filtRaw[i] = r;
                lastSentRaw[i] = r;
                lastValue[i] = mapValue((uint8_t)i, r);
                lastSent[i] = lastValue[i];
                lastSendPotUs[i] = 0;
                pickPending[i] = false;
                pickTarget[i] = 0;
            }
            addrSet(0);

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
