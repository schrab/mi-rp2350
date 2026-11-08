/***************
Rack UI controller — T5 stage 1 link layer implementation. Mirrors the seed
LinkClient (LinkClient.cpp) structure: 16 KB rx driver ring, dedicated rx
task, one FULL tick per loop pass (vTaskDelay(1) — pdMS_TO_TICKS(5) rounds
to ZERO at FREERTOS_HZ=100 and busy-spins), offline watchdog) but with the
rack binary framing (see RackLink.hpp: MUST MATCH docs/ui-protocol.md and
rack-rp2350/src/ui_link.h).

Boot self-test (once, ESP_LOGI to the USB-JTAG console):
  (a) in-memory codec round-trip incl. corrupted-CRC rejection;
  (b) hardware loopback via uart_set_loop_back(): 32 frames incl. one
      max-len frame, all verified, RTT estimate in µs (esp_timer).
      One line: `selftest: codec OK | hw-loop 32/32 | rtt ~NN us`.

Live behavior (stage 1):
  - TELEM received -> log page/module/cell count + the first cell values.
  - UI_CMD echo request received -> echo back verbatim + RTT print.
  - MIDI/SWITCH/MOD_SET/CHAIN_ROUTE received -> one-line log (the C3 mainly
    SENDS these; receiving them proves the wire both ways).
  - While the link is NOT confirmed (no verified frame within 3 s), a P1
    page request is sent every 5 s to exercise the wire when it comes up;
    every send is logged.
***************/

#include "RackLink.hpp"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>

namespace CTAG {
    namespace CTRL {

        namespace {

            constexpr uart_port_t kUart = UART_NUM_0;
            constexpr int kLinkBaud = 921600;
            // C3 Super Mini wiring (3V3 logic, crossed with the RP2350):
            //   C3 TX = GPIO4 -> RP2350 RX = GP17
            //   C3 RX = GPIO5 <- RP2350 TX = GP16
            //   GND <-> GND
            // (RP2350-side validity verified against the earlephilhower
            // core's RP2350A SerialUART pin tables; GP16 is UART0-TX
            // capable, GP17 UART0-RX capable.)
            constexpr int kPinTx = 4;
            constexpr int kPinRx = 5;
            // RX ring must ride out burst traffic while the task drains
            // continuously (same sizing rationale as the seed LinkClient).
            constexpr size_t kRxRing = 16384;
            constexpr size_t kTxRing = 8192;
            constexpr uint32_t kOfflineAfterMs = 3000;
            constexpr uint32_t kProbePeriodMs = 5000;

            constexpr size_t kMaxFrame = 259;   // 1 + 1 + 1 + 255 + 1

            const char *TAG = "RackLink";

            TaskHandle_t hLinkTask = nullptr;
            volatile bool s_online = false;
            volatile uint32_t s_crc_errors = 0;
            volatile uint32_t s_rx_frames = 0;
            volatile uint32_t s_tx_frames = 0;
            int64_t s_probe_time_us = 0;        // last page-request send (RTT)
            volatile bool s_probe_pending = false;
            int64_t s_last_rx_us = 0;

            // ---- TELEM cache (renderer side; link task writes, UI task
            // reads — single core, guarded by a critical section) ----
            portMUX_TYPE telemMux = portMUX_INITIALIZER_UNLOCKED;
            RackTelemPage s_telem = {};
            volatile bool s_telem_dirty = false;
            volatile uint32_t s_last_midi_ms = 0;

            // ---- codec (byte-exact duplicate of ui_link.h) ----
            uint8_t RackLinkCrc8(const uint8_t *data, size_t n) {
                uint8_t crc = 0x00;
                for (size_t i = 0; i < n; i++) {
                    crc ^= data[i];
                    for (int b = 0; b < 8; b++) {
                        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                                           : (uint8_t)(crc << 1);
                    }
                }
                return crc;
            }

            // Build one frame; returns the frame length (len + 4).
            size_t RackLinkBuildFrame(uint8_t type, const uint8_t *payload,
                                      uint8_t len, uint8_t *out) {
                out[0] = RackLink::kFrameMark;
                out[1] = len;
                out[2] = type;
                if (len) memcpy(&out[3], payload, len);
                out[3 + len] = RackLinkCrc8(&out[2], (size_t)len + 1);
                return (size_t)len + 4;
            }

            // Frame parser state machine — mirrors ui_link.h's ui_link_step:
            // hunt 0x7E -> len -> type+payload -> crc verify; on CRC failure
            // drop and resync at the next 0x7E.
            struct RackParser {
                uint8_t state = 0;   // 0 hunt / 1 len / 2 body / 3 crc
                uint8_t flen = 0;
                // uint16_t: a len=255 frame body is 256 bytes — a uint8_t
                // counter wraps and the parser stalls mid-frame (same bug the
                // RP2350 bench caught in ui_link.h 2026-11-08).
                uint16_t idx = 0;
                uint8_t buf[256];    // [0] = type, [1..flen] = payload
                bool crc_ok = false;

                // Feed one byte; true when a complete frame is assembled.
                bool feed(uint8_t b) {
                    switch (state) {
                        case 0:
                            if (b == RackLink::kFrameMark) state = 1;
                            return false;
                        case 1:
                            flen = b;
                            idx = 0;
                            state = 2;
                            return false;
                        case 2:
                            buf[idx++] = b;
                            if (idx >= (uint16_t)flen + 1) state = 3;
                            return false;
                        default:
                            crc_ok = (RackLinkCrc8(buf, (size_t)flen + 1) == b);
                            state = 0;
                            return true;
                    }
                }
            };

            void sendFrame(uint8_t type, const uint8_t *payload, uint8_t len) {
                uint8_t frame[kMaxFrame];
                const size_t n = RackLinkBuildFrame(type, payload, len, frame);
                uart_write_bytes(kUart, frame, n);
                s_tx_frames++;
            }

            uint32_t nowMs() {
                return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            }

            // Probe bookkeeping: while the link is NOT confirmed, a P1 page
            // request goes out every 5 s (logged); the RP2350's full TELEM
            // reply both exercises the wire and yields the RTT estimate.
            uint32_t s_last_probe_ms = 0;
            uint32_t lastProbeMs() { return s_last_probe_ms; }
            void probeNow() {
                s_last_probe_ms = nowMs();
                s_probe_pending = true;
                s_probe_time_us = esp_timer_get_time();
                const uint8_t p[3] = { RackLink::kCmdPageRequest,
                                       RackLink::kPageP1, 0 };
                sendFrame(RackLink::kTypeUiCmd, p, 3);
                ESP_LOGI(TAG, "probe: P1 page request sent (link not "
                              "confirmed)");
            }

            // ---- RX dispatch ----
            // TELEM v2 parse into the cache (see RackLink.hpp / docs §5):
            //   [page][module][mode][count]
            //   [count x (cc, value, flags, color)]   4-byte cells
            //   [count x NUL-terminated labels]
            //   mode 0 only: [bpm][clocksrc][chain][link]
            // Mode 1 (delta) = the same format with count=1: patches ONE
            // cell of the cached page, matched by cc (ignored when the page
            // differs or the cc is unknown — the next page request heals).
            void parseTelem(const uint8_t *pl, uint8_t len) {
                if (len < 4) {
                    ESP_LOGW(TAG, "telem short (len=%u)", len);
                    return;
                }
                const uint8_t page = pl[0], mod = pl[1], mode = pl[2],
                               count = pl[3];
                if (count > 16 || count == 0) {
                    ESP_LOGW(TAG, "telem count %u invalid", count);
                    return;
                }
                const size_t cells_end = 4 + (size_t)count * 4;
                if (len < cells_end) {
                    ESP_LOGW(TAG, "telem truncated cells (len=%u)", len);
                    return;
                }
                taskENTER_CRITICAL(&telemMux);
                if (mode == RackLink::kTelemDelta) {
                    // Delta: patch one cell of the cached page.
                    if (!s_telem.valid || s_telem.page != page) {
                        taskEXIT_CRITICAL(&telemMux);
                        ESP_LOGW(TAG, "telem delta for page %u with no "
                                      "cached snapshot", page);
                        return;
                    }
                    const uint8_t cc = pl[4];
                    int8_t idx = -1;
                    for (uint8_t i = 0; i < s_telem.count; i++) {
                        if (s_telem.cell[i].cc == cc) { idx = (int8_t)i; break; }
                    }
                    if (idx < 0) {
                        taskEXIT_CRITICAL(&telemMux);
                        ESP_LOGW(TAG, "telem delta cc=%u not on cached page %u",
                                 cc, page);
                        return;
                    }
                    RackTelemCell &c = s_telem.cell[idx];
                    c.value = pl[5];
                    c.flags = pl[6];
                    c.color = pl[7];
                    size_t off = cells_end;
                    size_t n = 0;
                    while (off + n < len && pl[off + n] != 0 && n < 15) n++;
                    if (off + n < len) {
                        memcpy(c.label, &pl[off], n);
                        c.label[n] = '\0';
                    }
                    s_telem.module = mod;
                    taskEXIT_CRITICAL(&telemMux);
                    s_telem_dirty = true;
                    return;
                }
                // Full snapshot: replace the cache wholesale.
                s_telem.page = page;
                s_telem.module = mod;
                s_telem.mode = mode;
                s_telem.count = count;
                for (uint8_t i = 0; i < count; i++) {
                    RackTelemCell &c = s_telem.cell[i];
                    c.cc = pl[4 + i * 4];
                    c.value = pl[5 + i * 4];
                    c.flags = pl[6 + i * 4];
                    c.color = pl[7 + i * 4];
                }
                size_t off = cells_end;
                for (uint8_t i = 0; i < count && off < len; i++) {
                    RackTelemCell &c = s_telem.cell[i];
                    size_t n = 0;
                    while (off + n < len && pl[off + n] != 0 && n < 15) n++;
                    if (off + n < len) {           // found the NUL
                        memcpy(c.label, &pl[off], n);
                        c.label[n] = '\0';
                        off += n + 1;
                    } else {
                        break;                     // truncated label block
                    }
                }
                s_telem.valid = true;
                // Status tail: the 4 bytes after the label block.
                if (len >= off + 4) {
                    s_telem.bpm = pl[off];
                    s_telem.clocksrc = pl[off + 1];
                    s_telem.chain = pl[off + 2];
                    s_telem.link = pl[off + 3];
                    s_telem.tail_valid = true;
                }
                taskEXIT_CRITICAL(&telemMux);
                s_telem_dirty = true;
            }


            void dispatchFrame(RackParser &p) {
                const uint8_t type = p.buf[0];
                const uint8_t *pl = &p.buf[1];
                const uint8_t len = p.flen;
                s_rx_frames++;
                s_last_rx_us = esp_timer_get_time();
                if (!s_online) ESP_LOGI(TAG, "rack link online");
                s_online = true;

                switch (type) {
                    case RackLink::kTypeTelem: {
                        // v2 payload -> the renderer's cache (parseTelem).
                        // Full snapshots log one line (bring-up evidence);
                        // the delta stream would spam at <= 20 Hz and logs
                        // nothing.
                        const bool is_delta =
                            len >= 4 && pl[2] == RackLink::kTelemDelta;
                        if (s_probe_pending && !is_delta) {
                            s_probe_pending = false;
                            const int64_t rtt_us =
                                esp_timer_get_time() - s_probe_time_us;
                            ESP_LOGI(TAG, "page request RTT ~%lld us",
                                     (long long)rtt_us);
                        }
                        if (!is_delta) {
                            ESP_LOGI(TAG, "telem page=%u mod=%u mode=%u "
                                          "count=%u",
                                     pl[0], pl[1], pl[2],
                                     len >= 4 ? pl[3] : 0);
                        }
                        parseTelem(pl, len);
                        break;
                    }
                    case RackLink::kTypeUiCmd: {
                        // The RP2350 may echo our probes back (action 2) —
                        // reply verbatim + print the turnaround.
                        if (len == 3 && pl[0] == RackLink::kCmdEcho) {
                            const int64_t t0 = esp_timer_get_time();
                            sendFrame(RackLink::kTypeUiCmd, pl, 3);
                            const int64_t dt = esp_timer_get_time() - t0;
                            ESP_LOGI(TAG, "echo req seq=%u echoed back "
                                          "(turnaround ~%lld us)",
                                     (uint16_t)((pl[1] << 8) | pl[2]),
                                     (long long)dt);
                        } else {
                            ESP_LOGI(TAG, "UI_CMD received len=%u", len);
                        }
                        break;
                    }
                    case RackLink::kTypeMidi:
                        ESP_LOGI(TAG, "MIDI frame received [%02x %02x %02x %02x]"
                                       " (the C3 mainly sends these)",
                                 len == 4 ? pl[0] : 0, len == 4 ? pl[1] : 0,
                                 len == 4 ? pl[2] : 0, len == 4 ? pl[3] : 0);
                        break;
                    case RackLink::kTypeSwitch:
                        ESP_LOGI(TAG, "SWITCH received mod=%u", len ? pl[0] : 0);
                        break;
                    case RackLink::kTypeModSet:
                        ESP_LOGI(TAG, "MOD_SET received src=%u param=%u val=%u",
                                 len == 3 ? pl[0] : 0, len == 3 ? pl[1] : 0,
                                 len == 3 ? pl[2] : 0);
                        break;
                    case RackLink::kTypeChainRoute:
                        ESP_LOGI(TAG, "CHAIN_ROUTE received a=%u b=%u "
                                      "engage=%u",
                                 len == 3 ? pl[0] : 0, len == 3 ? pl[1] : 0,
                                 len == 3 ? pl[2] : 0);
                        break;
                    case RackLink::kTypePresetReq:
                    case RackLink::kTypePresetPart:
                    case RackLink::kTypePresetApply:
                        ESP_LOGI(TAG, "PRESET frame 0x%02x len=%u (T4a)", type,
                                 len);
                        break;
                    default:
                        ESP_LOGW(TAG, "unknown frame type 0x%02x len=%u", type,
                                 len);
                        break;
                    }
            }

            // ---- rx task (the seed linkTask's shape) ----
            void linkTask(void *) {
                RackParser parser;
                uint8_t rx[256];

                while (true) {
                    const int got = uart_read_bytes(kUart, rx, sizeof(rx), 0);
                    if (got > 0) {
                        for (int i = 0; i < got; i++) {
                            if (parser.feed((uint8_t)rx[i])) {
                                if (parser.crc_ok) {
                                    dispatchFrame(parser);
                                } else {
                                    s_crc_errors++;
                                    ESP_LOGW(TAG, "CRC fail (type=0x%02x "
                                                  "len=%u) - dropped, "
                                                  "resyncing",
                                             parser.buf[0], parser.flen);
                                }
                            }
                        }
                    }

                    // Offline watchdog + unconfirmed-link probe: while the
                    // cross-board link is NOT confirmed, send a P1 page
                    // request every 5 s (logged) — the RP2350's full TELEM
                    // reply is the wire exercise + the RTT measurement.
                    const uint32_t now = nowMs();
                    const bool online_now =
                        s_online &&
                        (esp_timer_get_time() - s_last_rx_us) <
                            (int64_t)kOfflineAfterMs * 1000;
                    if (online_now != s_online) {
                        if (!online_now)
                            ESP_LOGW(TAG, "rack link went quiet - offline");
                        s_online = online_now;
                    }
                    if (!s_online && now - lastProbeMs() >= kProbePeriodMs) {
                        probeNow();
                    }

                    // One FULL tick — pdMS_TO_TICKS(5) rounds to zero at
                    // FREERTOS_HZ=100 and busy-spins (see A1S LinkEndpoint).
                    vTaskDelay(1);
                }
            }

            // ---- boot self-test ----
            bool selftestCodec() {
                RackParser p;
                uint8_t frame[kMaxFrame];
                uint8_t payload[255];
                int fails = 0;

                // 1) UI_CMD echo frame round-trip.
                {
                    const uint8_t pl[3] = { RackLink::kCmdEcho, 0x12, 0x34 };
                    const size_t n = RackLinkBuildFrame(
                        RackLink::kTypeUiCmd, pl, 3, frame);
                    bool completed = false;
                    for (size_t i = 0; i < n; i++)
                        completed |= p.feed(frame[i]);
                    if (!(completed && p.crc_ok && p.flen == 3 &&
                          p.buf[0] == RackLink::kTypeUiCmd &&
                          p.buf[1] == RackLink::kCmdEcho &&
                          p.buf[2] == 0x12 && p.buf[3] == 0x34))
                        fails++;
                }
                // 2) MIDI frame round-trip (4-byte packet).
                {
                    const uint8_t pl[4] = { 0x09, 0x90, 0x3C, 0x64 };
                    const size_t n = RackLinkBuildFrame(
                        RackLink::kTypeMidi, pl, 4, frame);
                    bool completed = false;
                    for (size_t i = 0; i < n; i++)
                        completed |= p.feed(frame[i]);
                    if (!(completed && p.crc_ok && p.flen == 4 &&
                          p.buf[0] == RackLink::kTypeMidi && p.buf[4] == 0x64))
                        fails++;
                }
                // 3) 0x7E inside a payload (no byte stuffing).
                {
                    const uint8_t pl[3] = { 0x7E, 0x7E, 0x55 };
                    const size_t n = RackLinkBuildFrame(0x40, pl, 3, frame);
                    bool completed = false;
                    for (size_t i = 0; i < n; i++)
                        completed |= p.feed(frame[i]);
                    if (!(completed && p.crc_ok && p.buf[1] == 0x7E &&
                          p.buf[2] == 0x7E))
                        fails++;
                }
                // 4) Max-length frame (255-byte payload).
                {
                    for (uint16_t i = 0; i < 255; i++)
                        payload[i] = (uint8_t)(i & 0xFF);
                    const size_t n = RackLinkBuildFrame(
                        RackLink::kTypeTelem, payload, 255, frame);
                    bool completed = false;
                    for (size_t i = 0; i < n; i++)
                        completed |= p.feed(frame[i]);
                    if (!(completed && p.crc_ok && p.flen == 255 &&
                          p.buf[255] == 0xFE))   // buf[1+i]=payload[i]
                        fails++;
                }
                // 5) Corrupted-CRC rejection + resync.
                {
                    const uint8_t pl[3] = { 0x11, 0x22, 0x33 };
                    const size_t n = RackLinkBuildFrame(0x40, pl, 3, frame);
                    frame[n - 1] ^= 0xFF;
                    bool completed = false;
                    for (size_t i = 0; i < n; i++)
                        completed |= p.feed(frame[i]);
                    if (!(completed && !p.crc_ok && p.state == 0)) fails++;
                }
                // 6) Post-rejection resync (the same good frame completes).
                {
                    const uint8_t pl[3] = { 0x11, 0x22, 0x33 };
                    const size_t n = RackLinkBuildFrame(0x40, pl, 3, frame);
                    bool completed = false;
                    for (size_t i = 0; i < n; i++)
                        completed |= p.feed(frame[i]);
                    if (!(completed && p.crc_ok && p.flen == 3)) fails++;
                }
                return fails == 0;
            }

            // Hardware loopback: uart_set_loop_back feeds TX into RX inside
            // the UART peripheral. 32 frames incl. one max-len frame; every
            // frame must come back verified. Returns the frames received;
            // rtt_us = the average per-frame turnaround (tx start -> last
            // byte read back) over the 31 small echo frames (the max-len
            // frame is ~259 bytes of wire time and would dominate it).
            int selftestLoopback(int64_t *rtt_us) {
                RackParser p;
                uint8_t frame[kMaxFrame];
                uint8_t payload[255];
                uart_flush_input(kUart);
                uart_set_loop_back(kUart, true);
                int ok_count = 0;
                int64_t small_total = 0;
                for (int i = 0; i < 32; i++) {
                    const uint8_t *pl;
                    uint8_t len;
                    if (i == 0) {
                        for (uint16_t k = 0; k < 255; k++)
                            payload[k] = (uint8_t)(k & 0xFF);
                        pl = payload;
                        len = 255;
                    } else {
                        payload[0] = RackLink::kCmdEcho;
                        payload[1] = (uint8_t)(i >> 8);
                        payload[2] = (uint8_t)i;
                        pl = payload;
                        len = 3;
                    }
                    const size_t n = RackLinkBuildFrame(
                        i == 0 ? RackLink::kTypeTelem : RackLink::kTypeUiCmd,
                        pl, len, frame);
                    const int64_t t0 = esp_timer_get_time();
                    uart_write_bytes(kUart, frame, n);
                    // Read EXACTLY n bytes back (loopback is byte-perfect,
                    // in-order): uart_read_bytes returns as soon as n bytes
                    // arrived or the timeout lapses — the RTT is the true
                    // tx->rx turnaround, not a timeout artifact.
                    uint8_t rb[kMaxFrame];
                    const int got =
                        uart_read_bytes(kUart, rb, n, pdMS_TO_TICKS(50));
                    const int64_t t1 = esp_timer_get_time();
                    bool completed = false;
                    for (int k = 0; k < got; k++)
                        completed |= p.feed((uint8_t)rb[k]);
                    if (completed && p.crc_ok) {
                        ok_count++;
                        if (i != 0) small_total += (t1 - t0);
                    }
                    if (!completed)
                        ESP_LOGW(TAG, "hw-loop frame %d lost", i);
                }
                uart_set_loop_back(kUart, false);
                *rtt_us = small_total / 31;
                return ok_count;
            }

        } // namespace

        bool RackLink::IsOnline() { return s_online; }

        void RackLink::Stats(uint32_t *tx, uint32_t *rx, uint32_t *crc) {
            if (tx) *tx = s_tx_frames;
            if (rx) *rx = s_rx_frames;
            if (crc) *crc = s_crc_errors;
        }

        bool RackLink::SendMidi(const uint8_t pkt[4]) {
            sendFrame(kTypeMidi, pkt, 4);
            s_last_midi_ms = (uint32_t)(xTaskGetTickCount() *
                                        portTICK_PERIOD_MS);
            return true;
        }

        bool RackLink::SendSwitch(uint8_t module_id) {
            if (module_id > 3) return false;
            sendFrame(kTypeSwitch, &module_id, 1);
            return true;
        }

        bool RackLink::SendModSet(uint8_t src, uint8_t param, uint8_t value) {
            const uint8_t p[3] = { src, param, value };
            sendFrame(kTypeModSet, p, 3);
            return true;
        }

        bool RackLink::SendUiCmd(uint8_t action, uint8_t arg, uint8_t arg2) {
            const uint8_t p[3] = { action, arg, arg2 };
            sendFrame(kTypeUiCmd, p, 3);
            return true;
        }

        bool RackLink::SendPageRequest(uint8_t page) {
            return SendUiCmd(kCmdPageRequest, page, 0);
        }

        bool RackLink::SendSysToggle(uint8_t item) {
            return SendUiCmd(kCmdSysToggle, item, 0);
        }

        bool RackLink::SendAssignCycle(uint8_t cell, uint8_t dir) {
            if (cell > 15 || dir > 1) return false;
            return SendUiCmd(kCmdAssignCycle, cell, dir);
        }

        bool RackLink::SendChainRoute(uint8_t a, uint8_t b, uint8_t engage) {
            if (a > 3 || b > 3 || engage > 1) return false;
            const uint8_t p[3] = { a, b, engage };
            sendFrame(kTypeChainRoute, p, 3);
            return true;
        }

        bool RackLink::SendPresetReq(uint8_t slot) {
            const uint8_t p[1] = { slot };
            sendFrame(kTypePresetReq, p, 1);
            return true;
        }

        bool RackLink::TelemGet(RackTelemPage &out) {
            taskENTER_CRITICAL(&telemMux);
            out = s_telem;
            taskEXIT_CRITICAL(&telemMux);
            return out.valid;
        }

        bool RackLink::TelemDirty() { return s_telem_dirty; }

        void RackLink::TelemClearDirty() { s_telem_dirty = false; }

        uint32_t RackLink::LastMidiMs() { return s_last_midi_ms; }

        void RackLink::Start() {
            if (hLinkTask != nullptr) return;

            uart_config_t uc = {};
            uc.baud_rate = kLinkBaud;
            uc.data_bits = UART_DATA_8_BITS;
            uc.parity = UART_PARITY_DISABLE;
            uc.stop_bits = UART_STOP_BITS_1;
            uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            uc.rx_flow_ctrl_thresh = 0;
            uc.source_clk = UART_SCLK_DEFAULT;
            // Rack link on UART0 (free: the console is USB-Serial-JTAG),
            // TX=GPIO4, RX=GPIO5.
            ESP_ERROR_CHECK(uart_set_pin(kUart, kPinTx, kPinRx,
                                         UART_PIN_NO_CHANGE,
                                         UART_PIN_NO_CHANGE));
            ESP_ERROR_CHECK(uart_param_config(kUart, &uc));
            ESP_ERROR_CHECK(uart_driver_install(kUart, kRxRing, kTxRing, 0,
                                                nullptr, 0));

            // ---- boot self-test (once; logs to the USB-JTAG console) ----
            const bool codec_ok = selftestCodec();
            int64_t rtt_us = 0;
            const int loop_ok = selftestLoopback(&rtt_us);
            ESP_LOGI(TAG, "selftest: codec %s | hw-loop %d/32 | rtt ~%lld us",
                     codec_ok ? "OK" : "FAIL", loop_ok, (long long)rtt_us);

            if (xTaskCreate(linkTask, "rack_link", 4096, nullptr,
                            tskIDLE_PRIORITY + 2, &hLinkTask) != pdPASS) {
                ESP_LOGW(TAG, "could not start rack link task");
                hLinkTask = nullptr;
                return;
            }
            ESP_LOGI(TAG, "rack link on UART0 GPIO%d/%d @%d (frame protocol: "
                          "docs/ui-protocol.md)",
                     kPinTx, kPinRx, kLinkBaud);
        }

    } // namespace CTRL
} // namespace CTAG
