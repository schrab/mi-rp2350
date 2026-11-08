/***************
Rack UI controller — T5 stage 1 link layer. Seeded from ctag-tbd/uimcu
(main/LinkClient.* keeps the seed's structure: 16 KB rx ring, dedicated
FreeRTOS task, vTaskDelay(1) tick note) but with the RACK frame protocol:
the seed's STX/ETX-JSON framing is replaced by the binary framed codec below.

MUST MATCH docs/ui-protocol.md and rack-rp2350/src/ui_link.h — frame format,
type ids, payload layouts and crc8 are byte-exact duplicates (two firmware
ends, one contract; a change on either side must land in both + the doc).

  frame = [0x7E][len:u8][type:u8][payload... len bytes][crc8]
  crc8  = CRC-8/ATM (poly 0x07, init 0x00, MSB-first, no reflection/xorout)
          computed over [type, then payload bytes]; no byte stuffing —
          0x7E inside a payload is legal because len is known up front.
***************/

#pragma once

#include <cstdint>
#include <cstddef>

namespace CTAG {
    namespace CTRL {

        class RackLink final {
        public:
            RackLink() = delete;

            // ---- protocol constants (MUST MATCH, see the file header) ----
            // Public so the .cpp's codec helpers share ONE copy with callers;
            // the byte-exact twin lives in rack-rp2350/src/ui_link.h.
            static constexpr uint8_t kFrameMark = 0x7E;
            // Frame types.
            static constexpr uint8_t kTypeMidi        = 0x01;
            static constexpr uint8_t kTypeSwitch      = 0x02;
            static constexpr uint8_t kTypeModSet      = 0x03;
            static constexpr uint8_t kTypeUiCmd       = 0x04;
            static constexpr uint8_t kTypeChainRoute  = 0x05;
            static constexpr uint8_t kTypeTelem       = 0x10;
            static constexpr uint8_t kTypePresetReq   = 0x11;  // reserved T4a
            static constexpr uint8_t kTypePresetPart  = 0x12;  // reserved T4a
            static constexpr uint8_t kTypePresetApply = 0x13;  // reserved T4a
            // UI_CMD actions.
            static constexpr uint8_t kCmdPageRequest = 1;
            static constexpr uint8_t kCmdEcho        = 2;
            static constexpr uint8_t kCmdSysToggle   = 3;
            // Page ids.
            static constexpr uint8_t kPageP1 = 0;

            // UART_NUM_0 GPIO4(TX)/GPIO5(RX) @ 921600 8N1 + boot self-tests
            // (codec round-trip + uart_set_loop_back hardware loopback) +
            // the rx task. The seed console is USB-Serial-JTAG
            // (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG), so UART0 is free.
            static void Start();

            // ---- sending side (stages 3-5 call these) ----
            // 4-byte USB-MIDI event packet inside a 0x01 MIDI frame (the
            // RP2350 funnels it through the same midiHandleEventPacket the
            // USB device/host/DIN paths use).
            static bool SendMidi(const uint8_t pkt[4]);
            // 0x02 SWITCH (module id 0-3 -> the RP2350's console switch path).
            static bool SendSwitch(uint8_t module_id);
            // 0x03 MOD_SET [src][param][value] (param ids = the RP2350's
            // kModParamNames indexes; value 0-127, bench-mux quantization).
            static bool SendModSet(uint8_t src, uint8_t param, uint8_t value);
            // 0x04 UI_CMD [action][arg][arg2] (1 page request / 2 echo /
            // 3 sys toggle).
            static bool SendUiCmd(uint8_t action, uint8_t arg, uint8_t arg2);
            // Convenience: UI_CMD action 1 = page request (0 P1, 1 P2,
            // 2 MOD-A, 3 MOD-B, 4 MOD-C, 5 SYS) -> RP2350 replies a full
            // TELEM snapshot.
            static bool SendPageRequest(uint8_t page);

            // True while a verified frame arrived within the last 3 s
            // (mirrors the seed LinkClient's offline watchdog).
            static bool IsOnline();
        };

    } // namespace CTRL
} // namespace CTAG
