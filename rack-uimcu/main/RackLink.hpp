/***************
Rack UI controller — T5 link layer. Seeded from ctag-tbd/uimcu
(main/LinkClient.* keeps the seed's structure: 16 KB rx ring, dedicated
FreeRTOS task, vTaskDelay(1) tick note) but with the RACK frame protocol:
the seed's STX/ETX-JSON framing is replaced by the binary framed codec below.

STAGE 2-6 EXTENSIONS over stage 1 (all additive; see docs/ui-protocol.md §5):
  - TELEM v2 parse: cells are 4 bytes (cc, value, flags, color — color = the
    RP2350 registry's class byte, the C3 does NOT duplicate the registry);
    full snapshots (mode 0) end with the 4-byte status tail
    [bpm][clocksrc][chain][link] rendered by the status strip.
  - flags bits 3-6 = routed mod source id + 1 (0 unrouted); bits 0-2 as
    stage 1 (mod-routed / chain-B / inert).
  - MOD/SYS cells: cc field = kModCellBase + cell index (opaque cell id,
    not a MIDI CC). MOD page pot turns send MOD_SET [src][param][value]
    with the (src,param) map mirrored in ui/RackUI (MUST MATCH the
    RP2350's kUiMod*Cell tables).
  - New senders: SendChainRoute (SYS chain pair/engage), SendPresetReq
    (T4a stub, payload [slot]), SysToggle, AssignCycle (UI_CMD action 4).
  - TelemGet: consistent snapshot copy for the renderer + a dirty flag;
    SendMidi stamps the MIDI-activity dot.

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

        // One parsed TELEM cell (v2). Label <= 10 chars + NUL on the wire;
        // 16 here absorbs any trailing byte-truncation safely.
        struct RackTelemCell {
            uint8_t cc;      // MIDI CC, or kModCellBase + cell index (MOD/SYS)
            uint8_t value;   // 0-127
            uint8_t flags;   // bit0 routed, bit1 chain-B, bit2 inert,
                             // bits 3-6 routed source id + 1
            uint8_t color;   // 0 white / 1 red / 2 teal (registry class)
            char label[16];
        };

        // Parsed TELEM page (the renderer's source of truth; placeholders
        // render where valid == false — the C3 never invents values).
        struct RackTelemPage {
            uint8_t page, module, mode, count;
            bool valid;                       // a snapshot/delta landed
            RackTelemCell cell[16];
            // Status tail (mode 0 full snapshots refresh it; deltas keep it):
            bool tail_valid;
            uint16_t bpm;      // internal BPM 20-300, 0 = external MIDI clock
            uint8_t clocksrc;  // 0 internal / 1 midi
            uint8_t chain;     // 0 off / 1-3 engaged pair index
            uint8_t link;      // bit0 DIN saw bytes, bit1 UART link fresh
        };

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
            static constexpr uint8_t kTypePresetReq   = 0x11;  // T4a bank
            static constexpr uint8_t kTypePresetPart  = 0x12;  // reserved T4a
            static constexpr uint8_t kTypePresetApply = 0x13;  // reserved T4a
            // UI_CMD actions.
            static constexpr uint8_t kCmdPageRequest = 1;
            static constexpr uint8_t kCmdEcho        = 2;
            static constexpr uint8_t kCmdSysToggle   = 3;
            static constexpr uint8_t kCmdAssignCycle = 4;
            static constexpr uint8_t kCmdSetGlobal   = 5;
            // SET_GLOBAL items (action 5, [item][value 0-127]).
            static constexpr uint8_t kGlobalBpm = 0;   // 20 + v*280/127 BPM
            // Sys toggle item ids (action 3; semantics = TOGGLE, arg2 = 0).
            static constexpr uint8_t kSysTelemDelta  = 0;
            static constexpr uint8_t kSysTelemFull   = 1;
            static constexpr uint8_t kSysClockToggle = 2;
            // TELEM modes.
            static constexpr uint8_t kTelemFull  = 0;
            static constexpr uint8_t kTelemDelta = 1;
            // Cell flags.
            static constexpr uint8_t kFlagModRouted = 1 << 0;
            static constexpr uint8_t kFlagChainB    = 1 << 1;
            static constexpr uint8_t kFlagInert     = 1 << 2;
            static constexpr uint8_t kFlagSrcShift  = 3;
            static constexpr uint8_t kFlagSrcMask   = 0x0F << 3;
            // MOD/SYS opaque cell id base (cc field = base + cell index).
            static constexpr uint8_t kModCellBase = 200;
            // Page ids.
            static constexpr uint8_t kPageP1        = 0;
            static constexpr uint8_t kPageP2        = 1;
            static constexpr uint8_t kPageModLfo12  = 2;
            static constexpr uint8_t kPageModLfo34  = 3;
            static constexpr uint8_t kPageModTrig   = 4;
            static constexpr uint8_t kPageModGate   = 5;
            static constexpr uint8_t kPageModEnv    = 6;
            static constexpr uint8_t kPageSys       = 7;
            // Chain-B panel (page 8): while a chain is engaged the RP2350
            // serves the CLOUDS P1 registry table with the B slot's live
            // values — the C3 navigates to it only while chained.
            static constexpr uint8_t kPageChainB    = 8;
            static constexpr uint8_t kPageCount     = 9;

            // UART_NUM_0 GPIO4(TX)/GPIO5(RX) @ 921600 8N1 + boot self-tests
            // (codec round-trip + uart_set_loop_back hardware loopback) +
            // the rx task. The seed console is USB-Serial-JTAG
            // (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG), so UART0 is free.
            static void Start();

            // ---- sending side (stages 3-6) ----
            // 4-byte USB-MIDI event packet inside a 0x01 MIDI frame (the
            // RP2350 funnels it through the same midiHandleEventPacket the
            // USB device/host/DIN paths use). Stamps the activity dot.
            static bool SendMidi(const uint8_t pkt[4]);
            // 0x02 SWITCH (module id 0-3 -> the RP2350's console switch path).
            static bool SendSwitch(uint8_t module_id);
            // 0x03 MOD_SET [src][param][value] (param ids = the RP2350's
            // kModParamNames indexes; value 0-127, bench-mux quantization).
            // The RP2350 acks with a single-cell TELEM delta for the
            // matching cell of the C3's active page.
            static bool SendModSet(uint8_t src, uint8_t param, uint8_t value);
            // 0x04 UI_CMD [action][arg][arg2] (1 page request / 2 echo /
            // 3 sys toggle / 4 assign cycle).
            static bool SendUiCmd(uint8_t action, uint8_t arg, uint8_t arg2);
            // Convenience: UI_CMD action 1 = page request (0 P1, 1 P2,
            // 2 MOD-A, 3 MOD-B, 4 MOD-C, 5 SYS) -> RP2350 replies a full
            // TELEM snapshot.
            static bool SendPageRequest(uint8_t page);
            // Sys toggle (action 3, arg = item id, semantics = TOGGLE).
            static bool SendSysToggle(uint8_t item);
            // Mod-assign gesture: cycle the ACTIVE page's cell [0-15] through
            // [off, src0..src11]; dir 1 = up, 0 = down. The RP2350 resolves
            // the cell, owns routing + validation, and replies a full TELEM.
            static bool SendAssignCycle(uint8_t cell, uint8_t dir);
            // 0x05 CHAIN_ROUTE [a][b][engage] -> the console `chain` path
            // (SYS page chain pair select + engage toggle).
            static bool SendChainRoute(uint8_t a, uint8_t b, uint8_t engage);
            // 0x11 PRESET_REQ [slot:u8] — "give me your current state part"
            // (save flow; slot echoed in the PART reply, 0xFF = autosave pull).
            static bool SendPresetReq(uint8_t slot);
            // 0x13 PRESET_APPLY as chunked [seq][total][data <=252] frames —
            // (cap MUST MATCH the RP2350's kPresetChunkData) the RP2350
            // validates + applies the blob and acks a full TELEM.
            static bool SendPresetApply(const uint8_t *blob, uint16_t n);

            // True while a verified frame arrived within the last 3 s
            // (mirrors the seed LinkClient's offline watchdog).
            static bool IsOnline();
            // Frame counters for the console `ui` command.
            static void Stats(uint32_t *tx, uint32_t *rx, uint32_t *crc);

            // ---- TELEM cache (renderer side) ----
            // Copy the parsed page out under a critical section. Returns
            // false when nothing has landed yet (renderer draws placeholders).
            static bool TelemGet(RackTelemPage &out);
            // True when a TELEM landed since the last TelemClearDirty().
            static bool TelemDirty();
            static void TelemClearDirty();
            // Optimistic local update of a cell's value when a live pot moves
            // (absorbs the RP2350's 50 ms TELEM delta throttle so fast turns
            // update at 60 fps and never miss the final trailing value).
            static void TelemLocalCellWrite(uint8_t slot, uint8_t value);
            // Optimistic local update of a cell's routed mod source (0 = OFF,
            // 1..12 = LFO1..ADS2).
            static void TelemLocalAssignWrite(uint8_t slot, uint8_t srcIdxPlus1);
            // Optimistic local update when switching active module on SYS.
            static void TelemLocalSysSwitch(uint8_t newMod);

            // ms timestamp of the last SendMidi (the strip's activity dot).
            static uint32_t LastMidiMs();
        };

    } // namespace CTRL
} // namespace CTAG
