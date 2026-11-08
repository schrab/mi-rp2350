/***************
Rack UI controller — T5 stage 6: BLE-MIDI PERIPHERAL (NimBLE).

ROLE/DIRECTION (verified against the seed + the plan): the seed's
BtMidiReceiver was a NimBLE CENTRAL (scan -> connect -> subscribe). The
plan's T5 role is the opposite: the rack panel is a BLE-MIDI DEVICE — a
central (phone/DAW controller) connects to IT and sends MIDI (plan §T5:
"NimBLE stack: BLE-MIDI device (the plan's role)"; bring-up stage 6).
This file is that peripheral: advertise the STANDARD BLE-MIDI service
(the seed's UUIDs kept verbatim — MIDI-BLE spec 03B80E5A-EDE8-4B33-A751-
6CE34EC4C700 service, ...C702 I/O characteristic), accept writes, parse
the Apple BLE-MIDI packet encoding and forward every message as a 4-byte
USB-MIDI event packet to RackLink::SendMidi (the RP2350 funnels it through
the SAME midiHandleEventPacket as USB device/host/DIN — zero new dispatch
semantics).

Packet conversion (docs/ui-protocol.md §2 MIDI frames):
  channel voice 0x80-0xEF -> CIN = status>>4, cable 0
  system real-time 0xF8-0xFF -> CIN 0x0F single byte
  system common / SYsex 0xF0-0xF7 -> dropped + counted (the RP2350's
  funnel ignores them; sysex streaming is out of scope)

Name: "Rack UI". Connection params: the stack defaults (7.5-15 ms windows
are requested by the central; no custom L2CAP magic). Heap: logged before
NimBLE init at boot (README bring-up) — NimBLE + the 43 KB framebuffer +
the link rings coexist in the C3's ~320 KB.
***************/

#pragma once

#include <cstdint>

namespace CTAG::DRIVERS {

class BtMidiReceiver {
public:
    static void Init();
    // True while a central holds the connection (strip "BLE" marker).
    static bool IsConnected();
    // Counters for the `ui` console: BLE-MIDI messages forwarded / dropped.
    static uint32_t MsgCount();
    static uint32_t DropCount();
};

}
