/***************
Rack UI controller — T5 stage 6 + T4a-era addendum: BLE-MIDI, BOTH ROLES
(NimBLE).

ROLE/DIRECTION: the rack panel's BLE-MIDI runs in one of two modes, user-
selectable on the SYS page (cells 9/10) and the C3 console:

  PERIPHERAL (mode 0, default): advertise "Rack UI" with the STANDARD
  BLE-MIDI service (03B80E5A-...-C700 service / ...-C702 I/O
  characteristic); a central (phone/DAW controller/PC) connects to IT
  and sends MIDI (plan §T5 role). Writes -> parseBleMidi -> SendMidi.

  CENTRAL (mode 1): scan, auto-connect to the first advertiser carrying
  the BLE-MIDI service (e.g. the desk's SMC-PAD, which advertises it
  natively), subscribe to its MIDI characteristic (NOTIFY) and feed
  notifications through the SAME parseBleMidi -> SendMidi funnel.
  Lifted from the seed's central (git d901e76), which was built against
  this pad — its hard-won quirks are kept: auto-accept the pad's
  passkey action (ignoring it stalls its stack), reliable (with-
  response) CCCD write, CCCD re-subscribe after pairing/encryption
  settles, and a BLE-MIDI keepalive poke right after subscribing (the
  pad starts notifying only once the client talks to it).

Packet conversion (docs/ui-protocol.md §2 MIDI frames): channel voice
0x80-0xEF -> CIN = status>>4, cable 0; system real-time 0xF8-0xFF ->
CIN 0x0F single byte; system common / sysex dropped + counted. Both
RX paths (GATT write, GATT notify) feed ONE parser (parseBleMidi — the
seed's position logic: per-message timestamps, mid-stream status
overrides, running status across packets).

Names/persistence: peripheral name "Rack UI"; enabled + mode persist in
NVS namespace "bleui" (keys en/mode) — boot restores the last state,
default ON + PERIPHERAL.

Connection params: stack defaults (no custom L2CAP magic). Heap: logged
before NimBLE init at boot — NimBLE + the 43 KB framebuffer + the link
rings coexist in the C3's ~320 KB (benched 2026-11-11: 103 KB free with
an active peripheral connection).
***************/

#pragma once

#include <cstdint>

namespace CTAG::DRIVERS {

class BtMidiReceiver {
public:
    static void Init();

    // Mode select: 0 = peripheral (advertise "Rack UI"), 1 = central
    // (scan + auto-connect to a BLE-MIDI peripheral). Switching tears
    // down the current role (advertise stop / peer disconnect) and
    // starts the other; persisted to NVS.
    static constexpr uint8_t kModePeripheral = 0;
    static constexpr uint8_t kModeCentral = 1;
    static void SetMode(uint8_t mode);
    static uint8_t Mode();
    // Radio on/off (off = no advertise + disconnect + no scan).
    static void SetEnabled(bool on);
    static bool Enabled();

    // True while ANY role holds a connection (strip "BLE" marker).
    static bool IsConnected();
    // Counters for the console `ui` line: BLE-MIDI messages forwarded /
    // dropped.
    static uint32_t MsgCount();
    static uint32_t DropCount();

    // Console: `ble [status|on|off|per|cen|scan|connect]`.
    static void ConsoleCmd(const char *args);
};

}
