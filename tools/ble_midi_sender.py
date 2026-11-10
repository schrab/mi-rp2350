#!/usr/bin/env python3
"""BLE-MIDI central bench sender for the rack UI MCU (T5 stage 6).

Connects to the ESP32-C3 "Rack UI" BLE-MIDI PERIPHERAL (standard BLE-MIDI
service 03B80E5A-...-6CE34EC4C700, I/O characteristic ...C702) and sends
note-ons / CCs encoded per the Apple BLE-MIDI packet format. Every message
must surface on the RP2350 through RackLink::SendMidi -> the shared
midiHandleEventPacket funnel (audible notes, `notes=` counter advancing).

Usage:
  python tools/ble_midi_sender.py                 # scan + note walk C2..C3
  python tools/ble_midi_sender.py --scan          # scan only
  python tools/ble_midi_sender.py --cc 7 100      # single CC then exit
  python tools/ble_midi_sender.py --hold 60       # one sustained note 2 s
  python tools/ble_midi_sender.py --count 16 --interval 0.25
"""
import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

MIDI_SVC = "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
MIDI_IO = "03b80e5a-ede8-4b33-a751-6ce34ec4c702"

_t0 = time.monotonic()


def ts13() -> int:
    """13-bit timestamp: ms since sender start, mod 8192."""
    return int((time.monotonic() - _t0) * 1000) & 0x1FFF


def packet(*msgs: bytes) -> bytes:
    """Wrap raw MIDI messages in one Apple BLE-MIDI packet."""
    t = ts13()
    head = 0x80 | ((t >> 7) & 0x3F)
    tsb = 0x80 | (t & 0x7F)
    return bytes([head]) + b"".join(bytes([tsb]) + m for m in msgs)


def note_on(n: int, vel: int = 96) -> bytes:
    return bytes([0x90, n & 0x7F, vel & 0x7F])


def note_off(n: int) -> bytes:
    return bytes([0x80, n & 0x7F, 0x00])


def cc(num: int, val: int) -> bytes:
    return bytes([0xB0, num & 0x7F, val & 0x7F])


async def scan() -> str | None:
    print(f"scanning for BLE-MIDI peripherals (name 'Rack UI' / svc {MIDI_SVC[:8]}...) ...")
    devs = await BleakScanner.discover(timeout=8.0, return_adv=True)
    hit = None
    for d, adv in devs.values():
        tag = ""
        if d.name and "rack" in d.name.lower():
            tag = "  <-- NAME MATCH"
            hit = d
        uuids = list(adv.service_uuids)
        if any(str(u).lower() == MIDI_SVC for u in uuids) and not hit:
            tag = "  <-- SERVICE MATCH"
            hit = d
        print(f"  {d.address}  rssi={adv.rssi:<4} name={d.name!r}{tag}")
    return hit.address if hit else None


async def run(addr: str, args) -> None:
    print(f"connecting to {addr} ...")
    async with BleakClient(addr, timeout=15.0) as cli:
        print("connected:", cli.is_connected)
        svcs = cli.services
        svc = svcs.get_service(MIDI_SVC)
        if svc is None:
            print(f"ERROR: BLE-MIDI service {MIDI_SVC} not in GATT; has:",
                  [str(s.uuid) for s in svcs])
            return
        print("service OK, characteristics:",
              [(str(c.uuid), c.properties) for c in svc.characteristics])

        if args.cc is not None:
            num, val = args.cc
            await cli.write_gatt_char(MIDI_IO, packet(cc(num, val)),
                                      response=True)
            print(f"sent CC {num} = {val}")
            return

        if args.hold is not None:
            await cli.write_gatt_char(MIDI_IO, packet(note_on(args.hold)),
                                      response=True)
            print(f"note {args.hold} ON - holding {args.hold_time}s ...")
            await asyncio.sleep(args.hold_time)
            await cli.write_gatt_char(MIDI_IO, packet(note_off(args.hold)),
                                      response=True)
            print(f"note {args.hold} OFF")
            return

        # Default: walk C2..C3 (36..48, the Elements auto-trigger range),
        # a CC 1 sweep in the middle, note-offs interleaved.
        interval = args.interval
        for i in range(args.count):
            n = 36 + (i % 13)
            await cli.write_gatt_char(MIDI_IO,
                                      packet(note_on(n), cc(1, 20 + i * 5)),
                                      response=True)
            print(f"[{i + 1}/{args.count}] note {n} ON (cc1={20 + i * 5})")
            await asyncio.sleep(interval)
            await cli.write_gatt_char(MIDI_IO, packet(note_off(n)),
                                      response=True)
            await asyncio.sleep(0.05)
        print("done - check the RP2350 serial report notes= counter + ears")


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scan", action="store_true", help="scan only")
    ap.add_argument("--address", help="skip scan, connect this address")
    ap.add_argument("--cc", nargs=2, type=int, metavar=("NUM", "VAL"))
    ap.add_argument("--hold", type=int, metavar="NOTE",
                    help="one sustained note")
    ap.add_argument("--hold-time", type=float, default=2.0)
    ap.add_argument("--count", type=int, default=8)
    ap.add_argument("--interval", type=float, default=0.5)
    args = ap.parse_args()

    addr = args.address or await scan()
    if args.scan:
        return
    if not addr:
        print("no 'Rack UI' peripheral found - is the C3 powered and "
              "unconnected (it advertises until a central holds it)?")
        sys.exit(1)
    await run(addr, args)


if __name__ == "__main__":
    asyncio.run(main())
