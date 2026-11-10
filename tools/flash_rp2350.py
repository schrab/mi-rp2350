#!/usr/bin/env python3
"""Flash the RP2350 rack firmware from Windows: 1200-baud CDC touch ->
BOOTSEL MSC drive appears -> copy firmware.uf2.

Usage: python tools/flash_rp2350.py [uf2-path] [comport]
Default: rack-rp2350/.pio/build/rack-elements/firmware.uf2 on COM15.
"""
import ctypes
import os
import shutil
import sys
import time

import serial

UF2 = sys.argv[1] if len(sys.argv) > 1 else \
    "rack-rp2350/.pio/build/rack-elements/firmware.uf2"
PORT = sys.argv[2] if len(sys.argv) > 2 else "COM15"


def find_rp2350_drive(timeout=20.0):
    """Find a removable drive labeled RP2350 (BOOTSEL MSC)."""
    kernel32 = ctypes.windll.kernel32
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
            root = letter + ":\\"
            if not os.path.exists(root):
                continue
            name = ctypes.create_unicode_buffer(64)
            if kernel32.GetVolumeInformationW(root, name, 64, None, None,
                                              None, None, 0):
                if name.value.upper().startswith("RP2350"):
                    return root
        time.sleep(0.4)
    return None


def main():
    if not os.path.exists(UF2):
        sys.exit(f"no UF2 at {UF2}")
    print(f"touching {PORT} at 1200 baud (reboot to BOOTSEL)...")
    with serial.Serial(PORT, 1200, timeout=0.5) as s:
        s.dtr = True
        time.sleep(0.3)
    root = find_rp2350_drive()
    if not root:
        sys.exit("no RP2350 BOOTSEL drive appeared (known erratics: retry "
                 "the touch, fall back to button-BOOTSEL)")
    print(f"drive {root} - copying {UF2}...")
    shutil.copy(UF2, os.path.join(root, "firmware.uf2"))
    # give the copy a moment, then the board reboots itself
    time.sleep(2.0)
    print("flashed.")


if __name__ == "__main__":
    main()
