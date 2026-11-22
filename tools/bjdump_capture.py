#!/usr/bin/env python3
"""Capture the benjolin device dump: inject the A/B CCs, run `bjdump 64`,
save the raw int16 pairs. Bench helper for the device-vs-host comparison."""
import argparse
import serial
import time

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--out", default="/tmp/benj_device.bin")
ap.add_argument("--baud", type=int, default=115200)
args = ap.parse_args()

# The user's norns A/B settings, in RAW CC (cc -> value 0-127).
CCS = [(60, 0), (61, 32), (62, 114), (63, 13), (64, 0), (65, 42), (66, 123),
       (67, 53), (68, 95), (69, 127), (70, 0), (71, 127), (72, 76), (73, 64)]

ser = serial.Serial(args.port, args.baud, timeout=0.5)
ser.dtr = True   # CDC gates TX on DTR; NEVER 1200 baud here (BOOTSEL touch)
ser.rts = False
time.sleep(0.3)
ser.reset_input_buffer()

def cmd(line):
    ser.write((line + "\r\n").encode())
    time.sleep(0.05)
    ser.read(4096)  # drain the echo/status

for cc, val in CCS:
    cmd(f"cc {cc} {val}")
time.sleep(0.3)          # let the params land (the block-boundary apply)
ser.reset_input_buffer()

# bjdump: the status line + the RAW payload arrive back-to-back (the handler
# blocks core 0 during the write, so the 2 s report cannot interleave) —
# read everything for 3 s, then locate the marker in the concatenated stream.
ser.write(b"bjdump 64\r\n")
time.sleep(0.05)
buf = b""
deadline = time.monotonic() + 3
while time.monotonic() < deadline:
    buf += ser.read(4096)
marker = b"bjdump 64 16384\r\n"
idx = buf.find(marker)
if idx < 0:
    raise SystemExit("bjdump status line not found; got: " + repr(buf[:200]))
payload = buf[idx + len(marker):idx + len(marker) + 64 * 16 * 2 * 2]
open(args.out, "wb").write(payload)
print(f"wrote {args.out} ({len(payload)} bytes)")
