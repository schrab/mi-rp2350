#!/usr/bin/env python3
"""Measure the swede FX cost on the rack console: switch to Elements, capture
the 2 s report's max-Process with FX OFF, then with FX TYPE = SWEDE + drive,
and print the delta. Bench helper — never flashes."""
import re
import serial
import time

PORT = "COM15"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.2)
ser.dtr = True          # CDC TX gate (NEVER 1200 baud + DTR = BOOTSEL touch)
ser.rts = False
ser.reset_input_buffer()


def send(cmd):
    ser.write((cmd + "\n").encode())
    ser.reset_input_buffer()   # drop the echo of our own command


def collect(secs):
    """Read report lines; return (last_max_us, last_stalls, lines)."""
    lines = []
    last_max = None
    last_stalls = None
    pat = re.compile(r"max Process: (\d+) us .* i2s stalls: (\d+)")
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        data = ser.read(4096)
        if not data:
            continue
        for chunk in data.decode(errors="replace").split("\r\n"):
            if chunk.strip():
                lines.append(chunk.strip())
            m = pat.search(chunk)
            if m:
                last_max = int(m.group(1))
                last_stalls = int(m.group(2))
    return last_max, last_stalls, lines


time.sleep(0.3)
send("")                      # newline: stale-byte recovery after flash
send("module")
time.sleep(0.5)
# Force TWO real swaps: proc_us_max resets on every swap (the self-swap
# guard would skip switch-to-same-module), so bouncing lira8 -> elements
# gives a clean per-phase counter for the OFF baseline.
send("switch 4")
time.sleep(1.0)
send("switch 0")
time.sleep(1.0)
max_off, stalls_off, lines_off = collect(8.0)
send("cc 42 5")               # FX TYPE = SWEDE
send("cc 43 90")              # D DRIVE ~0.71 — the shaper + IR engaged
max_sw, stalls_sw, lines_sw = collect(8.0)

print("== last OFF report ==")
for l in lines_off[-3:]:
    print(l)
print("== last SWEDE report ==")
for l in lines_sw[-3:]:
    print(l)
if max_off is not None and max_sw is not None:
    print(f"\nOFF max Process:   {max_off} us")
    print(f"SWEDE max Process: {max_sw} us")
    print(f"delta (conv + shaper + FMV): {max_sw - max_off} us "
          f"({(max_sw - max_off) / 496.0 * 100:.1f}% of the 496 us budget)")
    print(f"i2s stalls: OFF={stalls_off} SWEDE={stalls_sw}")
ser.close()
