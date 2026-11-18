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
    last_avg = None
    last_stalls = None
    pat = re.compile(r"max Process: (\d+) us avg (\d+) us .* i2s stalls: (\d+)")
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
                last_avg = int(m.group(2))
                last_stalls = int(m.group(3))
    return last_max, last_avg, last_stalls, lines


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
max_off, avg_off, stalls_off, lines_off = collect(8.0)
send("cc 42 5")               # FX TYPE = SWEDE
send("cc 43 90")              # D DRIVE ~0.71 — the shaper + IR engaged
send("cc 48 0")               # bench: cab conv OFF
max_nc, avg_nc, stalls_nc, lines_nc = collect(8.0)
send("cc 48 100")             # bench: cab conv ON
max_sw, avg_sw, stalls_sw, lines_sw = collect(8.0)

print("== last OFF report ==")
for l in lines_off[-2:]:
    print(l)
print("== last SWEDE (conv OFF) report ==")
for l in lines_nc[-2:]:
    print(l)
print("== last SWEDE (conv ON) report ==")
for l in lines_sw[-2:]:
    print(l)
if max_off is not None and max_sw is not None:
    print(f"\nOFF          max: {max_off} us  avg: {avg_off} us")
    print(f"SWEDE noconv max: {max_nc} us  avg: {avg_nc} us")
    print(f"SWEDE conv   max: {max_sw} us  avg: {avg_sw} us")
    print(f"shaper(noconv) - OFF = {avg_nc - avg_off} us avg")
    print(f"conv delta = {avg_sw - avg_nc} us avg")
    print(f"i2s stalls: OFF={stalls_off} noconv={stalls_nc} conv={stalls_sw}")
ser.close()
