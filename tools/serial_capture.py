#!/usr/bin/env python3
"""Capture N seconds of a COM port to stdout (bench helper)."""
import argparse
import serial
import time

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--secs", type=float, default=20.0)
args = ap.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=0.2)
# CDC gates TX on DTR (Adafruit_USBD_CDC) — assert it. NEVER open at 1200
# baud with DTR: that is the reboot-to-BOOTSEL touch.
ser.dtr = True
ser.rts = False
t_end = time.monotonic() + args.secs
ser.reset_input_buffer()
while time.monotonic() < t_end:
    data = ser.read(4096)
    if data:
        print(data.decode(errors="replace"), end="", flush=True)
ser.close()
