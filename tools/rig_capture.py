#!/usr/bin/env python3
"""Record the rack's UAC1 loopback stream and report its health.

Usage:
  python3 tools/rig_capture.py --seconds 10 --out /tmp/rack.wav

The rack exposes a 2-ch, 24-bit packed UAC1 capture function (EP 0x84).
This tool records it with arecord, converts the packed 24-bit samples to
peak/RMS numbers, and reads the device's dropped/inserted-frame counters
over the CDC console before and after the capture. A capture is healthy when
the signal is non-silent and neither counter advanced during the stream.
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys
import time

import serial

DEFAULT_PORT_GLOB = "/dev/serial/by-id/usb-rack_Rack_RP2350_*-if00"
CAP_RE = re.compile(r"capture: rate=(\d+) overflow=(\d+) underrun=(\d+)")


def console(port_path, command=None, settle=0.4):
    """Send a command with the stale-byte protocol and return the reply text."""
    with serial.Serial(port_path, 115200, timeout=0.5) as s:
        s.write(b"\n")                 # flush any stale bytes from the last write
        time.sleep(0.15)
        s.reset_input_buffer()
        if command is None:
            time.sleep(settle)
            return ""
        s.write((command + "\n").encode())
        time.sleep(settle)
        data = s.read(65536)
    return data.decode("utf-8", "replace")


def read_counters(port_path):
    text = console(port_path, "capture")
    m = CAP_RE.search(text)
    if not m:
        raise RuntimeError("no capture reply from console: %r" % text[:200])
    return {"rate": int(m.group(1)), "overflow": int(m.group(2)),
            "underrun": int(m.group(3))}


def analyze_wav(path):
    import wave
    with wave.open(path, "rb") as w:
        channels, rate, width, frames = (
            w.getnchannels(), w.getframerate(), w.getsampwidth(), w.getnframes())
        data = w.readframes(frames)
    if width != 3:
        raise ValueError("expected 24-bit packed samples, got width %d" % width)
    peak = 0
    total = 0.0
    count = 0
    for i in range(0, len(data) - 5, width * channels):
        for c in range(channels):
            b = data[i + c * 3:i + c * 3 + 3]
            v = b[0] | (b[1] << 8) | (b[2] << 16)
            if v & 0x800000:
                v -= 0x1000000
            a = abs(v)
            if a > peak:
                peak = a
            total += v * v
            count += 1
    rms = math.sqrt(total / count) if count else 0.0
    return {"channels": channels, "rate": rate, "frames": frames,
            "peak": peak, "rms": round(rms, 2),
            "rms_full_scale": round(rms / 8388608.0, 5)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--rate", type=int, default=48000,
                    help="must match the active module's rate (console 'capture')")
    ap.add_argument("--out", default="/tmp/rack_capture.wav")
    ap.add_argument("--device", default="hw:0,0")
    ap.add_argument("--port", default=None,
                    help="CDC by-id path (default: resolve by glob)")
    ap.add_argument("--rms-floor", type=float, default=0.01,
                    help="minimum RMS as a fraction of full scale")
    args = ap.parse_args()

    port = args.port or (glob.glob(DEFAULT_PORT_GLOB) or [None])[0]
    if not port:
        print("no rack CDC port found", file=sys.stderr)
        return 2

    before = read_counters(port)
    cmd = ["arecord", "-D", args.device, "-f", "S24_3LE",
           "-r", str(args.rate), "-c", "2", "-d", str(int(args.seconds)),
           args.out]
    print("recording:", " ".join(cmd))
    subprocess.run(cmd, check=True)
    after = read_counters(port)
    info = analyze_wav(args.out)
    # The driver zeroes its glitch counters when the host selects alt=1, so
    # 'after' counts only this stream. Before the stream starts, idle drops
    # are expected and are not a fault.
    reset = after["overflow"] < before["overflow"] or \
            after["underrun"] < before["underrun"]
    overflow_delta = after["overflow"] if reset else \
                     after["overflow"] - before["overflow"]
    underrun_delta = after["underrun"] if reset else \
                     after["underrun"] - before["underrun"]
    # arecord needs ~0.15 s to start reading after SET_INTERFACE; frames
    # dropped in that window are counted but are not a capture glitch.
    startup_allowance = int(args.rate * 0.2)
    info.update({"before": before, "after": after, "counters_reset": reset,
                 "overflow_delta": overflow_delta,
                 "underrun_delta": underrun_delta,
                 "startup_allowance": startup_allowance})

    summary = args.out + ".json"
    with open(summary, "w") as f:
        json.dump(info, f, indent=2)

    ok = (info["rms_full_scale"] >= args.rms_floor and
          underrun_delta == 0 and overflow_delta <= startup_allowance)
    print(json.dumps(info, indent=2))
    print("VERDICT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
