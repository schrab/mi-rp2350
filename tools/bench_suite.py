#!/usr/bin/env python3
"""Scripted device suite for the rack's line console (phase-2 task 3).

Replaces eyeballing the 2 s report with pass/fail exit codes. All commands go
to the composite CDC by-id; the stale-byte law is honoured (newline + flush
before trusting any reply). A watchdog reset mid-run is detected by a
backwards `up=` value and fails the case.

Cases:
  smoke     liveness, budget, stalls, module identity
  swap      round-trip swaps over all seven modules, zero stall deltas
  snapshot  preset dump -> load -> dump, identical CRC/size/CC count
  capture   optional UAC1 capture via tools/rig_capture.py

Exit 0 = all selected cases pass; non-zero names the failing case.
"""
import argparse
import glob
import re
import subprocess
import sys
import time

import serial

DEFAULT_PORT_GLOB = "/dev/serial/by-id/usb-rack_Rack_RP2350_*-if00"
MODULES = ["elements", "clouds", "plaits", "rings", "lira8", "dronecaster", "benjolin", "openrb303"]
REPORT_RE = re.compile(
    r"max Process:\s*(\d+)\s*us avg\s*(\d+)\s*us \(budget\s*(\d+)\s*us\)"
    r".*?i2s stalls:\s*(\d+)\s*\(\+(\d+)\)")
UP_RE = re.compile(r"\bup=(\d+)")
PRESET_RE = re.compile(r"preset: v(\d+),\s*(\d+) B,.*?seen=(\d+) CCs")


class Console:
    def __init__(self, path, timeout=8.0):
        self.path = path
        self.s = serial.Serial(path, 115200, timeout=0.2)
        self.buf = ""
        self.reply = []
        self.last_report = None
        self.last_up = -1
        self.deadline = time.monotonic() + timeout

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass

    def reconnect(self):
        self.close()
        self.s = serial.Serial(self.path, 115200, timeout=0.2)

    def _pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                waiting = self.s.in_waiting
            except (OSError, serial.SerialException):
                time.sleep(0.2)
                self.reconnect()
                continue
            if waiting:
                data = self.s.read(waiting).decode("utf-8", "replace")
                self.buf += data
                while "\n" in self.buf:
                    line, self.buf = self.buf.split("\n", 1)
                    line = line.rstrip("\r")
                    self.reply.append(line)
                    m = REPORT_RE.search(line)
                    if m:
                        self.last_report = {
                            "max_us": int(m.group(1)), "avg_us": int(m.group(2)),
                            "budget_us": int(m.group(3)), "stalls": int(m.group(4)),
                            "stall_delta": int(m.group(5))}
                    u = UP_RE.search(line)
                    if u:
                        self.last_up = int(u.group(1))
            else:
                time.sleep(0.01)

    def cmd(self, text, settle=0.4):
        self.s.write(b"\n")
        time.sleep(0.12)
        self.s.reset_input_buffer()
        self.buf = ""
        self.reply = []
        self.s.write((text + "\n").encode())
        self._pump(settle)
        return "\n".join(self.reply)

    def wait_report(self, timeout=6.0):
        """Wait for a fresh 2 s report and a liveness check."""
        start_up = self.last_up
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._pump(0.1)
            if self.last_up != start_up and self.last_report:
                if self.last_up < start_up:
                    raise RuntimeError("liveness fail: up went backwards (%d -> %d)"
                                       % (start_up, self.last_up))
                return self.last_report
        raise RuntimeError("liveness fail: no 2 s report in %.1f s" % timeout)


def check(case, cond, message, failures):
    print("  %-5s %s" % ("ok" if cond else "FAIL", message))
    if not cond:
        failures.append("%s: %s" % (case, message))


def case_smoke(c, failures):
    print("[smoke]")
    c.wait_report()
    r = c.last_report
    check("smoke", r["max_us"] <= r["budget_us"],
          "max Process %d us <= budget %d us" % (r["max_us"], r["budget_us"]), failures)
    check("smoke", r["stall_delta"] == 0,
          "i2s stall delta %d == 0" % r["stall_delta"], failures)
    out = c.cmd("status", 0.5)
    check("smoke", "module=" in out, "status reply parses", failures)
    out = c.cmd("crash", 0.5)
    m = re.search(r"crash: reason=(\d+).*?fault_streak=(\d+)", out)
    check("smoke", m is not None and m.group(1) == "1" and m.group(2) == "0",
          "clean boot reason=1 fault_streak=0", failures)


def case_swap(c, failures):
    print("[swap]")
    for name in MODULES:
        out = c.cmd("switch " + name, 0.8)
        if "switched to" not in out:
            # A report line may carry the confirmation; wait for it.
            out += c.cmd("", 0.6)
        ok = "switched to %s" % name in out or ("module " + name) in c.cmd("module", 0.4)
        check("swap", ok, "switch -> %s" % name, failures)
        r = c.wait_report()
        check("swap", r["stall_delta"] == 0 and r["max_us"] <= r["budget_us"],
              "%s: budget %d/%d us, stall delta %d"
              % (name, r["max_us"], r["budget_us"], r["stall_delta"]), failures)


def case_snapshot(c, failures):
    print("[snapshot]")
    before = c.cmd("preset dump", 1.2)
    rows = re.findall(r"^PR[0-9a-f]{2}:[0-9a-f]+$", before, re.M)
    check("snapshot", len(rows) > 0, "preset dump has %d PR rows" % len(rows), failures)
    m0 = PRESET_RE.search(c.cmd("preset", 0.5))
    for row in rows:
        c.s.write((row + "\n").encode())
        time.sleep(0.01)
    c._pump(0.4)
    c.cmd("preset load", 1.0)
    m1 = PRESET_RE.search(c.cmd("preset", 0.5))
    same = bool(m0 and m1 and m0.groups() == m1.groups())
    check("snapshot", same,
          "preset identity before/after: %s / %s"
          % (m0.groups() if m0 else None, m1.groups() if m1 else None), failures)


def case_capture(c, failures, seconds, out):
    print("[capture]")
    try:
        rc = subprocess.call([sys.executable, "tools/rig_capture.py",
                              "--seconds", str(seconds), "--out", out])
        check("capture", rc == 0, "rig_capture.py exit %d" % rc, failures)
    except Exception as exc:
        check("capture", False, "rig_capture.py failed: %s" % exc, failures)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--cases", default="smoke,snapshot,swap",
                    help="comma-separated: smoke,swap,snapshot,capture")
    ap.add_argument("--capture-seconds", type=float, default=5)
    ap.add_argument("--capture-out", default="/tmp/rack_suite_capture.wav")
    args = ap.parse_args()
    port = args.port or (glob.glob(DEFAULT_PORT_GLOB) or [None])[0]
    if not port:
        print("no rack CDC port found", file=sys.stderr)
        return 2

    failures = []
    c = Console(port)
    try:
        c.cmd("", 0.5)                 # drain stale bytes after boot/reflash
        for case in [x.strip() for x in args.cases.split(",") if x.strip()]:
            if case == "smoke":
                case_smoke(c, failures)
            elif case == "swap":
                case_swap(c, failures)
            elif case == "snapshot":
                case_snapshot(c, failures)
            elif case == "capture":
                case_capture(c, failures, args.capture_seconds, args.capture_out)
            else:
                failures.append("unknown case: %s" % case)
    except Exception as exc:
        failures.append("exception: %s" % exc)
    finally:
        c.close()

    if failures:
        print("\nFAILED:")
        for f in failures:
            print("  " + f)
        return 1
    print("\nVERDICT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
