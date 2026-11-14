#!/usr/bin/env python
"""Drive the LIRA-8 reference patch over the l8bridge TCP channel and render
a scenario to a WAV via writesf~. pd must be running with _LIRA-8-golden.pd.

    drive_ref.py <scenario-name>
"""
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).parent
PORT = 7001


def parse_scenario(name):
    events = []
    active = False
    for ln in (HERE / "scenarios.txt").read_text().splitlines():
        ln = ln.strip()
        if ln.startswith("SCEN "):
            active = (ln.split()[1] == name)
            continue
        if not active or not ln or ln.startswith("#"):
            continue
        t, rest = ln.split(None, 1)
        events.append((int(t), rest))
    if not events:
        sys.exit(f"scenario '{name}' not found")
    return events


def main():
    name = sys.argv[1]
    outdir = HERE / "goldens"
    outdir.mkdir(exist_ok=True)
    events = parse_scenario(name)
    s = socket.create_connection(("127.0.0.1", PORT))
    t0 = time.monotonic()
    for t_ms, msg in events:
        wait = t_ms / 1000.0 - (time.monotonic() - t0)
        if wait > 0:
            time.sleep(wait)
        if msg.startswith("render open ") and not msg.startswith("render open /"):
            msg = "render open " + str(outdir / (name + ".wav"))
        s.send((msg + ";\n").encode())
    time.sleep(0.8)
    s.send(b"render stop;\n")
    time.sleep(0.3)
    s.send(b"quit;\n")
    s.close()
    print(f"scenario '{name}' done -> goldens/{name}.wav")


if __name__ == "__main__":
    main()
