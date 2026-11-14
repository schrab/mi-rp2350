#!/usr/bin/env python
"""Generate patch/abs/l8bridge.pd — the reference-patch control bridge.

Vanilla pd only: [netreceive 7001] (TCP) -> route -> named sends into the
host patch ($1 = the host canvas's $0, passed as creation arg).
Message protocol (semicolon-terminated lines over TCP):
  sensor N v          -> s $1-s-sensor-N
  param NAME V        -> s $1-s-NAME   (NAME from the 47-param list)
  render start PATH   -> s l8-render-ctl (root [r] feeds writesf~)
  render stop         -> s l8-render-ctl
  quit                -> ; pd quit
"""
from pathlib import Path

NAMES = (["tune-%d" % i for i in range(1, 9)]
         + ["fast-12", "fast-34", "fast-56", "fast-78"]
         + ["sharp-12", "sharp-34", "sharp-56", "sharp-78"]
         + ["mod-12", "mod-34", "mod-56", "mod-78"]
         + ["source-12", "source-34", "source-56", "source-78"]
         + ["pitch-1234", "pitch-5678", "hold-1234", "hold-5678"]
         + ["switch", "total-fb", "vibrato", "drv", "dst-mix", "vol",
            "f-a", "f-b", "andor", "link"]
         + ["time-1", "time-2", "feedback", "del-mix",
            "mod-1", "mod-2", "del-mod", "lfo-wav", "quantize"])
assert len(NAMES) == 47

objs = []  # (pd_line)
conns = []  # (from, outlet, to, inlet)

objs.append("#X obj 30 30 netreceive 7001;")                        # 0
objs.append("#X obj 30 90 route sensor param render quit;")         # 1
objs.append("#X obj 30 150 route 1 2 3 4 5 6 7 8;")                 # 2
for i in range(1, 9):                                               # 3-10
    objs.append("#X obj %d 150 s \\$1-s-sensor-%d;" % (30 + i * 80, i))
objs.append("#X obj 30 220 route %s;" % " ".join(NAMES))            # 11
for k, n in enumerate(NAMES):                                       # 12-58
    objs.append("#X obj 30 %d s \\$1-s-%s;" % (280 + k * 24, n))
objs.append("#X obj 900 90 s l8-render-ctl;")                      # 59 (render passes start/stop verbs intact)
objs.append("#X msg 900 170 \\; pd quit;")                          # 61

conns += [(0, 0, 1, 0), (1, 0, 2, 0), (1, 1, 11, 0), (1, 2, 59, 0),
          (1, 3, 60, 0)]
for i in range(8):
    conns.append((2, i, 3 + i, 0))
for k in range(47):
    conns.append((11, k, 12 + k, 0))

lines = ["#N canvas 0 22 1200 900 l8bridge 0;"] + objs + \
        ["#X connect %d %d %d %d;" % c for c in conns]
out = Path(__file__).parent / "patch" / "abs" / "l8bridge.pd"
out.write_text("\n".join(lines) + "\n")
print(f"generated {out}")
