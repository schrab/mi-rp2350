#!/usr/bin/env python
"""Generate patch/_LIRA-8-golden.pd: pristine patch + writesf tap +
l8bridge instance. Root indices (gui subpatch counts as obj 0):
0 gui, 1 tgl, 2-4 text, 5 nbx, 6 dac~, 7 msg, 8-9 text ->
appended: 10 writesf~, 11 l8bridge, 12 r l8-render-ctl.
"""
import sys
from pathlib import Path

HERE = Path(__file__).parent

def main():
    base = (HERE / "patch" / "_LIRA-8.pd").read_text(errors="replace")
    seq = [
        "#X obj 32 780 writesf~ 2;",               # 10
        "#X obj 500 700 l8bridge \\$0;",           # 11
        "#X obj 500 760 r l8-render-ctl;",         # 12
        "#X connect 0 0 10 0;",
        "#X connect 0 0 10 1;",
        "#X connect 12 0 10 0;",
    ]
    out = HERE / "patch" / "_LIRA-8-golden.pd"
    out.write_text(base + "\n" + "\n".join(seq) + "\n")
    print(f"generated {out}")

if __name__ == "__main__":
    main()
