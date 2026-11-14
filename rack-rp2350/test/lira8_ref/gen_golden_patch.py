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

    # The DSP receivers (prm.tune/hold/sensor...) live INSIDE the "gui"
    # subpatch — their $0 is the GUI canvas's, not the root's. The bridge
    # must be instantiated inside gui so its $1 resolves to the right
    # namespace. Insert it just before the gui canvas's #X restore.
    gui_restore = "#X restore 32 32 pd gui;"
    assert gui_restore in base, "gui restore anchor missing"
    bridge_inside_gui = "#X obj 900 1100 l8bridge \\$0;\n"
    base = base.replace(gui_restore, bridge_inside_gui + gui_restore, 1)

    seq = [
        "#X obj 32 780 writesf~ 2;",               # root obj 10
        "#X obj 500 760 r l8-render-ctl;",         # root obj 11
        "#X connect 0 0 10 0;",
        "#X connect 0 0 10 1;",
        "#X connect 11 0 10 0;",
    ]
    out = HERE / "patch" / "_LIRA-8-golden.pd"
    out.write_text(base + "\n" + "\n".join(seq) + "\n")
    print(f"generated {out} (bridge inside gui)")

if __name__ == "__main__":
    main()
