#!/usr/bin/env python
"""Generate a write-safe golden patch with a writesf tap and GUI l8bridge.

Root indices (gui subpatch counts as obj 0):
0 gui, 1 tgl, 2-4 text, 5 nbx, 6 dac~, 7 msg, 8-9 text ->
appended: 10 writesf~, 11 r l8-render-ctl.
"""
from pathlib import Path

HERE = Path(__file__).parent


def replace_unique(text, anchor, replacement, label):
    count = text.count(anchor)
    if count != 1:
        raise ValueError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(anchor, replacement, 1)


def main():
    base = (HERE / "patch" / "_LIRA-8.pd").read_text(errors="replace")

    # gui.link: object 55 is del 500; object 30 is write ./parameters.txt.
    # Remove only their edge, leaving all objects/indices and the parameter
    # read/load wiring intact. Fail closed if the source patch has drifted.
    gui_link_start = "#N canvas 191 498 1414 497 gui.link 0;\n"
    gui_link_end = "#X restore 29 750 pd gui.link;"
    for anchor in (gui_link_start, gui_link_end):
        if base.count(anchor) != 1:
            raise ValueError("gui.link: expected exactly one canvas boundary")
    before, gui_link = base.split(gui_link_start)
    gui_link, after = gui_link.split(gui_link_end)
    gui_link = replace_unique(gui_link, "#X connect 55 0 30 0;\n", "",
                              "delayed parameter-write connection")
    for line in gui_link.splitlines():
        fields = line.split()
        if fields[:2] == ["#X", "connect"] and fields[4] == "30":
            raise ValueError(f"unexpected parameter-write input: {line}")
    base = before + gui_link_start + gui_link + gui_link_end + after

    # The DSP receivers (prm.tune/hold/sensor...) live INSIDE the "gui"
    # subpatch — their $0 is the GUI canvas's, not the root's. The bridge
    # must be instantiated inside gui so its $1 resolves to the right
    # namespace. Insert it just before the gui canvas's #X restore.
    gui_restore = "#X restore 32 32 pd gui;"
    bridge_inside_gui = "#X obj 900 1100 l8bridge \\$0;\n"
    base = replace_unique(base, gui_restore, bridge_inside_gui + gui_restore,
                          "gui restore")

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
