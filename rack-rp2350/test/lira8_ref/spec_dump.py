#!/usr/bin/env python
"""Dump resolved wiring for LIRA-8 canvases + global s~/r~ bus table.

Usage: .venv/bin/python -I spec_dump.py [section ...]
Sections: buses, voice, delay, master, lfo, keyboard, prm, tri, gui, all
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
from spec_pdlib import (Parser, Inst, load_abstractions, instantiate_abs,
                        instantiate_tree)

HERE = Path(__file__).parent
P = Parser()
root = P.parse_file(HERE / "patch/_LIRA-8.pd")
ABS = load_abstractions(HERE / "patch/abs", P)

gui = root.children[0]  # pd gui


def sig_objs(inst: Inst):
    """objs as (idx, rendered) skipping comments"""
    return [(i, t) for i, (o, t) in enumerate(inst.objs) if o.kind != "text"]


def ep_label(inst: Inst, ep):
    kind, ref = ep
    if kind == 'o':
        return f"[{ref}]"
    return f"<sub{ref}:{inst.canvas.children[ref].name}>"


def dump_canvas(inst: Inst, title: str, resolve_abs=True):
    print(f"\n===== {title}  ({inst.canvas.file.split('/')[-1]}"
          f"{':' + str(inst.canvas.header_line) if inst.canvas.header_line else ''}) "
          f"args={inst.args} =====")
    for i, t in sig_objs(inst):
        line = inst.objs[i][0].line
        print(f"  [{i:3d}] L{line:<5} {t}")
    for (s, so, d, di) in inst.conns:
        st = inst.obj_text(s[1]) if s[0] == 'o' else ep_label(inst, s)
        dt = inst.obj_text(d[1]) if d[0] == 'o' else ep_label(inst, d)
        print(f"  conn {ep_label(inst, s)}({so}) -> {ep_label(inst, d)}({di})"
              f"   # {st} -> {dt}")
    # recurse into subpatch children (not abstractions; those are explicit)
    for pos, (ch, ci) in enumerate(inst.children):
        dump_canvas(ci, f"{title}/sub{pos}:{ch.name}")


def walk_gui():
    """instantiate gui tree incl. abstraction instances"""
    gi = Inst(gui, [])
    # abstraction instances inside gui
    for i, (o, t) in enumerate(gi.objs):
        if o.kind == "obj" and t.split() and t.split()[0] in ABS:
            instantiate_abs(gi, ABS, i, t)
    return gi


gi = walk_gui()

# lira.voice instances (4) with tags
voices = {}
for i, (o, t) in enumerate(gi.objs):
    if t.startswith("lira.voice"):
        tag = " ".join(t.split()[1:])
        voices[i] = (i, instantiate_abs(gi, ABS, i, t, tag))


def find(inst: Inst, name: str):
    for ch, ci in inst.children:
        if ch.name == name:
            return ci
    return None


SECTIONS = sys.argv[1:] or ["all"]


def want(name):
    return "all" in SECTIONS or name in SECTIONS


if want("buses"):
    # ---- global bus table -------------------------------------------------
    from collections import defaultdict
    sends = defaultdict(list)
    recvs = defaultdict(list)

    def scan(inst: Inst, path: str):
        for i, (o, t) in enumerate(inst.objs):
            if o.kind != "obj":
                continue
            toks = t.split()
            if not toks:
                continue
            k = toks[0]
            full = f"{path}[{i}]"
            if k in ("s", "send") and len(toks) > 1:
                sends[toks[1]].append((full, k))
            elif k in ("s~",) and len(toks) > 1:
                sends[toks[1]].append((full, k))
            elif k in ("r", "receive") and len(toks) > 1:
                recvs[toks[1]].append((full, k))
            elif k in ("r~",) and len(toks) > 1:
                recvs[toks[1]].append((full, k))
            # msg boxes: 'send <name>' / 'receive <name>' configs
            elif o.kind == "msg" and toks and toks[0] in ("send", "receive"):
                (sends if toks[0] == "send" else recvs)[toks[1]].append(
                    (full, "msg"))
            elif o.kind == "obj" and k in ("vsl", "hsl", "tgl", "hradio",
                                           "vradio", "nbx", "bng"):
                # GUI atoms: this patch names send symbols '$0-s-...' and
                # receive symbols '$0-r-...'; match prefixes over all args.
                for tk in toks[1:]:
                    if tk.startswith("$0-s-"):
                        sends[tk].append((full, k))
                    elif tk.startswith("$0-r-"):
                        recvs[tk].append((full, k))
            elif k == "v" and len(toks) > 1:  # value storage: read+write
                recvs[toks[1]].append((full, "v(read)"))
                sends[toks[1]].append((full, "v(write)"))
        for ch, ci in inst.children:
            scan(ci, f"{path}/{ch.name}")
        for idx, ai in inst.abs_instances.items():
            scan(ai, f"{path}@{idx}:{ai.name}")

    root_inst = Inst(root, [])
    instantiate_tree(root_inst, ABS)
    scan(root_inst, "root")

    print("=" * 100)
    print("GLOBAL BUS TABLE (after $-substitution)")
    print("=" * 100)
    for name in sorted(set(sends) | set(recvs)):
        print(f"\n--- {name}")
        for p, k in sends.get(name, []):
            print(f"    send    {k:8s} {p}")
        for p, k in recvs.get(name, []):
            print(f"    recv    {k:8s} {p}")
        if not sends.get(name) or not recvs.get(name):
            tag = "NO-SENDER" if not sends.get(name) else "NO-RECEIVER"
            print(f"    ({tag})")

if want("voice"):
    for idx, (gi_idx, v) in sorted(voices.items()):
        dump_canvas(v, f"lira.voice instance {v.tag} (gui obj {gi_idx})")
        for ch, ci in v.children:
            dump_canvas(ci, f"  {v.tag}/{ch.name}")
        for aidx, ai in sorted(v.abs_instances.items()):
            dump_canvas(ai, f"  {v.tag}@{aidx} {ai.name} args={ai.args}")
            for a2, a2i in ai.abs_instances.items():
                dump_canvas(a2i, f"    {v.tag}@{aidx}@{a2} {a2i.name} args={a2i.args}")

if want("delay"):
    dump_canvas(find(gi, "delay"), "pd delay")
    # compressor/expander instances inside delay
    d = find(gi, "delay")
    for aidx, ai in sorted(d.abs_instances.items()):
        dump_canvas(ai, f"  delay@{aidx} {ai.name} args={ai.args}")
        for a2, a2i in ai.abs_instances.items():
            dump_canvas(a2i, f"    delay@{aidx}@{a2} {a2i.name} args={a2i.args}")

if want("master"):
    dump_canvas(find(gi, "master_output"), "pd master_output")
    m = find(gi, "master_output")
    for aidx, ai in sorted(m.abs_instances.items()):
        dump_canvas(ai, f"  master@{aidx} {ai.name}")

if want("lfo"):
    dump_canvas(find(gi, "hyper-lfo"), "pd hyper-lfo")
    h = find(gi, "hyper-lfo")
    for ch, ci in h.children:
        dump_canvas(ci, f"  hyper-lfo/{ch.name}")

if want("keyboard"):
    dump_canvas(find(gi, "keyboard"), "pd keyboard")
    k = find(gi, "keyboard")
    for aidx, ai in sorted(k.abs_instances.items()):
        dump_canvas(ai, f"  keyboard@{aidx} {ai.name} args={ai.args}")

if want("prm"):
    # prm.hold subpatches (canvas children of gui), prm.pitch/prm.source instances
    for ch, ci in gi.children:
        if ch.name == "prm.hold":
            dump_canvas(ci, f"pd prm.hold (r {ci.objs[2][1]})")
    for i, (o, t) in enumerate(gi.objs):
        if t.startswith("prm.pitch") or t.startswith("prm.source"):
            ai = gi.abs_instances.get(i)
            if ai:
                dump_canvas(ai, f"gui@{i} {t}")

if want("tri"):
    # os.triangle~ instances (first voice of first lira.voice is enough)
    for gi_idx, (i, v) in sorted(voices.items()):
        for aidx, ai in sorted(v.abs_instances.items()):
            if ai.name == "os.triangle~":
                dump_canvas(ai, f"{v.tag}@{aidx} os.triangle~ args={ai.args}")
                for ch, ci in ai.children:
                    dump_canvas(ci, f"  {ai.name}/{ch.name}")
                break  # one per voice
        break  # just voice 1

if want("gui"):
    dump_canvas(gi, "pd gui (main synth canvas)")
