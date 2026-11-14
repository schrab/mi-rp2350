#!/usr/bin/env python
"""Minimal but faithful Pd patch parser for the LIRA-8 porting spec.

Parses #N canvas/#X obj/msg/text/... statements (multi-line, ';'-terminated),
builds the canvas tree, keeps ORIGINAL per-canvas object indices for
#X connect (comments/texts count, unlike hvcc), and resolves $-args per
abstraction instantiation chain.

$-substitution model:
  * object boxes: $0 -> "$0" (kept literal), $n -> nth creation arg.
  * message boxes: runtime $n rendered as {n}; "\\$" unescaped to "$".
"""
from __future__ import annotations
import re
from pathlib import Path

SUBPATCH_HDR = 8  # tokens: #N canvas x y w h name fs


def sub_obj(atom: str, args: list[str]) -> str:
    def repl(m):
        if m.group(1) == "0":
            return "$0"
        i = int(m.group(1))
        if 1 <= i <= len(args):
            return args[i - 1]
        return m.group(0)
    return re.sub(r"\$(\d+)", repl, atom)


def sub_msg(atom: str) -> str:
    a = re.sub(r"\$(\d+)", lambda m: "{%d}" % int(m.group(1)), atom)
    return a


class Obj:
    __slots__ = ("idx", "pd_idx", "kind", "text", "x", "y", "line")

    def __init__(self, idx, kind, text, x, y, line):
        self.idx, self.kind, self.text = idx, kind, text
        self.pd_idx = None  # raw-file index space (subpatches consume slots)
        self.x, self.y, self.line = x, y, line

    def render(self, args):
        if self.kind == "msg":
            return sub_msg(self.text)
        return sub_obj(self.text, args)

    def __repr__(self):
        return f"[{self.idx}] {self.text}"


class Conn:
    __slots__ = ("src", "outlet", "dst", "inlet")

    def __init__(self, s, so, d, di):
        self.src, self.outlet, self.dst, self.inlet = s, so, d, di


class Canvas:
    def __init__(self, name, x, y, parent, file, header_line):
        self.name = name
        self.x, self.y = x, y
        self.parent = parent
        self.file = file
        self.header_line = header_line
        self.objs: list[Obj] = []
        # conns: (src, outlet, dst, inlet); src/dst = ('o', parser_idx)
        # for an object or ('s', child_pos) for a subpatch slot.
        self.conns: list[tuple] = []
        self.children: list[Canvas] = []
        self.arrays: list[str] = []

    def path(self):
        p, c = [], self
        while c:
            p.append(c.name)
            c = c.parent
        return "/".join(reversed(p))

    def by_idx(self, i):
        return self.objs[i]


class Parser:
    def __init__(self):
        self.roots: dict[str, Canvas] = {}

    def parse_file(self, path: Path) -> Canvas:
        raw = Path(path).read_text(errors="replace")
        root = Canvas(Path(path).stem, 0, 0, None, str(path), 1)
        self._build(root, raw)
        self.roots[root.name] = root
        return root

    def _build(self, root: Canvas, raw: str):
        stack: list[Canvas] = [root]
        cur = root
        idx = {id(root): 0}      # parser index (objects only)
        pdx = {id(root): 0}      # raw-file index space (subpatches take slots)
        pd2idx = {id(root): {}}  # pd_idx -> parser idx
        stmt: list[str] = []
        stmt_line = None

        def flush():
            nonlocal stmt, stmt_line, cur
            if not stmt:
                return
            text = " ".join(stmt)
            ln = stmt_line
            stmt, stmt_line = [], None
            t = text.replace(";", " ; ").split()
            if not t:
                return
            head = f"{t[0]} {t[1]}" if len(t) > 1 else t[0]
            if head == "#N canvas":
                if len(t) == SUBPATCH_HDR + 1:  # named subpatch: x y w h name fs
                    sub = Canvas(t[6], int(t[2]), int(t[3]), cur, cur.file, ln)
                    cur.children.append(sub)
                    # a subpatch consumes ONE pd index slot in its parent
                    pd2idx[id(cur)][pdx[id(cur)]] = ('s', len(cur.children) - 1)
                    pdx[id(cur)] += 1
                    stack.append(cur)
                    cur = sub
                    idx[id(sub)] = 0
                    pdx[id(sub)] = 0
                    pd2idx[id(sub)] = {}
                # else: file's own root header -> ignore (root already made)
            elif head == "#X restore":
                if len(stack) > 1:
                    cur = stack.pop()
            elif len(t) >= 4 and t[0] == "#X" and t[1] in (
                    "obj", "msg", "text", "float", "symbol", "list"):
                kind = t[1]
                x, y = int(t[2]), int(t[3])
                body = " ".join(t[4:-1])
                # .pd file format escapes: \$ -> literal '$' atom
                body = body.replace("\\$", "$")
                i = idx[id(cur)]
                idx[id(cur)] = i + 1
                o = Obj(i, kind, body, x, y, ln)
                o.pd_idx = pdx[id(cur)]
                pd2idx[id(cur)][o.pd_idx] = ('o', i)
                pdx[id(cur)] += 1
                cur.objs.append(o)
            elif len(t) >= 5 and t[0] == "#X" and t[1] == "connect":
                m = pd2idx[id(cur)]
                s = m.get(int(t[2]))
                d = m.get(int(t[4]))
                if s is None or d is None:
                    return  # unknown slot
                cur.conns.append((s, int(t[3]), d, int(t[5])))
            elif t[0] == "#A":
                cur.arrays.append(" ".join(t[1:-1]))
            # everything else (coords, declare, scalar...) ignored

        for i, ln in enumerate(raw.splitlines(), 1):
            if not ln.strip():
                continue
            if stmt_line is None:
                stmt_line = i
            stmt.append(ln)
            if ln.rstrip().endswith(";"):
                flush()
        flush()


# ---- instantiation resolution ----------------------------------------------

class Inst:
    """A resolved canvas instance: name, args, resolved objects, connections,
    child instances."""

    def __init__(self, canvas: Canvas, args: list[str], parent=None, tag=""):
        self.canvas = canvas
        self.args = args
        self.parent = parent
        self.tag = tag  # disambiguator e.g. "voice 1-2"
        self.objs = [(o, o.render(args)) for o in canvas.objs]
        self.conns = canvas.conns
        self.children: list[tuple[Canvas, "Inst"]] = []
        self.name = canvas.name
        # build children: subpatches inherit args; abstractions get new args
        for ch in canvas.children:
            self.children.append((ch, Inst(ch, args, self, tag)))
        self.abs_instances: dict[int, Inst] = {}

    def obj_text(self, i):
        return self.objs[i][1]

    def path(self):
        p, c = [], self
        while c:
            p.append(c.name)
            c = c.parent
        return "/".join(reversed(p))


def load_abstractions(abs_dir: Path, parser: Parser):
    abs_map = {}
    for f in sorted(Path(abs_dir).glob("*.pd")):
        abs_map[f.stem] = parser.parse_file(f)
    return abs_map


def instantiate_abs(inst: Inst, abs_map: dict, obj_idx: int, text: str,
                    tag="") -> Inst:
    """obj text like 'lira.voice $0 1 2' -> resolved Inst of the abstraction."""
    toks = text.split()
    stem = toks[0]
    raw_args = toks[1:]
    args = [sub_obj(a, inst.args) for a in raw_args]
    sub = Inst(abs_map[stem], args, inst, tag)
    inst.abs_instances[obj_idx] = sub
    _recurse_abs(sub, abs_map)
    return sub


def _recurse_abs(inst: Inst, abs_map: dict):
    for i, (o, txt) in enumerate(inst.objs):
        if o.kind == "obj" and txt.split() and txt.split()[0] in abs_map:
            instantiate_abs(inst, abs_map, i, txt)


def instantiate_tree(inst: Inst, abs_map: dict):
    """Instantiate known abstractions at this level and in every child canvas."""
    _recurse_abs(inst, abs_map)
    for _, ci in inst.children:
        instantiate_tree(ci, abs_map)
