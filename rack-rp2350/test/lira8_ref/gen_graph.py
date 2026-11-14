#!/usr/bin/env python
"""Dump the LIRA-8 Pd patch graph to JSON via hvcc's pd2hv parser.

The full hvcc pipeline aborts on the GUI subpatch's unsupported objects
(24x `v` value objects etc.); here we keep the parse result anyway (the
DSP subpatches parse fine), dump graph.json, and re-join subpatch names
that hvcc drops.

Name join: every graph node in the .hv JSON carries the (x, y) window
geometry of its source `#N canvas` header, but NOT the name. We collect
all canvas headers from all patch files (inline subpatches + abstraction
top-level headers -> file stem) into a geometry -> [candidate names] map,
then disambiguate candidates by object-census fingerprint (dot product of
type counters between the .pd file and the JSON node).
"""
import collections
import json
from pathlib import Path

from hvcc.interpreters.pd2hv.PdParser import PdParser

HERE = Path(__file__).parent
PATCH_DIR = HERE / "patch"
patch = PATCH_DIR / "_LIRA-8.pd"
out_json = HERE / "graph.json"

parser = PdParser()
parser.add_absolute_search_directory(PATCH_DIR / "abs")
graph = parser.graph_from_file(patch)
notices = graph.get_notices()
graph_dict = graph.to_hv().model_dump(by_alias=True, mode="json")

PD_FILES = [patch, *sorted((PATCH_DIR / "abs").glob("*.pd"))]

# geometry -> {name: fingerprint Counter}; also every file's own header
# maps to its stem so abstraction-instance nodes get named.
geom = collections.defaultdict(dict)


def file_census(lines, own_name):
    """(geometry->name registration, object-type Counter) for one .pd file."""
    cens = collections.Counter()
    headers = []  # (i, x, y, name)
    for i, ln in enumerate(lines):
        t = ln.split()
        if ln.startswith("#N canvas ") and len(t) >= 7:
            name = own_name if i == 0 else t[6]
            headers.append((i, int(t[2]), int(t[3]), name))
        elif ln.startswith("#X obj "):
            cens[t[3]] += 1
        elif ln.startswith("#X msg"):
            cens["message"] += 1
        elif ln.startswith("#X text"):
            cens["comment"] += 1
        elif ln.startswith("#X inlet"):
            cens["inlet"] += 1
        elif ln.startswith("#X inlet~"):
            cens["inlet~"] += 1
        elif ln.startswith("#X outlet~"):
            cens["outlet~"] += 1
        elif ln.startswith("#X outlet"):
            cens["outlet"] += 1
        elif ln.startswith("#X restore"):
            cens["graph"] += 1
    # second pass: abstraction instances (obj type == a known file stem)
    stems = {f.stem for f in PD_FILES} | {h[3] for h in headers}
    for ln in lines:
        t = ln.split()
        if ln.startswith("#X obj ") and t[3] in stems:
            cens["graph"] += 1
            cens[t[3]] -= 1
    for _, x, y, name in headers:
        geom[(x, y)][name] = dict(cens)
    return cens


FILE_CENSUS = {}
for f in PD_FILES:
    lines = f.read_text(errors="replace").splitlines()
    FILE_CENSUS[f.name] = file_census(lines, f.stem)


def score(fp, node_cens):
    return sum(v * node_cens.get(k, 0) for k, v in fp.items())


def node_census(node):
    c = collections.Counter()
    for o in node.get("objects", {}).values():
        c[o.get("type", "?")] += 1
    c.pop("graph", None)  # counted separately below
    for o in node.get("objects", {}).values():
        if o.get("type") == "graph":
            c["graph"] += 1
    return dict(c)


AMBIGUOUS = []


def annotate(node):
    p = node.get("properties", {})
    cands = geom.get((p.get("x"), p.get("y")), {})
    if cands:
        nc = node_census(node)
        best = max(cands.items(), key=lambda kv: score(kv[1], nc))
        # flag ties among DIFFERENT names
        top = score(best[1], nc)
        ties = {n for n, fp in cands.items() if score(fp, nc) == top}
        if len(ties) > 1:
            AMBIGUOUS.append((ties, p.get("x"), p.get("y")))
        node["annotations"]["@pd_name"] = best[0]
    for o in node.get("objects", {}).values():
        if o.get("type") == "graph":
            annotate(o)


annotate(graph_dict)
out_json.write_text(json.dumps(graph_dict, separators=(",", ":")))

print(f"objects parsed : {sum(parser.obj_counter.values())} total")
print(f"graph.json     : {out_json} ({out_json.stat().st_size} bytes)")
print(f"notice errors  : {len(notices.errors)}, warnings: {len(notices.warnings)}")
print(f"geometry keys  : {len(geom)}, ambiguous name ties: {len(AMBIGUOUS)}")
for ties, x, y in AMBIGUOUS[:10]:
    print(f"  @({x},{y}): {sorted(ties)}")


def msg_text(m):
    return getattr(m, "message", None) or str(m)


groups = collections.Counter(
    msg_text(m).split(" in ")[0][:90] for m in notices.errors)
for msg, n in groups.most_common(20):
    print(f"  {n:4d}x {msg}")
