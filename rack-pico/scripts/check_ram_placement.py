#!/usr/bin/env python3
"""Verify the rack-pico ELF's hot-code placement and memory budget.

Adapted from DSPi's scripts/check_ram_placement.py (GPL-3.0; the structure
and the objdump/nm parsing are shared, the symbol tables and the checks are
the rack's). Exits non-zero on any FAIL.

Checks against one linked ELF:
  A  the tc_* module hot-code overlay must sit at the RACK_WINDOW VMA
     (tc_clouds at window+0xEC00, the A-slot members at the window origin)
     and every member must fit its slot — the runtime swap copies these at
     block boundaries, a misplaced member is a swap-corrupting fault
  A2 platform hot symbols (the I2S DMA bank driver, the UI-link transport,
     the transport IRQs, the audio core-1 loop) must be RAM-resident — XIP
     execution is acceptable for cold paths (the RP2350 does no runtime
     flash writes; the presets live on the C3), so flash branching from hot
     code is report-only (Check B), not a failure
  B  BFS over the RAM closure from the hot roots: flash edges reported
     (INFO/WARN) so a regression is visible without blocking the build
  B2 core-1 (loop1 + the swap/chain paths) must hold no flash-range literals
     it would dereference during a swap — the swap copies the window while
     core 1 is masked, but a stale flash pointer in a masked window would
     corrupt the copy (same shape as DSPi's flash-write window check)
  C  RAM/flash size accounting: .data/.bss vs the RAM region, the heap slack
     (>= 8 KiB gate: the I2S DMA bank + the TinyUSB FIFOs malloc at runtime)

Usage: python3 scripts/check_ram_placement.py <elf>
Requires arm-none-eabi-nm / -objdump / -size on PATH (stdlib only).
"""

import bisect
import json
import os
import re
import subprocess
import sys

# ---------------------------------------------------------------------------
# Address map (memmap_rack_pico.ld + linker_overrides/memory_extra.incl)
# ---------------------------------------------------------------------------
FLASH_LO, FLASH_HI = 0x10000000, 0x20000000   # XIP flash window (incl. aliases)
RAM_LO = 0x20000000
RACK_WINDOW = 0x20063000                       # the hot-code window origin
RACK_WINDOW_LEN = 0x1D000                      # 116 KiB
RACK_CLOUDS_B_OFFSET = 0xEC00                  # the chain-B slot offset
SCRATCH_X = 0x20080000                         # the window must end exactly here
RAM_HI = SCRATCH_X                             # valid main RAM above the window
ROM_HI = 0x00008000                            # bootrom below here is always fine
RAM_LEN = 0x63000                              # the RAM region (below the window)

# The tc_* overlay members: (section, slot). The A-slot members share ONE
# VMA (the window origin); tc_clouds owns the B slot at +RACK_CLOUDS_B_OFFSET.
TC_A_MEMBERS = ["tc_elements", "tc_plaits", "tc_rings", "tc_lira8",
                "tc_dronecaster", "tc_benjolin"]
TC_B_MEMBER = "tc_clouds"

# Platform hot symbols: (name, static_ok). static_ok => "not found" is a
# WARN (inlined) rather than a FAIL. One-line additions keep this table the
# single source of truth.
HOT_COMMON = [
    # The I2S DMA bank driver (the AudioBufferManager replacement): the
    # write16 loop paces the render, the IRQ hands buffers back.
    ("_ZN10RackI2sOut7write16Ess", True),          # RackI2sOut::write16(short, short)
    ("_ZN10RackI2sOut8handoff_Ev", True),          # the bank handoff (the pacing point)
    ("_ZN10RackI2sOut8dma_irq_Ev", True),          # the DMA completion IRQ
    # The UI-link transport (the IRQ feeds the RX ring; the drain paces TX).
    ("_ZL11ui_uart_irqv", True),              # ui_uart_irq() (static)
    ("_ZL12din_uart_irqv", True),            # din_uart_irq() (static)
    ("ui_uart_tx_drain", True),
    ("ui_uart_tx_service", True),
    # The console + diagnostics rings (drained every loop pass).
    ("dbg_ring_write", True),
    # The MIDI funnel + the mod engine (per-block on core 1).
    # Core-1 audio loop + the swap/chain machinery it drives.
    ("_Z6setup1v", True),                          # setup1()
    ("_Z16chain_render_blockv", True),             # chain_render_block()
    ("_Z22chain_render_block_pipev", True),        # chain_render_block_pipe()
    ("_Z14chain_b_workerv", True),                 # chain_b_worker()
    ("_Z18chain_pipe_startv", True),               # chain_pipe_start()
    ("_Z16do_module_swaph", True),                 # do_module_swap(unsigned char)
    ("_Z16rack_window_copyhPmS_", True),           # rack_window_copy(...)
    ("_Z15rack_checksum32PKvm", True),             # rack_checksum32(...)
    # The audio_ready poll + the capture feed (core 1, per block).
    ("usb_capture_push_stereo", True),  # extern "C" — C symbol, not mangled
    ("_ZL17fill_audio_packetPh", True),            # fill_audio_packet(unsigned char*)
]

# Flash callees that are cold-path-only and safe for a hot caller to reference.
WHITELIST = {
    "panic", "panic_unsupported", "__assert_func", "hard_assert",
    "isr_hardfault", "crash_record_c", "__wrap_printf", "weak_raw_printf",
    "stdio_flush",
}

# Core-1 roots for the literal-pool scan (Check B2): loop1 + everything it
# can reach into the window/swap paths.
CORE1_ROOTS = ["_Z5loop1v", "_Z6setup1v"]

# Flash literals in core-1 code that are provably init-time only.
B2_LITERAL_WHITELIST = {
    "__VECTOR_TABLE", "__vectors",
    # RAM-resident setup1() calls the flash-resident SDK irq_set_enabled();
    # the linker emits a long-call veneer whose .word is a BRANCH TARGET,
    # not a dereferenced data pointer. No swap-window read of flash occurs.
    "irq_set_enabled",
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def in_ram(a):
    return RAM_LO <= a < RAM_HI

def in_window(a):
    return RACK_WINDOW <= a < SCRATCH_X

def in_flash(a):
    return FLASH_LO <= a < FLASH_HI

def run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    except FileNotFoundError:
        sys.exit("error: %s not found; arm-none-eabi binutils must be on PATH" % cmd[0])

def load_symbols(elf):
    out = run(["arm-none-eabi-nm", "--print-size", "--defined-only", elf])
    syms = []
    for line in out.splitlines():
        t = line.split()
        if len(t) >= 4 and re.fullmatch(r"[0-9a-fA-F]+", t[1] or ""):
            addr, size, name = int(t[0], 16), int(t[1], 16), " ".join(t[3:])
        elif len(t) == 3:
            addr, size, name = int(t[0], 16), 0, t[2]
        else:
            continue
        syms.append({"name": name, "addr": addr, "size": size})
    return syms

def match_symbols(syms, name):
    return [s for s in syms if s["name"] == name or s["name"].startswith(name + ".")]

def nearest_symbol(sorted_addrs, addr):
    a = addr & ~1
    i = bisect.bisect_right(sorted_addrs, (a, "￿")) - 1
    return sorted_addrs[i][1] if i >= 0 else ""

# ---------------------------------------------------------------------------
# Disassembly parsing (objdump -d --no-show-raw-insn) — same as DSPi's
# ---------------------------------------------------------------------------

LABEL_RE = re.compile(r"^([0-9a-fA-F]+) <(.+)>:$")
INSN_RE = re.compile(r"^([0-9a-fA-F]+):\t(\S+)\t?(.*)$")
TARGET_RE = re.compile(r"^([0-9a-fA-F]+)\s+<([^>]+)>")
BRANCH_MNEMONICS = {"bl", "blx", "b", "b.w", "b.n"}

def parse_disasm(elf):
    out = run(["arm-none-eabi-objdump", "-d", "--no-show-raw-insn", elf])
    funcs = {}
    cur = None
    for line in out.splitlines():
        m = LABEL_RE.match(line)
        if m:
            start = int(m.group(1), 16)
            cur = {"name": m.group(2), "start": start, "end": start,
                   "insns": [], "words": []}
            funcs[start] = cur
            continue
        m = INSN_RE.match(line)
        if not m or cur is None:
            continue
        addr, mnem, operand = int(m.group(1), 16), m.group(2), m.group(3).strip()
        cur["end"] = addr + 2
        if mnem == ".word":
            wm = re.match(r"0x([0-9a-fA-F]+)", operand)
            if wm:
                cur["words"].append((addr, int(wm.group(1), 16)))
        elif mnem in BRANCH_MNEMONICS:
            cur["insns"].append((addr, mnem, operand))
    starts = sorted(funcs)
    for i, s in enumerate(starts):
        funcs[s]["end"] = starts[i + 1] if i + 1 < len(starts) else s + funcs[s]["end"]
    return funcs, starts

def enclosing_func(starts, funcs, addr):
    i = bisect.bisect_right(starts, addr) - 1
    if i < 0:
        return None
    s = starts[i]
    if funcs[s]["start"] <= addr < funcs[s]["end"]:
        return s
    return None

def parse_target(operand):
    m = TARGET_RE.match(operand)
    if m:
        base = m.group(2).split("+")[0]
        return ("addr", int(m.group(1), 16), base)
    if re.match(r"^(r\d+|sl|fp|ip|lr|pc|sb)\b", operand):
        return ("indirect", None, None)
    return ("none", None, None)

def load_sections(elf):
    out = run(["arm-none-eabi-size", "-A", elf])
    sec = {}
    for line in out.splitlines():
        t = line.split()
        if len(t) >= 2 and t[0].startswith(".") and re.fullmatch(r"\d+", t[1]):
            sec[t[0]] = int(t[1])
    return sec

# ---------------------------------------------------------------------------
# Per-ELF driver
# ---------------------------------------------------------------------------

HEAP_GATE = 8192   # the I2S DMA bank + the TinyUSB FIFOs malloc at runtime

def check_elf(elf, baseline):
    lines = []
    fails = warns = 0

    lines.append("=" * 78)
    lines.append("ELF: %s" % elf)
    lines.append("=" * 78)

    syms = load_symbols(elf)
    funcs, starts = parse_disasm(elf)
    sec = load_sections(elf)
    sym_addr = {s["name"]: s["addr"] for s in syms}
    sym_by_addr = {s["addr"]: s for s in syms}
    sorted_addrs = sorted((s["addr"], s["name"]) for s in syms)

    # --- Check A: the tc_* hot-code window placement ------------------------
    lines.append("[Check A] tc_* overlay members must sit at the RACK_WINDOW VMA")
    a_fail = 0
    ram_roots = []
    window_start = sym_addr.get("__tc_elements_start", 0)
    if window_start == 0:
        # Fall back to any A-slot member's start symbol.
        for m in TC_A_MEMBERS:
            v = sym_addr.get("__%s_start" % m, 0)
            if v:
                window_start = v
                break
    if window_start != RACK_WINDOW:
        lines.append("  FAIL  tc_* A-slot VMA 0x%08x != RACK_WINDOW 0x%08x"
                     % (window_start, RACK_WINDOW))
        a_fail += 1
    for m in TC_A_MEMBERS:
        st = sym_addr.get("__%s_start" % m, 0)
        en = sym_addr.get("__%s_end" % m, 0)
        if not st or not en:
            lines.append("  FAIL  %s start/end symbols missing" % m)
            a_fail += 1
            continue
        if st != RACK_WINDOW:
            lines.append("  FAIL  %s VMA 0x%08x != window origin 0x%08x" % (m, st, RACK_WINDOW))
            a_fail += 1
        if en - st > RACK_CLOUDS_B_OFFSET:
            lines.append("  FAIL  %s size %d > A slot %d (overlaps the chain-B slot)"
                         % (m, en - st, RACK_CLOUDS_B_OFFSET))
            a_fail += 1
        lines.append("  ok    %-14s VMA 0x%08x size %6d (A slot)" % (m, st, en - st))
    st = sym_addr.get("__tc_clouds_start", 0)
    en = sym_addr.get("__tc_clouds_end", 0)
    if st and en:
        want = RACK_WINDOW + RACK_CLOUDS_B_OFFSET
        if st != want:
            lines.append("  FAIL  tc_clouds VMA 0x%08x != window+B 0x%08x" % (st, want))
            a_fail += 1
        if RACK_CLOUDS_B_OFFSET + (en - st) > RACK_WINDOW_LEN:
            lines.append("  FAIL  tc_clouds size %d overflows the window" % (en - st))
            a_fail += 1
        lines.append("  ok    %-14s VMA 0x%08x size %6d (B slot)"
                     % ("tc_clouds", st, en - st))
    else:
        lines.append("  FAIL  tc_clouds start/end symbols missing")
        a_fail += 1
    if window_start and window_start + RACK_WINDOW_LEN != SCRATCH_X:
        lines.append("  FAIL  window end 0x%08x != SCRATCH_X 0x%08x"
                     % (window_start + RACK_WINDOW_LEN, SCRATCH_X))
        a_fail += 1
    lines.append("  A summary: %d FAIL" % a_fail)
    fails += a_fail

    # --- Check A2: platform hot symbols in RAM ------------------------------
    lines.append("[Check A2] platform hot symbols must be RAM-resident")
    a2_fail = a2_warn = 0
    for name, static_ok in HOT_COMMON:
        matches = match_symbols(syms, name)
        ram_hits = [s for s in matches if in_ram(s["addr"])]
        flash_hits = [s for s in matches if in_flash(s["addr"])]
        if flash_hits:
            for s in flash_hits:
                lines.append("  FAIL  %s -> FLASH @0x%08x (%s)" % (name, s["addr"], s["name"]))
                a2_fail += 1
        elif ram_hits:
            pass
        elif static_ok:
            lines.append("  WARN  %s not found (static_ok; likely inlined)" % name)
            a2_warn += 1
        else:
            lines.append("  FAIL  %s not found (expected a definition)" % name)
            a2_fail += 1
    lines.append("  A2 summary: %d FAIL, %d WARN" % (a2_fail, a2_warn))
    fails += a2_fail
    warns += a2_warn

    # --- Check B (report-only): flash edges from the RAM closure ------------
    # The rack does NO runtime flash writes on the RP2350 (the presets live
    # on the C3), so XIP execution is acceptable everywhere; this check is
    # the visibility DSPi's Check B enforced as a failure.
    lines.append("[Check B] flash edges from the hot RAM closure (report-only)")
    b_info = 0
    hot_starts = []
    for name, _ok in HOT_COMMON:
        for s in match_symbols(syms, name):
            if in_ram(s["addr"]):
                fs = enclosing_func(starts, funcs, s["addr"])
                if fs is not None:
                    hot_starts.append(fs)
    visited = set()
    queue = list(dict.fromkeys(hot_starts))
    visited.update(queue)
    head = 0
    while head < len(queue):
        fs = queue[head]
        head += 1
        f = funcs[fs]
        for (addr, mnem, operand) in f["insns"]:
            kind, tgt, tname = parse_target(operand)
            if kind != "addr" or tgt is None:
                continue
            if f["start"] <= tgt < f["end"]:
                continue
            if tgt < 0x8000:
                continue
            if in_flash(tgt):
                b_info += 1
                continue
            if in_ram(tgt) or in_window(tgt):
                ts = enclosing_func(starts, funcs, tgt)
                if ts is not None and ts not in visited:
                    visited.add(ts)
                    queue.append(ts)
    lines.append("  B summary: RAM closure = %d functions, %d direct flash edges (INFO; "
                 "no runtime flash writes on the RP2350)" % (len(visited), b_info))

    # --- Check B2: core-1 literal-pool flash scan ---------------------------
    # The swap copies the module window while core 1 is masked; a flash-
    # range literal dereferenced in that window would read dead flash.
    lines.append("[Check B2] core-1 function bodies must hold no flash-range literals "
                 "(swap-window safety)")
    b2_fail = 0
    c1_seed = []
    for name in CORE1_ROOTS:
        for s in match_symbols(syms, name):
            if in_ram(s["addr"]):
                fs = enclosing_func(starts, funcs, s["addr"])
                if fs is not None:
                    c1_seed.append(fs)
    c1_visited, _, _, _ = bfs_from(c1_seed, funcs, starts, syms)
    for fs in sorted(c1_visited):
        f = funcs[fs]
        for (waddr, val) in f["words"]:
            if in_flash(val):
                wname = nearest_symbol(sorted_addrs, val)
                if wname in B2_LITERAL_WHITELIST:
                    lines.append("  INFO  %s @0x%08x -> .word 0x%08x (%s; whitelisted)"
                                 % (f["name"], waddr, val, wname))
                else:
                    lines.append("  FAIL  %s @0x%08x -> .word 0x%08x (FLASH pointer, near %s)"
                                 % (f["name"], waddr, val, wname))
                    b2_fail += 1
    lines.append("  B2 summary: %d core-1 function(s) scanned, %d FAIL"
                 % (len(c1_visited), b2_fail))
    fails += b2_fail

    # --- Check C: size accounting ------------------------------------------
    lines.append("[Check C] size accounting")
    data = sec.get(".data", 0)
    bss = sec.get(".bss", 0)
    vtab = sec.get(".ram_vector_table", 0)
    heap = sec.get(".heap", 0)
    ram_used = vtab + data + bss + heap
    heap_free = sym_addr.get("__HeapLimit", 0) - sym_addr.get("__end__", 0)
    flash_code = sec.get(".text", 0) + sec.get(".rodata", 0)
    lines.append("  .data=%d  .bss=%d  vtab=%d  heap=%d  ram_used=%d (RAM region %d)"
                 % (data, bss, vtab, heap, ram_used, RAM_LEN))
    lines.append("  heap_free (__HeapLimit - __end__, before runtime allocation)=%d" % heap_free)
    lines.append("  flash_code (.text+.rodata)=%d" % flash_code)
    if heap_free < HEAP_GATE:
        lines.append("  FAIL  heap slack %d < %d gate (the I2S bank + TinyUSB FIFOs "
                     "malloc at runtime)" % (heap_free, HEAP_GATE))
        fails += 1
    else:
        lines.append("  heap slack %d >= %d gate" % (heap_free, HEAP_GATE))
    if bss + data > RAM_LEN:
        lines.append("  FAIL  .data+.bss %d exceeds the RAM region %d" % (bss + data, RAM_LEN))
        fails += 1

    if baseline and "rp2350" in baseline:
        b = baseline["rp2350"]
        lines.append("  %-20s %12s %12s %12s" % ("metric", "baseline", "current", "delta"))
        for key, cur in (("data", data), ("bss", bss), ("flash_code", flash_code)):
            bval = b.get(key, 0)
            lines.append("  %-20s %12d %12d %+12d" % (key, bval, cur, cur - bval))
    elif baseline:
        lines.append("  baseline has no rp2350 entry")

    verdict = "FAIL" if fails else ("PASS (with warnings)" if warns else "PASS")
    lines.append("VERDICT: %s  (%d FAIL, %d WARN)" % (verdict, fails, warns))

    return {"elf": elf, "fails": fails, "warns": warns, "data": data, "bss": bss,
            "heap_free": heap_free, "text": "\n".join(lines)}

def bfs_from(seeds, funcs, starts, syms):
    """BFS over the RAM/window function closure from seed function starts."""
    visited = set()
    queue = list(dict.fromkeys(seeds))
    visited.update(queue)
    head = 0
    while head < len(queue):
        fs = queue[head]
        head += 1
        f = funcs[fs]
        for (addr, mnem, operand) in f["insns"]:
            kind, tgt, tname = parse_target(operand)
            if kind != "addr" or tgt is None:
                continue
            if f["start"] <= tgt < f["end"]:
                continue
            if tgt < 0x8000:
                continue
            if in_ram(tgt) or in_window(tgt):
                ts = enclosing_func(starts, funcs, tgt)
                if ts is not None and ts not in visited:
                    visited.add(ts)
                    queue.append(ts)
    return visited, [], [], []

def main(argv):
    if len(argv) < 2:
        sys.stderr.write(__doc__)
        return 2

    here = os.path.dirname(os.path.abspath(__file__))
    baseline = None
    bpath = os.path.join(here, "ram_baseline.json")
    try:
        with open(bpath) as fh:
            baseline = json.load(fh)
    except OSError:
        pass

    results = [check_elf(elf, baseline) for elf in argv[1:]]
    for r in results:
        print(r["text"])
        print()

    print("=" * 78)
    print("OVERALL SUMMARY")
    print("%-30s %5s %5s %10s %10s %10s" % ("elf", "FAIL", "WARN", ".data", ".bss", "heapFree"))
    any_fail = False
    for r in results:
        any_fail = any_fail or r["fails"] > 0
        print("%-30s %5d %5d %10d %10d %10d" % (
            os.path.basename(r["elf"]), r["fails"], r["warns"], r["data"], r["bss"],
            r["heap_free"]))
    print("=" * 78)
    return 1 if any_fail else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
