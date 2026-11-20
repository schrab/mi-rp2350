# Handoff — Dronecaster T-D2 crash investigation

Date: 2026-11-21. Branch `rack-rp2350`, HEAD `24250ae` (code) / `1a6a0c8`
(ledger). Both firmwares flashed and verified. This document is for the next
agent tasked with fixing the drone crashes. **Task 10 is NOT accepted** — the
user has not confirmed listening by ear.

## What the user asked for (verbatim intent)

- "why only 4 drones can be selected??????" — the allowlist I added was wrong.
- "how can i listen if its crashing? how do i change drones if i cant scroll
  thru crashing drone" — the pot must be usable for auditioning.
- "create handoff and i try to fix crashes with other agent" — this document.

## Current state (what is DONE, do not redo)

1. **All 14 drones are selectable on the RP.** CC 51 `val<=36` = direct catalog
   id, `val 37..127` = 14-step sweep. No allowlist. `module_dronecaster.h`.
2. **C3 DRONE pot = 10-step AUDITION list** (`RackUI.cpp`), ids
   `{0,7,10,15,22,23,25,27,29,36}` with names
   `APPAR/FLDST/HECK/MTZION/SINE/SQR/SUPER/THX/TRI/HARMS`. The 4 hanging ids
   are deliberately excluded from the pot so scrolling cannot trigger a reset;
   they remain reachable via browser panel (`tools/rack_panel.html`, all 14
   steps) and serial `cc 51 <id>` for debugging. Once crashes are fixed, add
   ids 14/20/24/30 back to the three `kIds[]` tables, the name table, and the
   `p1p2DiscreteCount` count in `RackUI.cpp` (currently 10).
3. **C3 SYS module name fixed** — `potToDisplayValue()` already returns a
   module index for SYS cell 0; index `kModNames` directly (do NOT divide by
   128). All 5-module assumptions replaced with `kModuleCount = 6`.
4. **General rings-swap crash FIXED** — `rackdist::process` was compiled into
   `.tc_elements` (single-TU include-guard effect: `module_elements.h` defines
   `RACK_DIST_MARK` first), so a rings window copy overwrote its address →
   core-1 `UNDEFINSTR` (pc/lr in `g_rack_pool`). Dispatcher now stays in flash;
   rings clamps `dist_type % 4` to avoid the elements-only swede helper.
   All 8 module swaps now work incl. rings back-and-forth.
5. **Heartbeat watchdog** (`main.cpp`): core 1 bumps `g_audio_hb` per render;
   core 0 feeds the 2 s HW watchdog only when it advances. A hung audio core
   self-resets in ~2 s (`reason=3`) with no manual replug. Verified on Mt. Lion.
6. Voice carve raised 136→168 KiB in `module_dronecaster.h` (Hecker needs
   162,312 B of arena). Existing host gate unchanged: 133 refs, 14 golden
   cases, `PASS (0 failures)`.
7. C3 flashed 2026-11-21; RP flashed via BOOTSEL (USB CDC was wedged after a
   hang — see "Recovery" below).

## Hardware crash matrix (clean-state retest, one drone per reset)

2 s report `max Process` against 1333 µs budget:

| id | drone | result | max (µs) |
|----|-------|--------|----------|
| 0  | Apparatus | OK | 881 |
| 7  | Fieldsteel | OVER | 2187 |
| 10 | Hecker | OVER | 12528 |
| 14 | Mt. Lion | AUTO-RESET (hang) | — |
| 15 | Mt. Zion | OVER | 2870 |
| 20 | Sachiko | AUTO-RESET (hang) | — |
| 22 | Sine | OK | 38 |
| 23 | Square | OK | 141 |
| 24 | Starlids | AUTO-RESET (hang) | — |
| 25 | Supersaw | OK | 932 |
| 27 | Thx | OVER | 2354 |
| 29 | Triangle | OVER | 2443 |
| 30 | Twin Pks | AUTO-RESET (hang) | — |
| 36 | harms way | OK | 687 |

"OVER" = renders but misses the block budget (audio may glitch); "AUTO-RESET"
= watchdog caught a core-1 hang and rebooted to elements. Numbers are from
one pass; re-measure after any fix. Some numbers look suspicious (Triangle
2443 vs earlier 114) — **re-measure from a clean state before trusting them**;
the earlier cascade test was invalid (see below).

## CRITICAL testing methodology (my earlier mistake — do not repeat)

A first sweep switched drones back-to-back on one boot. After Hecker wedged
core 1, every later id showed the SAME stale report (`Hecker avg=1674u sw=4`,
`max Process: 0`, `in/out=0`) — the board was already dead, so those drones
were never actually exercised. I wrongly concluded 10 drones were broken and
blocked them. **Always reset the board between drones** (via the watchdog or
BOOTSEL), switch to dronecaster, send `cc 51 <id>`, and verify the report tail
contains the matching drone name before recording a number. With the watchdog
armed, an id that hangs self-resets to elements — wait for re-enumeration,
then re-issue `cc 32 5` before the next id. Scripts: `/tmp/dc_drone_verify2.py`
(robust reset handling), earlier flawed ones `/tmp/dc_t10_sweep*.py`.

## Fault signatures captured (from `crash` command)

- Rings swap: `core=1 pc=0x200039f0 lr=0x2002239c cfsr=0x00010000
  (UNDEFINSTR) hfsr=0x40000000`. pc/lr were in `g_rack_pool` (data). Root cause
  found and fixed (rackdist section) — see item 4 above.
- Mt. Lion (earlier, pre-watchdog): `core=1 pc=0 lr=0x40000 cfsr=0 hfsr=0x80000000`.
  This was likely a cascade artifact; the clean retest shows a hang that the
  watchdog catches. Re-capture a fresh `crash` line for each hanging drone and
  `addr2line` the pc: `arm-none-eabi-addr2line -f -C -e
  rack-rp2350/.pio/build/rack/firmware.elf <pc>`. Note pc in the 0x200xxxxx
  range = SRAM (window or arena), 0x100xxxxx = flash XIP veneer.
- Watchdog reset (hang): `crash: reason=3 wd=0x01 ...` and boot line
  `boot: reason=3`.

## Likely root-cause areas for the remaining 4 hanging drones

These are hypotheses, not conclusions — verify with `addr2line` from a fresh
crash before changing code:

1. **Arena pressure at swap.** Each drone builds a UGen graph into the 168 KiB
   dronecaster carve (measured worst Hecker 162,312 B). A larger port may
   overrun the carve and corrupt adjacent state → core-1 fault. Check
   `rack_mem_used()` after `SetDrone` for ids 14/20/24/30 (host gate has an
   arena budget test in `rack-rp2350/test/test_dronecaster.cc` — extend it to
   print exact per-drone bytes and compare to the carve).
2. **Core-1 stack overflow** — core 1 has an 8 KB stack
   (`core1_separate_stack = true`, `main.cpp`). Deep UGen graphs / large stack
   locals in a port could overflow. If pc/lr land near a stack guard or the
   arena, suspect this.
3. **Unsupported SC op / wild pointer in a port** — the 4 hanging ports are
   the most complex audio-equivalent matches; a missed SC runtime guard
   (e.g. an uninitialized buffer or a `Delay` with unchecked index) can fault.
   Compare the port in `rack-rp2350/src/dronecaster/drones/drone_*.h` against
   the host golden run — host gate passes for all 14, so the DSP math is
   correct on x86; a firmware-only failure points to memory/layout, not math.
4. **CPU overrun cascading** — a port that renders >1333 µs starves I2S; if a
   helper then runs re-entrantly, the stack can corrupt. Fieldsteel/Hecker
   overrun without hanging, so overrun alone is not sufficient — find what is
   different about the 4 hanging ids.

## Files involved

- `rack-rp2350/src/main.cpp` — audio loop, watchdog, console (`cc 51`,
  `cc 32`, `crash`, `status`, `reboot`), `do_module_swap`.
- `rack-rp2350/src/module_dronecaster.h` — CC 51 catalog dispatch, voice carve,
  `render()`, heartbeat, `report_dc()`.
- `rack-rp2350/src/dronecaster/dronecaster_engine.h` — `SetDrone`, `Recast`,
  swap state; `drones/drone_registry.h` — `Catalog()` and specs;
  `drones/drone_common.h` — shared DSP helpers.
- `rack-rp2350/lib/dronecaster_engine/dronecaster_engine_o3.cpp` — the -O3/SRAM
  hot entry (`process_block_o3`), unique section token in
  `src/dronecaster/dronecaster_tc.h`.
- `rack-rp2350/src/rack_dist.h`, `module_rings.h` — the fixed rings swap bug.
- `rack-uimcu/ui/RackUI.cpp` — C3 DRONE audition list, SYS names.
- `tools/rack_panel.html` — browser panel, all 14 drone steps.
- Host gate: `cd rack-rp2350/test && make test` (distro g++ only; no g++ on
  Windows — use WSL).

## Build / flash quick reference

```sh
pio run -d rack-rp2350                      # → .pio/build/rack/firmware.uf2
cd rack-uimcu && . /home/schrab/esp/esp-idf-v5.5.4/export.sh && idf.py build
```

RP2350: by-id port, never ttyACM number (C3 can take ACM0). 1200-baud touch →
BOOTSEL drive → copy `firmware.uf2` → `sync` → unmount. If the app's USB CDC is
wedged after a hang, the 1200-baud touch fails with `Errno 71` — hold the
physical BOOTSEL button while replugging. C3: `idf.py -p <C3-by-id> flash`
(esptool handles reset). Full procedure in `AGENTS.md` and
`docs/PORTING-HIGHLIGHTS.md`.

## Acceptance criteria for the next agent

1. All 14 drones selectable from the C3 pot again (restore the 10→14 tables),
   each renders a full 2 s report with `nan=0`, `i2s stalls: 0 (+0)`, and
   `max Process < 1333 µs` (or document a justified higher budget).
2. No drone causes a watchdog reset/hang. Re-run the clean-state matrix above.
3. Host gate stays green; add a regression test if a root cause is found.
4. Update `.superpowers/sdd/progress.md` and this task dir with the fix + commit.
5. **Listening confirmation must come from the user** — do not write
   "user-accepted" or mark Task 10 accepted yourself.
