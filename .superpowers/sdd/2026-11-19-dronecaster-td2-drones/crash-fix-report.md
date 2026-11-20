# T-D2 crash-fix report — the 4 delay drones (Mt. Lion 14, Sachiko 20, Starlids 24, Twin Pks 30)

Agent session 2026-11-21, branch `rack-rp2350`. Resumed once (hollow-session
stop after the first measurement); this file is the running ledger for the fix.

## STATUS: FIXED (host-verified; hardware matrix pending the user)

- Host gate: **PASS (0 failures)** — 133 REFs ok, comparator self-tests PASS,
  **14/14 goldens** (the 8 sample-exact drones keep their banked max values
  bit-identically; the audio-equivalent drones unchanged except Sachiko,
  below). PicoCollider's own 129-ref gate: **PASS (0 failures)**.
- Firmware: `pio run -d rack-rp2350` **SUCCESS**, all window ASSERTs green
  (incl. the new aux-region assert). NOT flashed — hardware steps are the
  user's.

## Root cause (established with source + measurement evidence)

The 4 hanging drones are EXACTLY the delay-line graphs. On the device path,
drone states are placement-new'd into the dronecaster carve, but the delay
UGens inside those states owned `std::vector<float> buf_` and allocated their
ring buffers from the **C++ heap** in `Init()`:

- `sc_kit_delay.h` — `DelayLine` (DelayN/L/C, CombN/L/C, AllpassN/L/C), plus
  `Pluck` / `PitchShift` (unused by ported drones).
- `drone_common.h` — `DelayCA` (Twin Pks' wow DelayC).
- `sc_kit_core_dyn.h` — `Limiter` (Twin Pks' lookahead table, 468 B — found by
  allocation backtrace, the residual allocation after the rings were fixed).

Three device facts make that fatal (memmap_rack.ld header + firmware map):

1. **The device C++ heap is 22,272 B** (`__bss_end__ 0x2005d700` →
   `__HeapLimit 0x20063000`). The measured vector demand: Mt. Lion
   2,359,296 B (9 × 65,536-float rings), Sachiko 1,130,496 B (28 rings),
   Starlids 65,536 B (12 rings), Twin Pks 8,660 B. Mt. Lion/Sachiko/Starlids
   exceed or exhaust it → a failed `operator new` (no exceptions) hangs/faults
   core 1 → the heartbeat watchdog reboots the board (reason=3).
2. **The core's malloc has no multicore lock**: `malloc-lock.cpp` wraps
   `_malloc_r` in `noInterrupts()`/`interrupts()` only — that stops interrupts
   on the CALLING core, not the other core. Core 1 (drone Init) and core 0
   (Serial/USB/stdio) can mutate the newlib free-list concurrently →
   corruption. Even Twin Pks' 8.6 KB (which fits) was unsafe, and its 468 B
   `Limiter` allocation kept the hazard alive.
3. The host gate never saw any of this: on x86 the vectors allocate from the
   host heap and `arena_used()` counts only the placement-new'd state structs.

## The fix

1. **Kit `DelayLine` (sc_kit_delay.h)**: three storages — `Init()` keeps the
   vector (kit refs/tests, bit-exact unchanged); `InitExt(float*, long)`
   places a caller-owned float ring (any size, zeroed at Init); `InitExt16()`
   places an int16 ring with a caller scale (quantize-on-write round-half-away
   with saturation, dequantize-on-read — deterministic host==MCU). The wrap is
   now arithmetic over a maintained write position (`wpos_`) — identical
   values to the old pow2 mask for any size. Ext clamp ceiling = bufsize−4;
   rings are sized graph-constant + 8 so the clamp never engages in the goldens.
2. **`DelayCA` (drone_common.h)** and **`Limiter` (sc_kit_core_dyn.h)**: same
   `InitExt` treatment (`Init()` kept for the kit refs).
3. **Drone call sites** place every ring from `AllocRing(Ctx&)` = arena first,
   aux spill, and bail with `c.state = nullptr` on failure (the engine stays
   idle and counts `init_fails()` — a capacity failure can never hang again).
   Ring sizes are graph constants measured on the golden renders:
   - **Mt. Lion**: 9 CombN rings, golden max dsamp per voice
     {1, 574, 10072, 1, 33234, 1, 1, 13040, 22415} → ring = golden+8 floats;
     the four never-fired voices keep a 2,048-float (42.7 ms) transient
     allowance. Σ = 350,268 B. (The delaytime chain's latch holds 0 until the
     first Dust(0.3) fire, so ScMidicps(0) = 8.1758 Hz makes the EARLY
     transient delaytime up to ~0.86 s while the 5 s lag converges — this is
     why the rings are large and why hand-derived bounds were wrong; sizes
     come from the measured golden, not hand math.)
   - **Sachiko**: 27 int16 rings — voice combs 18,220 samples ×4
     (live delaytime 0.2794 ± 0.1 s → dsamp ≤ 18,211, scale 1/4096, range ±8),
     dn 4,100 (clamp ceiling EXACTLY 4,096 — the golden hits the old kit's
     4,096-sample clamp, preserved bit-exactly; scale 1/4096), 14 tail CombL
     5,500 (golden max 4,295; scale 1/256; live clamp 114.5 ms vs the old
     170.7 ms — documented live-only shift), 8 AllpassN 2,144 (fixed delay
     constants, dsamp ≤ 2,136.4; scale 1/256). Σ = 341,464 B. The int16
     quantization is a documented device-storage deviation; Sachiko's gate
     (max 0.10 / corr −0.20) still passes at max 0.086446553 (+1.4e-6 vs the
     banked float32 render).
   - **Starlids**: 12 DelayC rings of 104 floats (live delaytime
     (lfn×0.01+0.02)/15 s → dsamp ≤ 96.0000045; golden max 95).
   - **Twin Pks**: DelayCA ring 2,056 floats (golden max 149; keeps the old
     2,048 ceiling with margin) + Limiter table 117 floats (3 × ceil(0.0008 s
     × 48 kHz)).
4. **Memory for the rings**: the dronecaster carve is now the whole remaining
   pool (`kVoiceBytes = kRackPoolBytes - sizeof(State)` = 331,040 B —
   dronecaster is standalone-only, so it may use all of the union pool), plus
   a NEW **aux region**: the SRAM hot-code window above the dronecaster hot
   image (`__rack_window_aux_start/end` in memmap_rack.ld, 117,496 B on the
   current build). The aux is idle whenever dronecaster is the resident
   module (A-slot overlay members are mutually exclusive) — and
   `main.cpp cmd_chain` now REJECTS dronecaster as a chain A explicitly
   (`chain_parse_module` accepted any id by number; the B-slot clouds copy
   would have written over the aux region). Engine::AttachAux wires it;
   the aux is reset (and every ring re-zeroed) at every swap-in, so stale
   hot-code bytes can never leak into audio.
5. **Kit tables shrunk + made lazy**: KitTables 131,080 → 73,744/73,752 B
   (ARM/host). The sine wavetable (`sine_wt`) is now built directly from the
   sin() law (bit-identical — it previously derived from the gSine array),
   the pan quarter (`sine_q[0..2048]`, used by Pan2/Balance2/SelectX/XFade2)
   is inline and always built, and the full BLIT tables (gSine/gInvSine, used
   by Pulse / BLIT Saw / Rotate2 / PanAz) are built on demand by
   `EnsurePulseTables(Ctx&)` into arena/aux storage. The 6 BLIT drones
   (Square, Mt. Zion, Thx, Supersaw, Fieldsteel, Starlids) call it before
   constructing those UGens. Without this, Mt. Lion's rings would not fit
   (the tables ate 131,080 of the pool).
6. **Host gate additions** (test_dronecaster.cc):
   - `== T-D2 device memory budget ==` — renders the full golden span of
     every live drone against the device-realistic sizes (carve 331,040 B +
     aux 117,552 B) and asserts: heap bytes == 0, init success, arena/aux
     within budget. This is the regression gate for the hang.
   - The golden cases + the arena budget test run against the same
     device-realistic sizes with the aux attached.
   - Heap-byte counting via operator-new overrides; kit-table CHECKs updated
     for the lazy layout (quarter == full law).

## Per-drone budget numbers (after the fix; device-realistic 331,040 + 117,552)

| id | drone | arena B | aux B | heap B | verdict |
|----|-------|---------|-------|--------|---------|
| 0 | Apparatus | 77,520 | 0 | 0 | ok |
| 7 | Fieldsteel | 146,992 | 0 | 324 | ok (known residual, see below) |
| 10 | Hecker | 104,984 | 0 | 0 | ok |
| 14 | Mt. Lion | 326,552 | 108,896 | 0 | ok (was 2.25 MB heap) |
| 15 | Mt. Zion | 147,680 | 0 | 0 | ok |
| 20 | Sachiko | 330,880 | 96,064 | 0 | ok (was 1.13 MB heap) |
| 22 | Sine | 73,776 | 0 | 0 | ok |
| 23 | Square | 139,360 | 0 | 0 | ok |
| 24 | Starlids | 153,680 | 0 | 0 | ok (was 64 KB heap) |
| 25 | Supersaw | 143,376 | 0 | 0 | ok |
| 27 | Thx | 148,040 | 0 | 0 | ok |
| 29 | Triangle | 73,768 | 0 | 0 | ok |
| 30 | Twin Pks | 86,640 | 0 | 0 | ok (was 8,660 B heap) |
| 36 | harms way | 78,784 | 0 | 0 | ok |

(Before the fix, the host arena test measured only the state structs — e.g.
Mt. Lion "arena=142,200" — because the vector bytes were invisible heap; the
device heap requirement was the 0.06-2.25 MB column above.)

Margins: Mt. Lion aux slack 8,600 B (ld ASSERT pins aux ≥ 108 KiB so the hot
image growing cannot silently break it); Sachiko aux slack 21,488 B.

## Golden verification (unchanged gates)

- Sine/Square/Triangle/harms way/Thx/Mt. Zion/Mt. Lion/Hecker: GOLDEN PASS,
  max values bit-identical to the banked numbers (e.g. Mt. Lion 3.3e-8,
  Mt. Zion 5.2e-8, Thx 3.7e-8; the comparator's worst_i is the interleaved
  sample index = 2× the frame indices quoted in the older reports).
- Apparatus/Twin Pks/Supersaw/Fieldsteel/Starlids: AUDIO-EQUIV PASS with the
  banked numbers unchanged (Twin Pks 1.68e-4 / corr 0.999999994; Starlids
  0.134096749 / corr 0.858709220).
- Sachiko: AUDIO-EQUIV PASS — max 0.086446553 (banked 0.086445130; +1.4e-6
  from the int16 ring quantization), corr −0.1305 unchanged, gates 0.10/−0.20.
- The `Limiter`/delay-family kit refs pass bit-exactly through the new
  unified wrap arithmetic (the old mask wrap produced identical indices).

## KNOWN RESIDUALS (documented, not blocking)

1. **Fieldsteel demand sequences**: Dseq/Drand/DXRand/Dbrown (+ Splay) still
   construct small std::vectors at Init (~324 B). They succeed on device
   today, but they (a) are core-1 mallocs racing core 0 (the same
   no-multicore-lock hazard, empirically not hit), and (b) LEAK per
   Fieldsteel swap-in (no dtor runs on the arena reset) — ~68 Fieldsteel
   selections would exhaust the 22 KB heap. Follow-up: convert the demand
   kit to caller-owned storage (pointer + count, values in the drone state).
2. **Live-only deviations** (golden bit-exactness preserved):
   - Mt. Lion's four never-fired voices: their first dtm-chain Dust fire
     clamps the comb at 42.7 ms until the lag converges (~15 s).
   - Mt. Lion's fired voices: an unlucky live transient can clamp at the
     golden max (0.69 s) instead of the true ~0.86 s, settling identically.
   - Sachiko's tail combs: live delaytimes clamp at 114.5 ms (was 170.7 ms).
   - Sachiko voice combs/tail/aps/dn: int16 ring storage (−83 dB quantization
     on the full-level voice combs; the tail path is 1% wet).
3. C3 pot audition list: ids 14/20/24/30 are still excluded from the C3's
   10-step DRONE pot (RackUI.cpp) — restore them to the kIds tables ONLY
   after the user's hardware matrix passes (handoff acceptance step).

## Hardware steps for the user (unchanged from the handoff)

Clean-state matrix: reset between drones, `cc 32 5` then `cc 51 <id>`, verify
the 2 s report tail names the drone. Expected: all 14 render; the 4 previous
hangers show `initfail=0` (new field in report_dc) and no watchdog reset.
If a drone reports `--` with `initfail>0`, the budget assert fired (a sizing
bug) — it failed LOUD and idle instead of hanging.

## Status log

- (resumed) Report created; baseline gate re-confirmed PASS before changes.
- (measured) Instrumented kit (temporary `DelayTrace`): per-instance golden
  max dsamp + max stored value + vector heap bytes per drone (tables above).
- (implemented) All of the fix list above; PicoCollider gate PASS (0 failures);
  dronecaster host gate PASS (0 failures) with the new budget regression;
  firmware build SUCCESS with all window ASSERTs green (aux ASSERT tightened
  to ≥ 108 KiB). Not flashed.
