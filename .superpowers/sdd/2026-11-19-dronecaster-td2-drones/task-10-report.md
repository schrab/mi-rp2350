# Task 10 report — hardware acceptance (requires the user)

**STATUS: SAFE SUBSET MEASURED ON HARDWARE; LISTENING ACCEPTANCE STILL PENDING.**
No "user-accepted" claim — the user has not confirmed the audio by ear.

## What is verified locally (no hardware claim)

- Firmware built: `rack-rp2350/.pio/build/rack/firmware.uf2` (3,800,064 B,
  from `pio run -d rack-rp2350` SUCCESS — RAM 72.9%, Flash 39.2%).
- RP2350 visible at `/dev/serial/by-id/usb-Raspberry_Pi_Rack_RP2350_*-if00`
  (board connected). No flashing or serial monitoring performed by the agent.
- Host gate: 133 refs, 14 golden cases (8 exact + 6 audio-equivalent), 11/11
  arena swaps, `PASS (0 failures)`.
- Panel byte-sync: CC 51 sync bytes = the 14 live ids; pot map reaches all 14.

## What the user must do (Step 1 + Step 2)

1. Flash the UF2 (by-id port; 1200-baud touch → BOOTSEL → copy UF2) per AGENTS.md.
2. For each live id, send CC 51 raw byte and read the 2 s report. Check:
   active drone matches, `nan=0`, `i2s stalls: 0 (+0)`, `max Process` < 1333 µs.
   Flag any drone over budget for profiling — do not silently accept.
3. Listen to each of the 14 live drones and confirm the sound (especially the
   6 audio-equivalent ones: Apparatus, Twin Pks, Supersaw, Fieldsteel,
   Starlids, Sachiko). Coil (2) is deferred, not playable.
4. Report back; only then may the ledger say "user-accepted".

## Live ids for the CC 51 sweep

`0, 7, 10, 14, 15, 20, 22, 23, 24, 25, 27, 29, 30, 36` (14 ids).
Deferred: `2` (Coil — different timbre).

## Ledger data already known (for the final entry)

- Golden results (host): harms way 3.7e-8/1.0; Thx 3.7e-8/1.0;
  Mt. Zion 5.2e-8/1.0; Mt. Lion 3.3e-8/1.0; Hecker 9.3e-8/1.0.
- Audio-equivalent: Apparatus 6.8e-6/1.0; Twin Pks 1.7e-4/1.0;
  Supersaw 0.131/0.923; Fieldsteel 0.029/0.257; Starlids 0.134/0.859;
  Sachiko 0.086/−0.131 (spectral envelope L1 0.105/0.169 for the last two).
- Deferred: Supersaw was re-landed; Coil 0.125/corr −0.003, spectral L1 0.90
  → different timbre, catalog id 2 inert.
- Commit hashes (port commits): `ec012eb` (harms way), `c29d117` (Thx/Zion/Lion),
  `ad0d952` (Hecker + Fieldsteel), `09a7345` (Apparatus + Twin Pks),
  `136f251` (Supersaw + Fieldsteel re-land), `1b5b476` (Starlids + Sachiko),
  `1850cfa` (Coil + arena gate), `61738d1` (CC 51 + panel).
- Law corrections: filter ctor primes, EnvGen Node lifetime, SinOsc iphase in
  radians, `LFPar(hz:)` keyword trap, nova tanh sign, div-by-scalar reciprocal,
  kr Dust rate, checked cubic DelayC, def-build-time rrand def reuse.

## Hardware findings (2026-11-21)

- **Crash cause:** the T-D2 ports exceed the firmware's dronecaster voice
  carve / CPU budget. Hard-fault record on core 1: `reason=2 wd=0x01`,
  `pc=0x00000000`, `hfsr=0x80000000` (FORCED) — a hard fault that kills core
  1's render loop; core 0 then prints `max Process: 0` and `in/out=0` forever,
  the C3 UI shows `---` (no valid TELEM), and only a power cycle recovers.
- Two concrete over-budget measurements: Fieldsteel **2318 µs**, Hecker
  **13813 µs** against the 1333 µs block budget. Mt. Lion / Mt. Zion /
  Sachiko / Starlids / Thx / Twin Pks also faulted core 1 when selected.
- **Fix applied:** `module_dronecaster.h` carve raised 136 KiB → 168 KiB
  (Hecker alone needs 162,312 B of arena), and CC 51 now uses a hardware
  allowlist `{0, 22, 23, 29}` — all other ids are rejected with a serial
  warning and the board stays alive. Panel + ccMap updated to match.
- Post-fix hardware sweep (all on the flashed UF2, board never froze):
  - id 0 Apparatus: max Process **967 µs**, nan=0, stalls=0
  - id 22 Sine: **845 µs**, id 23 Square: **127 µs**, id 29 Triangle: **114 µs**
  - blocked ids 7,10,14,20,24,25,27,30,36: each emitted
    "not hardware-safe yet - blocked" and the 2 s report kept running
    (`status:` still responded after the full sweep).
- Host gate unchanged: 133 refs, 14 golden cases, PASS (0 failures).
- **C3 UI fixes (user-reported, 2026-11-21):** DRONE selector had no names
  and the SYS module name showed `---` for dronecaster. Root cause: the C3
  display value is POT SPACE (0..127), but both name tables were indexed with
  the raw value. `rack-uimcu/ui/RackUI.cpp` now: (1) treats CC 51 as a 4-step
  selector with wire bytes `{0,22,23,29}` and names `{APPAR,SINE,SQR,TRI}`
  (pot↔wire conversion added in `PotToWireValue`/`WireToPotValue`, mirroring
  the RP allowlist); (2) converts SYS cell 0 pot space `n*127/5` back to an
  index before `kModNames`; (3) replaces every remaining `5`-module assumption
  with `kModuleCount = 6` (discrete count, overlay `%5`, selector pot). This
  is C3-only — no RP2350 reflash was required. C3 built + flashed, both
  boards verified alive; RP kept reporting `mod=dronecaster`, Apparatus.
- **CORRECTION (2026-11-21):** the 4-drone allowlist rested on a flawed test —
  after Hecker wedged the board all later ids showed a stale report and were
  never exercised. Clean per-reset retest: Fieldsteel 1571 us and Hecker 7417
  us both RUN (over budget, flagged); only Mt. Lion genuinely faults. The
  allowlist was removed; all 14 drones are selectable again (C3 14-step named
  selector + browser panel + ccMap restored). A core-1 heartbeat watchdog now
  auto-resets on a hang, so no drone can freeze the board.
- Separately, a general swap bug was fixed: `rackdist::process` lived in
  `.tc_elements`, so a rings swap overwrote its address → UNDEFINSTR/core-1
  death. It now lives in flash and rings clamps its dist type to 0..3. Rings
  swaps repeatedly with no fault.
- **Still required from the user:** listen to the drones and confirm which are
  acceptable. Over-budget ones are flagged, not silently accepted.
