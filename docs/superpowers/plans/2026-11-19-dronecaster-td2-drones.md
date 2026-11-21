# Dronecaster T-D2 Drone Ports Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the 12 curated T-D2 drones into the dronecaster engine, each sample-exact against its own scsynth NRT golden, with no debugger and no hardware until the final acceptance flash.

**Architecture:** The T-D2 UGen kit is already complete in the `PicoCollider` submodule (129 refs green). Each drone is a line-for-line C++ mirror of its `.scd` graph, composed from `sc::*` kit UGens — no new UGen ports. All drone state is arena-allocated via `Ctx`; `DroneSpec` entries go into one phase file (`drones/drone_t2.h`) and the 37-slot catalog in `drone_trio.h`. Verification is entirely host-side: extend the existing golden harness, render one 4 s float32 WAV per drone, drive `sc::Engine` with the identical schedule, compare with the existing `compare()` gate.

**Tech Stack:** C++17 / plain g++ host tests, SuperCollider 3.14.1 (system install) NRT renders, existing float32 WAV comparator, PlatformIO firmware build only for the final gate.

**Spec:** `docs/superpowers/specs/2026-11-19-dronecaster-port-design.md` (phase table T-D2), `docs/superpowers/plans/2026-11-19-dronecaster-td2-handoff.md` (kit status + working method), ledger §DRONECASTER T-D1/T-D2.

## Scope decision (my call — confirm at review)

The spec says "T-D2 = mid-tier kit + ~12 curated drones chosen by greedy UGen coverage", but T-D2 was later narrowed to **kit-only** and the drone list was never pinned in a document. With the kit now complete, coverage is no longer the binding constraint. This plan defines the curated set as **the 12 cheapest in-scope drones whose UGens are all reference-exact in the kit and which need no new engine infrastructure** (no LocalIn/Out bus, no tape buffer, no Greyhole's 2.63 MB, no law-only chaos, no MouseX/Y CC wiring), ordered by distinct-UGen count:

| # | Drone | Catalog id | Distinct UGens in the .scd |
|---|---|---|---|
| 1 | harms way | 36 | SinOsc, Splay |
| 2 | Supersaw | 25 | BPF, Saw, SinOsc, Splay |
| 3 | Thx | 27 | BLowPass, LFNoise2, Mix, Pan2, Saw |
| 4 | Mt. Zion | 15 | Dust, Latch, Mix, Pan2, Pulse, WhiteNoise |
| 5 | Mt. Lion | 14 | CombN, Dust, Latch, LeakDC, LFPulse, Mix, Pan2, WhiteNoise |
| 6 | Hecker | 10 | BPF, LFNoise0, LinExp, LPF, Mix, PinkNoise, SelectX, VarLag, WhiteNoise |
| 7 | Fieldsteel | 7 | BPF, Demand, Dust, EnvGen, Impulse, LFNoise1, LFSaw, Saw, Splay, SVF |
| 8 | Apparatus | 0 | BPF, Clip, Crackle, Decimator, Dust2, HPF, LeakDC, LFNoise2, LFPar, LFTri, PinkNoise, SinOsc |
| 9 | Twin Pks | 30 | BPF, Compander, ControlRate, Crackle, Decimator, DelayC, Dust2, HPF, LFPar, Limiter, LinExp, LinLin, PinkNoise, SinOsc |
| 10 | Starlids | 24 | DelayC, HPF, LFNoise0, LFNoise1, LFTri, LinExp, LinLin, LPF, Mix, MoogLadder, Pan2, Pulse, SawDPW, SinOsc, VarLag |
| 11 | Sachiko | 20 | AllpassN, CombC, CombL, DelayN, EnvGen, LeakDC, LFNoise0, LFNoise1, LFTri, LinExp, Mix, MoogFF, Pan2, PulseDPW, RLPF, Splay, VarLag |
| 12 | Coil | 2 | AllpassN, BLowPass, CombL, DelayC, DelayN, Dust, EnvGen, Impulse, Latch, LFNoise0, LFNoise1, Mix, Pan2, SelectX, SinOsc, SinOscFB, TChoose, Trig, WhiteNoise |

Deferred to T-D3/T-D4 by this rule: Gristle (Greyhole), Eliane/Magicicada/Ruins/Grove (LocalIn/Out engine wiring), UNRELACC (HenonC law-only), Rehberg (DFM1 epoch pinning), the FFT drones, the 5 tape drones, and the MouseX/Y drones (Dreamcrusher, Takita).

## Global Constraints

- Golden render params, verbatim from the spec: **48 kHz float32, 192,000 frames, hz 55, amp 0.4, fade 1.0 s, gate off at frame 144,000 (block 2250)**; seed `0xC0FFEE` in both sclang (`thisThread.randSeed_`) and the server (`RandSeed.kr(1, 16rC0FFEE)`).
- Gate: `max_abs ≤ 1e-6` **and** `corr ≥ 0.999999` (existing `compare()` / `ref_check()` — never weaken).
- Host gate must stay green after every task: `cd rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster` → `PASS (0 failures)` (129 UGen refs + trio goldens still pass).
- `scsynth -N` renders one block past the marker; goldens are trimmed to 192,000 frames and determinism is compared from the `data` chunk payload (the PEAK chunk carries a wall-clock stamp).
- No debugger, no flashing, no hardware trust during implementation. The only hardware step is Task 10's acceptance flash (user ears required).
- Compose from existing `sc::*` kit headers under `PicoCollider/include/picocollider/`; do not re-port or patch a UGen. Every new drone header carries the GPL-3.0 attribution + the `.scd` source filename, and cites any law taken from the graph.
- **Any UGen a drone needs but the kit lacks is ported properly** — source law with `file:line`, a canonical NRT graph in `gen_ugen_ref.scd`, a rendered 48,000-frame ref, a `ref_check` test, and its own commit — following handoff §3. No inline approximations or "math helper" shortcuts. `LinLin` and `LinExp` are the known missing pair and get Task 2; any further gap found mid-port gets the same treatment before the drone that uses it.
- Catalog ids are the **byte-wise sort** of `test/dronecaster_ref/drones/*.scd` (0=Apparatus … 36=harms way); ids in the table above are fixed.
- `sc::Engine` arena: 131,080 B of tables + per-drone state, carve 136 KiB on device, `static uint8_t mem[160*1024]` in host tests. Every new state struct is `static_assert`ed against the carve.
- CC map (no collisions): 51 drone, 52 amp, 53 fade, 54 hz-offset, pseudo-CC 64 recast.

## Review Focus

- **Random drones vs the pinned seed** — harms way (`Rand`), Mt. Zion / Mt. Lion / Fieldsteel / Apparatus / Twin Pks / Coil (`Dust`, `Dust2`, `WhiteNoise`, `PinkNoise`, `Crackle`). A missing ctor prime or a shared-RGen mismatch desyncs the whole render. Pin: `run_goldens.sh` renders twice and byte-compares the data payload for every new name; the golden gate then catches sequence errors.
- **kr→ar ramp + block alignment** — `VarLag`, `EnvGen`, `LinExp`, and every `*.kr` control in these graphs must use the CALCSLOPE ramp and the internal 64-sample block law. A wrong ramp shows up as `max_abs` in the first blocks. Pin: each drone's golden test uses the exact engine schedule (voice at block 0, gate off at block 2250) and the unmodified `compare()` gate.
- **Arena reuse across swaps** — a drone's state and delay lines must not outlive `Arena::ResetTo(tables_mark_)`, and must not overwrite the 131,080 B kit tables. Pin: a host test selects each new drone *after* a completed trio drone and re-checks the golden, plus a `used() ≤ carve` assertion per drone.
- **Unported ids stay inert** — CC 51 bytes for the 25 not-yet-ported ids must not crash or swap the voice. Pin: a host table-walk over ids 0..36 asserting `Catalog(id) == nullptr` for the unported set, plus the adapter's existing `not ported` guard unchanged.
- **Device budget** — a 19-UGen graph (Coil) with comb/allpass delay lines can exceed the 136 KiB carve or the 1333 µs block budget even when the host golden passes. Pin: per-drone `sizeof(state)` + delay-buffer report in the host test and a `static_assert` on the carve; firmware build's window ASSERT + the 2 s report at acceptance.

---

### Task 1: Extend the golden harness and render the 12 goldens

**Files:**
- Modify: `rack-rp2350/test/dronecaster_ref/gen_goldens.scd:14` (`drones` array)
- Modify: `rack-rp2350/test/dronecaster_ref/run_goldens.sh:15,27,60` (name loops)
- Create: `rack-rp2350/test/dronecaster_ref/goldens/<Name>.expected.wav` × 12

**Interfaces:**
- Produces: 12 float32 golden WAVs at `dronecaster_ref/goldens/<Name>.expected.wav`, 192,000 frames, 2 ch, 48 kHz — consumed by Tasks 3–8.

- [ ] **Step 1: Add the 12 names to the renderer**

In `gen_goldens.scd` replace the `drones` array with the 12 names in table order (`harms way`, `Supersaw`, `Thx`, `Mt. Zion`, `Mt. Lion`, `Hecker`, `Fieldsteel`, `Apparatus`, `Twin Pks`, `Starlids`, `Sachiko`, `Coil`). Keep the trio in the array too (idempotent re-render), and keep the existing wrap/score/schedule unchanged. Note in a comment that `harms way.scd` has a space in its filename — the `"drones/" ++ name ++ ".scd"` path already handles it.

- [ ] **Step 2: Generalize the trim and determinism loops**

In `run_goldens.sh` replace the literal `for f in Sine Square Triangle` loops with one `DC_DRONES` array holding the same 15 names (trio + 12), used by the missing-file check, `trim_goldens()`'s Python `for n in (...)`, and the determinism compare. Do not change the trim target (`192000 * 2 * 4`) or the data-payload compare.

- [ ] **Step 3: Render and verify**

Run: `cd rack-rp2350/test/dronecaster_ref && bash run_goldens.sh`
Expected: two full render passes; `determinism: <name> OK` for all 15 names; every new WAV is 192,000 frames and non-silent (spot-check with the existing `wav_io` tooling if needed).

- [ ] **Step 4: Commit**

```bash
git add rack-rp2350/test/dronecaster_ref/gen_goldens.scd rack-rp2350/test/dronecaster_ref/run_goldens.sh rack-rp2350/test/dronecaster_ref/goldens/
git commit -m "[Add] dronecaster: T-D2 drone goldens (12 curated, seed-pinned)"
```

### Task 2: Port the mapping UGens (LinExp, LinLin)

**Files:**
- Create: `PicoCollider/include/picocollider/sc_kit_lin.h` (inside the submodule — a normal, committable change there)
- Modify: `rack-rp2350/test/dronecaster_ref/gen_ugen_ref.scd` (canonical graphs)
- Modify: `rack-rp2350/test/test_dronecaster.cc` (include + ref checks)

**Interfaces:**
- Consumes: the SC 3.14.1 source laws — `server/plugins/LFUGens.cpp:99` (`struct LinExp`), `:2203` (`LinExp_next`), `:2361` (`LinExp_Ctor`); `SCClassLibrary/Common/Audio/Line.sc:29` (`LinLin` UGen) + `Common/Math/SimpleNumber.sc:382,399` (linlin/linexp laws) + `Common/Core/AbstractFunction.sc:177-181` (UGen expansion); the existing `ref_check` / `ref_ar_fill` / `ref_kr_fill` harness.
- Produces: `sc::LinExp` and `sc::LinLin` (ar and kr variants, `ProcessBlock`/`ProcessKr`/`ctor_value()`); four new 48,000-frame refs `ugen_ref/{LinExp,LinLin}{ar,kr}.expected.wav`.

- [ ] **Step 1: Read the law from source** — never from memory. Quote `file:line` in the new header. Note the UGen's exact float/double types, ctor priming call, and the kr input's CALCSLOPE ramp (handoff §6).
- [ ] **Step 2: Add canonical graphs to `gen_ugen_ref.scd`** — one ar and one kr graph per UGen, using the same argument shapes the drones use (e.g. `LinExp.kr(in, -1, 1, 20, 20000)`, `LinLin.kr(in, 0, 1, 0.8, 0.33)`, with the default `clip=\minmax`). Fix seeds exactly like the other refs.
- [ ] **Step 3: Render the refs** — `cd rack-rp2350/test/dronecaster_ref && bash run_ugen_ref.sh`; expect four new non-silent refs, 48,000 frames each.
- [ ] **Step 4: Write the failing tests** — add `ref_ar_fill`/`ref_kr_fill` sections for all four; run `cd rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster` and confirm four FAILs (missing symbols) while the existing 129 refs stay green.
- [ ] **Step 5: Port the UGens into `sc_kit_lin.h`** — exact arithmetic from the source; no libm in the per-sample path except where the source calls `pow`/`exp` per block, and match its ctor value so block-0 K2A starts from the right point.
- [ ] **Step 6: Verify** — run the gate; expect all four new refs `max=0`/`corr=1` (or within 1e-6) and `PASS (0 failures)`; active ref count 129 → 133.
- [ ] **Step 7: Commit** — submodule bump plus test changes: `[Add] dronecaster: T-D2 LinExp + LinLin UGens — 4/4 references exact`.

### Task 3: harms way (id 36) + Supersaw (id 25)

**Files:**
- Create: `rack-rp2350/src/dronecaster/drones/drone_t2.h`
- Modify: `rack-rp2350/src/dronecaster/drones/drone_trio.h:59-77` (catalog entries)
- Modify: `rack-rp2350/test/test_dronecaster.cc:983-984` (`GoldenCase` table)

**Interfaces:**
- Consumes: goldens from Task 1; `sc::Splay` (`sc_kit_pan.h`), `sc::SinOsc` (`sc_kit_osc.h`), `sc::BPF` / `sc_kit_filter.h`, `sc::Saw` (`sc_kit_osc2.h`), `sc::Ctx`, `sc::DroneSpec`, `sc::Engine`.
- Produces: `sc::drones::HarmsWay{Init,Process}`, `sc::drones::Supersaw{Init,Process}`, `sc::kDroneHarmsWay`, `sc::kDroneSupersaw`, catalog ids 36 and 25.

- [ ] **Step 1: Write the failing golden tests**

Add `{36, "harms way"}` and `{25, "Supersaw"}` to `GoldenCase cases[]` (the existing loop already renders the engine at hz 55 / amp 0.4 / fade 1.0, gate-off at 144,000, 192,000 frames, and `compare(ref, got, 1e-6, 0.999999)`). Also add `CHECK(sc::Catalog(36) != nullptr, "id 36 ported")` and the same for 25 in a new `== T-D2 catalog ==` block.

- [ ] **Step 2: Run to verify they fail**

Run: `cd rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster`
Expected: `GOLDEN FAIL` for both new names and the two catalog CHECKs fail; all pre-existing sections stay green.

- [ ] **Step 3: Port the two graphs**

Read `dronecaster_ref/drones/harms way.scd` and `Supersaw.scd` first and mirror them in `drone_t2.h` using the `drone_trio.h` pattern: one state struct per drone allocated from `c.arena`, `Init(Ctx&)` primes every UGen from its source ctor, `Process(Ctx&, float* l, float* r, int n)` renders the full 64-sample block and writes `l`/`r`. Add `inline const DroneSpec kDroneHarmsWay` / `kDroneSupersaw` and wire ids 36/25 into `Catalog()`.

- [ ] **Step 4: Run to verify goldens pass**

Run: `cd rack-rp2350/test && make test_dronecaster && ./build/test_dronecaster`
Expected: both `GOLDEN PASS`, `PASS (0 failures)` overall. If a golden misses, diff the C types and ctor priming against the `.scd` (handoff §6 laws) — do not weaken the gate.

- [ ] **Step 5: Commit**

```bash
git add rack-rp2350/src/dronecaster/drones/ rack-rp2350/test/test_dronecaster.cc
git commit -m "[Add] dronecaster: T-D2 harms way + Supersaw — 2/2 goldens exact"
```

### Task 4: Thx (27) + Mt. Zion (15) + Mt. Lion (14)

**Files:** Modify `rack-rp2350/src/dronecaster/drones/drone_t2.h`, `drone_trio.h`, `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `sc::BLowPass`/BEQ, `LFNoise2`, `Mix`, `Pan2`, `Dust`, `Latch`, `Pulse`, `WhiteNoise`, `CombN`, `LeakDC`, `LFPulse` from the kit; golden WAVs from Task 1.
- Produces: `kDroneThx`, `kDroneMtZion`, `kDroneMtLion` at ids 27, 15, 14.

- [ ] **Step 1:** Add the three `{id, name}` rows to `GoldenCase` and three catalog `CHECK`s. Run `make test_dronecaster && ./build/test_dronecaster`; expect the three `GOLDEN FAIL`s and three failed CHECKs.
- [ ] **Step 2:** Read the three `.scd` graphs; port into `drone_t2.h` with the same state/Init/Process pattern. Note `Dust`/`WhiteNoise` share `Ctx::rgen` with every other noise UGen — ctor priming draws are part of the sequence (handoff §6).
- [ ] **Step 3:** Wire ids 27/15/14 into `Catalog()`. Run the gate; expect three `GOLDEN PASS`, overall `PASS (0 failures)`.
- [ ] **Step 4:** Commit `[Add] dronecaster: T-D2 Thx + Mt. Zion + Mt. Lion — 3/3 goldens exact`.

### Task 5: Hecker (10) + Fieldsteel (7)

**Files:** Modify `rack-rp2350/src/dronecaster/drones/drone_t2.h`, `drone_trio.h`, `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `sc::VarLag` / `EnvGen` (`sc_kit_envgen.h`), `SelectX` (`sc_sugar.h`), `LinExp` (kit defs/sugar), `Demand`+`Dseq` (`sc_kit_demand.h`), `SVF` (`sc_kit_sc3p_filter.h`), `LFNoise0/1`, `PinkNoise`, `BPF`, `LPF`, `Dust`, `Impulse`, `LFSaw`, `Splay`, `Mix`.
- Produces: `kDroneHecker`, `kDroneFieldsteel` at ids 10, 7.

- [ ] **Step 1:** Add the two golden cases + catalog CHECKs; run the gate and confirm both fail while the 5 already-ported T-D2 drones stay green.
- [ ] **Step 2:** Read `Hecker.scd` / `Fieldsteel.scd`; port using the same pattern. **Before relying on `VarLag`, verify its reference status** — commit `e853324` claims a VarLag ref while handoff §2 calls the expansion law-only. If no `ugen_ref/VarLag.expected.wav` test exists, port the expansion from `sc_kit_envgen.h:211` and note the limitation in the header; the drone golden remains the gate either way.
- [ ] **Step 3:** Wire ids 10/7 into `Catalog()`. Run the gate; expect `PASS (0 failures)`.
- [ ] **Step 4:** Commit `[Add] dronecaster: T-D2 Hecker + Fieldsteel — 2/2 goldens exact`.

### Task 6: Apparatus (0) + Twin Pks (30)

**Files:** Modify `rack-rp2350/src/dronecaster/drones/drone_t2.h`, `drone_trio.h`, `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `sc::Crackle`, `Dust2`, `PinkNoise`, `LFPar`, `LFTri`, `LFNoise2`, `Clip`, `HPF`, `LeakDC`, `Decimator`, `Compander`, `Limiter`, `DelayC`, `LinLin`, `LinExp`, `ControlRate`, `SinOsc`, `BPF`.
- Produces: `kDroneApparatus`, `kDroneTwinPks` at ids 0, 30.

- [ ] **Step 1:** Add two golden cases + catalog CHECKs; run and confirm both fail.
- [ ] **Step 2:** Read both `.scd`s; port. Both are noise-heavy — replicate each ctor's priming draw exactly, and keep `Dust2`'s coin-bias scheduling (handoff §6).
- [ ] **Step 3:** Wire ids 0/30 into `Catalog()`; run the gate; expect `PASS (0 failures)`.
- [ ] **Step 4:** Commit `[Add] dronecaster: T-D2 Apparatus + Twin Pks — 2/2 goldens exact`.

### Task 7: Starlids (24) + Sachiko (20)

**Files:** Modify `rack-rp2350/src/dronecaster/drones/drone_t2.h`, `drone_trio.h`, `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `sc::MoogLadder`, `MoogFF`, `SawDPW`, `PulseDPW`, `VarSaw`, `CombC/L`, `AllpassN`, `DelayN/C`, `RLPF`, `Splay`, `EnvGen`, `LFNoise0/1`, `LFTri`, `Mix`, `Pan2`, `LinExp/LinLin`.
- Produces: `kDroneStarlids`, `kDroneSachiko` at ids 24, 20.

- [ ] **Step 1:** Add two golden cases + catalog CHECKs; run and confirm both fail.
- [ ] **Step 2:** Read both `.scd`s; port. Delay-line sizes are graph constants — allocate them from the arena in `Init`, never on the stack.
- [ ] **Step 3:** Wire ids 24/20 into `Catalog()`; run the gate; expect `PASS (0 failures)`.
- [ ] **Step 4:** Commit `[Add] dronecaster: T-D2 Starlids + Sachiko — 2/2 goldens exact`.

### Task 8: Coil (2) + arena-swap regression gate

**Files:** Modify `rack-rp2350/src/dronecaster/drones/drone_t2.h`, `drone_trio.h`, `rack-rp2350/test/test_dronecaster.cc`

**Interfaces:**
- Consumes: `sc::CombL`, `DelayC/N`, `AllpassN`, `SinOscFB`, `TChoose`, `SelectX`, `Latch`, `Trig`, `Impulse`, `EnvGen`, `LFNoise0/1`, `WhiteNoise`, `Dust`, `Mix`, `Pan2`.
- Produces: `kDroneCoil` at id 2; the final 12-drone catalog; an arena-budget assertion.

- [ ] **Step 1:** Add Coil's golden case + catalog CHECK; run and confirm it fails.
- [ ] **Step 2:** Read `Coil.scd`; port into `drone_t2.h`; wire id 2 into `Catalog()`.
- [ ] **Step 3:** Add a `== T-D2 arena budget ==` test that, for each of the 12 ids, creates a fresh `Engine` on `static uint8_t mem[160*1024]`, sets a Trio drone first, swaps to the T-D2 drone, runs one 64-block, and asserts `arena.used() < 136*1024` (expose a `used()` accessor if absent). Add a `static_assert` on each state struct's `sizeof` in `drone_t2.h`.
- [ ] **Step 4:** Run the gate; expect Coil `GOLDEN PASS`, the budget test green, overall `PASS (0 failures)`.
- [ ] **Step 5:** Commit `[Add] dronecaster: T-D2 Coil + arena swap budget — 12/12 goldens exact`.

### Task 9: Expose the 12 drones in the adapter and panel

**Files:**
- Modify: `rack-rp2350/src/module_dronecaster.h:111-140` (CC 51 selector)
- Modify: `tools/rack_panel.html:545-550` (DRONECASTER tab selector)

**Interfaces:**
- Consumes: the 12 catalog ids from Tasks 3–8.
- Produces: CC 51 bytes 0–36 selecting any of the 15 ported ids; panel dropdown with 15 labels.

- [ ] **Step 1:** Add a host test in `test_dronecaster.cc` that walks ids 0..36 and asserts `Catalog(id) != nullptr` for exactly the 15 ported ids (`0,2,7,10,14,15,20,22,23,24,25,27,29,30,36`) and `nullptr` for the other 22. Run it — this is a guard and must already pass after Tasks 3–8; if it fails, fix the catalog before touching the adapter.

- [ ] **Step 2:** Keep CC 51 raw bytes `0..36` as direct catalog ids. Change the pot branch (`val > 36`) from the trio-only thirds mapping to a linear map over the **15 ported ids in catalog order**: `ported[k] = {0,2,7,10,14,15,20,22,23,24,25,27,29,30,36}`, `id = ported[(val-37)*15/91]`. Document the choice in a comment (my call; preserves one-slice-per-drone pot travel). The existing `last_bad_drone` guard stays unchanged.

- [ ] **Step 3:** In the panel's DRONECASTER `steps` entry, replace `labels:['SINE','SQUARE','TRIANGLE'], bytes:[22,23,29]` with 15 labels (uppercase drone names) and the 15 raw bytes above, in the same order. Leave ccMap/info text otherwise unchanged except the "(T-D1 trio)" note → "(T-D1 + T-D2, 15 drones)".

- [ ] **Step 4:** Verify the panel byte-sync headlessly as in the T-D1 panel convention (stubbed Web MIDI, compare the connect-sync CC bytes to the 15 ids), and run `pio run -d rack-rp2350` (expect SUCCESS; window ASSERTs must stay green — `tc_dronecaster` may grow, the window has headroom).

- [ ] **Step 5:** Commit `[Update] dronecaster: expose 15 ported drones on CC 51 + panel`.

### Task 10: Hardware acceptance + ledger (requires the user)

**Files:** Modify `.superpowers/sdd/progress.md`

**Interfaces:**
- Consumes: the firmware built in Task 9.
- Produces: a T-D2 acceptance entry with commit hashes and measured numbers.

- [ ] **Step 1:** Flash `rack-rp2350/.pio/build/rack/firmware.uf2` via BOOTSEL (one reader per serial port, by-id). Send CC 51 raw bytes for each of the 12 new ids from the panel/console; the 2 s report must show the new active drone, `nan=0`, `i2s stalls: 0 (+0)`, and `max Process` under the 1333 µs budget (T-D1 measured 189 µs; flag any drone over budget for profiling, do not silently accept).
- [ ] **Step 2:** Ask the user to listen to each of the 12 and confirm. Do **not** write "user-accepted" until they say so (AGENTS.md rule).
- [ ] **Step 3:** Append the T-D2 ledger entry: status, commit hashes, per-drone golden results, measured `max Process` per drone, and any law corrections found during the ports. Commit it.

## Self-review notes

- Spec coverage: phase table T-D2 addressed (kit already done; drones now done); spec §Rack integration CC 51/52/53/54, panel, presets (blob unchanged — no new params) all covered by Task 9; UGen additions (LinExp/LinLin) covered by Task 2 per the handoff §3 method; golden harness §Golden harness covered by Tasks 1–8; memory §Memory covered by Task 8; acceptance §Phases covered by Task 10.
- Out of scope by the spec: FFT drones, tape drones, animations/norns glue, chain participation — none appear in the task list.
- Type consistency: `DroneSpec{name, init, process}`, `Ctx{sr, rgen, hz, amp, tables, arena, state}`, `Catalog(int)` — used identically in every task.
