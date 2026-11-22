# LIRA-8 DSP Review Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Resolve all verified LIRA-8 findings F1–F8 from the 2026-11-21 DSP review and add regression coverage for the changed laws and optimized paths.

**Architecture:** Keep the hand-ported engine header-only and source-of-truth in `rack-rp2350/src/lira8/`. Split implementation into three non-overlapping code tasks (1: Pd primitive/expander/ramp/delay state/mailbox; 2: oscillator BLEP and fast-path tests; 3: output conversion and golden-fixture safety), followed by integration and documentation. Host tests will build into a fresh `/tmp` directory to avoid overwriting the repository’s already-modified or deleted `test/build` artifacts.

**Tech Stack:** C++17, g++ host harness, Pure Data 0.57 laws, RP2350 PlatformIO firmware build, Python fixture generator.

**Spec:** `docs/superpowers/specs/2026-11-21-lira8-dsp-repair-design.md`; findings: `docs/REVIEW-lira8-dsp-2026-11-21.md`.

## Global Constraints

- Do not change the unrelated Elements `test_reverb` canary.
- Do not flash hardware or claim hardware acceptance.
- Preserve current unrelated worktree changes, including files under `rack-rp2350/test/build/` and untracked top-level source snapshots.
- Make no commit unless requested; record local/uncommitted state honestly in the SDD ledger.
- Keep firmware DSP edits in existing source-of-truth files; do not introduce patch/sync infrastructure.
- Match `stmlib::Clip16(static_cast<int32_t>(sample * 32768.0f * gain))` used by sibling adapters.

## Review Focus

1. **Env power near full scale and below the expander threshold:** Pd uses `100 + 10*log10(power)`; verify exact dB landmarks, corrected expander slope, and the `-64 dB` target at silence (`dbtorms(-64 + 100) ≈ 0.000630957`).
2. **Ramp duration at both supported sample rates and short/nonpositive times:** verify 64-sample tick quantization; 23.22 ms gives 1024 samples at 44.1 kHz and 704 at 32 kHz; 11 ms gives 448 and 320 respectively; nonpositive duration remains an immediate jump.
3. **High-frequency oscillator phases where the BLEP windows overlap:** test the combined rising/falling correction at `dt > 0.5`, and compare optimized and general oscillator paths over deterministic sample sequences.
4. **Different state and concurrent parameter writers:** prove two delay-filter chains do not share history, SELF feedback uses the preceding block’s newly completed writes, and concurrent `set_param`/render has no data race; also verify sensor-mask updates from both cores are atomic and its report-side snapshot is consistent. Run host ThreadSanitizer stress coverage for parameter slots and verify sensor-mask access sites structurally plus with the firmware build.
5. **Output and reference-fixture safety:** verify clipping at/beyond signed 16-bit full scale; regenerate the golden patch and verify its write-to-parameters path is disconnected, while its parameter-load path and restored fixture values remain intact.

---

### Task 1: Correct Pd Transfer, Ramp, and Delay-Engine State

**Files:**
- Modify: `rack-rp2350/src/lira8/pd_primitives.h`
- Modify: `rack-rp2350/src/lira8/lira_delay.h`
- Modify: `rack-rp2350/src/lira8/lira_engine.h`
- Modify: `rack-rp2350/test/test_pd_primitives.cc`
- Modify: `rack-rp2350/test/test_engine.cc`
- Modify: `rack-rp2350/test/Makefile` (link threaded engine test with `-pthread`; only add sanitizer target if it can avoid default build outputs)
- Modify: `rack-rp2350/test/lira8_ref/PD-PRIMITIVES.md`
- Modify: `rack-rp2350/test/lira8_ref/PORTING-SPEC.md`

**Interfaces:**
- Produces `l8pd::powtodb(float power)` with Pd `env~` semantics: `0` below the `1e-10` power floor, otherwise `100 + 10*log10(power)`.
- Produces `l8pd::line_samples(float ms, float sr)` returning `0` for `ms <= 0`, otherwise `64 * max(1, floor(ms*sr/(1000*64)))`.
- Produces `l8pd::expander_target_db(float env_db, float thresh_db, float ratio)` using `min(0, (env_db - 100 - thresh_db) * 2*(1 - 1/max(1,ratio)))`.
- Each feedback bus owns its own `Hip`/`Lop` state; `DelayFilterChain(float sr)::process(float sample)` filters one sample with independent hip~1 then lop~4000 history. A SELF staging helper consumes the current block's float writes and prepares the next block's modulator.
- Pending float values are bit-preserved in `uint32_t` slots with `memcpy` encode/decode helpers, `-1.0f` sentinel bits, and relaxed atomic store/exchange. Require IEEE-754 32-bit float and lock-free 32-bit atomics at compile time.

- [ ] **Step 1: Add failing primitive tests**
  - In `test_pd_primitives.cc`, assert `powtodb(1)=100`, `powtodb(0.5)≈96.9897`, `powtodb(0)=0`, and the env~ staircase values for the existing `kStep=19318` alignment become approximately `94.76835`, `99.94455`, then `100` dB.
  - Pin `line~` sample counts: `23.22 ms` → 1024 at 44.1 kHz and 704 at 32 kHz; `11 ms` → 448 and 320; a positive duration shorter than one tick → 64; `ms <= 0` remains an immediate jump.
  - Add `powtodb(1e-30f) == 0` to pin the low-power floor, and verify `expander_target_db(40, -60, 5) == 0` / unity gain at the `1e-6` power threshold.
  - In `test_engine.cc`, assert `expander_target_db(0, -60, 5) == -64 dB` and its gain conversion is approximately `0.000630957`; replace the existing no-op silence assertion with deterministic checks of the computed target and settled gain behavior.
- [ ] **Step 2: Run the focused tests and confirm they fail**

  Create a fresh empty scratch directory with `mktemp -d /tmp/rack-lira8-repair.XXXXXX` and retain its absolute path as `repair_build`. Throughout this plan, `/tmp/rack-lira8-repair/` in example commands denotes that actual fresh directory with a trailing slash. Use separate scratch directories for sanitizer builds. Do not reuse a pre-existing directory.

  Run from the repository root (the trailing slash in `BUILD` is required):
  `make -C rack-rp2350/test BUILD="$repair_build/" test_pd_primitives test_engine`

  Then execute `"$repair_build/test_pd_primitives"` and `"$repair_build/test_engine"`; named Make targets only compile the binaries. Every later verification step in this plan likewise includes executing the binaries it builds.

  Expected before implementation: failures on the dB landmarks, staircase values, ramp tick counts, and expander law.
- [ ] **Step 3: Correct the Pd laws**
  - Change `powtodb` to the power law above; preserve the Pd zero/sub-floor behavior.
  - Compute positive `line~` durations in 64-sample ticks using `max(1, floor(ms*sr/(1000*64))) * 64`; keep nonpositive durations immediate. Reuse this conversion in `Expander`.
  - Correct expander slope to `2*(1 - 1/ratio)` and retain its existing exponential gain ramp (linear ramp in dB).
  - Update the Pd primitive documentation to remove the old `20*log10(power)` claim and record the corrected tick law.
- [ ] **Step 4: Verify the primitive/expander tests pass**

  Run and execute both focused binaries:
  `make -C rack-rp2350/test BUILD="$repair_build/" test_pd_primitives test_engine`
  `"$repair_build/test_pd_primitives" && "$repair_build/test_engine"`

  Expected: both binaries report all green.
- [ ] **Step 5: Add failing delay-engine tests**
  - Test the proposed `DelayFilterChain` value type directly: after nonzero input has changed one instance’s `Hip`/`Lop` history, a second independent instance must still match its fresh-state reference.
  - Add a focused SELF staging test using a `Lira8EngineTestAccess` friend or other test-only access (no public runtime API): feed one known 64-sample write buffer, stage the filtered half-amplitude values, and verify those exact values are consumed by the following block. A second distinct block proves it does not replay the older block.
  - Add a host-only threaded stress test with a render consumer and concurrent writers calling `set_param` only; do not call other non-atomic engine APIs concurrently.
- [ ] **Step 6: Fix delay and mailbox ownership**
  - Make hip~1/lop~4000 filter state per bus (prefer a small `DelayFilterChain` value type if it keeps the paired state together and testable).
  - Stage SELF output from current `write_buf_` data for consumption on the following block; remove stale staging storage if no longer needed. Preserve one-block timing and the existing per-sample SELF replay behavior otherwise.
  - Publish/consume float pending slots with the type-generic `__atomic_store` / `__atomic_exchange` builtins (the `_n` forms do not accept float), or use a lock-free 32-bit integer representation with bit-preserving conversion. Use atomic stores on every writer and a single atomic exchange on consumption, keeping the `-1.0f` empty sentinel. Relaxed ordering suffices for independently published values; do not use a float-to-integer numeric cast that loses fractional CC values. Verify lock-free ARM code generation and absence of unresolved atomic runtime helpers.
  - Preserve `Ramp::next()`'s current increment-first sample indexing; this task corrects duration quantization, not the additional Pd output-first indexing difference. Record that remaining difference explicitly.
- [ ] **Step 7: Verify delay/mailbox tests**

  Run the focused binaries as above. Create a fresh `tsan_build` directory, then compile and execute the threaded `test_engine` with `-fsanitize=thread -pthread` and the Makefile’s normal include roots; expected: no race report for pending parameter publication/consumption. Do not reuse the shared scratch output directories for sanitizer builds.

### Task 2: Fix Oscillator BLEP Overlap and Gate Fast Paths

**Files:**
- Modify: `rack-rp2350/src/lira8/lira_osc.h`
- Modify: `rack-rp2350/test/test_voice.cc`

**Interfaces:**
- The general `LiraOsc::process`, constant-frequency `prepare`/`process_const`, and prepared-coefficient `prepare_coeff`/`process_freq_prepared` APIs remain unchanged.

- [ ] **Step 1: Add failing oscillator regression tests**
  - Exercise a deterministic `dt=0.75` sequence that reaches the overlapping BLEP region and assert the sample matches the sum of both independently gated BLEP terms (the rising and falling windows can both contribute).
  - Compare general rendering against `prepare`/`process_const` and against `prepare_coeff`/`process_freq_prepared` with matching frequency/coefficient inputs, identical starting states, varied PWM, both sample rates, and frequency changes at block boundaries over at least 48,000 samples. Require bit identity for these equivalent-input cases. Separately, a coefficient prepared at the base tune with vibrato-changing per-sample frequency is a deliberate approximation; do not claim that case is bit-exact or conflate it with the equivalence gate.
- [ ] **Step 2: Run `test_voice` and confirm the BLEP regression fails before the fix**

  Run: `make -C rack-rp2350/test BUILD=/tmp/rack-lira8-repair/ test_voice`

  Expected before implementation: the overlap assertion fails; fast-path comparisons establish the current baseline.
- [ ] **Step 3: Sum both BLEP terms when their gates overlap**

  Replace the mutually exclusive falling-edge branches in `LiraOsc::render_sample` with independent gates while retaining the current polynomial and signs.
- [ ] **Step 4: Verify oscillator tests pass**

  Run: `make -C rack-rp2350/test BUILD=/tmp/rack-lira8-repair/ test_voice`

  Expected: `test_voice: all green`, including the overlap and fast-path comparisons.

### Task 3: Saturate Adapter Output and Make Golden Fixtures Reproducible

**Files:**
- Modify: `rack-rp2350/src/module_lira8.h` (output clamp and atomic sensor mask)
- Modify: `rack-rp2350/src/main.cpp` (atomic sensor-mask snapshot for reporting)
- Modify: `rack-rp2350/src/module_dronecaster.h`
- Modify: `rack-rp2350/test/Makefile` (only if adding a host output-conversion test target)
- Create/modify: `rack-rp2350/test/test_output_clip.cc` (only if needed to give the shared conversion a direct host test)
- Modify: `rack-rp2350/test/lira8_ref/gen_golden_patch.py`
- Create: `rack-rp2350/test/lira8_ref/test_golden_patch.py` (standard-library `unittest` for deterministic generation, no active save edge, intact read path, and unique anchor failure)
- Regenerate: `rack-rp2350/test/lira8_ref/patch/_LIRA-8-golden.pd`
- Restore: `rack-rp2350/test/lira8_ref/patch/parameters.txt` from `/home/schrab/Projects/LIRA-8/LIRA-8_Pd_Standalone/parameters.txt`
- Modify: `rack-rp2350/test/lira8_ref/scenarios.txt`

**Interfaces:**
- Adapter output stays I2S signed 16-bit; clamp the scaled sample via the project’s `stmlib::Clip16` convention in both LIRA-8 and the matching Dronecaster conversion.
- Golden patch generation must preserve parameter reading but disconnect the `del 500` write-to-`parameters.txt` path every time it is regenerated.

- [ ] **Step 1: Add failing safety/reproducibility checks**
  - With the project’s exact float-to-int32 cast followed by `stmlib::Clip16`, pin the expected conversion: `+1.0 → 32767`, `−1.0 → −32768`, `+2.0 → 32767`, `−2.0 → −32768`, and `0.5 → 16384` (truncation toward zero).
  - Verify golden patch generation is deterministic and contains no active connection from the delayed write trigger to `write ./parameters.txt`, while retaining `read ./parameters.txt`.
- [ ] **Step 2: Run the checks and confirm the fixture guard detects the current write connection**
- [ ] **Step 3: Apply `Clip16` at both adapter output loops**

  Include the existing stmlib DSP declaration as needed. Match the sibling adapter multiply order and clamp before the I2S write.
- [ ] **Step 4: Make golden generation write-safe**
  - Add a checked, unique-match transformation in `gen_golden_patch.py` that removes only the delayed write connection from its input patch; fail generation if the expected connection is absent or duplicated.
  - Regenerate the checked-in golden patch and verify read/load remains wired.
  - Restore the tracked saved-parameter fixture from the pristine standalone file.
  - Correct the `scenarios.txt` header to distinguish the five full 47-parameter trajectories and full-init `drone-nodelay` from the eight partial-init probe scenarios (all with delay mix zero).
- [ ] **Step 5: Verify adapter and fixture changes**

  Run the output-conversion test if created, regenerate the golden wrapper twice and compare outputs, assert the write connection is absent and the read path remains present, and compare restored `parameters.txt` with the pristine standalone fixture. Do not run Pd rendering as part of this task; `render_ref.sh` removes existing WAVs and its legacy driver does not provide a safe regeneration path.
- [ ] **Step 6: Build the RP2350 firmware**

  Run: `pio run -d rack-rp2350`

  Expected: the rack environment builds. Do not flash.

### Task 4: Integrate and Update Review Records

**Files:**
- Modify: `docs/REVIEW-lira8-dsp-2026-11-21.md`
- Modify: `.superpowers/sdd/progress.md`

- [ ] **Step 1: Review combined source and test diffs**

  Ensure tasks 1–3 did not alter the user’s unrelated working-tree changes. Verify F1–F8 each map to the actual code and a test or explicit structural check. Check that no outdated comment still claims the old dB law, ramp duration, shared delay filter state, SELF block lag, or write-enabled golden fixture.
- [ ] **Step 2: Run focused LIRA-8 tests and sanitizers from a clean external build directory**

  Run: `make -C rack-rp2350/test BUILD=/tmp/rack-lira8-repair/ test_pd_primitives test_voice test_engine test_harmony`

  Run `test_engine` under ThreadSanitizer and the engine/voice tests under AddressSanitizer + UndefinedBehaviorSanitizer. Preserve the repo’s pre-existing `test/build` deletions and modified binaries; do not run `make clean`.
- [ ] **Step 3: Attempt the broader host suite only if artifact safety is confirmed**

  Check each test’s output paths first. If the broader suite would rewrite pre-existing files under `rack-rp2350/test/build/`, skip it and state that; do not overwrite or restore user files. Report the known unrelated nondeterministic `test_reverb` separately if it is run.
- [ ] **Step 4: Update review and SDD ledger with verified status**

  Record exact tests/build results, deviations, and remaining hardware acceptance limits. Correct the report’s own silence arithmetic: with env floor `0`, threshold `-60`, and ratio `5`, the corrected target is `-64 dB` (`0.000630957`), not `0.063`; retain the correct `-48 dB` comparison at the report’s separate `P=10^-6` boundary only if revalidated against the corrected law. Mark any unrun acceptance as pending. Do not invent a commit hash; no commit is planned.

**End-to-end verification:** focused host tests pass; TSAN/ASAN/UBSAN results are recorded; firmware build result is reported; generated fixture is stable and write-safe; unrelated worktree changes remain unchanged. Hardware behavior remains unverified until the user tests it.
