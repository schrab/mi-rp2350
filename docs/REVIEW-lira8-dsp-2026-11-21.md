# LIRA-8 DSP review — porting accuracy + M33 optimizations (2026-11-21)

Read-only review of the rack-lira8 engine at HEAD `9044110` (branch
`rack-rp2350`). Scope: `rack-rp2350/src/lira8/` (all 9 headers), adapter
`rack-rp2350/src/module_lira8.h`, the `-O3` wrapper `rack-rp2350/lib/lira8_engine/`,
tc-window integration, and the host harness — cross-checked against the
pristine Pd patch in `/home/schrab/Projects/LIRA-8/LIRA-8_Pd_Standalone/`
(v3.3) and the resolved dumps
(`rack-rp2350/test/lira8_ref/delay_master.txt`, `comp.txt`). The initial
review was read-only; the approved repair and its validation are recorded below.

Method: parallel audit agents + coordinator verification. Every load-bearing
claim below was re-pinned by a probe run during the review (noted inline).
Pd laws verified against pd 0.57.0 source (`d_ctl.c`, `d_delay.c`,
`d_osc.c`, `d_filter.c`, `x_acoustics.c`) + the repo's own recorded probes,
not re-derived from every object.

---

## Verified findings

### F1 — HIGH: expander dB law is doubly wrong (engagement 30 dB early, up to 48 dB extra ducking)

1. `pd_powtodb` (`rack-rp2350/src/lira8/pd_primitives.h:58-60`) implements
   `100 + 20·log10(p)`. Pd's `env~` output is `powtodb(mean-square)` =
   `100 + 10·log10(power)` (pd `x_acoustics.c`; the repo's own recorded sine
   probe "sine → 96.99" in `PORTING-SPEC.md` §14.4 pins this:
   `10·log10(0.5) = −3.01`, while the port computes 93.98). Probe: the
   port's verbatim `EnvRms` accumulation gives power 0.2998 at the
   `test_env_rms` alignment → 89.5367 under the port's law — the test's
   pinned value was matched by hand-tuning `kStep=19318` under the wrong
   law. The recorded staircase "amp" values (0.29979/0.95298/1.0) map to
   the recorded dB values only as **RMS** amplitudes under `20·log10(rms)`
   — the same `10·log10(power)` law.
2. `Expander` slope (`rack-rp2350/src/lira8/lira_delay.h:183`) is
   `1 − 1/ratio`; pd's `expander~.pd` multiplies the
   `(env−100−thresh)·(1−1/max(1,ratio))` product by **2** (verified in
   `comp.txt` obj [13], conns 207/213) → slope 1.6 at ratio 5.

Compound effect: expansion engages below −30 dBFS rms instead of −60 dBFS,
and ducks quiet delay-loop/tail content up to 48 dB more than pd (at pd's
boundary P = 10⁻⁶: pd 0 dB, port −48 dB). Invisible in loud goldens (both
unity above −30 dBFS) — which is exactly why the earlier "expander bypass =
no change" A/B was flat.

Tests enshrine both wrongs: `test/test_pd_primitives.cc:110-149` (env
staircase) and `test/test_engine.cc:109` (the `(0−100+60)*0.8 = −32 dB`
comment). Correction to the original review's arithmetic: on silence, the
corrected Pd target is −64 dB, gain `0.000630957`; the old port target was
−32 dB, gain `0.0251189`. At the separate `P=10^-6` boundary, the old port's
−48 dB versus Pd's 0 dB comparison above remains valid.

**Fix order**: correct `powtodb` to `100 + 10·log10`, add the ×2 slope, then
re-pin both tests' expected values (they currently fail against a corrected
law).

### F2 — MEDIUM: one `hip~1`/`lop~4000` pair shared across both parallel delay buses

`rack-rp2350/src/lira8/lira_engine.h:421-422` declares single `hip1_` /
`lop4k_` members, used for both buses in `render_delay` (line 357). Pd has
two independent chains (`delay_master.txt` [31]/[32] and [69]/[70]). The
defaults already diverge the buses (TIME-1 94.574 vs TIME-2 56.745, MOD-1
40.736 vs MOD-2 47.925), so each bus's DC-block and 4 kHz tilt is fed the
other bus's sample every block — cross-contamination whenever the buses
differ. Plausible contributor to the still-open "fully-wet delay ~10 dB
brighter at 12.5–16 kHz" investigation (ledger 3253), unverified as the
cause. Compressor/expander/tanh are correctly per-bus (`comp_[2]`,
`exp_[2]`); only these two one-poles are shared. Fix: make them arrays[2].

### F3 — LOW/MEDIUM: polyBLEP gates use else-if; pd's gates overlap and sum

`rack-rp2350/src/lira8/lira_osc.h:126-142`: the patch computes both gated
terms and sums them into the final subtract; the port's `else if` drops the
rising term when `dt > 0.5` (windows overlap on `[1−dt, dt]`). Reachable
only above sr/2 — maxed TUNE-8 (hi note 131.22 → ≈16.4 kHz) × pitch max 2.0
≈ 32 kHz vs 22.05 kHz sr/2 at 44.1k (easier at the 32 kHz chain grid). That
regime is already heavily aliased in pd, so this is an artifact-character
delta, not a crash.

### F4 — LOW: SELF delay-modulation source lags pd by one extra block

`rack-rp2350/src/lira8/lira_engine.h:352-353` computes `self_next_` from
`self_buf_` (which still holds block N−1's writes) and it is consumed at
N+1 → the port replays N−1's writes where pd's zerodel `vd~` replays N's:
+1.45 ms lag on the SELF modulation source only (audio path timing
unaffected). Side note: the ledger's "500 Hz staircase" rationale for the
`self_buf_` replay is wrong (pd does replay the previous block, not hold
one sample), but the replay approach itself is pd-correct except for this
lag.

### F5 — LOW: `line~` ramp duration quantization differs

`rack-rp2350/src/lira8/pd_primitives.h:84-87` rounds ramp length to single
samples; pd floors the duration to whole 64-sample ticks
(`nticks = ms·sr/(1000·64)`, min 1 → `d_ctl.c:97-100`). Exact for the
ubiquitous 23.22 ms @ 44.1 kHz (16 ticks = 1024 samples); ~5.6% longer ramps
at 32 kHz chain-A mode (743 vs 704) and for the expander's 11 ms ramp (485
vs 448). Sub-block control timing only; no steady-state error.

### F6 — LOW: CC mailbox lost-update window (cross-core)

`pending_params_` is written from core 0 (MIDI/console via
`rack_module_control_change`, `main.cpp:740`) and core 1 (mod engine routed
CCs `mod_engine.h:1349`, preset replay on core 1), and read-then-cleared on
core 1 (`lira_engine.h:234-241`) with no synchronization: a core-0 write
landing between core 1's load and its `-1` clear is lost. Self-heals for
continuous params (the next CC applies); a discrete switch flip could be
dropped only in the swap-replay overlap window. Also `sensor_mask |= / &=`
(`module_lira8.h:209-215`) is a non-atomic RMW from both cores — report-only
(cosmetic). Fix direction: per-slot atomic exchange (`__atomic_exchange_n`)
or core confinement. Not urgent.

### F7 — LOW (latent): full-scale output cast is UB, reachability undetermined

`module_lira8.h:324` `static_cast<int16_t>(l[i] * gain * 32768.0f)` is UB at
exactly ±1.0; on truncating targets 1.0 → −32768 (full-scale sign-flip
click). Measured margin is razor-thin: loud-legal-config probe max 0.9744;
adversarial 105 s / 7-config sweep max 0.99975 with 0 samples ≥ 1.0 (the
static bound does not exclude crossing under adversarial alignment). Sibling
modules saturate via `stmlib::Clip16(static_cast<int32_t>(...))`
(`module_elements.h:563`, `module_clouds.h:437`, `module_rings.h:464`) —
lira8 should match. Same pattern at `module_dronecaster.h:232` (outside this
review's scope, same one-line fix).

### F8 — LOW (hygiene): checked-in `test/lira8_ref/patch/parameters.txt` is render residue

Mechanism verified: `gui.link.pd` wires any changed knob to
`s $1-write` → `_LIRA-8.pd` `r $0-write → del 500 → write ./parameters.txt`,
so **every pd golden render rewrites the checked-in file**. The committed
content is the frozen leftover of the onevoice render (commit f9c5942), not
the original's saved state. The engine's `apply_defaults()`
(`lira_engine.h:150-179`) instead matches the original's
`LIRA-8_Pd_Standalone/parameters.txt` and spec §12 term for term — the
faithful choice. **No existing golden is affected**: the five trajectory
scenarios carry a full 47-param t=0 init applied on both sides; the 8 probe
scenarios have partial init (the `scenarios.txt` header comment overstates
coverage) but all set del-mix=0, so the residual delay-path deltas are
inaudible. Hygiene actions: restore the file to the original's saved state
or disable the write path in the golden patch; correct the header comment.

---

## M33 / RP2350 optimization audit — verdict: sound

- **SRAM placement verified on the current ELF** (`nm`/`objdump -h` on
  `rack-rp2350/.pio/build/rack/firmware.elf`): `tc_lira8` = 19,640 B;
  `render_delay`, `render_internal_block`, `process_block_o3`, and the
  constprop'd `VoicePair::process` all live at `0x2006xxxx` in the window.
  Remaining flash-resident lira symbols are constructors/prepare paths only
  (one cache-miss burst per note-on, harmless) — matches the ledger's
  2026-11-19 audit.
- **No per-sample libm**: the only `bl` targets inside the tc section are
  the wrapped `expf/powf/log10f` veneers (control-rate/block-boundary) plus
  `memcpy` and the apply_params/prepare veneers. The `-O3` wrapper TU
  (`lib/lira8_engine/library.json` `-O3`) correctly defeats the
  earlephilhower core's appended `-Os`; the adapter calls
  `process_block_o3` and never `Process()` directly.
- **Fast-path equivalence proven but not regression-gated**: probes show
  `prepare+process_const` and `prepare_coeff+process_freq_prepared` are
  bit-identical to the general `process` path (48k samples, 0 diffs), and
  the vibrato-lerp / mux-fusion approximations carry measured dB gates
  (−59..−82 dB, corr 1.0000). But no automated test pins fast-path vs
  general equivalence — a future edit to `render_sample` could silently
  break it. Recommended: one host test.
- **Approximations re-checked and cleared**: fixed-point LFO phases (2³²/
  cycle, zero drift; `cos_lookup` 2048.0 exact-top guard returns 1.0),
  `exp_neg_range` degree-11 Taylor (worst 2.24e-6 over the guarded domain —
  the `< 1e-6` comment overstates by ~2×; 3.0e-7 over the actually
  reachable domain), int16 ±2 soft-knee storage (knee slope continuous,
  cap exactly 2.0), block-rate vibrato lerp (≤2.2e-4 full scale). All
  deliberate, documented, ledger-gated.
- Minor: `MasterOut` member-init `-Wreorder` (cosmetic, no dependency);
  dead code — `ModDelay::read_min_tap` (orphaned by the SELF-replay fix),
  `MasterOut::drive_raw_` (2 wasted powf per `set_drv`, never read),
  `ModDelay::bytes_for`, `Rpole`, test-only `TanhShaper`/`pd_random`;
  `render_sample`'s numeric guard (`lira_osc.h:113`) misses NaN (defensive
  only, no identified NaN source on the input paths).

## Confirmed-correct (high-value checks that passed)

Tune law `note = lo + (k/127)·(hi−lo)` (via the `msg $2 $1` swap in
`prm.tune.pd`; resolved with a live pd control-chain probe), triangle
crossfade `tri·(1−sharp)` (pd's scalar-left reverse-minus), cross-mod
`f·(1+0.005V)·(1+M·mod)`, delay input = raw current-block voice sum ×1/6
with noise only at the write summers, `vd~` 4-point taps/clamps/zerodel
verbatim (`d_delay.c:341-390`), gate `min(e²+hold,1)` for both oscs, PWM
`0.53125+0.025V`, per-pair vibrato RATE draws (744/930/728/920, dormant
seed), hyper-LFO rate/link/tri/sqr/sqr-lfo laws, master drive/dist/vol/
total-fb chain, `prm.source` routing table (kSrcA/kSrcB exact), all
one-block `s~/r~` semantics, delay carve arithmetic (176,416 B standalone /
128,016 B chain, no overlap, static_asserts hold), `apply_params` change
mask, ASan+UBSan clean on `test_engine`/`test_voice`.

## Test-suite health (side observation)

`make test` is not a reliable gate right now: `test_reverb` (the Elements
`FxEngine<21632>` canary, outside lira8) fails nondeterministically — same
freshly-built binary, 5 runs: 4 fail ("impulse response does not decay:
first-20-block peak 0.015020 vs last-10 0.630760", the number drifts between
runs), 1 passes. Same-g++ (16.2.1) clean rebuilds both fail and pass →
uninitialized-memory-dependent behavior (ASLR-sensitive) in that test path;
consistent with the ledger's "test_reverb flaky" note but worse than flaky —
worth an MSan pass sometime. The lira8 tests (`test_pd_primitives`,
`test_voice`, `test_engine`) are deterministic and green.

Fresh-render spot check: current-HEAD engine vs the committed pd `onevoice`
golden: aligned corr 0.838, rmsdiff −4.99 dB — consistent with the recorded
post-L4 state (corr retired as a gate; spectral exactness is that
scenario's gate). The pd golden generator still hangs (ledger 3261), so no
reference re-render was possible during this review.

## Suggested fix order

1. F1 expander law (powtodb `10·log10` + ×2 slope) + re-pin
   `test_pd_primitives`/`test_engine` expectations.
2. F2 per-bus `hip1_[2]`/`lop4k_[2]`, then re-run the wet-delay HF
   bisection against a pd reference.
3. F7 one-line `stmlib::Clip16` convention (lira8 + dronecaster).
4. F3/F4/F5/F6/F8 opportunistically; add the fast-path equivalence test.

## Approved repair — 2026-11-21/22 — LANDED `4ee6dde`, hardware-accepted

The user approved `docs/superpowers/plans/2026-11-21-lira8-dsp-review-repair.md`.
The following fixes are implemented against base `b6c94ff` and committed as
`4ee6dde` on `rack-rp2350` (2026-11-22, after the user's hardware
acceptance). The findings above describe the pre-repair code.

| Finding | Repair and regression evidence |
|---|---|
| F1 | `powtodb` uses `100+10*log10(power)` with the Pd power floor; expander slope includes ×2 and clamps ratios below one. Tests pin half-power 96.9897 dB, the envelope staircase, the unity threshold, silence target −64 dB/gain 0.000630957, and settled/exponential gain behavior. |
| F2 | Each bus owns a `DelayFilterChain` (`hip~1` then `lop~4000`). Tests check fresh-state independence and actual engine bus isolation. The wet-delay HF hypothesis remains unverified. |
| F3 | Both BLEP gates contribute when their windows overlap. The `dt=0.75` regression failed 12 checks before the fix and passes afterward. Both prepared paths match general rendering bit-for-bit across 14 × 48,000 samples per path at 32/44.1 kHz, varied PWM and block-boundary frequency changes. Base-tune coefficients with per-sample vibrato remain a deliberate approximation. |
| F4 | SELF staging filters the current block's float writes at half amplitude after the bus consumes the preceding SELF block; obsolete staging buffers are removed. Three-block tests check the staged values and actual following-block delay reads. |
| F5 | Positive ramp times floor to 64-sample ticks (minimum one); nonpositive times jump. Tests pin 23.22 ms → 1024/704 and 11 ms → 448/320 at 44.1/32 kHz. `Ramp::next()` retains increment-first indexing; Pd's output-first indexing remains a documented difference. |
| F6 | Pending parameter slots preserve float bits in lock-free 32-bit atomic store/exchange slots with the −1 sentinel. Concurrent writers/render tests preserve fractional values. Sensor-mask updates use atomic OR/AND in stable namespace storage, with atomic resets at both init entry points; reporting counts one snapshot without touching arena mask storage. |
| F7 | LIRA-8 and Dronecaster use the sibling `Clip16(static_cast<int32_t>(sample * 32768.0f * gain))` convention. Nineteen boundary/gain/truncation checks pass, including +1 → 32767 and −1 → −32768. |
| F8 | Golden generation removes the unique delayed parameter-save edge inside uniquely identified `gui.link` bounds, rejects missing/duplicate anchors and any remaining live save inlet, and retains parameter loading. Nine Python tests pass; two generations are byte-identical. Saved parameters match the pristine standalone file; scenario init coverage is accurately described. No reference WAVs were rendered. |

Validation of the combined changes:

- Fresh external build `/tmp/rack-lira8-integrated.TMPSNh/`: all 15 host binaries
  built; 14 passed. Only the unrelated, unchanged `test_reverb` failed:
  `impulse response does not decay: first-20-block peak 0.015020 vs
  last-10-block peak 0.630760`. The final nine Python fixture checks passed.
  Binaries ran from `/tmp/rack-lira8-suite-cwd.r7DCGt/` so relative output
  paths did not touch the repository's existing artifacts.
- ThreadSanitizer caught two mailbox races before the fix (exit 66), then
  passed `test_engine` with no race diagnostics in a fresh build
  `/tmp/rack-lira8-tsan.RbQMuE/` (exit 0).
- Combined `test_engine` and `test_voice` pass AddressSanitizer and
  UndefinedBehaviorSanitizer in `/tmp/rack-lira8-asan.rwgN1G/`.
  Output checks also pass UBSan plus float-cast-overflow. The output test
  additionally checks both production statements and rejects a narrowing
  mutation. A production-snippet mask harness with eight writers, 2,000
  arena resets and a reader passes TSAN; it uses MCU-independent state
  stubs and covers the report mask, not the engine's gate/level mailbox.
- `pio run -d rack-rp2350` succeeds: RAM 382,372 B, flash 1,647,996 B;
  `tc_lira8` is 20,032 B at SRAM `0x20063000`. Actual ELF inspection shows
  mailbox exchange and sensor OR/AND use inline `ldrex`/`strex` retry loops,
  publication uses `str`, and no atomic runtime helper symbols are linked.
  Existing resource-macro/extern warnings remain; host builds retain the
  pre-existing `MasterOut` member-order and unused `flips` warnings.
- All 32 pre-existing `test/build` artifact hashes remain unchanged, and its
  tracked binary diff is byte-identical to the initial state. No unrelated
  snapshots were edited. `git diff --check` passes.

The implementation tasks passed independent spec/quality review. Review
caught the arena report-mask lifetime gap; the stable-storage fix, stricter
fixture guard and production clipping checks passed scoped re-review.
Final broad review found no additional defects in the approved repairs.

**Hardware acceptance (2026-11-22, user):** the UF2 was flashed via the
documented runbook; the board booted clean on `mod=lira8` (439-447 µs /
1451 budget, i2s stalls 0, sens=0) and the user accepted by ear
("ok, sounds good"). The wet-delay HF comparison and a Pd reference
rerender remain pending. Final review also identified a pre-existing, separate
sensor-event mailbox race: `set_sensor()` writes `pending_sensors_` and
`pending_sensor_levels_` from both cores while rendering reads them. The
approved F6 repair and TSAN stress cover parameter slots and the report
mask; they do not establish race freedom for sensor-event publication.
That path needs a separate coherent gate/level publication repair and
concurrent sensor regression. The generated fixture SHA256 is
`4131b5de9a8608d694de605411d78711307321bccf4aba2a80f0fbb14ef8e574`.

## Limits

The original review ran with no device access; the repaired firmware was
subsequently flashed and accepted by the user (2026-11-22, numbers in the
"Hardware acceptance" note above). ARM codegen claims (int16 wrap, FMA
contraction under `-ffast-math`-class fusion) are reasoning + x86 probes,
not device-observed. Pd laws verified against pd 0.57.0 source + recorded
probes; the `LiraOsc` samphold-on-wrap law and bus sort order were verified
by the original porting batches and no contradicting evidence was found,
but were not re-derived here.
