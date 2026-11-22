# LIRA-8 DSP review findings repair design

## Intent
Correct the verified LIRA-8 DSP, concurrency, output conversion, and golden-fixture issues documented in `docs/REVIEW-lira8-dsp-2026-11-21.md`, with regression tests that make the corrected behavior durable.

## Goals
- Correct the Pd `env~` dB conversion and expander slope, filter state isolation per delay bus, oscillator polyBLEP overlap, SELF modulation timing, and `line~` ramp tick quantization.
- Eliminate the LIRA-8 pending-parameter lost-update race and sensor-mask read/modify/write race, clamp audio-to-I2S conversions using the established `stmlib::Clip16` convention, and stop Pd golden renders from rewriting the checked-in parameter fixture.
- Add tests for each corrected law and for optimized oscillator paths matching the general path.
- Update the review and SDD ledger with changes and verification results.

## Non-goals and constraints
- No hardware flashing or claims of on-device/ear acceptance.
- Do not change the independent flaky Elements `test_reverb` canary.
- Preserve existing user working-tree changes; edit only the necessary tracked LIRA-8 source, harness, report, and ledger files (plus the same full-scale output-cast fix in Dronecaster, which the review explicitly identifies as the matching instance).
- Keep all DSP edits in the vendored/source files; no patch/sync layer.

## Approaches considered
- **Full repair (selected):** address F1–F8, plus the report-identified identical Dronecaster output-cast UB, and add isolated regression tests. This closes the review’s verified findings in one coherent pass.
- **Priority-only:** fix F1/F2/F7 and defer the remaining findings. Smaller, but leaves verified correctness and fixture issues unresolved, contrary to the requested full repair.

## Implementation outline
Split work by non-overlapping source/test ownership across subagents: (1) Pd primitive, expander, oscillator laws and their tests; (2) delay-bus/SELF/mailbox corrections and engine tests; (3) output saturation and golden fixture hygiene plus optimized-path regression coverage where it does not conflict. Integrate and review the combined diff, run focused LIRA-8 host tests and sanitizers, then attempt the full host suite and RP2350 build. Record actual results, including known unrelated test instability, without flashing hardware.

## Open implementation details
The builder may choose the smallest idiomatic atomic exchange strategy compatible with both host g++ and the RP2350 toolchain; may use a bounded pure-C++ fixture guard test to prove the golden patch has no active file write; and should preserve test behavior at 32 kHz and 44.1 kHz.
