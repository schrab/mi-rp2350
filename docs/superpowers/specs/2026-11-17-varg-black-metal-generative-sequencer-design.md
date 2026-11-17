# VARG — black-metal generative sequencer (design spec)

**Date:** 2026-11-17
**Status:** approved design (2026-11-17, user-confirmed in brainstorm); implementation plan follows separately
**Sources:** `docs/Architectural Frameworks and Implementation Patterns for Embedded Generative Melodic Sequencers.md`, `docs/hyp1-mono-arp-algorithms.md`, research clipping "Black Metal Chords, Scales, Harmonies, and Progressions" (2026-11-17), existing MOD-ARP implementation in `rack-rp2350/src/mod_engine.h`

## 1. Problem

The rack's generative engine (MOD-ARP, sources arp1/arp2) has five monophonic
chord-arpeggiator algorithms (HYP-1, MOTIF, MARKOV, METRO, DEJAVU) built on
the generic W7-diatonic / Neo-Riemannian chord machinery. There is no
generator with an idiomatic **black-metal** voice: tremolo/ostinato riffs,
power/minor/tritone/dim/aug/add9 voicings, non-diatonic progressions
(Weitzmann regions, chromatic mediants, parallel planing), section-level key
shifts. VARG adds a sixth algorithm that generates black-metal **chords,
harmonies and rhythmic patterns** from the genre research.

## 2. Decisions locked with the user (2026-11-17)

1. **Output = mono riff + harmony.** VARG plays one note at a time
   (tremolo/ostinato riff lines over the generated progression) through the
   existing ARP voice path (`shared_note`/`shared_gate` + module triggers),
   and publishes the generated chord (root, quality, chord-tone pool) to the
   LIRA-8 8-voice harmony snapshot (CC 54 ARP HARM), which already retunes
   its voices from the routed MOD-ARP. No new polyphonic output path in v1.
2. **Integration = 6th MOD-ARP algorithm.** `MOD_ARP_VARG = 5`; the ALGO pot
   becomes 18 positions (6 algos × 3 octave ranges; VARG 1/2/3 = 1/2/3
   octaves). Reuses the C3 MOD-ARP page, shared ROOT/SCALE/TRANS, p1–p4
   knobs, clock/div/dest, preset record.
3. **Knob mapping = RIFF / PROG / VOICE / DISS** (p1–p4; ROOT/SCALE stay
   shared, DIV stays the clock division).
4. **Harmony engine = archetype library driven by W7/NEO kernels.** PROG
   selects 8 research-derived archetypes; diatonic ones walk via
   `DIATONIC_W7_WEIGHTS` × `CHORD_MARKOV_7X7`, chromatic ones via
   `apply_neo_riemannian()` with minor-first weighting. W7 and NEO are the
   transition machinery, not decoration (explicit user requirement).
5. **Web panel = out of scope v1** (panel has no ARP section today; C3 +
   console are the control surfaces).

## 3. Architecture

### 3.1 Pure logic header

New `rack-rp2350/src/varg.h` — no Arduino/pico/MCU headers, includes only
`gen_tables.h` (same portability discipline as `src/lira8/lira_harmony.h`,
which is compiled by the host test harness). `mod_engine.h` adapts
`varg::State` ↔ `ModSource`/`ModArpExt` and reuses the existing clock bus,
step timing, gate window, pitch glide, accent capacitor and destination
routing — none of that changes.

### 3.2 State

Per-instance `varg::State` in the `ModArpExt.st` union (~16 B; the union is
already ≥32 B via the Deja-Vu member):

| Field | Purpose |
|---|---|
| `prog_idx`, `chord_idx`, `prev_deg` | archetype, position in its cycle, last W7 degree |
| `root_pc`, `quality`, `is_minor`, `anchor_pc` | current chord (NEO/Weitzmann state) |
| `bars`, `section` | bars since last section change; section index |
| `pool[6]`, `pool_n` | **absolute semitone offsets from the chord root** for the current bar |

Quality ids: 0 minor, 1 major, 2 power, 3 dim, 4 aug, 5 add9 (third per
`is_minor`).

### 3.3 Chord pool and note rendering

Each bar, `varg::build_bar()` derives the chord-tone pool from the current
chord (root + quality + VOICE voicing, section 5). Riff steps store **pool
indices** in `ModArpStep.off` (not global scale degrees), so non-diatonic
chords need no change to `degree_to_midi_folded()` semantics.

New `varg::calc_note()`:

```
note = chord_root_midi + pool[off mod pool_n] + 12 * (oct + off / pool_n)
```

then folds into the 28–96 MIDI window by octave shifts (pitch class
preserved). `mod_arp_calc_note()` branches to it for `algo == MOD_ARP_VARG`.

## 4. Harmony engine (PROG, p2)

Eight archetypes, `(p2 * 8) / 101`:

| # | Name | Chord vocabulary | Transition kernel |
|---|---|---|---|
| 0 | AEOLIAN | i, bVI, bVII (minor triads) | **W7**: CDF over `DIATONIC_W7_WEIGHTS[d] * CHORD_MARKOV_7X7[prev][d]`, restricted to degrees {0,5,6} |
| 1 | OSTINATO | i, bIII, bVI, bVII | same W7/Markov kernel, degrees {0,2,5,6} |
| 2 | WEITZMANN | six triads around an augmented anchor: minors rooted on `anchor+1`, majors on `anchor` | region cycle (documented Leviathan/Shining motion) + NEO pivot mutations |
| 3 | MEDIANT | minor triads a major/minor third apart | **NEO** `R/L/N/S`, minor-first weighted |
| 4 | ADD9DIM | minor add9, dim7(add6), minor #vii | fixed 4-chord seed (Am(add9)–D#dim7(add6)–G#m–A) + W7/Markov substitutions |
| 5 | HALFTRN | power/minor chords, root motion ±1 and ±6 | fixed root-motion set + NEO substitution |
| 6 | B2B5 | Phrygian power-chord pool (root, b2, b3, 4, 5, b7 + chromatic b5/#vii) | W7 weights on Phrygian degrees + chromatic b2/b5 injection |
| 7 | NEOFREE | all 24 major/minor triads | **NEO** `P/L/R/N/S/H` weighted; W7 pull as fallback |

Kernel details:

- **W7 path**: restricted CDF over the archetype's allowed degrees using both
  the empirical W7 weights and the 7×7 voice-leading matrix; the picked
  degree maps to a chord root through the shared scale.
- **NEO path**: `apply_neo_riemannian()` on `(root_pc, is_minor)`; black-metal
  weighting prefers minor outcomes and `N/R/L` motion, `S/H` rarer. The
  existing shared `TRANS` pot (`neo_op`, CC 34 param 12) still pins/forces
  the operator.
- **ROOT modes** (shared ROOT pot): `MIDI`/`PARAL` set the tonic as today;
  **`W7` forces the W7 kernel** for the selected archetype; **`NEO` forces
  the NEO kernel** and honors `TRANS`. This keeps W7/NEO involved in every
  archetype, per the user requirement.
- **DISS** (p4): adds NEO substitutions on chord changes, chromatic planing
  (±1 semitone root shifts), tritone/dim/aug color, and — every 8 bars — a
  tonal-center shift with probability scaled by DISS (section-level key
  shifts per the research). DISS 0 = strict archetype behavior.
- **Weitzmann math** (verified against the research's `{Bb, D, F#}` anchor):
  for anchor pcs `{a, a+4, a+8}`, minor triads root on `a+1` (Bm {B,D,F#},
  Ebm {Eb,Gb,Bb}, Gm {G,Bb,D}) and major triads root on `a` (D {D,F#,A},
  Gb {Gb,Bb,Db}, Bb {Bb,D,F}). The documented cycle Ebm–Cbm/Bm–Gm–Bb is the
  archetype's default 4-chord pass.

## 5. Rhythm and voicing

### RIFF (p1) — six zones, `(p1 * 6) / 101`

| Zone | Grid | Behavior |
|---|---|---|
| SUSTAIN | steps 0, 8, held (slide) | one chord tone per half-bar; drone feel |
| HALF | quarters 0,4,8,12 | root/fifth, half-time |
| 8TH | even steps | 8th-note tremolo |
| 16TH | all steps | full tremolo picking |
| GALLOP | 3+1 cells (0-1-2 / 4-5-6 …) | accents on beat roots |
| BLAST | every step | root/octave alternation, accents every 4 |

Gate density rises within each zone; downbeats always anchored. RIFF=BLAST
at DIV 1/16 is a double-time feel; a true 32nd blast uses DIV 1/32 (all 16
steps then span a half-bar of 16ths — documented in the README).

### VOICE (p3) — six pool builders, `(p3 * 6) / 101`

The harmony engine supplies root + quality; VOICE selects the voicing:

| VOICE | Pool (semitones from chord root) |
|---|---|
| POWER | `{0, 7, 12}` |
| TRIAD | `{0, 3/4, 7, 12}` (third per chord quality) |
| TRITONE | `{0, 6, 7, 12}` |
| DIMAUG | `{0, 3, 6, 9}` dim / `{0, 4, 8, 12}` aug (color per DISS) |
| ADD9 | `{0, 3/4, 7, 14}` |
| PEDAL | chord tones + fixed tonic/open-string pedal note (b2/b9 friction) |

### Riff grammar

Deterministic 4-step cells (call/response, not white noise): root/fifth on
strong beats, chord-tone cycling on weak beats, octave displacement in
tremolo zones. DISS injects b2/tritone passing tones and mutates cells at
bar boundaries.

## 6. Lifecycle

- Chord changes are **bar-quantized** (16 steps of VARG's division). VARG
  branches around the generic `mod_arp_next_chord()` drift and the HYP-1
  turnaround fills (`mod_arp_apply_turnaround` / `ext.dd_fill` stay inert for
  VARG) — the two state machines never double-advance.
- Per bar: `varg::advance()` (progression + section state) →
  `varg::build_bar()` (pool + 16-step riff) → steps render through the
  existing path (retrig edge, ~55 ms pitch glide, accent capacitor, 55%/100%
  gate window, `mod_fire_trigger`).
- **LIRA-8 publish**: `mod_lira_harmony_publish()` gains a VARG branch —
  root = chord root (folded), `scale_st` = pool, `chord_deg` = 0. The
  existing group voicing (voices 1–4 bass stack, 5–8 scale chord) and the
  CC 25/26 exact-note mappings keep working unchanged.
- **Both ARPs on VARG**: each keeps its own progression (shared ROOT/SCALE
  still locks the key); the first routed ARP remains the LIRA-8 harmony
  source. Documented limitation for v1.

## 7. Firmware integration points

| Site | Change |
|---|---|
| `ModArpAlgo` | add `MOD_ARP_VARG = 5`; new `MOD_ARP_ALGO_COUNT = 6`; replace hardcoded `% 5` / `15`-choice math with the constant |
| `ModArpExt.st` union | add the `varg` struct (section 3.2) |
| `mod_arp_rebuild_pattern()` | `case MOD_ARP_VARG: mod_varg_build(s, ext)` |
| `mod_arp_calc_note()` | VARG branch → `varg::calc_note()` |
| Bar-line block in `mod_engine_process_block()` | VARG skips generic drift/fill; calls `mod_varg_bar()` |
| `mod_lira_harmony_publish()` | VARG branch (section 6) |
| `mod_apply_cc_value()` param 0 | algo choice quantizer 15 → 18 steps |
| Console parser | `varg` alias beside `hyp1/motif/markov/metro/dejavu`; `mod?` gains a VARG line (prog, chord root/quality, bar, pool) |
| `mod_engine_init()` | defaults unchanged (HYP-1/MOTIF); VARG never a default |
| `preset.h` | **no format change** — algo=5 + p[0..3] persist in the existing 28-B source record (24-B payload); progression state is transient (regenerated from seed/params on load) |

## 8. UI

- **C3 (`rack-uimcu`), in lockstep with the firmware commit**: `kArpAlgoNames[18]`
  (+ `VARG 1/2/3`), ALGO pot buckets 15 → 18, `potToDisplayValue` choice math
  15 → 18, VARG value formatting (RIFF 6 zones, PROG 8 archetypes, VOICE 6
  voicings, DISS %). The ledger records the soft-takeover lockout trap when
  firmware echo and C3 bucket math disagree — both sides land in one commit.
- **RP2350 `ui_link.h`**: `ui_aux_cell_label()` / `ui_aux_cell_value()` algo
  modulo 5 → 6 and choice width 15 → 18; `kArp1Labels`/`kArp2Labels` grow to
  `[6][4]` with `P1 RIFF / P1 PROG / P1 VOICE / P1 DISS` (dynamic per-algo
  labels already exist).
- **Web panel**: out of scope v1 (no ARP section exists); optional stretch.

## 9. Tests

New host test `rack-rp2350/test/test_varg.cc` (+ Makefile entry) against the
pure `varg.h`:

1. Pool builders per VOICE/quality — exact semitone sets.
2. Note folding stays 28–96 and preserves pitch class.
3. W7 restricted CDF never leaves the archetype's degrees; NEO ops match
   `NEO_RIEMANNIAN_DELTAS`; Weitzmann region membership (minors on
   `anchor+1`, majors on `anchor`).
4. Seeded 8-bar progression runs produce deterministic golden chord
   sequences for all 8 archetypes.
5. RIFF zones: expected gate/step patterns per zone; every `step.off` within
   the pool; accents on beat roots.
6. DISS=0 → no chromatic substitution; DISS=100 → substitutions occur (seeded).
7. LIRA-8 snapshot fill from a VARG pool (`lira_harmony.h` structures).

Firmware gate: all five `pio` envs compile; the host test runs via the
existing test Makefile (`make -C rack-rp2350/test test_varg`).

## 10. Bench & acceptance

1. Build → verify "Project build complete" (no compound build+flash; verify
   before flash per project rule); flash RP2350 + C3 by the standard
   procedures (1200-baud touch UF2; `build_uimcu.bat flash -p COM22`).
2. Console: `mod arp1 algo varg`, `dest voice`; sweep RIFF/PROG/VOICE/DISS;
   `mod?` VARG line; 2 s report shows mod engine cost within the ~50 µs gate
   and `i2s stalls: 0`.
3. LIRA-8: arp1 = VARG routed, CC 54 ARP HARM on — voices follow the
   generated progression.
4. W7/NEO: ROOT `w7` vs `neo` with TRANS sweep changes progression
   character; NEOFREE walks transform chains.
5. Preset dump/collect/load roundtrip with VARG selected; chain mode
   (elements→clouds, lira8→clouds) unchanged.
6. **User listens** and confirms the black-metal character — firmware
   acceptance requires their ears/eyes, not just a successful build.

## 11. Risks (pre-registered)

- C3/firmware algo-bucket mismatch → pot soft-takeover lockout (known trap
  from the ledger); mitigated by a lockstep commit.
- `mod_arp_rebuild_scale()` call order must not clobber VARG's pool after
  `build_bar()` (VARG applies its pool last).
- Generic bar-line drift / fill logic must be fully guarded for VARG.
- RIFF=BLAST at DIV 1/16 is a double-time feel; true 32nd blast needs DIV
  1/32 (README note).
- Both ARPs on VARG = independent progressions (documented limitation).

## 12. Out of scope (v1)

- Polyphonic chord fan-out (true simultaneous 3–4 note chords to module
  voice stacks).
- Web-panel ARP section.
- Sub-16th in-step ratchets for blast beats (use DIV 1/32).
- VARG-specific cadential turnaround fills (section shifts replace them).
