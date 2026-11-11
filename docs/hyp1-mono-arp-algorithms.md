# HYP-1 algorithms for a monophonic chord-arpeggio generator

Everything needed to generate **monophonic notes** from HYP-1's harmony — an arpeggiator
that plays a chord output — with HYP-1's **pattern switching** (bar-quantized change, fill
bar, mutation). All references are to `firmware/src/dsp/amb.c` / `amb.h` in this repository
unless noted. A complete, self-contained engine assembled from these algorithms is in §8.

> **Licence**: the quoted code is GPL-3.0-only (see `LICENSING.md`). The assembled engine in
> §8 is a derivative of it and inherits GPL-3.0. If you need a different licence for your
> ESP32-C3 firmware, re-implement from the *descriptions* (algorithms/probabilities are not
> copyrightable; this code's expression is). §8's file states this in its header.

---

## 1. The data model: what a "pattern" is

HYP-1's monophonic voice (the acid line) is a **loop of 1..16 sixteenth-steps**, each step
holding five decisions. This is the whole pattern format:

```c
/* amb.c:948-950 */
static int8_t  ac_off[EN_NSTEP], ac_oct[EN_NSTEP];   /* each step: scale steps above the chord's degree, octave */
static uint16_t ac_gate, ac_accent, ac_slide;        /* bit s: step s plays; is accented; slides into the next */
static int ac_pos;                                   /* the step playing now */
```

| field | meaning | range |
|---|---|---|
| `ac_gate` (bit s) | the step sounds or is a rest | 0/1 per step |
| `ac_off[s]` | note = **scale steps above the chord's degree** | any int (wrapped by the scale) |
| `ac_oct[s]` | octave above the base register | 0..RANGE−1 (RANGE = 1..3) |
| `ac_accent` (bit s) | accented: louder + filter opens further + faster decay | 0/1 |
| `ac_slide` (bit s) | this step **slides into the next** (legato, no retrigger) | 0/1 |

The crucial design decision: **note offsets are stored relative to the chord's degree, not
as absolute notes**. The chord drifts through the scale (§3); the pattern is re-tiled onto
whatever chord is current at the moment a step fires. A pattern written over chord I
automatically becomes a pattern over chord IV. This is why the riff "follows the chord".

---

## 2. The scale and the chord

### 2.1 Scale = 12-bit pitch-class mask

```c
/* amb.h:41-42 — the modes; amb.c:17-19 — the table.
 * Bit i = root + i semitones. Bit 0 (root) is always forced on. */
enum { MODE_IONIAN, MODE_DORIAN, MODE_PHRYGIAN, MODE_LYDIAN, MODE_MIXOLYDIAN,
       MODE_AEOLIAN, MODE_PENT_MAJ, MODE_PENT_MIN, EN_NMODES };
const uint16_t EN_MODE_POOL[EN_NMODES] = {
    B(0,2,4,5,7,9,11), B(0,2,3,5,7,9,10), B(0,1,3,5,7,8,10), B(0,2,4,6,7,9,11),
    B(0,2,4,5,7,9,10), B(0,2,3,5,7,8,10), B(0,2,4,7,9,9,9),  B(0,3,5,7,10,10,10)};

/* amb.c:325-333 — expand mask to ascending semitone steps; returns count */
int en_scale(uint32_t pool, int *steps)
{
    int i, m = 0;
    pool |= 1u;
    for (i = 0; i < 12; i++)
        if (pool >> i & 1u)
            steps[m++] = i;
    return m;
}
```

`static uint32_t pool` + `static int nsteps, steps[12]` are the harmonic ground truth
(`amb.c:498-499`). `pool_set()` (`amb.c:1881`) masks to 12 bits, forces bit 0, re-expands,
and re-targets the voices.

### 2.2 Chord = pitch classes built on a scale degree

The chord's degree lives in `static int deg` (`amb.c:501`) — the one piece of harmonic
state. `en_chord_kind()` builds 3–4 pitch classes on any degree:

```c
/* amb.c:337-367 */
int en_chord_kind(uint32_t pool, int root, int deg, int kind, int *pcs)
{
    /* the scale steps above the degree to take, in order */
    static const uint8_t OFF[5][7] = {
        {0, 2, 4, 6, 1, 3, 5},                  /* thirds: 1 3 5 7 */
        {0, 2, 6, 1, 4, 3, 5},                  /* ninths: 1 3 7 9 (the fifth left out) */
        {0, 3, 4, 6, 1, 2, 5},                  /* sus: 1 4 5 b7 */
    };
    static const uint8_t SHAPE[2][4] = {{0, 3, 10, 14}, {0, 5, 10, 15}};   /* dub: m9 (no fifth); fourths */
    int st[12], m = en_scale(pool, st), want, n = 0, i, j;
    const uint8_t *off = OFF[0];
    deg = ((deg % m) + m) % m;
    if (kind == EN_CH_DUB || kind == EN_CH_4THS) {   /* a fixed shape on the degree's note */
        for (i = 0; i < 4; i++)
            pcs[i] = (root + st[deg] + SHAPE[kind == EN_CH_4THS][i]) % 12;
        return 4;
    }
    want = kind == EN_CH_TRIAD ? 3 : kind == EN_CH_AUTO ? (m >= 6 ? 4 : 3) : 4;
    if (kind == EN_CH_9TH)  off = OFF[1];
    else if (kind == EN_CH_SUS) off = OFF[2];
    for (i = 0; i < 7 && n < want; i++) {
        int pc = (root + st[(deg + off[i]) % m]) % 12, dup = 0;
        for (j = 0; j < n; j++)
            dup |= pcs[j] == pc;
        if (!dup)
            pcs[n++] = pc;
    }
    return n;
}
```

Kinds (`amb.h:185`): `AUTO` (7ths on 7-note scales, triads on penta), `TRIAD`, `7TH`, `9TH`
(1 3 7 9, no 5th), `SUS` (1 4 5 b7), `DUB` (fixed minor-9 `{0,3,10,14}` — parallel dub
stabs), `4THS` (stacked fourths `{0,5,10,15}`). The result `chord_pcs[4], chord_nt`
(`amb.c:501-502`) is what the arpeggio draws from.

For a pure mono arp, **the chord-tone count `chord_nt` is the arp's note-set size**; the
pattern's `ac_off` picks among chord tones by *scale-step distance*, so 3rds/5ths/7ths all
come out in-register automatically.

---

## 3. The chord progression (when and where the chord goes)

There is no progression table. The next degree is drawn by weighted random, executed **on
the bar line**:

```c
/* amb.c:1069-1093 */
static void chord_change(void)                  /* to another degree: the near, strong moves first */
{
    static const float W7[7] = {4.0f, 2.0f, 1.0f, 3.0f, 3.0f, 3.0f, 0.4f};
    float w[12], sum = 0.0f, r;
    int i;
    for (i = 0; i < nsteps; i++) {
        w[i] = i == deg ? 0.0f : nsteps == 7 ? W7[i] : i == 0 ? 3.0f : 1.5f;
        sum += w[i];
    }
    r = frand() * sum;
    for (i = 0; i < nsteps; i++) {
        r -= w[i];
        if (r <= 0.0f && w[i] > 0.0f)
            break;
    }
    if (i >= nsteps)
        i = deg == 0 ? (nsteps > 1 ? 1 : 0) : 0;
    deg = i;
    pad_retarget();      /* (polyphonic — skip for mono arp) */
    pad_leave();         /* (polyphonic — skip) */
    bass_to(chord_pcs[0]);
    if (par[P_DE_TRIG] == 0)
        bass.env_up = 1;
    drift_reset();
}
```

**Weights, 7-note scale**: root 4, 2nd 2, 3rd 1, 4th/5th/6th 3, leading tone 0.4.
**Pentatonic**: root 3, others 1.5. Never repeats the current degree. If the draw lands on a
zero weight (float edge), fall back to degree 0 or 1.

**Timing — Drift**: a chord lasts `20 s … 5 min`, re-randomized ±30% every change:

```c
/* amb.c:207 — knob 0..100 to seconds */
static float drift_secs(int v) { return 20.0f * fm_exp2f((float)v * 0.039069f); }

/* amb.c:1062-1067 */
static void drift_reset(void)
{
    drift_total = drift_secs(par[P_DRIFT]) * (0.7f + 0.6f * frand()) * EN_SR;   /* 0.7x .. 1.3x */
    drift_left = drift_total;
    fifth_done = 0;
}
```

When `drift_left` expires, `change_pending = 1` (`control()`, `amb.c:3226-3230`), and the
change **waits for the bar line** (`tick()`, `amb.c:1505-1510`):

```c
if (st == 0 && change_pending && !en_frozen) {   /* chords move on the bar */
    change_pending = 0;
    chord_change();
    ...
}
```

`en_next()` (the EDIT button) sets `change_pending` directly — a forced change next bar.
After a section drop, the chord changes with probability `0.4 + 0.5·Evolve`
(`section_cmd()`, `amb.c:1293-1297`).

For an arp-only port: keep `deg`, `W7`, drift counting, and the on-bar execution. Everything
melodic keys off `deg + ac_off[s]`.

---

## 4. Step generation — `ac_roll()`: how one step is decided

The whole melodic algorithm of the 303 line, per step:

```c
/* amb.c:1096-1114 */
static void ac_roll(int st)                     /* a step made anew */
{
    float dens = (float)par[P_AC_DENS] * 0.01f;
    uint16_t m = (uint16_t)(1u << st);
    if (frand() < (st == 0 ? fm_minf(dens + 0.3f, 1.0f) : dens))   /* the first step: more often a note */
        ac_gate |= m;
    else
        ac_gate &= (uint16_t)~m;
    ac_off[st] = (int8_t)(frand() < 0.8f ? 2 * irand(chord_nt) : irand(nsteps));   /* a chord tone, or now and then not */
    ac_oct[st] = (int8_t)(frand() < 0.55f ? 0 : irand(par[P_AC_RANGE]));
    if (frand() < 0.3f)
        ac_accent |= m;
    else
        ac_accent &= (uint16_t)~m;
    if (frand() < (float)par[P_AC_SLIDE] * 0.007f)
        ac_slide |= m;
    else
        ac_slide &= (uint16_t)~m;
}
```

Decision by decision:

| decision | probability | notes |
|---|---|---|
| **gate** | `Density/100` (step 0: `min(dens+0.3, 1)`) | step 0 is biased to sound — every loop starts on a note |
| **chord tone** | 80%: `off = 2·irand(chord_nt)` | chord tones are *even* scale-step offsets above the degree (0 = root, 2 = 3rd, 4 = 5th, 6 = 7th) |
| **passing tone** | 20%: `off = irand(nsteps)` | any scale step — connects the chord tones |
| **octave** | 55%: 0; else `irand(RANGE)` | RANGE knob 1..3 octaves |
| **accent** | 30% | |
| **slide** | `Slide·0.007` (0..0.7) | slide into the *next* step |

The whole loop is rolled at once by `ac_regrow()` (`amb.c:1123-1129`) — called at init, on
"new piece", and when **Density or Range knobs move** (heard at once: `apply()`
`amb.c:1802-1805` re-rolls the line live).

---

## 5. The note itself — offset → MIDI note

```c
/* amb.c:997-1006 */
static int acid_note(int s)                     /* acid step s's note: from the chord's degree, in its register */
{
    int d = deg + ac_off[s], o = d / nsteps, n;
    n = 36 + par[P_ROOT] + steps[d % nsteps] + 12 * (o + ac_oct[s] + par[P_AC_OCT] + par[P_OCT]);  /* the root: C2 .. B2 */
    while (n > 96)
        n -= 12;
    while (n < 28)
        n += 12;
    return n;
}
```

- Base register: `36 + root` (C2..B2) + octave shifts.
- `d = deg + ac_off[s]` may leave the scale's range; `d % nsteps` / `d / nsteps` wrap it
  through octaves (note: for negative `d`, C division truncates — HYP-1's offsets are never
  negative here because `ac_off ≥ 0` and `deg ≥ 0`; keep it that way in a port).
- Fold into **28..96** by octave shifting.

*(The polyphonic sibling, if you ever want chord tones in a fixed window rather than a mono
register: `en_voice_lead()` `amb.c:382-408` + register math in `pad_retarget()`
`amb.c:1008-1039`. Not needed for mono.)*

---

## 6. Playback — `acid_step()`, slides, accents, gate

Once per sixteenth:

```c
/* amb.c:1141-1175 */
static void acid_step(void)                     /* a sixteenth of the acid line */
{
    int st = ac_pos, len = par[P_AC_LEN], prev = (st + len - 1) % len;
    uint16_t m = (uint16_t)(1u << st);
    en_acid_pos = (uint8_t)st;
    if (acid.play || par[P_AC_MAN]) {           /* the keys have it: the line steps on, unheard */
    } else if (ac_gate & m) {
        int n = acid_note(st), slid = (ac_slide >> prev & 1u) && (ac_gate >> prev & 1u) && acid.amp_t > 0.0f;
        float dec = 0.06f * fm_exp2f((float)par[P_AC_DEC] * 0.05f);   /* 0.06 .. 1.9 s */
        acid.inc_t = mtof((float)n) * (1.0f / EN_SR);
        if (!slid) {                            /* a new note: the envelope again */
            acid.inc = acid.inc_t;
            acid.env = 1.0f;
            acid.acc = (ac_accent & m) ? 1.0f : 0.0f;
            if (lk_hat && links_on())            /* the links: a hat here accents it */
                acid.acc = fm_maxf(acid.acc, (float)par[P_LK_HAT] * 0.012f);
            acid.amp_t = 1.0f;
            acid.edec = fm_expf(-6.9f * 32.0f / (dec * (acid.acc > 0.0f ? 0.6f : 1.0f) * EN_SR));
            if (acid.note)
                midi(0x83, acid.note, 0);
            midi(0x93, (uint32_t)n, acid.acc > 0.0f ? 120u : 90u);
            acid.note = (uint8_t)n;
        }
        acid.sliding = (uint8_t)slid;
        acid.gate_left = (ac_slide & m) ? -1 : (int32_t)(0.55f * step_len);
        en_acid_glow = 1.0f;
    } else if (acid.gate_left < 0) {            /* a slide into a rest: let go */
        acid.gate_left = 0;
        acid.amp_t = 0.0f;
    }
    if (++ac_pos >= len) {
        ac_pos = 0;
        ac_mutate();                            /* the loop's end: Mutate re-rolls a few steps */
    }
}
```

Algorithm, stripped of the synth state:

1. If the step is gated:
   - **Slide check**: the *previous* step had slide set, was gated, and the voice is still
     sounding → this step is a **legato continuation**: no retrigger, pitch glides
     (HYP-1's glide factor is `0.0012` per sample, `amb.c:2734` — ≈50 ms to the new pitch).
     On MIDI out this is simply: no note-off/note-on, new pitch (or pitch-bend to it).
   - Otherwise a **new note**: velocity **120 if accented else 90** (`midi(0x93, n, vel)`).
2. **Gate length**: if this step has slide set, the gate is held (`gate_left = -1`) until the
   next gated step or a rest; otherwise **0.55 × step_len** (a staccato-ish 303 gate).
3. A slide that lands on a **rest** releases the note.
4. `ac_pos` wraps at `len` (1..16 — can turn against the bar); **each wrap runs
   `ac_mutate()`** (§7.1).

---

## 7. Pattern switching — the three mechanisms

HYP-1 switches patterns in three distinct ways. All of them are bar-quantized or
loop-quantized — nothing ever switches mid-step.

### 7.1 In-place mutation on loop wrap — `ac_mutate()`

The acid line's pattern is never swapped; it **decays toward new material** every time the
loop goes round:

```c
/* amb.c:1131-1139 */
static void ac_mutate(void)                     /* the loop's end: Mutate re-rolls a few steps */
{
    float m = (float)par[P_AC_MUT] * 0.0025f + 0.05f * evolve;
    int st;
    for (st = 0; st < par[P_AC_LEN]; st++)
        if (frand() < m)
            ac_roll(st);
    ac_publish();
}
```

Per loop pass, each step re-rolls with probability `Mutate·0.0025 + 0.05·Evolve`
(Mutate knob 0..100 → 0..0.25; Evolve adds up to 0.05). `ac_roll` re-decides gate, tone,
octave, accent, slide independently — so material appears and dissolves in place. With
Mutate at default 25: p ≈ 0.0625 per step per pass.

Also note the **loop length** is itself a live parameter (`P_AC_LEN` 1..16): `apply()`
(`amb.c:1797-1800`) resets `ac_pos` if it now exceeds the new length. Length changes are
heard at the next wrap — the loop can turn against the bar.

### 7.2 Bar-quantized bank switching — the drum patterns' rule

For pattern *banks* (HYP-1 has 6 drum patterns, `EN_NPAT`), the rule is: **the selection is
staged, the switch lands on the bar line**:

```c
/* state: amb.c:1188 */
static int sec, sec_bars, sec_req, pat_now, pat_next;   /* pat_now: playing; pat_next: waiting for the bar */

/* apply() — knob turns only stage it: amb.c:1808-1813 */
case P_PATTERN:                              /* waits for the bar (now, while stopped) */
    pat_next = v;
    if (!en_playing)
        pat_now = v;
    drum_publish();
    break;

/* tick() — the switch itself: amb.c:1428 */
if (st == 0) {                              /* a bar */
    ...
    pat_now = pat_next;                     /* a new pattern: on the bar */
```

Two consequences worth copying:
- Turning through several patterns within one bar plays **none** of them — only the last
  one staged before the bar line sounds.
- A change always lands with the step counter reset at the top of the bar.

### 7.3 Automatic switching with a fill bar — DRUM DRIFT

This is HYP-1's generative pattern switcher: a counter, a **fill bar immediately before the
switch**, then the next pattern (sequential or random within a pool), then optional mutation
of what was switched to.

```c
/* amb.c:1193-1273 */
static int8_t dd_in = -1;                        /* bar lines to the change, -1 none */
static uint8_t dd_fill;                          /* this bar ends in a fill */
static uint32_t dd_bars;
static int dd_len(void) { static const uint8_t L[6] = {0, 4, 8, 16, 32, 64};
                          return L[(unsigned)par[P_DD_EVERY] < 6u ? par[P_DD_EVERY] : 0]; }

static void dd_mutate(int p)                     /* Shift / Morph, on the incoming pattern */
{
    static const uint8_t LN[4] = {L_SNARE, L_HAT, L_OHAT, L_PERC};   /* the kick and the clap stay as written */
    int c;
    for (c = 0; c < 2; c++) {                   /* a hit or two, a step either way */
        int l = LN[irand(4)], on[16], m = 0, st;
        uint32_t b = en_pats[p][l];
        for (st = 0; st < 16; st++)
            if (b >> st & 1u)
                on[m++] = st;
        if (m) {
            int k = on[irand(m)], to = (k + (frand() < 0.5f ? 15 : 1)) & 15;
            if (!(b >> to & 1u))
                en_pats[p][l] = (uint16_t)((b & ~(1u << k)) | 1u << to);
        }
    }
    if (par[P_DD_MUTATE] == 2) {                /* Morph: one more, or one fewer */
        int l = LN[irand(4)], st;
        uint32_t b = en_pats[p][l];
        int h = bits16(b);
        if ((frand() < 0.5f && h > 1) || h >= 10) {
            for (st = irand(16); !(b >> st & 1u); st = (st + 1) & 15)
                ;
            b &= ~(1u << st);
        } else if (h < 10) {
            for (st = irand(16); b >> st & 1u; st = (st + 1) & 15)
                ;
            b |= 1u << st;
        }
        en_pats[p][l] = (uint16_t)b;
    }
    en_dd_mutations++;
}

static void dd_bar(void)                        /* at each bar line, before the new pattern is taken */
{
    int len = dd_len();
    if (!par[P_DD_CHANGE] && !par[P_DD_MUTATE]) {
        dd_in = en_dd_left = -1;
        dd_fill = 0;
        dd_bars = 0;
        return;
    }
    if (dd_in > 0)
        dd_in--;
    if (dd_in == 0) {
        int pool = par[P_DD_POOL] < 2 ? 2 : par[P_DD_POOL] > EN_NPAT ? EN_NPAT : par[P_DD_POOL], nb = pat_now;
        if (par[P_DD_CHANGE] == 1)
            nb = pat_now + 1 >= pool ? 0 : pat_now + 1;             /* Next: wrap within the pool */
        else if (par[P_DD_CHANGE] == 2)
            do
                nb = irand(pool);
            while (nb == pat_now);                                  /* Random: never the same */
        if (par[P_DD_CHANGE]) {
            pat_next = nb;                                          /* staged: lands at this bar line */
            par[P_PATTERN] = (int16_t)nb;
            en_dd_changes++;
        }
        if (par[P_DD_MUTATE])
            dd_mutate(pat_next);                                    /* the incoming pattern is mutated */
        dd_in = -1;
        dd_bars = 0;
    } else if (len && dd_in < 0 && ++dd_bars >= (uint32_t)len - 1u) {
        dd_in = 1;                                                  /* one bar to go */
    }
    dd_fill = dd_in == 1 && par[P_DD_CHANGE];   /* the bar before the switch is the fill bar */
    en_dd_left = (int8_t)(dd_in >= 0 ? dd_in : len ? len - (int)dd_bars : -1);
}
```

And the **fill itself** (`tick()`, `amb.c:1448-1450`) — during the fill bar's second half,
extra hits pile onto the snare and hats, rising:

```c
if ((fill_hold || (dd_fill && st >= 8 && sec == SEC_GROOVE)) && (l == L_SNARE || l == L_HAT)) {
    hit = 1;
    vel = l == L_SNARE ? 0.35f + 0.04f * (float)st : 0.5f;
}
```

**The state machine, in words:**

1. Idle (`dd_in = -1`): count bars (`dd_bars++`). When `Every` − 1 bars have passed, enter
   countdown `dd_in = 1` — *one bar to go*.
2. **Fill bar** (`dd_in == 1`): `dd_fill` is on — the second half of this bar (steps 8–15)
   gets the fill (in the drums: snare+hat roll with rising velocity).
3. At that bar line: pick the next pattern — `Next` (wraps within the first `Pool` patterns)
   or `Random` (never the same twice) — **stage it in `pat_next`**, optionally `dd_mutate()`
   the *incoming* pattern (Shift: move 1–2 hits ±1 step, never the anchor lane; Morph: also
   add/remove one, capped at 10 hits), then `pat_now = pat_next` at the same bar line.
4. Counter restarts. With `Every = Chord`, the chord change starts the count instead
   (`tick()`, `amb.c:1508-1509`: `dd_in = 2` on a chord change — fill next bar, switch the
   bar after).

**For a mono arp**, the direct translation: lanes → your one voice; the fill bar's back
half → a rising run (force gate on steps 8–15 with ascending chord-tone offsets and rising
accents); Shift/Morph operate on the pattern's gated steps. §8 implements exactly this.

### 7.4 What starts a count besides the clock

- A **chord change** (Every = Chord, above).
- **Evolve** (0..100 → 0..1) feeds every mutator: acid mutation `+0.05·evolve`, the drop's
  chord-change probability `0.4 + 0.5·evolve`.

---

## 8. `hyp_arp.c` — the algorithms assembled, one file

A complete monophonic engine for the ESP32-C3 (or anything): scale + chord kinds + W7
progression + drift + the acid step-roll + mutate-on-wrap + a pattern bank with the full
DRUM DRIFT switcher (fill bar, Next/Random, Shift/Morph). **No floats, no libc** — pure
integer C99; output is a per-sixteenth event struct you map to your synth or MIDI.

Derived from the GPL code above → **GPL-3.0-only** if you distribute it (header states this;
re-implement from §§1–7 for other licences).

```c
/* hyp_arp.c — HYP-1's monophonic algorithms in one file: a generative chord-arpeggio with
 * bar-quantized pattern switching. Derived from M-VAVE FM-1 HYP-1's firmware/src/dsp/amb.c
 * (AMB-1 (C) Charles Vestal, HYP-1 changes RedtailWorks, GPL-3.0-only). This file is a
 * derivative work: distribute it only under GPL-3.0 with its source, or re-implement from
 * the algorithm descriptions in docs/hyp1-mono-arp-algorithms.md. Integer-only C99. */
#include <stdint.h>
#include <string.h>

#define HA_NSTEP 16
#define HA_NPAT  6                            /* the pattern bank (HYP-1: EN_NPAT) */

/* modes: pools relative to the root (bit i: root + i semitones) — amb.c:17-19 */
enum { HA_IONIAN, HA_DORIAN, HA_PHRYGIAN, HA_LYDIAN, HA_MIXO, HA_AEOLIAN,
       HA_PENT_MAJ, HA_PENT_MIN, HA_NMODES };
static const uint16_t HA_MODE_POOL[HA_NMODES] = {
    0x0AB5u, /* Ionian     0 2 4 5 7 9 11  */
    0x06ADu, /* Dorian     0 2 3 5 7 9 10  */
    0x06B5u, /* Phrygian   0 1 3 5 7 8 10  */
    0x0ABDu, /* Lydian     0 2 4 6 7 9 11  */
    0x0AADu, /* Mixolydian 0 2 4 5 7 9 10  */
    0x06ADu, /* Aeolian    0 2 3 5 7 8 10  */
    0x0125u, /* Pent maj   0 2 4 7 9       */
    0x04A9u, /* Pent min   0 3 5 7 10      */
};
enum { HA_CH_AUTO, HA_CH_TRIAD, HA_CH_7TH, HA_CH_9TH, HA_CH_SUS, HA_CH_DUB, HA_CH_4THS };

typedef struct {
    /* harmony */
    uint8_t  root;        /* pitch class 0..11 */
    uint8_t  mode;        /* HA_* */
    uint8_t  chord;       /* HA_CH_* */
    uint8_t  bpm;         /* for the drift timer */
    uint16_t drift_secs;  /* a chord lasts about this long: 20..300 */
    /* the line */
    uint8_t  density;     /* 0..100: chance a step sounds */
    uint8_t  len;         /* loop length 1..16 */
    uint8_t  range;       /* octaves 1..3 */
    uint8_t  mutate;      /* 0..100: per-step re-roll chance per pass = v*0.0025 */
    uint8_t  slide;       /* 0..100: chance a step slides = v*0.007 */
    int8_t   oct;         /* the line's octave shift, -3..3 */
    /* pattern switching (DRUM DRIFT) */
    uint8_t  dd_change;   /* 0 off, 1 next, 2 random */
    uint8_t  dd_every;    /* 0..5 -> 0/4/8/16/32/64 bars (0: on chord change) */
    uint8_t  dd_pool;     /* switch within patterns 1..dd_pool (2..HA_NPAT) */
    uint8_t  dd_mutate;   /* 0 off, 1 shift, 2 morph */
    uint8_t  evolve;      /* 0..100: restlessness */
} ha_cfg_t;

/* one step's decision — the acid line's data model (amb.c:948-950) */
typedef struct {
    uint8_t gate, acc, slide;
    int8_t  off;          /* scale steps above the chord's degree (even = chord tone) */
    int8_t  oct;          /* 0..range-1 */
} ha_step_t;

typedef struct { ha_step_t s[HA_NSTEP]; } ha_pat_t;

/* one event per sixteenth, out */
typedef struct {
    int16_t note;         /* MIDI note, -1 = rest */
    uint8_t vel;          /* 120 accented, 90 plain (HYP-1's acid velocities) */
    uint8_t slide;        /* 1: legato from the previous note (no retrigger) */
    uint8_t hold;         /* 1: gate held into the next step (the step had slide set) */
    uint8_t off16;        /* 1: an off-16th (apply swing delay) */
    uint8_t fill;         /* 1: part of the fill run */
} ha_event_t;

typedef struct {
    ha_cfg_t cfg;
    uint32_t rng;
    uint8_t  steps[12], nsteps;
    int8_t   deg;
    uint8_t  chord_pcs[4], chord_nt;
    uint32_t drift_left;  /* in sixteenths */
    uint8_t  change_pending;
    ha_pat_t pat[HA_NPAT];
    uint8_t  pat_now, pat_next;
    int8_t   dd_in;
    uint8_t  dd_fill;
    uint32_t dd_bars;
    uint32_t sixteenth;
    uint8_t  pos;
    int16_t  last_note;
    uint8_t  held;        /* the voice is sounding (a slide may continue it) */
} ha_t;

static ha_t ha;

/* --------------------------------------------- rng (amb.c:505-513) ------- */
static uint32_t ha_rnd(void)
{
    ha.rng ^= ha.rng << 13;
    ha.rng ^= ha.rng >> 17;
    ha.rng ^= ha.rng << 5;
    return ha.rng;
}
static float ha_frand(void) { return (float)(ha_rnd() >> 8) * (1.0f / 16777216.0f); }
static int   ha_irand(int n) { return n > 0 ? (int)((ha_rnd() >> 8) % (uint32_t)n) : 0; }

/* --------------------------------------------- harmony ------------------- */
static void ha_scale_set(void)
{
    int i;
    uint16_t pool = HA_MODE_POOL[ha.cfg.mode % HA_NMODES] | 1u;
    ha.nsteps = 0;
    for (i = 0; i < 12; i++)
        if (pool >> i & 1u)
            ha.steps[ha.nsteps++] = (uint8_t)i;
}

/* en_chord_kind (amb.c:337-367) */
static void ha_chord_build(void)
{
    static const uint8_t OFF3[7] = {0, 2, 4, 6, 1, 3, 5};
    static const uint8_t OFF9[7] = {0, 2, 6, 1, 4, 3, 5};
    static const uint8_t OFFS[7] = {0, 3, 4, 6, 1, 2, 5};
    static const uint8_t SHAPE[2][4] = {{0, 3, 10, 14}, {0, 5, 10, 15}};
    const uint8_t *off = OFF3;
    int want = 4, n = 0, i, j;
    int d = ((ha.deg % (int)ha.nsteps) + ha.nsteps) % (int)ha.nsteps;
    if (ha.cfg.chord == HA_CH_DUB || ha.cfg.chord == HA_CH_4THS) {
        int k = ha.cfg.chord == HA_CH_4THS;
        for (i = 0; i < 4; i++)
            ha.chord_pcs[i] = (uint8_t)((ha.cfg.root + ha.steps[d] + SHAPE[k][i]) % 12);
        ha.chord_nt = 4;
        return;
    }
    switch (ha.cfg.chord) {
    case HA_CH_TRIAD: want = 3; break;
    case HA_CH_9TH:   off = OFF9; break;
    case HA_CH_SUS:   off = OFFS; break;
    default:          want = ha.nsteps >= 6 ? 4 : 3; break;
    }
    for (i = 0; i < 7 && n < want; i++) {
        uint8_t pc = (uint8_t)((ha.cfg.root + ha.steps[(d + off[i]) % ha.nsteps]) % 12);
        int dup = 0;
        for (j = 0; j < n; j++)
            dup |= ha.chord_pcs[j] == pc;
        if (!dup)
            ha.chord_pcs[n++] = pc;
    }
    ha.chord_nt = (uint8_t)n;
}

/* chord_change (amb.c:1069-1093) + drift_reset (amb.c:1062-1067), sixteenths */
static void ha_chord_change(void)
{
    static const uint8_t W7[7] = {40, 20, 10, 30, 30, 30, 4};   /* x10 */
    uint32_t w[12], sum = 0, r;
    int i;
    for (i = 0; i < (int)ha.nsteps; i++) {
        w[i] = (i == ha.deg) ? 0u
             : (ha.nsteps == 7) ? (uint32_t)W7[i]
             : (i == 0) ? 30u : 15u;
        sum += w[i];
    }
    r = (uint32_t)(ha_frand() * (float)sum);
    for (i = 0; i < (int)ha.nsteps; i++) {
        if (r < w[i] && w[i])
            break;
        r -= w[i];
    }
    if (i >= (int)ha.nsteps)
        i = (ha.deg == 0 && ha.nsteps > 1) ? 1 : 0;
    ha.deg = (int8_t)i;
    ha_chord_build();
    {   /* secs x (0.7 .. 1.3), to sixteenths: secs * bpm * 4 / 60 */
        uint32_t secs = ha.cfg.drift_secs ? ha.cfg.drift_secs : 60;
        secs = secs * 7u / 10u + (uint32_t)((uint64_t)secs * 6u * ha_irand(1000) / 10000u);
        ha.drift_left = secs * (ha.cfg.bpm ? ha.cfg.bpm : 120) / 15u;
    }
}

/* --------------------------------------------- pattern generation -------- */
static int8_t ha_rand_offset(void)   /* a chord tone, or now and then a passing tone */
{
    if (ha_frand() < 0.8f) {
        int o = 2 * ha_irand(ha.chord_nt);
        return (int8_t)(o + (ha_frand() < 0.15f ? ha.nsteps : 0));
    }
    return (int8_t)ha_irand(ha.nsteps);
}

static void ha_roll(uint8_t pat, uint8_t st)   /* ac_roll (amb.c:1096-1114) */
{
    ha_step_t *s = &ha.pat[pat].s[st];
    float dens = (float)ha.cfg.density * 0.01f;
    s->gate  = ha_frand() < (st == 0 ? (dens + 0.3f > 1.0f ? 1.0f : dens + 0.3f) : dens);
    s->off   = ha_rand_offset();
    s->oct   = (int8_t)(ha_frand() < 0.55f ? 0 : ha_irand(ha.cfg.range ? ha.cfg.range : 1));
    s->acc   = ha_frand() < 0.3f;
    s->slide = ha_frand() < (float)ha.cfg.slide * 0.007f;
}

static void ha_regrow(uint8_t pat)
{
    int st;
    for (st = 0; st < HA_NSTEP; st++)
        ha_roll(pat, (uint8_t)st);
}

static void ha_mutate(void)          /* ac_mutate (amb.c:1131-1139), on the loop wrap */
{
    float m = (float)ha.cfg.mutate * 0.0025f + 0.05f * ((float)ha.cfg.evolve * 0.01f);
    int st;
    for (st = 0; st < ha.cfg.len; st++)
        if (ha_frand() < m)
            ha_roll(ha.pat_now, (uint8_t)st);
}

static void ha_dd_mutate(void)       /* dd_mutate (amb.c:1206-1238) on the incoming pattern */
{
    ha_pat_t *p = &ha.pat[ha.pat_next];
    int c, st, h = 0;
    for (c = 0; c < 2; c++) {        /* Shift: a hit or two, a step either way */
        st = ha_irand(HA_NSTEP);
        if (p->s[st].gate) {
            int to = (st + (ha_frand() < 0.5f ? 15 : 1)) & 15;
            if (!p->s[to].gate) {
                p->s[to] = p->s[st];
                p->s[st].gate = 0;
            }
        }
    }
    for (st = 0; st < HA_NSTEP; st++)
        h += p->s[st].gate;
    if (ha.cfg.dd_mutate == 2) {     /* Morph: one more, or one fewer */
        if ((ha_frand() < 0.5f && h > 1) || h >= 10) {
            for (st = ha_irand(HA_NSTEP); !p->s[st].gate; st = (st + 1) & 15)
                ;
            p->s[st].gate = 0;
        } else if (h < 10) {
            for (st = ha_irand(HA_NSTEP); p->s[st].gate; st = (st + 1) & 15)
                ;
            ha_roll(ha.pat_next, (uint8_t)st);
        }
    }
}

static void ha_dd_bar(void)          /* dd_bar (amb.c:1240-1273), at each bar line */
{
    static const uint8_t LEN[6] = {0, 4, 8, 16, 32, 64};
    int len = LEN[ha.cfg.dd_every < 6 ? ha.cfg.dd_every : 0];
    if (!ha.cfg.dd_change && !ha.cfg.dd_mutate) {
        ha.dd_in = -1; ha.dd_fill = 0; ha.dd_bars = 0;
        return;
    }
    if (ha.dd_in > 0)
        ha.dd_in--;
    if (ha.dd_in == 0) {
        uint8_t pool = ha.cfg.dd_pool < 2 ? 2 : ha.cfg.dd_pool > HA_NPAT ? HA_NPAT : ha.cfg.dd_pool;
        uint8_t nb = ha.pat_now;
        if (ha.cfg.dd_change == 1)
            nb = (uint8_t)(ha.pat_now + 1 >= pool ? 0 : ha.pat_now + 1);
        else if (ha.cfg.dd_change == 2)
            do { nb = (uint8_t)ha_irand(pool); } while (nb == ha.pat_now);
        ha.pat_next = nb;
        if (ha.cfg.dd_mutate)
            ha_dd_mutate();
        ha.pat_now = ha.pat_next;    /* lands at this bar line */
        ha.dd_in = -1;
        ha.dd_bars = 0;
    } else if (len && ha.dd_in < 0 && ++ha.dd_bars >= (uint32_t)len - 1u) {
        ha.dd_in = 1;                /* one bar to go: the next bar is the fill bar */
    }
    ha.dd_fill = (uint8_t)(ha.dd_in == 1 && ha.cfg.dd_change);
}

/* --------------------------------------------- the note ------------------ */
static int ha_note(const ha_step_t *s)   /* acid_note (amb.c:997-1006) */
{
    int d = ha.deg + s->off;
    int n = 36 + ha.cfg.root + ha.steps[d % ha.nsteps]
          + 12 * (d / (int)ha.nsteps + s->oct + ha.cfg.oct);
    while (n > 96) n -= 12;
    while (n < 28) n += 12;
    return n;
}

/* --------------------------------------------- the clock ----------------- */
void ha_tick(ha_event_t *ev)         /* one call per sixteenth */
{
    int st = ha.pos, len = ha.cfg.len ? ha.cfg.len : HA_NSTEP;
    ha_step_t *s;
    uint8_t fill = 0;

    if ((ha.sixteenth & 15u) == 0u) {            /* a bar line */
        if (ha.change_pending) {
            ha.change_pending = 0;
            ha_chord_change();                   /* the chord moves on the bar (amb.c:1505) */
        }
        ha_dd_bar();
    }
    if (ha.drift_left && --ha.drift_left == 0)
        ha.change_pending = 1;
    if (ha.dd_fill && st >= 8)
        fill = 1;                                /* the fill bar's back half */

    memset(ev, 0, sizeof *ev);
    ev->off16 = (uint8_t)(st & 1u);
    ev->fill = fill;

    if (fill) {   /* the run: up the chord tones, rising accents (the mono fill) */
        static const int8_t RUN[8] = {0, 2, 4, 6, 8, 10, 12, 14};
        ha_step_t run = {1, (uint8_t)(st >= 12), 0, 0, 0};
        run.off = RUN[st - 8];
        s = &run;
    } else {
        s = &ha.pat[ha.pat_now].s[st];
    }

    if (s->gate) {
        int prev = (st + len - 1) % len;
        ha_step_t *ps = &ha.pat[ha.pat_now].s[prev];
        int slid = !fill && ps->slide && ps->gate && ha.held;
        ev->note = (int16_t)ha_note(s);
        ev->vel = (uint8_t)(s->acc ? 120 : 90);
        ev->slide = (uint8_t)(slid && ev->note != ha.last_note);
        ev->hold = s->slide;                     /* gate held into the next step */
        ha.last_note = ev->note;
        ha.held = 1;
    } else {
        ev->note = -1;
        ha.held = 0;                             /* a slide into a rest lets go */
    }

    if (++st >= len) {
        st = 0;
        ha_mutate();                             /* the loop's end (amb.c:1171-1174) */
    }
    ha.pos = (uint8_t)st;
    ha.sixteenth++;
}

/* --------------------------------------------- api ----------------------- */
void ha_init(uint32_t seed, const ha_cfg_t *cfg)
{
    int i;
    memset(&ha, 0, sizeof ha);
    ha.rng = seed ? seed : 0x9E3779B9u;          /* HYP-1's default seed (amb.c:503) */
    ha.cfg = *cfg;
    if (ha.cfg.len < 1 || ha.cfg.len > HA_NSTEP) ha.cfg.len = HA_NSTEP;
    if (ha.cfg.range < 1) ha.cfg.range = 1;
    ha_scale_set();
    ha.deg = 0;
    ha_chord_build();
    for (i = 0; i < HA_NPAT; i++) {              /* a bank of distinct patterns */
        ha_regrow((uint8_t)i);
    }
    ha.pat_now = ha.pat_next = 0;
    ha_chord_change();
}

void ha_set_density(uint8_t v) { ha.cfg.density = v; ha_regrow(ha.pat_now); }  /* heard at once */
void ha_set_range(uint8_t v)   { if (v) { ha.cfg.range = v; ha_regrow(ha.pat_now); } }
void ha_set_root(uint8_t v)    { ha.cfg.root = (uint8_t)(v % 12); ha_chord_build(); }
void ha_set_mode(uint8_t v)
{
    ha.cfg.mode = (uint8_t)(v % HA_NMODES);
    ha_scale_set();
    if (ha.deg >= (int)ha.nsteps)
        ha.deg = 0;
    ha_chord_build();
}
void ha_next_chord(void)       { ha.change_pending = 1; }   /* en_next: on the next bar */
void ha_set_len(uint8_t v)
{
    if (v >= 1 && v <= HA_NSTEP) {
        ha.cfg.len = v;
        if (ha.pos >= v)
            ha.pos = 0;                          /* apply(), amb.c:1797-1800 */
    }
}

/* feed your own chord (e.g. detected from incoming MIDI): pitch classes, 1..4 of them.
 * The pattern's offsets stay relative to the degree the pattern was rolled over, so set
 * deg alongside: the arp re-tiles onto your chord at once (see §1). */
void ha_set_chord(const uint8_t *pcs, uint8_t n, int8_t deg)
{
    int i;
    if (n < 1 || n > 4)
        return;
    for (i = 0; i < n; i++)
        ha.chord_pcs[i] = (uint8_t)(pcs[i] % 12);
    ha.chord_nt = n;
    ha.deg = deg;                                /* the root's degree in the current scale */
}
```

If the chord comes from *outside* and only the arpeggio is HYP-1's, skip §3 entirely
(no `deg`/drift/`W7`) and call `ha_set_chord()` when your chord changes; everything in
§§4–6 works unchanged because the pattern stores *offsets*, not notes. For that case make
`ha_rand_offset()` pick from `chord_nt` only (drop the passing-tone branch, or map it to
your own scale).

### Usage on the ESP32-C3

```c
static ha_cfg_t cfg = {
    .root = 9, .mode = HA_AEOLIAN, .chord = HA_CH_AUTO, .bpm = 128,
    .drift_secs = 60,
    .density = 60, .len = 16, .range = 1, .mutate = 25, .slide = 30,
    .dd_change = 1, .dd_every = 3 /* 16 bars */, .dd_pool = 4, .dd_mutate = 1,
    .evolve = 40,
};

/* sixteenth clock: bpm*4/60 Hz — from esp_timer, or sample-accurate from your I2S block
 * counter (keep step_len in samples = 44100*60/bpm/4 and accumulate the fraction, exactly
 * as amb.c's en_render does at amb.c:3279-3298). */

void on_sixteenth(void)
{
    ha_event_t ev;
    ha_tick(&ev);
    if (ev.note >= 0) {
        if (!ev.slide)
            midi_note_on(0, ev.note, ev.vel);        /* HYP-1: ch 4, vel 120/90 */
        else
            midi_send_pitch(0, ev.note);             /* legato: new pitch, no retrigger */
        if (!ev.hold)
            schedule_note_off(ev.note, step_ms * 55 / 100);   /* gate = 0.55 x step */
        /* ev.hold: no note-off — the next event (or a rest) releases it */
    } else if (voice_sounding()) {
        midi_note_off(0, last_note);                 /* a slide into a rest lets go */
    }
}
```

---

## 9. Mapping notes (HYP-1's own MIDI out, for reference)

The engine's MIDI (`midi()` calls, e.g. `amb.c:1160-1162`): acid on channel 4
(`0x93`/`0x83`), velocity **120 accented / 90 plain**; a slide = **no** note-off/note-on,
just the new pitch; gate = 0.55 × step; the chord's tones also go out on channel 1 (pad
voices) and the root on channel 3 (`0x92`/`0x82`, vel 80) — useful if your mono arp should
double the drone's root.

## 10. What was deliberately left out (polyphonic / not mono)

`pad_retarget`/`en_voice_lead`/`pad_leave` (4-voice handoff), the stabs, drone glide,
sections (breakdown/build/drop), and the links. One helper worth keeping: `en_euclid()`
(`amb.c:281-323`, full Bjorklund) — `en_euclid(k, 16, rot)` gives a fixed gate bitmask you
can AND into the pattern's gates when you want *structured* rhythms instead of rolled
density.
