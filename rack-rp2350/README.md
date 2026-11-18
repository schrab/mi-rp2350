# Rack RP2350 — five modules in one firmware, live-switchable

ONE firmware running the four accepted MI ports (Elements, Clouds, Plaits
98-engine, Rings MiniElements) plus the hand-ported **LIRA-8** (module id 4,
a Pure Data patch port — not an MI library) on the RP2350 nano + PCM5102A,
with LIVE MODULE SWITCHING (T2 of the combined plan,
`docs/superpowers/plans/2026-11-04-combined-rp2350.md`) and a DIN MIDI input
+ serial bench console (T4).

## Module selection (T2: boot-select + live swap)

ONE PlatformIO env — all five module adapters compile into the same unified
binary:

```sh
pio run -d rack-rp2350
# → .pio/build/rack/firmware.uf2
```

`RACK_BOOT_MODULE` (0 elements / 1 clouds / 2 plaits / 3 rings / 4 lira8 —
new modules continue the ids) selects the boot module: `setup()` copies that
module's flash image into the SRAM window (first action, crt0-like) and
registry-Inits it; the build boots LIRA-8. The module registry
(`src/module_registry.h`, table in `src/main.cpp`) dispatches init / render /
note / CC hooks, and any module can be swapped in at runtime (MIDI CC 32 /
serial `switch <id|name>`).

### Live swap — MIDI CC 32 (Bank Select)

**CC 32, value 0-4 = module id** (0 elements, 1 clouds, 2 plaits, 3 rings,
4 lira8; other values ignored, same id ignored). CC 32 is verified unused by
all five modules' CC maps. Core 0 queues the request; core 1 executes the swap
at the next block boundary (max one per call):

1. **MUTE** — linear output-gain ramp-down over 4 render blocks, then the
   whole I2S DMA bank is flushed with silence (the AudioBufferManager also
   loops a dedicated silence buffer on underflow).
2. **COPY** — the new module's hot-section flash image is memcpy'd into the
   SRAM window (~109 KB worst case → tens of ms; plain memcpy is correct, the
   RP2350 M33 I/D-caches are disabled, T1 finding) and FNV-checksummed. On a
   checksum mismatch the OLD module's image is restored from its own flash
   LMA and the swap aborts with the old module still live.
3. **INIT** — the new module placement-news its state into the object arena
   and carves its buffers from the buffer arena; factory defaults via its
   `Init()`.
4. **I2S** — `end()` → bits/buffers/frequency set to the new module's native
   rate + bank → `begin()` (48 kHz for plaits, 44.1 kHz for lira8, 32 kHz for
the others).
5. **UNMUTE** — 4-block gain ramp-up on the new module.

One line per swap: `rack: switched to <module> in <ms> ms (window <n> bytes)`
(stats collected on core 1, printed by core 0 — core 1 never writes CDC).
MIDI adapter hooks are dropped during the swap window (the note stack still
updates, so pitch/gate survive); **CC state of the new module starts at
factory defaults** — web-panel knob positions desync on swap and re-sync on
panel reconnect (T4a presets will restore parameter sets after a swap).

### Serial console (T2/T3/T4 scope)

115200 baud, non-blocking, `\n`-terminated lines (`src/main.cpp`
`kConsoleCommands` — one data-driven table; the T5 C3 console reuses it):

| Command | Effect |
|---|---|
| `switch <0-3|elements|clouds|plaits|rings>` | queue a live swap (same path as CC 32) |
| `module` | print active module, rate/block/budget, CC map |
| `mod <src> <param> <value>` | set a mod-engine source param (T3; see `mod?`) |
| `mod?` | dump the whole engine (params + routing + telemetry) |
| `route <source> <dest>` | alias of `mod <src> dest <dest>` (dest = CC 0-127 \| off) |
| `bpm <20-300>` | internal clock BPM (0.1 steps) |
| `clock <internal\|midi> [src]` | global clock source / per-source override |
| `preset` | T4a preset status (version, size, module/chain/bpm, dirty, seen-CC count) |
| `preset dump` | print the serialized state blob as `PRxx:<hex>` rows (37 rows x 24 B) |
| `preset collect` + rows | reassemble pasted rows into the collect buffer |
| `preset load` | validate + apply the collected blob (a full buffer applies directly) |
| `preset apply\|reload` | re-apply the collected blob without re-collecting |
| `status` | one-line summary: active module, clock source/BPM, swaps, DIN counters, uptime |
| `din [clear]` | DIN MIDI counters + last message (+ `clear` resets them) |
| `dintest` | inject a canned MIDI stream into the DIN parser → funnel (see below) |
| `telemetry on\|off` | mute/unmute the 2 s report for quiet benching (default on) |
| `led on\|off` | enable/disable the WS2812 status LED (default on; off removes all core-0 LED work) |
| `panic` | all notes off: releases held notes on the active module, clears the note stack, forces the gate off |
| `help` | list commands |

Web panels drive everything else (per-module CC maps, unchanged from the
standalone ports; see the four port READMEs).

## DIN MIDI in (T4)

DIN notes/CC/clock land in the SAME funnel as USB (`midiHandleEventPacket`,
`src/midi.h`) — downstream they are indistinguishable from device/host MIDI,
and DIN F8/FA/FB/FC reach the mod engine's sample-stamped clock bus with no
extra code. Parser: `src/din_midi.h` (MIDI 1.0 byte stream → 4-byte USB-MIDI
event packets; running status; realtime F8-FF passed through inline anywhere;
sysex DISCARDED and counted; host-unit-tested, see the ledger T4 findings).

### Wiring — MIDI DIN-IN jack (standard 6N138 opto circuit)

RX pin: **GP5** (UART1 RX, 31250 8N1). GP4 is muxed as UART1 TX but RESERVED
— DIN OUT is not in T4. The init sits in `setup()` after the module/engine
init, before the PIO-USB host init (hardware UART claims no PIO/DMA/alarm
resources, so the PIO-USB/I2S/LED order law does not apply to it).

| MIDI DIN-5 jack (IN) | → | RP2350 side | Note |
|---|---|---|---|
| pin 4 | 220 Ω | 6N138 pin 2 (anode) | sender's current loop + |
| pin 5 | 220 Ω | 6N138 pin 3 (cathode) | current return |
| across pins 2-3 | 1N4148, reverse | — | recommended LED reverse-voltage protection |
| — | 6N138 pin 8 (Vcc) | 3V3 | |
| — | 6N138 pin 6 (output, open collector) | **GP5**, 10 kΩ pull-up to 3V3 | idle = mark (high); the firmware also enables the GP5 internal pull-up |
| — | 6N138 pin 5 (emitter) | GND | |
| — | 6N138 pin 7 (base) | n.c. | |
| pin 2 (shield) | — | GND (optional) | |

(6N138 DIP-8: 1 NC, 2 anode, 3 cathode, 4 NC, 5 emitter/GND, 6 collector out,
7 base, 8 Vcc. 5 V for the current loop comes from the SENDER — the opto
fully isolates the rack.)

RX path: the core's `Serial2` (HW UART1) in IRQ mode — its IRQ drains the
32-byte HW FIFO into a 256-entry SW ring (`setFIFOSize(256)`, ~82 ms of
max-rate traffic), `loop()` parses into `midiHandleEventPacket()`. Ring
overflow folds into the DIN `dropped` counter.

### `dintest` — without-jack bench verification

Injects the canned stream `kDinTestStream` (`src/din_midi.h`, shared
byte-for-byte with the host unit test) directly INTO the parser — the UART1
link is bypassed (honest scope: parser + funnel only). Expected output: 36
per-message lines (34 packets + 2 discards), then `52 bytes -> 34 packets
(27 realtime), +2 dropped | note_on_count +3`, plus the clock hint
`FA + 25 F8 + FC`. Side effects are real: three short notes sound on the
active module (60/64/67), CC 7 applies, and `mod?` shows midi `ticks +25`
once core 1 drains the stamp ring (within one audio block).

## rack-lira8 — the fifth module (LIRA-8 hand port)

`rack-lira8` is not a vendored MI library: it is a hand port of Mike Moreno's
LIRA-8 Pure Data patch (BSD) into `src/lira8/` (pure C++, no MCU headers; the
host golden harness compiles the same headers — that is also the
ctag-tbd portability proof). Fidelity is gated against headless-Pd renders
(`test/lira8_ref/`): the onevoice harmonic table matches Pd exactly and the
dry path matches within ±0.2 dB in every band; the fully-wet delay is
documented as ~10 dB brighter at 12.5–16 kHz than Pd (open item).

- **Module id 4**, the boot default. **44.1 kHz standalone** (user call
  2026-11-15; the Pd abstraction's rlpf constants are 44.1k-authored) and
  **32 kHz as a chain A** (the chain grid; the engine derives every filter/
  phasor constant from its construction rate). The int16 delay is 1 s in
  both contexts: 176,416 B standalone (2 × 44,004 samples), 128,016 B
  chained (2 × 32,004) — the smaller chain carve is what fits alongside
  clouds-B in the unified SRAM pool.
- **Engine**: Pd-style 64-sample internal blocks, rendered in bursts of one
  block per 4 host calls. Device cost at 44.1 kHz: **~663–680 µs per
  64-block (46% of the 1.451 ms period)** after the T8 voice optimization
  (voices 360 µs vs 508 before; delay ~220, LFO ~39, master ~42), i2s
  stalls 0; the 2.9 ms I2S bank covers the burst. The optimization hoists
  each oscillator's rlpf coefficient to one exp per block when cross-mod is
  zero (the patch default), keeping per-sample vibrato in `dt`; measured
  −59…−67 dB RMS vs the pre-change render, correlation 1.0000 (host goldens
  gate). The 2 s report shows `budget 1451 us` (the internal block, not the
  16-sample host grid) and a
  `sens=N st=<params>/<voices>/<delay>/<lfo>/<master>` tail (active sensors +
  per-stage maxima, µs, resets on print).
- **Voice architecture** (from the manual): 8 voices in four pairs (12/34/56/
  78), pairs in two groups (1234/5678). TUNE per voice; SHARP/MOD/FAST/FM-
  source per pair; PITCH/HOLD per group; TOTAL FB, VIBRATO and the FM
  structure switch are global.
- **MIDI**:
  - **Sensors**: notes **24–31 (C1–G1)** press/release sensors 1–8 — the
    patch's own notemap (36–43) shifted one octave down; **velocity scales
    the press peak** (the patch's e² gate law keeps the hardware's touch
    curve); latch triggers **CC 55–62**; PLAY (pseudo-CC 64) pulses sensor 1
    for 400 ms.
  - **ARP pitch** (T8b): **CC 54 (P3 "ARP HARM")** retunes all eight voices
    from the routed MOD-ARP's scale/root/chord — voices 1–4 a perfect bass
    stack (root/fifth over two octaves), voices 5–8 the scale chord
    (3rd/5th/7th/root), following W7 chord drift and NEO major/minor flips.
    Routing **arp1/arp2 to PITCH 1234 / PITCH 5678 (CC 25/26)** instead makes
    that group play the ARP's **exact step notes** as its root. While either
    is active, sensor notes bypass the ARP's MIDI-follow hijack (pads stay
    playable; other notes transpose the ARP root). Voicing math lives in
    `src/lira8/lira_harmony.h` (host-tested).
  - **CC map** (`module` prints it): 1–8 tune, 9–12 fast, 13–16 sharp, 17–20
    mod, 21–24 FM source (0 audio / 1 off / 2 LFO), 25–26 pitch, 27–28 hold,
    29 FM structure, 30 total-fb, 31 vibrato, 36 drive, 37 dst-mix, 38 vol,
    39 LFO freq A, **80 LFO freq B, 81 and/or, 82 link, 83–84 time 1/2, 85
    feedback, 86 delay mix, 87–88 delay mod 1/2**, **51 delay mode
    (0 SELF / 1 LFO-tri / 2 LFO-sqr)**, 52 LFO waveform (direct override),
    53 quantize (ON/OFF cell at P3 pot 14), 54 arp-harm (ON/OFF at P3 pot
    15). T8 moved the delay section 41–49 → 80–88 so the clouds-B alias bank
    (40–50) cannot steal it while lira8 is chain A.
  - Delay storage is int16 with a **soft-knee limiter** at the ±2 headroom
    (linear to ±1.5): at maximum FEEDBACK the self-oscillating loop saturates
    instead of hard-clipping.
- **UI pages** (`src/ui_link.h` tables; the C3 renders labels from TELEM):
  - **P1 = group 1234 + delay line 1** — row0: FAST 12, FM SRC 12, **MOD 1,
    TIME 1**, FAST 34, FM SRC 34; row1: TUNE 1–4; row2: SHARP/MOD of both
    pairs + PITCH 1234 + HOLD 1234.
  - **P2 = group 5678 + delay line 2**, mirrored (MOD 2 / TIME 2 at pots 3/4).
  - **P3 = shared delay + distortion + LFO + globals** — row0: DEL MODE, FB,
    MIX, DRIVE, DIST MIX, VOL; row1: FREQ A, FREQ B, TOTAL FB, VIBRATO;
    row2: FM STRUCT, AND/OR, LINK, QUANTIZE, ARP HARM (+1 spare).
  - Switch cells are **live** (no soft takeover): SLOW/FAST, FM/OFF/LFO,
    SELF/TRI/SQR, SPLIT/LOOP, ON/OFF.
- Chain: lira8 is **chain A pair 4** (`chain lira8` / C3 SYS pair selector
  `L>C`): lira8 at the 32 kHz grid → clouds (granular/looping + reverb).
  Pool demand 283,872 B of 331,776; window demand 71,672 B of 118,784.
  Standalone lira8 with an added reverb would also fit the pool (16,088 +
  176,416 + 65,536 = 258,040 B), but clouds-B is the intended effect stage.

## Elements × Warps stage (T9, internal cross-modulation)

The Elements module carries a Mutable **Warps** cross-modulation stage
between the resonator mixdown and the reverb — the canonical hardware patch
(Elements MAIN → Warps CARRIER, AUX → MODULATOR) rendered internally:

    exciter → resonator → mixdown → Warps (carrier = main, mod = raw exciter)
             → reverb → out

- **Carrier** = the resonator main output (post-SoftLimit); **modulator** =
  the raw exciter (`Part`'s `raw_buffer_`, the AUX-out signal at space 0).
  Output is mono into `main` (`aux` untouched); `warps_mix` 0 = untouched
  stereo, and **algorithm 0 = bypass with zero cost** (no Process call).
- **CC map additions** (all continuous mod-engine destinations; P2 page):
  **36 warps-algo** (0..1 — 0 bypass, 0.125 fold, 0.25 diode RM, 0.375
  digital RM, 0.5 XOR, 0.625 comparator; the DSP clamps there — the vocoder
  zone is excluded), **37 warps-timbre**, **38 warps-mod** (modulator LEVEL
  knob — the top end is the manual's warm overdrive via warps' own
  SaturatingAmplifier), **39 warps-car**, **40 warps-mix**.
- **Memory**: the `warps::Modulator` (~2 KB after stripping the vocoder and
  easter egg, `kMaxBlockSize` 16) is pool-placed behind the reverb buffer,
  whose engine was sized down to its true 21,617-frame Dattorro span
  (65,536 → 43,264 B carve) — no new static .bss, no pool growth.
- **Sources**: vendored in place at `rack-rp2350/lib/elements/warps/` (from
  the pristine pichenettes eurorack warps via the ctag-tbd fork, then edited
  directly — the repo no longer carries a patch/sync layer; the lib IS the
  source of truth). Hot code rides the `.tc_elements.warps_*` sections
  (window A slot).
- **Excluded on this build**: the vocoder algorithm (its filter-bank SRCs
  silently drop block sizes not divisible by 3/4 — a 48-sample burst scheme
  would be needed; revisit as a phase-2 plan) and the easter-egg frequency
  shifter.

## MOD-ARP VARG — black-metal generative algorithm (6th ARP algo)

`mod arp1 algo varg` (ALGO pot positions 16–18 = VARG 1/2/3 = 1/2/3 octaves).
Mono riff + harmony like every ARP algo — but the harmony engine generates
black-metal progressions with the W7 / Neo-Riemannian kernels:

- **p1 RIFF** — rhythm archetype + density zones: SUSTAIN (half-bar holds) /
  HALF (quarters) / 8TH / 16TH (tremolo) / GALLOP (3+1) / BLAST (every step;
  a true 32nd blast uses DIV 1/32).
- **p2 PROG** — progression archetypes: AEOLIAN (i–bVII / i–bVI–bVII),
  OSTINATO (i–bIII–bVI–bVII), WEITZMANN (augmented-anchor region cycle),
  MEDIANT (NEO R→P / L→P minor mediants), ADD9DIM (Am(add9)–D#dim7–G#m–A
  seed), HALFTRN (root ±1/±6), B2B5 (Phrygian power-chord pool), NEOFREE
  (NEO walk).
- **p3 VOICE** — chord-pool voicing: POWER / TRIAD / TRITONE / DIMAUG /
  ADD9 / PEDAL (tonic pedal note joins the pool).
- **p4 DISS** — NEO substitutions, chromatic planing, b2/tritone passing
  tones, riff-cell mutation, and section key shifts every 8 bars.
- Shared ROOT/SCALE/TRANS still apply: MIDI/PARAL set the tonic, **W7 forces
  the diatonic kernel, NEO forces the NEO walk** (TRANS pins the operator).
  LIRA-8 CC 54 (ARP HARM) voices the generated chord on all eight voices;
  the engine publishes root + chord-tone pool (not a scale degree).
- Engine: `src/varg.h` (pure, host-tested via `test/test_varg.cc`).
- Both ARPs on VARG keep independent progressions (shared ROOT/SCALE still
  locks the key); the first routed ARP is the LIRA-8 harmony source.
- **Shared ARP octave transpose**: the MOD-ARP page `OCTAVE` pot (or
  `mod <arp> octave <-2..2>`) shifts **all six** ARP algos by −2 / −1 / 0 / +1 /
  +2 octaves. For VARG it shifts both the mono riff and the chord published to
  Rings mode 5 / LIRA-8 ARP HARM.

## Rings VARG chord mode + post-render distortion

RINGS mode 5 (`cc 20 5`, shown as `MODE 5/5` on the C3) is a **simultaneous
chord** engine: instead of the native one-note-per-strum resonator, it renders
the VARG chord-tone pool with the string-synth voice bank (all tones at once —
no arpeggiation). The chord is read from the same bar-quantized harmony
snapshot LIRA-8 uses; the published root + pool are converted to absolute MIDI
notes (`src/rings_chord.h`) and re-strummed on every chord change. MIDI notes
keep firing the strum while a VARG ARP is routed (e.g. `mod arp1 dest voice`).

The four ctag-tbd fast distortion algorithms run **after the render in every
Rings mode** (0–5), not just the chord mode (`src/rings_dist.h`, ported from
ctag-tbd commits `2166c84c` / `4f086145` / `a62ad652`):

- **CC 24** drive (0 = zero-cost bypass, default 0.55),
- **CC 27** type: 0 tube (SoftClip knee glue), 1 fold (Vult/Befaco wavefolder),
  2 fuzz (biased asymmetric SoftClip), 3 shape (TBD03's seeded waveshaper),
  4 **swede** (Swedish Chainsaw — HM-2's three maxed graphic-EQ biquads +
  cascaded tanh stages, from Barabas5532/SwedishChainsaw; CC 28 >= 64 selects
  the "tight" 700 Hz input HP),
- **CC 28** bias (fuzz only; also Swedish "tight" HP), **CC 29** tone (1 = open).
  There is no wet/dry mix control: **CC 24 drive is the amount** (0 = zero-cost
  bypass, stage fully wet whenever engaged).

The C3 Rings P1 page exposes drive / type / mix on the three formerly empty
pots. The shared SRAM window's chain-A boundary moved from 0xE400 to 0xEC00
(60,416 B) to fit the patched `tc_rings` hot set (60,336 B); the B slot still
fits with ~1.4 KB of window slack.

## Elements FX — 5 distortions + Warps (one active)

Elements runs **both** DSPs compiled in, with only one active per block — no
parallel execution, no SRAM overlay. **CC 42** is the FX type selector:

| CC 42 | FX | Controls |
|---|---|---|
| 0 | **OFF** — no FX in the chain before reverb | — |
| 1-5 | distortion (tube/fold/fuzz/shape/swede, same stage as Rings) | CC 43 drive / 44 bias / 45 tone |
| 6 | **Warps** (original T9 internal cross-mod stage) | CC 36 algo / 37 timbre / 38 mod / 39 car / 40 mix |

The Warps controls are the **original T9 mapping** (CC 36-40), staged and
applied to the Patch only when FX type = 6; in distortion/OFF modes they are
zeroed so `Part::Process` skips Warps (zero cost). Likewise `ringdist::process`
is skipped in OFF and Warps modes. Switching FX preserves both sets of control
values.

**Signal routing (rack build):** the raw exciter is the Warps MODULATOR ONLY.
`part.inc` forces `raw_gain = 0` at **every** space setting, so `main`/`aux`
always carry the clean stereo resonator (R/L; at `space=0` both are the centre
sum) into the pre-reverb distortion insert, the Dattorro tank and the I2S
outputs. No exciter is heard directly — a non-zero CONTOUR (envelope shape)
previously made the sustained raw exciter bleed into the tail as "crackles".
Carrier and modulator are mixed only inside the Warps stage when FX TYPE = 6.
The standalone build keeps upstream `raw_gain` (the delta is guarded by
`RACK_RP2350_TC` and maintained in
`tools/patches/elements/elements-warps-stage.patch`).

The distortion is the same 5-algo stage Rings uses; on Elements it runs
INSIDE `Part::Process`, after the mixdown/Warps stage and **before** the
Dattorro reverb (reverb is the last FX — nothing shapes the tail). CC 42 picks
the algorithm (1 tube / 2 fold / 3 fuzz / 4 shape / 5 swede; 0 = OFF); the
shaper **replaces** the signal outright (100% wet, no dry path), drive 0 is an
additional zero-cost bypass, and drive alone is the amount.

The C3 P2 cells switch with the FX type (CC 42 cache): distortion shows FX
TYPE + D DRIVE/BIAS/TONE; Warps shows FX TYPE + W ALGO/TIMBRE/MOD/MIX/CAR. No
C3 reflash needed for label changes.

## SRAM architecture (T2 + T6/T6b two-slot window)

- **One window for all five modules**: `memmap_rack.ld` (single script, every
  env) places the elements/plaits/rings/lira8 `.tc_<module>.*` hot sets into
  ONE ld
  OVERLAY — same VMA = the window origin (the chain A slot), with clouds in
  its OWN section at origin+0xE400 (the T6 chain B slot; T6b moved it from
  +0x9000 so rings fits the A slot); LMAs are sequential in the reserved
  flash region RACK_LMA (origin 0x10389000). Measured members (rack ELF,
  2026-11-06): elements 36,752 B, clouds **56,992 B** (T6b demotion, was
  79,744), plaits **14,132 B** (T6b chain-tier, was 108,732), rings 58,084 B
  — the window is 0x1D000 (118,784 B; unchanged since T6). The A slot is
  0xE400 = 58,368 B (fits rings with 284 B spare and plaits' chain-tier
  image with 44 KB spare — all four A members are chain-A-capable); A + B
  = 115,360 B → window slack 3,424 B.
  `rack_window_copy(module_id)` copies ANY member into the window (boot or
  swap); in single-module mode the copied image is the only live one, in
  chain mode A + clouds-B are live together (see Chain mode v2).
- **Pool** (`src/rack_arena.h`, T8): ONE unified **331,776 B** .bss pool
  (was object 132 KiB + buffer 192 KiB as two separately-sized unions),
  `alignas(8)`, served by a deterministic bump allocator that is reset once
  per resident set (boot/swap/chain engage/off). Every adapter declares
  `state_size()` + `buf_bytes()` and takes `(obj, buf)`; a chain places
  A state+buffers then B state+buffers from the same reset. The module's
  placement/carve is the live one (UNION discipline; swaps re-place +
  re-Init factory defaults, no destructors in the five DSP states).
  Demands: elements Part 113,488 B + 64 KiB reverb; clouds processor
  8,104 B + 128 KiB large + 64 KiB small; plaits Voice 21,748 B + 32 KiB
  allocator workspace; rings Part 123,352 B + StringSynthPart 2,004 B +
  64 KiB reverb; lira8 engine 16,072 B + 176,416 B delay (44.1 kHz) /
  128,016 B (32 kHz chain). Worst chain (rings+clouds) 331,516 B — the
  same 260 B slack as before the merge; lira8+clouds 283,872 B.
- **RAM per env** (identical map in all five): static .data + .bss (of which
  pool 324 KB → non-pool static ≈ 35 KB) → heap **33,744 B** below the
  window; boot heap consumers: core-1 separate stack 8 KB, I2S DMA bank
  2-3 KB, USB device+host buffers → ≥ ~16 KB headroom maintained.
- **Plaits re-tier** (T2 prerequisite; patches `05/06/07`) **+ T6b chain-tier**
  (user-approved; patch `09-plaits-chain-tier.patch`): tc_plaits
  272,564 → 108,732 → **14,132 B** in the rack. KEPT in the window: the
  Voice::Render path (voice + post-processor + sub-oscillator), engine 0
  (VirtualAnalog — boot default) and `lut_sine`. DEMOTED to flash in the
  rack (`PLAITS_RAM_CODE_DEMOTABLE` — the standalone keeps every mark and
  produces a byte-identical firmware, sha256-verified):
  - the T2 long tail (~54 engine bodies ~116 KB + engine-only LUTs 47.6 KB —
    kWaves 32 KB, kBuzzZones 7.7 KB, lut_fold×2, lut_ws×4,
    lut_lpc_excitation_pulse, kFormantShapeSin), as before, and now also
  - the fork-flagged hot engines (chord, particle, modal, string, chiptune,
    six_op, scanned — 37.9 KB), the physical-modelling/fm support DSP they
    call (String + ProcessInternal, Resonator, ModalVoice, StringVoice,
    `fm::Voice<6>` + `fm::RenderOperators` — 14.7 KB) and
    `wav_integrated_waves_1/2/3` (50,688 B — chord was their only
    window-resident reader, so chord now runs from flash entirely, LUTs
    included).
  Consequence (accepted by the user): SWITCHED plaits and chained plaits
  both run those bodies through the XIP cache — sound-neutral, slower
  sustained; the per-engine re-sweep is the T7 bench. Per-engine promotion
  back to the window = one token line (window slack 44 KB).
- **Clouds spectral excluded + T6b dead-mode demotion** (plan, "SRAM
  architecture"; patch `tools/patches/clouds/clouds-tc-demote-dead-modes.patch`
  + `tools/patches/cmsis_dsp/01-rack-tc-names.patch`): Clouds playback mode
  3 (spectral) is REJECTED by the rack adapter (CC 23 values ≤ 2 only), so
  its code path is DEAD in every rack build. T6b made that official: the
  spectral code (STFT::Process/Buffer, PhaseVocoder, FrameTransformation —
  1,916 B) + the CMSIS FFT CODE (arm_cfft*/radix8/rfft_fast — 3,122 B; only
  stft.cc ever called it) + `lut_sine_window_4096` (16,384 B — the spectral
  window) + the dead-KEEP'd tables (`lut_db`, `src_filter_1x_2_31/63/91` —
  1,254 B, no remaining reader; kept alive only by the linker's KEEP)
  **moved out of the SRAM window into flash** (22,752 B; the four dead tables
  were garbage-collected entirely). tc_clouds 79,744 → 56,992 B — that is
  what paid for the 0xE400 A slot WITHOUT growing the window. The demotion
  is rack-conditional (`CLOUDS_RAM_CODE_DEMOTABLE` in the clouds
  `ram_code.h`): standalone clouds keeps every mark and produces a
  byte-identical firmware (sha256-verified). Granular/stretch/looping keep
  their hot code in the window, including `lut_window` (granular
  GRAIN_QUALITY_HIGH envelope + stretch) and `lut_sin` (granular pan gains);
  stretch stays fully available in single-module mode.

### Per-engine tier table (plaits — RACK tiers; the standalone keeps the full T3 hot set)

| Engine (slot) | Rack tier | Basis |
|---|---|---|
| virtual_analog (0) | **SRAM (window)** | boot default; T5b ≤148 µs — the ONLY engine the rack keeps window-resident |
| waveshaping (3), fm (4), grain (5), additive (6), wavetable (7), speech (9), swarm (12), noise (13), va_dual (1), va_crossfade (2), va_variant (71) | flash | unflagged long tail; speech measured 25 µs sustained (T5b, SRAM basis) |
| bass_drum (17), snare_drum (18), hi_hat (19) | flash | drum tail (drum support headers demoted too) |
| chord (8), particle (14), string (15), modal (16) | **flash (T6b chain-tier)** | fork-flagged hot set — demoted 2026-11-06 (user-approved); T5b SRAM basis: particle/chord/modal/string bodies; from XIP expect ~1.5-3× sustained |
| six_op / dx7 a/b/c (22-24) | **flash (T6b chain-tier)** | dx7 67-69 µs sustained (T5b, SRAM basis) |
| chiptune (27) | **flash (T6b chain-tier)** | fork-flagged hot set, demoted with the rest |
| formant/lpc/natural/sam speech (10, 11, 28), vosim (59) | flash | speech measured 25 µs, vosim 100-115 µs sustained (T5b, SRAM basis) |
| scanned (31) | **flash (T6b chain-tier)** | fork-flagged hot set, demoted with the rest |
| buzz (49), wave_scan (67), shakers (84), acid (96), noise_bank (53), analog_percussion (91), freshets (92), bubbletime (93), zx* (94/95), grainlet-based (5, 60) + ~55 more | flash | unflagged long tail (T3 method: working set < XIP cache) |

(Engine slots per the 98-catalog registry order. The T5b sweep measured
every engine ONCE with everything in SRAM — zero stalls, sustained worst
≈ 268 µs of the 500 µs budget at 48 kHz. Since the T6b chain-tier ONLY
engine 0 + the voice path stay window-resident in the rack: switched AND
chained plaits run every other engine from XIP — sound-neutral, sustained
cost expected ~1.5-3× the T5b number, to be re-swept on the T7 bench.
Per-engine promotion back = one token line; window slack after the
chain-tier is ~44 KB (A slot 58,368 − tc_plaits 14,132).)

## Chain mode v2 (T6)

One A→B serial chain, bench-controlled via the serial console (`chain`,
`chain <a> [b]`, `chain off`; no CC for T6 — the C3's CHORD_ROUTE arrives
later). The chain grid is **32 kHz / 24-sample blocks (750 µs)**: A renders
24 samples into float buffers (elements internally as 16+8 — `Part::Process`
scratch is kMaxBlockSize 16), the route copies A's out/aux into clouds'
stereo `FloatFrame` input, clouds-B processes 24 frames, and **B's output is
the chain output** (clouds is the effect stage). Entering/leaving is a
rate-context event: mute (4-block ramp + silent bank flush) → both slot
images re-copied flash→window (checksummed) → I2S reconfigured on CORE 0
via a handshake identical to the swap's → both slots re-Init (a chained
plaits would re-capture 32 kHz via the runtime `kSampleRate`,
`08-plaits-runtime-rate.patch`) → unmute. Prints:
`rack: chain <a>-><b> engaged (<ms> ms)` / `rack: chain off (<ms> ms)`.

### Pair table (user-amended 2026-11-05 — all X→clouds) vs chain residency

Chain mode needs BOTH slots' hot images resident in the two-slot SRAM
window simultaneously (memmap_rack.ld: the elements/plaits/rings overlay at
the window origin = the A slot; clouds' own section at origin+0xE400 = the
B slot). Engagement is validated at the console against the REAL
linker-symbol sizes:

| Pair | A hot (B) | B hot | Window need (B offset + B) | Pool need (A state+buf + B state+buf) | Verdict in this build |
|---|---|---|---|---|---|
| elements→clouds | 36,752 B | 56,992 B | 58,368+56,992 = 115,360 ≤ 118,784 | 113,784+65,536+8,692+131,712 = 319,724 ≤ 331,776 | **ENGAGEABLE** (benched T6: 373-401 µs) |
| rings→clouds | 58,084 B | 56,992 B | 58,368+56,992 = 115,360 ≤ 118,784 | 125,708+65,536+8,692+131,712 = 331,648 ≤ 331,776 (128 B spare) | **ENGAGEABLE** (T6b; budget clamp below) |
| plaits→clouds | 14,132 B (chain-tier) | 56,992 B | 58,368+56,992 = 115,360 ≤ 118,784 | 21,856+32,768+8,692+131,712 = 195,028 | **ENGAGEABLE** (T6b chain-tier; flash-engine budget = bench gate) |
| lira8→clouds | 14,680 B | 56,992 B | 58,368+56,992 = 115,360 ≤ 118,784 | 15,920+128,016+8,692+131,712 = 284,340 ≤ 331,776 | **ENGAGEABLE** (T8; 32k chain context) |

The T6b geometry (clouds dead-mode demotion 79,744 → 56,992 B + 0xE400 A
slot) made rings→clouds resident; the user-approved plaits chain-tier
(09-plaits-chain-tier.patch: tc_plaits 108,732 → 14,132 B — the rack's
window image keeps ONLY the voice path + engine 0 + lut_sine) made
plaits→clouds resident; T8's unified 331,776 B pool made lira8→clouds
resident (283,872 B needed — the old split rejected it at 259,088 B of
buffer while 110 KB of the object arena sat idle). The benches must still
measure the engages (budgets below).

**Per-pair budget math (standalone numbers, sustained):**
elements 101-207 µs @16-sample blocks (≈150-310 @24) + clouds granular
238-260 µs @32-sample (≈180-195 @24) + mod engine ≤50 µs ≈ **380-555 µs of
750 (51-74%)** — benched at 373-401 µs sustained (T6). rings typical
270-320 µs + clouds ≈ 500-565 µs (67-75%) — but rings' sympathetic-string
models measured **511 µs ALONE** (standalone T5a sweep) → 756 µs with
clouds+mod: over budget, hence the chained-rings clamp below. plaits@32k:
engine 0 (SRAM) ≈ 25-35 µs → pair ≈ 260-280 µs (35-37%); flash engines =
T5b sustained ÷1.5 (32 kHz) × an XIP factor ~1.5-3× (bench to measure) —
the T5b sustained worst 268 µs (48k SRAM basis) bounds 179 µs at 32k SRAM,
i.e. **270-540 µs from flash** + clouds 245 ≈ **515-785 µs of 750**: the
worst engines may exceed the pair budget from flash — the bench sweep
decides, and any offender gets a one-line token promotion (window slack
44 KB) or a documented rejection.

### Chain-mode behavior (documented choices)

- **CC routing while chained**: CC 23/24/65/66 AND the clouds grain alias
  bank **CC 40-48 → the B slot** (clouds mode clamp / quality / freeze /
  trig / grain params — 65/66 are unused by every chain-A map, 40-50 is
  unused by every map after the T8 lira8 remap); every other CC → the A
  slot. CC 23 is dual-mapped in every module's map (elements geometry /
  plaits engine / rings chord), so while chained it MEANS clouds mode and
  the A slot loses CC 23/24 (the mod engine can still drive A —
  destinations bind to A).
- **Clouds grain alias bank (v2)**: CC 40-48 always addresses clouds' grain
  parameters 14-22 (position..reverb) whenever clouds is the TARGET —
  single-module clouds via the registry dispatch, the B slot while chained
  via the chain intercept (translated in `rack_chain_control_change`). The
  band is always on, so a controller sends 40-48 in both modes with no
  chain-state knowledge; native 14-22 keep their existing meaning everywhere
  (A's panel params while chained — the collision that makes the alias
  necessary). Free-band check: 40-50 is unused by all five maps + the rack
  system CCs (32 bank, 33-35 mod mux) — lira8's delay section moved to
  80-88 in T8 specifically to keep this true.
- **Clouds PITCH range (fixed T8)**: the normalized panel value is mapped to
  **±48 semitones** (Clouds' hardware range), not the raw 0..1 that only
  moved one semitone. Applies to single-module clouds and the chain B slot.
- **Dual-core chain render (T8)**: core 1 renders the A slot and writes I2S;
  core 0 renders clouds B through two lock-free 4-slot SPSC rings. The
  per-block wall cost becomes A + handoff instead of A + B; the A slot's
  burst no longer delays clouds. Blocks carry a sequence number and core 1
  consumes `B(n-2)` — a depth-aware lag, not "whatever is newest": core 0
  gets two block periods (1.5 ms) to finish, which absorbs its 350-1200 µs
  loop-latency outliers, so core 1 does not wait in steady state. The B ring
  is primed with two sync-rendered blocks at pipeline start (the first
  pipelined block finds its B already waiting; an underrun is silenced and
  NOT consumed, so the stream resumes in order). Transitions (mute ramp,
  window copy, init) run the synchronous path with the B worker paused
  (`pipe_run` gate + busy handshake), then re-enable the pipeline after the
  ramp-up. Extra latency: two chain blocks (1.5 ms). While chained the 2 s
  report shows `bm=` (B cost), `c0=` (core-0 loop max) and `u=` (B-not-ready
  underruns).
- **The I2S DMA IRQ lives on the AUDIO core (core 1)**: the
  AudioBufferManager's list helpers use `noInterrupts()`, which is per-core;
  with the writer on core 1 and the IRQ on core 0 the `_empty/_filled` lists
  raced and eventually corrupted (persistent stall storms that only a reload
  cleared). Core 0 disables `DMA_IRQ_0` after every `i2s.begin()`; core 1
  enables it in `setup1()` and masks it across each runtime reconfig. Never
  enable the I2S DMA IRQ on core 0 again.
- **UI-link TX is non-blocking (T8)**: frames are queued in a 4 KiB ring and
  drained by `ui_link_tx_pump()` in the core-0 loop, writing only what the
  UART TX ring can take immediately. The preset autosave PART burst (1,071 B,
  3 s after any CC) used to block core 0 for ~10 ms, starving the core-0 I2S
  DMA IRQ and producing audible chain dropouts ("grit at high dry/wet").
  `ui` prints `dropped=` for ring overflows. Verified: a PART burst landing
  during a full-wet lira8→clouds chain reports i2s stalls 0.
- **Clouds-B mode clamp**: chain mode is granular/looping ONLY. CC 23 = 0
  (granular) / 2 (looping) accepted; **1 (stretch) and 3 (spectral)
  REJECTED** with a console line, no crash (belt-and-braces: the adapter's
  apply-time clamp lands anything illegal on granular). Stretch stays
  available in single-module mode; spectral is excluded everywhere (T2).
- **Chained-rings budget clamp (T6d, re-measured)**: while rings is the
  chain A slot, the T6b blanket model clamp is GONE — the sympathetic pair
  was MEASURED on hardware at **692 µs of 750 (92%)**, well under the
  predicted 756. One narrow guard remains (single-module rings is fully
  unclamped):
  - **CC 19 (polyphony) > 2 while a sympathetic model (1/4) is live in a
    Part mode**: rejected with a console line — the sympathetic+high-poly
    corner is the only combination whose pair cost is unmeasured (the T5a
    sweep bounded poly-4 by the poly-1 sympathetic worst ALONE; the pair
    adds clouds+mod). Poly 1-2 sympathetic plays: measured 655-692 µs,
    zero stalls.
  - Everything else is open while chained: all models in all Part modes
    (incl. sympathetic 1/4), mode 4 easter egg across fx 1-5, poly 1-4 on
    non-sympathetic models (the sweep's poly-4 rows never exceeded the
    poly-1 sympathetic worst, which the poly guard now covers).
  Elements A needs no clamp (193-207 µs benched); plaits A has no CC clamp
  either (its chained cost is engine-dependent — the bench sweep + the
  per-engine promotion path govern it, see the pair-table budget math).
- **Routing scale**: the route passes A's floats at **gain 1.0** — the
  adapters' outputs are the same ±1.0 class the I2S write path Clip16s, and
  the standalone test exciter produced 0.6-0.8 peaks, so routed material
  lands at the level the granulator was validated with. No fixed gain
  stage.
- **Exciter bypass**: while chained, clouds' test exciter is bypassed
  entirely — the routed frames ARE the exciter input (`parameters.gate`
  stays false; its only consumer is the excluded spectral path).
- **Chained clouds buffer split** (documented deviation): chained clouds
  runs large 90,496 B (stereo: 41,216 B sample memory/channel + 49,280 B
  FX workspace) + small 41,216 B = 131,712 B. The FX workspace must be
  **≥ 49,152 B** (8,192 diffuser + 32,768 reverb + 8,192 correlator/pitch
  shifter — the lib patch `clouds-z-pitchshifter-workspace.patch` makes
  that region cover the pitch shifter's full FxEngine<4096> clear; before
  it, Prepare() wrote 6.6 KB past the workspace). Stereo 0.64 s/channel.
  **Mono qualities are coerced to stereo while chained**: they need a
  49,152 B workspace in the small buffer, and the tightest pair (rings)
  cannot fit `2*small + 49,152 ≤ 131,840`. Mono stays available in
  single-module clouds. Grain parameters are unaffected.
- **Mod engine in chain mode**: continuous destinations bind to the A slot
  (the sound source; `g_active_module` IS A while chained, and the engine's
  per-block pass runs the chain grid via the additive `g_mod_chain_*`
  overrides), **except the clouds-B band** (T8c, 2026-11-16): CC 40–48 (the
  grain alias bank) and 65/66 (freeze/trig) are accepted as destinations and
  routed through the same `rack_chain_control_change` the MIDI intercept
  uses — the C3's assign gesture works on the CLOUDS B page (its cells show
  the routed/yellow state). Mode/quality (49/50) stay panel selectors, not
  mod destinations.
- **Swap-while-chained (decision 5)**: CC 32 / `switch` while chained
  targets the chain slots — v == A: fresh re-Init of A (no I2S change, no
  window copy; routing re-patches automatically), v == B: rejected (clouds
  is the only B in T6), otherwise rejected with the A-slot window math.
- **Report**: the module field extends to `mod=elements>clouds` and the
  budget field to the chain's 750 µs; `proc_us_max` times A+route+B
  honestly (the write stage stays excluded, same convention as the
  adapters); meters become the B slot's in/out RMS.

### Plaits @ 32 kHz — runtime rate (`08-plaits-runtime-rate.patch`)

Survey of every `kSampleRate`/`kCorrectedSampleRate` use in the fork (rack
ELF-verified): **all code uses are runtime float arithmetic inside
Init()/Render()** — drum envelope times, SVF cutoffs, delay lengths,
SAM/LPC frame timers, six-op FM voice Init, brass per-sample scaling. NO
compile-time use anywhere (no array sizes, template arguments, case labels,
or static initializers outside dsp.h). Mechanism: under
`-DRACK_PLAITS_RUNTIME_RATE` (rack envs only — the standalone builds stay
byte-identical, proven by firmware.bin sha256), `kSampleRate`,
`kCorrectedSampleRate` and `a0` become runtime globals set by
`plaits::SetSampleRate()`; `kCorrectedPitchOffset` stays compile-time 0
(both rates track the same true rate → log2(1) = 0 at ANY rate), so the
derived namespace-scope constants (`kParticlePitchCorrection`,
`kStruckDrumPitchCorrection`) never become a static-init-order hazard.
Engines capture the rate at Init — the chain state machine re-Inits the
Voice on every rate-context change.

Documented 48k-baked exceptions (behave slightly differently at 32 kHz, by
design accepted):

| Engine | What is baked | 32 kHz effect |
|---|---|---|
| zxphase48k (94) / zxpulse48k (95) | fixed-point phase increments, "one internal (4x) step per sample" at 48 kHz out | internal time effects (1-bit flanger delay, pin widths) stretch ×1.5 in seconds |
| particle_burst (14) | `kParticleStockDecay` = 64763/65536 per-48kHz-sample envelope coefficient | burst decays stretch ×1.5 in real time (pitch/pitch-correction stay exact) |

## Wiring

Identical to the four standalone ports: PCM5102A BCK GP6 / DIN GP8 /
LRCK GP7, XSMT 3V3; USB-A host D+ GP14 / D- GP15; WS2812 GP22; USB-C =
CDC serial (115200) + USB MIDI device ("Rack RP2350"). T4 adds the MIDI
DIN-IN jack on GP5 (6N138 opto — table above; GP4 = reserved UART1 TX).

## Boot-wedge notes (2026-11-04)

Three real findings from the bring-up:

1. **First CDC print vs enumeration.** An unconditional `Serial` write within
   milliseconds of boot (while the host is still enumerating) wedged the chip
   nondeterministically. Fix: the window line is composed early and printed
   once enumeration has settled (~1 s in); conditional error prints stay
   early, like the proven standalone.
2. **The detach/attach re-enum guard was itself a boot killer.** The old
   "re-enumerate so the host sees MIDI" detach intermittently died in the
   post-detach enumeration (kernel log: SET_ADDRESS refused / descriptor
   timeouts until a power cycle) — the same "CDC port death" family the
   standalone era patched with a 500 ms window, never eliminated. Fix: the
   MIDI interface is registered as the FIRST setup action, so it is already
   in the host's first configuration read (≥100 ms after attach per USB
   reset recovery) — the guard is deleted entirely; the device now
   enumerates exactly once.
3. **The 1200-baud touch is flaky-destructive.** `reset_usb_boot` sometimes
   leaves the BOOTROM unable to enumerate (kernel log: attach without
   descriptors ×3 → "unable to enumerate" → dead to USB until a power
   cycle). It works reliably when NO reader holds the CDC port open (the
   touch requires DTR low — a serial monitor blocks it); if the drive fails
   to appear after two touches, fall back to button-BOOTSEL. For ANY "board
   wedged" report: read `journalctl -k` first — attach-without-descriptors
   is a touch/bootrom victim, not a firmware hang.

## Report format

`max Process: N us (budget B) | i2s stalls: T (+d) | in= out= notes= |
host: conn= st= dev= midi= frame= | mod=<active module>` — budgets:
elements 496, clouds 1000, plaits 500 (48 kHz), rings 750 µs, **lira8 1451
µs** (its internal 64-sample block at 44.1 kHz — the engine bursts once per 4
host calls and the 2.9 ms I2S bank absorbs the burst; `in=`/`out=` are
max-since-report meters ×1000 and lira8 reports its output RMS in `in=`); lira8 appends
`| sens=N st=<params>/<voices>/<delay>/<lfo>/<master>` (active sensor count +
per-stage maxima in µs, reset on print). While chained
the module field reads `mod=<a>><b>` (e.g. `elements>clouds`) and the
budget is the chain grid's 750 µs with `proc_us_max` = A+route+B. Swap
lines: `rack: switched to <module> in <ms> ms (window <n> bytes)`; chain
lines: `rack: chain <a>-><b> engaged (<ms> ms)` / `rack: chain off (<ms>
ms)`. `telemetry off`
mutes ONLY this line (cadence + counter resets continue; `status` prints a
one-liner instead). T4 DIN counters deliberately live in `din`/`status`, not
here — the format above is byte-compatible with the T1/T2/T3 bench history.

## Reset forensics + overclock bench (no SWD on this board)

The BRIDLE mini_usb_rp2350 has no SWD, so the console + the SRAM crash record
(`.uninitialized_data`, survives warm resets) are the only fault tools.

At every boot the firmware captures the previous boot's evidence *before*
`watchdog_enable()` (which would overwrite the marker in `scratch[4]`), then
prints:

```
boot: reason=<N> (0 cold | 1 POR/hardware | 2 fault | 3 watchdog-in-loop |
                  4 watchdog-in-setup | 5 clean-reboot) wd=0x<bits>
      unhealthy_boots=<n> fault_streak=<n>[ SAFE-MODE(240 MHz) ...]
```

`wd` is the raw `WATCHDOG_REASON` bits, printed **for reference only**: on this
chip the TIMER bit was observed sticky across soft resets (a `reboot` after a
BOOTSEL touch still showed `wd=0x01`). The decisive signal is three firmware-
owned scratch words (POR clears them, soft/watchdog resets preserve them):
`scratch[4]` boot-done, `[5]` clean-shutdown (set by `reboot`), `[6]`
setup-active. So:

- **reason 4** = died DURING setup (setup-active set, no boot-done) — the real
  clock verdict: the clock hung XIP/init before setup finished.
- **reason 3** = reset after setup completed (loop-time hang/external).
- **reason 5** = intentional `reboot`; **reason 1** = POR/brownout/USB flap
  (not a clock verdict). Note the 1200-baud BOOTSEL touch itself uses a
  watchdog reboot and clears the boot-done marker, so the first boot after
  flashing typically reads reason 1 — ignore it; use `reboot` for clean tests.

A HardFault prints a one-shot `CRASH:`
line with `pc=...` (resolve with `arm-none-eabi-addr2line -e firmware.elf <pc>`).
`crash` reprints the record; `crash clear` re-arms after SAFE MODE. Sticky
`fault_streak >= 2` or `unhealthy_boots >= 3` forces one 240 MHz boot so the
record is always readable.

### Finding the stable frequency

**Boot default: 320 MHz, VREG 1.25 V, QMI CLKDIV 4 / RXDELAY 3 (SCK 80 MHz).**
VREG is 1.25 V (not 1.20): the Warps DSP's sustained load at 320 MHz locked
the core up at 1.20 V under repeated CC/switch traffic; 1.25 V is stable.
A pre-clock constructor in `main.cpp` raises VREG and pins the QMI timing
before the core applies `F_CPU` — the core's own VREG auto-adjust is RP2040-
only. Hardware sweep: 270 MHz at the default 1.15 V resets in soak even at
SCK 67.5 (so the lever is **core voltage**, not flash timing); at 1.25 V,
270–320 soak clean, 328 is the outer edge, 336 is the cliff. The old 280
rejection was CLKDIV 3 (SCK 93 MHz) — at CLKDIV 4 it is clean.

Flash SCK = `clk_sys / CLKDIV` (the RP2350 ROM pins CLKDIV 3 / RXDELAY 2;
`set_sys_clock_khz` does NOT re-scale it). Keep SCK ≤ 88 MHz by raising CLKDIV
as the PLL goes up:

```
oc <khz> [clkdiv] [rxdelay]     # live switch; prints FNV + a CLKDIV hint
oc off                          # restore build clock, stop FNV watch
qmi                             # read live CLKDIV/RXDELAY/SCK
reboot                          # full restore (also Serial1/I2S dividers)
```

`oc` re-reads the boot module's FLASH through XIP and compares its FNV against
the boot-time window-copy checksum (`fnv=OK|MISMATCH`). With a clock override
active the 2 s report gains `| oc: <khz> kHz fnv=OK|MISMATCH` — a **continuous**
integrity check over a soak, which is what catches the intermittent corruption
that killed 280. The check is read-only; core 1 executes from the SRAM window
and is never disturbed. Above 280 MHz `oc` raises VREG to 1.20 V. Serial1/I2S
dividers stay stale until `reboot` (audio off-rate, UI link dead) — bench with
the console, not the C3.

**Suggested sweep** (each step: `oc`, wait for a 2 s report, watch `fnv=` and
`max Process`, then soak ≥ 1 min; after any reset read `boot: reason=`):

```
oc 328000 4 3    oc 336000 4 3    ...   (bench only; 320 is the boot default)
```

Advance while `fnv=OK`, stalls stay 0 and no reason-2/4 reset appears; back off
one step at the first MISMATCH or reason-4. Always finish with `oc off` /
`reboot` before flashing. **`vreg <1100|1150|1200|1250>`** reads/sets the core
supply live — raising to 1200 first is what makes >264 MHz stable.

Note: 320 MHz is not a 12 MHz multiple, so PIO-USB host timing would be
unreliable — it is disabled in this build (`RACK_ENABLE_USB_HOST=0`); use
312 MHz if host is re-enabled.
