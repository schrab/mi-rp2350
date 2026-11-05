# Rack RP2350 — all four MI ports in one firmware, live-switchable

ONE firmware running the four accepted ports (Elements, Clouds, Plaits
98-engine, Rings MiniElements) on the RP2350 nano + PCM5102A, with LIVE
MODULE SWITCHING (T2 of the combined plan,
`docs/superpowers/plans/2026-11-04-combined-rp2350.md`) and a DIN MIDI input
+ serial bench console (T4).

## Module selection (T2: boot-select + live swap)

Four PlatformIO envs — they build the SAME unified binary and differ ONLY in
which module boots:

```sh
pio run -d rack-rp2350 -e rack-elements   # or rack-clouds / rack-plaits / rack-rings
# → .pio/build/<env>/firmware.uf2
```

`RACK_BOOT_MODULE` (0 elements / 1 clouds / 2 plaits / 3 rings) selects the
boot module: `setup()` copies that module's flash image into the SRAM window
(first action, crt0-like) and registry-Inits it. All four adapters compile in
every env (state is arena-placed); the module registry
(`src/module_registry.h`, table in `src/main.cpp`) dispatches init / render /
note / CC hooks.

### Live swap — MIDI CC 32 (Bank Select)

**CC 32, value 0-3 = module id** (0 elements, 1 clouds, 2 plaits, 3 rings;
other values ignored, same id ignored). CC 32 is verified unused by all four
modules' CC maps. Core 0 queues the request; core 1 executes the swap at the
next block boundary (max one per call):

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
   rate + bank → `begin()` (48 kHz for plaits, 32 kHz for the others).
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
| `status` | one-line summary: active module, clock source/BPM, swaps, DIN counters, uptime |
| `din [clear]` | DIN MIDI counters + last message (+ `clear` resets them) |
| `dintest` | inject a canned MIDI stream into the DIN parser → funnel (see below) |
| `telemetry on\|off` | mute/unmute the 2 s report for quiet benching (default on) |
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

## SRAM architecture (T2 + T6/T6b two-slot window)

- **One window for all four modules**: `memmap_rack.ld` (single script, every
  env) places the elements/plaits/rings `.tc_<module>.*` hot sets into ONE ld
  OVERLAY — same VMA = the window origin (the chain A slot), with clouds in
  its OWN section at origin+0xE400 (the T6 chain B slot; T6b moved it from
  +0x9000 so rings fits the A slot); LMAs are sequential in the reserved
  flash region RACK_LMA (origin 0x10389000). Measured members (rack ELF,
  2026-11-06): elements 36,752 B, clouds **56,992 B** (T6b demotion, was
  79,744), plaits 108,732 B, rings 58,084 B — the window is 0x1D000
  (118,784 B; unchanged since T6). The A slot is 0xE400 = 58,368 B (fits
  rings with 284 B spare; plaits stays over it and is rejected as chain A at
  runtime); A + B = 115,360 B → window slack 3,424 B.
  `rack_window_copy(module_id)` copies ANY member into the window (boot or
  swap); in single-module mode the copied image is the only live one, in
  chain mode A + clouds-B are live together (see Chain mode v2).
- **Arenas** (`src/rack_arena.h`): object arena **132 KiB** (T6b, was 128 —
  the rings+clouds chain states need 134,908 B: RingsState 126,212 +
  CloudsState 8,692) + buffer arena 192 KiB, both `alignas(8)`, .bss-
  resident, used as UNIONS — the active module's placement/carve is the live
  one: elements Part 113,488 B + 64 KiB reverb; clouds processor 8,104 B +
  128 KiB large + 64 KiB small (fills the buffer arena exactly); plaits
  Voice 21,748 B + 32 KiB allocator workspace; rings Part 123,352 B +
  StringSynthPart 2,004 B (the object arena's sizing module) + 64 KiB
  reverb. Swaps re-place + re-Init factory defaults; no destructors exist in
  the four ports' DSP state.
- **RAM per env** (identical map in all four): static .data + .bss (of which
  arenas 332 KB → non-arena static ≈ 35 KB) → heap **33,120 B** below the
  window (T6: 37,640; T6b paid the object-arena bump's 4 KiB from the heap —
  the window itself did not grow); boot heap consumers: core-1 separate
  stack 8 KB, I2S DMA bank 2-3 KB, USB device+host buffers → ≥ ~16 KB
  headroom maintained.
- **Plaits re-tier** (T2 prerequisite; patches `05/06/07` regenerated):
  tc_plaits 272,564 → 108,820 B (−163,744). KEPT in SRAM: the Voice::Render
  path (voice + post-processor + sub-oscillator), support DSP
  (`fm::Voice<6>` + `fm::RenderOperators`, resonator, String +
  ProcessInternal, modal/string/scanned voices), engine 0 (VirtualAnalog —
  boot default), the fork-flagged hot engines (particle, modal, string,
  chord, chiptune, six_op, scanned) and the per-sample LUTs the kept set
  touches (`lut_sine`, `wav_integrated_waves_1/2/3` — the KEPT chord engine
  reads them per sample). DEMOTED to flash (XIP, small working sets — the
  plaits T3 method): the long-tail engine bodies (~54 functions, ~116 KB)
  and their engine-only LUTs (kWaves 32 KB, kBuzzZones 7.7 KB, lut_fold×2,
  lut_ws×4, lut_lpc_excitation_pulse, kFormantShapeSin — 47.6 KB).
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

### Per-engine tier table (plaits)

| Engine (slot) | Tier | Basis |
|---|---|---|
| virtual_analog (0) | SRAM | boot default; T5b ≤148 µs |
| waveshaping (3), fm (4), grain (5), additive (6), wavetable (7), speech (9), swarm (12), noise (13), va_dual (1), va_crossfade (2), va_variant (71) | flash | unflagged long tail; speech measured 25 µs sustained (T5b) |
| bass_drum (17), snare_drum (18), hi_hat (19) | flash | drum tail (drum support headers demoted too) |
| chord (8), particle (14), string (15), modal (16) | SRAM | fork-flagged hot set |
| six_op / dx7 a/b/c (22-24) | SRAM | fork-flagged; dx7 67-69 µs sustained (T5b) |
| chiptune (27) | SRAM | fork-flagged hot set |
| formant/lpc/natural/sam speech (10, 11, 28), vosim (59) | flash | speech measured 25 µs, vosim 100-115 µs sustained (T5b, SRAM basis) |
| scanned (31) | SRAM | fork-flagged hot set |
| buzz (49), wave_scan (67), shakers (84), acid (96), noise_bank (53), analog_percussion (91), freshets (92), bubbletime (93), zx* (94/95), grainlet-based (5, 60) + ~55 more | flash | unflagged long tail (T3 method: working set < XIP cache) |

(Engine slots per the 98-catalog registry order; the flash-resident engines
were measured ONCE with everything in SRAM — T5b sweep had zero stalls and
sustained worst ≈ 268 µs of the 500 µs budget; from XIP they are EXPECTED
slower — accepted by the plan, re-tier any offender = one token line, window
slack ~5.9 KB.)

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

| Pair | A hot (B) | B hot | Window need (B offset + B) | Object arena | Verdict in this build |
|---|---|---|---|---|---|
| elements→clouds | 36,752 B | 56,992 B | 58,368+56,992 = 115,360 ≤ 118,784 | 113,920+8,692 ≤ 135,168 | **ENGAGEABLE** |
| rings→clouds | 58,084 B | 56,992 B | 58,368+56,992 = 115,360 ≤ 118,784 | 126,216+8,692 = 134,908 ≤ 135,168 | **ENGAGEABLE** (T6b) |
| plaits→clouds | 108,732 B | 56,992 B | A slot check fails first | fits | rejected (A slot 58,368 B < 108,732; needs the chain-tier plaits image — an architecture decision, not a tuning knob) |

Rejected pairs print the exact numbers on `chain plaits clouds` and stay
single-module. The T6b geometry (dead-mode demotion 79,744 → 56,992 B +
0xE400 A slot + 132 KiB object arena) is what made rings→clouds resident;
the bench must still measure the engage (budget below).

**Per-pair budget math (standalone numbers, sustained):**
elements 101-207 µs @16-sample blocks (≈150-310 @24) + clouds granular
238-260 µs @32-sample (≈180-195 @24) + mod engine ≤50 µs ≈ **380-555 µs of
750 (51-74%)** — benched at 373-401 µs sustained (T6). rings typical
270-320 µs + clouds ≈ 500-565 µs (67-75%) — but rings' sympathetic-string
models measured **511 µs ALONE** (standalone T5a sweep) → 756 µs with
clouds+mod: over budget, hence the chained-rings clamp below. plaits@32k
≈ 268/1.5 ≈ 179 µs (worst sustained, 48k basis) + clouds ≈ 420-470 µs
(56-63%) — blocked on residency, not budget.

### Chain-mode behavior (documented choices)

- **CC routing while chained**: CC 23/24/65/66 → the B slot (clouds mode
  clamp / quality / freeze / trig — 65/66 are unused by every chain-A map);
  every other CC → the A slot. CC 23 is dual-mapped in every module's map
  (elements geometry / plaits engine / rings chord), so while chained it
  MEANS clouds mode and the A slot loses CC 23/24 (the mod engine can still
  drive A — destinations bind to A).
- **Clouds-B mode clamp**: chain mode is granular/looping ONLY. CC 23 = 0
  (granular) / 2 (looping) accepted; **1 (stretch) and 3 (spectral)
  REJECTED** with a console line, no crash (belt-and-braces: the adapter's
  apply-time clamp lands anything illegal on granular). Stretch stays
  available in single-module mode; spectral is excluded everywhere (T2).
- **Chained-rings budget clamp (T6b)**: while rings is the chain A slot, the
  CCs that select combinations the standalone sweep measured over the
  pair's headroom are REJECTED with a console line (single-module rings is
  fully unclamped):
  - **CC 18 = 1 or 4** (RESONATOR_MODEL_SYMPATHETIC_STRING /
    SYMPATHETIC_STRING_QUANTIZED) in modes 0-3: the sweep (standalone T5a,
    19 combos, all under the 750 µs single budget) measured model 1 at
    **511 µs alone** — 511 + clouds-B ~195 + mod ≤50 = 756 > 750. Model 4
    runs the same string bank (the sweep's string-family range 461-511 tops
    out there). Everything else keeps ≥ 44 µs of pair headroom (string
    floor 461 + 195 + 50 = 706 ≤ 750; modal 279; MiniElements modes 271-274;
    easter egg 300-320 across fx 1-5 — CC 18 is NOT clamped in mode 4, where
    it selects the FxType).
  - **CC 20 → modes 0-3 while a sympathetic model byte (1/4) is applied**:
    rejected — in mode 4 the same byte is an FxType index, so entering a
    Part mode with fx 1/4 set would silently become a sympathetic model.
    Change CC 18 first (the reject line says so).
  - **Polyphony (CC 19) is NOT clamped**: the sweep's poly-4 rows never
    exceeded its poly-1 sympathetic worst (511 µs), so no measured
    combination puts poly > 2 over the pair budget beyond what the model
    clamp already removes.
  Elements A needs no clamp (193-207 µs benched); plaits cannot be chain A
  in this build (window).
- **Routing scale**: the route passes A's floats at **gain 1.0** — the
  adapters' outputs are the same ±1.0 class the I2S write path Clip16s, and
  the standalone test exciter produced 0.6-0.8 peaks, so routed material
  lands at the level the granulator was validated with. No fixed gain
  stage.
- **Exciter bypass**: while chained, clouds' test exciter is bypassed
  entirely — the routed frames ARE the exciter input (`parameters.gate`
  stays false; its only consumer is the excluded spectral path).
- **Chained clouds buffer split** (documented deviation): the buffer arena
  is exactly full for the elements/rings pairs, so chained clouds runs
  large 98,304 B (32,768 B sample memory ch0 + 65,536 B FX workspace — the
  FX depth is IDENTICAL to standalone) + small 32,768 B (ch1) =
  **0.512 s/channel sample memory vs 1.024 s standalone**. Grain parameters
  are unaffected; maximum loop length halves.
- **Mod engine in chain mode**: destinations bind to the A slot (the sound
  source; `g_active_module` IS A while chained, and the engine's per-block
  pass runs the chain grid via the additive `g_mod_chain_*` overrides).
  B-destination binding is future work, same as B grain-parameter CC
  routing (B stays at factory defaults while chained).
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
elements 496, clouds 1000, plaits 500 (48 kHz), rings 750 µs; while chained
the module field reads `mod=<a>><b>` (e.g. `elements>clouds`) and the
budget is the chain grid's 750 µs with `proc_us_max` = A+route+B. Swap
lines: `rack: switched to <module> in <ms> ms (window <n> bytes)`; chain
lines: `rack: chain <a>-><b> engaged (<ms> ms)` / `rack: chain off (<ms>
ms)`. `telemetry off`
mutes ONLY this line (cadence + counter resets continue; `status` prints a
one-liner instead). T4 DIN counters deliberately live in `din`/`status`, not
here — the format above is byte-compatible with the T1/T2/T3 bench history.
