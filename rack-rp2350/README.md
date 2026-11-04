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

## SRAM architecture (T2)

- **One window for all four modules**: `memmap_rack.ld` (single script, every
  env) places ALL FOUR `.tc_<module>.*` hot sets into ONE ld OVERLAY — same
  VMA = the 112 KiB SRAM window at 0x20064000 (top of main SRAM), sequential
  LMAs in the reserved flash region RACK_LMA (origin 0x10389000). Measured
  members (rack ELF, 2026-11-04): elements 36,752 B, clouds 79,744 B, plaits
  108,820 B, rings 58,084 B — all ≤ 0x1C000 (114,688 B; ~5.9 KB promotion
  slack). `rack_window_copy(module_id)` copies ANY member into the window
  (boot or swap); the copied image is the only live one.
- **Arenas** (`src/rack_arena.h`): object arena 128 KiB + buffer arena
  192 KiB, both `alignas(8)`, .bss-resident, used as UNIONS — the active
  module's placement/carve is the live one: elements Part 113,488 B + 64 KiB
  reverb; clouds processor 8,104 B + 128 KiB large + 64 KiB small (fills the
  buffer arena exactly); plaits Voice 21,748 B + 32 KiB allocator workspace;
  rings Part 123,352 B + StringSynthPart 2,004 B (the object arena's sizing
  module) + 64 KiB reverb. Swaps re-place + re-Init factory defaults; no
  destructors exist in the four ports' DSP state.
- **RAM per env** (identical map in all four): static .data 19,576 B + .bss
  345,044 B (of which arenas 327,680 B → non-arena static ≈ 37 KB) → heap
  44,708 B below the window; boot heap consumers: core-1 separate stack
  8 KB, I2S DMA bank 2-3 KB, USB device+host buffers → ≥ ~16 KB headroom.
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
- **Clouds spectral excluded** (plan, "SRAM architecture"): the 57 KB of
  CMSIS FFT TABLES (twiddleCoef_4096 + rfft + bitrev) are flash-resident in
  the rack (patch `tools/patches/cmsis_dsp/01-rack-tc-names.patch` demotes
  them; the CMSIS FFT CODE stays in the window). Clouds playback mode 3
  (spectral) is therefore REJECTED by the rack adapter (CC 23 values ≤ 2
  only) — the 13.8 ms transform burst would no longer be bank-absorbable
  from XIP. Granular/stretch/looping are unchanged.

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
elements 496, clouds 1000, plaits 500 (48 kHz), rings 750 µs. Swap lines:
`rack: switched to <module> in <ms> ms (window <n> bytes)`. `telemetry off`
mutes ONLY this line (cadence + counter resets continue; `status` prints a
one-liner instead). T4 DIN counters deliberately live in `din`/`status`, not
here — the format above is byte-compatible with the T1/T2/T3 bench history.
