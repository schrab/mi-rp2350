# UI-MCU link protocol (ESP32-C3 <-> RP2350) — T5 stage 1

This is the **byte-exact contract** between the two firmware ends:

- RP2350: `rack-rp2350/src/ui_link.h` (header-only, included by `main.cpp`)
- C3: `rack-uimcu/main/RackLink.hpp` / `RackLink.cpp` (seeded from
  ctag-tbd/uimcu; mirrors its LinkClient task/ring structure with the rack
  framing instead of STX/ETX-JSON)

Both files duplicate the constants below under a "MUST MATCH" header. Any
change lands in **all three places** (two firmwares + this doc).

Stage scope: stage 1 = framed link bring-up (codec, self-tests, TELEM
snapshot/delta, bench console commands). Stages 2-6 (display backend,
HC4067 pots, encoder, gestures, BLE-MIDI) consume the sending side
(`RackLink::SendMidi/SendSwitch/SendModSet/SendPageRequest`) and the TELEM
renderer. Stage 2-6 extensions (2026-11-08, all additive): TELEM cells are
4 bytes (a `color` class byte was added) and full snapshots carry a 4-byte
status tail; TELEM flags gained the routed-source id (bits 3-6); MOD-A/B/C
and SYS pages report real cells; UI_CMD action 3 (sys toggle) is wired
(ids 0/1/2) and action 4 (ASSIGN_CYCLE) is new.

## 1. Frame format

```
[0x7E][len:u8][type:u8][payload... len bytes][crc8]
```

| Field  | Meaning |
|--------|---------|
| `0x7E` | frame mark (start-of-frame) |
| `len`  | payload length 0-255 (does NOT count type/crc/mark) |
| `type` | frame type byte (table below) |
| `payload` | exactly `len` bytes, layout per type |
| `crc8` | CRC-8/ATM over `[type, then payload bytes]` (len+1 bytes) |

**CRC-8/ATM**: poly `0x07`, init `0x00`, MSB-first, no reflection, no final
xorout.

```
crc = 0x00
for byte in [type, payload...]:
    crc ^= byte
    for i in 1..8:
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1)
```

**No byte stuffing** — `0x7E` inside a payload is legal because `len` is
known up front. The receiver never hunts inside a payload.

**Parser** (identical state machine both ends): hunt `0x7E` -> read `len` ->
read type+payload (`len+1` bytes) -> read crc -> verify. On CRC failure the
frame is dropped and the parser resyncs at the **next** `0x7E` (counter +
one log line per failure, both ends). On a CRC pass, `rx_frames` increments
and the frame dispatches.

Max frame = 259 bytes (`0x7E` + len + type + 255 payload + crc).

## 2. Frame types

| Type | Name | Direction | Payload |
|------|------|-----------|---------|
| `0x01` | MIDI | C3 -> RP2350 | 4-byte USB-MIDI event packet `[CIN<<4|cable, status, d1, d2]` — funneled into `midiHandleEventPacket()` (the SAME entry point the USB device/host/DIN paths use; zero new dispatch semantics). RP2350 rejects wrong lengths with a log line. |
| `0x02` | SWITCH | C3 -> RP2350 | `[module_id:u8]` (0 elements / 1 clouds / 2 plaits / 3 rings) — routed through the console `switch` path verbatim (acks, chain-slot semantics, self-swap guard). |
| `0x03` | MOD_SET | C3 -> RP2350 | `[src:u8][param:u8][value:u8]` — applied through the mod engine's bench-mux setter (same validation + 0-127 quantization as the console `mod` command / CC 33-35 mux); one ack log line per set. Param-id table: §3. |
| `0x04` | UI_CMD | C3 -> RP2350 | `[action:u8][arg:u8][arg2:u8]`: action 1 = page request (arg = page id, §4) -> RP2350 replies a full TELEM snapshot; action 2 = echo (arg/arg2 = 16-bit seq) -> RP2350 echoes the frame back verbatim and records its turnaround (the C3 measures the full wire RTT of its own probes); action 3 = sys toggle (arg = sys item id, semantics = TOGGLE, arg2 reserved 0): id 0 = telemetry-deltas gate on/off, id 1 = RP2350-pushed full-snapshot gate on/off, id 2 = global clock source internal<->midi; other ids are log-only. action 4 = **ASSIGN_CYCLE** `[cell:u8 0-15][dir:u8 1 up / 0 down]` — the RP2350 resolves `cell` against its ACTIVE page: that cell's CC's currently-routed mod source (or none) cycles through `[off, src0..src11]`, applied via `mod_set_dest` (the RP2350 owns the routing table + validation — "the C3 never enforces"; an invalid target for this module is inert and shows as TELEM flags bit2), and the RP2350 replies a full TELEM snapshot so the C3 renders the result. RATIONALE: the gesture sends ASSIGN_CYCLE, not MOD_SET (the plan's original sketch) — MOD_SET `[src][param][value]` would require the C3 to track which source drives each cell off-protocol; only the RP2350 knows. MOD_SET stays for the MOD pages' direct param writes. action 5 = **SET_GLOBAL** `[item:u8][value:u8 0-127]` — global state the C3 pots address but MOD_SET (per-source) cannot express: item 0 = BPM (`bpm = 20 + value*280/127`, the `bpm` console cmd's quantization family); other items log-only. The RP2350 acks with a full TELEM snapshot when the affected cell is on the active page (MOD-C BPM / SYS). |
| `0x05` | CHAIN_ROUTE | C3 -> RP2350 | `[a:u8][b:u8][engage:u8]`: engage 1 -> console `chain <a> <b>` path (pair validation, window math, acks); engage 0 -> `chain off`. |
| `0x10` | TELEM | RP2350 -> C3 | telemetry, §5. |
| `0x11` | PRESET_REQ | C3 -> RP2350 | `[slot:u8]` — "send me your current state part" (the C3 appends its own part and writes its flash bank). `slot` 0-15 = the bank slot the user is saving into (echoed in the PART reply for context); `0xFF` = the pull side of an autosave request. The RP2350 replies with a PRESET_PART burst. |
| `0x12` | PRESET_PART | RP2350 -> C3 | `[slot:u8][seq:u8][total:u8][data <=252 B]` — chunks of the RP2350's serialized state blob (magic "RKPR", version byte, ~871 B, crc16-CCITT; layout in `rack-rp2350/src/preset.h`). Sent on PRESET_REQ, or **unsolicited with `slot=0xFF` from the RP2350's autosave debounce** (3 s after the last persistent state change; the C3 writes its rotating autosave sectors). The C3 treats the blob as opaque. |
| `0x13` | PRESET_APPLY | C3 -> RP2350 | `[seq:u8][total:u8][data <=252 B]` — chunked blob (read from a bank slot; chunk cap MUST MATCH the RP2350's `kPresetChunkData`). The RP2350 reassembles, validates magic/version/crc, applies (clock bus + mod engine -> CC cache + seen -> module swap via the pending machinery -> chain via the pending machinery) and acks with a full TELEM snapshot. Rejects log a reason and never touch state. |

Unknown types: logged + counted, never crash (both ends tolerate a peer
newer/older than themselves).

## 3. MOD_SET param-id table

`src` = mod-engine source id 0-9 (the same ids the console `mod` command
and CC 33 use):

| src | source | params (id: name) |
|-----|--------|-------------------|
| 0-3 | LFO 1-4 | 0 mode (0-63 Hz-mode / 64+ clocked), 1 rate (0.02-20 Hz exp), 2 wave (0 sine / 1 tri / 2 saw / 3 square / 4 S&H), 3 phase (0-127 -> 0-360 deg), 4 level, 5 clk (global/internal/midi), 6 div (10 divisions), 7 dest (CC 0-127, 0 = off) |
| 4-5 | EU 1-2 | 0 steps (1-32), 1 pulses (0-32), 2 rot, 3 clk, 4 div, 5 dest |
| 6-7 | GATE 1-2 | 0 mode (0 GEN, 1-16 presets ALT8..RISE, 17 TMR, 18 MRKV), 1 dens (1-16), 2 sync (0-100%), 3 glen (1-100%), 4 clk (global/internal/midi), 5 div (10 divisions), 6 dest (CC 0-127, 0 = off) |
| 8-9 | ADSR 1-2 | 0 mode (gate/retrig), 1 atk, 2 dec, 3 sus, 4 rel, 5 clk, 6 div, 7 dest |

- `value` is always 0-127 and quantized exactly like CC 35 (the bench mux's
  `mod_apply_cc_value`): rates/sus/glen exponential/linear mappings, steps =
  `1 + v*31/127`, pulses = `v*32/127`, rot = `v*31/127`, wave/div = scaled
  indexes, clk thresholds `<43 / <85`.
- The `dest` param goes through the same one-shot validation as the console
  (`mod_set_dest`): destination CCs are checked against the ACTIVE module's
  mod-destination table; a continuous source (LFO/ADSR) on a non-continuous
  CC — or a trigger source (EU/GATE) on a non-trigger CC — is INERT and
  logged (it also shows in TELEM flags bit2, §5).
- Param names live in `mod_engine.h` (`kModParamNames`); the RP2350 rejects
  out-of-range src/param with a log line.

## 4. Page ids

| id | page | contents |
|----|------|----------|
| 0 | P1 | the active module's primary 16 pot cells (fully implemented; tables below) |
| 1 | P2 | elements: CC 28/29/64 (MODEL / OMINOUS / PLAY, count = 3); clouds/plaits/rings: count = 0 (no duplicate params) |
| 2 | MOD-LFO 1/2 | LFO 1 (Row 0 pots 0-5) & LFO 2 (Row 2 pots 10-15): 6 params each (RATE, WAVE, PHASE, LEV, MODE, DIV) |
| 3 | MOD-LFO 3/4 | LFO 3 (Row 0 pots 0-5) & LFO 4 (Row 2 pots 10-15): 6 params each (RATE, WAVE, PHASE, LEV, MODE, DIV) |
| 4 | MOD-TRIG | 2x Euclidean: EUC 1 (Row 0 pots 0-4) & EUC 2 (Row 2 pots 10-14): 5 params each (STPS, PULS, ROT, CLK, DIV) |
| 5 | MOD-GATE | 2x Gate: GATE 1 (Row 0 pots 0-4) & GATE 2 (Row 2 pots 10-14): 5 params each (LEN, MASK, GLEN, CLK, DIV) |
| 6 | MOD-ENV | 2x ADSR: ADSR 1 (Row 0 pots 0-5) & ADSR 2 (Row 2 pots 10-15): 6 params each (ATK, DEC, SUS, REL, MODE, DIV) |
| 7 | SYSTEM | 4 module cells, chain pair + engage (Row 0 pots 0-5); telem-d, telem-f, preset (Row 1 pots 6-8); BPM (20-300) & CLKSRC (int/midi) (Row 2 pots 10-11) |

A page request (`UI_CMD` action 1) sets the RP2350's "active page" — the
target of the throttled TELEM deltas (§5) — and replies with a full snapshot.

## 5. TELEM (RP2350 -> C3)

Payload (v2, stage 2-6):

```
[page:u8][module:u8][mode:u8][count:u8]
[count x (cc:u8, value:u8, flags:u8, color:u8)]
[count x NUL-terminated label strings, in cell order]
[mode 0 only: bpm:u8][clocksrc:u8][chain:u8][link:u8]   (status tail)
```

- `page` = page id (§4), `module` = the RP2350's active module id (0-3).
- `mode`: 0 = full snapshot (all cells of the page + status tail), 1 =
  single-cell delta — the **same cell/label format** with `count = 1` and
  NO status tail (the C3 keeps its last tail). The delta's `cc` names the
  cell (a real CC on P1/P2, `200+index` on MOD/SYS pages); the C3 patches
  the matching cached cell and ignores deltas for pages it does not hold.
- `color` = the registry color class (0 white / 1 red / 2 teal, §6) — the
  C3 does NOT duplicate the registry; the RP2350 is the single source of
  truth for labels AND colors.
- `value` = the module's cached CC value (0-127) from the per-module
  per-CC observation cache (4 modules x 128 CCs, updated at the single
  control_change dispatch point). Cache starts zeroed — a cell shows its
  panel value once any controller (web panel / BLE / UI pots) has sent that
  CC since boot. On MOD/SYS pages `value` = the mod engine's / rack state's
  current setting re-expressed 0-127 (the inverse of the MOD_SET
  quantization; a MOD_SET round-trip of a displayed value is stable to
  within the quantizer's floor — imperceptible).
- `cc` on MOD-A/B/C and SYS pages = an **opaque cell id** `200 + cell
  index` — these cells are not MIDI CCs. The C3 renders and addresses by
  CELL INDEX (its pot position); the cc byte is informational.
- `flags` bits:
  - bit0 (`0x01`) **mod-routed** — at least one mod-engine source has this
    CC as its destination (the audio value is overridden by the source; the
    cached panel value still shows).
  - bit1 (`0x02`) **chain-B** — the cell belongs to the chain B slot (set
    when the module is the B slot while a chain is engaged).
  - bit2 (`0x04`) **inert/rejected** — a mod routing exists but is inert on
    this module (continuous/trigger kind mismatch vs the module's
    destination table).
  - bits 3-6 (`0x78`) **routed source id + 1** (0 = unrouted, 1-12 = mod
    source 0-11) — the FIRST source routed to this CC; the mod-assign
    gesture needs to know which source drives the cell. (Multiple sources
    may share a destination; the cycle tracks the first.) bit7 reserved 0.
- Status tail (mode 0 only):
  - `bpm`: internal BPM 20-300 rounded to 1 BPM (low 8 bits: `bpm & 0xFF`);
    **0 = external MIDI clock** (the clocksrc byte disambiguates).
  - `clocksrc`: bit0 = 0 internal / 1 MIDI; bit7 (`0x80`) = BPM >= 256 flag
    (unwraps 256..300 BPM cleanly without ambiguity against low 20..44 BPM).
  - `chain`: 0 off / 1-3 = engaged pair index (1 elements->clouds,
    2 plaits->clouds, 3 rings->clouds).
  - `link`: bit0 = the RP2350's DIN parser has seen bytes since boot,
    bit1 = the RP2350 verified a frame on THIS UART link within the last
    3 s. The C3 status strip renders DIN/UART health from this byte.

Send policy (stages 1 + 2-6):

- Full snapshot (mode 0) on every `UI_CMD` page request.
- Full snapshot (mode 0) pushed by the RP2350 when the active module or the
  chain state changes, gated by the C3's telemetry-full toggle (sys id 1,
  default on) — "the firmware acks drive the UI".
- Throttled delta (mode 1, the changed cell only) after `control_change` on
  the active module, **>= 50 ms spacing**, only when the changed CC is a
  cell of the C3's active page, not a chain-B write, and the telemetry-
  deltas gate (sys id 0, default on) is set.
- Single-cell delta after every applied MOD_SET (the matching cell of the
  C3's active page, if any) — the MOD-pot ack path.
- Max payload: 4 + 16x4 + labels (~130) + 4 = ~200 bytes for elements P1 —
  inside the 255-byte `len`.

## 6. Registry page tables (P1/P2 summary)

Single source of truth: `rack-rp2350/src/ui_link.h` (`kUiPages`). Labels
<= 10 chars (digi-slim 6 px rows on the C3). Color classes: **0 = white**
(standard params), **1 = pink #e6295e group**, **2 = teal #17a3bd group**,
**3 = yellow #f2d21f** (MOD-LFO pages). Modulated parameters render with
**yellow** label, value, and underline bar.
Colors follow `tools/rack_panel.html` and the physical Mutable front panels.

Cell order = physical pot order: top row 1-6, middle row 7-10, bottom row
11-16 (the 6/4/6 HC4067 layout the screen mirrors).

### elements — P1 (16 cells, physical front-panel order)

| cell | cc | label | color | description |
|------|----|-------|-------|-------------|
| 1 | 15 | BOW | white | Bow exciter level |
| 2 | 17 | BLOW | pink | Blow exciter level |
| 3 | 20 | STRIKE | teal | Strike exciter level |
| 4 | 1 | PITCH | white | Pitch offset (-24..+24 st, mod-capable) |
| 5 | 11 | STRENGTH | white | Resonator strength |
| 6 | 14 | CONTOUR | white | Envelope contour |
| 7 | 18 | FLOW | pink | Blow flow |
| 8 | 21 | MALLET | teal | Strike mallet |
| 9 | 23 | GEOMETRY | white | Resonator geometry |
| 10 | 24 | BRIGHTNESS | white | Resonator brightness |
| 11 | 16 | TIMBRE | white | Bow timbre |
| 12 | 19 | TIMBRE | pink | Blow timbre |
| 13 | 22 | TIMBRE | teal | Strike timbre |
| 14 | 25 | DAMPING | white | Resonator damping |
| 15 | 26 | POSITION | white | Resonator position |
| 16 | 27 | SPACE | white | Space / reverb |

elements P2 (3 cells): CC 28 MODEL (white), CC 29 OMINOUS (white), CC 64 PLAY (white, trigger exciter).

### clouds — P1 (16 cells)

| cell | cc | label | color |
|------|----|-------|-------|
| 1 | 1 | EX TONE | white |
| 2 | 11 | EX LEVEL | white |
| 3 | 13 | EX SRC | white |
| 4 | 40 | POSITION | pink |
| 5 | 41 | SIZE | teal |
| 6 | 42 | PITCH | white |
| 7 | 43 | DENSITY | pink |
| 8 | 44 | TEXTURE | teal |
| 9 | 45 | DRY/WET | white |
| 10 | 46 | SPREAD | white |
| 11 | 47 | FEEDBACK | white |
| 12 | 48 | REVERB | white |
| 13 | 23 | MODE | white |
| 14 | 24 | QUALITY | white |
| 15 | 65 | FREEZE | white |
| 16 | 66 | TRIG | white |

Grain params travel the **alias bank CC 40-48** (`kCloudsAliasCcBase`) —
the convention the rack web panel established (works single-module AND
chained, where native 14-22 belong to the A slot). P2 count = 0 (no duplicate page).

### plaits — P1 (13 cells; 3 pots empty, all white)

| cell | cc | label | color |
|------|----|-------|-------|
| 1 | 14 | TIMBRE | white |
| 2 | 15 | MORPH | white |
| 3 | 16 | HARMONICS | white |
| 4 | 17 | MACRO | white |
| 5 | 18 | LPG | white |
| 6 | 19 | DECAY | white |
| 7 | 20 | FM AMT | white |
| 8 | 21 | TIM MOD | white |
| 9 | 22 | MORPH MOD | white |
| 10 | 25 | LEVEL | white |
| 11 | 23 | ENGINE | white |
| 12 | 24 | AUX | white |
| 13 | 64 | TRIG | white (momentary) |

P2 count = 0 (no duplicate page).

### rings — P1 (11 cells; 5 pots empty, all white)

| cell | cc | label |
|------|----|-------|
| 1 | 14 | STRUCTURE |
| 2 | 15 | BRIGHTNESS |
| 3 | 16 | DAMPING |
| 4 | 17 | POSITION |
| 5 | 18 | MODEL/FX |
| 6 | 19 | POLY |
| 7 | 20 | MODE |
| 8 | 21 | WAVE |
| 9 | 22 | CHORD TB |
| 10 | 23 | CHORD |
| 11 | 64 | STRUM (momentary) |

P2 = elements has 2 unique parameters (MODEL CC 28, OMINOUS CC 29); clouds, plaits, rings have count = 0 (no duplicate parameters). Page navigation automatically skips P2 when the active module has no secondary parameter page (P1 navigates directly to/from MOD-LFO 1/2).

### Dedicated Modulation + SYS — aux cell tables

Cell order = physical pot order; the TELEM `cc` field = `200 + cell index`
(opaque cell id — not a MIDI CC). The C3's MOD-page pot map sends
`MOD_SET [src][param][value]` with EXACTLY the (src, param) pairs below —
MUST MATCH `kUiMod*Cells` in `ui_link.h` and `kModPotMap` in the C3's
`ui/RackUI`. Color classes: yellow = continuous modulators (LFO), teal =
trigger/sequenced sources (EUclid/gate/ADSR), white = global clock & sys.
Values on these pages are the mod engine's current settings re-expressed
0-127 (inverse of the MOD_SET quantization: rate/ADSR log domains mapped
back, wave/div/enum = `ceil(index*128/counts)`, gates = mode index or density,
BPM = `(bpm-20)*127/280`, clock src = 0/127). The C3 renderer displays
clear text names rather than raw numbers wherever discrete parameters occur:
- **LFO Wave**: `SINE`, `TRI`, `SAW`, `SQR`, `S&H`
- **LFO Mode**: `HZ`, `CLK`
- **Divisions**: `1/1`, `1/2`, `1/4`, `1/8`, `1/16`, `1/32`, `1/4T`, `1/8T`, `16T`, `32T`
- **Clock Source**: `GLB`, `INT`, `MIDI`
- **Gate Mode**: `GEN`, `ALT8`..`RISE`, `TMR`, `MRKV` (19 modes)
- **ADSR Mode**: `GATE`, `RTRG`
- **SYS Toggles / Pair**: `E>C`/`P>C`/`R>C`, `ON`/`OFF`, `INT`/`MIDI`

MOD-LFO 1/2 — Row 0 = LFO 1 (pots 0-5), Row 2 = LFO 2 (pots 10-15):

| row | pot | cell | src | param | label | color |
|-----|-----|------|-----|-------|-------|-------|
| 0 | 0-5 | 0-5 | 0 (lfo1) | 1 rate / 2 wave / 3 phase / 4 level / 0 mode / 6 div | L1 RATE/WAVE/PHASE/LEV/MODE/DIV | yellow |
| 1 | 6-9 | 6-9 | - | - | (empty) | - |
| 2 | 10-15 | 10-15 | 1 (lfo2) | same | L2 RATE/WAVE/PHASE/LEV/MODE/DIV | yellow |

MOD-LFO 3/4 — Row 0 = LFO 3 (pots 0-5), Row 2 = LFO 4 (pots 10-15):

| row | pot | cell | src | param | label | color |
|-----|-----|------|-----|-------|-------|-------|
| 0 | 0-5 | 0-5 | 2 (lfo3) | 1 rate / 2 wave / 3 phase / 4 level / 0 mode / 6 div | L3 RATE/WAVE/PHASE/LEV/MODE/DIV | yellow |
| 1 | 6-9 | 6-9 | - | - | (empty) | - |
| 2 | 10-15 | 10-15 | 3 (lfo4) | same | L4 RATE/WAVE/PHASE/LEV/MODE/DIV | yellow |

MOD-TRIG — 2x Euclidean: Row 0 = EUC 1 (pots 0-4), Row 2 = EUC 2 (pots 10-14):

| row | pot | cell | src | param | label | color |
|-----|-----|------|-----|-------|-------|-------|
| 0 | 0-4 | 0-4 | 4 (eu1) | 0 steps / 1 pulses / 2 rot / 3 clk / 4 div | E1 STPS/PULS/ROT/CLK/DIV | teal |
| 1 | 6-9 | 6-9 | - | - | (empty) | - |
| 2 | 10-14 | 10-14 | 5 (eu2) | same | E2 STPS/PULS/ROT/CLK/DIV | teal |

MOD-GATE — 2x Gate: Row 0 = GATE 1 (pots 0-5), Row 2 = GATE 2 (pots 10-15):

| row | pot | cell | src | param | label | color |
|-----|-----|------|-----|-------|-------|-------|
| 0 | 0-5 | 0-5 | 6 (gate1) | 0 mode / 1 dens / 2 sync / 3 glen / 4 clk / 5 div | G1 MODE/DENS/SYNC/GLEN/CLK/DIV | teal |
| 1 | 6-9 | 6-9 | - | - | (empty) | - |
| 2 | 10-15 | 10-15 | 7 (gate2) | same | G2 MODE/DENS/SYNC/GLEN/CLK/DIV | teal |

MOD-ENV — 2x ADSR: Row 0 = ADSR 1 (pots 0-5), Row 2 = ADSR 2 (pots 10-15):

| row | pot | cell | src | param | label | color |
|-----|-----|------|-----|-------|-------|-------|
| 0 | 0-5 | 0-5 | 8 (adsr1) | 1 atk / 2 dec / 3 sus / 4 rel / 0 mode / 6 div | A1 ATK/DEC/SUS/REL/MODE/DIV | teal |
| 1 | 6-9 | 6-9 | - | - | (empty) | - |
| 2 | 10-15 | 10-15 | 9 (adsr2) | same | A2 ATK/DEC/SUS/REL/MODE/DIV | teal |

SYS — 16 cells (Row 0 modules & chain, Row 1 BPM, CLKSRC & preset, Row 2 telemetry gates):

| row | pot | cell | value | label |
|-----|-----|------|-------|-------|
| 0 | 0-3 | 0-3 | 0 (the active module = the TELEM header byte) | ELEMENTS/CLOUDS/PLAITS/RINGS |
| 0 | 4 | 4 | configured pair index 1-3 | CHAIN PR |
| 0 | 5 | 5 | 0 off / 127 engaged | CHAIN ON |
| 1 | 6 | 6 | 20-300 master tempo (UI_CMD action 5 BPM) | BPM |
| 1 | 7 | 7 | 0 internal / 127 midi (UI_CMD action 3 sys id 2) | CLKSRC |
| 1 | 8 | 8 | preset slot 0-15 (C3-local; SAVE/LOAD are ring actions, §9) | PRESET |
| 2 | 14 | 14 | 0/127 telemetry-deltas gate (sys id 0) | TELEM D |
| 2 | 15 | 15 | 0/127 pushed-full-snapshot gate (sys id 1) | TELEM F |

SYS short-push ring (one action per push, marked with yellow underline;
7 items since T4a — see §9): chain-pair cycle -> chain engage toggle ->
preset slot cycle (0-15, pot 8 also selects) -> preset SAVE -> preset
LOAD -> telem-deltas toggle -> telem-full toggle -> wrap.

## 7. Pin map + UART setup

| Signal | ESP32-C3 (UI MCU) | RP2350 (rack) |
|--------|-------------------|---------------|
| C3 TX -> RP2350 RX | GPIO4 (UART0 TX) | **GP1** (UART0/Serial1 RX) |
| RP2350 TX -> C3 RX | GPIO5 (UART0 RX) | **GP0** (UART0/Serial1 TX) |
| GND | GND | GND |

- Both 3V3 logic — direct wiring, no level shifting.
- **921600 8N1**, no flow control.
- RP2350 pad validity (earlephilhower core's RP2350A tables): UART0 TX
  valid = {0, 2, 12, 14, 16, 18, 28}, RX valid = {1, 3, 13, 15, 17, 19,
  29}. HISTORY: the first pick was GP16/GP17 (after correcting a TX/RX
  pad swap in the original brief), but the user's RP2350 nano board only
  POPULATES GPIO 0-5 + 26-29 — the link moved to **TX=GP0, RX=GP1**
  (2026-11-08), the clean adjacent pair from the populated set (GP2/3
  also valid; GP28/29 kept free as the only ADC-capable pins). GP0/1 are
  unused by the rack (4/5 DIN, 6/7/8 I2S, 14/15 host, 22 LED, 24/25 USB).
- C3 console: USB-Serial-JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`), so
  UART0 is free for the link (GPIO20/21 stay unused).
- No hardware loopback exists on the RP2350's PL011-style UART (no LBM bit
  in UARTLCR_H — checked the pico-sdk register headers); the ESP32-C3 UART
  has `uart_set_loop_back()` and its self-test uses it.

## 8. Bench procedures

### 8.1 RP2350 self-test (no wiring needed)

Flash any rack env (or use the running board) and on the serial console:

```
uitest
```

Expected: six PASS lines (MIDI round-trip, MOD_SET round-trip,
0x7E-in-payload, max-len 255, corrupted-CRC rejection, post-rejection
resync), then `uitest: codec OK (... n/a on RP2350 UART (no LBM bit) ...)`.
The corrupt-CRC case bumps `crc_err` by one by design. There is NO hardware
loopback leg on the RP2350 (the silicon has none) — the wire check is the
C3's loopback plus the cross-board bench.

### 8.2 C3 self-test (no wiring needed)

Flash `rack-uimcu` (`idf.py build && idf.py -p PORT flash`) and watch the
USB-JTAG console at boot. Expected once:

```
selftest: codec OK | hw-loop 32/32 | rtt ~NN us
```

`hw-loop` runs the ESP32 UART's internal TX->RX loopback
(`uart_set_loop_back`): 32 frames including one max-len (255-byte) frame;
`rtt` is the average turnaround over the 31 small echo frames.

### 8.3 Cross-board check (wire 3 conductors)

Wire per §7 (C3 GPIO4 -> RP2350 GP1, C3 GPIO5 <- RP2350 GP0, GND-GND).
Power both. Expected:

- C3 console: `rack link on UART0 GPIO4/5 @921600`, then
  `probe: P1 page request sent (link not confirmed)` every 5 s, then
  `rack link online` + `page request RTT ~NN us` + a `telem page=0 mod=M
  mode=0 count=N` line with the first cell values once the RP2350 replies.
- RP2350 console (`ui`): `tx`/`rx` frame counts advancing, `crc_err` steady
  at 0, `rtt` = its echo turnaround. Sending MIDI/CC from any controller
  updates `telem` deltas on the wire (visible as `rx` on the C3).
- Console round trips: `uisend 0` (RP2350 -> C3 one-shot snapshot), and on
  the C3 the probe stream doubles as the C3 -> RP2350 -> C3 exercise.
- Bench helpers on the RP2350: `ui` (link status), `uitest` (codec),
  `uisend [page]` (one full snapshot + sizes), `switch`/`chain`/`mod`
  (the same paths the UI frames take).

### 8.4 What NOT to expect in stage 1

Historical (stage 1): no screen/pots/encoder/BLE; MOD-A/B/C + SYS replied
count=0; the C3 only received. Stages 2-6 (2026-11-08) replace these: the
C3 firmware now renders the ST7789P3 bar, scans the HC4067 pots, runs the
encoder gestures, and advertises BLE-MIDI (a PERIPHERAL — a central
connects to it). The seed's ctag LinkClient/DspProxy machinery is excluded
from the build (headers carry "seed reference, not built" notes).

### 8.5 C3 bring-up, stages 2-6 (rack-uimcu)

Screen/pots/encoder/BLE bench procedures live in `rack-uimcu/README.md`
(GPIO allocation table, wiring, gesture timings, `testpat`/`calib`/`pots`/
`heap` console commands, the per-stage wiring checklist). The C3 console
is the USB-Serial-JTAG wire (line-based commands; ESP_LOG shares it). The
RP2350-side bench helpers are unchanged: `ui` (counters), `uitest`
(codec), `uisend [page]` (one snapshot), plus the same paths the new
frames take (`switch`/`chain`/`mod`).

## 9. Presets (T4a, benched 2026-11-11)

One preset = full rack state, split by ownership (the plan's decision):
the **RP2350 owns serialization + apply** (it holds the state), the **C3
owns the flash bank** (its own 4 MB flash — RP2350 flash writes would
stall XIP audio). The RP2350 blob (`rack-rp2350/src/preset.h`, 871 B,
versioned + crc16-CCITT) = active module, chain pair/engage, clock bus
(global source + BPM), the mod engine's 10 persistent source records,
and all four modules' observed CC values + a seen-bitmap (which CCs a
controller ever sent — cache zeros are ambiguous). The C3 part (12 B,
`PresetStore`) = version, last page, module mirror, slot.

Flows:

- **SAVE** (SYS ring item 3 / C3 console `preset save <slot>`):
  `PRESET_REQ[slot]` -> PRESET_PART burst -> `PresetBank::SaveSlot`
  (raw `esp_partition`, 16 slots x 1 sector). RP2350 audio untouched.
- **LOAD** (SYS ring item 4 / C3 console `preset load <slot>`):
  `LoadSlot` -> apply the C3 part locally (page) -> chunked
  `PRESET_APPLY` -> the RP2350 validates + applies via its pending
  machinery (module swap first if the preset says so; every factory
  re-Init replays the CC cache — `preset_replay_module`) -> full TELEM
  ack. Same-module loads touch no audio; a module/chain change mutes for
  the usual 20-44 ms swap window.
- **AUTOSAVE**: the RP2350 marks its state dirty on every persistent
  change (CC, mod param/dest, switch, chain, bpm/clock) and pushes a
  PRESET_PART burst with `slot=0xFF` once the state has settled (3 s);
  the C3 writes 4 rotating autosave sectors (highest valid seq wins).
- **BOOT RESTORE**: once the link first comes online after a power
  cycle, the C3 reads the autosave and replays it through the LOAD path
  — power-cycle restores exactly where you were. `PresetBank::ClearAll`
  (C3 console `preset clear`) wipes slots + autosave.

Bench (verified on hardware 2026-11-11):

- RP2350 console: `preset` (status), `preset dump` (the blob as
  `PRxx:<hex>` rows — 24 B per row except the last, 37 rows),
  `preset collect` + rows (reassemble), `preset load` (validate +
  apply; a full buffer applies directly), `preset apply`/`reload`
  (re-apply the collected blob). Corrupt crc / wrong magic / wrong
  version are REJECTED with a reason; state is never touched.
- C3 console: `preset` (bank status), `preset save/load <0-15>`,
  `preset clear`.
- SYS page: pot 8 = slot select 0-15; the short-push ring is
  pair -> engage -> slot-cycle -> **SAVE** -> **LOAD** -> telem-d ->
  telem-f (7 items; the strip shows `SYSTEM SAVE>` / `SYSTEM LOAD>`
  while armed).
