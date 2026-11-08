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
renderer. MOD pages report empty (count=0) until stage 4; SYS toggles are
log-only until a later stage wires them to real state.

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
| `0x04` | UI_CMD | C3 -> RP2350 | `[action:u8][arg:u8][arg2:u8]`: action 1 = page request (arg = page id, §4) -> RP2350 replies a full TELEM snapshot; action 2 = echo (arg/arg2 = 16-bit seq) -> RP2350 echoes the frame back verbatim and records its turnaround (the C3 measures the full wire RTT of its own probes); action 3 = sys toggle -> **log-only in stage 1**. |
| `0x05` | CHAIN_ROUTE | C3 -> RP2350 | `[a:u8][b:u8][engage:u8]`: engage 1 -> console `chain <a> <b>` path (pair validation, window math, acks); engage 0 -> `chain off`. |
| `0x10` | TELEM | RP2350 -> C3 | telemetry, §5. |
| `0x11` | PRESET_REQ | reserved (T4a) | receive-and-log both ends until T4a implements the preset bank. |
| `0x12` | PRESET_PART | reserved (T4a) | receive-and-log. |
| `0x13` | PRESET_APPLY | reserved (T4a) | receive-and-log. |

Unknown types: logged + counted, never crash (both ends tolerate a peer
newer/older than themselves).

## 3. MOD_SET param-id table

`src` = mod-engine source id 0-11 (the same ids the console `mod` command
and CC 33 use):

| src | source | params (id: name) |
|-----|--------|-------------------|
| 0-3 | LFO 1-4 | 0 mode (0-63 Hz-mode / 64+ clocked), 1 rate (0.02-20 Hz exp), 2 wave (0 sine / 1 tri / 2 saw / 3 square / 4 S&H), 3 phase (0-127 -> 0-360 deg), 4 level, 5 clk (global/internal/midi), 6 div (10 divisions), 7 dest (CC 0-127, 0 = off) |
| 4-7 | EU 1-4 | 0 steps (1-32), 1 pulses (0-32), 2 rot, 3 clk, 4 div, 5 dest |
| 8-9 | GATE 1-2 | 0 len (1-32), 1 mask (density fill 0-127), 2 glen, 3 clk, 4 div, 5 dest |
| 10-11 | ADSR 1-2 | 0 mode (gate/retrig), 1 atk, 2 dec, 3 sus, 4 rel, 5 clk, 6 div, 7 dest |

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

| id | page | stage-1 contents |
|----|------|------------------|
| 0 | P1 | the active module's primary 16 pot cells (fully implemented; tables below) |
| 1 | P2 | elements: CC 28/29 selectors + alt-layout duplicates; clouds/plaits/rings: duplicate of P1 |
| 2 | MOD-A | empty in stage 1 (count=0) — stage 4 fills from the mod engine |
| 3 | MOD-B | empty in stage 1 |
| 4 | MOD-C | empty in stage 1 |
| 5 | SYS | empty in stage 1 |

A page request (`UI_CMD` action 1) sets the RP2350's "active page" — the
target of the throttled TELEM deltas (§5) — and replies with a full snapshot.

## 5. TELEM (RP2350 -> C3)

Payload:

```
[page:u8][module:u8][mode:u8][count:u8]
[count x (cc:u8, value:u8, flags:u8)]
[count x NUL-terminated label strings, in cell order]
```

- `page` = page id (§4), `module` = the RP2350's active module id (0-3).
- `mode`: 0 = full snapshot (all cells of the page), 1 = single-cell delta —
  the **same format** with `count = 1` (the C3 parser is identical for both).
- `value` = the module's cached CC value (0-127) from the per-module
  per-CC observation cache (4 modules x 128 CCs, updated at the single
  control_change dispatch point). Cache starts zeroed — a cell shows its
  panel value once any controller (web panel / BLE / UI pots) has sent that
  CC since boot.
- `flags` bits:
  - bit0 (`0x01`) **mod-routed** — at least one mod-engine source has this
    CC as its destination (the audio value is overridden by the source; the
    cached panel value still shows).
  - bit1 (`0x02`) **chain-B** — the cell belongs to the chain B slot (set
    when the module is the B slot while a chain is engaged).
  - bit2 (`0x04`) **inert/rejected** — a mod routing exists but is inert on
    this module (continuous/trigger kind mismatch vs the module's
    destination table).

Send policy (stage 1):

- Full snapshot (mode 0) on every `UI_CMD` page request.
- Throttled delta (mode 1, the changed cell only) after `control_change` on
  the active module, **>= 50 ms spacing**, only when the changed CC is a
  cell of the C3's active page and not a chain-B write.
- Max payload: 4 + 16x3 + labels (~110) = ~162 bytes for elements P1 —
  inside the 255-byte `len`.

## 6. Registry page tables (P1/P2 summary)

Single source of truth: `rack-rp2350/src/ui_link.h` (`kUiPages`). Labels
<= 10 chars (digi-slim 6 px rows on the C3). Color classes: **0 = white**
(standard params), **1 = red #e6295e group**, **2 = teal #17a3bd group**
(the panels' yellow #f2d21f is a STATE accent, never a param class).
Colors follow `tools/rack_panel.html` (the rack's current recolor;
`tools/elements_panel.html` has the same trio for elements).

Cell order = physical pot order: top row 1-6, middle row 7-10, bottom row
11-16 (the 6/4/6 HC4067 layout the screen mirrors).

### elements — P1 (16 cells)

| cell | cc | label | color |
|------|----|-------|-------|
| 1 | 1 | MOD | red |
| 2 | 11 | STRENGTH | red |
| 3 | 14 | CONTOUR | white |
| 4 | 15 | BOW | white |
| 5 | 16 | TIMBRE B | white |
| 6 | 17 | BLOW | teal |
| 7 | 18 | FLOW | teal |
| 8 | 19 | TIMBRE Bl | teal |
| 9 | 20 | STRIKE | teal |
| 10 | 21 | MALLET | teal |
| 11 | 22 | TIMBRE S | teal |
| 12 | 23 | GEOMETRY | white |
| 13 | 24 | BRIGHTNESS | white |
| 14 | 25 | DAMPING | white |
| 15 | 26 | POSITION | white |
| 16 | 27 | SPACE | white |

elements P2 (16 cells): CC 28 MODEL (white), CC 29 OMINOUS (white), then the
alt-layout duplicates of cells 3-16 above (14-27, same colors; no CC 1/11).

### clouds — P1 (16 cells, all white)

| cell | cc | label |
|------|----|-------|
| 1 | 1 | EX TONE |
| 2 | 11 | EX LEVEL |
| 3 | 13 | EX SRC |
| 4 | 40 | POSITION |
| 5 | 41 | SIZE |
| 6 | 42 | PITCH |
| 7 | 43 | DENSITY |
| 8 | 44 | TEXTURE |
| 9 | 45 | DRY/WET |
| 10 | 46 | SPREAD |
| 11 | 47 | FEEDBACK |
| 12 | 48 | REVERB |
| 13 | 23 | MODE |
| 14 | 24 | QUALITY |
| 15 | 65 | FREEZE |
| 16 | 66 | TRIG |

Grain params travel the **alias bank CC 40-48** (`kCloudsAliasCcBase`) —
the convention the rack web panel established (works single-module AND
chained, where native 14-22 belong to the A slot). P2 = duplicate of P1.

### plaits — P1 (13 cells; 3 pots empty)

| cell | cc | label | color |
|------|----|-------|-------|
| 1 | 14 | TIMBRE | white |
| 2 | 15 | MORPH | white |
| 3 | 16 | HARMONICS | white |
| 4 | 17 | MACRO | red |
| 5 | 18 | LPG | white |
| 6 | 19 | DECAY | white |
| 7 | 20 | FM AMT | teal |
| 8 | 21 | TIM MOD | teal |
| 9 | 22 | MORPH MOD | teal |
| 10 | 25 | LEVEL | white |
| 11 | 23 | ENGINE | white |
| 12 | 24 | AUX | white |
| 13 | 64 | TRIG | white (momentary) |

P2 = duplicate of P1.

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

P2 = duplicate of P1.

## 7. Pin map + UART setup

| Signal | ESP32-C3 (UI MCU) | RP2350 (rack) |
|--------|-------------------|---------------|
| C3 TX -> RP2350 RX | GPIO4 (UART0 TX) | GP17 (UART0/Serial1 RX) |
| RP2350 TX -> C3 RX | GPIO5 (UART0 RX) | GP16 (UART0/Serial1 TX) |
| GND | GND | GND |

- Both 3V3 logic — direct wiring, no level shifting.
- **921600 8N1**, no flow control.
- RP2350 pin validity was verified against the earlephilhower core's
  SerialUART tables (RP2350A branch): UART0 TX valid pins =
  {0, 2, 12, 14, 14, 16, 18, 28}, RX valid = {1, 3, 13, 15, 17, 19, 29}.
  NOTE: the original brief's TX=GP17/RX=GP16 was reversed — **GP16 is the
  UART0 TX pad, GP17 the RX pad**; the wiring above is the corrected pair.
  Both pins are free (used: 4/5 DIN, 6/7/8 I2S, 14/15 PIO-USB host,
  22 LED, 24/25 native USB).
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

Wire per §7 (C3 GPIO4 -> RP2350 GP17, C3 GPIO5 <- RP2350 GP16, GND-GND).
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

- No screen, no pots, no encoder, no BLE (stages 2-6).
- MOD-A/B/C and SYS pages reply count=0.
- The C3 logs MIDI/SWITCH/MOD_SET frames it RECEIVES; it does not yet SEND
  them on its own (the sending functions exist for stages 3-6).
- The seed's ctag LinkClient/DspProxyLink machinery stays compiled but the
  seed link task does NOT start (DspProxy::Init no longer calls
  LinkClient::Start — RackLink owns UART0).
