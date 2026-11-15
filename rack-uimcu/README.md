# rack-uimcu — T5 rack UI controller (ESP32-C3)

Firmware for the rack's UI panel MCU: ST7789P3 284x76 bar display, 16
HC4067-scanned pots, one encoder (+push), BLE-MIDI peripheral, and the
framed UART link to the rack RP2350 (`docs/ui-protocol.md` is the byte-
exact wire contract; the RP2350 twin lives in `rack-rp2350/src/ui_link.h`).

Seeded from ctag-tbd/uimcu (READ-ONLY upstream); the seed's link/menu/net
stack is excluded from the build — `main/CMakeLists.txt` lists what runs
(`RackLink`, `RackPots`, `RackConsole`, `input/UserInput`, `ui/Display`,
`ui/RackUI`, `BtMidiReceiver`). RapidJSON is vendored under
`components/rapidjson` (kept for the T4a preset bank; nothing includes it
yet).

Build/flash:

```sh
# toolchain (export.sh alone does NOT put the riscv toolchain on PATH)
export PATH="$HOME/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20260121/riscv32-esp-elf/bin:$PATH"
. /home/schrab/esp/esp-idf-v5.5.4/export.sh
idf.py build && idf.py -p PORT flash     # console = USB-Serial-JTAG
```

## C3 GPIO allocation (the full table)

| GPIO | Function | Notes |
|------|----------|-------|
| 0 | encoder A | switch to GND: internal pull-up enabled in firmware; 10k external pull-up recommended |
| 1 | encoder B | same as A |
| 2 | HC4067 addr S2 | STRAPPING — push-pull output, floats harmlessly at reset |
| 3 | HC4067 **SIG** (common, Z) + encoder push | ADC1_CH3; encoder push shares this line through a 470R series resistor to GND (detected in `RackPots` scan task) |
| 4 | UART link TX -> RP2350 GP1 | 921600 8N1 (stage 1) |
| 5 | UART link RX <- RP2350 GP0 | 921600 8N1 (stage 1) |
| 6 | HC4067 addr S3 | push-pull output |
| 7 | display SPI RST | active-low reset pulsed by firmware *after* SPI2 bus acquire (locks SCK idle HIGH in SPI Mode 3 before reset releases) |
| 8 | HC4067 addr S0 | STRAPPING — push-pull output (also onboard LED pin on some C3 SuperMini boards) |
| 9 | HC4067 addr S1 | STRAPPING — push-pull output, never held low at power-on |
| 10 | display SPI SCK | 26.67 MHz, SPI Mode 3 (`CPOL=1, CPHA=1`, SCK idles HIGH) |
| 11-17 | flash on this module | DO NOT USE |
| 18/19 | native USB D-/D+ (console) | DO NOT USE |
| 20 | display SPI MOSI | 26.67 MHz |
| 21 | display SPI DC | low = command, high = data |

Pin-saving tricks:

- display CS tied GND — the SPI bus is exclusive to the panel (requires SPI Mode 3 with SCK idling HIGH before RST releases).
- display BL tied 3V3 — always backlit, no PWM dimming.
- encoder push shared with mux SIG (`GPIO3`) via a 470R series resistor to GND.
- mux E (enable) tied GND — always enabled.

## Wiring summary

| From | To |
|------|----|
| C3 GPIO4 | RP2350 GP1 (UART0 RX) |
| C3 GPIO5 | RP2350 GP0 (UART0 TX) |
| C3 GPIO10/20/21/7 | ST7789P3 SCK/MOSI/DC/RST; panel CS->GND, BL->3V3, VCC 3V3, GND |
| C3 GPIO3 | HC4067 SIG (common; datasheet Z) — pots 0-15 on CH0-15; encoder push -> 470R -> GND |
| C3 GPIO8/9/2/6 | HC4067 S0/S1/S2/S3 — push-pull outputs |
| C3 GPIO0/1 | encoder A/B (switch to GND; internal + 10k external pull-ups) |

All 3V3 logic, no level shifting.

## Rack pages (labels/values arrive over TELEM — the RP2350 is the source)

- **Per-module parameter pages**: P1 (id 0) always; P2 (id 1) when the module
  has a second bank (elements, lira8); **P3 (id 10) for lira8 only**. The
  page cycle follows the module: P1 -> [P2] -> [P3] -> [CLOUDS B while
  chained] -> MOD-LFO 1/2 -> MOD-LFO 3/4 -> MOD-TRIG -> MOD-GATE -> MOD-ENV ->
  MOD-ARP -> SYS (wraps).
- **Five modules** (SYS selector and the hold+rotate overlay): ELEMENTS,
  CLOUDS, PLAITS, RINGS, **LIRA8**. The SYS selector is ONE pot over 5 slots
  (module ids 0-4; while chained it selects the 4 chain pairs E>C/P>C/R>C/L>C).
- Switch cells are **live** (no soft takeover): FAST (SLOW/FAST), FM source
  (FM/OFF/LFO), delay mode (SELF/TRI/SQR), FM structure (SPLIT/LOOP),
  TOTAL FB / VIBRATO (ON/OFF) — the first movement sends the selection.
- lira8 pages (rack-lira8 table, pot numbering 1..16): **P1** = group 1234 +
  delay line 1 (row0 FAST 12, FM SRC 12, MOD 1 p3, TIME 1 p4, FAST 34,
  FM SRC 34; row1 TUNE 1-4; row2 SHARP/MOD both pairs + PITCH 1234 + HOLD
  1234); **P2** = group 5678 + delay line 2; **P3** = DEL MODE/FB/MIX/DRIVE/
  DIST MIX/VOL, FREQ A/FREQ B/TOTAL FB/VIBRATO, FM STRUCT/AND/OR/LINK/
  QUANTIZE.

## Gestures (the whole UI: turn pots + one encoder)

| Gesture | Action |
|---------|--------|
| Rotate | page cycle P1 -> [P2] -> [P3 for lira8] -> [CLOUDS B while chained] -> MOD-LFO 1/2 -> ... -> SYS (wraps); the C3 requests the page, dashes render until the TELEM snapshot lands |
| Short push (< 600 ms) on MOD-C | toggle global clock source (UI_CMD sys id 2) |
| Short push on SYS | the SYS action ring, one action per push, next action marked with the yellow underline (7 items): chain-pair cycle -> chain engage toggle -> preset slot cycle (0-15; pot 8 also selects) -> preset SAVE -> preset LOAD -> telemetry-deltas toggle -> telemetry-full toggle -> wrap. The strip shows `SYSTEM SAVE>` / `SYSTEM LOAD>` while those are armed. SAVE/LOAD run the T4a preset bank (docs/ui-protocol.md §9): SAVE pulls the RP2350's state part over the link and writes the slot (audio untouched); LOAD sends the stored blob back as PRESET_APPLY (the RP2350 swaps module/params/mod/clock via its pending machinery and acks a full TELEM) |
| Push-hold >= 600 ms | arms MOD-ASSIGN mode (strip shows `>MOD<` in yellow) |
| Hold + turn a POT | selects modulation source across the pot's rotation `[OFF, LFO1..4, EUC1..4, GAT1..2, ADS1..2]`; strip shows `>MOD LFO1<` / `>MOD OFF<` etc.; fully CCW unmaps the parameter (`>MOD OFF<`); releasing the push locks the selection without release glitches |
| Hold + ROTATE | module-select overlay (strip shows `>ELEMX/CLOUDS/PLAITS/RINGS/LIRA8<`, 5 slots); releasing the hold CONFIRMS (sends SWITCH); releasing without rotating cancels |
| Short push on P1/P2/MOD-A/MOD-B | no action (parameters live on the pots) |

Gesture timings: short < 600 ms <= hold; debounce 20 ms; encoder = one
event per detent (GPIO-ISR quadrature, seed detent table).

## Pot behavior

- HC4067 round-robin at ~100 sweeps/s (8 us address settle + one ADC
  oneshot read per channel, one full-tick yield per sweep — the scan task
  must not free-run: pdMS_TO_TICKS(1) is ZERO ticks at FREERTOS_HZ=100,
  the seed's documented trap); 0-127 mapped through the per-pot NVS
  min/max calibration.
- Send-on-move: 2-count deadband vs the last SENT value, aggregate rate
  limit 200 msgs/s. P1/P2/P3 -> MIDI CC (the ACTIVE page cell's CC), MOD
  pages -> MOD_SET [src][param] (the fixed map in ui/RackUI.cpp — MUST
  MATCH the RP2350's kUiMod*Cells, docs/ui-protocol.md §6), MOD-C BPM pot
  -> UI_CMD SET_GLOBAL, SYS -> pot 0 = module / chain-pair selector, pot 5
  chain engage, pot 6 BPM, pot 7 clock source, pot 8 preset slot, pots
  9/10 BLE on/mode, pots 14/15 telemetry gates.
- **Full-range pot mapping**: Discrete and small-range parameters (2-value
  toggles, Elements MODEL 0..2, Clouds QUALITY 0..3, Rings POLY 1..4,
  WAVE 0..4, DIV 0..9, STEPS 1..32, Plaits ENGINE 0..97, BPM 20..300)
  span the entire 0..100% pot turn and display their true parameter value
  and proportional bar fill.
- **Soft takeover / pot pickup** (user spec 2026-11-08): a pot whose
  position does not match its cell's value never jumps the parameter.
  Page switches and external changes (web panel, BLE) arm a per-pot
  PENDING state: the pot sends nothing until its position aligns with the
  cell value; the renderer shows a `<` / `>` arrow beside the value —
  the turn direction that picks the value up. Own TELEM echoes never
  re-arm pending (a fast turn cannot be stalled by echo lag). **Discrete
  cells (switches / selectors) are LIVE — no takeover at all**: a switch
  has no meaningful "current position", and arming against the current
  bucket left a pot parked above the value permanently uneditable (the
  FM STRUCT "stuck on SPLIT" bench bug). The first movement sends the
  selection; continuous cells keep the bucket-aware takeover.
- **Mod-routed cells** (TELEM flags bits 3-6 = routed source): rendered
  YELLOW as a whole (label + value + bar — the state accent). The pot
  does NOT write the cell's CC (the mod source owns the parameter
  RP2350-side) — it RIDES the routed source's speed parameter instead:
  LFO -> rate, Euclid -> clock division, rhythmic gate -> division,
  ADSR -> division (a no-op while the ADSR gate-follows). Touching any
  mod-routed pot also displays `MOD:LFO1` (etc.) in the top bar. To adjust
  the parameter's own base value, un-route it first (hold the encoder and
  turn the pot fully CCW to `>MOD OFF<`) — the pot then picks the
  base value up with the normal pending/arrow behavior.

## Bench procedures (no wiring beyond the stage)

- `testpat 4` — ST7789P3 offset/orientation discovery: a 1-px frame at the
  configured window edges + ruler ticks every 20 px + the 4 hue bars.
  If the frame is clipped on one side / wraps, adjust `ST7789_XOFF` /
  `ST7789_YOFF` / `ST7789_MADCTL` at the top of `ui/Display.cpp`
  (candidates documented there) and re-flash; document the found values
  back into those #defines. `testpat 1/2/3` = full white / full black /
  hue bars (dead-pixel + inversion check, `ST7789_INVERT` flips it).
- `calib` — 8 s min/max sweep on all 16 pots (sweep each pot end-to-end
  once or twice), stored to NVS (`rpots/cal`); `calib clear` resets;
  `pots` dumps raw + mapped values. Uncalibrated pots work full-range.
- `ui` — link + page state; `page <0-10>` — page switch without the
  encoder; `heap` — free-heap report (NimBLE + 43 KB FB + link rings).
- `ble` — BLE-MIDI status (`on|off|per|cen|scan`). The C3 runs BLE-MIDI
  in ONE of two roles:
  - **PERIPHERAL** (default): advertises "Rack UI" with the standard
    BLE-MIDI service — a central (phone/DAW) connects and sends MIDI.
  - **CENTRAL**: scans and auto-connects to the first advertiser carrying
    the BLE-MIDI service (the desk's SMC-PAD), then subscribes with the
    ctag uimcu panel's chained flow — explicit MTU exchange, 0x2902 CCCD
    via RELIABLE write, the Apple-spec READ of the MIDI I/O
    characteristic, then a `write_no_rsp` keepalive. That chain (vs. a
    bare CCCD write) is what makes the pad's stock firmware actually
    stream — benched 2026-11-11: pad → C3 → RP2350 notes live, 547 msgs
    0 drops. The pad's MIDI I/O characteristic is its SINCO variant
    (`7772E5DB-3868-4112-A1A9-F2669D106BF3`), not the Apple C702.
  - SYS page cells: pot 9 = `BLE` on/off, pot 10 = `BLE MOD`
    (`PER`/`CEN`) — both C3-local (NVS-persisted across power cycles).
    Role switches tear the current role down and start the other.
  - Reference copy of the proven upstream file:
    `tools/ctag_uimcu_BtMidiReceiver.cpp` (from the ctag-tbd uimcu
    panel — read the header before touching the flow).
- `preset` — T4a bank status (partition ready, autosave seq, boot-restore
  flag); `preset save <0-15>` / `preset load <0-15>` — the bank flows via
  the link; `preset clear` — wipe slots + autosave. The bank lives in the
  raw `presets` partition (partitions.csv: 16 slots + 4 rotating autosave
  sectors @ 0x3E0000; the unused `storage` littlefs was shrunk to make
  room). After a power cycle the C3 replays the autosave through
  PRESET_APPLY once the link comes online — the rack comes back exactly
  where it was.

## Bring-up stages 2-6 (what still needs the user's wiring)

1. **Stage 2 (display)**: wire SCK/MOSI/DC + CS->GND/RST/BL; flash; watch
   the dash placeholder page + strip render; `testpat 4` to lock the
   window offsets; raise `ST7789_SPI_CLOCK_HZ` (8 MHz -> 26-40 MHz) once
   mounted.
2. **Stage 3 (pots + encoder)**: wire the HC4067 (SIG->GPIO3, S0-S3 =
   GPIO10/20/21/8, pots) + encoder (A/B/push = GPIO6/7/9); `calib`; verify
   `pots` tracks and page rotation works; pot moves on P1 send MIDI CC
   (visible in the RP2350 console `ui` counters).
3. **Stage 4 (TELEM pages)**: needs the cross-board link (stage-1 wiring):
   P1/P2/MOD pages render live values; verify colors/dims/blink against a
   routed + inert mod destination.
4. **Stage 5 (gestures)**: hold+pot assignment round-trip (RP2350 `mod`
   console shows the acks), hold+rotate module swap, SYS ring + MOD-C
   clock toggle.
5. **Stage 6 (BLE-MIDI)**: a BLE central connects to "Rack UI"; notes land
   in the RP2350 funnel (its 2 s report's note-on counter moves). Watch
   `heap` before/after NimBLE for the RAM budget.
