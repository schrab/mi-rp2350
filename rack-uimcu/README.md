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
| 3 | HC4067 **SIG** (common, Z) + encoder push | ADC1_CH3; encoder push shares this line through a 4.7k series resistor to GND (detected in `RackPots` scan task) |
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
- encoder push shared with mux SIG (`GPIO3`) via a 4.7k series resistor to GND.
- mux E (enable) tied GND — always enabled.

## Wiring summary

| From | To |
|------|----|
| C3 GPIO4 | RP2350 GP1 (UART0 RX) |
| C3 GPIO5 | RP2350 GP0 (UART0 TX) |
| C3 GPIO10/20/21/7 | ST7789P3 SCK/MOSI/DC/RST; panel CS->GND, BL->3V3, VCC 3V3, GND |
| C3 GPIO3 | HC4067 SIG (common; datasheet Z) — pots 0-15 on CH0-15; encoder push -> 4.7k -> GND |
| C3 GPIO8/9/2/6 | HC4067 S0/S1/S2/S3 — push-pull outputs |
| C3 GPIO0/1 | encoder A/B (switch to GND; internal + 10k external pull-ups) |

All 3V3 logic, no level shifting.

## Gestures (the whole UI: turn pots + one encoder)

| Gesture | Action |
|---------|--------|
| Rotate | page cycle P1 -> P2 -> MOD-A -> MOD-B -> MOD-C -> SYS (wraps); the C3 requests the page, dashes render until the TELEM snapshot lands |
| Short push (< 600 ms) on MOD-C | toggle global clock source (UI_CMD sys id 2) |
| Short push on SYS | the SYS action ring, one action per push, next action marked with the yellow underline: chain-pair cycle -> chain engage toggle -> preset slot cycle + PRESET_REQ (T4a stub) -> telemetry-deltas toggle -> telemetry-full toggle -> wrap |
| Push-hold >= 600 ms | arms MOD-ASSIGN mode (strip shows `>MOD<` in yellow) |
| Hold + turn a POT | sends UI_CMD ASSIGN_CYCLE [pot][dir] — the RP2350 cycles that cell's mod source [off, src0..src11] and the full TELEM reply re-renders the result (invalid targets are inert on the RP2350 and the cell blinks via TELEM flags bit2) |
| Hold + ROTATE | module-select overlay (strip shows `>ELEM/CLDS/PLTS/RNGS<`); releasing the hold CONFIRMS (sends SWITCH); releasing without rotating cancels |
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
  limit 200 msgs/s. P1/P2 -> MIDI CC (the ACTIVE page cell's CC), MOD
  pages -> MOD_SET [src][param] (the fixed map in ui/RackUI.cpp — MUST
  MATCH the RP2350's kUiMod*Cells, docs/ui-protocol.md §6), MOD-C BPM pot
  -> UI_CMD SET_GLOBAL, SYS -> inert.
- **Soft takeover / pot pickup** (user spec 2026-11-08): a pot whose
  position does not match its cell's value never jumps the parameter.
  Page switches and external changes (web panel, BLE) arm a per-pot
  PENDING state: the pot sends nothing until its position aligns with the
  cell value; the renderer shows a `<` / `>` arrow beside the value —
  the turn direction that picks the value up. Own TELEM echoes never
  re-arm pending (a fast turn cannot be stalled by echo lag).
- **Mod-routed cells** (TELEM flags bits 3-6 = routed source): rendered
  YELLOW as a whole (label + value + bar — the state accent). The pot
  does NOT write the cell's CC (the mod source owns the parameter
  RP2350-side) — it RIDES the routed source's speed parameter instead:
  LFO -> rate, Euclid -> clock division, rhythmic gate -> division,
  ADSR -> division (a no-op while the ADSR gate-follows). To adjust the
  parameter's own base value, un-route it first (hold the encoder and
  turn the pot until the source cycles to off) — the pot then picks the
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
- `ui` — link + page state; `page <0-5>` — page switch without the
  encoder; `heap` — free-heap report (NimBLE + 43 KB FB + link rings).

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
