# RP2350 board — NoLogo "Mini USB RP2350"

The rack's RP2350 board (called the "RP2350 nano" in older docs) is the
**NoLogo Mini USB RP2350** (RP2350A, Cortex-M33). This page is the board
reference: pinout, peripherals, and how the rack firmware uses them.

**Provenance:** the most complete public pinout is the Zephyr board
definition (`boards/nologo/mini_usb_rp2350`, compatibles
`nologo,miniusb-header` / `nologo,miniusb-pcbpads`), which itself is the
result of "extensive technical evaluation, correction, rectification, and
supplementation" of the (unreliable) Chinese vendor site + AliExpress
retailer data. The rack does NOT run Zephyr — our firmware is
Arduino/earlephilhower — but the hardware map below is authoritative.
Reference: Zephyr upstream issue for RP2350 peripheral status:
<https://github.com/zephyrproject-rtos/zephyr/issues/53810>.

## Board facts

- **Crystal:** external **12 MHz** (stock Zephyr PLL → 125 MHz). This is the
  origin of the PIO-USB law "f_cpu must be a multiple of 12 MHz" when the
  host port is enabled. The rack runs **320 MHz @ VREG 1.20 V** with
  PIO-USB host disabled (`RACK_ENABLE_USB_HOST=0`).
- **Flash:** QSPI, **4 MB** in the default configuration; a **16 MB** variant
  exists as a hardware revision (Zephyr: `mini_usb_rp2350@16mb`). The rack
  UF2 currently uses ~1.62 MB of the 4 MB map.
- **External pin mapping is NOT Pico-compatible** — a compact rearrangement
  (18 edge pins + 2 Qwiic + 13 bottom solder pads). All internal RP2350A
  GPIO lines are usable (edge or bottom pads).
- **ADC voltage reference:** `ADC_VREF` is **hard-wired to the digital 3.3 V
  rail** — not decoupled, no external-reference option. Relevant if CV
  inputs are ever added: the ADC full scale is the 3.3 V rail.
- **GP22** is doubly routed: the **on-board user RGB LED (WS2812)** DI line
  AND one of the bottom solder pads — the rack's status LED sits there
  without losing the pad.
- USB device controller: stock Zephyr product string is
  `Mini USB RP2350 (CDC ACM)` (VID 2e8a, PID 000a); the rack firmware
  renames the CDC device to **"Rack RP2350"** (that's the `/dev/serial/by-id`
  discriminator, see AGENTS.md).

## Pin map

### Edge (1–18)

| # | GPIO | Default function (Zephyr) | Notes |
|---|------|---------------------------|-------|
| 1 | GP0 | UART0_TX (PWM0 ch A) | |
| 2 | GP1 | UART0_RX (PWM0 ch B) | |
| 3 | GP2 | PIO/PWM (PWM2) | optional UART0_CTS |
| 4 | GP3 | PIO/PWM (PWM3) | optional UART0_RTS |
| 5 | GP4 | SPI0_RX (PWM4) | rack: **UART1 TX, RESERVED** |
| 6 | GP5 | SPI0_CSN (PWM5) | rack: **DIN MIDI IN** (UART1 RX, 6N138 opto) |
| 7 | GP6 | SPI0_SCK (PWM6) | rack: **I2S BCK** (PCM5102A) |
| 8 | GP7 | SPI0_TX (PWM7) | rack: **I2S LRCK** |
| 9 | GP8 | PIO/PWM (PWM8) | rack: **I2S DIN** |
| 10 | GP14 | I2C1_SDA (PWM14) | rack: **USB host D+** (PIO-USB DPDM) |
| 11 | GP15 | I2C1_SCL (PWM15) | rack: **USB host D−** |
| 12 | GP26 | ADC_CH0 (PWM10) | |
| 13 | GP27 | ADC_CH1 (PWM11) | |
| 14 | GP28 | ADC_CH2 (PWM12) | |
| 15 | GP29 | ADC_CH3 (PWM13) | |
| 16 | — | **3V3 OUT** | |
| 17 | — | **GND** | |
| 18 | — | **VSYS** | |

### Qwiic / STEMMA QT (Grove "Laced" interface)

| Pad | GPIO | Function |
|-----|------|----------|
| QC | GP17 | I2C0_SCL (PWM0 ch B) |
| QD | GP16 | I2C0_SDA (PWM0 ch A) |

The vendor's "Laced Grove Signal Interface" allows remapping signal groups
onto Grove connectors; the mapping list is explicitly **not stable** across
board revisions — verify against the actual PCB before relying on it. The
rack does not use the Qwiic connector today (GP16/17 are free; a future UI
or peripheral could ride I2C0 there).

### Bottom solder pads (A–M, 13 pads)

| Pad | GPIO | Pad | GPIO |
|-----|------|-----|------|
| A | GP9 (PWM9) | H | GP20 (PWM4) |
| B | GP10 (PWM10) | I | GP21 (PWM5) |
| C | GP11 (PWM11) | J | **GP22** (PWM6 — also the on-board RGB LED) |
| D | GP12 (PWM12) | K | GP23 (PWM7) |
| E | GP13 (PWM13) | L | GP24 (PWM8) |
| F | GP18 (PWM2) | M | GP25 (PWM9) |
| G | GP19 (PWM3) | | |

## Peripheral inventory

- **UART ×2** (UART0 on GP0/GP1 by default; the rack uses **UART1** on
  GP4/GP5 for DIN MIDI — UART functions are muxable to many pins).
- **SPI ×2** (SPI0 wired GP4–GP7; the rack muxes those pins to I2S/UART
  instead). **I2C ×2** (I2C0 → Qwiic GP16/17; I2C1 → edge GP14/15 — both
  free in the rack, GP14/15 serve USB host).
- **PWM: 8 units × 2 channels** = 16 channels, nearly all reachable.
- **ADC: 4 external channels (GP26–GP29) + on-chip temperature sensor**;
  VREF fixed at 3.3 V (see above).
- **PIO ×3** — the rack claims PIO0 entirely for PIO-USB host when enabled
  (init-order law), status LED on **PIO2 SM0** (GP22), I2S on **PIO2 SM1**.
- **DMA**, **watchdog**, native **USB device** (CDC-ACM console).
- Zephyr does NOT yet support **POWMAN / VREG / QMI (flash) on RP2350** —
  the rack firmware drives these registers directly (the pre-clock
  constructor in `main.cpp` raises VREG 1.20 V and pins QMI CLKDIV/RXDELAY
  before F_CPU is applied; console readback via `qmi` / `vreg`).

## Cross-references

- Rack wiring and jack table: `rack-rp2350/README.md` (I2S/DIN/host/LED).
- Clock/overclock bench + constraints: `AGENTS.md` + ledger
  (`.superpowers/sdd/progress.md`, §T9 + rp2350-overclocking entries).
