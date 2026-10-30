# elements-rp2350

Standalone Mutable Instruments Elements synth voice on an RP2350 "nano" board
with a PCM5102A I2S DAC. PlatformIO project using the earlephilhower
arduino-pico core (core package version 1.60100.0).

Part of the plan in `../docs/superpowers/plans/2026-10-30-elements-rp2350.md`.

- Sample rate: 32000 Hz
- CPU: 240 MHz, `-O3` (240, not 250 — see Build notes)
- Core 0: USB MIDI device (TinyUSB) + USB MIDI host (PIO-USB, GP14/GP15),
  MIDI parsing, patch/CC application, stuck-gate watchdog, serial diagnostics
- Core 1: `elements::Part::Process()` in 16-sample blocks -> I2S DMA;
  applies pending resonator-model changes between blocks
- USB device name: "Elements RP2350" (product descriptor + MIDI port name)

## Wiring

### PCM5102A breakout -> RP2350 nano

| PCM5102A | RP2350 | Note |
|---|---|---|
| BCK | GP6 | bit clock |
| LCK/WS | GP7 | = BCK + 1 (required by the arduino-pico I2S library) |
| DIN | GP8 | I2S data into the DAC |
| SCK/MCLK | GND | internal PLL, no external MCLK |
| XSMT | 3V3 | unmute (jumper H3 on some breakouts) |
| FLT (H1), DEMP (H2), FMT (H4) | GND | I2S format, normal filter (usually pre-set) |
| VIN | 3V3 | |
| GND | GND | |

### USB-A socket (host, for the MIDI controller) -> RP2350 nano

| USB-A | RP2350 | Note |
|---|---|---|
| VBUS | 5V pin | powers the controller; plain 5 V rail, NOT GPIO-switched |
| D+ | GP14 | PIO-USB requires D- = D+ + 1 |
| D- | GP15 | |
| GND | GND | connect the shell/shield to GND too |

Bus-powered controllers draw from the board's 5 V rail; current-hungry hubs
need an external VBUS feed.

## MIDI

MIDI enters simultaneously from both ports and is merged (same parser):

- **USB device** (USB-C): PC / DAW. Enumerates as "Elements RP2350" with one
  MIDI port ("Elements RP2350 MIDI 1").
- **USB host** (USB-A socket): hardware MIDI controller, via PIO-USB.

### Behavior

- **Note stack (last-note priority):** up to 8 held notes. Pitch always
  follows the newest held note; releasing it falls back to the previous one;
  the gate only closes when the last note is released. Re-pressing a held
  note moves it to the top (no duplicate, no retrigger while the gate stays
  high). When the stack is full, the oldest note is dropped.
- **Pitch bend:** ±2 semitones, remembered and applied on top of the current
  top note (survives legato note changes).
- **Velocity** maps to exciter strength: `strength = 0.4 + 0.6 * vel / 127`.
- **Stuck-gate watchdog:** if the gate stays on > 30 s with no note event
  (e.g. a lost note-off after unplugging the controller mid-note), the synth
  forces the gate off, clears the stack and logs
  `WARNING: gate stuck >30s ...` once.

### CC map

| CC | Parameter | CC | Parameter |
|---|---|---|---|
| 1 | modulation (PerformanceState) | 20 | exciter strike level |
| 11 | strength (PerformanceState) | 21 | exciter strike meta (mallet) |
| 14 | exciter envelope shape | 22 | exciter strike timbre |
| 15 | exciter bow level | 23 | resonator geometry |
| 16 | exciter bow timbre | 24 | resonator brightness |
| 17 | exciter blow level | 25 | resonator damping |
| 18 | exciter blow meta (flow) | 26 | resonator position |
| 19 | exciter blow timbre | 27 | space (reverb) |
| 29 | ominous voice (easter egg): 0 normal / ≥1 ominous | 28 | resonator model: 0 modal / 1 string / 2 strings |

CC 28 values > 2 are ignored; re-sending a value re-selects that model. The
change is applied by core 1 between render blocks (pending slot), so it never
lands inside a running `Process()`. CC 29 switches to the hidden ominous
voice live (0 = normal, any nonzero value = ominous — absolute values, not a
toggle): the dark 2-op FM path replaces the exciter/resonator, the ringing
voice's tail cuts on switch (no crossfade), and CC 28 has no effect while
ominous. `reverb_diffusion` / `reverb_lp` are
derived from `space` by the engine and are not mapped;
`resonator_modulation_frequency` is not mapped (randomized per device by
`Part::Seed()`).

## Status LED

One onboard WS2812 on GP22:

| Color | Meaning |
|---|---|
| dark | idle |
| amber | note gated (base color) |
| turquoise tint | exciter meter active |
| brighter amber | resonator meter active |
| white-ish | both meters active |
| 3 green blinks | USB-A MIDI controller mounted |
| red/green blink | boot error (I2S init failed) |

The gate, exciter and resonator indicators are additively blended into one
LED: amber for the gate, turquoise for the exciter (the original Elements
`LED1` color) and amber for the resonator. The exciter and resonator
channels use the original `lut_db_led_brightness` curve and the original
per-sample meter smoothing (~31 ms exciter release, ~62 ms resonator
release), so the LED follows the same activity and decay tails as the
hardware. While a note is held the amber base is always visible; the
exciter shifts the hue toward cyan, the resonator brightens the amber, and
both together can wash it toward white.

At boot the LED flashes white three times as a self-test, then goes dark.
Mounting a MIDI controller on the USB-A host port triggers three short green
blinks; the note/meter color resumes afterwards. The blink has priority over
the note color, so it is visible even if a note is held.

The LED is driven by PIO2 SM0 using the SDK `ws2812` PIO program. Two details
matter:

- `statusLed.Init()` calls `pio_sm_claim(pio2, 0)` before `i2s.begin()`.
  The Arduino `PIOProgram` allocator used by I2S scans PIOs from the highest
  index down and would otherwise claim PIO2 SM0, reconfigure it for I2S and
  freeze the LED on its boot color while audio kept running.
- The 24-bit GRB frame is left-aligned (`<< 8`) because the PIO shifts left
  with a 24-bit autopull threshold and therefore transmits the top 24 bits of
  each 32-bit FIFO word.

## Serial diagnostics (115200 baud)

Every 2 s, one line:

```
max Process: 213 us (budget 500 us) | i2s stalls: 0 (+0)
```

- `max Process` — worst `Part::Process` block time (16 samples) since the
  last report; budget = 500 us (one block @ 32 kHz). Must stay comfortably
  below it.
- `i2s stalls` — cumulative count of render loops whose 16 I2S writes took
  > 900 us (normally the write block waits at most ~500 us for the next DMA
  buffer to free — see Build notes for the threshold rationale); `(+n)` is
  the delta since the previous report. In steady state this stays `0 (+0)`.

Boot log: `I2S up`, `PIO2 SM0 claimed=1 pc=... (ws2812 @...) | SM1 claimed=1
pc=...`, `Elements engine up (USB MIDI device + PIO-USB host)`; `Host MIDI
mounted idx=... daddr=... cables=...` when a controller is plugged into the
USB-A socket, `Host MIDI unmounted idx=...` on removal. The PIO2 line confirms
the LED's state machine survived I2S initialization: SM0's PC must stay inside
the 4-instruction WS2812 program.

## Building

```sh
pio run -d elements-rp2350
```

The first build downloads the platform and toolchain (several minutes).
Artifacts: `.pio/build/elements-rp2350/firmware.uf2` (+ `.elf` / `.bin`).

### Build notes

- **Board ID:** the plan's `board = pico2` is the Arduino core's board name;
  this PlatformIO platform has no such ID. `board = rpipico2` is used
  (Raspberry Pi Pico 2, earlephilhower core, picotool upload protocol).
- **240 MHz (not 250):** PIO-USB bit-bangs full-speed USB on the CPU and
  requires a CPU clock that is a multiple of 12 MHz; 240 = 12 x 20. The I2S
  sample clock is PLL-derived, so audio stays exactly 32 kHz. A runtime guard
  prints an error if `f_cpu` is reverted.
- **Library sync (`tools/sync_pio_libs.sh`):** the ELEMENTS/STMLIB submodules
  double every `.cc` as an `.inc` compiled through a single wrapper, and the
  submodules must not be modified. The script copies `STMLIB/src ->
  lib/stmlib/` and `ELEMENTS/src -> lib/elements/` with all `.cc` files
  deleted (`.inc` duplicates, headers and the `*_all.cpp` wrappers are kept)
  and **regenerates each library's `library.json` with
  `"build": {"flags": ["-O3"]}` on every run** — the earlephilhower core
  appends `-Os` after the project's `-O3`, so per-library flags are the only
  way to keep the DSP compiled at `-O3`. Re-run the script after submodule
  updates:
  ```sh
  tools/sync_pio_libs.sh
  ```
- **PIO-USB init order (do not reorder setup()):** PIO-USB hard-claims PIO0
  (all 32 instruction slots + 3 SMs), DMA channel 0 and an alarm-pool slot;
  `pio_sm_claim` panics on a taken SM. The USB host must be initialized
  BEFORE `i2s.begin()`: the I2S library's `PIOProgram` then flexibly
  allocates a free PIO+SM and lands on PIO1 (GP6/7/8 are reachable from any
  PIO on the RP2350). Reversed order = boot panic.
- **`-DUSE_TINYUSB`** requires `#include <Adafruit_TinyUSB.h>` in the sketch
  and LDF `lib_ldf_mode = chain+` (with `off`, the core maps `Serial` to the
  TinyUSB CDC whose symbols live in the bundled Adafruit_TinyUSB_Arduino
  library that LDF then never compiles -> undefined `Adafruit_USBD_CDC::begin`
  / `Serial` at link time).
- `elements::Dsp::kSampleRate / kSrFactor / kIntervalCorrection` are declared
  as class statics in `ELEMENTS/src/elements/dsp/dsp.h` but defined nowhere
  in the library; `src/main.cpp` defines them.

## Uploading

### BOOTSEL / UF2 drag-drop (always works)

1. Build (see above) -> `.pio/build/elements-rp2350/firmware.uf2`.
2. Unplug the USB-C cable.
3. Hold **BOOTSEL**, plug USB-C back in, release. The board enumerates as a
   USB mass-storage drive named `RP2350`.
4. Copy `firmware.uf2` onto that drive (file manager drag-drop, or
   `cp .pio/build/elements-rp2350/firmware.uf2 /run/media/$USER/RP2350/`).
   The board flashes and reboots into the new firmware.

### `pio run -t upload`

Touches 1200 bps and uses the platform-bundled `picotool`
(`~/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool`;
no system install needed). Caveats verified 2026-10-30:

- With the factory ("Betaflight RP2350A") firmware the 1200 bps touch does
  NOT reboot to BOOTSEL -> upload fails. Flash once via BOOTSEL; after our
  own firmware runs, reboot-to-BOOTSEL (1200 bps touch / double reset) works
  and `pio run -t upload` should succeed from then on.
- picotool needs raw USB write access; without a udev rule the device node is
  `root:root` and upload fails (Error 249). Optional fix:
  `/etc/udev/rules.d/99-picotool.rules` containing
  `SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666"`, then
  `sudo udevadm control --reload-rules && sudo udevadm trigger`
  (or run the upload under sudo).

## Soak test (T6 acceptance)

1. Flash via BOOTSEL with the MIDI controller in the USB-A socket and USB-C
   in the PC.
2. Play from the controller for 30+ minutes (sequenced or jammed notes, some
   CC movement, pitch-bend riding, legato intervals).
3. Watch the serial monitor (115200):
   - `max Process` stays stable and well under 500 us;
   - `i2s stalls` stays `0 (+0)`;
   - no repeated `Host MIDI mounted` / `unmounted` without touching the
     cable (mount flaps would indicate a PIO-USB signal/power problem);
   - no audio drops, no crash or hang.
4. Also confirm: the board now enumerates as "Elements RP2350"
   (`aseqdump -l` / `lsusb` — takes effect on this flash's re-enumeration),
   CC 28 audibly switches modal / string / strings, and legato behaves as
   described under Note stack.
