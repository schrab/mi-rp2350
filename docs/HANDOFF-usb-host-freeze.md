# HANDOFF — PIO-USB host: board freeze on device attach (2026-10-31)

> **RESOLUTION UPDATE (same night):** Suspect 1 **CONFIRMED** — the hard
> freeze was core 0 overrunning its 4 KB half of the shared dual-core stack
> at attach. Fix applied on `elements-clone`: `bool core1_separate_stack =
> true;` in main.cpp (core 1 gets a private 8 KB stack; core 0 gets the full
> system stack). With it, the attach-time hard freeze is gone (board stays
> alive through repeated enumeration attempts; serial never stops mid-line).
> **Still open:** the controllers still never ACK our SETUP (EP0 result 00)
> — prime suspect is now Suspect 2 (the controllers' USB PHY stays off until
> they see VBUS; the 5 V wire's delivery has never been measured at the
> device end). Suspects 3/4 remain valid hardening items.

For the next engineer/model. Read this fully before touching the USB host code.
Everything below was measured on hardware during one overnight session; the
symptom is real, reproducible, and **controller-independent**.

## Symptom

Connecting any USB MIDI controller to the PIO-USB host port (GP14=D+, GP15=D-)
causes, within seconds:

1. **Board freeze** — core 0 stops (serial report stops mid-line, status LED
   freezes), core 1 keeps clocking the I2S DMA (user reports "noise played").
   Freeze is at/around the first enumeration transactions. LED breadcrumb
   (see Diagnostics) shows the freeze happens **after SETUP requests have been
   sent** (magenta stage), and a second variant freezes **before any SETUP**
   (blue stage) when the controller is connected at board boot.
2. Often the controller's own chip crash-loops first (observed: pad freezing
   and self-rebooting every ~2 s), then the board freezes.
3. The serial EP0 log (last 8 EP0 SETUP packets) always shows the identical
   GET_DESCRIPTOR sequence with **result 00 — no device handshake, ever**:

```
EP0: dddf:f77f7775fd:00 ... (×8, byte-identical every attempt)
```

   (`dddf:f77f7775fd` is the NRZI-encoded `SYNC 80 2D` + GET_DESCRIPTOR
   payload — deterministic, so our TX data itself is stable.)

4. `max Process` creeps 380 → 520 µs across a session (budget 496 µs) with no
   OMINOUS active — core 1 slowdown correlated with session length (XIP cache/
   temperature suspect), worth checking independently.

**Controller-independent:** reproduced with a JieLi BR25-based pad (SMC-PAD,
`4353:4b4d`, full-speed) AND a second, different MIDI controller. Same EP0
signature, same freeze. → The bug is in our host code path, not in one chip.

## Environment

- RP2350 nano (silicon revision **3** — `chip_version=3`, read via SYSINFO),
  240 MHz (PIO-USB requires a 12 MHz multiple), arduino-pico core 1.60100.0,
  bundled TinyUSB 3.7.7, **vendored pio_usb v0.7.3 = upstream HEAD as of
  2026-10-30** — which includes brand-new (Oct 2026) host RX rework commits
  (park_receiver / arm-at-own-EOP / TX-EOP-wait). A pre-rework variant was
  also tested (see matrix) — freeze persists.
- Host port wiring: D+→GP14, D-→GP15, GND, VBUS→5V (present but of unproven
  integrity; controllers are self-powered). Soldered, not jumpered.
- Board used as USB device to the laptop simultaneously (native controller,
  CDC serial + USB-MIDI device) — works, but its port dies on every soft
  reboot (latched D+ pull-up; replug required after every flash).

## Verified hardware facts (multimeter + logic, all measured)

- **Board silicon rev 3:** the pio-usb RP2350-E9 errata workaround
  (`chip_version <= 2` gate in `pio_usb_bus_get_line_state`) is SKIPPED.
  Forcing the dance ON breaks the line reads (conn=0 — reads return wrong
  levels); stock (skip) reads correctly (st=1 = proper FS idle with a device
  attached). **Never force the E9 dance on this board.**
- **Line-state reads from the main loop race the SOF alarm ISR** via that
  same IE-dance (a loop-side dance can be preempted mid-dance; the ISR then
  runs a whole transaction with the pad input disabled → lost responses).
  All loop-side `pio_usb_bus_get_line_state()` calls were removed. Keep it
  that way — read line state only from the SOF ISR (connection_check /
  connect detection).
- The pad standalone (board unpowered) holds **both lines ~3.3 V** — its
  no-VBUS wait state (PHY disabled until it sees VBUS; it enumerates
  instantly on a laptop, which supplies VBUS). D- showed unstable values all
  night (1.87 / 3.3 / 3.53 V) while D+ was stable (3.2 / 2.7 / 2.8 V).
- `max Process` is measured over `Part::Process` only; the I2S stall counter
  stayed 0 throughout.

## Variant matrix (all flashed and tested tonight)

| # | Build | Result |
|---|---|---|
| 1 | Morning baseline `4523171` (watchdog 4 s + loop-side line reads) | Mounted twice early; later never; freeze eventually |
| 2 | Watchdog 15 s, no loop-side reads | conn=1, SETUP result 00, no mount |
| 3 | E9 dance forced | reads break (conn=0) — rejected |
| 4 | Bus reset no-oped | pad crash-loop persists → reset is not (the only) trigger |
| 5 | SOF gated until mount | board stops freezing; pad still crash-loops on SETUP |
| 6 | Pre-rework pio_usb (5a37a66: before park_receiver/arm-EOP) | same EP0 result 00 silence |
| 7 | Current HEAD lib + all fixes + 300 ms debounce + reset restored | EP0 result 00; freeze at attach (blue/magenta per breadcrumbs) |

Conclusion so far: **no combination of reset/SOF/debounce/RX-vintage fixes
the no-handshake**. The freeze and the silence co-exist; both are
controller-independent. Something in our connect-time host behavior breaks
the device exchange — or the bus itself.

## EP0 signature detail

`dd df f7 7f 77 75 fd` = NRZI-encoded first bytes of `SYNC 80 2D 80 06 00 01
00 00 40 00` (GET_DESCRIPTOR Device). It is byte-identical across attempts —
our TX data path is deterministic. Result 00 = `usb_setup_transaction` got no
handshake within its 3 µs RX-start / 7 µs handshake windows. Either the device
never saw a valid token (TX signal integrity at the device? E9 pad leak
distorting our own driven edges?) or its reply was lost (RX path).

## Code state

- **Known-good-for-audio baseline:** commit `4523171` on `elements-clone`
  (mounted a pad twice early in the session — by luck, see matrix row 1).
- **OMINOUS mode (CC29 easter egg)** implemented and committed on
  `elements-clone` — works, but has a **separate known issue: core-1 overload,
  max Process ~1.1 ms vs 496 µs budget** while active (8× oversampled FM
  chain). Audio plays at wrong pitch while OMINOUS is on. Must be fixed
  before shipping CC29 (levers: the OMINOUSVoice chain in
  `lib/elements/elements/dsp/ominous_voice.*`).
- **Debug WIP preserved on branch `usb-host-jieli-debug`**: the full
  diagnostic build — pre-rework pio_usb variant, EP0 setup ring
  (`pio_usb_setup_log[8][8]` in pio_usb_host.c, printed as `EP0:` in the 2 s
  report), 300 ms connect debounce (`attach_idle_frames` in root_port_t),
  LED stage breadcrumbs (ForceColor in status_led.h), watchdog
  instrumentation, pad-register dumps. Diff vs `4523171` also meaningful.
- Serial monitoring works alongside everything: `stty -F /dev/ttyACM0 115200
  raw -echo && cat /dev/ttyACM0`. The board's CDC dies after every soft
  reboot (latched D+ pull-up) — **physical replug required after each flash.**
- Flash without a button: `stty -F /dev/ttyACM0 1200 raw -echo` (1200-baud
  touch) → board re-enumerates as mass storage `/dev/sdb1` labeled `RP2350` →
  mount (udisksctl) → copy `elements-rp2350/.pio/build/elements-rp2350/
  firmware.uf2` → sync. If the touch fails, hold BOOTSEL while plugging.

## Next steps, in order

1. **Crash capture (do this first).** The freeze kills the serial before it
   can explain itself. Enable the RP2350 watchdog (~2 s) in setup and write
   an enumeration-stage breadcrumb to a **scratch register / RTC scratch**
   (survives warm reboot) at every host step: attach detected, reset phase,
   first SETUP, first ACK, mounted. On WDT reboot, print the scratch value at
   boot → the frozen stage is known across a freeze. Bisect from there.
2. **Check `max Process` creep** after a cold boot vs after hours — if it
   tracks session length, suspect XIP flash cache contention (the PIO-USB ISR
   + DSP both live in flash) and consider `-m`... measure first.
3. **Signal-integrity check of the TX path**: scope D+/D- during a SETUP
   burst if a scope is available. The E9 pad leak (input-enable leak on
   GP14/15, documented errata family) biases released lines; during our own
   driven packets the 12 mA drive should dominate — verify edges at the
   *device* end of the cable, not at the board.
4. **Compare with a known-good PIO-USB host at the same commit**: build the
   stock pio_usb `DualRole`/`serial_host_bridge` example for this exact board
   and attach the controllers. If the example freezes too, the bug is in the
   vendored lib + our clock/pins; if it enumerates, bisect our integration
   (init order, alarm pool, the SOF/EP0 ring instrumentation).
5. **Do not**: force the E9 dance (breaks reads on rev 3), read line state
   from the main loop (races the SOF ISR), or retry enumeration faster than
   ~15 s (reset storms wedge fragile chips — documented for the JieLi in
   open-smc-pad `docs/usb-br25.md` item 6, confirmed by observation).

## References

- Pad reverse-engineering (same hardware): https://github.com/fedepaj/open-smc-pad
  — `docs/usb-br25.md` documents the JieLi BR25 USB controller and its
  reset-into-empty-vector crash.
- pio_usb upstream: https://github.com/sekigon-gonnoc/Pico-PIO-USB (HEAD
  vendored; the Oct 2026 RX-rework commits are the newest, least-tested code
  in the path).
