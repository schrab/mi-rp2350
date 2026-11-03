# Rack RP2350 — all four MI ports in one firmware

ONE firmware running the four accepted ports (Elements, Clouds, Plaits
98-engine, Rings MiniElements) on the RP2350 nano + PCM5102A. T1 of the
combined plan (`docs/superpowers/plans/2026-11-04-combined-rp2350.md`):
boot-select ONE module end-to-end, with the other three linked and
flash-resident, ready for T2's live swap.

## Module selection (T1: compile-time boot-select)

Four PlatformIO envs, one per boot module:

```sh
pio run -d rack-rp2350 -e rack-elements   # or rack-clouds / rack-plaits / rack-rings
# → .pio/build/<env>/firmware.uf2
```

`RACK_BOOT_MODULE` (0 elements / 1 clouds / 2 plaits / 3 rings) is a per-env
define. All four modules' code is linked in every build; only the selected
module instantiates state. T2 replaces this with a runtime live swap.

## SRAM window (the T1 mechanism)

- The four `RAM_CODE` patch sets emit `.tc_<module>.<token>` sections when
  `RACK_RP2350_TC` is defined (standalone builds keep `.time_critical.*`).
- Per-env linker scripts `memmap_rack_<module>.ld` (forks of the core's
  rp2350 script, set via per-env `board_build.ldscript`) place the SELECTED
  module's sections into an ld OVERLAY occupying that env's SRAM window at
  the top of main SRAM (origin = 0x20000000 + the env's RAM length), LMA in
  a reserved flash region at 0x10381000. The other three modules are linked
  FLASH-RESIDENT (`.tc_flash_*` sections, VMA = LMA — dead weight until
  T6's re-tier). (The old shared `memmap_rack.ld` sized ONE window for
  plaits; clouds' heap then could not malloc core 1's separate 8 KB stack,
  so core 1 never launched and core 0 hung at the alarm-pool handshake.)
- `rack_window_copy(module_id)` memcpy's the selected module's flash copy
  into the window as the FIRST action of `setup()` (crt0-like: before any
  peripheral is up) and prints size + FNV checksums (the print is deferred
  past enumeration — see the boot-wedge note below).
- Window sizing (measured rack `.tc_*` totals, 2026-11-04): elements
  36,752 B → window 0x9000; clouds 136,960 B (incl. 60,344 CMSIS) →
  0x21800; plaits 272,564 B → 0x43000; rings 58,084 B → 0xE400. Heap
  headroom is now per-env (see each script's header) instead of shared.

## Wiring

Identical to the four standalone ports: PCM5102A BCK GP6 / DIN GP8 /
LRCK GP7, XSMT 3V3; USB-A host D+ GP14 / D- GP15; WS2812 GP22; USB-C =
CDC serial (115200) + USB MIDI device ("Rack RP2350").

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
host: conn= st= dev= midi= frame=` — budgets: elements 496, clouds 1000,
plaits 500 (48 kHz), rings 750 µs.
