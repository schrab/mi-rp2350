# HANDOFF — the Pico SDK migration + phase-2 rig state (2026-11-23)

Read this plus:
- `docs/superpowers/plans/2026-11-23-pico-phase2.md` (the next-phase plan — the DSPi adoption candidates, task-by-task)
- `.superpowers/sdd/2026-11-23-pico-migration/progress.md` (the migration ledger — every root cause + commit)
- `rack-pico/README.md` (the build + the pin map)

## What is working (all user-confirmed by ear, 2026-11-23)

- **The full rack firmware runs on the Pico SDK + CMake** (`rack-pico/`,
  target `rack`): 7 modules, the live swap, chain mode, the console, the
  UI link, presets (blob v5/v6, pushed to the C3), the watchdog + crash
  forensics, the 2 s report. Boot module = benjolin (RACK_BOOT_MODULE=6).
- **Audio verified by ear on every module** ("yes now all fine and benjolin
  sound is much cleaner"). Page switching works (~500 ms updates).
- **All 7 modules sound after module switches.**
- The C3 UI is paired and stable (esp-idf **v6.1.1** — this machine has
  `~/esp/esp-idf-v6.1.1`; the stale v5.5.4 CMake cache must be wiped before
  a rebuild). The C3 preset blob (v6, 1359 B, 6 chunks) pushes/receives.
- The UI link delivers full snapshots: 12/12 one-per-second parses in the
  bench run; page-request replies land in ~100 ms.
- `scripts/check_ram_placement.py` (rack-pico): VERDICT PASS (0 FAIL) —
  the tc_* window placement exact, the heap slack 17.3 KiB ≥ 8 KiB gate.

## The five root causes fixed during bring-up (all bench-caught)

1. **The core-1 harness** (6897587): the SDK's multicore trampoline runs the
   entry function ONCE and parks the core. The Arduino core's
   setup1()+loop1() harness had to be reproduced (`core1_main`). Launch core 1
   AFTER the sys-clock raise.
2. **The i2s_out autopull** (cd12ced): the PIO program has no explicit
   `pull`; autopull=false shifted stale OSR bits forever — the TX FIFO never
   drained, the DMA never completed, the render stalled at the bank depth.
3. **The I2S FORMAT — the "all plugins silent" root cause** (c8d6f32): the
   hand-written program was left-justified + LSB-first; the PCM5102A wants
   standard I2S. Every sample arrived bit-rotated (the sign bit in the LSB
   position) = digital near-silence at the DAC while the engine meters showed
   healthy signal — the meters sit BEFORE the I2S path. The fix mirrors the
   core's proven `pio_i2s_out` (WCLK flips ON the last data bit, LEFT shift
   MSB-first, autopull 32, y = bits-2, instruction rate = FS*64). User:
   "benjolin sound is much cleaner".
4. **The silent-after-swap** (e36cafb): `RackI2sOut::end()` cleared the DMA
   channel's INTE bit but begin() only re-enabled it inside the first-claim
   block — after the first swap the DMA sat idle forever. begin() re-arms it
   every call.
5. **The page-update lag** (dc9eb8a + a1ac68d): the UI transport's 512-B
   staging ring only drained while the app ring was non-empty — a 135-B
   snapshot overflowed the 32-B HW FIFO in one pass and the tail (127 B)
   stranded forever. FIX: `ui_uart_tx_service()` (the drain) runs
   unconditionally FIRST in the tx pump, every pass. Plus a 1 s echo
   heartbeat from the rack: the C3's RackLink goes offline after 4 s
   without RX bytes (kOfflineAfterMs) and the offline cycle renders "---"
   and re-probes — the heartbeat keeps the freshness timer fed.

## Infrastructure landed

- **The USB composite**: CDC console (the stdio bridge via a registered
  stdio driver — output flows through the non-blocking dbg ring) + TinyUSB
  MIDI (CFG_TUD_MIDI=1, tud_midi_packet_read → midiHandleEventPacket, the
  same 4-byte packet contract as the Arduino build). The UAC1 capture
  function is PARKED (`usb_capture.cpp/h` complete, not compiled) — Task 1
  of the plan activates it.
- **The 1200-baud BOOTSEL watcher** in the CDC console
  (tud_cdc_line_coding_cb → reset_usb_boot): the documented flash recipe
  works against the Pico build. Without it, the composite CDC ignores the
  touch (the first flash only worked because the board still ran the
  Arduino firmware).
- **The linker fork via the SDK override mechanism**
  (`linker_overrides/memory_extra.incl` = RACK_WINDOW + RACK_LMA regions;
  `section_extra_post_end.incl` = the tc_* overlay + asserts;
  `RAM_LENGTH=0x63000` via pico_set_linker_script_var) — no monolithic
  script duplication. ELF-verified vs the Arduino build (bss parity ±720 B,
  the tc_* hot sets matching).
- **The placement checker** (`scripts/check_ram_placement.py`, cc7a63d):
  Check A the tc_* window placement, A2 the platform hot symbols
  RAM-resident, B flash edges (report-only — the RP2350 does no runtime
  flash writes, the presets live on the C3), B2 the core-1 literal-pool
  scan, C the size accounting. Run: `python3 scripts/check_ram_placement.py
  build/rack.elf`. Gate: Task 2a wires it as a post-link build step.
- **The I2S bank driver hot path RAM-resident** (write16/handoff_/dma_irq_
  via __not_in_flash_func); the transport IRQs (ui/din_uart_irq) too.
  loop1 stays XIP by design (documented in the checker's table).
- **The shared headers carry RACK_PICO_BUILD arms** (midi/ui_link/
  status_led/mod_engine/preset/module_plaits/module_lira8/
  module_dronecaster + the lib ram_code.h re-gates): the Arduino arms are
  byte-identical; `pio run -d rack-rp2350` re-verified green after every
  header edit. `rack_now_ms()` / `ui_uart_*` are per-build shims.

## Gotchas (new this session)

- **The CDC console output goes through the dbg ring** (printf →
  cdc_out_chars → dbg_ring_write): a drop-style writer truncated mod?
  dumps past the 256 B FIFO, and a drain-spin variant wedged core 0
  (no watchdog feed from inside pumpConsole). Non-blocking or nothing.
- **The USB CDC echo verification law**: after any reflash, send a newline
  and retry before trusting the first console reply (stale bytes garble
  it — the AGENTS.md law, hit repeatedly).
- **The C3's log port is backlogged**: the ESP_LOG output over the
  USB-Serial-JTAG (/dev/ttyACM0 on this bench) delivers in delayed
  batches (up to 76 s of backlog). Never timestamp-compare against it
  directly; use the in-log (esp_timer) timestamps.
- **The C3's RackLink goes offline after 4 s without RX bytes**
  (kOfflineAfterMs in RackLink.cpp) and re-probes — each cycle renders
  "---" on the UI. The rack's 1 s echo heartbeat is the fix; anything
  that silences the rack's TX for >4 s re-triggers it.
- **The ACM numbering is NOT stable** (the C3's JTAG takes ttyACM0, the
  rack's CDC wanders ttyACM1↔2 after re-enumerations) — always resolve
  by-id (`usb-rack_Rack_RP2350_*` — the NEW pattern, changed from
  `usb-Raspberry_Pi_Rack_RP2350_*` with the composite device).
- **The touch-then-flash races**: a watchdog reset mid-flash kills the
  BOOTSEL drive; flash attempts must wait for the by-id port AND retry.
- **The dbg ring drops when no console host reads** — the diagnostic dbg
  lines vanish without a reader; don't conclude "didn't happen" from a
  missing line unless a reader was attached the whole time.
- **The scratch-register crumbs are stale across builds**: RACK_HANG_DEBUG
  crumbs survive resets, so a value from an OLDER build confounds the
  read. The live s0/s1 line in `crash` exists for this.
- **esp-idf v6.1.1 on this machine**: `~/esp/esp-idf-v6.1.1/export.sh`;
  wipe `rack-uimcu/build` when switching IDF versions (the CMake cache
  pins the old source path).

## Open items (the phase-2 plan, docs/superpowers/plans/2026-11-23-pico-phase2.md)

1. **Task 1 — the loopback capture rig**: un-park `usb_capture.cpp/h`;
   descriptors extend to CDC+MIDI+UAC1 (interfaces 4/5, iso IN EP 0x84 —
   0x83 is MIDI IN); the tap in RackI2sOut::write16; mirror DSPi's
   descriptor numbers (2-ch 24-bit, wMaxPacketSize 312, 52 frames) — the
   earlier arecord set-params failure used a 16-bit/208 B variant, do NOT
   debug that variant. The host tool `tools/rig_capture.py`. Gate: a 10-s
   capture per module + the benjolin capture vs the test/benjolin_ref
   golden.
2. **Task 2 — the placement checker as a post-link build gate** (2a–2c).
3. **Task 3 — the scripted device suite** (`tools/bench_suite.py`): the
   by-id transport, the liveness sentinel (up= continuity, budget checks,
   stall deltas), preset snapshot/restore, exit codes.
4. **Task 4 — closeout**: the C3 preset persistence re-verify, AGENTS.md
   updates (the rack-pico build recipe + the NEW by-id pattern + the C3
   JTAG console port note), the bench diagnostics revert (the c0/st/hbw
   report fields, the stage tracker, the wire counter, the page-request
   dbg log in ui_link.h), the T4–T6 ear checkpoints with the user.
5. The elements "in=0" observation: the elements meter (exciter_level/
   resonator_level) reads ~0 at idle with factory defaults even after a
   strike decays — the user confirms elements WORKS by ear, so this is a
   metering/idle-state question, not a silence bug. Investigate only if
   the user reports it again.
6. The Arduino build stays last-known-good (`pio run -d rack-rp2350`
   green through every shared-header edit) until the user retires it.

## Machine facts (this Linux box)

- The toolchain: system `arm-none-eabi-gcc` (/usr/sbin), ninja + cmake via
  mise, PICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk
  (SDK 2.3.0, TinyUSB 0.18).
- esp-idf: v6.1.1 (`~/esp/esp-idf-v6.1.1/export.sh`); the C3 flashes via
  `idf.py -p /dev/ttyACM0 flash` (the USB-Serial-JTAG = ttyACM0; its log
  port backlogs up to ~76 s — read with patience).
- The DSPi reference clone: `/home/schrab/Projects/DSPi` (GPL-3.0 —
  pattern-reference only, keep the attribution comments).
- The board by-id: `usb-rack_Rack_RP2350_D04E1EA073701924-if00` (the
  composite CDC+MIDI device; the flash recipe in AGENTS.md works as-is
  with the new pattern).
