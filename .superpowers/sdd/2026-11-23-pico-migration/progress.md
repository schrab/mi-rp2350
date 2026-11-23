# Pico SDK migration — progress ledger (2026-11-23)

Migration of the rack firmware from Arduino/PlatformIO (rack-rp2350/) to
Pico SDK + CMake (rack-pico/). Order (user): migrate platform → working
build → USB audio after. 320 MHz is a hard requirement (the chain
combinations wedge at 307.2). Direct SDK rewrite, no Arduino shim layer.

## Task status

| Task | Status | Commit | Notes |
|---|---|---|---|
| T1 build + linker + RAM_CODE re-gate | DONE | e212470 | pio build SUCCESS + host tests PASS after the re-gates; ELF layout verified vs Arduino (bss parity ±720 B, tc_* hot sets matching, all window asserts pass) |
| T2 console + clock + watchdog/crash | DONE | 6897587 | CDC console alive (printf via the stdio bridge + Serial.* replacements); 320 MHz via rack_pre_clock (VREG 1.25 V + QMI CLKDIV 4/RXDELAY 3) + set_sys_clock_khz(320000) at setup start; watchdog + scratch forensics + crash record ported |
| T3 I2S + benjolin audible | PENDING EARS | cd12ced | DMA bank driver (rack_i2s.cpp) renders ~3090 blocks/s (the exact 48 kHz period), stalls 0; `cc 72 100` lifts in= 0→597 (engine→DAC meter). Ear check owed |
| T4 MIDI + swap | PENDING | — | USB MIDI (CFG_TUD_MIDI=1 + tud_midi_packet_read pump → midiHandleEventPacket) built in; DIN UART ported (IRQ + 256 B ring); swap machinery ported — needs bench |
| T5 UI link + presets + LED + piezo | PENDING | — | ui_link.h SDK transport ported (UART0 921600, IRQ RX ring + SW TX ring); presets/LED/piezo headers armed |
| T6 chain mode | PENDING | — | machinery ported (SPSC pipe rings, chain_b_worker); ear checks at 320 MHz owed |

## The two bring-up root causes (both bench-caught 2026-11-23)

1. **Core-1 harness missing** (6897587): the SDK's multicore trampoline
   runs the entry function ONCE and parks the core on return. The Arduino
   build's runtime provided the setup1()+loop1() harness; the port launched
   setup1 directly — core 1 finished setup1 (poll exited, audio_ready
   observed, DMA_IRQ_0 taken) and went silent: hb stuck at 0, watchdog
   bootloop. FIX: core1_main() = setup1() + loop1() forever.
   ALSO: launch core 1 AFTER the sys-clock raise (the freeze telemetry
   showed the launch must follow the PLL switch).
2. **i2s_out autopull** (cd12ced): the PIO program has no explicit `pull`
   and the SM config set autopull=false — the OSR shifted stale bits
   forever, the TX FIFO never drained, the DMA transfer never completed, no
   completion IRQ; the render stalled after the first 8-buffer bank (hb
   frozen at bank depth + 1). FIX: autopull=true. The scaffold was never
   ear-verified so this was latent.

## Infrastructure landed

- 1200-baud BOOTSEL watcher in the CDC console (tud_cdc_line_coding_cb →
  reset_usb_boot) — the documented flash recipe works against the Pico
  build (the first flash worked only because the board still ran the
  Arduino firmware; soft paths then died until the watcher landed).
- Linker fork via the SDK override mechanism: memory_extra.incl
  (RACK_WINDOW + RACK_LMA), section_extra_post_end.incl (tc_* overlay +
  asserts), RAM_LENGTH defsym — no monolithic script duplication.
- Shared headers carry RACK_PICO_BUILD arms + rack_now_ms()/ui_uart_*_
  shims; the Arduino arms are byte-identical (pio build re-verified after
  every header edit).

## C3 pairing (2026-11-23, later bench round)

- C3 rebuilt + flashed with **esp-idf v6.1.1** on this machine
  (~/esp/esp-idf-v6.1.1; the stale v5.5.4 CMake cache needed wiping) — the
  handoff's open item 5 (preset blob capacity) is now closed on hardware.
- The "---" flicker every ~3 s root-caused from the C3's own logs
  (ESP_LOG over the JTAG console): RackLink's kOfflineAfterMs=4000 expired
  (no RX bytes from the RP2350) -> offline -> page probe -> online; each
  cycle clears s_fullArrived and the UI renders "---" until the probe's
  TELEM snapshot refills. The echo replies flowed (1/s, 0 crc) but did not
  satisfy the freshness timer. FIX (a1ac68d): the RP2350 sends a 1 s UI
  heartbeat; wire-proven via a MIDI-frame diagnostic the C3 logs
  unconditionally. Post-fix: 0 offline events in 15 s.
- Console command output routed through the dbg ring (printf ->
  cdc_out_chars -> dbg_ring_write): the drop-style writer truncated mod?
  dumps past the 256 B CDC FIFO and a drain-spin variant wedged core 0
  (watchdog starved from inside pumpConsole) — both bench-caught.
- 1200-baud BOOTSEL watcher in the CDC console restored the documented
  soft flash recipe against the Pico build.
- Board stable across the session: renders ~3090 blocks/s, stalls 0,
  cc 72 100 lifted in= 0->597 (engine->DAC).

## The page-update lag — root cause + fix (2026-11-23, dc9eb8a)

- The Pico UI transport uses TWO rings: the ui_link app ring (4 KiB) and a
  512-B staging ring ahead of the UART's 32-B HW FIFO. The staging ring
  drained only lazily inside available_for_write()/write_bytes(), which the
  tx pump calls only while the app ring is non-empty.
- A full TELEM snapshot (~135 B wire) overflowed the 32-B HW FIFO in one
  pass: 8 bytes entered, 127 B stranded in the staging ring, the pump (app
  ring empty) never serviced them. Every snapshot stuck its tail; the C3's
  page requests got replies that never arrived; the UI sat at "---" and the
  freshness timer cycled it offline. The 8-B heartbeat keepalives flowed
  fine, which masked the bug (size-dependent loss).
- Instrumentation that found it: the RP2350's wire-byte counter (the drain's
  actual FIFO pushes) + the C3's per-2s RX byte counter + the C3's
  frame-level dispatch log. The 8-B frames parsed, the 135-B frames never
  completed — the tail was stranded, not lost on the wire.
- FIX: ui_uart_tx_service() exported; the pump calls it unconditionally
  FIRST, every pass. Verified: 12/12 one-per-second full snapshots parsed
  on the C3; page-request replies land in ~100 ms.
- The MIDI-frame keepalive diagnostic was reverted to the 3-B echo frame.

## MILESTONE — all modules working (user-confirmed by ear, 2026-11-23)

- The I2S format fix (c8d6f32) accepted: "yes now all fine and benjolin
  sound is much cleaner". Elements + plaits + benjolin verified across
  swaps; the user confirms the rest as well.
- Page switching works with ~500 ms update delay (was seconds-to-never).
- Commits: e212470 (migration) -> 6897587 (harness) -> cd12ced (autopull)
  -> a1ac68d (keepalive+console ring) -> e36cafb (swap IRQ) -> dc9eb8a
  (staging-ring drain) -> c8d6f32 (I2S format) — all pushed to origin.

## Next phase — DSPi adoption candidates (user's ranked list, 2026-11-23)

1. The loopback capture rig (the DSPi pattern): device-vs-golden for every
   module. The parked UAC1 capture (usb_capture.cpp/h) activates; the tap
   = RackI2sOut::write16 (post-Clip16, module-agnostic); needs the
   descriptor extension (CDC+MIDI+UAC1 composite) + an iso IN EP.
2. check_ram_placement.py — the DSPi script ported: nm/objdump stdlib-only,
   verifies the hot symbols' placement (the .tc_* window + .time_critical).
3. The scripted device suite: snapshot/restore + liveness sentinel +
   exit codes over the console.

## Known-open

- Report carries temporary bench fields (c1/c1t/up/tu/st/hbw/mod) from the
  hang hunt — trim at acceptance.
- RACK_HANG_DEBUG back to 0 (the crumbs stay in the tree per the law).
- USB audio capture parked behind RACK_PICO_USB_CAPTURE (phase 3 per the
  user's order); the arecord hw-params negotiation failure from the first
  capture attempt gets investigated then.
- C3 preset blob v6 reflash (esp-idf v6.1.1 on this machine:
  ~/esp/esp-idf-v6.1.1) still owed.
- AGENTS.md build/flash sections still describe the Arduino build; update
  at acceptance (the by-id pattern changed: usb-rack_Rack_RP2350_*).

## Task 1 — loopback capture rig (2026-11-23, phase-2 plan)

- The parked UAC1 driver activated: descriptors now CDC(0/1) + MIDI(2/3) +
  capture AC(4)/AS(5), ITF_NUM_TOTAL 6, iso async IN EP 0x84, 2-ch 24-bit
  packed, 52 frames x 6 B = 312 B (DSPi's ALSA-verified numbers), discrete
  44.1/48 kHz, entity ids 3/4. Built with `RACK_PICO_USB_CAPTURE=1`
  (default ON; `-DRACK_PICO_USB_CAPTURE=OFF` builds without it).
- `usb_capture.cpp`: int16 DAC pairs -> 24-bit packed LE in the servo packet;
  glitch counters zeroed on SET_INTERFACE(alt=1) so they describe the stream,
  not idle time. Tap in `RackI2sOut::write16` (post-Clip16, module-agnostic).
- `rack_engine_rate_hz` set next to every i2s.setFrequency (swap, chain, boot);
  console `capture` command prints rate/overflow/underrun. Host tool
  `tools/rig_capture.py`: arecord S24_3LE + WAV analysis + counter deltas.
- Evidence on the bare Pico2 (SWD + USB, no C3 UI attached): `lsusb -t` shows
  interfaces 4/5 bound to snd-usb-audio, ALSA card `RP2350`, 5/10 s captures
  at 48 kHz parse as stereo 24-bit with RMS ~0.139 FS, peak ~0.24 FS;
  underrun=0, overflow ~6.9k frames from the ~0.15 s arecord startup window
  (constant across 3/5/10 s) — the tool allows 0.2 s of startup drops.
- Rate follows swaps: elements -> 32000, benjolin -> 48000 (`capture` reply).
- SWD flashing faulted mid-erase on this board (FAULT ACK/sector timeout
  with the rack firmware running); BOOTSEL + UF2 copy was used instead.
  SWD still works for halt/status and for the standalone porttest firmware.

## Task 2 — placement checker as a build gate (2026-11-23, phase-2 plan)

- First clean VERDICT: `VERDICT: PASS (with warnings) (0 FAIL, 13 WARN)`
  (`python3 scripts/check_ram_placement.py build-swd/rack.elf`) — heap slack
  11,644 B >= 8,192 B gate, RAM closure 8 functions, 0 flash edges.
- The first run found `setup1()` in FLASH: marked it `__not_in_flash_func`
  (the RAM-resident core-1 harness entry), clearing check A2.
- Checker bug fixed: `sorted_addrs` was used in B2 without being built; it is
  now sorted from `load_symbols()` before the literal-pool scan. B2 then
  flagged the linker's `__irq_set_enabled_veneer` (a BRANCH TARGET from the
  RAM setup1 to flash `irq_set_enabled`, not a dereferenced data pointer) —
  whitelisted `irq_set_enabled` with that rationale.
- Stale symbol name corrected: `usb_capture_push_stereo` has C linkage, so the
  expected name is the unmangled C symbol (it is RAM-resident at 0x20000680).
  The remaining 13 WARNs are `static_ok` names not present (inlined/gone).
- Gate wired as a POST_BUILD custom command on `rack`, skippable with
  `-DRACK_SKIP_PLACEMENT_CHECK=ON`. Verified: a rebuild prints the checker's
  summaries and `VERDICT: PASS` as part of `cmake --build build-swd`.

## Task 3 — scripted device suite (2026-11-23, phase-2 plan)

- `tools/bench_suite.py`: by-id CDC transport with the stale-byte protocol,
  a 2 s-report liveness sentinel (`up=` continuity, max/budget, stall delta),
  preset dump/load/dump snapshot identity, and a full seven-module swap pass.
  Exit 0 = all cases pass, 1 = named failure, 2 = no port.
- Bench run: `python3 tools/bench_suite.py --cases smoke,snapshot,swap` -> VERDICT PASS.
  All seven modules swapped with zero stall deltas and within budget
  (worst: lira8 825/1451 us, benjolin 827/1333 us); preset identity
  v5/1359 B/105 CCs identical before and after the round-trip.
- Both USB/IP attachments (probe 2-1 and rack 2-2) dropped out during an
  early run; the suite now reconnects on serial I/O errors. With the devices
  re-attached, the full pass completed unattended.
