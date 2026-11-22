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
