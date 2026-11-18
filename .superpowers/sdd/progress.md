# SDD Progress Ledger — Elements RP2350 + PCM5102A

Plan: `docs/superpowers/plans/2026-10-30-elements-rp2350.md`
Started: 2026-10-30. Board: RP2350 nano at `/dev/ttyACM0`.

| Task | Status | Committed | Notes |
|---|---|---|---|
| T1 scaffolding | done | 00c29a7 | fallback lib-sync strategy; board id `rpipico2` (plan's `pico2` unknown to platform); see findings below |
| T2 I2S bring-up | done | 3e3fec5 | user confirmed audible 220 Hz tone after wiring + BOOTSEL flash (2026-10-30) |
| T3 engine integration | done | 7140db6 | user confirmed struck-string pings stepping C2-B2 + reverb tail (2026-10-30); Process-time serial report moves to T6 diagnostics; orchestrator fixed Seed() stack overread |
| T4 USB MIDI device | done | bd620a3 | user confirmed MIDI works on hardware (2026-10-30, "tested it already all fine"); device enumerates as "Pico 2 MIDI 1" — product descriptor rename queued to T6; packet indexing corrected vs plan (4-byte packet); orchestrator-reviewed; findings below |
| T5 USB host MIDI | code ready | bd620a3 | user reports "all fine" but explicit host-controller-in-socket confirmation not yet recorded; if not tested: wire USB-A (VBUS→5V, D+→GP14, D-→GP15) and check mount log; PIO-USB vendored in lib/ (+LICENSE added at review), f_cpu 250→240 MHz (PIO-USB needs 12 MHz multiple), host must init BEFORE i2s.begin() (PIO0 fully claimed by pio_usb — 3 SMs verified in pio_usb.c — I2S lands on PIO1); findings below |
| T6 hardening + docs | code ready | | NOT uploaded — user soak test pending (procedure in findings); note stack + CC 28 model selector + stuck-gate watchdog + I2S stall counter + "Elements RP2350" product descriptor + README rewrite; auto-trigger test deleted; findings below |

## Investigation findings

### T1 — build (2026-10-30)

- Command: `pio run -d elements-rp2350` → `SUCCESS`; artifacts
  `.pio/build/elements-rp2350/firmware.uf2` (also `.elf`/`.bin`).
- Platform `maxgerhardt/platform-raspberrypi` @ raspberrypi@1.20.0; earlephilhower
  core package `framework-arduinopico` version 1.60100.0; bundled toolchains
  `toolchain-rp2040-earlephilhower` (arm-none-eabi 16.1.0), `tool-picotool-rp2040-earlephilhower`
  (picotool 2.3.0), `tool-pioasm-rp2040-earlephilhower`.
- **Board id:** plan's `board = pico2` is not a PlatformIO board id in this
  platform (`UnknownBoard`). Used `board = rpipico2` (Pico 2, RP2350, earlephilhower
  core, upload protocol `picotool`). Alternative: `generic_rp2350`.
- **Library strategy:** primary (`build_src_filter` with `+<../STMLIB/src/stmlib_all.cpp>`
  etc.) **failed silently** — PlatformIO ignores filter patterns outside the project
  `src/` dir (only `src/main.cpp` compiled). **Fallback implemented and working:**
  `tools/sync_pio_libs.sh` copies `STMLIB/src → elements-rp2350/lib/stmlib/` and
  `ELEMENTS/src → elements-rp2350/lib/elements/` with all `.cc` files removed
  (`.inc`, headers, `*_all.cpp` wrappers kept); LDF `chain+` compiles the wrappers
  as project libraries. Re-run the script after submodule updates. Submodules untouched.
- `lib_ldf_mode = chain+` (not `off`) is required with `-DUSE_TINYUSB`: the core
  maps `Serial` to `Adafruit_USBD_CDC` whose symbols live in the bundled
  `Adafruit_TinyUSB_Arduino` library, which only LDF can pull in (via
  `#include <Adafruit_TinyUSB.h>` in the sketch). With `off`: undefined
  `Adafruit_USBD_CDC::begin(unsigned long)` + `Serial`.
- `elements::Dsp::kSampleRate/kSrFactor/kIntervalCorrection` are private class
  statics defined nowhere in the library → undefined references as the plan
  predicted. Defined in `src/main.cpp` (same definitions as plan T3) so the
  scaffold links.
- Proof of inclusion: `.pio/build/elements-rp2350/lib0bc/elements/elements_all.cpp.o`
  (410 KB object) + `lib6a7/stmlib/stmlib_all.cpp.o` compiled and linked directly
  into `firmware.elf` (no `.a` archives for the project libs).
  Flash 1.5% / RAM 2.9% because main.cpp doesn't yet reference `Part` (T3 will).

### T1 — upload to /dev/ttyACM0 (2026-10-30)

- Board is a "mini usb rp2350" development board (RP2350 chip), which shipped with
  factory firmware presenting USB descriptor `2e8a:0009` "Betaflight RP2350A" (factory
  USB product string, not a flight-controller board), serial
  `/dev/ttyACM0` (user is in `uucp` group, tty access OK).
- `pio run -t upload` = 1200 bps touch + wait + `picotool load`. The factory
  firmware does NOT reboot to BOOTSEL on the touch; picotool also cannot open the
  running device: `/dev/bus/usb` node is `root:root 0664`, no picotool udev rules
  installed, `sudo` needs a password → upload fails with `Error 249`.
- picotool IS bundled with the platform (no system install needed), but all raw
  USB access (`picotool info/load` outside BOOTSEL) is blocked by the missing
  USB write permission for non-root.
- **Documented procedure (BOOTSEL / UF2 drag-drop, permission-free)** in
  `elements-rp2350/README.md`: unplug → hold BOOTSEL → plug in → drive `RP2350`
  appears → copy `firmware.uf2` onto it. Once our own firmware runs, the core
  supports reboot-to-BOOTSEL (1200 bps touch + double reset via
  `pico_bootsel_via_double_reset`), so `pio run -t upload` should work from then
  on (given a udev rule or sudo for USB write access).

### T1 — I2S API (installed `~/.platformio/packages/framework-arduinopico/libraries/I2S/src/I2S.h`, core 1.60100.0)

- **Stereo write API:** `size_t write16(int16_t l, int16_t r);` EXISTS (also
  `write8(int8_t l, int8_t r)`, `write24(int32_t l, int32_t r)` — 24-bit values
  left-aligned, `write32(int32_t l, int32_t r)`). All "will block until completed".
- **Single-sample overloads:** `write(uint8_t/int8_t/uint16_t/int16_t/uint32_t/int32_t)`
  — each writes ONE sample (L or R alternating) at the configured bit width;
  two successive 16-bit writes fill one 32-bit L/R frame (`_writeNatural` holds
  the first sample in the high half of the word). So the MarvelousMI
  `i2s.write(l); i2s.write(r);` pattern works, and `write16(l, r)` is the
  equivalent one-call form.
- Bulk/raw: `size_t write(const uint8_t *buffer, size_t size)` (Stream) and
  `size_t write(int32_t val, bool sync)` (user packs the 32-bit word).
- **begin():** `virtual bool begin();` (no args) AND convenience
  `bool begin(long sampleRate) { setFrequency(sampleRate); return begin(); }`.
  `bool setFrequency(int newFreq)` separate as usual.
- **setBuffers():** `virtual bool setBuffers(size_t buffers, size_t bufferWords,
  int32_t silenceSample = 0)`. Constraints enforced: `buffers >= 3`,
  `bufferWords >= 8`. Second arg is **32-bit words per buffer**; at 16-bit
  stereo one word == one stereo frame, so `setBuffers(8, 16)` = 8 × 16 = 128
  stereo frames ≈ 4 ms @ 32 kHz — plan comment is correct.
- **setPIO()-style method: DOES NOT EXIST** in this I2S class (relevant for T5
  PIO allocation collisions). No public way to force a PIO block; the library
  allocates a free PIO+SM internally at `begin()` (`PIOProgram *_i2s`). Other
  relevant methods: `setBCLK/setDATA/setDOUT/setDIN/setMCLK`, `setSlave()`,
  `setLSBJFormat()`, `setTDMFormat()`, `setTDMChannels()`, `swapClocks()`,
  `setMCLKmult()`, `setSysClk()`, `getOverUnderflow()/getUnderflow()`,
  `onTransmit()/onReceive()` (interrupt context).
- Constructor: `I2S(PinMode direction = OUTPUT, pin_size_t bclk = 26,
  pin_size_t data = 28, pin_size_t mclk = 25, pin_size_t data_rx = 22)`.
- Also installed and available: `AudioBufferManager`, `AudioOutputBase` (base
  class of I2S), plus ADCInput/PDMInput etc. in the same core lib dir.

### T1 — Adafruit TinyUSB (bundled in framework-arduinopico, library "Adafruit TinyUSB Library" v3.7.7)

- Device MIDI header:
  `~/.platformio/packages/framework-arduinopico/libraries/Adafruit_TinyUSB_Arduino/src/arduino/midi/Adafruit_USBD_MIDI.h`
  - `Adafruit_USBD_MIDI(uint8_t n_cables = 1);`
  - `bool begin(void);` + `bool begin(uint32_t baud)` (baud ignored).
  - **BOTH read APIs exist:** Stream (`int read(); int available(); int peek();`)
    and raw packet: `bool readPacket(uint8_t packet[4]);` /
    `bool writePacket(const uint8_t packet[4]);` (+ `setCables`, `setCableName`).
    T4's plan code (`usb_midi.readPacket(pkt)` with 3-byte payload consumed from
    the 4-byte packet) matches the installed API — note `readPacket` fills the
    full 4-byte USB-MIDI event packet [CIN, b0, b1, b2].
- Host MIDI header (exists):
  `~/.platformio/packages/framework-arduinopico/libraries/Adafruit_TinyUSB_Arduino/src/class/midi/midi_host.h`
  - Host MIDI class driver is enabled: `CFG_TUH_ENABLED 1` and `CFG_TUH_MIDI 1`
    in `src/arduino/ports/rp2040/tusb_config_rp2040.h`; stream API on
    (`CFG_TUH_MIDI_STREAM_API 1`).
  - **API is INDEX-based, not device-address based — differs from the plan's T5
    sketch:**
    - `uint8_t tuh_midi_itf_get_index(uint8_t daddr, uint8_t itf_num);` (returns
      0xFF if not found) — convert daddr → idx first.
    - `bool tuh_midi_mounted(uint8_t idx);` — takes the interface INDEX, not dev_addr.
    - `uint32_t tuh_midi_read_available(uint8_t idx);` (raw bytes available).
    - Packet API: `bool tuh_midi_packet_read(uint8_t idx, uint8_t packet[4]);`,
      `uint32_t tuh_midi_packet_read_n(uint8_t idx, uint8_t *buffer, uint32_t bufsize);`
      (returns bytes, always multiple of 4). There is **no `tuh_midi_read(daddr, ...)`.**
    - Stream API: `uint32_t tuh_midi_stream_read(uint8_t idx, uint8_t *p_cable_num,
      uint8_t *p_buffer, uint16_t bufsize);` — unpacks to raw MIDI bytes (ignores
      CIN); call in a loop until it returns 0 (per header note, bytes left in the
      FIFO can stall subsequent bulk IN). Write side: `tuh_midi_stream_write(idx,
      cable_num, buf, size)` + `tuh_midi_write_flush(idx)`.
    - Callbacks (all take idx, not dev_addr):
      `void tuh_midi_mount_cb(uint8_t idx, const tuh_midi_mount_cb_t *mount_cb_data);`
      (struct has `daddr`, `bInterfaceNumber`, `rx_cable_count`, `tx_cable_count`),
      `void tuh_midi_umount_cb(uint8_t idx);` (spelling: umount),
      `void tuh_midi_descriptor_cb(uint8_t idx, const tuh_midi_descriptor_cb_t *desc_cb_data);`,
      `void tuh_midi_rx_cb(uint8_t idx, uint32_t xferred_bytes);`,
      `void tuh_midi_tx_cb(uint8_t idx, uint32_t xferred_bytes);`.
  - Bundled host examples only `Host/Simple/host_device_info.ino` and
    `DualRole/.../serial_host_bridge.ino` (no MIDI host example in the package).

### T2 — I2S bring-up code (2026-10-30)

- `src/main.cpp` replaced with the 32 kHz I2S sine test (plan T2 sketch, adapted):
  stereo write uses the **verified 2-arg `i2s.write16(s, s)`** (blocking) instead
  of the plan's dual `write()` fallback; init order per plan/ledger facts:
  `setBitsPerSample(16)` → `setBuffers(8, 16)` → `setFrequency(32000)` →
  `begin()` (no-arg form); `begin()` return value printed ("I2S up" / "I2S
  begin FAILED") for the plan's step-3 debug path. Pins BCLK=GP6 (WS auto on
  GP7), DOUT=GP8.
- Kept from T1 scaffold: `#include <Adafruit_TinyUSB.h>` (required — chain+ LDF
  pulls the TinyUSB lib via this include, else `Serial`/`Adafruit_USBD_CDC`
  undefined), `<STMLIB.h>`/`<ELEMENTS.h>` includes and the
  `elements::Dsp::kSampleRate/kSrFactor/kIntervalCorrection` definitions
  (needed for linking; T3 reuses).
- `pio run -d elements-rp2350` → **SUCCESS** (10 s): firmware.uf2/elf/bin,
  RAM 3.1%, Flash 1.7%; LDF graph: I2S @ 2.0 + elements + stmlib +
  Adafruit TinyUSB. Signatures re-verified against installed I2S.h
  (`write16(int16_t,int16_t)`, `begin()`, `begin(long)`,
  `setBuffers(size_t,size_t,int32_t=0)`) — no drift from T1 facts.
- **NOT uploaded** (udev perms, per T1 findings). Acceptance (audible clean
  220 Hz, ±1 Hz on tuner) pending: user wires PCM5102A (BCK→GP6, LCK→GP7,
  DIN→GP8, SCK→GND, XSMT→3V3, VIN→3V3) + flashes `firmware.uf2` via BOOTSEL.
  If silent: check Serial for "I2S up", then XSMT high, then LCK on GP7.

### T3 — engine integration code (2026-10-30)

- `src/main.cpp` replaced with the plan-T3 engine sketch (kept from T1/T2: the
  `elements::Dsp::kSampleRate/kSrFactor/kIntervalCorrection` definitions,
  `#include <Adafruit_TinyUSB.h>` for Serial CDC linking, I2S pins BCLK=GP6 /
  DOUT=GP8, and the `i2s.begin()` return print "I2S up" / "I2S begin FAILED").
- Implemented per plan: static `uint16_t reverb_buffer[32768]` (64 KB),
  `part.Init(reverb_buffer)` + `Seed(&seed=0x1fff7a10, 3)` (exact proven Scarp
  pattern) + `set_easter_egg(false)` +
  `set_resonator_model(RESONATOR_MODEL_STRING)`; default strike/mallet patch
  (envelope_shape 0.1, strike_level 0.8, geometry 0.25, brightness 0.4,
  damping 0.7, position 0.3, space 0.4, rest 0/0.5 per plan);
  `setup1()`/`loop1()` on core 1 gated on `volatile bool audio_ready` set at
  the end of core 0 `setup()`; `perf_state` (gate/note/strength/modulation)
  filled from `volatile` shared MIDI vars each block;
  `part.Process(perf_state, blow_in, strike_in, main_out, aux_out, 16)`;
  `stmlib::Clip16(int32)` → `i2s.write16(l, r)` blocking pacing;
  core 0 `loop()` auto-trigger (strike every 1.5 s, gate held 300 ms, note
  36..47 stepping) + 2 s Serial report of `proc_us_max` vs the 500 µs/block
  budget (16 samples @ 32 kHz). All shared vars (`shared_*`, `audio_ready`,
  `proc_us_max`) are `volatile` at file scope in the single main.cpp TU.
- API verified before compiling: `Part::Process(const PerformanceState&,
  float* blow_in, float* strike_in, float* main, float* aux, size_t n)` at
  `ELEMENTS/src/elements/dsp/part.h:59` — plan's call order is correct.
  `Patch` field names all match `patch.h`. `RESONATOR_MODEL_STRING` at
  `voice.h:52`. `stmlib::Clip16` at `STMLIB/src/stmlib/dsp/dsp.h:131`.
- **Adapted (minimal):** added `#include "stmlib/dsp/dsp.h"` — the plan's
  include set does not pull it in (wrappers include `stmlib/utils/dsp.h`, a
  different file) and `Clip16` would be undefined. Used the verified
  `i2s.write16(l, r)` instead of the plan's dual `write()` fallback (same
  proven T2 path). Init order keeps T2's `setBitsPerSample(16)` →
  `setBuffers(8, 16)` → `setFrequency(32000)` → no-arg `begin()` with return
  check, then `audio_ready = true` AFTER `i2s.begin()` so core 1 never renders
  into an unstarted I2S.
- Build: `pio run -d elements-rp2350` → **SUCCESS** (10.5 s, only main.cpp
  recompiled), zero warnings. **RAM 37.3% (195,616 / 524,288 bytes)** — jump
  from T2's 3.1% is the 64 KB reverb tank + Part/Voice/OminousVoice state;
  **Flash 11.3% (473,224 / 4,190,208 bytes)** — jump from 1.7% is the full
  Elements DSP now referenced (T1's elements_all.o was dead-stripped).
  Artifacts: `.pio/build/elements-rp2350/firmware.uf2` (+ `.elf`/`.bin`/`.bin.signed`).
- **NOT uploaded** (per plan: user flashes via BOOTSEL, see
  `elements-rp2350/README.md`).
- **Expected serial output on the user's hardware check** (115200 baud, USB-C
  CDC): at boot `I2S up` then `Elements engine up (auto-trigger test)`;
  thereafter every 2 s exactly one line
  `max Process: <N> us (budget 500 us)` where <N> is the worst-case
  `Part::Process` block time (16 samples) since the last report — acceptance
  is <N> comfortably below 500 and audible struck-string pings stepping up
  note-by-note every 1.5 s (C2..B2, notes 36..47), reverb tail from space 0.4.
  If audio is silent but the report prints: check XSMT high, LCK on GP7 (same
  T2 debug order).

### T4 — USB MIDI device code (2026-10-30)

- Files: `elements-rp2350/src/midi.h` (NEW: parser + extern shared-state
  decls), `elements-rp2350/src/main.cpp` (extended).
- **Packet semantics VERIFIED — plan's parser was WRONG and is corrected:**
  `Adafruit_USBD_MIDI::readPacket(uint8_t packet[4])`
  (`src/arduino/midi/Adafruit_USBD_MIDI.h:60` → `tud_midi_packet_read` →
  `tud_midi_n_packet_read`, "Read an event 4-byte packet") fills the FULL
  4-byte USB-MIDI event packet:
  `pkt[0] = (cable<<4) | CIN`, `pkt[1] = MIDI status`, `pkt[2] = data1`,
  `pkt[3] = data2` (0 for 2-byte messages). Confirmed by the packing side in
  `src/class/midi/midi_device.c` (status byte lands at buffer[1]). The plan
  indexed `pkt[0..2]` as `[CIN, d1, d2]` over a 3-byte buffer; implemented
  parser uses `cin = pkt[0]&0x0F`, `d1 = pkt[2]&0x7F`, `d2 = pkt[3]&0x7F`,
  cable nibble ignored (1 cable). Note-on with velocity 0 → note-off.
  Pitch bend: pkt[2]=LSB, pkt[3]=MSB, 14-bit centered 8192 → ±2 semitones
  around `base_note`. CIN outside 0x8..0xE ignored.
- **Device init ordering:** the core calls `TinyUSB_Device_Init(0)` in
  `main()` BEFORE `setup()` (`cores/rp2040/main.cpp:148` →
  `TinyUSBDevice.begin(0)` → `tusb_init`, pull-up already up), while
  `usb_midi.begin()` only does `TinyUSBDevice.addInterface()`. Adopted the
  bundled example `examples/MIDI/midi_test/midi_test.ino` guard: after
  `usb_midi.begin()`, if `TinyUSBDevice.mounted()` → detach/delay(10)/attach
  so a host that already configured the CDC-only device re-enumerates and
  sees the MIDI interface. Product string: "Elements RP2350" (via
  `setStringDescriptor`). `CFG_TUD_MIDI 1` is default
  (`tusb_config_rp2040.h:94`). Only the packet API is used (header warns
  Stream + packet share internal state — do not mix).
- Arrangement: midi.h declares `extern volatile float shared_note/
  modulation/strength`, `extern volatile bool shared_gate`,
  `extern elements::Part part;`; main.cpp holds the definitions (dropped
  `static` on exactly those five). `static int16_t base_note` stays in
  midi.h (internal linkage; single TU includes it). CC 28 →
  `resonator_modulation_frequency` per plan; resonator model selector
  deferred to T6.
- loop(): drains `usb_midi.readPacket(pkt[4])` each pass →
  `midiHandleEventPacket(pkt)`; T3 auto-trigger kept behind
  `static const bool kAutoTriggerTest = false` (compiles away; deleted in
  T6); the 2 s `max Process` serial report stays always-on.
- Build: `pio run -d elements-rp2350` → **SUCCESS**, zero warnings (7.7 s
  incremental). RAM 37.3% (195,656 / 524,288, +40 B vs T3), Flash 11.3%
  (475,352 / 4,190,208, +2,128 B vs T3 — MIDI class + CDC already present).
  Artifacts: `.pio/build/elements-rp2350/firmware.uf2`. **NOT uploaded**
  (user flashes via BOOTSEL, `elements-rp2350/README.md`).
- **User test procedure (flash → verify):**
  1. BOOTSEL-flash `firmware.uf2` (unplug → hold BOOTSEL → plug USB-C →
     copy the .uf2 to the `RP2350` drive).
  2. Board enumerates as CDC serial + USB MIDI. Check: `aseqdump -l` (or
     DAW/Ardour MIDI prefs) shows client "Elements RP2350", port
     "Elements RP2350 MIDI 1". Serial monitor still works at 115200 and
     prints `Elements engine up (USB MIDI device)` + the 2 s Process report.
  3. Send notes from the PC: easiest CLI check is
     `amidi -l` (find the Elements port) then
     `amidi -p <port> -S "90 3C 60"` (note on, C4, vel 96) and
     `amidi -p <port> -S "80 3C 00"` (note off) — expect a struck-string
     ping at C4 that decays; with a sustained sender (VMPK/DAW held note)
     the voice rings until note-off. Velocity maps to strength
     (0.4 + 0.6·vel/127).
  4. CC checks (any CC sender / DAW lane): 23 geometry, 25 damping,
     26 position, 27 space (reverb), 11 strength, 1 modulation — all
     audible on the next strike. Pitch bend wheel → ±2 semitones.
  5. After confirming MIDI works: set nothing — T6 deletes
     `kAutoTriggerTest` and the whole block.

### T5 — USB host MIDI code (2026-10-30)

- Files: `elements-rp2350/src/main.cpp` (extended), `elements-rp2350/lib/pio_usb/`
  (NEW: Pico-PIO-USB v0.7.3 vendored, src only + minimal library.json),
  `elements-rp2350/platformio.ini` (f_cpu 250→240 MHz + comment).
- **PIO-USB library was NOT present anywhere in the packages** (`find
  ~/.platformio/packages -name 'pio_usb.h'` = empty): the port config
  `tusb_config_rp2040.h:46` only sets `CFG_TUH_RPI_PIO_USB 1` when
  `__has_include("pio_usb.h")`, else it falls back to MAX3421 host. Without
  the lib, `USBHost.configure_pio_usb()` compiles but returns false and no
  PIO host driver exists. Tried `lib_deps = github sekigon-gonnoc/Pico-PIO-USB`
  first: its `library.properties` `depends=Adafruit TinyUSB Library` made
  PlatformIO install a duplicate project-level TinyUSB next to the core's
  force-linked bundled copy (link-failure risk) → **switched to vendoring**
  `lib/pio_usb/` (clean build: LDF graph = `pio_usb @ 0.7.3` + the one
  framework-bundled `Adafruit TinyUSB Library`; 3.7.7 per its
  library.properties; LDF's "3.7.4" display is platform metadata noise).
  hcd glue compat verified: bundled TinyUSB 3.7.7's `hcd_pio_usb.c` uses
  `pio_usb_host_init`/`pio_usb_bus_get_line_state`/`endpoint_t`/
  `root_port_t` fields, all present in 0.7.3.
- **f_cpu 250 MHz → 240 MHz (mandatory):** the bundled DualRole
  `usbh_helper.h` hard-errors unless `cpu_hz % 12000000UL == 0` — PIO-USB
  bit-bangs FS USB (48 MHz PIO clocks via fractional clkdiv,
  `pio_usb_host.c:87-99`); 250 is not a 12 MHz multiple. 240 MHz = 12·20.
  I2S sample clock is PLL-derived → audio stays exactly 32 kHz; core-1 DSP
  headroom drops ~4% only. main.cpp has a runtime guard that Serial-prints
  an error (no hang) if someone reverts f_cpu.
- **Init pattern (adapted from `examples/DualRole/CDC/serial_host_bridge`
  + its `usbh_helper.h`):** `pio_usb_configuration_t pio_cfg =
  PIO_USB_DEFAULT_CONFIG; pio_cfg.pin_dp = 14;` (DPDM pinout → D- = GP15)
  → `USBHost.configure_pio_usb(1, &pio_cfg)` → `USBHost.begin(1)`, each
  return checked with Serial error path; `USBHost.task()` in core 0
  `loop()` (= `tuh_task_ext(UINT32_MAX, false)`, a non-blocking event-queue
  drain on the OS-less port). The example runs host init/task on core 1 to
  spread load ("To run USB SOF interrupt in core1, call this init in
  core1"); here core 1 renders audio, so the host lives on core 0 — its
  work is a 1 ms SOF alarm-pool timer ISR (`alarm_pool_create(2, 1)`,
  alarm 2; the core's device task uses the default pool alarm 3) that
  bit-bangs transfers, plus the loop-side task(). Device stack keeps
  working unchanged: the core services `tud_task()` via its own soft-IRQ +
  alarm (`cores/rp2040/USB.cpp:529`), independent of loop().
- **PIO/I2S allocation — REAL COLLISION avoided by init order, not by
  config:** pio_usb hard-claims exact resources (`pio_get_instance(cfg)`
  + `pio_sm_claim` panics on taken SM; `dma_claim_mask(1<<tx_ch)` for DMA0;
  no flexible allocator) and loads 5 (tx@0) + 10 (nrzi decoder) + 17 (edge
  detector = EOP SM) = **all 32 PIO0 instruction slots**. The I2S library
  has no setPIO() (T1 finding) but its PIOProgram flexibly picks a free
  PIO+SM+program-space at begin(). Therefore setup() inits the USB host
  FIRST (virgin PIO0 → claims SMs 0/1/2, fills instruction memory, DMA0,
  alarm 2) and `i2s.begin()` AFTER → lands on PIO1 (GP6/7/8 reachable from
  any PIO on RP2350; PIO0 keeps 1 free SM but zero program space, so the
  allocator skips it). Reversed order = boot panic in pio_sm_claim.
- **Host MIDI polling (packet API, index-based per T1 findings):**
  `tuh_midi_mount_cb(uint8_t idx, const tuh_midi_mount_cb_t*)` /
  `tuh_midi_umount_cb(uint8_t idx)` defined in `extern "C"` (they override
  weak C syms — nm confirms strong `T` in firmware.elf); mount stores idx
  in a 4-slot registry (dedup + capacity guard) and prints
  `Host MIDI mounted idx=%u daddr=%u cables=%u` (cables = rx_cable_count);
  umount removes it. `pollHostMidi()` iterates tracked idxs,
  `tuh_midi_mounted(idx)` guard, drains `tuh_midi_packet_read(idx, pkt[4])`
  in a while loop → same `midiHandleEventPacket()` as the device path
  (packet layout identical: [cable<<4|CIN, status, d1, d2]). Callbacks fire
  from USBHost.task() (core 0 loop context) — no concurrency with the
  poller. Stream API untouched (avoid FIFO-stall semantics).
- Build: clean rebuild after `pio run -t clean` → **SUCCESS** (52.8 s).
  RAM 40.2% (210,960 / 524,288 B, +15.3 KB vs T4 — host stacks: hub/HID/
  CDC/MIDI pools + tuh event queue), Flash 11.6% (486,868 / 4,190,208,
  +11.5 KB). Linked proof: `pio_usb_host_init`, `sof_timer`,
  `tuh_midi_packet_read_n`, strong `tuh_midi_mount_cb/umount_cb` in
  firmware.elf; `hcd_pio_usb.c.o` compiled (native `hcd_rp2040.c.o` is an
  empty TU via #if guards). Only warnings: upstream pio_usb
  `usb_crc.h:12` "inline function given attribute noinline" (-O3 cosmetic;
  attribute wins, left unpatched). **NOT uploaded** (user flashes via
  BOOTSEL, `elements-rp2350/README.md`).
- **User test procedure (wire → flash → verify):**
  1. Wire the USB-A socket: VBUS → 5V pin, D+ → GP14, D- → GP15, GND →
     GND (shield to GND). VBUS is plain 5 V rail, not GPIO-switched.
  2. BOOTSEL-flash `firmware.uf2` (procedure in README), USB-C still in
     the PC.
  3. Boot serial (115200) should show: `I2S up` (host init runs before
     I2S but prints nothing on success), then
     `Elements engine up (USB MIDI device + PIO-USB host)`; no
     `PIO-USB configure FAILED` / `USB host begin FAILED` / CPU-clock
     error lines.
  4. Plug the MIDI controller into the USB-A socket → expect
     `Host MIDI mounted idx=0 daddr=1 cables=1` within ~2 s.
  5. Play notes / turn CCs on the controller → same audible behavior as
     the PC path (notes, velocity→strength, CC map, pitch bend).
  6. Unplug → `Host MIDI unmounted idx=0`; replug re-mounts.
  7. Simultaneity check: PC (DAW/amidi to "Elements RP2350") and the
     controller must both drive the synth; USB-C must still enumerate as
     CDC serial + USB MIDI device while the controller is attached.
  8. If the controller doesn't mount: check VBUS 5 V first (most common),
     then D+/D- not swapped (D+ MUST be GP14, D- GP15), then look for
     repeated `USB host begin FAILED`/configure errors on serial.
- Left open / risks: (a) PIO-USB at 240 MHz is the Adafruit-validated path
  but unproven on THIS board until the user test; watch for mount timeouts
  or CRC-noise (would show as flaky mount/unmount). (b) The 1 ms SOF alarm
  ISR bit-bangs bus transfers on core 0 — the device-side CDC/MIDI can
  jitter by a few hundred µs during host traffic (audio is on core 1 and
  unaffected). (c) If a future lib needs PIO0 program space, PIO0 is now
  fully occupied by pio_usb (only PIO1/PIO2 remain for other PIOProgram
  users). (d) Bus-powered controllers draw from the board's 5 V rail —
  current-hungry hubs/devices need external VBUS feed.

### T6 — hardening (2026-10-30)

- **Resonator-model thread-safety VERDICT (evidence-checked, hypothesis
  overturned):** `Part::set_resonator_model()` is SAFE even mid-Process —
  it is a pure inline enum store (`part.h:83` → `resonator_model_`, aligned
  32-bit, atomic on ARMv8-M, same class as the existing `shared_*` floats).
  `Part::Process()` re-pushes it into the voice EVERY block
  (`part.cc:181` → `voice_[i].set_resonator_model(...)`), and
  `Voice::set_resonator_model` is likewise a pure store (`voice.h:78–80`).
  On a model change nothing is reallocated or cleared and no buffers are
  touched: `Voice::Process()` merely BRANCHES on `resonator_model_`
  (`voice.cc:212` modal path vs `voice.cc:223–226` string path) between
  filter states that are all `Init()`ed once in `Voice::Init()/
  ResetResonator()`. Worst case of a direct core-0 write would be the new
  model taking effect at the next block boundary.
- **Implementation kept the pending-slot pattern anyway (per plan T6):**
  core 0's CC 28 handler stores 0..2 in `volatile int8_t
  pending_resonator_model` (extern in midi.h, defined in main.cpp, -1 =
  none); core 1's `loop1()` applies `part.set_resonator_model()` at the top
  of the loop, before `Process()`, at most one change per block, and resets
  to -1. Rationale: keeps the change landing strictly between two blocks
  regardless of future Part setters, and preserves the core1-renders/
  core0-controls discipline. Known (accepted) race: a CC arriving inside
  core 1's few-instruction read→clear window could be dropped — irrelevant
  for human/DAW CC streams and consistent with at-most-one semantics.
  CC 28 mapping: 0/1/2 → RESONATOR_MODEL_MODAL/STRING/STRINGS (enum order
  verified `voice.h:50–54`), >2 ignored, re-send re-selects. The old CC 28 →
  `resonator_modulation_frequency` mapping is removed (param now unmapped —
  randomized per device by `Part::Seed()`).
- **Product-descriptor API:** `TinyUSBDevice.setProductDescriptor(const
  char*)` — `Adafruit_USBD_Device.h:126`, impl `Adafruit_USBD_Device.cpp:133`
  just swaps `_desc_str_arr[STRID_PRODUCT]`; the array default is the board's
  `USB_PRODUCT` ("Pico 2") set in the device constructor (.cpp:203). The
  string callback resolves the pointer per GET_DESCRIPTOR(String) request,
  so the rename takes effect at the next descriptor read — called as the
  FIRST line of setup(), before the detach/attach re-enumeration guard, so
  the host picks it up on re-enum. (Note: the pre-existing
  `usb_midi.setStringDescriptor()` only renames the MIDI INTERFACE/port,
  not the device — kept as the port name.) Confirmation of the visible name
  is pending the user's next flash (re-enumeration).
- **Note stack (last-note priority):** replaces `static int16_t base_note`
  with `struct NoteStack` (static in midi.h; midi.h included ONLY by
  main.cpp — verified by grep — so internal linkage is safe). Top of stack
  = newest note = `shared_note`; gate drops only when the stack empties;
  duplicate note-on moves to top (no dup); stack full (8) drops the oldest;
  note-off of a non-top note leaves pitch/gate untouched; bend offset is
  remembered and re-applied over the current top note via `applyNote()`
  (48.0 base while idle). All handlers still core-0-only (device + host
  packets both drain in core 0's loop).
- **Stuck-gate watchdog:** `last_note_event_ms` (static in midi.h) stamped
  by `midiNoteOn/Off`; core 0's loop forces `shared_gate = false` + clears
  the stack if gate is true with no note event for > 30 s, logging once per
  incident ("WARNING: gate stuck >30s ..."). Safe because gate can only
  become true inside midiNoteOn, which re-stamps the timestamp.
- **I2S stall counter + threshold rationale:** loop1 writes exactly one
  16-word DMA buffer per iteration (16 × write16 = 16 frames) and buffers
  free every 500 µs (16 frames @ 32 kHz), so the write block normally waits
  at most ~500 µs for the next buffer-free. Threshold 900 µs = expected
  ~500 µs + 400 µs slack: exceeding it means no buffer freed for ~2 block
  periods → the I2S output genuinely stalled (DMA/PIO wedged or clock stop).
  `i2s_stall_count` is cumulative (core 1 writes, core 0 prints) with a
  per-report delta, in the same single 2 s line as max Process:
  `max Process: N us (budget 500 us) | i2s stalls: T (+d)`.
- Removed: the whole `kAutoTriggerTest` block + flag (MIDI confirmed on
  hardware in T4/T5).
- Build: `pio run -d elements-rp2350` → **SUCCESS** (10.2 s incremental,
  zero warnings). RAM 40.2% (210,984 / 524,288 B, +24 B vs T5 — note stack +
  pending slot), Flash 11.6% (487,344 / 4,190,208 B, +476 B vs T5 — watchdog,
  stall metering, descriptor call). Artifacts:
  `.pio/build/elements-rp2350/firmware.uf2`. **NOT uploaded.**
- **README** (`elements-rp2350/README.md`) rewritten: full PCM5102A +
  USB-A wiring tables, behavior section (note stack / bend / velocity /
  watchdog), CC map (CC 28 = model), serial-diagnostics legend, build notes
  (rpipico2 board id, 240 MHz PIO-USB rationale, sync_pio_libs.sh incl. the
  -O3 library.json regeneration, PIO-USB-before-I2S init order,
  -DUSE_TINYUSB/LDF chain+ requirement, Dsp statics), BOOTSEL upload
  procedure + `pio run -t upload` caveats (udev rule for `2e8a:*`), and the
  soak-test checklist.
- **User soak-test procedure (T6 acceptance, ≥ 30 min):**
  1. BOOTSEL-flash `firmware.uf2` (procedure in README) with the MIDI
     controller in the USB-A socket and USB-C in the PC.
  2. Play from the hardware controller for 30+ min: sequenced or jammed
     notes (include legato intervals and re-pressed held notes), some CC
     movement, pitch-bend riding.
  3. Watch serial (115200): `max Process` stable well under 500 µs;
     `i2s stalls` stays `0 (+0)`; no repeated `Host MIDI mounted/unmounted`
     without touching the cable (mount flaps = PIO-USB signal/power issue);
     no audio drops, no crash/hang.
  4. Confirm the rename: `aseqdump -l` / `lsusb` shows "Elements RP2350"
     (effective after this flash's re-enumeration — the one open
     confirmation for item D).
  5. Confirm CC 28 audibly switches modal/string/strings (0/1/2; a synth
     pad or DAW lane sending raw bytes `B0 1C 00/01/02` works), and that
     legato matches the note-stack description (second note while holding
     the first → pitch follows the newest; release → falls back).
  6. Optional watchdog check: unplug the controller while holding a note →
     within 30 s the serial log shows the stuck-gate warning and the voice
     releases.

### T1 — review fix: -O3 was silently defeated (2026-10-30)

- The earlephilhower core appends `-Os` AFTER the project `build_flags`
  (`framework-arduinopico/tools/platformio-build.py:150`), so project-level
  `-O3` lost by last-wins and the DSP compiled at `-Os`.
- `extra_scripts` cannot fix this (tested — script runs but flags don't reach
  compile lines). **Working fix:** per-library `library.json` with
  `"build": {"flags": ["-O3"]}` — library flags are appended after the core's
  `-Os`. `tools/sync_pio_libs.sh` now (re)generates these on every sync.
- Verified compile lines after fix: `elements_all.cpp` / `stmlib_all.cpp` get
  `-O3 -Os -O3` (effective `-O3`); `src/main.cpp` stays `-O3 -Os` (effective
  `-Os`, fine for control code).
- Upload note: the core's signing step references `src/private.key` via
  signing.py — build succeeds without the key present; `.bin.signed` is
  generated alongside `.uf2`.

### Status LED — WS2812 PIO fixes (2026-10-30)

- **Symptom:** boot flashed 3x white, then the LED stuck on constant blue
  and ignored notes/CC; audio and MIDI stayed healthy.
- **Bug 1 — frame alignment:** `showColor()` packed GRB into the low 24 bits,
  but `ws2812_program_init()` configures the SM with shift-left + 24-bit
  autopull, so the PIO transmits the top 24 bits. White became violet and
  blue-only frames became black. Fixed by left-aligning the word (`<< 8`).
- **Bug 2 — SM stolen by I2S:** the WS2812 used raw SDK PIO calls and never
  claimed PIO2 SM0. Arduino's `PIOProgram` allocator (used by I2S) scans
  PIO2 first and claimed the unclaimed SM0 during `i2s.begin()`, reconfiguring
  it for I2S and freezing the LED on its last boot color. Fixed with
  `pio_sm_claim(pio2, 0)` in `StatusLed::Init()` before `pio_add_program()`;
  I2S now lands on PIO2 SM1.
- **Boot diagnostic:** `PIO2 SM0 claimed=... pc=... (ws2812 @...) | SM1 ...`
  after `i2s.begin()` confirms SM0 is still claimed and its PC is inside the
  WS2812 program.
- **Hardware confirmation (2026-10-30):** user reports 3 white boot flashes,
  green idle, blue on notes, audio working. LED mapping: blue = gate,
  red = exciter, green = resonator, dim green = idle.
- **Files:** `elements-rp2350/src/status_led.h` (both fixes + `Offset()`),
  `elements-rp2350/src/main.cpp` (boot diagnostic), `elements-rp2350/README.md`
  (Status LED section + boot log legend).
- Build: `pio run -d elements-rp2350` → SUCCESS; RAM 40.2% / Flash 11.6%;
  `firmware.uf2` flashed by the user via BOOTSEL.

### Status LED — simplified mapping + BOW audit (2026-10-30)

- **User spec:** dark when idle, amber when a note is gated, 3 green blinks
  when a USB-A MIDI controller mounts.
- Removed: dim-green idle, red exciter, green resonator, CC 28 white pulse,
  single green boot-OK flash. Kept: 3 white boot self-test flashes and the
  boot-error red/green blink.
- `StatusLed::Poll(bool gate)` now only needs the gate. The controller blink
  is triggered from `tuh_midi_mount_cb` via `NotifyControllerConnected()` and
  has priority over the note color. 3 × 150 ms on/off.
- **Meter blend (follow-up):** the exciter and resonator meters are blended
  additively into the amber gate color. Gate = amber, exciter = turquoise
  (original `LED1` on PA8/TIM1_CH1), resonator = amber (user preference;
  the original `LED2` on PA11/TIM1_CH4 was rose/fuchsia, but the fuchsia
  tint was rejected). Both meters go through the original
  `lut_db_led_brightness` 54 dB curve; the gate amber base is 180/255 so the
  meter channels can shift the hue without clipping.
  Core 1 publishes `part.exciter_level()` / `part.resonator_level()` as
  volatile floats after `Process()`; core 0 reads them in `Poll()`.
- Re-enabled the upstream exciter meter in `voice.inc` (it had been commented
  out), so `part.exciter_level()` is live again. The LED inherits the original
  ~31 ms exciter / ~62 ms resonator release tails.
- **BOW audit:** the BOW path is a verbatim copy of upstream Elements
  (`pichenettes/eurorack`). `voice.inc`, `exciter.inc`, and `part.inc` diff
  clean against upstream except for `kSampleRate` → `Dsp::getSr()`, the
  commented-out exciter meter, and the port's reverb-time/freeze tweaks.
  - BOW = `EXCITER_MODEL_FLOW` with fixed meta `parameter_ = 0.7`; its
    contribution is `bow_buffer * envelope * bow_level * 0.125 * accent`.
    BLOW = granular sample player, contribution `blow_buffer * envelope`
    with `blow_level` capped at 0.4. At the same level BOW is ~10 dB quieter
    than BLOW — original behavior, not a port bug.
  - BOW timbre is a low-pass cutoff multiplied by
    `0.4 + 0.6 * resonator_brightness`; at timbre 0 the cutoff is 0, so the
    exciter is nearly silent. Also original.
  - BOW and BLOW are both noise-based and can sound similar; BOW is a
    continuous flow, BLOW is granular. Original Elements behaves this way.
- **Metering artifact:** `voice.inc` comments out the exciter meter, so
  `part.exciter_level()` is always 0. The old red LED could never light; the
  new mapping does not depend on the meter.

### USB host — re-enumeration watchdog (2026-10-30)

- **Symptom:** SMC-PAD mounted once (3 green blinks, notes played); after a
  power off/on with the cable still attached it was never detected again.
  Powering the pad on BEFORE plugging the cable always enumerated fine. Not a
  cable/wiring issue (user reproduced without touching wires; VBUS unconnected
  — the pad is self-powered, only D+/D-/GND wired).
- **Diagnosis (parallel serial monitoring on /dev/ttyACM0 alongside MIDI —
  works fine):** power-off produces a clean disconnect
  (`Host MIDI unmounted idx=0`); power-on with the cable attached produces
  ZERO serial output — no mount, no error. TinyUSB gives up on enumeration
  after 3 failed control-transfer attempts (`usbh.c process_enumeration`,
  ATTEMPT_COUNT_MAX=3) and never retries; PIO-USB connect detection is
  edge-based (`!root->connected` + idle line), so once the failed attempt
  leaves the root port marked connected with nothing mounted, no new CONNECT
  event can ever fire. Root cause: the pad's USB pull-up appears before its
  firmware is ready to answer control transfers.
- **Fix:** `elements-rp2350/src/usb_host_watchdog.c` (NEW) + main.cpp loop
  poll. C accessors to the PIO-USB root port (`pio_host_line_idle/
  connected/force_reconnect`) — MUST stay a .c file: `pio_usb_ll.h` declares
  `pio_usb_root_port` etc. without `extern "C"`, so including it from C++
  emits mangled references that don't link against the C-compiled library.
  Every 4 s: if the root-port line is idle (device electrically present) but
  nothing is mounted → `root->connected = false` → the 1 ms SOF frame loop
  re-fires the CONNECT interrupt → TinyUSB re-enumerates from scratch.
- **"Nothing mounted" gating (device-level):** new strong overrides of the
  weak `tuh_mount_cb`/`tuh_umount_cb` (usbh.c device-level callbacks) keep
  `host_device_mounted`; the watchdog only kicks when it is false — a foreign
  non-MIDI device never triggers re-enumeration spam. Safe ordering: a
  healthy enumeration always completes (< ~1.5 s) before the first 4 s check;
  a kick during a wedged enumeration hits tuh_task's duplicated-attach branch
  (abort + restart), healing that case too.
- **Diagnostics:** 2 s report extended with `| host: conn=%u idle=%u dev=%u
  midi=%u` (root-port connect flag, line-idle flag, device-mounted, tracked
  MIDI idx count); device-level `USB device (un)mounted daddr=%u` lines added.
- **Live verification (2026-10-30, agent flashed via 1200-baud touch + UF2
  copy):** serial captured a real power cycle healing itself — unmount on
  power-off, then `USB host watchdog: line idle, nothing mounted - forcing
  re-enumeration` followed by `Host MIDI mounted idx=0 daddr=1 cables=3`
  within ~4 s of power-on. The exact previously-stuck scenario now self-heals.
- Build: `pio run -d elements-rp2350` → SUCCESS. RAM 40.2%, Flash 11.7%
  (+3 KB vs T6 — watchdog + host device tracking + report tail).

### Ominous mode — CC 29 easter-egg switch (2026-10-30)

- **Runtime switching confirmed safe by code audit:** `easter_egg_` is only
  branched on per block inside `Process()` (`part.inc:161` voice select,
  `:211` soft-limit skip, `:232` metering) and `ominous_voice_[i].Init()`
  runs unconditionally in `Part::Init()` (`part.cc:66`) — both voice sets
  stay initialized; the flag merely redirects the next block. Boot-only
  (the original's BOW-at-power-up) is unnecessary. Correction to an earlier
  chat's claim: `part.inc:211-232` is soft-limit/metering, not a
  tube-resonator swap — the dark FM character is inside `OminousVoice`
  (`ominous_voice.cc`, FM/FBFM with `lut_fm_frequency_quantizer`), which
  reuses the resonator patch fields (damping/brightness/geometry) its own
  way. This port's `Patch` has no `easter_egg` field (the original
  persisted the flag in patch storage) — here it is a plain RAM bool, so
  persistence doesn't exist and boot-only would have meant power-cycling to
  change modes.
- **Implementation (T6 pending-slot pattern):** CC 29 ABSOLUTE semantics —
  0 = normal, 1..127 = ominous, deliberately not a toggle (the panel cannot
  read synth state, a toggle would desync knob vs engine). Core 0's
  `midiControlChange` writes `volatile int8_t pending_easter_egg` (0/1, -1
  idle); core 1's `loop1()` applies `part.set_easter_egg()` at the top,
  between render blocks, and resets to -1. Switch behavior: the ringing
  voice's tail cuts (separate voice state, no crossfade), CC 28 is a no-op
  while ominous, knobs are reinterpreted as FM op amounts. No LED change
  (kept minimal per user preference).
- **Panel:** `OMINOUS` knob added to the right section (row 4, under
  BRIGHTNESS) — snapped 2-state control (NORMAL/OMINOUS), sends 0/1,
  `def:0` so page-load sync sends CC 29 = 0 (matches firmware default).
  **Interaction fix (user-reported: knob wouldn't turn back):** the wheel /
  arrow-key handlers step 1/127, but the 2-state snap is `Math.round(v)` —
  from v=1 one notch gives 0.992 → rounds back to 1, needing ~64 notches to
  escape (the 3-way MODE knob had the same defect, ~33 notches). Snapped
  selectors now step one full position per notch/key (`stepOf()`), and the
  OMINOUS drag scale is 100 px full travel (vs 200 px) so flipping back is
  quick. MODE behavior otherwise unchanged.
- **README:** CC map row filled (`29 ominous / 28 model`) + CC 29 semantics
  paragraph.
- Build: `pio run -d elements-rp2350` → SUCCESS; flashed via 1200-baud
  touch; user audible confirmation of the ominous mode pending.

### USB host — board freeze on controller attach (2026-10-31, UNRESOLVED — handed off)

- **Symptom:** any MIDI controller (SMC-PAD JieLi BR25 + a second controller)
  attaching to the PIO-USB host port freezes the BOARD (core 0 dead, serial
  stops, LED frozen; core 1 clocks noise to the I2S) within seconds, at/around
  the first enumeration SETUPs. EP0 log: GET_DESCRIPTOR sent, result 00 (no
  device handshake) every attempt. `max Process` creeps 380→520 µs over a
  session (budget 496) with OMINOUS inactive. **Controller-independent.**
- Full handoff with variant matrix, hardware facts (chip rev 3 E9 gate, loop-
  side line-read race; earlier "no-VBUS wait state" hypothesis disproven since
  SMC-PAD is self-powered and standard 5V USB keyboards fail identically), and
  the next-steps plan (WDT + scratch-register crash capture): **`docs/HANDOFF-usb-host-freeze.md`**
- Diagnostic WIP preserved on branch `usb-host-jieli-debug` (pre-rework
  pio_usb variant, EP0 ring, LED breadcrumbs, debounce).
- Fixes that SURVIVED review and are kept: no loop-side line-state reads
  (E9-dance race with the SOF ISR), watchdog backoff (reset storms wedge
  fragile chips — documented in open-smc-pad usb-br25.md), stock E9 gate for
  chip rev 3 (forcing the dance breaks reads on this board).
- Board left on the diag build; pad unplugged; pad verified healthy on a
  laptop (enumerates, 12M FS) after the session.

### USB host — root cause split + final fix set (2026-10-31 night, elements-clone)

- **Board freeze ROOT CAUSE (confirmed on hardware):** core 0 overran its 4 KB
  half of the default shared dual-core stack at attach (TinyUSB enumeration +
  blocking waits + mount callbacks + Serial.printf with the SOF alarm ISR,
  TinyUSB device task and I2S DMA interrupt nesting on top) — killing
  interrupts mid-line, corrupting core 1's stack (noise on I2S), freezing the
  board, controller-independently. Fix: `bool core1_separate_stack = true;`
  (core 1 gets a private 8 KB stack; core 0 the full system stack). Confirmed:
  board survives repeated enumeration attempts.
- **SOF ISR moved to core 1** via `alarm_pool_create_with_unused_hardware_alarm`
  called in setup1 (pools fire callbacks on their creating core); core 0's
  host init waits for the pool flag, then passes the pool via
  `pio_cfg.alarm_pool`. Frame loop verified on core 1: `frame=` field in the
  2 s report advances exactly 1000/s (`pio_usb_host_get_frame_number()`).
- **CDC port death after soft reboot FIXED:** 500 ms detach/attach window at
  boot (10 ms was too short for the hub driver to notice; port stayed dead
  until a physical replug).
- **Loop-side line-state reads REMOVED** (watchdog + report): the E9
  workaround inside `pio_usb_bus_get_line_state()` briefly disables the pad
  input; a core-0 call racing the SOF ISR blinds packet exchange. Line state
  is read only from the ISR-side frame loop now. Report gained `st=`
  (folded line state) and `frame=` (SOF counter) fields instead.
- **10 kΩ pull-downs on D+/D- soldered (user):** D+ now 2.8 V = exact divider
  value (3.3 × 10/(10+1.5)) — the pad's PHY IS enabled and presenting. But
  **D- is still held ~3.3 V by the pad itself** (active drive — the JieLi
  wedge), in silence or with traffic, recovering only on the pad's own power
  cycle. The pad re-verified healthy on a laptop after the whole session.
- **Verdict:** the pad (JieLi BR25, stock firmware) is incompatible in
  practice with the bit-banged PIO-USB host; it enumerates instantly on real
  hosts. Controllers remain usable **through the laptop** (pad → laptop →
  PipeWire → the synth's device port). Reviving the host port for this pad
  needs scope-level measurement at its receiver. Full handoff:
  `docs/HANDOFF-usb-host-freeze.md` (updated through acf8476 + this commit).
- Files: `elements-rp2350/src/main.cpp` (alarm pool handshake, replug fix,
  race removal, frame counter), `elements-rp2350/src/usb_host_watchdog.c`
  (line-state accessor), `elements-rp2350/README.md` (known-issue note).

### OMINOUS core-1 overload — hot DSP path moved to SRAM (2026-10-31)

- **Symptom:** with CC 29 ominous active, `max Process` ~1.1 ms vs the 496 µs
  budget (16 samples @ 32 kHz); the I2S write then blocks, buffers starve and
  audio comes out warped (reported as "wrong pitch"). Normal mode measured
  380-478 µs — also far above what a 240 MHz M33 with hard float should need.
- **Analysis:** kNumVoices = 1, so the cost is ONE OminousVoice (2× 8×
  oversampled FM: 256 kHz FmOscillator + IIR + 101-tap FIR, ~6 KB of code) +
  Part mixdown + reverb. Disassembly showed properly -O3 hard-float loops
  (~42 instr/sample in FmOscillator) — the math is fine. Every hot function
  lived in flash (0x1000xxxx) and executed through the XIP cache; QSPI line
  misses (~200+ cycles each) under the DSP + LUT working set explain the 3-4×
  gap. PIO-USB already worked around this exact problem internally (its
  bit-banged functions are `.time_critical`/SRAM-resident).
- **Fix:** pico-sdk mechanism — functions placed in `.time_critical.*`
  sections land in SRAM (inside `.data`, copied from flash at boot by crt0).
  New `lib/elements/elements/dsp/ram_code.h` defines `ELEMENTS_RAM_CODE(token)`
  (per-function section names — one shared name collides at link time between
  inline/COMDAT and plain copies: "section type conflict"). Marked (on their
  declarations in the headers): `OminousVoice::Process` (carries the inlined
  Upsample/FIR/IIR), `FmOscillator::Process`, `Spatializer::Process`,
  `Part::Process`, `Reverb::Process`. All verified at 0x2000xxxx in the ELF.
  Zero math changes — sound-neutral. RAM 42.2% (+10 KB), flash 11.7%.
- **Build system hardening (regression found during this work):** the
  committed `lib/elements` contained a hand edit vs the upstream submodule
  (`voice.inc` exciter meter kept active for the status LED) that the sync
  script would have silently reverted. Both port-specific deltas are now
  maintained as `tools/patches/elements/*.patch`
  (`elements-ram-hot.patch`, `elements-keep-exciter-meter.patch`) applied by
  `tools/sync_pio_libs.sh` with `--fuzz=0` (conflicts abort loudly).
  Round-trip verified: sync → build reproduces byte-identical addresses.
  Trap recorded: a second stdin redirect (`</dev/null`) after `< "$p"` made
  patch read EMPTY input, succeed with exit 0 and apply nothing — always
  verify "applied" against the file content.
- **Pending hardware acceptance (user):** flash
  `elements-rp2350/.pio/build/elements-rp2350/firmware.uf2`, switch CC 29 to
  OMINOUS, read `max Process` in the 2 s report. Expected ~1.1 ms → roughly
  300-450 µs (analysis estimate; XIP stall share is the unknown). If normal
  mode's 380-478 µs also drops, the same treatment extends to
  `Voice::Process`/resonators next.

- **Measured (hardware, 2026-10-31):** `b35c03a` (code in SRAM): OMINOUS
  524-660 µs — 2× better than 1.1 ms, still over budget, i2s stalls 0 (the
  4 ms buffer bank absorbs it, but sustained overrun would starve within
  ~30 s). `d2ea825` (+ lut_sine 16.4 KB and kDownsamplingFilter in SRAM):
  normal mode 260-269 µs stable. OMINOUS retest on d2ea825 pending user.
- Flash note: one 1200-baud touch attempt wedged (no BOOTSEL MSC, CDC silent
  for minutes); a retry two minutes later worked first-try (MSC in 2 s). If a
  touch ever hangs: retry before suspecting hardware; "no sound" during a
  flash window is just the board sitting in BOOTSEL.

### Main voice path to SRAM — normal mode 113-122 µs (2026-10-31, 9ba6cbe)

- Same treatment for the main (non-ominous) path: `Voice::Process` (6 KB,
  carries the exciters + modal resonator inlined) and
  `String::ProcessInternal<true/false>` (3.5 KB, both template
  instantiations share one `.time_critical` section — same-flags weak copies
  link fine; the earlier section-type conflict only bites mixed
  inline/COMDAT vs plain). Patch: `tools/patches/elements/elements-main-voice-ram.patch`.
  RAM 47.3% (+9.5 KB). **Measured on hardware: normal mode 260-284 →
  113-122 µs** (originally 380-478 + creep) — 23% of budget, stalls 0.
  The XIP-creep vector for normal mode is gone.
- **Flash-path findings (2026-10-31):** the 1200-bps touch became erratic
  (worked 2×, then: no reboot / straight-to-firmware reboot / BOOTSEL
  appeared and auto-ejected back to firmware within ~10 s when nobody wrote
  to it). `pio run -t upload` DID trigger BOOTSEL but died with Error 249
  (picotool USB node root:root — udev rule not installed; needs user sudo).
  **Button-BOOTSEL is the reliable path**: drive appears and STAYS; udisks
  mount + cp + sync flashed instantly. Board now runs 9ba6cbe.
- OMINOUS on 9ba6cbe: pending user retest (chain: Part/OminousVoice/FmOsc/
  Spatializer/Reverb + sine + FIR tables all SRAM now).

### clouds-rp2350 — Clouds port started (2026-10-31, plan: docs/superpowers/plans/2026-10-31-clouds-rp2350.md)

- **C1 DONE:** `tools/sync_pio_libs.sh` generalized to a per-project form
  (`sync_project <dir> "SRC:dst[:mode]"`); mode `cc` = keep `.cc` files (the
  CLOUDS wrapper `clouds_all.cpp` #includes them, so the wrapper is deleted
  on sync instead) + generated `library.json` gets a `srcFilter` excluding
  `clouds/settings.cc` (STM32-flash preset persistence, unportable as-is,
  unused by the processor). Elements lib verified byte-identical after the
  rewrite. `clouds-rp2350/` scaffolding mirrors elements-rp2350 (same wiring,
  vendored pio_usb, status_led.h + ws2812 + host watchdog reused; status_led.h
  decoupled from elements resources via `src/led_brightness_lut.h` — the
  exact 513-entry curve, verified count).
- **C2/C3 code complete:** GranularProcessor + 128/64 KB buffer split
  (stereo 16-bit = small buffer both channels = 1.02 s @ 32 kHz, same as the
  F405 original), `set_sample_rate(32000)`, 32-sample blocks (budget 1000 µs),
  test exciter (burst pluck / gated tone; CC 11 = 0 = chaining input slot),
  full CC map (14-24 panel knobs, 65 freeze, 66 trigger) with pending-slot
  between-block application, USB device + PIO-USB host MIDI, LED meters.
- **C4 DONE:** `tools/patches/clouds/clouds-ram-hot.patch` — clouds-local
  `ram_code.h` (`CLOUDS_RAM_CODE(token)`), ALL resource LUTs + hot code in
  SRAM: GranularProcessor::Process/ProcessGranular, Granular/Looping/WSOLA
  Play templates, Correlator::EvaluateNextCandidate, Diffuser/Reverb/
  PitchShifter, STFT::Process/Buffer, PhaseVocoder, FrameTransformation.
  nm-verified at 0x2000xxxx; WSOLA/looping Play bodies inlined into the
  SRAM-resident ProcessGranular. RAM 60.1% (315 KB incl. 192 KB sample
  memory + ~68 KB moved code/LUTs), flash 4.4%. Round-trip from upstream +
  patch reproduces the build.
- **Pending C5:** flash (button-BOOTSEL), user ear check (grains/freeze/mode
  switching), 30 min soak, ledger update with measured max Process.

- **C5 progress (2026-10-31):** flashed via the touch procedure (works; the
  only race is udisks probing after the node appears — retry
  `udisksctl mount` for a few seconds). **User ear-confirmed granular mode**
  (plucks + tones played by the agent over the device MIDI port via
  aplaymidi/aseq port 24:0). **CRITICAL FIX found by mode sweep:** the port's
  `GranularProcessor::Process()` early-returns SILENT whenever
  `previous_playback_mode_ != playback_mode_`, and only `Prepare()` updates
  that — the original firmware calls `Prepare()` EVERY block; calling it once
  in setup() latched the processor silent+cheap (4 µs) after the first mode
  change. Fixed: `Prepare()` every block in loop1. Also fixed the report
  budget print (992 → 1000 µs, integer-division order).
- **Per-mode steady `max Process` (all-SRAM build, measured with timestamped
  serial + isolated mode sequences):** granular 238-260 µs ✓; looping delay
  181-260 µs ✓; **stretch 1156-1159 µs = 1.16× budget (overruns every
  block)**; **spectral ~14,390 µs = 14.4× budget**. Spectral cause: the pvoc
  STFT uses stmlib's ShyFFT (≈7 ms per 4096-pt transform, 2 per block); the
  fast path (`USE_ARM_FFT` → CMSIS `arm_rfft_fast`) is commented out upstream
  and CMSIS-DSP is not in the repo — known poetaster-port limitation. Stretch
  cause not yet localized (correlator search per block is the suspect).
- **Open decisions:** (a) stretch: profile/optimize (needs one instrumented
  build); (b) spectral: vendor CMSIS-DSP rfft_fast subset + `-DUSE_ARM_FFT`,
  or clamp the mode CC to 0-2 and document spectral as unavailable.

### Spectral profiling + CMSIS-DSP vendor (2026-10-31, after 9c8a03c)

- **Stage-level profiling of spectral mode** (instrumented STFT::Buffer, per
  stage max µs, printed in the report): win 277 / fwd-FFT 2113 / modifier
  2121-2429 / inverse 4477-4907 per channel-frame; ×2 channels = the 13-13.5 ms
  spike. NOT flash-bound: the whole FFT path (functions + 4096 twiddle/coef
  tables + bitrev table, ~56 KB) verified at 0x2000xxxx and the numbers
  barely moved — it is real compute: a 4096-pt transform pair every
  1024 samples (hop = fft/4), i.e. one 13.4 ms burst every 32 blocks
  (~560 µs/block amortized).
- **CMSIS-DSP v1.14.4 subset vendored** to `clouds-rp2350/lib/cmsis_dsp/`
  (rfft_fast + cfft + radix8 + bitreversal2.c + const structs + common
  tables; `library.json` = -O3 + -DARM_MATH_CM33). stft.h
  `USE_ARM_FFT` enabled (→ tools/patches/clouds/clouds-cmsis-fft.patch).
  First attempts: `#pragma GCC section text` is a NO-OP under PlatformIO's
  -ffunction-sections — per-function
  `__attribute__((section(".time_critical.cmsis")))` required (functions) +
  selective table attributes (twiddleCoef_4096, twiddleCoef_rfft_4096,
  armBitRevIndexTable4096 — ONLY the 4096 tables; the init switch references
  all lengths, gc keeps ~100 KB of unused tables in flash).
- **I2S DMA bank deepened** 8→32 buffers (4→16 ms): the 13.5 ms spectral
  frame burst exceeds the old 4 ms bank (underrun glitch every 32 ms =
  user-reported "distorted"); with 16 ms the burst is absorbed (bank drains
  13.5, refills 0.8 ms/block over the following 31 blocks). Avg still
  ~560 µs/block < 1000. Granular/looping unchanged.
- **Instrumentation still in the tree** (stft stage timings in the report +
  `pico/time.h` include in stft.cc): keep until spectral is ear-confirmed,
  then remove (stft.cc delta must go into the clouds patch file).
- **Open:** stretch 1156 µs (1.16×) — LoadCorrelator runs every block in
  stretch mode (Prepare → ws_player_.LoadCorrelator); profiling TBD.
  Spectral ear check pending user.
- **MIDI delivery instrumentation added** (per-user report "heard only two"):
  `note_on_count` (firmware-side note-on counter) + max-since-report in/out
  RMS meters in the 2 s report. Findings: sparse playback (1 note/1.5 s via
  aplaymidi → kernel seq port 24:0) delivers 8/8 with audio on every pluck
  (IN 110-169 / OUT 51-99 — the exciter→granulator→I2S chain works per note);
  the dense multi-CC sweep file lost events host-side (2 of 10 note-ons,
  aseqdump tap on the kernel port invalid for input monitoring — it reads the
  device's OUTPUT side). Delivery loss is a host PipeWire/seq quirk, NOT the
  firmware: earlier sweeps delivered fully through the same path. User should
  test spectral from their own sender (CC 23=3 + keys + CC 65 freeze).
- **Open:** stretch 1.16×; spectral ear-check by user; remove STFT
  instrumentation (stft.cc dbg + report fields) after acceptance.
- **Raw-MIDI test player** (bypasses the flaky seq bridge): python writes
  timed raw bytes to /dev/snd/midiC2D0 directly — 100% delivery where
  aplaymidi/PipeWire dropped events. **Exciter upgraded to a pitched pluck**
  (decaying sine at the note + noise attack) so granulation of NOTES is
  audible; full acceptance sweep played raw (granular plucks → spectral
  tone+freeze+position sweep → looping → stretch). Serial confirmed every
  segment (notes counter 9/9, IN/OUT meters live, zero stalls). Spectral
  spikes 11.6-13.2 ms absorbed by the 16 ms bank.

### Clouds port CLOSED — accepted (2026-10-31)

- **User acceptance: "all that i hear as fine"** — granular, looping, spectral
  (tone + freeze + position sweep) confirmed on the pitched-pluck exciter;
  stretch plucks delivered but the mode still overruns (below). The final
  clean build (instrumentation stripped, patches regenerated
  `clouds-cmsis-fft.patch`, round-trip verified) flashed and sanity-checked
  (3 plucks, counters/meters live).
- **Report format (final):** `max Process | i2s stalls | in= out= notes= | host: ...`
  — in/out = max-since-report RMS (per-mille), notes = firmware note-on count.
- **KNOWN ISSUE (open): stretch mode 1156-1159 µs = 1.16× the 1000 µs block
  budget** — sustained overrun, audible glitches. Suspect: Prepare() runs
  `ws_player_.LoadCorrelator()` EVERY block in stretch mode (4096-sample
  copy + search setup) + the correlator search. Needs the same
  instrumentation treatment as spectral. Spectral by contrast is fine:
  13.5 ms burst every 32 blocks, amortized ~560 µs, absorbed by the 16 ms
  DMA bank (setBuffers(32,16) — REQUIRED for spectral, see profiling above).
- Commits this task: b43b104, 9c8a03c, af49e79, 7d9567b, 5ebbda9, c8dac92,
  f9a0019, (this commit). Board runs the final clean build.

### Stretch mode FIXED — Clouds port fully accepted (2026-10-31)

- **WSOLA profiling** (instrumented play/overlap/sched/loadcorr/evalsome):
  the audio render is only ~50 µs/block — the cost was the **correlation
  search**: `EvaluateSomeCandidates()` drained a FIXED count of
  `(size_>>2)+16 ≈ 1056` candidates per block (up to 2575 µs) and
  `LoadCorrelator` bursts 1339 µs per grain (sign-bit extraction of up to
  12k samples). OverlapAdd per sample: 6-7 µs.
- **Fix:** time-budget the search drain at **150 µs/block** (checked every
  16 candidates; the search completes over ~30 blocks, well inside a grain's
  128 ms lifetime; `best_match()` returns the partial best meanwhile — the
  16 ms I2S bank absorbs LoadCorrelator's per-grain burst). Verified:
  stretch steady 880-964 µs (mode-change block 1743 µs, one-off,
  bank-absorbed), zero stalls. **USER EAR-CONFIRMED all four modes: "all
  sounds ok"** (stretch plucks + pitch-swept stretched tone).
- **Instrumentation stripped** (wsola/granular probes + report fields;
  the in/out/notes meters kept — permanent diagnostics). Both clouds patches
  regenerated from the clean lib (disjoint file sets: ram-hot = everything
  except stft.*, cmsis-fft = stft.*), round-trip verified, final build
  flashed + sanity-checked (stretch 964 µs on the final binary).
- **FINAL per-mode numbers (all-SRAM build, 32-sample blocks @ 32 kHz):
  granular ~240 µs, looping 181-260 µs, stretch 880-964 µs, spectral
  13.5 ms burst every 32 blocks (~560 µs amortized, bank-absorbed) — all
  within the 1000 µs budget.** RAM 71.0% (372 KB incl. 192 KB sample memory
  + ~68 KB SRAM-resident code/LUTs + 56 KB CMSIS tables), flash 7.3%.
- Commits: af49e79, 7d9567b, 5ebbda9, c8dac92, f9a0019, (instrumentation
  strip + this entry). README known-issues section cleared.

### Clouds web panel (tools/clouds_panel.html) — 2026-11-01

- Mirrors the original Clouds panel layout (jacks omitted per user): FREEZE
  button (CC 65 latched, yellow glow), 4 mode buttons (∞ ◯ ⧉ ✳ → CC 23
  0-3) with yellow LEDs, 4 teal buttons repurposed as the QUALITY selector
  (CC 24 0-3: 16S/16M/8S/8M) with teal LEDs, trimmers FEEDBACK (CC 21) +
  REVERB (CC 22), TRIG momentary (CC 66) in the pencil slot; big knobs
  POSITION/SIZE/PITCH (CC 14/15/16), small row IN GAIN (CC 11) / DENSITY
  (17) / TEXTURE (18) / BLEND (19) / SPREAD (20). Keyboard = exciter notes
  (same map as the Elements panel), Space = freeze toggle, Enter = trig,
  SRC button toggles exciter burst/tone (CC 13). Architecture copied from
  elements_panel.html (same knob widget/MIDI layer).
- **Headless byte-dump validation (chromium --headless=new + stubbed Web
  MIDI):** 22 messages; mode/quality/freeze/note-on-C3/note-off/trig all
  byte-exact; connect-sync sends defaults matching the firmware seed
  (CC 14 = 44 = 0.35). Gotcha for future tests: `#pragma`-style DOM
  attribute checks can race --virtual-time-budget (retry the dump); a
  header-level `extern "C"`-style initialized global in a header included
  by two TUs double-defines (use declarations in headers, definitions in
  .cc).

### PORTING-HIGHLIGHTS doc written for RINGS/PLAITS sessions (2026-11-01)

- **`docs/PORTING-HIGHLIGHTS.md`** — the distillation of the Elements +
  Clouds porting experience, written so a fresh session can implement RINGS
  then PLAITS without re-learning: project template (copy clouds-rp2350),
  sync-script modes (inc vs cc + wrapper deletion), settings.cc-style
  srcFilter excludes, the XIP/SRAM law (ram_code macro, unique tokens,
  pragma-no-op under -ffunction-sections, header-globals double-definition,
  nm verification), the original-firmware call-contract lesson
  (Prepare()-every-block), block/rate facts per module, I2S bank sizing
  (burst vs sustained), the profiling pattern (time_us_32 probes → per-mode
  measurement → time-budget drains), MIDI/panel/test-exciter patterns, the
  flash procedure, and **RINGS/PLAITS prep sections** (verified against the
  sources: RINGS = mode cc + settings.cc exclude + Dsp::setSr already
  present + kMaxBlockSize 32 + ~55-60 KB RAM; PLAITS = mode cc + 16 engines
  + hard-wired 48 kHz + 24-sample blocks + BufferAllocator-based
  Voice::Init + 10.5k-line resources.cc).
- AGENTS.md doc map points at it ("START HERE for new module ports").

### Plaits RP2350 port — T1+T2 landed (2026-11-02)

- **Source:** lylepmills/eurorack (Rubato "Plaits Lab" fork) added as submodule
  `EURORACK-STEREO` @ `83f86ae` — all models stereo-converted, 98-engine
  catalog (63 MI incl. Braids-derived engine2 set / 22 Rubato Lab / 13
  community; 2 catalogued "retired" but functional). Plan:
  `docs/superpowers/plans/2026-11-02-plaits-rp2350.md`.
- **Key fork facts (survey):** no engine enum — registry index; the 32-slot
  ceiling is ONLY the uint32 speech/chiptune masks (13 `1u << engine_index`
  sites, all in voice.cc); `Frame{short out; short aux}` native 48 kHz,
  `kMaxBlockSize=24`; workspace = largest engine only (16.4 KB, Particle);
  worst engine ≈30% of our 500 µs budget per the fork's own F373 sweeps;
  no FFT, one deliberate `double` (struck_drum); poetaster STMLIB works as-is
  (14 headers verified).
- **T1 (485d94c, 72a5ee4, f91f1f1):** 5-patch chain in `tools/patches/plaits/`:
  PLAITS.h wrapper, user_data STM32-flash stub (`-DPLAITS_HAS_USER_DATA_BANK=1`
  → DX7 banks from syx_bank_0/1/2), build_config ceiling 32→98,
  engine_config.h full 98-catalog rewrite (uint64_t masks at slots speech=9 /
  lpc=11 / chiptune=27 / natural-speech=28; kEngineUserDataBank 22/23/24),
  voice.cc `1u`→`1ULL` ×13. All 98 slots linked — nm cross-check vs
  catalog.json: 0 engines missing. sync script gained `SRC:dst[:mode[:nest]]`
  + eurorack-layout source resolution + plaits prune (118 TUs).
- **T2 (f91f1f1):** src/ from clouds template: 48 kHz / 24-sample blocks,
  setBuffers(48,16)=16 ms bank, budget 500 µs multiply-first, Voice +
  BufferAllocator(32 KB, aligned 8); patch.note = MIDI note + bend (69 = 440 Hz
  per engine.h NoteToFrequency); velocity → trigger pulse 0.3+0.7·v/127
  (threshold 0.3); MACRO via modulations.engine with model_cv_option=1;
  modulation amounts ±1 (v·2−1); aux_output_option=1 (stereo) via CC 24;
  engine CC 23 → patch.engine 0-97; raw write16(f.out, f.aux) (already
  Clip16'd + stereo-limited in ChannelPostProcessor); no exciter. README with
  the full 98-engine table.
- **Build:** SUCCESS 93 s — RAM 17.3% (90,616 B / 524,288), flash 15.1%
  (634,632 B). sizeof(plaits::Voice) = 21,748 B (98 engines ≈ +2.9 KB over the
  stock 24). voice/workspace at 0x2000xxxx; engine Render fns still in flash.
- **Also:** committed the previously-uncommitted clouds-ram-hot.patch fix-up
  (a655a20 — WSOLA time-budget hunk; round-trip proves it matches the shipped
  clouds lib).
- **Next:** T3 SRAM placement (voice hot paths + fork-flagged hot set:
  particle, modal(+voice), string(+voice), chord, chiptune, six_op, scanned)
  + selective LUT SRAM + per-engine CPU sweep; T4 web panel; T5 hardware
  acceptance + soak (user).

### Plaits RP2350 — T3/T4/T5a/T5b done (2026-11-03)

- **T3 SRAM placement** (`f391da0`): patches 05/06/07 — `ram_code.h` +
  **73 `.time_critical` tokens** (Voice::Render path, all stock engine1
  Renders, fork-hot-list engine2 bodies, support DSP incl. `fm::Voice<6>`) +
  **14 per-sample LUTs (100.8 KB: wav_integrated_waves ×3, wave-scan kWaves
  32 KB, kBuzzZones, lut_sine/fold, freshets formant shapes)**. Tiered:
  marking ALL ~96 engine bodies measured RAM 76.3% (over the 75% ceiling) →
  ~66 small engine2 bodies stay in flash by design (working set < XIP cache;
  promotion = one-line token if the sweep flags one). **RAM 17.3% → 69.3%
  (363 KB), 0 marked symbols left in flash (nm-verified).** GCC quirk found:
  section attrs on out-of-line member-template definitions (string.cc) are
  silently ignored — attribute goes on the in-class template declaration.
- **T4 panel** (`b2b6ddf`): `tools/plaits_panel.html` — 98-engine dropdown
  (id+origin), stereo/subosc AUX selector (CC 24), ±1 attenuverter tooltips,
  computer-keyboard notes; headless byte-validated 31/31 exact.
- **T5a flash**: 1200-bps touch first-try; board re-enumerates as
  "Plaits RP2350" (seq client 24:0).
- **T5b per-engine CPU sweep (98/98 engines, notes + CC 23 via
  `aplaymidi`, 2 s reports over serial): `i2s stalls: 0 (+0)` throughout.**
  Every report over 500 µs (speech 463, lpc-speech 641, dx7-a/b/c 499/419/472,
  vosim 686) was proven by a 12 s dwell (6 report windows/engine) to be a
  **ONE-BLOCK engine-change spike** (change block = Reset + LoadUserData +
  cold caches); sustained: speech 25 µs, lpc-speech 25 µs, dx7 67-69 µs,
  vosim 100-115 µs. Sustained worst ≈ 268 µs (54% of budget). The 16 ms I2S
  bank absorbs every change spike (same pattern as clouds' 1743 µs
  mode-change). **No SRAM promotions needed.**
- **Sweep `out=0` anomaly (LPG-routed engines) — not a firmware bug.** The
  first sweep's reports showed out=0 for the non-self-enveloping engines;
  temporary probes (`08-plaits-debug.patch`, since REMOVED, never committed)
  proved the whole chain correct on hardware: `cl=945 → lg=804 → fo=7876`,
  meters live, releases clean. Attributed to the earlier mid-iteration
  binary; **clean rebuild from committed patches (`rm -rf .pio` + sync +
  build) verified sound on hardware** (engines 0/7 through real engine
  changes, fresh-boot note counter from 0). Lesson recorded: acceptance
  builds must come from a clean .pio of the committed state.
- **Usage note:** LPG-routed synth engines need LEVEL (CC 25; boot default
  0.8, panel sends 0.8 on connect). Self-enveloped engines (drums, speech,
  strings, chiptune) sound from note-ons alone (velocity → TRIG pulse,
  accent path).
- **Test-tooling notes:** aplaymidi + serial capture works (sparse events
  safe through the seq layer); SMF gotchas — a bare "gap" delta with no
  following event is INVALID MIDI (merge gaps into the next event's delta),
  aplaymidi requires the EOT meta, 1 ms/tick = PPQ 960 + tempo 960000.
  rawmidi `/dev/snd/midiC2D0` open can wedge kernel-side after
  flash/re-enum (EBUSY with no fd holder) — the seq path sidesteps it.
- **Board state:** running the final clean build (`93 s`, RAM 69.3%, flash
  15.1%), eng 0/7 verified sounding, zero stalls. **T5c USER EAR-CONFIRMED
  (2026-11-03): "plaits sound fine too" — PLAITS PORT ACCEPTED.** All four
  ports (Elements, Clouds, Plaits, Rings) now user-accepted on
  `elements-clone`; merge to `main` unblocked.

### Rings RP2350 port — T1-T5a landed (2026-11-03)

- **Source:** same `EURORACK-STEREO` submodule, `rings/` — the fork's Rings is
  the **MiniElements** alt firmware. Survey corrected PORTING-HIGHLIGHTS §9 in
  four load-bearing ways: **32 kHz / 24-sample blocks** (750 µs budget, no
  `Dsp::setSr` — constants baked), ~184 KB worst-case RAM (not 55-60; 8 Strings
  ≈ 99 KB + Resonators + 64 KB caller-owned reverb buffer), **no STM32 leak in
  dsp/** (no settings/user_data stub needed — prune suffices), 5 performance
  modes (Rings stereo/waveform, MiniElements stereo/exciter, easter-egg string
  synth) × 6 resonator models/fx × 3 chord tables. out/aux = ODD/EVEN → L/R.
  Plan: `docs/superpowers/plans/2026-11-03-rings-rp2350.md`.
- **T1** (`177237e` sync pair+prune, `b84dcfc` project): 8-TU compile set
  (7 dsp .cc + resources.cc), `RINGS.h` wrapper, `bryanChords` → const
  (1.4 KB out of .data). Build green 47 s, **RAM 43.2%**.
- **T2** (`b84dcfc`): firmware from the plaits template — fresh
  Patch/PerformanceState per block (rings.cc contract), internal_exciter=true
  (standalone: pluck/pulse/envelope + MiniElements bow/blow/strike from the
  flash-resident 338 KB smp_* ROMs), strum pulses from MIDI note-ons
  (Strummer/OnsetDetector dropped), CC 14-23 (structure/brightness/damping/
  position/model+fx/polyphony/mode/waveform/chord-table/chord — chord clamped
  per table), set_model/set_fx/set_polyphony between blocks, Clip16 ours.
  **RAM 43.4% / flash 12.2%.**
- **T3** (`76ed5e2`): `02-rings-ram-code.patch` — 23 `.time_critical`
  functions (Part/String+ProcessInternal templates/Resonator/FMVoice/Exciter×7
  /Tube/Reverb/Chorus/Ensemble/StringSynthPart) + 9 LUTs (28.2 KB incl.
  lut_sine 20.5 KB); smp_* ROMs + stmlib deliberately flash. **RAM → 54.4%,
  nm-verified 35/35 at 0x2000xxxx.**
- **T4** (`c0c6b56`): `tools/rings_panel.html` — mode/model (dynamically
  relabeled to FX in easter egg)/polyphony/waveform/chord-table selectors +
  chord dropdown + 4 knobs; headless byte-validated PASS.
- **T5a flash + sweep** (19 combos: mode 0 × models × poly 1/4, modes 1-4,
  easter egg × fx 1-5): **`i2s stalls: 0` throughout; ALL under the 750 µs
  budget** — worst 511 µs (sympathetic string, poly 1), string-family modes
  461-511, modal 279, MiniElements 271-274, easter egg 300-320. No spike
  analysis needed (nothing near budget). Board re-enumerates as
  "Rings RP2350" (client 24:0).
- **T5b USER EAR-CONFIRMED (2026-11-03): "sound fine, max process near
  250-300"** — matches the sweep's sustained numbers (270-320 µs, 36-43% of
  the 750 µs budget), zero-stall expectation holding. RINGS PORT ACCEPTED.
  Plaits ear check confirmed same day — all four ports (Elements, Clouds,
  Plaits, Rings) user-accepted; elements-clone → main merge unblocked.
- **MERGED (2026-11-03): `main` fast-forwarded d8e6184 → `03b1119` (61
  commits: Elements T1-T6, Clouds, Plaits 98-engine, Rings MiniElements
  ports + panels + docs), pushed to origin; both branches pushed. Working
  checkout stays on `elements-clone` (established workflow).**

### Combined rack (rack-rp2350) — SDD status table

Plan: `docs/superpowers/plans/2026-11-04-combined-rp2350.md` (read first —
decisions are final; the ORCHESTRATION protocol is at its top: this session's
agent spawns one subagent per task, sequentially, verifies each acceptance
gate itself on hardware, and updates the row + appends dated findings below
after each task).

| Task | Status | Committed | Notes |
|---|---|---|---|
| T1 Linker/sections (module-prefixed .time_critical, NOLOAD SRAM window, selective copy) | done | 14a9436 | 4 boot-select envs benched on hardware 2026-11-04; per-env SRAM windows; 2 firmware boot bugs fixed (early CDC print, post-detach re-enum death); findings below |
| T2 Arena + live swap (bench switching via switch-CC + serial console) | done | 50d0489 | CC 32 + console swaps among all 4, 19-44 ms mutes, zero stalls, user ear-clean; plaits re-tiered 272→109 KB, clouds CMSIS tables to flash (spectral excluded v1); crash fix: I2S reconfig moved to core 0; findings below |
| T3 Mod engine + clock bus (internal BPM / MIDI 24 PPQ, 1-32-step euclid+gate, bench settings via CC/console) | done | 238b4d0 | 12 sources; u: ≤ 45 µs observed (gate 50, 31 µs under 12-source load); ext-clock verified (48/s @ 120 BPM, tempo-follow 120→100, START re-phase / STOP freeze); phase survives swaps; findings below |
| T4 RP2350 DIN 31250 parser + serial console | done (code-complete; DIN-jack bench pending wiring) | 67d6f77 | parser 109/109 host asserts; dintest proves parser→funnel→module on hardware (+3 notes, CC, 24/25 clock stamps); console full; findings below |
| T4a Presets (serialization + console dump/load + last-state autosave + C3 flash bank) | done (benched 2026-11-11) | 8636643 | blob 871 B v1 crc16 on the RP2350 (`preset.h`); console dump/collect/load/apply hex bench; PRESET_REQ/PART/APPLY wired both ends; C3 bank = raw `presets` partition (16 slots + 4 rotating autosave sectors); SYS ring 7 items (SAVE/LOAD); autosave 3 s debounce; boot-restore on link-online; CC-cache replay after EVERY factory re-Init (swap desync retired); bench: round-trip + state restore + save/load slot 3 + power-cycle restore + reject paths, all PASS; findings below |
| T5 C3 UI + UART link (after DSP acceptance AND hardware) | code-complete (stages 1-6): stage 1 link BENCHED 2026-11-08; stages 2-6 code-verified, benches pending wiring | d901e76 + feacd34 (rack-uimcu), e278d69 + 656aff5 (rack-rp2350) | seed rack-uimcu/ from ctag-tbd/uimcu (zero build deps on ctag-tbd now); protocol docs/ui-protocol.md (TELEM v2 + ASSIGN_CYCLE); RP2350 side strictly additive (RAM +1,060 B / flash +9.9 KB total, main.cpp untouched by stages 2-6); stage-1 bench: uitest 6/6 + C3 hw-loop 32/32 @921600; wiring list in rack-uimcu/README.md |
| T6 Chain mode v2 (plaits@32k patch, pair table) | done (3 of 3 pairs) | 972c58a + c106458 + (T6c commit) | ALL THREE pairs benched: elements→clouds 52%, rings→clouds 36-48%, plaits→clouds 36% (flash engines); switched-plaits engines now flash (~3× T5b numbers, sustained ≤ 269 µs of 500, change spikes bank-absorbed); user dropped Plaits→Elements |
| T7 Hardware bring-up + sweeps + user acceptance + soak | pending | — | |

Findings (append per task below):

### Rack T1 — linker/sections + selective copy (2026-11-04)

- **New project `rack-rp2350/`**, 4 envs (`rack-elements/clouds/plaits/rings`,
  `-DRACK_BOOT_MODULE=0..3`, board rpipico2, 240 MHz, USE_TINYUSB, chain+).
  All four module libs link in every env (LDF needs unguarded includes);
  only the selected module instantiates state (compile-guarded adapters in
  `src/module_*.h`, state/handlers lifted verbatim from the four ports'
  main.cpp/midi.h). Uniform 2 s report: `max Process | i2s stalls | in= out=
  notes= | host: …` with per-module budgets (496/1000/500/750 µs).
- **Patch regeneration ×4** (elements-ram-hot, clouds-ram-hot,
  05-plaits-ram-hot, 02-rings-ram-code): the ram_code.h macro is conditional —
  `RACK_RP2350_TC` → `.tc_<module>.<token>`, else the original
  `.time_critical.<token>`. New `tools/patches/cmsis_dsp/01-rack-tc-names.patch`
  retargets the vendored CMSIS copy inside the rack only. Standalone
  round-trip verified: all four standalone projects clean-build with
  byte-identical RAM/flash numbers (elements 47.3%/11.7%, clouds 71.0%/7.3%,
  plaits 69.3%/15.1%, rings 54.4%/12.2%).
- **PER-ENV linker scripts** `memmap_rack_<module>.ld` (forks of the core
  rp2350 script — a raw -T fork reaches ld un-substituted, so
  `__FLASH_LENGTH__` etc. are hardened to the rpipico2 values). The SELECTED
  module's sections = an ld OVERLAY member (VMA = that env's SRAM window
  directly above the heap, LMA = reserved flash RACK_LMA @0x10381000); the
  other three modules = flash-resident `.tc_flash_*` sections (VMA=LMA,
  linked, never called in that env). **Why per-env:** the first cut shared
  ONE window sized for plaits (0x43000 @0x2003d000) — the clouds env was
  left with 7,320 B of heap, the core's `malloc(0x2000)` for core 1's
  separate stack FAILED, core 1 never launched and core 0 hung forever at
  the `g_host_alarm_pool_ready` handshake (hardware symptom: LED self-test
  runs, USB enumerates, zero report lines). With per-env windows clouds has
  144 KB of heap.
- Window sizing (rack ELF-measured `.tc_*` totals): elements 36,752 B →
  window 0x9000 @0x20077000; clouds 136,960 B (incl. 60,344 CMSIS) → 0x21800
  @0x2005E800; plaits 272,564 B → 0x43000 @0x2003D000; rings 58,084 B →
  0xE400 @0x20071C00. **T2 NOTE: live swap needs ONE window that fits every
  module → the plaits re-tier (~120 KB, plan-T6) must land before/in T2.**
- **Selective copy API** `src/rack_window.h`: per-module {VMA, LMA, size}
  table from linker symbols (`__tc_<m>_start/_end`,
  `__rack_flash_<m>_start/_end` — all four pairs exist in every env; for
  unselected modules both pairs coincide at flash addresses, a dead path
  since boot-select is compile-time), `rack_window_copy(id)` memcpy
  flash→window + FNV-1a both sides; `setup()` copies as its ABSOLUTE FIRST
  action (crt0-like, before Serial/LED/USB) and prints size + checksums.
- **Verification (per env)**: `.tc_*` inputs in `.data` = 0; selected overlay
  VMA = window origin, unselected `.tc_flash_*` VMA=LMA in RACK_LMA; hot
  symbols at window addresses (elements Part::Process, clouds
  GranularProcessor::Process + twiddleCoef_4096, plaits Voice/engine Renders,
  rings Part::Process, CMSIS arm_rfft_fast_f32); pio_usb stays
  .time_critical→.data (low SRAM); `__rack_flash_end` ≤ RACK_LMA end with
  ~11.5 KB slack; ASSERTs per env (window fit, flash end, stack/heap).

### Rack T1 — boot-wedge saga: three root causes, two firmware fixes (2026-11-04)

- **Symptom that started it:** rack boots "wedged" nondeterministically
  (CDC-up-but-silent or device-gone-from-bus), ~40% in early testing, while
  the standalone control was clean through the identical procedure. Cost a
  day of LED-breadcrumb diag builds before the ground truth appeared.
- **Fix 1 (real, firmware): first CDC print vs enumeration.** The rack's
  first unconditional `Serial` write (the `rack: window …` line) fired within
  milliseconds of boot, while the host was still enumerating → chip hang.
  The proven standalone prints its first unconditional line only AFTER
  host+I2S init (~1 s in) — that timing difference was the boot delta that
  mattered. Fix: compose the line early, print it once enumeration has
  settled; conditional error prints stay early (as in the standalone).
- **Fix 2 (real, firmware): the detach/attach re-enum guard was a boot
  killer.** Kernel log: firmware enumerates fine (product read), the guard's
  own detach fires 1 s in, and the POST-DETACH re-enumeration intermittently
  dies (SET_ADDRESS refused / descriptor-timeouts → dead until power cycle).
  This is the same "CDC port death" family the standalone era mitigated with
  the 500 ms detach window (10-31 session) — mitigated, never eliminated.
  **Fix: register the MIDI interface as the FIRST setup action** (the host's
  first GET_DESCRIPTOR lands ≥100 ms after attach per USB reset recovery, so
  MIDI is in the first configuration) and DELETE the guard — the device now
  enumerates exactly once. NOTE: the four standalone ports carry the same
  guard and the same latent defect (rare); not touched (accepted code).
- **Non-firmware cause: the 1200-baud touch.** `reset_usb_boot` sometimes
  leaves the BOOTROM unable to enumerate (kernel: attach-no-descriptor ×3 →
  "unable to enumerate" → dead to USB until power cycle) — the erratic-touch
  behavior recorded in the standalone era. Touch-induced USB resets also
  explain the "black/frozen LED" reads (reset_usb_boot resets PIO/pad state
  → WS2812 dark), the "3 s DAC noise at boot" (touch landing mid-boot) and
  partial breadcrumb sequences. **Procedure: the touch requires DTR low — a
  serial monitor holding the port blocks it; two failed touches → fall back
  to button-BOOTSEL; and ALWAYS read `journalctl -k` for USB ground truth
  (attach-without-descriptors = touch/bootrom victim, not firmware) before
  touching firmware.**
- Distractor eliminated: RP2350 M33 I/D-caches are NOT enabled by the
  core/SDK (no SCB_Enable*Cache anywhere; symbols not linked) —
  cache-incoherence after the setup()-time window copy was impossible.

### Rack T1 — hardware bench matrix (2026-11-04, orchestrator)

All four boot-select builds flashed (touch + button-BOOTSEL mix) and benched;
each enumerated exactly once (guard removed), MIDI port "Rack RP2350 MIDI 1":

| env | idle max Process | loaded max (bench sweep) | notes/meters | stalls |
|---|---|---|---|---|
| elements | 101-125 µs (budget 496) | 193-207 µs (strings model active); ominous dwell 260-267 µs | 4/4 notes, user ear-confirmed ("notes played", "i hear sound"), in/out RMS to 1000 | 0 |
| clouds | ~236 µs granular (budget 1000) | mode-change spike 1737 µs (standalone: 1743); spectral burst 13.8-14.2 ms bank-absorbed (standalone 13.5) | notes 4/4, exciter meters live | 0 in 11/11 reports |
| plaits | 34 µs engine 0 (budget 500) | ≤148 µs across engines 0/7/40 + level CC | notes 8/8, meters live | 0 |
| rings | 122-125 µs (budget 750) | 473 µs across mode/model sweep (standalone worst 511) | notes 7/7, out RMS ~410 | 0 |

Standalone-signature parity: every number lands on (or under) the recorded
standalone ranges — the window-resident hot code behaves like the standalone
builds' crt0-copied .time_critical. User ear-confirmed elements; the other
three are metrics-verified (ears at the DSP-acceptance checkpoint).

### Rack T2 — arenas + live swap (2026-11-05)

- **UNIFIED BINARY.** The four per-env linker scripts collapsed back into ONE
  `memmap_rack.ld`: all four `tc_*` sections share the window VMA (ld
  OVERLAY), LMAs sequential in RACK_LMA. The four envs remain as boot-module
  selects sharing identical maps.
- **PLAITS RE-TIER: tc_plaits 272,564 → 108,820 B.** Demoted via regenerated
  06/07 patches: 39 long-tail engine bodies (~116.2 KB) + 10 engine-private
  LUTs (47.6 KB, incl. kWaves 32 KB, kBuzzZones, lut_fold/ws/lpc/formant).
  KEEP: Voice::Render path, support DSP (fm::Voice<6>, Resonator, String,
  ModalVoice, StringVoice), fork-flagged hot engines (chord/particle/modal/
  string/chiptune/six_op/scanned) + boot-default engine 0 (VA, 5.8 KB —
  required by the "engine 0 stays well under budget" constraint), per-sample
  LUTs of the kept set (wav_integrated_waves ×3 = 50.7 KB, lut_sine).
  **Standalone plaits round-trip: RAM 69.3% → 38.0% (−163,872 B), flash
  +84 B** — the standalone port gets the re-tier's RAM win too.
- **CLOUDS CMSIS TABLES TO FLASH (rack only):** tc_clouds 137,216 → 79,744 B
  (twiddleCoef_4096/rfft/bitrev = 56 KB → .rodata; the FFT CODE stays in the
  window). This is the plan's designated sacrifice — **spectral mode is
  EXCLUDED in combined v1** (CC 23=3 rejected on clouds; verified on
  hardware: no crash, no burst, mode unchanged). One-line revert path
  documented. This was REQUIRED: clouds couldn't fit the shared ≤125 KB
  window otherwise (the plan's re-tier row names only plaits — flagged).
- **Window: 0x1C000 = 114,688 B @ VMA 0x20064000** (max member plaits
  108,820 + 5.9 KB promotion slack). RACK_LMA moved to 0x10389000 (main
  image grew by the demoted tables); extent ends 0x103ce308 (~200 KB slack).
- **ARENAS** (`rack_arena.h`): object 128 KiB + buffer 192 KiB, both `alignas(8)`
  .bss unions; adapters placement-new their state at Init (adapters' file-scope
  state symbols GONE from .bss — nm-verified). Carves: elements reverb 64 KB /
  clouds large+small 192 KB / plaits allocator pool 32 KB / rings reverb 64 KB;
  objects: rings Part 123.4 KB is the sizing module. RAM per env: .data
  19,576 + .bss 345,044 (incl. arenas) → heap 44,708 B, ~20-25 KB boot
  headroom (core-1 stack 8 KB + I2S bank + USB).
- **MODULE REGISTRY** (`module_registry.h`): {name, id, rate, block, budget,
  i2s config, stall threshold, cc_map, mod_destinations (T3 placeholder),
  init/render/note/cc hooks}; zero per-adapter `#if` remains. CC 32 = module
  switch (verified unused by all four CC maps); serial console `switch
  <0-3|name>` / `module` / `help` (non-blocking line parser, ready for T4).
- **SWAP SEQUENCING (core 1, block boundary):** mute (4-block gain ramp +
  full silent bank flush) → window copy (FNV-checksummed; corrupt copy
  restores the old image and aborts) → placement-new + factory Init → I2S
  reconfig → ramp-up. Measured mutes: 19-44 ms (plaits copy = 44 ms).
- **SWAP CRASH FOUND AND FIXED (bench, 2026-11-05):** the swap's
  `i2s.end()/setBuffers/begin()` ran on CORE 1 while the I2S DMA IRQ lives on
  CORE 0 (i2s.begin() ran there; RP2350 routes enabled IRQs to the enabling
  core) — teardown under the live ISR = core 0 hard fault (reports die, DMA
  clocks freed buffers = noise burst, power cycle needed). Reproduced
  mid-note clouds→elements after earlier timing-luck passes. **Fix: the I2S
  reconfig moved to core 0 via a bounded handshake** (core 1 publishes the
  target config + waits ≤1 s; core 0 performs end/config/begin + acks; no
  concurrent I2S access from both cores). Verified: the exact crash scenario
  + 8 consecutive swaps (all 4 modules ×2, held note) — 8/8 clean, zero
  stalls, no faults; user ear-confirmed two mid-note swaps "Both clean".
- **Bench:** CC 32 swap cycle (4/4, 23/44/31/28 ms), mid-note swaps, per-
  module post-swap audio (clouds granular 239 µs / plaits 35-50 µs / rings
  via earlier cycle), spectral rejection, console swaps by name and id,
  8-swap stress. Swap semantics documented (factory defaults on swap; panel
  CC positions desync — reconnect re-syncs; T4a presets will restore).

### Rack T3 — mod engine + clock bus (2026-11-05)

- **`src/mod_engine.h` (~1230 lines, pure rack-src, no lib changes):**
  persistent `g_mod_engine` = 1,776 B in .bss, nm-proven OUTSIDE both arenas
  (ends exactly where the buffer arena begins). 12 sources (the plan's list
  sums to 12 despite the "(8)" header — deviation accepted): 4× LFO
  (free-Hz 0.02-20 exponential OR clocked-division; sine/tri/saw/square/S&H;
  phase offset; level), 4× Euclidean (Toussaint-canon maximally-even masks,
  first pulse at step 0, rotation-reachable; steps 1-32; host-tested
  invariants), 2× rhythmic gate (1-32 mask, per-step gate length), 2× ADSR
  (gate-follow from the note stack OR clock-retrigger; A/D/R 1 ms-10 s — a
  host-side unit test caught and fixed a 1000 s exponent bug pre-build).
- **Clock bus:** block-rate 24-PPQ accumulator (internal, BPM 20-300 @ 0.1)
  OR sample-stamped external F8/FA/FB/FC: core 0 stamps at the single
  `midiHandleEventPacket` funnel (both USB paths; T4's DIN adds one call)
  with `sample_counter + sub-block µs interpolation`; 64-deep SPSC ring;
  core 1 drains at block top. Per-source clock override + 10 divisions
  (1/1…1/32 + triplets, exact 24-PPQ integers). Measured-tempo EMA drives
  clocked LFOs (smooth ramp, no stair-step).
- **Bench control:** CC 33/34/35 mux (source/param/value — grep-proven
  collision-free vs all four maps) + serial `mod/mod?/bpm/clock`; CC 64 =
  trigger pseudo-CC (plaits TRIG, rings STRUM); destinations per module
  (continuous CC lists with mode/quality/engine selectors excluded; clouds
  freeze=65 LATCH / trig=66 PULSE); inert destinations logged + tagged in
  `mod?` (`INERT! on <module>` — verified on hardware).
- **Bench (hardware):** EU odometer +16/2 s exactly (1/16 @ 120 BPM);
  LFO1→CC 27 sweeps elements' space; routing re-resolves per module across
  swaps (INERT on plaits, live again on elements, out= continuous mid-cycle
  — phase survived two swaps, odometer never reset); trigger recipes
  exercised (plaits TRIG retrigger with notes, rings STRUM, clouds
  freeze+trig); **external sequencer via a C ALSA-seq client
  (/tmp/seqclock.c — aplaymidi DROPS F8/FA/FC in its SMF→seq conversion,
  ticks=0; rawmidi wedges post-flash per the known EBUSY issue): m:48/s at
  120 BPM, tempo change 120→100 followed (EMA slew), clocked LFO re-phased
  on START, frozen on STOP; `modeng=` telemetry + `u:` (mod_us_max) field;
  u: ≤ 45 µs across all tests, 31 µs under all-12-sources routing — 50 µs
  gate met with margin.**
- Bench-tooling note for T4/T7: external-clock tests must use a seq CLIENT
  sender (SND_SEQ_EVENT_CLOCK/START/STOP direct events), not aplaymidi SMFs.

### Rack T4 — DIN parser + console (2026-11-05, DIN hardware NOT wired)

- **`src/din_midi.h` (pure, no SDK types):** 4-state MIDI-1.0 byte parser
  (IDLE/READY/COLLECT/SYSEX) → 4-byte USB-MIDI packets [cable<<4|CIN, …].
  Realtime F8-FF emits inline anywhere and preserves running status; sysex
  discarded (counted); F4/F5/orphans/aborted messages counted as dropped;
  no buffer → no overflow. **109/109 host assertions** (running status,
  interleaved realtime, sysex abort/restart, F1-F6, packet/CIN shapes; the
  host test caught a stale-d1 bug in 1-data-byte messages pre-build).
- **UART1 RX = GP5 (TX GP4 reserved), 31250 8N1** via the core's SerialUART
  (IRQ + 256 B ring; `setPinout(4,5)` MANDATORY — the rpipico2 variant
  defaults Serial2 to GP8/GP9 = I2S DOUT). No PIO/alarm/DMA claims → the
  init-order law doesn't apply. 6N138 wiring table in the README
  (datasheet-verified pinout; 10 kΩ pull-up on GP5 + internal pull-up).
- **Console grew:** route (mod-dest alias) · status (one-liner incl. din
  counters + swaps + uptime) · din [clear] · dintest · telemetry on|off ·
  panic (real note-offs + stack clear + gate off). Data-driven table.
- **Bench (without jack):** `dintest` injects a canned stream into the
  parser and dispatches through the REAL funnel — 52 bytes → 34 packets
  (27 realtime), +2 dropped, **note_on_count +3 audible notes + CC through
  midiHandleEventPacket**, 24/25 clock stamps drained (one tick short of
  the stream's 25 F8 — drain-timing, stamps_dropped=0; noted honestly).
  status/din/panic/telemetry/route all verified on hardware.
- **ORCHESTRATOR FIX during bench:** the subagent's dintest PARSED and
  PRINTED packets but never called midiHandleEventPacket (the claimed
  funnel dispatch didn't exist — note_on_count +0, ticks=0 on first bench).
  One-line fix: dispatch inside the parse branch. Lesson: "expected-but-
  unverified" claims must not restate intent as fact — dintest's summary
  line asserted funnel delivery that the code didn't do.
- **PENDING (needs the DIN jack wired per the README table):** real-UART
  byte flow (UART1 IRQ → parser), hardware-controller notes/CC/clock
  through DIN, the modeng=m:48 DIN-clock signature. The funnel + parser
  halves are verified; only the UART-to-jack half awaits hardware.

### DSP-ACCEPTANCE CHECKPOINT — PASSED (2026-11-05)

- **User ear-accepted the rack build across all four modules** ("all fine")
  after one bench-routing artifact was cleared (a leftover T3/T4 console
  route of LFO1→elements CC 27 space made the reverb sweep at 1 Hz — routed
  sources persist until power cycle; nothing persists settings until T4a).
- Elements, swaps (mid-note, "Both clean"), clouds (granular/looping/
  stretch; spectral excluded v1), plaits (engine sweep), rings (modes/
  models): all confirmed on the final T4 build (`67d6f77`).
- **T5/T6/T7 are unblocked pending the user's go** — T5 additionally gates
  on the C3 hardware existing; T7 adds the 30-min soak and full sweeps.
  One listening artifact to remember: routed mod sources override panel CCs
  until unrouted (standard mod-matrix behavior, documented in README).

### Rack T6 — chain mode v2 (2026-11-05)

- **Scope (user-amended):** Plaits→Elements DROPPED by the user 2026-11-05;
  pair table = elements→clouds, plaits→clouds, rings→clouds (all X→clouds —
  uniform routing into clouds' FloatFrame input, exciter fully bypassed when
  chained).
- **Plaits runtime-rate patch** (`tools/patches/plaits/08-plaits-runtime-rate.patch`,
  rack-conditional `-DRACK_PLAITS_RUNTIME_RATE`): the fork's kSampleRate /
  kCorrectedSampleRate / a0 become runtime globals + `plaits::SetSampleRate()`.
  Survey: 94 code uses, ALL runtime float reads — zero compile-time uses
  (no array sizes/templates/case labels). Standalone plaits round-trips
  **byte-identical** (sha256-verified). 48k-baked exceptions documented
  (README): zxphase48k/zxpulse48k time effects ×1.5 at 32k; particle burst
  decay ×1.5 slower. Voice re-Init at every rate-context change.
- **PHYSICAL RESIDENCY WALL (the T6 stop item):** chain mode needs BOTH
  modules' hot code resident — the one-VMA overlay cannot express that.
  Re-architected to a **two-slot window** (window 0x1C000→0x1D000, RAM −4 KiB;
  A slot @origin, clouds-B slot @+0x9000). Measured verdicts:
  - elements→clouds: **fits** (116,496 of 118,784 B window; objects
    122,468/131,072; buffers exactly 196,608) — IMPLEMENTED.
  - rings→clouds: needs 137,828 B (> window ceiling ~139,264 only with a
    clouds spectral-code demotion + 8 KB arena bump) — REJECTED at engage
    with exact numbers; ALSO the pair budget is 93-99% (rings worst 511 +
    clouds ≈ 700-740 + mod ≤50 — over 750 in the worst case): the squeeze
    likely buys residency without budget.
  - plaits→clouds: needs ~188.7 KB — impossible without a SECOND, harder
    chain-tier plaits image (~60 KB: voice path + engine 0 only); ~140 KB
    window vs the ~139 KB ceiling — edge-of-feasible, another re-tier campaign.
- **Chain machinery:** persistent chain state (routing survives swaps);
  enter/leave = mute → checksummed double window copy → I2S via the core-0
  handshake (32 kHz / 32×16 bank) → both slots re-Init (plaits at 32k) →
  unmute; measured 32 ms engage / 24 ms disengage, click-free. Chain render:
  A(24) → route → clouds Prepare+Process(24) → B → I2S; proc_us_max times
  A+route+B. Mod engine runs the chain grid via two override reads
  (destinations bind to A). Chained-clouds buffer split = 0.512 s/channel
  (exact-fit arena). CC 23/24/65/66 pass to B while chained (A loses 23/24 —
  dual-mapped; the mod engine still reaches A). B-mode clamp: stretch/spectral
  rejected while chained (verified on hardware).
- **Bench (elements→clouds, hardware):** engage 32 ms; sustained pair
  **373-401 µs of 750 (~52%)**, in=923/out=516 meters live, zero stalls;
  CC 23=1/3 rejected while chained; swap-while-chained: A fresh re-Init
  6 ms + routing survives, rings/B rejects with window math; off→on→off
  cycles clean (single-module budget/meters restore).
- **ORCHESTRATOR FIX during bench:** `cmd_switch`'s single-module "already
  active" guard swallowed the chain A-slot re-Init (while chained, A IS the
  active module) — chain semantics now own the request before the guard.
  CC-path note: amidi cannot address seq ports ("Unknown RawMidi 24:0") —
  CC-path validation is code-shared with the console path (exercised via
  console); the C seq client covers CC 32 if a dedicated test is ever needed.
- **OPEN (user decision):** accept 1-of-3 pairs (recommended: the canonical
  pairing), authorize the rings squeeze (budget-infeasible in worst case —
  not recommended), or authorize the plaits chain-tier image (edge-of-
  feasible, ~60 KB keep-list re-tier + second flash image).

### Rack T6b — rings→clouds made resident (2026-11-05, user pushback honored)

- **The user challenged the 1-of-3 verdict** — and the challenge was right:
  the T6 squeeze estimate missed the real lever. clouds' window image still
  carried the code of the modes the rack excludes. **Dead-mode demotion**
  (new `clouds-tc-demote-dead-modes.patch`, applies last): spectral
  stft/pvoc/frame_transform (1.9 KB) + lut_sine_window_4096 (16.4 KB, only
  PhaseVocoder::Init reads it) + CMSIS fft code (3.1 KB — the cfft trio had
  NO caller at all and got GC'd) + dead-KEEP'd src/lut_db tables (1.3 KB)
  → **tc_clouds 79,744 → 56,992 B (−22,752)**. Survey corrections: WSOLA/
  Looping/Diffuser/PitchShifter have NO out-of-line copies (fully inlined
  into the SRAM ProcessGranular — their marks are no-ops); lut_window/lut_sin
  are LIVE-shared with granular (kept). **Stretch stays reachable
  single-module** (correlator + body kept resident) — zero behavior change
  anywhere in single-module mode.
- **Geometry:** A slot grown to 0xE400 (rings-sized; elements still fits),
  clouds-B slot at +0xE400 → 115,360 ≤ window 118,784 (slack 3.4 KB);
  object arena 128 → 132 KiB (exact need 134,908); **heap 33.1 KB** (≥30 KB
  gate). Window size itself unchanged. Standalone clouds byte-identical.
- **Chained-rings budget clamp** (mirrors the clouds-B clamp): CC 18 = 1/4
  (sympathetic-string models, 511 µs standalone) rejected in modes 0-3 with
  the reason line; CC 20 mode-entry interdicted while the model byte is
  sympathetic; poly unclamped (sweep basis: poly-4 never exceeded the poly-1
  sympathetic worst); mode 4 easter egg unclamped. Single-module rings fully
  unclamped.
- **Bench (hardware):** `chain rings clouds` engaged 45 ms with exact
  validation numbers; pair sustained **268-357 µs of 750 (36-48%)** — well
  under the 93-99% worst-case estimates (those assumed sympathetic-heavy
  loads, which the clamp now gates); sympathetic reject fired live;
  clouds-B mode switch mid-chain clean; zero stalls; regressions: both
  chains re-engage with the new numbers (elements→clouds 93,744 window
  bytes), single-module budget/meters restore.
- **plaits→clouds (REPORT ONLY, needs user approval):** a chain-tier plaits
  keep-list (voice path + engine 0 + lut_sine ≈ 14.3 KB, demote ≈ 94.4 KB)
  fits the A slot trivially — but the rack has ONE plaits image, so the
  demotion would also move the fork-hot engines + 50.7 KB of LUTs to flash
  for SWITCHED plaits (T5b sustained numbers were all-SRAM; flash engines
  need a re-sweep). That is a sound/performance tradeoff on an accepted
  port — user must opt in.

### Rack T6c — plaits→clouds chain-tier (2026-11-06, user-approved)

- **`09-plaits-chain-tier.patch`** (applies last; `PLAITS_RAM_CODE_DEMOTABLE`
  macro = no-op under RACK_RP2350_TC): demotes the fork-hot engines (chord/
  particle/modal/string/chiptune/six_op/scanned), support DSP (fm operators/
  voice, string, resonator, string/modal voices) and wav_integrated_waves ×3
  (50.7 KB — chord their only window reader, chord itself now runs from
  flash). KEPT: voice path + engine 0 (va_render) + lut_sine.
  **tc_plaits 108,732 → 14,132 B.** Standalone plaits byte-identical
  (sha256 00a12fc2…). No geometry change: 14,132 ≤ A slot 58,368; all three
  pairs pass residency by construction; heap unchanged 33,120 B.
- **Bench (hardware):** `chain plaits clouds` engaged 38 ms (copies 71,316 B
  — the lightest pair); chained sustained **236-269 µs of 750 (32-36%)**
  across engine 0 (SRAM) AND the flash-engine sweep (vosim 59 / dx7 22 /
  speech 9) with notes; zero stalls. **Switched-plaits regression (48 kHz,
  budget 500):** flash engines run ~3× their T5b all-SRAM numbers (dx7
  221-269 µs = 54% of budget sustained; vosim ~210-244; speech 178-244;
  engine 0 unchanged 39-42 µs SRAM) — ALL sustained under budget across
  25 reports, stalls 0; the engine-CHANGE spikes (1009/538 µs, one block
  each: Reset+LoadUserData+cold XIP) are 16 ms-bank-absorbed. Any future
  offending engine = one-token promotion (44 KB window slack).
- **Remaining bench debt for T7:** chained/switched plaits per-engine ear
  pass by the user; the 30-min soak; full re-sweep if the user wants a
  per-engine ledger table update.

### Rack v2 — clouds grain alias bank + rack web panel (2026-11-07, user-accepted)

- **`e903145` firmware:** kCloudsAliasCcBase = 40 (midi.h) — CC 40-48 always
  addresses clouds' grain params 14-22 when clouds is the TARGET
  (single-module clouds via rack_clouds_alias_active; the B slot while
  chained via the widened rack_chain_wants_cc + translation in
  rack_chain_control_change). Native 14-22 keep their meaning (A's panel
  params while chained — the collision that forces the alias). Band range
  checked free: 36-63 unused by all four maps + rack system CCs. B-slot CCs
  now dropped during chain transitions (g_swapping guard). Strings updated
  ('chain' status/help, clouds kCcMap).
- **`473051f` tools/rack_panel.html:** the rack's web control panel (doc-map
  row in AGENTS.md). Web MIDI: CC 32 MODULE strip, all four maps with
  adapter-exact defaults + selector byte encodings, plaits ENGINE slider,
  mod-engine bench mux with firmware-exact quantizers + DEST select.
  Web Serial: CHAIN strip pair buttons + bpm/clock/log; parses firmware
  acks to follow switches and badge chains; connect/disconnect toggle +
  pagehide release. While chained: DUAL-PANEL layout (A + clouds), clouds
  live on the alias bank except the bypassed exciter, A's 23/24 retagged
  → CLOUDS + disabled, other tabs inert, MODE STRETCH snap-back.
- **Bench (user):** elements→clouds chained, all 9 grain knobs swept on the
  dual panel — ears accepted ("all fine"). Build: all 4 envs SUCCESS.
  Flash: 1200-baud touch while the panel console held the port (EBUSY
  lesson → Disconnect button + pagehide release added).

### Rack T6d — chained-rings clamp re-measured and narrowed (2026-11-07)

- The user asked which T1-T6 reductions were recoverable with the measured
  headroom. The chained-rings sympathetic clamp was the one likely-wrong
  reduction: the T6b prediction (756 µs) double-counted — measured pairs
  came in at roughly half the predictions. A/B: clamp disabled → the
  sympathetic pair MEASURED **692 µs of 750 (92%)**, zero stalls.
- **Blanket model clamp removed; narrow guard added:** CC 19 polyphony > 2
  rejected while a sympathetic model (1/4) is live chained (the only
  unmeasured corner — poly-4 sympathetic + clouds + mod). Bench-fix during
  verification: the first cut compared the RAW MIDI byte (rejected poly 2
  by mistake); the clamp now decodes poly = 1 + val/32 like the adapter.
- Verified: sympathetic poly 1-2 chained 655-692 µs sustained, zero stalls;
  poly 4 rejected with the reason line; single-module rings unclamped.
  Commit 650078a. The OTHER reductions stand: chained-clouds 0.512 s/ch
  (buffer-arena wall, hardware-bound), spectral excluded (single-module
  on-demand window extension possible someday, ~1.1 KB window slack),
  switched-plaits flash engines = the user-approved T6c trade (targeted
  promotions possible, 44 KB A-slot slack).

### Rack T5 stage 1 — C3↔RP2350 framed link (2026-11-08, benched)

- **Seed copy** `rack-uimcu/` from ctag-tbd/uimcu (read-only; IDF v5.5.4 —
  the seed's exact compiler is
  `~/.espressif/tools/riscv32-esp-elf/esp-14.2.0_20260121`; export.sh alone
  does NOT put the riscv toolchain on PATH). STOPGAP: the copy's
  `EXTRA_COMPONENT_DIRS` points at the READ-ONLY seed's `protocol` +
  `components/rapidjson` by absolute path (build reads, never writes);
  stages 2+ must vendor rapidjson and drop the protocol dep.
- **Protocol** `docs/ui-protocol.md` = the byte-exact contract:
  `[0x7E][len:u8][type:u8][payload…][crc8]` (CRC-8/ATM 0x07/0x00 over
  type+payload, no stuffing, len-bounded resync). Types: MIDI/SWITCH/
  MOD_SET/UI_CMD/CHAIN_ROUTE/TELEM + PRESET_* reserved (T4a). TELEM =
  `[page][module][mode][count][count×(cc,val,flags)][labels]`, flags
  mod-routed/chain-B/inert; deltas = same format count=1. MOD_SET param ids
  = `kModParamNames` indexes, values quantized exactly like the CC 33-35
  bench mux.
- **RP2350 side strictly additive**: `src/ui_link.h` (header-only, single-TU
  like midi.h) + `ui_link_observe_cc` observation call at the top of
  midiControlChange + `mod_engine_apply_ui_set` (the bench-mux setter
  verbatim) + 3 console rows (`ui`/`uitest`/`uisend`). MIDI frames →
  midiHandleEventPacket (the SAME funnel); SWITCH/CHAIN_ROUTE reuse the
  console paths verbatim. Audio path untouched — 2 s report byte-identical,
  elements report unchanged (130-176 µs of 496, zero stalls through the
  bench). All 4 envs: RAM +1056 B (512 B CC cache 4×128 + rx/tx), flash
  +6.2 KB each.
- **PINS (corrected at bench prep):** RP2350A UART0 pads — TX valid
  {0,2,12,14,16,18,28}, RX valid {1,3,13,15,17,19,29}, so **TX=GP16,
  RX=GP17** (the brief's TX=17/RX=16 would have panicked). C3: UART0
  TX=GPIO4, RX=GPIO5 (console is USB-Serial-JTAG → UART0 free). 921600 8N1,
  3V3 direct. WIRING: C3 GPIO4→RP2350 GP17, C3 GPIO5←RP2350 GP16, GND↔GND.
- **Bench (hardware, 2026-11-08):** RP2350 `uitest` = 6/6 codec PASS (MIDI,
  MOD_SET, 0x7E-in-payload, max-len 255, corrupt-CRC reject, resync); NO hw
  loopback exists on the RP2350's PL011-style UART (no LBM bit — checked the
  pico-sdk register headers). C3 boot selftest: `codec OK | hw-loop 32/32 |
  rtt ~205 µs` — REAL UART peripheral loopback (uart_set_loop_back) incl.
  one max-len frame at 921600. Cross-board echo NOT yet run: the UART wires
  are not connected (RP2350 `ui` rx=0 while the C3 probes every 5 s — the
  probe stream doubles as the wire test the moment the 3 conductors land).
- **Bench-caught fixes** (why hardware benches exist): (1) BOTH parsers'
  body-index counter was uint8_t — a len=255 frame body is 256 bytes, the
  counter wrapped and the parser stalled mid-frame swallowing everything
  downstream (uitest 4-6 all failed on the first flash); fixed to uint16_t
  on both ends. (2) C3 selftest case-1 assert was shifted by one payload
  byte (checked buf[1]==0x12 instead of buf[1]==kCmdEcho) — codec FAIL on a
  healthy codec; fixed. (3) The 1200-baud touch flash re-verified fine with
  the additive build.
- Commits: d901e76 (rack-uimcu), e278d69 (rack-rp2350); docs/ui-protocol.md
  + this ledger land in the stage-1 docs commit.

### Rack T5 stages 2-6 — display/pots/encoder/gestures/BLE (2026-11-08, code-verified only)

- **TELEM v2** (all three places updated together: docs/ui-protocol.md +
  ui_link.h + RackLink): cell = 4 bytes (cc, value, flags, color — the
  registry color class rides the wire, the C3 never duplicates the
  registry); flags bits 3-6 = first routed source id + 1; full snapshots
  end with a status tail [bpm (0 = external MIDI clk)][clocksrc][chain
  (0 off / 1-3 pair)][link (bit0 DIN saw bytes, bit1 UART fresh)]; deltas =
  count=1, no tail. New UI_CMD actions: 3 wired (0 = delta gate,
  1 = full-snapshot gate, 2 = global clock internal<->midi),
  4 = ASSIGN_CYCLE [cell][dir] — DEVIATION from the plan's "gesture sends
  MOD_SET": only the RP2350 knows the routing table, so the hold+pot
  gesture sends assign-cycle and the RP2350 cycles [off, src0..11] through
  mod_set_dest validation and replies a full TELEM; MOD_SET stays for the
  MOD pages' direct param writes. 5 = SET_GLOBAL [item][value] (item 0 =
  BPM, 20 + v*280/127) — MOD_SET is per-source and cannot express the
  MOD-C BPM pot.
- **MOD-A/B/C + SYS aux-cell tables** (RP2350-owned, labels + colors on the
  wire): MOD-A 16 cells src-major (L1 RATE/LEV/WAVE/PHASE, L2…) — DEVIATION
  from the design's param-major listing (rate ×4 then level ×4 …): each
  LFO's 4 params sit adjacent on the physical row; the design's "exact
  mapping decided during T5" clause covers this. MOD-B 16 = EU1-4
  steps/pulses/rot + gate1 len/glen + ADSR1 mode + DIV (EU1's division —
  the engine has per-source divisions only). MOD-C 13 = gate2 len/mask/
  glen + ADSR1/2 A/D/S/R + BPM + CLKSRC (globals = src 0xFF selector ids;
  CLKSRC pot turn inert by design — push toggles). SYS 8 = 4 module cells
  + chain pair/on + the two telemetry gates; presets = C3-local UI with
  PRESET_REQ [slot] send (T4a owns the RP2350 side). MOD cell `cc` field =
  opaque id 200+index.
- **C3 build stack**: RackUI fixed-page renderer REPLACES the seed
  UIMenu/menupages framework (nav model + DspProxy content don't map);
  Display rewritten SSD1309-mono → ST7789P3 RGB565 (configurable
  window/offset #defines = the panel-discovery step, dirty-row flushes,
  testpat discovery mode, hand-rolled SPI over esp_lcd — arbitrary masked
  window needed); RackPots HC4067 ~700 sweeps/s + NVS min/max calib
  (console `calib`, 8 s all-pot sweep) + 2-count deadband send-on-move +
  200 msg/s token bucket; RackConsole on USB-Serial-JTAG (ui/page/pots/
  calib/testpat/heap); BtMidiReceiver central→peripheral, BLE-MIDI →
  4-byte packets → SendMidi. Seed link/menu/net stack EXCLUDED from build
  (kept in tree, "seed reference, not built" headers); rapidjson vendored;
  ZERO build-time reads from ctag-tbd (the stage-1 EXTRA_COMPONENT_DIRS
  stopgap is gone).
- **C3 GPIO allocation** (rack-uimcu/README.md has the full table): 0 =
  mux COM (ADC1_CH0 — ADC2 dead while NimBLE holds the radio, GPIO4 = the
  other ADC1 pin is the link TX), 1/2/3/6 = mux addr (GPIO2 STRAPPING —
  ext 10k pull-up), 4/5 = UART link, 7/8/9 = encoder A/B/push (8/9
  STRAPPING — pull-ups REQUIRED; never hold push at power-on = download
  mode), 10/20/21 = SPI SCK/MOSI/DC. Pin-saving: CS→GND (bus exclusive),
  RST→3V3 RC (no firmware panel reset — power-cycle), BL→3V3 (no PWM
  dimming), mux E→GND.
- **Builds**: rack-uimcu clean build/ delete → SUCCESS, app 569,808 B
  (82% of the 3 MB slot free), Display FB = 43,168 B DRAM as budgeted;
  heap measured at boot + `heap` console cmd. rack-rp2350 all 4 envs:
  RAM 373,172 B (+4 B), flash 1,553,380 B (+3,664 B) vs stage 1 —
  main.cpp untouched (the v2 fits inside ui_link.h's existing hooks).
- RGB565 conversion caught + fixed during self-review: red #e6295e →
  0xE14B (arithmetic documented in Display.cpp).
- **NOT verified (needs wiring — benches staged per bring-up order):**
  stage 2 panel wiring + window/offset/MADCTL discovery (`testpat 4`) +
  SPI clock raise + IPS inversion; stage 3 HC4067 + encoder with the
  strapping pull-ups + `calib` + pots→CC; stage 4 cross-board UART (3
  wires) + live pages/colors/blink/status tail; stage 5 gesture
  round-trips (assign/overlay/SYS ring/clock toggle); stage 6 BLE central
  → "Rack UI" advertising + notes funnel + heap budget.
- Commits: feacd34 (rack-uimcu), 656aff5 (rack-rp2350); docs + this ledger
  in the stages-2-6 docs commit.

### Rack T5 — pot pickup (soft takeover) + mod-ride pots (2026-11-08, user-directed)

- The user caught the missing **soft takeover**: pots keep their physical
  positions across page switches, so the first move jumped the parameter
  to the pot position. Implemented C3-side (no protocol change): a pot
  whose position is >1 count from its cell's value NEVER writes — page
  switches / module changes arm pending via `PickupResync()` (full TELEM),
  external changes via `PickupCellNote()` (deltas; a value equal to what
  that pot sent within 400 ms = our own echo, never arms — a fast turn
  cannot be stalled by echo lag). Pending pots send nothing until the
  position crosses the value; the renderer shows a `<` / `>` arrow beside
  the value (the turn direction that picks it up; the displayed value
  stays the parameter's). Pickup crossings from the pot task get their
  own per-pass render diff (no TELEM needed to redraw an arrow).
- **Mod-ride pots** (user: "if modulated — pot not affecting it, but it
  can change one modulation parameter like for LFO - its changing LFO
  rate"): on a mod-routed cell (TELEM flags bits 3-6) the pot does NOT
  write the cell's CC — it RIDES the routed source's SPEED parameter:
  LFO→rate(1), Euclid→div(4), gate→div(4), ADSR→div(6) (gate-follow ADSR
  ignores div = no-op there). MUST MATCH kModParamNames indexes. No
  pickup on ride cells. To adjust a routed param's base, un-route first
  (hold+pot cycle to off), then the normal pickup applies. Modulated
  cells now render YELLOW as a whole (label + value + bar — the state
  accent was already on value/bar; the label joined).
- rack-uimcu only (RackPots/.hpp, RackUI/.hpp, README); rack-rp2350
  untouched. Build SUCCESS (app 570,512 B, 82% free), C3 reflashed
  (selftest codec OK | hw-loop 32/32 | rtt ~205 µs). Benches with the
  rest of stage 3-5 once pots/screen/UART are wired.

### Rack T5 — C3 pinout rearranged (2026-11-08, user-directed, pre-wiring)

- **Encoder A/B off the strapping pins** (user call): A=GPIO6, B=GPIO7;
  push stays GPIO9 (a push is only ever low while deliberately pressed;
  never hold at power-on). The mux S3 address line took GPIO8 — strap
  levels are don't-care in normal boot.
- **Display onto the 0-4 header** (user call): SCK=GPIO0, MOSI=GPIO1,
  DC=GPIO2. The mux SIG (the common pin — datasheet Z, breakouts silk
  SIG/COM; the user caught the naming) moved to **GPIO3 = ADC1_CH3**, the
  last free ADC1 pin (ADC1 = only GPIO0-4; GPIO4 = UART TX; ADC2 dead
  under NimBLE), and the address lines took the freed GPIO10/20/21
  (S0=10, S1=20, S2=21, S3=8).
- Strap audit of the final map: GPIO2 (DC out, floats at reset — no SCK
  edges, panel idles: harmless), GPIO3 (ADC input, high-Z at reset; ROM
  pull-up holds the JTAG-source strap; worst case a pot at min weakly
  pulls it low → only OpenOCD-over-USB-JTAG would break, which we never
  use), GPIO8 (S3 out, pull-up), GPIO9 (push, behavioral rule).
- Firmware: RackPots pins + ADC1_CHANNEL_3, Display SPI pins, UserInput
  encoder pins, README tables. Build SUCCESS (app 570,400 B, 82% free),
  C3 reflashed + boot verified. Final wiring table = rack-uimcu/README.md
  (the single source for the user's wiring pass).
- **RP2350 link pins moved to GP0/GP1** (user call): the RP2350 nano only
  POPULATES GPIO 0-5 + 26-29 — GP16/17 don't exist on the header. Valid
  UART0 pads from the populated set: TX {0,2,28} / RX {1,3,29}; picked
  TX=GP0, RX=GP1 (GP2/3 also valid; GP28/29 kept free as the only
  ADC-capable pins). Wiring: C3 GPIO4 (TX) → GP1, C3 GPIO5 (RX) ← GP0.
  Verified on hardware: `ui` prints `TX GP0 (-> C3 RX), RX GP1 (<- C3
  TX)`; all 4 envs rebuilt green; RP2350 reflashed. The whole GP16/17
  history in one line: brief said TX=17/RX=16 (reversed), bench prep
  corrected to TX=16/RX=17, the board then removed both — **TX=GP0,
  RX=GP1 is final**. docs/ui-protocol.md §7/§8.3 updated.

### Rack T5 — cross-board link BENCHED (2026-11-09, wires connected)

The user wired the 3 link conductors + both boards came up. Three
bench-caught bugs between the pin move and a working link — each found by
hardware symptoms, decoded from panic dumps + TWDT captures + addr2line:

1. **UserInput::Init never called** (crash loop): the ONLY call site lived
   in the seed UIMenu framework — excluded from the build. evQueue stayed
   nullptr → RackUI's first GetEvent assert-crashed
   (xQueueReceive pxQueue) ~470 ms after every boot. Fix: UserInput::Init()
   in app_main before RackUI::Init.
2. **Display::Flush exceeded the SPI DMA cap** (silent full-page failure):
   whole-page coalescing = 21584 px = 345344 bits > the C3's
   SPI_LL_DMA_MAX_BIT_LEN (2^18) — check_trans_valid rejected every
   full-page redraw ("txdata transfer > hardware max supported len"),
   logged + swallowed since the stages-2-6 build. Fix: chunk each coalesced
   row-run to ≤16000 px (256000 bits).
3. **pdMS_TO_TICKS(1) = 0 ticks** (the seed's own documented trap, re-hit):
   CONFIG_FREERTOS_HZ=100 → the pot scan task's "1 ms" delay was ZERO →
   the prio-3 task free-ran → IDLE starved (TWDT every 5 s) AND the
   prio-2 link task never ran again (silent wire, no probes, dead
   console). ADC + wire were NEVER at fault — a boot probe proved all
   five ADC1 channels complete (ch3 raw≈775 = the mux SIG mid-pot, ch4
   raw=4095 = idle-high UART TX). Fix: one full tick per complete sweep
   (~100 sweeps/s, each pot sampled every ~12 ms). Also migrated RackPots
   to esp_adc oneshot (kills the deprecation warnings; the legacy
   adc1_get_raw hung the same way).

**Bench results (both boards):** C3 boot selftest `codec OK | hw-loop
32/32 | rtt ~205 µs`; cross-board: probe → `rack link online` → full TELEM
`page=0 mod=0 count=16` (elements P1, 16 cells) both directions; RP2350
`ui` counters advance at the probe cadence, crc_err=0, `uisend` pushes a
192-byte v2 snapshot; RP2350 echo turnaround 4-8 µs, C3-measured page
request RTT ~50 ms (the RP2350's throttled reply path — bring-up number,
not a fault). The C3 cycles online→quiet→probe every ~5 s BY DESIGN (the
3 s offline watchdog with no idle traffic — stages 3-6 add real traffic).
C3 console answers `ui` (page cells + link stats). The 64 fps flood seen
mid-bench was the crash-loop era's selftest bursts + the RP2350's verbatim
echoes — gone with the fixes. Stages 2-6 benches (screen, pots, encoder,
gestures, BLE) are next per the wiring table.

### Rack T5 — Stage 2 display (ST7789P3 284×76 IPS bar) BENCH-VERIFIED (2026-11-09)

- **Final display wiring (4 wires + power):** `SCK = GPIO10`, `MOSI = GPIO20`,
  `DC = GPIO21`, `RST = GPIO7`, `CS = GND`, `BL = 3V3`. (Encoder moved to
  `A = GPIO0`, `B = GPIO1`, `PUSH = GPIO3` shared w/ mux SIG via 4.7k; mux
  `S0..S3 = GPIO8/9/2/6`.)
- **Root causes of the white-screen bugs & fixes:**
  1. **ESP-IDF `spi_master.c` clock idle polarity glitch after `RST` with
     `CS = GND`:** With `CS` tied to `GND`, ST7789 requires `SPI_MODE3`
     (`CPOL=1, CPHA=1`, `SCK` idling HIGH). In ESP-IDF v5.4.2,
     `spi_bus_initialize()` leaves `SCK` low (`0`), and `spi_bus_add_device()`
     does **not** call `spi_setup_device()` — `SPI_CK_IDLE_EDGE = 1` is only
     written to hardware inside `spi_device_acquire_bus()` or on the very first
     transaction. Pulsing `RST` before `spi_device_acquire_bus()` released the
     panel from reset while `SCK = 0`, and the first `sendCommand(0x11)` jumped
     `SCK` from `0 → 1` while `CS = GND`, shifting every subsequent byte by 1
     bit. **Fix:** Hold `GPIO10 (SCK)` HIGH in `app_main()` housekeeping and
     call `spi_device_acquire_bus()` + `spi_device_release_bus()` in
     `Display::Init()` *before* pulsing `GPIO7 (RST)`.
  2. **Panel inversion & CGRAM offsets (`TFT_eSPI` #3804 / PR #3769):** This
     2.25" 76×284 ST7789P3 panel has normal (non-inverted) polarity — `INVON`
     (`0x21`) inverts black (`0x0000`) to solid white. Set `ST7789_INVERT = 0`
     (`0x20` = `INVOFF`, matching `tft.invertDisplay(false)` in `#3804`) and
     landscape geometry `ST7789_MADCTL = 0x60`, `ST7789_XOFF = 18`,
     `ST7789_YOFF = 82` (CASET `18..301`, RASET `82..157`).
  3. **`bus.quadhd_io_num = -1`:** Explicitly initialized in `Display::Init()`
     so zero-initialized `spi_bus_config_t` does not route `SPI2_HD` to `GPIO0`.
- **Clock speed:** Bumped `ST7789_SPI_CLOCK_HZ` from 8 MHz bring-up rate to
  26.67 MHz (`26 * 1000 * 1000`, 80 MHz APB / 3, matching the 27 MHz
  `TFT_eSPI` recommendation).
- **Hardware verification:** Confirmed working and right-side up on `COM22` by
  the user.

### Rack T5 — Stages 3–5 Pots, Encoder & Mod Matrix HARDWARE-VERIFIED (2026-11-09)

1. **HC4067 16-Pot Scanner & Layout (`GPIO3` SIG, `S0..S3 = GPIO8/9/2/6`):**
   - **Zero ADC Jitter on `C0` (`ch = 0`):** Fixed tick-wakeup SAR ADC reference
     settling spike via a dummy warm-up conversion at loop start, plus 10-sample
     trimmed mean (middle 8). Measured `0` count drift when untouched.
   - **Row 2 Staggered Layout:** Row 2 cells shifted to `x = 25, 72, 166, 213`
     matching `docs/UI_sketch.png`.
   - **Full-Range Scaling for Discrete Selectors:** Fixed 97% dead-band on
     stepped CCs (e.g. Elements `MODEL 0..2`, `OMINOUS 0..1`; Clouds `MODE 0..2`,
     `QUALITY 0..3`; Rings `POLY 1..4`, `WAVE 0..2`; Plaits `ENGINE 0..97`).
     Pots sweep the full `0..100%` rotation and map proportionally across all valid
     options with true range display and proportional underline bar.

2. **Encoder Push & Modulation Routing (`A=GPIO0, B=GPIO1, PUSH=GPIO3 via 470R`):**
   - **Push-Button Protection:** Multiplexed SIG voltage drops on button press are
     evaluated *before* EMA filtering, freezing `filtRaw[0..15]`. Added 250 ms
     post-release guard and mid-sweep release detection (`otherRiseSum`) so button
     press/release never triggers parameter jumps or bogus modulation cycles.
   - **Modulation Selection & Top-Bar Display:** Hold button + rotate pot CW to step
     through `[OFF, LFO1..4, EUC1..4, GAT1..2, ADS1..2]`; status strip shows live
     `>MOD LFO1<` / `>MOD EUC2<` / `>MOD OFF<`. Releasing the button confirms the
     source without release glitching.
   - **CCW Modulation Unmapping (`>MOD OFF<`):** Turning pot fully CCW enters the
     hysteresis OFF zone (`raw <= 115`) to unmap modulation. Added `ClearResidualUnmap`
     and boot-time `ClearAllModSources()` to clear residual routes on the RP2350.
   - **Top-Bar Page Label Spacing:** Replaced fixed 44 px jump with dynamic string width
     measurement so `MOD-A/B/C` labels are never clipped by `INT 120`.

3. **SYS Page & Responsive Soft Takeover:**
   - **SYS Page Pots Enabled:** Pots 0..3 act as module radio selectors, pot 4 controls
     chain pair (1..3), pot 5 controls chain engage (0/1), pots 6/7 control telemetry
     gates (0/1), and pot 8 controls preset slots (0..3).
   - **Bucket-Aware Soft Takeover:** Discrete parameters are immediately active if the
     pot is already within the parameter's bucket; crossing the bucket boundary picks
     up the value without requiring a trip to the 0 or 127 endstops.
   - **BPM Unwrapping:** Unwrapped 8-bit overflow for `256..300 BPM` in telemetry
     status tail and guarded active turns across all pages.

- **Hardware verification:** User confirmed modulation assignment (`>MOD LFO1<`..`>MOD ADS2<`), CCW unmapping (`>MOD OFF<`), and pot responsiveness are working fine on hardware (2026-11-09).

### Rack T5 — Status Strip Polish, Telemetry Gates & BPM Stabilized (2026-11-09)

1. **Full Plugin Names in Status Bar:**
   - Expanded `kModNames` from `{"ELEM", "CLDS", "PLTS", "RNGS"}` to `{"ELEMENTS", "CLOUDS", "PLAITS", "RINGS"}`.
   - Screen width is 284 px; all full names and page identifiers fit with ample margin.
   - Module switch overlay now clearly displays `>ELEMENTS<`, `>CLOUDS<`, `>PLAITS<`, `>RINGS<`.

2. **Global SYSTEM Page Strip Header:**
   - Suppressed redundant plugin name prefix on the `SYS` page (`s_page == 5`). The status strip now renders `SYSTEM` directly rather than `ELEMENTS SYS`.

3. **Telemetry Gates Explained:**
   - `TELEM D` (SYS pot 6 / cell 6): Telemetry Deltas toggle. When enabled (127), the RP2350 emits throttled (>= 50 ms) single-cell updates (mode 1) when parameters move via external MIDI CC, serial console, or mod engines, keeping the screen synchronized with external controls.
   - `TELEM F` (SYS pot 7 / cell 7): Telemetry Full Snapshots push toggle. When enabled (127), the RP2350 pushes a full page telemetry snapshot (mode 0) on module switch or chain routing changes ("firmware acks drive the UI").

4. **BPM Jitter & Random Values Fixed:**
   - **Root cause:** In `RackLink.cpp`, an unconditional wrap check `if (rawBpm < 45) rawBpm += 256;` erroneously treated all low tempos `20..44 BPM` as uint8 overflows, causing them to jump to `276..300 BPM`.
   - **Protocol enhancement:** RP2350 `ui_link_status_tail` now flags tempos `>= 256` in bit 7 of `clocksrc` (`out[1] & 0x80`), while preserving `0/1` external MIDI state in bit 0.
   - **C3 parser fix:** `RackLink.cpp` checks `pl[off + 1] & 0x80` or hysteresis `s_telem.bpm >= 200` before unwrapping, protecting `20..44 BPM`.
   - **MOD-C synchronization:** On page 4 (`MOD-C`), `s_telem.bpm` is derived directly from `cell[11].value` (`(20 + value * 280 / 127)`), eliminating any lag or disagreement between cell 11 and the top status bar.
   - **Echo Storm & Periodic Page 0 Reset Eliminated:**
     - Discovered that after 3 seconds of idle, C3 `RackLink` declared the link "quiet - offline" and `probeNow()` sent an unconditional request for `kPageP1` (page 0), forcing RP2350 to replace the active page with Page 0 every 3 seconds! This caused active pages (`MOD-C`, `SYS`) to jitter, reset, and glitch intermittently.
     - Discovered an infinite echo bounce loop where C3 bounced RP2350 echo responses back to RP2350 every 80 µs. Removed echo-of-echo bouncing in `dispatchFrame`.
     - Added a clean heartbeat echo ping (`kCmdEcho [0xAA, 0x55]`) after 1.2 s of silence to keep the link permanently online without full page telemetry reloads, and bound `probeNow()` to `RackUI::GetPage()`.
   - **Both boards flashed:** Flashed RP2350 via 1200-baud touch (`firmware.uf2` to `RP2350` drive) and ESP32-C3 via `esptool.py` to `COM22`. Confirmed completely calm serial stream on `COM22` with zero offline drops.

### Rack T5 — 8-Page UI Architecture, Dedicated Mod Pages, Clamped Navigation & BPM FIFO Fix (2026-11-09)

1. **BPM Drift Root Cause & UART Saturation Fix:**
   - **Root cause:** Turning the BPM knob sent continuous `kUiCmdSetGlobal` packets. On every step, `ui_link.h` emitted a full ~200-byte `kUiTelemFull` snapshot frame. A quick turn flooded the 115200-baud UART FIFO with thousands of bytes. When the user stopped turning, C3's `RecentlyMoved` guard expired after 250 ms and C3 began processing the delayed snapshot queue, replaying the old BPM history (jumping back to 157 and slowly counting down to 125 before settling).
   - **Fix:** Removed `ui_link_send_telem(..., kUiTelemFull, 0)` from `kUiCmdSetGlobal`. Global parameter changes on the RP2350 no longer flood the UART FIFO.
   - **Relocation to SYSTEM Page:** Relocated `BPM` (pot 10, cell 10) and `CLKSRC` (pot 11, cell 11) to page 7 (`SYSTEM`), removing them from modulation pages entirely.

2. **Dedicated, Un-Shrunk Modulation Pages (8-Page Architecture):**
   - Expanded UI from 6 to 8 pages:
     - `kPageP1 (0)`: `<PLUGIN> P1` (Plugin parameters)
     - `kPageP2 (1)`: `<PLUGIN> P2` (Plugin secondary parameters)
     - `kPageModLfo12 (2)`: `MOD-LFO 1/2`
     - `kPageModLfo34 (3)`: `MOD-LFO 3/4`
     - `kPageModTrig (4)`: `MOD-TRIG`
     - `kPageModGate (5)`: `MOD-GATE`
     - `kPageModEnv (6)`: `MOD-ENV`
     - `kPageSys (7)`: `SYSTEM`
   - Modulation headers on pages 2..6 render clean labels without plugin name prefixes.
   - Pair A maps to Row 0 (pots 0..5, `x = 2, 49, 96, 143, 190, 237`) and Pair B maps to Row 2 (pots 10..15, identical `x` coordinates), aligning identical parameters column-for-column.

3. **Euclidean Generators Reduced (4 → 2):**
   - Reduced Euclidean generators from 4 to 2 (`EU1`, `EU2`), bringing total modulation sources to 10 (`MOD_SOURCE_COUNT = 10`): 4 LFOs, 2 EU, 2 GATE, 2 ADSR.
   - Updated rotary mod assign list: `OFF, LFO1..4, EUC1..2, GAT1..2, ADS1..2`.

4. **Eliminated Redundant Parameters on P2:**
   - `ELEMENTS P2`: Reduced to only 2 cells — `MODEL` (CC 28) on pot 0 and `OMINOUS` (CC 29) on pot 1. Pots 2..15 are blank.
   - `CLOUDS`, `PLAITS`, `RINGS`: Set P2 cell count to 0 (no duplicate parameters).

5. **Clamped Page Navigation & Empty P2 Skip:**
   - Rotary encoder page navigation clamped to `[0..7]`: CCW stops at `P1` (no wrap to `SYSTEM`); CW stops at `SYSTEM` (no wrap to `P1`).
   - Plugins without a secondary page (`CLOUDS`, `PLAITS`, `RINGS`) automatically skip `P2` during navigation — `P1` advances directly to `MOD-LFO 1/2`, and `MOD-LFO 1/2` steps directly back to `P1`. If the active module changes to one without P2 while on page 1, the UI auto-resets to `P1` (page 0).

6. **SYSTEM Page Layout Refinement:**
   - Moved `BPM` (20..300) to Pot 7 (slot 6) and `CLKSRC` (INT/MIDI) to Pot 8 (slot 7) in Row 1, alongside `PRESET` (Pot 9, slot 8).
   - Moved `TELEM D` to Pot 15 (slot 14) and `TELEM F` to Pot 16 (slot 15) in the bottom-right corner of Row 2.
   - Updated C3 `RackLink` telemetry BPM extraction, local cell writes, `RackUI` soft takeover bucket logic, discrete toggle mappings, and the push-ring cycle indicator (`ringCell: 4, 5, 8, 14, 15`).

- **Hardware verification:** Confirmed working on real hardware by the user (2026-11-09): clamped page scrolling without wrap, single-page plugins automatically skipping empty P2, dedicated modulation page layout and alignment, smooth non-jittering BPM on Pot 7, CLKSRC on Pot 8, and TELEM D/F on Pots 15/16.

### Rack T5 — Elements Front-Panel Realignment, Pitch Control & Standardized Palette (2026-11-10)

1. **Elements Resonator PITCH Knob Replaces MOD:**
   - Replaced redundant CC 1 `MOD` parameter with direct resonator `PITCH` (-24..+24 semitones, centered at 0 st at pot value 64).
   - `shared_pitch` in `module_elements.h` scales $(v - 0.5\text{f}) \times 48.0\text{f}$, updating `perf_state.modulation` on every audio block and in CC dispatch.
   - PITCH is directly modulatable by any mod-engine source (LFO, envelope, etc.).
   - `RackUI::potToDisplayValue()` formats CC 1 as signed integer `-24`..`+24` semitones with center `0` and a full 0..127 underline bar.

2. **Elements P1 Physical Front-Panel Alignment (Pots 1–16):**
   - **Row 0 (Pots 1–6):** `BOW` (white, CC 15), `BLOW` (pink, CC 17), `STRIKE` (teal, CC 20), `PITCH` (white, CC 1), `STRENGTH` (white, CC 11), `CONTOUR` (white, CC 14).
   - **Row 1 (Pots 7–10):** `FLOW` (pink, CC 18), `MALLET` (teal, CC 21), `GEOMETRY` (white, CC 23), `BRIGHTNESS` (white, CC 24).
   - **Row 2 (Pots 11–16):** `TIMBRE` (white, CC 16 - bow timbre), `TIMBRE` (pink, CC 19 - blow timbre), `TIMBRE` (teal, CC 22 - strike timbre), `DAMPING` (white, CC 25), `POSITION` (white, CC 26), `SPACE` (white, CC 27).

3. **Standardized Color Palette & ST7789P3 Panel BGR Fix:**
   - Standardized all module parameters to 3 base colors:
     - White (`kUiColorWhite = 0`, RGB565 `0xE73C`)
     - Pink (`kUiColorPink = 1`, RGB565 `0xE14B`, `#e6295e`)
     - Teal (`kUiColorTeal = 2`, RGB565 `0x1517`, `#17a3bd`)
   - Modulated parameter accent: Yellow (`kUiColorYellow = 3`, RGB565 `0xF683`, `#f2d21f`) for text, value, and underline bar.
   - **ST7789P3 BGR Bug Resolution:** Discovered root cause for yellow displaying as cyan/teal and pink displaying as purple/blue in hardware: `#define ST7789_MADCTL 0x60` configured landscape mode with bit 3 (RGB/BGR selector) cleared to RGB. The physical panel has a BGR subpixel filter. Changed `ST7789_MADCTL` to `0x68` (`0x60 | 0x08`), aligning ST7789 controller color order with the physical display.
   - **MOD-LFO Pages (Pages 2 & 3):** Updated all cells in `kUiModLfo12Cells` and `kUiModLfo34Cells` to Yellow (`kUiColorYellow = 3`).
   - **Clouds P1:** Set `SIZE` (CC 41) & `TEXTURE` (CC 44) to Teal; set `POSITION` (CC 40) & `DENSITY` (CC 43) to Pink; remaining cells White.
   - **Plaits P1:** Set all 13 cells to White.
   - **Web Panel & Documentation:** Synchronized `tools/rack_panel.html` and `docs/ui-protocol.md`.

4. **Build & Flash:**
   - RP2350: Built `rack-elements` via PlatformIO, flashed via 1200-baud touch and UF2 copy to `RP2350` drive (`E:\`).
   - ESP32-C3: Built `rack-uimcu` via Ninja/ESP-IDF, flashed to `COM22` at 460800 baud.

### Rack T5 — Elements P2 PLAY Trigger & Mod-Engine Trigger Routing (2026-11-10)

1. **Elements PLAY Trigger (CC 64) on P2:**
   - Added `PLAY` (pseudo-CC 64) as cell 2 on `kUiPageElementsP2`, alongside `MODEL` (cell 0, CC 28) and `OMINOUS` (cell 1, CC 29). P2 cell count expanded to 3.
   - Connected `PLAY` to `pending_play_pulse` in `module_elements.h`, asserting `perf_state.gate` for one block on triggers to excite the physical mallet strike and multistage attack envelope in both standalone `render()` and chained `chain_render_a()`.
   - Exposed CC 64 in `kModDestinations` under `kModTriggerCcs` (`RACK_TRIG_PULSE`), allowing Euclidean (`EUC1/2`) and Gate (`GAT1/2`) trigger modulation sources to drive exciter strikes.
   - Wired `mod_fire_trigger()` in `mod_engine.h` for `RACK_MODULE_ELEMENTS` to `rack_modules::mod_elements::mod_trigger_cc()`.
   - Updated C3 `RackUI::p1p2DiscreteCount()` with 2-state boolean wire mapping for CC 64, `tools/rack_panel.html` with a momentary `PLAY` button, and `docs/ui-protocol.md`.

### Rack T5 — Mod-Assign Sweep Wiping Fix & Authoritative TELEM Sync (2026-11-10)

1. **Mod-Assign Sweep Overwriting & Unrouting Bug:**
   - **Root cause:** While the user held the encoder push button and rotated a pot to choose a modulation source, `RackPots::potTask()` was calling `applyModAssign(p, choice)` live on **every single intermediate choice** as the pot swept across values.
     Because each mod source in `g_mod_engine.src[10]` supports one destination, sweeping through choice 5 (`EUC 1`) immediately stole `EUC 1` away from `PLAY` (CC 64). When the user continued rotating to choice 6 or beyond, `applyModAssign` saw `localSrc == 5` and sent `SendModSet(4, destParam(4), 0)`, actively clearing `EUC 1` to 0 (`OFF`). Furthermore, pots near 12 o'clock center (like `PITCH`) start at choice 5 (`EUC 1`), causing any touch to immediately steal and clear `EUC 1`. Finally, `assignPotLocked` was not guarded against re-evaluating, allowing channel noise to hop between pots while held.
   - **Fix:**
     - In `rack-uimcu/main/RackPots.cpp`:
       - Guarded `assignPotLocked < 0` so the pot selection locks firmly onto the first pot that moves >= 25 counts from anchor.
       - During rotation while button is held: ONLY update the screen top bar (`RackUI::NoteAssignPot(p, choice)`) and record `assignLastChoice = choice`. Completely removed `applyModAssign` from the rotation loop so zero frames are sent while sweeping.
       - On button release (`if (pushActive)`): call `applyModAssign((uint8_t)assignPotLocked, assignLastChoice)` **once** to commit the user's finalized choice.
       - In `applyModAssign`: when mapping a source to a cell, clear any other cell in `s_potModSrc` that was previously using that source.
     - In `rack-rp2350/src/ui_link.h`:
       - When a `MOD_SET` frame modifies a `dest` parameter (`pl[1] == n - 1`), RP2350 immediately responds with `ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0)`, giving C3 authoritative real-time confirmation of the new routing matrix without requiring page switches.
   - **Flashing & Status:**
     - Built and flashed `firmware.uf2` to RP2350 (`COM15`) and `rack-uimcu.bin` to ESP32-C3 (`COM22`). Both boards online and communicating.
   - **Hardware verification:** Confirmed working on hardware by the user (2026-11-10): mapping is completely stable across P1/P2, and changing or clearing mod assignments on other parameters no longer unmaps PLAY. Committed: `227ad5a`.

### Rack T5 — Continuous Gate Modulation & Plugin Output Volume (VCA) (2026-11-10)

1. **Continuous Gate & Euclidean Modulation Dispatch:**
   - **Root cause:** `mod_engine_process_block()` branched strictly by source type: `MOD_LFO` and `MOD_ADSR` were dispatched to continuous destinations via `mod_write_cc()`, while `MOD_GATE` and `MOD_EUCLID` only checked `mod_trigger_kind()`. Any destination lacking a trigger input (like Plaits `LEVEL` CC 25) returned `kind < 0` and was dropped as inert.
   - **Fix:** In `mod_engine.h`:
     - Added `mod_is_continuous(module_id, cc)` helper.
     - For `MOD_GATE`: if `is_cont`, dispatch `mod_write_cc(active_module, dest, s.out)` (rhythmic 0.0/1.0 square gate for audio chopping/stuttering); if trigger, handle rising edges and latches.
     - For `MOD_EUCLID`: if `is_cont`, dispatch `mod_write_cc(active_module, dest, s.out)` (euclidean 0.0/1.0 pulse level); if trigger, handle step pulse.
     - In `mod_set_dest()`: determine `can_continuous = (LFO || ADSR || GATE || EUCLID)` and `can_trigger = (EUCLID || GATE)`. Log continuous routing if valid, trigger routing if valid, and only log `INERT` if neither.
     - When unassigning or switching a continuous destination (`dest == 0`), automatically restore the panel's static knob value from `g_ui_cc_cache` via `ui_link_restore_cached_cc()`.
     - In `ui_link.h`: updated `ui_link_cell_flags()` to clear `kUiFlagInert` when GATE or EUCLID are routed to continuous CCs.

2. **Master Output Volume (VCA, CC 7) across Modules:**
   - **Elements (`module_elements.h`):** Added `shared_vca` (CC 7, 0..127 float 0.0..1.0, default 1.0f). Multiplies output in `render()` and `chain_render_a()`. Added to `kCcMap` and `kModContinuousCcs` (count 17). Placed on **Elements P2** (Cell 3, pot 4; P2 count expands to 4: `MODEL`, `OMINOUS`, `PLAY`, `VCA`).
   - **Rings (`module_rings.h`):** Added `shared_vca` (CC 7, 0..127 float 0.0..1.0, default 1.0f). Multiplies output in `render()` and `chain_render_a()`. Added to `kCcMap` and `kModContinuousCcs` (count 6). Placed on **Rings P1** (Cell 11, pot 12; P1 count expands to 12).
   - **Plaits (`module_plaits.h`):** Aliased CC 7 to `shared_level` (pot 10 on P1) in `control_change()` and `mod_set_cc_float()`. Added CC 7 to `kModContinuousCcs` (count 11).
   - **UI Cache Defaults:** Initialized CC 7 cache defaults to 127 across all modules in `ui_link_uart_init()`, and Plaits CC 25 default to 102.

3. **Build, Flash & Bench Test:**
   - Built with PlatformIO: all four targets (`rack-elements`, `rack-clouds`, `rack-plaits`, `rack-rings`) compiled cleanly.
   - Flashed RP2350 via 1200-baud touch and `firmware.uf2` copy to `E:\`.
   - Verified via serial console: `mod gate1 dest 7` routes as continuous on Elements without inert flag, `mod gate1 dest 0` restores static value, module switches to Plaits (CC 7/25 continuous) and Rings (CC 7 continuous) run with zero stalls and low process times (~58 µs Plaits, ~291 µs Rings, ~154 µs Elements).
   - Committed: `386a243`.

### Rack T5 — 16 Canonical Trance Gate Presets + Barlow & Sioros-Guedes Generative Rhythmic Gate (2026-11-10)

1. **Architecture & 6-Pot Surface:**
   - Evaluated 4 non-Euclidean architectures against hardware constraints (RP2350 M33 + ESP32-C3 ST7789 display). Selected **Architecture 1: Metrical Indispensability (Clarence Barlow) + Syncopation Anticipation (Sioros-Guedes model)** combined with the **16 Curated Canonical 16-Step Trance Gate Presets**.
   - Mapped 6 physical pots symmetrically per gate on Page 5 (`MOD-GATE`):
     - Pot 0 / 10: `MODE` (`GEN` = 0, `01:ALT8` .. `16:RISE`)
     - Pot 1 / 11: `DENS` (1..16 active steps along 4/4 Barlow hierarchy)
     - Pot 2 / 12: `SYNC` (0..100% Sioros-Guedes left-shift anticipation with anti-offset protection)
     - Pot 3 / 13: `GLEN` (1..100% pulse width / gate duty cycle)
     - Pot 4 / 14: `CLK`  (`GLB`, `INT`, `MIDI`)
     - Pot 5 / 15: `DIV`  (`1/1` .. `1/32T`, default `1/16`)
     - Parameter 6: `DEST` (destination CC, assigned via push button cycle gesture)

2. **DSP Firmware Implementation (`rack-rp2350/src/mod_engine.h`):**
   - Added `kTranceGatePresets[16]` lookup table containing the 16 canonical trance gate patterns.
   - Added `kBarlow16[16]` metric indispensability array and `kSyncCandidates[8]`.
   - Implemented `mod_gate_recompute(ModSource& s)`: deterministic 16-step mask calculation for both preset trimming and generative Barlow 4/4 skeletons with reproducible syncopated pickup shifts.
   - Added asymmetric one-pole slew filter (~1 ms attack, ~3 ms release) to continuous gate modulation (`s.out`), eliminating zero-crossing clicks on low bass pads without compromising punchy 16th transients.
   - Updated `kModParamNames[MOD_GATE]` to 7 params (`mode`, `dens`, `sync`, `glen`, `clk`, `div`, `dest`) with uniform tail rule.
   - Supported console commands (`mod gate1 mode <gen|name|0-16>`, `mod gate1 dens <1-16>`, `mod gate1 sync <0-100%>`, `mod gate1 glen <0-1>`, `mod gate1 mask <hex>`) and `mod?` telemetry dump.

3. **UI Link & UI MCU Integration (`ui_link.h`, `RackPots.cpp`, `RackUI.cpp`):**
   - Updated `kUiModGateCells[16]` in `ui_link.h` to expose all 6 parameters per gate and formatted 0..127 telemetry.
   - In `RackPots.cpp`: updated `rideParam` (param 5) and `destParam` (param 6) for `MOD_GATE` to maintain seamless modulation assignment and speed riding.
   - In `RackUI.cpp`: mapped Pots 0..5 to Gate 1 and Pots 10..15 to Gate 2, configured discrete ranges in `potToDisplayValue`, and formatted on-screen values (`GEN`, `ALT8`, `PUMP`, etc., `GLB`/`INT`/`MIDI`, and clock divisions).

4. **Build Verification:**
   - Both `rack-rp2350` (all 4 targets: `elements`, `clouds`, `plaits`, `rings`) and `rack-uimcu` compiled cleanly with zero errors.

### Rack T5 — Architecture 4: Shift-Register Mutation (TMR) & Hamming Markov Chains (MRKV) (2026-11-10)

1. **Architecture 4 Implementation (`mod_engine.h`):**
   - Implemented Mode 17 (`TMR`): Turing Machine shift-register mutation.
     - Bit register seeded via Barlow metric indispensability `kBarlow16`.
     - At `sync == 0%`, the loop is 100% locked.
     - At `sync > 0%`, bit mutations are inversely weighted by metric indispensability `(15 - Barlow[step])`, keeping strong beats anchored while weak subdivisions mutate dynamically.
   - Implemented Mode 18 (`MRKV`): Whole-Bar Hamming-Distance Markov Chain transitions across the 16 canonical trance gate presets.
     - Computes population count `mod_hamming_dist(a, b)` using `__builtin_popcount`.
     - At bar boundaries (`step == 0`), transitions to candidate presets with probabilities decaying exponentially by Hamming distance from the current preset.
     - `sync` controls transition rate and temperature; `sync == 0%` locks current preset indefinitely.
   - Updated console commands (`mod gate1 mode tmr|mrkv`), help text, and telemetry.
   - Verified live mask mutation and locking over serial console.
   - Committed: `e093006`.

### Rack T5 — Textual Parameter Display across Modulation Pages (2026-11-10)

1. **Textual Display Formatting in UI MCU (`RackUI.cpp`):**
   - Replaced raw numeric display with text labels across all modulation engines and system pages:
     - **LFO (Pages 2 & 3):**
       - `WAVE` (0..4) -> `SINE`, `TRI`, `SAW`, `SQR`, `S&H`
       - `MODE` (0..1) -> `HZ`, `CLK`
       - `DIV` (0..9)  -> `1/1`, `1/2`, `1/4`, `1/8`, `1/16`, `1/32`, `1/4T`, `1/8T`, `16T`, `32T`
     - **TRIG (Page 4):**
       - `CLK` (0..2)  -> `GLB`, `INT`, `MIDI`
       - `DIV` (0..9)  -> `1/1` .. `32T`
     - **GATE (Page 5):**
       - `MODE` (0..18) -> `GEN`, `ALT8`..`RISE`, `TMR`, `MRKV`
       - `SYNC` / `GLEN` -> formatted with `%` suffix
       - `CLK` (0..2)  -> `GLB`, `INT`, `MIDI`
       - `DIV` (0..9)  -> `1/1` .. `32T`
     - **ENV (Page 6):**
       - `MODE` (0..1) -> `GATE`, `RTRG`
       - `DIV` (0..9)  -> `1/1` .. `32T`
     - **SYS (Page 7):**
       - `CHAIN PR` (1..3) -> `E>C`, `P>C`, `R>C`
       - `CHAIN ON`, `TELEM D`, `TELEM F` -> `OFF`, `ON`
       - `CLKSRC` -> `INT`, `MIDI`
     - **P1/P2 Discrete & Booleans:**
       - Boolean toggles (`PLAY`, `FREEZE`, `TRIG`, `STRUM`, `OMINOUS`) -> `OFF`, `ON`
       - Elements `MODEL` -> `MODAL`, `STRING`, `STRINGS`
       - Clouds `MODE` -> `GRAN`, `STRE`, `LOOP`
       - Clouds `QUALITY` -> `32-16`, `32-8`, `16-16`, `16-8`
       - Rings `MODE` -> `STEREO`, `WAVE`, `EL-ST`, `EL-EX`, `EGG`
       - Rings `MODEL/FX` -> `MODAL`, `SYMP`, `STRING`, `FM`, `SYMP-Q`, `STR+RV` (swaps to `FORMNT`..`REVB2` in Easter Egg mode)
       - Rings `WAVE` -> `SQR`, `IMP`, `SAW` (swaps to `BOW`, `BLOW`, `STRK` in Mini-Elements modes)
       - Rings `CHORD TB` -> `BRYAN`, `JON`, `JOE`
   - Fixed `GetPotBucketBounds` in `RackPots.cpp`/`RackUI.cpp` so Page 4 and Page 5 pots have discrete bucket deadbands without interference (expanded `n <= 19`).
   - Updated `tools/rack_panel.html` with the 7-parameter GATE definition (including `TMR` and `MRKV`) to synchronize the web panel with the firmware.
   - Flashed `COM22` (ESP32-C3) and confirmed active communication with RP2350 at 921600 baud (`0` CRC errors, `8 us` turnaround).







### Rack T4a — Presets + Stage 6 BLE bench (2026-11-11, hardware-verified)

**Stage 6 BLE-MIDI PERIPHERAL benched first** (NimBLE peripheral was
already in the running C3 firmware — no reflash needed):
- New bench tool `tools/ble_midi_sender.py` (bleak central; scans by the
  BLE-MIDI service UUID 03B80E5A-..., connects, sends Apple-BLE-MIDI
  encoded notes/CCs). New helper `tools/serial_capture.py` (COM-port
  capture; **DTR must be asserted — Adafruit_USBD_CDC gates TX on DTR**,
  an unstated-DTR capture silently reads nothing).
- PASS: scan finds "Rack UI" by service UUID; central connects
  (`BLE_MIDI: central connected (handle 1)`); 12/12 note-ons + 12 CCs
  delivered across two walks (`ble msgs=36 drops=0` on the C3 `ui` line;
  RP2350 `notes=` 12→24, OUT meter live); zero stalls, Process numbers
  unchanged (BLE lives on the C3); heap with NimBLE + 43 KB FB + ACTIVE
  connection: free=103,496 / largest=77,824 — budget healthy.
- The strip "BLE" marker while connected is rendered in code — needs the
  user's eyes on the panel to close visually.

**T4a presets, both ends:**
- RP2350 (`preset.h`, header-only single-TU like midi.h/ui_link.h; all 4
  envs build): 871 B v1 blob = magic "RKPR" + version + active module +
  chain pair/engage + clock bus (global src + BPM f32) + 10 mod-source
  records (28 B each, field-by-field, clamped on read + eu/gate masks
  recomputed) + cc_seen[4] bitmaps + cc_cache[4][128] + crc16-CCITT.
  Console: `preset [dump|collect|load|apply|reload]` (rows `PRxx:<hex>`,
  24 B/row; collect mode swallows rows before command dispatch).
  PRESET_REQ [slot] -> chunked PART burst [slot][seq][total][data<=252];
  PRESET_APPLY reassembly -> validate -> `preset_apply_blob` (clock+mod
  -> cache+seen -> module swap via g_pending_module -> chain via
  g_pending_chain) -> full-TELEM ack.
- **CC-cache replay** (`preset_replay_module`): after EVERY factory
  re-Init (swap, chain engage A+B, chain leave A, chain A-reinit) every
  SEEN CC of the module is re-sent through its control_change hook —
  retires the T2 "factory defaults on swap desync the panel" debt.
  Skips CC 64 (momentary pulses must not fire from stored latches) and
  the chained-rings poly clamp. Replay runs on CORE 1 — verified the
  adapters' control_change hooks are print-free (rings'
  chain_rejects_cc is called from the core-0 wrapper, not the adapter).
- **Dirty tracking**: `preset_mark_dirty()` (defined in ui_link.h) at
  ui_link_observe_cc, the mod setters/dest/bpm/clock (mod_engine.h),
  rack_request_switch/cmd_switch/cmd_chain. Core 0's loop pushes the
  blob via `preset_autosave_poll()` 3 s after the last change.
- C3 (`PresetBank` raw esp_partition + `PresetStore` flows): new
  `presets` data partition (subtype 0x99, 0x3E0000, 128 KB — the unused
  `storage` littlefs shrunk 0xF0000→0xD0000; it held nothing). 16 slots
  x 1 sector + 4 rotating autosave sectors ("RKPB"/"RKPA" records,
  crc16, rpLen/c3Len). RackLink routes RX PRESET_PART -> PresetStore
  (reassembles -> SaveSlot / autosave) and sends chunked PRESET_APPLY.
  SYS ring extended to 7 items (pair, engage, slot-cycle 0-15, SAVE,
  LOAD, telem-d, telem-f; strip shows `SYSTEM SAVE>`/`SYSTEM LOAD>`);
  pot 8 = slot select. Boot restore: PresetStore::Poll() (UI task)
  fires once on link-online + first TELEM.
- **Bench-caught bugs (why hardware benches exist):**
  1. **C3 autosave collect torn apart by its own timeout**: the
     implicit re-arm `s_collect = Collect{}` left start_us=0 → the 3 s
     timeout fired on the NEXT UI-task pass, tearing down the collect
     between every chunk ("collect timed out 1/4" + "PART hole" spam).
     Fix: start_us = esp_timer_get_time() at re-arm.
  2. **Blob-offset arithmetic drift**: hardcoded kPresetOffSeen=301
     assumed a 293-byte mod record; mod_engine_preset_write writes 285
     (1+4+10x28) → the cache/crc shifted down, bytes 871-878 = stack
     garbage in every dump → deterministic crc rejects. Symptom:
     collect-time crc EQUALS the embedded crc (perfect collect) but
     validate computes differently — a DBG head/tail print pinned it to
     the crc bytes. Fix: offsets derived from kModPresetSrcRecord
     (871 B total; blob constants now self-maintain).
  3. **APPLY chunk-cap mismatch**: C3 SendPresetApply chunked at 253 B
     vs the RP2350's kPresetChunkData=252 → reassembly misaligned, crc
     read back as garbage ("APPLY REJECTED (crc 0000 != ...)"). Fix:
     252 both ends (comment: MUST MATCH).
- **Bench (hardware, all PASS):** console hex round-trip (dump →
  collect → validate → apply, same-module no-swap path); state restore
  (bpm 250/dest 27/rate 40/plaits → load → elements/120/dest0/rate1.00
  exactly, 29 ms swap, zero stalls); C3 `preset save 3` → "slot 3
  saved (871 B rp part + 12 B c3 part): OK"; C3 `preset load 3` →
  applied incl. module swap; autosave pushes ("autosave written
  (871 B rp part, seq N): OK" on every settled change); **power-cycle
  boot restore** (both boards reset → boot restore: autosave found
  seq 39 → applied clock+mod+cache → "module swap to plaits queued" →
  "switched to plaits in 25 ms" → autosave re-pushed seq 40); reject
  paths (corrupt crc + bad magic rejected with reasons, state
  untouched).
- **Bench tooling for Windows (new `tools/`)**: flash_rp2350.py (1200-
  baud CDC touch → BOOTSEL MSC find by volume label → UF2 copy),
  serial_capture.py (DTR-aware), build_uimcu.bat — **the idf.py trap:
  its __main__ silently SKIPS main() whenever MSYSTEM is set (prints
  the "MSys/Mingw no longer supported" warning and exits 0), and Git
  Bash injects MSYSTEM into every child (env -u cannot remove it — the
  MSYS runtime re-adds it). Builds must run through cmd.exe with
  MSYSTEM cleared (the .bat does that + sets IDF_PATH/python-env/
  toolchain PATH for this machine: IDF v5.4.2 at ~/esp/v5.4.2).**
- BLE-MIDI central bench script + capture helpers stay in tools/ for T7.
### Rack T5 — BLE-MIDI dual-role + SMC-PAD central WORKING (2026-11-11)

- **Stage 6 peripheral benched first** (bleak central sender,
  `tools/ble_midi_sender.py`): 12/12 notes + 12 CCs delivered, 0 drops,
  heap healthy (103 KB free with NimBLE + FB + active connection). CDC
  gotcha recorded: Adafruit_USBD_CDC gates TX on DTR — a serial capture
  without DTR reads nothing.
- **Central mode added** (user request: connect the desk's SMC-PAD):
  dual-role `BtMidiReceiver` — PERIPHERAL (advertise "Rack UI", the
  benched stage-6 path) / CENTRAL (scan → auto-connect to the first
  BLE-MIDI advertiser → subscribe → the SAME parseBleMidi → SendMidi
  funnel). Role switch + on/off from SYS page cells 9/10 (pot 9 = BLE
  on/off, pot 10 = BLE MOD PER/CEN — C3-local, NVS-persisted) and the
  console (`ble [status|on|off|per|cen|scan]`). Mode switch terminates
  the current role's connection and restarts the other (DISCONNECT
  handler follows the NEW mode; role of the live connection tracked via
  ble_gap_conn_find's desc.role).
- **ROOT CAUSE of the silent pad (the long debugging story):** the pad
  (JieLi BR25 / M-VAVE family, MIDI I/O chr = SINCO UUID
  7772E5DB-3868-4112-A1A9-F2669D106BF3, NOT the Apple C702) accepts the
  connection and even a bare CCCD write, but its stock firmware only
  ARMS THE STREAM for the full chained subscribe of the ctag uimcu C3
  panel: explicit MTU exchange on connect → service/chr discovery →
  0x2902 CCCD via RELIABLE write → Apple-spec READ of the MIDI I/O
  characteristic (chained so one GATT procedure is in flight) →
  write_no_rsp keepalive {0x80,0x80,0xF8}. Also `ble_store_config_init()`
  in Init and the uimcu sdkconfig (NVS_PERSIST=y, BT_CTRL_BLE_MAX_ACT=10,
  NO host flow control — an earlier flow-control experiment came from
  the WRONG reference: the classic-ESP32 bba config, not the C3 uimcu).
- **Dead ends eliminated with evidence:** IDF 5.4.2 vs 6.1 (both silent
  with the wrong flow — the 6.1 build needed explicit NimBLE include
  dirs in main/CMakeLists.txt since 6.x stopped propagating them);
  subscribe-everything (all 4 notify chars — EBUSY races + pointless);
  250 ms note-off keepalive (no effect alone); fast conn interval (no
  effect alone); PC-side bleak subscribe (the pad DROPS WinRT centrals
  ~2 s in — even after successful pairing — so the PC could never
  receive either). The ctag-tbd BBA builds exclude BtMidiReceiver
  entirely (radio-free) — the working reference was the uimcu C3 panel
  project at /home/ubuntu/ctag-tbd/uimcu (found by diffing all ctag
  branches/builds).
- **Bench (hardware, user playing):** pad → C3 central → RP2350 notes
  live (`notes=` 221→273 while watched), IN/OUT meters hot, rings
  envelopes firing, zero stalls; C3 `ble status`: msgs=547 drops=0.
  Reference copy of the proven upstream file kept at
  `tools/ctag_uimcu_BtMidiReceiver.cpp`.
- **Bench tooling lessons recorded:** WSL builds on /mnt/c are slow AND
  ninja/CMake can reuse stale state across FS boundary edits (a
  CMakeLists edit did not reconfigure until forced); build in the WSL
  native FS (~/rack-uimcu61 pattern) or locally via
  tools/build_uimcu.bat. ALWAYS verify "Project build complete" +
  zero `error:` lines before flashing (a broken grep once flashed a
  stale binary), and confirm the board actually rebooted (fresh
  uptime timestamps in the boot log).
### Rack T5 — SYS one-pot module/pair selector, chain-B page, text aliases (2026-11-11)

User-requested UX round:
- **SYS single selector pot** (cell 0, dual-purpose): not chained = the
  active module over 4 slots; engaged = the chain pair over 3 slots
  (switching re-engages off->on with the new pair). The old 4-pot module
  radio + the separate CHAIN PR pot are GONE (SYS cells 1-4 blank on the
  wire). Value text aliases: ELEMENTS/CLOUDS/PLAITS/RINGS or
  E>C/P>C/R>C. The value encoding (module*32 / (pair-1)*64) works for
  BOTH modes under one decode formula (1 + val*3/128 gives the pair for
  the ring + CHAIN ON paths).
- **Wire & local switch fix:** `WireToPotValue` in `RackUI.cpp` passes
  through the wire value for cell 0 on SYS (no longer overwrites with 127/0
  from the old 4-pot radio code) and zeroes cells 1-4; `TelemLocalSysSwitch`
  in `RackLink.cpp` sets `cell[0].value = newMod * 32` instead of 4-pot
  radio values; `GetPotTarget` on SYS disables blank cells 1-4 and enables
  C3-local BLE pots 9/10 with `/ 15` preset slot scaling.
- **Chain-B page (page 8, kUiPageChainB)**: while engaged, the C3
  navigation gains a CLOUDS B page after P2 = the CLOUDS P1 registry
  table with live B values (the observe hook double-writes B-directed
  CCs — 23/24/65/66 + alias bank 40-48 — into the CLOUDS cache while
  chained; deltas target the B page when it is the C3's active page).
  Pot writes on the page address clouds-B via the existing chain
  dispatcher; leaving the chain snaps back to P1. Not reachable when no
  chain is engaged. TELEM full + delta + labels all handle page 8.
- **Plaits text aliases on P1**: ENGINE (CC 23) shows all 98 catalog
  aliases (VA/DX7B/MODAL/ACID/... — collision-checked, short forms);
  AUX (CC 24, 3 states) shows AUX/STER/SUB.
- **Hardware flash check**: Serial uptime verified both boards (RP2350
  COM15 @ 32,895s, ESP32-C3 COM22 @ 32,949s) had not been reflashed since
  ~03:04 AM; commit b82b856 was never flashed to hardware prior to this fix.

### Rack T5 — Chained variant switching, SYS cleanup, chain-B order, Elements ominous fold (2026-11-11)

- **Direct chain-pair switching**:
  - Root cause of chain turning off: `RackUI.cpp` was sending `SendChainRoute(pair, 1, 0)` immediately followed by `SendChainRoute(pair, 1, 1)`. RP2350 `cmd_chain` rejected re-engaging when `a != g_chain.a` with "a different pair needs 'chain off' first". The deferred `chain off` on Core 1 then disengaged the chain.
  - Fix: Removed spurious `engage = 0` transmission in `RackUI.cpp#ApplyPotAction`. In `rack-rp2350/src/main.cpp#cmd_chain`, allowed seamless pair switching when `a != g_chain.a`; in `do_chain_engage()`, conditionally ramp-down using `chain_ramp_down_chain()` if chain was already active.
- **Removed legacy SYS yellow underline & encoder push**:
  - The 1-px yellow underline under cell slots on SYS and `sysRingAction()` short push cycling were legacy scaffolding from T5 before SYS parameters were assigned directly to pots.
  - Fix: Completely removed `s_sysRing` variable, the yellow underline rendering in `renderPage()`, `sysRingAction()`, and simplified SYS strip title to "SYSTEM". Encoder short push is a clean no-op.
- **Page ordering (Chain-A -> Chain-B -> MODs -> SYS)**:
  - SYSTEM (page 7) is always guaranteed as the last page.
  - When chained: navigation order is `kPagesChained[8] = { 0, 8, 2, 3, 4, 5, 6, 7 }`, placing `CLOUDS B` (page 8) right after Chain-A (page 0) and before modulation pages (2-6).
  - When unchained (Elements): `kPagesUnchainedP2[8] = { 0, 1, 2, 3, 4, 5, 6, 7 }`.
  - When unchained (Clouds/Plaits/Rings): `kPagesUnchained[7] = { 0, 2, 3, 4, 5, 6, 7 }`.
- **Elements MODEL & OMINOUS consolidation**:
  - Folded `OMINOUS` into CC 28 (`MODEL`) as a 4th discrete mode (`MODAL`, `STRING`, `STRINGS`, `OMINOUS`).
  - In `module_elements.h`, CC 28 values 0..2 select resonator models and set `pending_easter_egg = 0`; value 3 sets `pending_easter_egg = 1`.
  - Removed CC 29 `OMINOUS` from `kUiPageElementsP2` (cell count reduced from 4 to 3: `MODEL`, `PLAY`, `VCA`).
  - Updated `RackUI.cpp` discrete count to 4 and added `OMINOUS` to `kElemModels` text table.
- **Future plan noted (Live Performance layout)**:
  - Target UX: In chained mode, merge the most-used performance controls from both modules onto Page 1 (e.g. key timbral/resonator/grain controls), and put remaining secondary controls onto Page 2, rather than strict single-module pages. To be designed and grouped in a future update.

### Rack T5 — Fix DSP crash during live chained variant switching (2026-11-11)

- **Root Cause Analysis**:
  - `cmd_chain` on Core 0 previously wrote `g_chain.a = static_cast<uint8_t>(a);` directly into global state.
  - While chained, Core 1 executes `chain_render_block()` every 750 µs, which checks `switch (g_chain.a)`.
  - Overwriting `g_chain.a` from Core 0 caused Core 1 to immediately execute the new module's render (`chain_render_a`) before:
    1. The new module's hot image was copied into the SRAM window via `rack_window_copy()`.
    2. The new module's state was initialized in the object arena via `init_chain_a()` / `init_at()`.
  - Furthermore, `do_chain_engage()` called `chain_ramp_down_chain()`, which called `chain_render_block()` 5 times to ramp down gain, executing against the uninitialized new module.
  - This resulted in an immediate HardFault on Core 1, halting DSP while I2S DMA continued looping static/noise.
- **Fix**:
  - Decoupled target slot state: Introduced `g_pending_chain_a` and `g_pending_chain_b`.
  - Core 0 only writes `g_pending_chain_a` and `g_pending_chain_b` and queues `g_pending_chain = 1`. `g_chain.a` is never touched by Core 0.
  - Core 1 consumes `g_pending_chain` at the block boundary and passes `(req_a, req_b)` into `do_chain_engage(new_a, new_b)`.
  - During step (a) MUTE: `chain_ramp_down_chain()` safely renders `old_chain_a` (which is fully resident and initialized) to fade cleanly to 0, then flushes chain DMA buffers.
  - During step (b) COPY: copies both `new_a` and `new_b` into the window.
  - During step (c) I2S: if already chained, skips redundant I2S reconfig.
  - During step (d) INIT: initializes `new_a` and `new_b` in the object and buffer arenas, replays CCs.
  - During step (e) GO CHAIN: updates `g_chain.a = new_a`, `g_chain.b = new_b`, and `g_active_module = new_a`.
  - During step (f) UNMUTE: ramps up audio through `chain_render_block()`, now safely executing `new_a`.
  - In `rack_request_switch`: if called while chained, routes requests to `cmd_chain` instead of rejecting, enabling smooth module switching from UI overlay.

### Rack T5 — Fix unresponsive pots on Clouds B page and remove exciter parameters (2026-11-11)

- **Root Cause Analysis (Unresponsive Pots on Page 8)**:
  - `RackUI::GetPotTarget` in `RackUI.cpp` only handled `s_page <= 1`, `s_page >= 2 && s_page <= 6`, and `s_page == 7`. For `s_page == 8` (`CLOUDS B`), it fell through to `return false;`.
  - When `GetPotTarget` returned `false`, `RackPots.cpp` discarded all pot inputs on page 8, making all pots completely unresponsive.
- **Exciter Parameter Removal in Chained Mode**:
  - Mutable Instruments Clouds in hardware is an audio effect without internal oscillators. The standalone port added an internal test exciter (`EX TONE`, `EX LEVEL`, `EX SRC`) to make Clouds playable standalone over MIDI without audio inputs.
  - In chained mode (`E>C`, `P>C`, `R>C`), Clouds receives audio directly from Module A, bypassing and silencing the test exciter.
  - Updated `ui_link.h` on RP2350: for `kUiPageChainB`, cells 0..2 are blanked (empty label `""`, zeroed telemetry), so the UI clears slots 0..2 to black.
  - In `RackUI.cpp`: pots 0..2 on page 8 return `false` in `GetPotTarget`.
- **Clouds B Parameter Mapping**:
  - `GetPotTarget`: routes pots 3..15 on `s_page == 8` to `PotTarget::kCC`.
  - `ApplyPotAction`, `WireToPotValue`, `PotToWireValue`, `potToDisplayValue`, `renderCell`, and `GetPotBucketBounds`:
    - Use `modForPage = (s_page == 8) ? 1 : ...` so Clouds parameter definitions apply on page 8 regardless of slot A's active module.
    - `MODE` (CC 23): constrained to 2 options (0 = GRANULAR, 1 = LOOPING -> wire 2), matching the firmware chain clamp.
    - `QUALITY` (CC 24): 4 options (`32-16`, `32-8`, `16-16`, `16-8`).
    - `FREEZE` (CC 65) and `TRIG` (CC 66): 2-state toggles (`OFF`/`ON`).
    - `PITCH` (CC 42): -24..+24 st display.
- **Flashed & Verified**:
  - Both ESP32-C3 (`COM22`) and RP2350 (`COM15`) built and flashed successfully. Link online and responsive.
  - User hardware verified: "Clouds page B working fine now."

### Rack T6 — Preset system audit, chain replay fixes & SYS page manual save/load gestures (2026-11-11)

- **Preset System & Autosave Audit**:
  - Verified autosave runs in background (3 s debounce) pushing 871 B blob across UART to ESP32-C3 flash partition (`presets`, rotating 4 sectors) and restoring on boot.
  - **Bug Fix 1 in `preset.h` (`preset_apply_blob`)**:
    - Loading a preset with a chain pair previously wrote directly to `g_chain.a = a;` without setting `g_pending_chain_a/b`. Updated to use non-racing handoff (`g_pending_chain_a = a; g_pending_chain_b = RACK_MODULE_CLOUDS; g_pending_chain = 1;`).
  - **Bug Fix 2 in `preset.h` (`preset_replay_module`)**:
    - Replaying Clouds (slot B) previously sent raw CC 40..48 to `control_change`, which only handled 14..22. Added translation `cc - 40 + 14` so Clouds grain parameters restore on preset/swap replay.
- **Manual Preset Save & Load Gestures on SYS Page**:
  - When the yellow ring was previously removed from page 7, encoder short-push became a no-op, disabling the physical trigger for `PresetStore::SaveTo()` and `LoadFrom()`.
  - Added dedicated encoder gestures on SYS page (`s_page == 7`) in `RackUI.cpp`:
    - **Short push** (`InputEvent::PUSH_SHORT`): Triggers `PresetStore::LoadFrom(s_presetSlot)` and shows `"LOADED P<slot>"` banner on the status strip for 2 seconds.
    - **Long press (3s)**: Monitored via `UserInput::IsPressed()`. Displays countdown `"SAVE IN 3s..."`, `"SAVE IN 2s..."`, `"SAVE IN 1s..."` on the status strip. At 3.0s, triggers `PresetStore::SaveTo(s_presetSlot)` and shows `"SAVED P<slot>"` banner for 2.5 seconds. Releasing early cancels without saving.
    - Encoder rotation while pressed on page 7 is ignored to prevent accidental page switching.
    - Regular encoder navigation and module overlay on pages 0..6 & 8 remain unchanged.
- **Flashed**:
  - Both RP2350 (`COM15`) and ESP32-C3 (`COM22`) rebuilt and flashed. Link online, autosave partition verified intact.
  - User hardware verified: "ok, everything working fine."

### Rack T7 — Elements DSP audit against upstream: reverb restore, modal boot & 280 MHz overclock (2026-11-11)

- **Comprehensive Upstream Code Audit (against `pichenettes/eurorack/elements`)**:
  - Performed a byte-by-byte and AST diff of all DSP routines, sample tables, and filter LUTs against official Mutable Instruments Elements sources (`https://github.com/pichenettes/eurorack/tree/master/elements/dsp`).
  - **Reverb Bug Discovered**:
    - In commit `c267d79`, poetaster commented out the reverb feedback decay calculation in `part.cc`/`part.inc`:
      `float reverb_time = 0.01f; //0.35f + 1.2f * reverb_amount;`
    - Because `reverb_time` was locked at `0.01f`, the Dattorro feedback loop decayed in ~10 ms (effectively zero tail / completely muted).
    - When `SPACE` transitions from 0 to 0.1, `raw_gain` crossfades the AUX channel from the raw exciter to the stereo resonator. From 0.1 to 0.5, only stereo spread increases (reverb is 0). Above 0.5, reverb wet mix enters, but with `reverb_time = 0.01f`, there was no perceptible decay tail.
    - **Fix**: Restored `float reverb_time = 0.35f + 1.2f * reverb_amount;`. Turning `SPACE` past 12 o'clock now blooms into a lush, expansive reverb tail with feedback scaling up to 0.83.
  - **Strike Exciter & Low Frequencies Analysis**:
    - The strike sample tables (`hit_01.wav` .. `hit_09.wav` in `smp_sample_data`) and the SVF impulse filters are 100% identical to upstream.
    - Strike exciters in Elements do not synthesize low frequencies directly: they deliver an acoustic impulse or sample transient that strikes the *resonator*, whose vibrating modes provide the low-end resonance.
    - Discovered that `module_elements.h` was booting with:
      1. `resonator_model = RESONATOR_MODEL_STRING;` instead of `RESONATOR_MODEL_MODAL`. (The string model inserts a DC blocker and models thin plucked strings; modal synthesis models resonant plates, bowls, membranes, and deep bass bells).
      2. `resonator_damping = 0.7f;` instead of upstream's default `0.25f`. Because mode Q scales inversely with damping (`lut_4_decades`), 0.7 choked mode Q down to ~2,000, muting all low-frequency mode ringing. At 0.25, mode Q exceeds 100,000, sustaining rich acoustic low end.
      3. `previous_position_ = 0.0f;` had been stripped from `Resonator::Init()`, leaving pickup position interpolation uninitialized.
    - **Fix**: Set default boot model to `RESONATOR_MODEL_MODAL`, default damping to `0.25f`, default space to `0.5f`, and restored `previous_position_ = 0.0f;`.
- **Overclocking & Clock Speed Audit (240 MHz vs 280 MHz)**:
  - While USB host bit-banging is disabled (`RACK_ENABLE_USB_HOST = 0`), running the system clock at 280 MHz clocks the external QSPI flash memory at 140 MHz (RP2350 default flash divider /2).
  - External SPI/QSPI flash chips (rated for 133 MHz max) experience instruction fetch timing violations and bit corruption at 140 MHz, causing boot freezes and HardFault crashes.
  - Reverting to 240 MHz (`board_build.f_cpu = 240000000L`) clocks the flash at 120 MHz (safe, zero flash bit flips).
  - Telemetry at 240 MHz confirms `max Process: 444–481 us (budget 496 us)` with `0 i2s stalls`, completely stable audio and link.
- **Built & Flashed**:
  - Rebuilt and flashed `rack-elements` at 240 MHz to `COM15` (`c1e344a`). Link and audio streaming active.
  - **User hardware verified**: "ok, now it sounds stable and looks like no volume drops". Modal resonator low end, strike transients, and full reverb decay confirmed clean.

### Rack T8 — Rings Pitch / Frequency Parameter Restoration (`647b866`, 2026-11-11)

- **Audit Findings**:
  - Upstream Mutable Instruments Rings physical Eurorack module has a central **FREQUENCY** pot that controls root tuning (`performance_state.tonic`), which transposes the resonator modes / string tuning across all models.
  - The standalone port (`rings-rp2350`) previously hardcoded `perf.tonic = 0.0f;` and omitted the frequency knob, assuming pitch would only come from MIDI note numbers.
  - In `rack-rp2350/src/module_rings.h`, CC 1 was unhandled and missing from the UI cell table `kUiPageRingsP1`.
- **Implementation**:
  - Added `volatile float shared_pitch;` to `State` in [`rack-rp2350/src/module_rings.h`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/src/module_rings.h).
  - Mapped CC 1 in `control_change()` (-24..+24 semitones around center value 64) and continuous modulation via `mod_set_cc_float()`.
  - Wired `perf.tonic = s->shared_pitch;` in both `render()` and `chain_render_a()`.
  - Added `{1, "PITCH", kUiColorWhite}` as cell 0 (pot 0) in `kUiPageRingsP1` ([`rack-rp2350/src/ui_link.h`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/src/ui_link.h)).
  - Updated `kUiPages` count to 13 cells, updated [`docs/ui-protocol.md`](file:///C:/Users/schra/Developer/mi-rp2350/docs/ui-protocol.md) and [`tools/rack_panel.html`](file:///C:/Users/schra/Developer/mi-rp2350/tools/rack_panel.html).
- **Built & Flashed**:
  - Rebuilt and flashed unified firmware to `COM15` (`647b866`).
  - Switched to Rings live; verified `max Process: 278 us (budget 750 us)`, 0 I2S stalls. Pot 0 now controls Pitch/Frequency (-24..+24 st).

### Rack T9 — Rings Front Panel Pot Layout Rearrangement (`292c03e`, 2026-11-11)

- **Audit & Pot Mapping**:
  - Front panel physical potentiometers are grouped in 3 rows (6 / 4 / 6):
    - Row 0 (physical p1..p6 / pot indices 0..5)
    - Row 1 (physical p7..p10 / pot indices 6..9)
    - Row 2 (physical p11..p16 / pot indices 10..15)
  - Rearranged `kUiPageRingsP1` in [`rack-rp2350/src/ui_link.h`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/src/ui_link.h) to 16 slots:
    - **Row 0**: p1 BRIGHTNESS (CC 15), p2 DAMPING (CC 16), p3 POSITION (CC 17), p4..p6 empty (CC 0, label `""`).
    - **Row 1**: p7 PITCH (CC 1), p8 STRUCTURE (CC 14), p9 STRUM (CC 64), p10 VCA (CC 7).
    - **Row 2**: p11 MODEL/FX (CC 18), p12 CHORD TB (CC 22), p13 CHORD (CC 23), p14 MODE (CC 20), p15 WAVE (CC 21), p16 POLY (CC 19).
  - Empty cells (`cc == 0`) safely ignored in `ui_link_cell_flags()`, delta sending, `ui_link_observe_cc()`, and `ASSIGN_CYCLE`.
  - Added safety guard in [`rack-uimcu/ui/RackUI.cpp`](file:///C:/Users/schra/Developer/mi-rp2350/rack-uimcu/ui/RackUI.cpp) `GetPotTarget()` to ignore empty/unlabeled pots.
  - Updated [`docs/ui-protocol.md`](file:///C:/Users/schra/Developer/mi-rp2350/docs/ui-protocol.md) and [`tools/rack_panel.html`](file:///C:/Users/schra/Developer/mi-rp2350/tools/rack_panel.html).
- **Built & Flashed**:
  - Rebuilt `rack-elements` environment and flashed to RP2350 on `COM15` (`292c03e`).
  - Switched to `mod=rings` live; report confirms: `max Process: 258 us (budget 750 us) | i2s stalls: 0 (+0)`.
  - In-memory `uitest` verified 6/6 test cases PASS.
  - **User hardware verified**: "all fine". Physical front-panel pots p1..p3, p7..p10, p11..p16 confirmed responding correctly to all Rings parameters with proper layout and blank slots on p4..p6.

### Rack T10 — Plaits->Clouds Chained AUX / CC Hijacking & Buffer Overflow Fix (`a729a72`, 2026-11-11)

- **Root Cause Analysis**:
  - **User report**: "Plaits chained to clouds. When I switch Plaits AUX to Stereo white noise is appeared imediately on wet Clouds output. Dry is fine... in 32-8. Other quality setting - silent wet output".
  - **CC Hijacking**: In `rack_chain_wants_cc()` ([`rack-rp2350/src/main.cpp`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/src/main.cpp)), CC 23 and CC 24 were intercepted whenever a chain was active. When the user turned Plaits AUX (pot 9, CC 24 = 1 for Stereo), RP2350 intercepted CC 24 and called `rack_modules::mod_clouds::control_change(24, 1)` (Quality = 1, mono). Plaits AUX never received the CC.
  - **Chained Clouds Buffer Overflow**: In chained mode, Clouds' small buffer is partitioned to 32,768 bytes (down from standalone's 65,536 bytes). When Quality is 0 (16-bit stereo), the Reverb/Diffuser workspace (41,368 bytes) is allocated from the large buffer (65,536-byte workspace, fits cleanly). But when Quality is set to 1 or 3 (mono), `GranularProcessor::Prepare()` allocates the workspace from `buffer_[1]` (small buffer, only 32,768 bytes), overflowing the buffer arena by 8,600 bytes and blasting digital white noise into the wet reverb/diffuser output. In quality 2/3 (8-bit), the downsampler failed under the chained block size, resulting in silence.
  - **Uninitialized Silence Flag**: In `granular_processor.cc`, `GranularProcessor::Init()` did not explicitly initialize `silence_ = false;`.
- **Implementation**:
  1. **Unhijacked CC 23 & 24 for Chain A**: Removed `cc == 23 || cc == 24` from `rack_chain_wants_cc()`. Now while chained, CC 23 and CC 24 reach the active Chain A module (Plaits ENGINE & AUX, Elements GEOMETRY & BRIGHTNESS, Rings CHORD TB & CHORD).
  2. **Clouds-B Mode & Quality Aliases (CC 49 & 50)**:
     - Mapped CC 49 -> Clouds-B Mode (0 granular, 2 looping delay; stretch/spectral rejected).
     - Mapped CC 50 -> Clouds-B Quality (strictly clamped to Quality 0 = 16-bit stereo).
     - Updated `kUiPageCloudsP1` cells 12 & 13 to CC 49 and CC 50 in [`rack-rp2350/src/ui_link.h`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/src/ui_link.h).
     - Synced cache between 49/50 and 23/24 in `ui_link_observe_cc()` and `preset_replay_module()`.
  3. **Strict Quality Locking on Chained Clouds**:
     - Clamped `pending_quality` to 0 whenever `s->chain_b == true` in both `control_change()` and `chain_render_b()` in [`rack-rp2350/src/module_clouds.h`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/src/module_clouds.h).
  4. **GranularProcessor Initialization**:
     - Added `silence_ = false;` to `GranularProcessor::Init()` in [`rack-rp2350/lib/clouds/clouds/dsp/granular_processor.cc`](file:///C:/Users/schra/Developer/mi-rp2350/rack-rp2350/lib/clouds/clouds/dsp/granular_processor.cc).
  5. **UI MCU Support & Tooling**:
     - Updated [`rack-uimcu/ui/RackUI.cpp`](file:///C:/Users/schra/Developer/mi-rp2350/rack-uimcu/ui/RackUI.cpp) to support CC 49/50 for Clouds-B Mode and locked Quality.
     - Updated [`tools/rack_panel.html`](file:///C:/Users/schra/Developer/mi-rp2350/tools/rack_panel.html) and [`docs/ui-protocol.md`](file:///C:/Users/schra/Developer/mi-rp2350/docs/ui-protocol.md).
- **Built & Flashed**:
  - Rebuilt unified RP2350 firmware (`rack-elements`) and flashed to `COM15`.
  - Rebuilt ESP32-C3 UI MCU firmware with `build_uimcu.bat` and flashed to `COM22`.
  - Live serial report confirms `plaits>clouds` running stable at `max Process: 169 us (budget 750 us) | i2s stalls: 0 (+0)`. Ready for hardware verification.

### Rack T11 — Chained Clouds 85k/43k Buffer Sizing, Unlocked Quality Switch & Density Dead Zone Resolution (`46786b5`, 2026-11-11)

- **User Issue**:
  - "quality switch not working in clouds. wet output is silent"
  - "with all chained modules wet output in clouds is silent"
- **Root Cause Analysis**:
  1. **Why Quality Switch Was Not Working**: In commit `a729a72`, CC 50 / CC 24 was clamped/locked to 0 across firmware (`main.cpp`, `module_clouds.h`, `ui_link.h`) and UI MCU (`RackUI.cpp`) because mono modes previously overflowed the workspace buffer.
  2. **Why Wet Output Was Silent (Clouds Density Center Dead Zone)**:
     - Pot telemetry read directly from ESP32-C3 showed:
       - Pot 8 (DRY/WET): `127` (100% wet, dry path muted)
       - Pot 11 (REVERB): `0` (reverb off)
       - Pot 6 (DENSITY): `65` (exact center detent)
     - In upstream Mutable Instruments Clouds (`granular_processor.cc`), Density centered at 0.50 (MIDI value 64) calculates `overlap == 0.0f` and intentionally spawns exactly ZERO grains.
     - With 100% wet mix, 0 grains spawned, and 0 reverb, wet output is pure digital 0.0f (silence). Turning Density clockwise (random grains) or counter-clockwise (periodic grains), or turning Reverb up, immediately restores full wet output.
  3. **Why Mono Modes Previously Produced White Noise**:
     - In mono (`num_channels_ == 1`), `GranularProcessor::Prepare()` assigns `workspace = small_buffer`.
     - The workspace requires 42,520 bytes (Diffuser 8,192 B + Reverb 32,768 B + Correlator 1,560 B).
     - The old split was 98,304 B large + 32,768 B small; since 32,768 B < 42,520 B, `BufferAllocator::Allocate` returned `NULL`, leaving reverb delay line pointers at `0x00000000` (flash base), generating loud white noise.
- **Implementation**:
  1. **85 KiB / 43 KiB Buffer Split**:
     - Changed chained partition to `kChainCloudsLargeBytes = 87040` (85 KiB) and `kChainCloudsSmallBytes = 44032` (43 KiB) = 131,072 B (128 KiB).
     - Stereo (quality 0 / 2): sample memory = 44,032 B (0.688 s/ch), workspace = 87,040 - 44,032 = 43,008 B (fits 42,520 B with 488 B headroom).
     - Mono (quality 1 / 3): sample memory = 87,040 B (1.36 s), workspace = 44,032 B (fits 42,520 B with 1,512 B headroom).
     - Both stereo and mono modes fit with positive headroom and zero NULL pointer crashes.
  2. **Stereo-to-Mono Mixdown**:
     - Re-enabled input mixdown in `granular_processor.cc` for `num_channels_ == 1` so stereo outputs from Chain A (e.g. Plaits out + aux) are summed into mono rather than dropping channel R.
  3. **Unlocked Quality Selection**:
     - Removed clamps on CC 50 / CC 24 in `main.cpp`, `module_clouds.h`, `ui_link.h`, `RackUI.cpp`, and `rack_panel.html`.
     - Pot 13 on page 8 (Clouds-B) now smoothly selects between all 4 quality modes: `32-16`, `32-8`, `16-16`, and `16-8`.
- **Built & Hardware Verified**:
  - Unified RP2350 firmware compiled and flashed to `COM15` (`46786b5`).
  - ESP32-C3 UI MCU firmware compiled and flashed to `COM22`.
  - Real hardware audio tests over USB-MIDI across all 3 Chain A sound sources:
    - Plaits -> Clouds: `in=282 out=182` with Density=100; verified dead zone at Density=64 produces `in=282 out=0`.
    - Elements -> Clouds: engaged 41 ms; `out=51` wet.
    - Rings -> Clouds: engaged 43 ms; note strumming verified `in=315 out=119`.
    - Verified all 4 quality modes (0: `32-16`, 1: `32-8`, 2: `16-16`, 3: `16-8`) run clean with 0 stalls and low process times (~275-526 µs, well within 750 µs budget). Ready for user testing.

### Rack T12: Elements P2 Page in Chained Mode Navigation (2026-11-12)
- **User Issue**:
  - "in chained mode elements have only one page and single mode's P2 parameters MODEL PLAY and VCA not available."
- **Root Cause Analysis**:
  - In `rack-uimcu/ui/RackUI.cpp`, chained page navigation was hardcoded to `kPagesChained[8] = { 0, 8, 2, 3, 4, 5, 6, 7 }`.
  - It assumed Chain A only ever had Page 0 (P1), omitting Page 1 (P2: MODEL on CC 28, PLAY on CC 64, VCA on CC 7) when Elements was the active Chain-A sound source.
  - RP2350 telemetry (`ui_link.h`) and DSP (`module_elements.h`) already fully supported Page 1 for Elements while chained; only the C3 UI MCU encoder page list skipped it.
- **Implementation**:
  - In `rack-uimcu/ui/RackUI.cpp`, added `kPagesChainedP2[9] = { 0, 1, 8, 2, 3, 4, 5, 6, 7 }`.
  - When chained with Elements (`ModuleHasP2(s_cur.module) == true`), encoder rotates through 9 pages: P1 -> P2 -> CLOUDS B -> MOD-LFO 1/2 -> MOD-LFO 3/4 -> MOD-TRIG -> MOD-GATE -> MOD-ENV -> SYS.
  - When chained with Plaits or Rings (`ModuleHasP2(s_cur.module) == false`), encoder retains the 8-page sequence (skipping P2).
- **Built & Hardware Verified**:
  - `rack-uimcu` compiled and flashed to `COM22` (`31084e4`).
  - Switched chain to Elements -> Clouds; verified C3 requests and receives `telem page=1 mod=0 mode=0 count=3` (MODEL, PLAY, VCA).
  - User hardware verified on physical board: Elements P2 page is fully accessible and functional in chained mode ("yes all fine now").

### Rack T13: Switchable Monophonic Chord Arpeggiators arp1 and arp2 (9844bb0, 2026-11-12)
- **User Request**:
  - Implement 3 switchable monophonic chord-arpeggiator algorithms (HYP-1 Acid/Chord Arp, Barlow + Sioros-Guedes Syncopated Arp, and Grids 2D Topographic Trance-Gate Arp) as modulation sources rp1 and rp2 (MOD_ARP, sources 10 and 11, bringing MOD_SOURCE_COUNT to 12 while preserving gate1 and gate2 completely untouched) based on the Google Argon analysis in docs/hyp1-mono-arp-algorithms.md, docs/mod_engine.h, and docs/ui_link.h.
- **Implementation**:
  1. **RP2350 Modulation Engine (
ack-rp2350/src/mod_engine.h)**:
     - Defined MOD_ARP = 4, MOD_TYPE_COUNT = 5, MOD_SOURCE_COUNT = 12, MOD_SRC_ARP_FIRST = 10, MOD_ARP_COUNT = 2, MOD_DEST_VOICE = 127.
     - Standardized 8 parameters per ARP unit: lgo, 
oot, scale, dens, groove, clk, div, dest.
     - Implemented 3 algorithms:
       - Algo 0 (HYP-1): Probabilistic step gates, slide legato (55% staccato vs 100% legato), chord voicing pools (HA_MODE_POOL), mutation, 8-bar fill turnaround.
       - Algo 1 (Barlow): Metrical indispensability table (kBarlowSyncOrder), syncopation anticipation, ties.
       - Algo 2 (Grids): 2D map node interpolation (kGridsNode*), topographic fill threshold, chaos jitter, continuous accent CV slewing.
     - Sub-ranges within each third select octave range (1–3 octaves).
     - Patching dest to 1 (PITCH), 42 (Clouds PITCH), 127 (MOD_DEST_VOICE), or trigger CCs (64/66) drives shared_note, shared_gate, and fires module triggers.
     - Unpatching dest releases shared_gate = 0 and calls ui_link_restore_cached_cc(g_active_module, prev_cc).
     - Console parser and status commands updated for natural names (mod arp1 algo hyp1, mod arp1 root midi, mod arp1 scale aeolian, mod arp1 dens 65, mod arp1 groove 45, mod arp1 dest voice).
  2. **RP2350 UI Link & Presets (
ack-rp2350/src/preset.h, ui_link.h)**:
     - Preset format bumped to version 2 (kPresetModLen = 341 bytes, 12 sources).
     - Page layout updated: kUiPageModArp = 7, kUiPageSys = 8, kUiPageChainB = 9, kUiPageCount = 10.
     - Added kUiModArpCells[16] table and telemetry encoders/quantizers.
  3. **ESP32-C3 UI Controller (
ack-uimcu)**:
     - RackLink.hpp: Page constants shifted (kPageModArp = 7, kPageSys = 8, kPageChainB = 9, kPageCount = 10).
     - RackPots.cpp: Mod source assignment range expanded from 10 to 12; push-turn assignment scaled across 12 sources; ride/dest pots mapped for ARP.
     - RackUI.cpp: Added MOD-ARP page pot mapping (kModPotMap[5]), discrete pot bucketing and value formatting (ALGO HYP.1..GRD.3, ROOT MIDI/C1..C6, SCAL ION..PMI+, DENS/GRV %, DIV 1/1..1/32T), and updated encoder page navigation lists.
     - RackConsole.cpp: Updated page <0-9> console command bounds.
- **Built, Flashed & Hardware Verified**:
  - RP2350 firmware compiled and flashed to COM15 (9844bb0).
  - ESP32-C3 UI firmware compiled and flashed to COM22 (9844bb0).
  - RP2350 serial console confirmed:
    - mod arp1 algo hyp1/barlow/grids switches live.
    - mod arp1 dest voice correctly routes pitch+gate+trig to active module voice (plaits), with max Process: 34 us (budget 500 us) and 0 I2S stalls.
    - mod arp1 dest off releases gate and restores parameter cache cleanly.
  - ESP32-C3 console confirmed:
    - Page 7 (MOD-ARP) displays all 16 cell values and labels for P1 (rp1) and P2 (rp2).
    - Bidirectional link and pots are online and scanning.


### Rack T14: Generative Melodic Sequencer Upgrade (2026-11-12)
- **User Request**:
  - Incorporate research findings from `docs/Architectural Frameworks and Implementation Patterns for Embedded Generative Melodic Sequencers.md` to improve the `MOD_ARP` module.
- **Architectural & DSP Implementation**:
  1. **Canonical Scales & Bitmask Math (`rack-rp2350/src/gen_tables.h`)**:
     - Bit-verified 20 canonical scale bitmasks (`SCALE_BITMASKS`) replacing legacy buggy masks (Phrygian was 0x06B5 Mixolydian, Major Pentatonic was 0x0125).
     - Euclidean negative-safe floor division and modulo (`floor_div_i16`, `floor_mod_i16`) fixing octave fold calculations.
     - Krumhansl-Schmuckler tonal hierarchy tables (`KS_GRAVITY_MAJOR`, `KS_GRAVITY_MINOR`), metrical beat weight hierarchies, and 7x7 diatonic voice-leading Markov matrix.
     - Neo-Riemannian triad transformation state machine (P, L, R, N, S, H).
     - 16 canonical melodic pitch contours (`MELODIC_CONTOURS_DEG[16][16]`) and metrical mutation sensitivities.
  2. **5 Algorithmic Generators (`rack-rp2350/src/mod_engine.h`)**:
     - `MOD_ARP_HYP1` (0): HYP-1 Acid generator with dynamic 303 articulations, slide legato, and accent capacitor modeling.
     - `MOD_ARP_MOTIF` (1): Phrase-Grammar Motif Transformer (ABAC, AABC, ABBC, AABA) with diatonic inversion, retrograde, rhythmic displacement, and step 15 pickups.
     - `MOD_ARP_MARKOV` (2): Tonal-Gravity Markov Walker combining K-S tonal gravity, chord-root bias, voice-leading transition matrices, and 16 contour templates.
     - `MOD_ARP_METRO` (3): Metropolix 8-Stage Accumulator with per-stage pulse counts (1..8), ratchets (1..4 bursts), and pendulum scale accumulator.
     - `MOD_ARP_DEJAVU` (4): Marbles Deja-Vu Shift Register with weak-beat mutation bias, 12 o'clock lock notch, permute jumps, and STEPS progressive salience quantizer.
     - Cadential turnaround engine (`mod_arp_apply_turnaround()`): 4-bar and 8-bar fills.
     - Q16.16 exponential pitch glide (~55 ms) and slide retrigger logic.
  3. **UI Link & Physical Controller Sync (`rack-rp2350/src/ui_link.h`, `rack-uimcu/ui/RackUI.cpp`)**:
     - Normalized 15-algo pot range (5 algos x 3 octave ranges: `HYP.1..3`, `MOT.1..3`, `MRK.1..3`, `MET.1..3`, `DJV.1..3`).
     - Display for 20 scales and root modes (`MIDI`, `PARAL`, `W7`, `NEO`, `C1..C6`).
- **Built, Flashed & Hardware Verified**:
  - RP2350 flashed via 1200-baud touch to `COM15` (`firmware.uf2`).
  - ESP32-C3 UI controller flashed via `build_uimcu.bat flash -p COM22`.
  - Serial console verification:
    - Booted into Elements (`mod=elements`), modulation engine running smoothly with 0 stalls (`i2s stalls: 0 (+0)`), process load ~89-111 us.
    - Live switching between all 5 algorithms (`mod arp1 algo hyp1/motif/markov/metro/dejavu`) and 20 scales (`mod arp1 scale acidpool/blues/...`) verified over serial telemetry.
    - UART link between RP2350 and C3 confirmed operational at 921600 baud with 0 CRC errors.

- **UI Label Expansion (774d312)**:
  - Expanded MOD-ARP cell labels in `rack-rp2350/src/ui_link.h` (`P1/P2 SCALE`, `P1/P2 GROOVE`, `P1/P2 DENS`).
  - Expanded display value strings in `rack-uimcu/ui/RackUI.cpp` up to 8–9 characters (`HYP-1 1..3`, `MOTIF 1..3`, `MARKOV 1..3`, `METRO 1..3`, `DEJAVU 1..3`; scales `CHROM`, `IONIAN`, `DORIAN`, `PHRYGIAN`, `MAJ PENT`, `MIN PENT`, `HARM MIN`, `MEL MIN`, `PHRYGDOM`, `WHOLETON`, `OCTATON`, `HIRAJOSH`, `INSEN`, `DBL HARM`, `ACIDPOOL`; roots `MIDI`, `PARALLEL`, `W7 DRIFT`, `NEO-RIEM`).
  - Both RP2350 (`COM15`) and ESP32-C3 (`COM22`) rebuilt and flashed.

### Rack T15: Shared Root/Scale, Algorithm-Specific Parameters & Slower Divisions (`a65c0e0`, 2026-11-12)
- **User Requests**:
  1. Make `ROOT` and `SCALE` shared between `ARP1` and `ARP2` for complete harmonic/tonal lock across both melody generators. Move them to middle row pots 9 / 10 (slots 8 and 9).
  2. Free up 2 slots per ARP row (pots 1..4 on Row 0 for ARP1, pots 11..14 on Row 2 for ARP2) to expose 4 algorithm-specific parameters with dynamic onscreen label & value text switching.
  3. Expand clock divisions to support slower notes: add `2/1` (2x), `4/1` (4x), and `8/1` (8x) before `1/1` (13 divisions total).
  4. Rename Algo 0 variants to `"HYP-1"`, `"HYP-2"`, `"HYP-3"`.
  5. Flash both RP2350 and ESP32-C3 immediately after building.
- **Implementation**:
  - **Clock Divisions (`rack-rp2350/src/mod_engine.h`, `rack-uimcu/ui/RackUI.cpp`)**:
    - `ModDivision::ppq` changed to `uint16_t` to hold up to 768 ticks. Added `8/1` (768), `4/1` (384), `2/1` (192) before `1/1` (96). `MOD_DIVISION_COUNT = 13`.
    - `kDivNames[13]` updated across all UI pages, discrete bucket pickup bounds, and mod-ride formatting.
  - **Shared Tonal Controls**:
    - Middle row pots 9 and 10 (slots 8 and 9) mapped to param 10 (shared root) and param 11 (shared scale).
    - `mod_arp_set_shared_root()` and `mod_arp_set_shared_scale()` synchronize chord degree and Neo-Riemannian triad root/minor flag across both ARP1 and ARP2.
    - Incoming external MIDI notes synchronize degree across both units simultaneously.
  - **Dynamic Algorithm Parameters & Onscreen Switching**:
    - Slots 1..4 (Row 0, ARP1) and slots 11..14 (Row 2, ARP2) dynamically update labels and value strings based on active algorithm:
      - **HYP-1..3**: `DENS` (%), `SLID` (%), `ACC` (%), `MUTE` (%)
      - **MOTIF 1..3**: `DENS` (%), `FORM` (`ABAC`, `AABC`, `ABBC`, `AABA`), `XFORM` (`INV`, `RETRO`, `TRANS`, `DISP`), `MUTE` (%)
      - **MARKOV 1..3**: `PROB` (%), `CNTR` (16 contours e.g. `ARCH`, `VALLEY`, `ASCEND`), `PEDL` (`OFF`, `ROOT`, `5TH`, `BOUNCE`), `GRAV` (%)
      - **METRO 1..3**: `PROB` (%), `PULS` (`1p`..`8p`), `RTCH` (`OFF`, `2X`, `3X`, `4X`), `ACUM` (`PENDLM`, `WRAP`, `RANDOM`, `HOLD`)
      - **DEJAVU 1..3**: `PROB` (%), `DJVU` (`MUT%d`, `LOCK`, `PRM%d`), `STEP` (`ALL`, `PENT`, `TRIAD`, `ROOT`), `SPRD` (%)
    - Dynamic labels generated on RP2350 (`ui_aux_cell_label()`), pushed via TELEM deltas/full snapshots on algorithm switches.
    - `kArpAlgoNames[15]` updated with clean Algo 0 names: `"HYP-1"`, `"HYP-2"`, `"HYP-3"`.
  - **Bug Fix**:
    - Corrected legacy `page == 7` references in `rack-uimcu/main/RackLink.cpp` to `page == 8` so SYS page BPM pot and module selector operate without clashing with MOD-ARP.
- **Hardware Flashed & Verified**:
  - `rack-elements` built and flashed to RP2350 on `COM15`.
  - `rack-uimcu` built and flashed to ESP32-C3 on `COM22`.
  - Live console testing on `COM15`: `mod 10 root neo`, `mod 10 scale 8`, `mod 10 div 8/1`, and `mod 10 algo dejavu` confirmed simultaneous dual-unit synchronization and 0 audio/I2S stalls.

### Modulation Yellow Theme & Algo Pot Pickup Stabilization (`49d113f`, 2026-11-12)
- **User Requests**:
  1. Change ARP color from pink to yellow. All modulation pages should be yellow.
  2. Fix ALGO pot instability (values jumping back to initial values when switching algorithm).
- **Root Cause Analysis (Algo Value Jump & Unstable Switching)**:
  1. **Three-Way Math Mismatch**:
     - `RackUI::potToDisplayValue`: used `potVal / 25` (5 segments of 25.0).
     - `RackUI::GetPotBucketBounds`: used `n = 15` (`(t.value * 15) / 128`, 15 buckets of ~8.53).
     - `mod_apply_cc_value`: used `val * 5 / 128` and `(val % 26) * 3 / 26`.
     - `ui_aux_cell_value`: returned `(algo % 5) * 25 + (range - 1) * 8` (un-normalized to 0..127).
     - Result: when pot was turned into an algorithm bucket, the returned echo fell into a different bucket calculation, tripping `cur > bHi + PICK_BAND` and immediately engaging soft-takeover (`pickPending = true`). When pickup was cleared, `lastValue` was forcefully snapped back to the echo target, locking the user out of switching algorithms.
  2. **Soft Takeover Echo Overwrite**:
     - `PickupCellNote` checked `lastSendPotUs < ECHO_WINDOW_US` *after* the `pickPending` block. If `pickPending` was armed, it updated `pickTarget` and never checked `ECHO_WINDOW_US`. Moving the check to the very top of `PickupCellNote` prevents any incoming telemetry echo from disturbing an actively moved pot.
  3. **Full Snapshot Flooding on Pot Motion**:
     - On every `MOD_SET` with param 0, RP2350 was broadcasting a full 152-byte `kUiTelemFull` frame. At 50–100 pot ticks/s, this caused 15 kB/s UART spam and full screen re-renders. Now, full snapshots are sent only when `s.p.arp.algo` family actually changes (`new_algo != prev_algo`); intra-algo turns send lightweight single-cell deltas.
- **Implementation**:
  - **Color Palette Alignment (`rack-rp2350/src/ui_link.h`)**:
    - Changed all cell colors in `kUiModTrigCells`, `kUiModGateCells`, `kUiModEnvCells`, and `kUiModArpCells` to `kUiColorYellow`. All modulation pages (LFO1/2, LFO3/4, TRIG, GATE, ENV, ARP) are now unified yellow.
  - **Unified 15-Choice Math Across Both MCUs**:
    - `choice = ((int)potVal * 15) / 128` (0..14).
    - `algo = choice / 3` (0..4), `range = 1 + (choice % 3)` (1..3).
    - Wire echo: `(choice * 128 + 64) / 15` (lands at the exact dead center of bucket `choice`).
    - Matched across `mod_apply_cc_value`, `ui_aux_cell_value`, `potToDisplayValue`, `renderCell`, and `GetPotBucketBounds`.
  - **Soft Takeover Guard (`rack-uimcu/main/RackPots.cpp`)**:
    - Added `ECHO_WINDOW_US` guard to top of `PickupCellNote()`.
- **Hardware Flashed & Verified**:
  - RP2350 flashed (`firmware.uf2` on `COM15`); ESP32-C3 flashed (`rack-uimcu.bin` on `COM22`).
  - Core 1 running at 472 µs (budget 496 µs), 0 I2S stalls. ALGO switching operates smoothly across all 15 choices without snap-backs.

### Generative ARP MIDI Keyboard Root Transposition (`34fdcb9`, 2026-11-12)
- **User Requests**:
  1. "transpose ARP from incoming midi notes when its assigned to PITCH parameter of any module, now incoming notes always played by synth. If ARP asigned to other parameter - current behaviour"
  2. "like setting the root note from midi keyboard"
  3. "so in this case dont need check where ARP is routed. I can just switch root to midi in its settings, thats fine for me."
- **Root Cause & Fixes**:
  1. **Synth Voice Double-Triggering / Raw Note Striking**:
     - In `midi.h`, `midiNoteOn()` unconditionally called `rack_module_note_on(note, vel)`, `applyNote()`, and `shared_gate = true`. When the user played their MIDI keyboard while an ARP was running, the raw keyboard note struck the synthesizer voice directly, fighting the arpeggio stream.
     - Solution: added `mod_arp_wants_midi_notes()`. When an active ARP has ROOT set to MIDI follow (`root_param == 0 || root_param == 1`) or is routed to voice pitch (`MOD_DEST_VOICE`, CC 1, CC 42/16 on Clouds), incoming MIDI notes call `mod_arp_note_transpose(note)` and bypass `rack_module_note_on()`, leaving `shared_gate` strictly under ARP control. When ARP is not routed or ROOT is set to a fixed note / autonomous mode, normal synth playback is preserved.
  2. **Octave Truncation Bug in `degree_to_midi_folded()` (`rack-rp2350/src/gen_tables.h`)**:
     - `int16_t midi_note = 36 + (int16_t)(root_pc % 12u) ...` hardcoded octave 36 (C2) and discarded `root_pc`'s octave entirely. C2, C3, C4, C5 all mapped to octave 36.
     - Solution: updated base note calculation to `int16_t base = (root_pc >= 24) ? (int16_t)root_pc : (int16_t)(36 + (root_pc % 12u));`. Incoming keyboard notes preserve their full octave across [28..96].
  3. **Legato Transposition & Latching**:
     - `note_stack` tracks active held keys. On key release, if another key is held, ARP transposes back to the top held key; when all keys are released, the ARP latches on the last root note.
- **Hardware Flashed & Verified**:
  - RP2350 built and flashed to `COM15` via `tools/flash_rp2350.py`.
  - Serial diagnostics on `COM15`: `max Process: 424 us (budget 496 us) | i2s stalls: 0 (+0)` confirmed live and responsive.

### Rack Module Default Parameters from ctag-tbd (`c507423`, 2026-11-12)
- **User Requests**:
  1. "ok, now i figured out that default settings for most modules are 0. take default parameters from C:\Users\schra\Developer\ctag-tbd - it has ports of all MI modules and each one has good starting parameters. But they need recalculation because of different numbers range. leave pitch at 0 and VCA at 127. If module has reverb set it to 0. trig always off. i mean take default values"
- **Recalculations & Implementations**:
  1. **ctag-tbd Data Sources**:
     - `mp-Elements.jsn`: Geometry 819/4095 (0.2f / CC 25), Damping 1024/4095 (0.25f / CC 32), Position 1229/4095 (0.3f / CC 38), Strike 3276/4095 (0.8f / CC 102), Contour/Mallet/Timbres/Flow/Brightness 2047/4095 (0.5f / CC 64), Bow/Blow 0, Reverb 0, Play 0, Pitch 0 st (CC 64), VCA 127.
     - `mp-Claude.jsn`: Density 2835/4095 (0.692f / CC 88), Texture 2729/4095 (0.666f / CC 85), Dry/Wet 3167/4095 (0.773f / CC 98), Spread 2827/4095 (0.690f / CC 88), Feedback 399/4095 (0.097f / CC 12), Position/Size 2047/4095 (0.5f / CC 64), Reverb 0, Pitch 0 st (CC 64), Mode 0 (Granular), Quality 0 (16-bit stereo), Trig/Freeze 0.
     - `mp-TBDaits.jsn`: Timbre, Morph, Harmonics, LPG Colour, Decay 2047/4095 (0.5f / CC 64), Macro 0.0f (CC 64), Engine 0 (VA), Aux 1 (Stereo OUT/AUX), Level 127 (1.0f / CC 25 & CC 7), Pitch 0 st (CC 64), Trig 0.
     - `mp-TBDings.jsn`: Structure, Brightness, Damping, Position 2047/4095 (0.5f / CC 64), Model 0 (Modal), Poly 0 (1 voice), Mode 0 (Stereo), Wave 0 (Square), Chord Table 0 (Bryan), Chord 0, Pitch 0 st (CC 64), VCA 127, Strum 0.
  2. **Cache & UI Telemetry Alignment (`rack-rp2350/src/ui_link.h`, `rack-rp2350/src/main.cpp`)**:
     - Implemented `ui_link_init_defaults()` and `ui_link_set_cached_cc()` populating `g_ui_cc_cache` and `g_ui_cc_seen` for all 4 modules.
     - Called `preset_replay_module(RACK_BOOT_MODULE)` after `ui_link_uart_init()` in `main.cpp` `setup()`.
     - C3 TELEM snapshots send the non-zero defaults on page requests.
  3. **C3 PresetBank Capacity Fix (`rack-uimcu/main/PresetBank.hpp`)**:
     - Increased `PresetBank::kMaxRp` from 896 to 1024 bytes to accommodate the 12-source mod record (927 bytes), eliminating the `PresetStore: PART overflow - dropped` error.
- **Hardware Flashed & Verified**:
  - `rack-rp2350` (`rack-elements`) flashed to `COM15` via `tools/flash_rp2350.py`.
  - `rack-uimcu` flashed to `COM22` via `tools/build_uimcu.bat flash -p COM22`.
  - Serial verified: `COM15` reports 0 I2S stalls (`max Process: 458 us`), `COM22` connects via BLE-MIDI and receives full TELEM snapshots without dropouts or overflows.

### Generative ARP Scale Preservation, Stable Root Hysteresis & Neo-Riemannian Transform Pot (`5ccdd3f`, 2026-11-12)
- **User Requests**:
  1. "the issue is on ARP page when i change scale and swich page then go back and the scale is not saved."
  2. "another issue root selection is jumpy and i selecting right item is hard. it jumps over parameters."
  3. "and how this transformations selected? ... we have free pots so if NEO selected we can adjust transformation"
- **Root Cause & Fixes**:
  1. **Scale Persistence Fix Across Page Swaps**:
     - `mod_engine.h:1801` had an ad-hoc `/ 64` and `0x40u` split that mismatched `ui_link.h:825` (`* 127 / 19`) and `RackUI.cpp:535` (`* 20 / 128`), causing scale telemetry to corrupt or zero out when returning to the ARP page.
     - Solution: Unified scale bucket math across both MCUs for all 20 scales (0..19):
       - RP2350 set: `sc = ((uint16_t)val * 20) / 128; if (sc >= 20) sc = 19;`
       - RP2350 telem echo: `val = (sc * 128 + 64) / 20;` (exact bucket center).
       - C3 display/pot: `v = ((int)potVal * 20) / 128; if (v > 19) v = 19;`
       - Every scale in 0..19 round-trips with ±3 ADC margins; scale persists identically across page switches.
  2. **Jumpy Root Selection & Soft Takeover Fix**:
     - Root selection packed 4 special modes (`MIDI`, `PARAL`, `W7`, `NEO`) into single ADC units 0..3 (1/128th of rotation each) and omitted pot 8 from `GetPotBucketBounds()`, disabling pickup hysteresis and causing violent jumps.
     - Solution: Replaced mapping with 16 stable 8-ADC-wide discrete intervals (0: `MIDI`, 1: `PARAL`, 2: `W7`, 3: `NEO`, 4..15: fixed notes `C` through `B` mapping to base MIDI note 48..59 = C3..B3).
     - Added `if (pot == 8) n = 16;` to `RackUI::GetPotBucketBounds()` on ESP32-C3 for 8-unit pickup hysteresis.
  3. **Neo-Riemannian Transformation (`TRANS` / `NEO-OP`) Pot**:
     - Assigned pot 7 (middle row next to ROOT) to parameter 12 on source 10 (`kModPotMap[5][7] = {10, 12};`, `kUiModArpCells[7] = {"TRANS", kUiColorYellow, 10, 12};`).
     - Dynamic label: appears as `TRANS` when ROOT is `NEO` (3), hides as `""` (blank black slot) in other root modes.
     - 8 voice-leading operations (16-ADC wide intervals, `n = 8` in `GetPotBucketBounds()`):
       - `0: RAND` (random across all 6 transformations P, L, R, N, S, H)
       - `1: P` (Parallel: Major ↔ Minor on same root)
       - `2: L` (Leittonwechsel: Leading-tone exchange, e.g. C Maj ↔ E Min)
       - `3: R` (Relative: Relative major/minor, e.g. C Maj ↔ A Min)
       - `4: N` (Nebenverwandt: 4th/5th shift with quality flip, e.g. C Maj ↔ F Min)
       - `5: S` (Slide: Moves root/5th by half-step holding 3rd, e.g. C Maj ↔ C♯ Min)
       - `6: H` (Hexatonic Pole: Chromatic counter-pole shift, e.g. C Maj ↔ A♭ Min)
       - `7: PLR` (Classic Neo-Riemannian cycling: random among P, L, R only)
     - `mod_arp_next_chord()` applies the selected operation, dynamically adapts scale triads to Major or Minor, synchronizes both ARPs, and persists `neo_op` in presets (`pl[8]`).
- **Hardware Flashed & Verified**:
  - `rack-rp2350` (`rack-elements`) flashed to `COM15` via `tools/flash_rp2350.py`.
  - `rack-uimcu` flashed to `COM22` via `tools/build_uimcu.bat flash -p COM22`.
  - Verified live: RP2350 reports 0 I2S stalls (`max Process: 148 us`), ESP32-C3 restored 927-byte autosave, page 7 active, telemetry clean.



### Hardware PLAY button (T1) landed + plaits TRIG semantics decoded (2026-11-14)
Plan: `docs/superpowers/plans/2026-11-14-rack-button-piezo.md`. Commits: `e9dfacf` (plan), `0822e3b` (button + rings CC 64), `25bcde0` (clouds twin), `a074351` (plaits twin), `52ad7a0` (probe). **T1 user-accepted on hardware: button works on all four modules.**
- **Button**: GP2, internal pull-up, 30 ms debounce, core-0 poll; dispatch via the REGISTRY pseudo-CC (elements/plaits/rings CC 64, clouds CC 66 — registry path, BELOW the chain intercepts). `btn=` cumulative counter in the 2 s report; console `trig [down|up]` = headless twin. rings' adapter had NO `case 64` (raw CC 64 was a silent no-op) — added.
- **Plaits TRIG semantics (the root of all three "bugs")**: UNPATCHED (CC 64 off) → the voice BYPASSES the LPG (`lpg_bypass = !level_patched && !trigger_patched`, voice.cc:862) → **ALL trigger pulses are ignored** (stock MI: free-running drone at the current patch.note). PATCHED → silent at rest (LPG gain decays to 0, `ProcessPing` settle), pings per trigger, **ping length = the DECAY knob** (mid decay = a few-ms blip). The ARP voice+trig route (dest=1) does NOT patch plaits (`mod_is_dest_routed(64)` checks dest_cc==64; ARP dest=1) — ARP-driven plaits was always the pitched drone. The GATE route (dest=64) DOES auto-patch via trig_routed.
- **User-verified working state**: TRIG patched + DECAY swept up → rhythmic gate plucks audible (gate1→64, ALT8 50 %, 120 bpm); button plucks. Probe (`tg=` pulses consumed, `go=` articulation bits: bit0 gate_resp, bit1 velocity_resp, bit2 patched) confirms pulses arrive + bits behave.
- **Diagnostics gotchas (cost hours, do not repeat)**: (1) multiple `cat` readers on ttyACM0 SPLIT the USB-CDC byte stream — phantom "state corruption"/truncated dumps; ONE reader at a time, and `(cat … &); kill $!` kills the SUBSHELL not the cat — use run_in_background + TaskStop. (2) Bytes written during the BOOTSEL window flush into the console after flash re-enumeration — the first post-flash command can arrive garbled ("unknown command"); discard the first exchange. (3) The 1200-baud touch wedges while any reader holds DTR — release the port first (killing the reader triggered the pending reboot immediately).
- **ROOT display drift (W7/NEO, reported once)**: engine `root_param` verified stable (3 console polls, 30 s) + link counters byte-identical over 10 s — transient on the C3 side, self-resolved after replug; watch for recurrence. RP2350 `g_ui_active_page` was stale (page=8 while the user was on 7) — separate cosmetic issue, unfixed.
- Pending: T2/T3 piezo (GP26 ADC, software Ears) per the plan; probe removal at next flash if unneeded.

## Rack-lira8 — LIRA-8 (Pure Data) hand-port to RP2350 (2026-11-15)

Plan: `docs/superpowers/plans/2026-11-15-rack-lira8.md` (read first).
User constraints: hand port (hvcc GPL-3.0 → generated C never ships; used as
analysis tool only), **32 kHz first** (engine stays sample-rate-agnostic —
44.1 kHz later is a constant change), module name rack-lira8, TDD with
headless-Pd golden references, subagents for exploration/data-gathering,
ctag-tbd (ESP32) portability data collected but not focused on.

| Task | Status | Committed | Notes |
|---|---|---|---|
| L0 Tooling + graph spec | in progress | — | venv (hvcc 0.17.3 + numpy) at `rack-rp2350/test/lira8_ref/.venv`; `graph.json` (4.3 MB, subpatch names re-joined) via `gen_graph.py`; PORTING-SPEC.md from spec subagent pending |
| L1 Reference harness (pd -nogui goldens) | pending | — | Pd 0.57.0 installed by user |
| L2 Voice core TDD (LiraOsc/SensorEnv/vibrato/FM) | pending | — | |
| L3 Effects chain TDD (delays/LFOs/compander/tanh/master) | pending | — | |
| L4 Rack integration (5th module: registry/ld/ui/preset/mod-engine/boot-env) | pending | — | |
| L5 Docs + ctag-portability data | pending | — | |

### Rack-lira8 L0 — patch→JSON tooling findings (2026-11-15)

- **hvcc as analysis tool**: `uv venv` + `pip install hvcc numpy` in
  `rack-rp2350/test/lira8_ref/` (no sudo). Full `hvcc` pipeline aborts on
  this patch: 238 errors = GUI plumbing (`gui.link`/`labels`/`gui`/camomile
  param routing) + **24× `v` (value) objects unsupported** (mostly
  `prm.sensor` attack/release storage) + `[route]` right-inlet warnings.
  Hand port unaffected (`v` = named float).
- **`graph.json`**: hvcc's pd2hv parser produces the whole graph as JSON
  even when the pipeline fails — dump via
  `hvcc.interpreters.pd2hv.PdParser.PdParser().graph_from_file()` +
  `to_hv().model_dump()`. hvcc DROPS subpatch names (properties only carry
  the canvas-header x/y); `gen_graph.py` re-joins names from all `#N canvas`
  headers via geometry key + object-census fingerprint (dot product of type
  counters) — different abstraction files saved at the same window position
  disambiguate cleanly. Unnamed residue: `@hv_obj` helper subpatches
  (`__and~f`, `>=~`, `coeff`, …) and the 4 lira.voice instances (identifiable
  by root coords 405,511 / 435,563 / 595,511 / 625,563).
- **Patch map (from JSON + .pd reads)**: 4× `lira.voice` = 8 osc paths (8×
  `os.triangle~`: phasor+2×polyBLEP+one-pole rlpf+samphold PWM); main
  subpatches `delay` (120 objs, dual delwrite~ 5944 @44.1k = 134.7 ms),
  `master_output` (45), `hyper-lfo` (28), `triangle-lfo`, `square-lfo`,
  `keyboard` (8 sensor sends), GUI-only `gui`/`labels`/`gui.link`.
  `ma.tanh~` = Pade x(27+x²)/(27+9x²) clip ±3; `compressor~` = ar-switched
  one-pole env follower + dB gain law; `expander~` = env~512 dB-domain.
  Census: ~490 `*~`, rpole~ 10, phasor~ 10, line~ 47, vd~ 4, samphold~ 8,
  exp~ 8, wrap~ 8, mtof 18, noise~ 1 — no FFT/tables/files.
- **Machine tooling state**: Pd 0.57.0 installed (user); uv/pip fine;
  sox absent (ffmpeg present); numpy in venv. No test infra existed in the
  repo — host test harness modeled on `EURORACK-STEREO/elements/test/`
  makefile (upstream trees have real host-side test precedent).

### Rack-lira8 L0 — Pd primitive laws verified (probes + source, 2026-11-15)

- **Probe recipe (cost an hour, do not repeat)**: pd -nogui does NOT run DSP
  unless the PATCH sends `; pd dsp 1(` (loadbang → msg — the `-send` CLI flag
  never delivers; `-nodac -noadc` alone = silent chain; `-jack` via PipeWire
  hung the patch entirely). Capture via `snapshot~`+`metro` — pd 0.57
  `print~` prints NOTHING headless and `snapshot~` has no bang inlet;
  `bang~` is also silent headless. Probes live in
  `rack-rp2350/test/lira8_ref/probes/` (self-driving, self-quitting).
- **Verified empirically @32 kHz**: block 64; `s~/r~` = one-block delay;
  `vd~` zerodel = one block; pd honors `-r 32000` (env hop cadence).
- **Verified verbatim from pd source** (d_ctl.c/d_osc.c/d_filter.c) and
  encoded in `PD-PRIMITIVES.md`: hip~ (`normal=0.5(1+coef)`, out =
  normal·(x+coef·last−last)), lop~ (coef=2πf/sr plain one-pole), env~
  (Hann/N window, MAXOVERLAP=4 ring, hop=np/2 block-rounded,
  powtodb = 100+20·log10, silence→0), osc~/cos~ (2048+1 table, LINEAR
  interp, quadrant pins), phasor~ (output-then-increment), noise~ (LCG
  435898247/382842987, per-instance seed 307·1319^k), line~ (per-sample
  ramp), mtof/ftom/dbtorms exact constants.
- **delwrite~ size is MILLISECONDS** (take-1 caught it; time law max
  5944.32 ms ≈ buffer arg confirms) → patch delay = 5.944 s/line, NOT
  135 ms as first estimated → f32@32k = 1.52 MB impossible on RP2350.
- **User decision (2026-11-15)**: int16 storage @32 kHz, cap 1.45 s/line
  (knob top ~12% compresses into cap); ESP32/ctag port keeps 5.944 s via
  PSRAM. Delay RAM on device ≈ 185 KB of the 192 KiB buffer arena.

### Rack-lira8 L0/L2 — spec cross-validation: three corrections (2026-11-15)

PORTING-SPEC.md landed (790 lines, bus table + per-section wiring + CC map).
Cross-checking it against take-1 + headless probes re-verified and CORRECTED
three claims (corrections box prepended to the spec):

1. **delwrite~ 5944 IS msec** — probe: impulse into `delwrite~ bigbuf 5944`,
   read via `sig~ 2000 → vd~` returns at entry+2000 ms exactly. Spec's
   "author conflated samples" + "clamped to 134.74 ms" = WRONG; the 5.944 s
   buffer and the full 1.45→5944 ms time law stand. User's int16@32k/1.45 s
   cap decision unaffected (it was sized for this reality).
2. **Delay buffers are PARALLEL with shared write input** (dump: `[19] =
   inlet~+noise` feeds BOTH write summers; per-bus feedback from its OWN
   expander × shared gain line~), not series as the spec claims.
3. **Compressor mux quirk moot** in-patch (attack=release=2.5 ms in both
   delay paths → both mux branches identical; implement take-1's law).

Also pinned: the four vibrato depth draws are pd's default-PRNG constants
744/930/728/920 (probe; the $0-seed receiver is dormant) — hard-coded as
`kVibDraws` in lira_voice.h with a unit test. rlpf "hazards" (44100/22050)
confirmed overridden by samplerate~ (correction in PD-PRIMITIVES.md).

**Probe gotcha (recurring)**: hand-written probe patches keep silently
dropping `#X connect 0 0 1 0` (loadbang→`; pd dsp 1`) — always check that
connect FIRST when a probe prints nothing.

**L2 status**: pd_primitives + voice layer (LiraOsc/SensorEnv/VoicePair)
TDD GREEN on host (06314cc, 4558005): osc freq tracking/PWM/BLEP bounds,
probe-exact hip (0.778916), sensor A/R + cos morph, tune table frequencies,
pair gating, bus DC-block, vib depth constants. Next: L3 engine (buses +
delays + master) per the corrected topology.

### Rack-lira8 L1/L3 — golden harness live + tune-law bug found (2026-11-15)

- **Golden harness WORKS end-to-end**: vanilla-pd-only bridge
  (`gen_l8bridge.py` → patch/abs/l8bridge.pd: [netreceive 7001] TCP → route →
  named sends into `$1`-scoped receivers; udpreceive is an EXTERNAL, not
  vanilla). Driver `drive_ref.py` (TCP, `;\n` framing), `render_ref.sh`
  (per-scenario wrapper patch `_LIRA-8-golden.pd` via `gen_golden_patch.py`:
  pristine patch + writesf~ tap + bridge). pd writesf protocol =
  **open <file> → start → stop** (not start <file>). compare_wav.py gained
  --align (cross-correlation lag search).
- **BUG FOUND & FIXED via goldens**: `VoicePair::pair_` was never initialized
  from the ctor arg → all four pairs played tune rows 0-1 (voices 3-8 at
  voice-1/2 pitches). One-line fix (469dfa1).
- **BUG FOUND & FIXED via goldens: THE TUNE LAW WAS INVERTED.** Empirical:
  pd at tune-1=127 renders 1838 Hz = mtof(93)·pitch (the HI note); the
  spec §7.2/take-1 law (hi + k·(lo−hi)) predicted 3.2 Hz. Correct law:
  **note = lo + (k/127)·(hi − lo)**. With the fix the aligned correlations
  jumped: drone-all 0.001 → 0.665, single-pair 0.48, delay-extremes 0.36
  (fuzz-push 0.05 and transients 0.005 still failing).
- **$0 correction**: the standalone root canvas $0 = **1003**, not the
  spec's 1001 (probe). Irrelevant in practice — the bridge receives $0 as
  an abstraction arg.
- **Remaining fidelity deltas (aligned, corr 0.05-0.67)**: (1) PWM duty —
  pd's square shows pure odd harmonics (duty locked 0.5) while ours sweeps
  with the vibrato (PWM = 0.53125 + 0.025·V) — the samphold/PWM path needs
  re-derivation from the patch; (2) transients scenario envelope timing;
  (3) fuzz-push (master drive chain). Spectral peak SETS now match
  (voice pitches correct across all 8 voices).
- Test suite state: 4 binaries green (primitives/voice/engine/wav_io);
  golden compare = the new red/green loop for the fidelity tail.

### Rack-lira8 L3 — vibrato semantics corrected (2026-11-15, second law fix)

Connect-list re-derivation of lira.voice: the random-1001 chain feeds the
osc~'s FREQUENCY inlet → the per-pair draws (744/930/728/920) are the
**VIBRATO RATES** (0.5–3.5 Hz), not depths. V = cos(rate·t)·vib_param ∈ ±1;
freq = T·(1+0.005·V); PWM = 0.53125 + 0.025·V (duty 0.506–0.556, weak even
harmonics ✓ matches pd's spectrum). Fixed in lira_voice.h (kVibRate table).

Aligned-golden status after the fix: drone-all corr 0.775 (best), pair
scenarios regressed to 0.18-0.26 (vibrato phase drift across multi-second
windows + remaining PWM/env deltas; the align search window ±250 ms is at
its edge). Remaining fidelity tail, in order of expected impact:
1. PWM duty path — verify the samphold sample instant matches pd (the even-
   harmonic structure differs).
2. Transients scenario (corr 0.005) — envelope event timing/attack shape.
3. Fuzz-push (corr 0.05) — master drive chain (pow~31 law + makeup).
4. Bus one-block semantics per-bus vs pd's actual sort order.
Tests still green; goldens + engine renders committed for reproducibility.

### Rack-lira8 L3 — master tanh clip + hold law + clean goldens (2026-11-15)

User ear report on the goldens ("noisy like light bitcrushing", "fuzz-push
harsh then volume drops", "cleantest also distorted") drove three more fixes:

1. **MasterOut tanh had NO ±3 input clip** (ma.tanh~ clips before the Pade):
   at drv=127 the pre-tanh signal hits ±19 → the Pade ratio grows ~x/9 →
   pow(2.25,31) exploded the chain to full scale (fuzz-push eng rms 1.0 vs
   pd 0.29). Fixed; the fuzz-push rms trajectory now matches pd per-second
   (0.264/0.291 → drop 0.037/0.040 → tail 0.029/0.029 — the "volume drop"
   the user heard = the SCRIPTED vol=40 event, by design).
2. **int16 delay headroom**: ±1 clipped the feedback loop (bitcrush);
   ±4 made the floor 12 dB worse; settled on ±2 (floor −79 dB vs pd −83,
   in-loop compander self-limits content).
3. **HOLD LAW decoded**: hold SUMS into the gate input (pd signal inlets sum):
   gate = min(e² + hold, 1) — hold=0 = normal envelope, hold=1 = latch-on
   (the Lyra hold button). Was min(e², hold). Explains pd's transients being
   audible at hold=0 AND drone-all sounding from t=0 (baked sensors ON,
   hold baked 1 → latch from load).
4. **$0 = 1003** root canvas (probe); the DSP receivers live INSIDE the gui
   subpatch → the bridge is now instantiated INSIDE gui (gen_golden_patch
   inserts it before the gui restore) — prior goldens had the init landing
   on nothing (pd played its baked state).
5. **Goldens were polluted**: a cancelled render loop left a 57-s fuzz-push
   (pd kept recording baked audio). Full clean re-render, sizes verified
   (1151788-1152044 B = 9.0 s each).

Aligned-corr state (fresh goldens, correct init): drone-all 0.694,
single-pair 0.813, fuzz-push 0.643, transients 0.366 (rms/s tracks pd within
~15-20%), delay-extremes 0.059 (chaotic feedback loop — sample corr may be
the wrong metric there; spectral/texture comparison pending). HF floor:
eng −72.9 vs pd −85.2 at 8-15 kHz — 3.2 kHz-below bands match within 1 dB.
The patch's dirt (noise~ + fuzz + feedback) is INTENDED (the author's demo
track is 8-10 dB hotter in mids/HF than our default-state renders).

### Rack-lira8 L3 — delay-extremes fb boundary (2026-11-15)

User ear report: "delay-extremes distorted and noisy, not like delay; engine
worse". Root cause: the scenario's feedback=100 → g=2.35 > 1 — before the
in-loop compander's 2.5 ms attack catches up, each loop pass multiplies by
2.35 → transient overshoots past ±10 → the int16 ±2 line hard-clips (engine
audibly worse than pd's float loop, which rides the transients out). pd's
own fb=100 output IS self-oscillation distortion (loop gain > 1 — "not like
delay" = correct perception of that setting).

Fix: delay-extremes scenario re-scoped to feedback=60 (g=0.91, loop gain
0.71 — stable with musical self-oscillation onset). rms trajectories now
track: pd 0.208/0.186/0.134/0.125… vs eng 0.288/0.225/0.150/0.132…
(converges to ±5% by t=3). Sample corr stays ~0 — EXPECTED: a fb-0.9
self-modulating delay is chaotic; waveform alignment is meaningless there.
Metrics for that scenario = rms envelope (matching) + ear texture.
Documented in scenarios.txt: keep golden feedback <= ~60 on the int16
delay; fb > 65 = self-oscillation territory (authentic but clipped by
int16 — the ESP32 float port keeps the full range).

### Rack-lira8 L3 — harmonic-table diagnosis (2026-11-15, session end state)

Clean-voice (single osc, tri path, drv=40) harmonic table, normalized %:
pd  [100, 61, 15, 24, 26, 7, 20, 2, 18, 2] vs eng [100, 47, 11, 6, 21, 5, 2, 1, 1, 3]
→ pd carries strong H4/H7/H9 that the engine lacks — the audible "string vs
harsh" difference. Hypothesis: the vibrato→PWM depth swing in pd is larger
than the V ∈ ±1 model (duty ≈ 0.6 excursions → strong even harmonics), i.e.
take-1's depth reading vs my rate-only reading remains UNRESOLVED — the
connect list says the random chain feeds osc~'s frequency inlet (rate), but
pd's measured H4=24% demands more duty swing. NEXT: instrument the pd patch
(snapshot~ on the PWM signal) to measure the true duty range directly.

Also this session: delay bypass bisect (del-mix=0) — the HF floor (−72 dB)
is in the VOICES/MASTER, delay/int16 exonerated; LiraOsc phasor moved to
double (correctness, pd d_osc.c uses double); LFO triangle depth fixed
(converter = |p−0.5|·4−1, no extra scaling — was halved); delay-extremes
scenario re-scoped fb=60 (int16 clip boundary); hold law fixed
(min(e²+hold, 1)); master tanh input clip (fuzz explosion). All tests green.
Aligned corr: drone-all 0.69, single-pair 0.81, fuzz-push 0.64, transients
0.37, delay-extremes 0.06 (chaotic loop — use envelope/texture metrics).

### Rack-lira8 L3 — single-voice spectral EXACT match (2026-11-15)

The onevoice scenario (hold=0 + sensor 1 only = TRUE single voice; all prior
"single voice" measurements were 4-voice latch mixes — hold=127 latches all):
pd f0 81.0 Hz / period 395.6 / duty 0.532 vs eng IDENTICAL; harmonic table
[100, 9, 31, 10, 18, 9, 10, 9, 7, 8] matches to the last digit. The PWM
model (duty = 0.53125 + 0.025·V, V ∈ ±1, per-pair vibrato RATE) is CONFIRMED
correct; the earlier H4/H7/H9 "anomaly" = 4-voice latch mixes, not an osc
defect. Waveform corr 0.30 on a stationary tone = pure phase alignment
(meaningless). THE OSC CHAIN IS SPECTRALLY FAITHFUL.

Also fixed: render_golden end-time parsing (the "render stop" line's t is
the true end; was last-event+1000 — mismatched render lengths).
Remaining fidelity tail: multi-voice phase/coherence (corr is the wrong
metric there too), delay-extremes chaotic texture, the −12 dB HF floor
delta (source still unknown — delay/int16/rate/float-phase ALL exonerated
by bisect; next suspect = the voice-sum/compander micro-differences).

### Rack-lira8 L3 — full golden re-render on current build (2026-11-15)

All five scenario engine renders + pd goldens regenerated on the current
code (post LFO-depth fix + double phasor). Waveform corr shifted (drone-all
0.69→0.33, single-pair 0.81→0.38) — EXPECTED: the corrected 2× deeper LFO
makes the chaotic multi-voice renders phase-sensitive; pd's LFO phasors and
the engine's start at different absolute phases → the delay texture
decorrelates regardless of port fidelity. Corr is retired as the primary
gate for drone scenarios; the gates are now: single-voice spectral exactness
(ACHIEVED, onevoice), envelope trajectories (matching), spectral envelopes
(matching within ~1-2 dB below 3.2 kHz), and the user's ears.
Fresh A/B set: rack-rp2350/test/lira8_ref/goldens/ (X.wav vs engine_X.wav,
all five scenarios + onevoice + drone-nodelay + cleantest).

### Rack-lira8 L4 — rack integration complete, all five envs build (2026-11-15)

Fifth module wired end-to-end following the established adapter pattern:
- **src/module_lira8.h**: the adapter (arena-placed State = Lira8Engine BY
  VALUE; the int16 delay carve 185,616 B from the BUFFER arena at init —
  ~96% of the 192 KiB arena, so **chain A rejected by design**, documented in
  the chain command + the pair validation). CC map remapped off the reserved
  ranges (32/33-35/40-50/64-66): tune 1-8, fast 9-12, sharp 13-16, mod 17-20,
  source 21-24, pitch 25-26, hold 27-28, switch 29, total-fb 30, vibrato 31,
  drv 36, dst-mix 37, vol 38, f-a 39, f-b 41, andor 42, link 43, time-1/2
  44/45, feedback 46, del-mix 47, mod-1/2 48/49, del-mod 51, lfo-wav 52,
  quantize 53; **sensors = notes 36-43 (the patch's notemap) + latch triggers
  CC 55-62**; PLAY button (pseudo-CC 64) = sensor-1 400 ms pluck.
- **lira8_tc.h** + LIRA8_TC applied to 10 hot functions (osc/voice/env/delay
  read+write/comp/exp/lfo/master/engine block+mux+delay): nm-verified all in
  the SRAM window (0x20063000-0x20063ff0, tc_lira8 = 4,080 B = 7% of the A
  slot). Host tests compile the same headers PLAIN (no RACK_RP2350_TC) —
  the ctag-tbd portability proof holds.
- **Engine refactor**: NO heap — ModDelay takes external int16 storage, the
  engine holds value members, delay_bytes_for/nsamps_for constexpr (the
  adapter's static_assert pins the carve). Host tests re-verified after the
  refactor (onevoice corr 0.83 aligned — the plumbing is sound-neutral).
- **Registry/enum/ld/env**: RACK_MODULE_LIRA8=4 + COUNT=5; the tc_lira8
  overlay member + flash extents + the window assert in memmap_rack.ld;
  rack_window.h symbols/table; main.cpp boot-guard 0-4 + the include (no
  LUT_* collisions — the engine defines none) + the registry row; the chain
  pair table REJECTS lira8 as A (buffer-arena conflict, honest serial line);
  mod_engine dispatch cases; ui_link kUiPageLira8P1 (16 cells: tunes 1-8,
  hold, volume, sharp, mod, vibrato, times, feedback); platformio env
  rack-lira8 (RACK_BOOT_MODULE=4).
- **preset.h v2 → v3**: the cc_seen/cc_cache arrays auto-grew 4 → 5 modules
  (+720 B blob) — the version bump invalidates old C3-side presets cleanly.
- **2 s report**: a lira8 tail `sens=N` (the requested-on sensor count).
- ALL FIVE envs build SUCCESS. bss 361,596 B (unchanged accounting — the
  arenas dominate), heap ≥ the 30 KB gate preserved.

KNOWN LIMITATION (v1): the C3 display side (rack-uimcu) has no lira8 page
yet — switching to lira8 from the C3 shows the generic header until the
rack-uimcu page tables gain the module. Next: on-device flash + ear test.

### Rack-lira8 L4b — on-device performance: 3.1 ms → 1.31 ms per 64-block (2026-11-15)

The first L4 flash booted at max Process 5,100 us (10x over the 496 us host
budget); the double->float LiraOsc phase revert took it to ~3,400 us. Stage
probes (dbg_* maxima per engine stage, printed as `st=<params>/<voices>/
<delay>/<lfo>/<master>`) isolated the cost at idle, per internal 64-sample
block:

| stage | before (us) | after (us) |
|---|---|---|
| voices | 2,414-2,610 | 902-954 |
| delay | 243-281 | 217-239 |
| lfo | 201-260 | 157-171 |
| master | 258-279 | 41 |
| apply_params | 0 (spikes 134-155 when dirty) | 0 |
| **max Process** | **3,108-3,272** | **1,303-1,380** |

Root causes and fixes:
1. **-Os build trap on src/**: main.cpp compiles with the project's -O3 THEN
   the earlephilhower core's appended -Os (last -O wins) — the header-only
   engine inlined into src ran at -Os. Fix: `lib/lira8_engine/` (library.json
   -O3 + `lira8_engine_o3.cpp` wrapper `process_block_o3` carrying its own
   SRAM token; the adapter calls it and never Process()). Verified: the lib
   TU's command line is `-O3 -Os -O3`; the wrapper lands in the SRAM window
   (tc_lira8 grew 4,600 -> 10,352 B, 0x20063000-0x20065870). lib_ldf_mode
   chain+ needs the include from module_lira8.h (lira8_engine_o3.h) — a
   symbol-only reference does NOT link the library.
2. **libm calls per sample**: SensorEnv did 2x std::cos per pair-sample
   (-> pd's cos~ 2048-entry table, which IS what pd uses; 512 calls/block
   removed); LiraOsc did std::exp for the tri-filter coefficient
   (-> exp_neg_range, degree-11 Taylor about -1.6, |rel err| < 1e-6, 512
   calls/block removed); master did pow(t,31) (-> repeated squaring); the
   hyper-LFO did mtof(rb_) per sample (-> hoisted into set_rate_b, exact);
   phasor floor -> while-subtract.
3. **Double phase kept where it matters**: CosOsc (vibrato) and Phasor
   (LFOs) stay double — float drifted the single-pair A/B to -16.5 dB rmsdiff
   / corr 0.989. Audio-rate LiraOsc stays float (that is where the 10x
   software-double cost lived).
4. Sensor cos~ and the osc table are pd-faithful (pd's objects are tables).

Fidelity A/B (host; base = handoff state vs optimized): drone-all -52.4 dB /
corr 0.999997, single-pair -25.6 dB / corr 0.9986 (the divergence starts
exactly at the t=5 s mod-12 event: audio-rate cross-mod amplifies the -80 dB
residue into audio phase; harmonic magnitudes identical to 0.00 dB — the
documented gate), delay-extremes -54.2 dB, fuzz-push -50.8 dB, transients
-48.8 dB. All host unit tests green.

On device: 1,303-1,380 us per internal 64-sample block = 65% of the 2 ms block
period. The 4x burst (one internal block per 4 host calls) is the documented
pd block architecture; the 8x16 I2S bank (4 ms) absorbs it — i2s stalls 0,
including with sensor 1 plucked (sens=1, non-zero output meter). Next: the
user's ear test is the acceptance gate.

### Rack-lira8 L4c — C3 UI: three pages, five modules, preset cap fix (2026-11-15)

The C3 (rack-uimcu) was blind to the fifth module (4-name table + `& 3`
masks) and lira8's 47 params cannot fit two 16-cell pages. Implemented:

- **RP2350 `ui_link.h`**: `kUiPageP3 = 10`, `kUiPageCount = 11`, plus
  `ui_page_is_module()` / `ui_module_page_idx()` helpers used by TELEM full/
  delta and ASSIGN_CYCLE (page 10 is a module page everywhere). lira8 page
  tables: P1 unchanged except clearer labels (HOLD 1-4, SHARP 12, MOD 12);
  **P2** = SHARP/MOD 34/56/78 (completing the 8x tune/sharp/mod matrix),
  PITCH 1-4/5-8, HOLD 5-8, DRIVE, DST MIX, DEL MIX, MOD D1/D2, SWITCH,
  TOTAL FB; **P3** = FAST 1-4, SOURCE 1-4, LFO A/B, AND/OR, LINK, DEL-MOD,
  LFO WAV, QUANTIZE + 1 empty. Console lists module 4.
- **C3 `RackUI.cpp`**: 5 module names + `ModuleClamp`, `ModuleHasP2` =
  elements+lira8, new `ModuleHasP3`, `PageIsModule` in every page-kind check
  (pot target/bucket/render/mod-ride), dynamic page-list builder
  (P1,[P2],[P3],chain-B,2-7,SYS), SYS selector over 5 slots, page-10 gating
  on module change, lira8 discrete cells (FAST/SWITCH/TOTAL FB/VIBRATO
  bool127; SOURCE/DEL-MOD 3 raw; LFO WAV/QUANTIZE 2 raw), pitch display
  guard (lira8 CC 1 = TUNE 1, not pitch). RackLink: SendSwitch accepts 0-4,
  local sys-switch maps 5 slots, TELEM module-change detection on page 10.
- **BENCH-FOUND BUG: preset v3 blob is 1071 B but kPresetBlobMax and the
  C3's kMaxRp were 1024** — `preset_serialize` overran its 1024-byte array
  and the C3 dropped every legitimate PART burst ("PART overflow - dropped",
  5th chunk off 1008+63 > 1024). Both caps raised to 1088 + a static_assert
  pinning kPresetSize. **CC 53 (QUANTIZE) was never applied** — the engine's
  apply_params was missing the set_quantize call (inert UI cell); wired.
- **Verified on the hardware link**: TELEM page=0/1/10 all mod=4 (lira8),
  page cycling 0->1->10->2..8, SYS selector acks modules 0-4, autosave
  "1071 B rp part ... OK". Firmware: only rack-lira8 + rack-uimcu rebuilt/
  flashed (user instruction: do not rebuild unchanged modules).
- Known: the browser rack_panel.html has no lira8 tab yet; the physical C3
  UI is the control surface for now.

### Rack-ui follow-ups from the bench (2026-11-15)

- **SYS module selector reported a uint8_t wrap**: `ui_aux_cell_value` returned
  `g_active_module * 32` (4-slot spacing) — lira8 (module 4) became 128 -> 0,
  and rings (96) landed in the plaits bucket, so the C3 showed no LIRA8 and
  the selection jumped. Now `g_active_module * 127 / (RACK_MODULE_COUNT - 1)`
  -> 0/31/63/95/127, each module its own 5-slot bucket (verified on the C3
  console: `page SYS mod=4`, cell[0] val=127 after CC 32=4).
- **Stale page snapshots across a link dropout/reflash**: the C3 kept the old
  firmware's SYS cells until a navigation happened (bench: showed ELEMENTS
  while the rack was lira8). RackUI now drops the snapshot on any link edge
  and re-requests the ACTIVE page on offline->online.
- C3 `RackLink::kPageCount` 10 -> 11 (console `page 10` was rejected), console
  help text, `kModuleSlots` constant for the 5-slot selector, and stale
  4-module comments (midi.h CC 32, ui_link aux cells, main.cpp).

- **The SYS selector's NAME display still used 4 slots** (`potToDisplayValue`
  SYS slot 0: `n = chained ? 3 : 4`) — the send path selected module 4 at the
  top of the pot but the value text computed RINGS, so lira8 looked
  unselectable (bench report "only elements clouds plaits rings"). n is now
  `kModuleCount` (5) and the bar max 4. Sweep check: a continuous turn sends
  0->1->2->3->4 while the pickup is skipped inside the echo window.

### Rack-lira8 correctness batch A (2026-11-15, review-driven)

Two parallel review agents (lira8 internals + optimized-code survey) found
fidelity bugs; the four confirmed ones are fixed:
1. **LFO-A was frozen**: `set_rate_a()` stored `ra_` but never called
   `pa_.set_freq(mtof(ra_))` → the A phasor never advanced (F-A knob inert,
   `square_a_` pinned +1, del-tri offset/half depth). Now set.
2. **Delay line~s ran 64x too slow**: `time_line_`/`mod_line_` advanced once
   per 64-block, `fb_line_` twice, `del_mix_line_` once → 23.22 ms smoothing
   became 1.49 s (feedback 743 ms). Now per sample, with the feedback-gain
   line advanced once per block and shared by both write summers + the wet
   divisor (pd has ONE line~).
3. **Sensor fast-mode re-ramp on every CC**: `apply_params()` calls
   `set_fast()`; it re-ramped the attack of held sensors on ANY parameter
   change. Now only acts when the FAST mode actually changes (and re-emits
   the stored target for OFF sensors too).
4. **Wet-monitor tanh missed the ±3 clip**: pd's ma.tanh~ clips before the
   Pade; without it the wet path could exceed 1.0 and grow in the loop.

Verification: host unit tests green; **onevoice render bit-identical**
(rmsdiff -213 dB, corr 1.0000 — the spectral-exact gate untouched); vs the Pd
goldens the 0.5 s RMS-trajectory correlation improved on 4/5 scenarios
(delay-extremes 0.871→0.972, fuzz-push 0.978→0.998, transients 0.683→0.864,
drone-all 0.498→0.536; single-pair 0.748→0.739, flat). On device: max
Process 1259-1432 us (delay stage +~10 us for the per-sample ramps), i2s
stalls 0. Batch B (polyBLEP falling edge, delay-input source, SELF tap,
vd~ tap alignment, expander dB ramp) awaits verification against the Pd dump.

### Rack-lira8 correctness batch B (2026-11-15, review claims verified + fixed)

Each claim was checked against the Pd source/resolved dump before changing:
1. **vd~ tap window shifted +1**: pd sigvd_perform uses bp = wp - idelsamps and
   d=bp[-3], c=bp[-2], b=bp[-1], a=bp[0]; the port started at bp[-1] for b
   (reads landed one sample late, D-1). Taps corrected.
2. **Delay input source**: the voice subpatch's signal OUTLET is the raw
   current-block pair sum (+~ -> outlet~); the hip~3'd copy goes only to the
   s~ del mod bus. The port summed buses_prev_ (hip~3'd, one block late) for
   the dry/write path -> 2 ms lag + 3 Hz HPF. Now sums blk_ (current raw).
3. **SELF tap**: pd has a SECOND vd~ per bus with an UNCONNECTED time inlet
   (minimum delay, output constant across the block) -> /2 -> lop~689.
   The port fed the main wet vd_out back as SELF. Added ModDelay::read_min_tap()
   (most recent sample; one block of absolute lag on the mod source only).
4. **polyBLEP falling-edge term**: the abs patch computes (p-1)/n + 1 = u+1;
   the port added +1 before the square (=> u+2). Fixed in both edges.
5. **Expander line~ domain**: pd ramps the dB value over 11 ms then
   dbtorms~ per sample; the port ramped LINEAR gain from 0 instead of 0 dB =
   unity. Now a constant per-sample gain ratio (exactly the exponential
   dB-linear curve, one powf per env~ fire) starting at 1.0.
6. **Fractional saved defaults**: set_sharp/set_mod take the raw float (pd
   defaults 14.377/43.132/... were truncated by (int)).

Verification: host tests green; onevoice harmonic table still matches the Pd
golden within 0.02% per harmonic; Pd 0.5 s RMS-trajectory corr improved
again: single-pair 0.739->0.838, delay-extremes 0.972->0.985, fuzz-push
0.998->0.999, transients 0.859 (flat), drone-all 0.536->0.502 (chaotic/
phase-sensitive; RMS ratio 0.0 dB). On device: max Process 1252-1475 us,
delay stage +~20 us (SELF tap + dB ramp), i2s stalls 0.

### Rack-lira8 batch C: remaining LOW fidelity items + fixed-point phasors (2026-11-15)

- **Source mux gates**: pd computes (==)*0.999 + 0.001 -> exactly 1.0 when
  ON; the port used 0.999 (verified from abs/prm.source.pd). ON = 1.0 now.
- **Delay-mod gates**: pd uses hard ==0 / ==2 comparators for the self/LFO
  CV into the read time; the port's 0.999/0.001 soft gates leaked 0.1%.
  Now hard 1.0/0.0.
- **Hyper-LFO rate smoothing**: pd = r s-f-a -> /127, pow 2, *127, -75 ->
  mtof -> line~ 23.22 ms -> phasor~ freq (the smoothing is IN HZ). Both rates
  are now per-sample Ramps; set_rate_a/b ignore repeated identical values
  (apply_params re-applies on every CC).
- **CosOsc/Phasor -> 32-bit fixed-point phase** (2^32/cycle): the M33 has no
  double FPU; every double op was a ~50-cycle soft-float libcall. Integer
  phase addition is exact modulo 2^32 (zero drift), the increment is one
  float mul + convert, negative increments wrap correctly. Removes the
  soft-double cost from the vibrato LFO and the hyper-LFO without the float
  drift measured earlier.
Verification: host tests green; onevoice bit-identical; B->C differences
-20..-40 dB (LFO rate ramps at boot/CC changes); Pd trajectory gates
unchanged (0.504 / 0.837 / 0.985 / 0.999 / 0.859).

### Rack-lira8 perf batch D (2026-11-15, behavior-preserving)

- **D1 (dc438d3)**: LiraOsc hoists 1/sr and -2*pi/sr (3 divisions/pair-sample
  -> 1 branch-only); cos_lookup's table became a C++17 inline variable (one
  instance, no per-call thread-safe-init guard on ~900 lookups/block);
  EnvRms window state sized to the 512-point env~ actually used (~4 KB per
  expander); ModDelay::s2f + compander ratio are reciprocal multiplies.
  Host A/B -58..-78 dB, corr 1.000000.
- **D2**: VoicePair settled-block fast path — when the six control line~s
  and both sensor envelopes are idle (normal drone/held state), hoist them
  and skip 6 Ramp::next plus 2 cos lookups per pair-sample; same
  expressions/rounding order. Host A/B bit-identical (-213..-227 dB).
- On device: max Process **1252-1475 -> 1123-1159 us** (57% of the 2 ms
  block); voices 865-968 -> **676-687 us**; delay 247-268 -> 220-238; LFO
  125-177 -> **39-42 us** (fixed-point phasors + per-sample rate line~s);
  master 41-47; i2s stalls 0. The remaining >100 us item on CC bursts is
  apply_params (158-211 us transient, not steady state).

### Rack-lira8 perf batch E + a real cos_lookup bug (2026-11-15)

- **cos_lookup exact-top bug**: the fixed-point->float conversion can land on
  exactly 2048.0 table units (rounds the top 256 phase units). The old code
  wrapped only `idx`, leaving frac = 2048 -> a + 2048*(b-a) = a huge spike
  once per cycle on the vibrato path (unclipped). Wrapped idx now sets
  frac = 0 (t[0] = t[N] = 1 are pinned), so the top sample is exact. Latent
  in v1 (double/float phases never reached 2048.0); exposed by fixed-point.
- **Ramp::set no-op**: re-sending the CURRENT value now holds (pd's line~
  zero-delta ramp) instead of starting a 743/256000-sample idle ramp. This
  keeps idle() true, so the settled fast paths survive the every-CC
  apply_params pass.
- **Incremental apply_params**: a per-param change mask from the dirty scan;
  only the affected setters run (was: all 47 params -> 15+ exp/pow on every
  CC = the 158-211 us transient). apply_defaults still passes nullptr.
- **Mid-state voice path**: control ramps idle while sensor envelopes move
  (attacks / the 8 s release) -> hoist the six control values, keep the
  per-sample sensor law.
Verification: host tests green; D2->E renders bit-identical (-213..-227 dB);
onevoice unchanged. On device: max Process 1123-1159 -> **807-808 us** (40%
of the 2 ms block), voices 676-687 -> **508-509**, delay 211-212, LFO 40-43,
master 41-47, params spikes 158-211 -> **0-46**, i2s stalls 0.

### Rack-lira8 HW delay cap + manual-based UI (2026-11-15)

User: the ORIGINAL HARDWARE's delay maximum is **1 second** (the Pd patch's
5.944 s range is a patch-ism), and the UI should follow the LYRA-8 USER
MANUAL's structure.
- **kCapMs 1450 -> 1000**: the int16 carve is now 2 x 32,004 = 128,016 B
  (~65% of the 192 KiB arena). The pd TIME law is kept, so the top ~22% of
  the TIME knob saturates at the cap (like a pot hitting its stop). Chain-A
  still rejected: A + clouds-B needs 128,016+87,040+44,032 = 259,088 B.
- **UI re-arranged from the manual** (pairs 12/34/56/78, groups 1234/5678):
  P1 = group 1234 (row0 pair12 TUNE1/2 SHARP/MOD/FAST/FM-SRC; row1 PITCH
  1234, HOLD 1234, TUNE3/4; row2 SHARP/MOD/FAST/FM-SRC 34 + 2 spare),
  P2 = group 5678 mirrored, P3 = HYPER LFO (FREQ A/B + the delay TRI/SQR +
  SELF/LFO) + the three globals (TOTAL FB, VIBRATO, FM STRUCT) on row0,
  MOD DELAY row1 (TIME1/2, MOD1/2), DELAY+DISTORTION row2 (FB, MIX, DRIVE,
  DIST MIX, VOL). Naming follows the manual (FM SRC, FM STRUCT, TRI/SQR,
  SELF/LFO, FB, MIX, DIST MIX, VOL). The manual has 18 controls for P3 and
  the page holds 16: AND/OR (CC 42) and LINK (CC 43) stay MIDI/preset-only,
  as does QUANTIZE (a Pd-patch extra not on the hardware).
- **Harshness investigation (unresolved)**: the dry path matches the Pd
  golden within +-0.2 dB in every band; the FULLY-WET static delay is ~10 dB
  brighter at 12.5-16 kHz than the Pd golden while the LF bands match. The
  wet chain (hip1 -> lop4k -> compressor -> expander -> /max(g,1.5) -> tanh),
  the vd~ taps, the delay input source and the line~ rates all match the
  resolved dump. A/Bs: expander bypass = no change (unity gain there),
  compressor bypass = louder with the tilt intact, drive 0/86 = +-1 dB,
  int16 headroom +-4 = ~4.5 dB louder (some +-2 clipping) but the tilt
  persists (~1 dB of it explained). The Pd golden generator now hangs
  (pd exit 124 even for delay-extremes) so a static-wet pd reference could
  not be produced; the port stays int16 (float needs 256 KB at 1 s).
Device: 803-810 us/block, delay stage 207-222, stalls 0.

### Rack-lira8 SELF grit + drive grit + merged delay-mode switch + user pot map

Bench report: LFO delay modulation (del-mode 2) is clean; SELF (0) is gritty;
DRIVE at high values is gritty.
- **SELF grit root cause**: pd's SELF vd~ has an UNCONNECTED time inlet, so it
  holds ONE sample across each block -> the delay-time modulation is a 500 Hz
  staircase (a read-position jump of up to |self|*mod_ms per block = clicks +
  HF). Fixed by replaying the previous block's FLOAT writes (self_buf_) as
  the SELF source: same "modulated by its own output" law, smooth time
  signal, and no int16 quantization on the mod source.
- **DRIVE grit**: one-pole at 7 kHz on the DISTORTION branch only (the
  manual's "the highs sound softer" hardware voicing; the Pd patch has no
  such filter). Tames the pow~31 aliasing without darkening the clean path.
- **Merged delay-mode switch** (the old 3-state CC51 was an artifact: the
  patch gates ==0 self / ==2 lfo, leaving 1 = no modulation, which the
  hardware does not have). CC51 is now the manual's SELF/LFO switch merged
  with the waveform switch: 0 = SELF, 1 = LFO-triangle, 2 = LFO-square.
  Small values are the C3's discrete states, larger MIDI values map in
  thirds; CC52 stays a direct MIDI waveform override.
- **Pot map (user)**: P1/P2 = FAST + FM SRC of both pairs on row0 (2 spares),
  the four TUNEs on the middle row, SHARP/MOD of both pairs + PITCH + HOLD
  on row2. P3 = DEL MODE, MOD1, TIME1, TIME2, MOD2, FB (row0) / MIX, DRIVE,
  DIST MIX, VOL (row1) / FREQ A, FREQ B, TOTAL FB, VIBRATO, FM STRUCT (row2).
- **C3 switch texts**: FAST -> SLOW/FAST; FM SRC -> FM/OFF/LFO; DEL MODE ->
  SELF/TRI/SQR; FM STRUCT -> SPLIT/LOOP; TOTAL FB/VIBRATO -> ON/OFF.
- Engine housekeeping: all delay/audio work arrays memset in the ctor (the
  SELF float feedback path needs defined initial content).
Device: 804-862 us/block, delay 212-218, voices 504-535, i2s stalls 0.

### Rack-lira8 max-FB grit, delay controls on P1/P2, live switches (2026-11-15)

- **Gritty at maximum FEEDBACK**: the self-oscillating loop (FB 127 -> loop
  gain 4) squared off at the hard int16 storage clamp. f2s now has a
  SOFT-KNEE limit: linear to +-1.5, then a Pade-tanh shoulder asymptoting at
  +-2 (continuous slope; below 1.5 identical to before). The loop now
  saturates like an analog delay instead of hard-clipping.
- **Delay line controls moved to the group pages** (user): P1 row0 = FAST 12,
  MOD 1 (pot2), TIME 1 (pot3), FM SRC 12, FAST 34, FM SRC 34; P2 mirrored
  with MOD 2/TIME 2. P3 now holds the delay's SHARED controls and the
  distortion (row0: DEL MODE, FB, MIX, DRIVE, DIST MIX, VOL), LFO + globals
  (row1: FREQ A, FREQ B, TOTAL FB, VIBRATO) and the remaining switches
  (row2: FM STRUCT, AND/OR, LINK + 3 spares — AND/OR/LINK are back on the UI).
- **"FM STRUCT shows SPLIT and can't be edited"**: the C3's soft takeover
  armed the switch cells against the current bucket, so a pot parked above
  the value never "crossed" it. Discrete cells (bucket bounds differ) are now
  LIVE in both PickupResync and PickupCellNote: the first movement sends the
  selection (continuous cells keep takeover).
Device: 805-817 us/block, voices 502-503, delay 216-224, stalls 0.

### Rack-lira8 L5 — documentation wrap-up (2026-11-15)

All rack-facing docs brought in line with the five-module firmware:
- **rack-rp2350/README.md**: title/intro = five modules; five envs +
  RACK_BOOT_MODULE 0-4; CC 32 0-4; a full **rack-lira8** section (what it is,
  module id 4 + standalone-only chain rule, 64-block burst contract, 805-817
  us measured, the report budget 1984 + `sens=`/`st=` tails, voice/pair/
  group architecture, the sensor notemap 36-43 + CC 55-62 + PLAY 64, the
  complete CC map incl. the combined delay mode 51 and 52 override, the 1 s
  HW delay cap + soft-knee storage, the P1/P2/P3 pot maps, live switch
  cells); report-format section updated with the lira8 budget/tails.
- **rack-uimcu/README.md**: a "Rack pages" section (module pages P1/P2/P3,
  the 5-module selector/overlay, switch labels, lira8's page/pot map),
  page-cycle gesture, pot behavior (P1/P2/P3 + the real SYS pot mapping),
  discrete cells LIVE (no takeover) with the FM-STRUCT bench bug, `page
  <0-10>`.
- **docs/ui-protocol.md**: §4 page ids corrected to the code (2-7 MOD
  pages incl. MOD-ARP, 8 SYS, 9 CLOUDS B) + page 10 P3; module id 0-4;
  5-module CC cache; SWITCH 0-4; preset blob 1071 B (v3).
- **docs/PORTING-HIGHLIGHTS.md**: new §12 — hand-port rules (verify against
  the .pd/dump, the -Os trap for header-only engines, fixed-point phases,
  settled-block fast paths, int16 soft-knee + block-held SELF grit, the
  hardware manual as the control-semantics authority) + the 805-817 us
  result.
- **docs/superpowers/plans/2026-11-15-rack-lira8.md**: L4/L5 status rows.
Not documented (out of scope): the browser rack_panel.html still has no
lira8 tab — the C3 is the lira8 control surface.

- **QUANTIZE restored on the UI** (user, 2026-11-15): it had been dropped as a
  Pd-patch extra, but the user wants it. P3 pot 14 = QUANTIZE, an ON/OFF cell
  (CC 53 now a bool127 discrete cell; C3 value text ON/OFF); P3 has 2 spares
  left. READMEs (rack + C3) updated.

### Rack-lira8 44.1 kHz switch + RAM/reverb feasibility (2026-11-15)

User call: run lira8 at **44.1 kHz** (the Pd abstraction's rlpf constants are
44.1k-authored: `os.triangle~` `/~ 44100` + `clip~ 0 22050`, so 44.1k also
matches the author's coefficients; the port adapts them from sr).
- module_lira8: kSampleRate 44100, kBudgetUs 1451 (rounded 64-sample period;
  was 1984 = the floored 32k block), delay carve (44100+4)*2*2 = 176,416 B of
  the 192 KiB arena (~90%), I2S bank 8x16 = 128 frames = 2.9 ms > the ~0.8 ms
  burst, kStallUs 900.
- Device: max Process 812-823 us / 1451 budget (56% load), voices 505,
  delay 218-231, LFO 39-40, master 42, i2s stalls 0.
- **RAM answer (user asked why so low)**: the linker's 512 KiB is statically
  partitioned for the worst case: 116 KiB hot-code window (chain A+B
  residency; the largest single image is rings 58 KB), 132 KiB object arena
  (Elements' ~119 KB state), 192 KiB buffer arena (chain elements>clouds =
  64+87+44 = 195 KB), ~42 KB data/bss, ~30 KB heap gate. Nothing is wasted by
  lira8; the only large reclaimable block is the 116 KB chain window
  (standalone needs 64 KB -> +52 KB buffers, chains retired).
- **Reverb feasibility**: the ctag MIVerb/MIVerb2 *wrappers* are GPL-3.0, but
  their `mifx/reverb.h` core is MIT (Emilie Gillet 2014) — the same MI reverb
  already vendored in elements/clouds, and the ctag wrapper allocates a 128 KB
  FLOAT buffer vs the MI 16-bit reverb's real need: elements 21,617 frames =
  43,254 B, clouds 16,375 = 32,770 B. At 44.1 kHz the 1 s delay (176,416) +
  elements reverb = 219,670 > 196,608 -> options: delay cap 0.8 s (141,136 +
  43,254 = 184,390 fits, chains keep working) OR shrink the chain window to
  64 KiB (+52 KB arena, 1 s + reverb = 219,670 fits, chains retired).
  Decision pending (user declined the immediate question).

### Rack T8 — unified SRAM pool + lira8→clouds chain (2026-11-15)

User: "why can't all memory be used with single module loaded... make enough
allocation for Lira8 + Reverb?" — answered WITHOUT a lira8 reverb: clouds-B
already has the MIT reverb (alias CC 48), so the chain is the better use of
the fix and costs no new DSP in lira8.

- **Root cause**: the old OBJECT (132 KiB) + BUFFER (192 KiB) arrays were two
  unions sized by DIFFERENT worst cases. With lira8 active: object 16,080 of
  135,168 (110 KB idle) while the buffer arena was 62,480 B short for
  lira8->clouds. Total was enough, the split double-booked.
- **T8 implementation**: one `g_rack_pool[331,776]` (the arrays were already
  physically adjacent: buffer @0x20003ee0 + object @0x20033ee0) + a
  deterministic bump allocator (`rack_mem_reset/alloc`, 8-aligned, loud OOM
  halt). Each adapter declares `state_size()` + `buf_bytes()` and takes
  `(obj_base, buf_base)`; `init()` resets+allocates for single-module paths;
  chain engage resets ONCE and places A state+buf then B state+buf; chain A
  re-init reuses the deterministic A addresses so clouds-B survives.
  Worst chain (rings+clouds) 331,520 of 331,776 (same 256 B slack as before).
- **lira8 as chain A (pair 4)**: `init_chain_a` constructs the engine at
  32,000 Hz (the chain grid; all rlpf/phasor constants derive from sr) and
  carves the 1 s int16 delay at 128,016 B vs 176,416 B standalone at 44.1k.
  `chain_render_a` calls `process_block_o3(.., 24)`; the engine already
  buffers its internal 64-sample block. Window: tc_lira8 14,680 fits the
  0xE400 A slot; A+B 71,744 of 118,784.
- **CC de-collision**: clouds-B owns 40-50 + 65/66 while chained; lira8's
  delay section was 41-49. Moved to 80-88 (f-b 80, and/or 81, link 82,
  time-1/2 83/84, feedback 85, mix 86, mod-1/2 87/88). C3 pages get cell CCs
  from TELEM (no C3-side CC table); preset version 3 -> 4 invalidates old
  lira8 CC values cleanly. C3 gained a 4th chain-pair slot (L>C) in the SYS
  selector / strip / kPairA; RP2350 pair index 4 + `kPresetPairA[5]`.
- **Device verification** (rack-lira8 env, flashed; C3 flashed):
  - `chain lira8` validated: window 14,680+57,064 of 118,784; pool 283,864
    of 331,776 (lira8 state 16,080 + buf 128,016, clouds state 8,692 + buf
    131,072); engaged 24 ms.
  - chain idle + with a sensor note: max Process 971-988 us (budget 750 —
    the lira8 64-sample burst; average fits), **i2s stalls +0 over 12 s**,
    meters live (`in=218 out=276` on note 36).
  - standalone lira8 still 821-847 us / 1451, stalls 0; all five modules
    switched through the pool (clouds 33 ms, elements 31, plaits 9) with no
    crash.
  - **C3 chain-route guard bug (2026-11-15, user-caught)**: after the C3 gained
  the 4th pair, selecting `L>C` did nothing — the chain stayed on the
  previously engaged pair (rings->clouds). Root cause: `RackLink::
  SendChainRoute` still validated `a > 3` (pre-lira8 module range) and
  SILENTLY returned false for lira8 id 4. Fixed to `a/b <= 4`; C3 rebuilt +
  flashed. Lesson: every module-id guard on the UI-link path needs the same
  0..RACK_MODULE_COUNT-1 range — pair selectors are not the only place ids
  travel.
- **Clouds PITCH mapping FIXED (2026-11-15, user: "clouds pitch is not
  working")**: the adapter copied the normalized panel value straight into
  `clouds::Parameters::pitch`, which is in SEMITONES (-48..+48, the hardware
  range; BraidingCloudsScarp clamps there). PITCH therefore moved by at most
  one semitone. Both the single-module and chain-B copies now use
  `(shared_pitch - 0.5f) * 96.0f` (0.5 = 0 st, ends = ±48 st). This bug was
  inherited from the standalone clouds-rp2350 port.
- **Non-blocking diagnostic TX (2026-11-15)**: the 2 s report and the preset
  autosave "PART burst sent" line did blocking USB-CDC `Serial.printf`s
  (~250-400 B into a 256 B TinyUSB FIFO; no console attached -> core-0
  stalls). Both now go through a 2 KiB `dbg_printf` ring drained by
  `dbg_pump()` with `Serial.availableForWrite()` (drops when nobody reads).
  Result: chain max Process peaks ~1500 -> ~960-1050 us, core-0 loop max
  typically 330-430 us (occasional 1.2 ms frames still include IRQ service),
  i2s stalls 0. The remaining >750 peaks are lira8's own 64-sample burst plus
  a short B ready-wait; no dropouts.
- **LIRA-8 voice code optimization (2026-11-15, user: "what can be optimized
  in voice... multiplied by 8")**: the voice stage was fully SRAM-resident
  (everything inlined into `render_internal_block`, 11 KB in tc_lira8), so
  the cost was pure math. The rlpf coefficient `a = exp_neg_range(-2*pi*fc/sr)`
  ran a 12-term exp polynomial PER SAMPLE per oscillator (512 exps per
  64-block) even though `fc = f/4` barely moves. New `mod == 0` path (the
  patch default): `prepare_coeff(tune)` computes `a` once per block;
  per-sample `dt` still carries the exact vibrato/mod-free frequency. A
  second `vib==0 && mod==0` path also hoists `dt`/pwm and skips the vibrato
  cos osc (phase advanced by `CosOsc::skip`). NOTE: the patch default vibrato
  is raw 1/127, not 0, so the mod==0 path is the one that fires in practice.
  MEASURED: voices **508 -> 360 us (-148 us, -29%)**, total Process
  **814-826 -> 663-680 us** of the 1451 budget (46% load), delay/lfo/master
  unchanged, stalls 0. Fidelity A/B vs the pre-change engine over all seven
  host scenarios: **-59 to -67 dB RMS, corr 1.0000** (project gate -50 dB),
  unit tests green. The change is an approximation only in the one-pole
  coefficient's vibrato term (<0.01% cutoff at default vib, <0.5% at full).
  Further ideas (not done): block-rate vibrato cos / sensor cos, fusing
  `mux_source` into the voice loop, `-ffast-math` A/B, dual-core voice-pair
  split (core 0 now has the chain-B headroom).
- **I2S DMA IRQ moved to the AUDIO core (2026-11-15, ROOT CAUSE of the
  persistent chain stall storms)**: `AudioBufferManager`'s `_addToList` /
  `_takeFromList` protect themselves with `noInterrupts()`, which is
  PER-CORE. The rack wrote from core 1 while `i2s.begin()` enabled the DMA
  IRQ on core 0, so the IRQ and the writer raced on `_empty`/`_filled`; the
  lists eventually corrupted — the stall storm appeared after minutes,
  survived DSP/parameter changes, and only a reload/I2S reconfigure cleared
  it. FIX: core 0 disables `DMA_IRQ_0` immediately after every
  `i2s.begin()`; core 1 enables it in `setup1()` and masks it across every
  runtime reconfig handshake (swap / engage / disengage). 60 s soak with
  note on/off + clouds feedback/reverb CC changes (autosave 1,071 B PART
  bursts landing live): **i2s stalls 0 (+0) throughout**, `in` up to 439
  (0.44 RMS), B cost up to 443 us, core-0 loop up to 1,369 us, underruns 0.
  This supersedes the earlier UI-TX / LED / C3-side theories — those were
  real secondary contributors, but this is the one that corrupted the DMA
  ring until reload.
- **Dual-core chain pipeline (2026-11-15, user directive "optimize for
  dualcore")**: chain A render + I2S write stay on core 1; clouds B runs on
  core 0 ONE 24-sample block behind through two 4-slot SPSC float rings
  (`g_chain_a_ring` core1->core0, `g_chain_b_ring` core0->core1, volatile
  head/tail + __dmb). Per-block cost is A + handoff instead of A+B.
  Transitions (mute ramp, window copy, init, A re-init) set `pipe_run=false`,
  wait for the `chain_b_worker_busy` handshake, and run the SYNC render on
  core 1; normal play re-enables the pipeline after the ramp-up. Measured
  full-wet lira8->clouds: B cost **bm=148-222 us**, core-0 loop
  **c0=469-1185 us**, underruns **u=0**, max Process 949-1281 us (A burst +
  core-0 jitter; B is no longer in that sum), **i2s stalls 0 over 12 s**.
  A 2-block pipeline depth was tried and REVERTED: producer/consumer are 1:1,
  so the ring never reached 2 and every block waited out its 3 ms timeout
  (max 3.8 ms). One block of lag = 0.75 ms extra latency is the operating
  point. The 2 s report now extends `mod=<a>><b>` with `bm=/c0=/u=`
  diagnostics.
- **Status LED off the audio-critical path (2026-11-15)**: `showColor` used
  `delay(1)` after every WS2812 frame — 1 ms of core-0 task time every 20 ms
  (5%) even when the LED was dark/idle. T8 removes the wait (the 20 ms poll
  and boot/blink delays already exceed the >50 us reset gap), skips unchanged
  colors, and adds `led on|off` (default on; off = one black frame then
  `Poll` no-ops). Measured with `led off`: full-wet lira8->clouds chain, note
  held, 12 s -> **i2s stalls 0 (+0)**, max 946-984 us. If the long-session
  bitcrush survives this, the next step is the dual-core chain pipeline
  (A+B currently sequential on core 1).
- **UI-link TX ring (2026-11-15, T8 grit ROOT CAUSE + FIX)**: the chain
  "grit at high dry/wet" was NOT clouds' DSP or clipping. `preset_autosave_poll`
  pushes a **1,071 B PART burst 3 s after ANY CC/pot change** (the user turning
  dry/wet), and `ui_link_send_frame` did a blocking `Serial1.write`: ~10 ms of
  core-0 spin. The I2S DMA IRQ lives on core 0 and must service the 2-channel
  ping-pong chain every ~500 us, so the burst caused audio dropouts (grit) and
  the `i2s stalls` storm; a fresh chain engage (I2S reconfigure) or reload
  cleared the stale state. User symptoms all fit: grit after a pot move,
  reverb-off no help, single-mode clean, "like DMA/buffer issue".
  FIX: `ui_link_send_frame` now enqueues into a 4 KiB TX ring (drop+count on
  overflow) and `ui_link_tx_pump()` (called from `ui_link_pump` every core-0
  loop) writes only what `Serial1.availableForWrite()` can take immediately.
  Verified on device: `chain lira8`, full-wet note, a CC marks dirty, autosave
  `PART burst sent (1071 B)` lands DURING the chain — **i2s stalls 0 (+0)**
  across all 2 s reports. `ui` prints `dropped=`.
- TRANSIENT: right after the C3 reflash, the first chain runs accumulated
    i2s stalls (~435/2 s) for both lira8 and rings pairs AND the user heard
    dirty audio with every chain pair (even clouds dry/wet 0); a fresh chain
    engage after the C3 settled ran +0. 2026-11-15 follow-up: a C3-side reload
    cleared the dirt completely while the RP2350's `i2s_stall_count` stayed
    at 250,783 (so the RP2350 did NOT reboot) — the culprit is the C3/UI-link
    side (stale post-reflash state or a chain-route/preset reconciliation
    loop whose blocking 921600 UART bursts delay the core-0 I2S DMA IRQ), not
    the unified pool or the chain render. If it recurs: capture the console
    for repeated `chain ... engaged` / `preset: PART burst sent` lines and
    consider a TX ring for the UI link (core 0 must never block ~9 ms on a
    1,071 B PART burst while audio runs). Single-module audio was clean
    throughout; the user can set lira8->clouds from the C3 SYS `L>C` selector.
- **LEDGER CORRECTION**: the Elements/rings reverb allocation requirement is
  **64 KiB**, not the 43,254 B "delay-line footprint" recorded above.
  `elements::FxEngine<32768>::Init` does `std::fill(buffer_[0], buffer_[size])`
  over the full declared 32,768-token template, i.e. 65,536 B — allocating
  43.3 KB would let Init corrupt the next 21 KB. (Clouds' reverb is 32,768 B
  as recorded.) A standalone lira8 + Elements reverb would need 16,080 +
  176,416 + 65,536 = 258,032 B of the 331,776 pool, so it would fit — but the
  chain route is the shipped design.
- Docs: rack README (pool, chain table, lira8 CC map), ui-protocol (chain
  pair 1-4), C3 README (4 pairs), memmap_rack.ld (lira8 A-slot ASSERT +
  heap notes).
- **VOICE STAGE: block-rate vibrato cos + fused prm.source mux (2026-11-15)**:
  two micro-optimizations in `src/lira8/` (host harness + on-device verified).
  1) The vibrato osc~ (2.7-3.3 Hz, per-sample cos table lookup x 4 pairs) now
  evaluates the cosine at the block endpoints and lerps in between
  (`CosOsc::begin_block/next_lerp`); the phase advances EXACTLY (integer add),
  so blocks always start on the true cosine. 2) `prm.source`'s 64-sample mux
  buffer pass is gone: `ModSource::at(i)` is evaluated inline only in the
  voice's mod != 0 loops, so the patch-default mod=0 drone state does zero mux
  work. HOST A/B (old vs new render_golden, all 14 scenarios): worst delta
  **-63.9 dB RMS / corr 1.0000** (single-pair), -69.6 pairfm, -72..-82 the
  others; **bit-identical wherever vibrato=0** (onevoice/pitch0/pitch127/
  onecyc). Two NEW isolation scenarios (mod=64, vibrato=0, switch+total-fb
  flipped and source-12/34=2 LFO-CV) are **bit-identical old vs new**, proving
  the mux fusion is exact — the -64 dB delta is entirely the vibrato chord
  interpolation, which integrates into oscillator phase (same class as the
  accepted rlpf-hoist optimization, -59..-67 dB). ON DEVICE (same bench,
  back-to-back old/new builds, chain off, settled state): standalone voices
  **361 -> 301-307 us**, max Process **665-680 -> 590-632/1451**; chained
  lira8>clouds with a note held 12 s: settle max **758-769 us**, bm=153-156,
  c0=355-379, **u=0**, **i2s stalls 0 (+0)**. Fidelity ceiling deliberately
  kept at the linear interp (quadratic would cost an extra lookup/mul-add per
  block for no audible gain).
- **FLASH TRAP**: the documented 1200-baud touch silently no-ops without
  `hupcl` — stty closes with DTR still asserted, so the firmware never sees
  the 1200-baud DTR drop and stays in the app (device number unchanged, no
  BOOTSEL drive). `stty -F "$RP" 1200 raw -echo hupcl` reboots every time;
  AGENTS.md now says so. (The pyserial fallback's `dtr=False` can raise
  Errno 71 — verify re-enumeration to "RP2350 Boot" before trusting it.)
- **CHAIN PIPELINE v2: depth-aware lag (2026-11-15)** — the ec48013 pipeline
  kept a 1:1 handshake: core 1 pushed A(n), then waited for the B ring, and in
  steady state the block it got was B(A(n)) — so it waited out the ENTIRE B
  render every block. That is why chain max Process stayed 755-769 us (A + B
  wait) despite the two cores. Fix: every block now carries a SEQUENCE number;
  core 1 consumes **B(n - kChainPipeLag)** with `kChainPipeLag = 2`, never
  "whatever is newest", and `chain_pipe_start()` PRIMES the B ring with two
  sync-rendered blocks (the first pipelined block finds its B already
  waiting). A ring DEPTH alone cannot do this — the producer is paced by the
  audio clock, so a 1:1 producer/consumer ring never fills; the consumer must
  ask for an OLD block. On a real core-0 stall the block is silenced and NOT
  consumed (u++), so the stream resumes in order (no skip/repeat).
  **Measured**: idle chained **624-657 us** (was 755-769), note held
  **603-641 us** (was 758-769), bm=157-201, c0=302-761, **u=0, stalls 0**
  (14 s runs). Five engage/off cycles + rings/elements A slots: all engage
  (rings 35 ms, elements 48 ms, lira8 28-29 ms — priming adds the 2 blocks),
  **u=0, stalls 0**. The remaining >750 lines are CC-application transients
  (`st=29-89`, voices slow path 466-670), not the pipeline. **Cost**: +1.5 ms
  chain latency (two 750 us blocks); rack README updated. A stagnation note:
  the lag is a fixed 2 blocks — if core-0 stalls longer than 1.5 ms recur,
  raise kChainPipeLag (latency = lag x 750 us).
- **CHAIN-B LEVEL: dry passthrough is now unity (2026-11-15; user: "adding
  clouds to the chain its output is quieter than module alone, even at
  dry/wet 0")** — not the pipeline: Clouds' float output stage is
  `SoftLimit(l * 0.5f)` (MI's `SoftConvert` int16 conversion gain) and the
  equal-power crossfade puts the dry endpoint at fade_out(0) = 0.7071, so a
  fully-dry chain passed only **0.5 x 0.7071 = 0.3536 (-9.03 dB)** of the A
  slot. Verified against the MI original (`SoftConvert(x) =
  Clip16(SoftLimit(x * 0.5f) * 32768)`), so the 0.5 is faithful — the RACK
  chain just needs its own calibration. Fix: `mod_clouds::chain_render_b`
  scales the processed output by `kChainBOutputGain = 2.8284272`
  (= 1/(0.5*0.7071)) AFTER `Process()`, so dry_wet 0 is unity and the wet
  endpoint carries MI's natural post_gain 1.2. No gain enters the DSP loops
  (limiter/feedback/reverb keep MI's internal scale). Standalone clouds
  keeps the raw MI output scale (its own accepted level). **On device:
  CC45=0 -> in=347, out=345 (unity).** Caveat: at full wet with every
  clouds parameter at max the summed output can exceed 0 dBFS after the
  trim (out meter ~1.36 RMS) and `Clip16` hard-clips; a soft knee on the
  chain output is the follow-up if it ever bothers.
- **BENCH MIDI GOTCHA**: `aseqsend -p 24:0 "B0 2D 00"` prints
  "Sent : 3 bytes" but the event NEVER reaches the RP2350 (`preset status`
  seen-CC count unchanged). `aplaymidi -p 24:0 file.mid` with a generated
  single-event SMF delivers every time. Use aplaymidi for bench CCs.
- **CHAINED CLOUDS-B MONO QUALITY CRASH — ROOT CAUSE FOUND (2026-11-16)**:
  user: "crashed in rings-clouds chain when switching page from rings to
  clouds, 1-2 s of audio noise burst". Chain of evidence: (1) page switch →
  C3 sends its QUALITY pot CC 50 = 3; (2) with host-only CCs, `CC 50 = 1/3`
  (mono) kills the audio core while `0/2` (stereo) is fine; single-module
  clouds mono is fine; (3) reports show core 1 blocked in `chain_write_b`'s
  I2S write (`w1=20`), `bm=0`, then core 0 dies and the watchdog reboots.
  (4) **ADDRESS-SANITIZER HOST HARness proved the write**: compile the
  clouds lib on the host + `-fsanitize=address`, Init(87,040/44,032), mono
  quality → `heap-buffer-overflow WRITE of size 2 at 0 bytes after the
  87040-byte region` in `clouds::FxEngine<4096>::Clear()` ←
  `PitchShifter::Init()` ← `GranularProcessor::Prepare()` (line 456).
  `Prepare()` allocates only **1,560 B** for the correlator
  (`(kMaxWSOLASize/32)+2)*3*4`) then passes that pointer to
  `pitch_shifter_.Init()`, whose FxEngine<4096> clears **8,192 B** → a
  **6,632 B overshoot of the workspace**. In the chained B slot the mono
  layout puts the workspace at the END of the carve (small buffer), so the
  overshoot leaves the pool: harmless by luck for elements/lira8/plaits
  (their pool slack absorbs it) but for rings (768 B slack) it lands past
  `g_rack_pool` (0x200038f0..0x200548f0) in libc/heap/I2S state → the I2S
  DMA stops and core 1 blocks. This is why the user saw elements/lira8
  chains work and plaits/rings crash (same code, different pool layout).
  FIX NOW: chain-B quality clamp to STEREO modes only (0/2) in the adapter
  (`chain_b` check) and in the chain CC dispatcher (with a log line).
  Verified on device: CC 50 = 1/3 rejected, audio stays alive; 0/2 apply;
  single-module clouds mono unchanged. REAL fix (TODO): allocate
  `max(correlator_bytes, kMaxWSOLASize * 2)` = 8,192 B for the
  correlator/pitch-shifter region and enlarge the chained workspace to
  >= 49,152 B (needs a carve/pool budget change — the 131,072 B chained
  clouds carve cannot satisfy both mono and stereo layouts at once).
- **HANG DIAGNOSTICS + HARDENING (2026-11-16)**: the crash looked like a
  frozen board: 2 s core-0 **hardware watchdog** (`watchdog_enable`, boot
  8 s) now reboots it, with the last core-0 stage in a watchdog **scratch**
  register (SRAM does not survive a watchdog reset; scratch[0] does) shown
  as `hang=N`; core 1's stage in scratch[1] as `w1=` (10-14 chain render,
  20/21 I2S write, 40/41 clouds-B) plus the I2S DMA channel busy/count
  `d0=/d1=` in every 2 s report. `reboot` and `dma` console commands added.
  ALL event-driven core-0 diagnostics now go through the non-blocking
  `dbg_printf` ring (UI-link RX handler, preset apply/chunk paths, mod
  engine UI sets, rings clamp, deferred swap/chain reports, USB host
  callbacks, boot prints) — a blocking CDC write with no console reader was
  itself a hang class (the boot prints could hang a console-less boot).
  The DIN and UI-link UARTs now use **direct UART drivers** with hard-capped
  drains (64 B/IRQ or per pass), replacing the Arduino core's unbounded
  `while (uart_is_readable())` RX handler (a break on a floating GP5 hung
  core 0).
- **LED + PIEZO OFF (user request, 2026-11-16)**: `status_led::kEnabled =
  false` (no PIO program, no SM claim; PIO2 is still un-reset for I2S) and
  `rack_piezo::kEnabled = false` (no free-running ADC, no DMA channel — the
  piezo is not wired yet; its DMA ring was a hang suspect). The button press
  report line is removed (user: "we dont need to print button presses
  anymore").
- **PROPER FIX for the chained clouds-B workspace overrun (2026-11-16,
  `clouds-z-pitchshifter-workspace.patch` + split change)**: the clouds lib
  now allocates `max(correlator 1,560 B, kMaxWSOLASize*2 = 8,192 B)` for the
  correlator/pitch-shifter region (the pitch shifter's FxEngine<4096> clears
  8,192 B through that pointer). The FX workspace need is therefore
  **49,152 B** (8,192 diffuser + 32,768 reverb + 8,192 pitch shifter). New
  chained split **90,496 large / 41,216 small** (131,712 total): stereo
  channels 41,216 B (0.64 s/ch @ 32 kHz) and stereo workspace 49,280 B
  (128 B margin); the rings pair now uses 331,656 of 331,776 B. Mono quality
  remains impossible for the rings pair by arithmetic
  (2*small + 49,152 <= 131,840 can't hold with small >= 49,152), so chained
  clouds-B COERCES mono (1/3) to stereo — adapter clamp + a last-line guard
  in `chain_render_b`. ASan host proof: clean for stereo 16/8-bit on the new
  split, clean for ALL four qualities + looping on the standalone split.
  On device: rings>clouds engage 30 ms, all four CC 50 values alive,
  stalls 0.
- **CLEANUP after the clouds investigation (2026-11-16)**: removed the
  hunt-specific diagnostics — watchdog breadcrumbs (`crumb`/`crumb1`,
  scratch-register stage numbers), the `w1=/d0=/d1=/wd=/hang=` tail in the
  2 s report, the `dma` console dump, and the direct UART0/UART1 drivers.
  The UART-driver rewrites were a MISDIAGNOSIS: the "hangs" at stages 8/9/21
  were core 0 dying wherever it happened to be once the clouds mono write
  corrupted the memory after the pool; with the clouds patch in place the
  stock core Serial1/Serial2 paths (weeks of prior use) are correct and
  restored. KEPT (genuine): the clouds pitch-shifter patch + split + mono
  coercion, non-blocking `dbg_printf` for every event-driven diagnostic
  (boot prints included — a console-less boot could block on the CDC FIFO),
  the 2 s core-0 hardware watchdog (restarts a frozen board), the `reboot`
  console command, the HardFault record/reset handler, and the LED/piezo
  disable + button-print removal requested by the user.

- **T8b ARP-driven voice pitch + sensor retrigger (2026-11-16, CODE COMPLETE,
  bench acceptance pending)**: user request — sensor notes shifted one octave
  down (36–43 → **24–31, C1–G1**), **MIDI velocity = press peak**, and the eight
  voice pitches driven by the MOD-ARP. Design locked with the user:
  - **CC 54 (P3 p15 "ARP HARM")**: all 8 voices retuned from the routed ARP's
    harmony `(root, scale_st[12], nsteps, chord_deg, chord_nt)`. Low group
    (voices 1–4 / pairs 0-1 / P1) = perfect bass stack `{−12,−5,0,+7}` from the
    chord root; high group (voices 5–8 / pairs 2-3 / P2) = scale chord
    `{third+12, fifth+12, seventh+12 | third+24, +24}`. Root folded to C3–B3.
    W7 bar-line chord drift re-roots both groups; NEO's triad flip moves the
    upper third. Harmony source = first ROUTED ARP, else arp1 (works with no
    ARP routed; arp1 defaults C3 Aeolian).
  - **ARP → PITCH 1234 / PITCH 5678 (CC 25/26) destinations** (the C3's normal
    assign gesture): the mapped ARP's **exact current step note** becomes that
    group's root, same voicing law. Always active regardless of CC 54;
    precedence mapped-ARP > harmony > knobs. No gate/trigger side effects.
  - **Voice tuning**: `VoicePair::set_note_hz()` + a per-oscillator override
    mask; bypasses the ×1.99 PITCH multiplier (default 66 = 74 cents sharp
    would detune the harmony) and quantize. `refresh_tune()` skips overridden
    oscillators; clearing re-applies the knob law, so TUNE CCs record normally
    while ARP-driven and restore on release.
  - **MIDI hijack bypass (required)**: with ARP root modes 0/1
    (`mod_arp_wants_midi_notes`) every incoming note was hijacked for root
    transposition → pads dead + pad notes re-rooting the ARP. When the lira8
    ARP-pitch feature is active, notes 24–31 bypass the hijack (adapter only),
    all other notes still transpose the root; symmetric on note-off
    (`rack_lira8_arp_pitch_on` / `rack_lira8_sensor_note` in midi.h →
    main.cpp hooks).
  - **Velocity**: `SensorEnv::set(on, level)`, level = vel/127, gate stays
    `min(e²+hold,1)` (hardware touch curve, velocity also drives the cos
    morph); CC 55–62 latches and PLAY keep level 1.0; same-level re-press is a
    no-op (no attack restart — the patch's line~ law).
  - **Snapshot plumbing**: new pure header `src/lira8/lira_harmony.h`
    (`GroupCtx`/`Snapshot`/`group_offsets`/`fold_root`/`note_hz`, no MCU
    headers). `mod_engine_process_block` publishes `g_lira_harmony` AFTER the
    source pass and BEFORE the module render (same core, `seq` bumps only on
    change). Published only when the active module is lira8 (chain A included).
  - **UI**: `kUiPageLira8P3` p15 = `{54, "ARP HARM"}`; C3
    `p1p2DiscreteCount` lira8 case adds CC 54 (ON/OFF, wireBool). Presets
    capture CC 54 automatically (`cc_cache[module][128]` + replay).
  - **Verification (this pass)**: new `test/test_harmony.cc` (major/Aeolian/
    degree-5/pentatonic voicings, root folding, full Cmaj7 map C2 G2 C3 G3 |
    E4 G4 B4 C5, mtof law) — `make test` all green; `pio run -d rack-rp2350
    -e rack-lira8` SUCCESS (RAM 72.0%, flash 37.7%); C3 `idf.py build` SUCCESS.
    NOT flashed/tested on hardware yet — user bench acceptance required.
  - **Bench checks for the user**: (1) pad notes now 24–31; velocity changes
    loudness/timbre; (2) CC 54 ON with arp1 unrouted = C3 Aeolian chord on the
    pads; change the ARP scale/root → chord follows; W7/NEO bar line → voicing
    moves; (3) assign arp1 → PITCH 1234: voices 1–4 walk the ARP's step notes
    (bass stack on each); arp2 → PITCH 5678 for the upper group; (4) with the
    ARP on MIDI-follow (root mode 0/1) pads still trigger.

- **T8c clouds modulation mapping fix (2026-11-16, flashed + console-verified)**:
  user report "clouds parameters can't be mapped with button pressed — list is
  empty". TWO real causes, both fixed:
  1) **Chain-B page (9) was not assignable**: the C3's assign sweep was gated
     on `RackUI::GetPage() <= 1`, so holding the encoder + turning a pot on
     the CLOUDS B page did nothing (the `>MOD<` prompt appeared, no source
     ever showed). The RP2350 side also had no B-destination binding — the
     README documented it as future work.
     Fix: `mod_engine.h` now treats the clouds-B band as valid destinations
     while a chain is engaged — CC 40-48 (grain alias bank) continuous and
     65/66 (freeze latch / trig pulse) triggers — routed through the SAME
     `rack_chain_control_change` the MIDI intercept uses (`mod_write_cc`,
     `mod_fire_trigger`, `mod_trigger_kind`, `mod_is_continuous`,
     `mod_set_dest` validation + cache restore from the CLOUDS cache).
     `g_mod_chain_grid` is the chain-active flag in the engine. Mode/quality
     (49/50) remain panel selectors, NOT mod destinations (inert by design).
     C3: the assign gesture accepts page 9, and `s_potModSrc` gained a third
     bank (2 = CLOUDS B) so P1/P2 bookkeeping is untouched
     (`pageBank()` helper). Page-9 TELEM now carries routed/inert/source
     flags (chain-B dim bit stripped — it IS the B page) so mapped B cells
     show yellow/ride like A cells.
  2) **Single-module clouds alias bank was not in the destination table**:
     the clouds P1 cells (C3 + web panel) send CC 40-48, but
     `kModContinuousCcs` only listed native 14-22, so every C3 assignment
     ended INERT (`mod_set_dest` warning; cell blinked). Fix: the table now
     lists 14-22 + 40-48 (18 entries) and `mod_set_cc_float` /
     `control_change` translate the alias bank onto the native params (the
     MIDI path already translated; direct calls — unassign restore, preset
     replay bypass — now work too). Panel `MOD_DESTS.clouds.cont` switched
     to the alias bank (labels resolve; the firmware still accepts native
     14-22 for console/legacy routing).
  - Console verification (board, this session): `chain lira8` then
    `mod arp1 dest 45` -> "mod: arp1 -> clouds CC 45 (slewed gate CV)" and
    `mod?` shows `dest=45(chain-B clouds on lira8)`; clear works; chain off
    clean. `switch clouds` then `mod lfo1 dest 45` -> "continuous" ack (was
    INERT before). Chain soak 14 s: 629-640 us peaks, stalls 0 (the 1.2 ms
    first-window peak is the known engage transient). RP2350 + C3 reflashed
    with the fix. Hardware UI check (C3 page-9 assign gesture) still needs
    the user's hands.

- **T8d MOD-ASSIGN gesture rewrite (2026-11-16, C3 built + flashed)**: user
  report — mapping was glitchy: DENSITY->LFO worked, POSITION would not map,
  sometimes another pot got the source. ROOT CAUSE (not plugin-related): the
  old gesture derived the pot ANGLE from the PRESSED ADC reading. The encoder
  push shares the mux SIG pad (GPIO3) through the 4.7k series resistor, so
  pressing turns the reading into a divider between the pot's wiper
  impedance and 4.7k — position-dependent and, for a 10k pot, NON-MONOTONIC
  near the top (it peaks ~70% and falls again), i.e. one pressed value maps
  to two angles. The old `pressedPosFromRaw()` knot curve was wrong for most
  pot positions, the 5-step lock threshold was below the noise floor at low
  positions, and the "largest pressed delta wins" rule let a non-turned pot
  lock. Symptoms match exactly (low-position POSITION 35% failed, high
  DENSITY 75% worked, ghost pots).
  FIX — LATCHED assign mode, unpressed readings only:
  - `PUSH_HOLD` (350 ms) arms `s_assignArmed` and the mode SURVIVES the
    release (that release is what used to cancel it).
  - Turn a pot -> the scan task locks that cell as the target from CLEAN raw
    deltas (~4 counts; winner must lead the runner-up by ~2 counts; ~8
    counts to retarget). Strip shows the cell label + source
    (`>POSITION OFF<`). All sends stay muted while armed.
  - ROTATE the encoder -> choose the source (OFF, LFO1-4, EU1-2, GATE1-2,
    ADS1-2, ARP1-2).
  - SHORT PUSH -> confirm (RackUI stores `TakePendingAssign`, the pot task
    applies it via `applyModAssign`); no target = cancel. 15 s timeout and
    any page change abort the mode.
  - On exit the moved pots are re-seeded and `PickupResync()` re-arms soft
    takeover so no parameter jumps.
  Pressed readings are now used ONLY for the button state (which mutes the
  pots). `pressedPosFromRaw` deleted with a physics note. Files:
  `main/RackPots.cpp` (+hpp doc), `ui/RackUI.cpp` (+hpp), `rack-uimcu/README.md`
  gesture table (also corrected: HOLD = 350 ms, not 600).
  Console/link path unchanged (T8c cloud destinations still verified).
  Hardware gesture test = user's hands; old flow is unrecoverable by design.

## T9 — Elements × Warps internal stage (2026-11-17)

Owner decisions (from the ctag-tbd plan, re-derived for this architecture):
modulator = raw exciter (`raw_buffer_`); **vocoder excluded** (memory law —
see findings); carrier = **main only** (rack default space=0 puts the raw
exciter on aux); C3 UI via the RP2350 TELEM tables (no rack-uimcu change).
Plan: `docs/superpowers/plans/2026-11-17-elements-warps.md`.

| Task | Status | Committed | Notes |
|---|---|---|---|
| T9a Vendor warps DSP + sync step + rack-port patch + host tests | done (build-verified) | 030a719 | pristine `WARPS/` from ctag-tbd eurorack → `lib/elements/warps/` via a new sync-script step + `tools/patches/warps/rack-port.patch`; deltas: vocoder + easter-egg strip, debug_pin include removal, `kMaxBlockSize` 96→16, `warps_lut_xfade_in/out` rename (clouds collision), `WARPS_RAM_CODE` → `.tc_elements.warps_*` markers; host test `test_warps.cc` (RED watched: sizeof 73,192 B / kMaxBlockSize 96 / egg present) |
| T9b Reverb engine sized to its true span | done (build-verified) | 342fd43 | `FxEngine<32768>` → `<21632>` under RACK_RP2350_TC (Dattorro Reserve sum = 21,617 frames, ledger-documented 43,254 B); elements carve 65,536 → 43,264 B frees the warps room; canary host test `test_reverb.cc` RED (upstream Init zeroes the guard zone) → GREEN |
| T9c Warps stage inside Part + CCs 36-40 | done (build-verified) | 7619ca5 | stage in `part.inc` after metering/panic, before `reverb_.Process`; carrier = main, modulator = `raw_buffer_`, mono into main (aux untouched); algorithm clamped 0.625 (comparator; vocoder zone excluded); `Part::Init(reverb_buffer, warps*)`; Modulator pool-placed behind the reverb carve (`static_assert` ≤ 4096 — actual **2,312 B**); CCs 36-40 in `control_change` + `mod_set_cc_float`, kModContinuousCcs 17→22; stage host tests: bypass bit-exact vs nullptr-warp Part, aux untouched, alg 1.0 ≡ 0.625, silent-mod mix 0.5 = exact halving |
| T9d UI surfaces (P2 cells + panel + README) | done (build-verified) | 9df714f | `kUiPageElementsP2` +5 cells (`W ALGO/TIMBRE/MOD/CAR/MIX`) — C3 renders pages from TELEM, **no rack-uimcu change, no C3 reflash**; `rack_panel.html` knobs/ccMap/MOD_DESTS; README "Elements × Warps stage (T9)" section |
| T9e Flash + per-algorithm bench + user acceptance + push | in progress (clock work done; bench pending) | 9dc98ab + 0457273 + (264 clock commit) | bench gates re-anchored — see the clock findings below; per-algorithm warps delta table still pending user hands |

Findings (T9):

### Rack T9-clock — 240 floor truth, 280 rejected, 264 adopted (2026-11-17)

- **T9 (warps) is EXONERATED from the budget story**: a whole-build bisect
  (flashing pre-T9 44a3024) measured the SAME idle — pre-T9 elements idles at
  **504-518 µs of the 496 budget** on device (stalls 0, the 2.9 ms I2S bank
  absorbs it — the accepted operating condition). The plan's ≤ 460 µs gate was
  anchored on the ledger's old 101-125 µs numbers (strings-model era) — wrong
  baseline, mine. The warps stage DELTA measured on hardware at 240:
  **+30-135 µs per algorithm** (OFF ~640 → XFADE 734, DIODE 808, DIGI 645,
  XOR 745, COMP 712, drone baseline 673). Gate re-anchored to the delta.
- **The idle floor is WAIT-STATE bound, not compute bound**: ~510-540 µs
  wandering to 600-700 at BOTH 240 and 264 MHz — raising the clock does not
  move it. The unmarked (XIP) fraction of the hot path dominates; further
  gains need tc-marking promotions (tc_elements has 7,544 B A-slot slack),
  not clocks.
- **280 MHz is NOT stable on this board and was rejected on evidence**:
  - The flash (QMI) timing is set by the RP2350 ROM (measured live:
    **CLKDIV 3, RXDELAY 2** — boot2/PICO_FLASH_SPI_CLKDIV does NOT apply on
    RP2350, pico-sdk issue #1903) and set_sys_clock_khz does NOT re-scale it,
    so flash SCK = clk_sys/3: 80 MHz at 240, **93 MHz at 280** — the data
    sample point goes marginal and XIP reads corrupt INTERMITTENTLY.
  - Observed at 280 across four builds (incl. one with a constructor-time
    QMI pin to the proven 4/2 combo): intermittent boot (sometimes reached
    link+TELEM, sometimes died in core init), **structurally-valid TELEM
    frames with corrupt CRC** (C3 log), CDC dark in every variant, and a
    bootloop triggered by the C3's stale autosave applying a lira8 swap whose
    window copy hit the corruption. 264 MHz shows none of this.
  - Adopted: **264 MHz = 12×22** (PIO-USB-safe multiple), SCK 88 MHz, ROM
    timing, soaked stable (0 stalls). 280 was the T7 note's intent but its
    "flash SCK w/in legal bounds" concern was real — at 280 the margin was
    already gone before any divider fix could land. 280 needs a real QMI
    retune (RXDELAY ≥ 3 at 93 SCK) to revisit; not attempted further.
- **New console tools (T9)**: `cc <0-127> <0-127>` injects a MIDI CC through
  the REAL midiControlChange funnel (TELEM cache + autosave-dirty + registry
  dispatch) — scripted benching without hardware knobs; `qmi` /
  `qmi set <clkdiv> <rxdelay>` reads/writes the live QMI M0 timing (runtime
  write validated at 240: set → readback → audio alive).
- **UI fix along the way**: elements P2 showed 3 of 8 cells because
  `kUiPages` hardcoded the cell counts — the registry now DERIVES counts from
  its tables (`UI_PAGE_CELL_COUNT`); the C3 renders TELEM as told, so the
  warps cells appeared only after that fix (user-confirmed 8/8).
- **Panel**: W ALGO is a labeled step selector (OFF/XFADE/FOLD/DIODE/DIGI/
  XOR/COMP at bytes 0/1/16/32/48/64/79 — the pure algorithms sit at p×8 =
  0..5; byte 0 is the bypass). C3 pots stay continuous (TELEM has no steps
  metadata; protocol extension deferred).
- **Process lesson recorded in agent memory**: never flash a known-bad build
  to the board without a recovery in hand (the 280 bootloop incident);
  BOOTSEL mass-storage copy is the race-free flash path; COM22 (C3) is the
  liveness channel when RP2350 USB CDC is down.
- **Warps per-algorithm delta at 264 MHz (scripted, `cc`-driven, drone + mix 1)**:
  OFF 601-628 | XFADE 702 | FOLD 689-710 | DIODE 712 | DIGI 683-695 |
  XOR 693 | COMP 677-692 — **delta +49-111 µs/algorithm**, stalls 0
  throughout. The OFF floor (~600-630 with drone) is the pre-T9 wait-state
  condition; the stage itself is a modest constant add.
- **Host test harness**: `test_warps.cc` (rack deltas: sizeof 73,192 →
  2,312 B, kMaxBlockSize 16, egg stripped), `test_reverb.cc` (canary: Init
  fills exactly the 21,632-frame carve), `test_warps_stage.cc` (bypass
  bit-exact, aux untouched, alg clamp, exact halving) — all RED→GREEN
  watched; the full lira8 suite still green (one g++13.3 fix in test_wav_io).
- **Crash forensics without a debugger (2026-11-17)**: the board's docs
  confirm the BRIDLE mini_usb_rp2350 exposes **no SWD** ("debugging is not
  possible on this board"; UF2 only) — so the crash record is the primary
  tool. Extended: record v2 in `.uninitialized_data` with `boot_count`,
  sticky `fault_streak`, `reset_reason` (0 cold / 1 pre-setup death / 2 fault
  / 3 watchdog-in-loop via the watchdog scratch marker) + **sticky SAFE MODE**
  (fault_streak >= 2 or boot_count >= 3 -> setup drops clk_sys to 240 BEFORE
  the window copy, so one boot is ALWAYS console-readable; `crash clear`
  re-arms). `crash` prints the counters/record.
- **Runtime overclock bench (2026-11-17)**: `oc <khz 24000-400000>
  [clkdiv] [rxdelay]` - pins the QMI timing FIRST (the SDK's PSRAM
  precedent), raises VREG to 1.20 V above 280 MHz, switches the PLL live. The
  integrity oracle re-reads the boot module's FLASH image through XIP and
  compares against the boot-time window-copy FNV (`g_window_src_sum`) - a
  pure-read test that never touches the SRAM window core 1 executes from.
  Validated: 264 -> 270000 live, FNV OK, readback correct, console alive;
  Serial1/I2S dividers stay stale until `reboot` (audio off-rate, UI link
  dead - by design). A 270 soak ended in a reset with `fault_streak=0` and
  `reason=1` (no scratch marker = POR-class, not watchdog/fault; the same
  USB/CDC flap family as healthy clocks - ambiguous, NOT a 270 verdict).
  Next bench: sweep 252/264/270 with periodic FNV re-verifies + counters;
  the 280-300 matrix (RXDELAY 2/3/4 x CLKDIV 3/4) is now console-cheap.
- **The 270 "reason=1" was a BROKEN signal (2026-11-17, analysis)**: the code
  called `watchdog_enable(8000,false)` as setup's FIRST action; the SDK writes
  `WATCHDOG_NON_REBOOT_MAGIC` (0x6ab73121) into `scratch[4]`, so the boot-done
  marker read immediately after was ALWAYS clobbered -> `reason=1` on every
  non-fault boot, and `reason=3` (watchdog-in-loop) was unreachable dead code.
  The "POR-class" reading was therefore unfounded: **there is no 270 verdict
  yet**. FIX (uncommitted, build-green on rack-lira8): capture
  `watchdog_hw->reason` + `scratch[4]` BEFORE `watchdog_enable()`; add raw
  `wd` bits to the crash record; new reason codes 4 = watchdog fired during
  setup (TIMER bit + no marker = real clock verdict) and 5 = clean reboot
  (no TIMER + marker present). Also: `oc <khz>` now prints a CLKDIV hint to
  keep flash SCK <= 88 MHz (raise CLKDIV as the PLL climbs — the actual lever
  for going past 280, whose rejection was SCK-driven at CLKDIV 3), `oc off`
  restores the build clock, and the 2 s report carries a continuous XIP FNV
  re-check while an override is active (`| oc: <khz> kHz fnv=OK|MISMATCH`)
  so a soak catches the intermittent corruption a one-shot FNV misses.
  README gained a "Reset forensics + overclock bench" section with sweep.
- **Live hardware test of the decoder (2026-11-17)**: flashed via 1200-baud
  touch to COM15 (RP2350 app CDC; COM22 = C3, left alone). Board re-enumerated
  fine, `crash` reported `reason=4 wd=0x01` on the first boot — a FALSE POSITIVE:
  the SDK's touch-to-BOOTSEL uses `watchdog_reboot` (scratch[4]=0 + TIMER bit),
  which looks exactly like a setup hang. A `reboot` command then reported
  `reason=3 wd=0x01` — proving `WATCHDOG_REASON`'s TIMER bit is STICKY on this
  chip and cannot be the discriminator. FIX (build pending reflash): stop using
  `wd` for classification (print advisory only); use three firmware scratch
  words that POR clears but soft/watchdog resets preserve — `[4]`=boot-done
  (0xB007CAFE), `[5]`=clean-shutdown (0xC1EAN0FF, set by `reboot`), `[6]`=
  setup-active (0x5E7AC71B, set at setup entry, cleared at end). Precedence:
  fault magic -> reason 2; clean-shutdown -> 5; setup-active && !boot-done ->
  4 (clock verdict); boot-done -> 3; else -> 1. The 1200-baud touch (pc=0 path
  of watchdog_reboot) clears [4] only, so post-flash boot reads reason 1.
  VERIFIED on hardware: clean `reboot` -> `reason=5 wd=0x01` (wd advisory,
  sticky) with the new scratch decoder; the touch-flash boot no longer
  misreports as reason 4.
- **Overclock sweep on hardware (2026-11-17, bench-only — not a boot-clock
  change)**: flashed the new bench firmware to COM15, added a live `vreg`
  console command, swept via `oc <khz> <clkdiv> <rxdelay>` with 8 s soaks,
  checking continuous `fnv=OK`, `stalls 0 (+0)` and no reset:
  - **270 @ default VREG 1.15 V, CLKDIV 4 (SCK 67.5): RESET** `reason=3`
    (post-setup hang/watchdog). So the earlier "270 POR flap" was really a
    core-stability failure at 1.15 V — and the flash SCK was NOT the lever.
  - **270 / 280 / 288 / 300 / 312 / 320 @ VREG 1.20 V, CLKDIV 4**
    (SCK 67.5-80): ALL PASS — FNV OK every report, zero stalls, no reset.
    280 here uses CLKDIV 4 (SCK 70) and does NOT reproduce the old CLKDIV-3
    (SCK 93) corruption — confirming the divider was the 280 issue.
  - **328 @ 1.20 V, CLKDIV 4 (SCK 82): PASS** (ack + FNV OK, no reset).
  - **336 @ 1.20 V: immediate reset** `reason=3`. **336 @ 1.25 V: HARD HANG**
    — USB stayed enumerated, CDC wedged (Win error 31/22), the 2 s loop
    watchdog did NOT rescue it (core stuck inside the live PLL/VREG switch),
    and even the 1200-baud touch could not be opened. Physical BOOTSEL (hold
    button + replug) / power-cycle required; a simple power-cycle restores
    the 264 MHz build clock because `oc` is runtime-only, flash unchanged.
  - **Verdict: 320 MHz is the last cleanly stable point, 328 the outer edge,
    336 the cliff.** Root lever is CORE VREG (1.20 V), with CLKDIV >= 4 to
    keep flash SCK <= 82 MHz. User called the bench here with "enough data",
    then approved 320 MHz / 1.20 V as the new boot default.
- **320 MHz / 1.20 V adopted as the boot default (2026-11-17, user-approved)**:
  `platformio.ini` `f_cpu` 264 -> **320000000L**. A `__attribute__((constructor
  (101)))` `rack_pre_clock()` in main.cpp raises VREG to 1.20 V and pins QMI
  CLKDIV 4 / RXDELAY 3 BEFORE the Arduino core's main() calls
  `set_sys_clock_khz(F_CPU)` — the core's own VREG auto-adjust is RP2040-only,
  and the ROM's CLKDIV 3 would put SCK at 106.7 MHz at 320. A dummy XIP read +
  `dsb` follows the SDK's psram_reinit_timing precedent. All 5 envs build-green.
  Flashed rack-lira8 to the board (was in BOOTSEL after the 336 hang): booted
  clean, `qmi` = 320000000 Hz / CLKDIV 4 / RXDELAY 3 / SCK 80 MHz, `vreg` = 13
  (= 0b01101 = VREG_VOLTAGE_1_20), `crash` reason=1 (BOOTSEL flash), zero
  stalls, `max Process` 464-509 us of 1451. CAVEAT: 320 is NOT a 12 MHz
  multiple — PIO-USB host (currently disabled, RACK_ENABLE_USB_HOST=0) would
  need 312; flag preserved in platformio.ini + README + the setup() guard.
  LEDGER NOTE: only a fresh boot verified so far — a real multi-module /
  multi-hour soak is still needed before calling this accepted.
- **VREG 1.20 -> 1.25 V (2026-11-17, Warps-at-320 hang root cause)**: re-enabling
  the Warps stage (mode switch via CC 37 = 5) produced a HARD freeze under
  repeated CC/switch traffic: USB stayed enumerated, CDC wedged (error 31),
  watchdog did NOT rescue it (interrupts frozen) and no crash record was
  written. Bisected by A/B: committed firmware (warps bypassed) survived 20+
  mixed stress cycles; WIP hung at cycle 2-17; `switch`-only (no warps) survived
  30; `cc 37 5`-only (no switch) hung at cycle 11 -> the Warps DSP sustained
  load was the trigger. **Raising VREG 1.20 -> 1.25 V in `rack_pre_clock()`
  eliminated the hang**: cc-only 40 cycles, cc37 30 cycles, mixed 20 cycles,
  45 s idle soak all clean, 0 stalls, VREG readback 14 = 1.25 V. The earlier
  "spontaneous 320 MHz wedges" (before any Warps code) were this same marginal
  core voltage — 1.25 V is the right 320 MHz default. Boot default + README
  updated. The 264 MHz sweep had only tested 1.20 V in short soaks; the extra
  DSP load is what pushed it over.
- **Switchable hang diagnostics are now PERSISTENT (2026-11-17, user rule)**:
  `-DRACK_HANG_DEBUG=0/1` in platformio.ini gates core-0/core-1 watchdog
  scratch breadcrumbs (`crumb()` / `crumb1()`, stages 10-13 render, 50/51
  cmd_cc, 60-62 swap), a CRUMB boot print, and `crash: hang0=/hang1=`. Set 1 to
  hunt a freeze, 0 for production (current). NEVER delete/rewrite these — the
  prior investigation had stripped them and we re-added them under time
  pressure; keep them for next time. Crumbs are cleared each clean loop pass
  (else they are stale); scratch[0]/[1] survive a watchdog reset.
  `cmd_cc`'s ack was also moved to the non-blocking `dbg_printf` (a blocking
  CDC printf was the documented 2026-11-16 hang class).

- **Final FX control layout (2026-11-17, user decision: no parallel loading,
  both DSPs compiled in, original Warps controls)**: CC 42 = FX selector
  (0-4 distortion tube/fold/fuzz/shape/swede, 5 Warps). Distortion params CC 43
  drive / 44 bias / 45 tone. CC 36-40 are the ORIGINAL T9 Warps controls
  (algo/timbre/mod/car/mix) again, staged in State and applied to the Patch by
  `apply_fx_params()` only in Warps mode; CC 37 is W TIMBRE once more (no longer
  overloaded as the type selector — that caused "missing original controls" +
  gritty sound via an aggressive car=1.0 seed). One DSP active per block (no
  coexistence, no SRAM overlay). Clean defaults (all warps params 0 = bypass).
  C3 P2 cells switch on the CC 42 cache: FX TYPE + D DRIVE/BIAS/TONE or FX TYPE
  + W ALGO/TIMBRE/MOD/CAR (W MIX on web panel). `rack_panel.html` updated (FX
  TYPE stepped bytes 0-5, not 0/16/.../127 — byte must equal firmware val<=5).
  Verified: CC 42=0 dist params respond; CC 42=5 Warps runs (Process ~623 us),
  20-cycle mixed stress clean at 1.25 V. User confirmed "dry signal at max
  distortion" was the waveshaper's carrier/fundamental, not a bug.
- **Two user-found bugs fixed (2026-11-17)**:
  1. *Screen not updating after FX switch*: changing CC 42 (FX type) sent only a
     TELEM delta (value, no cell labels), so the C3 kept stale P2 cells until a
     page switch. Fixed: `ui_link_observe_cc` sends a FULL TELEM snapshot on
     CC 42 for Elements (relabels the mode-dependent P2 cells).
  2. *Modulator bleed into reverb/distortion*: at `space=0`, `raw_gain=1` makes
     Elements' `aux` = raw exciter (the Warps modulator), so it reached
     reverb/distortion unprocessed ("modulator output without reverb"; with
     space on, raw_gain=0 and the signal was fine). Fixed in `part.inc`: in
     Warps mode (`warps_ && warps_algorithm>0`) force `raw_gain=0` so `aux`
     carries the clean stereo resonator. Both carrier+modulator feed Warps and
     are mixed there; reverb/distortion get main without the modulator. Delta
     lives in `tools/patches/elements/elements-warps-stage.patch` (new hunk
     `@@ -150,6 +164,13 @@`), applied by sync to both rack + standalone libs
     under `RACK_RP2350_TC`. Host test `test_warps_stage.cc` assertion updated
     (aux must now differ from the warps-less Part in Warps mode). All 5 envs
     build-green, rack-lira8 flashed. NOTE: host test could not be rebuilt here
     (no g++ on this machine) — re-run `make test_warps_stage` in WSL.


### Rack T9a — warps memory truth + ctag-plan bugs (2026-11-17)

- **`sizeof(warps::Modulator)` is ~68-73 KB WITH the vocoder, not the
  8-12 KB the ctag plan estimated**: `Vocoder` holds two `FilterBank`s, each
  with `delay_buffer_[6144]` floats (24,576 B) + `samples_[960]` (3,840 B).
  Host-measured pristine: **73,192 B**. The rack cannot host that (RAM is
  statically partitioned to ~zero slack; no PSRAM; heap gate ~30 KiB taken)
  — hence the owner decision to strip the vocoder. Stripped + `kMaxBlockSize`
  16: **2,312 B** (host-measured; `static_assert` ≤ 4096 in
  `module_elements.h`).
- **ctag-plan bug 1 — `modulation_algorithm` scale**: warps' parameter is
  **0..1** (internally `min(alg*8, 5.999)`; vocoder from 0.675; the
  `skewed_modulation_parameter()` 1..5 branches are vestigial). The plan's
  `val/4095*5` mapping would land every position > ~0.125 in the vocoder
  zone. Rack mapping: CC byte/127 → algorithm, clamped ≤ 0.625 = pure
  comparator (knob zones: 0 xfade, 0.125 fold, 0.25 diode RM, 0.375 digital
  RM, 0.5 XOR, 0.625 comparator).
- **ctag-plan bug 2 — vocoder SRC divisibility**: `SampleRateConverter
  <SRC_DOWN>::Process` **returns without processing** when
  `input_size % ratio != 0` — the mid band needs size divisible by 3, low by
  4. ctag's Part block 32 and the rack's 16 both fail → their vocoder path
  would be silently broken. Any future vocoder phase needs 48-sample burst
  staging (48 % 3 = 0, 48 % 4 = 0, ~150 µs latency) plus a measured
  `kDelayLineSize` shrink (Init caps per-band delays at 256 samples — the
  6,144-float pool is heavily over-provisioned).
- **LUT collisions**: warps' `lut_xfade_in/out` collide with clouds' kept
  (referenced, `.tc_clouds`-sectioned) symbols → renamed
  `warps_lut_xfade_in/out` in the vendored copy. `lut_sin` does NOT collide
  despite clouds defining the same name: warps' definition is unreferenced
  after the easter-egg strip and gc-sections drops it (the same mechanism
  that keeps clouds' and plaits' `lookup_table_table` coexisting).
- **Patch-generation levels**: the warps patch applies at `-p1 -d
  lib/elements/warps`, so its labels are `a/dsp/...` `a/resources.h` (module
  subtree stripped) — NOT `a/warps/dsp/...`; the elements patches apply at
  `-p1 -d lib/elements` with `a/elements/...` labels (`git -C <lib> diff
  --relative`).
- **Sync-script caveat**: the full sync aborts when `EURORACK-STEREO/` is
  not checked out (pre-existing; it exited before the rack line, and one
  aborted run left CRLF churn in `clouds-rp2350/lib` — restored). The warps
  step was verified standalone (copy + `patch -p1` reproduction); re-run the
  full sync only with all submodules present.
- **Host-test harness extended** (`rack-rp2350/test/Makefile`): new
  `WARPSFLAGS` (-I lib/elements + lib/stmlib) and `RACKFLAGS` (+ -D
  RACK_RP2350_TC) test targets `test_warps`, `test_reverb`,
  `test_warps_stage`; stmlib's class-static tables come from including
  `stmlib/dsp/{atan,units}.inc` + `stmlib/utils/random.inc` (mirroring
  `stmlib_all.cpp`), and `elements::Dsp::kSampleRate/kSrFactor/
  kIntervalCorrection` are defined in the test TU exactly as
  `module_elements.h` does. `test_warps_stage` compiles the elements lib
  single-TU style (all 8 `.inc` + `elements/resources.inc` + `part.inc`).
- **Window budget**: `tc_elements` 36,752 → **50,824 B** (warps hot set
  14,072 B: mod_process, xmod_process ×6 COMDAT, 7 Xmod specializations,
  Diode, SaturatingAmplifier, both SRC Processes) — A slot 58,368 B →
  **7,544 B slack**; tc_clouds unchanged (57,008 at B slot; window slack
  3,424 B). All five rack envs + the standalone build green; the stage is
  RACK_RP2350_TC-guarded so the standalone object code is unchanged.
- **Pool math**: elements→clouds chain now demands obj ~128 K (Part +24 B
  patch floats + 4 B pointer; clouds 8,692) + buf 43,264 + 2,312 + 131,072 ≈
  305 K ≤ 331,776 — ~26 K headroom; rings→clouds worst case untouched
  (331,516, 260 B slack). Init zero-fill shrank 64 KB → 43 KB (faster
  boot/swap as a side effect).

### Rack VARG — black-metal generative sequencer (6th MOD-ARP algo) (2026-11-17)
- Spec: docs/superpowers/specs/2026-11-17-varg-black-metal-generative-sequencer-design.md
  (design locked with the user: mono riff + harmony; 6th ARP algo; RIFF/PROG/VOICE/DISS;
  archetype library with W7/NEO kernels; web panel out of scope v1)
- `src/varg.h` pure engine, host-tested (`test/test_varg.cc`): pool builders per
  VOICE/quality, W7-restricted + NEO kernels, Weitzmann region law
  (minors anchor+1 / majors anchor), 8 seeded archetype sequences, riff zones.
- `mod_engine.h`: MOD_ARP_VARG=5 / MOD_ARP_ALGO_COUNT=6, VARG state in the ARP ext
  union, bar-line branch (generic drift/fill bypassed), calc_note branch, lira8
  chord publish, algo quantizer 15→18, preset read clamp 4→5, console alias + mod?.
- `ui_link.h` labels/echo + `rack-uimcu/ui/RackUI.cpp` names/buckets/formatting in
  lockstep (soft-takeover trap avoided by landing both sides before flashing).
- Commit: 404a1e3 (code series ce50f74..404a1e3).
- **Bench (2026-11-17, automated part)**: flashed rack-elements UF2 to RP2350
  (COM15, 1200-baud touch → picotool, verified) and C3 app to ESP32-C3 (COM22,
  `build_uimcu.bat -p COM22 flash` — the plan's `flash COM22` form leaves a
  harmless `ninja: unknown target 'COM22'` tail; `-p` is the correct form).
  `mod?` reported `algo=varg(2oct) ... prog=weitzmn chord=pc.. q=.. pool=[...]`;
  all 8 PROG bands, ROOT w7/neo/midi forcing and TRANS 0..7 tracked (neo_op
  rand/p/l/r/n/s/h/plr). LIRA-8: `switch lira8` + arp1 dest 25 + CC 54 ARP HARM
  on → 683-718 µs vs 1451 µs budget, `i2s stalls: 0`. Preset
  dump/collect/load roundtrip restored algo=varg + p1-p4 (84/97/85/55).
  `i2s stalls: 0` throughout; the 496 µs elements budget was already exceeded
  pre-VARG (743 µs on first status) and spikes during long console prints.
- **Extra fix**: 8395ac1 — `mod_set_param_console` guard rejected shared TRANS
  (param 12), so `mod 10 trans <0-7>` was silently ignored; added 12 to the
  allowed out-of-table params (all five envs rebuilt green).
- **User acceptance (2026-11-17)**: user listened across the PROG/RIFF ranges
  and confirmed the black-metal character — **BENCH DONE**. Board left on
  elements, arp1 VARG (RIFF 8TH / PROG WEITZMANN / VOICE TRIAD / DISS 20,
  dest voice).

### Rack Rings — VARG chord mode + ctag-tbd distortion (2026-11-17)
- Request: a 6th Rings mode playing full simultaneous VARG chords/triads (NOT
  arpeggiated) over evolving W7/NEO harmony, with the 4 fast distortion algos
  from ctag-tbd's latest commits applied after the resonator. Design locked:
  re-source StringSynthPart's chord renderer, adapter-level post-render
  shaper (all modes), implement directly.
- **Distortion source**: the 4 algos were NOT in this repo or local refs;
  fetched ctag-tbd (user-approved) and found tube / fold / fuzz / shape in
  commits `2166c84c`, `4f086145`, `a62ad652` (DSP in eurorack@5642bf4
  `rings/dsp/part.cc`, Vult `Fold_do`, braids `wav_sine` table). Ported
  verbatim into the pure header `src/rings_dist.h`; the 257-entry int32 shape
  table is a static rodata singleton (~1 KB .bss), never in the arena pool.
- `src/rings_chord.h` (host-tested): VARG pool / l8harmony snapshot -> absolute
  MIDI notes for simultaneous chord slots; keeps root/third/fifth when the
  pool exceeds the available voices.
- `tools/patches/rings/03-varg-chord.patch` adds `PerformanceState.varg_chord_n`
  (default 0) + `varg_chord[8]` and overrides StringSynthPart's static chord
  table. Hot-set side effect: tc_rings 58,084 -> **60,336 B**; the chain-A
  boundary `RACK_CLOUDS_B_OFFSET` moved 0xE400 -> 0xEC00 (B slot still fits:
  60,416 + 56,964 = 117,380 <= 118,784).
- Wiring: mode 5 = VARG chords (re-strum on chord change, MIDI strum bypass
  via `rack_rings_varg_chord_on`); distortion on **all** modes via CC 24/27-30;
  C3 MODE bucket 5 -> 6 + DIST TYPE bucket 4. All five rack envs, standalone
  rings and C3 build green.
- Commits: 7fd33aa (pure headers/tests) .. 66af79c (docs); 6129105 fixed the
  C3 Rings MODE value-name table (5 -> 6 entries, "VARG" at index 5) — the
  pot bucket was widened earlier but the displayed label still read "---".
- **Bench (2026-11-17, automated part)**: flashed rack-rings UF2 (COM15) and
  C3 app (`-p COM22 flash`). Mode 5 engaged via `cc 20 5` with arp1 VARG
  routed to `voice`: `mod?` showed `algo=varg ... prog=weitzmn chord=pc8 q=min
  bar=1 pool=[0,3,7,12]`. Audio meters active (in ~700-960, out ~930-999).
  Distortion sweep CC 27 = 0/1/2/3 (tube/fold/fuzz/shape) and CC 24 = 0 (bypass)
  / 127 (max drive) all stable: `max Process 178-360 us` vs the 750 us budget,
  `i2s stalls: 0` throughout. Preset dump/collect/load restored mode 5 and the
  distortion CCs (CC cache replay, no swap). NOTE: `out` meter frequently hits
  999 (full scale) at the default drive 0.55 + mix 1 — user ears will judge
  whether to back off CC 24 / CC 30.
- **User acceptance (2026-11-17)**: user listened and confirmed full
  simultaneous chords (not arpeggiated), evolving W7/NEO progressions and
  distinct tube/fold/fuzz/shape characters — **BENCH DONE**. Board left on
  rings mode 5, arp1 VARG routed to voice, CC 24 ~0.55 / CC 27 tube /
  CC 29-30 open.
- **Follow-up 2026-11-17**: user asked that strumming come ONLY from the VARG
  riff (guitar-style picking), not from an extra re-strum on chord changes.
  `a1ae0f8` removed the seq-triggered strum and the now-unused `arp_seq_seen`
  state; strums now come solely from riff gates (dest 127 -> CC 64 -> strum).
  Five envs rebuilt green, re-flashed; BLAST riff shows continuous audio,
  SUSTAIN only fires on its two gated steps, `i2s stalls: 0`.
- **Shared ARP OCTAVE transpose (2026-11-17)**: user reported pitch changes had
  no effect on VARG chords; they chose a transpose in the ARP for ALL algos on
  a MOD-ARP page pot, and corrected that it must be OCTAVE steps, not semitones.
  `05cb674` added the shared control (CC/param 13, console
  `mod <arp> octave <-2..2>`, `oct=%+doct` telemetry, preset byte pl[9], MOD-ARP
  cell 6 "OCTAVE", C3 5-step bar + "+Noct" label); the final pass stores octaves
  in `g_mod_engine.arp_octave` and multiplies by 12 in `mod_arp_calc_note()`
  (ALL six algos share this path) and in the published harmony root
  (`ringchord::from_ctx` carries the root delta so octave shifts reach Rings).
  `dc9812a` fixed the console guard to admit param 13 (same class of bug as
  the earlier TRANS param-12 fix).
- **Pot fix**: the C3's `kModPotMap[5][6]` was still `kNoSet`, so the new MOD-ARP
  cell 6 never sent MOD_SET — the OCTAVE pot did nothing. `8a0f2f0` maps it to
  `{src 10, param 13}` and makes `mod_arp_set_octave()` refresh both generators'
  `cur_note` immediately (previously `note=` only changed on a gate).
- **Bench**: all SIX algos swept at octave -2/0/+2 with `mod?` confirming both
  `oct=` and shifted `note=` (e.g. VARG 18/43/72, MOTIF 24/60/72 with clamping,
  METRO 29/50/92, DEJAVU 27/51/75). Audio active, `i2s stalls: 0`, 196-259 us
  vs 750 us. All five envs + C3 green; both devices re-flashed.
- **Second pot bug**: user reported the C3 OCTAVE display switched but pitch
  did not. Root cause: `mod_engine_apply_ui_set()` (the RP2350 MOD_SET receiver)
  whitelisted only shared params 10/11/12, so param 13 was REJECTED even with
  the C3 map fixed. `f693b93` adds 13 to that whitelist; all five envs green
  and RP2350 re-flashed. C3 was already flashed with the `{10,13}` pot map.
- **User acceptance (2026-11-17)**: user turned the OCTAVE pot through the 5
  steps and confirmed the actual pitch jumps by whole octaves across the ARP
  algos — **BENCH DONE**. Board left on Rings mode 5, VARG, OCTAVE +1.

### Swedish Chainsaw distortion + Elements share (2026-11-17)
- User asked to implement the algorithm from Barabas5532/SwedishChainsaw in our
  distortion module, then to put the same stage on Elements because the T9
  Warps stage was not working as planned — explicitly "don't delete the code".
- Fetched the repo: the signature is the Boss HM-2 EQ at all knobs max — three
  fixed MATLAB biquads (GPLv3, `HM2Filter.cpp`) wrapped in two tanh saturators
  (the active `waveshape()` is just `tanh`; the commented piecewise curve is
  unused). Ported verbatim into `src/rings_dist.h` as DIST TYPE 4 ("SWEDE"):
  HP 60 Hz (700 Hz tight via bias), tanh, 3 HM2 biquads, 7 kHz LP, tanh, with
  the common tone-LP/dry-wet stage. Filter state is a static singleton (no
  arena growth). Coefficients are fixed-digital, so bands sit ~0.73x lower at
  32 kHz — documented. Host tests cover finite/bounded output across 8 blocks.
- Same stage added to Elements (`module_elements.h`) post-resonator using the
  former Warps CCs 36-40 (drive/type/bias/tone/mix). **Warps code preserved**:
  its Patch fields and the lib patch remain; original CC mappings are kept as
  comments, `warps_mix` stays 0 (zero-cost bypass) until the user wants it back.
- C3: DIST TYPE now 5 options with names (TUBE/FOLD/FUZZ/SHAPE/SWEDE) on both
  the Rings and Elements pages; UI labels updated on both sides.
- All five envs + C3 green; both devices flashed. Elements sweep CC37 0..4
  stable: 212-302 us vs 496 us budget, `i2s stalls: 0`. No notes were played in
  the automated sweep — audible character pending user ears.
- Commits: 0b627da (Swedish algo) .. 7c2fe13 (Elements share).
- **User feedback / fixes (2026-11-17)**: "D DRIVE shows distortion types,
  D TYPE shows numbers, not all algos sound different." Two real bugs found:
  (1) Elements' C3 type-name table + discrete count were keyed to CC 36 (which
  is D DRIVE) instead of CC 37 (D TYPE) — `fe2e261` moved both to 37; RINGS'
  table was already correct at CC 27. (2) FUZZ was bit-identical to TUBE at
  the default bias 0 (bias_sub=0, bias_norm=1) — `fe2e261` adds a 0.25 bias
  floor so it is always asymmetric. Host tests + all five envs + C3 green;
  Elements + C3 re-flashed.
- **User acceptance (2026-11-17)**: user re-tested and confirmed the type names
  display on the correct cells and all five distortion types sound distinct
  on both Elements and Rings — **BENCH DONE**. Warps remains preserved and
  bypassed as requested.
- **MIX removed (2026-11-17)**: user pointed out MIX is redundant — drive 0 is
  already a zero-cost bypass, so mix never adds a distinct state. `cad6a8a`
  removed `Params::mix` and the blend (always fully wet when engaged), the
  `dist_mix` state + CC 30 (Rings) / CC 40 (Elements) handlers, the mod-dest
  entries and the C3 "D MIX" cells. Host tests updated; all five envs green;
  RP2350 re-flashed. Bench: CC map no longer lists mix, drive 0/64/127 stable,
  `i2s stalls: 0`. **User accepted** DRIVE as the sole amount control.

### Mod-mapping / pot-handling investigation (2026-11-17)
- User reported systemic MOD_SET bugs: knobs map random parameters and
  unassign fails randomly, across all plugins. Three parallel audits agreed the
  static `(src,param)` tables are CORRECT (96/96 cells match across C3
  `kModPotMap` and the RP2350 `kUiMod*Cells`) — the failure is in shared state.
- Findings: (1) C3 `s_cur` (~320 B) is assigned wholesale by the UI task while
  the pot-scan task reads `s_cur.cell[].cc/.flags` — same priority, no lock →
  torn reads; `GetPotTarget` had no `s_cur.page == s_page` guard. (2) `sendTarget`
  resolved the target a SECOND time (double `GetPotTarget`) so a page change
  between the two calls wrote a different param. (3) Unassign sends clears only
  for `localSrc`/`curSrc`; a torn/zero read sends nothing (silent no-op).
  (4) Preset load never forced a TELEM refresh, leaving ROUTED flags stale.
  (5) ARP mod-ride wrote param 3 while the strip said "DENS" (param 1).
  (6) Historical: `ASSIGN_CYCLE` is dead code on the C3 (call site deleted in
  `6dbb695`); the C3-local `s_potModSrc` cache + raw MOD_SET dest writes are
  the architectural root of drift vs the RP2350's authoritative routing.
  (7) Stale duplicate `docs/ui_link.h` still documents the old 3-algo contract.
- **Fixes applied (b620840, C3 only)**: critical-section around `s_cur` writes;
  page guard in `GetPotTarget`; single target resolution passed into
  `sendTarget`; TELEM re-request after preset load; ARP ride param 1.
  **Encoder source scroll now CLAMPS at OFF/ARP2 (no wrap)** per user request.
- C3 build green + re-flashed. **User re-test**: "mapping PITCH but it jumps
  and maps WAVE on encoder confirm" on a module P1 page. On Rings P1 PITCH is
  slot 6 and WAVE slot 14 (not neighbours), so this is the ASSIGN TARGET LOCK,
  not the send path: anchors were captured from `filtRaw` (still settling from
  the encoder-press artifact) and a single spike could steal the target.
  `54c8bf4` anchors from the fresh `rawSweep` and requires the same leading pot
  across 3 consecutive scans before locking/switching, with candidate state
  reset on enter/exit of assign mode. C3 rebuilt + re-flashed.
- **User acceptance (2026-11-17)**: user confirmed PITCH now locks correctly,
  confirm maps the right parameter and unassign reliably disables it —
  **BENCH DONE**. Deferred (not needed now): restoring authoritative
  ASSIGN_CYCLE / extending its RP handler to MOD pages 2..7, and refreshing
  the stale `docs/ui_link.h` duplicate.


## HANDOFF — Elements FX (Warps + distortion) + reverb, 2026-11-17 (end of session)

**Board state flashed + booted clean**: commit `511808c`, `rack-lira8` UF2
`33075A35A77B`, 320 MHz / VREG 1.20 V, COM15 healthy.

**Final working design (Elements FX, user decision = no parallel loading, both
DSPs compiled in, one active per block):**
- CC 42 = FX TYPE: 0-4 distortion (tube/fold/fuzz/shape/swede), 5 Warps.
- Distortion params CC 43 drive / 44 bias / 45 tone (Rings uses 24/27/28/29 —
  taken on Elements by brightness/space/model/easter-egg).
- Warps params CC 36-40 = ORIGINAL T9 mapping (algo/timbre/mod/car/mix), staged
  in State, applied to the Patch only in Warps mode (`apply_fx_params()`).
  CC 37 = W TIMBRE (no longer overloaded). Boot defaults: DIODE algo, 0.5
  timbre/mod/car, 0.75 mix so C3-only users hear Warps immediately.
- C3 P2 cells switch on the CC-42 cache (full TELEM sent on CC 42 to relabel):
  FX TYPE + D DRIVE/BIAS/TONE, or FX TYPE + W ALGO/TIMBRE/MOD/W MIX (W CAR on
  web panel). Web panel `tools/rack_panel.html` updated to match.

**Reverb — DO NOT RE-BREAK (two bugs I introduced and reverted):**
- `bd286f1` forced `raw_gain=0` globally → starved reverb of the raw-exciter
  signal ("no diffusion, repeated delay"). REVERTED in `85e8063`: `raw_gain=0`
  ONLY while Warps runs (`warps_ && warps_algorithm>0`); distortion/reverb path
  is byte-identical to the pre-warps state.
- `ffd3d3b` cut the reverb's LFO modulation 80->16 / 100->20 to tame a flange
  at 95-100% space — this KILLED the Dattorro's essential chorus and sounded
  "very weird". REVERTED in `511808c`. **The mild flange at high space is
  upstream Elements character (since 8f48630) — leave it alone.** Reverb.h is
  now byte-identical to 9466f43 (pre-my-work); `git diff 9466f43 HEAD -- reverb.h`
  is EMPTY — use that as the canary.
- The working reverb decay fix (`reverb_time = 0.35 + 1.2*reverb_amount`) is
  from 8f48630 (Nov 11), predates all my Warps work — KEEP.

**Rule learned (user)**: never edit generated `lib/` by hand + maintain a
separate patch that can drift — but ALSO don't churn the sync script. The
`elements-warps-stage.patch` / `elements-reverb-restore.patch` are the source
of truth for lib deltas; verify with `sync_pio_libs.sh` before finalizing.

**Still pending (honest status)**: scripted console stress + 1.25 V showed no
hangs; real multi-module soak + user's ears on Warps algorithms/reverb tail
not yet signed off. Host test `test_warps_stage` could not be rebuilt here (no
g++ on Windows; binaries are WSL ELFs) — run `make test_warps_stage` in WSL.


## Reverb flange ROOT CAUSE — FxEngine<21632> broke fx_engine.h's power-of-two wrap (2026-11-17, evening)

The handoff above called the high-space flange "upstream Elements character". **Wrong.** User pushed
back ("find the misalignment with elements"); systematic diff found the DSP path byte-identical to
pristine pichenettes (ctag-tbd `components/mutable/eurorack`) and poetaster ELEMENTS/src everywhere
EXCEPT the T9 engine shrink — and the shrink itself is the bug:

- `fx_engine.h` wraps **every** delay-line access with `& MASK` where `MASK = size - 1`. That is a
  correct circular wrap only for **power-of-two** sizes. Upstream `FxEngine<32768>` = 2^15 ✓.
  `342fd43` (2026-11-17 04:48, T9 SRAM reclaim) instantiated `FxEngine<21632>` — NOT a power of two.
  Any access with `write_ptr_ + base + offset >= 21632` (constant: the delay regions span 21,626
  frames — the Reserve chain sum 21,617 **plus 9 per-line `+1` gaps** — and `write_ptr_` cycles the
  whole buffer) aliases to a scattered address instead of wrapping (e.g. `43257 & 21631` = 121 =
  inside AP1's region). Reads/writes land in wrong slots → comb artifacts recirculated by the
  krt-0.83 loop = the "flange". Inaudible below space 0.6 (reverb_amount 0 → dry bypass), worst at
  95-100% space (max feedback) — exactly the reported symptom, and why neither the LFO taming
  (ffd3d3b, reverted) nor raw_gain hacks touched it. The Nov 11 decay restore (8f48630) predates the
  shrink, so "flange since 8f48630" was coincidence of first-audibility, not cause.
- **Evidence** (host A/B, `test/reverb_ab.cc`, rack arm vs upstream arm, identical input/settings,
  space=1.0 params): before fix checksums `1304711d` vs `2b128693`; outputs differ from the FIRST
  sample, 100 % of samples, max |diff| 1.34 (beyond full scale — aliased garbage recirculating).
  After fix: **BIT-IDENTICAL** (`2b128693` both) — engine size is provably sound-neutral once the
  wrap is correct (delay lengths are compile-time; only absolute addresses move).
- **Fix (direct lib edit, user decision — NO patch file)**: `fx_engine.h` gains `WrapAddress()`:
  pow2 sizes keep `& MASK` (constant-folded; every other FxEngine instantiation in the repo —
  clouds/rings/plaits fx — is pow2, so their behavior is byte-identical), non-pow2 wraps with ONE
  conditional subtract (regions are STATIC_ASSERTed < size → any address < 2×size, single subtract
  exact). `make reverb_ab` is the regression gate; `make test` + all 5 pio envs green after.
- **Caveat**: the fix lives ONLY in the lib copy — a future `tools/sync_pio_libs.sh` run regenerates
  `lib/elements` from pristine and **reverts it** (bug returns). Re-apply the WrapAddress edit after
  any sync, or fold it into a patch then (user's call; they explicitly rejected the patch route now).
- The `git diff 9466f43 HEAD -- reverb.h` canary still holds (reverb.h untouched); fx_engine.h now
  intentionally deviates from upstream — that is the misalignment fix, not drift.

### Second regression found + fixed: always-on post-reverb shaper, no bypass (2026-11-17, night)

User re-tested after the wrap fix: "still phased or flanged noises like looped trails but heavily
distorted and in mid range, this is not how reverb was soundin all time before." With the reverb DSP
now provably upstream (bit-identical A/B), the remaining delta vs the pre-FX-session sound was the
FX-session's ringdist stage: it sits AFTER reverb_ in the render path, runs EVERY non-Warps block,
and boots at fx_type=0 (TUBE) drive=0.55 — and CC 42 has no OFF position, so the "dry + reverb"
state was unreachable. Host probe (`test/reverb_dist_probe.cc`, real Part render, strike exciter,
modal, space 0.95, identical phrase both arms):
- pre-shaper (upstream tail): peak 0.68, RMS 0.078, crest 8.7 (diffuse/airy)
- after tube @ 0.55: peak 0.999 (softclip ceiling), RMS 0.240 (**×3.1**), crest 4.2 (squashed wash)
- mechanism: k = 1 + 9·d² = 3.72 into softclip; WAVs at test/build/tail_{raw,distorted}.wav.
**Fix (user decision from 3 options — D DRIVE boots at 0)**: `module_elements.h` boot default
`dist_drive 0.55 -> 0.0` — rings_dist's designed zero-cost bypass (`drive <= 0` returns before any
read/write), so boot is bit-identical to the pre-FX-session chain; FX TYPE selects the shaper, D
DRIVE engages it. No CC map / panel / C3 changes. Autosave does not cover FX state (verified —
preset.h has no dist/fx fields), so the default governs every boot. rack-elements rebuilt clean.
Still honest: user ears sign-off pending; commit pending.

### Chain law implemented: REVERB IS THE LAST FX (2026-11-17, night; user directive)

User: "reverb should be last fx in the chain. distortion and warps should be BEFORE!" Warps already
complied (pre-reverb inside Part); the ringdist shaper did not (post-reverb, an inheritance artifact
from the Rings "adapter-level post-render shaper" design — see the entry two above).

- **Lib hook (direct edit, rack lib only)**: `part.h`/`part.inc` gain a `PreReverbInsert` — an
  adapter-owned `void (*)(float* main, float* aux, size_t size, void* user)` called inside
  `Part::Process` AFTER the warps stage and BEFORE `reverb_.Process` (guarded `RACK_RP2350_TC`;
  `Init` sets it null → upstream-exact). The adapter installs it in `init_at`:
  `part.set_pre_reverb_insert(pre_reverb_shaper, &s->dist)`; the trampoline skips itself in Warps
  mode (one DSP per block) and rings_dist's `drive <= 0` is the zero-cost bypass. The two
  post-reverb `ringdist::process` calls in `render()`/`chain_render_a()` are REMOVED.
- **test_fx_chain.cc (new, Makefile)** laws, all GREEN: (1) structural — a capture-only insert is
  recorded silent (<0.001) during gate-off stretches while the Process output still rings the tail
  (>0.01): 13,780 proof samples — impossible for a post-reverb insert; (2) inert inserts
  (capture-only AND drive-0 shaper) render BIT-EXACT vs no insert; (3) drive-0.55 runs pre-reverb,
  bounded. Informational: the reverb OF a shaped signal has a denser deep-tail crest (3.8 → 2.0 on
  the rig) — that is fuzz-into-reverb by design, not tail corruption. Note: whole-render crest
  (8.7/4.2 numbers in the entry above) ≠ deep-tail crest (3.8 clean) — different statistics.
- Warps ordering note: shaper insert sits AFTER the warps stage; moot — FX TYPE makes them
  exclusive (one DSP per block). Rings keeps its own post-resonator shaper (no reverb downstream;
  untouched). All 5 envs + full host suite green after the lib change.
- Still honest: user ears pending; commit pending (whole changeset: wrap fix + boot drive 0 +
  chain law + tests).

### Four user-reported bugs after the chain fix (2026-11-18, rack-elements UF2 pending)

User report: (a) reverb tails fixed but crackles "when contour non-zero — like bleeding of
modulation signal"; (b) at SPACE 0 only the left channel carries Elements, the right carries the
raw exciter — "no exciter should be heard, it only routed to Warps; now it routes to all distortion
algos"; stereo sum of the resonator should feed them; (c) Warp CARRIER missing from the C3 UI;
(d) FX TYPE only switches to Warps, the first 5 algos sound identical.

Root causes + fixes:
1. **Crackles == exciter bleed (same root as (b)).** The user's "contour non-zero" clue pins it:
   a sustained raw exciter (`raw_buffer_`) was reaching `aux` via `raw_gain=1` at low space, so it
   fed the pre-reverb shaper, the Dattorro input and the right output. Earlier (ledger 4317) a global
   `raw_gain=0` was reverted as "starved reverb" — that diagnosis was wrong: `raw_gain` is already 0
   for SPACE > 0.1, i.e. whenever reverb_amount > 0, so forcing it to 0 in the rack build cannot
   touch the wet tail at all. Fix: `part.inc` RACK_RP2350_TC guard now forces `raw_gain = 0.0f`
   UNCONDITIONALLY (was only `if (warps_ && warps_algorithm>0)`). `main`/`aux` now always carry the
   clean stereo resonator (R/L; at SPACE 0 both are the centre sum) into the shaper, reverb and I2S.
   Exciter stays available only as the Warps modulator. Patch-of-truth updated
   (`tools/patches/elements/elements-warps-stage.patch`, same RACK guard); standalone unchanged.
2. **Distortion algorithm switching.** `dist_params()` read `State::dist_type`, which was never
   written by any CC — only `fx_type` (CC 42) was. So FX 0-4 all ran the same tube shaper and only
   FX 5 (Warps) changed anything. Fix: removed `dist_type` (state, boot init, `dist_params()`);
   `dist_params()` now uses `fx_type` directly (0-4, Warps never reaches the shaper). CC 42 / the mod
   engine's `mod_set_cc_float(42)` remain the single source of truth.
3. **C3 Warp CARRIER.** `kUiPageElementsP2Warps` had only ALGO/TIMBRE/MOD/MIX; W CAR (CC 39) was
   web-panel-only. Added `{39, "W CAR"}` as cell index 8 (the C3 has 16 pots/page; TELEM count is
   derived from the table). No C3 firmware reflash needed — the C3 renders whatever cells TELEM
   sends and CC 39 needs no discrete-count entry. Web panel already had W CAR.
4. **test harness rot found while verifying.** `test_fx_chain.cc` accumulated `g_output_main`
   across all four renders but `g_insert_main` once → length check always failed; the structural
   proof threshold (>8000) was calibrated to that 4× count. Also `Part::Init` does not zero `panic_`
   (firmware's arena is zeroed, the stack test rig was not) → first block could skip the insert.
   Fixes: `g_capture_output` scopes output capture to the capture rig; `init_rig` zeroes the whole
   Rig; proof threshold is now >4000 for the single 8 s render. `test_warps_stage.cc` updated:
   at SPACE 0 `main == aux` bit-exact in both rigs, and Warps leaves aux untouched (no bleed).

Verification: all five rack envs build green (`pio run -d rack-rp2350`, 4m19s total); WSL host
tests `test_warps_stage` + `test_fx_chain` both pass (5943 structural proof samples, deep-tail
crest 3.81 clean / 2.04 shaped unchanged). Not flashed — user ears pending. Uncommitted.

### Flashed + second round of user bugs (2026-11-18)

Flashed rack-elements UF2 (1200-baud touch on COM15 → E: BOOTSEL → copy, board rebooted clean;
`mod=elements`, 243 µs / 496 µs budget, 0 stalls). User then reported three more bugs; two independent
deep investigations (subagents) found them:

1. **C3 mod map loss** ("map PITCH on P1, map PLAY on P2, back to P1 → PITCH unmapped"): the RP2350
   mod engine has exactly ONE destination per source (`mod_engine.h:226 dest_cc`, `mod_set_dest()`
   overwrites it). The C3 assign gesture allowed source reuse and `RackPots.cpp:221-227` silently
   cleared the older cell's bookkeeping while the new `SendModSet` re-pointed the source, so the old
   cell's TELEM route flag vanished. Fix (C3): refuse a duplicate source with a `SRC IN USE` banner
   instead of stealing — added `RackUI::ShowBanner()` wrapper (file-local `showBanner`) to the public
   class, and replaced the uniqueness loop in `applyModAssign()`. Also cleared `s_pendingAssign*`
   in `SetPage()` (secondary race: a pending confirm resolved against the new page).
2. **MOD-ARP DIV jumps to 8/1 on page entry**: `PickupResync()` (`RackPots.cpp:619-623`) armed
   discrete cells with `lastSent = 0xFFFF`; combined with the `atEnd` clause in `potTask()`
   (`v==0||v==127 && v!=lastSent`), a pot parked at an endstop emitted a spurious CC/MOD_SET with
   zero movement on every full TELEM (page switch, reconnect, 30 s refresh). DIV 0 → `kModDivisions[0]`
   = "8/1". Fix: arm discrete cells at the CURRENT mapped value (`mapValue(i, filtRaw[i])`), so
   `atEnd` is false at rest and only real >= RAW_HYST movement sends. The old `prevModBDiv` hack in
   RackLink.cpp (page 3 cell 15) addressed only one symptom of this same bug; left in place, harmless.
3. **FX TYPE needs an explicit OFF + 100% wet distortion**: added `kFxOff=0`, distortions 1-5,
   `kFxWarps=6` across firmware (`module_elements.h`), C3 (`p1p2DiscreteCount` 7, name table with
   OFF first) and web panel (7 stepped labels/bytes). Boot now FX TYPE = OFF. `pre_reverb_shaper`
   skips OFF and Warps; a selected distortion is 100% wet — `rings_dist::process()` has no dry blend
   (verified against the MIX-removal commit cad6a8a: `chans[c][i] = lp`), drive 0 is an additional
   zero-cost bypass. No literal dry path exists anywhere in the render chain (grep of all
   `part.Process`/`ringdist::process`/`i2s.write16` sites); if the user still hears an unshaped copy
   with FX 1-5 / drive 100% / SPACE 0, it is inside the waveshaper's near-linear small-signal region
   or stale reverb-tank content, and needs a scoped follow-up with those exact settings.

Both devices rebuilt and flashed: all 5 rack envs green; C3 built via `cmd.exe //c tools\\build_uimcu.bat`
(MSYSTEM must be cleared — idf.py skips main() when it is present) and flashed to COM22 (`-p` form
still leaves the known harmless trailing `ninja: unknown target 'COM22'`). RP2350 re-flashed; switched
back to elements via console (`switch elements`) after the C3 restored its cached rings page.
Uncommitted. User re-test pending.

### Third round: FX page labels, level matching, reverb audit (2026-11-18)

- **SWEDE showed Warps cells / WARPS showed distortion cells**: `ui_link.h:663` still keyed the
  mode-dependent Elements P2 table on `g_ui_cc_cache[ELEMENTS][42] == 5u` (old Warps value; 5 is now
  SWEDE). Changed to `== 6u`. Only stale FX-value check in the tree (grepped RP2350 + C3).
- **"Volume when FX off is much lower"** — measured with `test/dist_gain_probe.cc` (0.12 RMS two-tone
  input, 32 kHz): TUBE 1.63x/3.34x/5.89x RMS gain at drive 0.25/0.55/1.0, FOLD 3.00/5.18/4.14,
  FUZZ 1.17/2.59/4.76, SHAPE 4.86/6.08/6.75, SWEDE ~1.02 across all drives. So FX ON was 2-7x
  louder than OFF. Added `dist_level_trim()` in `module_elements.h`, applied after the shaper in
  `pre_reverb_shaper`: TUBE `1/(1+5d)`, FUZZ `1/(1+3d)`, FOLD 0.24, SHAPE 0.18, SWEDE 1.0.
  Post-trim drive-1 gains ≈ 0.98 / 0.83 / 1.19 / 1.22 / 1.03 — FX ON/OFF now roughly matched.
- **Reverb audit (user asked if reverb was changed during troubleshooting)**: NO reverb DSP math
  changed. Verified patches, not just the generated lib: `elements-reverb-restore.patch` RESTORES
  `reverb_time = 0.35 + 1.2*reverb_amount` (the pristine submodule has a broken `0.01f`) — this
  predates all FX work and is the lush full-length decay. `elements-reverb-rack-size.patch` only
  changes engine CAPACITY 32768 -> 21632 (delay lengths are compile-time constants, unchanged).
  `elements-ram-hot.patch` moves `Reverb::Process` to SRAM (sound-neutral). The 80->16 LFO taming
  was reverted in `511808c`. The only direct lib edit is `fx_engine.h WrapAddress()` — required
  because 21632 is not a power of two; ledger A/B proves bit-identical output vs upstream.
  The perceived "trimmed / not lush" tail is the missing shaper gain (FX OFF no longer feeds ~3-6x
  boosted signal into the tank) plus no saturation harmonics, not reverb degradation.
- All 5 envs rebuilt green; RP2350 re-flashed + confirmed on elements (417 us / 496, 0 stalls).
  C3 unchanged since its last flash (the page-selector fix was RP2350-side). Uncommitted.

## Repo restructure — patch system removed, ONE env, stale trees deleted (2026-11-18)

User directive: upstream Mutable Instruments code is frozen (never updated), so the
pristine-upstream + patch-sync layer is pointless AND harmful (agents diffing against it
find nothing; the lib IS the truth). The five per-module envs were a testing-phase
artifact — "now we have only one rack code"; new modules WILL be added in the future.

- **Bake verification BEFORE deletion** (reverse-apply check, `patch -R --dry-run` in
  GIT BASH — WSL patch rejects CRLF patches wholesale, 27/27 false MISMATCH there):
  20/27 clean BAKED; 7 "MISMATCH" all verified semantically present in the lib:
  clouds-cmsis-fft + clouds-ram-hot = "Reversed (or previously applied)" per patch tool;
  plaits ram-hot trio (05/06/07) = 44 `PLAITS_RAM_CODE` markers ≥ patch counts + the
  ram_code.h includes in engine.h/voice.h/square_oscillator.h; warps rack-port =
  `kMaxBlockSize = 16`, vocoder branch stripped, 11 `WARPS_RAM_CODE` markers;
  elements-warps-stage = superseded by lib evolution (pre-reverb insert hook +
  raw_gain=0, build-green proof). Conclusion: the rack libs were already the source of
  truth; the patches were stale descriptions.
- **Removed**: `tools/patches/` (27 files) + `tools/sync_pio_libs.sh` (cdb3206).
  The sync-revert caveat from the reverb entries is MOOT — nothing regenerates lib/.
- **ONE env** (0450e7a): `platformio.ini` → single `[env:rack]`,
  `RACK_BOOT_MODULE=4` (boots LIRA-8 — the board's current UF2). New modules continue
  ids 5+: adapter in src/ + registry/window tables + the define. Runtime swap
  unchanged (CC 32 / serial `switch`). main.cpp boot comments updated.
- **Deleted 34 stale top-level trees** (3a8aac4): upstream forests + sketch dirs
  (BRAIDS, CLOUDS, ELEMENTS, STMLIB, PLAITS, RINGS, TIDES, WARPS, the
  `*Engines*`/`*Scarp*` folders, MMM, marvelousMI, EURORACK-STEREO, CloudyPlaits…,
  Toepler/Peaches braids) + the four legacy standalone ports (elements/clouds/
  plaits/rings-rp2350). Git history preserves everything (pristine sources included).
  Top level is now: docs/ rack-rp2350/ rack-uimcu/ tools/.
- **Docs**: AGENTS.md rewritten for the new reality (libs = source of truth, one env,
  flash path `.pio/build/rack/`, new FxEngine pow2 + FX-chain-law constraints);
  README.md replaced with a repo overview (upstream poetaster context → history);
  PORTING-HIGHLIGHTS.md marked historical where it describes the sync/patch era;
  rack-rp2350/README.md build/sources sections updated (single env, warps vendored
  in place).
- **Verified post-restructure**: `pio run -d rack-rp2350` (single env) SUCCESS;
  `make test` 11/11 green (test_fx_chain now in TEST_BINS, ec2dcd3).

## Swede (FX TYPE = SWEDE) upgraded to the full Swedish Chainsaw chain (2026-11-18, evening)

User: "Chainsaw effect has Bass Middle and Treble controls. We dont have such" + compare with
BrunoDG/Swedish_Chainsaw_Zig and Barabas5532/SwedishChainsaw. Our swede was a simplified port of
Barabas5532 (identical MATLAB HM-2 biquads) WITHOUT the JCM800 FMV tonestack, oversampling, amp HP
and speaker IR. BrunoDG = strictly simpler subset (single tanh, 2 knob bands) — nothing to take.

- **D BIAS removed from UI first** (26d48e5): inaudible on tube/fold/shape, a no-op floor on fuzz,
  on/off-only on swede → gone from C3 P2 (ui_link), panel, CC map + mod destinations; stays 0
  internally (swede input HP fixed at the 60 Hz circuit-measured default).
- **HM-2 bands redesigned at 32 kHz**: solved (f0, gain, Q) from the MATLAB coefficients — RBJ
  peaking at **86 Hz +20.0 dB Q1.073 / 900 Hz +11.1 dB Q1.789 / 1400 Hz +19.1 dB Q0.980**, design
  fs = 44100 (round-trip-exact; the clean 86/900/1400 numbers confirm it). The old fixed-digital
  coefficients ran the bands 0.726x low at 32 kHz (62/653/1016 Hz). GOTCHA: first spec attempt used
  20*log10 for the dB labels (RBJ peaking uses A = 10^(gain/40) → gain = 40*log10(A)); the host
  band-check caught it (0.009..0.15 mismatches) before it reached the board.
- **FMV tonestack ported verbatim-in-math** (FMVDesign.cpp, GPLv3 (C) 2019 Barabas Raffai —
  license noted in rings_dist.h): analog 3rd-order JCM800 network (R1 220k/R2 1M/R3 22k/R4 33k,
  C1 470p/C2 C3 22n), knobs scale RC time-constant products, bilinear at 32 kHz, A0-normalized
  3rd-order DF2T per channel, coefficients recomputed per block. Knob defaults = the plugin's own
  parameter defaults: bass 0.05 (fine-grained — most of its change is 0..0.1), middle 0.5,
  treble 0.5.
- **UI**: CC 44 = D BASS (re-purposed), 46 = D MID, 47 = D TREBLE (both unused on this module);
  CC 45 stays the global D TONE. SWEDE-mode P2 variant (`kUiPageElementsP2Swede`, keyed on the
  CC-42 cache = 5, same mechanism as the Warps variant) shows D DRIVE/BASS/MID/TREBLE/TONE; web
  panel cells added. Swede chain order = reference: HP 60 → pedal clip → HM-2 biquads → 7k LP →
  amp clip → **FMV stack** → makeup 0.35.
- **test_swede_eq.cc** (in TEST_BINS): band-design round-trip vs the reference MATLAB coefficients
  (<5e-4), 32 kHz redesign is not a no-op, stability at defaults + all 8 B/M/T corners, knobs
  change the output, FMV design enters the knobs. Suite 12/12 green; single-env build SUCCESS.
- Honest gaps vs the reference, deliberately not ported: 4x oversampling (aliasing of the
  clippers) and the speaker IR (the plugin README requires one) — the FMV + true bands are the
  missing "Bass Middle Treble" the user asked for. User ears pending (flash + FX TYPE = SWEDE).

## Speaker cab convolution for the Swedish Chainsaw (2026-11-19)

User supplied the IR: tools/"Hesu 2x12 V30 - 440 center edge.wav" (44.1 kHz mono 24-bit, 1.83 s).
Firmware flashed first (COM15, earlephilhower uploader 1200-baud touch -> BOOTSEL -> UF2, verify
OK + "device was rebooted").

- **MI FIR/IR survey**: clouds dsp/pvoc/stft = STFT overlap-add already ported to CMSIS
  arm_rfft_fast_f32 on this board (port note: ShyFFT 7 ms/4096-pt vs CMSIS under the I2S buffer
  depth); warps SRC = scipy-remez static polyphase FIR banks (48-tap); plaits 4x_downsampler =
  small static FIR. CMSIS DSP is vendored (arm_fir_f32/rfft available).
- **IR prep** (tools/ir2array.py, prints to stdout — python never writes sources): FFT-domain
  resample 44.1->32 kHz, **pre-delay compaction** (the Hesu IR has ~26 ms of silent mic-distance
  lead-in — peak at 27.4 ms; skip to 3% of peak, back off 1 ms), truncate 1024 taps (32 ms =
  99.9% of the IR's energy; the full tail adds nothing until 2048+), peak-normalize, 64-sample
  raised-cosine tail fade. Result baked as rodata: src/speaker_ir.h (4 KB).
- **DSP**: SpeakerConv in rack_dist.h — mono-summed 1024-tap linear-history FIR (per-sample dot,
  contiguous windows, block-wise history shift), conv gain 0.25, +1023 samples (~32 ms) latency
  (inaudible, swede is fully wet), ~51 µs per 500 µs block at 320 MHz (1024 MACs/sample mono at
  ~1 MAC/cycle). Applied swede-only, AFTER the FMV stack, BEFORE the global D TONE — the
  reference plugin's chain position. State: singleton like SwedishState (host-testable, no CMSIS
  dependency — pure header discipline kept).
- **test_speaker_ir.cc** laws, all GREEN: (1) impulse law — a 1.0 impulse yields exactly
  kSpeakerIr * kSpeakerConvGain (end-to-end MAC/delay bookkeeping); (2) block impl == naive
  direct convolution (y[i] = Σ ir[k]·x[i−k]) within 2e-5 across 40 blocks — first attempt's
  naive reference was itself misaligned (hist-frame indexing) and the conv was right; (3)
  stability at defaults. Suite 12/12 green; single-env build SUCCESS.
- Cost note: the conv is gated on the swede branch — tube/fold/fuzz/shape and OFF pay nothing.
  If the ~10% block cost shows up as stalls on the 2 s report, drop to 512 taps (16 ms) or add
  CMSIS arm_fir_f32 (asm-optimized, ~2 MAC/cycle).

## Measured: swede FX cost on the board + two real fixes (2026-11-19, night)

**Denormal storm, part 2 — FPSCR is per-core.** The FZ/DN write in setup() covered core 0 only;
the render runs on core 1 (setup1/loop1), whose FPU still flushed nothing. Log evidence: with FX
OFF, max Process crept 624 -> 955 -> 1027 us (budget 496) — spikes even without the shaper. Fix:
same FZ|DN write at the top of setup1() (the audio core). Result: OFF max dropped to 520 us.
NOTE for the future: any per-core FPU mode must be set on BOTH cores.

**Measured swede cost** (tools/measure_fx.py: bounce lira8->elements to force the swap-side
proc_us_max reset — the self-swap guard skips switch-to-same — then 8 s per phase, D DRIVE 0.71):
- FX OFF: max 520 us (0 stalls)
- FX TYPE = SWEDE: max 721 us (0 stalls)
- delta = +201 us = 40.5% of the 496 us nominal budget (shaper + per-block FMV design + 1024-tap
  cab FIR + tone LP). Worst single block; typical lower. 0 stalls over 16 s incl. the autosave
  PART burst. The 4-accumulator unroll (f7259ee) + SRAM-marked dist code (tc_elements 51,664 ->
  57,728 B, window slot 60,416 B, 2.7 KB spare) are part of this result.
- Boot default changed to Elements (RACK_BOOT_MODULE=0) — testing focus; runtime switch unchanged.
- If swede stalls ever appear under sustained load: 512-tap IR (halves the conv) or CMSIS
  arm_fir_f32 built with -ffast-math (see the 4-accumulator analysis — CMSIS-as-built saves
  nothing without it).

## Swede cab conv — decimated to 16 kHz + real avg measurement (2026-11-19, late)

User: "not comfortable... still with distortion caused by starvation" + pointed at CMSIS-DSP and
the Yiu book ch.19/20 (docs/Definitive Guide...md). Findings:

- **CMSIS-DSP verdict**: we ALREADY vendor it; arm_fir_f32 on M33 scalar = the same acc chain as
  our C loop (our cmsis builds at plain -O3 — arm_math.h wants -Ofast). The book ch.19.4: the REAL
  lever is SMLALD int16 dual-MAC = 1.5 cycles/MAC (Fig 19.4 = the FIR dual-MAC pattern) — 2x over
  the float scalar floor, available via __SMLALD (the build already uses
  -march=armv8-m.main+fp+dsp). Kept as the next lever; NOT needed yet.
- **avg instrumentation**: the report now prints `avg N us` (proc_us_sum/count, reset on print +
  swap/chain resets) and an Elements-only `ins=avg/max` tail (the pre-reverb insert timed).
- **Measured (tools/measure_fx.py, 3 phases via the runtime conv gate CC 48)**:
  OFF avg 293 / max 450 | SWEDE noconv avg 336 / max 496 (shaper+FMV = +43 avg — matches the
  op model) | SWEDE conv avg 466 / max 708. The 1024-tap float conv measured +174 us avg
  (~5-7 cycles/MAC — load-use stall bound, not MAC-bound; the disasm shows a clean 13-instr/4-MAC
  VMLA loop, so no codegen bug — it is the scalar float shape's floor).
- **Fix: decimated cab path** — the IR is band-limited to 8 kHz (0.42% energy above), so:
  31-tap halfband dec (32k->16k) -> 512-tap FIR @ 16 kHz (32 ms, 4,096 MACs) -> 31-tap halfband
  interp (its odd phase = the delayed passthrough, free; even phase = 14 taps x2). Conv delta
  174 -> 130 us avg; **swede avg 466 < 500 us block period = no sustained erosion = starvation
  gone** (max spikes 708 are absorbed by the 4 ms I2S buffer, 0 stalls measured).
- test_speaker_ir.cc rewritten: naive full-decimated-chain reference (dec/conv/interp per the
  definitions, zero-padded histories, the impl's dec anchors at mono 2m-1 — the first v is at
  mono -1), 13/13 host suite green.
- Levers left if ears still object: SMLALD int16 conv (130 -> ~50 us), 16k->8k second decimation
  stage, or core-0 offload (double-buffered mono handoff; core 0 is mostly idle — USB/UI/console).

## Cab conv offloaded to core 0 (2026-11-19, night) — swede avg 403/500 on the audio core

User: "almost good but still some crackles. what about second core?" The residual crackles =
the 708 us max spikes eroding the buffer under sustained play (avg 466 was under, but the max
margin was thin). Core-0 offload implemented:

- **Pipeline** (module_elements.h): core 1's pre-reverb insert publishes the post-FMV mono block
  into a 2-slot ping-pong (s_conv_in), core 0's loop() runs conv_pump() which convolves ready
  slots via rackdist::speaker_conv() (the dec/hist/vhist state now lives on core 0), core 1
  consumes the PREVIOUS block's result one block later (+500 us, inaudible). Hand-off ordering
  via __dmb() (pico-sdk) around the flag flips; bounded 250 us waits; a core-0 miss bypasses to
  the mono pre-conv signal for one block and counts conv_misses (never stalls audio).
- **Bench** (report `ins=a/b cv=c/d miss=n` tail, Elements-only, reset per report):
  ins 43/55-290 us (audio-core shaper+FMV) | core0 conv 117/192-219 us | misses 0-1 (warmup).
  Audio-core render avg 402-404 us vs the 500 us period — ~100 us headroom with the FULL swede
  chain; max spikes 635-654 absorbed by the 4 ms I2S buffer; stall deltas 0 across all windows.
- CC 48 (bench conv gate) now skips the whole hand-off — no spin-storm when gated off (the first
  gated measurement spun 250 us x2 per block — the gate must short-circuit BEFORE the waits).
- Params.conv removed from rack_dist (the gate lives in the adapter; rack_dist::process no longer
  calls the conv at all). SpeakerConv::Process is invoked from core 0 only — the state singleton
  has a single consumer. FZ/DN is set on BOTH cores (FPSCR is per-core — see the entry above).
- Core-0 headroom: conv ~117 us of the 500 us frame (~23%) + USB/UI/console — fits. The conv avg
  117 us vs ~58 us op-model = core-0 contention/IRQ jitter — irrelevant while misses stay 0; the
  SMLALD int16 lever (ch.19.4, 1.5 cyc/MAC) halves it if ever needed.
- 13/13 host suite green; flashed COM15, verify OK. User ears pending (expect: no crackles under
  sustained swede play; if the SMLALD/miss counter ever spikes, the ins/cv/miss tail shows it).

## Volume inconsistency fixed: level-matched bypass + 3-slot hand-off (2026-11-19, night)

User: "the volume is inconsistent... volume changing with distortion off and other distortion
modules too". Read-only console state (no flashing) localized it:

- **`miss=36` per report window** — the core-0 conv hand-off missed its 250 us deadline ~30x/s
  (core-0 bursts: the C3 UI-link TELEM replies + the report printer). EVERY miss bypassed with
  the RAW mono-sum at FULL level while the conv path runs at 0.25 gain + cab attenuation — the
  output jumped between two level families = the volume inconsistency. (The level-matching edit
  existed in rack_dist.h but was never wired into the adapter's bypass.)
- **With distortion OFF**: mod source 10 (VARG ARP) is routed `dest=1(voice+trig on elements)`
  (chord=pc6 pool=[0,6,7,12]) + the hardware button strikes (btn=29/window) — the volume/pitch
  following the ARP pattern + strikes is the instrument working, not a bug.
- **Fixes**: bypass = mono x speaker_bypass_gain() (= sqrt(sum ir^2) x conv gain, RMS-matched,
  computed once from the IR); waits budget-aware (400/460 us since insert start instead of
  absolute 250+250 which could overrun the block); kConvSlots 2 -> 3 (core-0 jitter headroom);
  slot advance (slot+1)%3.
- **Verified on board (strikes during capture)**: miss 0 across all windows, peaks in 0.35 /
  out 0.25 (the cab-attenuated signal flowing through the core-0 pipeline), render avg 316 us
  (184 us headroom), max 632-653 (strike spikes, absorbed), stall deltas 0. 13/13 host suite.
- Note: dbg_printf's line buffer was 400 B — the report line (avg + 4 tails) exceeded it and
  silently truncated the bench tail; enlarged to 560. newlib-nano vsnprintf has NO %f support
  (printed `p=/`) — integer hundredths used for the peaks.

**CORRECTION (user, 2026-11-19)**: the ARP-routing explanation for the earlier "volume changing
with distortion off" report was WRONG — mod source 10 was routed AFTER that observation ("i routed
it later, this is not the cause"; the report is "not actual anymore" — the audio is clean now on
the current build). The actual cause of that transient observation was never pinned. Candidate if
it recurs: the C3 pot->CC mapping vs the elements P2 cell list (the D BIAS cell removal shifted the
cell count — a C3 pot may send an unintended CC, e.g. VCA). Check RackPots.cpp/RackUI.cpp against
the live TELEM cells before touching DSP code. Audio acceptance: "ok now it sounds clean" (user).

## HANDOFF — q15 SMLAD conv implemented (Windows session ends), continue on Linux (2026-11-19, late night)

Audio ACCEPTED CLEAN by the user (post 761c290: "no volume drops anymore"). The session then
implemented the q15 SMLAD conv per the user's "SMLALD is a must" — **implemented + host-verified,
NOT yet flashed/measured on HW** (the user stopped the Windows flash-test loop; the Linux machine
continues).

**The full arc of this session** (all committed on rack-rp2350):
1. Silence root cause: the core-0 conv pump never copied the conv result to s_conv_out (core 1
   consumed the zero-initialized buffer) — fixed with the in-place result move (in the conv_pump
   commit e36b545-era, refined later).
2. Volume inconsistency: the conv hand-off missed its 250 us deadline 36x/window (core-0 bursts:
   the C3 TELEM replies + the report printer) and EVERY miss bypassed with the LOUD raw mono-sum
   vs the 0.25-gain conv path. Fixed: level-matched bypass (speaker_bypass_gain = sqrt(sum ir^2)
   * conv gain), 3-slot ping-pong, budget-aware waits (761c290).
3. Denormal storm (max-Process spikes to 1.7 ms): FPSCR FZ/DN must be set on BOTH cores (FPSCR is
   per-core; setup1() got the audio-core write) (955728e/805ffa3).
4. The CMSIS-DSP question answered: we already vendor it; arm_fir_f32 on M33 = the same acc chain
   as our C loop (our cmsis builds at plain -O3); the REAL lever is the book ch.19.4 int16
   dual-MAC (SMLAD/SMLALD = 1.5 cycles/MAC).
5. The repo restructure: the patch system + sync script REMOVED (the libs are the source of
   truth), ONE env `rack` (boots Elements), 34 stale trees deleted (cdb3206/0450e7a/3a8aac4).
6. Board docs: docs/rp2350-board.md (NoLogo Mini USB RP2350, fbc7451).
7. The rack renamed: rings_dist.h -> rack_dist.h, namespace rackdist (bacf4e7).

**The q15 SMLAD conv (the current uncommitted-then-committed state)**:
- tools/ir2array.py emits BOTH the float IR (speaker_bypass_gain + the reference) AND the q15
  packed pairs kSpeakerIrQ15[256] (uint32: lo = ir16[2j], hi = ir16[2j+1], chronological pairs),
  scaled so sum|ir16| = 32700 <= 32700 (the int32 acc cannot overflow: |acc| <= 32700*32767 <
  2^31; Yiu ch.19.5). kSpeakerQ15Norm = 1/(scale*32767) converts acc -> the float domain (the
  unity conv; kSpeakerConvGain 0.25 applies after). GOTCHA fixed: the norm was first printed as
  sum|h|/(scale*32767) — off by sum|h|.
- rack_dist.h SpeakerConv: hist_ (float) -> hist16_ (int16, the v stream clamped to q15); the
  conv loop: per output 256 packed dual-MACs, the x word assembled from 2 int16 loads
  (lo = the newer v16, hi = the older) — the halves pair DIRECTLY with the chronological ir word
  via plain SMLAD (NOT SMLADX — the first SMLADX attempt crossed the tap pairs, caught by the
  naive law, err 0.33).
- **The Windows toolchain GAS GAP: `smlad` does not assemble AT ALL** ("bad arguments" for ANY
  operands, armv7e-m and armv8-m alike — `smuad r3,r0,r1` works, `smlad r0,r0,r0` fails;
  binutils 2.43.1 Windows build). The equivalent SMUAD + add form is used (the same 1.5
  cycles/MAC as the book's SMLALD count). ALSO: armv8-M SMLAD has the Rd != Rn/Rm constraint
  (a breaking change vs armv7E-M — M4 allowed Rd == Rn) — the inline asm needs the early-clobber
  "=&l" + "l" constraints so gcc never allocates the dest over the inputs.
- test_speaker_ir.cc: the naive reference REWRITTEN to the q15 int64 definition (v16 quantized
  with the same clamp; y16 = the int64 sum * norm * gain) — the impl vs the naive must be
  BIT-EXACT (the int32 acc is provably in range). 13/13 host suite GREEN with the SMUAD form.

**STATE: NOT flashed. The board still runs the pre-q15 float conv (flashed at the 761c290-era
build).** The tree builds clean for the firmware (the last full pio build pre-dates the final
SMUAD rename — rebuild before flashing).

**Next steps on the Linux machine (in order)**:
1. `pio run -d rack-rp2350 -e rack` (the single env) — verify SUCCESS.
2. `pio run -d rack-rp2350 -t upload --upload-port <port>` — flash.
3. Engage: cc 42 5 / cc 43 110 / cc 48 100 / cc 20 100; strike via cc 64 edges or the button.
4. Read the `ins=a/b cv=c/d m=n` tail: **cv= should drop from ~110-117 us to ~25-40 us** (the
   q15 dual-MAC at ~1.5-2 cycles/MAC + the packing). miss= expected 0.
5. Ears: the cab tone should be UNCHANGED (the q15 quantization: the 12-bit effective
   coefficients — inaudible under the chainsaw).
6. If the cv= does not drop or miss > 0: the SMLAD asm on Linux GAS (which HAS a working smlad)
   could use the REAL SMLAD opcode instead of the SMUAD+add — swap the asm string, re-measure.
- Levers left after this: 1024-tap @ 16 kHz (64 ms window, the same cost as the current 512),
  the SMLALD 64-bit accumulate (no coefficient scaling — the full 15-bit coefficients), or the
  FFT overlap-add (the clouds STFT pattern, ~10-15 us amortized) for multi-second IRs.

## OPTIMIZATION TECHNIQUES INVENTORY — the swede cab chain (consolidated 2026-11-19)

All techniques used to bring the swede chain from "starving" (avg 671-891 us, crackles/silence)
to 466 -> 316 us avg with the conv offloaded (the q15 version pending HW measure). Each: what /
where / measured effect.

1. **FPU denormal control (FPSCR FZ+DN)** — main.cpp setup() AND setup1() (FPSCR is per-core;
   both cores need it). The M33 defaults to FULL denormal support: the conv/FMV/biquad states
   decaying into the denormal range cost ~30 cycles/hit -> recurring 1.7 ms render spikes.
   Effect: -1.2 ms spikes. Audio DSP never needs subnormals.
2. **SRAM-resident hot code (tc_* overlay)** — RACK_DIST_MARK/ELEMENTS_RAM_CODE land the dist
   dispatch + cab IR table in the tc_elements SRAM window (57,728/60,416 B slot; XIP flash
   cache-stall elimination for the whole swede chain incl. the FIR loops).
3. **q15 int16 packed dual-MAC (SMLAD-family, Yiu ch.19.4)** — the conv IR quantized to int16
   pairs packed in uint32 words (kSpeakerIrQ15), the v stream int16 (hist16_); per pair ONE
   dual-MAC instruction = 2 MACs. Sum|ir16| <= 32700 -> the int32 accumulator provably cannot
   overflow at full-scale int16 input (ch.19.5); the output rescaled via kSpeakerQ15Norm.
   Expected: the conv 110-117 -> ~25-40 us. HW measure pending (Linux).
   - The pairs pair DIRECTLY with plain SMLAD (lo=the newer x, hi=the older; ir chronological
     lo=ir[2j], hi=ir[2j+1]). The first SMLADX attempt CROSSED the tap pairs — caught by the
     naive law (err 0.33).
   - The Windows GAS has NO working `smlad` opcode (any operands, any arch) while `smuad`
     assembles — the SMUAD + add form is used at the same 1.5 cycles/MAC. armv8-M also imposes
     Rd != Rn/Rm (a breaking change vs armv7E-M) -> the inline asm needs "=&l" + "l" constraints.
   - On Linux GAS `smlad` works — the asm string may be swapped back to the real SMLAD.
4. **4-accumulator float unroll** (the pre-q15 float conv) — the M33 FPU VMLA has 3-cycle
   latency/1-cycle throughput; a single serialized acc = 3 cycles/MAC (gcc cannot reassociate
   float without -ffast-math). 4 independent accumulators -> ~1 MAC/cycle. Superseded by the q15
   path but the technique stands for any float dot-product.
5. **Sample-rate decimation of the conv** — the IR is band-limited to 8 kHz (0.42% energy above;
   a V30 cab has nothing up there): 31-tap halfband dec -> 512-tap FIR @ 16 kHz -> halfband
   interp. MACs halved vs 512 taps @ 32 kHz, the window DOUBLED to 32 ms. The halfband's odd
   interp phase is a FREE delayed passthrough (only the 14-tap even phase costs).
6. **Pre-delay compaction** — distant mics put 10-30 ms of SILENCE before the IR transient
   (the Hesu IR: the peak at 27.4 ms). Skip to 3% of the peak minus 1 ms: the taps are spent on
   the body, not the air. (ir2array.py does it automatically.)
7. **Energy-based truncation** — 512 taps = 98.8% of the compacted body energy (the measured
   cumulative table: 384 = 98.0%, 768 = 99.7%, 1024 = 99.9%). Truncating BELOW the transient
   start costs nothing; above, use the cumulative-energy table, not the tap count.
8. **Core-0 offload (DMB ping-pong)** — the conv runs on core 0's loop() (2->3 slots, ~23% core-0
   load); core 1 publishes the post-FMV mono and consumes the PREVIOUS block's result one block
   later (+500 us, inaudible). __dmb() around EVERY flag flip (M33 is weakly ordered; volatile
   alone is not a barrier). The conv result is written IN-PLACE into the first arg — it MUST be
   copied to the out slot (the original silence bug).
9. **Level-matched bypass** — on a core-0 miss the fallback emits the mono pre-conv signal x
   speaker_bypass_gain() (= sqrt(sum ir^2) x conv gain) — the SAME loudness as the conv path, so
   a miss is a brief timbre dip, never a volume jump. (The unmatched bypass was the "volume
   inconsistent" bug: 36 misses/window x a ~4x level jump.)
10. **Budget-aware bounded waits** — the hand-off waits are bounded by the elapsed time since
    the insert start (400/460 us of the 500 us period), not absolute 250+250 which could overrun
    the block on a miss.
11. **True-band HM-2 redesign at 32 kHz** — the reference plugin's fixed-digital coefficients
    were designed at 44.1 kHz: at 32 kHz every band sat 0.726x low. Solving (f0, gain, Q) from
    them (round-trip-exact RBJ) and redesigning at 32 kHz restored 86 Hz/+900 Hz/1400 Hz.
12. **Bit-exact host validation of DSP instructions** — the portable emu of SMLADX is
    semantically identical to the hardware instruction, so the host tests validate the board's
    algorithm without hardware. The int64 naive reference (sum ir16*v16) is bit-exact vs the
    int32 acc (proven in range).
13. **dbg_printf observability** — avg + per-stage counters in the 2 s report (proc avg,
    ins=avg/max, cv=avg, miss count) + the runtime conv gate (CC 48): the cost breakdown without
    reflashing. GOTCHAs fixed: the dbg line buffer was 400 B (the tail silently truncated ->
    560); newlib-nano vsnprintf has NO %f support (integer hundredths instead).

## q15 conv kernel on HW — 3 stacked causes, ldmdb stride bug, then SMLALD full-scale (2026-11-19, Linux session)

The handoff's next-steps executed (Linux machine). Audio **USER-ACCEPTED** at the end ("ok, now
it sounds fine"). The measured arc of `cv=` (core-0 conv cost per 16-sample block):

| build | cv= | verdict |
|---|---|---|
| float 4-acc (761c290 era) | 110-117 us | accepted baseline |
| q15 SMUAD+add (9d0d2ba) | 202-263 us | WORSE — three stacked causes (below); user ear-accepted anyway ("sounds fine" was casual) |
| register-pinned SMLAD kernel (3e7ef20-era) | 79-83 us | NOISE bug (ldmdb, below) — fixed in 8f8060c -> 83 |
| SMLALD 64-bit + full-scale coefficients (5582716) | **86 us** | **clean + accepted** |

**Cause 1 — the q15 IR table was XIP-resident.** 9d0d2ba added `kSpeakerIrQ15` WITHOUT a
RACK_DIST_MARK; the float `kSpeakerIr` had one. 2048 sequential word loads per block from XIP
flash. Fix: `RACK_DIST_MARK(dist_speaker_ir_q15)` (now tc_elements SRAM, adjacent to the float
table, no overlap — verified by nm).

**Cause 2 — the SMUAD+add form made gcc SPILL the accumulator to the stack every iteration**
(`ldr r3,[sp,#8] / add / str` around each MAC; ~12 instructions per 2 MACs). The `=&l` fresh-dest
constraint forbids reusing acc's register as the destination. Fix: accumulate-form asm.

**GAS FACTS (binutils 2.4x — Windows 2.43.1 AND Linux 2.47):** the 3-operand Thumb `smlad`/
`smladx` DO NOT PARSE ("bad arguments", any arch) — the handoff's "on Linux GAS smlad works"
was wrong. The 4-operand form `smlad Xd, Xn, Xm, Xa` (T32 requires Xa == Xd) assembles — the
same form gcc 16's arm_acle `__smlad(a, b, acc)` emits. `pkhtb` QUIRK: the no-shift
`pkhtb Rd, Rn, Rm` canonicalizes to the PKHBT encoding with operands SWAPPED (top/bottom roles
invert silently) — always spell `pkhbt`. `smlald/smlaldx` parse fine in their native 4-operand
form. `pkhbt`/`smuad` parse fine.

**Cause 3 — the x word was assembled per pair** (2 halfword loads + orr inside the MAC loop).
Fix: hand-written kernel with REGISTER-PINNED operands (CMSIS q15-FIR pattern): gcc spilled one
of the two descending pointers to the stack in the C-loop version and burned ~6 movs shuffling
asm operands. Final kernel = 8 instructions per 4 MACs: `ldr ir` (post-inc) / `ldr b`
(post-dec) / `ldr a` (plain) / `smlaldx` (even output) / `pkhbt` (straddle re-pack) / `smlald`
(odd output) / subs / bne. Two outputs per iteration off packed W words; even-j pairs via the
SMLALDX swap, odd-j via pkhbt (the parity split the first SMLADX-everywhere attempt got wrong).

**The ldmdb NOISE bug (8f8060c):** replacing the two pointer loads with ONE `ldmdb` of two regs
slides the base 8 B/iter — but the FIR window slides ONE WORD/iter (consecutive tap pairs
OVERLAP: b of iter t = a of iter t+1's neighbor). Every tap after the first summed a pair
shifted by an accumulating extra word. Symptom: constant background noise on EVERY dist algo
(the conv output REPLACES the audio while CC 48 is on) while warps stayed CLEAN (bypasses the
dist insert). Lesson: adjacent-word loads via ldm only work when the window stride equals the
load count.

**The DIRTY bug (5582716):** noise gone but "sounds dirty" — NOT starvation (playing counters
clean: max 188-213 us, stalls 0, miss=0; the 739 us max in the transcripts was the console CC +
preset-PART burst on core 0, not playback). Root cause: the int32-SMLAD scaling
(sum|ir16| <= 32700, ch.19.5) squeezed the coefficients to ~12 bits = ~-44 dB quantization
noise riding the cab output. The host test STRUCTURALLY could not catch it: the naive reference
is q15 too (bit-exact impl-vs-naive, not q15-vs-float audio). Fix: 64-bit accumulate
(SMLALD/SMLALDX register pairs; C int64 on host) -> NO scaling cap -> full 15-bit coefficients
(peak at 32767; 361961 x 32767 ~= 1.2e10 << 2^63) -> ~11x less quantization noise. ir2array.py
now emits the full-scale table. cv 83 -> 86 (+3 for the 64-bit acc). Ears: ACCEPTED.
(Levers left if ever needed: 1024-tap @ 16 kHz same cost; FFT overlap-add for long IRs.)

**Plumbing note:** the residual cv - 8 (kernel ~34 us @ 240 MHz + hist16_ 511-halfword shift
~8 us + Ws scratch build ~5 us + dec/interp ~6). The shift/scratch are the next optimizations
if the conv ever needs to shrink below ~60 us.

**Host-suite note:** `test_reverb` is FLAKY at HEAD (fails ~2/6 runs, "impulse response does
not decay" with DIFFERENT peak values per run — uninitialized state somewhere in the FxEngine
test path; matches the documented FxEngine power-of-two wrap issue). `test_reverb_rack`/
`test_reverb_upstream` binaries in test/build/ are stale and not executable (the suite loop
reports FAIL spuriously). Unrelated to the conv work.

## Warps mix: dry right channel — aux now crossfades to the wet (2026-11-19, evening)

USER REPORT: turning W MIX from 0, the warps wet came out LEFT and the untreated resonator
RIGHT. Root cause: the T9 stage (part.inc) crossfaded ONLY main — aux never touched the wet;
w_out.r is the vendored Modulator's aux path (not the cross-mod), so using it was never an
option. Fix: blend aux toward the SAME wet (mono warps out on both channels — the user's spec:
"only warps out, both channels"; at mix 0 both channels stay dry). Raw exciter still never
reaches aux. test_warps_stage laws updated: staged aux == staged main at space 0 / mix 1
(bit-exact), staged aux != base aux, aux-halving at w=0. All warps tests green. Flashed;
**USER-ACCEPTED** ("wait now its working" — first test ran against stale state, second confirmed).
