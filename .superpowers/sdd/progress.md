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

- Board enumerated as `2e8a:0009` "Betaflight RP2350A" (factory firmware), serial
  `/dev/ttyACM0` (user is in `uucp` group, tty access OK).
- `pio run -t upload` = 1200 bps touch + wait + `picotool load`. The Betaflight
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
  side line-read race, pad's no-VBUS wait state), and the next-steps plan
  (WDT + scratch-register crash capture): **`docs/HANDOFF-usb-host-freeze.md`**
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
| T4a Presets (serialization + console dump/load + last-state autosave) | pending | — | bank lands on C3 flash in T5 |
| T5 C3 UI + UART link (after DSP acceptance AND hardware) | pending | — | encoder+button menus, HC4067, ST7789, BLE-MIDI, preset bank |
| T6 Chain mode v2 (plaits@32k patch, pair table) | done (2 of 3 pairs) | 972c58a + c106458 | elements→clouds (52% of 750 µs) AND rings→clouds (36-48%) benched; runtime-rate plaits patch; clouds dead-mode demotion freed 22.7 KB; plaits→clouds = chain-tier decision pending (findings); user dropped Plaits→Elements |
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
