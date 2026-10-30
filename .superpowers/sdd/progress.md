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
| T6 hardening + docs | pending | | |

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
