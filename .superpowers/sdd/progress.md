# SDD Progress Ledger — Elements RP2350 + PCM5102A

Plan: `docs/superpowers/plans/2026-10-30-elements-rp2350.md`
Started: 2026-10-30. Board: RP2350 nano at `/dev/ttyACM0`.

| Task | Status | Committed | Notes |
|---|---|---|---|
| T1 scaffolding | done | 00c29a7 | fallback lib-sync strategy; board id `rpipico2` (plan's `pico2` unknown to platform); see findings below |
| T2 I2S bring-up | code ready | | build SUCCESS (32 kHz sine via `write16(l,r)`); audible 220 Hz acceptance pending user wiring + BOOTSEL flash; see T2 note below |
| T3 engine integration | pending | | |
| T4 USB MIDI device | pending | | Adafruit_USBD_MIDI API facts below |
| T5 USB host MIDI | pending | | needs USB-A socket wired; midi_host.h API facts below (index-based, differs from plan sketch) |
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
