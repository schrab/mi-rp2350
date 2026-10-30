// Task 4 — USB MIDI device: core 0 polls Adafruit_USBD_MIDI event packets
// and applies notes/CCs to the shared engine state (parser in midi.h).
// Task 5 — USB MIDI host: PIO-USB bit-bangs a second USB controller on
// GP14/GP15 (rhport 1, native USB-C stays the device on rhport 0); core 0
// polls tuh_midi_packet_read() over the mounted interface indexes and feeds
// the same 4-byte event packets into midiHandleEventPacket().
// Part::Process() still renders on core 1 at 32 kHz into the PCM5102A via I2S.
// The T3 auto-trigger bring-up test stays in loop() behind kAutoTriggerTest
// (false) until the MIDI path is confirmed on hardware; deleted in T6.
//
// Wiring (PCM5102A): BCK -> GP6 (LRCK/WS follows on GP7 automatically),
// DIN -> GP8, SCK/MCLK -> GND, XSMT -> 3V3 (unmute), FLT/DEMP/FMT -> GND.
//
// Wiring (USB-A socket, USB host port for the MIDI controller — PIO-USB):
//   VBUS -> 5V pin (hardware power for the peripheral, NOT GPIO-switched),
//   D+   -> GP14  (PIO-USB requires D- = D+ + 1),
//   D-   -> GP15,
//   GND  -> GND (shield to GND too).
#include <Arduino.h>
#include <I2S.h>
#include <Adafruit_TinyUSB.h>

#include "pio_usb.h"             // Pico-PIO-USB (vendored in lib/pio_usb/)

#include <STMLIB.h>
#include <ELEMENTS.h>

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (not pulled in by the wrappers)
#include "elements/dsp/dsp.h"
#include "elements/dsp/part.h"

#include "midi.h"                // shared parser + extern shared-state decls

// Firmware-side definitions of the elements::Dsp statics. They are declared as
// class statics in ELEMENTS/src/elements/dsp/dsp.h but defined nowhere in the
// library.
float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

static const uint32_t kSampleRate = 32000;
static const size_t kBlockSize = elements::kMaxBlockSize;

static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);

// --- engine ---
static uint16_t reverb_buffer[32768];   // 64 KB, verified FxEngine<32768>
// Non-static: declared extern in midi.h (parser writes patch CCs via
// mutable_patch()). All other engine objects stay file-local.
elements::Part part;
static elements::Patch* patch;
static elements::PerformanceState perf_state;

static float blow_in[kBlockSize];
static float strike_in[kBlockSize];
static float main_out[kBlockSize];
static float aux_out[kBlockSize];

// --- shared MIDI state (core0 writes, core1 reads) ---
// Non-static: these are the definitions for the externs declared in midi.h.
volatile float shared_note = 48.0f;
volatile float shared_modulation = 0.0f;
volatile float shared_strength = 0.8f;
volatile bool shared_gate = false;

static volatile bool audio_ready = false;

// --- perf telemetry (core1 writes, core0 prints) ---
static volatile uint32_t proc_us_max = 0;

// --- USB MIDI device (core 0) ---
Adafruit_USBD_MIDI usb_midi;
// T3 bring-up test, disabled until the user confirms MIDI on hardware
// (plan T6 deletes it).
static const bool kAutoTriggerTest = false;

// --- USB MIDI host (PIO-USB on rhport 1, core 0) ---
// Pattern from the bundled DualRole/serial_host_bridge example (device =
// native rhport 0, host = PIO-USB rhport 1). Adapted: the example runs
// USBHost.task() on core 1 to spread load, but core 1 here renders audio,
// so the host task + MIDI poll run in core 0's free-running loop().
Adafruit_USBH_Host USBHost;

// Mounted host MIDI interface indexes, tracked by tuh_midi_mount_cb /
// tuh_midi_umount_cb. Callbacks fire from USBHost.task() context (core 0
// loop) — same context as pollHostMidi(), no concurrency.
// Host MIDI API is INDEX-based (tuh_midi_itf_get_index maps daddr+itf ->
// idx); callbacks hand us the idx directly.
static constexpr uint8_t kHostMidiMaxIdxs = 4;
static uint8_t host_midi_idxs[kHostMidiMaxIdxs];
static uint8_t host_midi_idx_count = 0;

extern "C" {
// Invoked when a device with a MIDI streaming interface is mounted.
void tuh_midi_mount_cb(uint8_t idx, const tuh_midi_mount_cb_t *mount_cb_data) {
  bool known = false;
  for (uint8_t i = 0; i < host_midi_idx_count; ++i) {
    if (host_midi_idxs[i] == idx) { known = true; break; }
  }
  if (!known && host_midi_idx_count < kHostMidiMaxIdxs) {
    host_midi_idxs[host_midi_idx_count++] = idx;
  }
  Serial.printf("Host MIDI mounted idx=%u daddr=%u cables=%u\r\n", idx,
                mount_cb_data->daddr, mount_cb_data->rx_cable_count);
}

// Invoked when the MIDI interface is unmounted (unplugged / bus reset).
void tuh_midi_umount_cb(uint8_t idx) {
  for (uint8_t i = 0; i < host_midi_idx_count; ++i) {
    if (host_midi_idxs[i] == idx) {
      host_midi_idxs[i] = host_midi_idxs[--host_midi_idx_count];
      break;
    }
  }
  Serial.printf("Host MIDI unmounted idx=%u\r\n", idx);
}
} // extern "C"

// Drain pending 4-byte USB-MIDI event packets from every mounted host MIDI
// interface. Packet layout matches the device side (pkt[0] = cable<<4|CIN,
// pkt[1] = status, pkt[2..3] = data) -> same midiHandleEventPacket() entry.
static void pollHostMidi() {
  for (uint8_t i = 0; i < host_midi_idx_count; ++i) {
    const uint8_t idx = host_midi_idxs[i];
    if (!tuh_midi_mounted(idx)) continue;
    uint8_t pkt[4];
    while (tuh_midi_packet_read(idx, pkt)) {
      midiHandleEventPacket(pkt);
    }
  }
}

static void initEngine() {
  elements::Dsp::setSr(kSampleRate);

  memset(reverb_buffer, 0, sizeof(reverb_buffer));
  memset(blow_in, 0, sizeof(blow_in));
  memset(strike_in, 0, sizeof(strike_in));

  part.Init(reverb_buffer);
  // Seed word count matches the field-proven Scarp sketch, but as a real
  // 3-word array — Seed() hashes all `size` words it is given.
  uint32_t seed[3] = {0x1fff7a10, 0x1fff7a10, 0x1fff7a10};
  part.Seed(seed, 3);
  part.set_easter_egg(false);
  part.set_resonator_model(elements::RESONATOR_MODEL_STRING);

  patch = part.mutable_patch();
  // sensible default: strike/mallet voice
  patch->exciter_envelope_shape = 0.1f;
  patch->exciter_bow_level = 0.0f;
  patch->exciter_blow_level = 0.0f;
  patch->exciter_strike_level = 0.8f;
  patch->exciter_blow_meta = 0.5f;
  patch->exciter_strike_meta = 0.5f;
  patch->exciter_bow_timbre = 0.5f;
  patch->exciter_blow_timbre = 0.5f;
  patch->exciter_strike_timbre = 0.5f;
  patch->exciter_signature = 0.0f;
  patch->resonator_geometry = 0.25f;
  patch->resonator_brightness = 0.4f;
  patch->resonator_damping = 0.7f;
  patch->resonator_position = 0.3f;
  patch->space = 0.4f;
}

// ================= core 1: audio =================
void setup1() {
  while (!audio_ready) { delay(1); }
}

void loop1() {
  perf_state.gate = shared_gate;
  perf_state.note = shared_note;
  perf_state.strength = shared_strength;
  perf_state.modulation = shared_modulation;

  uint32_t t0 = time_us_32();
  part.Process(perf_state, blow_in, strike_in, main_out, aux_out, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  for (size_t i = 0; i < kBlockSize; ++i) {
    int16_t l = stmlib::Clip16(static_cast<int32_t>(main_out[i] * 32768.0f));
    int16_t r = stmlib::Clip16(static_cast<int32_t>(aux_out[i] * 32768.0f));
    i2s.write16(l, r);   // blocking write, one L/R frame; paces loop at 32 kHz
  }
}

// ================= core 0: control =================
void setup() {
  Serial.begin(115200);
  initEngine();

  // --- USB host (PIO-USB, rhport 1): MUST init BEFORE i2s.begin() ---
  // PIO-USB hard-claims its config's PIO/SMs (pio_sm_claim panics on a taken
  // SM) and loads 5 (tx) + 10 (nrzi decoder) + 17 (edge detector/EOP) = all
  // 32 PIO0 instruction slots, plus DMA channel 0 and its own 1 ms SOF alarm
  // pool. If I2S began first and grabbed PIO0, pio_usb would panic at boot.
  // I2S's PIOProgram instead flexibly allocates a free PIO+SM at begin() and
  // lands on PIO1 (GP6/7/8 are reachable from any PIO on the RP2350).
  if (F_CPU % 12000000UL) {
    // Same guard as the bundled DualRole usbh_helper: PIO-USB bit-banging
    // needs a CPU clock that is a multiple of 12 MHz. Build uses 240 MHz.
    Serial.println("ERROR: CPU clock not a multiple of 12 MHz - PIO-USB host timing unreliable");
  }
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = kPinHostDP;    // D+ = GP14, D- = GP15 (DPDM pinout)
  if (!USBHost.configure_pio_usb(1, &pio_cfg)) {
    Serial.println("PIO-USB configure FAILED (host driver missing for rhport 1?)");
  }
  if (!USBHost.begin(1)) {
    Serial.println("USB host begin FAILED");
  }

  i2s.setBitsPerSample(16);
  i2s.setBuffers(8, 16);          // 8 buffers x 16 words = 128 stereo frames (~4 ms)
  i2s.setFrequency(kSampleRate);
  if (!i2s.begin()) {
    Serial.println("I2S begin FAILED");
  } else {
    Serial.println("I2S up");
  }

  // USB MIDI device: the core already ran TinyUSB_Device_Init(0) before
  // setup() (cores/rp2040/main.cpp), so register the MIDI interface now and
  // force re-enumeration if the host already configured the CDC-only device
  // (same guard as the bundled midi_test example) — otherwise the new
  // interface would not appear until the next reconnect.
  usb_midi.setStringDescriptor("Elements RP2350");
  if (!usb_midi.begin()) {
    Serial.println("USB MIDI begin FAILED");
  }
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  audio_ready = true;
  Serial.println("Elements engine up (USB MIDI device + PIO-USB host)");
}

void loop() {
  // --- USB host: service the host stack, then poll host MIDI ---
  // task() drains the TinyUSB host event queue (non-blocking on non-OS);
  // mount/umount callbacks fire from here. Data arrives via pio_usb's 1 ms
  // SOF alarm; pollHostMidi() only pulls from the driver's RX FIFOs.
  USBHost.task();
  pollHostMidi();

  // --- USB MIDI device: drain all pending 4-byte event packets ---
  // readPacket() shares state with the Stream API (per midi_device.h) — we
  // use only the packet API, so no mixing.
  uint8_t pkt[4];
  while (usb_midi.readPacket(pkt)) {
    midiHandleEventPacket(pkt);
  }

  static uint32_t last_report = 0;
  uint32_t now = millis();

  if (kAutoTriggerTest) {
    // T3 bring-up test: strike every 1.5 s, gate held 300 ms. Kept behind a
    // compile-time flag until MIDI is confirmed on hardware (removed in T6);
    // with the flag false the block compiles away entirely.
    static uint32_t last_trigger = 0;
    static uint8_t step = 0;
    if (now - last_trigger > 1500) {
      last_trigger = now;
      shared_note = 36.0f + (step++ % 12);
      shared_strength = 0.9f;
      shared_gate = true;
    }
    if (now - last_trigger > 300) {
      shared_gate = false;
    }
  }

  // Process-time report stays always-on (useful for T6 diagnostics too).
  if (now - last_report > 2000) {
    last_report = now;
    Serial.printf("max Process: %u us (budget %u us)\n", proc_us_max, (unsigned)(1000000UL / kSampleRate * kBlockSize));
    proc_us_max = 0;
  }
}
