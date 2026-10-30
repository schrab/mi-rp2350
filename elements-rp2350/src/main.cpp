// Task 4 — USB MIDI device: core 0 polls Adafruit_USBD_MIDI event packets
// and applies notes/CCs to the shared engine state (parser in midi.h).
// Task 5 — USB MIDI host: PIO-USB bit-bangs a second USB controller on
// GP14/GP15 (rhport 1, native USB-C stays the device on rhport 0); core 0
// polls tuh_midi_packet_read() over the mounted interface indexes and feeds
// the same 4-byte event packets into midiHandleEventPacket().
// Part::Process() still renders on core 1 at 32 kHz into the PCM5102A via I2S.
// Task 6 — hardening: last-note-priority note stack, resonator-model CC 28
// (applied by core 1 between render blocks via pending_resonator_model),
// stuck-gate watchdog (30 s), I2S stall counter in the 2 s serial report,
// USB product name "Elements RP2350". The T3 auto-trigger test is gone
// (MIDI verified).
// Status LED: WS2812 on GP22 via PIO2 SM0 — dark when idle, amber while a
// note is gated, with the exciter (turquoise) and resonator (rose/fuchsia)
// meters blended in using the original Elements LED colors. 3 green blinks
// when a USB-A MIDI controller mounts. Blocks core 0 ~5% of the time (1 ms
// per 20 ms poll); no impact on core 1 audio.
//
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

#include "status_led.h"          // WS2812 status LED on GP22 (before midi.h)
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

// Status LED instance — defined here (midi.h holds only an extern reference).
StatusLed statusLed;

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
// Meter levels for the status LED (core 1 writes after Process, core 0 reads).
// Part's meters already include the original fast-attack / slow-release
// smoothing, so the LED follows the same ~31 ms (exciter) / ~62 ms (resonator)
// decay tails. 32-bit aligned reads are atomic on Cortex-M33.
volatile float shared_exciter_level = 0.0f;
volatile float shared_resonator_level = 0.0f;
// CC 28 hand-off: core 0 stores the requested model (0..2), core 1 applies
// part.set_resonator_model() between render blocks and resets to -1.
volatile int8_t pending_resonator_model = -1;

static volatile bool audio_ready = false;

// --- perf telemetry (core1 writes, core0 prints) ---
static volatile uint32_t proc_us_max = 0;
// Count of render loops whose 16 I2S writes took > 900 us (normal worst case
// is ~500 us = one 16-frame block @ 32 kHz; see loop1()).
static volatile uint32_t i2s_stall_count = 0;

// --- USB MIDI device (core 0) ---
Adafruit_USBD_MIDI usb_midi;

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
    statusLed.NotifyControllerConnected();  // 3 green blinks
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
  // Apply at most one pending resonator-model change (from core 0's CC 28).
  // Doing it here keeps the change strictly between two Process() calls;
  // Part::Process() re-pushes the model into the voice every block
  // (part.cc:181) and Voice only branches on it (voice.cc:212), so nothing
  // else needs to be reset on a model change.
  int8_t m = pending_resonator_model;
  if (m >= 0) {
    pending_resonator_model = -1;
    part.set_resonator_model(static_cast<elements::ResonatorModel>(m));
  }

  perf_state.gate = shared_gate;
  perf_state.note = shared_note;
  perf_state.strength = shared_strength;
  perf_state.modulation = shared_modulation;

  uint32_t t0 = time_us_32();
  part.Process(perf_state, blow_in, strike_in, main_out, aux_out, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Publish the metering values after the block is rendered. Poll() on core 0
  // turns them into LED brightness; they carry the original meter smoothing.
  shared_exciter_level = part.exciter_level();
  shared_resonator_level = part.resonator_level();

  // The writes pace loop1: one iteration = exactly one 16-word DMA buffer,
  // and buffers free every 500 us (16 frames @ 32 kHz), so the write block
  // normally waits at most ~500 us for the next buffer to free. > 900 us
  // means no buffer freed for ~2 block periods -> I2S output stalled.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    int16_t l = stmlib::Clip16(static_cast<int32_t>(main_out[i] * 32768.0f));
    int16_t r = stmlib::Clip16(static_cast<int32_t>(aux_out[i] * 32768.0f));
    i2s.write16(l, r);   // blocking write, one L/R frame; paces loop at 32 kHz
  }
  if (time_us_32() - w0 > 900) {
    ++i2s_stall_count;
  }
}

// ================= core 0: control =================
void setup() {
  // USB product name (T6): the core's TinyUSB_Device_Init() ran before setup()
  // with the board default product string ("Pico 2"). setProductDescriptor()
  // only swaps the string pointer served on GET_DESCRIPTOR(String) requests
  // (Adafruit_USBD_Device.cpp:133), so setting it here — before the
  // detach/attach re-enumeration guard further down — makes the host pick up
  // "Elements RP2350" when it re-reads the descriptors after re-enum.
  TinyUSBDevice.setProductDescriptor("Elements RP2350");
  Serial.begin(115200);
  initEngine();
  statusLed.Init();  // 3 white self-test flashes, then dark

  // --- USB host (PIO-USB, rhport 1): MUST init BEFORE i2s.begin() ---
  // PIO-USB hard-claims its config's PIO/SMs (pio_sm_claim panics on a taken
  // SM) and loads 5 (tx) + 10 (nrzi decoder) + 17 (edge detector/EOP) = all
  // 32 PIO0 instruction slots, plus DMA channel 0 and its own 1 ms SOF alarm
  // pool. If I2S began first and grabbed PIO0, pio_usb would panic at boot.
  // I2S's PIOProgram instead flexibly allocates a free PIO+SM at begin().
  // The status LED claims PIO2 SM0 in statusLed.Init(), so I2S lands on
  // PIO2 SM1 (GP6/7/8 are reachable from any PIO on the RP2350).
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
    statusLed.NotifyBootError();  // PANIC blink pattern
  } else {
    Serial.println("I2S up");
  }

  // Boot diagnostic: confirm I2S did not steal the status LED's PIO2 SM0.
  // SM0's PC must stay inside the 4-instruction WS2812 program.
  Serial.printf("PIO2 SM0 claimed=%u pc=%u (ws2812 @%u) | SM1 claimed=%u pc=%u\r\n",
                (unsigned)pio_sm_is_claimed(pio2, 0), (unsigned)pio_sm_get_pc(pio2, 0),
                (unsigned)statusLed.Offset(),
                (unsigned)pio_sm_is_claimed(pio2, 1), (unsigned)pio_sm_get_pc(pio2, 1));

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

  // Status LED: dark when idle, amber while a note is held, exciter/resonator
  // meters blended in, green blinks on controller mount. Blocks ~1 ms per
  // update for the WS2812 reset gap.
  statusLed.Poll(shared_gate, shared_exciter_level, shared_resonator_level);

  static uint32_t last_report = 0;
  uint32_t now = millis();

  // --- stuck-gate watchdog (T6, defensive) ---
  // A lost note-off (e.g. controller unplugged mid-note) would leave the
  // voice gated forever. shared_gate can only become true inside
  // midiNoteOn(), which also stamps last_note_event_ms, so
  // "gate && 30 s since the last note event" means genuinely stuck. Forcing
  // the gate off ends the incident, so this logs once per incident.
  if (shared_gate && now - last_note_event_ms > 30000UL) {
    shared_gate = false;
    note_stack.clear();
    Serial.println("WARNING: gate stuck >30s (lost note-off?) - forced off, note stack cleared");
  }

  // One-line diagnostics report (T6): worst render time + I2S stall counter.
  // i2s stalls is cumulative; (+n) is the delta since the previous report.
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u)\r\n",
                  proc_us_max,
                  (unsigned)(1000000UL / kSampleRate * kBlockSize),
                  stalls, stalls - last_stall_total);
    last_stall_total = stalls;
    proc_us_max = 0;
  }
}
