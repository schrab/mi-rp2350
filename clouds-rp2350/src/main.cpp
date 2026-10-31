// Clouds RP2350 — Mutable Instruments Clouds granular processor on the
// RP2350 nano + PCM5102A, using the proven elements-rp2350 architecture:
// core 0 = USB MIDI device (CDC + USB-MIDI) + PIO-USB host MIDI + status LED
// + diagnostics; core 1 = audio render at 32 kHz into the I2S DAC.
//
// The module is an EFFECT: it granulates an input signal. This hardware has
// no audio input jack — standalone, a MIDI-triggered test exciter (burst
// pluck or gated tone) feeds Clouds' input; in future multi-module builds
// the exciter is replaced by the upstream module's block FIFO (RINGS/PLAITS/
// ELEMENTS out -> Clouds in).
//
// Clouds core (recon in docs/superpowers/plans/2026-10-31-clouds-rp2350.md):
// GranularProcessor::Init(large, small) mirrors the original F405 two-region
// split: stereo 16-bit sample memory = the SMALL buffer for both channels
// (64 KB -> 32768 samples/ch = 1.02 s @ 32 kHz, same as the hardware), the
// LARGE buffer holds mono/low-fidelity sample memory + the FX workspace
// (diffuser, reverb, WSOLA correlator). kMaxBlockSize = 32 samples = 1 ms;
// native rate 32 kHz matches our I2S clock exactly (no resampling in
// high-fidelity modes; low-fidelity modes run 16 kHz grains with the
// built-in SRC).
//
// SRAM policy: ALL hot Clouds code and ALL resource LUTs are placed in SRAM
// via .time_critical sections (lib patches, see tools/patches/clouds/) —
// XIP flash execution cost more than the whole audio block budget on
// Elements; Clouds gets the same treatment from day one.
//
// Wiring (identical to elements-rp2350): PCM5102A BCK GP6 / DIN GP8 / LRCK
// GP7, XSMT 3V3; USB-A host D+ GP14 / D- GP15, VBUS 5V; WS2812 GP22.
#include <Arduino.h>
#include <I2S.h>

// Core 1 gets a private 8 KB stack (see elements-rp2350/src/main.cpp: core
// 0's stack peaks at USB attach and a shared 4 KB/4 KB split corrupted core 1).
bool core1_separate_stack = true;
#include <Adafruit_TinyUSB.h>

#include "pio_usb.h"             // Pico-PIO-USB (vendored in lib/pio_usb/)

#include <STMLIB.h>
#include <CLOUDS.h>

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16

#include "status_led.h"          // WS2812 status LED on GP22 (before midi.h)
#include "midi.h"                // shared parser + extern shared-state decls

// PIO-USB root-port accessors for the re-enumeration watchdog (C side).
extern "C" {
bool pio_host_line_idle(void);
bool pio_host_connected(void);
void pio_host_force_reconnect(void);
uint8_t pio_host_line_state(void);
}

static const uint32_t kSampleRate = 32000;
static const size_t kBlockSize = clouds::kMaxBlockSize;   // 32 samples = 1 ms

static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);

// --- engine ---
// Sample memory, mirroring the original F405 two-region split (main SRAM +
// CCM): large = mono/low-fidelity memory + FX workspace, small = the stereo
// 16-bit sample memory (32768 samples/ch = 1.02 s @ 32 kHz). Static in .bss —
// 192 KB of the ~276 KB free after the platform's own usage.
static uint32_t large_buffer[32768];   // 128 KB
static uint32_t small_buffer[16384];   //  64 KB

static clouds::GranularProcessor processor;
static clouds::FloatFrame clouds_in[kBlockSize];
static clouds::FloatFrame clouds_out[kBlockSize];

// Status LED instance — defined here (midi.h holds only an extern reference).
StatusLed statusLed;

// --- shared MIDI state (core0 writes, core1 reads; declared in midi.h) ---
volatile float shared_note = 48.0f;
volatile float shared_exciter_tone = 0.5f;
volatile float shared_exciter_level = 0.8f;
volatile float shared_exciter_mode = 0.0f;
volatile bool  shared_gate = false;
volatile float shared_position = 0.35f;
volatile float shared_size = 0.5f;
volatile float shared_pitch = 0.5f;
volatile float shared_density = 0.75f;
volatile float shared_texture = 0.5f;
volatile float shared_dry_wet = 0.8f;
volatile float shared_stereo_spread = 0.5f;
volatile float shared_feedback = 0.0f;
volatile float shared_reverb = 0.3f;

// Pending-change slots are the definitions for the externs in midi.h.
volatile int8_t pending_playback_mode = -1;
volatile int8_t pending_quality = -1;
volatile int8_t pending_freeze = -1;
volatile bool   pending_trigger = false;

// LED meters (core 1 writes after each block, core 0 reads in statusLed.Poll).
static volatile float shared_exciter_out = 0.0f;    // input envelope
static volatile float shared_resonator_out = 0.0f;  // output envelope

static volatile bool audio_ready = false;

// --- perf telemetry (core1 writes, core0 prints) ---
static volatile uint32_t proc_us_max = 0;
// Count of render loops whose 32 I2S writes took > 1800 us (normal worst case
// is ~1000 us = one 32-frame block @ 32 kHz).
static volatile uint32_t i2s_stall_count = 0;

// --- USB MIDI device (core 0) ---
Adafruit_USBD_MIDI usb_midi;

// --- USB MIDI host (PIO-USB on rhport 1, core 0) ---
Adafruit_USBH_Host USBHost;

static constexpr uint8_t kHostMidiMaxIdxs = 4;
static uint8_t host_midi_idxs[kHostMidiMaxIdxs];
static uint8_t host_midi_idx_count = 0;
static volatile bool host_device_mounted = false;

extern "C" {
void tuh_mount_cb(uint8_t daddr) {
  host_device_mounted = true;
  Serial.printf("USB device mounted daddr=%u\r\n", daddr);
}

void tuh_umount_cb(uint8_t daddr) {
  host_device_mounted = false;
  Serial.printf("USB device unmounted daddr=%u\r\n", daddr);
}

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

// Alarm pool created on CORE 1 (see elements-rp2350/src/main.cpp: the PIO-USB
// SOF ISR must run on core 1, isolated from core 0's I2S DMA and TinyUSB
// device interrupts — on core 0 the collisions missed the 2-7 µs handshake
// windows).
static alarm_pool_t* g_host_alarm_pool = nullptr;
static volatile bool g_host_alarm_pool_ready = false;

void setup1() {
  g_host_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(1);
  g_host_alarm_pool_ready = true;
  while (!audio_ready) { delay(1); }
}

// --- test exciter (standalone input source for the granular processor) ---
// Burst mode: every note-on fires a decaying filtered-noise pluck (~0.8 s).
// Tone mode: a sine at the held note's pitch for as long as the gate is on
// (continuous material for freeze/texture exploration). Level 0 mutes the
// exciter entirely (the input slot for future multi-module chaining).
static float burst_env = 0.0f;
static float tone_phase = 0.0f;
static float noise_lp_state = 0.0f;
static uint32_t rng_state = 0x1fff7a10;

static inline float ExciterSample(float note_hz) {
  const float level = shared_exciter_level;
  if (level <= 0.0f) return 0.0f;
  float s;
  if (shared_exciter_mode < 0.5f) {
    // Burst pluck: white noise through a one-pole whose cutoff tracks the
    // note (higher notes = brighter pluck), exponential decay ~0.8 s.
    rng_state = rng_state * 1664525u + 1013904223u;
    const float noise = static_cast<float>(static_cast<int32_t>(rng_state) >> 9) /
                        4194304.0f;   // ±1.0
    const float cutoff = 0.003f + 0.12f * shared_exciter_tone;
    noise_lp_state += cutoff * (noise - noise_lp_state);
    s = noise_lp_state * burst_env * 0.9f;
    burst_env *= 0.99985f;            // ~0.8 s tail at 32 kHz
  } else {
    // Gated tone: sine at the held note (continuous grain material).
    const float inc = note_hz / kSampleRate;
    tone_phase += inc;
    if (tone_phase >= 1.0f) tone_phase -= 1.0f;
    s = sinf(2.0f * 3.14159265f * tone_phase) * 0.6f;
    // A little breath so the granulator has transients to work with.
    rng_state = rng_state * 1664525u + 1013904223u;
    s += static_cast<float>(static_cast<int32_t>(rng_state) >> 14) /
         262144.0f * 0.05f * shared_exciter_tone;
  }
  return s * level;
}

// ================= core 1: audio =================
void loop1() {
  // Apply pending changes strictly between two Process() calls (same
  // discipline as the Elements port's pending_resonator_model slot).
  if (pending_playback_mode >= 0) {
    processor.set_playback_mode(
        static_cast<clouds::PlaybackMode>(pending_playback_mode));
    pending_playback_mode = -1;
  }
  if (pending_quality >= 0) {
    processor.set_quality(pending_quality);
    pending_quality = -1;
  }
  if (pending_freeze >= 0) {
    processor.set_freeze(pending_freeze != 0);
    pending_freeze = -1;
  }
  if (pending_trigger) {
    processor.mutable_parameters()->trigger = true;
    pending_trigger = false;
  } else {
    processor.mutable_parameters()->trigger = false;
  }

  // Copy the panel parameters between blocks (plain float writes, same
  // shared-state pattern as the Elements port's shared_* variables).
  clouds::Parameters* p = processor.mutable_parameters();
  p->position       = shared_position;
  p->size           = shared_size;
  p->pitch          = shared_pitch;
  p->density        = shared_density;
  p->texture        = shared_texture;
  p->dry_wet        = shared_dry_wet;
  p->stereo_spread  = shared_stereo_spread;
  p->feedback       = shared_feedback;
  p->reverb         = shared_reverb;

  // Exciter -> input frames. Note-on retriggers the burst envelope (rising
  // gate edge); the pitch tracks the newest held note + bend.
  static bool previous_gate = false;
  const float note_hz = 440.0f * powf(2.0f, (shared_note - 69.0f) / 12.0f);
  if (shared_gate && !previous_gate) {
    burst_env = 1.0f;
  }
  previous_gate = shared_gate;

  processor.mutable_parameters()->gate = shared_gate;
  for (size_t i = 0; i < kBlockSize; ++i) {
    const float s = ExciterSample(note_hz);
    clouds_in[i].l = s;
    clouds_in[i].r = s;
  }

  uint32_t t0 = time_us_32();
  // Prepare() MUST run every block (original firmware contract): it applies
  // mode/quality changes and updates previous_playback_mode_; without it the
  // processor latches into its silent "mode mismatch" early-return forever.
  processor.Prepare();
  processor.Process(clouds_in, clouds_out, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters for the status LED: input envelope (exciter channel) and output
  // envelope (resonator channel) reuse the Elements LED semantics.
  float in_acc = 0.0f, out_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    in_acc += clouds_in[i].l * clouds_in[i].l;
    out_acc += clouds_out[i].l * clouds_out[i].l;
  }
  shared_exciter_out = sqrtf(in_acc / kBlockSize);
  shared_resonator_out = sqrtf(out_acc / kBlockSize);

  // The writes pace loop1: one iteration = one 32-frame DMA block, blocks
  // free every 1000 us; > 1800 us means no buffer freed for ~2 block
  // periods -> I2S output stalled.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    int16_t l = stmlib::Clip16(static_cast<int32_t>(clouds_out[i].l * 32768.0f));
    int16_t r = stmlib::Clip16(static_cast<int32_t>(clouds_out[i].r * 32768.0f));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > 1800) {
    ++i2s_stall_count;
  }
}

// ================= core 0: control =================
void setup() {
  TinyUSBDevice.setProductDescriptor("Clouds RP2350");
  Serial.begin(115200);

  // Clouds engine: same memory partition as the original F405 firmware
  // (large = mono memory + FX workspace, small = stereo 16-bit memory).
  memset(large_buffer, 0, sizeof(large_buffer));
  memset(small_buffer, 0, sizeof(small_buffer));
  processor.Init(large_buffer, sizeof(large_buffer),
                 small_buffer, sizeof(small_buffer));
  processor.set_sample_rate(static_cast<float>(kSampleRate));
  processor.set_playback_mode(clouds::PLAYBACK_MODE_GRANULAR);
  processor.set_quality(0);          // 16-bit stereo
  processor.Prepare();
  statusLed.Init();  // 3 white self-test flashes, then dark

  // --- USB host (PIO-USB, rhport 1): MUST init BEFORE i2s.begin() ---
  // (PIO-USB hard-claims all of PIO0; see elements-rp2350/src/main.cpp.)
  if (F_CPU % 12000000UL) {
    Serial.println("ERROR: CPU clock not a multiple of 12 MHz - PIO-USB host timing unreliable");
  }
  while (!g_host_alarm_pool_ready) { delay(1); }
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = kPinHostDP;    // D+ = GP14, D- = GP15 (DPDM pinout)
  pio_cfg.alarm_pool = g_host_alarm_pool;
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

  Serial.printf("PIO2 SM0 claimed=%u pc=%u (ws2812 @%u) | SM1 claimed=%u pc=%u\r\n",
                (unsigned)pio_sm_is_claimed(pio2, 0), (unsigned)pio_sm_get_pc(pio2, 0),
                (unsigned)statusLed.Offset(),
                (unsigned)pio_sm_is_claimed(pio2, 1), (unsigned)pio_sm_get_pc(pio2, 1));

  usb_midi.setStringDescriptor("Clouds RP2350");
  if (!usb_midi.begin()) {
    Serial.println("USB MIDI begin FAILED");
  }
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    // 500 ms: after a soft reboot the host's hub driver needs a clearly
    // visible disconnect window (see elements-rp2350/src/main.cpp).
    delay(500);
    TinyUSBDevice.attach();
  }

  audio_ready = true;
  Serial.println("Clouds engine up (granular, 16-bit stereo, USB MIDI device + PIO-USB host)");
}

void loop() {
  USBHost.task();
  pollHostMidi();

  uint8_t pkt[4];
  while (usb_midi.readPacket(pkt)) {
    midiHandleEventPacket(pkt);
  }

  statusLed.Poll(shared_gate, shared_exciter_out, shared_resonator_out);

  static uint32_t last_report = 0;
  uint32_t now = millis();

  // --- USB host re-enumeration watchdog (see elements-rp2350/src/main.cpp) ---
  static uint32_t last_host_watchdog_ms = 0;
  if (now - last_host_watchdog_ms >= 4000UL) {
    last_host_watchdog_ms = now;
    if (pio_host_connected() && !host_device_mounted) {
      Serial.println("USB host watchdog: connected but nothing mounted - forcing re-enumeration");
      pio_host_force_reconnect();
    }
  }

  // --- stuck-gate watchdog (defensive; input for exciter tone mode) ---
  if (shared_gate && now - last_note_event_ms > 30000UL) {
    shared_gate = false;
    note_stack.clear();
    Serial.println("WARNING: gate stuck >30s (lost note-off?) - forced off, note stack cleared");
  }

  // One-line diagnostics report: worst render time + I2S stall counter +
  // PIO-USB host tail. Budget = 1 ms (32-sample block @ 32 kHz).
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | host: conn=%u st=%u dev=%u midi=%u frame=%u\r\n",
                  proc_us_max,
                  (unsigned)(1000000UL * kBlockSize / kSampleRate),
                  stalls, stalls - last_stall_total,
                  (unsigned)pio_host_connected(),
                  (unsigned)pio_host_line_state(),
                  (unsigned)host_device_mounted,
                  (unsigned)host_midi_idx_count,
                  (unsigned)pio_usb_host_get_frame_number());
    last_stall_total = stalls;
    proc_us_max = 0;
  }
}
