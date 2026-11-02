// Plaits RP2350 — Mutable Instruments Plaits (the Rubato "Plaits Lab" fork,
// all 98 stereo-capable engines) on the RP2350 nano + PCM5102A, using the
// proven clouds-rp2350 architecture: core 0 = USB MIDI device (CDC + USB-MIDI)
// + PIO-USB host MIDI + status LED + diagnostics; core 1 = audio render at
// 48 kHz into the I2S DAC.
//
// The module is a SYNTH VOICE — no input jack, no test exciter (unlike the
// Clouds port, which needed a MIDI-triggered input source). MIDI drives the
// voice through the exact Patch/Modulations contract of the original firmware
// main (EURORACK-STEREO/plaits/plaits.cc FillBuffer -> ui.Poll fills the
// Patch, voice.Render synthesizes); the mapping copied from that contract is
// documented inline below and in midi.h.
//
// Voice core (see docs/superpowers/plans/2026-11-02-plaits-rp2350.md):
// plaits::Voice owns all 98 engines via PLAITS_ENGINE_MEMBERS; their state
// RAM is carved from one shared pool (allocator->Free() before each engine
// Init, voice.cc:109-113), so the pool must cover the LARGEST engine only
// (measured max 16.4 KB: ParticleEngine uint16_t[8192]) — 32 KB with margin.
// Native rate 48 kHz (plaits/dsp/dsp.h kSampleRate; the fork's 47872.34 Hz
// module-clock pitch shim computes to zero thanks to
// -DPLAITS_CORRECTED_SAMPLE_RATE=48000.0f), kMaxBlockSize = 24 samples
// (budget 500 us/block). Frame = { short out; short aux } = interleaved
// stereo: with patch.aux_output_option = 1 and a stereo-capable engine,
// OUT = left, AUX = right (voice.cc:496-501, shared limiter voice.cc:904-935).
//
// SRAM policy: the hot Plaits code and resource LUTs are placed in SRAM via
// .time_critical sections by tools/patches/plaits/ (T3) — XIP flash cache
// misses cost more than the whole audio block budget (measured on Elements
// and Clouds). Verified via nm after the first build: hot symbols at
// 0x2000xxxx.
//
// Wiring (identical to elements/clouds-rp2350): PCM5102A BCK GP6 / DIN GP8 /
// LRCK GP7, XSMT 3V3; USB-A host D+ GP14 / D- GP15, VBUS 5V; WS2812 GP22.
#include <Arduino.h>
#include <I2S.h>

// Core 1 gets a private 8 KB stack (see elements-rp2350/src/main.cpp: core
// 0's stack peaks at USB attach and a shared 4 KB/4 KB split corrupted core 1).
bool core1_separate_stack = true;
#include <Adafruit_TinyUSB.h>

#include "pio_usb.h"             // Pico-PIO-USB (vendored in lib/pio_usb/)

#include <STMLIB.h>
#include <PLAITS.h>

#include "plaits/dsp/voice.h"    // plaits::Voice, Patch, Modulations, kMaxEngines
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x —
                                 // never declare it extern)

#include "status_led.h"          // WS2812 status LED on GP22 (before midi.h)
#include "midi.h"                // shared parser + extern shared-state decls

// The MIDI CC map bounds engine selection to the Plaits-Lab catalog size;
// this must equal the registry the lib was generated with (platformio.ini
// passes -DPLAITS_ENGINE_COUNT=98 to src/ and lib/ alike).
static_assert(plaits::kMaxEngines == kPlaitsEngineCount,
              "midi.h catalog size must match PLAITS_ENGINE_COUNT");

// PIO-USB root-port accessors for the re-enumeration watchdog (C side).
extern "C" {
bool pio_host_line_idle(void);
bool pio_host_connected(void);
void pio_host_force_reconnect(void);
uint8_t pio_host_line_state(void);
}

static const uint32_t kSampleRate = 48000;                    // = plaits::kSampleRate
static const size_t kBlockSize = plaits::kMaxBlockSize;       // 24 samples
// Budget = 500 us: 1000000 * 24 / 48000, multiplied FIRST (the divide-first
// grouping truncates — a hard-won reporting bug from the Elements port).
static const unsigned kBudgetUs = (unsigned)(1000000UL * kBlockSize / kSampleRate);

static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);

// --- engine ---
// plaits::Voice embeds all 98 engine instances (PLAITS_ENGINE_MEMBERS) plus
// per-engine post-processors; it lives in .bss. The workspace pool is carved
// per engine by stmlib::BufferAllocator (voice.cc:105-113 frees before every
// engine Init) — 32 KB covers the largest single engine (16.4 KB) with
// margin. aligned(8): Allocate<T> returns typed pointers WITHOUT adjusting
// alignment (same reason the original plaits.cc:104-107 aligns its pool).
static uint32_t workspace[8192] __attribute__((aligned(8)));   // 32 KB

static plaits::Voice voice;
static plaits::Voice::Frame frames[kBlockSize];   // interleaved stereo shorts

// Patch/Modulations are filled per block by core 1 (values written by core 0
// into the shared_* volatiles / pending slots). Defaults mirror a centered
// panel at boot; amounts are exactly neutral (0 = no modulation).
static plaits::Patch patch;
static plaits::Modulations mod;

// Status LED instance — defined here (midi.h holds only an extern reference).
StatusLed statusLed;

// --- shared MIDI state (core0 writes, core1 reads; declared in midi.h) ---
volatile float shared_note = 48.0f;      // idle note (C3); overwritten by MIDI
volatile bool  shared_gate = false;
volatile float shared_timbre = 0.5f;      // CC 14
volatile float shared_morph = 0.5f;       // CC 15
volatile float shared_harmonics = 0.5f;   // CC 16
volatile float shared_macro_cv = 0.0f;    // CC 17: (val-64)/127, 0 = macro 0.5
volatile float shared_lpg_colour = 0.5f;  // CC 18
volatile float shared_decay = 0.5f;       // CC 19
volatile float shared_fm_amount = 0.0f;   // CC 20: v*2-1, 0 = off
volatile float shared_timbre_mod = 0.0f;  // CC 21: v*2-1, 0 = off
volatile float shared_morph_mod = 0.0f;   // CC 22: v*2-1, 0 = off
volatile float shared_level = 0.8f;       // CC 25 (accent/LPG level input)

// Pending-change slots are the definitions for the externs in midi.h.
volatile int8_t   pending_engine = -1;      // -1 none, 0..97 engine index
volatile int8_t   pending_aux_option = -1;  // -1 none, 0/1/2 aux option
volatile float    pending_trigger = -1.0f;  // <0 none, else TRIG pulse voltage
volatile uint32_t note_on_count = 0;

// LED meters (core 1 writes after each block, core 0 reads in statusLed.Poll):
// OUT/AUX block RMS, max since the last 2 s report (a held or decaying note
// would otherwise read 0 at the print instant).
volatile float shared_out_meter = 0.0f;
volatile float shared_aux_meter = 0.0f;

// Active engine mirror for the 2 s report (core 1 publishes, core 0 prints —
// avoids reading Voice state cross-core in loop()).
static volatile uint32_t active_engine_report = 0;

static volatile bool audio_ready = false;

// --- perf telemetry (core1 writes, core0 prints) ---
static volatile uint32_t proc_us_max = 0;
// Count of render loops whose 24 I2S writes took > 900 us (see loop1: 1.8x
// the 500 us block period, the same ratio Clouds used at 32 kHz).
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
// device interrupts — on core 0 the collisions missed the 2-7 us handshake
// windows).
static alarm_pool_t* g_host_alarm_pool = nullptr;
static volatile bool g_host_alarm_pool_ready = false;

void setup1() {
  g_host_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(1);
  g_host_alarm_pool_ready = true;
  while (!audio_ready) { delay(1); }
}

// ================= core 1: audio =================
//
// ---------------------------------------------------------------------------
// MIDI -> Patch/Modulations mapping, copied from the original firmware's
// per-block fill (EURORACK-STEREO/plaits/plaits.cc + ui_poll.cc + voice.cc).
// Per block, BEFORE Render (the original order: ui.Poll() fills, then
// plaits.cc:254 `voice.Render(patch, modulations, (Voice::Frame*)output, size)`):
//
//   patch.note
//     NOTE NUMBER in semitones (MIDI domain: 69 = 440 Hz via
//     engine.h NoteToFrequency: a0*0.25*SemitonesToRatio(note-9),
//     a0 = 55/kCorrectedSampleRate). The panel writes the same domain
//     (ui_poll.cc:169-195, e.g. :192 `patch_->note = 60.0f + t*48.0f`).
//     We write the NoteStack top + pitch bend; idle default 48 (C3).
//     modulations.note (the V/OCT CV ADD-ON; voice.cc:704
//     `p.note = ApplyModulations(patch.note + note, ...)`) stays 0.0f and
//     frequency_patched = false. With trigger_patched and FM amount 0 the
//     internal pitch envelope contributes nothing (the amount scales to 0 in
//     ApplyModulations, voice.cc:377-386).
//
//   patch.harmonics / patch.timbre / patch.morph          CC 16/14/15, 0..1
//   patch.decay / patch.lpg_colour                        CC 19/18, 0..1
//     (ui.cc:158-160 pot scale 1.0 offset 0.0; LoadState ui.cc:264-265 /256.)
//   patch.frequency_modulation_amount                     CC 20, v*2-1
//   patch.timbre_modulation_amount                        CC 21, v*2-1
//   patch.morph_modulation_amount                         CC 22, v*2-1
//     (Panel pots span [-1, +1]: ui.cc:161-166 PotController scale 2.0
//     offset -1.0, pot_controller.h:123 `value_ * scale_ + offset_`.)
//
//   patch.engine                                          CC 23 (raw 0..97)
//     voice.cc:216-218: engine_index = engine_quantizer_.Process(patch.engine,
//     model_cv_option == 0 ? engine_cv_ : 0.0f). We set model_cv_option = 1,
//     so the offset is 0 and the index is exactly patch.engine, CONSTRAINed
//     to the registry size (quantizer Init'd symmetric, voice.cc:118).
//     On change, voice.cc:382-489 reloads user data + Reset()s the engine
//     between blocks.
//
//   patch.aux_output_option = 1 (default)                 CC 24 (0/1/2)
//     voice.cc:496-501: stereo_render = aux_is_stereo() &&
//     e->stereo_capable() -> OUT = left, AUX = right, one linked limiter
//     (voice.cc:904-913). 0 = regular aux model, 2 = suboscillator
//     (voice.cc:808-822, shape/octave via patch.aux_subosc_option = 0).
//
//   MACRO (the fork's fourth synthesis macro, "TWIST")    CC 17
//     voice.cc:510-514: p.macro = 0.5f; p.macro += macro_cv, where
//     macro_cv = modulations.engine when patch.model_cv_option == 1
//     (voice.cc:322-326 — the MODEL CV input repurposed). We send
//     (CC-64)/127 as modulations.engine so macro == CC/127 with the exact
//     engine-stock neutral 0.5 at CC 64 (ApplyMacro contract, engine.h:111).
//     Engines that ignore macro are untouched.
//
//   modulations.trigger (TRIG)                            note-on pulse
//     The calibrated TRIG CV in 0..1. Voice delays it 5 blocks
//     (kTriggerDelay, voice.cc:154-156), thresholds >0.3 / <0.1, latches
//     trigger_velocity_ = pulse value on the rising edge (voice.cc:199-202)
//     and fires decay_envelope_/lpg_envelope_.Trigger(). We hold it high for
//     exactly one block on each note-on at 0.3 + 0.7*velocity — always above
//     the 0.3 threshold (velocity 1..38 would otherwise never strike) — so
//     with patch.trig_response_option = 2 (velocity trigger) the latch
//     becomes the velocity: p.accent = trigger_velocity_ *
//     compressed_level (voice.cc:589-593 ApplyVelocityAccent) and the LPG
//     ping/level scales with it (voice.cc:879-884).
//
//   modulations.level (LEVEL)                             CC 25, level_patched
//     compressed_level = 1.3*L/(0.3+|L|) (voice.cc:574). With
//     level_patched = true the LPG tracks compressed_level * velocity while
//     held (voice.cc:879-884) -> sustained note; on note-off we drop it to
//     0 and the vactrol releases with patch.decay (LPGEnvelope::ProcessLP,
//     envelope.h:69-88). Already-enveloped engines (drums etc.) bypass the
//     LPG (voice.cc:845-846) and get velocity via p.accent.
//     level_cv_option = 0 keeps LEVEL as level (voice.cc:259-266 no-op).
//
//   Everything else unpatched, exactly like the panel with only TRIG/LEVEL
//   jack normals closed: mod.engine = macro CV (only read as macro under
//   model_cv_option = 1), mod.note/frequency/harmonics/timbre/morph = 0.0f,
//   hard_sync = 0, frequency_audio[] = 0, all *_patched flags false except
//   trigger_patched/level_patched (ui_poll.cc:65-69 writes CV readings only
//   into these same fields; normalization detectors set the *_patched flags).
// ---------------------------------------------------------------------------
void loop1() {
  // Apply pending changes strictly between two Render() calls (same
  // discipline as the Clouds port's pending_playback_mode slot). Engine
  // changes go through patch.engine; Voice reloads/resets the engine itself
  // at the top of Render (voice.cc:382-489).
  if (pending_engine >= 0) {
    int engine = pending_engine;
    if (engine > plaits::kMaxEngines - 1) engine = plaits::kMaxEngines - 1;
    patch.engine = engine;
    pending_engine = -1;
  }
  if (pending_aux_option >= 0) {
    patch.aux_output_option = static_cast<uint8_t>(pending_aux_option);
    pending_aux_option = -1;
  }

  // TRIG pulse: one block high at the note-on velocity (see the mapping
  // notes above), then low so trigger_state_ clears (delayed value < 0.1,
  // voice.cc:207-209). Rapid re-notes each get their own edge.
  if (pending_trigger > -0.5f) {
    mod.trigger = pending_trigger;
    pending_trigger = -1.0f;
  } else {
    mod.trigger = 0.0f;
  }

  // --- Patch fill (units documented above) ---
  patch.note = shared_note;
  patch.harmonics = shared_harmonics;
  patch.timbre = shared_timbre;
  patch.morph = shared_morph;
  patch.frequency_modulation_amount = shared_fm_amount;
  patch.timbre_modulation_amount = shared_timbre_mod;
  patch.morph_modulation_amount = shared_morph_mod;
  patch.decay = shared_decay;
  patch.lpg_colour = shared_lpg_colour;

  // --- Modulations fill: TRIG + LEVEL patched, everything else unpatched ---
  mod.engine = shared_macro_cv;        // fourth macro under model_cv_option=1
  mod.note = 0.0f;
  mod.frequency = 0.0f;
  mod.harmonics = 0.0f;
  mod.timbre = 0.0f;
  mod.morph = 0.0f;
  mod.level = shared_gate ? shared_level : 0.0f;  // held = sustain, off = release
  mod.hard_sync = 0;
  for (size_t i = 0; i < kBlockSize; ++i) {
    mod.frequency_audio[i] = 0.0f;     // unused: PLAITS_BUILD_FREQUENCY_OFFSET_FM=0
  }
  mod.frequency_audio_rate = false;
  mod.frequency_patched = false;
  mod.timbre_patched = false;
  mod.morph_patched = false;
  mod.trigger_patched = true;
  mod.level_patched = true;

  uint32_t t0 = time_us_32();
  // Render() every block (the original firmware contract: plaits.cc:254).
  // Output Frames are already limited/clipped to int16 inside Voice
  // (ChannelPostProcessor::Write, voice.cc:197-201 stmlib::Clip16; the
  // stereo limiter voice.cc:140-156) — write them raw, do NOT double-clip.
  voice.Render(patch, mod, frames, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  active_engine_report = (uint32_t)voice.active_engine();

  // Meters for the status LED: OUT/AUX block RMS normalized to 0..1, held as
  // MAX since the last 2 s report (reported as x1000 ints, like Clouds).
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    const float l = static_cast<float>(frames[i].out) / 32768.0f;
    const float r = static_cast<float>(frames[i].aux) / 32768.0f;
    out_acc += l * l;
    aux_acc += r * r;
  }
  const float out_rms = sqrtf(out_acc / kBlockSize);
  const float aux_rms = sqrtf(aux_acc / kBlockSize);
  if (out_rms > shared_out_meter) shared_out_meter = out_rms;
  if (aux_rms > shared_aux_meter) shared_aux_meter = aux_rms;

  // The writes pace loop1: one iteration = one 24-frame DMA block, blocks
  // free every 500 us (24 / 48 kHz); > 900 us = 1.8x the block period (the
  // same ratio Clouds used: 1800 vs 1000 at 32 kHz) means no buffer freed
  // for ~2 block periods -> I2S output stalled. Plaits has no FFT bursts,
  // so the DMA bank only needs to absorb scheduling jitter.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    i2s.write16(frames[i].out, frames[i].aux);
  }
  if (time_us_32() - w0 > 900) {
    ++i2s_stall_count;
  }
}

// ================= core 0: control =================
void setup() {
  TinyUSBDevice.setProductDescriptor("Plaits RP2350");
  Serial.begin(115200);

  // Plaits voice: one shared workspace pool carved per engine (voice.cc:
  // 105-113); -DPLAITS_CORRECTED_SAMPLE_RATE=48000.0f makes the fork's
  // module-clock pitch shim exactly zero (we clock I2S at true 48 kHz).
  memset(workspace, 0, sizeof(workspace));
  stmlib::BufferAllocator allocator(workspace, sizeof(workspace));
  voice.Init(&allocator);

  // Patch option fields the CC map does not reach (voice.h:245-286), set to
  // the MIDI-sensible defaults the per-block fill relies on:
  patch.engine = 0;                      // CC 23 from here on
  patch.aux_output_option = 1;           // stereo OUT/AUX (CC 24)
  patch.aux_subosc_option = 0;           // square, no octave shift (aux option 2)
  patch.trig_response_option = 2;        // velocity trigger: pulse voltage = velocity
  patch.model_cv_option = 1;             // MODEL CV = fourth macro (CC 17)
  patch.locked_frequency_pot_option = 0; // no pitch lock: patch.decay is DECAY
  patch.level_cv_option = 0;             // LEVEL stays level (no decay rerouting)
  patch.hold_on_trigger_option = 0;      // no parameter hold on trigger
  patch.chord_set_option = 0;            // first chord table (chord engines)
  patch.attenuverter_mode = plaits::ATTENUVERTER_MODE_STOCK;  // 0: no drift/step randomizer
  patch.freqlock_param = 0.5f;           // unused while locked_frequency_pot_option=0
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
  // 48 buffers x 16 words = 768 stereo frames = 16 ms bank at 48 kHz (one
  // 32-bit word per buffer = one 16-bit stereo frame; 768/48000 = 16 ms).
  // No engine bursts (the fork has no FFT); the bank absorbs scheduling
  // jitter and USB-host blips on core 1's neighbor. Bank RAM is cheap
  // (768 frames x 4 B = 3 KB).
  i2s.setBuffers(48, 16);
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

  usb_midi.setStringDescriptor("Plaits RP2350");
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
  Serial.println("Plaits engine up (98 engines, stereo aux, 48 kHz / 24-sample blocks, USB MIDI device + PIO-USB host)");
}

void loop() {
  USBHost.task();
  pollHostMidi();

  uint8_t pkt[4];
  while (usb_midi.readPacket(pkt)) {
    midiHandleEventPacket(pkt);
  }

  statusLed.Poll(shared_gate, shared_out_meter, shared_aux_meter);

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

  // --- stuck-gate watchdog (defensive; gate holds the LPG open) ---
  if (shared_gate && now - last_note_event_ms > 30000UL) {
    shared_gate = false;
    note_stack.clear();
    Serial.println("WARNING: gate stuck >30s (lost note-off?) - forced off, note stack cleared");
  }

  // One-line diagnostics report: worst render time + I2S stall counter +
  // active engine + output meters + PIO-USB host tail. Budget = 500 us
  // (24-sample block @ 48 kHz; kBudgetUs multiplies before dividing).
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | eng=%u out=%u aux=%u notes=%u | host: conn=%u st=%u dev=%u midi=%u frame=%u\r\n",
                  proc_us_max,
                  kBudgetUs,
                  stalls, stalls - last_stall_total,
                  (unsigned)active_engine_report,
                  (unsigned)(shared_out_meter * 1000.0f),
                  (unsigned)(shared_aux_meter * 1000.0f),
                  note_on_count,
                  (unsigned)pio_host_connected(),
                  (unsigned)pio_host_line_state(),
                  (unsigned)host_device_mounted,
                  (unsigned)host_midi_idx_count,
                  (unsigned)pio_usb_host_get_frame_number());
    last_stall_total = stalls;
    proc_us_max = 0;
    shared_out_meter = 0.0f;        // max-since-report meters (see loop1)
    shared_aux_meter = 0.0f;
  }
}
