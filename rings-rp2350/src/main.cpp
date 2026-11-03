// Rings RP2350 — Mutable Instruments Rings ("MiniElements" fork of
// pichenettes' eurorack by lylepmills: stock Rings DSP + internal
// Elements-style exciters + a string-synth easter egg) on the RP2350 nano +
// PCM5102A I2S DAC, using the proven plaits-rp2350 architecture: core 0 =
// USB MIDI device (CDC + USB-MIDI) + PIO-USB host MIDI + status LED +
// diagnostics; core 1 = audio render at 32 kHz into the I2S DAC.
//
// The module is a SYNTH — no input jack, no external test exciter (rings
// self-excites: internal_exciter = true gives MODAL a pulse, STRING a pluck
// burst, FM an envelope, and the MiniElements modes their bow/blow/strike
// exciters). MIDI drives the voice through the exact Patch/PerformanceState
// per-block contract of the original firmware main (EURORACK-STEREO/rings/
// rings.cc FillBuffer, lines 88-127: fresh Patch + PerformanceState locals
// every block, then ONE Part renders — Part for modes 0-3, StringSynthPart
// for the mode-4 easter egg — then Clip16(out/aux) to the codec). The
// Strummer/OnsetDetector front-end of the original is DROPPED: we drive
// strum ourselves from MIDI note-ons (internal_strum = false).
//
// Engine core (see docs/superpowers/plans/2026-11-03-rings-rp2350.md):
// rings::Part + rings::StringSynthPart over ONE caller-owned reverb buffer
// (rings.cc:43: uint16_t reverb_buffer[32768]; reverb/chorus/ensemble of
// BOTH parts point at it — safe because the two Parts are mutually
// exclusive by mode, exactly one Process per block). Native rate 32 kHz
// (rings/dsp/dsp.h kSampleRate — a compile-time constant, there is NO rate
// setter in this fork), kMaxBlockSize = 24 samples (budget 750 us/block).
// Output convention: out = L, aux = R (the fork's ODD/EVEN stereo; in modes
// 1/3 the AUX jack carries the exciter/waveform monitor instead — see the
// README).
//
// SRAM policy: the hot rings code and resource LUTs are placed in SRAM via
// .time_critical sections by tools/patches/rings/ (T3) — XIP flash cache
// misses cost more than the whole audio block budget (measured on Elements
// and Clouds). The MiniElements blow/strike exciters read the ~338 KB
// smp_* sample ROMs in resources.cc — those stay flash-resident (read
// through the XIP cache, do NOT SRAM).
//
// Wiring (identical to elements/clouds/plaits-rp2350): PCM5102A BCK GP6 /
// DIN GP8 / LRCK GP7, XSMT 3V3; USB-A host D+ GP14 / D- GP15, VBUS 5V;
// WS2812 GP22.
#include <Arduino.h>
#include <I2S.h>

// Core 1 gets a private 8 KB stack (see elements-rp2350/src/main.cpp: core
// 0's stack peaks at USB attach and a shared 4 KB/4 KB split corrupted core 1).
bool core1_separate_stack = true;
#include <Adafruit_TinyUSB.h>

#include "pio_usb.h"             // Pico-PIO-USB (vendored in lib/pio_usb/)

#include <STMLIB.h>
#include <RINGS.h>               // rings::Part + rings::StringSynthPart
                                 // (part.h pulls patch.h + performance_state.h)

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (rings has no output clipper
                                 // of its own — the original firmware clips at
                                 // the codec write, rings.cc:120-121)

#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x —
                                 // never declare it extern)

#include "status_led.h"          // WS2812 status LED on GP22 (before midi.h)
#include "midi.h"                // shared parser + extern shared-state decls

// PIO-USB root-port accessors for the re-enumeration watchdog (C side).
extern "C" {
bool pio_host_line_idle(void);
bool pio_host_connected(void);
void pio_host_force_reconnect(void);
uint8_t pio_host_line_state(void);
}

static const uint32_t kSampleRate = 32000;                    // = rings::kSampleRate
static const size_t kBlockSize = rings::kMaxBlockSize;        // 24 samples
// Budget = 750 us: 1000000 * 24 / 32000, multiplied FIRST (the divide-first
// grouping truncates — a hard-won reporting bug from the Elements port).
static const unsigned kBudgetUs = (unsigned)(1000000UL * kBlockSize / kSampleRate);

static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);

// --- engine ---
// One caller-owned reverb/chorus/ensemble workspace shared by both Parts
// (rings.cc:43 uses the same single buffer for part.Init + string_synth.Init).
// Safe because the firmware contract renders exactly ONE part per block
// (mode-selected; rings.cc:98-116): the idle part's FX state simply goes
// stale — the original firmware shares the ONE buffer the same way
// (rings.cc:137-138) and never re-Inits the parts on mode changes.
static uint16_t reverb_buffer[32768];   // 64 KB

static rings::Part part;
static rings::StringSynthPart string_synth;

static float in[kBlockSize];    // no input jack: fed with silence every block
static float out[kBlockSize];
static float aux[kBlockSize];

// Status LED instance (single definition; driven from core 0's loop() and
// the host-MIDI mount callback).
StatusLed statusLed;

// --- shared MIDI state (core0 writes, core1 reads; declared in midi.h) ---
volatile float shared_note = 48.0f;       // idle note (C3); overwritten by MIDI
volatile bool  shared_gate = false;
volatile float shared_structure = 0.5f;   // CC 14 (clamped <= 0.9995 at fill)
volatile float shared_brightness = 0.5f;  // CC 15
volatile float shared_damping = 0.5f;     // CC 16
volatile float shared_position = 0.5f;    // CC 17

// Pending-change slots are the definitions for the externs in midi.h.
volatile int8_t  pending_model = -1;         // -1 none, 0..5 resonator model / FxType
volatile int8_t  pending_polyphony = -1;     // -1 none, 1..4
volatile int8_t  pending_mode = -1;          // -1 none, 0..4 PerformanceMode
volatile int8_t  pending_waveform = -1;      // -1 none, 0..2 waveform_exciter
volatile int8_t  pending_chord_table = -1;   // -1 none, 0..2 ChordTable
volatile int8_t  pending_chord = -1;         // -1 none, 0..127 raw (clamped at fill)
volatile bool    pending_strum = false;      // one-block strum pulse per note-on
volatile uint32_t note_on_count = 0;

// LED meters (core 1 writes after each block, core 0 reads in statusLed.Poll):
// OUT/AUX block RMS, max since the last 2 s report (a held or decaying note
// would otherwise read 0 at the print instant).
volatile float shared_out_meter = 0.0f;
volatile float shared_aux_meter = 0.0f;

// Applied-state mirrors for the 2 s report (core 1 publishes, core 0 prints —
// the plaits active_engine_report pattern). model is the APPLIED CC 18 byte
// (clamped 0-5): the ResonatorModel index in modes 0-3, the FxType index in
// mode 4.
static volatile uint32_t active_mode_report = 0;
static volatile uint32_t active_model_report = 0;

// --- applied settings (core 1 only; applied between blocks, NEVER mid-Process) ---
// Same discipline as the Clouds port's pending_playback_mode slot: the Part
// state (model/polyphony) is mutated via set_model/set_polyphony between two
// Process() calls, and mode/waveform/chord state is baked into the fresh
// per-block PerformanceState locals.
static int32_t applied_mode = rings::MODE_RINGS_STEREO;       // 0
static int32_t applied_model = 0;                      // RESONATOR_MODEL_MODAL
static int32_t applied_polyphony = 1;
static int32_t applied_waveform = 0;                   // square / bow
static int32_t applied_chord_table = rings::CHORD_TABLE_BRYAN; // 0
static int32_t applied_chord = 0;                      // raw, clamped at fill

static volatile bool audio_ready = false;

// --- perf telemetry (core1 writes, core0 prints) ---
static volatile uint32_t proc_us_max = 0;
// Count of render loops whose 24 I2S writes took > 1400 us (~1.8x the 750 us
// block period, the same ratio convention as Clouds 1800/1000 and Plaits
// 900/500) — no buffer freed for ~2 block periods -> I2S output stalled.
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

// Number of valid chord indexes for the current table/mode/polyphony — a
// mirror of CvScaler::NumChords (cv_scaler.cc:289-309). CHORD_TABLE_JOE is
// the quirk: 18 chords in the easter egg, otherwise the count depends on
// polyphony because the Part's joeChords rows are grouped per polyphony
// (15 scales / 21 functional-harmony / 16 harmonic intervals / 23 just
// intervals — part.cc:292-...).
static inline int32_t NumChordsFor(int32_t table, int32_t mode, int32_t polyphony) {
  using namespace rings;  // ChordTable/PerformanceMode enums + kNum*Chords
  switch (table) {
    case CHORD_TABLE_BRYAN:
      return kNumBryanChords;             // 11
    case CHORD_TABLE_JON:
      return kNumJonChords;               // 17
    default:                              // CHORD_TABLE_JOE
      if (mode == MODE_EASTER_EGG) return kNumJoeEasterEggChords;  // 18
      if (polyphony == 1) return 15;
      if (polyphony == 2) return 21;
      if (polyphony == 3) return 16;
      return kMaxNumJoeChords;            // 23
  }
}

// ================= core 1: audio =================
//
// ---------------------------------------------------------------------------
// MIDI -> Patch/PerformanceState mapping, copied from the original firmware's
// per-block fill (EURORACK-STEREO/rings/rings.cc:88-127 + cv_scaler.cc Read).
// Per block, BEFORE Process (the original order: cv_scaler.Read(&patch,
// &performance_state, ...) at rings.cc:96, then ONE part renders at :103/:116):
//
//   performance_state.note
//     NOTE NUMBER in semitones (MIDI domain: part.cc:701
//     `note = note_[voice] + performance_state.tonic + fm`, then
//     frequency = SemitonesToRatio(note - 69) * a3, so 69 = 440 Hz). The
//     original panel writes tonic = 12 + transpose (FREQUENCY pot,
//     cv_scaler.cc:263) and note = the V/OCT CV reading; we set tonic = 0
//     and note = MIDI note + bend instead — same sum, exact MIDI pitch.
//     Idle default 48 (C3).
//
//   performance_state.tonic = 0.0f, .fm = 0.0f
//     No FREQUENCY pot, no V/OCT/FM jack (fm is the CV input amount,
//     cv_scaler.cc:192-200; with nothing patched it is 0).
//
//   performance_state.internal_exciter = true   -> rings self-excites:
//     MODAL gets a pulse (part.cc:452-457), STRING a pluck burst
//     (part.cc:570-578), FM an internal envelope (part.cc:487-491), and the
//     MiniElements modes their bow/blow/strike exciters (part.cc:722-727,
//     808-811). internal_strum = false (the Strummer is dropped — note-ons
//     fire strum directly), internal_note = false.
//
//   performance_state.strum                     one-block pulse per note-on
//     Retrigger: part.cc:665-674 snaps note_[active_voice_] to the note
//     filter's stable_note and round-robins the active voice; the exciters
//     fire on the same edge (EXCITER_FLAG_RISING_EDGE, part.cc:825-827).
//     The easter egg uses it to start a new chord note
//     (string_synth_part.cc:435-441).
//
//   performance_state.strum_gate                gate while a note is held
//     Drives EXCITER_FLAG_GATE (part.cc:828-830) so bow/blow sustain while
//     held. Unused by the easter egg (its envelopes gate themselves,
//     string_synth_part.cc:442-449).
//
//   performance_state.mode / .chord_table / .waveform_exciter
//     Applied CC 20/22/21 values (pending slots, applied between blocks).
//     waveform_exciter = 0/1/2 = square/bow, impulse/blow, saw/strike
//     (part.cc:791-807 oscillator shapes in mode 1; part.cc:838-873 bow/
//     blow/strike exciters in modes 2-3).
//
//   performance_state.chord                     CC 23, clamped
//     The original scales the STRUCTURE CV by (num_chords - 1) with
//     hysteresis (cv_scaler.cc:274-283); we take a raw CC index clamped to
//     NumChordsFor(table, mode, polyphony) at fill time (the JOE table's
//     per-polyphony quirk included — see NumChordsFor above).
//
//   performance_state.*_option = 0, frequency_locked = 0,
//   locked_frequency_pot_value = 0.0f
//     strum_hold_option = 0 keeps update_patch = true every block
//     (part.cc:687-696); fm_input_option = 0 keeps fm un-repurposed
//     (FMInputRepurposed() false, part.cc:679-681); no frequency lock (the
//     fork's locked-FREQUENCY pot feature is not reachable from MIDI yet).
//
//   patch.structure / .brightness / .damping / .position     CC 14/15/16/17
//     0..1, exactly the ATTENUVERT macro ranges (cv_scaler.cc:187-190);
//     structure is clamped to <= 0.9995 like the original (cv_scaler.cc:187
//     CONSTRAIN 0.0..0.9995 — 1.0 would blow up the dispersion math,
//     part.cc:584-587). In the MiniElements modes with waveform_exciter 1/2
//     (blow/strike) POSITION is repurposed as the exciter parameter by the
//     Part itself (PositionRepurposed(), part.h:133-137).
//
//   in[] = silence (no input jack). Part::Process / StringSynthPart::Process
//     copy/accumulate it (rings.cc:100-116; string_synth_part.cc:455-456),
//     so zeros leave the internal exciters as the only sound source — the
//     standalone equivalent of an unplugged INPUT jack.
//
//   Output: out = L, aux = R via Clip16 (rings.cc:120-121). Both Parts run
//     their own Limiter internally (part.cc:779, string_synth_part.cc:564)
//     but that is a soft limiter — Clip16 stays as the final safety, exactly
//     like the original firmware.
// ---------------------------------------------------------------------------
void loop1() {
  // --- apply pending changes strictly between two Process() calls ---
  if (pending_mode >= 0) {
    applied_mode = pending_mode <= rings::MODE_EASTER_EGG
        ? pending_mode : rings::MODE_EASTER_EGG;
    pending_mode = -1;
  }
  if (pending_model >= 0) {
    applied_model = pending_model <= 5 ? pending_model : 5;
    // The SAME byte drives the resonator model (modes 0-3) and the easter
    // egg's FxType (mode 4); apply to both so a mode flip lands on the
    // already-selected model/FX.
    part.set_model(static_cast<rings::ResonatorModel>(applied_model));
    string_synth.set_fx(static_cast<rings::FxType>(applied_model));
    pending_model = -1;
  }
  if (pending_polyphony >= 0) {
    int32_t p = pending_polyphony;
    if (p < 1) p = 1;
    if (p > rings::kMaxPolyphony) p = rings::kMaxPolyphony;
    applied_polyphony = p;
    part.set_polyphony(p);            // Part clamps too (part.h:89-96)
    string_synth.set_polyphony(p);    // (string_synth_part.h:87-96)
    pending_polyphony = -1;
  }
  if (pending_waveform >= 0) {
    applied_waveform = pending_waveform <= 2 ? pending_waveform : 2;
    pending_waveform = -1;
  }
  if (pending_chord_table >= 0) {
    applied_chord_table = pending_chord_table <= rings::CHORD_TABLE_JOE
        ? pending_chord_table : rings::CHORD_TABLE_JOE;
    pending_chord_table = -1;
  }
  if (pending_chord >= 0) {
    applied_chord = pending_chord;    // raw 0..127, clamped at fill time
    pending_chord = -1;
  }

  // Strum pulse: exactly one block high per note-on (consumed by Part's
  // rising-edge detection and the exciters' EXCITER_FLAG_RISING_EDGE).
  const bool strum_pulse = pending_strum;
  pending_strum = false;

  // --- fresh Patch + PerformanceState locals EVERY block (rings.cc:92-96) ---
  rings::PerformanceState perf;
  rings::Patch patch;

  perf.internal_exciter = true;
  perf.internal_strum = false;
  perf.internal_note = false;
  perf.mode = static_cast<rings::PerformanceMode>(applied_mode);
  perf.chord_table = static_cast<rings::ChordTable>(applied_chord_table);
  perf.waveform_exciter = static_cast<uint8_t>(applied_waveform);
  perf.strum_hold_option = 0;
  perf.fm_input_option = 0;
  perf.frequency_locked = 0;
  perf.locked_frequency_pot_value = 0.0f;
  perf.tonic = 0.0f;
  perf.fm = 0.0f;
  perf.note = shared_note;
  perf.strum = strum_pulse;
  perf.strum_gate = shared_gate;
  perf.chord = applied_chord;
  {
    const int32_t num_chords =
        NumChordsFor(applied_chord_table, applied_mode, applied_polyphony);
    if (perf.chord > num_chords - 1) perf.chord = num_chords - 1;
    if (perf.chord < 0) perf.chord = 0;
  }

  patch.structure = shared_structure;
  if (patch.structure > 0.9995f) patch.structure = 0.9995f;  // cv_scaler.cc:187
  if (patch.structure < 0.0f) patch.structure = 0.0f;
  patch.brightness = shared_brightness;
  patch.damping = shared_damping;
  patch.position = shared_position;

  // No input jack: silence in (see the mapping notes above). 0.0f is
  // all-zero bytes, so memset is an exact float-zero fill.
  memset(in, 0, sizeof(in));

  uint32_t t0 = time_us_32();
  // ONE part renders per block (rings.cc:98-116): modes 0-3 -> Part, mode 4
  // (easter egg) -> StringSynthPart.
  if (applied_mode == rings::MODE_EASTER_EGG) {
    string_synth.Process(perf, patch, in, out, aux, kBlockSize);
  } else {
    part.Process(perf, patch, in, out, aux, kBlockSize);
  }
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  active_mode_report = (uint32_t)applied_mode;
  active_model_report = (uint32_t)applied_model;

  // Meters for the status LED: OUT/AUX block RMS normalized to 0..1, held as
  // MAX since the last 2 s report (reported as x1000 ints, like Plaits).
  float out_acc = 0.0f, aux_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    out_acc += out[i] * out[i];
    aux_acc += aux[i] * aux[i];
  }
  const float out_rms = sqrtf(out_acc / kBlockSize);
  const float aux_rms = sqrtf(aux_acc / kBlockSize);
  if (out_rms > shared_out_meter) shared_out_meter = out_rms;
  if (aux_rms > shared_aux_meter) shared_aux_meter = aux_rms;

  // The writes pace loop1: one iteration = one 24-frame DMA block, blocks
  // free every 750 us (24 / 32 kHz); > 1400 us (~1.8x the block period, the
  // ratio convention Clouds set at 1800/1000 and Plaits at 900/500) means no
  // buffer freed for ~2 block periods -> I2S output stalled. Rings has no
  // FFT bursts, so the DMA bank only needs to absorb scheduling jitter.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    // Clip16 is OURS (rings has no internal int16 clipper; the original
    // firmware clips at the codec write, rings.cc:120-121 — the Parts'
    // internal Limiters are soft limiters, not hard clippers).
    const int16_t l = static_cast<int16_t>(
        stmlib::Clip16(static_cast<int32_t>(out[i] * 32768.0f)));
    const int16_t r = static_cast<int16_t>(
        stmlib::Clip16(static_cast<int32_t>(aux[i] * 32768.0f)));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > 1400) {
    ++i2s_stall_count;
  }
}

// ================= core 0: control =================
void setup() {
  TinyUSBDevice.setProductDescriptor("Rings RP2350");
  Serial.begin(115200);

  // Rings engine: both Parts over the one shared reverb buffer (rings.cc:43,
  // :137-138). Defaults match Part::Init (MODAL, polyphony 1) and
  // StringSynthPart::Init; the model byte 0 also becomes FX_FORMANT so CC 18
  // means the same index in every mode (StringSynthPart::Init itself starts
  // on FX_ENSEMBLE).
  memset(reverb_buffer, 0, sizeof(reverb_buffer));
  part.Init(reverb_buffer);
  string_synth.Init(reverb_buffer);
  part.set_polyphony(1);
  part.set_model(rings::RESONATOR_MODEL_MODAL);
  string_synth.set_polyphony(1);
  string_synth.set_fx(rings::FX_FORMANT);
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
  // 32 buffers x 16 words = 512 stereo frames = 16 ms bank at 32 kHz (one
  // 32-bit word per buffer = one 16-bit stereo frame; 512/32000 = 16 ms —
  // the same bank math as the Clouds port). No bursts in rings; the bank
  // absorbs scheduling jitter and USB-host blips on core 1's neighbor. Bank
  // RAM is cheap (512 frames x 4 B = 2 KB).
  i2s.setBuffers(32, 16);
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

  usb_midi.setStringDescriptor("Rings RP2350");
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
  Serial.println("Rings engine up (5 modes, 6 resonator models, 32 kHz / 24-sample blocks, USB MIDI device + PIO-USB host)");
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

  // --- stuck-gate watchdog (defensive; gate sustains the bow/blow exciters) ---
  if (shared_gate && now - last_note_event_ms > 30000UL) {
    shared_gate = false;
    note_stack.clear();
    Serial.println("WARNING: gate stuck >30s (lost note-off?) - forced off, note stack cleared");
  }

  // One-line diagnostics report: worst render time + I2S stall counter +
  // applied mode/model + output meters + PIO-USB host tail. Budget = 750 us
  // (24-sample block @ 32 kHz; kBudgetUs multiplies before dividing).
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | mode=%u model=%u out=%u aux=%u notes=%u | host: conn=%u st=%u dev=%u midi=%u frame=%u\r\n",
                  proc_us_max,
                  kBudgetUs,
                  stalls, stalls - last_stall_total,
                  (unsigned)active_mode_report,
                  (unsigned)active_model_report,
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
