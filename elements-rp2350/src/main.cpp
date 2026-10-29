// Task 3 — Elements engine integration: Part::Process() rendered on core 1
// at 32 kHz, pushed to the PCM5102A via I2S. Core 0 runs the auto-trigger
// bring-up test (strike every 1.5 s, gate held 300 ms, notes stepping 36..47)
// and prints the max Process-time report every 2 s.
//
// Wiring (PCM5102A): BCK -> GP6 (LRCK/WS follows on GP7 automatically),
// DIN -> GP8, SCK/MCLK -> GND, XSMT -> 3V3 (unmute), FLT/DEMP/FMT -> GND.
#include <Arduino.h>
#include <I2S.h>
#include <Adafruit_TinyUSB.h>

#include <STMLIB.h>
#include <ELEMENTS.h>

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16 (not pulled in by the wrappers)
#include "elements/dsp/dsp.h"
#include "elements/dsp/part.h"

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

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);

// --- engine ---
static uint16_t reverb_buffer[32768];   // 64 KB, verified FxEngine<32768>
static elements::Part part;
static elements::Patch* patch;
static elements::PerformanceState perf_state;

static float blow_in[kBlockSize];
static float strike_in[kBlockSize];
static float main_out[kBlockSize];
static float aux_out[kBlockSize];

// --- shared MIDI state (core0 writes, core1 reads) ---
static volatile float shared_note = 48.0f;
static volatile float shared_modulation = 0.0f;
static volatile float shared_strength = 0.8f;
static volatile bool shared_gate = false;

static volatile bool audio_ready = false;

// --- perf telemetry (core1 writes, core0 prints) ---
static volatile uint32_t proc_us_max = 0;

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
  i2s.setBitsPerSample(16);
  i2s.setBuffers(8, 16);          // 8 buffers x 16 words = 128 stereo frames (~4 ms)
  i2s.setFrequency(kSampleRate);
  if (!i2s.begin()) {
    Serial.println("I2S begin FAILED");
  } else {
    Serial.println("I2S up");
  }
  audio_ready = true;
  Serial.println("Elements engine up (auto-trigger test)");
}

void loop() {
  // bring-up test: strike every 1.5 s, gate held 300 ms
  static uint32_t last_trigger = 0;
  static uint32_t last_report = 0;
  static uint8_t step = 0;
  uint32_t now = millis();

  if (now - last_trigger > 1500) {
    last_trigger = now;
    shared_note = 36.0f + (step++ % 12);
    shared_strength = 0.9f;
    shared_gate = true;
  }
  if (now - last_trigger > 300) {
    shared_gate = false;
  }
  if (now - last_report > 2000) {
    last_report = now;
    Serial.printf("max Process: %u us (budget %u us)\n", proc_us_max, (unsigned)(1000000UL / kSampleRate * kBlockSize));
    proc_us_max = 0;
  }
}
