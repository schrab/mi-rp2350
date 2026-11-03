// Rack RP2350 — CLOUDS adapter (boot-select via RACK_BOOT_MODULE == 1).
//
// Lifted VERBATIM from clouds-rp2350/src/main.cpp + midi.h: two-region buffer
// split (large = mono memory + FX workspace, small = stereo 16-bit sample
// memory, same as the original F405), the MIDI-triggered pitched-pluck/tone
// test exciter, Prepare()-every-block (the silent-early-return lesson), the
// pending-slot discipline, the full CC map and the max-since-report meters.
// Only the SELECTED adapter is compiled, so the 192 KB of sample memory is
// the only module buffer state in RAM.
#pragma once

// Module-library includes UNCONDITIONAL (see module_elements.h): all four
// module libs are linked in every rack env; only the state + hook bodies
// below are compile-time guarded.
#include <STMLIB.h>
#include <CLOUDS.h>

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#if RACK_BOOT_MODULE == 1  // ---- active only in [env:rack-clouds] ----

static const char* const kModuleName = "clouds";
static const uint32_t kModuleSampleRate = 32000;
static const size_t kModuleBlockSize = clouds::kMaxBlockSize;   // 32 = 1 ms
static const unsigned kModuleBudgetUs =
    (unsigned)(1000000UL * kModuleBlockSize / kModuleSampleRate);  // 1000
static const uint32_t kModuleI2sBuffers = 32;   // 32 x 16 words = 512 frames
static const uint32_t kModuleI2sWords = 16;     // = 16 ms bank (spectral burst)
static const uint32_t kModuleStallUs = 1800;    // >2x the 1000 us block period

// --- engine state (identical sizes/objects to the standalone main.cpp) ---
// Sample memory, mirroring the original F405 two-region split: large =
// mono/low-fidelity memory + FX workspace, small = the stereo 16-bit sample
// memory (32768 samples/ch = 1.02 s @ 32 kHz).
static uint32_t large_buffer[32768];   // 128 KB
static uint32_t small_buffer[16384];   //  64 KB

static clouds::GranularProcessor processor;
static clouds::FloatFrame clouds_in[kModuleBlockSize];
static clouds::FloatFrame clouds_out[kModuleBlockSize];

// Panel parameters (core 0 writes, core 1 copies into the processor between
// blocks — same shared-state pattern as the standalone port).
static volatile float shared_exciter_tone = 0.5f;
static volatile float shared_exciter_level = 0.8f;
static volatile float shared_exciter_mode = 0.0f;
static volatile float shared_position = 0.35f;
static volatile float shared_size = 0.5f;
static volatile float shared_pitch = 0.5f;
static volatile float shared_density = 0.75f;
static volatile float shared_texture = 0.5f;
static volatile float shared_dry_wet = 0.8f;
static volatile float shared_stereo_spread = 0.5f;
static volatile float shared_feedback = 0.0f;
static volatile float shared_reverb = 0.3f;

static volatile int8_t pending_playback_mode = -1;
static volatile int8_t pending_quality = -1;
static volatile int8_t pending_freeze = -1;
static volatile bool   pending_trigger = false;

// --- test exciter (standalone input source; see the standalone main.cpp) ---
static float burst_env = 0.0f;
static float burst_phase = 0.0f;
static float tone_phase = 0.0f;
static float noise_lp_state = 0.0f;
static uint32_t rng_state = 0x1fff7a10;

static inline float ExciterSample(float note_hz) {
  const float level = shared_exciter_level;
  if (level <= 0.0f) return 0.0f;
  float s;
  if (shared_exciter_mode < 0.5f) {
    // Pitched pluck: sine at the note + a noise attack transient, exp decay.
    const float inc = note_hz / kModuleSampleRate;
    burst_phase += inc;
    if (burst_phase >= 1.0f) burst_phase -= 1.0f;
    rng_state = rng_state * 1664525u + 1013904223u;
    const float noise = static_cast<float>(static_cast<int32_t>(rng_state) >> 9) /
                        4194304.0f;   // ±1.0
    const float cutoff = 0.003f + 0.12f * shared_exciter_tone;
    noise_lp_state += cutoff * (noise - noise_lp_state);
    const float tone = sinf(2.0f * 3.14159265f * burst_phase);
    s = (tone * 0.7f + noise_lp_state * 0.3f) * burst_env * 0.8f;
    burst_env *= 0.99985f;            // ~0.8 s tail at 32 kHz
  } else {
    // Gated tone: sine at the held note (continuous grain material).
    const float inc = note_hz / kModuleSampleRate;
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

void module_init() {
  // Same memory partition as the original F405 firmware.
  memset(large_buffer, 0, sizeof(large_buffer));
  memset(small_buffer, 0, sizeof(small_buffer));
  processor.Init(large_buffer, sizeof(large_buffer),
                 small_buffer, sizeof(small_buffer));
  processor.set_sample_rate(static_cast<float>(kModuleSampleRate));
  processor.set_playback_mode(clouds::PLAYBACK_MODE_GRANULAR);
  processor.set_quality(0);          // 16-bit stereo
  processor.Prepare();
}

void module_render() {
  // Apply pending changes strictly between two Process() calls.
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

  // Copy the panel parameters between blocks.
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
    burst_phase = 0.0f;
  }
  previous_gate = shared_gate;

  processor.mutable_parameters()->gate = shared_gate;
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    const float s = ExciterSample(note_hz);
    clouds_in[i].l = s;
    clouds_in[i].r = s;
  }

  uint32_t t0 = time_us_32();
  // Prepare() MUST run every block (original firmware contract): it applies
  // mode/quality changes and updates previous_playback_mode_; without it the
  // processor latches into its silent "mode mismatch" early-return forever.
  processor.Prepare();
  processor.Process(clouds_in, clouds_out, kModuleBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters: input envelope (exciter channel) and output envelope, MAX since
  // the last 2 s report (a sustained tone would otherwise read 0 at print).
  float in_acc = 0.0f, out_acc = 0.0f;
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    in_acc += clouds_in[i].l * clouds_in[i].l;
    out_acc += clouds_out[i].l * clouds_out[i].l;
  }
  float in_rms = sqrtf(in_acc / kModuleBlockSize);
  float out_rms = sqrtf(out_acc / kModuleBlockSize);
  if (in_rms > rack_meter_a) rack_meter_a = in_rms;
  if (out_rms > rack_meter_b) rack_meter_b = out_rms;

  // The writes pace the render loop: one iteration = one 32-frame DMA block,
  // blocks free every 1000 us; > 1800 us = stalled.
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kModuleBlockSize; ++i) {
    int16_t l = stmlib::Clip16(static_cast<int32_t>(clouds_out[i].l * 32768.0f));
    int16_t r = stmlib::Clip16(static_cast<int32_t>(clouds_out[i].r * 32768.0f));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > kModuleStallUs) {
    ++i2s_stall_count;
  }
}

void module_note_on(uint8_t note, uint8_t vel) {
  (void)note; (void)vel;
  pending_trigger = true;         // Clouds TRIGGER input pulses on every note-on
}

void module_note_off(uint8_t note) {
  (void)note;   // gate handled by the shared note stack
}

void module_control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 1:  shared_exciter_tone = v; break;
    case 11: shared_exciter_level = v; break;
    case 13: shared_exciter_mode = v; break;
    case 14: shared_position     = v; break;
    case 15: shared_size         = v; break;
    case 16: shared_pitch        = v; break;
    case 17: shared_density      = v; break;
    case 18: shared_texture      = v; break;
    case 19: shared_dry_wet      = v; break;
    case 20: shared_stereo_spread = v; break;
    case 21: shared_feedback     = v; break;
    case 22: shared_reverb       = v; break;
    // Playback mode selector: 0 granular / 1 stretch / 2 looping delay /
    // 3 spectral. Values > 3 are ignored; applied between render blocks.
    case 23:
      if (val <= 3) pending_playback_mode = static_cast<int8_t>(val);
      break;
    // Quality selector: 0 = 16-bit stereo, 1 = 16-bit mono, 2 = 8-bit stereo,
    // 3 = 8-bit mono. Changing quality resets the recording buffers in
    // Prepare().
    case 24:
      if (val <= 3) pending_quality = static_cast<int8_t>(val);
      break;
    // FREEZE: absolute held value (the panel knows the switch state).
    case 65: pending_freeze = val >= 64 ? 1 : 0; break;
    // TRIGGER: one-shot pulse consumed by core 1 on the next block.
    case 66: if (val >= 64) pending_trigger = true; break;
    default: break;
  }
}

#endif  // RACK_BOOT_MODULE == 1
