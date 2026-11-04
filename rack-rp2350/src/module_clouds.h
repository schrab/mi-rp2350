// Rack RP2350 — CLOUDS adapter.
//
// Lifted VERBATIM from clouds-rp2350/src/main.cpp + midi.h: two-region buffer
// split (large = mono memory + FX workspace, small = stereo 16-bit sample
// memory, same as the original F405), the MIDI-triggered pitched-pluck/tone
// test exciter, Prepare()-every-block (the silent-early-return lesson), the
// pending-slot discipline, the full CC map and the max-since-report meters.
//
// T2: compiles in EVERY rack env (state is arena-placed, see rack_arena.h);
// the two sample-memory regions carve from the BUFFER arena (128 KiB large
// + 64 KiB small = the arena's full 192 KiB).
#pragma once

// Module-library includes UNCONDITIONAL (see module_elements.h): all four
// module libs are linked in every rack env.
#include <STMLIB.h>
#include <CLOUDS.h>

#include <new>                   // placement-new into the arenas

#include "stmlib/dsp/dsp.h"      // stmlib::Clip16
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"

namespace rack_modules {
namespace mod_clouds {

static const char* const kName = "clouds";
static const uint32_t kSampleRate = 32000;
static const size_t kBlockSize = clouds::kMaxBlockSize;   // 32 = 1 ms
static const uint32_t kBudgetUs =
    (uint32_t)(1000000UL * kBlockSize / kSampleRate);  // 1000
static const uint32_t kI2sBuffers = 32;   // 32 x 16 words = 512 frames
static const uint32_t kI2sWords = 16;     // = 16 ms bank (spectral burst)
static const uint32_t kStallUs = 1800;    // >2x the 1000 us block period

static const char* const kCcMap =
    "1 ex-tone, 11 ex-level, 13 ex-mode, 14 position, 15 size, 16 pitch, "
    "17 density, 18 texture, 19 dry/wet, 20 spread, 21 feedback, 22 reverb, "
    "23 mode, 24 quality, 65 freeze, 66 trigger";
static const char* const kModDestinations = "";  // T3 placeholder

// --- engine state: arena-placed (was file-scope before T2) ---
struct State {
  clouds::GranularProcessor processor;
  clouds::FloatFrame clouds_in[kBlockSize];
  clouds::FloatFrame clouds_out[kBlockSize];

  // Panel parameters (core 0 writes, core 1 copies into the processor between
  // blocks — same shared-state pattern as the standalone port).
  volatile float shared_exciter_tone;
  volatile float shared_exciter_level;
  volatile float shared_exciter_mode;
  volatile float shared_position;
  volatile float shared_size;
  volatile float shared_pitch;
  volatile float shared_density;
  volatile float shared_texture;
  volatile float shared_dry_wet;
  volatile float shared_stereo_spread;
  volatile float shared_feedback;
  volatile float shared_reverb;

  volatile int8_t pending_playback_mode;
  volatile int8_t pending_quality;
  volatile int8_t pending_freeze;
  volatile bool   pending_trigger;

  // --- test exciter (standalone input source; see the standalone main.cpp) ---
  float burst_env;
  float burst_phase;
  float tone_phase;
  float noise_lp_state;
  uint32_t rng_state;
  bool previous_gate;
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

// Buffer carve offsets (the arenas are unions — only the ACTIVE module's
// carve is live; see rack_arena.h).
static const size_t kLargeBufferBytes = 131072;   // 32768 uint32
static const size_t kSmallBufferBytes = 65536;    // 16384 uint32

static inline float ExciterSample(float note_hz) {
  const float level = s->shared_exciter_level;
  if (level <= 0.0f) return 0.0f;
  float sv;
  if (s->shared_exciter_mode < 0.5f) {
    // Pitched pluck: sine at the note + a noise attack transient, exp decay.
    const float inc = note_hz / kSampleRate;
    s->burst_phase += inc;
    if (s->burst_phase >= 1.0f) s->burst_phase -= 1.0f;
    s->rng_state = s->rng_state * 1664525u + 1013904223u;
    const float noise = static_cast<float>(static_cast<int32_t>(s->rng_state) >> 9) /
                        4194304.0f;   // ±1.0
    const float cutoff = 0.003f + 0.12f * s->shared_exciter_tone;
    s->noise_lp_state += cutoff * (noise - s->noise_lp_state);
    const float tone = sinf(2.0f * 3.14159265f * s->burst_phase);
    sv = (tone * 0.7f + s->noise_lp_state * 0.3f) * s->burst_env * 0.8f;
    s->burst_env *= 0.99985f;         // ~0.8 s tail at 32 kHz
  } else {
    // Gated tone: sine at the held note (continuous grain material).
    const float inc = note_hz / kSampleRate;
    s->tone_phase += inc;
    if (s->tone_phase >= 1.0f) s->tone_phase -= 1.0f;
    sv = sinf(2.0f * 3.14159265f * s->tone_phase) * 0.6f;
    // A little breath so the granulator has transients to work with.
    s->rng_state = s->rng_state * 1664525u + 1013904223u;
    sv += static_cast<float>(static_cast<int32_t>(s->rng_state) >> 14) /
          262144.0f * 0.05f * s->shared_exciter_tone;
  }
  return sv * level;
}

void init() {
  s = new (g_rack_object_arena) State();
  // Buffer carve (rack_arena.h map): large @0, small @131072.
  uint32_t* large_buffer = reinterpret_cast<uint32_t*>(g_rack_buffer_arena);
  uint32_t* small_buffer =
      reinterpret_cast<uint32_t*>(g_rack_buffer_arena + kLargeBufferBytes);
  memset(large_buffer, 0, kLargeBufferBytes);
  memset(small_buffer, 0, kSmallBufferBytes);

  s->pending_playback_mode = -1;
  s->pending_quality = -1;
  s->pending_freeze = -1;
  s->pending_trigger = false;
  s->shared_exciter_tone = 0.5f;
  s->shared_exciter_level = 0.8f;
  s->shared_exciter_mode = 0.0f;
  s->shared_position = 0.35f;
  s->shared_size = 0.5f;
  s->shared_pitch = 0.5f;
  s->shared_density = 0.75f;
  s->shared_texture = 0.5f;
  s->shared_dry_wet = 0.8f;
  s->shared_stereo_spread = 0.5f;
  s->shared_feedback = 0.0f;
  s->shared_reverb = 0.3f;
  s->burst_env = 0.0f;
  s->burst_phase = 0.0f;
  s->tone_phase = 0.0f;
  s->noise_lp_state = 0.0f;
  s->rng_state = 0x1fff7a10;
  s->previous_gate = false;

  // Same memory partition as the original F405 firmware.
  s->processor.Init(large_buffer, kLargeBufferBytes,
                    small_buffer, kSmallBufferBytes);
  s->processor.set_sample_rate(static_cast<float>(kSampleRate));
  s->processor.set_playback_mode(clouds::PLAYBACK_MODE_GRANULAR);
  s->processor.set_quality(0);          // 16-bit stereo
  s->processor.Prepare();
}

void render() {
  // Apply pending changes strictly between two Process() calls.
  if (s->pending_playback_mode >= 0) {
    s->processor.set_playback_mode(
        static_cast<clouds::PlaybackMode>(s->pending_playback_mode));
    s->pending_playback_mode = -1;
  }
  if (s->pending_quality >= 0) {
    s->processor.set_quality(s->pending_quality);
    s->pending_quality = -1;
  }
  if (s->pending_freeze >= 0) {
    s->processor.set_freeze(s->pending_freeze != 0);
    s->pending_freeze = -1;
  }
  if (s->pending_trigger) {
    s->processor.mutable_parameters()->trigger = true;
    s->pending_trigger = false;
  } else {
    s->processor.mutable_parameters()->trigger = false;
  }

  // Copy the panel parameters between blocks.
  clouds::Parameters* p = s->processor.mutable_parameters();
  p->position       = s->shared_position;
  p->size           = s->shared_size;
  p->pitch          = s->shared_pitch;
  p->density        = s->shared_density;
  p->texture        = s->shared_texture;
  p->dry_wet        = s->shared_dry_wet;
  p->stereo_spread  = s->shared_stereo_spread;
  p->feedback       = s->shared_feedback;
  p->reverb         = s->shared_reverb;

  // Exciter -> input frames. Note-on retriggers the burst envelope (rising
  // gate edge); the pitch tracks the newest held note + bend.
  const float note_hz = 440.0f * powf(2.0f, (shared_note - 69.0f) / 12.0f);
  if (shared_gate && !s->previous_gate) {
    s->burst_env = 1.0f;
    s->burst_phase = 0.0f;
  }
  s->previous_gate = shared_gate;

  s->processor.mutable_parameters()->gate = shared_gate;
  for (size_t i = 0; i < kBlockSize; ++i) {
    const float sv = ExciterSample(note_hz);
    s->clouds_in[i].l = sv;
    s->clouds_in[i].r = sv;
  }

  uint32_t t0 = time_us_32();
  // Prepare() MUST run every block (original firmware contract): it applies
  // mode/quality changes and updates previous_playback_mode_; without it the
  // processor latches into its silent "mode mismatch" early-return forever.
  s->processor.Prepare();
  s->processor.Process(s->clouds_in, s->clouds_out, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters: input envelope (exciter channel) and output envelope, MAX since
  // the last 2 s report (a sustained tone would otherwise read 0 at print).
  float in_acc = 0.0f, out_acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) {
    in_acc += s->clouds_in[i].l * s->clouds_in[i].l;
    out_acc += s->clouds_out[i].l * s->clouds_out[i].l;
  }
  float in_rms = sqrtf(in_acc / kBlockSize);
  float out_rms = sqrtf(out_acc / kBlockSize);
  if (in_rms > rack_meter_a) rack_meter_a = in_rms;
  if (out_rms > rack_meter_b) rack_meter_b = out_rms;

  // The writes pace the render loop: one iteration = one 32-frame DMA block,
  // blocks free every 1000 us; > 1800 us = stalled. g_render_gain is the
  // swap-mute ramp (1.0 in normal render).
  uint32_t w0 = time_us_32();
  const float gain = g_render_gain;
  for (size_t i = 0; i < kBlockSize; ++i) {
    int16_t l = stmlib::Clip16(
        static_cast<int32_t>(s->clouds_out[i].l * 32768.0f * gain));
    int16_t r = stmlib::Clip16(
        static_cast<int32_t>(s->clouds_out[i].r * 32768.0f * gain));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > kStallUs) {
    ++i2s_stall_count;
  }
}

void note_on(uint8_t note, uint8_t vel) {
  (void)note; (void)vel;
  s->pending_trigger = true;      // Clouds TRIGGER input pulses on every note-on
}

void note_off(uint8_t note) {
  (void)note;   // gate handled by the shared note stack
}

void control_change(uint8_t cc, uint8_t val) {
  const float v = (float)val / 127.0f;
  switch (cc) {
    case 1:  s->shared_exciter_tone = v; break;
    case 11: s->shared_exciter_level = v; break;
    case 13: s->shared_exciter_mode = v; break;
    case 14: s->shared_position     = v; break;
    case 15: s->shared_size         = v; break;
    case 16: s->shared_pitch        = v; break;
    case 17: s->shared_density      = v; break;
    case 18: s->shared_texture      = v; break;
    case 19: s->shared_dry_wet      = v; break;
    case 20: s->shared_stereo_spread = v; break;
    case 21: s->shared_feedback     = v; break;
    case 22: s->shared_reverb       = v; break;
    // Playback mode selector: 0 granular / 1 stretch / 2 looping delay.
    // Values > 2 are ignored; applied between render blocks.
    // SPECTRAL (3) IS EXCLUDED in the rack (plan, "SRAM architecture"):
    // its 57 KB of CMSIS FFT tables are flash-resident here (memmap_rack.ld
    // header), so the 13.8 ms transform burst would no longer be bank-
    // absorbable — the mode is the designated sacrifice for the shared
    // 112 KiB window that live-swapping requires.
    case 23:
      if (val <= 2) s->pending_playback_mode = static_cast<int8_t>(val);
      break;
    // Quality selector: 0 = 16-bit stereo, 1 = 16-bit mono, 2 = 8-bit stereo,
    // 3 = 8-bit mono. Changing quality resets the recording buffers in
    // Prepare().
    case 24:
      if (val <= 3) s->pending_quality = static_cast<int8_t>(val);
      break;
    // FREEZE: absolute held value (the panel knows the switch state).
    case 65: s->pending_freeze = val >= 64 ? 1 : 0; break;
    // TRIGGER: one-shot pulse consumed by core 1 on the next block.
    case 66: if (val >= 64) s->pending_trigger = true; break;
    default: break;
  }
}

}  // namespace mod_clouds
}  // namespace rack_modules
