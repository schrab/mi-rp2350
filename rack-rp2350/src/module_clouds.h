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
#include "module_registry.h"     // RackModDestinations (T3 mod routing table)

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
    "23 mode, 24 quality, 65 freeze, 66 trigger, 40-48 = 14-22 (grain alias "
    "bank: single-module clouds + B slot while chained)";
// T3 mod destinations: the grain parameters 14-22 accept continuous
// modulation. 23/24 are EXCLUDED (playback-mode / quality selectors must not
// be modulated — mode changes also reset recording buffers). 1/11/13 (the
// test-exciter knobs) stay un-modulated per the T3 destination list. 65
// (freeze) / 66 (trig) are TRIGGER-ONLY: freeze latches on/off with the
// source level, trigger pulses on rising edges.
static const uint8_t kModContinuousCcs[] = {14, 15, 16, 17, 18, 19, 20, 21, 22};
static const uint8_t kModTriggerCcs[] = {65, 66};
static const uint8_t kModTriggerKinds[] = {RACK_TRIG_LATCH, RACK_TRIG_PULSE};
static const RackModDestinations kModDestinations = {
    "clouds: continuous 14-22 (23/24 excluded: mode/quality selectors) | "
    "triggers: 65 freeze (latch), 66 trig (pulse)",
    9, kModContinuousCcs,
    2, kModTriggerCcs, kModTriggerKinds,
};

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

  // T6: true when this State was placed by init_chain_b() as the chain B
  // slot — the chain loop feeds REAL frames into Process() (the exciter is
  // bypassed entirely) and the playback-mode selector is clamped to
  // granular/looping (see control_change + chain_render_b).
  bool chain_b;
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

// T6 chain hooks (see module_elements.h + main.cpp): state_size()/init_at()
// are the single-module path (unchanged placement/split); init_chain_b()
// places the B-slot State AFTER the chain A state in the object arena and
// carves the CHAINED buffer split from the caller's buffer-arena region.
static size_t state_size() { return sizeof(State); }

void init_at(uint8_t* base) {
  s = new (base) State();
  // Buffer carve (rack_arena.h map): large @0, small @131072.
  uint32_t* large_buffer = reinterpret_cast<uint32_t*>(g_rack_buffer_arena);
  uint32_t* small_buffer =
      reinterpret_cast<uint32_t*>(g_rack_buffer_arena + kLargeBufferBytes);
  memset(large_buffer, 0, kLargeBufferBytes);
  memset(small_buffer, 0, kSmallBufferBytes);
  s->chain_b = false;

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

void init() { init_at(g_rack_object_arena); }

// T6: chain B-slot init. The State goes AFTER the chain A state in the
// OBJECT arena (offset chosen by the caller); the buffers carve from the
// CALLER's region of the BUFFER arena (above the chain A carve).
//
// CHAINED SPLIT (documented deviation): the caller passes
//   large = 98,304 B (= 32,768 sample memory ch0 + 65,536 FX workspace)
//   small = 32,768 B (sample memory ch1)
// GranularProcessor::Init's stereo partition takes the per-channel sample
// memory = the SMALL size from the front of the large buffer + the whole
// small buffer, and the FX workspace = large - small (granular_processor.cc
// Prepare(): buffer_size[0] = buffer_size[1] = buffer_size_[1]; workspace =
// buffer_size_[0] - buffer_size_[1]). So vs the standalone split (64+64 KB
// sample memory, 64 KB FX) the chained split keeps the FX workspace
// IDENTICAL and halves the sample memory to 0.512 s/channel (32,768 B =
// 16,384 stereo frames @ 32 kHz) — forced by the buffer arena's exact fit
// (chain A carve 65,536 + 98,304 + 32,768 = 196,608 = the full arena).
void init_chain_b(uint8_t* obj_base, uint8_t* large, size_t large_bytes,
                  uint8_t* small, size_t small_bytes) {
  s = new (obj_base) State();
  memset(large, 0, large_bytes);
  memset(small, 0, small_bytes);
  s->chain_b = true;
  // The chain path bypasses the exciter entirely (real frames instead);
  // zero the exciter level as defense in depth.
  s->pending_playback_mode = -1;
  s->pending_quality = -1;
  s->pending_freeze = -1;
  s->pending_trigger = false;
  s->shared_exciter_tone = 0.5f;
  s->shared_exciter_level = 0.0f;
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

  s->processor.Init(large, large_bytes, small, small_bytes);
  s->processor.set_sample_rate(static_cast<float>(kSampleRate));  // 32000
  s->processor.set_playback_mode(clouds::PLAYBACK_MODE_GRANULAR);
  s->processor.set_quality(0);          // 16-bit stereo
  s->processor.Prepare();
}

// T6: chain B-slot render — exactly n samples (n <= 24, the chain grid) of
// ROUTED INPUT (the chain A slot's out/aux) through Prepare() + Process(),
// out into the caller's buffers, NO I2S write, NO proc_us_max update (the
// chain loop times A + route + B together). This hook REPLACES the exciter:
// clouds_in gets the routed frames, nothing else. parameters.gate stays
// false — its only consumer is the excluded spectral path's
// FrameTransformation.
//
// SCALE (documented choice): the routed frames pass at GAIN 1.0 — the A
// adapters' float outputs are the same ±1.0 class the write path Clip16s to
// I2S, and the standalone exciter produced 0.6-0.8 peaks, so the routed
// material is at (slightly above) the level the granulator was validated
// with. No fixed gain stage.
void chain_render_b(const float* in_l, const float* in_r,
                    float* out_l, float* out_r, size_t n) {
  // Apply pending changes strictly between two Process() calls — with the
  // CHAIN CLAMP as belt-and-braces: core 0 rejects CC 23 = 1 (stretch) / 3
  // (spectral) loudly while chained; anything illegal that still arrives
  // (future paths) lands on granular instead of stretching the budget.
  if (s->pending_playback_mode >= 0) {
    int8_t mode = s->pending_playback_mode;
    if (s->chain_b && mode != 0 && mode != 2) mode = 0;
    s->processor.set_playback_mode(
        static_cast<clouds::PlaybackMode>(mode));
    s->pending_playback_mode = -1;
  }
  if (s->pending_quality >= 0) {
    int8_t q = s->pending_quality;
    if (s->chain_b) q = 0; // Chained partition strictly requires 16-bit stereo
    s->processor.set_quality(q);
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

  // Copy the panel parameters between blocks (factory defaults while
  // chained — B-CC routing for the grain parameters is future work).
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
  p->gate           = false;

  // ROUTE: the chain input IS the exciter replacement (bypassed entirely).
  for (size_t i = 0; i < n; ++i) {
    s->clouds_in[i].l = in_l[i];
    s->clouds_in[i].r = in_r[i];
  }

  // Prepare() MUST run every block (original firmware contract).
  s->processor.Prepare();
  s->processor.Process(s->clouds_in, s->clouds_out, n);

  for (size_t i = 0; i < n; ++i) {
    out_l[i] = s->clouds_out[i].l;
    out_r[i] = s->clouds_out[i].r;
  }

  // Meters: routed-input RMS / output RMS, max since the last 2 s report —
  // while chained these feed the status LED + report for the PAIR.
  float in_acc = 0.0f, out_acc = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    in_acc += s->clouds_in[i].l * s->clouds_in[i].l;
    out_acc += s->clouds_out[i].l * s->clouds_out[i].l;
  }
  float in_rms = sqrtf(in_acc / n);
  float out_rms = sqrtf(out_acc / n);
  if (in_rms > rack_meter_a) rack_meter_a = in_rms;
  if (out_rms > rack_meter_b) rack_meter_b = out_rms;
}

void render() {
  // Apply pending changes strictly between two Process() calls.
  if (s->pending_playback_mode >= 0) {
    s->processor.set_playback_mode(
        static_cast<clouds::PlaybackMode>(s->pending_playback_mode));
    s->pending_playback_mode = -1;
  }
  if (s->pending_quality >= 0) {
    int8_t q = s->pending_quality;
    if (s->chain_b) q = 0; // Chained partition strictly requires 16-bit stereo
    s->processor.set_quality(q);
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
    // SPECTRAL (3) IS EXCLUDED in the rack (plan, "SRAM architecture") and
    // its code path is DEMOTED to flash since T6b (clouds-tc-demote patch:
    // STFT/pvoc/FrameTransformation + the CMSIS FFT code + the spectral
    // window LUT are no longer window-resident), so the 13.8 ms transform
    // burst would no longer be bank-absorbable — the mode stays the
    // designated sacrifice for the shared two-slot window. Stretch (1)
    // remains fully available single-module (its hot body is inlined in the
    // SRAM-resident ProcessGranular).
    case 23:
      if (val <= 2) {
        if (s->chain_b && val == 1) break;
        s->pending_playback_mode = static_cast<int8_t>(val);
      }
      break;
    // Quality selector: 0 = 16-bit stereo, 1 = 16-bit mono, 2 = 8-bit stereo,
    // 3 = 8-bit mono. Changing quality resets the recording buffers in
    // Prepare(). While chained, strictly locked to 0 (16-bit stereo) because
    // mono/8-bit modes overflow or corrupt the partitioned workspace.
    case 24:
      if (val <= 3) {
        s->pending_quality = s->chain_b ? 0 : static_cast<int8_t>(val);
      }
      break;
    // FREEZE: absolute held value (the panel knows the switch state).
    case 65: s->pending_freeze = val >= 64 ? 1 : 0; break;
    // TRIGGER: one-shot pulse consumed by core 1 on the next block.
    case 66: if (val >= 64) s->pending_trigger = true; break;
    default: break;
  }
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Same between-blocks discipline as the pending slots in render(); the mod
// engine only runs while not swapping, so `s` is always the live state.

// Continuous destination write (0..1 virtual CC position, float precision,
// identical mapping to control_change's CC bytes). 1/11/13 excluded per the
// destination list; 23/24 selectors excluded; 65/66 are trigger-only.
bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  switch (cc) {
    case 14: s->shared_position      = v; break;
    case 15: s->shared_size          = v; break;
    case 16: s->shared_pitch         = v; break;
    case 17: s->shared_density       = v; break;
    case 18: s->shared_texture       = v; break;
    case 19: s->shared_dry_wet       = v; break;
    case 20: s->shared_stereo_spread = v; break;
    case 21: s->shared_feedback      = v; break;
    case 22: s->shared_reverb        = v; break;
    default: return false;
  }
  return true;
}

// Trigger destination write. 65 FREEZE is LATCH-kind: the engine calls it on
// BOTH edges and freeze holds while the source is high. 66 TRIG is PULSE-kind:
// rising edges produce the one-block pending_trigger pulse.
bool mod_trigger_cc(uint8_t cc, bool high) {
  switch (cc) {
    case 65: s->pending_freeze = high ? 1 : 0; return true;
    case 66: if (high) s->pending_trigger = true; return true;
    default: return false;
  }
}

}  // namespace mod_clouds
}  // namespace rack_modules
