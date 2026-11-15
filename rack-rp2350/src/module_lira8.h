// Rack RP2350 — LIRA-8 adapter (rack-lira8, 2026-11-15).
//
// The ONLY rack module that is a hand port rather than a vendored upstream
// C++ lib: the engine (src/lira8/*.h) is a faithful re-implementation of
// MikeMorenoAudio's LIRA-8 Pure Data patch (BSD), developed TDD against
// headless-pd golden renders (rack-rp2350/test/lira8_ref/). Pure C++, no
// MCU headers in the engine — host compilation doubles as the ctag-tbd
// portability proof.
//
// Follows the established adapter shape (module_elements.h / module_plaits.h):
// arena-placed State (engine by value + the delay buffers carved from the
// BUFFER arena — 2 x 32,004 int16 = 128,016 B of the 192 KiB arena, the
// ORIGINAL HARDWARE's 1 s delay cap), pending
// slots for CCs/sensors, the mod-engine hooks, the T1 button twin.
//
// AUDIO CONTRACT: the engine renders pd-style 64-sample blocks internally;
// the rack grid is 16 samples — Process() accumulates 4 host blocks per
// internal block, so every 4th call bursts (up to ~4x its share of the
// block budget). The I2S DMA bank (8 x 16 frames = 4 ms) absorbs the burst
// (PORTING-HIGHLIGHTS §5 rule: bank > spike, average < budget).
//
// SENSOR SEMANTICS (the Lyra-8's 8 touch pads): MIDI notes 36-43 (C2-G2)
// gate sensors 1-8 (note-on = press, note-off = release); mod triggers
// CC 55-62 (LATCH: the sensor follows the source level); the PLAY button
// pulses sensor 1 for 400 ms (the "press a pad" feel).
#pragma once

#include <new>                   // placement-new into the arenas
#include <cstring>

#include "lira8/lira_engine.h"
#include "lira8/lira8_tc.h"
#include "lira8_engine_o3.h"     // -O3 Process() wrapper (lib/lira8_engine)
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"
#include "module_registry.h"     // RackModDestinations (T3 mod routing table)

// Forward declaration of mod_engine destination query (defined in mod_engine.h).
bool mod_is_dest_routed(uint8_t cc);

namespace rack_modules {
namespace mod_lira8 {

static const char* const kName = "lira8";
static const uint32_t kSampleRate = 32000;
static const size_t kBlockSize = 16;     // host grid; the engine bursts 64
// Report budget = the INTERNAL 64-sample block (2 ms at 32 kHz), NOT the
// 16-sample host grid: Process() renders a whole internal block on one call
// (every 4th), so proc_us_max is a single-block number — compare it against
// the block period. The 4x burst is absorbed by the 4 ms I2S bank; the
// per-call stall threshold is kStallUs below.
static const uint32_t kBudgetUs =
    (uint32_t)(1000000UL / kSampleRate * l8pd::Lira8Engine::kBlockSize);  // 2000
static const uint32_t kI2sBuffers = 8;   // 8 x 16 words = 128 frames (~4 ms)
static const uint32_t kI2sWords = 16;
static const uint32_t kStallUs = 900;    // >2x the 500 us block period

// Delay storage: the ORIGINAL HARDWARE's delay tops out at 1 SECOND (user,
// 2026-11-15) — the Pd patch's 5.944 s range is a patch-ism. int16, 2 lines:
// (32000+4) * 2 * 2 = 128,016 B of the 192 KiB buffer arena. The pd TIME law
// is kept (2^(12k/127)*1.45125 ms), so the top ~22% of the TIME knob
// saturates at the cap, like a pot hitting its stop.
static constexpr float kCapMs = 1000.0f;
static constexpr size_t kDelayBytes = l8pd::Lira8Engine::delay_bytes_for(kCapMs, kSampleRate);
static_assert(kDelayBytes <= kRackBufferArenaBytes,
              "lira8 delay carve exceeds the rack buffer arena");

// CC map (the spec §12 order, remapped OFF the rack-reserved CCs:
// 32 = module switch, 33/34/35 = mod mux, 40-50 = clouds alias bank,
// 64/65/66 = pseudo-CCs). Sensors also speak MIDI notes 36-43.
static const char* const kCcMap =
    "1-8 tune, 9-12 fast, 13-16 sharp, 17-20 mod, 21-24 source, "
    "25-26 pitch, 27-28 hold, 29 switch, 30 total-fb, 31 vibrato, "
    "36 drv, 37 dst-mix, 38 vol, 39 f-a, 41 f-b, 42 andor, 43 link, "
    "44 time-1, 45 time-2, 46 feedback, 47 del-mix, 48-49 mod-1/2, "
    "51 delay-mode (0 self / 1 lfo-tri / 2 lfo-sqr), 52 lfo-wave direct, "
    "53 quantize, 55-62 sensors 1-8 (latch)";

// T3 mod destinations: all 47 continuous params (the spec §12 order mapped
// to the CC numbers above); triggers = the 8 sensors (LATCH — a sensor gate
// follows the source level, exactly like the patch's tgl semantics).
static const uint8_t kModContinuousCcs[] = {
    1, 2, 3, 4, 5, 6, 7, 8,                 // tune 1-8
    13, 14, 15, 16,                         // sharp
    17, 18, 19, 20,                         // mod
    25, 26,                                 // pitch
    27, 28,                                 // hold
    36, 37, 38,                             // drv, dst-mix, vol
    39, 41,                                 // f-a, f-b
    44, 45, 46, 47, 48, 49,                 // time-1/2, feedback, del-mix, mod-1/2
};
static const uint8_t kModTriggerCcs[] = {55, 56, 57, 58, 59, 60, 61, 62};
static const uint8_t kModTriggerKinds[] = {
    RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH,
    RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH,
};
static const RackModDestinations kModDestinations = {
    "lira8: continuous tune 1-8, sharp 13-16, mod 17-20, pitch 25-26, "
    "hold 27-28, drv/vol 36/38, times 44-45, fb 46 | triggers: 55-62 "
    "sensors 1-8 (latch); notes 36-43 = sensors (the patch's notemap)",
    sizeof(kModContinuousCcs), kModContinuousCcs,
    8, kModTriggerCcs, kModTriggerKinds,
};

// --- engine state: arena-placed (the engine BY VALUE; the delay buffers
// carve from the BUFFER arena at init — 128,016 B of 192 KiB) ---
struct State {
  l8pd::Lira8Engine eng;

  // T1 hardware PLAY button: pulse sensor 1 for kPluckWindowMs.
  volatile bool     pluck_active;
  volatile uint32_t pluck_until_ms;

  // 2 s report mirrors (core 0 publishes the requested state, core 1 the
  // output RMS).
  volatile float    out_rms_report;
  volatile uint8_t  sensor_mask;   // bit N = sensor N+1 requested ON

  explicit State(int16_t* delay_mem)
      : eng(32000.0f, kCapMs, delay_mem),
        pluck_active(false),
        pluck_until_ms(0),
        out_rms_report(0.0f),
        sensor_mask(0) {}
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

static size_t state_size() { return sizeof(State); }

void init_at(uint8_t* base) {
  // Delay carve from the BUFFER arena offset 0 (the union discipline: the
  // active module owns the arena; a swap re-carves for the next module).
  memset(g_rack_buffer_arena, 0, kDelayBytes);
  s = new (base) State(reinterpret_cast<int16_t*>(g_rack_buffer_arena));
}

void init() { init_at(g_rack_object_arena); }

// Every sensor write goes through here so the report mask stays honest.
static void set_sensor(int8_t idx, bool on) {
  s->eng.set_sensor(idx, on);
  if (idx >= 0 && idx < 8) {
    if (on) s->sensor_mask |= (1u << idx);
    else    s->sensor_mask &= ~(1u << idx);
  }
}

// T1 hardware PLAY button twin (see module_plaits.h): press = sensor 1
// pressed for 400 ms (a musical "hit the pad"), release ends it early.
static const uint32_t kPluckWindowMs = 400;

void button_trig(bool down) {
  if (down) {
    set_sensor(0, true);
    s->pluck_active = true;
    s->pluck_until_ms = millis() + kPluckWindowMs;
  } else if (s->pluck_active) {
    set_sensor(0, false);
    s->pluck_active = false;
  }
}

void pluck_poll() {
  if (s->pluck_active && millis() >= s->pluck_until_ms) {
    set_sensor(0, false);
    s->pluck_active = false;
  }
}

// T6 chain A-slot render — NOT provided: lira8's delay carve takes ~65% of
// the BUFFER arena, and a chained clouds-B needs another 131 KiB above it
// (259,088 B > 192 KiB). The chain pair table in main.cpp must not offer
// lira8 as chain A (v1: rack-lira8 boots standalone).

void render() {
  pluck_poll();

  uint32_t t0 = time_us_32();
  float l[kBlockSize], r[kBlockSize];
  l8pd::process_block_o3(s->eng, l, r, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters: output block RMS (mono — both channels carry the same signal),
  // max since the last 2 s report.
  float acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) acc += l[i] * l[i];
  const float rms = sqrtf(acc / kBlockSize);
  if (rms > rack_meter_a) rack_meter_a = rms;
  s->out_rms_report = rms;

  // The writes pace the render loop: one iteration = one 16-frame DMA block,
  // blocks free every 500 us; > 900 us = stalled. g_render_gain = the
  // swap-mute ramp.
  const float gain = g_render_gain;
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out = static_cast<int16_t>(l[i] * gain * 32768.0f);
    i2s.write16(out, out);
  }
  if (time_us_32() - w0 > kStallUs) {
    ++i2s_stall_count;
  }
}

void note_on(uint8_t note, uint8_t vel) {
  (void)vel;
  // The patch's notemap: notes 36-43 (C2-G2) = sensors 1-8, note-on = press.
  if (note >= 36 && note <= 43) set_sensor(note - 36, true);
}

void note_off(uint8_t note) {
  if (note >= 36 && note <= 43) set_sensor(note - 36, false);
}

// CC -> engine param index (the spec §12 order; the CC numbers avoid the
// rack-reserved ranges). Returns -1 for CCs that are not params.
static int8_t cc_to_param(uint8_t cc) {
  if (cc >= 1 && cc <= 31) return static_cast<int8_t>(cc - 1);       // 0-30
  if (cc >= 36 && cc <= 39) return static_cast<int8_t>(cc - 5);      // 31-34
  if (cc >= 41 && cc <= 49) return static_cast<int8_t>(cc - 6);      // 35-43
  if (cc >= 51 && cc <= 53) return static_cast<int8_t>(cc - 7);      // 44-46
  return -1;
}

void control_change(uint8_t cc, uint8_t val) {
  // PLAY pseudo-CC (64, the registry button path): pulse sensor 1.
  if (cc == 64) {
    button_trig(val >= 64);
    return;
  }
  // Sensor latch CCs 55-62 (also reachable as mod triggers).
  if (cc >= 55 && cc <= 62) {
    set_sensor(cc - 55, val >= 64);
    return;
  }
  // CC 51 = the combined DELAY MODE switch. The hardware has a two-way
  // SELF/LFO switch plus a separate LFO-waveform switch; since the LFO does
  // not shape the delay in SELF mode, the UI merges them into one 3-state
  // selector: 0 = SELF, 1 = LFO triangle, 2 = LFO square. Small values are
  // the C3's discrete states; larger MIDI values map in thirds.
  if (cc == 51) {
    int mode;
    if (val <= 2) mode = val;
    else mode = (val < 43) ? 0 : (val < 85 ? 1 : 2);
    s->eng.set_param(44, (mode == 0) ? 0.0f : 2.0f);        // del-mod
    if (mode > 0) s->eng.set_param(45, (mode == 2) ? 1.0f : 0.0f);
    return;
  }
  const int8_t idx = cc_to_param(cc);
  if (idx >= 0) s->eng.set_param(idx, static_cast<float>(val));
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Same between-blocks discipline as every adapter: the mod engine only runs
// while not swapping, so `s` is always the live state. The engine applies
// pending params between its internal blocks (the pending_params_ slots).

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  if (cc == 51) {   // combined delay-mode selector (see control_change)
    const int mode = (v < 0.34f) ? 0 : (v < 0.67f ? 1 : 2);
    s->eng.set_param(44, (mode == 0) ? 0.0f : 2.0f);
    if (mode > 0) s->eng.set_param(45, (mode == 2) ? 1.0f : 0.0f);
    return true;
  }
  const int8_t idx = cc_to_param(cc);
  if (idx < 0) return false;
  s->eng.set_param(idx, v * 127.0f);
  return true;
}

// Trigger destinations: CCs 55-62 = sensors 1-8, LATCH semantics (the gate
// follows the source level — the patch's tgl behavior).
bool mod_trigger_cc(uint8_t cc, bool high) {
  if (cc >= 55 && cc <= 62) {
    set_sensor(cc - 55, high);
    return true;
  }
  return false;
}

}  // namespace mod_lira8
}  // namespace rack_modules
