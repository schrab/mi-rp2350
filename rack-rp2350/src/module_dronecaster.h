// Rack RP2350 — Dronecaster adapter (T-D1, 2026-11-19).
//
// Module id 5: hand-ported norns Dronecaster engine (src/dronecaster/, GPL-3.0
// upstream: northern-information/dronecaster + supercollider/supercollider),
// following the LIRA-8 adapter shape. Standalone 48 kHz, host grid 16,
// engine bursts 4x16 -> 64 (the engine renders scsynth-style 64-sample
// blocks internally). Spec:
// docs/superpowers/specs/2026-11-19-dronecaster-port-design.md.
//
// AUDIO CONTRACT: identical to lira8's — Process() serves from an internal
// 64-sample buffer; the 8x16 I2S bank (~2.7 ms at 48 kHz) absorbs the burst.
// Proc budget = the INTERNAL 64-sample period (1333 us at 48 kHz), like
// lira8's 1451.
//
// CC map (spec): 51 drone(0-36 catalog) / 52 amp / 53 fade(atk=rel) /
// 54 hz-offset; notes = re-hz (glide ~60 ms, note-off ignored — drones do
// not stop); PLAY pseudo-CC 64 = recast. Standalone-only: never a chain A
// slot (kChainABufferBytes[5] = 0).
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

#include "dronecaster/dronecaster_engine.h"
#include "dronecaster/drones/drone_registry.h"
#include "dronecaster/dronecaster_tc.h"
#include "dronecaster_engine_o3.h"   // -O3 Process() wrapper (lib/dronecaster_engine)
#include "pico/time.h"               // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"
#include "module_registry.h"         // RackModDestinations (T3 mod routing table)

namespace rack_modules {
namespace mod_dronecaster {

static const char* const kName = "dronecaster";
static const uint32_t kSampleRate = 48000;
static constexpr uint32_t kBlockSize = 16;   // host grid; the engine bursts 64
static const uint32_t kBudgetUs = 1333;      // the 64-sample internal period
static const uint32_t kI2sBuffers = 8;       // 8 x 16 frames ~ 2.7 ms
static const uint32_t kI2sWords = 16;
static const uint32_t kStallUs = 900;        // >2x the 333 us write period

// Voice carve + delay-ring aux (T-D2 crash fix):
//
// The drone states are placement-new'd into the carve; their DELAY RINGS and
// the lazy BLIT tables are additionally placed by AllocRing() — arena first,
// then the aux region. The aux region is the SRAM hot-code window above the
// dronecaster hot image (linker symbols __rack_window_aux_*): idle whenever
// dronecaster is the resident module, because the A-slot overlay members are
// mutually exclusive and dronecaster is standalone-only (main.cpp rejects it
// as a chain A, so the clouds B slot is never populated while it runs).
// Everything in the aux is re-claimed and zeroed at every swap-in.
//
// NO C++ HEAP anywhere in the drone path: the device heap is only ~22 KB,
// the old std::vector delay rings asked 0.3-2.25 MB per drone (a failed
// operator new hung core 1), and the core's malloc wraps in noInterrupts
// only — no multicore lock. The host budget gate in test_dronecaster.cc
// asserts heap bytes == 0 for every drone render and that every drone fits
// this carve + aux with the device-realistic sizes.
//
// KitTables is 73,748 B now (sine wavetable + the inline pan quarter; the
// BLIT tables are lazy). The carve is the whole remaining pool minus the
// module State (the pool is a union — dronecaster is standalone-only, so it
// may use all of it).

// Linker-provided aux region (memmap_rack.ld): the window tail above the
// dronecaster hot image. extern "C" — plain ld symbols, no C++ mangling.
extern "C" uint8_t __rack_window_aux_start[], __rack_window_aux_end[];

static const char* const kCcMap =
    "51 drone (0-36 direct catalog id; 14 live: 0 apparatus / 7 fieldsteel / "
    "10 hecker / 14 mt.lion / 15 mt.zion / 20 sachiko / 22 sine / 23 square / "
    "24 starlids / 25 supersaw / 27 thx / 29 triangle / 30 twin peaks / "
    "36 harms way) 52 amp 53 fade(atk=rel) 54 hz-offset +-24st; "
    "notes = re-hz (glide ~60 ms); PLAY 64 = recast";

static const uint8_t kModContinuousCcs[] = {52, 53, 54};
static const RackModDestinations kModDestinations = {
    "dronecaster: continuous amp 52, fade 53, hz-offset 54 | no triggers; "
    "notes = drone re-hz",
    sizeof(kModContinuousCcs), kModContinuousCcs, 0, nullptr, nullptr,
};

struct State {
  sc::Engine eng;
  // core-0 writes -> core-1 pending slots (block-aligned intents)
  volatile int16_t pending_drone;       // -1 = none
  volatile bool pending_recast;
  // report mirrors (core 1 -> core 0)
  volatile uint32_t proc_avg_us;
  volatile uint32_t nan_blocks;         // blocks zeroed by the NaN guard
  volatile uint32_t render_heartbeat;   // core-1 liveness (stall watchdog)
  // core-0 pitch state (CC 54 is an offset on top of the current note)
  float note_hz;                        // last note-on pitch (default 55)
  float offset_st;                      // CC 54 offset in semitones
  int16_t last_bad_drone;
  explicit State(uint8_t* buf, size_t bytes)
      : pending_drone(22), pending_recast(false), proc_avg_us(0), nan_blocks(0),
        render_heartbeat(0),
        note_hz(55.0f), offset_st(0.0f), last_bad_drone(-1) {
    eng.Init(static_cast<float>(kSampleRate), buf, bytes);
  }
};

static constexpr size_t kVoiceBytes = kRackPoolBytes - sizeof(State);
static_assert(sizeof(sc::KitTables) <= kVoiceBytes,
              "kit tables exceed the dronecaster voice carve");
static_assert(kVoiceBytes <= kRackPoolBytes, "dronecaster carve exceeds pool");
static_assert(sizeof(sc::KitTables) < 80 * 1024,
              "lazy BLIT tables must not sneak back into KitTables");

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

static size_t state_size() { return sizeof(State); }
static constexpr size_t buf_bytes() { return kVoiceBytes; }

void init_at(uint8_t* obj_base, uint8_t* buf_base) {
  memset(buf_base, 0, kVoiceBytes);
  s = new (obj_base) State(buf_base, kVoiceBytes);
  // T-D2: the window tail above our hot image hosts the delay rings / lazy
  // BLIT tables (stale from the previous resident module — zeroed per use).
  s->eng.AttachAux(
      __rack_window_aux_start,
      static_cast<size_t>(__rack_window_aux_end - __rack_window_aux_start));
}
void init() {
  rack_mem_reset();
  init_at(static_cast<uint8_t*>(rack_mem_alloc(state_size(), 8)),
          static_cast<uint8_t*>(rack_mem_alloc(buf_bytes(), 8)));
}

// CC 54 is an OFFSET on top of the current note pitch (spec): base hz is the
// last note (55 until a note arrives), target = base * 2^(offset/12).
static void apply_pitch() {
  s->eng.set_hz_target(s->note_hz * powf(2.0f, s->offset_st / 12.0f));
}
void note_on(uint8_t note, uint8_t vel) {
  (void)vel;
  s->note_hz = sc::midicps(static_cast<float>(note));
  apply_pitch();
}
void note_off(uint8_t note) { (void)note; }   // drones do not stop

void control_change(uint8_t cc, uint8_t val) {
  if (cc == 64) { if (val >= 64) s->pending_recast = true; return; }
  if (cc == 51) {
    // Two-layer selector (the lira8 CC 51 pattern):
    //  - raw bytes 0-36 = DIRECT catalog ids (the panel's steps knob and the
    //    console/serial path both use this layer);
    //  - pot travel 37-127 = LINEAR sweep over the 14 live (ported/audio-
    //    equivalent) ids in catalog order, one pot slice per drone. This is
    //    the controller's choice; Coil (2) is deferred and intentionally
    //    unreachable. A 0-127 C3 pot almost never lands on the raw ids, so
    //    this layer is what the physical pot uses.
    static const int kPorted[] = {0,7,10,14,15,20,22,23,24,25,27,29,30,36};
    int id;
    if (val <= 36) {
      id = val;
    } else {
      id = kPorted[(val - 37) * 14 / 91];   // 37..127 -> 0..13
    }
    if (sc::Catalog(id) != nullptr) {
      s->pending_drone = static_cast<int16_t>(id);
    } else if (static_cast<int16_t>(val) != s->last_bad_drone) {
      s->last_bad_drone = static_cast<int16_t>(val);
      Serial.printf("dc: drone id %u not ported (keep current)\r\n",
                    static_cast<unsigned>(val));
    }
    return;
  }
  if (cc == 52) { s->eng.set_amp(static_cast<float>(val) / 127.0f); return; }
  if (cc == 53) { const float v = static_cast<float>(val) / 127.0f;
                  s->eng.set_fade(16.0f * v * v); return; }             // 0..16 s
  if (cc == 54) { s->offset_st = (static_cast<float>(val) - 64.0f) * (24.0f / 64.0f);
                  apply_pitch(); return; }
}

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  if (cc == 52) { s->eng.set_amp(v); return true; }
  if (cc == 53) { s->eng.set_fade(16.0f * v * v); return true; }
  if (cc == 54) { s->offset_st = (v * 2.0f - 1.0f) * 24.0f; apply_pitch(); return true; }
  return false;
}
bool mod_trigger_cc(uint8_t cc, bool high) { (void)cc; (void)high; return false; }

uint32_t render_heartbeat() { return s ? s->render_heartbeat : 0; }

void render() {
  ++s->render_heartbeat;   // core-1 liveness for the core-0 stall watchdog
  // Pending intents land at the block boundary (the lira8 discipline). The
  // three float params (amp/fade/hz) are single-word atomic stores written
  // by core 0 and read by core 1 — safe by construction, applied next block.
  if (s->pending_drone >= 0) {
    const int id = s->pending_drone;
    s->pending_drone = -1;
    const sc::DroneSpec* spec = sc::Catalog(id);
    if (spec != nullptr) s->eng.SetDrone(spec);  // unported id: ignored
  }
  if (s->pending_recast) { s->pending_recast = false; s->eng.Recast(); }

  uint32_t t0 = time_us_32();
  float l[kBlockSize], r[kBlockSize];
  sc::process_block_o3(s->eng, l, r, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  s->proc_avg_us = (s->proc_avg_us * 15 + dt) / 16;   // ewma for the report

  // Per-block NaN/inf sweep (spec §Error handling): clamp to silence and
  // count; the 2 s report prints the count (no printing from the audio path).
  bool bad = false;
  for (size_t i = 0; i < kBlockSize; ++i) {
    if (!std::isfinite(l[i])) { bad = true; break; }
  }
  if (bad) {
    ++s->nan_blocks;
    for (size_t i = 0; i < kBlockSize; ++i) l[i] = 0.0f;
  }

  float acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) acc += l[i] * l[i];
  const float rms = sqrtf(acc / kBlockSize);
  if (rms > rack_meter_a) rack_meter_a = rms;

  const float gain = g_render_gain;
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out = static_cast<int16_t>(l[i] * gain * 32768.0f);
    i2s.write16(out, out);
  }
  if (time_us_32() - w0 > kStallUs) ++i2s_stall_count;
}

// 2 s report extra field (integer-only — no %f under newlib-nano).
void report_dc(char* buf, size_t n) {
  snprintf(buf, n, "%s avg=%uu sw=%lu nan=%lu initfail=%lu",
           s->eng.current_name(),
           static_cast<unsigned>(s->proc_avg_us),
           static_cast<unsigned long>(s->eng.swap_count()),
           static_cast<unsigned long>(s->nan_blocks),
           static_cast<unsigned long>(s->eng.init_fails()));
}

}  // namespace mod_dronecaster
}  // namespace rack_modules
