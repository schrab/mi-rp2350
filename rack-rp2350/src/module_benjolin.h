// Rack RP2350 — Benjolin adapter (2026-11-22).
//
// Module id 6: reference-exact port of the \benjolis SynthDef (scazan/benjolis,
// Alejandro Olarte / Scott Cazan) on the PicoCollider kit, following the
// dronecaster adapter shape. Standalone 48 kHz, host grid 16, engine bursts
// 4x16 -> 64 (the engine renders scsynth-style 64-sample blocks internally).
// Spec: docs/ASSESSMENT-benjolin-port-2026-11-22.md + the SDD ledger.
//
// AUDIO CONTRACT: identical to dronecaster's — Process16 serves from an
// internal 64-sample buffer; the 8x16 I2S bank (~2.7 ms at 48 kHz) absorbs
// the burst. Proc budget = the INTERNAL 64-sample period (1333 us at 48 kHz).
//
// CC map (all 14 continuous): 60 freq1 61 freq2 62 scale 63 rungler1
// 64 rungler2 65 runglerFilt 66 loop 67 filtFreq 68 q 69 gain 70 filterType
// 71 outSignal 72 amp 73 pan. Standalone-only: never a chain A slot
// (kChainABufferBytes[6] = 0).
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <new>

#include "benjolin/benjolin_engine.h"
#include "pico/time.h"               // time_us_32 (static inline in pico-sdk 2.x)
#include "stmlib/dsp/dsp.h"          // stmlib::Clip16

#include "rack_arena.h"
#include "module_registry.h"         // RackModDestinations (T3 mod routing table)

// The SRAM law (PORTING-HIGHLIGHTS §3): this TU compiles at the core's
// effective -Os (the earlephilhower core appends -Os after the project
// flags), which made GCC emit the kit UGen helpers out-of-line into FLASH
// and call them through veneers ~8x/sample. O3 from this point makes the
// kit calls inline into the SRAM-resident Process64. pop_options at the
// bottom of the file restores the TU's original optimization level.
#pragma GCC push_options
#pragma GCC optimize("O3")

// Hot-code section placement (SRAM law): the wrapper carries the token and
// the engine inlines into it at -O3 (the dronecaster pattern). One unique
// token per emitted function.
#if defined(RACK_RP2350_TC)
#define BENJOLIN_TC(token) __attribute__((section(".tc_benjolin." #token), noinline))
#else
#define BENJOLIN_TC(token)
#endif

namespace rack_modules {
namespace mod_benjolin {

static const char* const kName = "benjolin";
static const uint32_t kSampleRate = 48000;
static constexpr uint32_t kBlockSize = 16;   // host grid; the engine bursts 64
static const uint32_t kBudgetUs = 1333;      // the 64-sample internal period
static const uint32_t kI2sBuffers = 8;       // 8 x 16 frames ~ 2.7 ms
static const uint32_t kI2sWords = 16;
static const uint32_t kStallUs = 900;        // >2x the 333 us write period

static const char* const kCcMap =
    "60 freq1(log 1-500) 61 freq2(log .1-50) 62 scale 63 rungler1 64 rungler2 "
    "65 runglerFilt 66 loop 67 filtFreq(log 10-8k) 68 q 69 gain 70 filterType "
    "(0-3) 71 outSignal (0-6) 72 amp 73 pan; standalone-only";

static const uint8_t kModContinuousCcs[] = {60, 61, 62, 63, 64, 65, 66, 67,
                                            68, 69, 70, 71, 72, 73};
static const RackModDestinations kModDestinations = {
    "benjolin: continuous CC 60-73 (freqs/scale/rungler/loop/filter/amp/pan) "
    "| no triggers",
    sizeof(kModContinuousCcs), kModContinuousCcs, 0, nullptr, nullptr,
};

struct State {
  benjolin::Engine eng;
  // The 8 register-tap rings (8 x 16 floats = 512 B) and the 2 pan-delay
  // rings (2 x 1024 floats = 8 KB) live in the carve — the engine's device
  // path allocates NOTHING (the dronecaster no-heap contract).
  float tap_rings[8 * 16];
  float pan_rings[2 * 1024];
  // core-0 CC values (single-word atomic stores; core 1 snapshots them at
  // the block boundary — the lira8/dronecaster discipline, one float per
  // CC write so no torn multi-word state).
  volatile float cc_freq1, cc_freq2, cc_scale, cc_rungler1, cc_rungler2;
  volatile float cc_rungler_f, cc_loop, cc_filt_freq, cc_q, cc_gain;
  volatile float cc_filter_type, cc_out_signal, cc_amp, cc_pan;
  // report mirrors (core 1 -> core 0)
  volatile uint32_t proc_avg_us;
  volatile uint32_t nan_blocks;         // blocks zeroed by the NaN guard
  volatile uint32_t render_heartbeat;   // core-1 liveness (stall watchdog)
  // CC seeds = the SynthDef defaults expressed in RAW CC (0-127) so the
  // boot sound is the golden scenario. Mapping back through cc_to_params:
  // 40 Hz -> 127*ln40/ln500 = 75; 4 Hz -> 75; scale 1 -> 127; rungler1
  // 0.16 -> 127*0.16/0.5 = 41; rungler_f 9 -> 127*9/24 = 48; filtFreq
  // 40 Hz -> 127*ln4/ln800 = 26; q 0.82 -> 127*0.82/0.99 = 105; gain
  // 1 -> 127*1/4 = 32; amp 0.5 -> 64.
  explicit State() : eng(), tap_rings{}, cc_freq1(75.0f), cc_freq2(75.0f),
      cc_scale(127.0f), cc_rungler1(41.0f), cc_rungler2(0.0f),
      cc_rungler_f(48.0f), cc_loop(0.0f), cc_filt_freq(26.0f), cc_q(105.0f),
      cc_gain(32.0f), cc_filter_type(0.0f), cc_out_signal(6.0f),
      cc_amp(64.0f), cc_pan(0.0f), proc_avg_us(0), nan_blocks(0),
      render_heartbeat(0) {
    benjolin::Params p;   // SynthDef defaults with an audible amp
    p.amp = 0.5f;
    eng.Init(static_cast<double>(kSampleRate),
             benjolin::FeedbackMode::kReferenceSc,
             1795055191,               // the golden DFM1 epoch (deterministic)
             p, &tap_rings[0], &pan_rings[0]);
  }
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

static size_t state_size() { return sizeof(State); }

void init_at(uint8_t* obj_base) {
  s = new (obj_base) State();
}
void init() {
  rack_mem_reset();
  init_at(static_cast<uint8_t*>(rack_mem_alloc(state_size(), 8)));
}

// CC (raw 0-127) -> SC-unit scalings (the plan's CC table; log maps for the
// wide ranges). The CC vars stay in the CC domain end to end — the engine
// never sees raw CC values (T5 review C1: raw CCs reached Params, and
// scale >= 13 overflowed midicps to inf -> permanent NaN).
static void cc_to_params(benjolin::Params* p) {
  p->freq1 = powf(500.0f, s->cc_freq1 / 127.0f);                       // 1..500
  p->freq2 = 0.1f * powf(500.0f, s->cc_freq2 / 127.0f);                // .1..50
  p->scale = s->cc_scale * (1.0f / 127.0f);                            // 0..1
  p->rungler1 = s->cc_rungler1 * (0.5f / 127.0f);                      // 0..0.5
  p->rungler2 = s->cc_rungler2 * (0.5f / 127.0f);                      // 0..0.5
  p->rungler_f = s->cc_rungler_f * (24.0f / 127.0f);                   // 0..24
  p->loop = s->cc_loop * (1.0f / 127.0f);                              // 0..1
  p->filt_freq = 10.0f * powf(800.0f, s->cc_filt_freq / 127.0f);       // 10..8k
  p->q = s->cc_q * (0.99f / 127.0f);                                   // 0..0.99
  p->gain = s->cc_gain * (4.0f / 127.0f);                              // 0..4
  p->filter_type = s->cc_filter_type;
  p->out_signal = s->cc_out_signal;
  p->amp = s->cc_amp * (1.0f / 127.0f);                                // 0..1
  p->pan = s->cc_pan * (0.127f / 127.0f);  // the norns x0.001 law      // 0..0.127
}

// CC 60-73 -> 0..127 raw; helper for the direct control_change path.
static void set_cc_raw(uint8_t cc, float raw) {
  switch (cc) {
    case 60: s->cc_freq1 = raw; break;
    case 61: s->cc_freq2 = raw; break;
    case 62: s->cc_scale = raw; break;
    case 63: s->cc_rungler1 = raw; break;
    case 64: s->cc_rungler2 = raw; break;
    case 65: s->cc_rungler_f = raw; break;
    case 66: s->cc_loop = raw; break;
    case 67: s->cc_filt_freq = raw; break;
    case 68: s->cc_q = raw; break;
    case 69: s->cc_gain = raw; break;
    case 70: s->cc_filter_type = raw; break;
    case 71: s->cc_out_signal = raw; break;
    case 72: s->cc_amp = raw; break;
    case 73: s->cc_pan = raw; break;
    default: break;
  }
}

void note_on(uint8_t note, uint8_t vel) { (void)note; (void)vel; }
void note_off(uint8_t note) { (void)note; }   // the benjolin has no notes

void control_change(uint8_t cc, uint8_t val) {
  set_cc_raw(cc, static_cast<float>(val));
}

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  set_cc_raw(cc, v * 127.0f);
  return true;   // all 14 CCs accept continuous modulation
}
bool mod_trigger_cc(uint8_t cc, bool high) { (void)cc; (void)high; return false; }

uint32_t render_heartbeat() { return s ? s->render_heartbeat : 0; }

// The -O3 hot entry (the engine inlines into it; the SRAM token lives here).
BENJOLIN_TC(process)
static void process16_tc(benjolin::Engine& e, float* l, float* r) {
  e.Process16(l, r);
}

void render() {
  ++s->render_heartbeat;   // core-1 liveness for the core-0 stall watchdog
  // CC values are single-word atomic stores (core 0) — snapshot at the block
  // boundary and apply (the lira8/dronecaster pending-intent discipline).
  benjolin::Params p;
  cc_to_params(&p);
  s->eng.SetParams(p);

  uint32_t t0 = time_us_32();
  float l[kBlockSize], r[kBlockSize];
  process16_tc(s->eng, l, r);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  s->proc_avg_us = (s->proc_avg_us * 15 + dt) / 16;   // ewma for the report

  // Per-block NaN/inf sweep (the dronecaster discipline): clamp to silence
  // and count; the 2 s report prints the count.
  bool bad = false;
  for (size_t i = 0; i < kBlockSize; ++i) {
    if (!std::isfinite(l[i]) || !std::isfinite(r[i])) { bad = true; break; }
  }
  if (bad) {
    ++s->nan_blocks;
    for (size_t i = 0; i < kBlockSize; ++i) { l[i] = 0.0f; r[i] = 0.0f; }
  }

  float acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) acc += l[i] * l[i];
  const float rms = sqrtf(acc / kBlockSize);
  if (rms > rack_meter_a) rack_meter_a = rms;

  const float gain = g_render_gain;
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out = stmlib::Clip16(
        static_cast<int32_t>(l[i] * 32768.0f * gain));
    i2s.write16(out, out);
  }
  if (time_us_32() - w0 > kStallUs) ++i2s_stall_count;
}

// 2 s report extra field (integer-only — no %f under newlib-nano).
void report_benjolin(char* buf, size_t n) {
  snprintf(buf, n, "avg=%uu nan=%lu hb=%lu",
           static_cast<unsigned>(s->proc_avg_us),
           static_cast<unsigned long>(s->nan_blocks),
           static_cast<unsigned long>(s->render_heartbeat));
}

}  // namespace mod_benjolin
}  // namespace rack_modules

#pragma GCC pop_options
