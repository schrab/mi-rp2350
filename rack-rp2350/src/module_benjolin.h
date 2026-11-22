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
// CC map (13 continuous): 60 freq1 61 freq2 62 scale 63 rungler1
// 64 rungler2 65 runglerFilt 66 loop 67 filtFreq 68 q 69 gain 70 filterType
// 71 outSignal 72 amp. PAN (CC 73, the SynthDef's DelayL widening) is not
// UI-exposed — a delay-based pan on a sustained drone only comb-filters —
// but the firmware still accepts it and boots at the norns center (64 ->
// 0.064 s: left delayed, right dry). Standalone-only: never a chain A slot
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

// The parameter specs — the EXACT norns benjolis.lua ControlSpecs (the
// ranges, the curves, the steps, the defaults; T6 user demand: "each
// parameter should have the EXACT range as on benjolis and exact curve").
// The CC map = the raw 0-127 over each spec's normalized travel.
struct CcSpec {
  float min, max, step;
  bool exp;          // the norns "exp" warp: v = min*(max/min)^n
  float def;         // the norns default
};
static const CcSpec kCcSpecs[14] = {
    /* 60 freq1       */ {20.0f, 14000.0f, 0.0f, true, 70.0f},
    /* 61 freq2       */ {0.1f, 14000.0f, 0.0f, true, 4.0f},
    /* 62 scale       */ {0.0f, 1.0f, 0.0f, false, 1.0f},
    /* 63 rungler1    */ {0.001f, 1.0f, 0.0f, false, 0.16f},
    /* 64 rungler2    */ {0.001f, 1.0f, 0.0f, false, 0.001f},
    /* 65 runglerFilt */ {0.01f, 9.0f, 0.0f, false, 1.0f},
    /* 66 loop        */ {0.0f, 1.0f, 0.0f, false, 1.0f},
    /* 67 filtFreq    */ {20.0f, 20000.0f, 0.0f, true, 40.0f},
    /* 68 q           */ {0.001f, 1.0f, 0.0f, false, 0.02f},
    /* 69 gain        */ {0.0f, 6.0f, 1.0f, false, 6.0f},
    /* 70 filterType  */ {0.0f, 1.0f, 1.0f, false, 0.0f},
    /* 71 outSignal   */ {0.0f, 6.0f, 1.0f, false, 6.0f},
    /* 72 amp         */ {0.0f, 1.0f, 0.0f, false, 0.0f},
    /* 73 pan         */ {-1.0f, 1.0f, 0.001f, false, 0.0f},
};

// The CC seeds = each norns DEFAULT mapped back to its raw CC (so the boot
// state is exactly the norns benjolis default state: amp 0 — the SILENT
// boot, like norns; raise AMP to hear it).
static const uint8_t kCcSeeds[14] = {24, 40, 127, 20, 0, 14, 127,
                                     13, 2, 127, 0, 127, 0, 64};

static const char* const kCcMap =
    "60 freq1 exp 20-14k 61 freq2 exp .1-14k 62 scale 63 rungler1 "
    "64 rungler2 65 runglerFilt .01-9 66 loop 67 filtFreq exp 20-20k "
    "68 q .001-1 69 gain 0-6 step1 70 filterType 0-1 71 outSignal 0-6 "
    "72 amp 73 pan -1..1; standalone-only";

static const uint8_t kModContinuousCcs[] = {60, 61, 62, 63, 64, 65, 66, 67,
                                            68, 69, 70, 71, 72};
static const RackModDestinations kModDestinations = {
    "benjolin: continuous CC 60-73 (freqs/scale/rungler/loop/filter/amp/pan) "
    "| no triggers",
    sizeof(kModContinuousCcs), kModContinuousCcs, 0, nullptr, nullptr,
};

// CC (raw 0-127) -> the parameter value through the EXACT norns spec: the
// "exp" warp = min*(max/min)^n, "lin" = min + (max-min)*n, then the step
// quantization in value units (the norns ControlSpec semantics).
static float SpecValue(const CcSpec& sp, float cc) {
  const float n = cc * (1.0f / 127.0f);
  float v = sp.exp ? sp.min * powf(sp.max / sp.min, n)
                   : sp.min + (sp.max - sp.min) * n;
  if (sp.step > 0.0f) {
    v = sp.min + roundf((v - sp.min) / sp.step) * sp.step;
    v = v < sp.min ? sp.min : (v > sp.max ? sp.max : v);
  }
  return v;
}

// The CC list order = the kCcSpecs order. The CC vars stay in the CC domain
// end to end — the engine never sees raw CC values (T5 review C1).
static void cc_to_params(const volatile float* cc, benjolin::Params* p) {
  p->freq1 = SpecValue(kCcSpecs[0], cc[0]);
  p->freq2 = SpecValue(kCcSpecs[1], cc[1]);
  p->scale = SpecValue(kCcSpecs[2], cc[2]);
  p->rungler1 = SpecValue(kCcSpecs[3], cc[3]);
  p->rungler2 = SpecValue(kCcSpecs[4], cc[4]);
  p->rungler_f = SpecValue(kCcSpecs[5], cc[5]);
  p->loop = SpecValue(kCcSpecs[6], cc[6]);
  p->filt_freq = SpecValue(kCcSpecs[7], cc[7]);
  p->q = SpecValue(kCcSpecs[8], cc[8]);
  p->gain = SpecValue(kCcSpecs[9], cc[9]);
  p->filter_type = SpecValue(kCcSpecs[10], cc[10]);
  p->out_signal = SpecValue(kCcSpecs[11], cc[11]);
  p->amp = SpecValue(kCcSpecs[12], cc[12]);
  p->pan = SpecValue(kCcSpecs[13], cc[13]);
}

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
  volatile float cc_val[14];
  // report mirrors (core 1 -> core 0)
  volatile uint32_t proc_avg_us;
  volatile uint32_t nan_blocks;         // blocks zeroed by the NaN guard
  volatile uint32_t render_heartbeat;   // core-1 liveness (stall watchdog)
  // The CC seeds = kCcSeeds (each norns default mapped back to its raw CC) —
  // the boot state is EXACTLY the norns benjolis default state: amp 0, so
  // the boot is SILENT until AMP is raised (like norns). The engine's Init
  // params = the same values (the CC map applied once up front).
  explicit State() : eng(), tap_rings{}, pan_rings{},
      proc_avg_us(0), nan_blocks(0), render_heartbeat(0) {
    for (int i = 0; i < 14; ++i) cc_val[i] = kCcSeeds[i];
    benjolin::Params p;
    cc_to_params(cc_val, &p);   // the norns default state through the same map
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

// CC 60-73 -> the kCcSpecs index (cc - 60); helper for the direct paths.
static void set_cc_raw(uint8_t cc, float raw) {
  if (cc >= 60 && cc <= 73) s->cc_val[cc - 60] = raw;
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
  return true;   // all CCs accept continuous modulation
}
bool mod_trigger_cc(uint8_t cc, bool high) { (void)cc; (void)high; return false; }

uint32_t render_heartbeat() { return s ? s->render_heartbeat : 0; }

// --- device-output capture (the device-vs-host comparison) ---
// cmd_bjdump (core 0) carves a buffer and requests N blocks; render (core 1)
// appends the post-Clip16 int16 pairs; core 0 streams the buffer when the
// capture completes. Diagnostic only — allocation is transient (freed at the
// next swap by rack_mem_reset).
static int16_t* s_cap_data = nullptr;
static volatile uint32_t s_cap_req = 0;    // blocks requested (0 = idle)
static volatile uint32_t s_cap_have = 0;   // blocks captured

static bool dump_begin(uint32_t blocks) {
  if (blocks == 0 || blocks > 750) return false;
  s_cap_data = static_cast<int16_t*>(
      rack_mem_alloc(blocks * kBlockSize * 2 * sizeof(int16_t), 4));
  if (s_cap_data == nullptr) return false;
  s_cap_have = 0;
  s_cap_req = blocks;      // the release signal — set LAST
  return true;
}
static bool dump_done() { return s_cap_req != 0 && s_cap_have >= s_cap_req; }
static const int16_t* dump_data() { return s_cap_data; }
static uint32_t dump_bytes() { return s_cap_req * kBlockSize * 2 * sizeof(int16_t); }

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
  cc_to_params(s->cc_val, &p);
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
  // TRUE STEREO + HEADROOM: the engine's output = the tanh-saturated square
  // (±1) through LeakDC, whose edge response overshoots to ±2 — at the old
  // ×32768 scale EVERYTHING above ±1.0 flat-topped in int16 (the user's
  // "bit reduction" distortion; the float goldens held ±1.72 fine). Scale
  // by 16384 = ±2.0 headroom; the AMP knob sets the level (like norns).
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out_l = stmlib::Clip16(
        static_cast<int32_t>(l[i] * 16384.0f * gain));
    const int16_t out_r = stmlib::Clip16(
        static_cast<int32_t>(r[i] * 16384.0f * gain));
    // The device-output capture (the device-vs-host comparison): the EXACT
    // int16 pairs written to I2S, appended while a dump is in flight.
    if (s_cap_req != 0 && s_cap_have < s_cap_req) {
      const size_t idx = (s_cap_have * kBlockSize + i) * 2;
      s_cap_data[idx] = out_l;
      s_cap_data[idx + 1] = out_r;
      if (i == kBlockSize - 1) ++s_cap_have;
    }
    i2s.write16(out_l, out_r);
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
