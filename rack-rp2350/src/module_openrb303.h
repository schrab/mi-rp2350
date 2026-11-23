// Rack module id 7 — OpenRB-338 TB-303 + distortion + PCF + compressor.
// The DSP is the port in src/openrb303/ (headers + .bin tables linked in
// flash). Standalone-only: no chain A slot, no SRAM hot window (the chain
// runs from XIP at ~1.4k cycles/sample with DCP-backed doubles).
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>

#include "pico/time.h"
#include "stmlib/dsp/dsp.h"
#include "rack_i2s.h"
#include "module_benjolin.h"   // CcSpec (shared spec type)
#include "module_registry.h"
#include "orb303.h"
#include "pcfcomp.h"

namespace rack_modules {
namespace mod_openrb303 {

static const char* const kName = "openrb303";
static const uint32_t kSampleRate = 48000;
static constexpr uint32_t kBlockSize = 64;   // the engine's native block
static const uint32_t kBudgetUs = 1500;      // 64-sample host block = 1.33 ms pacing; DSP ~150 us
static const uint32_t kI2sBuffers = 8;
static const uint32_t kI2sWords = 64;
static const uint32_t kStallUs = 2500;   // 64-frame block = 1.33 ms pacing

// CC 90-105: 303 (90-95), distortion (96-97), PCF (98-103), comp (104-105).
static const mod_benjolin::CcSpec kCcSpecs[16] = {
    /* 90 cutoff   */ {0.0f, 127.0f, 0.0f, false, 20.0f},
    /* 91 reso     */ {0.0f, 127.0f, 0.0f, false, 90.0f},
    /* 92 envmod   */ {0.0f, 127.0f, 0.0f, false, 20.0f},
    /* 93 decay    */ {0.0f, 127.0f, 0.0f, false, 100.0f},
    /* 94 accent   */ {0.0f, 127.0f, 0.0f, false, 64.0f},
    /* 95 wave     */ {0.0f, 1.0f, 1.0f, false, 0.0f},
    /* 96 dist amt */ {0.0f, 127.0f, 0.0f, false, 70.0f},
    /* 97 dist shp */ {0.0f, 127.0f, 0.0f, false, 60.0f},
    /* 98 pcf freq */ {0.0f, 127.0f, 0.0f, false, 70.0f},
    /* 99 pcf reso */ {0.0f, 127.0f, 0.0f, false, 64.0f},
    /*100 pcf amt  */ {0.0f, 127.0f, 0.0f, false, 40.0f},
    /*101 pcf wave */ {0.0f, 127.0f, 0.0f, false, 24.0f},
    /*102 pcf dec  */ {0.0f, 127.0f, 0.0f, false, 90.0f},
    /*103 pcf mode */ {0.0f, 1.0f, 1.0f, false, 1.0f},
    /*104 comp amt */ {0.0f, 127.0f, 0.0f, false, 70.0f},
    /*105 comp thr */ {0.0f, 127.0f, 0.0f, false, 64.0f},
};

static const uint8_t kCcSeeds[16] = {
    20, 90, 20, 100, 64, 0, 70, 60, 70, 64, 40, 24, 90, 127, 70, 64};

static const char* const kCcMap =
    "90 cut 91 reso 92 envmod 93 dec 94 accent 95 wave 96 dist-amt 97 "
    "dist-shape 98 pcf-freq 99 pcf-reso 100 pcf-amt 101 pcf-wave 102 pcf-dec "
    "103 pcf-mode 104 comp-amt 105 comp-thr";

static const uint8_t kModContinuousCcs[] = {
    90, 91, 92, 93, 94, 96, 97, 98, 99, 100, 101, 102, 104, 105};
static const RackModDestinations kModDestinations = {
    "openrb303: continuous CC 90-105 (303/dist/PCF/comp) | no triggers",
    sizeof(kModContinuousCcs), kModContinuousCcs, 0, nullptr, nullptr};

static volatile uint32_t g_render_heartbeat = 0;
static volatile uint32_t g_proc_us_max = 0;
static volatile uint32_t g_proc_avg_us = 0;
static uint32_t g_tick = 0;

void init_at(uint8_t* obj_base) { (void)obj_base; }
void init() {
  orb_init();
  pcfcomp_init();
  g_tick = 0;
  g_render_heartbeat = 0;
  g_proc_us_max = 0;
  g_proc_avg_us = 0;
}
static size_t state_size() { return 0; }

void note_on(uint8_t note, uint8_t vel) {
  orb_note_on((int)note - 36, vel >= 100, false);
}
void note_off(uint8_t note) { (void)note; orb_note_off(); }

static void set_cc_raw(uint8_t cc, uint8_t value) {
  if (cc < 90 || cc > 105) return;
  const int idx = (int)(cc - 90);
  // The two switches are bool127 on the wire (<64 / >=64).
  if (idx == 5 || idx == 13) value = value >= 64 ? 1 : 0;
  if (idx < 6) {
    orb_set_303_param(idx, value);
  } else if (idx == 6 || idx == 7) {
    orb_set_dist_param(idx - 6, value);
  } else if (idx < 14) {
    pcfcomp_set_param(0, idx - 8, value);
  } else {
    pcfcomp_set_param(1, idx - 14, value);
  }
}

void control_change(uint8_t cc, uint8_t val) { set_cc_raw(cc, val); }

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  set_cc_raw(cc, (uint8_t)(v * 127.0f + 0.5f));
  return true;
}
bool mod_trigger_cc(uint8_t cc, bool high) { (void)cc; (void)high; return false; }

uint32_t render_heartbeat() { return g_render_heartbeat; }

void render() {
  ++g_render_heartbeat;
  const uint32_t t0 = time_us_32();
  OrbBlock bus;
  orb_tick_step((int)g_tick++);
  orb_process(bus);                        // 303 vintage -> distortion
  pcfcomp_process(bus, (int)g_tick);       // PCF -> compressor
  const uint32_t dt = time_us_32() - t0;   // DSP only — write16 can block
  if (dt > g_proc_us_max) g_proc_us_max = dt;
  g_proc_avg_us = (g_proc_avg_us * 15 + dt) / 16;

  float acc = 0.0f;
  const uint32_t w0 = time_us_32();
  for (uint32_t i = 0; i < kBlockSize; ++i) {
    const float s = bus[i];
    if (!std::isfinite(s)) { i2s.write16(0, 0); continue; }
    acc += s * s;
    const int16_t v = stmlib::Clip16(static_cast<int32_t>(s * 32768.0f));
    i2s.write16(v, v);
  }
  if (time_us_32() - w0 > kStallUs) ++i2s_stall_count;
  if (acc > rack_meter_a * rack_meter_a * (float)kBlockSize) {
    rack_meter_a = sqrtf(acc / (float)kBlockSize);
  }
}

void report_openrb303(char* buf, size_t n) {
  snprintf(buf, n, "avg=%uu max=%uu hb=%lu",
           (unsigned)g_proc_avg_us, (unsigned)g_proc_us_max,
           (unsigned long)g_render_heartbeat);
}

}  // namespace mod_openrb303
}  // namespace rack_modules
