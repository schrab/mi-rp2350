// Rack RP2350 — T2 piezo input (GP26 = ADC0): raw ADC bring-up.
// Plan: docs/superpowers/plans/2026-11-14-rack-button-piezo.md
//
// Hardware (mandatory protection — a hard tap can spike tens of volts):
//   piezo ∥ 1 MΩ bleed → 10 kΩ series → GP26; 1N4148 clamps to 3V3 and GND
//   at the ADC node (the 10 kΩ limits clamp current to ~2-3 mA).
//
// T2 SAFE MODE: the ADC free-runs at ~24 kHz into its 4-deep FIFO (adc_run),
// and loop() drains whatever accumulated each pass. Between-pass overflows
// lose samples (the FIFO is tiny) — fine for the bench read-out: a tap is
// tens of ms wide, min/max/peak survive drained-rate gaps. The measured
// rate field (pzr=) reports the DRAINED rate, i.e. the loop() pacing.
// The 24 kHz DMA-ring version for T3 is pending: the first attempt hung the
// board at boot (ring config under suspicion) — bisect separately before
// building the Ears pipeline on it.
//
// Digital gain (`piezo gain`, 1..16) stands in for Ears' omitted 40 dB amp:
// it scales the rectified samples BEFORE the follower in T3; sensitivity
// sits post-gain. All state here is core-0 only.
#pragma once
#include <stdint.h>

#include "hardware/adc.h"

namespace rack_piezo {

static const uint32_t kPinPiezo = 26;        // ADC0
static const uint32_t kSampleRate = 24000;   // ADC free-run rate (clk_adc/2000)

// Readout state (max-since-report semantics, like the rack meters).
static uint16_t g_min = 4095;
static uint16_t g_max = 0;
static uint16_t g_peak = 0;                  // max since last report
static uint32_t g_drained = 0;               // samples drained since report

static float g_gain = 1.0f;                  // digital gain (T3 pre-rectifier)

inline void init() {
  adc_init();
  adc_gpio_init(kPinPiezo);
  adc_select_input(0);                       // GP26 = ADC0
  adc_set_clkdiv(48000000u / kSampleRate - 1u);
  adc_fifo_setup(true,    // enable FIFO
                 false,   // no DREQ (no DMA in safe mode)
                 0,       // drain regardless of level
                 false,   // no error flag in final entry
                 false);  // keep the raw FIFO word (mask result in poll)
  adc_fifo_drain();
  adc_run(true);                             // start the free-running conversion
}

// Drain whatever accumulated since the last loop() pass. The FIFO is 4
// deep; between-pass overflows drop samples (reported in the rate field).
inline void poll() {
  while (adc_fifo_get_level() > 0) {
    const uint16_t raw = (uint16_t)(adc_fifo_get() & 0xFFFu);  // 12-bit result
    if (raw < g_min) g_min = raw;
    if (raw > g_max) g_max = raw;
    if (raw > g_peak) g_peak = raw;
    ++g_drained;
  }
}

// 2 s report hooks (reset-on-read, like the rack meters).
inline uint32_t report_sample_rate() {
  const uint32_t n = g_drained / 2u;         // drained samples per second
  g_drained = 0;
  return n;
}
inline uint16_t report_peak() { const uint16_t v = g_peak; g_peak = 0; return v; }
inline uint16_t report_min() { const uint16_t v = g_min; g_min = 4095; return v; }
inline uint16_t report_max() { const uint16_t v = g_max; g_max = 0; return v; }
inline float gain() { return g_gain; }
inline void set_gain(float g) { g_gain = (g < 1.0f) ? 1.0f : (g > 16.0f ? 16.0f : g); }

}  // namespace rack_piezo
