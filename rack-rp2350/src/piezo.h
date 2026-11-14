// Rack RP2350 — T2 piezo input (GP26 = ADC0): raw ADC bring-up.
// Plan: docs/superpowers/plans/2026-11-14-rack-button-piezo.md
//
// Hardware (mandatory protection — a hard tap can spike tens of volts):
//   piezo ∥ 1 MΩ bleed → 10 kΩ series → GP26; 1N4148 clamps to 3V3 and GND
//   at the ADC node (the 10 kΩ limits clamp current to ~2-3 mA).
//
// ADC free-run ~24 kHz (clk_adc 48 MHz / 2000) via the FIFO + ONE DMA
// channel in write-ring mode. BOOT BUG (fixed, was live on hardware): the
// ring flag is `channel_config_set_ring(c, bool WRITE, size_bits)` — passing
// false put the ring on the READ address while write_increment kept
// climbing -> the DMA wrote ever-increasing RAM at 24 kHz, corrupting the
// SRAM hot-code window mid-audio (tone -> glitch noise -> crash). WRITE it
// is; g_ring is 1024-B aligned to match the (1<<10) wrap. Core 0 drains the
// ring tail in loop() by chasing the live DMA write address. T3 adds the
// software-Ears pipeline (rectifier → envelope follower → Schmitt gate) on
// top of this ring. All state here is core-0 only.
//
// Digital gain (`piezo gain`, 1..16) stands in for Ears' omitted 40 dB amp:
// it scales the rectified samples BEFORE the follower in T3; sensitivity
// sits post-gain.
#pragma once
#include <stdint.h>

#include "hardware/adc.h"
#include "hardware/dma.h"

namespace rack_piezo {

static const uint32_t kPinPiezo = 26;        // ADC0
static const uint32_t kSampleRate = 24000;   // Hz (clk_adc 48 MHz / 2000)
static const uint16_t kRingSamples = 512;    // ~21 ms at 24 kHz
static const uint32_t kRingBytes = kRingSamples * sizeof(uint16_t);  // 1024

static int16_t g_dma_ch = -1;
static __attribute__((aligned(1024))) uint16_t g_ring[kRingSamples];

// Readout state (max-since-report semantics, like the rack meters).
static uint16_t g_min = 4095;
static uint16_t g_max = 0;
static uint16_t g_peak = 0;                  // max since last report
static uint16_t g_rd_idx = 0;                // last drained ring position
static uint32_t g_new_since_report = 0;      // for the measured sample rate

static float g_gain = 1.0f;                  // digital gain (T3 pre-rectifier)

inline void init() {
  adc_init();
  adc_gpio_init(kPinPiezo);
  adc_select_input(0);                       // GP26 = ADC0
  // Full sample period = div cycles (must stay >= the 96 conversion cycles).
  adc_set_clkdiv(48000000u / kSampleRate - 1u);
  adc_fifo_setup(true,    // enable FIFO
                 true,    // DREQ while >= threshold
                 1,       // threshold: 1 sample
                 false,   // no error flag in final entry
                 true);   // shift: DMA words = clean 12-bit results
  adc_fifo_drain();
  adc_run(true);                             // start the free-running conversion

  g_dma_ch = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(g_dma_ch);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_dreq(&c, DREQ_ADC);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  // Ring on the WRITE address (true!), 10 bits = 1024 B, + an effectively
  // infinite transfer count: the DMA runs forever, one sample per DREQ,
  // wrapping inside the 1024-B-aligned g_ring — no re-arm, no IRQ.
  channel_config_set_ring(&c, true, 10);
  dma_channel_configure(g_dma_ch, &c,
                        g_ring,                       // write (rings)
                        &adc_hw->fifo,                // read: ADC FIFO
                        0x0FFFFFFFu,                  // never exhausts
                        true);                        // start now
  g_rd_idx = 0;
  g_new_since_report = 0;
}

// Drain whatever the DMA has written since the last poll (chase the live
// write address around the ring), tracking min/max/peak. The T3 pipeline
// replaces this tracking with the Ears stages.
inline void poll() {
  if (g_dma_ch < 0) return;
  const uintptr_t base = (uintptr_t)g_ring;
  const uint16_t wr_idx = (uint16_t)(
      ((((uintptr_t)dma_hw->ch[g_dma_ch].write_addr) - base) & (kRingBytes - 1)) >> 1);
  uint16_t d = (uint16_t)((wr_idx - g_rd_idx + kRingSamples) % kRingSamples);
  if (d == 0) return;                        // nothing new since last poll
  for (uint16_t i = 0; i < d; ++i) {
    const uint16_t s = g_ring[g_rd_idx];
    g_rd_idx = (uint16_t)((g_rd_idx + 1) % kRingSamples);
    if (s < g_min) g_min = s;
    if (s > g_max) g_max = s;
    if (s > g_peak) g_peak = s;
  }
  g_new_since_report += d;
}

// 2 s report hooks (reset-on-read, like the rack meters).
inline uint32_t report_sample_rate() {
  const uint32_t n = g_new_since_report / 2u;  // samples per second
  g_new_since_report = 0;
  return n;
}
inline uint16_t report_peak() { const uint16_t v = g_peak; g_peak = 0; return v; }
inline uint16_t report_min() { const uint16_t v = g_min; g_min = 4095; return v; }
inline uint16_t report_max() { const uint16_t v = g_max; g_max = 0; return v; }
inline float gain() { return g_gain; }
inline void set_gain(float g) { g_gain = (g < 1.0f) ? 1.0f : (g > 16.0f ? 16.0f : g); }

}  // namespace rack_piezo
