// Status LED: one WS2812 on GP22 standing in for the three LEDs of the
// original Elements module (gate / exciter / resonator).
//
// Mapping: R = exciter, G = resonator, B = gate. Values are thresholded so
// the LED is unmistakable during play and off when idle.
//
// The WS2812 is driven via PIO2 (PIO0 = USB host, PIO1 = I2S) using the SDK
// ws2812 PIO program. Critically, pio_sm_put is called ONLY when the TX FIFO
// has room — core 0 (USB MIDI) is never blocked, so audio on core 1 is
// completely unaffected.
#pragma once
#include <Arduino.h>

#include "hardware/pio.h"
#include "hardware/structs/resets.h"  // unreset_block_mask_wait_blocking, RESET_PIO2

#include "ws2812.pio.h"   // pioasm-generated ws2812 PIO program
#include "elements/resources.h"  // lut_db_led_brightness

namespace status_led {

constexpr uint8_t kPin = 22;
static const PIO kPio = pio2;   // pio2 is a macro ((pio_hw_t *)PIO2_BASE) — not constexpr
constexpr uint kSm = 0;
constexpr uint8_t kBootColorValue = 40;   // dim blue during setup
constexpr uint8_t kGateBrightness = 200;
constexpr uint8_t kModelPulseBrightness = 120;
constexpr uint32_t kPollIntervalMs = 20;
constexpr uint32_t kBootOkFlashMs = 250;
constexpr uint32_t kModelShowMs = 1024;

// Original dB brightness LUT indexed as ui.cc: lut[int32(level * 512)].
// lut_db_led_brightness has 516 entries; cap at index 512.
inline uint8_t dbBrightness(float level) {
  int32_t idx = static_cast<int32_t>(level * 512.0f);
  if (idx < 0) idx = 0;
  if (idx > 512) idx = 512;
  return static_cast<uint8_t>(elements::lut_db_led_brightness[idx]);
}

}  // namespace status_led

class StatusLed {
 public:
  void Init() {
    // PIO2 starts held in reset on RP2350; un-reset before touching it.
    unreset_block_mask_wait_blocking(1u << RESET_PIO2);
    uint offset = pio_add_program(status_led::kPio, &ws2812_program);
    ws2812_program_init(status_led::kPio, status_led::kSm, offset,
                        status_led::kPin, 800000.0f, false);

    boot_error_ = false;
    boot_ok_ms_ = 0;
    model_show_until_ = 0;
    last_poll_ms_ = 0;
    // Dim blue while booting; turns green on I2S OK below.
    showColor(0, 0, status_led::kBootColorValue);
  }

  void NotifyBootError() { boot_error_ = true; }
  void NotifyBootOk() { boot_ok_ms_ = millis(); }

  void NotifyModelChange(uint8_t model) {
    model_ = (model > 2) ? model_ : model;
    model_show_until_ = millis() + status_led::kModelShowMs;
  }

  // Call every core-0 loop iteration. Never blocks: pio_sm_put is skipped
  // when the TX FIFO is full, so USB MIDI servicing on this core is never
  // delayed by the LED. Audio rendering runs on core 1 and is unaffected.
  void Poll(bool gate, float exciter_level, float resonator_level) {
    const uint32_t now = millis();
    if (now - last_poll_ms_ < status_led::kPollIntervalMs) return;
    last_poll_ms_ = now;

    if (boot_error_) {
      const bool blink = (now & 127) > 64;
      showColor(blink ? 255 : 0, blink ? 0 : 255, 0);
      return;
    }
    if (boot_ok_ms_ != 0 && now - boot_ok_ms_ < status_led::kBootOkFlashMs) {
      showColor(0, 255, 0);
      return;
    }
    if (now < model_show_until_) {
      const bool blink = (now & 255) > 128;
      const uint8_t count = (now >> 8) & 3;
      const bool pulse = (count <= model_) && blink;
      const uint8_t v = pulse ? status_led::kModelPulseBrightness : 0;
      showColor(v, v, v);
      return;
    }
    // Normal mode: R = exciter, G = resonator, B = gate. Thresholded so the
    // LED flashes on strike and glows during sustain, off when idle.
    const bool excite = exciter_level > 0.05f;
    const bool reso = resonator_level > 0.05f;
    showColor(excite ? 200 : 0, reso ? 200 : 0, gate ? status_led::kGateBrightness : 0);
  }

 private:
  // Non-blocking on audio (core 1). Core 0 waits ~100 us per LED update
  // (negligible for USB MIDI) to hold the WS2812 reset line low long enough
  // for the latch — without this gap the WS2812 ignores every frame after
  // the first, which is why the LED appeared dead after boot.
  void showColor(uint8_t r, uint8_t g, uint8_t b) {
    if (pio_sm_is_tx_fifo_full(status_led::kPio, status_led::kSm)) return;
    const uint32_t grb = (uint32_t(g) << 16) | (uint32_t(r) << 8) | b;
    pio_sm_put(status_led::kPio, status_led::kSm, grb);
    // Wait for the 24 bits to shift out (~30 us), then hold the line LOW
    // for the WS2812 reset gap (>50 us) so the color latches.
    busy_wait_us(100);
  }

  bool boot_error_ = false;
  uint32_t boot_ok_ms_ = 0;
  uint32_t model_show_until_ = 0;
  uint8_t model_ = 0;
  uint32_t last_poll_ms_ = 0;
};
