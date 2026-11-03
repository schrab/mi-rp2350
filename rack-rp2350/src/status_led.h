// Status LED: one WS2812 on GP22.
//
// Mapping: dark = idle; amber = note gated; the exciter and resonator meters
// are blended in additively (exciter = turquoise, resonator = amber), so the
// LED brightens and shifts hue with the DSP activity and follows the original
// meter release tails. Three green blinks = USB MIDI host controller mounted.
// Boot: 3 white flashes (self-test), then dark.
//
// The WS2812 is driven via PIO2 SM0 (PIO0 = USB host; I2S lands on PIO2 SM1
// after the SM0 claim in Init()) using the SDK ws2812 PIO program. Each frame
// is followed by a 1ms wait (~30us shift-out time + >50us WS2812 reset gap)
// so colors latch reliably. Core 0 is blocked ~5% of the time (1 ms per
// 20 ms poll) — negligible for USB MIDI, zero impact on core 1 audio.
#pragma once
#include <Arduino.h>

#include "hardware/pio.h"
#include "hardware/structs/resets.h"  // unreset_block_mask_wait_blocking, RESET_PIO2

#include "led_brightness_lut.h"       // lut_db_led_brightness (Elements curve)
#include "ws2812.pio.h"               // pioasm-generated ws2812 PIO program

namespace status_led {

constexpr uint8_t kPin = 22;
static const PIO kPio = pio2;   // pio2 is a macro ((pio_hw_t *)PIO2_BASE) — not constexpr
constexpr uint kSm = 0;
// Gate: amber base. Scaled to 180/255 so the meter colors have headroom to
// shift the hue instead of clipping on amber's already-saturated red.
constexpr uint8_t kGateAmberR = 255;
constexpr uint8_t kGateAmberG = 100;
constexpr uint8_t kGateAmberLevel = 180;
// Exciter keeps the original Elements LED1 turquoise. The resonator meter is
// amber (same hue as the gate) instead of the original rose/fuchsia, so it
// brightens the note color rather than tinting it pink.
constexpr uint8_t kExciterR = 0;
constexpr uint8_t kExciterG = 200;
constexpr uint8_t kExciterB = 160;
constexpr uint8_t kResonatorR = 255;
constexpr uint8_t kResonatorG = 100;
constexpr uint8_t kResonatorB = 0;
constexpr uint8_t kControllerGreen = 200;
constexpr uint32_t kPollIntervalMs = 20;
constexpr uint32_t kControllerBlinkOnMs = 150;
constexpr uint32_t kControllerBlinkOffMs = 150;
constexpr uint32_t kControllerBlinkCount = 3;

}  // namespace status_led

class StatusLed {
 public:
  void Init() {
    // PIO2 starts held in reset on RP2350; un-reset before touching it.
    unreset_block_mask_wait_blocking(1u << RESET_PIO2);

    // Claim SM0 BEFORE loading/enabling the program. The Arduino PIOProgram
    // allocator (used by I2S) scans PIOs from the highest index down and
    // claims the first unclaimed SM. Without this claim, i2s.begin() takes
    // PIO2 SM0, reconfigures it for I2S and the LED freezes on its last
    // color while audio keeps working.
    pio_sm_claim(status_led::kPio, status_led::kSm);

    offset_ = pio_add_program(status_led::kPio, &ws2812_program);
    ws2812_program_init(status_led::kPio, status_led::kSm, offset_,
                        status_led::kPin, 800000.0f, false);

    boot_error_ = false;
    controller_blink_active_ = false;
    controller_blink_start_ms_ = 0;
    last_poll_ms_ = 0;

    // Sanity flash: 3x bright white — confirms driver + PIO2 + wiring.
    for (int i = 0; i < 3; ++i) {
      showColor(120, 120, 120);
      delay(150);
      showColor(0, 0, 0);
      delay(150);
    }
    showColor(0, 0, 0);  // idle: dark
  }

  void NotifyBootError() { boot_error_ = true; }

  // TEMPORARY (boot-wedge diagnosis, round 2): raw color access for boot-stage
  // breadcrumbs; a wedged boot freezes on its stage color (latched until the
  // next reset). REMOVE after the wedge is fully localized.
  void Breadcrumb(uint8_t r, uint8_t g, uint8_t b) { showColor(r, g, b); }

  // USB-A MIDI host controller mounted: 3 green blinks, then resume normal.
  void NotifyControllerConnected() {
    controller_blink_start_ms_ = millis();
    controller_blink_active_ = true;
  }

  // Boot diagnostics: instruction-memory offset of the WS2812 program.
  uint Offset() const { return offset_; }

  // Call every core-0 loop iteration. Blocks ~1ms per color update for the
  // WS2812 reset gap — negligible for USB MIDI, zero impact on core 1 audio.
  //
  // exciter_level / resonator_level are Part::exciter_level() and
  // Part::resonator_level() — already scaled by Part and smoothed by the
  // original per-sample meters, so the LED inherits their ~31 ms / ~62 ms
  // release tails.
  void Poll(bool gate, float exciter_level, float resonator_level) {
    const uint32_t now = millis();
    if (now - last_poll_ms_ < status_led::kPollIntervalMs) return;
    last_poll_ms_ = now;

    if (boot_error_) {
      const bool blink = (now & 127) > 64;
      showColor(blink ? 255 : 0, blink ? 0 : 255, 0);
      return;
    }

    if (controller_blink_active_) {
      constexpr uint32_t period =
          status_led::kControllerBlinkOnMs + status_led::kControllerBlinkOffMs;
      const uint32_t elapsed = now - controller_blink_start_ms_;
      if (elapsed / period >= status_led::kControllerBlinkCount) {
        controller_blink_active_ = false;
      } else {
        const bool on = (elapsed % period) < status_led::kControllerBlinkOnMs;
        showColor(0, on ? status_led::kControllerGreen : 0, 0);
        return;
      }
    }

    const uint8_t exciter = meterBrightness(exciter_level);
    const uint8_t resonator = meterBrightness(resonator_level);

    // Idle is dark. The meters keep their release tail after the gate drops,
    // so the LED fades out over the original ~31/62 ms instead of snapping.
    if (!gate && exciter == 0 && resonator == 0) {
      showColor(0, 0, 0);
      return;
    }

    // Additive blend: gate amber + exciter turquoise + resonator amber.
    // The gate sets the base amber; the exciter tints it toward cyan and the
    // resonator simply brightens the amber. Both active washes toward white.
    const uint16_t gate_level =
        gate ? status_led::kGateAmberLevel : static_cast<uint16_t>(0);
    const uint16_t red =
        gate_level * status_led::kGateAmberR / 255 +
        exciter * status_led::kExciterR / 255 +
        resonator * status_led::kResonatorR / 255;
    const uint16_t green =
        gate_level * status_led::kGateAmberG / 255 +
        exciter * status_led::kExciterG / 255 +
        resonator * status_led::kResonatorG / 255;
    const uint16_t blue =
        exciter * status_led::kExciterB / 255 +
        resonator * status_led::kResonatorB / 255;

    showColor(clamp8(red), clamp8(green), clamp8(blue));
  }

 private:
  // Original Elements UI curve: lut_db_led_brightness[(level * 512)] maps a
  // 0..1 meter value over ~54 dB to 0..255. Below 1/512 the LED is off.
  static uint8_t meterBrightness(float level) {
    if (!(level > 0.0f)) return 0;  // also rejects NaN
    int32_t index = static_cast<int32_t>(level * 512.0f);
    if (index > 512) index = 512;
    return static_cast<uint8_t>(lut_db_led_brightness[index]);
  }

  static uint8_t clamp8(uint16_t value) {
    return value > 255 ? 255 : static_cast<uint8_t>(value);
  }

  // Sends one 24-bit GRB frame, then waits for the WS2812 reset gap
  // (>50us line-low so the color latches). 1ms is massive overkill but
  // guaranteed.
  void showColor(uint8_t r, uint8_t g, uint8_t b) {
    // The PIO program is configured with sm_config_set_out_shift(&c, false,
    // true, 24): shift LEFT, autopull at 24 bits. It therefore transmits the
    // TOP 24 bits of the FIFO word, so the GRB frame must be left-aligned
    // (the low 8 bits are discarded). Without the << 8 the LED receives
    // [0x00, g, r] instead of [g, r, b] — e.g. white becomes violet and
    // blue-only frames become black.
    const uint32_t grb = ((uint32_t(g) << 16) | (uint32_t(r) << 8) | b) << 8;
    pio_sm_put(status_led::kPio, status_led::kSm, grb);
    delay(1);
  }

  uint offset_ = 0;
  bool boot_error_ = false;
  bool controller_blink_active_ = false;
  uint32_t controller_blink_start_ms_ = 0;
  uint32_t last_poll_ms_ = 0;
};
