// Rack Pico SDK scaffold — benjolin-first (2026-11-22).
//
// Core 1: the benjolin engine renders 16-sample blocks and pushes the
//   packed stereo words into the PIO I2S FIFO (the FIFO full = the pacing,
//   the same discipline as the Arduino build's write16); the post-Clip16
//   int16 pairs also feed the USB capture ring (the device-vs-golden rig
//   AND a host-facing recording path).
// Core 0: the USB composite device (CDC console + UAC1 capture) via the
//   TinyUSB task at ~1 ms cadence, with the 2 s report (proc max/avg +
//   capture glitch counters).
//
// The clocks: 307.2 MHz sys (the PLL 1536/5/1) — the DSPi insight: the PIO
// instruction rate lands close to an integer multiple of the I2S bit clock,
// so the fractional clkdiv jitter is minimal.
//
// The flags: -O3, -ffp-contract=off (see CMakeLists.txt) — the SRAM-law
// fight with the Arduino core's -Os append cannot happen here.
//
// NOT in this build yet (the migration order per the handoff): USB MIDI,
// the module swap machinery, presets, the C3 link, the WS2812 LED.
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "tusb.h"
#include "benjolin_engine.h"
#include "i2s_out.pio.h"

#include <cstdio>

#include "usb_console.h"
#include "usb_capture.h"
#include "usb_descriptors.h"

// The rack pin map (docs/rp2350-board.md): I2S BCK GP6, LRCK GP7, DOUT GP8.
static const uint kPinData = 8;
static const uint kPinBck = 6;
static const uint kPinLr = 7;

static benjolin::Engine eng;
static float tap_rings[8 * 16];      // the register-tap rings (the no-heap law)
static float pan_rings[2 * 1024];    // the pan-delay rings

static PIO pio = pio0;
static uint sm = 0;
static uint sm_offset = 0;

static volatile uint32_t proc_max_us = 0;
static volatile uint32_t proc_sum_us = 0;
static volatile uint32_t proc_count = 0;

// The spec-driven adapter values (the exact norns defaults — amp 0, so the
// boot is silent until a control raises it; the console `amp 60`-style
// commands come later — for now the scaffold renders the norns default
// state and prints the report).
static void params_init(benjolin::Params* p) {
  p->freq1 = 70.0f;        // the norns default
  p->freq2 = 4.0f;
  p->scale = 1.0f;
  p->rungler1 = 0.16f;
  p->rungler2 = 0.001f;
  p->rungler_f = 1.0f;
  p->loop = 1.0f;
  p->filt_freq = 40.0f;
  p->q = 0.02f;
  p->gain = 6.0f;          // the norns default gain = 6 (integer-stepped)
  p->filter_type = 0.0f;
  p->out_signal = 6.0f;
  p->amp = 0.0f;           // the norns default amp = 0 (the silent boot)
  p->pan = 0.0f;
}

static void i2s_init() {
  const uint offset = pio_add_program(pio, &i2s_out_program);
  sm_offset = offset;
  pio_sm_config c = i2s_out_program_get_default_config(offset);
  sm_config_set_out_pins(&c, kPinData, 1);
  sm_config_set_sideset_pins(&c, kPinBck);   // BCK GP6 + LR GP7 (the sideset 2)
  sm_config_set_out_shift(&c, true /* right = MSB first */, false, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  pio_gpio_init(pio, kPinData);
  pio_gpio_init(pio, kPinBck);
  pio_gpio_init(pio, kPinLr);
  pio_sm_set_consecutive_pindirs(pio, sm, kPinData, 1, true);
  pio_sm_set_consecutive_pindirs(pio, sm, kPinBck, 2, true);
  // The instruction rate = FS * (2 halves * (2 + 2*16 + 1) instrs) = FS * 70
  // (the fractional clkdiv: the exact BCK average; the sub-cycle jitter is
  // absorbed by the DAC's clock tolerance).
  const float div = static_cast<float>(clock_get_hz(clk_sys)) /
                    (48000.0f * 70.0f);
  sm_config_set_clkdiv(&c, div);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
}

static inline void i2s_write_stereo(int16_t l, int16_t r) {
  // The OSR right-shifts: bit 31 first = the left MSB, then the right 16.
  pio_sm_put_blocking(pio, sm,
      (static_cast<uint32_t>(static_cast<uint16_t>(l)) << 16) |
      static_cast<uint32_t>(static_cast<uint16_t>(r)));
}

// Core 1: the audio render loop.
static void core1_audio() {
  float l[16], r[16];
  int16_t l16[16], r16[16];
  while (true) {
    const uint32_t t0 = time_us_32();
    eng.Process16(l, r);
    const uint32_t dt = time_us_32() - t0;
    if (dt > proc_max_us) proc_max_us = dt;
    proc_sum_us += dt;
    ++proc_count;
    for (uint i = 0; i < 16; ++i) {
      // The int16 conversion (the headroom law: x16384 = +-2.0; the engine
      // output saturates at ~+-1, the LeakDC edge spikes reach +-2).
      int32_t ls = static_cast<int32_t>(l[i] * 16384.0f);
      int32_t rs = static_cast<int32_t>(r[i] * 16384.0f);
      if (ls > 32767) ls = 32767; else if (ls < -32768) ls = -32768;
      if (rs > 32767) rs = 32767; else if (rs < -32768) rs = -32768;
      l16[i] = static_cast<int16_t>(ls);
      r16[i] = static_cast<int16_t>(rs);
      i2s_write_stereo(l16[i], r16[i]);
    }
    // The capture feed: exactly the int16 pairs that reached the DAC, so a
    // host recording is sample-exact against the device-vs-golden rig.
    usb_capture_push_stereo(l16, r16, 16);
  }
}

int main() {
  // 307.2 MHz sys (the PLL 1536/5/1) + the vreg bump: the DSPi insight — the
  // PIO instruction rate lands near-integer over the I2S bit clock. The
  // voltage 1.15 V is required at this clock (the DSPi measured note).
  vreg_set_voltage(VREG_VOLTAGE_1_15);
  busy_wait_us(10000);
  set_sys_clock_khz(307200, true);

  stdio_init_all();   // the stdio framework (the CDC driver registers below)

  // The engine: the norns default state through the same params the console
  // will adjust later.
  benjolin::Params p;
  params_init(&p);
  eng.Init(48000.0, benjolin::FeedbackMode::kReferenceSc,
           1795055191,   // the golden DFM1 epoch (deterministic)
           p, tap_rings, pan_rings);
  rack_engine_rate_hz = 48000;   // the capture reports/paces this rate

  i2s_init();

  // The composite USB device: CDC console + UAC1 capture (usb_capture.cpp
  // registers its class driver through the usbd_app_driver_get_cb hook).
  usb_descriptors_init();
  usb_console_init();
  tud_init(0);

  multicore_launch_core1(core1_audio);

  // Core 0: the USB task pump + the report loop.  The ~1 ms cadence keeps
  // CDC latency low; the report fires every 2000 ticks.
  uint32_t last_sum = 0, last_count = 0;
  uint32_t tick = 0;
  while (true) {
    usb_console_tick();
    sleep_ms(1);
    if (++tick < 2000) continue;
    tick = 0;
    const uint32_t sum = proc_sum_us, count = proc_count;
    const uint32_t avg = (count > last_count)
        ? (sum - last_sum) / (count - last_count) : 0;
    printf("benjolin: max %u us avg %u us (budget 1333 us) | "
           "cap ovr=%u unr=%u\n",
           static_cast<unsigned>(proc_max_us),
           static_cast<unsigned>(avg),
           static_cast<unsigned>(usb_capture_overflow_count()),
           static_cast<unsigned>(usb_capture_underrun_count()));
    proc_max_us = 0;
    last_sum = sum;
    last_count = count;
  }
  return 0;
}
