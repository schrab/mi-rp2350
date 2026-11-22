// Rack I2S output implementation (see rack_i2s.h for the design contract).
#include "rack_i2s.h"

#include <cstdlib>
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "i2s_out.pio.h"

namespace {
PIO s_pio = pio0;
uint s_sm = 0;
uint s_offset = 0;
bool s_pio_ready = false;
uint s_dma = 0;
bool s_dma_claimed = false;
RackI2sOut* s_inst = nullptr;   // the shared DMA IRQ handler dispatches here

inline void i2s_dma_arm(PIO pio, uint sm, uint dma, const void* buf,
                        uint words) {
  dma_channel_config c = dma_channel_get_default_config(dma);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  dma_channel_configure(dma, &c, reinterpret_cast<volatile void*>(&pio->txf[sm]),
                        buf, words, true /* trigger */);
}
}  // namespace

bool RackI2sOut::setBitsPerSample(uint bits) { return bits == 16; }

bool RackI2sOut::setBuffers(uint buffers, uint words) {
  if (running_) return false;
  if (buffers == 0 || buffers > kMaxBuffers || words == 0 || words > 256) {
    return false;
  }
  buffers_ = buffers;
  words_ = words;
  return true;
}

bool RackI2sOut::setFrequency(uint32_t hz) {
  if (hz < 8000 || hz > 192000) return false;
  rate_ = hz;
  return true;
}

void RackI2sOut::dma_irq_() {
  RackI2sOut* self = s_inst;
  if (!self || !self->running_) {
    // Reconfig window (core 1 masks the IRQ around end/begin) or teardown —
    // ack and get out without touching the rings.
    dma_hw->intr = 1u << s_dma;
    return;
  }
  dma_hw->intr = 1u << s_dma;   // ack first (fires per completed buffer)

  // Release the buffer just consumed back to the producer.
  if (self->dma_buf_ >= 0) {
    self->free_q_[self->free_head_] = static_cast<uint>(self->dma_buf_);
    self->free_head_ = (self->free_head_ + 1u) % self->buffers_;
    self->dma_buf_ = -1;
  }
  // Arm the next ready buffer, or the silence repeat on underflow.
  if (self->ready_tail_ != self->ready_head_) {
    const uint next = self->ready_q_[self->ready_tail_];
    self->ready_tail_ = (self->ready_tail_ + 1u) % self->buffers_;
    self->dma_buf_ = static_cast<int>(next);
    i2s_dma_arm(s_pio, s_sm, s_dma,
                self->bank_ + static_cast<size_t>(next) * self->words_,
                self->words_);
  } else {
    i2s_dma_arm(s_pio, s_sm, s_dma, self->silence_, self->words_);
  }
}

bool RackI2sOut::begin() {
  if (running_) return false;
  if (buffers_ == 0 || words_ == 0) return false;

  if (!s_pio_ready) {
    s_offset = pio_add_program(s_pio, &i2s_out_program);
    s_pio_ready = true;
  }
  // Full SM config on EVERY begin() (module swaps re-begin; a config built
  // only on first claim would lose the pin/shift settings on reconfig).
  pio_sm_config c = i2s_out_program_get_default_config(s_offset);
  sm_config_set_out_pins(&c, kPinData, 1);
  sm_config_set_sideset_pins(&c, kPinBck);
  // The core's pio_i2s_out config: LEFT shift (bit 31 = the left MSB goes
  // out first), autopull at 32 bits (one FIFO word = one L+R pair, packed
  // (left << 16) | right). AUTOPIULL also REQUIRED: the program has no
  // explicit `pull` — with it off the SM shifted stale OSR bits forever and
  // the render stalled after the 8-buffer bank (bench-caught 2026-11-23).
  sm_config_set_out_shift(&c, false /* left = bit 31 first */, true /* autopull */, 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  pio_gpio_init(s_pio, kPinData);
  pio_gpio_init(s_pio, kPinBck);
  pio_gpio_init(s_pio, kPinBck + 1);
  pio_sm_set_consecutive_pindirs(s_pio, s_sm, kPinData, 1, true);
  pio_sm_set_consecutive_pindirs(s_pio, s_sm, kPinBck, 2, true);
  pio_sm_set_set_pins(s_pio, s_sm, kPinData, 1);
  pio_sm_set_set_pins(s_pio, s_sm, kPinBck, 2);
  // y = bits - 2 (14 for 16-bit): the core's half-frame loop counter
  // (15 loop bits + the last bit with the WCLK flip = 16 per half).
  pio_sm_exec(s_pio, s_sm, pio_encode_set(pio_y, 14));
  // The instruction rate = FS * 64 (32 instrs per half-frame: the mov +
  // 15*2 loop instrs + the last bit; the core's bitClk = FS*bps*2ch*2edges
  // = FS*64 for 16-bit stereo). The fractional clkdiv is exact on average
  // with +-1 sys-cycle jitter — absorbed by the PCM5102A's PLL.
  const float div = static_cast<float>(clock_get_hz(clk_sys)) /
                    (static_cast<float>(rate_) * 64.0f);
  sm_config_set_clkdiv(&c, div);
  pio_sm_init(s_pio, s_sm, s_offset, &c);
  pio_sm_set_enabled(s_pio, s_sm, true);

  bank_ = static_cast<uint32_t*>(malloc(
      static_cast<size_t>(buffers_) * words_ * sizeof(uint32_t)));
  if (!bank_) return false;
  memset(bank_, 0, static_cast<size_t>(buffers_) * words_ * sizeof(uint32_t));
  memset(silence_, 0, sizeof(silence_));

  fill_buf_ = 0;
  fill_pos_ = 0;
  ready_head_ = ready_tail_ = 0;
  // Free ring holds every buffer except the producer-held 0: pushed 1..N-1.
  for (uint b = 1; b < buffers_; ++b) free_q_[b - 1] = b;
  free_head_ = buffers_ - 1;
  free_tail_ = 0;
  dma_buf_ = -1;

  if (!s_dma_claimed) {
    const int ch = dma_claim_unused_channel(true);
    if (ch < 0) {
      free(bank_);
      bank_ = nullptr;
      return false;
    }
    s_dma = static_cast<uint>(ch);
    s_dma_claimed = true;
    s_inst = this;
    irq_set_exclusive_handler(DMA_IRQ_0, &RackI2sOut::dma_irq_);
    // NOTE: the ENABLE side of the handoff is core 1's (setup1:
    // irq_set_enabled(DMA_IRQ_0, true)); the IRQ stays masked through core
    // 0's begin(), so this first arm runs un-interrupted.
  }
  // EVERY begin() re-arms the DMA completion IRQ: end() cleared the
  // channel's INTE bit (dma_channel_set_irq0_enabled(false)) and the
  // first-claim block above never runs again — without this re-enable the
  // first module swap left the bank undrained (the DMA sat idle after its
  // silence burst, every module silent; bench-caught 2026-11-23).
  dma_channel_set_irq0_enabled(s_dma, true);

  // Arm the first silence repeat; real audio replaces it on the first
  // completed-buffer IRQ once the producer has handed off a buffer.
  i2s_dma_arm(s_pio, s_sm, s_dma, silence_, words_);
  running_ = true;
  return true;
}

void RackI2sOut::end() {
  if (!running_) return;
  running_ = false;
  dma_channel_set_irq0_enabled(s_dma, false);
  dma_channel_abort(s_dma);          // stop an in-flight buffer transfer
  pio_sm_set_enabled(s_pio, s_sm, false);
  pio_sm_drain_tx_fifo(s_pio, s_sm);
  free(bank_);
  bank_ = nullptr;
  fill_pos_ = 0;
  ready_head_ = ready_tail_ = 0;
}

void RackI2sOut::handoff_() {
  // Push the full buffer into the ready ring (the DMA IRQ pops it).
  ready_q_[ready_head_] = fill_buf_;
  ready_head_ = (ready_head_ + 1u) % buffers_;
  // Take the next buffer to fill: normally one was released by the DMA IRQ.
  if (free_tail_ == free_head_) {
    // Bank exhausted: block until the DMA releases one. This is the pacing
    // point — one buffer period per block; the adapters time it against
    // kStallUs. 2 s with no release = the DMA is gone (teardown raced us):
    // reclaim buffer 0 rather than wedge the render loop forever.
    last_wait_t0_ = time_us_32();
    while (free_tail_ == free_head_) {
      tight_loop_contents();
      if (time_us_32() - last_wait_t0_ > 2000000u) {
        ++stall_count_;
        fill_buf_ = 0;
        fill_pos_ = 0;
        return;
      }
    }
  }
  fill_buf_ = free_q_[free_tail_];
  free_tail_ = (free_tail_ + 1u) % buffers_;
  fill_pos_ = 0;
}

void RackI2sOut::write16(int16_t l, int16_t r) {
  if (!running_) return;
  if (fill_pos_ >= words_) handoff_();
  bank_[static_cast<size_t>(fill_buf_) * words_ + fill_pos_] =
      (static_cast<uint32_t>(static_cast<uint16_t>(l)) << 16) |
      static_cast<uint32_t>(static_cast<uint16_t>(r));
  ++fill_pos_;
}

RackI2sOut i2s;
