// Rack I2S output — the AudioBufferManager replacement (Pico SDK build).
//
// Implements the exact adapter-facing surface the Arduino build's I2S object
// exposed (main.cpp + every module_*.h): setBitsPerSample(16) /
// setBuffers(n, words) / setFrequency(rate) / begin() / end() / blocking
// write16(l, r). Pacing is the DMA bank: n buffers of words 32-bit stereo
// frames; a full bank absorbs a module's burst (the stall metric measures
// the blocking time at the ADAPTER, unchanged).
//
// Mechanics: PIO0 SM0 runs i2s_out.pio (one FIFO word = one stereo frame,
// sideset BCK+LR); a single DMA channel transfers one buffer per arm; the
// DMA_IRQ_0 handler (owned by CORE 1 — the long-session bitcrush law, the
// Arduino build's fb5d404) pops the next ready buffer or repeats a silence
// buffer on underflow (AudioBufferManager's underflow behavior). The render
// loop (core 1) is the sole producer: write16 fills the current buffer and,
// when full, blocks until the DMA has released a buffer back.
//
// Reconfig discipline (identical to the Arduino build): end()/begin() run on
// CORE 0 inside the swap/chain handshake while core 1 has masked DMA_IRQ_0;
// the IRQ-side guards below make any stray completion harmless.
#ifndef _RACK_I2S_H_
#define _RACK_I2S_H_

#include <stdint.h>
#include <stdbool.h>

class RackI2sOut {
 public:
  RackI2sOut() = default;

  bool setBitsPerSample(unsigned bits);           // only 16 supported (adapter law)
  bool setBuffers(unsigned buffers, unsigned words);  // words = stereo frames per buffer
  bool setFrequency(uint32_t hz);
  bool begin();
  void end();

  // Blocking: paces at the output rate when the bank is full. Two int16
  // samples packed into one FIFO word ((L << 16) | (R & 0xFFFF)).
  void write16(int16_t l, int16_t r);

  uint32_t stall_count() const { return stall_count_; }

 private:
  static constexpr int kPinData = 8;
  static constexpr int kPinBck = 6;             // LRCLK follows on GP7
  static constexpr unsigned kMaxBuffers = 64;
  static constexpr unsigned kSilenceDmaWords = 16;  // underflow repeat buffer

  void handoff_();                    // push the full buffer, take a free one
  static void dma_irq_();             // shared handler (single instance)

  unsigned buffers_ = 0;
  unsigned words_ = 0;
  uint32_t rate_ = 48000;
  bool running_ = false;

  // Bank (allocated at begin: buffers_ * words_ * 4 B) + the silence repeat.
  uint32_t* bank_ = nullptr;
  uint32_t silence_[kSilenceDmaWords];

  // Producer state (core 1 render loop): the buffer it is filling.
  unsigned fill_buf_ = 0;
  unsigned fill_pos_ = 0;

  // Ready ring: filled buffers waiting for the DMA (producer pushes at
  // ready_head_, the DMA IRQ pops from ready_tail_).
  unsigned ready_q_[kMaxBuffers];
  volatile unsigned ready_head_ = 0, ready_tail_ = 0;
  // Free ring: buffers released by the DMA (the IRQ pushes at free_head_,
  // the producer pops from free_tail_).
  unsigned free_q_[kMaxBuffers];
  volatile unsigned free_head_ = 0, free_tail_ = 0;
  volatile int dma_buf_ = -1;         // bank buffer the DMA is reading now
  volatile uint32_t stall_count_ = 0;

  uint32_t last_wait_t0_ = 0;
};

// The adapters + main.cpp reference the SAME global name the Arduino build
// used (midi.h's extern matches this type in the Pico build).
extern RackI2sOut i2s;

#endif  // _RACK_I2S_H_
