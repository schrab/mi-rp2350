// sc_kit_localio.h — SC 3.14.1 LocalIn / LocalOut (GPL-3.0, IOUGens.cpp).
// Anchors: LocalIn_next_a 1504, LocalIn_next_k 1566, LocalIn_Ctor 1581,
// LocalOut_next_a 1634. The upstream bus lives in the synth's parent group
// (mLocalAudioBusUnit), is numChannels * BUFLENGTH floats, and each channel
// carries a "touched" block counter. This host-testable form keeps the exact
// law in a LocalBus object; the firmware wires one bus into the voice arena.
//
// Block counter semantics: LocalIn reads the bus when (counter - touched) is
// 0 or 1, otherwise it emits its default input. LocalOut copies into the bus
// on the first write of a block (setting touched) and ACCUMULATES thereafter.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_LOCALIO_H_
#define RACK_SRC_DRONECASTER_SC_KIT_LOCALIO_H_
#include <cstdint>
#include <vector>

namespace sc {

struct LocalBus {
  void Init(int channels) {
    buf_.assign(static_cast<size_t>(channels) * 64, 0.f);
    touched_.assign(static_cast<size_t>(channels), -1);
  }
  float* Channel(int ch) { return buf_.data() + static_cast<size_t>(ch) * 64; }
  std::vector<float> buf_;
  std::vector<int32_t> touched_;
};

struct LocalIn {
  void Init(int channels) { channels_ = channels; }
  // out[ch] gets either the previous block's bus data or default[ch].
  void ProcessBlock(LocalBus& bus, float** out, const float* defaults,
                    int block_counter, int n) {
    for (int ch = 0; ch < channels_; ++ch) {
      const int32_t diff = block_counter - bus.touched_[static_cast<size_t>(ch)];
      float* src = bus.Channel(ch);
      if (diff == 0 || diff == 1) {
        for (int i = 0; i < n; ++i) out[ch][i] = src[i];
      } else {
        for (int i = 0; i < n; ++i) out[ch][i] = defaults[ch];
      }
    }
  }
  int channels_ = 1;
};

struct LocalOut {
  void Init(int channels) { channels_ = channels; }
  void ProcessBlock(LocalBus& bus, const float* const* in, int block_counter,
                    int n) {
    for (int ch = 0; ch < channels_; ++ch) {
      float* dst = bus.Channel(ch);
      int32_t& touched = bus.touched_[static_cast<size_t>(ch)];
      if (touched == block_counter) {
        for (int i = 0; i < n; ++i) dst[i] += in[ch][i];
      } else {
        for (int i = 0; i < n; ++i) dst[i] = in[ch][i];
        touched = block_counter;
      }
    }
  }
  int channels_ = 1;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_LOCALIO_H_
