// sc_kit_buf.h — SC 3.14.1 buffer UGens (GPL-3.0, DelayUGens.cpp).
// Info constants: SampleRate 434, ControlRate 436, BufSampleRate 495,
// BufFrames 509, BufRateScale 565. BufWr 1199, BufRd 1118 with the
// LOOP_BODY_1/2/4 interpolation macros (793-884). LocalBuf 604 allocates a
// server SndBuf; the host form uses a HostBuffer with the same data layout.
// The canonical refs keep phases away from the buffer edges so the guard
// frames of interp 4 are not exercised (wrapping is still implemented).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_BUF_H_
#define RACK_SRC_DRONECASTER_SC_KIT_BUF_H_
#include <cmath>
#include <cstdint>
#include <vector>

namespace sc {

struct HostBuffer {
  void Init(int frames, int channels, float sr = 48000.f) {
    frames_ = frames; channels_ = channels; sr_ = sr;
    data.assign(static_cast<size_t>(frames) * channels, 0.f);
  }
  float* At(int frame, int channel) {
    return data.data() + static_cast<size_t>(frame) * channels_ + channel;
  }
  const float* At(int frame, int channel) const {
    return data.data() + static_cast<size_t>(frame) * channels_ + channel;
  }
  int frames_ = 0, channels_ = 1;
  float sr_ = 48000.f;
  std::vector<float> data;
};

static inline int BufLoopPhase(double p, int frames, int loop) {
  const double hi = loop ? static_cast<double>(frames)
                         : static_cast<double>(frames - 1);
  if (p >= hi) {
    if (!loop) return frames - 1;
    p = std::fmod(p, hi + 1.0);
  } else if (p < 0.) {
    if (!loop) return 0;
    p = std::fmod(p, hi + 1.0);
    if (p < 0.) p += hi + 1.0;
  }
  return static_cast<int>(p);
}

struct BufRd {
  void Init(HostBuffer* b, int interp) { buf_ = b; interp_ = interp; }
  void ProcessBlock(const float* phase, float* out, int n, int loop) const {
    for (int i = 0; i < n; ++i) {
      const int ip = BufLoopPhase(phase[i], buf_->frames_, loop);
      const float frac = static_cast<float>(phase[i]) - static_cast<float>(ip);
      const float a = *buf_->At(ip, 0);
      if (interp_ == 1) { out[i] = a; continue; }
      if (interp_ == 2) {
        const int ip1 = ip + 1 < buf_->frames_ ? ip + 1 : (loop ? 0 : ip);
        out[i] = a + frac * (*buf_->At(ip1, 0) - a);
        continue;
      }
      // interp 4: 4-point Hermite (cubicinterp). Clip the neighbour indices
      // to the buffer for the non-wrapping case.
      auto at = [&](int k) -> float {
        if (loop) {
          int q = k % buf_->frames_;
          if (q < 0) q += buf_->frames_;
          return *buf_->At(q, 0);
        }
        if (k < 0) k = 0;
        if (k >= buf_->frames_) k = buf_->frames_ - 1;
        return *buf_->At(k, 0);
      };
      const float y0 = at(ip - 1), y1 = a, y2 = at(ip + 1), y3 = at(ip + 2);
      const float c0 = y1;
      const float c1 = 0.5f * (y2 - y0);
      const float c2 = y0 - 2.5f * y1 + 2.f * y2 - 0.5f * y3;
      const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
      out[i] = ((c3 * frac + c2) * frac + c1) * frac + c0;
    }
  }
  HostBuffer* buf_ = nullptr;
  int interp_ = 2;
};

struct BufWr {
  void Init(HostBuffer* b) { buf_ = b; }
  void ProcessBlock(const float* phase, const float* in, int n, int loop) const {
    for (int i = 0; i < n; ++i) {
      const int ip = BufLoopPhase(phase[i], buf_->frames_, loop);
      *buf_->At(ip, 0) = in[i];
    }
  }
  HostBuffer* buf_ = nullptr;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_BUF_H_
