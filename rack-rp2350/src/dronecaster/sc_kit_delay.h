// sc_kit_delay.h — SC 3.14.1 core delay family (GPL-3.0, DelayUGens.cpp).
// One power-of-two ring buffer shared by DelayN/L/C, CombN/L/C and
// AllpassN/L/C. Sources:
//   CalcDelay 3456, DelayUnit_Reset 3462, DelayUnit_AllocDelayLine 3438,
//   CalcFeedback (SC_InlineUnaryOp.h:267), helpers 2180-2680,
//   Ctors 3602 (DelayN), 3823 (CombN), 4063 (AllpassN).
// The ctor sets ZOUT0 = 0 and starts iwrphase=0 / zeroed buffer, then the
// initial "checked" phase outputs zeros until the read phase is non-negative
// (the source switches to unchecked after a full buffer has been written —
// with a fresh zeroed buffer the two produce identical samples, so we keep
// the checked behavior throughout).
//  kind: 0 DelayN, 1 DelayL, 2 DelayC, 3 CombN, 4 CombL, 5 CombC,
//        6 AllpassN, 7 AllpassL, 8 AllpassC.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_DELAY_H_
#define RACK_SRC_DRONECASTER_SC_KIT_DELAY_H_
#include <cmath>
#include <cstdint>
#include <vector>

namespace sc {

static inline float LinInterp(float x, float a, float b) {
  return a + x * (b - a);
}
static inline float CubicInterp(float x, float y0, float y1, float y2,
                                float y3) {
  const float c0 = y1;
  const float c1 = 0.5f * (y2 - y0);
  const float c2 = y0 - 2.5f * y1 + 2.f * y2 - 0.5f * y3;
  const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
  return ((c3 * x + c2) * x + c1) * x + c0;
}

struct DelayLine {
  void Init(double sr, float maxdelay, float delay, float decay, int kind) {
    sr_ = static_cast<float>(sr);
    kind_ = kind;
    delay_ = delay;
    decay_ = decay;
    long sz = static_cast<long>(std::ceil(maxdelay * sr_ + 1.f)) + 64;
    bufsize_ = 1;
    while (bufsize_ < sz) bufsize_ <<= 1;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    mask_ = bufsize_ - 1;
    iwrphase_ = 0;
    dsamp_ = CalcDelay(delay);
    feedbk_ = kind_ >= 3 ? CalcFeedback(delay, decay) : 0.f;
  }
  void ProcessBlock(const float* in, float* out, int n, float delay,
                    float decay) {
    if (delay != delay_ || (kind_ >= 3 && decay != decay_)) {
      const float next_ds = CalcDelay(delay);
      const float next_fb = kind_ >= 3 ? CalcFeedback(delay, decay) : 0.f;
      const float ds_slope = (next_ds - dsamp_) * (1.f / 64.f);
      const float fb_slope = (next_fb - feedbk_) * (1.f / 64.f);
      for (int i = 0; i < n; ++i) {
        dsamp_ += ds_slope;
        feedbk_ += fb_slope;
        out[i] = Step(in[i]);
      }
      delay_ = delay;
      decay_ = decay;
    } else {
      for (int i = 0; i < n; ++i) out[i] = Step(in[i]);
    }
  }
  float ctor_value() const { return 0.f; }

 private:
  float CalcDelay(float delaytime) const {
    const float min_delay = (kind_ == 2 || kind_ == 5 || kind_ == 8) ? 2.f : 1.f;
    const float d = delaytime * sr_;
    return d < min_delay ? min_delay : (d > static_cast<float>(bufsize_)
                                            ? static_cast<float>(bufsize_)
                                            : d);
  }
  static float CalcFeedback(float delaytime, float decaytime) {
    if (delaytime == 0.f || decaytime == 0.f) return 0.f;
    const double kLog001 = -6.9077552789821368;  // std::log(0.001)
    const float absret = static_cast<float>(
        std::exp(kLog001 * static_cast<double>(delaytime) /
                 std::fabs(static_cast<double>(decaytime))));
    return std::copysign(absret, decaytime);
  }
  float Step(float x) {
    const long idsamp = static_cast<long>(dsamp_);
    const float frac = dsamp_ - static_cast<float>(idsamp);
    const long ird = static_cast<long>(iwrphase_) - idsamp;
    const long w = iwrphase_ & mask_;
    const long r1 = ird & mask_, r0 = (ird + 1) & mask_;
    const long rm1 = (ird - 1) & mask_, rm2 = (ird - 2) & mask_;
    float value = 0.f;

    if (kind_ == 0 || kind_ == 3 || kind_ == 6) {  // nearest-neighbour
      if (ird < 0) {
        // Every checked helper writes the raw input during this phase.
        if (kind_ == 6) {
          buf_[w] = x;
          value = -feedbk_ * x;
        } else {
          buf_[w] = x;
        }
      } else {
        value = buf_[r1];
        if (kind_ == 0) buf_[w] = x;
        else if (kind_ == 3) buf_[w] = x + feedbk_ * value;
        else {  // AllpassN
          const float dwr = feedbk_ * value + x;
          buf_[w] = dwr;
          value = value - feedbk_ * dwr;
        }
      }
    } else if (kind_ == 1 || kind_ == 4 || kind_ == 7) {  // linear
      if (ird < 0) {
        value = 0.f;
      } else if (ird - 1 < 0) {
        const float d1 = buf_[r1];
        value = d1 - frac * d1;
      } else {
        value = LinInterp(frac, buf_[r1], buf_[rm1]);
      }
      if (kind_ == 1) buf_[w] = x;
      else if (kind_ == 4) buf_[w] = x + feedbk_ * value;
      else {
        const float dwr = x + feedbk_ * value;
        buf_[w] = dwr;
        value = value - feedbk_ * dwr;
      }
    } else {  // cubic (2 DelayC, 5 CombC, 8 AllpassC)
      if (ird + 1 < 0) {
        buf_[w] = x;  // source still stores the input, output stays 0
      } else {
        float d0 = 0.f, d1 = 0.f, d2 = 0.f, d3 = 0.f;
        if (ird >= 0) d1 = buf_[r1];
        if (ird - 1 >= 0) d2 = buf_[rm1];
        if (ird - 2 >= 0) d3 = buf_[rm2];
        d0 = buf_[r0];
        value = CubicInterp(frac, d0, d1, d2, d3);
        if (kind_ == 2) buf_[w] = x;
        else if (kind_ == 5) buf_[w] = x + feedbk_ * value;
        else {
          const float dwr = x + feedbk_ * value;
          buf_[w] = dwr;
          value = value - feedbk_ * dwr;
        }
      }
    }
    ++iwrphase_;
    return value;
  }
  std::vector<float> buf_;
  long bufsize_ = 0, mask_ = 0, iwrphase_ = 0;
  float sr_ = 48000.f, dsamp_ = 0.f, feedbk_ = 0.f;
  float delay_ = 0.f, decay_ = 0.f;
  int kind_ = 0;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_DELAY_H_
