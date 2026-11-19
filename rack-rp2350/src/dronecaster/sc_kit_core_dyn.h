// sc_kit_core_dyn.h — SC 3.14.1 core dynamics (GPL-3.0, CORE FilterUGens.cpp).
// Named `core_dyn` to leave `sc_kit_dyn.h` for the plan's Task 14 (SC3P dyn).
// One-piece ports of Amplitude, Compander and the lookahead Limiter.
//
// Anchors (Version-3.14.1), FilterUGens.cpp:
//   Amplitude_Ctor 3481-3505 / Amplitude_next 3507-3522
//   Compander_Ctor 3230-3240 / Compander_next 3242-3298
//   Limiter_Ctor  3394-3418 / Limiter_next  3421-3477
//   log1 = log(0.1) (SC_Constants.h:51), CALCSLOPE = (next-prev)/blockSize
//   (SC_Rate.cpp:34), ZOFF = 0 on the post-increment x86 build (Unroll.h:122).
//
// Load-bearing laws:
//  * All three ctors call their _next(unit, 1) — the FIRST input sample is
//    consumed before the block loop, so state (previn / gain / pos+curmaxval)
//    is already advanced when the first real block starts.
//  * Amplitude: val = |x|; rising uses clampcoef, falling uses relaxcoef,
//    where coef = log1 == 0 ? 0 : exp(log(0.1)/(time*sr)). The output IS the
//    state (no separate output filter).
//  * Compander: the envelope tracks |control| (not |in|) over the WHOLE
//    block; ONE gain = (|ctrl|/thresh)^(slope-1) is derived from its FINAL
//    value (zapgremlins on the below-threshold branch), then CALCSLOPE-ramped
//    across the block. slope==1 short-circuits to 1. Computing the gain per
//    sample instead is wrong — it is a block-rate decision.
//  * Limiter: three bufsize ring buffers rotate once per bufsize samples;
//    output is delayed by two full buffers (flips >= 2) and scaled by a
//    level that ramps from the previous buffer's peak ratio. The ctor's
//    one sample leaves pos=1, curmaxval=|in[0]|, flips=0.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_CORE_DYN_H_
#define RACK_SRC_DRONECASTER_SC_KIT_CORE_DYN_H_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace sc {

// FilterUGens.cpp:3253 / 3498 / 3547: exp(log1/(time*SAMPLERATE)), log1 double.
static inline float DynTimeCoef(float time, double sr) {
  const double kLog1 = -2.30258509299404568402;  // std::log(0.1)
  return time == 0.0f
             ? 0.0f
             : static_cast<float>(std::exp(kLog1 / (static_cast<double>(time) * sr)));
}

// Amplitude — FilterUGens.cpp:3481 (ctor), 3507 (next).
struct Amplitude {
  void Init(double sr, float attack, float release, float first_in) {
    clampcoef_ = DynTimeCoef(attack, sr);
    relaxcoef_ = DynTimeCoef(release, sr);
    previn_ = std::fabs(first_in);
    float unused;
    Step(first_in, unused);  // ctor's Amplitude_next(unit, 1)
  }
  void ProcessBlock(const float* in, float* out, int n) {
    for (int i = 0; i < n; ++i) Step(in[i], out[i]);
  }
  float ctor_value() const { return previn_; }

 private:
  void Step(float x, float& out) {
    float val = std::fabs(x);
    if (val < previn_) {
      val = val + (previn_ - val) * relaxcoef_;
    } else {
      val = val + (previn_ - val) * clampcoef_;
    }
    out = previn_ = val;
  }
  float previn_ = 0.0f, clampcoef_ = 0.0f, relaxcoef_ = 0.0f;
};

// Compander — FilterUGens.cpp:3230 (ctor), 3242 (next).
// Params are the ctor/scalar-rate constants of the canonical graph; SC
// recomputes the two time coefs per block, which is a no-op at fixed params.
struct Compander {
  void Init(double sr, float thresh, float slope_below, float slope_above,
            float clamp_time, float relax_time, float first_in,
            float first_ctrl) {
    static_cast<void>(sr);
    thresh_ = thresh;
    sb_ = slope_below;
    sa_ = slope_above;
    clampcoef_ = DynTimeCoef(clamp_time, sr);
    relaxcoef_ = DynTimeCoef(relax_time, sr);
    clamp_ = relax_ = 0.0f;  // source ctor zeros the cached param values
    prevmax_ = 0.0f;
    gain_ = 0.0f;
    static_cast<void>(first_in);  // ctor's out[0] = in[0] * gain(=0) = 0
    Envelope(first_ctrl);         // ctor's Compander_next(unit, 1)
    gain_ += RampGain();          // out[0] = in[0]*0 = 0; gain advances once
  }
  void ProcessBlock(const float* in, const float* ctrl, float* out, int n) {
    for (int i = 0; i < n; ++i) Envelope(ctrl[i]);
    const float slope = RampGain();
    for (int i = 0; i < n; ++i) {
      out[i] = in[i] * gain_;
      gain_ += slope;
    }
  }
  float ctor_value() const { return gain_; }

 private:
  void Envelope(float control) {
    float val = std::fabs(control);
    if (val < prevmax_) {
      val = val + (prevmax_ - val) * relaxcoef_;
    } else {
      val = val + (prevmax_ - val) * clampcoef_;
    }
    prevmax_ = val;
  }
  // One gain from the block's FINAL envelope value; returns CALCSLOPE slope.
  float RampGain() {
    float next;
    if (prevmax_ < thresh_) {
      if (sb_ == 1.0f) {
        next = 1.0f;
      } else {
        next = static_cast<float>(std::pow(prevmax_ / thresh_, sb_ - 1.0f));
        const float absx = std::fabs(next);
        next = absx < 1e-15f ? 0.0f : (absx > 1e15f ? 1.0f : next);
      }
    } else {
      if (sa_ == 1.0f) {
        next = 1.0f;
      } else {
        next = static_cast<float>(std::pow(prevmax_ / thresh_, sa_ - 1.0f));
      }
    }
    return (next - gain_) * (1.0f / 64.0f);  // CALCSLOPE, ar
  }
  float thresh_ = 0.0f, sb_ = 1.0f, sa_ = 1.0f;
  float clampcoef_ = 0.0f, relaxcoef_ = 0.0f;
  float clamp_ = 0.0f, relax_ = 0.0f;
  float prevmax_ = 0.0f, gain_ = 0.0f;
};

// Limiter — FilterUGens.cpp:3394 (ctor), 3421 (next). Lookahead ring layout.
struct Limiter {
  void Init(double sr, float amp, float dur, float first_in) {
    bufsize_ = static_cast<long>(std::ceil(static_cast<double>(dur) * sr));
    if (bufsize_ < 1) bufsize_ = 1;
    table_.assign(static_cast<size_t>(bufsize_) * 3, 0.0f);
    in_ = table_.data();
    mid_ = in_ + bufsize_;
    out_ = mid_ + bufsize_;
    amp_ = amp;
    pos_ = 0;
    flips_ = 0;
    slope_ = 0.0f;
    level_ = 1.0f;
    prevmax_ = 0.0f;
    curmax_ = 0.0f;
    slopefactor_ = 1.0f / static_cast<float>(bufsize_);
    float unused;
    Step(first_in, unused);  // ctor's Limiter_next(unit, 1)
  }
  void ProcessBlock(const float* in, float* out, int n) {
    for (int i = 0; i < n; ++i) Step(in[i], out[i]);
  }
  float ctor_value() const { return 0.0f; }

 private:
  void Step(float x, float& out) {
    in_[pos_] = x;
    if (flips_ >= 2) {
      out = level_ * out_[pos_];
    } else {
      out = 0.0f;
    }
    level_ += slope_;
    const float ax = std::fabs(x);
    if (ax > curmax_) curmax_ = ax;

    if (++pos_ >= bufsize_) {
      pos_ = 0;
      const float maxval2 = prevmax_ > curmax_ ? prevmax_ : curmax_;
      prevmax_ = curmax_;
      curmax_ = 0.0f;
      const float next = maxval2 > amp_ ? amp_ / maxval2 : 1.0f;
      slope_ = (next - level_) * slopefactor_;
      float* const temp = out_;
      out_ = mid_;
      mid_ = in_;
      in_ = temp;
      ++flips_;
    }
  }
  std::vector<float> table_;
  float* in_ = nullptr;
  float* mid_ = nullptr;
  float* out_ = nullptr;
  long bufsize_ = 0, pos_ = 0, flips_ = 0;
  float amp_ = 1.0f, slope_ = 0.0f, level_ = 1.0f;
  float prevmax_ = 0.0f, curmax_ = 0.0f, slopefactor_ = 0.0f;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_CORE_DYN_H_
