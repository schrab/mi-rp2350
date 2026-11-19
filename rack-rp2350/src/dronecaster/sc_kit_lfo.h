// sc_kit_lfo.h — SC 3.14.1 low-frequency oscillators + line/clip/fold
// (GPL-3.0, CORE LFUGens.cpp / TriggerUGens.cpp).
//
// Anchors (Version-3.14.1):
//   LFUGens.cpp: LFSaw_next_k, LFSaw_Ctor (freqMul 2/sr, ctor rollback),
//   LFPar_next_k/_Ctor (4/sr), LFCub_next_k/_Ctor (2/sr, iphase+0.5),
//   LFPulse_next_k/_Ctor (1/sr, per-wrap duty update + 1-sample polarity
//   clamp), VarSaw_next_k/_Ctor (1/sr, duty clipped 0.001..0.999),
//   Line_next_loop/Line_next, XLine_next, Impulse_next_ik + the
//   Impulse_testWrapPhase law, Clip_next_*, Fold_next_*, DC.
//   TriggerUGens.cpp: Phasor_next_kk (wrap, reset edge).
//
// 3.14.1 law: every LF oscillator ctor primes one sample and then RESTORES
// the phase (the LFTri law from T-D1) — first rendered sample at iphase.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_LFO_H_
#define RACK_SRC_DRONECASTER_SC_KIT_LFO_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"
#include "sc_kit_noise.h"    // sc_wrap

namespace sc {

// LFSaw (LFUGens.cpp) — phase in [-1,1), wraps ±2.
struct LFSaw {
  void Init(double sr, float freq, float iphase = 0.0f) {
    freqmul_ = static_cast<float>(2.0 * (1.0 / sr));
    phase_ = iphase;
    freq_ = freq;
    ctor_ = Next();                            // ctor priming call
    phase_ = iphase;                           // 3.14 ctor rollback
  }
  void SetFreq(float f) { freq_ = f; }
  float Next() {
    const float out = static_cast<float>(phase_);
    phase_ += freq_ * freqmul_;
    if (freq_ * freqmul_ >= 0.0f) {
      if (phase_ >= 1.0) phase_ -= 2.0;
    } else {
      if (phase_ <= -1.0) phase_ += 2.0;
    }
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return ctor_; }
 private:
  double phase_ = 0.0;
  float freqmul_ = 0.0f, freq_ = 0.0f, ctor_ = 0.0f;
};

// LFPar (freqMul 4/sr): parabola 1-z^2 over the two half-cycles.
struct LFPar {
  void Init(double sr, float freq, float iphase = 0.0f) {
    freqmul_ = static_cast<float>(4.0 * (1.0 / sr));
    phase_ = iphase;
    freq_ = freq;
    ctor_ = Next();
    phase_ = iphase;
  }
  void SetFreq(float f) { freq_ = f; }
  float Next() {
    float out;
    if (phase_ < 1.0) {
      const float z = static_cast<float>(phase_);
      out = 1.0f - z * z;
    } else if (phase_ < 3.0) {
      const float z = static_cast<float>(phase_ - 2.0);
      out = z * z - 1.0f;
    } else {
      phase_ -= 4.0;
      const float z = static_cast<float>(phase_);
      out = 1.0f - z * z;
    }
    phase_ += freq_ * freqmul_;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return ctor_; }
 private:
  double phase_ = 0.0;
  float freqmul_ = 0.0f, freq_ = 0.0f, ctor_ = 0.0f;
};

// LFCub (freqMul 2/sr, iphase default 0.5): z^2(6-4z)-1 over [0,2).
struct LFCub {
  void Init(double sr, float freq, float iphase = 0.0f) {
    freqmul_ = static_cast<float>(2.0 * (1.0 / sr));
    // Source: mPhase = ZIN0(1) + 0.5 (the iphase ARG defaults to 0).
    const double ph = static_cast<double>(iphase) + 0.5;
    phase_ = ph;
    freq_ = freq;
    ctor_ = Next();
    phase_ = ph;                               // ctor rollback
  }
  void SetFreq(float f) { freq_ = f; }
  float Next() {
    float z;
    if (phase_ < 1.0) {
      z = static_cast<float>(phase_);
    } else if (phase_ < 2.0) {
      z = static_cast<float>(2.0 - phase_);
    } else {
      phase_ -= 2.0;
      z = static_cast<float>(phase_);
    }
    const float out = z * z * (6.0f - 4.0f * z) - 1.0f;
    phase_ += freq_ * freqmul_;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return ctor_; }
 private:
  double phase_ = 0.0;
  float freqmul_ = 0.0f, freq_ = 0.0f, ctor_ = 0.0f;
};

// LFPulse (freqMul 1/sr): phase [0,1), duty latched per wrap, and the
// wrap sample itself emits the OPPOSITE polarity for at least one sample.
struct LFPulse {
  void Init(double sr, float freq, float iphase, float duty) {
    freqmul_ = static_cast<float>(1.0 / sr);
    phase_ = static_cast<double>(iphase);
    duty_ = duty;
    next_duty_ = duty;                         // ctor: mDuty = ZIN0(2)
    freq_ = freq;
    ctor_ = Next();
    phase_ = static_cast<double>(iphase);      // ctor rollback
    duty_ = duty;
  }
  void SetFreq(float f) { freq_ = f; }
  void SetDuty(float d) { next_duty_ = d; }
  float Next() {
    float z;
    if (phase_ >= 1.0) {
      phase_ -= 1.0;
      duty_ = next_duty_;
      z = duty_ <= 0.5f ? 1.0f : 0.0f;         // opposite polarity sample
    } else {
      z = phase_ < duty_ ? 1.0f : 0.0f;
    }
    phase_ += freq_ * freqmul_;
    return z;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return ctor_; }
 private:
  double phase_ = 0.0;
  float freqmul_ = 0.0f, freq_ = 0.0f, ctor_ = 0.0f;
  float duty_ = 0.5f, next_duty_ = 0.5f;
};

// VarSaw (freqMul 1/sr, duty clipped 0.001..0.999 per wrap).
struct VarSaw {
  void Init(double sr, float freq, float iphase, float duty) {
    freqmul_ = static_cast<float>(1.0 / sr);
    const double ph = sc_wrap(static_cast<float>(iphase), 0.0f, 1.0f);
    phase_ = ph;
    next_duty_ = clampf(duty, 0.001f, 0.999f);
    duty_ = next_duty_;
    inv_duty_ = 2.0f / duty_;
    inv1_duty_ = 2.0f / (1.0f - duty_);
    freq_ = freq;
    ctor_ = Next();
    phase_ = ph;                               // ctor rollback
  }
  void SetFreq(float f) { freq_ = f; }
  void SetDuty(float d) { next_duty_ = d; }
  float Next() {
    if (phase_ >= 1.0) {
      phase_ -= 1.0;
      duty_ = clampf(next_duty_, 0.001f, 0.999f);
      inv_duty_ = 2.0f / duty_;
      inv1_duty_ = 2.0f / (1.0f - duty_);
    }
    const float p = static_cast<float>(phase_);
    const float z = p < duty_ ? p * inv_duty_ : (1.0f - p) * inv1_duty_;
    phase_ += freq_ * freqmul_;
    return z - 1.0f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return ctor_; }
 private:
  double phase_ = 0.0;
  float freqmul_ = 0.0f, freq_ = 0.0f, ctor_ = 0.0f;
  float duty_ = 0.5f, next_duty_ = 0.5f, inv_duty_ = 4.0f, inv1_duty_ = 4.0f;
};

// Impulse (LFUGens Impulse_testWrapPhase law): phase wraps [0,1); the wrap
// sample outputs 1. Ctor: phase = wrap(offset); offset 0 + positive freq
// starts the phase AT the wrap point so the first sample fires.
struct Impulse {
  void Init(double sr, float freq) {
    freqmul_ = static_cast<float>(1.0 / sr);
    inc_ = static_cast<float>(freq * freqmul_);   // mPhaseIncrement = float product
    double p = 0.0;
    if (p == 0.0 && inc_ >= 0.0) p = 1.0;
    phase_ = p;
    primed_ = Next();
    phase_ = p;                                // ctor rollback (no reprime)
  }
  void SetFreq(float f) { inc_ = static_cast<float>(f * freqmul_); }
  float Next() {
    float out = 0.0f;
    if (inc_ < 0.0f) {
      if (phase_ <= 0.0) {
        phase_ += 1.0;
        if (phase_ <= 0.0) phase_ -= std::ceil(static_cast<double>(phase_));
        out = 1.0f;
      }
    } else {
      if (phase_ >= 1.0) {
        phase_ -= 1.0;
        if (phase_ >= 1.0) phase_ -= std::floor(static_cast<double>(phase_));
        out = 1.0f;
      }
    }
    phase_ += inc_;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return primed_; }
 private:
  double phase_ = 0.0, inc_ = 0.0;
  float freqmul_ = 0.0f, primed_ = 0.0f;
};

// sc_wrap on doubles (SC_InlineUnaryOp.h: only two corrections, like the
// float law but at double precision).
inline double WrapDouble(double x, double lo, double hi) {
  const double range = hi - lo;
  if (x >= hi) { x -= range; if (x >= hi) x -= range; }
  else if (x < lo) { x += range; if (x < lo) x += range; }
  return x;
}

// Phasor (TriggerUGens Phasor_next_kk): wrap [start,end), rising gate
// resets to resetPos.
struct Phasor {
  void Init(double sr, double rate, double start, double end, float reset_pos) {
    sr_ = sr;
    rate_ = rate;
    start_ = start;
    end_ = end;
    reset_ = reset_pos;
    level_ = start;
    prev_in_ = 0.0f;
  }
  float Next(float in) {
    if (prev_in_ <= 0.0f && in > 0.0f) level_ = reset_;
    // Source: sc_wrap on DOUBLES (level/start/end), not a float cast.
    level_ = WrapDouble(level_, start_, end_);
    const float out = static_cast<float>(level_);
    level_ += rate_;
    prev_in_ = in;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next(0.0f);
  }
  float ProcessKr() { return Next(0.0f); }
  // Phasor's ctor never primes: the kr output slot K2A reads at ctor time is
  // the graph's zeroed buffer.
  float ctor_value() const { return 0.0f; }
 private:
  double sr_ = 48000.0, rate_ = 1.0, start_ = 0.0, end_ = 1.0, level_ = 0.0;
  float prev_in_ = 0.0f, reset_ = 0.0f;
};

// DC.kr — constant (LFUGens DC): output = input.
// A2K — returns the first sample of the block (LFUGens A2K_next).
// Clip / Fold (LFUGens Clip_next_kk / Fold_next_kk).
inline float ClipTo(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
inline float FoldTo(float x, float lo, float hi) {
  // Fold_next_kk law: x beyond [lo,hi] reflects around the limits.
  const float range = hi - lo;
  if (range <= 0.0f) return lo;
  float v = x - lo;
  const float n = 2.0f * range;
  v = fmodf(v, n);
  if (v < 0.0f) v += n;
  if (v > range) v = n - v;
  return v + lo;
}

}  // namespace sc
#endif
