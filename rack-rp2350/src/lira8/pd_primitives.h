// Pd primitive layer for the rack-lira8 port — verbatim re-implementations
// of the vanilla objects the LIRA-8 patch uses. Every formula is sourced
// from pure-data d_ctl.c/d_osc.c/d_filter.c and probe-verified; see
// rack-rp2350/test/lira8_ref/PD-PRIMITIVES.md. Pure C++ (no MCU headers) —
// host-compilable is the ctag-tbd portability proof.
#ifndef RACK_SRC_LIRA8_PD_PRIMITIVES_H_
#define RACK_SRC_LIRA8_PD_PRIMITIVES_H_

#include <cstdint>
#include <cmath>

namespace l8pd {

static constexpr float kPi = 3.14159265358979f;

// ---------- control-rate helpers (pd exact constants) ----------
inline float dbtorms(float db) {
  // pd d_osc/m_pd: rms = 10^((db-100)/20); 100 dB = 1.0
  return std::pow(10.0f, (db - 100.0f) / 20.0f);
}
inline float mtof(float m) {
  if (m <= -1500.0f) return 0.0f;
  if (m > 1499.0f) m = 1499.0f;
  return 8.17579891564f * std::exp(0.0577622650f * m);
}
inline float ftom(float f) {
  return (f > 0.0f ? 17.3123405046f * std::log(0.12231220585f * f) : -1500.0f);
}
inline float powtodb(float p) {
  return (p <= 0.0f ? 0.0f : 100.0f + 20.0f * std::log10(p));
}

// ---------- line~: per-sample linear ramp ----------
class Ramp {
 public:
  explicit Ramp(float sr) : sr_(sr) {}
  // (target, ms): inc computed over the whole ramp; ms <= 0 jumps.
  void set(float target, float ms) {
    target_ = target;
    if (ms <= 0.0f) {
      value_ = target;
      inc_ = 0.0f;
      remaining_ = 0;
      return;
    }
    int n = static_cast<int>(ms * sr_ * 0.001f);
    if (n < 1) n = 1;
    inc_ = (target_ - value_) / static_cast<float>(n);
    remaining_ = n;
  }
  float next() {
    if (remaining_ > 0) {
      value_ += inc_;
      if (--remaining_ == 0) value_ = target_;
    }
    return value_;
  }
  float value() const { return value_; }

 private:
  float sr_;
  float value_ = 0.0f, target_ = 0.0f, inc_ = 0.0f;
  int remaining_ = 0;
};

// ---------- hip~ (d_filter.c verbatim) ----------
class Hip {
 public:
  Hip(float freq, float sr) { set_freq(freq, sr); }
  void set_freq(float freq, float sr) {
    if (freq <= 0.0f) {
      bypass_ = true;
      return;
    }
    bypass_ = false;
    coef_ = 1.0f - freq * (2.0f * kPi) / sr;
    if (coef_ < 0.0f) coef_ = 0.0f;
    else if (coef_ > 1.0f) coef_ = 1.0f;
    normal_ = 0.5f * (1.0f + coef_);
  }
  float process(float x) {
    if (bypass_) return x;
    float newv = x + coef_ * last_;
    float y = normal_ * (newv - last_);
    last_ = newv;
    return y;
  }

 private:
  float coef_ = 1.0f, normal_ = 1.0f, last_ = 0.0f;
  bool bypass_ = false;
};

// ---------- lop~ (d_filter.c verbatim) ----------
class Lop {
 public:
  Lop(float freq, float sr) { set_freq(freq, sr); }
  void set_freq(float freq, float sr) {
    coef_ = freq * (2.0f * kPi) / sr;
    if (coef_ > 1.0f) coef_ = 1.0f;
    else if (coef_ < 0.0f) coef_ = 0.0f;
  }
  float process(float x) {
    last_ = coef_ * x + (1.0f - coef_) * last_;
    return last_;
  }

 private:
  float coef_ = 1.0f, last_ = 0.0f;
};

// ---------- rpole~: y = coef*y + x (coef supplied as a signal) ----------
class Rpole {
 public:
  float process(float x, float coef) {
    last_ = coef * last_ + x;
    return last_;
  }
  void reset(float v = 0.0f) { last_ = v; }

 private:
  float last_ = 0.0f;
};

// ---------- osc~: 2048+1 cosine table, LINEAR interpolation ----------
static constexpr int kCosTabSize = 2048;

struct CosTable {
  float t[kCosTabSize + 1];
  CosTable() {
    for (int i = 0; i <= kCosTabSize; ++i)
      t[i] = std::cos(2.0f * kPi * i / static_cast<float>(kCosTabSize));
    t[0] = t[kCosTabSize] = 1.0f;                       // quadrant pins
    t[kCosTabSize / 4] = t[3 * kCosTabSize / 4] = 0.0f;
    t[kCosTabSize / 2] = -1.0f;
  }
};

inline float cos_lookup(float phase /* table units, wrapped [0, N) */) {
  static const CosTable tab;
  int idx = static_cast<int>(phase);
  if (idx >= kCosTabSize) idx -= kCosTabSize;
  float frac = phase - static_cast<float>(idx);
  float a = tab.t[idx], b = tab.t[idx + 1];
  return a + frac * (b - a);
}

// osc~: double-precision phase, conv = table/sr, linear interp
class CosOsc {
 public:
  explicit CosOsc(float sr) : conv_(static_cast<double>(kCosTabSize) / sr) {}
  void set_freq(float f) { inc_ = static_cast<double>(f) * conv_; }
  float next() {
    float out = cos_lookup(static_cast<float>(phase_));
    phase_ += inc_;
    while (phase_ >= kCosTabSize) phase_ -= kCosTabSize;
    while (phase_ < 0.0) phase_ += kCosTabSize;
    return out;
  }

 private:
  double conv_;
  double inc_ = 0.0;
  double phase_ = 0.0;
};

// ---------- phasor~: output-then-increment, double phase, wrap [0,1) ----------
class Phasor {
 public:
  explicit Phasor(float sr) : conv_(1.0 / sr) {}
  void set_freq(float f) { inc_ = static_cast<double>(f) * conv_; }
  float next() {
    float out = static_cast<float>(phase_);
    phase_ += inc_;
    if (phase_ >= 1.0) phase_ -= 1.0;
    if (phase_ < 0.0) phase_ += 1.0;
    return out;
  }

 private:
  double conv_;
  double inc_ = 0.0;
  double phase_ = 0.0;
};

// ---------- noise~: verbatim LCG; seed = 307*1319^k per instance ----------
class Noise {
 public:
  explicit Noise(uint32_t seed) : val_(seed) {}
  static uint32_t first_instance_seed() { return 307u * 1319u; }
  float next() {
    float out = static_cast<float>(
                    static_cast<int32_t>(val_ & 0x7fffffffu) - 0x40000000) *
                (1.0f / 1073741824.0f);
    val_ = val_ * 435898247u + 382842987u;
    return out;
  }

 private:
  uint32_t val_;
};

// ---------- env~ (d_ctl.c verbatim): Hann/N window, ring of partial
// sums, hop = npoints/2 rounded up to block size, output powtodb(sumbuf[0]).
// BLOCK-level like pd: call once per DSP block (64 samples).
class EnvRms {
 public:
  EnvRms(int npoints, int block, float sr) {
    (void)sr;
    npoints_ = npoints;
    realperiod_ = npoints / 2;
    if (realperiod_ % block)
      realperiod_ = realperiod_ + block - (realperiod_ % block);
    for (int i = 0; i < npoints_; ++i)
      win_[i] = (1.0f - std::cos(2.0f * kPi * i / static_cast<float>(npoints_))) /
                static_cast<float>(npoints_);
    for (int i = npoints_; i < npoints_ + 64; ++i) win_[i] = 0.0f;  // pd INITVSTAKEN
    for (int i = 0; i < kMaxOverlap; ++i) sumbuf_[i] = 0.0f;
    phase_ = 0;
  }
  // Returns true when env~ fired its output this block (value in *out_db).
  bool process_block(const float* x, int n, float* out_db) {
    int j = 0;
    for (int count = phase_; count < npoints_; count += realperiod_, ++j) {
      float sum = sumbuf_[j];
      for (int i = 0; i < n; ++i) sum += win_[count + i] * x[i] * x[i];
      sumbuf_[j] = sum;
    }
    sumbuf_[j] = 0.0f;                       // pd: sump[0] = 0 after the loop
    phase_ -= n;
    if (phase_ < 0) {
      result_ = sumbuf_[0];
      int k = 0;
      for (int count = realperiod_; count < npoints_; count += realperiod_, ++k)
        sumbuf_[k] = sumbuf_[k + 1];
      sumbuf_[k] = 0.0f;
      phase_ = realperiod_ - n;
      *out_db = powtodb(result_);
      return true;
    }
    return false;
  }

 private:
  static constexpr int kMaxOverlap = 4;
  int npoints_, realperiod_, phase_;
  float win_[1024 + 64];
  float sumbuf_[kMaxOverlap];
  float result_ = 0.0f;
};

// ---------- ma.tanh~: Pade tanh, input clipped +-3 ----------
class TanhShaper {
 public:
  float process(float x) {
    if (x > 3.0f) x = 3.0f;
    else if (x < -3.0f) x = -3.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
  }
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_PD_PRIMITIVES_H_
