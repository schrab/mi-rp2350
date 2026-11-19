// sc_kit_filter.h — SC 3.14.1 core filters (GPL-3.0, CORE FilterUGens.cpp).
// One-piece ports of the 2-pole/SOS Butterworth pair (LPF/HPF), the
// bandwidth filter (BPF), the resonant pair (RLPF/RHPF), and the first-order
// family (OnePole/Integrator/LeakDC/Slew/Lag/LagUD).
//
// Anchors (Version-3.14.1): LPF_next/LPF_next_1 (FilterUGens.cpp), HPF,
// BPF, RLPF, RHPF, OnePole_next_k, Integrator_next, LeakDC_next, Slew_next,
// Lag_next, LagUD_next. Coefficient laws:
//   LPF: C = 1/tan(pfreq/2), sqrt2C = C*sqrt2, a0 = 1/(1+sqrt2C+C^2),
//        b1 = -2(1-C^2)a0, b2 = -(1-sqrt2C+C^2)a0; y0 = in+b1 y1+b2 y2;
//        out = a0*(y0 + 2 y1 + y2).
//   HPF: C = tan(pfreq/2), same a0/b1/b2 with +2(1-C^2); out = a0*(y0 - 2y1 + y2).
//   BPF: pfreq = 2pi f/sr; pbw = bw*pfreq/2; C = 1/tan(pbw); D = 2cos(pfreq);
//        a0 = 1/(1+C); b1 = C*D*a0; b2 = (1-C)*a0; out = a0*(y0 - y2).
//   RLPF/RHPF: D = tan(pfreq*qres/2); C = (1-D)/(1+D); cosf = cos(pfreq);
//        b1 = (1+C)cosf; b2 = -C; a0 = (1+C -+ b1)/4; y0 = a0 in + b1 y1 + b2 y2;
//        out = y0 +- 2y1 + y2.
// On a parameter change the coefficients are interpolated ACROSS the block
// (mFilterSlope = 1/mFilterLoops; one step per 4-sample group) — replicated
// exactly. Constant-parameter blocks use fixed coefficients.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_FILTER_H_
#define RACK_SRC_DRONECASTER_SC_KIT_FILTER_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"

namespace sc {

// 2-pole/SOS filters: ProcessBlock(in, out, n, freq[, q]) with the source's
// coefficient-change interpolation. `n` must be a multiple of 4 for the
// group-slope law (the rack calls them with kBlock = 64).
struct Filter2Pole {
  // kind: 0 LPF, 1 HPF, 2 BPF, 3 RLPF, 4 RHPF
  void Init(double sr, int kind) {
    sr_ = sr;
    kind_ = kind;
    a0_ = b1_ = b2_ = 0.0;
    y1_ = y2_ = 0.0;
    freq_ = 0.0f;
    q_ = 0.0f;
    // ctor: m_freq = uninitializedControl (NaN) so the first block recomputes
    have_freq_ = false;
  }
  void ProcessBlock(const float* in, float* out, int n, float freq,
                    float q = 0.0f) {
    const double rps = 2.0 * M_PI / sr_;
    // The ctor's next_1(unit,1) consumes the FIRST input sample and leaves
    // the coefficients computed for this freq; the block then runs the
    // UNCHANGED branch (source: ctor sets m_freq).
    if (!primed_) {
      ComputeCoefs(freq, q, rps, &a0_, &b1_, &b2_);
      float scratch;
      Step(in[0], scratch, a0_, b1_, b2_, y1_, y2_);
      primed_ = true;
      have_freq_ = true;
      freq_ = freq;
      q_ = q;
    }
    const bool changed = freq != freq_ || q != q_;
    double a0 = a0_, b1 = b1_, b2 = b2_;
    double y1 = y1_, y2 = y2_;
    double na0 = a0, nb1 = b1, nb2 = b2;
    double sa0 = 0.0, sb1 = 0.0, sb2 = 0.0;
    if (changed) {
      ComputeCoefs(freq, q, rps, &na0, &nb1, &nb2);
      const double kSlope = 1.0 / static_cast<double>(n / 4);
      sa0 = (na0 - a0) * kSlope;
      sb1 = (nb1 - b1) * kSlope;
      sb2 = (nb2 - b2) * kSlope;
      int i = 0;
      for (; i + 4 <= n; i += 4) {
        Step(in[i + 0], out[i + 0], a0, b1, b2, y1, y2);
        Step(in[i + 1], out[i + 1], a0, b1, b2, y1, y2);
        Step(in[i + 2], out[i + 2], a0, b1, b2, y1, y2);
        Step(in[i + 3], out[i + 3], a0, b1, b2, y1, y2);
        a0 += sa0; b1 += sb1; b2 += sb2;
      }
      for (; i < n; ++i) Step(in[i], out[i], a0, b1, b2, y1, y2);
    } else {
      for (int i = 0; i < n; ++i) Step(in[i], out[i], a0, b1, b2, y1, y2);
    }
    y1_ = ZapGremlins(y1);
    y2_ = ZapGremlins(y2);
    if (changed) { a0_ = na0; b1_ = nb1; b2_ = nb2; }
    freq_ = freq;
    q_ = q;
    have_freq_ = true;
  }
  // The ctor genrates one sample per the source (m_a0..b2 = 0, m_freq NaN)
  // — the state stays zeroed here, which matches next_1's in=ZIN0(0) only
  // when the first input is 0; drones call with their own first sample.

 private:
  void ComputeCoefs(float freq, float q, double rps, double* a0, double* b1,
                    double* b2) const {
    const double sqrt2 = 1.4142135623730951;
    if (kind_ <= 1) {                      // LPF / HPF
      const double pfreq = static_cast<double>(freq) * rps * 0.5;
      double C, C2, sqrt2C;
      if (kind_ == 0) { C = 1.0 / std::tan(pfreq); }
      else { C = std::tan(pfreq); }
      C2 = C * C;
      sqrt2C = C * sqrt2;
      *a0 = 1.0 / (1.0 + sqrt2C + C2);
      *b1 = (kind_ == 0 ? -2.0 : 2.0) * (1.0 - C2) * (*a0);
      *b2 = -(1.0 - sqrt2C + C2) * (*a0);
    } else if (kind_ == 2) {               // BPF
      const double pfreq = static_cast<double>(freq) * rps;
      const double pbw = static_cast<double>(q) * pfreq * 0.5;
      const double C = 1.0 / std::tan(pbw);
      const double D = 2.0 * std::cos(pfreq);
      *a0 = 1.0 / (1.0 + C);
      *b1 = C * D * (*a0);
      *b2 = (1.0 - C) * (*a0);
    } else {                               // RLPF / RHPF
      const double qres = q < 0.001f ? 0.001f : static_cast<double>(q);
      const double pfreq = static_cast<double>(freq) * rps;
      const double D = std::tan(pfreq * qres * 0.5);
      const double C = (1.0 - D) / (1.0 + D);
      const double cosf = std::cos(pfreq);
      *b1 = (1.0 + C) * cosf;
      *b2 = -C;
      *a0 = (1.0 + C + (kind_ == 4 ? *b1 : -*b1)) * 0.25;
    }
  }
  void Step(float in, float& out, double a0, double b1, double b2,
            double& y1, double& y2) const {
    if (kind_ <= 1) {                      // LPF / HPF
      const double y0 = in + b1 * y1 + b2 * y2;
      out = static_cast<float>(a0 * (y0 + (kind_ == 0 ? 2.0 : -2.0) * y1 + y2));
      y2 = y1;
      y1 = y0;
    } else if (kind_ == 2) {               // BPF
      const double y0 = in + b1 * y1 + b2 * y2;
      out = static_cast<float>(a0 * (y0 - y2));
      y2 = y1;
      y1 = y0;
    } else {                               // RLPF / RHPF
      const double y0 = a0 * in + b1 * y1 + b2 * y2;
      out = static_cast<float>(y0 + (kind_ == 3 ? 2.0 : -2.0) * y1 + y2);
      y2 = y1;
      y1 = y0;
    }
  }
  static double ZapGremlins(double x) {
    // SC_InlineUnaryOp zapgremlins: NaN/inf/huge -> 0
    return (x < 3.4e38 && x > -3.4e38) ? x : 0.0;
  }
  double sr_ = 48000.0;
  int kind_ = 0;
  double a0_ = 0.0, b1_ = 0.0, b2_ = 0.0, y1_ = 0.0, y2_ = 0.0;
  float freq_ = 0.0f, q_ = 0.0f;
  bool have_freq_ = false;
  bool primed_ = false;
};

// OnePole (FilterUGens OnePole_next_k): y = in + b1*(y1-in) for b1 >= 0,
// y = in + b1*(y1+in) for b1 < 0; on a coefficient change the OLD b1 is used
// for the block and ramps to the new value (CALCSLOPE); mixed-sign blocks use
// y = (1-|b1|)*in + b1*y1. The ctor stores the input's b1 and primes once.
struct OnePole {
  void Init(double sr) { sr_ = sr; b1_ = 0.0; y1_ = 0.0; have_ = false; }
  void ProcessBlock(const float* in, float* out, int n, float b1_in) {
    if (!have_) {
      // Ctor: m_b1 = b1 input, m_y1 = 0, then one priming sample.
      b1_ = b1_in;
      y1_ = (b1_in >= 0.0f) ? (in[0] + b1_in * (0.0 - in[0]))
                            : (in[0] + b1_in * (0.0 + in[0]));
      have_ = true;
    }
    const double b1_old = b1_;
    if (b1_old == static_cast<double>(b1_in)) {
      if (b1_old >= 0.0) {
        for (int i = 0; i < n; ++i) {
          const double y0 = in[i];
          y1_ = y0 + b1_old * (y1_ - y0);
          out[i] = static_cast<float>(y1_);
        }
      } else {
        for (int i = 0; i < n; ++i) {
          const double y0 = in[i];
          y1_ = y0 + b1_old * (y1_ + y0);
          out[i] = static_cast<float>(y1_);
        }
      }
    } else {
      const double slope = (static_cast<double>(b1_in) - b1_old) *
                           (1.0 / static_cast<double>(n));
      double b = b1_old;
      for (int i = 0; i < n; ++i) {
        const double y0 = in[i];
        if (b1_in >= 0.0f && b1_old >= 0.0) y1_ = y0 + b * (y1_ - y0);
        else if (b1_in <= 0.0f && b1_old <= 0.0) y1_ = y0 + b * (y1_ + y0);
        else y1_ = (1.0 - std::fabs(b)) * y0 + b * y1_;
        out[i] = static_cast<float>(y1_);
        b += slope;
      }
      b1_ = b1_in;
    }
    y1_ = ZapGremlins(y1_);
  }
 private:
  static double ZapGremlins(double x) {
    return (x < 3.4e38 && x > -3.4e38) ? x : 0.0;
  }
  double sr_ = 48000.0, b1_ = 0.0, y1_ = 0.0;
  bool have_ = false;
};

// Integrator (FilterUGens Integrator_next): y = in + b1*y1 (b1==1/0 fast paths).
struct Integrator {
  void Init() { b1_ = 0.0; y1_ = 0.0; primed_ = false; }
  void Prime(float in0, float b1) {
    // Ctor: m_b1 = coef, m_y1 = 0 (out = in0 with y1 empty); y1 stays 0.
    (void)in0; (void)b1;
    y1_ = 0.0;
    primed_ = true;
  }
  float Next(float in, float b1) {
    const double y0 = in;
    y1_ = (b1 == 1.0f) ? (y0 + y1_) : (b1 == 0.0f ? y0 : (y0 + b1 * y1_));
    return static_cast<float>(y1_);
  }
 private:
  double b1_ = 0.0, y1_ = 0.0;
  bool primed_ = false;
};

// LeakDC (FilterUGens LeakDC_next): y = x0 - x1 + b1*y1; x1 = x0.
struct LeakDC {
  void Init() { b1_ = 0.995; y1_ = 0.0; x1_ = 0.0; primed_ = false; }
  void Prime(float in0) {
    // Ctor: m_x1 = ZIN0(0), m_y1 = 0, one priming sample (x0 == x1 -> y=0).
    x1_ = in0;
    y1_ = 0.0;
    primed_ = true;
  }
  float Next(float in, float b1) {
    const double x0 = in;
    y1_ = x0 - x1_ + b1 * y1_;
    x1_ = x0;
    return static_cast<float>(y1_);
  }
 private:
  double b1_ = 0.995, y1_ = 0.0, x1_ = 0.0;
  bool primed_ = false;
};

// Slew (FilterUGens Slew_next): clamp the per-sample slope to [-dn, +up]
// (seconds-per-unit scaled by sampleDur).
struct Slew {
  void Init(double sr) { sample_dur_ = 1.0 / sr; level_ = 0.0; primed_ = false; }
  void Prime(float in0) { level_ = in0; primed_ = true; }
  float Next(float in, float up, float dn) {
    const float upf = up * static_cast<float>(sample_dur_);
    const float dnf = 0.0f - dn * static_cast<float>(sample_dur_);
    const double slope = in - level_;
    level_ += slope < dnf ? dnf : (slope > upf ? upf : slope);
    return static_cast<float>(level_);
  }
 private:
  double sample_dur_ = 0.0, level_ = 0.0;
  bool primed_ = false;
};

// Lag (FilterUGens Lag_next): one-pole with b1 = exp(log(0.001)/(lag*sr)).
struct Lag {
  void Init(double sr) { sr_ = sr; y1_ = 0.0; b1_ = 0.0; lag_ = 0.0f; primed_ = false; }
  void Prime(float in0) { y1_ = in0; primed_ = true; }
  float Next(float in, float lag) {
    if (lag != lag_) {
      b1_ = lag == 0.0f
                ? 0.0
                : std::exp(std::log(0.001) / (static_cast<double>(lag) * sr_));
      lag_ = lag;
    }
    const double y0 = in;
    y1_ = y0 + b1_ * (y1_ - y0);
    return static_cast<float>(y1_);
  }
 private:
  double sr_ = 48000.0, y1_ = 0.0, b1_ = 0.0;
  float lag_ = 0.0f;
  bool primed_ = false;
};

// LagUD (FilterUGens LagUD_next): separate up/down lags.
struct LagUD {
  void Init(double sr) {
    sr_ = sr; y1_ = 0.0; b1u_ = 0.0; b1d_ = 0.0; lagu_ = 0.0f; lagd_ = 0.0f;
    primed_ = false;
  }
  void Prime(float in0) { y1_ = in0; primed_ = true; }
  float Next(float in, float up, float dn) {
    if (up != lagu_) {
      b1u_ = up == 0.0f
                 ? 0.0
                 : std::exp(std::log(0.001) / (static_cast<double>(up) * sr_));
      lagu_ = up;
    }
    if (dn != lagd_) {
      b1d_ = dn == 0.0f
                 ? 0.0
                 : std::exp(std::log(0.001) / (static_cast<double>(dn) * sr_));
      lagd_ = dn;
    }
    const double y0 = in;
    y1_ = (y0 > y1_) ? (y0 + b1u_ * (y1_ - y0)) : (y0 + b1d_ * (y1_ - y0));
    return static_cast<float>(y1_);
  }
 private:
  double sr_ = 48000.0, y1_ = 0.0, b1u_ = 0.0, b1d_ = 0.0;
  float lagu_ = 0.0f, lagd_ = 0.0f;
  bool primed_ = false;
};

// MoogFF (FilterUGens.cpp:4064 MoogFF_Ctor / 4079 MoogFF_next): 4-state
// Moog ladder. Reset, freq-coefficient recompute and the k fixed-vs-ramp
// decision are once per BLOCK; the four one-pole stages run per sample.
// Ctor: m_freq = uninitializedControl (forces coef compute), m_k = RAW gain
// (unclipped), states 0, then one priming sample via MoogFF_next(unit, 1).
struct MoogFF {
  void Init(double sr, float freq, float gain, float reset, float first_in) {
    sr_ = sr;
    have_freq_ = false;
    k_ = gain;  // source stores IN0(2) unclipped; next() compares to clipped
    s1_ = s2_ = s3_ = s4_ = 0.0;
    float unused;
    ProcessBlock(&first_in, &unused, 1, freq, gain, reset);
  }
  void ProcessBlock(const float* in, float* out, int n, float freq, float gain,
                    float reset) {
    const float k = gain < 0.0f ? 0.0f : (gain > 4.0f ? 4.0f : gain);  // sc_clip
    if (reset > 0.0f) s1_ = s2_ = s3_ = s4_ = 0.0;
    if (!have_freq_ || freq_ != freq) {
      const double T = 1.0 / sr_;
      // MoogFF_next uses FilterUGens.cpp:29's float PI macro (3.1415926535898f
      // promoted to double), NOT SC_Constants' double pi — match it exactly.
      const double kSCFloatPi = static_cast<double>(3.1415926535898f);
      double wcD = 2.0 * std::tan(T * kSCFloatPi * static_cast<double>(freq)) * sr_;
      if (wcD < 0) wcD = 0;
      const double TwcD = T * wcD;
      b0_ = TwcD / (TwcD + 2.0);
      a1_ = (TwcD - 2.0) / (TwcD + 2.0);
      freq_ = freq;
      have_freq_ = true;
    }
    const bool k_changed = (k_ != k);
    float kcur = k, kslope = 0.0f;
    if (k_changed) {
      kslope = (k - k_) * (1.0f / 64.0f);  // CALCSLOPE, ar
      kcur = k_;
    }
    const double b04 = b0_ * b0_ * b0_ * b0_;
    for (int i = 0; i < n; ++i) {
      const double o = s4_ + b0_ * (s3_ + b0_ * (s2_ + b0_ * s1_));
      const double ins = in[i];
      const double outs = (b04 * ins + o) /
                          (1.0 + b04 * static_cast<double>(kcur));
      out[i] = static_cast<float>(outs);
      const double u = ins - static_cast<double>(kcur) * outs;
      double past = u;
      double future = b0_ * past + s1_;  s1_ = b0_ * past - a1_ * future;
      past = future; future = b0_ * past + s2_; s2_ = b0_ * past - a1_ * future;
      past = future; future = b0_ * past + s3_; s3_ = b0_ * past - a1_ * future;
      s4_ = b0_ * future - a1_ * outs;
      if (k_changed) kcur += kslope;
    }
    if (k_changed) k_ = k;
    s1_ = ZapGremlins(s1_); s2_ = ZapGremlins(s2_);
    s3_ = ZapGremlins(s3_); s4_ = ZapGremlins(s4_);
  }

 private:
  static double ZapGremlins(double x) {
    return (x < 3.4e38 && x > -3.4e38) ? x : 0.0;
  }
  double sr_ = 48000.0, b0_ = 0.0, a1_ = 0.0;
  double s1_ = 0.0, s2_ = 0.0, s3_ = 0.0, s4_ = 0.0;
  float freq_ = 0.0f, k_ = 0.0f;
  bool have_freq_ = false;
};

// BEQSuite (FilterUGens.cpp:4167 BPerformFilterLoop; Ctors at 4209 BLowPass,
// 4377 BHiPass, 4824 BPeakEQ, 4972 BAllPass, 5112 BLowShelf, 5272 BHiShelf).
// All six share one direct-form biquad; only the coefficient law differs.
// The ctor primes ONE sample with PUSH_LOOPVALS (loops=0, remain=1), matching
// the source. On a param change coefficients interpolate one step per 3-sample
// group (mFilterLoops = n/3, mFilterSlope = 1/mFilterLoops) — NOT the /4 law
// of Filter2Pole above. w0 = twopi*f/sr uses SC_Constants' double pi.
struct BEQFilter {
  // 0 BLowPass, 1 BHiPass, 2 BPeakEQ, 3 BAllPass, 4 BLowShelf, 5 BHiShelf.
  // p2 = rq (LP/HP/AP/Peak) or rs (shelves); p3 = db (Peak/shelves).
  void Init(double sr, int kind, float freq, float p2, float p3,
            float first_in) {
    sr_ = sr;
    kind_ = kind;
    Compute(freq, p2, p3, &a0_, &a1_, &a2_, &b1_, &b2_);
    freq_ = freq; p2_ = p2; p3_ = p3;
    y1_ = y2_ = 0.0;
    float unused;
    Run(first_in, unused);
  }
  void ProcessBlock(const float* in, float* out, int n, float freq, float p2,
                    float p3) {
    const bool changed = freq != freq_ || p2 != p2_ ||
                         ((kind_ == 2 || kind_ >= 4) && p3 != p3_);
    if (changed) {
      double na0, na1, na2, nb1, nb2;
      Compute(freq, p2, p3, &na0, &na1, &na2, &nb1, &nb2);
      const double kSlope = 1.0 / static_cast<double>(n / 3);
      RunBlock(in, out, n, a0_, a1_, a2_, b1_, b2_,
               (na0 - a0_) * kSlope, (na1 - a1_) * kSlope,
               (na2 - a2_) * kSlope, (nb1 - b1_) * kSlope,
               (nb2 - b2_) * kSlope);
      a0_ = na0; a1_ = na1; a2_ = na2; b1_ = nb1; b2_ = nb2;
      freq_ = freq; p2_ = p2; p3_ = p3;
    } else {
      RunBlock(in, out, n, a0_, a1_, a2_, b1_, b2_, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    y1_ = ZapGremlins(y1_);
    y2_ = ZapGremlins(y2_);
  }

 private:
  void Compute(float freq, float p2, float p3, double* a0, double* a1,
               double* a2, double* b1, double* b2) const {
    const double twopi = 2.0 * M_PI;
    const double w0 = twopi * static_cast<double>(freq) / sr_;
    const double cosw0 = std::cos(w0);
    if (kind_ <= 1) {                       // BLowPass / BHiPass
      const double i = kind_ == 0 ? 1.0 - cosw0 : 1.0 + cosw0;
      const double alpha = std::sin(w0) * 0.5 * static_cast<double>(p2);
      const double b0rz = 1.0 / (1.0 + alpha);
      *a0 = i * 0.5 * b0rz;
      *a1 = (kind_ == 0 ? i : -i) * b0rz;
      *a2 = *a0;
      *b1 = cosw0 * 2.0 * b0rz;
      *b2 = (1.0 - alpha) * -b0rz;
    } else if (kind_ == 2) {                // BPeakEQ
      const double a = std::pow(10.0, static_cast<double>(p3) * 0.025);
      const double alpha = std::sin(w0) * 0.5 * static_cast<double>(p2);
      const double b0rz = 1.0 / (1.0 + alpha / a);
      *b1 = 2.0 * b0rz * cosw0;
      *a0 = (1.0 + alpha * a) * b0rz;
      *a1 = -*b1;
      *a2 = (1.0 - alpha * a) * b0rz;
      *b2 = (1.0 - alpha / a) * -b0rz;
    } else if (kind_ == 3) {                // BAllPass
      const double alpha = std::sin(w0) * 0.5 * static_cast<double>(p2);
      const double b0rz = 1.0 / (1.0 + alpha);
      *a0 = (1.0 - alpha) * b0rz;
      *b1 = 2.0 * b0rz * cosw0;
      *a1 = -*b1;
      *a2 = 1.0;
      *b2 = -*a0;
    } else {                                // BLowShelf / BHiShelf
      const double a = std::pow(10.0, static_cast<double>(p3) * 0.025);
      const double sinw0 = std::sin(w0);
      const double alpha = sinw0 * 0.5 *
          std::sqrt((a + 1.0 / a) * (static_cast<double>(p2) - 1.0) + 2.0);
      const double i = (a + 1.0) * cosw0;
      const double j = (a - 1.0) * cosw0;
      const double k = 2.0 * std::sqrt(a) * alpha;
      const double b0rz = 1.0 / ((a + 1.0) + (kind_ == 4 ? j : -j) + k);
      *a0 = a * ((a + 1.0) - (kind_ == 4 ? j : -j) + k) * b0rz;
      *a1 = (kind_ == 4 ? 1.0 : -1.0) * 2.0 * a *
            ((a - 1.0) + (kind_ == 4 ? -i : i)) * b0rz;
      *a2 = a * ((a + 1.0) - (kind_ == 4 ? j : -j) - k) * b0rz;
      *b1 = (kind_ == 4 ? 1.0 : -1.0) * 2.0 *
            ((a - 1.0) + (kind_ == 4 ? i : -i)) * b0rz;
      *b2 = ((a + 1.0) + (kind_ == 4 ? j : -j) - k) * -b0rz;
    }
  }
  // Source's PUSH_LOOPVALS priming path: one sample, fixed coefficients.
  void Run(float x, float& out) {
    const double y0 = x + b1_ * y1_ + b2_ * y2_;
    out = static_cast<float>(a0_ * y0 + a1_ * y1_ + a2_ * y2_);
    y2_ = y1_;
    y1_ = y0;
  }
  // Source BPerformFilterLoop: 3-sample groups, coefficients step once per
  // group; the remainder uses the final coefficients.
  void RunBlock(const float* in, float* out, int n, double a0, double a1,
                double a2, double b1, double b2, double sa0, double sa1,
                double sa2, double sb1, double sb2) {
    int i = 0;
    for (int groups = n / 3; groups > 0; --groups) {
      const double y0 = in[i + 0] + b1 * y1_ + b2 * y2_;
      out[i + 0] = static_cast<float>(a0 * y0 + a1 * y1_ + a2 * y2_);
      const double y1 = in[i + 1] + b1 * y0 + b2 * y1_;
      out[i + 1] = static_cast<float>(a0 * y1 + a1 * y0 + a2 * y1_);
      const double y2 = in[i + 2] + b1 * y1 + b2 * y0;
      out[i + 2] = static_cast<float>(a0 * y2 + a1 * y1 + a2 * y0);
      y2_ = y1;   // previous output
      y1_ = y2;   // newest output
      i += 3;
      a0 += sa0; a1 += sa1; a2 += sa2; b1 += sb1; b2 += sb2;
    }
    for (; i < n; ++i) {
      const double y0 = in[i] + b1 * y1_ + b2 * y2_;
      out[i] = static_cast<float>(a0 * y0 + a1 * y1_ + a2 * y2_);
      y2_ = y1_;
      y1_ = y0;
    }
  }
  static double ZapGremlins(double x) {
    return (x < 3.4e38 && x > -3.4e38) ? x : 0.0;
  }
  double sr_ = 48000.0;
  int kind_ = 0;
  double a0_ = 0.0, a1_ = 0.0, a2_ = 0.0, b1_ = 0.0, b2_ = 0.0;
  double y1_ = 0.0, y2_ = 0.0;
  float freq_ = 0.0f, p2_ = 0.0f, p3_ = 0.0f;
};

}  // namespace sc
#endif
