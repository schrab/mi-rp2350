// sc_kit_chaos.h — SC 3.14.1 chaos UGens (GPL-3.0, ChaosUGens.cpp).
// LorenzL 1301/1403 (RK4 integration + linear interp), HenonC 800/880,
// FBSineC 437/508 (3rd-order ipol3 interpolation). All integrations are
// double; the output interpolation frac and counter are float. Each ctor
// primes one sample via next(unit, 1).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_CHAOS_H_
#define RACK_SRC_DRONECASTER_SC_KIT_CHAOS_H_
#include <cmath>
#include <cstdint>

namespace sc {

static inline void IPol3Coef(double xnm3, double xnm2, double xnm1, double xn,
                             double* c0, double* c1, double* c2, double* c3) {
  *c0 = xnm2;
  *c1 = 0.5f * (xnm1 - xnm3);
  *c2 = xnm3 - (2.5f * xnm2) + xnm1 + xnm1 - 0.5f * xn;
  *c3 = 0.5f * (xn - xnm3) + 1.5f * (xnm2 - xnm1);
}
static inline double IPol3(float frac, double c0, double c1, double c2,
                           double c3) {
  return ((c3 * frac + c2) * frac + c1) * frac + c0;
}
static inline double ChaosMod2Pi(double in) {
  const double hi = 6.283185307179586476925286766559;
  if (in >= hi) {
    in -= hi;
    if (in < hi) return in;
  } else if (in < 0.) {
    in += hi;
    if (in >= 0.) return in;
  } else return in;
  return in - hi * static_cast<double>(static_cast<int>(in / hi));
}
static inline void ChaosCycle(float freq, double sr, float* spc, double* slope) {
  if (freq < static_cast<float>(sr)) {
    const double dspc = sr / static_cast<double>(freq < 0.001f ? 0.001f : freq);
    *spc = static_cast<float>(dspc);
    const float fslope = 1.f / *spc;   // source: 1.f / samplesPerCycle (float)
    *slope = static_cast<double>(fslope);
  } else {
    *spc = 1.f;
    *slope = 1.;
  }
}

struct LorenzL {
  void Init(double sr, float freq, double s, double r, double b, double h,
            double x0, double y0, double z0) {
    sr_ = sr;
    x0_ = xn_ = xnm1_ = x0;
    y0_ = yn_ = ynm1_ = y0;
    z0_ = zn_ = znm1_ = z0;
    counter_ = 0.f;
    frac_ = 0.f;
    float dummy;
    Step(dummy, freq, s, r, b, h, x0, y0, z0);
  }
  void ProcessBlock(float* out, int n, float freq, double s, double r,
                    double b, double h, double x0, double y0, double z0) {
    for (int i = 0; i < n; ++i) Step(out[i], freq, s, r, b, h, x0, y0, z0);
  }
 private:
  void Step(float& out, float freq, double s, double r, double b, double h,
            double x0, double y0, double z0) {
    float spc; double slope;
    ChaosCycle(freq, sr_, &spc, &slope);
    if (x0_ != x0 || y0_ != y0 || z0_ != z0) {
      xnm1_ = xn_; ynm1_ = yn_; znm1_ = zn_;
      x0_ = xn_ = x0; y0_ = yn_ = y0; z0_ = zn_ = z0;
    }
    double dx = xn_ - xnm1_;
    if (counter_ >= spc) {
      counter_ -= spc;
      frac_ = 0.f;
      xnm1_ = xn_; ynm1_ = yn_; znm1_ = zn_;
      double k1x, k2x, k3x, k4x, k1y, k2y, k3y, k4y, k1z, k2z, k3z, k4z;
      double khx, khy, khz;
      const double hs = h * s;
      k1x = hs * (ynm1_ - xnm1_);
      k1y = h * (xnm1_ * (r - znm1_) - ynm1_);
      k1z = h * (xnm1_ * ynm1_ - b * znm1_);
      khx = k1x * 0.5; khy = k1y * 0.5; khz = k1z * 0.5;
      k2x = hs * (ynm1_ + khy - xnm1_ - khx);
      k2y = h * ((xnm1_ + khx) * (r - znm1_ - khz) - (ynm1_ + khy));
      k2z = h * ((xnm1_ + khx) * (ynm1_ + khy) - b * (znm1_ + khz));
      khx = k2x * 0.5; khy = k2y * 0.5; khz = k2z * 0.5;
      k3x = hs * (ynm1_ + khy - xnm1_ - khx);
      k3y = h * ((xnm1_ + khx) * (r - znm1_ - khz) - (ynm1_ + khy));
      k3z = h * ((xnm1_ + khx) * (ynm1_ + khy) - b * (znm1_ + khz));
      k4x = hs * (ynm1_ + k3y - xnm1_ - k3x);
      k4y = h * ((xnm1_ + k3x) * (r - znm1_ - k3z) - (ynm1_ + k3y));
      k4z = h * ((xnm1_ + k3x) * (ynm1_ + k3y) - b * (znm1_ + k3z));
      const double sixth = 0.1666666666666667;
      xn_ += (k1x + 2.0 * (k2x + k3x) + k4x) * sixth;
      yn_ += (k1y + 2.0 * (k2y + k3y) + k4y) * sixth;
      zn_ += (k1z + 2.0 * (k2z + k3z) + k4z) * sixth;
      dx = xn_ - xnm1_;
    }
    ++counter_;
    out = static_cast<float>((xnm1_ + dx * frac_) * 0.04f);
    frac_ += slope;
  }
  double sr_ = 48000., x0_ = 0., y0_ = 0., z0_ = 0.;
  double xn_ = 0., yn_ = 0., zn_ = 0., xnm1_ = 0., ynm1_ = 0., znm1_ = 0.;
  float counter_ = 0.f;
  double frac_ = 0.;
};

struct HenonC {
  void Init(double sr, float freq, double a, double b, double x0, double x1) {
    sr_ = sr;
    a_ = a; b_ = b; x0_ = x0; x1_ = x1;
    xn_ = x1; xnm1_ = x0; xnm2_ = xnm3_ = x1;
    counter_ = 0.f; stable_ = true; frac_ = 0.f;
    c0_ = c1_ = c2_ = c3_ = 0.;
    float dummy;
    Step(dummy, freq, a, b, x0, x1);
  }
  void ProcessBlock(float* out, int n, float freq, double a, double b,
                    double x0, double x1) {
    for (int i = 0; i < n; ++i) Step(out[i], freq, a, b, x0, x1);
  }
 private:
  void Step(float& out, float freq, double a, double b, double x0, double x1) {
    float spc; double slope;
    ChaosCycle(freq, sr_, &spc, &slope);
    if (a_ != a || b_ != b || x0_ != x0 || x1_ != x1) {
      if (!stable_) {
        xnm3_ = xnm2_; xnm2_ = x0; xnm1_ = x0; xn_ = x1;
      }
      stable_ = true;
      a_ = a; b_ = b; x0_ = x0; x1_ = x1;
    }
    if (counter_ >= spc) {
      counter_ -= spc;
      frac_ = 0.0;
      if (stable_) {
        xnm3_ = xnm2_; xnm2_ = xnm1_; xnm1_ = xn_;
        xn_ = 1.f - (a * xnm1_ * xnm1_) + (b * xnm2_);
        if (xn_ > 1.5 || xn_ < -1.5) {
          stable_ = false;
          xn_ = 1.; xnm1_ = xnm2_ = xnm3_ = 0.;
        }
        IPol3Coef(xnm3_, xnm2_, xnm1_, xn_, &c0_, &c1_, &c2_, &c3_);
      }
    }
    ++counter_;
    out = static_cast<float>(IPol3(frac_, c0_, c1_, c2_, c3_));
    frac_ += slope;
  }
  double sr_ = 48000., a_ = 0., b_ = 0., x0_ = 0., x1_ = 0.;
  double xn_ = 0., xnm1_ = 0., xnm2_ = 0., xnm3_ = 0.;
  double c0_ = 0., c1_ = 0., c2_ = 0., c3_ = 0.;
  float counter_ = 0.f;
  double frac_ = 0.;
  bool stable_ = true;
};

struct FBSineC {
  void Init(double sr, float freq, double a, double b, double c, double d,
            double x0, double y0) {
    sr_ = sr;
    x0_ = x0; y0_ = y0;
    xn_ = xnm1_ = xnm2_ = xnm3_ = x0;
    yn_ = y0;
    counter_ = 0.f; frac_ = 0.f;
    c0_ = c1_ = c2_ = c3_ = 0.;
    float dummy;
    Step(dummy, freq, a, b, c, d, x0, y0);
  }
  void ProcessBlock(float* out, int n, float freq, double a, double b,
                    double c, double d, double x0, double y0) {
    for (int i = 0; i < n; ++i) Step(out[i], freq, a, b, c, d, x0, y0);
  }
 private:
  void Step(float& out, float freq, double a, double b, double c, double d,
            double x0, double y0) {
    float spc; double slope;
    ChaosCycle(freq, sr_, &spc, &slope);
    if (x0_ != x0 || y0_ != y0) {
      x0_ = xn_ = x0;
      y0_ = y0;
      xnm3_ = xnm2_ = xnm1_ = xn_;
    }
    if (counter_ >= spc) {
      counter_ -= spc;
      frac_ = 0.f;
      xnm3_ = xnm2_; xnm2_ = xnm1_; xnm1_ = xn_;
      xn_ = std::sin(a * yn_ + b * xn_);
      yn_ = ChaosMod2Pi(c * yn_ + d);
      IPol3Coef(xnm3_, xnm2_, xnm1_, xn_, &c0_, &c1_, &c2_, &c3_);
    }
    ++counter_;
    out = static_cast<float>(IPol3(frac_, c0_, c1_, c2_, c3_));
    frac_ += slope;
  }
  double sr_ = 48000., x0_ = 0., y0_ = 0.;
  double xn_ = 0., yn_ = 0., xnm1_ = 0., xnm2_ = 0., xnm3_ = 0.;
  double c0_ = 0., c1_ = 0., c2_ = 0., c3_ = 0.;
  float counter_ = 0.f;
  double frac_ = 0.;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_CHAOS_H_
