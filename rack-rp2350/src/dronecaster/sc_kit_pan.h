// sc_kit_pan.h — SC 3.14.1 pan family (GPL-3.0, PanUGens.cpp).
// Pan2 ctor 792 / next 679; Balance2 ctor 268 / next 293; Rotate2 ctor 1464 /
// next 1432. Each ctor primes ONE sample via its _next_aa. The equal-power
// tables are SC's gSine (= KitTables::sine, 8193 entries) via ft->mSine.
// Parameter changes ramp the derived amplitudes/coefs with CALCSLOPE (1/64).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_PAN_H_
#define RACK_SRC_DRONECASTER_SC_KIT_PAN_H_
#include "sc_kit_osc.h"

namespace sc {

struct Pan2 {
  void Init(KitTables* t, float pos, float level, float first_in) {
    t_ = t;
    pos_ = pos;
    level_ = level;
    Compute(pos, level, &la_, &ra_);
    float l, r;
    Step(first_in, la_, ra_, l, r);  // ctor's next_aa(unit, 1)
  }
  void ProcessBlock(const float* in, float* left, float* right, int n,
                    float pos, float level) {
    bool changed = false;
    float nla = la_, nra = ra_, sla = 0.f, sra = 0.f;
    if (pos != pos_ || level != level_) {
      Compute(pos, level, &nla, &nra);
      sla = (nla - la_) * (1.f / 64.f);
      sra = (nra - ra_) * (1.f / 64.f);
      pos_ = pos;
      level_ = level;
      changed = true;
    }
    for (int i = 0; i < n; ++i) {
      Step(in[i], la_, ra_, left[i], right[i]);
      if (changed) { la_ += sla; ra_ += sra; }
    }
    if (changed) { la_ = nla; ra_ = nra; }
  }

 private:
  void Compute(float pos, float level, float* l, float* r) const {
    int ipos = static_cast<int32_t>(1024.f * pos + 1024.f + 0.5f);
    ipos = clampf(static_cast<float>(ipos), 0.f, 2048.f);
    *l = level * t_->sine[2048 - ipos];
    *r = level * t_->sine[ipos];
  }
  static void Step(float x, float la, float ra, float& l, float& r) {
    l = x * la;
    r = x * ra;
  }
  KitTables* t_ = nullptr;
  float pos_ = 0.f, level_ = 0.f, la_ = 0.f, ra_ = 0.f;
};

struct Balance2 {
  void Init(KitTables* t, float pos, float level, float first_l,
            float first_r) {
    t_ = t;
    pos_ = pos;
    level_ = level;
    Compute(pos, level, &la_, &ra_);
    float l, r;
    Step(first_l, first_r, la_, ra_, l, r);
  }
  void ProcessBlock(const float* inl, const float* inr, float* left,
                    float* right, int n, float pos, float level) {
    bool changed = false;
    float nla = la_, nra = ra_, sla = 0.f, sra = 0.f;
    if (pos != pos_ || level_ != level) {
      Compute(pos, level, &nla, &nra);
      sla = (nla - la_) * (1.f / 64.f);
      sra = (nra - ra_) * (1.f / 64.f);
      pos_ = pos;
      level_ = level;
      changed = true;
    }
    for (int i = 0; i < n; ++i) {
      Step(inl[i], inr[i], la_, ra_, left[i], right[i]);
      if (changed) { la_ += sla; ra_ += sra; }
    }
    if (changed) { la_ = nla; ra_ = nra; }
  }

 private:
  void Compute(float pos, float level, float* l, float* r) const {
    int ipos = static_cast<int32_t>(1024.f * pos + 1024.f + 0.5f);
    ipos = clampf(static_cast<float>(ipos), 0.f, 2048.f);
    *l = level * t_->sine[2048 - ipos];
    *r = level * t_->sine[ipos];
  }
  static void Step(float xl, float xr, float la, float ra, float& l,
                   float& r) {
    l = xl * la;
    r = xr * ra;
  }
  KitTables* t_ = nullptr;
  float pos_ = 0.f, level_ = 0.f, la_ = 0.f, ra_ = 0.f;
};

struct Rotate2 {
  void Init(KitTables* t, float pos, float first_x, float first_y) {
    t_ = t;
    pos_ = pos;
    Compute(pos, &sint_, &cost_);
    float x, y;
    Step(first_x, first_y, cost_, sint_, x, y);
  }
  void ProcessBlock(const float* xin, const float* yin, float* xout,
                    float* yout, int n, float pos) {
    bool changed = false;
    float ns = sint_, nc = cost_, ss = 0.f, cs = 0.f;
    if (pos != pos_) {
      Compute(pos, &ns, &nc);
      ss = (ns - sint_) * (1.f / 64.f);
      cs = (nc - cost_) * (1.f / 64.f);
      pos_ = pos;
      changed = true;
    }
    for (int i = 0; i < n; ++i) {
      Step(xin[i], yin[i], cost_, sint_, xout[i], yout[i]);
      if (changed) { sint_ += ss; cost_ += cs; }
    }
    if (changed) { sint_ = ns; cost_ = nc; }
  }

 private:
  void Compute(float pos, float* s, float* c) const {
    const int32_t isinpos = 8191 & static_cast<int32_t>(4096.f * pos);
    const int32_t icospos = 8191 & (2048 + isinpos);
    *s = t_->sine[isinpos];
    *c = t_->sine[icospos];
  }
  static void Step(float x, float y, float cost, float sint, float& xo,
                   float& yo) {
    xo = cost * x + sint * y;
    yo = cost * y - sint * x;
  }
  KitTables* t_ = nullptr;
  float pos_ = 0.f, sint_ = 0.f, cost_ = 1.f;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_PAN_H_
