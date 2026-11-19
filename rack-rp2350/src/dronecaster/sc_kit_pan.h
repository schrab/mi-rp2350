// sc_kit_pan.h — SC 3.14.1 pan family (GPL-3.0, PanUGens.cpp).
// Pan2 ctor 792 / next 679; Balance2 ctor 268 / next 293; Rotate2 ctor 1464 /
// next 1432. Each ctor primes ONE sample via its _next_aa. The equal-power
// tables are SC's gSine (= KitTables::sine, 8193 entries) via ft->mSine.
// Parameter changes ramp the derived amplitudes/coefs with CALCSLOPE (1/64).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_PAN_H_
#define RACK_SRC_DRONECASTER_SC_KIT_PAN_H_
#include <algorithm>
#include <vector>
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

// Splay — classlib Splay.sc:23-33. Expansion law:
//   n = max(2, inArray.size); positions = ((0..n-1)*(2/(n-1)) - 1)*spread+center
//   level = level * (levelComp ? n.reciprocal.sqrt : 1)
//   out = Mix(Pan2.ar(in_i, pos_i)) * level     (each Pan2 uses level = 1)
struct Splay {
  void Init(KitTables* t, int count, float spread, float level, float center,
            bool level_comp, const float* first_ins) {
    n_ = count < 2 ? 2 : count;
    pans_.resize(static_cast<size_t>(n_));
    for (int i = 0; i < n_; ++i) {
      const float p = (static_cast<float>(i) * (2.f / (n_ - 1)) - 1.f) *
                          spread + center;
      pans_[static_cast<size_t>(i)].Init(t, p, 1.f, first_ins[i]);
    }
    level_ = level * (level_comp ? 1.f / std::sqrt(static_cast<float>(n_))
                                 : 1.f);
  }
  void ProcessBlock(const float* const* ins, float* left, float* right, int n,
                    float spread, float level, float center) {
    for (int i = 0; i < n; ++i) { left[i] = 0.f; right[i] = 0.f; }
    // M33: fixed scratch, no heap per block (the old std::vector did a
    // malloc/free on every 64-sample block — a real-time no-no).
    for (int k = 0; k < n_; ++k) {
      const float p = (static_cast<float>(k) * (2.f / (n_ - 1)) - 1.f) *
                          spread + center;
      // Pan2 at level 1; the classlib scales the final Mix, not each pan.
      pans_[static_cast<size_t>(k)].ProcessBlock(ins[k], pl_, pr_, n, p, 1.f);
      for (int i = 0; i < n; ++i) { left[i] += pl_[i]; right[i] += pr_[i]; }
    }
    for (int i = 0; i < n; ++i) {
      left[i] *= level_;
      right[i] *= level_;
    }
    (void)level;
  }
  std::vector<Pan2> pans_;
  float pl_[64] = {}, pr_[64] = {};  // fixed scratch: no heap per block
  int n_ = 2;
  float level_ = 1.f;
};

// PanAz — PanUGens.cpp:1270 (ctor), 1302 (next_ak). Ambisonic-style panning
// over numChans outputs using the same gSine table, with CALCSLOPE on amps.
struct PanAz {
  void Init(KitTables* t, int nchans, float pos, float level, float width,
            float orientation, float first_in) {
    t_ = t;
    nchans_ = nchans;
    chanamp_.assign(static_cast<size_t>(nchans), 0.f);
    std::vector<float> next;
    Compute(pos, level, width, orientation, &next);
    chanamp_ = next;  // ctor's next_ak(unit,1) stores the target as current
    (void)first_in;
  }
  void ProcessBlock(const float* in, float** outs, int n, float pos,
                    float level, float width, float orientation) {
    std::vector<float> next;
    Compute(pos, level, width, orientation, &next);
    std::vector<float> cur = chanamp_;
    for (int ch = 0; ch < nchans_; ++ch) {
      const float slope = (next[static_cast<size_t>(ch)] - cur[static_cast<size_t>(ch)]) *
                          (1.f / 64.f);
      for (int i = 0; i < n; ++i) {
        outs[ch][i] = in[i] * cur[static_cast<size_t>(ch)];
        cur[static_cast<size_t>(ch)] += slope;
      }
    }
    chanamp_ = next;
  }

 private:
  void Compute(float pos, float level, float width, float orientation,
               std::vector<float>* out) const {
    width = std::fabs(width);
    const float rwidth = static_cast<float>(1.0 / static_cast<double>(width));
    const float range = static_cast<float>(nchans_) * rwidth;
    const float rrange = static_cast<float>(1.0 / static_cast<double>(range));
    pos = pos * 0.5f * static_cast<float>(nchans_) + width * 0.5f + orientation;
    out->resize(static_cast<size_t>(nchans_));
    for (int i = 0; i < nchans_; ++i) {
      float chanpos = pos - static_cast<float>(i);
      chanpos *= rwidth;
      chanpos = chanpos - range * std::floor(rrange * chanpos);
      (*out)[static_cast<size_t>(i)] =
          chanpos >= 1.f ? 0.f
                         : level * t_->sine[static_cast<long>(4096.f * chanpos)];
    }
  }
  KitTables* t_ = nullptr;
  int nchans_ = 4;
  std::vector<float> chanamp_;
};

// SplayAz — classlib Splay.sc:52-73. n = max(1, count);
// normSpread = (n - 1/n) * spread; positions = resamp1 of
// [center-normSpread, center+normSpread] to n; level *= n.reciprocal.sqrt;
// then per source a PanAz(numChans, source, pos_i, level, width, orientation)
// and Mix across sources for each channel.
struct SplayAz {
  void Init(KitTables* t, int nchans, int count, float spread, float level,
            float width, float center, float orientation, bool level_comp,
            const float* first_ins) {
    n_ = count < 1 ? 1 : count;
    nchans_ = nchans;
    // SC parses (n - 1 / n) left-to-right as (n-1)/n, not n - (1/n).
    const float norm = (static_cast<float>(n_ - 1) / static_cast<float>(n_)) * spread;
    level_ = level * (level_comp ? 1.f / std::sqrt(static_cast<float>(n_)) : 1.f);
    pans_.resize(static_cast<size_t>(n_));
    for (int k = 0; k < n_; ++k) {
      const float p = n_ == 1 ? center
          : (center - norm) + 2.f * norm * static_cast<float>(k) /
                                   static_cast<float>(n_ - 1);
      pans_[static_cast<size_t>(k)].Init(t, nchans, p, level_, width,
                                         orientation, first_ins[k]);
      positions_.push_back(p);
    }
    outs_.assign(static_cast<size_t>(n_),
                 std::vector<std::vector<float>>(
                     static_cast<size_t>(nchans), std::vector<float>(64, 0.f)));
    mixed_.resize(static_cast<size_t>(nchans));
    for (auto& v : mixed_) v.assign(64, 0.f);
  }
  void ProcessBlock(const float* const* ins, float** out, int n,
                    float spread, float level, float width, float center,
                    float orientation) {
    for (auto& v : mixed_) std::fill(v.begin(), v.begin() + n, 0.f);
    for (int k = 0; k < n_; ++k) {
      float* chans[64];
      for (int ch = 0; ch < nchans_; ++ch)
        chans[ch] = outs_[static_cast<size_t>(k)][static_cast<size_t>(ch)].data();
      pans_[static_cast<size_t>(k)].ProcessBlock(ins[k], chans,
                                                 n, positions_[static_cast<size_t>(k)],
                                                 level_, width, orientation);
      for (int ch = 0; ch < nchans_; ++ch)
        for (int i = 0; i < n; ++i)
          mixed_[static_cast<size_t>(ch)][static_cast<size_t>(i)] +=
              outs_[static_cast<size_t>(k)][static_cast<size_t>(ch)][static_cast<size_t>(i)];
    }
    for (int ch = 0; ch < nchans_; ++ch)
      for (int i = 0; i < n; ++i)
        out[ch][i] = mixed_[static_cast<size_t>(ch)][static_cast<size_t>(i)];
    (void)spread; (void)level; (void)center;
  }
  std::vector<PanAz> pans_;
  std::vector<float> positions_;
  std::vector<std::vector<std::vector<float>>> outs_;
  std::vector<std::vector<float>> mixed_;
  int n_ = 1, nchans_ = 4;
  float level_ = 1.f;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_PAN_H_
