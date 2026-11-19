// sc_kit_sc3p_dyn.h — sc3-plugins dynamics @ 25eacaf7 (GPL-2+ / GPL-3+).
// Source license quotes:
//   DistortionUGens/DistortionUGens.cpp: "This program is free software;
//   you can redistribute it and/or modify it under the terms of the GNU
//   General Public License as published by the Free Software Foundation;
//   either version 2 of the License, or (at your option) any later version."
//   DEINDUGens/DiodeRingMod.cpp: "Copyright (C) 2010, 2011 / Tony Hardie-Bick
//   (Java version) / Jonny Stutters (SuperCollider port) / GPL-2 or later."
//   MCLDUGens/MCLDDistortionUGens.cpp: "Copyright (c) 2004-2010 Michael
//   Codispoti / This program is free software ... GPL-2 or later."
// Anchors: CrossoverDistortion 105/159, Decimator 131/257, SineShaper 141/301,
// DiodeRingMod Ctor 60 / next 82, InsideOut 48.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_SC3P_DYN_H_
#define RACK_SRC_DRONECASTER_SC_KIT_SC3P_DYN_H_
#include <cmath>

namespace sc {

struct CrossoverDistortion {
  void Init(double sr, float amp, float smooth, float first_in) {
    sr_ = static_cast<float>(sr);
    amp_ = amp; smooth_ = smooth;
    rfade_ = 1.f / std::fabs(amp * smooth);
    float dummy;
    Step(first_in, dummy, amp, smooth);  // ctor primes one sample
  }
  void ProcessBlock(const float* in, float* out, int n, float amp,
                    float smooth) {
    for (int i = 0; i < n; ++i) Step(in[i], out[i], amp, smooth);
  }
 private:
  void Step(float x, float& out, float next_amp, float next_smooth) {
    float amp = amp_, smooth = smooth_, rf = rfade_;
    float as = 0.f, ss = 0.f, rs = 0.f;
    if (next_amp != amp_ || next_smooth != smooth_) {
      const float new_rfade = 1.f / std::fabs(next_amp * next_smooth);
      // Source stores the new unit values first, then ramps the locals that
      // still hold the OLD values toward them (CALCSLOPE, 1/64 per sample).
      rs = (new_rfade - rf) * (1.f / 64.f);
      as = (next_amp - amp) * (1.f / 64.f);
      ss = (next_smooth - smooth) * (1.f / 64.f);
      amp_ = next_amp; smooth_ = next_smooth; rfade_ = new_rfade;
    }
    float sig = std::fabs(x) - amp;
    if (sig < 0.f) sig *= (1.f + (sig * rf)) * smooth;
    out = x < 0.f ? -sig : sig;
    amp += as; smooth += ss; rf += rs;
  }
  float sr_ = 48000.f, amp_ = 0.f, smooth_ = 0.f, rfade_ = 0.f;
};

struct Decimator {
  void Init(double sr, float rate, float bits, float first_in) {
    sr_ = static_cast<float>(sr);
    count_ = 0.f; last_ = 0.f;
    float dummy;
    Step(first_in, dummy, rate, bits);
  }
  void ProcessBlock(const float* in, float* out, int n, float rate,
                    float bits) {
    for (int i = 0; i < n; ++i) Step(in[i], out[i], rate, bits);
  }
 private:
  void Step(float x, float& out, float rate, float bits) {
    float step, stepr;
    if (bits >= 31.f || bits < 1.f) { step = 0.f; stepr = 1.f; }
    else { step = std::pow(0.5f, bits - 0.999f); stepr = 1.f / step; }
    const float ratio = rate >= sr_ ? 1.f : rate / sr_;
    count_ += ratio;
    if (count_ >= 1.f) {
      count_ -= 1.f;
      const double v = (x + (x < 0 ? -1.0 : 1.0) * step * 0.5) * stepr;
      double dummy;
      const float delta = static_cast<float>(std::modf(v, &dummy)) * step;
      last_ = x - delta;
    }
    out = last_;
  }
  float sr_ = 48000.f, count_ = 0.f, last_ = 0.f;
};

struct SineShaper {
  void ProcessBlock(const float* in, float* out, int n, float limit) {
    const float inv = 1.f / limit;
    for (int i = 0; i < n; ++i)
      out[i] = limit * static_cast<float>(std::sin(in[i] * inv));
  }
};

struct DiodeRingMod {
  DiodeRingMod() {
    v_b_ = 0.2f; v_l_ = 0.6f; h_ = 1.f;
    curve_ = h_ / (2.f * (v_l_ - v_b_));
    lin_ = curve_ * std::pow(v_l_ - v_b_, 2.f) - h_ * v_l_;
  }
  void ProcessBlock(const float* carrier, const float* mod, float* out, int n) {
    for (int i = 0; i < n; ++i) {
      const float in1 = carrier[i] + 0.5f * mod[i];
      const float in2 = carrier[i] - 0.5f * mod[i];
      out[i] = (Transfer(in1) + Transfer(-in1)) -
               (Transfer(in2) + Transfer(-in2));
    }
  }
 private:
  float Transfer(float x) const {
    if (x > v_l_) return h_ * x + lin_;
    if (x > v_b_) return curve_ * std::pow(x - v_b_, 2.f);
    return 0.f;
  }
  float v_b_ = 0.2f, v_l_ = 0.6f, h_ = 1.f, curve_ = 0.f, lin_ = 0.f;
};

struct InsideOut {
  void ProcessBlock(const float* in, float* out, int n) {
    for (int i = 0; i < n; ++i) {
      const float v = in[i];
      out[i] = v > 0.f ? 1.f - v : (v < 0.f ? -1.f - v : 0.f);
    }
  }
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_SC3P_DYN_H_
