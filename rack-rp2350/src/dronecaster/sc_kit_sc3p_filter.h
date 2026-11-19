// sc_kit_sc3p_filter.h — SC3P filters for the Dronecaster kit (Task 7).
// Pin: /tmp/sc3p_full @ 25eacaf7a55143476ab68a5ea335281e7c7a052f.
// Every port quotes its source file's license header verbatim below.
//
// SVF — source/BlackrainUGens/BlackrainUGens.cpp:
//   "// supercollider 3 ugens"
//   "// blackrain at realizedsound dot net - 11 2006"
// MoogLadder — source/BhobUGens/BhobFilt.cpp:
//   "filter plugins by bhob rainey.  http://www.bhobrainey.net"
//   "MoogLadder is based on work by Victor Lazzarini and Antti Huovilainen."
//   "SuperCollider real time aud synthesis system"
//   "    Copyright (c) 2002 James McCartney. All rights reserved."
//   "    http://www.audsynth.com"
//   "    This program is free software; you can redistribute it and/or modify"
//   "    it under the terms of the GNU General Public License as published by"
//   "    the Free Software Foundation; either version 2 of the License, or"
//   "    (at your option) any later version."
//   "    This program is distributed in the hope that it will be useful,"
//   "    but WITHOUT ANY WARRANTY; without even the implied warranty of"
//   "    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the"
//   "    GNU General Public License for more details."
#ifndef RACK_SRC_DRONECASTER_SC_KIT_SC3P_FILTER_H_
#define RACK_SRC_DRONECASTER_SC_KIT_SC3P_FILTER_H_
#include <cmath>
#include <cstdint>

namespace sc {

static inline float SCClip(float x, float lo, float hi) {
  return x > hi ? hi : (x < lo ? lo : x);
}

// SVF — BlackrainUGens.cpp:99 (ctor), 113 (next). All float math. The ctor
// zeros state then consumes ONE sample through SVF_next (which also computes
// the coefficients because m_cutoff starts at 0). Two oversampled stages per
// output sample; the five output levels are clipped and summed.
struct SVF {
  void Init(double sr, float cutoff, float resonance, float lowlevel,
            float bandlevel, float highlevel, float notchlevel, float peaklevel,
            float first_in) {
    sr_ = static_cast<float>(sr);
    cutoff_ = resonance_ = freq_ = damp_ = 0.0f;
    notch_ = low_ = high_ = band_ = 0.0f;
    float unused;
    Step(first_in, cutoff, resonance, lowlevel, bandlevel, highlevel,
         notchlevel, peaklevel, unused);
  }
  void ProcessBlock(const float* in, float* out, int n, float cutoff,
                    float resonance, float lowlevel, float bandlevel,
                    float highlevel, float notchlevel, float peaklevel) {
    for (int i = 0; i < n; ++i)
      Step(in[i], cutoff, resonance, lowlevel, bandlevel, highlevel,
           notchlevel, peaklevel, out[i]);
  }

 private:
  void Step(float input, float cutoff, float resonance, float lowlevel,
            float bandlevel, float highlevel, float notchlevel, float peaklevel,
            float& out) {
    cutoff = SCClip(cutoff, 20.0f, sr_);
    resonance = SCClip(resonance, 0.0f, 1.0f);
    lowlevel = SCClip(lowlevel, 0.0f, 1.0f);
    bandlevel = SCClip(bandlevel, 0.0f, 1.0f);
    highlevel = SCClip(highlevel, 0.0f, 1.0f);
    notchlevel = SCClip(notchlevel, 0.0f, 1.0f);
    peaklevel = SCClip(peaklevel, 0.0f, 1.0f);
    if (cutoff != cutoff_ || resonance != resonance_) {
      cutoff_ = cutoff;
      resonance_ = resonance;
      const double kPi = 3.14159265358979323846;  // SC_Constants' double pi
      const double lim = 0.25 < cutoff / (sr_ * 2.0f)
                             ? 0.25
                             : static_cast<double>(cutoff / (sr_ * 2.0f));
      freq_ = static_cast<float>(2.0f * std::sin(kPi * lim));
      const double a = 2.0 * (1.0 - static_cast<double>(
          std::pow(resonance, 0.25f)));
      const float inner = 2.0f / freq_ - freq_ * 0.5f;
      const double b = static_cast<double>(inner < 2.0f ? inner : 2.0f);
      damp_ = static_cast<float>(a < b ? a : b);
    }
    float low = 0.0f, band = 0.0f, high = 0.0f, notch = 0.0f, peak = 0.0f;
    for (int pass = 0; pass < 2; ++pass) {
      notch_ = input - damp_ * band_;
      low_ = low_ + freq_ * band_;
      high_ = notch_ - low_;
      band_ = freq_ * high_ + band_;
      const float l = 0.5f * low_, bg = 0.5f * band_,
                  h = 0.5f * high_, n = 0.5f * notch_;
      low += l; band += bg; high += h; notch += n;
      peak += 0.5f * (low_ - high_);
    }
    out = low * lowlevel + band * bandlevel + high * highlevel +
          notch * notchlevel + peak * peaklevel;
  }
  float sr_ = 48000.0f, cutoff_ = 0.0f, resonance_ = 0.0f;
  float freq_ = 0.0f, damp_ = 0.0f;
  float notch_ = 0.0f, low_ = 0.0f, high_ = 0.0f, band_ = 0.0f;
};

// MoogLadder — BhobFilt.cpp:824 (ctor), 849 (next_kk). Float math, 2x
// oversampled. NOTE: the ctor's fcr grouping (line 841) differs from next()'s
// (line 879); with constant params the ctor's k2v persists, so both are kept.
// The ctor uses ClearUnitOutputs (no priming sample); all state starts zero.
struct MoogLadder {
  void Init(double sr, float cutoff, float resonance) {
    const double sample_dur = 1.0 / sr;
    const double twopi = 2.0 * 3.14159265358979323846;
    fco_ = cutoff;
    const float fc = static_cast<float>(fco_ * sample_dur);
    const float f = static_cast<float>(fco_ * sample_dur * 0.5);
    const float fcr = 1.8730f * fc * fc * fc + 0.4955f * fc * fc -
                      0.6490f * fc + 0.9988f;
    k2v_ = static_cast<float>(1.25 * (1.0 - std::exp(-twopi * fcr * f)));
    res_ = resonance;
    xnm1_ = y1nm1_ = y2nm1_ = y3nm1_ = y4nm1_ = 0.0f;
    y1n_ = y2n_ = y3n_ = y4n_ = y1n1_ = 0.0f;
  }
  void ProcessBlock(const float* in, float* out, int n, float cutoff,
                    float resonance) {
    float fco = fco_, res = res_, k2v = k2v_;
    float xnm1 = xnm1_, y1nm1 = y1nm1_, y2nm1 = y2nm1_,
          y3nm1 = y3nm1_, y4nm1 = y4nm1_, y1n1 = y1n1_;
    float y1n = y1n_, y2n = y2n_, y3n = y3n_, y4n = y4n_;
    const float i2v = 0.70466f;
    float k2vslope = 0.0f, resslope = 0.0f;
    const double sample_dur = 1.0 / sr_;
    const double twopi = 2.0 * 3.14159265358979323846;
    if (cutoff != fco) {
      const float fc = static_cast<float>(cutoff * sample_dur);
      const float f = static_cast<float>(cutoff * sample_dur * 0.5);
      const float fcr = 1.8730f * (fc * fc * fc + 0.4955f * fc * fc) -
                        0.6490f * fc + 0.9988f;
      const float nextk2v =
          static_cast<float>(1.25 * (1.0 - std::exp(-twopi * fcr * f)));
      k2vslope = (nextk2v - k2v) * (1.0f / 64.0f);
    }
    if (resonance != res) resslope = (resonance - res) * (1.0f / 64.0f);
    for (int i = 0; i < n; ++i) {
      for (int pass = 0; pass < 2; ++pass) {
        const float input = in[i];
        float xn = input - 8.0f * res * y4n;
        float xn1 = xn * i2v;
        float y2nm11 = y2nm1 * i2v, y3nm11 = y3nm1 * i2v,
              y4nm11 = y4nm1 * i2v;
        y1n = xnm1 + k2v * (xn1 / (1.0f + std::fabs(xn1)) -
                            y1n1 / (1.0f + std::fabs(y1n1)));
        y1n1 = y1n * i2v;
        y2n = y2nm1 + k2v * (y1n1 / (1.0f + std::fabs(y1n1)) -
                             y2nm11 / (1.0f + std::fabs(y2nm11)));
        float y2n1 = y2n * i2v;
        y3n = y3nm1 + k2v * (y2n1 / (1.0f + std::fabs(y2n1)) -
                             y3nm11 / (1.0f + std::fabs(y3nm11)));
        float y3n1 = y3n * i2v;
        y4n = y4nm1 + k2v * (y3n1 / (1.0f + std::fabs(y3n1)) -
                             y4nm11 / (1.0f + std::fabs(y4nm11)));
        y4n = (y4n + y4nm1) * 0.5f;
        xnm1 = xn; y1nm1 = y1n; y2nm1 = y2n; y3nm1 = y3n; y4nm1 = y4n;
      }
      if (k2vslope != 0.0f) k2v += k2vslope;
      if (resslope != 0.0f) res += resslope;
      out[i] = y4n;
    }
    fco_ = cutoff; res_ = res; k2v_ = k2v;
    xnm1_ = xnm1; y1nm1_ = y1nm1; y2nm1_ = y2nm1; y3nm1_ = y3nm1;
    y4nm1_ = y4nm1; y1n1_ = y1n1;
    y1n_ = y1n; y2n_ = y2n; y3n_ = y3n; y4n_ = y4n;
  }
  float sr_ = 48000.0f;
  float fco_ = 0.0f, res_ = 0.0f, k2v_ = 0.0f;
  float xnm1_ = 0.0f, y1nm1_ = 0.0f, y2nm1_ = 0.0f, y3nm1_ = 0.0f,
        y4nm1_ = 0.0f, y1n1_ = 0.0f;
  float y1n_ = 0.0f, y2n_ = 0.0f, y3n_ = 0.0f, y4n_ = 0.0f;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_SC3P_FILTER_H_
