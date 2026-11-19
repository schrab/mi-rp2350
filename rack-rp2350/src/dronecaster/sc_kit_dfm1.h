// sc_kit_dfm1.h — DFM1 filter (sc3-plugins @ 25eacaf7a55143476ab68a5ea33528
// 1e7c7a052f). Source files carry this license header, quoted verbatim:
//
//  * DFM-1 Digital Filter Module
//  * Copyright (C) 2010, 2011
//  *
//  * Tony Hardie-Bick <tony@entitysynth.net> - Java version
//  * Jonny Stutters <jstutters@jeremah.co.uk> - SuperCollider port
//  *
//  * This program is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 2 of the License, or
//  * (at your option) any later version.
//  *
//  * This program is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  * GNU General Public License for more details.
//
// Anchors: TJUGens.cpp:55 (Ctor), 81 (next); Dfm1.cpp:49 (filter), 224
// (initialize); Dfm1Lut.cpp:624 (getCoefficients); NoiseGen.cpp:144 (rnor),
// 164 (xor128), 183 (uni), 203 (nfix), 250 (initialize).
//
// The upstream Ctor seeds its xorshift from `time(NULL)`, so NRT renders are
// irreproducible. This port takes an explicit `epoch` (captured by the reference
// renderer into ugen_ref/DFM1.seed.txt) so the render can be matched exactly.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_DFM1_H_
#define RACK_SRC_DRONECASTER_SC_KIT_DFM1_H_
#include <cmath>
#include <cstdint>
#include "sc_kit_dfm1_tables.h"

namespace sc {

// xorshift128 + Marsaglia/Tsang ziggurat (NoiseGen.cpp). Tables in
// sc_kit_dfm1_tables.h are transcribed verbatim from the pinned source.
struct Dfm1Noise {
  uint32_t x = 0, y = 0, z = 0, w = 0;
  void Init(int epoch) {
    // NoiseGen::initialize with counter starting at 0: x=0,y=1,z=2,w=3*epoch.
    x = 0; y = 1; z = 2; w = static_cast<uint32_t>(3 * epoch);
  }
  int32_t xor128() {
    const uint32_t t = x ^ (x << 15);
    x = y; y = z; z = w;
    w = (w ^ (w >> 21)) ^ (t ^ (t >> 4));
    return static_cast<int32_t>(w);
  }
  double uni() {
    return 0.5 + static_cast<double>(xor128()) * (1.0 / 4294967296.0);
  }
  double rnor() {
    const int32_t hz = xor128();
    const int32_t iz = hz & 127;
    return std::fabs(static_cast<float>(hz)) < static_cast<float>(kNoiseKn[iz])
               ? static_cast<double>(hz) * kNoiseWn[iz]
               : nfix(hz, iz);
  }
  double nfix(int32_t hz, int32_t iz) {
    const double r = 3.442619855899;
    for (;;) {
      double x = static_cast<double>(hz) * kNoiseWn[iz];
      if (iz == 0) {
        double ax, y;
        do {
          ax = -std::log(uni()) * 0.2904764516147;
          y = -std::log(uni());
        } while (y + y < ax * ax);
        return hz > 0 ? r + ax : -r - ax;
      }
      if (kNoiseFn[iz] + uni() * (kNoiseFn[iz - 1] - kNoiseFn[iz]) <
          std::exp(-0.5 * x * x)) {
        return x;
      }
      const int32_t nhz = xor128();
      iz = nhz & 127;
      hz = nhz;
      if (std::fabs(static_cast<float>(hz)) <
          static_cast<float>(kNoiseKn[iz])) {
        return static_cast<double>(hz) * kNoiseWn[iz];
      }
    }
  }
};

// Dfm1Lut.cpp:624. 16 octaves x 32 steps x 3 coefficients (a, b, resonance).
static inline void Dfm1Coefficients(double freq, double sr, double* c) {
  if (sr < 1.0) sr = 1.0;
  freq /= sr;
  const double kMin = 1.0 / 131072.0;
  const double kMax = 0.5 - 1.0 / (32 * 4.0) - 0.001;
  if (freq < kMin) freq = kMin;
  if (freq > kMax) freq = kMax;
  int octave = 0;
  double base = kMin, next = kMin * 2.0;
  while (octave < 16) {
    if (freq < next) break;
    base = next;
    next *= 2.0;
    ++octave;
  }
  const double step = 32 * (freq - base) / base;  // octFreqSpan == base
  const int index = static_cast<int>(std::floor(step));
  const double fraction = step - static_cast<double>(index);
  int address = 3 * (index + octave * 32);
  const double xa = kDfm1Lut[address++], xb = kDfm1Lut[address++],
               xr = kDfm1Lut[address++];
  const double ya = kDfm1Lut[address++], yb = kDfm1Lut[address++],
               yr = kDfm1Lut[address];
  c[0] = xa + (ya - xa) * fraction;
  c[1] = xb + (yb - xb) * fraction;
  c[2] = xr + (yr - xr) * fraction;
}

// Dfm1.cpp:49 (filter) + TJUGens.cpp:55/81 (SC wrapper state plumbing).
struct Dfm1 {
  void Init(double sr, int epoch, float freq, float res, float input_gain,
            float type, float noise_level) {
    sr_ = static_cast<float>(sr);
    noise_.Init(epoch);
    l_ = h_ = a_ = b_ = r_ = s_ = 0.0;
    za_ = zb_ = zh_ = zr_ = zy_ = 0.0;
    freq_ = freq; res_ = res; gain_ = input_gain;
    type_ = type; noise_level_ = noise_level;
  }
  void ProcessBlock(const float* in, float* out, int n) {
    const double kNOISE = 0.00001, kNOISE_GAIN = 0.05;
    const double kPOST = (1.0 - 0.05) * 3.0 / 2.0;
    const double kPRE = 1.0 / (1.0 - 0.05) * 3.0 / 2.0;
    const double kN = 0.55032120814910446;

    float frequency = freq_, resonance = res_;
    if (frequency < 1.0f) frequency = 1.0f;
    if (resonance < 0.0f) resonance = 0.0f;
    if (resonance > 10.0f) resonance = 10.0f;

    double coeff[3];
    Dfm1Coefficients(frequency, sr_, coeff);
    const double at = coeff[0], bt = coeff[1], rt = coeff[2] * resonance;
    double lt = gain_ * kPRE, ht = gain_ * kPRE;
    if (type_ < 0.5f) ht = 0.0; else lt = 0.0;
    lt *= (1.0 - at);
    double st = noise_level_ * kNOISE_GAIN;
    if (noise_level_ < kNOISE) st = kNOISE;

    const double i = 1.0 / n;
    const double li = Check((lt - l_) * i), hi = Check((ht - h_) * i);
    const double ai = Check((at - a_) * i), bi = Check((bt - b_) * i);
    const double ri = Check((rt - r_) * i), si = Check((st - s_) * i);
    double l = l_, h = h_, a = a_, b = b_, r = r_, s = s_;
    double za = za_, zb = zb_, zh = zh_, zr = zr_, zy = zy_;
    for (int t = 0; t < n; ++t) {
      l += li; h += hi; a += ai; b += bi; r += ri; s += si;
      const double x = in[t];
      za = x * l + r * (zy - zr) + a * za + s * noise_.rnor();
      zb = za * (1.0 - b) + (x - zh) * h + b * zb + s * noise_.rnor();
      zh = x;
      zr = zy;
      zb = zb < -1.0 ? -1.0 : (zb > 1.0 ? 1.0 : zb);
      const double nx = kN * zb;
      zy = zb - 2.0 * nx * nx * nx + s * noise_.rnor();
      out[t] = static_cast<float>(zy * kPOST);
    }
    l_ = lt; h_ = ht; a_ = at; b_ = bt; r_ = rt; s_ = st;
    za_ = za; zb_ = zb; zh_ = zh; zr_ = zr; zy_ = zy;
  }
  void SetParams(float freq, float res) { freq_ = freq; res_ = res; }

 private:
  static double Check(double x) {
    return std::fabs(x) < 0.000001 ? 0.0 : x;
  }
  Dfm1Noise noise_;
  float sr_ = 48000.0f, freq_ = 1000.0f, res_ = 0.1f, gain_ = 1.0f;
  float type_ = 0.0f, noise_level_ = 0.0003f;
  double l_ = 0, h_ = 0, a_ = 0, b_ = 0, r_ = 0, s_ = 0;
  double za_ = 0, zb_ = 0, zh_ = 0, zr_ = 0, zy_ = 0;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_DFM1_H_
