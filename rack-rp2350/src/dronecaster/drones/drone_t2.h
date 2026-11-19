// drone_t2.h — T-D2 drones: harms way (id 36).
// Line-for-line C++ mirror of the .scd graph (GPL-3.0 upstream:
// northern-information/dronecaster drones/harms way.scd (@moonblind)):
//   harms way:  { arg hz=100, amp=0.5;
//                 var n = (1..16);
//                 var snd = Splay.ar((SinOsc.ar(hz * n) / n)
//                                    * SinOsc.kr({Rand(0.001,0.02)} ! 16),
//                                    0.5, amp, SinOsc.kr(0.001));
//                 snd; }
// Supersaw (id 25) DEFERRED (2026-11-20): the port reached max=2.682e-6 /
// corr=1.000000000 (worst sample 2044) after full law work below — the
// residual lives in Supersaw's changing-freq BPF coefficient-interpolation /
// kr->ar ramp fold, which no existing kit ref exercises. Revisit in T-D3; the
// Supersaw-specific laws below are preserved for that resume. Catalog id 25
// stays nullptr (inert per plan).
// The gen_goldens.scd wrapper passes hz/amp as K2A.ar(control) — flat for the
// golden's constant controls, which is the same block-constant c.hz / c.amp
// the trio drones consume.
//
// Laws (SC 3.14.1 sources, cited per block; the golden render is the gate):
// - Splay.sc:23-33 + LevelComp: out = Mix(Pan2.ar(in_i, pos_i)) * level with
//   pos_i = ((0..n-1)*(2/(n-1)) - 1) * spread + center and
//   level *= n.reciprocal.sqrt (ar). The position constants are computed in
//   sclang DOUBLES and rounded to float ONCE (def constants). The kit Splay's
//   float chain drifts up to 5 ULP (e.g. base 3: -0.30000001 vs
//   -0.29999998), which flips Pan2's ipos table index when the
//   SinOsc.kr(0.001) center slides across a lookup boundary, so the expansion
//   is inlined here with the exact constants (verified for n=16/spread 0.5:
//   9 of 16 bases differ; Supersaw's {-1,-0.5,0,0.5,1} are exact).
// - Mix.sc:3-18: clump(4) -> Sum4 per group -> Mix of the group sums;
//   Sum4 = nova simd_mix.hpp detail::sum: (sig0+sig1)+(sig2+sig3).
//   5 inputs: [Sum4(v0..v3), v4] -> sum = s0 + v4 (Collection.sum injects 0,
//   which is an exact no-op in float).
// - kr -> ar ramps (BinaryOpUGens.cpp:1538 add_ak_nova / 1848 mul_ak_nova +
//   nova slope_argument; BinaryOpUGen_Ctor:608 mPrevB = the kr signal's
//   primed output): block k runs lanes [p, p+s, p+2s, p+2s+s], each lane
//   += 4s per 4-sample chunk, s = CALCSLOPE(v(k), p) with p = v(k-1). All kr
//   signals here prime to 0 (SinOsc.kr phase 0; the MulAdd output buffer is
//   untouched pool memory, zero for a fresh NRT graph), so block 0 is flat 0.
//   SinOsc.kr phase 0 -> lookupi1 at phase 0 is exactly 0 (sine_wt pairs).
// - SinOsc.kr (OscUGens.cpp:1325 SinOsc_Ctor + Osc_ikk_perform): one step per
//   64-block; SAMPLERATE == unit->mRate->mSampleRate == 750 at 48 kHz
//   (SC_World.cpp:774 Rate_Init(&mBufRate, sr/64, 1)) and m_cpstoinc is a
//   DOUBLE member (SC_PlugIn.hpp TableLookup) — the kr increment is
//   (int32)(double*float), unlike the ar path which copies it to a float
//   local (Osc_iai_perform). Hence the drone-local SinOscKr below.
// - Saw (OscUGens.cpp:2366/2382): freq read ONCE per block via ZIN0 — and
//   the ar+kr add's FIRST ramp value is mPrevB = the PREVIOUS kr value, so
//   the Saw sees hz*i + v(k-1). freq 0: m_N = (int32)(24000/0) -> INT_MIN on
//   x86 (cvttsd2si), every phase-0 lookup hits the kBadValue band and the
//   bad-denominator branch emits 1.f + 0.999f*y1 (a decay toward 1).
// - BPF (FilterUGens.cpp:2290 BPF_Ctor / 2308 BPF_next / 2365 BPF_next_1):
//   ctor primes one sample (consuming the Saw's primed sample, which equals
//   the first real sample because the Saw ctor rolls its state back) and
//   stores the coefficients; on a freq change the coefficients interpolate
//   one step per 3-sample group — mFilterLoops = 64/3 = 21, mFilterSlope =
//   1/21 (SC_Rate.cpp Rate_Init) — plus one remain sample with the
//   accumulated coefficients. The kit's Filter2Pole uses a 4-sample group law
//   that no existing reference exercises (all filter refs are constant-freq),
//   so the changing-freq law is ported verbatim below; lift it into
//   sc_kit_filter.h when a kit-side sweep ref exists.
// - Pan2 (PanUGens.cpp:792 ctor + Pan2_next_ak; the nova variant ramps the
//   amps the same way): per-block pos/level, amps ramped with CALCSLOPE and
//   the TARGETS stored after the block (kit Pan2).
// - Rand (NoiseUGens.cpp:443 Rand_Ctor): frand()*(hi-lo)+lo at ctor time.
//   RandSeed.kr(1, seed) is the FIRST UGen of the golden def and its ctor
//   prime sees trig 0->1 (NoiseUGens.cpp:738), so the RGen is re-seeded
//   before the 16 Rand draws, which run in creation order — exactly the
//   engine's rgen.init(kGoldenSeed) followed by 16 RandIrg draws.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_T2_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_T2_H_
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "sc_defs.h"
#include "sc_kit_osc.h"
#include "sc_kit_osc2.h"     // Saw
#include "sc_kit_pan.h"      // Pan2
#include "../dronecaster_engine.h"

namespace sc {
namespace drones {

// SinOsc.kr — OscUGens.cpp:1325 SinOsc_Ctor + Osc_ikk_perform. One phase step
// per 64-block at the control rate (sr/64); the DOUBLE m_cpstoinc drives the
// (int32) increment cast. The ctor primes one sample at phase 0 (lookup at
// phase 0 is exactly 0) and rolls the phase back, so v(-1) = v(0) = 0 and the
// downstream K2A ramps start flat.
struct SinOscKr {
  void Init(const KitTables* t, double control_rate, float freq) {
    t_ = t;
    cpstoinc_ = static_cast<double>(kSineSize) * (1.0 / control_rate) * 65536.0;
    lomask_ = (kSineSize - 1) << 3;
    phase_ = 0;
    freq_ = freq;
  }
  float Process() {
    const float z = lookupi1(t_->sine_wt, t_->sine_wt + 1, phase_, lomask_);
    phase_ += static_cast<uint32_t>(static_cast<int32_t>(cpstoinc_ * freq_));
    return z;
  }

 private:
  const KitTables* t_ = nullptr;
  double cpstoinc_ = 0.0;
  int32_t lomask_ = 0;
  uint32_t phase_ = 0;
  float freq_ = 0.0f;
};

// BPF with a per-block (kr) freq — verbatim FilterUGens.cpp:2290-2363
// (BPF_Ctor / BPF_next / BPF_next_1). Currently UNUSED: Supersaw (the only
// consumer) is deferred; kept for the T-D3 resume — the changing-freq
// 3-sample coefficient-interpolation law below is exercised by no other
// graph and no kit ref. The ctor prime is folded into the first
// ProcessBlock: it consumes in[0] (the Saw's first real sample, equal to the
// prime the real ctor consumed) and computes/stores the coefficients, so the
// block then runs the UNCHANGED branch exactly like the source's first block.
struct BpfKr {
  void Init(double sr) {
    rps_ = 2.0 * M_PI / sr;   // mRadiansPerSample = twopi/sr
    a0_ = b1_ = b2_ = 0.0;
    y1_ = y2_ = 0.0;
    freq_ = 0.0f;
    bw_ = 0.0f;
    primed_ = false;
  }
  void ProcessBlock(const float* in, float* out, int n, float freq, float bw) {
    if (!primed_) {
      // BPF_next_1 (ctor prime): coefs from this freq/bw, one Step on in[0].
      double a0, b1, b2;
      ComputeCoefs(freq, bw, &a0, &b1, &b2);
      double y1 = y1_, y2 = y2_;
      const double y0 = in[0] + b1 * y1 + b2 * y2;  // prime output discarded
      y2 = y1;
      y1 = y0;
      y1_ = ZapGremlins(y1);
      y2_ = ZapGremlins(y2);
      a0_ = a0;
      b1_ = b1;
      b2_ = b2;
      freq_ = freq;
      bw_ = bw;
      primed_ = true;
    }
    double y1 = y1_, y2 = y2_;
    double a0 = a0_, b1 = b1_, b2 = b2_;
    if (freq != freq_ || bw != bw_) {
      double na0, nb1, nb2;
      ComputeCoefs(freq, bw, &na0, &nb1, &nb2);
      // mFilterSlope = 1/mFilterLoops, mFilterLoops = 64/3 = 21 (int).
      const double kSlope = 1.0 / static_cast<double>(kBlock / 3);
      const double sa0 = (na0 - a0) * kSlope;
      const double sb1 = (nb1 - b1) * kSlope;
      const double sb2 = (nb2 - b2) * kSlope;
      int i = 0;
      for (int g = 0; g < kBlock / 3; ++g, i += 3) {
        double y0 = in[i + 0] + b1 * y1 + b2 * y2;
        out[i + 0] = static_cast<float>(a0 * (y0 - y2));
        y2 = in[i + 1] + b1 * y0 + b2 * y1;
        out[i + 1] = static_cast<float>(a0 * (y2 - y1));
        y1 = in[i + 2] + b1 * y2 + b2 * y0;
        out[i + 2] = static_cast<float>(a0 * (y1 - y0));
        a0 += sa0;
        b1 += sb1;
        b2 += sb2;
      }
      // mFilterRemain = 1 sample with the accumulated coefficients.
      double y0 = in[i] + b1 * y1 + b2 * y2;
      out[i] = static_cast<float>(a0 * (y0 - y2));
      y2 = y1;
      y1 = y0;
      freq_ = freq;
      bw_ = bw;
      a0_ = na0;
      b1_ = nb1;
      b2_ = nb2;
    } else {
      int i = 0;
      for (int g = 0; g < kBlock / 3; ++g, i += 3) {
        double y0 = in[i + 0] + b1 * y1 + b2 * y2;
        out[i + 0] = static_cast<float>(a0 * (y0 - y2));
        y2 = in[i + 1] + b1 * y0 + b2 * y1;
        out[i + 1] = static_cast<float>(a0 * (y2 - y1));
        y1 = in[i + 2] + b1 * y2 + b2 * y0;
        out[i + 2] = static_cast<float>(a0 * (y1 - y0));
      }
      double y0 = in[i] + b1 * y1 + b2 * y2;
      out[i] = static_cast<float>(a0 * (y0 - y2));
      y2 = y1;
      y1 = y0;
    }
    y1_ = ZapGremlins(y1);
    y2_ = ZapGremlins(y2);
  }

 private:
  void ComputeCoefs(float freq, float bw, double* a0, double* b1,
                    double* b2) const {
    const double pfreq = static_cast<double>(freq) * rps_;
    const double pbw = static_cast<double>(bw) * pfreq * 0.5;
    const double C = 1.0 / std::tan(pbw);
    const double D = 2.0 * std::cos(pfreq);
    *a0 = 1.0 / (1.0 + C);
    *b1 = C * D * (*a0);
    *b2 = (1.0 - C) * (*a0);
  }
  static double ZapGremlins(double x) {
    // SC_InlineUnaryOp.h:301 zapgremlins(float64)
    const double absx = std::fabs(x);
    return (absx > 1e-15 && absx < 1e15) ? x : 0.0;
  }
  double rps_ = 0.0;
  double a0_ = 0.0, b1_ = 0.0, b2_ = 0.0;
  double y1_ = 0.0, y2_ = 0.0;
  float freq_ = 0.0f, bw_ = 0.0f;
  bool primed_ = false;
};

// K2A ramp of a kr signal multiplying an ar one (mul_ak_nova + nova
// slope_argument law, verified by ref_kr_fill): block k runs lanes
// [p, p+s, p+2s, p+2s+s] with each lane += 4s per 4-sample chunk. `div` is
// the FDiv constant applied BEFORE the multiply ((carrier / n) * ramp, as the
// graph orders the two BinaryOps); x/1.f is exact so div 1 is a no-op.
inline void MulRamp64(const float* in, float* out, int n, float prev, float v,
                      float div) {
  const float slope = static_cast<float>((v - prev) * (1.0 / 64.0));
  const float slope2 = slope + slope;
  float b0 = prev;
  float b1 = prev + slope;
  float b2 = prev + slope2;
  float b3 = prev + slope2 + slope;
  const float inc = slope2 + slope2;    // 4*slope (exact doubling)
  for (int j = 0; j < n; ++j) {
    float lane;
    switch (j & 3) {
      case 0: lane = b0; break;
      case 1: lane = b1; break;
      case 2: lane = b2; break;
      default: lane = b3; break;
    }
    out[j] = (in[j] / div) * lane;
    if ((j & 3) == 3) { b0 += inc; b1 += inc; b2 += inc; b3 += inc; }
  }
}

// ---------------------------------------------------------------------------
// harms way (id 36)
// ---------------------------------------------------------------------------
struct HarmsWayState {
  SinOsc carrier[16];       // SinOsc.ar(hz * n), n = 1..16
  SinOscKr env[16];         // SinOsc.kr({Rand(0.001,0.02)} ! 16)
  SinOscKr center;          // SinOsc.kr(0.001) — Splay center
  Pan2 pan[16];             // Splay's Pan2 bank (level 1)
  float ramp_prev[16];      // the mul's mPrevB = v(k-1); block -1 = 0
  float base[16];           // def-exact Splay position constants
  float lc;                 // LevelComp ar: n.reciprocal.sqrt
  float carrierbuf[64], voicebuf[64];
  float pan_l[64], pan_r[64], tmp_l[64], tmp_r[64];
  float t01_l[64], t01_r[64], t23_l[64], t23_r[64];
  float acc_a_l[64], acc_a_r[64], acc_b_l[64], acc_b_r[64];
};

inline void HarmsWay_Init(Ctx& c) {
  HarmsWayState* st =
      static_cast<HarmsWayState*>(c.arena->Alloc(sizeof(HarmsWayState)));
  // {Rand(0.001,0.02)} ! 16 — NoiseUGens.cpp:443 Rand_Ctor: frand()*(hi-lo)+lo
  // in creation order (0..15), after RandSeed re-seeded the RGen.
  float rands[16];
  for (int i = 0; i < 16; ++i) rands[i] = RandIrg(c.rgen, 0.001f, 0.02f);
  const double cr = static_cast<double>(c.sr) / static_cast<double>(kBlock);
  for (int i = 0; i < 16; ++i) {
    st->carrier[i].Init(c.tables, static_cast<double>(c.sr),
                        c.hz * static_cast<float>(i + 1));
    st->env[i].Init(c.tables, cr, rands[i]);
    // Splay.sc:23-33 position constant: sclang double, rounded to float once.
    st->base[i] = static_cast<float>((static_cast<double>(i) * (2.0 / 15.0)
                                      - 1.0) * 0.5);
    // Pan2_Ctor: m_pos/m_level from ZIN0 (level = 1, the Splay default), amps
    // at the ctor pos (base + center prime = base), one primed sample (the
    // voice's prime = 0), state kept.
    st->pan[i].Init(c.tables, st->base[i], 1.f, 0.f);
    st->ramp_prev[i] = 0.f;   // kr ctor primes at phase 0 -> v(-1) = 0
  }
  st->center.Init(c.tables, cr, 0.001f);
  st->lc = static_cast<float>(std::sqrt(1.0 / 16.0));   // LevelComp (ar)
  c.state = st;
}

inline void HarmsWay_Process(Ctx& c, float* l, float* r, int n) {
  HarmsWayState* st = static_cast<HarmsWayState*>(c.state);
  const float center_k = st->center.Process();   // SinOsc.kr block value
  // level_sig = K2A(amp) * 0.25 (ugen 50_*; flat for the golden's constant
  // amp), then Mix * level_sig: one per-sample multiply by that product.
  const float amp_level = c.amp * st->lc;
  for (int g = 0; g < 4; ++g) {
    for (int k = 0; k < 4; ++k) {
      const int i = g * 4 + k;
      st->carrier[i].ProcessBlock(st->carrierbuf, n);
      // voice = (carrier / n) * env_kr — the kr input ramps (mul_ak_nova);
      // the FDiv by the literal n runs BEFORE the multiply (voice 0's /1 is
      // folded away by sclang, and x/1.f is exact).
      const float v = st->env[i].Process();
      const float prev = st->ramp_prev[i];
      st->ramp_prev[i] = v;
      MulRamp64(st->carrierbuf, st->voicebuf, n, prev, v,
                static_cast<float>(i + 1));
      // Pan2: per-block pos = base + center (kr scalar+kr add, no ramp).
      st->pan[i].ProcessBlock(st->voicebuf, st->pan_l, st->pan_r, n,
                              st->base[i] + center_k, 1.f);
      // Mix tree (def dump 108/118/128/138_Sum4): each group of 4 pans is
      // Sum4(p3, p2, p1, p0) = (p3+p2) + (p1+p0) — the optimizer reverses the
      // operands. Streaming: voices 0,1 build pairB = (p1+p0); 2,3 build
      // pairA = (p3+p2); s_g = pairA + pairB.
      float* tl = (k < 2) ? st->t01_l : st->t23_l;   // t01 = pairB, t23 = pairA
      float* tr = (k < 2) ? st->t01_r : st->t23_r;
      if (k == 1 || k == 3) {
        for (int j = 0; j < n; ++j) { tl[j] += st->pan_l[j]; tr[j] += st->pan_r[j]; }
      } else {
        for (int j = 0; j < n; ++j) { tl[j] = st->pan_l[j]; tr[j] = st->pan_r[j]; }
      }
    }
    // s_g = pairA + pairB
    for (int j = 0; j < n; ++j) {
      st->t23_l[j] = st->t23_l[j] + st->t01_l[j];
      st->t23_r[j] = st->t23_r[j] + st->t01_r[j];
    }
    // Final Sum4 (def dump 139/142): Sum4(g3, g2, g1, g0) = (g3+g2)+(g1+g0).
    if (g == 0) {
      for (int j = 0; j < n; ++j) { st->acc_b_l[j] = st->t23_l[j]; st->acc_b_r[j] = st->t23_r[j]; }
    } else if (g == 1) {
      for (int j = 0; j < n; ++j) { st->acc_b_l[j] = st->acc_b_l[j] + st->t23_l[j]; st->acc_b_r[j] = st->acc_b_r[j] + st->t23_r[j]; }
    } else if (g == 2) {
      for (int j = 0; j < n; ++j) { st->acc_a_l[j] = st->t23_l[j]; st->acc_a_r[j] = st->t23_r[j]; }
    } else {
      for (int j = 0; j < n; ++j) { st->acc_a_l[j] = st->acc_a_l[j] + st->t23_l[j]; st->acc_a_r[j] = st->acc_a_r[j] + st->t23_r[j]; }
    }
  }
  // Final Sum4 of the group sums, then the level multiply.
  for (int j = 0; j < n; ++j) {
    const float mix_l = st->acc_a_l[j] + st->acc_b_l[j];
    const float mix_r = st->acc_a_r[j] + st->acc_b_r[j];
    l[j] = mix_l * amp_level;
    r[j] = mix_r * amp_level;
  }
}

}  // namespace drones

inline const DroneSpec kDroneHarmsWay = {"harms way", drones::HarmsWay_Init,
                                         drones::HarmsWay_Process};

}  // namespace sc
#endif
