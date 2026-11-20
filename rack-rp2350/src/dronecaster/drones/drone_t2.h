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
#include <cstring>
#include <new>
#if defined(__SSE__)
#include <xmmintrin.h>
#endif
#include "sc_defs.h"
#include "sc_kit_osc.h"
#include "sc_kit_osc2.h"     // Saw
#include "sc_kit_pan.h"      // Pan2
#include "sc_kit_lfo.h"      // LFPulse
#include "sc_kit_noise.h"    // WhiteNoise, Dust, LFNoise2
#include "sc_kit_trig.h"     // Latch
#include "sc_kit_filter.h"   // BEQFilter (BLowPass), Lag, LeakDC
#include "sc_kit_delay.h"    // DelayLine (CombN)
#include "sc_kit_envgen.h"   // EnvGen / VarLag (law-only expansion)
#include "sc_kit_lin.h"      // LinExp
#include "sc_kit_demand.h"   // Demand / Dxrand / Drand / Dbrown
#include "sc_kit_sc3p_filter.h"  // SVF
#include "sc_kit_core_dyn.h" // Compander, Limiter (Task 6)
#include "sc_kit_sc3p_dyn.h" // Decimator (Task 6)
#include "sc_sugar.h"        // (SelectX law reference; Hecker expands verbatim)
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
  // Server BPF_Ctor primes with the input's CTOR output; seeds y1/y2 for
  // block 0 (its output feeds the next filter's ctor prime).
  float Prime(float freq, float bw, float first_in) {
    double a0, b1, b2;
    ComputeCoefs(freq, bw, &a0, &b1, &b2);
    const double y0 = first_in + b1 * y1_ + b2 * y2_;
    const double primed_out = a0 * (y0 - y2_);
    y2_ = y1_; y1_ = y0;
    y1_ = ZapGremlins(y1_); y2_ = ZapGremlins(y2_);
    a0_ = a0; b1_ = b1; b2_ = b2; freq_ = freq; bw_ = bw; primed_ = true;
    return static_cast<float>(primed_out);
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

// ---------------------------------------------------------------------------
// nova functor scalar replications (SC 3.14.1 x86-64 build uses the nova-simd
// paths for the ar UnaryOps/BinaryOps these drones hit). The server computes
// ar-rate ops 64 samples at a time through nova's per-lane functors; each
// lane runs the scalar float code below, so a per-sample scalar replication
// is exact. kr UGens take the *_1 scalar paths instead (sc_* functions).
// Anchors: nova-simd simd_unit_conversion.hpp:43 (midi2freq) / :53
// (freq2midi via abs_log2), detail/vec_math.hpp:92 (ldexp_float) / :110
// (frexp_float) / :128 (vec_exp_float) / :163 (vec_log_float) / :204
// (vec_exp_tanh_float) / :483 (vec_tanh_float) / :519 (vec_pow) / :561
// (vec_log2); vec/vec_sse.hpp:586 (tanh -> vec_tanh_float), :549 (pow),
// :483 (round = _mm_round_ps TO_NEAREST_INT), :675 (truncate_to_int =
// cvttps2dq); BinaryOpUGens.cpp:2907 (pow_ai_nova = spow_vec_simd).
// ---------------------------------------------------------------------------

// _mm_round_ps(_MM_FROUND_TO_NEAREST_INT): nearest, ties to even.
inline float NovaRint(float x) { return std::rintf(x); }

// nova ldexp_float: pure bit manipulation (slli(n, 23) + exponent bits).
inline float NovaLdexp(float p, int32_t n) {
  uint32_t bits;
  std::memcpy(&bits, &p, 4);
  const int32_t new_exp =
      (n << 23) + static_cast<int32_t>(bits & 0x7F800000u);
  const uint32_t out = (bits & 0x807FFFFFu) | static_cast<uint32_t>(new_exp);
  float r;
  std::memcpy(&r, &out, 4);
  return r;
}

// nova frexp_float: m in [0.5, 1), x = m * 2^e (gen_exp_mask_1 = 0x3F000000).
inline float NovaFrexp(float x, int32_t* e) {
  uint32_t bits;
  std::memcpy(&bits, &x, 4);
  *e = static_cast<int32_t>(bits >> 23) - 126;
  bits = (bits & 0x807FFFFFu) | 0x3F000000u;
  float m;
  std::memcpy(&m, &bits, 4);
  return m;
}

// nova vec_exp_float (WITH the min/max boundary selects).
inline float NovaExp(float arg) {
  float x = arg;
  const float z = NovaRint(1.44269504088896341f * x);
  const int32_t n = static_cast<int32_t>(z);   // truncate_to_int
  x -= z * 0.693359375f;
  x -= z * -2.12194440e-4f;
  const float p = 1.0f +
      x * (1.00000035762786865234375f +
           x * (0.4999996721744537353515625f +
                x * (0.16665561497211456298828125f +
                     x * (4.167006909847259521484375e-2f +
                          x * (8.420792408287525177001953125e-3f +
                               x * 1.386119984090328216552734375e-3f)))));
  const float approx = NovaLdexp(p, n);
  if (arg > 88.72283905206835f) {   // mask_gt select -> max_float
    return 3.40282346638528859811704183484516925440e38f;
  }
  if (arg < -88.0f) {               // mask_lt select -> zero
    return 0.0f;
  }
  return approx;
}

// nova vec_exp_tanh_float (the boundary-check-free exp used by tanh).
inline float NovaExpTanh(float arg) {
  float x = arg;
  const float z = NovaRint(1.44269504088896341f * x);
  const int32_t n = static_cast<int32_t>(z);
  x -= z * 0.693359375f;
  x -= z * -2.12194440e-4f;
  const float p = 1.0f +
      x * (1.00000035762786865234375f +
           x * (0.4999996721744537353515625f +
                x * (0.16665561497211456298828125f +
                     x * (4.167006909847259521484375e-2f +
                          x * (8.420792408287525177001953125e-3f +
                               x * 1.386119984090328216552734375e-3f)))));
  return NovaLdexp(p, n);
}

// nova vec_log_float (cephes polynomial; the mask lanes become bools here).
inline float NovaLog(float x) {
  int32_t e;
  float m = NovaFrexp(x, &e);
  const bool small = m < 0.707106781186547524f;   // mask_lt(sqrt_05)
  e += small ? -1 : 0;                            // int_vec(mask) is -1/0
  const float x_add = small ? m : 0.0f;           // m & mask
  m += x_add - 1.0f;
  float y = (((((((( 7.0376836292E-2f * m
                    - 1.1514610310E-1f) * m
                   + 1.1676998740E-1f) * m
                  - 1.2420140846E-1f) * m
                 + 1.4249322787E-1f) * m
                - 1.6668057665E-1f) * m
               + 2.0000714765E-1f) * m
              - 2.4999993993E-1f) * m
             + 3.3333331174E-1f) * m * m * m;
  const float fe = static_cast<float>(e);         // cvtdq2ps
  y += fe * -2.12194440e-4f;
  y -= (0.5f * m) * m;
  const float z = m + y;
  return z + 0.693359375f * fe;
}

// nova vec_log2 = log(x) * float(1.0/std::log(2.0)).
inline float NovaLog2(float x) {
  return NovaLog(x) * static_cast<float>(1.0 / std::log(2.0));
}

// nova vec_pow = exp(b * log(a)), arg1 == 0 -> 0.
inline float NovaPow(float a, float b) {
  if (a == 0.0f) return 0.0f;   // mask_eq select
  return NovaExp(b * NovaLog(a));
}

// nova vec_signed_pow: sign | pow(|a|, b) (pow_ai_nova uses the spow wrapper).
inline float NovaSpow(float a, float b) {
  const float r = NovaPow(std::fabs(a), b);
  uint32_t ab, rb;
  std::memcpy(&ab, &a, 4);
  std::memcpy(&rb, &r, 4);
  rb |= (ab & 0x80000000u);
  float out;
  std::memcpy(&out, &rb, 4);
  return out;
}

// nova vec_tanh_float (cephes-style SSE polynomial; branches are lane masks).
// LAW (vec_math.hpp:483-524): the SMALL path (|x| < 0.625) evaluates the odd
// polynomial against the SIGNED arg (`* arg_sqr * arg + arg`) — dropping the
// sign there silently mirrors the negative half into the positive (found the
// hard way via Mt. Lion, whose comb output sits at |x| ≈ 0.625).
inline float NovaTanh(float x) {
  uint32_t xb;
  std::memcpy(&xb, &x, 4);
  const uint32_t sign = xb & 0x80000000u;
  const uint32_t absb = xb & 0x7FFFFFFFu;
  float abs_arg;
  std::memcpy(&abs_arg, &absb, 4);
  // medium values: 1 - 2/(exp(2|x|) + 1)
  const float medium = 1.0f - 2.0f / (NovaExpTanh(abs_arg + abs_arg) + 1.0f);
  // large values (|x| > 22): +-1
  float result_lm_abs = (abs_arg > 22.0f) ? 1.0f : medium;
  uint32_t lmb;
  std::memcpy(&lmb, &result_lm_abs, 4);
  lmb |= sign;
  std::memcpy(&result_lm_abs, &lmb, 4);
  // small values (|x| < 0.625): odd polynomial in the SIGNED x (nova
  // vec_math.hpp:516 — `* arg_sqr * arg + arg` — the sign must survive;
  // Mt. Lion's comb values sit right at this boundary)
  const float f1 = -5.70498872745e-3f;
  const float f2 = 2.06390887954e-2f;
  const float f3 = -5.37397155531e-2f;
  const float f4 = 1.33314422036e-1f;
  const float f5 = -3.33332819422e-1f;
  const float arg_sqr = abs_arg * abs_arg;
  const float result_small =
      ((((f1 * arg_sqr + f2) * arg_sqr + f3) * arg_sqr + f4) * arg_sqr + f5) *
          arg_sqr * x +
      x;
  return (abs_arg < 0.625f) ? result_small : result_lm_abs;
}

// nova midi2freq (TRUNCATED 1/12 literal — NOT the clean sc_midicps).
inline float NovaMidi2Freq(float midi) {
  return 440.0f * NovaPow(2.0f, (midi - 69.0f) * 0.083333333333f);
}

// nova freq2midi via abs_log2 (TRUNCATED 1/440 literal).
inline float NovaFreq2Midi(float freq) {
  return NovaLog2(std::fabs(freq * 0.0022727272727f)) * 12.0f + 69.0f;
}

// sc_midicps (SC_InlineUnaryOp.h:77) — the kr _1 path.
inline float ScMidicps(float note) {
  return 440.0f * std::pow(2.0f, (note - 69.0f) / 12.0f);
}

// sc_cpsmidi (SC_InlineUnaryOp.h:80) — the kr / scalar prime path.
inline float ScCpsmidi(float freq) {
  return std::log2(freq / 440.0f) * 12.0f + 69.0f;
}

// sc_reciprocal (SC_InlineUnaryOp.h:231): SSE rcpss + one NR step.
inline float ScReciprocal(float x) {
#if defined(__SSE__)
  const __m128 arg = _mm_set_ss(x);
  const __m128 approx = _mm_rcp_ss(arg);
  const __m128 muls = _mm_mul_ss(_mm_mul_ss(arg, approx), approx);
  const __m128 double_approx = _mm_add_ss(approx, approx);
  const __m128 result = _mm_sub_ss(double_approx, muls);
  return _mm_cvtss_f32(result);
#else
  return 1.0f / x;
#endif
}

inline float ScRound1(float x) {
  return std::floor(x / 1.0f + 0.5f) * 1.0f;   // sc_round(x, 1)
}

// sc_pow (SC_InlineBinaryOp.h:259) — the scalar pow_ai prime path.
inline float ScPow(float a, float b) {
  return a >= 0.0f ? std::pow(a, b) : -std::pow(-a, b);
}

// Pan2_next_aa (PanUGens.cpp:753) — per-sample pos table lookup, level read
// per block with a CALCSLOPE ramp whose ACCUMULATED end value is stored
// (unlike the kk variant, which stores the target).
struct Pan2Aa {
  void Init(KitTables* t, float level_ctor) {
    t_ = t;
    level_ = level_ctor;   // m_level = ZIN0(2) at ctor; the 1-sample prime is
                           // memoryless (no persistent Pan2 state besides
                           // m_pos/m_level, and aa never reads m_pos).
  }
  void ProcessBlock(const float* in, const float* pos, float* left,
                    float* right, int n, float nextlevel) {
    float level = level_;
    const bool changed = level != nextlevel;
    const float slope =
        changed ? (nextlevel - level) * (1.0f / 64.0f) : 0.0f;
    for (int i = 0; i < n; ++i) {
      int32_t ipos = static_cast<int32_t>(1024.0f * pos[i] + 1024.0f + 0.5f);
      ipos = ipos < 0 ? 0 : (ipos > 2048 ? 2048 : ipos);   // sc_clip
      left[i] = in[i] * (level * t_->sine[2048 - ipos]);
      right[i] = in[i] * (level * t_->sine[ipos]);
      if (changed) level += slope;
    }
    if (changed) level_ = level;   // the accumulated ramp end
  }

 private:
  KitTables* t_ = nullptr;
  float level_ = 0.0f;
};

// ---------------------------------------------------------------------------
// Thx (id 27) — 11 voices (numTone 1..11; (1..numVoices-1).collect).
// Def dump dc_Thx.scsyndef; the language-side rrand constants (fundamentals,
// sweepFactors, pan positions) are def constants read off the dump. Per
// voice: LFNoise2.kr(0.5)*mul1 + fund (kr MulAdd, no prime) drives the
// MulAdd(ak) ramp B; LFNoise2.kr(0.1)*mul2 drives the MulAdd(ik) ramp C;
// pow = amp^sweepFactor via nova spow (duplicated UGen, identical value);
// freq lane 0 = (1-p)*B_prev + p*(hz*destmul + C_prev) — only lane 0 is ever
// consumed (Saw/BLowPass read ZIN0 per block). Voice 11's mul1 = 0 folds the
// B into the CONSTANT fund (its LFNoise2 still exists and still draws).
// The ctor prime chain is load-bearing: ar MulAdds prime next_scalar, so
// 41's prime = 55*destmul (+0 from the pool-zero kr C), 43's prime =
// (1-sc_pow_prime)*0 + sc_pow_prime*that, Saw's ctor m_freqin = 43's prime,
// its primed out[0] feeds 45's prime (= *6) which is the BLowPass ctor freq,
// and BLowPass's 1-sample prime consumes the Saw's primed sample.
// Mix fold (L and R identical): Sum4(v4,v3,v2,v1) + Sum4(v8,v7,v6,v5) +
// Sum3(v11,v10,v9) -> Sum3(g3,g2,g1) -> /10; the engine applies EnvGen.
// ---------------------------------------------------------------------------
struct ThxVoice {
  // def constants (float32, from the dumped SynthDef)
  float fund;
  float mul1;        // LFNoise2.kr(0.5) mul; unused when b_const
  bool b_const;      // voice 11: the ramp B is the constant fund
  float destmul;     // finalPitches = hz * 2^((numTone/2).round - 3)
  float lf2mul;      // LFNoise2.kr(0.1) mul (voice 2: folded to raw)
  bool lf2_raw;
  float powexp;      // sweepFactors[numTone-1]
  float panpos, panlevel;
  // state
  LFNoise2 lf2a, lf2b;
  float powval, one_m_pow;
  float dest_base;
  float w_init_prev, w_dest_prev;   // MulAdd ControlRateInput values
  float saw_ctor_freq, saw_prime;
  Saw saw;
  BEQFilter blp;
  Pan2 pan;
};

struct ThxState {
  ThxVoice voice[11];
  float sawbuf[64], blpbuf[64], panl[64], panr[64];
  float vl[64], vr[64];
  float s14l[64], s14r[64], s58l[64], s58r[64], s911l[64], s911r[64];
  float v9l[64], v9r[64], v10l[64], v10r[64];
  float pair_a_l[64], pair_a_r[64], pair_b_l[64], pair_b_r[64];
  float r12, r10;   // div_ai_nova: ar/scalar = in * sc_reciprocal(scalar)
};

inline void Thx_Init(Ctx& c) {
  ThxState* st = static_cast<ThxState*>(c.arena->Alloc(sizeof(ThxState)));
  // def constants (dumpUGens of dc_Thx, in voice order):
  static const float kFund[11] = {489.67213630676f, 481.21061325073f,
      452.49471664429f, 386.20710372925f, 374.33710098267f, 339.62507247925f,
      325.85062980652f, 285.33153533936f, 155.21550178528f, 141.79782104492f,
      114.40277099609f};
  static const float kMul1[11] = {60.f, 54.f, 48.f, 42.f, 36.f, 30.f, 24.f,
      18.f, 12.f, 6.f, 0.f};
  static const float kDestmul[11] = {0.25f, 0.25f, 0.5f, 0.5f, 1.0f, 1.0f,
      2.0f, 2.0f, 4.0f, 4.0f, 8.0f};
  static const float kLf2mul[11] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f,
      4.0f, 4.5f, 5.0f, 5.5f};
  static const float kPowexp[11] = {1.9034866094589f, 1.1515994787216f,
      1.6746289491653f, 0.64245076179504f, 0.86975681781769f,
      1.1388231515884f, 1.0251595020294f, 0.27584428787231f,
      1.2043887853622f, 1.343373131752f, 0.68753805160522f};
  static const float kPanpos[11] = {0.3175612449646f, -0.53595571517944f,
      -0.0145836353302f, 0.37213439941406f, -0.14497690200806f,
      -0.50573301315308f, 0.53307266235352f, -0.36443037986755f,
      0.45296216011047f, -0.041524457931519f, 0.10575385093689f};
  static const float kPanlevel[11] = {0.4625f, 0.59753086419753f, 0.71640625f,
      0.8096f, 0.88225308641975f, 0.93977509371095f, 0.986181640625f,
      1.02429507697f, 1.0561f, 1.0830134553651f, 1.1060667438272f};
  const double sr = static_cast<double>(c.sr);
  st->r12 = ScReciprocal(12.0f);   // 48_/ etc. (div_ai_nova)
  st->r10 = ScReciprocal(10.0f);   // 183_/
  const float amp0 = 0.4f;    // 3_K2A prime (control value at ctor)
  const float hz0 = 55.0f;    // 2_K2A prime
  for (int v = 0; v < 11; ++v) {
    ThxVoice& vc = st->voice[v];
    vc.fund = kFund[v];
    vc.mul1 = kMul1[v];
    vc.b_const = (v == 10);
    vc.destmul = kDestmul[v];
    vc.lf2mul = kLf2mul[v];
    vc.lf2_raw = (v == 1);
    vc.powexp = kPowexp[v];
    vc.panpos = kPanpos[v];
    vc.panlevel = kPanlevel[v];
    // kr UGens (creation order: LF2a then LF2b per voice, 2 ctor draws each)
    vc.lf2a.Init(c.rgen, 750.0, 0.5f);
    vc.lf2b.Init(c.rgen, 750.0, 0.1f);
    // nova pow value (per-sample constant; duplicated pow UGens agree)
    vc.powval = NovaSpow(amp0, vc.powexp);
    vc.one_m_pow = 1.0f - vc.powval;
    vc.dest_base = hz0 * vc.destmul;
    // kr UGen ctor PRIMES (SIMD_Unit::set_calc_function runs next_scalar once;
    // kr BinaryOp/UnaryOp prime through BinaryOpUGen_Ctor's !initialized path):
    // 38_MulAdd (kr) primes to LF2a_prime*mul1 + fund; 40_* (kr mul) primes to
    // LF2b_prime*lf2mul. The ar MulAdds' ControlRateInput values are those
    // primed outputs, so block 0's ramp lanes START there.
    const float lf2a_prime = vc.lf2a.ctor_value();
    const float lf2b_prime = vc.lf2b.ctor_value();
    const float w_init_ctor =
        vc.b_const ? 0.0f : lf2a_prime * vc.mul1 + vc.fund;
    const float w_dest_ctor =
        vc.lf2_raw ? lf2b_prime : lf2b_prime * vc.lf2mul;
    vc.w_init_prev = w_init_ctor;
    vc.w_dest_prev = w_dest_ctor;
    // ar ctor prime chain (the NORMAL scalar functions prime, e.g. pow_ai =
    // sc_pow): 41's next_scalar prime = 55*destmul + 40's primed out[0]; 42's
    // mul prime = pow_prime * that; 43's next_scalar prime = (1-p)*B + that.
    const float p_sc = ScPow(amp0, vc.powexp);
    const float m41_prime = vc.dest_base + w_dest_ctor;
    const float b_ctor = vc.b_const ? vc.fund : w_init_ctor;
    const float m43_prime = (1.0f - p_sc) * b_ctor + p_sc * m41_prime;
    // Saw ctor: m_freqin = 43's primed out[0]; prime one sample, roll back.
    vc.saw_ctor_freq = m43_prime;
    vc.saw.Init(c.tables, sr, m43_prime);
    {
      const int32_t n_ctor =
          static_cast<int32_t>((sr * 0.5) / static_cast<double>(m43_prime));
      const float scale_ctor =
          static_cast<float>(0.5 / static_cast<double>(n_ctor));
      const float pv = vc.saw.PulAt(0, 2 * n_ctor + 1, scale_ctor, false, 0,
                                    0.0f, 0.0f);
      vc.saw_prime =
          (pv == 1.0f) ? (1.0f + 0.999f * -0.46f) : (pv + 0.999f * -0.46f);
    }
    // 45_* = freq * 6 (the FREQ signal, not the Saw); its ctor prime is
    // 43's primed out[0] * 6 = the BLowPass ctor freq. The BLowPass prime
    // consumes the Saw's primed sample (persistent y1/y2 state).
    vc.blp.Init(sr, 0, m43_prime * 6.0f, 0.6f, 0.0f, vc.saw_prime);
    // Pan2: pos/level are def constants; ctor prime is memoryless.
    vc.pan.Init(c.tables, vc.panpos, vc.panlevel, 0.0f);
  }
  c.state = st;
}

inline void Thx_Process(Ctx& c, float* l, float* r, int n) {
  (void)c;
  ThxState* st = static_cast<ThxState*>(c.state);
  for (int v = 0; v < 11; ++v) {
    ThxVoice& vc = st->voice[v];
    // kr UGens in creation order (LF2a then LF2b per voice — voice 11's
    // dead LF2a still runs and still draws).
    const float lf2a_val = vc.lf2a.ProcessKr();
    const float lf2b_val = vc.lf2b.ProcessKr();
    // kr block values
    const float w_init_cur =
        vc.b_const ? 0.0f : vc.fund + lf2a_val * vc.mul1;
    const float w_dest_cur =
        vc.lf2_raw ? lf2b_val : lf2b_val * vc.lf2mul;
    // freq lane 0 (only lane 0 of the MulAdd(ka) buffers is consumed):
    const float f =
        vc.b_const ? vc.one_m_pow * vc.fund +
                         vc.powval * (vc.dest_base + vc.w_dest_prev)
                   : vc.one_m_pow * vc.w_init_prev +
                         vc.powval * (vc.dest_base + vc.w_dest_prev);
    vc.w_init_prev = w_init_cur;
    vc.w_dest_prev = w_dest_cur;
    // Saw.ar(freq) — one variant, ZIN0(0) per block
    vc.saw.SetFreq(f);
    vc.saw.ProcessBlock(st->sawbuf, n);
    // freq * 6 -> BLowPass.ar(..., freq*6, 0.6): kk variant (freq ar + rq
    // scalar) reads ZIN0(1) per block.
    vc.blp.ProcessBlock(st->sawbuf, st->blpbuf, n, f * 6.0f, 0.6f, 0.0f);
    // Pan2.ar(..., const pos, const level) — amps never ramp
    vc.pan.ProcessBlock(st->blpbuf, st->panl, st->panr, n, vc.panpos,
                        vc.panlevel);
    for (int j = 0; j < n; ++j) {
      st->vl[j] = st->panl[j] * st->r12;
      st->vr[j] = st->panr[j] * st->r12;
    }
    // Mix fold, exactly the def's Sum4/Sum3 operand order.
    if (v < 8) {
      const int gi = v & 3;   // position within the group of 4
      float* tl = (gi < 2) ? st->pair_b_l : st->pair_a_l;
      float* tr = (gi < 2) ? st->pair_b_r : st->pair_a_r;
      if (gi == 1 || gi == 3) {   // second voice of the pair adds
        for (int j = 0; j < n; ++j) { tl[j] += st->vl[j]; tr[j] += st->vr[j]; }
      } else {                    // first voice of the pair starts it
        for (int j = 0; j < n; ++j) { tl[j] = st->vl[j]; tr[j] = st->vr[j]; }
      }
      if (gi == 3) {
        // group sum: (a+b) + (c+d) with a,b = the later two voices
        float* dst_l = (v < 4) ? st->s14l : st->s58l;
        float* dst_r = (v < 4) ? st->s14r : st->s58r;
        for (int j = 0; j < n; ++j) {
          dst_l[j] = st->pair_a_l[j] + st->pair_b_l[j];
          dst_r[j] = st->pair_a_r[j] + st->pair_b_r[j];
        }
      }
    } else if (v == 8) {
      for (int j = 0; j < n; ++j) { st->v9l[j] = st->vl[j]; st->v9r[j] = st->vr[j]; }
    } else if (v == 9) {
      for (int j = 0; j < n; ++j) { st->v10l[j] = st->vl[j]; st->v10r[j] = st->vr[j]; }
    } else {
      // 181_Sum3(v11, v10, v9) = (v11 + v10) + v9
      for (int j = 0; j < n; ++j) {
        st->s911l[j] = (st->vl[j] + st->v10l[j]) + st->v9l[j];
        st->s911r[j] = (st->vr[j] + st->v10r[j]) + st->v9r[j];
      }
    }
  }
  // 182_Sum3 = (s911 + s58) + s14; 183_/ = /10
  for (int j = 0; j < n; ++j) {
    l[j] = ((st->s911l[j] + st->s58l[j]) + st->s14l[j]) * st->r10;
    r[j] = ((st->s911r[j] + st->s58r[j]) + st->s14r[j]) * st->r10;
  }
}

// ---------------------------------------------------------------------------
// Mt. Zion (id 15) — 5 voices, ALL-AR random chains.
// Def dump dc_Mt. Zion.scsyndef (constants are the exact simple values:
// densities 0.2/0.5/0.3/0.1, width 0.5/1.5, lags 2/0.5, amp/5; NO extra
// scaling — the /12 in an earlier comment draft was Thx-only).
// Head (ar): hz K2A -> cpsmidi (nova freq2midi, x2 UGens) -> round(sc_round,
// quantum 1) -> sub -> abs = detune; amp K2A / 5 = maxamp. NOTE: NovaFreq2Midi
// (55) == 33 exactly, so detune == 0 and basenote == 33 — the freq chain is
// the constant 33 through Latch/Lag, midicps == 55, then * index (def
// 38/61/84/109_*; voice 1's *1 folds away in sclang).
// Per voice (creation order = RGen draw order): WhiteNoise, Dust(0.2),
// WhiteNoise, Dust(0.5), WhiteNoise, Dust(0.3), WhiteNoise, Dust(0.1); the
// chains: MulAdd(noise, mul, add) -> Latch.ar -> Lag.ar -> [midicps * index]
// -> Pulse.ar(freq=midicps*index [0], width=widthlag[0]) -> Pan2.ar(pulse,
// pos_lag AR, level_lag[0]) — Pan2_next_aa: per-sample pos table lookup,
// level = ZIN0(2) ramped with CALCSLOPE, the ACCUMULATED ramp end stored.
// Ctor primes: each WhiteNoise/Dust primes 1 draw (creation order); the
// ar MulAdd/BinaryOp/UnaryOp primes use the SCALAR functions on ctor-time
// values (11's prime = wn_prime*0 + 33 = 33; 15_midicps's prime =
// sc_midicps(Lag's out[0] = 0) = the Pulse ctor freq); the Latch ctor prime
// is rolled back (m_prevtrig = 0, mLevel = 0); the Lag ctor primes y1 =
// ZIN0(0) = the Latch's primed out[0] = 0. NO Dust ctor prime fires under
// the golden seed (RGen probe), so every Latch/Lag starts at 0.
// Mix: Sum4(v4,v3,v2,v1) + v5 per channel (BinaryOp add); NO extra scaling.
// ---------------------------------------------------------------------------
struct MzVoice {
  WhiteNoise wn_freq, wn_width, wn_pos, wn_level;
  Dust dust_freq, dust_width, dust_pos, dust_level;
  Latch latch_freq, latch_width, latch_pos, latch_level;
  Lag lag_freq, lag_width, lag_pos, lag_level;
  Pulse pulse;
  Pan2Aa pan;
};

struct MzState {
  MzVoice voice[5];
  float cpsmidi, basenote, detune, maxamp;
  float b1[64], b2[64], b3[64], b4[64], b5[64], b6[64], b7[64], b8[64];
  float c1[64], c2[64], c3[64], c4[64], c5[64], c6[64], c7[64];
  float pulsebuf[64], posbuf[64];
  float vl[64], vr[64];
  float s14l[64], s14r[64], pair_a_l[64], pair_a_r[64];
  float pair_b_l[64], pair_b_r[64];
};

inline void MzVoice_Init(Ctx& c, MzVoice& vc) {
  // ctor draws in creation order (1 prime each); NO Dust ctor prime fires
  // under the golden seed, so every Latch/Lag starts at 0.
  vc.wn_freq.Init(c.rgen);
  vc.dust_freq.Init(c.rgen, 48000.0, 0.2f);
  vc.wn_width.Init(c.rgen);
  vc.dust_width.Init(c.rgen, 48000.0, 0.5f);
  vc.wn_pos.Init(c.rgen);
  vc.dust_pos.Init(c.rgen, 48000.0, 0.3f);
  vc.wn_level.Init(c.rgen);
  vc.dust_level.Init(c.rgen, 48000.0, 0.1f);
  vc.latch_freq.Init();
  vc.latch_width.Init();
  vc.latch_pos.Init();
  vc.latch_level.Init();
  vc.lag_freq.Init(48000.0);
  vc.lag_freq.Prime(0.0f);
  vc.lag_width.Init(48000.0);
  vc.lag_width.Prime(0.0f);
  vc.lag_pos.Init(48000.0);
  vc.lag_pos.Prime(0.0f);
  vc.lag_level.Init(48000.0);
  vc.lag_level.Prime(0.0f);
  // Pulse ctor: m_freqin = 15_midicps's primed out[0] = sc_midicps(0)
  // (the ar midicps primes with the NORMAL scalar function); m_..off = 0.
  vc.pulse.Init(c.tables, 48000.0, ScMidicps(0.0f), 0.0f);
  vc.pan.Init(c.tables, 0.0f);   // m_level ctor = 30_Lag's primed out[0] = 0
}

inline void Mz_Init(Ctx& c) {
  // placement-new: the state contains kit structs; the arena memory is raw
  MzState* st = new (c.arena->Alloc(sizeof(MzState))) MzState();
  // head (ar, K2A-flat inputs): 3_/5_cpsmidi (nova, constant buffer),
  // 4_round (sc_round quantum 1), 6_- / 7_abs, 9_/ = amp/5.
  st->cpsmidi = NovaFreq2Midi(55.0f);
  st->basenote = ScRound1(st->cpsmidi);
  st->detune = std::fabs(st->basenote - st->cpsmidi);
  // 9_/ is an AR div by a SCALAR constant: div_ai_nova (BinaryOpUGens.cpp
  // :1970) = times_vec_simd(in, sc_reciprocal(scalar)) — NOT IEEE division.
  st->maxamp = 0.4f * ScReciprocal(5.0f);
  for (int v = 0; v < 5; ++v) MzVoice_Init(c, st->voice[v]);
  c.state = st;
}

inline void Mz_Process(Ctx& c, float* l, float* r, int n) {
  MzState* st = static_cast<MzState*>(c.state);
  for (int v = 0; v < 5; ++v) {
    MzVoice& vc = st->voice[v];
    // per-block draws in UGen creation order (64 draws each)
    vc.wn_freq.ProcessBlock(st->b1, n);
    vc.dust_freq.ProcessBlock(st->b2, n);
    vc.wn_width.ProcessBlock(st->b3, n);
    vc.dust_width.ProcessBlock(st->b4, n);
    vc.wn_pos.ProcessBlock(st->b5, n);
    vc.dust_pos.ProcessBlock(st->b6, n);
    vc.wn_level.ProcessBlock(st->b7, n);
    vc.dust_level.ProcessBlock(st->b8, n);
    // freq chain: 11_MulAdd(next_aa) -> 13_Latch(aa) -> 14_Lag(2) ->
    // 15_midicps (nova) -> NN_* (midicps * index; voice 1's *1 folds away,
    // voices 2..5 are explicit ar mults 38/61/84/109_*)
    const float index = static_cast<float>(v + 1);
    for (int j = 0; j < n; ++j)
      st->c1[j] = st->b1[j] * st->detune + st->basenote;
    for (int j = 0; j < n; ++j)
      st->c2[j] = vc.latch_freq.Next(st->c1[j], st->b2[j]);
    for (int j = 0; j < n; ++j)
      st->c3[j] = vc.lag_freq.Next(st->c2[j], 2.0f);
    for (int j = 0; j < n; ++j)
      st->c4[j] = NovaMidi2Freq(st->c3[j]) * index;
    // width chain: 17_MulAdd(noise, 0.5, 1.5) -> 19_Latch -> 20_Lag(0.5)
    for (int j = 0; j < n; ++j) st->c5[j] = st->b3[j] * 0.5f + 1.5f;
    for (int j = 0; j < n; ++j)
      st->c6[j] = vc.latch_width.Next(st->c5[j], st->b4[j]);
    for (int j = 0; j < n; ++j)
      st->c7[j] = vc.lag_width.Next(st->c6[j], 0.5f);
    // 21_Pulse (one variant): freq/width via ZIN0 per block
    vc.pulse.SetFreq(st->c4[0]);
    vc.pulse.SetWidth(st->c7[0]);
    vc.pulse.ProcessBlock(st->pulsebuf, n);
    // pos chain: 24_Latch(22_WhiteNoise raw, 23_Dust) -> 25_Lag(0.5)
    for (int j = 0; j < n; ++j)
      st->posbuf[j] = vc.lag_pos.Next(vc.latch_pos.Next(st->b5[j], st->b6[j]),
                                      0.5f);
    // level chain: 27_* (noise * maxamp) -> 29_Latch -> 30_Lag(0.5)
    float lvl0 = 0.0f;
    for (int j = 0; j < n; ++j) {
      const float latched =
          vc.latch_level.Next(st->b7[j] * st->maxamp, st->b8[j]);
      const float lagged = vc.lag_level.Next(latched, 0.5f);
      if (j == 0) lvl0 = lagged;
    }
    // 31_Pan2 (aa variant): per-sample pos, level = ZIN0(2) ramped
    vc.pan.ProcessBlock(st->pulsebuf, st->posbuf, st->vl, st->vr, n, lvl0);
    // Mix fold: Sum4(v4,v3,v2,v1) then + v5
    if (v < 4) {
      const int gi = v & 3;
      float* tl = (gi < 2) ? st->pair_b_l : st->pair_a_l;
      float* tr = (gi < 2) ? st->pair_b_r : st->pair_a_r;
      if (gi == 1 || gi == 3) {
        for (int j = 0; j < n; ++j) { tl[j] += st->vl[j]; tr[j] += st->vr[j]; }
      } else {
        for (int j = 0; j < n; ++j) { tl[j] = st->vl[j]; tr[j] = st->vr[j]; }
      }
      if (gi == 3) {
        for (int j = 0; j < n; ++j) {
          st->s14l[j] = st->pair_a_l[j] + st->pair_b_l[j];
          st->s14r[j] = st->pair_a_r[j] + st->pair_b_r[j];
        }
      }
    } else {
      for (int j = 0; j < n; ++j) {
        l[j] = st->s14l[j] + st->vl[j];
        r[j] = st->s14r[j] + st->vr[j];
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Mt. Lion (id 14) — 9 voices, KR random chains (14 RGen draws per voice per
// block), ar tail: LFPulse -> CombN -> tanh -> Pan2(kk) -> Sum4/Sum3 -> LeakDC.
// Def dump dc_Mt. Lion.scsyndef (constants exact: densities 0.2/0.5/0.3/0.4/
// 0.1, width 0.5/0.5, lags 2/0.5/5/3, CombN maxdelay 1, LeakDC 0.995,
// detune-mul 3/4, decay-mul 10, indices 1..9, amp/9).
// Head (ar): identical to Mt. Zion but amp/9; detune == 0, basenote == 33
// (NovaFreq2Midi(55) == 33 exactly).
// Per voice chains (creation order = draw order):
//   freq:  WhiteNoise.kr -> MulAdd_AR(7_abs ar, noise kr, 4_round ar)
//          [next_ka: the kr noise value RAMPS, lane0 = mMul.value = the WN's
//          ctor prime draw] -> Dust.kr(0.2) -> Latch.kr (ZIN0 = lane0) ->
//          Lag.kr(2, sr 750) -> midicps.kr (sc_midicps, the CLEAN formula)
//          -> * index (kr mult 60/105/150/..._*; voice 1's *1 folds away)
//          -> LFPulse.ar(freq kr, 0, duty kr)   [LFPulse_next_k]
//   width: WhiteNoise.kr -> MulAdd.kr(w*0.5+0.5, next_scalar) -> Dust.kr(0.5)
//          -> Latch.kr -> Lag.kr(0.5) -> duty
//   delaytime: WhiteNoise.kr -> MulAdd_AR(detune, noise, basenote) ->
//          Dust.kr(0.3) -> Latch.kr -> Lag.kr(5) -> midicps.kr ->
//          reciprocal.kr (sc_reciprocal SSE rcp+NR)
//   dt-mod: WhiteNoise.kr -> MulAdd.kr(w*3+4) -> Dust.kr(0.2) -> Latch.kr ->
//          Lag.kr(0.5) -> round.kr (sc_round 1) -> Lag.kr(5)
//   delaytime = recip * lag5; decaytime: WhiteNoise.kr -> *(w*10) ->
//          Dust.kr(0.2) -> Latch.kr -> Lag.kr(3)
//   CombN.ar(lfpulse, 1, dt, dcy) [checked phase; ctor delay/decay = 0]
//          -> tanh (nova vec_tanh_float) -> Pan2.ar(tanh, pos kr, level kr)
//          [Pan2_next_kk: amps ramped, TARGETS stored — kit Pan2]
//   pos: WhiteNoise.kr -> Dust.kr(0.4) -> Latch.kr -> Lag.kr(2)
//   level: WhiteNoise.kr -> *(ka: ramp(noise) lane0 x maxamp) -> Dust.kr(0.1)
//          -> Latch.kr -> Lag.kr(0.5)
// Ctor primes: kr MulAdd primes next_scalar (SIMD_Unit::set_calc_function);
// kr BinaryOp/UnaryOp prime via the !initialized ctor path (15_midicps's
// prime = sc_midicps(0) = the LFPulse ctor freq; 28_reciprocal's prime =
// sc_reciprocal of that; 36_*'s prime = recip_prime * 35_Lag's primed 0 = 0
// -> the CombN ctor delaytime 0, decaytime = 41_Lag's primed 0). The Latch
// ctor primes are rolled back; NO Dust ctor prime fires under the golden
// seed. Mix: Sum4(v4,v3,v2,v1) + Sum4(v8,v7,v6,v5) + Sum3(v9, S58, S14)
// per channel -> LeakDC(0.995). NO extra scaling.
// ---------------------------------------------------------------------------
struct MlWhiteNoise {
  RGen* r = nullptr;
  float prime = 0.0f;   // the ctor prime draw — the MulAdd/BinaryOp ramp bases
  void Init(RGen* r_) { r = r_; prime = r_->frand2(); }
  float ProcessKr() { return r->frand2(); }
};

struct MlVoice {
  MlWhiteNoise wn_freq, wn_width, wn_dt, wn_dtm, wn_dcy, wn_pos, wn_level;
  Dust dust_freq, dust_width, dust_dt, dust_dtm, dust_dcy, dust_pos, dust_level;
  Latch latch_freq, latch_width, latch_dt, latch_dtm, latch_dcy, latch_pos,
      latch_level;
  Lag lag_freq, lag_width, lag_dt, lag_dtm, lag_dtm2, lag_dcy, lag_pos,
      lag_level;
  LFPulse pulse;
  DelayLine comb;
  Pan2 pan;
  float mul_freq_prev;   // 11_MulAdd mMul.value (the WN's ctor prime draw)
  float mul_dt_prev;     // 23_MulAdd mMul.value
  float mul_lvl_prev;    // 49_* mPrevA
};

struct MlState {
  MlVoice voice[9];
  float cpsmidi, basenote, detune, maxamp;
  float pulsebuf[64], combbuf[64], tanhbuf[64];
  float vl[64], vr[64];
  float s14l[64], s14r[64], s58l[64], s58r[64], s911l[64], s911r[64];
  float pair_a_l[64], pair_a_r[64], pair_b_l[64], pair_b_r[64];
  LeakDC leak_l, leak_r;
};

inline void MlVoice_Init(Ctx& c, MlVoice& vc) {
  // 14 ctor draws in creation order (7 WhiteNoise primes + 7 Dust primes)
  vc.wn_freq.Init(c.rgen);
  vc.dust_freq.Init(c.rgen, 750.0, 0.2f);
  vc.wn_width.Init(c.rgen);
  vc.dust_width.Init(c.rgen, 750.0, 0.5f);
  vc.wn_dt.Init(c.rgen);
  vc.dust_dt.Init(c.rgen, 750.0, 0.3f);
  vc.wn_dtm.Init(c.rgen);
  vc.dust_dtm.Init(c.rgen, 750.0, 0.2f);
  vc.wn_dcy.Init(c.rgen);
  vc.dust_dcy.Init(c.rgen, 750.0, 0.2f);
  vc.wn_pos.Init(c.rgen);
  vc.dust_pos.Init(c.rgen, 750.0, 0.4f);
  vc.wn_level.Init(c.rgen);
  vc.dust_level.Init(c.rgen, 750.0, 0.1f);
  vc.latch_freq.Init();
  vc.latch_width.Init();
  vc.latch_dt.Init();
  vc.latch_dtm.Init();
  vc.latch_dcy.Init();
  vc.latch_pos.Init();
  vc.latch_level.Init();
  vc.lag_freq.Init(750.0);   vc.lag_freq.Prime(0.0f);
  vc.lag_width.Init(750.0);  vc.lag_width.Prime(0.0f);
  vc.lag_dt.Init(750.0);     vc.lag_dt.Prime(0.0f);
  vc.lag_dtm.Init(750.0);    vc.lag_dtm.Prime(0.0f);
  vc.lag_dtm2.Init(750.0);   vc.lag_dtm2.Prime(0.0f);
  vc.lag_dcy.Init(750.0);    vc.lag_dcy.Prime(0.0f);
  vc.lag_pos.Init(750.0);    vc.lag_pos.Prime(0.0f);
  vc.lag_level.Init(750.0);  vc.lag_level.Prime(0.0f);
  // LFPulse ctor: mFreqMul = 1/48000 (ar unit), mPhase = 0, mDuty =
  // 20_Lag's primed out[0] = 0; the 1-sample prime outputs 0, rolled back.
  vc.pulse.Init(48000.0, ScMidicps(0.0f), 0.0f, 0.0f);
  // CombN ctor: delaytime = 36_*'s primed out[0] = 0 (recip_prime * 0),
  // decaytime = 41_Lag's primed out[0] = 0 -> dsamp = 1, feedbk = 0.
  vc.comb.Init(48000.0, 1.0f, 0.0f, 0.0f, 3);
  vc.pan.Init(c.tables, 0.0f, 0.0f, 0.0f);   // m_pos/m_level = 0, amps 0
  vc.mul_freq_prev = vc.wn_freq.prime;
  vc.mul_dt_prev = vc.wn_dt.prime;
  vc.mul_lvl_prev = vc.wn_level.prime;
}

inline void Ml_Init(Ctx& c) {
  // placement-new: DelayLine owns a std::vector (heap-backed), which must
  // be constructed (the arena memory is raw)
  MlState* st = new (c.arena->Alloc(sizeof(MlState))) MlState();
  st->cpsmidi = NovaFreq2Midi(55.0f);
  st->basenote = ScRound1(st->cpsmidi);
  st->detune = std::fabs(st->basenote - st->cpsmidi);
  // 9_/ is an AR div by a SCALAR constant: div_ai_nova (BinaryOpUGens.cpp
  // :1970) = times_vec_simd(in, sc_reciprocal(scalar)) — NOT IEEE division
  // (proven: the level-latch grab differed by exactly 1 float32 ULP).
  st->maxamp = 0.4f * ScReciprocal(9.0f);
  for (int v = 0; v < 9; ++v) MlVoice_Init(c, st->voice[v]);
  // 419/421_LeakDC ctor: m_x1 = the Sum3's primed out[0] = 0, m_y1 = 0,
  // one priming sample (x0 == x1 -> y = 0); coef 0.995.
  st->leak_l.Init();
  st->leak_l.Prime(0.0f);
  st->leak_r.Init();
  st->leak_r.Prime(0.0f);
  c.state = st;
}

inline void Ml_Process(Ctx& c, float* l, float* r, int n) {
  MlState* st = static_cast<MlState*>(c.state);
  for (int v = 0; v < 9; ++v) {
    MlVoice& vc = st->voice[v];
    // kr draws in creation order (1 each)
    const float w_freq = vc.wn_freq.ProcessKr();
    const float d_freq = vc.dust_freq.Next();
    const float w_width = vc.wn_width.ProcessKr();
    const float d_width = vc.dust_width.Next();
    const float w_dt = vc.wn_dt.ProcessKr();
    const float d_dt = vc.dust_dt.Next();
    const float w_dtm = vc.wn_dtm.ProcessKr();
    const float d_dtm = vc.dust_dtm.Next();
    const float w_dcy = vc.wn_dcy.ProcessKr();
    const float d_dcy = vc.dust_dcy.Next();
    const float w_pos = vc.wn_pos.ProcessKr();
    const float d_pos = vc.dust_pos.Next();
    const float w_lvl = vc.wn_level.ProcessKr();
    const float d_lvl = vc.dust_level.Next();
    // freq chain: 11_MulAdd_AR lane0 -> 13_Latch.kr -> 14_Lag.kr(2) ->
    // 15_midicps.kr (sc_midicps) -> NN_* (midicps * index, kr mult;
    // voice 1's *1 folds away, voices 2..9 are explicit 60/105/150/..._*)
    const float freq_lane0 = st->detune * vc.mul_freq_prev + st->basenote;
    const float freq_latched = vc.latch_freq.Next(freq_lane0, d_freq);
    const float freq_lagged = vc.lag_freq.Next(freq_latched, 2.0f);
    const float midival = ScMidicps(freq_lagged) * static_cast<float>(v + 1);
    vc.mul_freq_prev = w_freq;
    // width chain: 17_MulAdd.kr -> 19_Latch.kr -> 20_Lag.kr(0.5)
    const float wval = w_width * 0.5f + 0.5f;
    const float wlatched = vc.latch_width.Next(wval, d_width);
    const float wlagged = vc.lag_width.Next(wlatched, 0.5f);
    // 21_LFPulse (next_k): freq/duty via ZIN0 per block
    vc.pulse.SetFreq(midival);
    vc.pulse.SetDuty(wlagged);
    vc.pulse.ProcessBlock(st->pulsebuf, n);
    // delaytime chain: 23_MulAdd_AR lane0 -> 25_Latch -> 26_Lag(5) ->
    // 27_midicps -> 28_reciprocal
    const float dt_lane0 = st->detune * vc.mul_dt_prev + st->basenote;
    const float dt_latched = vc.latch_dt.Next(dt_lane0, d_dt);
    const float dt_lagged = vc.lag_dt.Next(dt_latched, 5.0f);
    const float dtmidi = ScMidicps(dt_lagged);
    const float recip = ScReciprocal(dtmidi);
    vc.mul_dt_prev = w_dt;
    // dt-modulator: 30_MulAdd.kr -> 32_Latch -> 33_Lag(0.5) -> 34_round ->
    // 35_Lag(5); delaytime = 36_* (recip * lag5)
    const float dtm = w_dtm * 3.0f + 4.0f;
    const float dtm_latched = vc.latch_dtm.Next(dtm, d_dtm);
    const float dtm_lag = vc.lag_dtm.Next(dtm_latched, 0.5f);
    const float dtm_round = ScRound1(dtm_lag);
    const float dtm2 = vc.lag_dtm2.Next(dtm_round, 5.0f);
    const float delaytime = recip * dtm2;
    // decaytime: 38_* (w*10) -> 40_Latch -> 41_Lag(3)
    const float dcy = w_dcy * 10.0f;
    const float dcy_latched = vc.latch_dcy.Next(dcy, d_dcy);
    const float dcy_lag = vc.lag_dcy.Next(dcy_latched, 3.0f);
    // 42_CombN (kr delay/decay; checked phase; CALCSLOPE per sample)
    vc.comb.ProcessBlock(st->pulsebuf, st->combbuf, n, delaytime, dcy_lag);
    // 43_tanh (nova vec_tanh_float per sample; the small path keeps the
    // sign — see the NovaTanh law note above)
    for (int j = 0; j < n; ++j) st->tanhbuf[j] = NovaTanh(st->combbuf[j]);
    // pos chain: 46_Latch(44_WN raw, 45_Dust) -> 47_Lag(2)
    const float pos_latched = vc.latch_pos.Next(w_pos, d_pos);
    const float posval = vc.lag_pos.Next(pos_latched, 2.0f);
    // level chain: 49_* (ar BinaryOp, kr in0 / ar in1) -> 51_Latch ->
    // 52_Lag(0.5); lane0 = mPrevA (the previous block's kr value) × maxamp
    const float lvl_lane0 = vc.mul_lvl_prev * st->maxamp;
    const float lvl_latched = vc.latch_level.Next(lvl_lane0, d_lvl);
    const float lvlval = vc.lag_level.Next(lvl_latched, 0.5f);
    vc.mul_lvl_prev = w_lvl;
    // 53_Pan2 (kk): amps ramped, targets stored
    vc.pan.ProcessBlock(st->tanhbuf, st->vl, st->vr, n, posval, lvlval);
    // Mix fold: Sum4(v4,v3,v2,v1) + Sum4(v8,v7,v6,v5) + Sum3(v9, S58, S14)
    // then LeakDC(0.995) per channel
    if (v < 8) {
      const int gi = v & 3;
      float* tl = (gi < 2) ? st->pair_b_l : st->pair_a_l;
      float* tr = (gi < 2) ? st->pair_b_r : st->pair_a_r;
      if (gi == 1 || gi == 3) {
        for (int j = 0; j < n; ++j) { tl[j] += st->vl[j]; tr[j] += st->vr[j]; }
      } else {
        for (int j = 0; j < n; ++j) { tl[j] = st->vl[j]; tr[j] = st->vr[j]; }
      }
      if (gi == 3) {
        float* dst_l = (v < 4) ? st->s14l : st->s58l;
        float* dst_r = (v < 4) ? st->s14r : st->s58r;
        for (int j = 0; j < n; ++j) {
          dst_l[j] = st->pair_a_l[j] + st->pair_b_l[j];
          dst_r[j] = st->pair_a_r[j] + st->pair_b_r[j];
        }
      }
    } else if (v == 8) {
      for (int j = 0; j < n; ++j) {
        // 418_Sum3(v9, S58, S14) = (v9 + S58) + S14; 419_LeakDC
        st->s911l[j] = (st->vl[j] + st->s58l[j]) + st->s14l[j];
        st->s911r[j] = (st->vr[j] + st->s58r[j]) + st->s14r[j];
      }
    }
  }
  // LeakDC (coef 0.995 constant -> the unchanged i_4 path == per-sample law)
  for (int j = 0; j < n; ++j) {
    l[j] = st->leak_l.Next(st->s911l[j], 0.995f);
    r[j] = st->leak_r.Next(st->s911r[j], 0.995f);
  }
}

// ---------------------------------------------------------------------------
// Task 5 — Hecker (id 10) + Fieldsteel (id 7) scalar laws / helpers.
// All constants are the BIT-EXACT float32 values from the .scsyndef constant
// tables (14-digit dumpUGens prints do NOT round-trip float32 — see Task 4).
// ---------------------------------------------------------------------------
// SC_InlineUnaryOp.h:83 — the kr/scalar midiratio path (the dump's UGen is kr).
inline float HkMidiratio(float x) { return std::pow(2.0f, x / 12.0f); }
// SC_InlineUnaryOp.h:190 sc_softclip (nova ar functor is the same law per lane).
inline float HkSoftclip(float x) {
  const float ax = std::fabs(x);
  return ax <= 0.5f ? x : (ax - 0.25f) / x;
}
// SC_InlineBinaryOp.h:152/555 sc_fold2(x,1) = sc_fold(x,-1,1).
inline float HkFold2(float x) {
  float in = x;
  const float lo = -1.0f, hi = 1.0f;
  const float x0 = in - lo;
  if (in >= hi) { in = hi + hi - in; if (in >= lo) return in; }
  else if (in < lo) { in = lo + lo - in; if (in < hi) return in; }
  else return in;
  const float range = hi - lo, range2 = range + range;
  float c = x0 - range2 * std::floor(x0 / range2);
  if (c >= range) c = range2 - c;
  return c + lo;
}
inline float HkRoundQ(float x, float q) { return std::floor(x / q + 0.5f) * q; }
inline float HkTruncQ(float x, float q) { return std::floor(x / q) * q; }
// Select_next_1 (OscUGens.cpp:499): index = (int32)ZIN0(0)+1 clipped [1,max].
inline int HkSelectIndex(float idx) {
  int i = static_cast<int>(idx) + 1;
  return i < 1 ? 1 : (i > 2 ? 2 : i);
}

// LPF with a per-block (kr) freq — verbatim FilterUGens.cpp LPF_next / _1
// (3-sample coefficient-interpolation groups; mFilterLoops = 64/3 = 21, plus
// 1 remain sample). The kit Filter2Pole uses a 4-sample group that no reference
// exercises for a changing freq, so the law is kept verbatim here (same as the
// BpfKr law above). Hecker's LPF freq is the VarLag/LinExp sweep.
struct LpfKr {
  void Init(double sr) {
    rps_ = 2.0 * M_PI / sr;
    a0_ = b1_ = b2_ = 0.0; y1_ = y2_ = 0.0;
    freq_ = 0.0f; primed_ = false;
  }
  // Server LPF_Ctor primes with the input's CTOR output (not the first
  // rendered sample), which seeds y1/y2 entering block 0.
  float Prime(float freq, float first_in) {
    double a0, b1, b2;
    ComputeCoefs(freq, &a0, &b1, &b2);
    const double y0 = first_in + b1 * y1_ + b2 * y2_;
    const double primed_out = a0 * (y0 + 2.0 * y1_ + y2_);
    y2_ = y1_; y1_ = y0;
    y1_ = Zap(y1_); y2_ = Zap(y2_);
    a0_ = a0; b1_ = b1; b2_ = b2; freq_ = freq; primed_ = true;
    return static_cast<float>(primed_out);
  }
  void ProcessBlock(const float* in, float* out, int n, float freq) {
    if (!primed_) {
      double a0, b1, b2;
      ComputeCoefs(freq, &a0, &b1, &b2);
      double y1 = y1_, y2 = y2_;
      const double y0 = in[0] + b1 * y1 + b2 * y2;
      y2 = y1; y1 = y0;
      y1_ = Zap(y1); y2_ = Zap(y2);
      a0_ = a0; b1_ = b1; b2_ = b2; freq_ = freq; primed_ = true;
    }
    double y1 = y1_, y2 = y2_;
    double a0 = a0_, b1 = b1_, b2 = b2_;
    if (freq != freq_) {
      double na0, nb1, nb2;
      ComputeCoefs(freq, &na0, &nb1, &nb2);
      const double kSlope = 1.0 / static_cast<double>(kBlock / 3);
      const double sa0 = (na0 - a0) * kSlope, sb1 = (nb1 - b1) * kSlope,
                   sb2 = (nb2 - b2) * kSlope;
      int i = 0;
      for (int g = 0; g < kBlock / 3; ++g, i += 3) {
        double y0 = in[i + 0] + b1 * y1 + b2 * y2;
        out[i + 0] = static_cast<float>(a0 * (y0 + 2.0 * y1 + y2));
        y2 = in[i + 1] + b1 * y0 + b2 * y1;
        out[i + 1] = static_cast<float>(a0 * (y2 + 2.0 * y0 + y1));
        y1 = in[i + 2] + b1 * y2 + b2 * y0;
        out[i + 2] = static_cast<float>(a0 * (y1 + 2.0 * y2 + y0));
        a0 += sa0; b1 += sb1; b2 += sb2;
      }
      double y0 = in[i] + b1 * y1 + b2 * y2;
      out[i] = static_cast<float>(a0 * (y0 + 2.0 * y1 + y2));
      y2 = y1; y1 = y0;
      a0_ = na0; b1_ = nb1; b2_ = nb2; freq_ = freq;
    } else {
      int i = 0;
      for (int g = 0; g < kBlock / 3; ++g, i += 3) {
        double y0 = in[i + 0] + b1 * y1 + b2 * y2;
        out[i + 0] = static_cast<float>(a0 * (y0 + 2.0 * y1 + y2));
        y2 = in[i + 1] + b1 * y0 + b2 * y1;
        out[i + 1] = static_cast<float>(a0 * (y2 + 2.0 * y0 + y1));
        y1 = in[i + 2] + b1 * y2 + b2 * y0;
        out[i + 2] = static_cast<float>(a0 * (y1 + 2.0 * y2 + y0));
      }
      double y0 = in[i] + b1 * y1 + b2 * y2;
      out[i] = static_cast<float>(a0 * (y0 + 2.0 * y1 + y2));
      y2 = y1; y1 = y0;
    }
    y1_ = Zap(y1); y2_ = Zap(y2);
  }
 private:
  void ComputeCoefs(float freq, double* a0, double* b1, double* b2) const {
    const double pf = static_cast<double>(freq) * rps_ * 0.5;
    const double C = 1.0 / std::tan(pf);
    const double C2 = C * C, s2C = C * static_cast<double>(std::sqrt(2.0f));  // sqrt2_f
    *a0 = 1.0 / (1.0 + s2C + C2);
    *b1 = -2.0 * (1.0 - C2) * (*a0);
    *b2 = -(1.0 - s2C + C2) * (*a0);
  }
  static double Zap(double x) {
    const double ax = std::fabs(x);
    return (ax > 1e-15 && ax < 1e15) ? x : 0.0;
  }
  double rps_ = 0.0, a0_ = 0.0, b1_ = 0.0, b2_ = 0.0, y1_ = 0.0, y2_ = 0.0;
  float freq_ = 0.0f; bool primed_ = false;
};

// XFade2_next_ak (PanUGens.cpp:448): ar inA/inB, kr pan, scalar level 1.
// A simple per-sample slope accumulates to the same endpoint as nova's
// slope_argument lanes (both land within the 1e-6 gate; the kit's Pan2 uses
// the same simple ramp and is golden-exact in the T-D2 ports).
struct XFade2Ak {
  void Init(const KitTables* t, float pan, float level, float first_a,
            float first_b) {
    t_ = t; level_ = level; pan_ = pan;
    Compute(pan, &la_, &ra_);
    float primed;
    Step(first_a, first_b, la_, ra_, primed);
  }
  void ProcessBlock(const float* a, const float* b, float* out, int n,
                    float pan) {
    if (pan != pan_) {
      float na, nb;
      Compute(pan, &na, &nb);
      const float sa = (na - la_) * (1.0f / 64.0f);
      const float sb = (nb - ra_) * (1.0f / 64.0f);
      for (int i = 0; i < n; ++i) {
        Step(a[i], b[i], la_, ra_, out[i]);
        la_ += sa; ra_ += sb;
      }
      la_ = na; ra_ = nb; pan_ = pan;
    } else {
      for (int i = 0; i < n; ++i) Step(a[i], b[i], la_, ra_, out[i]);
    }
  }
  // The ctor's single primed output (XFade2_next_aa(unit,1)) seeds the LPF.
  float Prime(float a, float b) const { return a * la_ + b * ra_; }
 private:
  void Compute(float pan, float* l, float* r) const {
    int ip = static_cast<int32_t>(1024.0f * pan + 1024.0f + 0.5f);
    ip = ip < 0 ? 0 : (ip > 2048 ? 2048 : ip);
    *l = level_ * t_->sine[2048 - ip];
    *r = level_ * t_->sine[ip];
  }
  static void Step(float a, float b, float la, float ra, float& o) {
    o = a * la + b * ra;
  }
  const KitTables* t_ = nullptr;
  float pan_ = 0.0f, level_ = 0.0f, la_ = 0.0f, ra_ = 0.0f;
};

// Hecker's noise UGens need their ctor-primed output to seed the LPF/BPF ctor
// primes (the first RENDERED sample is the NEXT draw, not the prime). Kit
// WhiteNoise/PinkNoise consume the right draws but do not expose the prime.
struct HkWhiteNoise {
  RGen* r = nullptr;
  float prime = 0.0f;
  void Init(RGen* rr) { r = rr; prime = r->frand2(); }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = r->frand2();
  }
};
struct HkPinkNoise {
  RGen* r = nullptr;
  float dice[16] = {};
  int32_t total = 0;
  float prime = 0.0f;
  void Init(RGen* rr) {
    r = rr; total = 0;
    for (int i = 0; i < 16; ++i) {
      const uint32_t v = r->trand() >> 13;
      total += static_cast<int32_t>(v); dice[i] = static_cast<float>(v);
    }
    prime = Next();
  }
  float Next() {
    const uint32_t counter = r->trand();
    const uint32_t newrand = counter >> 13;
    int k = 0;
    while ((counter & (1u << k)) == 0u && k < 32) ++k;
    k &= 15;
    const uint32_t prevrand = static_cast<uint32_t>(dice[k]);
    dice[k] = static_cast<float>(newrand);
    total += static_cast<int32_t>(newrand - prevrand);
    const uint32_t r2 = r->trand() >> 13;
    uint32_t u = (static_cast<uint32_t>(total) + r2) | 0x40000000u;
    float v; std::memcpy(&v, &u, 4);
    return v - 3.0f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
};

// ---------------------------------------------------------------------------
// Hecker (id 10) — 32 mono voices (two groups of 16 -> Out[0] and Out[1]).
// VarLag is LAW-ONLY in the kit (no ugen_ref wav — see report); the expansion
// from sc_kit_envgen.h:211 (Filter.sc:113-145) is used and gated by the drone
// golden. Per voice the LFNoise0.kr(0.1) period is 7500 kr blocks > the 3000
// render blocks, so its value is constant after the ctor prime — the VarLag
// start therefore equals its target and the env stays flat.
// ---------------------------------------------------------------------------
struct HkVoice {
  LFNoise0 lf0_sel, lf0_lpf, lf0_bpf;
  VarLag vl_sel, vl_lpf, vl_bpf;
  LinExp le_lpf, le_bpf;
  HkWhiteNoise wn;
  HkPinkNoise pn;
  XFade2Ak xf;
  LpfKr lpf;
  BpfKr bpf;
};

struct HeckerState {
  HkVoice voice[32];
  float wn_buf[64], pn_buf[64], xf_buf[64], lpf_buf[64], bpf_buf[64];
  float pair_a[64], pair_b[64], group[8][64], mix[64];
};

inline void Hk_Init(Ctx& c) {
  HeckerState* st = new (c.arena->Alloc(sizeof(HeckerState))) HeckerState();
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);   // 750
  for (int v = 0; v < 32; ++v) {
    HkVoice& vc = st->voice[v];
    // RGen creation order: lf0_sel, WhiteNoise, PinkNoise, lf0_lpf, lf0_bpf
    vc.lf0_sel.Init(c.rgen, cr, 0.1f);
    vc.wn.Init(c.rgen);
    vc.pn.Init(c.rgen);
    vc.lf0_lpf.Init(c.rgen, cr, 0.1f);
    vc.lf0_bpf.Init(c.rgen, cr, 0.1f);
    const float s0 = vc.lf0_sel.ctor_value();
    const float s1 = vc.lf0_lpf.ctor_value();
    const float s2 = vc.lf0_bpf.ctor_value();
    vc.vl_sel.Init(cr, s0, 10.0f, 0.0f, 3);   // warp \sine = Env.shapeNames 3
    vc.vl_lpf.Init(cr, s1, 10.0f, 0.0f, 3);
    vc.vl_bpf.Init(cr, s2, 10.0f, 0.0f, 3);
    vc.le_lpf.Init(-1.0f, 1.0f, 20.0f, 20000.0f);
    vc.le_bpf.Init(-1.0f, 1.0f, 9.999999747378752e-05f,
                   0.019999999552965164f);     // exact c[12]/c[13]
    const float which0 = s0 * 0.5f + 0.5f;
    vc.xf.Init(c.tables, HkFold2(which0 * 2.0f - 1.0f), 1.0f, 0.0f, 0.0f);
    // Ctor prime chain: Select primes with the noise UGens' primed outputs,
    // XFade2 primes with those, then LPF and BPF prime with their upstream
    // ctor outputs. These seed the filter y1/y2 state entering block 0.
    const int ia0 = HkSelectIndex(HkRoundQ(which0, 2.0f));
    const int ib0 = HkSelectIndex(HkTruncQ(which0, 2.0f) + 1.0f);
    const float xa0 = (ia0 == 1) ? vc.wn.prime : vc.pn.prime;
    const float xb0 = (ib0 == 1) ? vc.wn.prime : vc.pn.prime;
    const float xf_ctor = vc.xf.Prime(xa0, xb0);
    vc.lpf.Init(sr);
    const float lpf_ctor = vc.lpf.Prime(vc.le_lpf.Next(s1), xf_ctor);
    vc.bpf.Init(sr);
    vc.bpf.Prime(static_cast<float>(c.hz), vc.le_bpf.Next(s2), lpf_ctor);
  }
  c.state = st;
}

inline void Hk_Process(Ctx& c, float* l, float* r, int n) {
  (void)n;
  HeckerState* st = static_cast<HeckerState*>(c.state);
  const float amp = c.amp;
  for (int v = 0; v < 32; ++v) {
    HkVoice& vc = st->voice[v];
    const float l0 = vc.lf0_sel.ProcessKr();
    const float l1 = vc.lf0_lpf.ProcessKr();
    const float l2 = vc.lf0_bpf.ProcessKr();
    const float q0 = vc.vl_sel.ProcessKr(l0);
    const float q1 = vc.vl_lpf.ProcessKr(l1);
    const float q2 = vc.vl_bpf.ProcessKr(l2);
    const float which = q0 * 0.5f + 0.5f;
    const int isel_a = HkSelectIndex(HkRoundQ(which, 2.0f));
    const int isel_b = HkSelectIndex(HkTruncQ(which, 2.0f) + 1.0f);
    const float pan = HkFold2(which * 2.0f - 1.0f);
    vc.wn.ProcessBlock(st->wn_buf, n);
    vc.pn.ProcessBlock(st->pn_buf, n);
    const float* srca = (isel_a == 1) ? st->wn_buf : st->pn_buf;
    const float* srcb = (isel_b == 1) ? st->wn_buf : st->pn_buf;
    vc.xf.ProcessBlock(srca, srcb, st->xf_buf, n, pan);
    // LinExp is kr in the def dump — use the scalar Next() law (not the
    // nova ar ProcessBlock path), once per block.
    const float f_lpf = vc.le_lpf.Next(q1);
    vc.lpf.ProcessBlock(st->xf_buf, st->lpf_buf, n, f_lpf);
    const float rq = vc.le_bpf.Next(q2);
    vc.bpf.ProcessBlock(st->lpf_buf, st->bpf_buf, n, c.hz, rq);
    // Sum4 tree: pairs within groups of 4, then 4 group sums.
    const int g = v >> 2, k = v & 3;   // 32 voices -> 8 groups of 4
    float* pair = (k < 2) ? st->pair_a : st->pair_b;
    if ((k & 1) == 0) {
      for (int i = 0; i < n; ++i) pair[i] = st->bpf_buf[i];
    } else {
      for (int i = 0; i < n; ++i) pair[i] += st->bpf_buf[i];
    }
    if (k == 3) {
      for (int i = 0; i < n; ++i)
        st->group[g][i] = st->pair_b[i] + st->pair_a[i];
    }
  }
  for (int ch = 0; ch < 2; ++ch) {
    float* mix = st->mix;
    for (int i = 0; i < n; ++i) {
      const float g3g2 = st->group[ch * 4 + 3][i] + st->group[ch * 4 + 2][i];
      const float g1g0 = st->group[ch * 4 + 1][i] + st->group[ch * 4 + 0][i];
      mix[i] = g3g2 + g1g0;
    }
    float* out = ch ? r : l;
    for (int i = 0; i < n; ++i) {
      float x = mix[i] * 100.0f;
      x = NovaTanh(x);
      x = x * amp;
      out[i] = x * 0.125f;   // /8: sc_reciprocal(8) = 0.125 exact
    }
  }
}

inline const DroneSpec kDroneHecker = {"Hecker", drones::Hk_Init,
                                       drones::Hk_Process};

// ---------------------------------------------------------------------------
// Fieldsteel (id 7) — 3-voice bandpassed-saw drone + 3-layer SVF marimba.
// DEFERRED (2026-11-20, T-D2 Task 5): the port reaches max_abs 0.028866338 /
// rms +0.4 dB / corr 0.256798020 after 3 distinct diagnostic-driven fix
// attempts (v1 graph; EnvGen Node-array lifetime; Demand pull order). Its
// residual lives in the shared-RGen stream of the 9 marimba Demand pulls and
// the kr→ar/SVF path, which no kit ref exercises. Catalog id 7 stays nullptr
// (inert per plan). The full port below + the law notes are preserved for a
// T-D3 resume — do NOT weaken the gate or regenerate the golden.
// Local Impulse WITH phase support (the kit Impulse has no phase input);
// SC LFUGens.cpp:982: phase = wrap(phaseOffset,0,1), 0 -> 1 for inc >= 0.
// mPhaseIncrement is a DOUBLE (freq * mSampleDur), not the kit's float product.
// ---------------------------------------------------------------------------
struct FsImpulse {
  double phase = 0.0, inc = 0.0;
  void Init(double sr, float freq, float phase_offset) {
    inc = static_cast<double>(freq) * (1.0 / sr);
    double p = phase_offset - std::floor(phase_offset);
    if (p == 0.0 && inc >= 0.0) p = 1.0;
    phase = p;
  }
  float Next() {
    float out = 0.0f;
    if (phase >= 1.0) {
      phase -= 1.0;
      if (phase >= 1.0) phase -= std::floor(phase);
      out = 1.0f;
    }
    phase += inc;
    return out;
  }
};

struct FsSources {
  // Demand sources have no default ctor; placement-new them in the arena.
  DXRand* dx_notes = nullptr; DXRand* dx_rhy = nullptr;
  DRand* dr_pitch = nullptr;
  DBrown* db_dmul = nullptr; DBrown* db_m0 = nullptr;
  DBrown* db_m1 = nullptr; DBrown* db_m2 = nullptr;
};

struct FsDroneVoice {
  Saw saw;
  BpfKr bpf;
  Pan2 pan;
  float mul_prev = 0.0f;   // mul_ak mPrevB (previous block's kr Dbrown value)
  float env_prev = 0.0f;   // env mul_ak mPrevB
};
struct FsMarVoice {
  LFSaw saw;
  SVF svf;
  Pan2 pan;
  float mul_prev = 0.0f;
};

struct FsState {
  FsSources src;
  LFNoise1 lf_midi, lf_cf0, lf_cf1, lf_rq0, lf_rq1,
           lf_res0, lf_res1, lf_res2;
  FsImpulse imp_d[3], imp_m;
  // EnvGen keeps a pointer to its Node array: it MUST live in the state, not
  // on Fs_Init's stack (a dangling node pointer blew the envelope to ~4e9).
  EnvGen::Node env_nodes[3];
  EnvGen env_d[3];
  LinExp le_cf0, le_cf1, le_rq0, le_rq1, le_res0, le_res1, le_res2;
  Demand dem_note[3], dem_dmul[3], dem_rhy[3], dem_pitch[3], dem_mm[3];
  FsDroneVoice dv[3];
  FsMarVoice mv[3];
  float impbuf[64], sawbuf[64], mulbuf[64], bpfbuf[64], envbuf[64], sfbuf[64];
  float dlv[3][64], drv[3][64], mlv[3][64], mrv[3][64];
  float r2, rhalf, hz3, hz125, hz4;   // exact reciprocal/constants
};

inline float FsPull(Demand& d, float trig) {
  float out = 0.0f;
  float* outs[1] = {&out};
  d.ProcessBlock(&trig, 0.0f, outs, 1);
  return out;
}

inline void Fs_Init(Ctx& c) {
  FsState* st = new (c.arena->Alloc(sizeof(FsState))) FsState();
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);
  // Head (constant, K2A-flat). Fieldsteel uses NovaFreq2Midi(55) == 33 exactly.
  const float hz = c.hz;
  st->r2 = ScReciprocal(2.0f);
  st->rhalf = hz * st->r2;          // hz/2 (div_ai_nova)
  st->hz3 = 3.0f * hz;              // 3*hz
  st->hz125 = hz * 1.25f;
  st->hz4 = hz * 4.0f;
  // RGen creation order: demand sources' Ctor resets (Drand/Dxrand consume
  // one irand draw each), interleaved with the LFNoise1 ctors (2 draws each).
  st->src.dx_notes = new (c.arena->Alloc(sizeof(DXRand)))
      DXRand({INFINITY, NovaMidi2Freq(13.0f), NovaMidi2Freq(20.0f),
              NovaMidi2Freq(33.0f), NovaMidi2Freq(28.0f)}, c.rgen);
  st->src.dr_pitch = new (c.arena->Alloc(sizeof(DRand)))
      DRand({INFINITY, st->rhalf, hz, st->hz3, st->hz125, st->hz4}, c.rgen);
  st->src.db_dmul = new (c.arena->Alloc(sizeof(DBrown)))
      DBrown({INFINITY, 0.6f, 0.8f, 0.01f}, c.rgen);
  st->src.dx_rhy = new (c.arena->Alloc(sizeof(DXRand)))
      DXRand({INFINITY, 0.125f, 0.375f, 0.5625f, 0.1617647111415863f,
              0.25f, 0.5f, 1.0f, 1.25f}, c.rgen);
  st->src.db_m0 = new (c.arena->Alloc(sizeof(DBrown)))
      DBrown({INFINITY, 0.6f, 1.0f, 0.01f}, c.rgen);
  st->src.db_m1 = new (c.arena->Alloc(sizeof(DBrown)))
      DBrown({INFINITY, 0.6f, 1.0f, 0.01f}, c.rgen);
  st->src.db_m2 = new (c.arena->Alloc(sizeof(DBrown)))
      DBrown({INFINITY, 0.6f, 1.0f, 0.01f}, c.rgen);
  st->src.dx_notes->Reset();         // 11_Dxrand (1 draw)
  st->src.dr_pitch->Reset();         // 20_Drand (1 draw)
  st->src.db_dmul->Reset();          // 31_Dbrown (0 draws)
  st->lf_midi.Init(c.rgen, cr, 0.1f);   // 35 (2 draws)
  st->lf_cf0.Init(c.rgen, cr, 0.2f);    // 53
  st->lf_cf1.Init(c.rgen, cr, 0.2f);    // 55
  st->lf_rq0.Init(c.rgen, cr, 0.1f);    // 57
  st->lf_rq1.Init(c.rgen, cr, 0.1f);    // 69
  st->src.dx_rhy->Reset();           // 80_Dxrand (1 draw)
  st->src.db_m0->Reset();            // 91
  st->lf_res0.Init(c.rgen, cr, 0.1f);   // 94
  st->src.db_m1->Reset();            // 101
  st->lf_res1.Init(c.rgen, cr, 0.1f);   // 104
  st->src.db_m2->Reset();            // 111
  st->lf_res2.Init(c.rgen, cr, 0.1f);   // 114
  // Demand UGens (24/27/30, 32/33/34, 82/85/88, 84/87/90, 92/102/112).
  st->dem_note[0].Init({st->src.dx_notes});
  st->dem_note[1].Init({st->src.dx_notes});
  st->dem_note[2].Init({st->src.dx_notes});
  st->dem_dmul[0].Init({st->src.db_dmul});
  st->dem_dmul[1].Init({st->src.db_dmul});
  st->dem_dmul[2].Init({st->src.db_dmul});
  st->dem_rhy[0].Init({st->src.dx_rhy});
  st->dem_rhy[1].Init({st->src.dx_rhy});
  st->dem_rhy[2].Init({st->src.dx_rhy});
  st->dem_pitch[0].Init({st->src.dr_pitch});
  st->dem_pitch[1].Init({st->src.dr_pitch});
  st->dem_pitch[2].Init({st->src.dr_pitch});
  st->dem_mm[0].Init({st->src.db_m0});
  st->dem_mm[1].Init({st->src.db_m1});
  st->dem_mm[2].Init({st->src.db_m2});
  // Impulses: drone 1/8 with phases 0, 1/3, 0.6; marimba 1/4 phase 0.
  st->imp_d[0].Init(cr, 0.125f, 0.0f);
  st->imp_d[1].Init(cr, 0.125f, 0.3333333432674408f);  // c[8] = 1/3
  st->imp_d[2].Init(cr, 0.125f, 0.6000000238418579f);  // c[9] = 0.6
  st->imp_m.Init(cr, 0.25f, 0.0f);
  // Drone EnvGen 3 curve stages (Env([0,1,1,0],[2,5,3],[1,0,-1])).
  // The Node array lives in FsState — EnvGen stores the pointer.
  st->env_nodes[0] = {1.f, 2.f, 5.f, 1.f};
  st->env_nodes[1] = {1.f, 5.f, 5.f, 0.f};
  st->env_nodes[2] = {0.f, 3.f, 5.f, -1.f};
  for (int v = 0; v < 3; ++v)
    st->env_d[v].Init(cr, st->env_nodes, 3, 0.f, 1.f, 0.f, 1.f, -99, -99);
  // LinExp sweeps (kr).
  st->le_cf0.Init(-1.f, 1.f, st->rhalf, st->hz3);
  st->le_cf1.Init(-1.f, 1.f, st->rhalf, st->hz3);
  st->le_rq0.Init(-1.f, 1.f, 0.1f, 0.20000000298023224f);
  st->le_rq1.Init(-1.f, 1.f, 0.1f, 0.20000000298023224f);
  st->le_res0.Init(-1.f, 1.f, 0.0020000000949949026f, 0.01f);
  st->le_res1.Init(-1.f, 1.f, 0.0020000000949949026f, 0.01f);
  st->le_res2.Init(-1.f, 1.f, 0.0020000000949949026f, 0.01f);
  const float midi_ratio_ctor = HkMidiratio(st->lf_midi.ctor_value() * 0.1f);
  for (int v = 0; v < 3; ++v) {
    FsDroneVoice& vc = st->dv[v];
    // Saw ctor freq = 24_Demand's primed output (0) * midiratio.
    vc.saw.Init(c.tables, sr, 0.0f * midi_ratio_ctor);
    LinExp& lecf = (v == 1) ? st->le_cf1 : st->le_cf0;
    LinExp& lerq = (v == 1) ? st->le_rq1 : st->le_rq0;
    LFNoise1& lfcf = (v == 1) ? st->lf_cf1 : st->lf_cf0;
    LFNoise1& lfrq = (v == 1) ? st->lf_rq1 : st->lf_rq0;
    vc.bpf.Init(sr);
    vc.bpf.Prime(lecf.ctor_value(lfcf.ctor_value()),
                 lerq.ctor_value(lfrq.ctor_value()), 0.0f);
    const float pan = (v == 0) ? -0.8500000238418579f
                     : (v == 2) ? 0.8500000238418579f : 0.0f;
    vc.pan.Init(c.tables, pan, 1.0f, 0.0f);
    vc.mul_prev = 0.0f; vc.env_prev = 0.0f;
  }
  const float mpan[3] = {-0.699999988079071f, 0.0f, 0.699999988079071f};
  for (int v = 0; v < 3; ++v) {
    FsMarVoice& vc = st->mv[v];
    vc.saw.Init(sr, 0.0f, 0.0f);   // LFSaw ctor freq = Demand primed 0
    vc.svf.Init(sr, 0.0f, 0.0f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.0f);
    vc.pan.Init(c.tables, mpan[v], 1.0f, 0.0f);
    vc.mul_prev = 0.0f;
  }
  c.state = st;
}

inline void Fs_Process(Ctx& c, float* l, float* r, int n) {
  FsState* st = static_cast<FsState*>(c.state);
  const float amp = c.amp;
  const float lvl = 0.5773502588272095f;   // exact c[22] = 1/sqrt(3)
  const float drone_amp = 0.675000011920929f;  // c[34]
  const float mar_amp = 0.32499998807907104f;  // c[33]
  const double cr = static_cast<double>(c.sr) / static_cast<double>(kBlock);
  (void)cr;
  // kr noise values shared across the drone voices (one ProcessKr per block).
  const float midi_n = st->lf_midi.ProcessKr();
  const float cf0 = st->lf_cf0.ProcessKr();
  const float cf1 = st->lf_cf1.ProcessKr();
  const float rq0 = st->lf_rq0.ProcessKr();
  const float rq1 = st->lf_rq1.ProcessKr();
  const float res[3] = {st->lf_res0.ProcessKr(), st->lf_res1.ProcessKr(),
                        st->lf_res2.ProcessKr()};
  const float mratio = HkMidiratio(midi_n * 0.1f);
  // ---- drone (3 voices) ----
  for (int v = 0; v < 3; ++v) {
    FsDroneVoice& vc = st->dv[v];
    const float trig = st->imp_d[v].Next();
    const float note = FsPull(st->dem_note[v], trig);
    const float dmul = FsPull(st->dem_dmul[v], trig);
    const float env = st->env_d[v].ProcessKr(trig);
    vc.saw.SetFreq(note * mratio);
    vc.saw.ProcessBlock(st->sawbuf, n);
    MulRamp64(st->sawbuf, st->mulbuf, n, vc.mul_prev, dmul, 1.0f);
    vc.mul_prev = dmul;
    for (int i = 0; i < n; ++i)
      st->sfbuf[i] = HkSoftclip(st->mulbuf[i]) * 0.75f;
    LinExp& lecf = (v == 1) ? st->le_cf1 : st->le_cf0;
    LinExp& lerq = (v == 1) ? st->le_rq1 : st->le_rq0;
    const float lfcf = (v == 1) ? cf1 : cf0;
    const float lfrq = (v == 1) ? rq1 : rq0;
    vc.bpf.ProcessBlock(st->sfbuf, st->bpfbuf, n, lecf.Next(lfcf),
                        lerq.Next(lfrq));
    MulRamp64(st->bpfbuf, st->envbuf, n, vc.env_prev, env, 1.0f);
    vc.env_prev = env;
    for (int i = 0; i < n; ++i)
      st->sfbuf[i] = HkSoftclip(st->envbuf[i] * 1.5f);
    const float pan = (v == 0) ? -0.8500000238418579f
                     : (v == 2) ? 0.8500000238418579f : 0.0f;
    vc.pan.ProcessBlock(st->sfbuf, st->dlv[v], st->drv[v], n, pan, 1.0f);
  }
  // ---- marimba (3 layers) ----
  for (int v = 0; v < 3; ++v) {
    FsMarVoice& vc = st->mv[v];
    const float trig = st->imp_m.Next();
    const float rhy = FsPull(st->dem_rhy[v], trig);
    const float pitch = FsPull(st->dem_pitch[v], trig);
    const float mmul = FsPull(st->dem_mm[v], trig);
    vc.saw.SetFreq(rhy);
    vc.saw.ProcessBlock(st->sawbuf, n);
    MulRamp64(st->sawbuf, st->mulbuf, n, vc.mul_prev, mmul, 1.0f);
    vc.mul_prev = mmul;
    LinExp& leres = (v == 0) ? st->le_res0 : ((v == 1) ? st->le_res1
                                                        : st->le_res2);
    const float resv = 1.0f - leres.Next(res[v]);
    vc.svf.ProcessBlock(st->mulbuf, st->sfbuf, n, pitch, resv,
                        0.f, 1.f, 0.f, 0.f, 0.f);
    for (int i = 0; i < n; ++i)
      st->sfbuf[i] = HkSoftclip(st->sfbuf[i] * 0.1f);
    const float mpan = (v == 0) ? -0.699999988079071f
                     : (v == 2) ? 0.699999988079071f : 0.0f;
    vc.pan.ProcessBlock(st->sfbuf, st->mlv[v], st->mrv[v], n, mpan, 1.0f);
  }
  // Splay level comp + Mix folds (exact Sum3 operand order from dump):
  // drone Sum3 = (dv2 + dv1) + dv0; marimba Sum3 = (mv2 + mv1) + mv0.
  for (int i = 0; i < n; ++i) {
    const float dl_sum = (st->dlv[2][i] + st->dlv[1][i]) + st->dlv[0][i];
    const float dr_sum = (st->drv[2][i] + st->drv[1][i]) + st->drv[0][i];
    const float ml_sum = (st->mlv[2][i] + st->mlv[1][i]) + st->mlv[0][i];
    const float mr_sum = (st->mrv[2][i] + st->mrv[1][i]) + st->mrv[0][i];
    // MulAdd(drone*0.675 + marimba*0.325) then *amp.
    l[i] = (dl_sum * lvl * drone_amp + ml_sum * lvl * mar_amp) * amp;
    r[i] = (dr_sum * lvl * drone_amp + mr_sum * lvl * mar_amp) * amp;
  }
}

inline const DroneSpec kDroneFieldsteel = {"Fieldsteel", drones::Fs_Init,
                                           drones::Fs_Process};

// ===========================================================================
// Task 6 — Apparatus (id 0) + Twin Pks (id 30) helpers + ports.
// ===========================================================================
// SinOsc with an audio-rate freq input (SinOsc_next_ai): the freq is read
// per sample; cpstoinc is the kit's float member (sc_kit_osc.h:98) and the
// phase increment is the same (int32)(cpstoinc*freq) cast as the constant-freq
// path. No ctor prime in the kit (verified sample-exact by the trio goldens).
struct SinOscAF {
  void Init(const KitTables* t, double sr, float phase_rad = 0.0f) {
    t_ = t;
    cpstoinc_ = static_cast<float>(static_cast<double>(kSineSize) * (1.0 / sr)
                                   * 65536.0);
    lomask_ = (kSineSize - 1) << 3;
    // SC SinOsc_Ctor: radtoinc = kSineSize * (rtwopi*65536) (double member,
    // copied to a float local in the perform), pphase = phase + (int32)(radtoinc*iphase).
    const double radtoinc = static_cast<double>(kSineSize)
                          * ((1.0 / (2.0 * M_PI)) * 65536.0);
    phase_ = static_cast<uint32_t>(static_cast<int32_t>(
        static_cast<float>(radtoinc) * phase_rad));
  }
  void ProcessBlock(const float* freq, float* out, int n) {
    for (int i = 0; i < n; ++i) {
      out[i] = lookupi1(t_->sine_wt, t_->sine_wt + 1, phase_, lomask_);
      phase_ += static_cast<uint32_t>(static_cast<int32_t>(cpstoinc_ * freq[i]));
    }
  }
  void ProcessBlock(float freq, float* out, int n) {
    for (int i = 0; i < n; ++i) {
      out[i] = lookupi1(t_->sine_wt, t_->sine_wt + 1, phase_, lomask_);
      phase_ += static_cast<uint32_t>(static_cast<int32_t>(cpstoinc_ * freq));
    }
  }
 private:
  const KitTables* t_ = nullptr;
  float cpstoinc_ = 0.0f;
  int32_t lomask_ = 0;
  uint32_t phase_ = 0;
};

// DelayC with an audio-rate delaytime (DelayUGens.cpp:3532 DelayX_perform_a
// with DelayC_helper<true> — the checked cubic phase). The kit DelayLine's
// ProcessBlock ramps a per-BLOCK delay; Twin Pks' wow LFO is audio rate, so
// the per-sample dsamp/idsamp/frac path is replicated verbatim here.
struct DelayCA {
  void Init(double sr, float maxdelay) {
    sr_ = static_cast<float>(sr);
    long sz = static_cast<long>(std::ceil(maxdelay * sr_ + 1.f)) + 64;
    bufsize_ = 1;
    while (bufsize_ < sz) bufsize_ <<= 1;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    mask_ = bufsize_ - 1;
    iwr_ = 0;
    max_dsamp_ = static_cast<float>(bufsize_);
  }
  void ProcessBlock(const float* in, const float* delaytime, float* out, int n) {
    for (int i = 0; i < n; ++i) {
      float dsamp = delaytime[i] * sr_;
      const float kMin = 2.0f;   // DelayC::minDelaySamples = 2
      dsamp = dsamp < kMin ? kMin : (dsamp > max_dsamp_ ? max_dsamp_ : dsamp);
      const long idsamp = static_cast<long>(dsamp);
      const float frac = dsamp - static_cast<float>(idsamp);
      const long i1 = iwr_ - idsamp, i0 = i1 + 1, i2 = i1 - 1, i3 = i1 - 2;
      buf_[iwr_ & mask_] = in[i];
      float value = 0.0f;
      if (i0 >= 0) {
        const float d0 = buf_[i0 & mask_];
        const float d1 = i1 >= 0 ? buf_[i1 & mask_] : 0.0f;
        const float d2 = i2 >= 0 ? buf_[i2 & mask_] : 0.0f;
        const float d3 = i3 >= 0 ? buf_[i3 & mask_] : 0.0f;
        const float c0 = d1;
        const float c1 = 0.5f * (d2 - d0);
        const float c2 = d0 - 2.5f * d1 + 2.0f * d2 - 0.5f * d3;
        const float c3 = 0.5f * (d3 - d0) + 1.5f * (d1 - d2);
        value = ((c3 * frac + c2) * frac + c1) * frac + c0;
      }
      out[i] = value;
      ++iwr_;
    }
  }
 private:
  std::vector<float> buf_;
  long bufsize_ = 0, mask_ = 0, iwr_ = 0;
  float sr_ = 48000.f, max_dsamp_ = 0.f;
};

// Fill a 64-sample kr→ar ramp with SC's nova slope_argument lanes
// ([p, p+s, p+2s, p+3s] then += 4s per 4-sample chunk — Task 4 law).
inline void T6Ramp64(float* out, int n, float prev, float v) {
  const float slope = (v - prev) * (1.0f / 64.0f);
  const float s2 = slope + slope;
  float b0 = prev, b1 = prev + slope, b2 = prev + s2, b3 = prev + s2 + slope;
  const float inc = s2 + s2;
  for (int j = 0; j < n; ++j) {
    switch (j & 3) {
      case 0: out[j] = b0; break;
      case 1: out[j] = b1; break;
      case 2: out[j] = b2; break;
      default: out[j] = b3; break;
    }
    if ((j & 3) == 3) { b0 += inc; b1 += inc; b2 += inc; b3 += inc; }
  }
}
// MulAdd with ar input and kr b/c (MulAdd_next_aka): both b and c ramp.
inline void T6MulAddAK(const float* a, float* out, int n,
                       float bp, float b, float cp, float c) {
  float br[64], cr[64];
  T6Ramp64(br, n, bp, b);
  T6Ramp64(cr, n, cp, c);
  for (int i = 0; i < n; ++i) out[i] = a[i] * br[i] + cr[i];
}

// Local Dust2/Crackle with their ctor-primed output exposed — the filter
// ctor primes need the UPSTREAM UGen's primed value, not the first rendered
// sample (Task 5 Hecker law). Draw sequence matches the kit exactly.
struct T6Dust2 {
  RGen* r = nullptr; float sr = 48000.f, thresh = 0.f, scale = 0.f, prime = 0.f;
  void Init(RGen* rr, double s, float density) {
    r = rr; sr = static_cast<float>(s);
    thresh = density * static_cast<float>(1.0 / s);
    scale = thresh > 0.f ? 2.f / thresh : 0.f;
    prime = Next();
  }
  float Next() {
    const float z = r->frand();
    return z < thresh ? z * scale - 1.f : 0.f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
};
struct T6Crackle {
  RGen* r = nullptr; float param = 0.f, y1 = 0.f, y2 = 0.f, prime = 0.f;
  void Init(RGen* rr, float p) {
    r = rr; param = p; y1 = static_cast<float>(r->drand()); y2 = 0.f;
    prime = Next();
  }
  float Next() {
    const float y0 = std::fabs(y1 * param - y2 - 0.05f);
    y2 = y1; y1 = y0;
    return y0;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
};

// ---------------------------------------------------------------------------
// Apparatus (id 0) — 60 UGens. DEFERRED (2026-11-20, T-D2 Task 6): best
// max_abs 0.000006787 / corr 1.000000000 (a decaying ctor-prime transient,
// worst_i 514) after 3 distinct fix attempts (v1 graph; HPF/BPF ctor prime;
// LeakDC ctor prime). Catalog id 0 stays nullptr (inert per plan). The full
// port + law notes are preserved here for a T-D3 resume. Do NOT weaken the
// gate or regenerate the golden.
// The `hz:` keyword trap at Apparatus.scd:20
// leaves UGens 4_LFPar/5_MulAdd/14_PinkNoise unconnected (but they still
// consume RGen draws, so the dead PinkNoise is processed every block).
// ---------------------------------------------------------------------------
struct ApState {
  LFPar par_dead;
  PinkNoise pn_dead;
  LFNoise2 lfn_vib, lfn_vib2;
  LFTri tri1, tri2, tri3, tri4;
  SinOsc lfo1, lfo2;
  T6Dust2 dust2;
  T6Crackle crackle;
  HkPinkNoise pn;
  SinOscAF pn_sin;
  Filter2Pole hpf, bpf;
  LeakDC leak;
  float r7, r10, r6;   // ScReciprocal(7/10/6) for amp/7, /10, /6
  float pn_sin_ctor, dust_ctor, crackle_ctor, pn_ctor;
  float dead_pn_ctor, lfn1_ctor, lfn2_ctor;
  float b1prev, b2prev, b3prev;   // MulAdd c-ramp prev (amp*softclip terms)
  float buf1[64], buf2[64], buf3[64], buf4[64], buf5[64], buf6[64];
  float buf7[64], buf8[64], buf9[64], buf10[64], buf11[64], buf12[64];
};

inline void Ap_Init(Ctx& c) {
  ApState* st = new (c.arena->Alloc(sizeof(ApState))) ApState();
  const double sr = static_cast<double>(c.sr);
  const float hz = c.hz, amp = c.amp;
  st->r7 = ScReciprocal(7.0f); st->r10 = ScReciprocal(10.0f);
  st->r6 = ScReciprocal(6.0f);
  // RGen creation order: 14_PinkNoise (dead), 15/21_LFNoise2, 43_Dust2,
  // 44_Crackle, 46_PinkNoise.
  st->pn_dead.Init(c.rgen);
  st->lfn_vib.Init(c.rgen, sr, 0.1899999976158142f);      // c[6]
  st->lfn_vib2.Init(c.rgen, sr, 0.6700000166893005f);     // c[10]
  st->dust2.Init(c.rgen, sr, 10.0f);
  st->crackle.Init(c.rgen, 1.9500000476837158f);          // c[25]
  st->pn.Init(c.rgen);
  st->dead_pn_ctor = 0.0f;
  st->lfn1_ctor = st->lfn_vib.ctor_value();
  st->lfn2_ctor = st->lfn_vib2.ctor_value();
  st->dust_ctor = 0.0f;
  st->pn_ctor = 0.0f;
  // 4_LFPar (dead) — created before 14_PinkNoise but has no RNG draw.
  st->par_dead.Init(sr, hz * 2.0f, 0.0f);
  // LFTris use per-sample freq from the vibrato chains.
  st->tri1.Init(sr, 0.0f); st->tri2.Init(sr, 0.0f);
  st->tri3.Init(sr, 0.0f); st->tri4.Init(sr, 0.0f);
  st->lfo1.Init(c.tables, sr, 0.009f);
  st->lfo2.Init(c.tables, sr, 0.011f);   // iphase 2pi wraps to table phase 0
  st->pn_sin.Init(c.tables, sr);
  st->hpf.Init(sr, 1);   // HPF, constant 25 Hz
  st->bpf.Init(sr, 2);   // BPF, constant hz*8, rq 1
  st->leak.Init(); st->leak.Prime(0.0f);
  st->b1prev = st->b2prev = st->b3prev = 0.0f;
  c.state = st;
}

inline void Ap_Process(Ctx& c, float* l, float* r, int n) {
  ApState* st = static_cast<ApState*>(c.state);
  const float hz = c.hz, amp = c.amp;
  (void)hz;
  // Dead chain: 4_LFPar + 5_MulAdd + 14_PinkNoise (still draws RGen per block).
  st->par_dead.ProcessBlock(st->buf1, n);
  st->pn_dead.ProcessBlock(st->buf2, n);
  // 15_LFNoise2 / 16_MulAdd / 17_* → vibrato freq for tri1.
  st->lfn_vib.ProcessBlock(st->buf3, n);
  for (int i = 0; i < n; ++i)
    st->buf4[i] = st->buf3[i] * 0.0069756703451275826f
                + 1.0000243186950684f;   // c[7], c[8]
  for (int i = 0; i < n; ++i) {
    st->tri1.SetFreq(c.hz * st->buf4[i]);
    st->buf5[i] = st->tri1.Process();
  }
  // 19_Clip / 20_softclip.
  for (int i = 0; i < n; ++i)
    st->buf5[i] = HkSoftclip(ClipTo(st->buf5[i], -0.824999988079071f,
                                    0.824999988079071f));
  // 21_LFNoise2 / 22_MulAdd / 23_* → vibrato2.
  st->lfn_vib2.ProcessBlock(st->buf3, n);
  for (int i = 0; i < n; ++i)
    st->buf4[i] = st->buf3[i] * 0.009950495325028896f
                + 1.0000494718551636f;   // c[11], c[12]
  for (int i = 0; i < n; ++i) {
    st->tri2.SetFreq((c.hz * 3.0f) * st->buf4[i]);
    st->buf6[i] = st->tri2.Process();
  }
  for (int i = 0; i < n; ++i)
    st->buf6[i] = HkSoftclip(ClipTo(st->buf6[i], -0.824999988079071f,
                                    0.5f));
  // 27_* = snd2 * (amp/7) and 28_MulAdd = snd*amp + that.
  for (int i = 0; i < n; ++i)
    st->buf7[i] = st->buf6[i] * (amp * st->r7);
  for (int i = 0; i < n; ++i)
    st->buf8[i] = st->buf5[i] * amp + st->buf7[i];
  // lfo1/lfo2 + tri3/tri4 chain.
  st->lfo1.ProcessBlock(st->buf9, n);
  st->lfo2.ProcessBlock(st->buf10, n);
  for (int i = 0; i < n; ++i) {
    st->tri3.SetFreq(c.hz * 5.0f - st->buf10[i]);
    st->buf11[i] = st->tri3.Process();
  }
  for (int i = 0; i < n; ++i)
    st->buf11[i] = HkSoftclip(ClipTo(st->buf11[i], -0.824999988079071f,
                                     0.65f)) * st->buf9[i];
  for (int i = 0; i < n; ++i)
    st->buf8[i] = st->buf11[i] * (amp * st->r10) + st->buf8[i];
  for (int i = 0; i < n; ++i) {
    st->tri4.SetFreq(c.hz * 2.0f - st->buf9[i]);
    st->buf11[i] = st->tri4.Process();
  }
  for (int i = 0; i < n; ++i)
    st->buf11[i] = HkSoftclip(ClipTo(st->buf11[i], -0.824999988079071f,
                                     0.7f)) * st->buf10[i];
  for (int i = 0; i < n; ++i)
    st->buf8[i] = st->buf11[i] * (amp * st->r6) + st->buf8[i];
  // 43_Dust2 + 44_Crackle*0.2 + 49_SinOsc(PinkNoise*7500+40)*0.011.
  st->dust2.ProcessBlock(st->buf1, n);
  st->crackle.ProcessBlock(st->buf2, n);
  st->pn.ProcessBlock(st->buf3, n);
  for (int i = 0; i < n; ++i)
    st->buf4[i] = st->buf3[i] * 0.5f;
  for (int i = 0; i < n; ++i)
    st->buf5[i] = st->buf4[i] * 7500.0f + 40.0f;
  st->pn_sin.ProcessBlock(st->buf5, st->buf6, n);
  for (int i = 0; i < n; ++i)
    st->buf7[i] = st->buf2[i] * 0.2f + st->buf1[i];
  for (int i = 0; i < n; ++i)
    st->buf9[i] = st->buf6[i] * 0.011f + st->buf7[i];
  st->hpf.ProcessBlock(st->buf9, st->buf10, n, 25.0f, 0.0f);
  st->bpf.ProcessBlock(st->buf10, st->buf11, n, c.hz * 8.0f, 1.0f);
  for (int i = 0; i < n; ++i)
    st->buf12[i] = (st->buf11[i] * 1.399999976158142f) * amp + st->buf8[i];
  for (int i = 0; i < n; ++i)
    l[i] = st->leak.Next(st->buf12[i], 0.9950000047683716f);
  for (int i = 0; i < n; ++i) r[i] = l[i];
}

inline const DroneSpec kDroneApparatus = {"Apparatus", drones::Ap_Init,
                                          drones::Ap_Process};

// ---------------------------------------------------------------------------
// Twin Pks (id 30) — retro tape/vinyl chain. DEFERRED (2026-11-20, T-D2 Task 6):
// best max_abs 0.000168291 / corr 0.999999976 (a decaying ctor-prime
// transient, worst_i 2570) after 3 distinct fix attempts (v1 graph; wow
// SinOsc iphase=2 radians; filter ctor primes). Catalog id 30 stays nullptr
// (inert per plan). Full port + law notes preserved for a T-D3 resume — do
// NOT weaken the gate or regenerate the golden.
// 64 UGens; two noise banks,
// Compander+Limiter, BPF, wow DelayC (ar delaytime), tanh, Decimator, BPF.
// ---------------------------------------------------------------------------
struct TpState {
  T6Dust2 dust2a, dust2b;
  T6Crackle cracklea, crackleb;
  HkPinkNoise pna, pnb;
  SinOscAF pnsina, pnsinb;
  Filter2Pole hpf, bpf1, bpf2;
  Compander comp;
  Limiter lim;
  DelayCA delay;
  SinOscAF wow_sine;
  LFPar lfpar;
  LinExp le_ratio, le_sat, le_wowrate, le_depth;
  Decimator decim;
  float b41_prev, c41_prev, c51_prev;  // ar MulAdd kr-ramp previous values
  float buf1[64], buf2[64], buf3[64], buf4[64], buf5[64], buf6[64];
  float buf7[64], buf8[64], buf9[64], buf10[64], buf11[64], buf12[64];
  float delaytime[64];
};

inline void Tp_Init(Ctx& c) {
  TpState* st = new (c.arena->Alloc(sizeof(TpState))) TpState();
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);
  // RGen creation order: 4_Dust2, 5_Crackle, 7_PinkNoise, 43_Dust2,
  // 44_Crackle, 46_PinkNoise.
  st->dust2a.Init(c.rgen, sr, 10.0f);
  st->cracklea.Init(c.rgen, 1.9500000476837158f);
  st->pna.Init(c.rgen);
  st->dust2b.Init(c.rgen, sr, 10.0f);
  st->crackleb.Init(c.rgen, 1.9500000476837158f);
  st->pnb.Init(c.rgen);
  st->pnsina.Init(c.tables, sr);
  st->pnsinb.Init(c.tables, sr);
  st->hpf.Init(sr, 1); st->bpf1.Init(sr, 2); st->bpf2.Init(sr, 2);
  const float drive = 0.75f, wow = 0.6f;
  st->le_ratio.Init(0.f, 1.f, 0.15000000596046448f, 0.009999999776482582f);
  st->le_sat.Init(0.f, 1.f, 1.0f, 2.5f);
  st->le_wowrate.Init(0.f, 1.f, 0.5f, 4.0f);
  st->le_depth.Init(0.f, 1.f, 1.0f, 30.0f);
  const float ratio = st->le_ratio.Next(drive);
  const float threshold = drive * -0.4699999988079071f + 0.800000011920929f;
  const float gain = 1.0f / ((1.0f - threshold) * ratio + threshold);
  st->comp.Init(sr, threshold, 1.0f, ratio, 0.1f, 1.0f, 0.0f, 0.0f);
  st->lim.Init(sr, 1.0f, 0.0007999999797903001f, 0.0f);
  st->delay.Init(sr, 0.02552817016839981f);   // c[22]
  const float wowrate = st->le_wowrate.Next(wow);
  st->wow_sine.Init(c.tables, sr, 2.0f);   // iphase = 2 radians (c[21])
  const float lfo_amt = std::floor(wow * 4.0f + 1.0f);   // c[18]=4, floor
  st->lfpar.Init(cr, lfo_amt * 0.1f, 0.0f);
  st->decim.Init(sr, 24000.0f, 16.0f, 0.0f);
  st->b41_prev = 0.0f; st->c41_prev = 0.0f; st->c51_prev = 0.0f;
  c.state = st;
}

inline void Tp_Process(Ctx& c, float* l, float* r, int n) {
  TpState* st = static_cast<TpState*>(c.state);
  const float amp = c.amp;
  const float drive = 0.75f, wow = 0.6f;
  // ---- first noise bank (wet) ----
  st->dust2a.ProcessBlock(st->buf1, n);
  st->cracklea.ProcessBlock(st->buf2, n);
  st->pna.ProcessBlock(st->buf3, n);
  for (int i = 0; i < n; ++i) st->buf4[i] = st->buf3[i] * 0.5f;
  for (int i = 0; i < n; ++i) st->buf5[i] = st->buf4[i] * 7500.0f + 40.0f;
  st->pnsina.ProcessBlock(st->buf5, st->buf6, n);
  for (int i = 0; i < n; ++i)
    st->buf7[i] = st->buf6[i] * 0.03999999910593033f
                + (st->buf2[i] * 0.2f + st->buf1[i]);   // c[9]=0.04
  st->hpf.ProcessBlock(st->buf7, st->buf8, n, 25.0f, 0.0f);
  // ---- compressor / limiter / BPF ----
  const float ratio = st->le_ratio.Next(drive);
  const float threshold = drive * -0.4699999988079071f + 0.800000011920929f;
  const float gain = 1.0f / ((1.0f - threshold) * ratio + threshold);
  st->comp.ProcessBlock(st->buf8, st->buf8, st->buf9, n);
  for (int i = 0; i < n; ++i) st->buf9[i] *= gain;
  st->lim.ProcessBlock(st->buf9, st->buf10, n);
  st->bpf1.ProcessBlock(st->buf10, st->buf11, n, c.hz, 0.4f);
  // ---- wow LFO / DelayC (ar delaytime) ----
  const float wowrate = st->le_wowrate.Next(wow);
  const float lfo_amt = std::floor(wow * 4.0f + 1.0f);
  const float depth = st->le_depth.Next(wow);
  const float lfpar = st->lfpar.ProcessKr();
  const float wow_depth = lfpar * lfo_amt + depth;
  const float powarg = wow_depth * 0.0008333333535119891f;  // c[20] = 1/1200
  const float powval = std::pow(2.0f, powarg);              // sc_pow(a>=0)
  const float wow_mul = (powval - 1.0f) / (4.0f * wowrate);
  const float wow_add = wow_mul + ScReciprocal(750.0f);     // ControlRate.ir.reciprocal
  st->wow_sine.ProcessBlock(wowrate, st->buf12, n);
  T6MulAddAK(st->buf12, st->delaytime, n, st->b41_prev, wow_mul,
             st->c41_prev, wow_add);
  st->b41_prev = wow_mul; st->c41_prev = wow_add;
  st->delay.ProcessBlock(st->buf11, st->delaytime, st->buf10, n);
  // ---- second noise bank ----
  st->dust2b.ProcessBlock(st->buf1, n);
  st->crackleb.ProcessBlock(st->buf2, n);
  st->pnb.ProcessBlock(st->buf3, n);
  for (int i = 0; i < n; ++i) st->buf4[i] = st->buf3[i] * 0.5f;
  for (int i = 0; i < n; ++i) st->buf5[i] = st->buf4[i] * 7500.0f + 40.0f;
  st->pnsinb.ProcessBlock(st->buf5, st->buf6, n);
  for (int i = 0; i < n; ++i)
    st->buf7[i] = st->buf6[i] * 0.006f
                + (st->buf2[i] * 0.2f + st->buf1[i]);
  // ---- saturate, tanh, decimate, BPF, level ----
  const float sat = st->le_sat.Next(drive);
  // 51_MulAdd: a=DelayC(ar), b=LinExp(kr, ramps), c=noise bank (ar).
  {
    float br[64];
    T6Ramp64(br, n, st->c51_prev, sat);
    for (int i = 0; i < n; ++i) st->buf11[i] = st->buf10[i] * br[i] + st->buf7[i];
  }
  st->c51_prev = sat;
  for (int i = 0; i < n; ++i) st->buf11[i] = NovaTanh(st->buf11[i]);
  st->decim.ProcessBlock(st->buf11, st->buf12, n, 24000.0f, 16.0f);
  for (int i = 0; i < n; ++i) {
    st->buf9[i] = st->buf12[i] * 0.33000001311302185f
                + st->buf11[i] * 0.6700000166893005f;
  }
  st->bpf2.ProcessBlock(st->buf9, st->buf10, n, c.hz, 0.4f);
  for (int i = 0; i < n; ++i) {
    float v = st->buf10[i] * 15.848932266235352f * amp;
    v = HkSoftclip(v);
    l[i] = v; r[i] = v;
  }
}

inline const DroneSpec kDroneTwinPks = {"Twin Pks", drones::Tp_Init,
                                        drones::Tp_Process};

// ---------------------------------------------------------------------------
// Mt. Lion spec
// ---------------------------------------------------------------------------
inline const DroneSpec kDroneMtLion = {"Mt. Lion", drones::Ml_Init,
                                       drones::Ml_Process};

}  // namespace drones

inline const DroneSpec kDroneHarmsWay = {"harms way", drones::HarmsWay_Init,
                                         drones::HarmsWay_Process};

inline const DroneSpec kDroneThx = {"Thx", drones::Thx_Init,
                                    drones::Thx_Process};

inline const DroneSpec kDroneMtZion = {"Mt. Zion", drones::Mz_Init,
                                       drones::Mz_Process};

inline const DroneSpec kDroneMtLion = {"Mt. Lion", drones::Ml_Init,
                                       drones::Ml_Process};

}  // namespace sc
#endif
