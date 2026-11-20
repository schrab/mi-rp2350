#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_COMMON_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_COMMON_H_
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
#include "sc_kit_osc2.h"
#include "sc_kit_pan.h"
#include "sc_kit_lfo.h"
#include "sc_kit_noise.h"
#include "sc_kit_trig.h"
#include "sc_kit_filter.h"
#include "sc_kit_delay.h"
#include "sc_kit_envgen.h"
#include "sc_kit_lin.h"
#include "sc_kit_demand.h"
#include "sc_kit_sc3p_filter.h"
#include "sc_kit_core_dyn.h"
#include "sc_kit_sc3p_dyn.h"
#include "sc_sugar.h"
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
      left[i] = in[i] * (level * t_->sine_q[2048 - ipos]);
      right[i] = in[i] * (level * t_->sine_q[ipos]);
      if (changed) level += slope;
    }
    if (changed) level_ = level;   // the accumulated ramp end
  }

 private:
  KitTables* t_ = nullptr;
  float level_ = 0.0f;
};

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
    *l = level_ * t_->sine_q[2048 - ip];
    *r = level_ * t_->sine_q[ip];
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
  // Vector-backed Init (kit refs / host tests only — no device drone path).
  void Init(double sr, float maxdelay) {
    sr_ = static_cast<float>(sr);
    long sz = static_cast<long>(std::ceil(maxdelay * sr_ + 1.f)) + 64;
    bufsize_ = 1;
    while (bufsize_ < sz) bufsize_ <<= 1;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    datap_ = buf_.data();
    iwr_ = 0;
    wpos_ = 0;
    max_dsamp_ = static_cast<float>(bufsize_);
  }
  // Caller-owned ring (device drone states): no heap; storage zeroed (the
  // device aux region may hold stale bytes from the previously resident
  // module). Clamp ceiling bufsize-4 (the checked cubic reads reach back
  // dsamp+2 and forward +1); the caller sizes the ring from the graph's
  // delaytime bound + margin so the clamp never engages in the golden.
  void InitExt(float* buf, long floats, double sr, float maxdelay) {
    (void)maxdelay;
    sr_ = static_cast<float>(sr);
    bufsize_ = floats;
    datap_ = buf;
    for (long i = 0; i < floats; ++i) datap_[i] = 0.f;
    iwr_ = 0;
    wpos_ = 0;
    max_dsamp_ = static_cast<float>(bufsize_ - 4);
  }
  void ProcessBlock(const float* in, const float* delaytime, float* out, int n) {
    for (int i = 0; i < n; ++i) {
      float dsamp = delaytime[i] * sr_;
      const float kMin = 2.0f;   // DelayC::minDelaySamples = 2
      dsamp = dsamp < kMin ? kMin : (dsamp > max_dsamp_ ? max_dsamp_ : dsamp);
      const long idsamp = static_cast<long>(dsamp);
      const float frac = dsamp - static_cast<float>(idsamp);
      // Checked-phase tests on the absolute counter; ring indices derived
      // from the wrapped write position (identical values to the old mask).
      const long i1abs = iwr_ - idsamp;
      const long w = wpos_;
      const long r1 = w - idsamp < 0 ? w - idsamp + bufsize_ : w - idsamp;
      const long r0 = (r1 + 1 == bufsize_) ? 0 : r1 + 1;
      const long r2 = (r1 == 0) ? bufsize_ - 1 : r1 - 1;
      const long r3 = (r1 >= 2) ? r1 - 2 : r1 - 2 + bufsize_;
      datap_[w] = in[i];
      float value = 0.0f;
      if (i1abs + 1 >= 0) {
        const float d0 = datap_[r0];
        const float d1 = i1abs >= 0 ? datap_[r1] : 0.0f;
        const float d2 = i1abs - 1 >= 0 ? datap_[r2] : 0.0f;
        const float d3 = i1abs - 2 >= 0 ? datap_[r3] : 0.0f;
        const float c0 = d1;
        const float c1 = 0.5f * (d2 - d0);
        const float c2 = d0 - 2.5f * d1 + 2.0f * d2 - 0.5f * d3;
        const float c3 = 0.5f * (d3 - d0) + 1.5f * (d1 - d2);
        value = ((c3 * frac + c2) * frac + c1) * frac + c0;
      }
      out[i] = value;
      ++iwr_;
      wpos_ = (wpos_ + 1 == bufsize_) ? 0 : wpos_ + 1;
    }
  }
 private:
  std::vector<float> buf_;   // vector mode only (refs/tests)
  float* datap_ = nullptr;   // active ring (vector or ext)
  long bufsize_ = 0, iwr_ = 0, wpos_ = 0;
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
// LFTri with the classlib phase argument (wrapped into [0,4) at ctor,
// one priming sample then phase reset — SC LFUGens.cpp:665).
struct LFTriP {
  double phase = 0.0;
  float freqmul = 0.0f, freq = 0.0f;
  void Init(double sr, float f, float ph) {
    freqmul = static_cast<float>(4.0 * (1.0 / sr));
    freq = f;
    double p = static_cast<double>(ph) - std::floor(static_cast<double>(ph) / 4.0) * 4.0;
    phase = p;
  }
  void SetFreq(float f) { freq = f; }
  float Process() {
    const float z = phase > 1.0 ? static_cast<float>(2.0 - phase)
                                : static_cast<float>(phase);
    phase += freq * static_cast<double>(freqmul);
    if (phase >= 3.0) phase -= 4.0;
    return z;
  }
};

}  // namespace drones
}  // namespace sc
#endif
