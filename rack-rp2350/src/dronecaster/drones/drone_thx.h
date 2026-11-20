// Thx — ported from drones/thx.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_THX_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_THX_H_
#include "drone_common.h"
namespace sc {
namespace drones {
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


}  // namespace drones
}  // namespace sc
#endif
