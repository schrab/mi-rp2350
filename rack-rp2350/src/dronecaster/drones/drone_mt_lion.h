// Mt Lion — ported from drones/mt_lion.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_MT_LION_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_MT_LION_H_
#include "drone_common.h"
namespace sc {
namespace drones {
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

// T-D2 device memory law: the 9 CombN rings are arena/aux-owned (NO heap —
// the device C++ heap is ~22 KB and the old std::vector rings asked for
// 2.25 MB; a failed operator new hung core 1). Ring sizes are GRAPH
// CONSTANTS measured on the golden render (delaytime = ScReciprocal(
// ScMidicps(Latch(Lag(...)))) * round(wn*3+4 lagged); the dt chain's latch
// holds 0 until the Dust(0.3) fires, so ScMidicps(0) = 8.1758 Hz makes the
// early-transient delaytime up to ~0.86 s while the 5 s lag converges):
//   golden max dsamp per voice = {1, 574, 10072, 1, 33234, 1, 1, 13040, 22415}
//   -> ring = golden + 8 (ext clamp ceiling = ring - 4, never engages).
// The four voices whose dtm Dust never fired in the golden keep a 2,048-
// sample (42.7 ms) transient allowance: live, their first dtm fire clamps
// there until the lag converges (documented live-only deviation; the golden
// render is bit-exact). Allocation order = descending size so the two big
// rings land in the arena and the small ones spill to the aux region.
struct MlState {
  MlVoice voice[9];
  float* comb_ring[9];
  float cpsmidi, basenote, detune, maxamp;
  float pulsebuf[64], combbuf[64], tanhbuf[64];
  float vl[64], vr[64];
  float s14l[64], s14r[64], s58l[64], s58r[64], s911l[64], s911r[64];
  float pair_a_l[64], pair_a_r[64], pair_b_l[64], pair_b_r[64];
  LeakDC leak_l, leak_r;
};

inline void MlVoice_Init(Ctx& c, MlVoice& vc, float* ring, long ring_floats) {
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
  // T-D2: arena/aux-owned ring (see the MlState comment for the sizing law).
  vc.comb.InitExt(ring, ring_floats, 48000.0, 1.0f, 0.0f, 0.0f, 3);
  vc.pan.Init(c.tables, 0.0f, 0.0f, 0.0f);   // m_pos/m_level = 0, amps 0
  vc.mul_freq_prev = vc.wn_freq.prime;
  vc.mul_dt_prev = vc.wn_dt.prime;
  vc.mul_lvl_prev = vc.wn_level.prime;
}

inline void Ml_Init(Ctx& c) {
  // placement-new the state into the arena, then place the 9 CombN rings
  // (arena first, aux spill — NO heap; see the MlState comment).
  MlState* st = new (c.arena->Alloc(sizeof(MlState))) MlState();
  if (st == nullptr) { c.state = nullptr; return; }
  static const long kRingFloats[9] = {2048, 582, 10080, 2048, 33242,
                                      2048, 2048, 13048, 22423};
  static const int kAllocOrder[9] = {4, 8, 5, 6, 7, 2, 0, 3, 1};
  for (int k = 0; k < 9; ++k) {
    const int v = kAllocOrder[k];
    st->comb_ring[v] = static_cast<float*>(
        AllocRing(c, static_cast<size_t>(kRingFloats[v]) * 4u));
    if (st->comb_ring[v] == nullptr) { c.state = nullptr; return; }
  }
  st->cpsmidi = NovaFreq2Midi(55.0f);
  st->basenote = ScRound1(st->cpsmidi);
  st->detune = std::fabs(st->basenote - st->cpsmidi);
  // 9_/ is an AR div by a SCALAR constant: div_ai_nova (BinaryOpUGens.cpp
  // :1970) = times_vec_simd(in, sc_reciprocal(scalar)) — NOT IEEE division
  // (proven: the level-latch grab differed by exactly 1 float32 ULP).
  st->maxamp = 0.4f * ScReciprocal(9.0f);
  for (int v = 0; v < 9; ++v) {
    MlVoice_Init(c, st->voice[v], st->comb_ring[v], kRingFloats[v]);
  }
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


}  // namespace drones
}  // namespace sc
#endif
