// Mt Zion — ported from drones/mt_zion.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_MT_ZION_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_MT_ZION_H_
#include "drone_common.h"
namespace sc {
namespace drones {
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
  // T-D2: the BLIT Pulse reads the full gSine/gInvSine tables (lazy).
  if (!EnsurePulseTables(c)) { c.state = nullptr; return; }
  // placement-new: the state contains kit structs; the arena memory is raw
  MzState* st = new (c.arena->Alloc(sizeof(MzState))) MzState();
  if (st == nullptr) { c.state = nullptr; return; }
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


}  // namespace drones
}  // namespace sc
#endif
