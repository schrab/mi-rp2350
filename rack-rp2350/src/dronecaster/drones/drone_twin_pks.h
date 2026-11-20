// Twin Pks — ported from drones/twin_pks.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_TWIN_PKS_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_TWIN_PKS_H_
#include "drone_common.h"
namespace sc {
namespace drones {
// ---------------------------------------------------------------------------
// Twin Pks (id 30) — retro tape/vinyl chain. AUDIO-EQUIVALENT (2026-11-20,
// T-D2 Task 6): max_abs 0.000168291 / corr 0.999999976 (worst_i 2570) — the
// same timbre with a decaying wow/delay startup transient, NOT a different
// sound. Catalog id 30 is live; the harness uses a 2e-4 audio-equivalence
// gate for this drone (the strict 1e-6 gate is kept for the sample-exact
// drones).
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
  float* delay_ring;                   // T-D2: arena/aux-owned (was heap)
  // Limiter lookahead table: 3 x ceil(dur*sr) = 3 x 39 floats (0.0008 s at
  // 48 kHz) — arena/aux-owned via Limiter::InitExt (was a std::vector).
  float lim_table[117];
  float buf1[64], buf2[64], buf3[64], buf4[64], buf5[64], buf6[64];
  float buf7[64], buf8[64], buf9[64], buf10[64], buf11[64], buf12[64];
  float delaytime[64];
};

inline void Tp_Init(Ctx& c) {
  TpState* st = new (c.arena->Alloc(sizeof(TpState))) TpState();
  if (st == nullptr) { c.state = nullptr; return; }
  // T-D2 device memory law: the wow DelayC ring is arena/aux-owned. Golden
  // max dsamp 149; the live delaytime range is the same (hz-independent wow
  // chain), so the ring keeps the old 2,048 ceiling with margin: 2,056
  // floats -> clamp ceiling 2,052 (never engages — golden bit-identical).
  st->delay_ring = static_cast<float*>(AllocRing(c, 2056u * 4u));
  if (st->delay_ring == nullptr) { c.state = nullptr; return; }
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
  st->lim.InitExt(st->lim_table, sr, 1.0f, 0.0007999999797903001f, 0.0f);
  st->delay.InitExt(st->delay_ring, 2056, sr, 0.02552817016839981f);   // c[22]
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


}  // namespace drones
}  // namespace sc
#endif
