// Apparatus — ported from drones/apparatus.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_APPARATUS_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_APPARATUS_H_
#include "drone_common.h"
namespace sc {
namespace drones {
// ---------------------------------------------------------------------------
// Apparatus (id 0) — 60 UGens. AUDIO-EQUIVALENT (2026-11-20, T-D2 Task 6):
// max_abs 0.000006787 / corr 1.000000000 (worst_i 514) — the same timbre with
// a tiny decaying ctor-prime transient, NOT a different sound. Per controller
// ruling, a modulation/transient-only difference is acceptable: catalog id 0
// is live and the golden harness uses a 1e-5 audio-equivalence gate for this
// drone (the strict 1e-6 gate is kept for the sample-exact drones).
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


}  // namespace drones
}  // namespace sc
#endif
