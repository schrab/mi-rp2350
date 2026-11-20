// Supersaw — ported from drones/supersaw.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_SUPERSAW_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_SUPERSAW_H_
#include "drone_common.h"
namespace sc {
namespace drones {
// ---------------------------------------------------------------------------
// Supersaw (id 25) — 5 voices, Saw(hz*i + SinOsc.kr(0.1*i)*0.5) → BPF(100+
// i*100 + SinOsc.kr(0.05*i)*100, rq 2) → Pan2(pos -1,-0.5,0,0.5,1) →
// Sum4(v4,v2,v1,v0) + v4 → *levelcomp → *amp. 55 UGens, NO RGen draws.
// Uses the verbatim BpfKr (3-sample coefficient interpolation) written for
// this drone and the double-cpstoinc SinOscKr (kr table-increment law).
// ---------------------------------------------------------------------------
struct SsVoice {
  SinOscKr saw_lfo, bpf_lfo;
  Saw saw;
  BpfKr bpf;
  Pan2 pan;
};
struct SsState {
  SsVoice voice[5];
  float sawbuf[64], bpfbuf[64];
  float vl[5][64], vr[5][64];   // per-voice Pan2 outputs (exact Sum4 fold)
};

inline void Ss_Init(Ctx& c) {
  SsState* st = new (c.arena->Alloc(sizeof(SsState))) SsState();
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);
  const float hz = c.hz;
  const float pos[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
  for (int i = 0; i < 5; ++i) {
    SsVoice& vc = st->voice[i];
    vc.saw_lfo.Init(c.tables, cr, 0.1f * static_cast<float>(i));
    vc.bpf_lfo.Init(c.tables, cr, 0.05f * static_cast<float>(i));
    // Both LFOs prime to 0 at phase 0, so the ctor freq is exactly hz*i.
    const float ctor_freq = hz * static_cast<float>(i);
    vc.saw.Init(c.tables, sr, ctor_freq);
    // The kit Saw::Init already primes-and-rolls-back; do NOT advance its
    // phase with an extra ProcessBlock. Compute the primed output with PulAt
    // (the same ctor-prime formula used by Thx in this header).
    int32_t n_ctor = static_cast<int32_t>((sr * 0.5) / static_cast<double>(ctor_freq));
    float scale_ctor = static_cast<float>(0.5 / static_cast<double>(n_ctor));
    float pv = vc.saw.PulAt(0, 2 * n_ctor + 1, scale_ctor, false, 0,
                            0.0f, 0.0f);
    float saw_prime = (pv == 1.0f) ? (1.0f + 0.999f * -0.46f)
                                   : (pv + 0.999f * -0.46f);
    vc.bpf.Init(sr);
    const float bpf_ctor = vc.bpf.Prime(100.0f + 100.0f * static_cast<float>(i),
                                        2.0f, saw_prime);
    vc.pan.Init(c.tables, pos[i], 1.0f, bpf_ctor);
  }
  c.state = st;
}

inline void Ss_Process(Ctx& c, float* l, float* r, int n) {
  SsState* st = static_cast<SsState*>(c.state);
  const float hz = c.hz;
  const float pos[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
  for (int i = 0; i < 5; ++i) {
    SsVoice& vc = st->voice[i];
    const float fi = static_cast<float>(i);
    const float saw_lfo = vc.saw_lfo.Process() * 0.5f;
    const float bpf_lfo = vc.bpf_lfo.Process() * 100.0f;
    vc.saw.SetFreq(hz * fi + saw_lfo);
    vc.saw.ProcessBlock(st->sawbuf, n);
    vc.bpf.ProcessBlock(st->sawbuf, st->bpfbuf, n,
                        100.0f + 100.0f * fi + bpf_lfo, 2.0f);
    vc.pan.ProcessBlock(st->bpfbuf, st->vl[i], st->vr[i], n, pos[i], 1.0f);
  }
  // Mix: 35_Sum4 = (v4+v2)+(v1+v0), then 45_+ = Sum4 + v4, then
  // 46_* = *Splay level comp (c[20]), then 47_* = *amp.
  const float level = 0.4472135901451111f;   // c[20], exact float32
  for (int i = 0; i < n; ++i) {
    const float sl = (st->vl[4][i] + st->vl[2][i])
                   + (st->vl[1][i] + st->vl[0][i]);
    const float sr_ = (st->vr[4][i] + st->vr[2][i])
                    + (st->vr[1][i] + st->vr[0][i]);
    l[i] = (sl + st->vl[4][i]) * level * c.amp;
    r[i] = (sr_ + st->vr[4][i]) * level * c.amp;
  }
}

inline const DroneSpec kDroneSupersaw = {"Supersaw", drones::Ss_Init,
                                         drones::Ss_Process};


}  // namespace drones
}  // namespace sc
#endif
