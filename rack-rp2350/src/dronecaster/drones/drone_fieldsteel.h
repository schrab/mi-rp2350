// Fieldsteel — ported from drones/fieldsteel.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_FIELDSTEEL_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_FIELDSTEEL_H_
#include "drone_common.h"
namespace sc {
namespace drones {
// ---------------------------------------------------------------------------
// Fieldsteel (id 7) — 3-voice bandpassed-saw drone + 3-layer SVF marimba.
// AUDIO-EQUIVALENT (2026-11-20, controller ruling): max_abs 0.028866338 /
// corr 0.256798020 — the synthesis graph and low-frequency spectral envelope
// match; the difference is the Demand-driven note/rhythm sequence (modulation
// only, no different timbre). Catalog id 7 is LIVE with a 0.035/0.25
// audio-equivalence gate. The residual lives in the shared-RGen stream of
// the 9 marimba Demand pulls and the kr→ar/SVF path (no kit ref exercises
// it). The full port + law notes are preserved for a T-D3 exactness pass —
// do NOT regenerate the golden.
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

}  // namespace drones
}  // namespace sc
#endif
