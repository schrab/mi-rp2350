// Coil — ported from drones/coil.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_COIL_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_COIL_H_
#include "drone_common.h"
namespace sc {
namespace drones {
static const float kCoilC[68] = {
  1.0f, 12648430.0f, 0.5f, 0.0f, 0.200000003f, 1.25f,
  1.75f, 1.45000005f, 1.54999995f, 2.0f, -99.0f, 3.0f,
  -1.0f, 0.00100000005f, 6.28318548f, 0.25f, 0.100000001f, 6.0f,
  0.600000024f, 5.0f, 10.0f, 0.00999999978f, 0.0199999996f, 15.0f,
  0.0164196547f, 0.0252477452f, 0.0208529048f, 0.300000012f, 1.5f, 0.0217190403f,
  0.400000006f, 0.0291209314f, 2.5f, 0.0105374316f, 0.600000024f, 0.0180459674f,
  0.699999988f, 3.5f, 0.0161049217f, 0.800000012f, 4.0f, 0.0276014321f,
  0.899999976f, 4.5f, 0.0161766838f, 0.0297038704f, 1.10000002f, 5.5f,
  0.0112564638f, 0.0480000004f, 0.098935172f, 0.0399999991f, 0.0500000007f, 0.0903932676f,
  0.0356686339f, 0.0312120318f, 0.0516909473f, 0.0633430257f, 0.0824305043f, 0.0463094339f,
  0.0329945795f, 0.0268352441f, 0.0178718083f, 0.00429535517f, 0.0327041671f, 0.0237236731f,
  0.0225685947f, -4.0f,
};

static const int kClDelay[12] = {24,25,26,29,31,33,35,38,41,44,45,48};

// ---------------------------------------------------------------------------
// Coil (id 2) — DEFERRED (2026-11-20, T-D2 Task 8): the port is finite and
// runs, but its spectral envelope differs from the golden (L1 0.90 — the
// golden is strongly pitched at 60-90 Hz, the port is broadband), i.e. a
// different timbre, which is the sole deferral criterion per the controller
// ruling. Catalog id 2 stays nullptr. Full port + law notes preserved for a
// T-D3 exactness pass. Do NOT regenerate the golden.
// 12 voices: Dust/Impulse-triggered EnvGen envelopes,
// Latch-ed LFNoise0 attack/release, TChoose pan/source selectors,
// SinOscFB/WhiteNoise SelectX, BLowPass, DelayC, Pan2 — then the shared
// Sachiko-style reverb (DelayN + 7 CombL + 4 AllpassN x2, arena-owned).
// ---------------------------------------------------------------------------
struct ClVoice {
  Dust dust05, dust1;
  Impulse imp;
  LFNoise0 lfn_atk, lfn_rel;
  Latch latch_atk, latch_rel;
  Trig trig;
  TChoose ti_l, ti_r, ti_w;
  SinOsc modosc, fbphase;
  SinOscFB fbosc;
  WhiteNoise wn;
  XFade2Ak xf;
  BEQFilter blp;
  LFNoise1 lfn1;
  DelayLine delay;
  Pan2 pan;
  EnvGen env, env2, env3;
  EnvGen::Node node[2];
  float atk_prev, rel_prev, env3_prev;
};
struct ClState {
  ClVoice v[12];
  float buf1[64], buf2[64], buf3[64], buf4[64];
  float mixl[64], mixr[64], dn_l[64], dn_r[64], rv_l[64], rv_r[64];
  float csum_l[64], csum_r[64], ap_l[64], ap_r[64];
  DelayLine dn, comb_l[7], comb_r[7], ap_l_[4], ap_r_[4];
  LFNoise1 apnoise[7];
};

inline void Cl_Init(Ctx& c) {
  ClState* st = new (c.arena->Alloc(sizeof(ClState))) ClState();
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);
  const float midi = NovaFreq2Midi(c.hz);
  for (int k = 0; k < 12; ++k) {
    ClVoice& vc = st->v[k];
    vc.dust05.Init(c.rgen, cr, 0.5f);   // Dust.kr — control rate 750
    vc.dust1.Init(c.rgen, cr, 1.0f);
    vc.imp.Init(cr, 0.0f);
    vc.lfn_atk.Init(c.rgen, cr, kCoilC[4]);
    vc.lfn_rel.Init(c.rgen, cr, kCoilC[4]);
    // Rand(0,2pi) consumes one frand at ctor.
    const float phase = RandIrg(c.rgen, 0.f, kCoilC[14]);
    vc.modosc.Init(c.tables, cr, 0.1f * static_cast<float>(k));
    vc.fbphase.Init(c.tables, cr, 0.2f);
    (void)phase;
    vc.fbosc.Init(c.tables, sr, c.hz, 0.0f);
    float dummy;
    vc.wn.Init(c.rgen);
    vc.xf.Init(c.tables, 0.f, 1.f, 0.f, 0.f);
    vc.blp.Init(sr, 0, c.hz * 6.0f, kCoilC[18], 0.f, 0.f);
    const float rnd = RandIrg(c.rgen, 5.f, 10.f);
    vc.lfn1.Init(c.rgen, cr, rnd);
    vc.delay.Init(sr, kCoilC[kClDelay[k]], 0.0f, 0.0f, 2);
    vc.pan.Init(c.tables, 0.f, 1.f, 0.f);
    // EnvGen node arrays live in the voice (arena), never on the stack.
    vc.node[0] = {1.f, 1.f, 3.f, 0.f};   // shape 3 = \sine
    vc.node[1] = {0.f, 1.f, 3.f, 0.f};
    // kr-rate envelopes for this block-rate approximation (dur in seconds).
    vc.env.Init(cr, vc.node, 2, 0.f, 1.f, 0.f, 1.f, -99, -99);
    vc.env2.Init(cr, vc.node, 2, 0.f, 1.f, 0.f, 1.f, -99, -99);
    vc.env3.Init(cr, vc.node, 2, 0.f, 1.f, 0.f, 1.f, -99, -99);
    vc.ti_l.Init(c.rgen, 2); vc.ti_r.Init(c.rgen, 2); vc.ti_w.Init(c.rgen, 2);
    vc.trig.Init(); vc.latch_atk.Init(); vc.latch_rel.Init();
    (void)dummy; (void)midi;
    vc.atk_prev = 1.f; vc.rel_prev = 1.f; vc.env3_prev = 0.f;
  }
  // Shared reverb (Sachiko law): arena-owned delay lines; combs read the
  // DelayN output, not their running sum.
  st->dn.Init(sr, 0.048f, 0.2f, 0.f, 2);
  for (int j = 0; j < 7; ++j) {
    st->comb_l[j].Init(sr, 0.1f, kCoilC[50 + j], 15.f, 4);
    st->comb_r[j].Init(sr, 0.1f, kCoilC[50 + j], 15.f, 4);
    st->apnoise[j].Init(c.rgen, cr, kCoilC[57 + j]);
  }
  for (int j = 0; j < 4; ++j) {
    // Dump order interleaves the two stereo chains: L = c[59,61,63,65],
    // R = c[60,62,64,66]. c[67] is an EnvGen curve constant, NOT a delay.
    st->ap_l_[j].Init(sr, 0.05f, kCoilC[59 + 2*j], 1.f, 6);
    st->ap_r_[j].Init(sr, 0.05f, kCoilC[60 + 2*j], 1.f, 6);
  }
  c.state = st;
}

inline void Cl_Process(Ctx& c, float* l, float* r, int n) {
  ClState* st = static_cast<ClState*>(c.state);
  const float amp = c.amp;
  const float midi = NovaFreq2Midi(c.hz);
  for (int i = 0; i < n; ++i) { st->mixl[i] = 0.f; st->mixr[i] = 0.f; }
  for (int k = 0; k < 12; ++k) {
    ClVoice& vc = st->v[k];
    const float pulse = vc.dust05.Next() + vc.imp.ProcessKr();
    const float phit = vc.dust1.Next();
    const float atk = vc.latch_atk.Next(
        vc.lfn_atk.ProcessKr() * kCoilC[5] + kCoilC[6], pulse);
    const float rel = vc.latch_rel.Next(
        vc.lfn_rel.ProcessKr() * kCoilC[7] + kCoilC[8], pulse);
    const float trig = vc.trig.Next(phit, atk + rel, static_cast<float>(c.sr));
    vc.node[0].dur = atk;
    vc.node[1].dur = rel;
    float e0 = vc.env.ProcessKr(trig);
    float e2 = vc.env2.ProcessKr(trig);
    float e3 = vc.env3.ProcessKr(trig);
    // TChoose on trigger edges.
    float vals2[2] = {-1.f, 1.f}, vals01[2] = {0.f, 1.f};
    float cho_l, cho_r, cho_w;
    vc.ti_l.ProcessBlock(&pulse, &cho_l, 1, vals2, 2);
    vc.ti_r.ProcessBlock(&pulse, &cho_r, 1, vals2, 2);
    vc.ti_w.ProcessBlock(&pulse, &cho_w, 1, vals01, 2);
    // SinOscFB with the LFO range and Rand phase (block-rate approximations).
    vc.modosc.SetFreq(0.1f * static_cast<float>(k));
    const float lfo = vc.modosc.Process();
    const float fmod = (c.hz * 6.f) * lfo
                     + (midi - 0.5f * static_cast<float>(k));
    vc.fbphase.SetFreq(0.2f);
    float fb = vc.fbphase.Process() * kCoilC[15] + kCoilC[15];
    vc.fbosc.SetFreq(c.hz + fmod);
    vc.fbosc.ProcessBlock(st->buf1, n);
    vc.wn.ProcessBlock(st->buf2, n);
    for (int i = 0; i < n; ++i) st->buf2[i] *= kCoilC[16];
    // SelectX with ar index — use the block's first value for the crossfade.
    const int ia = HkSelectIndex(HkRoundQ(e3, 2.0f));
    const int ib = HkSelectIndex(HkTruncQ(e3, 2.0f) + 1.0f);
    const float* sa = (ia == 1) ? st->buf1 : st->buf2;
    const float* sb = (ib == 1) ? st->buf1 : st->buf2;
    vc.xf.ProcessBlock(sa, sb, st->buf3, n, HkFold2(e3 * 2.f - 1.f));
    vc.blp.ProcessBlock(st->buf3, st->buf4, n, c.hz * 6.0f, kCoilC[18], 0.f);
    const float dly = (vc.lfn1.ProcessKr() * kCoilC[21] + kCoilC[22])
                    / kCoilC[23];
    vc.delay.ProcessBlock(st->buf4, st->buf1, n, dly, 0.f);
    for (int i = 0; i < n; ++i) {
      float env = e0 + (kCoilC[0] - e0) * 0.f;  // ramp value (e0 per block)
      st->buf2[i] = st->buf1[i] * env;
    }
    (void)cho_l; (void)cho_r; (void)cho_w;
    vc.pan.ProcessBlock(st->buf2, st->buf3, st->buf4, n, e2, 1.f);
    for (int i = 0; i < n; ++i) {
      st->mixl[i] += st->buf3[i];
      st->mixr[i] += st->buf4[i];
    }
  }
  for (int i = 0; i < n; ++i) {
    st->mixl[i] *= amp; st->mixr[i] *= amp;
  }
  // Reverb chain (identical structure to Sachiko).
  st->dn.ProcessBlock(st->mixl, st->dn_l, n, 0.2f, 0.f);
  st->dn.ProcessBlock(st->mixr, st->dn_r, n, 0.2f, 0.f);
  for (int i = 0; i < n; ++i) { st->csum_l[i] = 0.f; st->csum_r[i] = 0.f; }
  for (int j = 0; j < 7; ++j) {
    const float d = st->apnoise[j].ProcessKr() * kCoilC[51] + kCoilC[52];
    st->comb_l[j].ProcessBlock(st->dn_l, st->rv_l, n, d, 15.f);
    st->comb_r[j].ProcessBlock(st->dn_r, st->rv_r, n, d, 15.f);
    for (int i = 0; i < n; ++i) {
      st->csum_l[i] += st->rv_l[i]; st->csum_r[i] += st->rv_r[i];
    }
  }
  for (int i = 0; i < n; ++i) { st->dn_l[i] = st->csum_l[i]; st->dn_r[i] = st->csum_r[i]; }
  for (int j = 0; j < 4; ++j) {
    st->ap_l_[j].ProcessBlock(st->dn_l, st->rv_l, n, kCoilC[59+2*j], 1.f);
    st->ap_r_[j].ProcessBlock(st->dn_r, st->rv_r, n, kCoilC[60+2*j], 1.f);
    for (int i = 0; i < n; ++i) { st->dn_l[i] = st->rv_l[i]; st->dn_r[i] = st->rv_r[i]; }
  }
  // -20.dbamp = 0.1; reverb mix 0.05.
  for (int i = 0; i < n; ++i) {
    l[i] = (st->mixl[i] + st->rv_l[i] * 0.05f) * 0.1f;
    r[i] = (st->mixr[i] + st->rv_r[i] * 0.05f) * 0.1f;
  }
}

inline const DroneSpec kDroneCoil = {"Coil", drones::Cl_Init,
                                     drones::Cl_Process};

// T-D2 state-size budget (Task 8): every state must fit comfortably in the

}  // namespace drones
}  // namespace sc
#endif
