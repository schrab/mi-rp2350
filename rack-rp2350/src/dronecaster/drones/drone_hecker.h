// Hecker — ported from drones/hecker.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_HECKER_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_HECKER_H_
#include "drone_common.h"
namespace sc {
namespace drones {
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


}  // namespace drones
}  // namespace sc
#endif
