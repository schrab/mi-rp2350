// Harms Way — ported from drones/harms_way.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_HARMS_WAY_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_HARMS_WAY_H_
#include "drone_common.h"
namespace sc {
namespace drones {
// ---------------------------------------------------------------------------
// harms way (id 36)
// ---------------------------------------------------------------------------
struct HarmsWayState {
  SinOsc carrier[16];       // SinOsc.ar(hz * n), n = 1..16
  SinOscKr env[16];         // SinOsc.kr({Rand(0.001,0.02)} ! 16)
  SinOscKr center;          // SinOsc.kr(0.001) — Splay center
  Pan2 pan[16];             // Splay's Pan2 bank (level 1)
  float ramp_prev[16];      // the mul's mPrevB = v(k-1); block -1 = 0
  float base[16];           // def-exact Splay position constants
  float lc;                 // LevelComp ar: n.reciprocal.sqrt
  float carrierbuf[64], voicebuf[64];
  float pan_l[64], pan_r[64], tmp_l[64], tmp_r[64];
  float t01_l[64], t01_r[64], t23_l[64], t23_r[64];
  float acc_a_l[64], acc_a_r[64], acc_b_l[64], acc_b_r[64];
};

inline void HarmsWay_Init(Ctx& c) {
  HarmsWayState* st =
      static_cast<HarmsWayState*>(c.arena->Alloc(sizeof(HarmsWayState)));
  // {Rand(0.001,0.02)} ! 16 — NoiseUGens.cpp:443 Rand_Ctor: frand()*(hi-lo)+lo
  // in creation order (0..15), after RandSeed re-seeded the RGen.
  float rands[16];
  for (int i = 0; i < 16; ++i) rands[i] = RandIrg(c.rgen, 0.001f, 0.02f);
  const double cr = static_cast<double>(c.sr) / static_cast<double>(kBlock);
  for (int i = 0; i < 16; ++i) {
    st->carrier[i].Init(c.tables, static_cast<double>(c.sr),
                        c.hz * static_cast<float>(i + 1));
    st->env[i].Init(c.tables, cr, rands[i]);
    // Splay.sc:23-33 position constant: sclang double, rounded to float once.
    st->base[i] = static_cast<float>((static_cast<double>(i) * (2.0 / 15.0)
                                      - 1.0) * 0.5);
    // Pan2_Ctor: m_pos/m_level from ZIN0 (level = 1, the Splay default), amps
    // at the ctor pos (base + center prime = base), one primed sample (the
    // voice's prime = 0), state kept.
    st->pan[i].Init(c.tables, st->base[i], 1.f, 0.f);
    st->ramp_prev[i] = 0.f;   // kr ctor primes at phase 0 -> v(-1) = 0
  }
  st->center.Init(c.tables, cr, 0.001f);
  st->lc = static_cast<float>(std::sqrt(1.0 / 16.0));   // LevelComp (ar)
  c.state = st;
}

inline void HarmsWay_Process(Ctx& c, float* l, float* r, int n) {
  HarmsWayState* st = static_cast<HarmsWayState*>(c.state);
  const float center_k = st->center.Process();   // SinOsc.kr block value
  // level_sig = K2A(amp) * 0.25 (ugen 50_*; flat for the golden's constant
  // amp), then Mix * level_sig: one per-sample multiply by that product.
  const float amp_level = c.amp * st->lc;
  for (int g = 0; g < 4; ++g) {
    for (int k = 0; k < 4; ++k) {
      const int i = g * 4 + k;
      st->carrier[i].ProcessBlock(st->carrierbuf, n);
      // voice = (carrier / n) * env_kr — the kr input ramps (mul_ak_nova);
      // the FDiv by the literal n runs BEFORE the multiply (voice 0's /1 is
      // folded away by sclang, and x/1.f is exact).
      const float v = st->env[i].Process();
      const float prev = st->ramp_prev[i];
      st->ramp_prev[i] = v;
      MulRamp64(st->carrierbuf, st->voicebuf, n, prev, v,
                static_cast<float>(i + 1));
      // Pan2: per-block pos = base + center (kr scalar+kr add, no ramp).
      st->pan[i].ProcessBlock(st->voicebuf, st->pan_l, st->pan_r, n,
                              st->base[i] + center_k, 1.f);
      // Mix tree (def dump 108/118/128/138_Sum4): each group of 4 pans is
      // Sum4(p3, p2, p1, p0) = (p3+p2) + (p1+p0) — the optimizer reverses the
      // operands. Streaming: voices 0,1 build pairB = (p1+p0); 2,3 build
      // pairA = (p3+p2); s_g = pairA + pairB.
      float* tl = (k < 2) ? st->t01_l : st->t23_l;   // t01 = pairB, t23 = pairA
      float* tr = (k < 2) ? st->t01_r : st->t23_r;
      if (k == 1 || k == 3) {
        for (int j = 0; j < n; ++j) { tl[j] += st->pan_l[j]; tr[j] += st->pan_r[j]; }
      } else {
        for (int j = 0; j < n; ++j) { tl[j] = st->pan_l[j]; tr[j] = st->pan_r[j]; }
      }
    }
    // s_g = pairA + pairB
    for (int j = 0; j < n; ++j) {
      st->t23_l[j] = st->t23_l[j] + st->t01_l[j];
      st->t23_r[j] = st->t23_r[j] + st->t01_r[j];
    }
    // Final Sum4 (def dump 139/142): Sum4(g3, g2, g1, g0) = (g3+g2)+(g1+g0).
    if (g == 0) {
      for (int j = 0; j < n; ++j) { st->acc_b_l[j] = st->t23_l[j]; st->acc_b_r[j] = st->t23_r[j]; }
    } else if (g == 1) {
      for (int j = 0; j < n; ++j) { st->acc_b_l[j] = st->acc_b_l[j] + st->t23_l[j]; st->acc_b_r[j] = st->acc_b_r[j] + st->t23_r[j]; }
    } else if (g == 2) {
      for (int j = 0; j < n; ++j) { st->acc_a_l[j] = st->t23_l[j]; st->acc_a_r[j] = st->t23_r[j]; }
    } else {
      for (int j = 0; j < n; ++j) { st->acc_a_l[j] = st->acc_a_l[j] + st->t23_l[j]; st->acc_a_r[j] = st->acc_a_r[j] + st->t23_r[j]; }
    }
  }
  // Final Sum4 of the group sums, then the level multiply.
  for (int j = 0; j < n; ++j) {
    const float mix_l = st->acc_a_l[j] + st->acc_b_l[j];
    const float mix_r = st->acc_a_r[j] + st->acc_b_r[j];
    l[j] = mix_l * amp_level;
    r[j] = mix_r * amp_level;
  }
}

}  // namespace drones
}  // namespace sc
#endif
