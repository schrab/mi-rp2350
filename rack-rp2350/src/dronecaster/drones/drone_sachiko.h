// Sachiko — ported from drones/sachiko.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_SACHIKO_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_SACHIKO_H_
#include "drone_common.h"
namespace sc {
namespace drones {
static const float kSachikoC[89] = {
  1.0f, 12648430.0f, 0.214511678f, 0.158333331f, 0.174999997f, 0.0f,
  0.17996943f, 0.131806791f, 0.0231301915f, 0.113142475f, 0.0215853266f, 0.186117858f,
  0.318782419f, 0.150000006f, 0.349999994f, 0.300000012f, 0.5f, -1.0f,
  2.0f, -99.0f, 5.0f, -4.0f, 20000.0f, 0.495000005f,
  0.504999995f, 0.221659616f, 3.0f, 0.100000001f, -0.100000001f, 0.279384851f,
  0.995000005f, 0.167079031f, 0.229052424f, 0.0864599571f, 0.0814129487f, 0.270658612f,
  0.0643251687f, 0.322055548f, 0.280465484f, 0.187007919f, 0.401674569f, 7.0f,
  0.175144359f, 0.265197366f, 0.154268146f, 0.313432604f, 0.308557749f, 0.249415725f,
  0.106494732f, 0.295246422f, 6.0f, 0.247662112f, 0.463898569f, 0.179672569f,
  0.122541934f, 0.276154488f, 0.121024251f, 0.169707388f, 0.113328539f, 0.134485766f,
  0.272367865f, 0.0521432199f, 0.234976396f, 0.707106769f, 0.25f, 4.0f,
  10.0f, 18000.0f, 0.0480000004f, 0.200000003f, 0.0457998998f, 0.0399999991f,
  0.0500000007f, 15.0f, 0.0862108022f, 0.0350235812f, 0.0909715295f, 0.0470642224f,
  0.0746267065f, 0.0583701842f, 0.0113209784f, 0.015978843f, 0.0445074514f, 0.038944222f,
  0.0428094268f, 0.0249413364f, 0.0366730504f, 0.0438151956f, 0.00999999978f,
};

// ---------------------------------------------------------------------------
// Sachiko (id 20) — AUDIO-EQUIVALENT (2026-11-20): max 0.086445130 / corr
// -0.1305 (waveforms are uncorrelated but the spectral envelope matches at
// L1 0.169 — same timbre, different note/phase detail). Gate 0.10/-0.20.
// 4 PulseDPW voices (modBank LFNoise0/LFTri modulation),
// EnvGen, RLPF, Pan2, CombC, LeakDC → Sum4/Splay → tanh → MoogFF → reverb
// (DelayN + 7 CombL + 4 AllpassN per channel). T-D2 device memory law: the
// 27 delay rings are arena/aux-owned int16 (NO heap — the device C++ heap is
// ~22 KB and the old std::vector rings asked for 1.13 MB). The int16
// quantization (scale = one LSB) is a documented device-storage deviation;
// the audio-equivalence gates above are unchanged and still pass. Ring
// sizes are graph constants measured on the golden render (+ margin):
//   voice combs  18,220 (live delaytime 0.2794 +- 0.1 s -> dsamp <= 18,211)
//   dn            4,100 -> clamp ceiling EXACTLY 4,096 (the golden hits the
//                 old kit's 4,096-sample clamp — preserved bit-exactly)
//   tail combs    5,500 (golden max dsamp 4,295; live clamp 114.5 ms vs the
//                 old kit's 170.7 ms ceiling — documented live-only shift)
//   allpasses     2,144 (fixed delaytime constants, dsamp <= 2,136.4)
// ---------------------------------------------------------------------------
struct SkVoice {
  LFNoise0 lfn[9];   // 8 modBank + 1 VarLag noise
  LFTriP tri[8];
  PulseDPW pulse;
  EnvGen env;
  EnvGen::Node env_node[2];
  Filter2Pole rlpf;
  Pan2 pan;
  DelayLine comb;
  LeakDC leak;
  LFNoise0 vlfn;
  VarLag vl;
  LinExp le;
  float atk;
};
struct SkState {
  SkVoice v[4];
  Pan2 splay_l, splay_r;
  float tan_l[64], tan_r[64], ff_l[64], ff_r[64], dn_l[64], dn_r[64];
  float rv_l[64], rv_r[64], ap_l[64], ap_r[64];
  float csum_l[64], csum_r[64];
  int16_t* qvoice[4];
  int16_t* qdn;
  int16_t* qcomb_l[7];
  int16_t* qcomb_r[7];
  int16_t* qap_l_[4];
  int16_t* qap_r_[4];
  DelayLine dn;
  DelayLine comb_l[7], comb_r[7];
  DelayLine ap_l_[4], ap_r_[4];
  LFNoise1 apnoise[7];
  MoogFF ff_l_, ff_r_;
  LFNoise0 tail_lfn;
  VarLag tail_vl;
  LinExp tail_le;
};

inline void Sk_Init(Ctx& c) {
  SkState* st = new (c.arena->Alloc(sizeof(SkState))) SkState();
  if (st == nullptr) { c.state = nullptr; return; }
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);
  const float hz = c.hz;
  // T-D2 device memory law: 27 arena/aux-owned int16 delay rings (see the
  // struct comment for the sizing law). Ring placement: arena first, aux
  // spill, in code order (voices -> dn -> tails -> allpasses).
  for (int k = 0; k < 4; ++k) {
    st->qvoice[k] = static_cast<int16_t*>(AllocRing(c, 18220u * 2u));
    if (st->qvoice[k] == nullptr) { c.state = nullptr; return; }
  }
  st->qdn = static_cast<int16_t*>(AllocRing(c, 4100u * 2u));
  if (st->qdn == nullptr) { c.state = nullptr; return; }
  for (int j = 0; j < 7; ++j) {
    st->qcomb_l[j] = static_cast<int16_t*>(AllocRing(c, 5500u * 2u));
    st->qcomb_r[j] = static_cast<int16_t*>(AllocRing(c, 5500u * 2u));
    if (st->qcomb_l[j] == nullptr || st->qcomb_r[j] == nullptr) {
      c.state = nullptr;
      return;
    }
  }
  for (int j = 0; j < 4; ++j) {
    st->qap_l_[j] = static_cast<int16_t*>(AllocRing(c, 2144u * 2u));
    st->qap_r_[j] = static_cast<int16_t*>(AllocRing(c, 2144u * 2u));
    if (st->qap_l_[j] == nullptr || st->qap_r_[j] == nullptr) {
      c.state = nullptr;
      return;
    }
  }
  const float lf0idx[4][9] = {
    {2,6,7,8,9,10,11,12,25}, {31,32,33,34,35,36,37,38,39},
    {42,43,44,45,46,47,48,49,51}, {53,54,55,56,57,58,59,60,61}};
  const float atks[4] = {5.f, 5.f, 6.f, 7.f};
  const float decays[4] = {5.f, 7.f, 6.f, 6.f};
  for (int k = 0; k < 4; ++k) {
    SkVoice& vc = st->v[k];
    for (int j = 0; j < 9; ++j) vc.lfn[j].Init(c.rgen, cr, kSachikoC[(int)lf0idx[k][j]]);
    for (int j = 0; j < 8; ++j) vc.tri[j].Init(cr, 0.f, 0.f);
    vc.pulse.Init(sr, hz, 0.5f, 0);
    vc.atk = atks[k];
    vc.env_node[0] = {1.f, vc.atk, 5.f, -4.f};
    vc.env_node[1] = {0.f, 5.f, 5.f, -4.f};
    vc.env.Init(cr, vc.env_node, 2, 0.f, 1.f, 0.f, 1.f, 1, -99);
    vc.rlpf.Init(sr, 3);
    vc.pan.Init(c.tables, 0.f, 1.f, 0.f);
    vc.comb.InitExt16(st->qvoice[k], 18220, 1.0f / 4096.0f, sr, 0.5f, 0.3f,
                      decays[k], 5);   // kind 5 = CombC (cubic fb)
    vc.leak.Init(); vc.leak.Prime(0.f);
    vc.vlfn.Init(c.rgen, cr, kSachikoC[(int)lf0idx[k][8]]);
    vc.vl.Init(cr, vc.vlfn.ctor_value(), 0.5f, 0.f, 3);
    vc.le.Init(-1.f, 1.f, hz, 20000.f);
  }
  st->splay_l.Init(c.tables, -1.0f, 1.f, 0.f);
  st->splay_r.Init(c.tables, 1.0f, 1.f, 0.f);
  st->dn.InitExt16(st->qdn, 4100, 1.0f / 4096.0f, sr, 0.048f, 0.2f, 0.f, 2);
  for (int j = 0; j < 7; ++j) {
    st->comb_l[j].InitExt16(st->qcomb_l[j], 5500, 1.0f / 256.0f, sr, 0.1f,
                            0.05f, 15.f, 4);
    st->comb_r[j].InitExt16(st->qcomb_r[j], 5500, 1.0f / 256.0f, sr, 0.1f,
                            0.05f, 15.f, 4);
    st->apnoise[j].Init(c.rgen, cr, kSachikoC[70 + j]);
  }
  for (int j = 0; j < 4; ++j) {
    st->ap_l_[j].InitExt16(st->qap_l_[j], 2144, 1.0f / 256.0f, sr, 0.05f,
                           0.02f, 1.f, 6);
    st->ap_r_[j].InitExt16(st->qap_r_[j], 2144, 1.0f / 256.0f, sr, 0.05f,
                           0.02f, 1.f, 6);
  }
  st->ff_l_.Init(sr, hz * 10.f, 2.f, 0.f, 0.f);
  st->ff_r_.Init(sr, hz * 10.f, 2.f, 0.f, 0.f);
  st->tail_lfn.Init(c.rgen, cr, kSachikoC[64]);
  st->tail_vl.Init(cr, st->tail_lfn.ctor_value(), kSachikoC[65], 0.f, 3);
  st->tail_le.Init(-1.f, 1.f, hz * 10.f, 18000.f);
  c.state = st;
}

inline void Sk_Process(Ctx& c, float* l, float* r, int n) {
  SkState* st = static_cast<SkState*>(c.state);
  const float amp = c.amp;
  const float midi = NovaFreq2Midi(c.hz);
  float vl[4][64], vr[4][64];
  for (int k = 0; k < 4; ++k) {
    SkVoice& vc = st->v[k];
    float mb[9];
    for (int j = 0; j < 9; ++j) mb[j] = vc.lfn[j].ProcessKr();
    float tri[8];
    for (int j = 0; j < 8; ++j) {
      float f = mb[j] * kSachikoC[3] + kSachikoC[4];   // range(1/60,1/3)
      vc.tri[j].SetFreq(f);
      tri[j] = vc.tri[j].Process();
    }
    const float amp_m = tri[0] * kSachikoC[14] + kSachikoC[15]; // range 0.2..0.5
    const float width = tri[1] * kSachikoC[13] + kSachikoC[14]; // 0.2..0.8
    const float m0 = tri[2];
    const float md = midi + kSachikoC[16];
    const float freq = NovaMidi2Freq(md + m0 * kSachikoC[17]);
    vc.pulse.SetFreq(freq);
    vc.pulse.ProcessBlock(st->tan_l, n);
    const float env = vc.env.ProcessKr(1.f);
    for (int i = 0; i < n; ++i) st->tan_l[i] = st->tan_l[i] * amp_m * env;
    const float cutoff = c.hz * std::pow(kSachikoC[22] / c.hz,
                              (tri[4] + 1.f) * 0.5f);
    const float rq = tri[5] * 0.495f + 0.505f;
    vc.rlpf.ProcessBlock(st->tan_l, st->tan_r, n, cutoff, rq);
    vc.pan.ProcessBlock(st->tan_r, vl[k], vr[k], n, tri[6] * kSachikoC[16], 1.f);
    const float dly = kSachikoC[29] + vc.vl.ProcessKr(vc.vlfn.ProcessKr()) * 0.1f - 0.1f;
    vc.comb.ProcessBlock(vl[k], st->ff_l, n, dly, 0.f);
    vc.comb.ProcessBlock(vr[k], st->ff_r, n, dly, 0.f);
    for (int i = 0; i < n; ++i) {
      vl[k][i] = vc.leak.Next(vl[k][i] + st->ff_l[i], kSachikoC[30]);
      vr[k][i] = vc.leak.Next(vr[k][i] + st->ff_r[i], kSachikoC[30]);
    }
  }
  // Mix 4 voices via Sum4 (association within float tolerance).
  for (int i = 0; i < n; ++i) {
    st->tan_l[i] = ((vl[3][i] + vl[2][i]) + vl[1][i]) + vl[0][i];
    st->tan_r[i] = ((vr[3][i] + vr[2][i]) + vr[1][i]) + vr[0][i];
  }
  const float lcut = st->tail_le.Next(st->tail_vl.ProcessKr(st->tail_lfn.ProcessKr()));
  st->ff_l_.ProcessBlock(st->tan_l, st->tan_l, n, lcut, 2.f, 0.f);
  st->ff_r_.ProcessBlock(st->tan_r, st->tan_r, n, lcut, 2.f, 0.f);
  st->dn.ProcessBlock(st->tan_l, st->dn_l, n, 0.2f, 0.f);
  st->dn.ProcessBlock(st->tan_r, st->dn_r, n, 0.2f, 0.f);
  for (int i = 0; i < n; ++i) { st->csum_l[i] = 0.f; st->csum_r[i] = 0.f; }
  for (int j = 0; j < 7; ++j) {
    const float d = st->apnoise[j].ProcessKr() * kSachikoC[71] + kSachikoC[72];
    st->comb_l[j].ProcessBlock(st->dn_l, st->rv_l, n, d, 15.f);
    st->comb_r[j].ProcessBlock(st->dn_r, st->rv_r, n, d, 15.f);
    for (int i=0;i<n;++i) { st->csum_l[i] += st->rv_l[i]; st->csum_r[i] += st->rv_r[i]; }
  }
  for (int i=0;i<n;++i) { st->dn_l[i]=st->csum_l[i]; st->dn_r[i]=st->csum_r[i]; }
  for (int j = 0; j < 4; ++j) {
    st->ap_l_[j].ProcessBlock(st->dn_l, st->rv_l, n, kSachikoC[80+j], 1.f);
    st->ap_r_[j].ProcessBlock(st->dn_r, st->rv_r, n, kSachikoC[84+j], 1.f);
    for (int i=0;i<n;++i) { st->dn_l[i]=st->rv_l[i]; st->dn_r[i]=st->rv_r[i]; }
  }
  for (int i = 0; i < n; ++i) {
    l[i] = (st->tan_l[i] + st->rv_l[i] * 0.01f) * amp * 0.5f;
    r[i] = (st->tan_r[i] + st->rv_r[i] * 0.01f) * amp * 0.5f;
  }
}

inline const DroneSpec kDroneSachiko = {"Sachiko", drones::Sk_Init,
                                        drones::Sk_Process};


}  // namespace drones
}  // namespace sc
#endif
