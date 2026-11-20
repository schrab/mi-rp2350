// Starlids — ported from drones/starlids.scd.
// Self-contained port: state, Init/Process, and laws live here.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_STARLIDS_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_STARLIDS_H_
#include "drone_common.h"
namespace sc {
namespace drones {
static const float kStarlidsC[108] = {
  1.0f, 12648430.0f, 12.0f, 0.5f, 0.0f, 0.300000012f,
  0.0320459343f, 3.95437479f, 0.0102517661f, 1.97606909f, 4.0f, 5.0f,
  0.067432642f, 2.73871374f, -1.0f, 20.0f, 12000.0f, 10.0f,
  0.00999999978f, 0.0199999996f, 15.0f, 0.0107960012f, 0.333333343f, -99.0f,
  3.0f, 0.0295727644f, 0.105217747f, 0.0160599705f, 3.2140224f, 0.0910715088f,
  5.55292892f, 0.0298742205f, 0.0285571441f, 0.961855412f, 0.0285705514f, 0.339731812f,
  0.073877044f, 1.15625823f, 0.0197981745f, 0.0226839408f, 5.98207808f, 0.0262414627f,
  0.0721315518f, 0.088987112f, 5.10827732f, 0.0196666215f, 0.020462295f, 3.92784929f,
  0.0141709484f, 5.65260553f, 0.0982596353f, 4.11848831f, 0.0225396231f, 0.0202195793f,
  0.621015072f, 0.0231158268f, 0.404486328f, 0.0653149709f, 0.658406556f, 0.0192626063f,
  0.0149456179f, 1.96849287f, 0.0195837785f, 3.0541153f, 0.0606023483f, 2.76960611f,
  0.0297154021f, 0.0301861148f, 1.21443188f, 0.0211817361f, 4.70441818f, 0.0768103302f,
  5.96775675f, 0.0170052331f, 0.0109617403f, 1.87655807f, 0.0234792884f, 6.16228151f,
  0.0357598141f, 1.14085257f, 0.0120437499f, 0.0121552423f, 5.58400106f, 0.0164275896f,
  5.73252964f, 0.0840650722f, 2.01449943f, 0.0182993263f, 0.0235633943f, 2.33764744f,
  0.012379366f, 2.47296119f, 0.0515018925f, 0.190413609f, 0.0282435585f, 0.0233099516f,
  2.84768248f, 0.0132257966f, 2.28025937f, 0.0608886257f, 2.97993302f, 0.0114521245f,
  0.166666672f, 6.0f, 8.0f, 60.0f, 2.0f, -4.0f,
};

static const int kStVoice[12][7] = {
  {6,7,8,9,12,13,21},   {25,26,27,28,29,30,31},
  {32,33,34,35,36,37,38}, {39,40,41,42,43,44,45},
  {46,47,48,49,50,51,52}, {53,54,55,56,57,58,59},
  {60,61,62,63,64,65,66}, {67,68,69,70,71,72,73},
  {74,75,76,77,78,79,80}, {81,82,83,84,85,86,87},
  {88,89,90,91,92,93,94}, {95,96,97,98,99,100,101},
};

// ---------------------------------------------------------------------------
// Starlids (id 24) — AUDIO-EQUIVALENT (2026-11-20): max 0.134096749 / corr
// 0.858709220; spectral envelope L1 distance 0.105 (same timbre). Gate
// 0.15/0.85. 12 detuned SawDPW voices + PWM Pulse base. Each voice:
// 2 LFTri.kr floors (interval semitones) → midicps → SawDPW → LPF(sweep) →
// DelayC(chorus) → Pan2(VarLag pan)/12*amp. Mix → tanh → MoogLadder → HPF.
struct StVoice {
  LFTriP tri1, tri2;
  SinOscAF sinlfo;   // kr SinOsc with a phase argument (phase in radians)
  LFNoise1 lfn1;
  DelayLine delay;
  VarLag vl;
  LFNoise0 lf0;
  SawDPW saw;
  Filter2Pole lpf;
  Pan2 pan;
  float saw_ctor, lpf_ctor, pan_prev;
};
// T-D2 device memory law: the 12 chorus DelayC rings are arena/aux-owned
// (NO heap — the old std::vector rings asked for 48 MB: the declared SC
// maxdelay is 15 s while the live delaytime is (lfn*0.01+0.02)/15 s ->
// dsamp <= 96.0000045). Ring = 96 + 8 = 104 floats (ext clamp ceiling
// ring - 4 = 100 >= 96.0000045, never engages — golden bit-identical).
struct StState {
  StVoice v[12];
  LFTriP base_tri;
  SinOsc base_sin;
  Pulse base_pulse;
  Pan2 base_pan;
  LFNoise0 tail_lf0;
  VarLag tail_vl;
  LinExp tail_le;
  MoogLadder ladder;
  Filter2Pole hpf;
  float* delay_ring[12];
  float sawbuf[64], lpbuf[64], dlybuf[64], panl[64], panr[64];
  float mixl[64], mixr[64], tanl[64], tanr[64];
};

inline void St_Init(Ctx& c) {
  // T-D2: the base PWM voice is a BLIT Pulse (full gSine/gInvSine tables).
  if (!EnsurePulseTables(c)) { c.state = nullptr; return; }
  StState* st = new (c.arena->Alloc(sizeof(StState))) StState();
  if (st == nullptr) { c.state = nullptr; return; }
  for (int k = 0; k < 12; ++k) {
    st->delay_ring[k] =
        static_cast<float*>(AllocRing(c, 104u * 4u));
    if (st->delay_ring[k] == nullptr) { c.state = nullptr; return; }
  }
  const double sr = static_cast<double>(c.sr);
  const double cr = sr / static_cast<double>(kBlock);
  const float hz = c.hz;
  const float midi = NovaFreq2Midi(hz);   // ar cpsmidi == 33 for 55 Hz
  // Base PWM voice: Pulse(midicps(note-12), LinLin(LFTri(0.5),-1,1,0.2,0.8)).
  st->base_tri.Init(cr, kStarlidsC[3], 0.0f);
  st->base_sin.Init(c.tables, cr, 0.0f);
  st->base_pulse.Init(c.tables, sr, NovaMidi2Freq(midi - 12.0f),
                      kStarlidsC[3]);
  st->base_pan.Init(c.tables, 0.0f, 1.0f, 0.0f);
  // language-side constants for each voice: RGen draws in creation order.
  for (int k = 0; k < 12; ++k) {
    StVoice& vc = st->v[k];
    const int* ix = kStVoice[k];
    const float t1f = kStarlidsC[ix[0]], t1p = kStarlidsC[ix[1]];
    const float t2f = kStarlidsC[ix[2]], t2p = kStarlidsC[ix[3]];
    const float sf = kStarlidsC[ix[4]], sp = kStarlidsC[ix[5]];
    const float md = kStarlidsC[ix[6]];
    // 29_Rand (scalar) = Rand(5,10) draws one frand at ctor (creation order).
    const float rnd = RandIrg(c.rgen, 5.0f, 10.0f);
    vc.tri1.Init(cr, t1f, t1p);
    vc.tri2.Init(cr, t2f, t2p);
    vc.sinlfo.Init(c.tables, cr, sp);
    vc.lfn1.Init(c.rgen, cr, rnd);
    vc.lf0.Init(c.rgen, cr, kStarlidsC[102]);   // LFNoise0.kr(1/3)
    vc.vl.Init(cr, vc.lf0.ctor_value(), kStarlidsC[24], 0.0f, 3);
    const float f0 = NovaMidi2Freq(midi);  // ctor LFOs prime to 0 => base note
    vc.saw.Init(sr, f0, 0.0);
    vc.lpf.Init(sr, 0);
    vc.delay.InitExt(st->delay_ring[k], 104, sr, md, 0.0f, 0.0f, 2);
    vc.pan.Init(c.tables, 0.0f, 1.0f, 0.0f);
    vc.saw_ctor = f0; vc.lpf_ctor = 0.0f; vc.pan_prev = 0.0f;
  }
  st->tail_lf0.Init(c.rgen, cr, kStarlidsC[102]);
  st->tail_vl.Init(cr, st->tail_lf0.ctor_value(), kStarlidsC[103], 0.0f, 3);
  st->tail_le.Init(-1.f, 1.f, hz * 8.0f, hz * 60.0f);
  st->ladder.Init(sr, hz * 8.0f, 0.0f);
  st->hpf.Init(sr, 1);
  c.state = st;
}

inline void St_Process(Ctx& c, float* l, float* r, int n) {
  StState* st = static_cast<StState*>(c.state);
  const float hz = c.hz, amp = c.amp;
  const float midi = NovaFreq2Midi(hz);
  // Base PWM voice.
  const float base_width = st->base_tri.Process() * kStarlidsC[5] + kStarlidsC[3];
  st->base_pulse.SetFreq(NovaMidi2Freq(midi - 12.0f));
  st->base_pulse.SetWidth(base_width);
  st->base_pulse.ProcessBlock(st->sawbuf, n);
  for (int i = 0; i < n; ++i) st->sawbuf[i] = (st->sawbuf[i] / kStarlidsC[2]) * amp;
  st->base_pan.ProcessBlock(st->sawbuf, st->mixl, st->mixr, n, 0.0f, 1.0f);
  // 12 voices.
  for (int k = 0; k < 12; ++k) {
    StVoice& vc = st->v[k];
    const float o1 = std::floor(vc.tri1.Process() + 1.0f) * kStarlidsC[10];
    const float o2 = std::floor(vc.tri2.Process() + 1.0f) * kStarlidsC[11];
    const float freq = NovaMidi2Freq(midi + o1 + o2);
    float sf = 0.0f;
    vc.sinlfo.ProcessBlock(kStarlidsC[kStVoice[k][4]], &sf, 1);
    const float lpf_freq = 20.0f * std::pow(12000.0f / 20.0f,
                                            (sf + 1.0f) * 0.5f);
    const float dly = (vc.lfn1.ProcessKr() * kStarlidsC[18] + kStarlidsC[19])
                    / kStarlidsC[20];
    const float pan = vc.vl.ProcessKr(vc.lf0.ProcessKr());
    vc.saw.SetFreq(freq);
    vc.saw.ProcessBlock(st->sawbuf, n);
    vc.lpf.ProcessBlock(st->sawbuf, st->lpbuf, n, lpf_freq, 0.0f);
    vc.delay.ProcessBlock(st->lpbuf, st->dlybuf, n, dly, 0.0f);
    vc.pan.ProcessBlock(st->dlybuf, st->panl, st->panr, n, pan, 1.0f);
    for (int i = 0; i < n; ++i) {
      st->panl[i] = (st->panl[i] / kStarlidsC[2]) * amp;
      st->panr[i] = (st->panr[i] / kStarlidsC[2]) * amp;
    }
    // group accumulation (float association within 1e-7)
    for (int i = 0; i < n; ++i) {
      st->mixl[i] += st->panl[i];
      st->mixr[i] += st->panr[i];
    }
  }
  const float cutoff = st->tail_le.Next(st->tail_vl.ProcessKr(
      st->tail_lf0.ProcessKr()));
  for (int i = 0; i < n; ++i) {
    st->tanl[i] = NovaTanh(st->mixl[i]);
    st->tanr[i] = NovaTanh(st->mixr[i]);
  }
  st->ladder.ProcessBlock(st->tanl, st->panl, n, cutoff, 0.0f);
  st->hpf.ProcessBlock(st->panl, st->dlybuf, n, 20.0f, 0.0f);
  st->ladder.ProcessBlock(st->tanr, st->panr, n, cutoff, 0.0f);
  st->hpf.ProcessBlock(st->panr, st->lpbuf, n, 20.0f, 0.0f);
  for (int i = 0; i < n; ++i) { l[i] = st->dlybuf[i]; r[i] = st->lpbuf[i]; }
}

inline const DroneSpec kDroneStarlids = {"Starlids", drones::St_Init,
                                         drones::St_Process};


}  // namespace drones
}  // namespace sc
#endif
