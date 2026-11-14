// lira.voice + prm.* verbatim port — one voice PAIR (2 osc paths).
// Sourced from LIRA-8 abs/lira.voice.pd + prm.*.pd (connect-list verified),
// take-1 math, PD-PRIMITIVES.md laws.
//
// Per osc path k: freq_k = T_k*(1 + 0.005*V)*(1 + M*mod)
//   T_k = line~(23.22ms) of mtof(note_k)*pitch (quantize optional),
//   note_k = hi_k + (tune_k/127)*(lo_k - hi_k)   [per-osc tune table]
//   pitch = (P/127)*1.99 + 0.01, quantize: mtof(round(ftom(Hz*pitch)))
//   V = cos(2*pi*3*t) * (0.5 + 3*pd_random(seed + pair_first)/1000) * vib
//   M = mod-source signal (other pair's bus, one-block delayed by the bus)
//   mod = (MOD/127)^4 * 2 [line~ 23.22ms], pwm = 0.53125 + 0.025*V
//   sharp = (S/127)^2 [line~], hold = (H/127)^2 [line~]
//   mix_k = sq_k*sharp + tri_k*(1 - sharp) + cos_k     (cos_k = sensor cos-morph)
//   gate_k = min(e_k^2, hold)                          (e = sensor line~ A/R)
//   out_k = mix_k * gate_k
// pair_out = out_A + out_B; bus_send = hip~3(pair_out)   [to s~ del-XY]
#ifndef RACK_SRC_LIRA8_LIRA_VOICE_H_
#define RACK_SRC_LIRA8_LIRA_VOICE_H_

#include <cstdint>
#include <cmath>

#include "pd_primitives.h"
#include "lira_osc.h"

namespace l8pd {

// pd's [random] object LCG (x_arithmetic.c): state = state*472940017 + 1,
// result = (state * range) >> 32. Verified empirically against pd during
// golden phase (deterministic given seed).
inline int pd_random(uint32_t* state, int range) {
  *state = *state * 472940017u + 1u;
  return static_cast<int>((static_cast<uint64_t>(*state) *
                           static_cast<uint64_t>(range)) >> 32);
}

// Per-osc tune table (lo, hi MIDI notes) from prm.tune's text-seq rows.
static const float kTuneTable[8][2] = {
    {-16.f, 93.f},   {-16.f, 93.f},   {7.f, 109.f},    {9.f, 107.f},
    {20.f, 116.54f}, {20.f, 116.54f}, {33.f, 126.24f}, {33.f, 131.22f},
};

// pd default-PRNG draws of the four [random 1001] objects at patch load
// (probe: 744, 930, 728, 920 — seed machinery dormant, deterministic).
// These are the per-pair VIBRATO RATES in Hz: rate = 0.5 + 3*draw/1000
// (the random chain feeds the osc~'s frequency inlet, connect-list verified
// 2026-11-15; take-1's "depth" reading and this file's earlier "depth"
// misuse both corrected). V = cos(rate*t) * vib_param, V in [-1, +1].
static const int kVibDraws[4] = {744, 930, 728, 920};
static const float kVibRate[4] = {2.732f, 3.290f, 2.684f, 3.260f};

// Sensor envelope (prm.sensor): line~ A/R with e^2 gate + cos morph.
class SensorEnv {
 public:
  SensorEnv(float sr) : line_(sr) {}
  // fast=true: attack/release 100/100 ms; fast=false: 200/8000 ms.
  void set_fast(bool fast) {
    attack_ms_ = fast ? 100.0f : 200.0f;
    release_ms_ = fast ? 100.0f : 8000.0f;
    if (on_) line_.set(1.0f, attack_ms_);   // s $0-trigger re-attack
  }
  void set(bool on) {
    on_ = on;
    line_.set(on ? 1.0f : 0.0f, on ? attack_ms_ : release_ms_);
  }
  inline void process(float& e2, float& cos_term) {
    const float e = line_.next();
    e2 = e * e;
    cos_term = std::cos(2.0f * kPi * (e - 0.25f)) * 0.5f;
  }

 private:
  Ramp line_;
  float attack_ms_ = 100.0f, release_ms_ = 8000.0f;
  bool on_ = false;
};

class VoicePair {
 public:
  // pair 0..3 (voices 2i+1 / 2i+2). seed_base kept for API compat: the
  // patch's $0-seed receiver is DORMANT (nothing sends to it), so the
  // vibrato depths are the MEASURED pd default-PRNG draws (744/930/728/920
  // from `random 1001` at patch load, probed 2026-11-15) — deterministic.
  VoicePair(float sr, int pair, uint32_t seed_base)
      : sr_(sr),
        pair_(pair),
        osc_a_(sr),
        osc_b_(sr),
        sensor_a_(sr),
        sensor_b_(sr),
        tune_a_(sr),
        tune_b_(sr),
        sharp_(sr),
        mod_(sr),
        hold_(sr),
        vib_(sr),
        osc3_(sr),
        hip_(3.0f, sr) {
    (void)seed_base;
    osc3_.set_freq(kVibRate[pair]);
  }

  // ---- control-rate setters (values are raw 0..127 param bytes) ----
  void set_tune(int osc /*0|1*/, int v) {
    const float* row = kTuneTable[pair_ * 2 + osc];
    // LAW CORRECTED (empirical 2026-11-15): pd renders k=127 -> hi note
    // (1838 Hz = mtof(93)*pitch measured), k=0 -> lo (sub-audio). The spec
    // §7.2 / take-1 law (hi + k*(lo-hi)) was misread from the unpack order.
    const float note = row[0] + (v / 127.0f) * (row[1] - row[0]);
    if (osc == 0) tune_a_hz_ = mtof(note);
    else tune_b_hz_ = mtof(note);
  }
  void set_pitch(int v) {   // shared by the pair (prm.pitch)
    pitch_ = (v / 127.0f) * 1.99f + 0.01f;
  }
  void set_quantize(int v) { quantize_ = (v != 0); }
  void set_sharp(int v) { sharp_raw_ = (v / 127.0f); sharp_.set(sharp_raw_ * sharp_raw_, 23.22f); }
  void set_mod(int v) { mod_raw_ = (v / 127.0f); mod_.set(mod_raw_ * mod_raw_ * mod_raw_ * mod_raw_ * 2.0f, 23.22f); }
  void set_hold(int v) { hold_.set((v / 127.0f) * (v / 127.0f), 23.22f); }
  void set_vibrato(int v) { vib_.set(v / 127.0f, 23.22f); }
  void set_sensor(int osc /*0|1*/, bool on) {
    (osc == 0 ? sensor_a_ : sensor_b_).set(on);
  }
  void set_fast(bool fast) {
    sensor_a_.set_fast(fast);
    sensor_b_.set_fast(fast);
  }

  // Apply tune/pitch/quantize -> Hz (recomputed at control rate, feeds line~).
  void refresh_tune() {
    auto hz = [&](Ramp& line, float base_hz) {
      float h = base_hz * pitch_;
      if (quantize_) h = mtof(std::round(ftom(h)));
      line.set(h, 23.22f);
    };
    hz(tune_a_, tune_a_hz_);
    hz(tune_b_, tune_b_hz_);
  }

  // ---- per-sample render; mod_sig = mod-source signal (other pair bus) ----
  inline void process(const float* mod_sig, int n, float* pair_out,
                      float* bus_send) {
    for (int i = 0; i < n; ++i) {
      const float V = osc3_.next() * vib_.next();
      const float sharp = sharp_.next();
      const float mod = mod_.next();
      const float hold = hold_.next();
      const float pwm = 0.53125f + 0.025f * V;

      float e2a, cosa, e2b, cosb;
      sensor_a_.process(e2a, cosa);
      sensor_b_.process(e2b, cosb);

      const float fA = tune_a_.next() * (1.0f + 0.005f * V) *
                       (1.0f + mod_sig[i] * mod);
      const float fB = tune_b_.next() * (1.0f + 0.005f * V) *
                       (1.0f + mod_sig[i] * mod);

      float sqA, triA, sqB, triB;
      osc_a_.process(fA, pwm, sqA, triA);
      osc_b_.process(fB, pwm, sqB, triB);

      const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
      const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
      // Gate law (connect-list verified): e^2 AND hold SUM into the min~
      // inlet (pd signal inlets sum) -> gate = min(e^2 + hold, 1). hold=0:
      // normal envelope; hold=1: latch-on (the Lyra's hold button).
      float gateA = e2a + hold;
      if (gateA > 1.0f) gateA = 1.0f;
      float gateB = e2b + hold;
      if (gateB > 1.0f) gateB = 1.0f;
      const float outA = mixA * gateA;
      const float outB = mixB * gateB;

      const float pair = outA + outB;
      pair_out[i] = pair;
      bus_send[i] = hip_.process(pair);
    }
  }

 private:
  float sr_;
  int pair_ = 0;
  LiraOsc osc_a_, osc_b_;
  SensorEnv sensor_a_, sensor_b_;
  Ramp tune_a_, tune_b_;        // final smoothed Hz
  float tune_a_hz_ = 0.f, tune_b_hz_ = 0.f;
  float pitch_ = (64.f / 127.f) * 1.99f + 0.01f;
  bool quantize_ = false;
  Ramp sharp_, mod_, hold_, vib_;
  float sharp_raw_ = 0.f, mod_raw_ = 0.f;
  CosOsc osc3_;
  Hip hip_;
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_VOICE_H_
