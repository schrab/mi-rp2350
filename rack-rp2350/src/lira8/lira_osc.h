// os.triangle~ verbatim port — polyBLEP pulse osc with PWM + filtered
// triangle output. Sourced from LIRA-8 abs/os.triangle~.pd (connect-list
// verified) + take-2 derivation; laws cross-checked in PD-PRIMITIVES.md.
//
// Graph: abs(freq) -> phasor~; phase -> 2x polyblep(p, f/sr) with the
// patch's inverted-sign formula (canonical usage SUBTRACTS standard blep,
// so (2p-1) + blep_patch is correct); pulse = A - B + 2*pw - 1 where
// pw = samphold(clip(PWM,0,1), phase) sampled on phase wrap;
// square out = 0.59 * pulse; triangle out = 2.65 * rlpf(pulse) - 0.145
// with rlpf: fc = clip(max(f/4, 100), 0, sr/2), a = exp(-2*pi*fc/sr),
// y += (1-a)*(x-y) (input-normalized one-pole, exp~-based coefficient).
#ifndef RACK_SRC_LIRA8_LIRA_OSC_H_
#define RACK_SRC_LIRA8_LIRA_OSC_H_

#include <cmath>

#include "pd_primitives.h"
#include "lira8_tc.h"

namespace l8pd {

class LiraOsc {
 public:
  explicit LiraOsc(float sr) : sr_(sr) {}

  // pwm: PWM control 0..1 (0.53125 + 0.025*V from the voice); freq: Hz.
  // Returns square via out_sq, triangle via out_tri.
  LIRA8_TC(lira_osc_process)
  inline void process(float freq, float pwm, float& out_sq, float& out_tri) {
    const float f = std::fabs(freq);
    const float dt = f / sr_;
    if (dt < 1e-9f) {
      // pd's phasor freezes at f=0; skip the blep divisions (0/0). The
      // patch never reaches this (tune line~ ramps > 0) but never NaN.
      const float p = phase_;
      float p2 = p + sh_;
      p2 -= static_cast<float>(static_cast<int>(p2));
      const float pulse =
          (2.0f * p - 1.0f) - (2.0f * p2 - 1.0f) + 2.0f * sh_ - 1.0f;
      out_sq = 0.59f * pulse;
      out_tri = 2.65f * lpf_y_ - 0.145f;
      return;
    }

    // phasor~: output-then-increment, wrap [0,1) — DOUBLE phase (pd's
    // d_osc.c uses double dphase; float accumulation = phase jitter =
    // broadband HF sidebands, measured 12 dB above pd's floor)
    const float p = static_cast<float>(phase_);
    phase_ += static_cast<double>(dt);
    phase_ -= std::floor(phase_);                            // wrap [0,1)
    if (phase_ < 0.0 || phase_ >= 1.0) phase_ = 0.0;         // numeric guard

    // samphold~: samples the PWM value when the phase control DECREASES,
    // i.e. at the wrap.
    if (p < last_phase_for_sh_) sh_ = pwm < 0.0f ? 0.0f : (pwm > 1.0f ? 1.0f : pwm);
    last_phase_for_sh_ = p;
    const float pw = sh_;

    // patch polyblep = -standard_blep: (t-1)^2*[p<=dt] - (t+1)^2*[p>=1-dt]
    float blep1 = 0.0f;
    if (p <= dt) {
      const float t = p / dt;
      blep1 = (t - 1.0f) * (t - 1.0f);
    } else if (p >= 1.0f - dt) {
      const float t = (p - 1.0f) / dt + 1.0f;
      blep1 = -(t + 1.0f) * (t + 1.0f);
    }
    float p2 = p + pw;
    p2 -= static_cast<float>(static_cast<int>(p2));
    float blep2 = 0.0f;
    if (p2 <= dt) {
      const float t = p2 / dt;
      blep2 = (t - 1.0f) * (t - 1.0f);
    } else if (p2 >= 1.0f - dt) {
      const float t = (p2 - 1.0f) / dt + 1.0f;
      blep2 = -(t + 1.0f) * (t + 1.0f);
    }

    const float A = (2.0f * p - 1.0f) + blep1;
    const float B = (2.0f * p2 - 1.0f) + blep2;
    const float pulse = A - B + 2.0f * pw - 1.0f;

    // rlpf: fc = clip(max(f/4, 100), 0, sr/2); a = exp(-2*pi*fc/sr)
    float fc = f * 0.25f;
    if (fc < 100.0f) fc = 100.0f;
    const float half = sr_ * 0.5f;
    if (fc > half) fc = half;
    const float a = std::exp(-2.0f * kPi * fc / sr_);
    lpf_y_ += (1.0f - a) * (pulse - lpf_y_);

    out_sq = 0.59f * pulse;
    out_tri = 2.65f * lpf_y_ - 0.145f;
  }

 private:
  float sr_;
  double phase_ = 0.0;
  double last_phase_for_sh_ = 0.0;
  float sh_ = 0.0f;
  float lpf_y_ = 0.0f;
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_OSC_H_
