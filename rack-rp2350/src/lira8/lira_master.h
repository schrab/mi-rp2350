// master_output verbatim port (PORTING-SPEC.md §3, take-1 verified).
// drive gain = 10^((3^(1+2k/127) + 3)/20) (2.0 .. 31.6, default k=86 -> 6.5);
// dist = hip10( tanh(hip20(in*g)) + pow31(tanh(hip20(in*g)))/4 ) / clip(g,1,4);
// wet = 0.1*in + dist; out_pre = tanh(wet*mix + in*(1-mix)); out = out_pre*vol;
// total-feedback tap = hip~3 of out_pre (pre-volume).
#ifndef RACK_SRC_LIRA8_LIRA_MASTER_H_
#define RACK_SRC_LIRA8_LIRA_MASTER_H_

#include "pd_primitives.h"
#include "lira8_tc.h"

namespace l8pd {

class MasterOut {
 public:
  explicit MasterOut(float sr)
      : drive_(sr), dst_mix_(sr), vol_(sr),
        hip20_(20.0f, sr), hip10_(10.0f, sr), hip3_(3.0f, sr),
        dist_lp_c_(1.0f - std::exp(-2.0f * kPi * 7000.0f / sr)) {}

  void set_drv(int k) {
    const float v = 1.0f + 2.0f * (k / 127.0f);
    drive_.set(std::pow(10.0f, (std::pow(3.0f, v) + 3.0f) / 20.0f), 23.22f);
    drive_raw_ = std::pow(10.0f, (std::pow(3.0f, v) + 3.0f) / 20.0f);
  }
  void set_dst_mix(int k) { dst_mix_.set(k / 127.0f, 23.22f); }
  void set_vol(int k) { vol_.set((k / 127.0f) * (k / 127.0f), 23.22f); }

  // Per-sample; feedback_tap receives the hip~3'd pre-volume tanh output.
  LIRA8_TC(lira_master_process)
  inline void process(float in, float& out, float& feedback_tap) {
    const float g = drive_.next();
    const float t_in = hip20_.process(in * g);
    const float t = tanh_pade(t_in);
    // pow~ 31 as repeated squaring (t^(16+8+4+2+1)) — same value within a
    // float ulp or two, minus one libm powf per sample.
    const float t2 = t * t;
    const float t4 = t2 * t2;
    const float t8 = t4 * t4;
    const float t16 = t8 * t8;
    const float shaped = t + (t16 * t8 * t4 * t2 * t) * 0.25f;  // /~ 4
    const float shaped10 = hip10_.process(shaped);
    // The manual: the hardware's "highs sound softer" (its distortion stage
    // is band-limited). One-pole at 7 kHz on the DISTORTION branch only —
    // tames the pow~31 drive's aliasing/grit without darkening the clean
    // path. (The Pd patch has no such filter; this is an HW-voicing choice.)
    dist_lp_y_ += dist_lp_c_ * (shaped10 - dist_lp_y_);
    float gc = g;  // clip(g, 1, 4)
    if (gc > 4.0f) gc = 4.0f;
    if (gc < 1.0f) gc = 1.0f;
    const float dist = dist_lp_y_ / gc;
    const float mix = dst_mix_.next();
    const float wet = in * 0.1f + dist;
    const float out_pre = tanh_pade(wet * mix + in * (1.0f - mix));
    feedback_tap = hip3_.process(out_pre);
    out = out_pre * vol_.next();
  }

 private:
  // ma.tanh~: input clipped to +-3 BEFORE the Pade formula (the patch's
  // clip~ -3 3). Without the clip the Pade ratio grows ~x/9 and pow~31 of
  // the unclipped value explodes the chain (the drv=127 fuzz bug).
  static float tanh_pade(float x) {
    if (x > 3.0f) x = 3.0f;
    else if (x < -3.0f) x = -3.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
  }
  Ramp drive_, dst_mix_, vol_;
  float dist_lp_c_;
  float dist_lp_y_ = 0.0f;
  float drive_raw_ = 1.f;
  Hip hip20_, hip10_, hip3_;
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_MASTER_H_
