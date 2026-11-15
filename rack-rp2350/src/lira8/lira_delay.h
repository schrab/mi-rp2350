// LIRA-8 delay section port — ModDelay (pd delwrite~/vd~ verbatim, int16
// storage per the RP2350 decision), Compander (compressor~ + expander~
// verbatim), DelaySection (parallel buses with shared write input, per-bus
// feedback, wet monitor). Sources: patch/_LIRA-8.pd delay subpatch (resolved
// dump delay_master.txt), take-1 math, PORTING-SPEC.md (with corrections
// box), PD-PRIMITIVES.md.
#ifndef RACK_SRC_LIRA8_LIRA_DELAY_H_
#define RACK_SRC_LIRA8_LIRA_DELAY_H_

#include <cstdint>
#include <cmath>
#include <cstring>

#include "pd_primitives.h"
#include "lira8_tc.h"

namespace l8pd {

// ---------- delwrite~/vd~ verbatim (int16 storage) ----------
// Buffer = nsamps + 4 guard (pd XTRASAMPS); phase starts at 4; wrap copies
// the last 4 samples to the head. Reads: pd's 4-point cubic, delay in ms,
// clamped [1.00001, nsamps − block], zerodel = one block (measured).
class ModDelay {
 public:
  // cap_ms = storage cap (1.0 s on RP2350 = the original hardware's max;
  //          5.944 s on ESP32/PSRAM).
  // mem = EXTERNAL storage (the rack buffer arena / PSRAM on ESP32) —
  // nsamps_+4 int16 samples, caller-owned, zeroed by the caller. No heap.
  ModDelay(float sr, float cap_ms, int block, int16_t* mem)
      : sr_(sr),
        nsamps_(static_cast<int>(cap_ms * sr / 1000.0f)),
        block_(block),
        buf_(mem) {
    memset(buf_, 0, (nsamps_ + 4) * sizeof(int16_t));
    phase_ = 4;
  }

  static int bytes_for(float sr, float cap_ms) {
    return static_cast<int>(cap_ms * sr / 1000.0f + 4) *
           static_cast<int>(sizeof(int16_t));
  }

  // Reads use the write position BEFORE this block's write (zerodel=block).
  // delay_ms_signal: per-sample delay time in ms; n = block size.
  LIRA8_TC(lira_delay_read)
  void read_block(const float* delay_ms_signal, int n, float* out) {
    const float limit = static_cast<float>(nsamps_ - n);
    const float sr_ms = sr_ / 1000.0f;
    float fn = static_cast<float>(n - 1);
    for (int i = 0; i < n; ++i) {
      float delsamps = sr_ms * delay_ms_signal[i] -
                       static_cast<float>(block_);  // zerodel = 1 block
      if (!(delsamps >= 1.00001f)) delsamps = 1.00001f;
      if (delsamps > limit) delsamps = limit;
      delsamps += fn;
      fn -= 1.0f;
      int idelsamps = static_cast<int>(delsamps);
      float frac = delsamps - static_cast<float>(idelsamps);
      // bp = write_head − idelsamps (relative to data region [4, 4+nsamps))
      int idx = phase_ - idelsamps;
      if (idx < 4) idx += nsamps_;
      // pd sigvd_perform: bp = wp - idelsamps; d = bp[-3], c = bp[-2],
      // b = bp[-1], a = bp[0]. The v1 port started at bp[-1] for b (window
      // shifted +1): reads landed one sample late (D-1).
      const float a = s2f(buf_[idx]);
      const float b = s2f(buf_[idx - 1]);
      const float c = s2f(buf_[idx - 2]);
      const float d = s2f(buf_[idx - 3]);
      const float cminusb = c - b;
      out[i] = b + frac * (cminusb - 0.1666667f * (1.0f - frac) *
                                               ((d - a - 3.0f * cminusb) * frac +
                                                (d + 2.0f * a - 3.0f * b)));
    }
  }

// pd SELF tap = a SECOND vd~ whose time inlet is UNCONNECTED (time 0 ->
    // minimum delay); its output is constant across the whole block. This
    // port reads before writing, so the most recent available sample is the
    // previous block's last write (one block of absolute lag on a
    // modulation source only — the audio path keeps its own timing).
    LIRA8_TC(lira_delay_min_tap)
    float read_min_tap() const { return s2f(buf_[phase_ - 1]); }

    LIRA8_TC(lira_delay_write)
    void write_block(const float* in, int n) {
    for (int i = 0; i < n; ++i) {
      buf_[phase_++] = f2s(in[i]);
      if (phase_ == nsamps_ + 4) {  // wrap: copy last 4 to head (pd verbatim)
        buf_[0] = buf_[nsamps_];
        buf_[1] = buf_[nsamps_ + 1];
        buf_[2] = buf_[nsamps_ + 2];
        buf_[3] = buf_[nsamps_ + 3];
        phase_ = 4;
      }
    }
  }

 private:
  // int16 storage with 2x headroom: pd's float delay exceeds +-1 transiently
  // in the feedback loop (hard-clipping there = bitcrush distortion), but the
  // in-loop compander self-limits the content, so +-2 rarely clips while the
  // quantization floor (-79 dB) stays within ~5 dB of pd's own noise floor
  // (noise~ * 0.001 in the patch). Measured floors: pd -83 dB, +-1 clip,
  // +-4 scale -73 dB (too hissy), +-2 = the sweet spot.
  static constexpr float kDelayScale = 0.5f;
  static constexpr float kS2f = 1.0f / (32767.0f * kDelayScale);
  static inline float s2f(int16_t s) {
    return static_cast<float>(s) * kS2f;      // reciprocal (was a divide)
  }
  static inline int16_t f2s(float f) {
    if (f > 2.0f) f = 2.0f;
    else if (f < -2.0f) f = -2.0f;
    return static_cast<int16_t>(f * 32767.0f * kDelayScale);
  }
  float sr_;
  int nsamps_;
  int block_;
  int phase_;
  int16_t* buf_;
};

// ---------- compressor~ (attack/release-switched env follower) ----------
// coeff = exp(-4.60517 / (ms * sr/1000)) (99% settle in ms);
// env = temp*(1-c) + c*env, temp = |in|, c = attack if temp>env else release;
// gain = ma.tanh(clip((thresh + (env-thresh)/ratio) / env, 0, 1));
// out = in * gain. Below threshold gain = tanh(1) = 0.7616 (patch quirk).
class Compander {
 public:
  Compander(float sr) : sr_(sr) {}
  void set(float attack_ms, float release_ms, float thresh_db, float ratio) {
    c_attack_ = coeff(attack_ms);
    c_release_ = coeff(release_ms);
    thresh_ = dbtorms(thresh_db + 100.0f);
    inv_ratio_ = 1.0f / ratio;
  }
  LIRA8_TC(lira_comp_process)
  float process(float in) {
    const float temp = std::fabs(in);
    const float c = (temp > env_) ? c_attack_ : c_release_;
    env_ = temp * (1.0f - c) + c * env_;
    float g = (thresh_ + (env_ - thresh_) * inv_ratio_) / (env_ + 1e-30f);
    if (g > 1.0f) g = 1.0f;
    if (g < 0.0f) g = 0.0f;
    return in * tanh_pade(g);
  }
  float env() const { return env_; }

 private:
  float coeff(float ms) const {
    float samples = ms * sr_ / 1000.0f;
    if (samples < 1.0f) samples = 1.0f;
    return std::exp(-4.60517f / samples);
  }
  static float tanh_pade(float x) {
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
  }
  float sr_;
  float c_attack_ = 0.f, c_release_ = 0.f, thresh_ = 0.f, inv_ratio_ = 1.f;
  float env_ = 0.f;
};

// ---------- expander~ (env~ 512 RMS in dB domain) ----------
// gain_dB = min(0, (env_dB - thresh_dB) * (1 - 1/ratio)); on each env~ fire
// the target goes through msg "$1 11" -> line~ (11 ms) -> +100 -> dbtorms~.
// pd ramps the dB value, so the gain follows an EXPONENTIAL curve; a
// constant per-sample gain ratio is exactly that curve. v1 ramped the
// LINEAR gain and started from 0 instead of 0 dB = unity.
class Expander {
 public:
  Expander(float sr) : sr_(sr), env_(512, 64, sr) {}
  void set(float thresh_db, float ratio) {
    thresh_ = thresh_db;
    slope_ = 1.0f - 1.0f / ratio;
  }
  // Per 64-sample block: feed the env, apply the smoothed gain per sample.
  LIRA8_TC(lira_exp_process_block)
  void process_block(const float* in, int n, float* out) {
    float db = 0.f;
    if (env_.process_block(in, n, &db)) {   // env~ fires every 256 samples
      const float gdb = std::fmin(0.0f, (db - 100.0f - thresh_) * slope_);
      const float target = dbtorms(gdb + 100.0f);
      int ns = static_cast<int>(11.0f * sr_ * 0.001f);   // line~ 11 ms
      if (ns < 1) ns = 1;
      target_ = target;
      if (target <= 0.0f || gain_ <= 0.0f) {
        gain_ = target;
        ratio_ = 1.0f;
        step_left_ = 0;
      } else {
        ratio_ = std::pow(target / gain_, 1.0f / static_cast<float>(ns));
        step_left_ = ns;
      }
    }
    for (int i = 0; i < n; ++i) {
      out[i] = in[i] * gain_;
      if (step_left_ > 0) {
        gain_ *= ratio_;
        if (--step_left_ == 0) gain_ = target_;
      }
    }
  }

 private:
  float sr_;
  EnvRms env_;
  float thresh_ = -20.0f, slope_ = 0.8f;
  float gain_ = 1.0f;      // pd line~ starts at 0 dB = unity
  float ratio_ = 1.0f;
  float target_ = 1.0f;
  int step_left_ = 0;
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_DELAY_H_
