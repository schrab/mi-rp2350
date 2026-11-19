// sc_kit_delay.h — SC 3.14.1 core delay family (GPL-3.0, DelayUGens.cpp).
// One power-of-two ring buffer shared by DelayN/L/C, CombN/L/C and
// AllpassN/L/C. Sources:
//   CalcDelay 3456, DelayUnit_Reset 3462, DelayUnit_AllocDelayLine 3438,
//   CalcFeedback (SC_InlineUnaryOp.h:267), helpers 2180-2680,
//   Ctors 3602 (DelayN), 3823 (CombN), 4063 (AllpassN).
// The ctor sets ZOUT0 = 0 and starts iwrphase=0 / zeroed buffer, then the
// initial "checked" phase outputs zeros until the read phase is non-negative
// (the source switches to unchecked after a full buffer has been written —
// with a fresh zeroed buffer the two produce identical samples, so we keep
// the checked behavior throughout).
//  kind: 0 DelayN, 1 DelayL, 2 DelayC, 3 CombN, 4 CombL, 5 CombC,
//        6 AllpassN, 7 AllpassL, 8 AllpassC.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_DELAY_H_
#define RACK_SRC_DRONECASTER_SC_KIT_DELAY_H_
#include <cmath>
#include <cstdint>
#include <vector>
#include "sc_rgen.h"

namespace sc {

static inline float LinInterp(float x, float a, float b) {
  return a + x * (b - a);
}
static inline float CubicInterp(float x, float y0, float y1, float y2,
                                float y3) {
  const float c0 = y1;
  const float c1 = 0.5f * (y2 - y0);
  const float c2 = y0 - 2.5f * y1 + 2.f * y2 - 0.5f * y3;
  const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
  return ((c3 * x + c2) * x + c1) * x + c0;
}

struct DelayLine {
  void Init(double sr, float maxdelay, float delay, float decay, int kind) {
    sr_ = static_cast<float>(sr);
    kind_ = kind;
    delay_ = delay;
    decay_ = decay;
    long sz = static_cast<long>(std::ceil(maxdelay * sr_ + 1.f)) + 64;
    bufsize_ = 1;
    while (bufsize_ < sz) bufsize_ <<= 1;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    mask_ = bufsize_ - 1;
    iwrphase_ = 0;
    max_dsamp_ = static_cast<float>(bufsize_);
    dsamp_ = CalcDelay(delay);
    feedbk_ = kind_ >= 3 ? CalcFeedback(delay, decay) : 0.f;
  }
  // Buf* variant: the SndBuf supplies a power-of-two frame count; the delay
  // is clipped to PREVIOUSPOWEROFTWO(bufSamples)-1 (BufCalcDelay, line 2144).
  void InitBuf(double sr, long bufsamples, float delay, float decay, int kind) {
    sr_ = static_cast<float>(sr);
    kind_ = kind;
    delay_ = delay;
    decay_ = decay;
    bufsize_ = bufsamples;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    mask_ = bufsize_ - 1;
    iwrphase_ = 0;
    max_dsamp_ = static_cast<float>(bufsize_ - 1);
    dsamp_ = CalcDelay(delay);
    feedbk_ = kind_ >= 3 ? CalcFeedback(delay, decay) : 0.f;
  }
  void ProcessBlock(const float* in, float* out, int n, float delay,
                    float decay) {
    if (delay != delay_ || (kind_ >= 3 && decay != decay_)) {
      const float next_ds = CalcDelay(delay);
      const float next_fb = kind_ >= 3 ? CalcFeedback(delay, decay) : 0.f;
      const float ds_slope = (next_ds - dsamp_) * (1.f / 64.f);
      const float fb_slope = (next_fb - feedbk_) * (1.f / 64.f);
      for (int i = 0; i < n; ++i) {
        dsamp_ += ds_slope;
        feedbk_ += fb_slope;
        out[i] = Step(in[i]);
      }
      delay_ = delay;
      decay_ = decay;
    } else {
      for (int i = 0; i < n; ++i) out[i] = Step(in[i]);
    }
  }
  float ctor_value() const { return 0.f; }

 private:
  float CalcDelay(float delaytime) const {
    const float min_delay = (kind_ == 2 || kind_ == 5 || kind_ == 8) ? 2.f : 1.f;
    const float d = delaytime * sr_;
    return d < min_delay ? min_delay : (d > max_dsamp_ ? max_dsamp_ : d);
  }
  static float CalcFeedback(float delaytime, float decaytime) {
    if (delaytime == 0.f || decaytime == 0.f) return 0.f;
    const double kLog001 = -6.9077552789821368;  // std::log(0.001)
    const float absret = static_cast<float>(
        std::exp(kLog001 * static_cast<double>(delaytime) /
                 std::fabs(static_cast<double>(decaytime))));
    return std::copysign(absret, decaytime);
  }
  float Step(float x) {
    const long idsamp = static_cast<long>(dsamp_);
    const float frac = dsamp_ - static_cast<float>(idsamp);
    const long ird = static_cast<long>(iwrphase_) - idsamp;
    const long w = iwrphase_ & mask_;
    const long r1 = ird & mask_, r0 = (ird + 1) & mask_;
    const long rm1 = (ird - 1) & mask_, rm2 = (ird - 2) & mask_;
    float value = 0.f;

    if (kind_ == 0 || kind_ == 3 || kind_ == 6) {  // nearest-neighbour
      if (ird < 0) {
        // Every checked helper writes the raw input during this phase.
        if (kind_ == 6) {
          buf_[w] = x;
          value = -feedbk_ * x;
        } else {
          buf_[w] = x;
        }
      } else {
        value = buf_[r1];
        if (kind_ == 0) buf_[w] = x;
        else if (kind_ == 3) buf_[w] = x + feedbk_ * value;
        else {  // AllpassN
          const float dwr = feedbk_ * value + x;
          buf_[w] = dwr;
          value = value - feedbk_ * dwr;
        }
      }
    } else if (kind_ == 1 || kind_ == 4 || kind_ == 7) {  // linear
      if (ird < 0) {
        value = 0.f;
      } else if (ird - 1 < 0) {
        const float d1 = buf_[r1];
        value = d1 - frac * d1;
      } else {
        value = LinInterp(frac, buf_[r1], buf_[rm1]);
      }
      if (kind_ == 1) buf_[w] = x;
      else if (kind_ == 4) buf_[w] = x + feedbk_ * value;
      else {
        const float dwr = x + feedbk_ * value;
        buf_[w] = dwr;
        value = value - feedbk_ * dwr;
      }
    } else {  // cubic (2 DelayC, 5 CombC, 8 AllpassC)
      if (ird + 1 < 0) {
        buf_[w] = x;  // source still stores the input, output stays 0
      } else {
        float d0 = 0.f, d1 = 0.f, d2 = 0.f, d3 = 0.f;
        if (ird >= 0) d1 = buf_[r1];
        if (ird - 1 >= 0) d2 = buf_[rm1];
        if (ird - 2 >= 0) d3 = buf_[rm2];
        d0 = buf_[r0];
        value = CubicInterp(frac, d0, d1, d2, d3);
        if (kind_ == 2) buf_[w] = x;
        else if (kind_ == 5) buf_[w] = x + feedbk_ * value;
        else {
          const float dwr = x + feedbk_ * value;
          buf_[w] = dwr;
          value = value - feedbk_ * dwr;
        }
      }
    }
    ++iwrphase_;
    return value;
  }
  std::vector<float> buf_;
  long bufsize_ = 0, mask_ = 0, iwrphase_ = 0;
  float sr_ = 48000.f, dsamp_ = 0.f, feedbk_ = 0.f, max_dsamp_ = 0.f;
  float delay_ = 0.f, decay_ = 0.f;
  int kind_ = 0;
};

// Pluck — DelayUGens.cpp:5577 (ctor), 5851/5947 (kk / kk_z perform), 5610
// (aa). Karplus-Strong: a trigger opens the excitation input for
// round(delaytime*sr) samples; a cubic-read delay line is fed through a
// one-pole (coef) and feedback (decaytime). Ctor: no priming sample, buffer
// zeroed, lastsamp=0, prevtrig=0, inputsamps=0, coef=IN0(5).
struct Pluck {
  void Init(double sr, float maxdelay, float delay, float decay, float coef) {
    sr_ = static_cast<float>(sr);
    delay_ = delay;
    decay_ = decay;
    coef_ = coef;
    long sz = static_cast<long>(std::ceil(maxdelay * sr_ + 1.f)) + 64;
    bufsize_ = 1;
    while (bufsize_ < sz) bufsize_ <<= 1;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    mask_ = bufsize_ - 1;
    iwrphase_ = 0;
    dsamp_ = CalcDelay(delay);
    feedbk_ = CalcFeedback(delay, decay);
    lastsamp_ = 0.f;
    prevtrig_ = 0.f;
    inputsamps_ = 0;
    numoutput_ = 0;
  }
  void ProcessBlock(const float* in, const float* trig, float* out, int n,
                    float delay, float decay, float coef) {
    for (int i = 0; i < n; ++i) {
      const float curtrig = trig[i];  // a-rate edge detection (aa path)
      if (prevtrig_ <= 0.f && curtrig > 0.f)
        inputsamps_ = static_cast<long>(delay * sr_ + .5f);
      prevtrig_ = curtrig;
      const long idsamp = static_cast<long>(dsamp_);
      const float frac = dsamp_ - static_cast<float>(idsamp);
      const long ird1 = static_cast<long>(iwrphase_) - idsamp;
      const long ird0 = ird1 + 1, ird2 = ird1 - 1, ird3 = ird1 - 2;
      const float thisin = inputsamps_ > 0 ? (--inputsamps_, in[i]) : 0.f;
      float value = 0.f;
      if (ird0 < 0) {
        buf_[iwrphase_ & mask_] = thisin;
      } else {
        float d0 = 0.f, d1 = 0.f, d2 = 0.f, d3 = 0.f;
        if (ird1 < 0) {
          d0 = buf_[ird0 & mask_];
        } else if (ird2 < 0) {
          d0 = buf_[ird0 & mask_]; d1 = buf_[ird1 & mask_];
        } else if (ird3 < 0) {
          d0 = buf_[ird0 & mask_]; d1 = buf_[ird1 & mask_];
          d2 = buf_[ird2 & mask_];
        } else {
          d0 = buf_[ird0 & mask_]; d1 = buf_[ird1 & mask_];
          d2 = buf_[ird2 & mask_]; d3 = buf_[ird3 & mask_];
        }
        value = CubicInterp(frac, d0, d1, d2, d3);
        const float onepole = (1.f - std::fabs(coef)) * value + coef * lastsamp_;
        buf_[iwrphase_ & mask_] = thisin + feedbk_ * onepole;
        lastsamp_ = onepole;
      }
      out[i] = lastsamp_;
      ++iwrphase_;
    }
    delay_ = delay; decay_ = decay; coef_ = coef;
  }

 private:
  float CalcDelay(float d) const {
    const float v = d * sr_;
    return v < 2.f ? 2.f : (v > static_cast<float>(bufsize_)
                                ? static_cast<float>(bufsize_) : v);
  }
  static float CalcFeedback(float d, float dec) {
    if (d == 0.f || dec == 0.f) return 0.f;
    const double kLog001 = -6.9077552789821368;
    const float a = static_cast<float>(
        std::exp(kLog001 * static_cast<double>(d) / std::fabs(dec)));
    return std::copysign(a, dec);
  }
  std::vector<float> buf_;
  long bufsize_ = 0, mask_ = 0, iwrphase_ = 0, inputsamps_ = 0, numoutput_ = 0;
  float sr_ = 48000.f, dsamp_ = 0.f, feedbk_ = 0.f, lastsamp_ = 0.f;
  float prevtrig_ = 0.f, delay_ = 0.f, decay_ = 0.f, coef_ = 0.f;
};

// PitchShift — DelayUGens.cpp:4967 (ctor), 4583/4731 (next / next_z). Four
// crossfading read heads share one ring buffer; at every framesize/4 samples
// the active head is re-seeded from pchratio/pchdisp/timedisp using the
// voice's shared RGen (RandSeed pins scsynth's RGen in the NRT reference).
struct PitchShift {
  void Init(double sr, float winsize, float ratio, float pchdisp,
            float timedisp, float first_in) {
    sr_ = static_cast<float>(sr);
    winsize_ = winsize;
    ratio_ = ratio;
    pchdisp_ = pchdisp;
    timedisp_ = timedisp;
    if (winsize_ < 3.f / sr_) winsize_ = 3.f / sr_;
    long dbs = static_cast<long>(std::ceil(winsize_ * sr_ * 3.f + 3.f));
    dbs += 64;
    bufsize_ = 1;
    while (bufsize_ < dbs) bufsize_ <<= 1;
    buf_.assign(static_cast<size_t>(bufsize_), 0.f);
    buf_[0] = first_in;  // ctor: *dlybuf = ZIN0(0)
    mask_ = bufsize_ - 1;
    buf_[mask_] = buf_[mask_ - 1] = buf_[mask_ - 2] = 0.f;
    framesize_ = (static_cast<long>(winsize_ * sr_) + 2) & ~3L;
    slope_ = 2.f / static_cast<float>(framesize_);
    stage_ = 3;
    counter_ = framesize_ >> 2;
    ramp_[0] = 0.5f; ramp_[1] = 1.f; ramp_[2] = 0.5f; ramp_[3] = 0.f;
    rs_[0] = -slope_; rs_[1] = -slope_; rs_[2] = slope_; rs_[3] = slope_;
    for (int i = 0; i < 4; ++i) { ds_[i] = 2.f; dss_[i] = 1.f; }
    iwrphase_ = 0;
    numoutput_ = 0;
  }
  void ProcessBlock(const float* in, float* out, int n, RGen* rng) {
    float timedisp = timedisp_ < 0.f ? 0.f
                     : (timedisp_ > winsize_ ? winsize_ : timedisp_);
    timedisp *= sr_;
    int remain = n;
    int written = 0;  // ZXP(in)/ZXP(out) advance across the whole block
    while (remain > 0) {
      if (counter_ <= 0) {
        counter_ = framesize_ >> 2;
        stage_ = (stage_ + 1) & 3;
        float ratio = ratio_;
        if (pchdisp_ != 0.f) ratio += pchdisp_ * rng->frand2();
        ratio = ratio < 0.f ? 0.f : (ratio > 4.f ? 4.f : ratio);
        float ratio1 = ratio - 1.f;
        const float samp_slope = -ratio1;
        float startpos = ratio1 < 0.f ? 2.f
            : static_cast<float>(framesize_) * ratio1 + 2.f;
        startpos += timedisp * rng->frand();
        const int st = stage_;
        ds_[st] = startpos;
        dss_[st] = samp_slope;
        ramp_[st] = 0.f;
        rs_[st] = slope_;
        rs_[(st + 2) & 3] = -slope_;
      }
      const int nsmps = remain < counter_ ? remain : counter_;
      remain -= nsmps;
      counter_ -= nsmps;
      for (int k = 0; k < nsmps; ++k) {
        ++numoutput_;
        iwrphase_ = (iwrphase_ + 1) & mask_;
        float value = 0.f;
        for (int tap = 0; tap < 4; ++tap) {
          ds_[tap] += dss_[tap];
          const long idsamp = static_cast<long>(ds_[tap]);
          const float frac = ds_[tap] - static_cast<float>(idsamp);
          const long rd = (iwrphase_ - idsamp) & mask_;
          const long rdb = (rd - 1) & mask_;
          float v;
          if (numoutput_ < bufsize_) {
            if (rd > iwrphase_) v = 0.f;
            else if (rdb > iwrphase_) {
              const float d1 = buf_[rd];
              v = d1 - frac * d1;
            } else {
              const float d1 = buf_[rd], d2 = buf_[rdb];
              v = d1 + frac * (d2 - d1);
            }
          } else {
            const float d1 = buf_[rd], d2 = buf_[rdb];
            v = d1 + frac * (d2 - d1);
          }
          value += v * ramp_[tap];
          ramp_[tap] += rs_[tap];
        }
        buf_[iwrphase_] = in[written + k];
        out[written + k] = value * 0.5f;
      }
      written += nsmps;
    }
  }

 private:
  std::vector<float> buf_;
  long bufsize_ = 0, mask_ = 0, iwrphase_ = 0;
  long framesize_ = 0, counter_ = 0, stage_ = 0, numoutput_ = 0;
  float sr_ = 48000.f, winsize_ = 0.2f, ratio_ = 1.f;
  float pchdisp_ = 0.f, timedisp_ = 0.f, slope_ = 0.f;
  float ds_[4] = {}, dss_[4] = {}, ramp_[4] = {}, rs_[4] = {};
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_DELAY_H_
