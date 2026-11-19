// sc_kit_osc2.h — SC 3.14.1 oscillator family: Saw (BLIT), SinOscFB, Select,
// Klank, and the sc3-plugins DPW oscillators (SawDPW / PulseDPW composite).
//
// Anchors (Version-3.14.1):
//   CORE OscUGens.cpp: Saw_Ctor 2366, Saw_next 2382 (Pulse's BLIT structure
//   with ONE lookup, y1 PRIMED to -0.46, "(numer*denom - 1)*scale" integrator,
//   ctor primes + rolls back); SinOscFB_next_kk 1364/_Ctor (feedback phase
//   modulation + CALCSLOPE feedback ramp); Select_next_1/k (index+1 clipped);
//   Klank_SetCoefs (R = exp(log(0.001)/(time*sr)), b1 = 4R^2 cos(w)/(1+R^2),
//   b2 = -R^2, a0 = level*0.25, 4-wide interleaved resonator bank, output =
//   sum(a0*y0)); Klank_next.
//   sc3-plugins (GPL-3.0, MCLDUGens/MCLDOscUGens.cpp: SawDPW_next_a/_k):
//   parabolic DPW saw; PulseDPW is the classlib composite
//   SawDPW(freq,0) - SawDPW(freq,(2*width).wrap(-1,1)).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_OSC2_H_
#define RACK_SRC_DRONECASTER_SC_KIT_OSC2_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"
#include "sc_kit_osc.h"
#include "sc_kit_noise.h"    // sc_wrap

namespace sc {

// Saw.ar/.kr (OscUGens.cpp:2366) — the BLIT saw: one sine/cosecant lookup, a
// leaky integrator, y1 primed to -0.46, and the frequency-change crossfade
// exactly like Pulse (N = int(sr*0.5/freq), scale = 0.5/N).
class Saw {
 public:
  void Init(const KitTables* t, double sr, float freq) {
    t_ = t;
    sr_ = sr;
    freq_ = freq;
    freqin_ = freq;
    cpstoinc_ = static_cast<double>(kSineSize) * (1.0 / sr) * 65536.0 * 0.5;
    n_ = static_cast<int32_t>((sr * 0.5) / static_cast<double>(freq));
    scale_ = static_cast<float>(0.5 / static_cast<double>(n_));
    phase_ = 0;
    y1_ = -0.46f;
    ProcessBlock(nullptr, 0);                  // ctor primes (no output)
    scale_ = static_cast<float>(0.5 / static_cast<double>(n_));
    phase_ = 0;
    y1_ = -0.46f;
  }
  void SetFreq(float f) { freq_ = f; }
  // One half of the Saw_next arithmetic (the non-crossfade path and the two
  // crossfade halves share this law).
  float PulAt(uint32_t phase, int32_t N2, float scale, bool crossfade,
              int32_t prevN2, float prevscale, float xfade) const {
    const float* tbl = t_->cosecant + ((phase >> 16) & (kSineSize - 1));
    float t0 = tbl[0];
    float t1 = tbl[1];
    const float pfrac = PhaseFrac(phase);
    if (t0 == kBadValue || t1 == kBadValue) {
      tbl = t_->sine + ((phase >> 16) & (kSineSize - 1));
      t0 = tbl[0];
      t1 = tbl[1];
      const float denom = t0 + (t1 - t0) * pfrac;
      if (fabsf(denom) < 0.0005f) return 1.0f;   // caller adds 0.999*y1
      const uint32_t rphase = phase * static_cast<uint32_t>(N2);
      const float numer = SineAt(rphase);
      if (!crossfade) return (numer / denom - 1.0f) * scale;
      const uint32_t rphase_prev = phase * static_cast<uint32_t>(prevN2);
      const float n1 = (SineAt(rphase_prev) / denom - 1.0f) * prevscale;
      const float n2 = (numer / denom - 1.0f) * scale;
      return n1 + xfade * (n2 - n1);
    }
    const float denom = t0 + (t1 - t0) * pfrac;
    const uint32_t rphase = phase * static_cast<uint32_t>(N2);
    const float numer = SineAt(rphase);
    if (!crossfade) return (numer * denom - 1.0f) * scale;
    const uint32_t rphase_prev = phase * static_cast<uint32_t>(prevN2);
    const float n1 = (SineAt(rphase_prev) * denom - 1.0f) * prevscale;
    const float n2 = (numer * denom - 1.0f) * scale;
    return n1 + xfade * (n2 - n1);
  }
  float SineAt(uint32_t rphase) const {
    const float* tbl = t_->sine + ((rphase >> 16) & (kSineSize - 1));
    return lininterp(PhaseFrac(rphase), tbl[0], tbl[1]);
  }
  void ProcessBlock(float* out, int n);

 private:
  const KitTables* t_ = nullptr;
  double sr_ = 48000.0;
  double cpstoinc_ = 0.0;
  uint32_t phase_ = 0;
  int32_t n_ = 0;
  float freqin_ = 0.0f, freq_ = 0.0f;
  float scale_ = 0.0f, y1_ = -0.46f;
};

inline void Saw::ProcessBlock(float* out, int n) {
  const float freqin = freq_;
  uint32_t phase = phase_;
  float y1 = y1_;

  int32_t freq, N, prevN = 0;
  float scale, prevscale = 0.0f;
  bool crossfade;
  if (freqin != freqin_) {
    N = static_cast<int32_t>((sr_ * 0.5) / static_cast<double>(freqin));
    if (N != n_) {
      const float maxfreqin = freqin > freqin_ ? freqin : freqin_;
      freq = static_cast<int32_t>(cpstoinc_ * static_cast<double>(maxfreqin));
      crossfade = true;
    } else {
      freq = static_cast<int32_t>(cpstoinc_ * static_cast<double>(freqin));
      crossfade = false;
    }
    prevN = n_;
    prevscale = scale_;
    n_ = N;
    scale_ = scale = static_cast<float>(0.5 / static_cast<double>(N));
  } else {
    N = n_;
    freq = static_cast<int32_t>(cpstoinc_ * static_cast<double>(freqin));
    scale = scale_;
    crossfade = false;
  }
  freqin_ = freqin;
  const int32_t N2 = 2 * N + 1;

  if (crossfade) {
    const int32_t prevN2 = 2 * prevN + 1;
    float xfade = 0.0f;
    for (int i = 0; i < n; ++i) {
      const float p = PulAt(phase, N2, scale, true, prevN2, prevscale, xfade);
      // The bad-denom branch outputs a raw 1.0 (no scale multiply).
      if (p == 1.0f) out[i] = y1 = 1.0f + 0.999f * y1;
      else out[i] = y1 = p + 0.999f * y1;
      phase += static_cast<uint32_t>(freq);
      xfade += 1.0f / 64.0f;
    }
  } else {
    for (int i = 0; i < n; ++i) {
      const float p = PulAt(phase, N2, scale, false, 0, 0.0f, 0.0f);
      if (p == 1.0f) out[i] = y1 = 1.0f + 0.999f * y1;
      else out[i] = y1 = p + 0.999f * y1;
      phase += static_cast<uint32_t>(freq);
    }
  }

  phase_ = phase;
  y1_ = y1;
}

// SinOscFB.ar — phase-modulated feedback sine (OscUGens.cpp:1364). The
// feedback input is scaled by radtoinc and ramped across the block
// (CALCSLOPE). Ctor: phase 0, prevout 0, feedback = in*radtoinc, prime +
// rollback of the phase.
struct SinOscFB {
  void Init(const KitTables* t, double sr, float freq, float feedback) {
    t_ = t;
    lomask_ = (kSineSize - 1) << 3;
    // m_radtoinc = tableSize2 * (rtwopi * 65536), rtwopi = 1/(2*pi)
    radtoinc_ = static_cast<float>(kSineSize * (1.0 / (2.0 * M_PI)) * 65536.0);
    cpstoinc_ = static_cast<float>(static_cast<double>(kSineSize) * (1.0 / sr) * 65536.0);
    phase_ = 0;
    prevout_ = 0.0f;
    fb_cur_ = 0.0f;
    fb_next_ = feedback * radtoinc_;
    freq_ = freq;
    Next();
    phase_ = 0;                                // ctor rollback
    prevout_ = 0.0f;
    fb_cur_ = feedback * radtoinc_;
  }
  void SetFreq(float f) { freq_ = f; }
  void SetFeedback(float fb) { fb_next_ = fb * radtoinc_; }
  float Next() {
    prevout_ = lookupi1(t_->sine_wt, t_->sine_wt + 1,
                        phase_ + static_cast<uint32_t>(
                                     static_cast<int32_t>(fb_cur_ * prevout_)),
                        lomask_);
    phase_ += static_cast<uint32_t>(static_cast<int32_t>(cpstoinc_ * freq_));
    fb_cur_ += (fb_next_ - fb_cur_) * (1.0f / 64.0f);
    return prevout_;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  const KitTables* t_ = nullptr;
  int32_t lomask_ = 0;
  float radtoinc_ = 0.0f, cpstoinc_ = 0.0f;
  float freq_ = 0.0f, prevout_ = 0.0f, fb_cur_ = 0.0f, fb_next_ = 0.0f;
  uint32_t phase_ = 0;
};

// SawDPW (sc3-plugins MCLDUscUGens.cpp) — parabolic DPW saw; k-law
// scalefac = 1/(4*freq*(1-freq/sr)).
struct SawDPW {
  void Init(double sr, float freq, double iphase = 0.0) {
    sr_ = sr;
    freqmul_ = static_cast<float>(2.0 * (1.0 / sr));
    val_ = 0.0f;
    freq_ = 0.0f;
    scalefac_ = 0.0f;
    freq_in_ = freq;
    phase_ = iphase;
    // The ctor primes with the A-RATE law (scalefac = sr/(8 f (1-f/sr)))
    // and does NOT roll the phase back; mFreq/mScalefac keep the A values.
    phase_ += freq * freqmul_;
    if (phase_ >= 1.0) phase_ -= 2.0;
    if (phase_ < -1.0) phase_ += 2.0;
    val_ = static_cast<float>(phase_ * phase_);
    freq_ = freq;                              // a-law stores the raw Hz
    scalefac_ = static_cast<float>(sr / (8.0 * freq * (1.0 - freq / sr)));
    primed_ = 0.0f;                            // primed output unused
  }
  void SetFreq(float f) { freq_in_ = f; }
  float Next() { return ProcessBlockOne(freq_in_); }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = ProcessBlockOne(freq_in_);
  }
  float ctor_value() const { return primed_; }
 private:
  float ProcessBlockOne(float f) {
    const float freq = f * freqmul_;
    float scalefac;
    if (freq != freq_) {
      freq_ = freq;
      scalefac = 1.0f / (4.0f * freq * (1.0f - freq / static_cast<float>(sr_)));
      scalefac_ = scalefac;
    } else {
      scalefac = scalefac_;
    }
    phase_ += freq;
    if (phase_ >= 1.0) phase_ -= 2.0;
    if (phase_ < -1.0) phase_ += 2.0;
    const float oldval = val_;
    val_ = static_cast<float>(phase_ * phase_);
    return (val_ - oldval) * scalefac;
  }
  double sr_ = 48000.0, phase_ = 0.0;
  float freqmul_ = 0.0f, freq_ = 0.0f, freq_in_ = 440.0f;
  float val_ = 0.0f, scalefac_ = 0.0f, primed_ = 0.0f;
};

// PulseDPW = SawDPW(freq, 0) - SawDPW(freq, wrap(2*width, -1, 1))
struct PulseDPW {
  void Init(double sr, float freq, float width, int n /*unused*/) {
    a_.Init(sr, freq, 0.0);
    b_.Init(sr, freq, static_cast<double>(sc_wrap(2.0f * width, -1.0f, 1.0f)));
    a_.SetFreq(freq);
    b_.SetFreq(freq);
  }
  void SetFreq(float f) { a_.SetFreq(f); b_.SetFreq(f); }
  float Next() { return a_.Next() - b_.Next(); }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  SawDPW a_, b_;
};

// Select: index law — (int)index + 1 clipped to [1, numInputs-1] (i.e. the
// caller picks in[clip((int)idx + 1, 1, n) - 1]).
inline int SelectIndex(float idx, int n) {
  int index = static_cast<int>(idx) + 1;
  index = index < 1 ? 1 : (index > n ? n : index);
  return index - 1;
}

// Klank — a bank of up to 4-wide interleaved biquad resonators
// (OscUGens.cpp Klank_SetCoefs / Klank_next). Each partial: R = exp(log(0.001)
// / (time*decayscale*sr)), b1 = 4R^2 cos(w)/(1+R^2), b2 = -R^2,
// a0 = level*0.25; y0 = in + b1*y1 + b2*y2; out = sum(a0*y0).
struct Klank {
  static constexpr int kMaxPartials = 16;
  void Init(double sr, const float* freqs, const float* levels,
            const float* times, int numpartials, float freqscale,
            float freqoffset, float decayscale) {
    sr_ = sr;
    numpartials_ = numpartials > kMaxPartials ? kMaxPartials : numpartials;
    x1_ = x2_ = 0.0f;
    const double rps = 2.0 * M_PI / sr;
    // Source computes these coefficients in float, not double.
    const float fs = static_cast<float>(static_cast<double>(freqscale) * rps);
    const float fo = static_cast<float>(static_cast<double>(freqoffset) * rps);
    const float sampleRate = static_cast<float>(sr);
    for (int i = 0; i < numpartials_; ++i) {
      const float w = freqs[i] * fs + fo;
      const float level = levels[i];
      const float time = times[i] * decayscale;
      // Source: R = exp(log001 / (time * sampleRate)) — log001 is double,
      // time and sampleRate are floats.
      const float R = time == 0.0f
                          ? 0.0f
                          : static_cast<float>(
                                std::exp(std::log(0.001) /
                                         static_cast<double>(time * sampleRate)));
      const float twoR = 2.0f * R;
      const float R2 = R * R;
      const float cost = (twoR * std::cos(w)) / (1.0f + R2);
      b1_[i] = twoR * cost;
      b2_[i] = -R2;
      a0_[i] = level * 0.25f;
      y1_[i] = 0.0f;
      y2_[i] = 0.0f;
    }
  }
  // Resonator bank feeds a final differentiator (OscUGens.cpp tail):
  //   buf[n] = sum of resonators; out[n] = buf[n] - buf[n-2].
  float Next(float in) {
    float sum = 0.0f;
    for (int i = 0; i < numpartials_; ++i) {
      const float y0 = in + b1_[i] * y1_[i] + b2_[i] * y2_[i];
      y2_[i] = y1_[i];
      y1_[i] = y0;
      sum += a0_[i] * y0;
    }
    const float buf = sum;
    const float out = buf - x2_;
    x2_ = x1_;
    x1_ = buf;
    return out;
  }
  void ProcessBlock(const float* in, float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next(in[i]);
  }
 private:
  double sr_ = 48000.0;
  int numpartials_ = 0;
  float b1_[kMaxPartials] = {0}, b2_[kMaxPartials] = {0};
  float a0_[kMaxPartials] = {0};
  float y1_[kMaxPartials] = {0}, y2_[kMaxPartials] = {0};
  float x1_ = 0.0f, x2_ = 0.0f;  // differentiator history
};

}  // namespace sc
#endif
