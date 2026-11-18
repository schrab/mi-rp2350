// sc_kit_pulse.h — exact port of SC 3.14.1 Pulse (server/plugins/OscUGens.cpp
// 2506-2721, identical to 3.13.1; GPL-3.0). A BLIT-style pulse train: sine/
// cosecant lookup with a duty phase offset, an N-change crossfade, and a
// 0.999 leaky integrator. There are NO pulse wavetables in SC 3.x (that
// model is from SC 2.x).
//
// Indexing law verified against the source: the C tables are addressed with
// a BYTE offset ((phase >> 14) & 0x7FFC), which equals the float index
// ((phase >> 16) & 8191) used here. `scale = 0.5 / N` is a DOUBLE division
// rounded to float (the source assigns the double quotient to m_scale).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_PULSE_H_
#define RACK_SRC_DRONECASTER_SC_KIT_PULSE_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"
#include "sc_kit_osc.h"

namespace sc {

class Pulse {
 public:
  void Init(const KitTables* t, double sr, float freq, float width) {
    t_ = t;
    sr_ = sr;
    freq_ = freq;
    width_ = width;
    freqin_ = freq;                       // ctor: unit->m_freqin = ZIN0(0)
    cpstoinc_ = static_cast<double>(kSineSize) * (1.0 / sr) * 65536.0 * 0.5;
    n_ = static_cast<int32_t>((sr * 0.5) / static_cast<double>(freq));
    scale_ = static_cast<float>(0.5 / static_cast<double>(n_));
    phase_ = 0;
    phaseoff_ = 0;
    y1_ = 0.0f;
  }
  void SetFreq(float f) { freq_ = f; }
  void SetWidth(float w) { width_ = w; }
  void ProcessBlock(float* out, int n);

 private:
  // One half of the pulse difference at `phase` (the source computes pul1 at
  // `phase` and pul2 at `phase + phaseoff` with identical arithmetic).
  float PulseAt(uint32_t phase, int32_t N2, float scale, bool crossfade,
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
      if (fabsf(denom) < 0.0005f) {
        return crossfade ? 1.0f : (1.0f / scale + 1.0f);
      }
      const uint32_t rphase = phase * static_cast<uint32_t>(N2);
      const float numer = SineAt(rphase);
      if (!crossfade) return numer / denom;
      const uint32_t rphase_prev = phase * static_cast<uint32_t>(prevN2);
      const float n1 = (SineAt(rphase_prev) / denom - 1.0f) * prevscale;
      const float n2 = (numer / denom - 1.0f) * scale;
      return lininterp(xfade, n1, n2);
    }
    const float denom = t0 + (t1 - t0) * pfrac;
    const uint32_t rphase = phase * static_cast<uint32_t>(N2);
    const float numer = SineAt(rphase);
    if (!crossfade) return numer * denom;
    const uint32_t rphase_prev = phase * static_cast<uint32_t>(prevN2);
    const float n1 = (SineAt(rphase_prev) * denom - 1.0f) * prevscale;
    const float n2 = (numer * denom - 1.0f) * scale;
    return lininterp(xfade, n1, n2);
  }

  // lininterp(PhaseFrac(rphase), sine[i], sine[i+1]) — the source's numtbl
  // lookup with the (rphase>>14)&0x7FFC byte offset (= float idx >>16&8191).
  float SineAt(uint32_t rphase) const {
    const float* tbl = t_->sine + ((rphase >> 16) & (kSineSize - 1));
    return lininterp(PhaseFrac(rphase), tbl[0], tbl[1]);
  }

  const KitTables* t_ = nullptr;
  double sr_ = 48000.0;
  double cpstoinc_ = 0.0;
  uint32_t phase_ = 0, phaseoff_ = 0;
  int32_t n_ = 0;
  float freqin_ = 0.0f, freq_ = 0.0f, width_ = 0.5f;
  float scale_ = 0.0f, y1_ = 0.0f;
};

inline void Pulse::ProcessBlock(float* out, int n) {
  const float freqin = freq_;
  const float duty = width_;
  uint32_t phase = phase_;
  float y1 = y1_;

  int32_t freq, N, prevN = 0;
  float scale, prevscale = 0.0f;
  bool crossfade;

  // Pulse_next prelude (OscUGens.cpp:2530-2565)
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
  const int32_t phaseoff_start = static_cast<int32_t>(phaseoff_);
  const int32_t next_phaseoff = static_cast<int32_t>(duty * (1L << 28));
  const int32_t phaseoff_slope =
      static_cast<int32_t>((next_phaseoff - phaseoff_start) * (1.0 / 64.0));
  phaseoff_ = static_cast<uint32_t>(next_phaseoff);
  uint32_t phaseoff = static_cast<uint32_t>(phaseoff_start);

  if (crossfade) {
    const int32_t prevN2 = 2 * prevN + 1;
    float xfade = 0.0f;
    for (int i = 0; i < n; ++i) {
      const float pul1 = PulseAt(phase, N2, scale, true, prevN2, prevscale, xfade);
      const float pul2 = PulseAt(phase + phaseoff, N2, scale, true, prevN2,
                                 prevscale, xfade);
      out[i] = y1 = pul1 - pul2 + 0.999f * y1;
      phase += static_cast<uint32_t>(freq);
      phaseoff += static_cast<uint32_t>(phaseoff_slope);
      xfade += 1.0f / 64.0f;
    }
  } else {
    for (int i = 0; i < n; ++i) {
      const float pul1 = PulseAt(phase, N2, scale, false, 0, 0.0f, 0.0f);
      const float pul2 = PulseAt(phase + phaseoff, N2, scale, false, 0, 0.0f, 0.0f);
      out[i] = y1 = (pul1 - pul2) * scale + 0.999f * y1;
      phase += static_cast<uint32_t>(freq);
      phaseoff += static_cast<uint32_t>(phaseoff_slope);
    }
  }

  phase_ = phase;
  y1_ = y1;
}

}  // namespace sc
#endif
