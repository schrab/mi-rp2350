// sc_sugar.h — SC classlib expansion helpers for the Dronecaster kit.
// These are EXPANSION LAWS, not new DSP. PMOsc is a composition of two
// SinOsc UGens (already in the kit); VarLag/Env and DynKlank remain separate
// tasks. Mix = array sum; SelectX = XFade2 of two Selects; TChoose =
// Select(TIRand). Licensing: classlib code is GPL-3.0.
#ifndef RACK_SRC_DRONECASTER_SC_SUGAR_H_
#define RACK_SRC_DRONECASTER_SC_SUGAR_H_
#include <cmath>
#include <cstddef>
#include <vector>
#include "sc_kit_osc.h"
#include "sc_kit_osc2.h"
#include "sc_rgen.h"

namespace sc {

// Mix([a,b,c]) = a + b + c (classlib Mix: a UGen per input, summed).
template <typename T>
static inline T Mix(const std::vector<T>& ins) {
  T sum = T(0);
  for (const auto& v : ins) sum = sum + v;
  return sum;
}

// SelectX — Osc.sc:256-264. Equal-power crossfade between Select(which.round(2))
// and Select(which.trunc(2)+1), with xfade = (which*2-1).fold2(1).
struct SelectX {
  void Init(const KitTables* t) { t_ = t; }
  void ProcessBlock(float which, const float* const* ins, int count, float* out,
                    int n) const {
    const int ia = static_cast<int>(std::floor(which + 0.5f));
    const int ib = static_cast<int>(std::floor(which)) + 1;
    float p = which * 2.f - 1.f;
    if (p > 1.f) p = 1.f;
    if (p < -1.f) p = -1.f;
    const int idx = static_cast<int>((p + 1.f) * 1024.f + 0.5f);
    const float la = t_->sine[2048 - idx], ra = t_->sine[idx];
    for (int i = 0; i < n; ++i) {
      auto sel = [&](int k) -> float {
        if (count < 1) return 0.f;
        int q = k % count;
        if (q < 0) q += count;
        return ins[q][i];
      };
      out[i] = sel(ia) * la + sel(ib) * ra;
    }
  }
  const KitTables* t_ = nullptr;
};

// TChoose — Osc.sc:317-325: Select(TIRand(0, lastIndex, trig), array).
struct TChoose {
  void Init(RGen* rng, int count) {
    rng_ = rng;
    prevtrig_ = 0.f;
    // TIRand_Ctor draws once at construction (m_value) before any trigger.
    index_ = rng_->irand(count);
  }
  void ProcessBlock(const float* trig, float* out, int n, const float* vals,
                    int count) {
    for (int i = 0; i < n; ++i) {
      if (count > 0 && trig[i] > 0.f && prevtrig_ <= 0.f)
        index_ = rng_->irand(count);  // TIRand: range = hi - lo + 1 = count
      prevtrig_ = trig[i];
      out[i] = count > 0 ? vals[index_] : 0.f;
    }
  }
  RGen* rng_ = nullptr;
  int index_ = 0;
  float prevtrig_ = 0.f;
};

// DynKlank — FSinOsc.sc:68-84 expands to
//   Ringz.ar(input, freqs*freqscale+freqoffset, decays*decayscale, amps).sum
// Ringz is the same 2-pole resonator bank as Klank (OscUGens.cpp:2905).
// Law-only wrapper: reuse sc::Klank's per-partial recurrences/sum, feeding
// the current partial vectors on every block (classlib rewrites the UGen
// graph when a spec is a demand input; here the caller supplies the arrays).
struct DynKlank {
  void Init(double sr, const float* freqs, const float* amps,
            const float* decays, int count, float freqscale, float freqoffset,
            float decayscale) {
    sr_ = sr;
    freqs_ = freqs; amps_ = amps; decays_ = decays; count_ = count;
    fs_ = freqscale; fo_ = freqoffset; ds_ = decayscale;
    std::vector<float> f(static_cast<size_t>(count)), a(static_cast<size_t>(count)),
        d(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
      f[static_cast<size_t>(i)] = freqs[i] * fs_ + fo_;
      a[static_cast<size_t>(i)] = amps[i];
      d[static_cast<size_t>(i)] = decays[i] * ds_;
    }
    bank_.Init(sr, f.data(), a.data(), d.data(), count, 1.f, 0.f, 1.f);
  }
  void ProcessBlock(const float* in, float* out, int n) {
    bank_.ProcessBlock(in, out, n);
  }
  double sr_ = 48000.;
  const float* freqs_ = nullptr;
  const float* amps_ = nullptr;
  const float* decays_ = nullptr;
  int count_ = 0;
  float fs_ = 1.f, fo_ = 0.f, ds_ = 1.f;
  sc::Klank bank_;
};

// PMOsc — backwards_compatibility/PMOsc.sc: SinOsc.ar(carfreq,
//   SinOsc.ar(modfreq, modphase, pmindex)).
// Law-only port: two phase accumulators over KitTables::sine_wt; the
// modulator's scaled output is added to the carrier phase before lookup.
// The index conversion is SC's m_radtoinc = (kSineSize/2)*(1/2pi)*65536.
struct PMOsc {
  void Init(const KitTables* t, double sr, float carfreq, float modfreq,
            float pmindex, float modphase) {
    t_ = t;
    const double inc_scale = static_cast<double>(kSineSize) * (1.0 / sr) * 65536.0;
    car_inc_ = static_cast<uint32_t>(static_cast<int32_t>(inc_scale * carfreq));
    mod_inc_ = static_cast<uint32_t>(static_cast<int32_t>(inc_scale * modfreq));
    radtoinc_ = static_cast<float>(
        static_cast<double>(kSineSize >> 1) * (1.0 / 6.2831853071795864769) *
        65536.0);
    pmindex_ = pmindex;
    car_phase_ = 0;
    mod_phase_ = static_cast<uint32_t>(static_cast<int32_t>(modphase * radtoinc_));
  }
  void ProcessBlock(float* out, int n, float pmindex) {
    const int32_t mask = (kSineSize - 1) << 3;
    for (int i = 0; i < n; ++i) {
      const float m = lookupi1(t_->sine_wt, t_->sine_wt + 1, mod_phase_, mask);
      const uint32_t p = car_phase_ +
          static_cast<uint32_t>(static_cast<int32_t>(m * pmindex * radtoinc_));
      out[i] = lookupi1(t_->sine_wt, t_->sine_wt + 1, p, mask);
      car_phase_ += car_inc_;
      mod_phase_ += mod_inc_;
    }
  }
  const KitTables* t_ = nullptr;
  uint32_t car_phase_ = 0, mod_phase_ = 0, car_inc_ = 0, mod_inc_ = 0;
  float radtoinc_ = 0.f, pmindex_ = 0.f;
};

// Linen — LFUGens.cpp:2910 (ctor) / 2923 (next_k). A four-stage envelope:
// 0 attack, 1 sustain, 2 release, 3 done, 4 idle. Ctor: level=0, stage=4,
// prevGate=0; gate <= -1 at construction starts an early release (stage 1).
// Control-rate: one output per control block.
struct Linen {
  void Init(double sr, float attack, float sus_level, float release) {
    sr_ = static_cast<float>(sr);
    level_ = 0.f;
    stage_ = 4;
    prev_gate_ = 0.f;
    attack_ = attack; sus_ = sus_level; release_ = release;
  }
  float Process(float gate) {
    if (prev_gate_ <= 0.f && gate > 0.f) {
      stage_ = 0;
      const int counter = std::max(1, static_cast<int>(attack_ * sr_));
      slope_ = (sus_ - level_) / static_cast<float>(counter);
      counter_ = counter;
    }
    float out;
    switch (stage_) {
      case 0: case 2:
        out = level_;
        level_ += slope_;
        if (--counter_ == 0) ++stage_;
        break;
      case 1:
        out = level_;
        if (gate <= -1.f) {
          stage_ = 2;
          const float rt = -gate - 1.f;
          const int counter = std::max(1, static_cast<int>(rt * sr_));
          slope_ = -level_ / static_cast<float>(counter);
          counter_ = counter;
        } else if (gate <= 0.f) {
          stage_ = 2;
          const int counter = std::max(1, static_cast<int>(release_ * sr_));
          slope_ = -level_ / static_cast<float>(counter);
          counter_ = counter;
        }
        break;
      case 3: out = 0.f; ++stage_; break;
      default: out = 0.f; break;
    }
    prev_gate_ = gate;
    return out;
  }
  float sr_ = 48000.f, level_ = 0.f, slope_ = 0.f;
  float attack_ = 0.01f, sus_ = 1.f, release_ = 1.f, prev_gate_ = 0.f;
  int counter_ = 0, stage_ = 4;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_SUGAR_H_
