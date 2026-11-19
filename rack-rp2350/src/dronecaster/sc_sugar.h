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

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_SUGAR_H_
