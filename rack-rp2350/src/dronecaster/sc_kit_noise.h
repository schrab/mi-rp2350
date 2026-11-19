// sc_kit_noise.h — SC 3.14.1 noise + random UGens (GPL-3.0, CORE
// NoiseUGens.cpp / DynNoiseUGens.cpp). Every port keeps the ctor PRIMING
// draws exactly as the source: the priming calls advance the shared RGen,
// so a port that skips them desyncs every later random value in the drone.
//
// Anchors (Version-3.14.1):
//   NoiseUGens.cpp: ClipNoise_next 208, GrayNoise 216, WhiteNoise 235,
//   PinkNoise 251, BrownNoise 296 (ctor primes frand2), Dust 313/323,
//   Dust2 336/345, Crackle 372 (ctor seeds y1 = drand), Rand_Ctor 443,
//   TRand 453/467/509, TExpRand 530/544, IRand_Ctor 606, TIRand 615,
//   CoinGate 692/703/713, LinRand_Ctor 806, NRand_Ctor 825, ExpRand 841,
//   LFNoise0 923/985, LFNoise1 1000/1029, LFNoise2 1041/1076.
//   DynNoiseUGens.cpp: LFDNoise0 189/208, LFDNoise3 216/247.
//   SC_RGen.h: frand()/frand0()/frand2()/frand8()/fcoin()/drand()/irand().
//   SC_SndBuf.h: cubicinterp (4-point Hermite).
//
// Rate law: scsynth runs a .kr UGen once per block with inNumSamples = 1;
// this kit mirrors that with ProcessKr() (one value per internal block) and
// ProcessBlock() for the .ar forms. LF noise gates its update on
// sampleRate/freq where sampleRate is sr (ar) or sr/64 (kr).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_NOISE_H_
#define RACK_SRC_DRONECASTER_SC_KIT_NOISE_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"
#include "sc_rgen.h"

namespace sc {

// SC_SndBuf.h cubicinterp (4-point, 3rd-order Hermite)
inline float cubicinterp(float x, float y0, float y1, float y2, float y3) {
  const float c0 = y1;
  const float c1 = 0.5f * (y2 - y0);
  const float c2 = y0 - 2.5f * y1 + 2.0f * y2 - 0.5f * y3;
  const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
  return ((c3 * x + c2) * x + c1) * x + c0;
}

// sc_wrap (SC_InlineUnaryOp.h law): wrap to [lo, hi)
inline float sc_wrap(float x, float lo, float hi) {
  const float range = hi - lo;
  if (x >= hi) { x -= range; if (x >= hi) x -= range; }
  else if (x < lo) { x += range; if (x < lo) x += range; }
  return x;
}

// WhiteNoise.ar — NoiseUGens.cpp:235 (ctor primes 1 sample)
struct WhiteNoise {
  void Init(RGen* r) { r_ = r; r_->frand2(); }   // ctor priming draw
  float Process() { return r_->frand2(); }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = r_->frand2();
  }
  float ProcessKr() { return r_->frand2(); }
 private:
  RGen* r_ = nullptr;
};

// ClipNoise.ar — NoiseUGens.cpp:208 (ctor primes 1 sample)
struct ClipNoise {
  void Init(RGen* r) { r_ = r; r_->fcoin(); }
  float Process() { return r_->fcoin(); }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = r_->fcoin();
  }
 private:
  RGen* r_ = nullptr;
};

// GrayNoise.ar — NoiseUGens.cpp:216 (ctor primes 1 sample)
struct GrayNoise {
  void Init(RGen* r) { r_ = r; counter_ = 0; Step(); }
  float Step() {
    counter_ ^= 1L << (r_->trand() & 31);
    return static_cast<float>(counter_) * 4.65661287308e-10f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Step();
  }
 private:
  RGen* r_ = nullptr;
  int32_t counter_ = 0;
};

// PinkNoise.ar — NoiseUGens.cpp:251 (ctor: 16 dice draws + 1 sample prime)
struct PinkNoise {
  void Init(RGen* r) {
    r_ = r;
    total_ = 0;
    for (int i = 0; i < 16; ++i) {
      const uint32_t v = r_->trand() >> 13;
      total_ += static_cast<int32_t>(v);
      dice_[i] = v;
    }
    Next();                                    // ctor priming sample
  }
  float Next() {
    uint32_t counter = r_->trand();            // Magnus Jonsson's suggestion
    const uint32_t newrand = counter >> 13;
    const int k = (ctz(counter)) & 15;
    const uint32_t prevrand = dice_[k];
    dice_[k] = newrand;
    total_ += static_cast<int32_t>(newrand - prevrand);
    const uint32_t r2 = r_->trand() >> 13;
    union { uint32_t u; float f; } val;
    val.u = (static_cast<uint32_t>(total_) + r2) | 0x40000000u;
    return val.f - 3.0f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  static int ctz(uint32_t v) {
    int n = 0;
    while ((v & 1u) == 0u && n < 32) { v >>= 1; ++n; }
    return n;
  }
  RGen* r_ = nullptr;
  uint32_t dice_[16] = {0};
  int32_t total_ = 0;
};

// BrownNoise.ar — NoiseUGens.cpp:296 (ctor seeds level = frand2)
struct BrownNoise {
  void Init(RGen* r) { r_ = r; level_ = r_->frand2(); }
  float Next() {
    level_ += r_->frand8();
    if (level_ > 1.0f) level_ = 2.0f - level_;
    else if (level_ < -1.0f) level_ = -2.0f - level_;
    return level_;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  RGen* r_ = nullptr;
  float level_ = 0.0f;
};

// Crackle.ar — NoiseUGens.cpp:372 (ctor seeds y1 = drand(), then primes)
struct Crackle {
  void Init(RGen* r, float param) {
    r_ = r;
    param_ = param;
    y1_ = static_cast<float>(r_->drand());
    y2_ = 0.0f;
    Next();                                    // ctor priming sample
  }
  void SetParam(float p) { param_ = p; }
  float Next() {
    // Source: float math throughout (y1*paramf - y2 - 0.05f in float).
    const float y0 = fabsf(y1_ * param_ - y2_ - 0.05f);
    y2_ = y1_;
    y1_ = y0;
    return y0;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  RGen* r_ = nullptr;
  float param_ = 0.0f;
  float y1_ = 0.0f, y2_ = 0.0f;
};

// Dust.ar/.kr — NoiseUGens.cpp:313/323 (thresh = density*sampleDur;
// scale = 1/thresh; ctor primes 1 sample with density 0).
struct Dust {
  void Init(RGen* r, double sr, float density) {
    r_ = r; sr_ = sr; density_ = density;
    Update();
    Next();
  }
  void SetDensity(float d) { density_ = d; Update(); }
  float Next() {
    const float z = r_->frand();
    return (z < thresh_) ? z * scale_ : 0.0f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  void Update() {
    thresh_ = density_ * static_cast<float>(1.0 / sr_);
    scale_ = thresh_ > 0.0f ? 1.0f / thresh_ : 0.0f;
  }
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float density_ = 0.0f, thresh_ = 0.0f, scale_ = 0.0f;
};

// Dust2.ar — NoiseUGens.cpp:336/345 (bipolar: z*scale - 1)
struct Dust2 {
  void Init(RGen* r, double sr, float density) {
    r_ = r; sr_ = sr; density_ = density;
    Update();
    Next();
  }
  void SetDensity(float d) { density_ = d; Update(); }
  float Next() {
    const float z = r_->frand();
    return (z < thresh_) ? z * scale_ - 1.0f : 0.0f;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
 private:
  void Update() {
    thresh_ = density_ * static_cast<float>(1.0 / sr_);
    scale_ = thresh_ > 0.0f ? 2.0f / thresh_ : 0.0f;
  }
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float density_ = 0.0f, thresh_ = 0.0f, scale_ = 0.0f;
};

// CoinGate.kr — NoiseUGens.cpp:703 (frand() < probability on a rising edge;
// no ctor draw, m_trig = trig input).
struct CoinGate {
  void Init(RGen* r, float prob, float trig) { r_ = r; prob_ = prob; prev_ = trig; }
  float Next(float trig) {
    float level = 0.0f;
    if (trig > 0.0f && prev_ <= 0.0f) {
      if (r_->frand() < prob_) level = trig;
    }
    prev_ = trig;
    return level;
  }
 private:
  RGen* r_ = nullptr;
  float prob_ = 0.0f, prev_ = 0.0f;
};

// LFNoise0/1/2 (kr + ar) — NoiseUGens.cpp:923/1000/1041. The ctor sequences
// are exact: LFNoise0 draws 1; LFNoise1 draws level then one nextlevel in its
// priming call; LFNoise2 draws nextvalue, then nextvalue+midpt in the prime.
struct LFNoise0 {
  void Init(RGen* r, double sr, float freq) {
    r_ = r; sr_ = sr; freq_ = freq;
    counter_ = 0; level_ = 0.0f;
    if (counter_ <= 0) {                        // ctor priming (next_1)
      counter_ = Period();
      level_ = r_->frand2();
    }
    counter_ -= 1;
    primed_ = level_;                           // the ctor's output value
  }
  float Next() {
    if (counter_ <= 0) {
      counter_ = Period();
      level_ = r_->frand2();
    }
    counter_ -= 1;
    return level_;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  // K2A/CALCSLOPE consumers ramp from the ctor value on block 0 (see
  // ControlRateInput::slope — verified against the T-D1 env law).
  float ctor_value() const { return primed_; }
 private:
  int32_t Period() const {
    int32_t c = static_cast<int32_t>(sr_ / (freq_ > 0.001f ? freq_ : 0.001f));
    return c < 1 ? 1 : c;
  }
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float freq_ = 0.0f, level_ = 0.0f, primed_ = 0.0f;
  int32_t counter_ = 0;
};

struct LFNoise1 {
  void Init(RGen* r, double sr, float freq) {
    r_ = r; sr_ = sr; freq_ = freq;
    counter_ = 0;
    level_ = r_->frand2();
    slope_ = 0.0f;
    // ctor next(1): counter<=0 -> period, nextlevel, slope; output; advance
    counter_ = Period();
    {
      primed_ = level_;                          // the ctor's output value
      const float nextlevel = r_->frand2();
      slope_ = (nextlevel - level_) / static_cast<float>(counter_);
    }
    level_ += slope_;
    counter_ -= 1;
  }
  float Next() {
    const float out = level_;
    if (counter_ <= 0) {
      counter_ = Period();
      const float nextlevel = r_->frand2();
      slope_ = (nextlevel - level_) / static_cast<float>(counter_);
    }
    level_ += slope_;
    counter_ -= 1;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return primed_; }
 private:
  int32_t Period() const {
    int32_t c = static_cast<int32_t>(sr_ / (freq_ > 0.001f ? freq_ : 0.001f));
    return c < 1 ? 1 : c;
  }
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float freq_ = 0.0f, level_ = 0.0f, slope_ = 0.0f, primed_ = 0.0f;
  int32_t counter_ = 0;
};

struct LFNoise2 {
  void Init(RGen* r, double sr, float freq) {
    r_ = r; sr_ = sr; freq_ = freq;
    counter_ = 0;
    slope_ = 0.0f;
    level_ = 0.0f;
    next_value_ = r_->frand2();
    next_midpt_ = next_value_ * 0.5f;
    primed_ = Next();                          // ctor priming call
  }
  float Next() {
    if (counter_ <= 0) {
      const float value = next_value_;
      next_value_ = r_->frand2();
      level_ = next_midpt_;
      next_midpt_ = (next_value_ + value) * 0.5f;
      int32_t c = static_cast<int32_t>(sr_ / (freq_ > 0.001f ? freq_ : 0.001f));
      counter_ = c < 2 ? 2 : c;
      const float fseglen = static_cast<float>(counter_);
      curve_ = 2.0f * (next_midpt_ - level_ - fseglen * slope_) /
               (fseglen * fseglen + fseglen);
    }
    // Source order: output level, then advance (slope += curve; level +=
    // slope). On a segment-start sample `level` is the new midpt.
    const float out = level_;
    slope_ += curve_;
    level_ += slope_;
    counter_ -= 1;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return primed_; }
 private:
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float freq_ = 0.0f, level_ = 0.0f, slope_ = 0.0f, curve_ = 0.0f;
  float next_value_ = 0.0f, next_midpt_ = 0.0f, primed_ = 0.0f;
  int32_t counter_ = 0;
};

// LFClipNoise.ar — NoiseUGens.cpp:179/…: level = fcoin() on each period.
struct LFClipNoise {
  void Init(RGen* r, double sr, float freq) {
    r_ = r; sr_ = sr; freq_ = freq;
    counter_ = 0; level_ = 0.0f;
    primed_ = Next();                          // ctor priming call
  }
  float Next() {
    // Source order: counter check + level draw, THEN output (the segment's
    // first sample already carries the new level — unlike LFNoise0/1/2,
    // whose segment init also sets the output).
    if (counter_ <= 0) {
      counter_ = static_cast<int32_t>(sr_ / (freq_ > 0.001f ? freq_ : 0.001f));
      if (counter_ < 1) counter_ = 1;
      level_ = r_->fcoin();
    }
    const float out = level_;
    counter_ -= 1;
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return primed_; }
 private:
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float freq_ = 0.0f, level_ = 0.0f, primed_ = 0.0f;
  int32_t counter_ = 0;
};

// LFDNoise0 (kr/ar) — DynNoiseUGens.cpp:189/208: phase ramp, level = frand2
// on wrap. The ctor sets phase=0, level=0, levels a=b=c=d=0, then primes.
struct LFDNoise0 {
  void Init(RGen* r, double sr, float freq) {
    r_ = r; sr_ = sr; freq_ = freq;
    phase_ = 0.0f; level_ = 0.0f;
    primed_ = Next();                          // ctor priming call
  }
  float Next() {
    // Source order: phase update, wrap (draw), THEN output level.
    phase_ -= static_cast<float>(freq_ * (1.0 / sr_));
    if (phase_ < 0.0f) {
      phase_ = sc_wrap(phase_, 0.0f, 1.0f);
      level_ = r_->frand2();
    }
    return level_;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return primed_; }
 private:
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float freq_ = 0.0f, phase_ = 0.0f, level_ = 0.0f, primed_ = 0.0f;
};

// LFDNoise3 (kr/ar) — DynNoiseUGens.cpp:216/247: 4-point Hermite over
// frand2()*0.8 on a phase ramp (max interpolation overshoot limited to 1).
struct LFDNoise3 {
  void Init(RGen* r, double sr, float freq) {
    r_ = r; sr_ = sr; freq_ = freq;
    phase_ = 0.0f;
    // Ctor: FOUR frand2 draws (each x0.8) into a..d, THEN the priming call.
    a_ = r_->frand2() * 0.8f;
    b_ = r_->frand2() * 0.8f;
    c_ = r_->frand2() * 0.8f;
    d_ = r_->frand2() * 0.8f;
    primed_ = Next();                          // ctor priming call
  }
  float Next() {
    float out;
    phase_ -= static_cast<float>(freq_ * (1.0 / sr_));
    if (phase_ < 0.0f) {
      phase_ = sc_wrap(phase_, 0.0f, 1.0f);
      a_ = b_;
      b_ = c_;
      c_ = d_;
      d_ = r_->frand2() * 0.8f;
    }
    out = cubicinterp(1.0f - phase_, a_, b_, c_, d_);
    return out;
  }
  void ProcessBlock(float* out, int n) {
    for (int i = 0; i < n; ++i) out[i] = Next();
  }
  float ProcessKr() { return Next(); }
  float ctor_value() const { return primed_; }
 private:
  RGen* r_ = nullptr;
  double sr_ = 48000.0;
  float freq_ = 0.0f, phase_ = 0.0f, primed_ = 0.0f;
  float a_ = 0.0f, b_ = 0.0f, c_ = 0.0f, d_ = 0.0f;
};

// ---- init/trigger-time random UGens (NoiseUGens.cpp) ----
inline float RandIrg(RGen* r, float lo, float hi) {   // Rand (443)
  return r->frand() * (hi - lo) + lo;
}
inline int32_t IRandIrg(RGen* r, int lo, int hi) {    // IRand (606)
  return r->irand(hi - lo + 1) + lo;
}
struct TRand {                                        // TRand (509)
  void Init(RGen* r, float lo, float hi, float trig) {
    r_ = r; lo_ = lo; hi_ = hi;
    value_ = r_->frand() * (hi - lo) + lo;             // ctor draw
    prev_ = trig;
  }
  float Next(float trig) {
    if (trig > 0.0f && prev_ <= 0.0f) {
      value_ = r_->frand() * (hi_ - lo_) + lo_;
    }
    prev_ = trig;
    return value_;
  }
 private:
  RGen* r_ = nullptr;
  float lo_ = 0.0f, hi_ = 0.0f, value_ = 0.0f, prev_ = 0.0f;
};
struct TExpRand {                                     // TExpRand (530/544)
  void Init(RGen* r, float lo, float hi, float trig) {
    r_ = r; lo_ = lo; hi_ = hi;
    ratio_ = hi / lo;
    value_ = std::pow(ratio_, r_->frand()) * lo;       // ctor draw
    prev_ = trig;
  }
  float Next(float trig) {
    if (trig > 0.0f && prev_ <= 0.0f) {
      value_ = std::pow(ratio_, r_->frand()) * lo_;
    }
    prev_ = trig;
    return value_;
  }
 private:
  RGen* r_ = nullptr;
  float lo_ = 0.0f, hi_ = 0.0f, value_ = 0.0f, prev_ = 0.0f, ratio_ = 1.0f;
};
inline float LinRandIrg(RGen* r, float lo, float hi, int n) {  // LinRand (806)
  const float a = r->frand();
  const float b = r->frand();
  return (n <= 0 ? (a < b ? a : b) : (a > b ? a : b)) * (hi - lo) + lo;
}
inline float NRandIrg(RGen* r, float lo, float hi, int n) {    // NRand (825)
  float sum = 0.0f;
  for (int i = 0; i < n; ++i) sum += r->frand();
  return (sum / static_cast<float>(n)) * (hi - lo) + lo;
}
inline float ExpRandIrg(RGen* r, float lo, float hi) {         // ExpRand (841)
  return std::pow(hi / lo, r->frand()) * lo;
}

}  // namespace sc
#endif
