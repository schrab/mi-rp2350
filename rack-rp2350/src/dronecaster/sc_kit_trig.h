// sc_kit_trig.h — SC 3.14.1 trigger UGens (GPL-3.0, CORE TriggerUGens.cpp).
//
// Anchors (Version-3.14.1): Trig1_next_k 394, Trig_next_k 551,
// Latch_next_ak 999, SetResetFF_next_k 876, Gate_next_ak 1079,
// Peak_next_ak 1740, TDelay_next 1353.
//
// Model: the drones use the .kr forms, so every port here is called ONCE
// PER BLOCK (inNumSamples = 1 at kr). ar forms get their block loop through
// ProcessBlock where the law is per-sample; the value inputs are read as the
// block's first sample (ZIN0 semantics), matching the kr calc functions.
// `counter = (long)(dur * sr + .5f)` uses the rate's sample rate — pass
// sr_kr = sr/64 for .kr call sites.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_TRIG_H_
#define RACK_SRC_DRONECASTER_SC_KIT_TRIG_H_
#include <cmath>
#include <cstdint>
#include "sc_defs.h"

namespace sc {

// Trig1.kr — a one-shot window: rising edge starts a `dur`-second 1.0.
struct Trig1 {
  void Init() { prev_ = 0.0f; counter_ = 0; }
  float Next(float trig, float dur, float sr_kr) {
    float zout;
    if (counter_ > 0) {
      zout = --counter_ ? 1.0f : 0.0f;
    } else {
      if (trig > 0.0f && prev_ <= 0.0f) {
        counter_ = static_cast<long>(dur * sr_kr + 0.5f);
        if (counter_ < 1) counter_ = 1;
        zout = 1.0f;
      } else {
        zout = 0.0f;
      }
    }
    prev_ = trig;
    return zout;
  }
 private:
  float prev_ = 0.0f;
  long counter_ = 0;
};

// Trig.kr — like Trig1 but the window level = the trigger value.
struct Trig {
  void Init() { prev_ = 0.0f; counter_ = 0; level_ = 0.0f; }
  float Next(float trig, float dur, float sr_kr) {
    float zout;
    if (counter_ > 0) {
      zout = --counter_ ? level_ : 0.0f;
    } else {
      if (trig > 0.0f && prev_ <= 0.0f) {
        counter_ = static_cast<long>(dur * sr_kr + 0.5f);
        if (counter_ < 1) counter_ = 1;
        level_ = trig;
        zout = level_;
      } else {
        zout = 0.0f;
      }
    }
    prev_ = trig;
    return zout;
  }
 private:
  float prev_ = 0.0f, level_ = 0.0f;
  long counter_ = 0;
};

// TDelay.kr — delays a trigger by `dur` seconds (output pulses 1.0).
struct TDelay {
  void Init() { prev_ = 0.0f; counter_ = 0; }
  float Next(float trig, float dur, float sr_kr) {
    float zout;
    if (counter_ > 1) {
      --counter_;
      zout = 0.0f;
    } else if (counter_ <= 0) {
      if (prev_ <= 0.0f && trig > 0.0f) {
        counter_ = static_cast<long>(dur * sr_kr + 0.5f);
        if (counter_ < 1) counter_ = 1;
      }
      zout = 0.0f;
    } else {
      counter_ = 0;
      zout = 1.0f;
    }
    prev_ = trig;
    return zout;
  }
 private:
  float prev_ = 0.0f;
  long counter_ = 0;
};

// Latch.kr — samples the input on a rising trigger, holds otherwise.
struct Latch {
  void Init() { prev_ = 0.0f; level_ = 0.0f; }
  float Next(float value, float trig) {
    if (prev_ <= 0.0f && trig > 0.0f) level_ = value;
    prev_ = trig;
    return level_;
  }
 private:
  float prev_ = 0.0f, level_ = 0.0f;
};

// SetResetFF.kr — set/reset flip-flop (output 0 or 1).
struct SetResetFF {
  void Init() { prev_high_ = 0.0f; prev_low_ = 0.0f; level_ = 0.0f; }
  float Next(float high, float low) {
    // Source order: reset first, then set (reset wins on a shared edge).
    if (prev_low_ <= 0.0f && low > 0.0f) level_ = 0.0f;
    else if (prev_high_ <= 0.0f && high > 0.0f) level_ = 1.0f;
    prev_high_ = high;
    prev_low_ = low;
    return level_;
  }
 private:
  float prev_high_ = 0.0f, prev_low_ = 0.0f, level_ = 0.0f;
};

// Gate: trig > 0 passes the input through, otherwise the last level is held.
struct Gate {
  void Init() { level_ = 0.0f; }
  void ProcessBlock(const float* in, float trig, float* out, int n) {
    if (trig > 0.0f) {
      for (int i = 0; i < n; ++i) { level_ = in[i]; out[i] = level_; }
    } else {
      for (int i = 0; i < n; ++i) out[i] = level_;
    }
  }
  float Next(float value, float trig) {
    if (trig > 0.0f) level_ = value;
    return level_;
  }
 private:
  float level_ = 0.0f;
};

// Peak.kr — running max of |input|; a rising trigger resets to the current
// |input| (Peak_next_ak's post-loop reset law).
struct Peak {
  void Init() { prev_ = 0.0f; level_ = 0.0f; }
  float Next(float value, float trig) {
    const float inlevel = std::fabs(value);
    level_ = inlevel > level_ ? inlevel : level_;
    if (prev_ <= 0.0f && trig > 0.0f) level_ = inlevel;
    prev_ = trig;
    return level_;
  }
 private:
  float prev_ = 0.0f, level_ = 0.0f;
};

// Changed (classlib Filter.sc:227) — `HPZ1.kr(input).abs > threshold`.
// HPZ1 (FilterUGens.cpp:1496) sets m_x1 = ZIN0(0) at ctor, so the first
// block's difference is 0 and the first output is 0.
struct Changed {
  void Init(float threshold = 0.0f) { prev_ = 0.0f; threshold_ = threshold; started_ = false; }
  float Next(float value) {
    if (!started_) {
      started_ = true;
      prev_ = value;
      return 0.0f;
    }
    const float out = fabsf(value - prev_) > threshold_ ? 1.0f : 0.0f;
    prev_ = value;
    return out;
  }
 private:
  float prev_ = 0.0f, threshold_ = 0.0f;
  bool started_ = false;
};

}  // namespace sc
#endif
