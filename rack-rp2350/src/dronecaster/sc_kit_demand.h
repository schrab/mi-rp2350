// sc_kit_demand.h — SC 3.14.1 demand-rate kit (GPL-3.0, DemandUGens.cpp).
// Demand is PULLED by a triggering UGen: on each trigger edge it asks every
// source for its next value. A NaN means "sequence done" (the source is reset
// by the reset input); the previous sample is held instead.
// Anchors: Demand_Ctor 402 / Demand_next_ak 314; Dseries 1290/1314;
// Dbrown 1424/1458; Dseq 1506/1562; Drand 1620/1661; Dxrand 1668/1715.
// Dbufrd is host-testable via DemandBuffer; Dbufwr's NRT reference renders
// garbage here, so only the read side has a reference so far.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_DEMAND_H_
#define RACK_SRC_DRONECASTER_SC_KIT_DEMAND_H_
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include "sc_rgen.h"

namespace sc {

static inline float DemandNaN() {
  return std::numeric_limits<float>::quiet_NaN();
}
static inline bool IsNaN(float x) { return x != x; }
static inline float Fold(float in, float lo, float hi) {
  float x = in - lo;
  if (in >= hi) {
    in = hi + hi - in;
    if (in >= lo) return in;
  } else if (in < lo) {
    in = lo + lo - in;
    if (in < hi) return in;
  } else return in;
  if (hi == lo) return lo;
  const float range = hi - lo, range2 = range + range;
  float c = x - range2 * std::floor(x / range2);
  if (c >= range) c = range2 - c;
  return c + lo;
}

struct DemandSource {
  virtual ~DemandSource() = default;
  virtual void Reset() = 0;
  virtual float Pull() = 0;
};

// values[0] = repeats, values[1..] = sequence. inf repeats uses INFINITY.
struct DSeq : DemandSource {
  explicit DSeq(std::vector<float> v) : v_(std::move(v)) {}
  void Reset() override { repeats_ = -1.f; count_ = 0; index_ = 1; }
  float Pull() override {
    if (repeats_ < 0.f) repeats_ = std::floor(v_[0] + 0.5f);
    while (true) {
      if (index_ >= static_cast<int>(v_.size())) { index_ = 1; ++count_; }
      if (count_ >= repeats_) return DemandNaN();
      const float x = v_[static_cast<size_t>(index_)];
      ++index_;
      return x;
    }
  }
 private:
  std::vector<float> v_;
  float repeats_ = -1.f;
  int count_ = 0, index_ = 1;
};

// values[0] = repeats, values[1..] = choices; RNG-driven with replacement.
struct DRand : DemandSource {
  DRand(std::vector<float> v, RGen* rng) : v_(std::move(v)), rng_(rng) {}
  void Reset() override {
    repeats_ = -1.f; count_ = 0;
    index_ = rng_->irand(static_cast<int32_t>(v_.size()) - 1) + 1;
  }
  float Pull() override {
    if (repeats_ < 0.f) repeats_ = std::floor(v_[0] + 0.5f);
    if (count_ >= repeats_) return DemandNaN();
    const float x = v_[static_cast<size_t>(index_)];
    index_ = rng_->irand(static_cast<int32_t>(v_.size()) - 1) + 1;
    ++count_;
    return x;
  }
 private:
  std::vector<float> v_;
  RGen* rng_ = nullptr;
  float repeats_ = -1.f;
  int count_ = 0, index_ = 1;
};

// values[0] = repeats, values[1..] = choices; no immediate repeat.
struct DXRand : DemandSource {
  DXRand(std::vector<float> v, RGen* rng) : v_(std::move(v)), rng_(rng) {}
  void Reset() override {
    repeats_ = -1.f; count_ = 0;
    // Source's reset path also consumes an irand draw and skips past self.
    const int nw = rng_->irand(static_cast<int32_t>(v_.size()) - 2) + 1;
    index_ = nw < index_ ? nw : nw + 1;
  }
  float Pull() override {
    if (repeats_ < 0.f) repeats_ = std::floor(v_[0] + 0.5f);
    while (true) {
      if (index_ >= static_cast<int>(v_.size())) index_ = 1;
      if (count_ >= repeats_) return DemandNaN();
      const float x = v_[static_cast<size_t>(index_)];
      const int nw = rng_->irand(static_cast<int32_t>(v_.size()) - 2) + 1;
      index_ = nw < index_ ? nw : nw + 1;
      ++count_;
      return x;
    }
  }
 private:
  std::vector<float> v_;
  RGen* rng_ = nullptr;
  float repeats_ = -1.f;
  int count_ = 0, index_ = 1;
};

// values[0] = repeats, values[1] = start, values[2] = step.
struct DSeries : DemandSource {
  explicit DSeries(std::vector<float> v) : v_(std::move(v)) {}
  void Reset() override { repeats_ = -1.f; count_ = 0; }
  float Pull() override {
    if (repeats_ < 0.f) {
      repeats_ = std::floor(v_[0] + 0.5f);
      value_ = v_[1];
    }
    if (count_ >= repeats_) return DemandNaN();
    const float out = value_;
    value_ += v_[2];
    ++count_;
    return out;
  }
 private:
  std::vector<float> v_;
  float repeats_ = -1.f, value_ = 0.f;
  int count_ = 0;
};

// values[0] = repeats, [1] = lo, [2] = hi, [3] = step.
struct DBrown : DemandSource {
  DBrown(std::vector<float> v, RGen* rng) : v_(std::move(v)), rng_(rng) {}
  void Reset() override { repeats_ = -1.f; count_ = 0; }
  float Pull() override {
    if (repeats_ < 0.f) {
      repeats_ = std::floor(v_[0] + 0.5f);
      value_ = rng_->frand() * (v_[2] - v_[1]) + v_[1];
    }
    if (count_ >= repeats_) return DemandNaN();
    ++count_;
    const float out = value_;
    value_ = Fold(value_ + rng_->frand2() * v_[3], v_[1], v_[2]);
    return out;
  }
 private:
  std::vector<float> v_;
  RGen* rng_ = nullptr;
  float repeats_ = -1.f, value_ = 0.f;
  int count_ = 0;
};

// Host stand-in for the SndBuf a Dbufrd reads (Task 12 buffer infrastructure).
struct DemandBuffer {
  void Init(int frames) { data.assign(static_cast<size_t>(frames), 0.f); }
  std::vector<float> data;
};
struct DBufRd : DemandSource {
  DBufRd(DemandBuffer* b, float phase, int loop)
      : buf_(b), phase_(phase), loop_(loop) {}
  void Reset() override {}
  float Pull() override {
    int ip = static_cast<int>(phase_);
    const int frames = static_cast<int>(buf_->data.size());
    if (ip >= frames) ip = loop_ ? ip % frames : frames - 1;
    if (ip < 0) ip = loop_ ? ((ip % frames) + frames) % frames : 0;
    return buf_->data[static_cast<size_t>(ip)];
  }
 private:
  DemandBuffer* buf_; float phase_; int loop_;
};

// DBufWr — DemandUGens.cpp:2273. Demand source that writes `value` at
// floor(phase) into the shared SndBuf and returns the written value. Law-only
// (its NRT ref renders a no-op here — the write never lands in LocalBuf).
struct DBufWr : DemandSource {
  DBufWr(DemandBuffer* b, float phase, float value, int loop)
      : buf_(b), phase_(phase), value_(value), loop_(loop) {}
  void Reset() override {}
  float Pull() override {
    int ip = static_cast<int>(phase_);
    const int frames = static_cast<int>(buf_->data.size());
    if (ip >= frames) ip = loop_ ? ip % frames : frames - 1;
    if (ip < 0) ip = loop_ ? ((ip % frames) + frames) % frames : 0;
    buf_->data[static_cast<size_t>(ip)] = value_;
    return value_;
  }
 private:
  DemandBuffer* buf_;
  float phase_, value_;
  int loop_;
};

struct Demand {
  void Init(std::vector<DemandSource*> sources) {
    sources_ = std::move(sources);
    prev_.assign(sources_.size(), 0.f);
    prevtrig_ = 0.f;
    prevreset_ = 0.f;
    done_ = false;
  }
  void ProcessBlock(const float* trig, float reset, float** out, int n) {
    for (int i = 0; i < n; ++i) {
      if (reset > 0.f && prevreset_ <= 0.f)
        for (auto* s : sources_) s->Reset();
      if (trig[i] > 0.f && prevtrig_ <= 0.f) {
        for (size_t k = 0; k < sources_.size(); ++k) {
          float x = sources_[k]->Pull();
          if (IsNaN(x)) { x = prev_[k]; done_ = true; }
          else prev_[k] = x;
          out[k][i] = x;
        }
      } else {
        for (size_t k = 0; k < sources_.size(); ++k) out[k][i] = prev_[k];
      }
      prevtrig_ = trig[i];
      prevreset_ = reset;
    }
  }
  std::vector<DemandSource*> sources_;
  std::vector<float> prev_;
  float prevtrig_ = 0.f, prevreset_ = 0.f;
  bool done_ = false;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_DEMAND_H_
