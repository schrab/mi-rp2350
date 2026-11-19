// sc_kit_envgen.h — SC 3.14.1 EnvGen / VarLag (GPL-3.0, LFUGens.cpp).
// Anchors: enum shapes 2393, Ctor 2413, initSegment 2466, nextSegment 2604,
// perform 2635, next_k 2753. The T-D1 delta: the ctor primes one control
// sample then RESTORES INITIAL CONDITIONS (counter rolled back by 1), so the
// first block starts at level0 and the whole env shifts one call later.
// VarLag (Filter.sc:113-145) expands to Env([start,in],[time],warp)+EnvGen.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_ENVGEN_H_
#define RACK_SRC_DRONECASTER_SC_KIT_ENVGEN_H_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace sc {

struct EnvGen {
  enum Shape { Step=0, Linear=1, Exponential=2, Sine=3, Welch=4,
               Curve=5, Squared=6, Cubed=7, Hold=8, Sustain=9999 };
  struct Node { float level, dur, shape, curve; };
  static constexpr int kNotStarted = 1000000000;

  void Init(double sr, const Node* nodes, int num_stages, float init_level,
            float level_scale = 1.f, float level_bias = 0.f,
            float time_scale = 1.f, int release_node = -99, int loop_node = -99) {
    sr_ = static_cast<float>(sr);
    nodes_ = nodes;
    num_stages_ = num_stages;
    ls_ = level_scale; lb_ = level_bias; ts_ = time_scale;
    release_node_ = release_node;
    loop_node_ = loop_node;
    const double init = ScaleLevel(init_level);
    end_level_ = level_ = init;
    counter_ = 0;
    stage_ = kNotStarted;
    shape_ = Hold;
    prev_gate_ = 0.f;
    released_ = false;
    residual_ = 0.;
    done_ = false;
    const bool hold0 = static_cast<int>(nodes[0].shape) == Hold;
    if (hold0) level_ = nodes[0].level;
    // Ctor prime + restore (T-D1 law).
    float primed = ProcessKr(0.f);
    (void)primed;
    end_level_ = level_ = init;
    counter_ += 1;             // roll back the decrement in next_k
    stage_ = kNotStarted;
    shape_ = Hold;
    prev_gate_ = 0.f;
    released_ = false;
    residual_ = 0.;
    done_ = false;
    if (hold0) level_ = nodes[0].level;
  }

  float ctor_value() const { return static_cast<float>(level_); }
  float ProcessKr(float gate) {
    // check_gate
    if (prev_gate_ <= 0.f && gate > 0.f) {
      stage_ = -1; released_ = false; done_ = false;
      counter_ = 0; residual_ = 0.;
    } else if (gate <= -1.f && prev_gate_ > -1.f) {
      const double dur = -static_cast<double>(gate) - 1.;
      counter_ = std::max(1, static_cast<int32_t>(dur * sr_));
      stage_ = num_stages_ - 1;
      released_ = true;
      InitSegment(counter_, level_, dur);
      residual_ = 0.;
    } else if (prev_gate_ > 0.f && gate <= 0.f && release_node_ >= 0 && !released_) {
      counter_ = 0;
      stage_ = release_node_ - 1;
      released_ = true;
      residual_ = 0.;
    }
    prev_gate_ = gate;

    if (counter_ <= 0 && !NextSegment(counter_, level_)) return 0.f;
    float out = static_cast<float>(level_);
    Perform(out, level_);
    if (done_) level_ = end_level_;
    counter_ -= 1;
    return out;
  }

 private:
  double ScaleLevel(float l) const {
    return static_cast<double>(l) * ls_ + lb_;
  }
  void InitSegment(int& counter, double& level, double dur = -1.) {
    if (stage_ < 0 || stage_ >= num_stages_) { counter = 1; return; }
    const Node& n = nodes_[stage_];
    const double prev_end = end_level_;
    if (shape_ == Hold) level = prev_end;
    const double end = ScaleLevel(n.level);
    if (dur < 0) dur = static_cast<double>(n.dur) * ts_;
    shape_ = static_cast<Shape>(static_cast<int>(n.shape));
    const double curve = n.curve;
    end_level_ = end;
    const double stage_dur = dur * sr_ + residual_;
    const int32_t floor_v = static_cast<int32_t>(stage_dur);
    const double counter_frac = std::max(1.0, stage_dur);
    counter = std::max(1, floor_v);
    residual_ = stage_dur - counter;
    if (counter == 1) shape_ = Linear;

    switch (shape_) {
      case Step: level = end; break;
      case Hold: level = prev_end; break;
      case Linear: grow_ = (end - level) / counter_frac; break;
      case Exponential: grow_ = std::pow(end / level, 1.0 / counter_frac); break;
      case Sine: {
        const double w = M_PI / counter_frac;
        a2_ = (end + level) * 0.5;
        b1_ = 2. * std::cos(w);
        y1_ = (end - level) * 0.5;
        y2_ = y1_ * std::sin(M_PI * 0.5 - w);
        level = a2_ - y1_;
        break;
      }
      case Welch: {
        const double w = (M_PI * 0.5) / counter_frac;
        b1_ = 2. * std::cos(w);
        if (end >= level) {
          a2_ = level; y1_ = 0.; y2_ = -std::sin(w) * (end - level);
        } else {
          a2_ = end; y1_ = level - end; y2_ = std::cos(w) * (level - end);
        }
        level = a2_ + y1_;
        break;
      }
      case Curve:
        if (std::fabs(curve) < 0.001) {
          shape_ = Linear;
          grow_ = (end - level) / counter_frac;
        } else {
          const double a1 = (end - level) / (1.0 - std::exp(curve));
          a2_ = level + a1; b1_ = a1;
          grow_ = std::exp(curve / counter_frac);
        }
        break;
      case Squared:
        y1_ = std::sqrt(level); y2_ = std::sqrt(end);
        grow_ = (y2_ - y1_) / counter_frac;
        break;
      case Cubed:
        y1_ = std::pow(level, 1.0/3.0); y2_ = std::pow(end, 1.0/3.0);
        grow_ = (y2_ - y1_) / counter_frac;
        break;
      default: break;
    }
  }
  bool NextSegment(int& counter, double& level) {
    if (stage_ + 1 >= num_stages_) {
      counter = std::numeric_limits<int>::max();
      shape_ = Step;
      done_ = true;
    } else if (stage_ == kNotStarted) {
      counter = std::numeric_limits<int>::max();
      return true;
    } else if (stage_ + 1 == release_node_ && !released_) {
      if (loop_node_ >= 0 && loop_node_ < num_stages_) {
        stage_ = loop_node_;
        InitSegment(counter, level);
      } else {
        counter = std::numeric_limits<int>::max();
        shape_ = Sustain;
        level = end_level_;
      }
    } else {
      ++stage_;
      InitSegment(counter, level);
    }
    return true;
  }
  void Perform(float& out, double& level) {
    switch (shape_) {
      case Step: case Hold: case Sustain: out = static_cast<float>(level); break;
      case Linear: out = static_cast<float>(level); level += grow_; break;
      case Exponential: out = static_cast<float>(level); level *= grow_; break;
      case Sine: case Welch: {
        out = static_cast<float>(level);
        const double y0 = b1_ * y1_ - y2_;
        level = (shape_ == Sine) ? a2_ - y0 : a2_ + y0;
        y2_ = y1_; y1_ = y0;
        break;
      }
      case Curve:
        out = static_cast<float>(level);
        b1_ *= grow_;
        level = a2_ - b1_;
        break;
      case Squared: case Cubed:
        out = static_cast<float>(level);
        y1_ += grow_;
        level = y1_ * y1_ * ((shape_ == Cubed) ? y1_ : 1.);
        break;
      default: break;
    }
  }
  float sr_ = 48000.f;
  const Node* nodes_ = nullptr;
  int num_stages_ = 0, stage_ = kNotStarted, counter_ = 0;
  int release_node_ = -1, loop_node_ = -1;
  float ls_ = 1.f, lb_ = 0.f, ts_ = 1.f, prev_gate_ = 0.f;
  double level_ = 0., end_level_ = 0., residual_ = 0.;
  double grow_ = 0., a2_ = 0., b1_ = 0., y1_ = 0., y2_ = 0.;
  Shape shape_ = Hold;
  bool released_ = false, done_ = false;
};

// VarLag — Filter.sc:113-145. Classlib expansion:
//   start = start ? in; curve = Env.shapeNames[warp] ? warp;
//   e = Env([start, in], [time], warp); e[6] = curve; e[7] = curvature;
//   trig = Changed(in) + Impulse(0);  EnvGen(e, trig)
// (warp 1 is the VLag UGen fast path; this wrapper covers the EnvGen path.)
struct VarLag {
  void Init(double sr, float start, float time, float curvature, int warp) {
    EnvGen::Node n[1] = {{0.f, time, static_cast<float>(warp), curvature}};
    static_assert(sizeof(EnvGen::Node) == 16, "EnvGen Node layout");
    node_ = n[0];
    node_.level = 0.f;  // target level is set per block from `in`
    env_.Init(sr, &node_, 1, start);
    sr_ = sr;
    prev_in_ = start;
  }
  float ProcessKr(float in) {
    const bool changed = prev_in_ != in;
    prev_in_ = in;
    node_.level = in;
    const float gate = (first_ || changed) ? 1.f : 0.f;
    first_ = false;
    return env_.ProcessKr(gate);
  }
  double sr_ = 48000.;
  EnvGen::Node node_{};
  EnvGen env_;
  float prev_in_ = 0.f;
  bool first_ = true;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_ENVGEN_H_
