// sc_kit_env.h — EnvGen subset: Env.asr with SC's default curve -4 at
// CONTROL rate (EnvGen.kr computes ONE value per 64-sample block; the graph
// multiplies the whole block by it).
//
// Source: server/plugins/LFUGens.cpp (Version-3.14.1): EnvGen_Ctor 2413,
// EnvGen_initSegment 2470, check_gate 2564, EnvGen_nextSegment 2607,
// EnvGen_perform 2638, EnvGen_next_k 2755; SCClassLibrary/Common/Audio/Env.sc
// (*asr, curve -4.0).
//
// 3.14.1 delta vs the 3.13.1 rev-2 plan: the ctor primes one control sample
// and then RESTORES INITIAL CONDITIONS (m_prevGate = 0, m_stage =
// ENVGEN_NOT_STARTED, counter rolled back) — so the gate transition is seen
// by the FIRST RENDERED call, which outputs the attack's step-0 value (0.0)
// and initializes the attack segment. Under 3.13.1 the priming call already
// consumed that step, shifting every subsequent value one block earlier.
// The segment law itself is unchanged for integer-duration segments
// (m_stageResidual == 0; counter_fractional == counter).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_ENV_H_
#define RACK_SRC_DRONECASTER_SC_KIT_ENV_H_
#include <cmath>
#include <cstdint>

namespace sc {

class EnvAsr {
 public:
  // sr = the AUDIO sample rate; the kr rate (one Next() per 64-sample
  // block) is sr/64, like scsynth's calc_BufRate.
  void Init(float sr, float attack, float release) {
    sr_kr_ = static_cast<double>(sr) / 64.0;
    attack_ = attack;
    release_ = release;
    level_ = 0.0;                 // initLevel (levels[0] of Env.asr)
    end_level_ = 0.0;
    a2_ = b1_ = 0.0;
    grow_ = 1.0;
    counter_ = 0;
    stage_ = kNotStarted;         // ENVGEN_NOT_STARTED
    shape_ = kHold;               // ctor: m_shape = shape_Hold
    prev_gate_ = 0.0f;
    released_ = false;
    done_ = false;
    stage_residual_ = 0.0;
  }
  // Just sets the gate input (the ctor-priming rollback means the NEXT
  // Next() performs the check_gate transition itself).
  void Gate(bool on) { gate_ = on; }
  // setFadeTime: sets the attack/release ARGS. SC initializes a segment when
  // it begins, so live segments keep their rates; the next segment uses the
  // new value (the release is initialized at gate-off).
  void set_times(float attack, float release) {
    attack_ = attack;
    release_ = release;
  }
  // One CONTROL sample (call once per 64-sample block). Mirrors
  // EnvGen_next_k: check_gate -> nextSegment (when counter <= 0) -> perform.
  float Next() {
    const float gate = gate_ ? 1.0f : 0.0f;
    // check_gate (numstages=2, releaseNode=1 >= 0)
    if (prev_gate_ <= 0.0f && gate > 0.0f) {
      stage_ = -1;                // retrigger: stage -1 -> nextSegment inits 0
      released_ = false;
      done_ = false;
      counter_ = 0;
      stage_residual_ = 0.0;
    } else if (prev_gate_ > 0.0f && gate <= 0.0f && !released_) {
      counter_ = 0;
      stage_ = 0;                 // releaseNode - 1
      released_ = true;
      stage_residual_ = 0.0;
    }
    prev_gate_ = gate;

    if (counter_ <= 0) NextSegment();

    // EnvGen_perform (one sample)
    float out;
    switch (shape_) {
      case kCurve:
        out = static_cast<float>(level_);
        b1_ *= grow_;
        level_ = a2_ - b1_;
        break;
      case kLinear:
        out = static_cast<float>(level_);
        level_ += grow_;         // grow = (end-level)/counter_fractional
        break;
      default:                    // Hold / Step / Sustain
        out = static_cast<float>(level_);
        break;
    }
    if (done_) level_ = end_level_;
    counter_ -= 1;
    return out;
  }
  bool done() const { return done_; }

 private:
  enum Shape { kStep = 0, kHold = 1, kLinear = 2, kSustain = 3, kCurve = 5 };
  static constexpr int kNotStarted = 1000000000;  // ENVGEN_NOT_STARTED

  // EnvGen_nextSegment specialized to Env.asr (numstages=2, releaseNode=1,
  // loopNode=-1).
  void NextSegment() {
    if (stage_ + 1 >= 2) {        // last stage completed
      counter_ = 2147483647;      // INT_MAX
      shape_ = kStep;
      done_ = true;
    } else if (stage_ == kNotStarted) {
      counter_ = 2147483647;
    } else if (stage_ + 1 == 1 && !released_) {   // sustain stage
      counter_ = 2147483647;
      shape_ = kSustain;
      level_ = end_level_;
    } else {
      stage_++;
      InitSegment();
    }
  }

  // EnvGen_initSegment for stage 0 (attack -> 1.0) and stage 1 (release -> 0),
  // shape_Curve (Env.asr default curve -4.0).
  void InitSegment() {
    const double endLevel = (stage_ == 0) ? 1.0 : 0.0;
    const double dur = static_cast<double>((stage_ == 0) ? attack_ : release_);
    shape_ = kCurve;
    end_level_ = endLevel;
    // Carry the residual stage duration forward to the next stage.
    const double stage_dur = dur * sr_kr_ + stage_residual_;
    const int32_t floor_ = static_cast<int32_t>(stage_dur);
    const double counter_fractional = stage_dur < 1.0 ? 1.0 : stage_dur;
    counter_ = floor_ < 1 ? 1 : floor_;
    stage_residual_ = stage_dur - counter_;
    if (counter_ == 1) {                 // source: counter==1 -> shape_Linear
      shape_ = kLinear;
      grow_ = (endLevel - level_) / counter_fractional;
      return;
    }
    const double curve = -4.0;
    const double a1 = (endLevel - level_) / (1.0 - std::exp(curve));
    a2_ = level_ + a1;
    b1_ = a1;
    grow_ = std::exp(curve / counter_fractional);
  }

  double sr_kr_ = 750.0;
  float attack_ = 1.0f, release_ = 1.0f;
  double a2_ = 0.0, b1_ = 0.0, grow_ = 1.0, level_ = 0.0, end_level_ = 0.0;
  double stage_residual_ = 0.0;
  int32_t counter_ = 0;
  int stage_ = kNotStarted;
  Shape shape_ = kHold;
  float prev_gate_ = 0.0f;
  bool gate_ = false, released_ = false, done_ = false;
};

}  // namespace sc
#endif
