// dronecaster_engine.h — the DroneCaster_SynthSocket port (GPL-3.0 upstream:
// northern-information/dronecaster engine/Dronecaster_SynthSocket.sc).
// Mono voice; "we don't want xfades, we want down+up": SetDrone releases the
// current synth (gate->0); when its Env.asr finishes, the cued synth starts.
// hz glides via a one-pole per-block slew (~60 ms to 95% — the approved
// spec's "short lag" deviation from SC's stepped control bus).
//
// Rate law: the host calls Process(n <= 64) on the 16-sample grid; a fresh
// internal 64-sample block is rendered whenever the buffer drains (lira8 law).
// EnvGen.kr semantics: ONE env value per internal block.
#ifndef RACK_SRC_DRONECASTER_DRONECASTER_ENGINE_H_
#define RACK_SRC_DRONECASTER_DRONECASTER_ENGINE_H_
#include <cstddef>
#include <cstdint>
#include <cstring>
#include "sc_defs.h"
#include "sc_rgen.h"
#include "sc_arena.h"
#include "sc_kit_osc.h"
#include "sc_kit_pulse.h"
#include "sc_kit_env.h"

namespace sc {

struct Ctx {
  float sr = 48000.0f;
  RGen* rgen = nullptr;
  float hz = 55.0f, amp = 0.4f;
  KitTables* tables = nullptr;
  Arena* arena = nullptr;
  void* state = nullptr;     // per-voice DSP state, arena-owned
};

struct DroneSpec {
  const char* name;
  void (*init)(Ctx&);
  void (*process)(Ctx&, float*, float*, int);
};

constexpr int kCatalogCount = 37;

class Engine {
 public:
  void Init(float sr, uint8_t* buf_mem, size_t buf_bytes) {
    sr_ = sr;
    arena_.Init(buf_mem, buf_bytes);
    rgen_.init(kGoldenSeed);                 // matches RandSeed.kr(1, seed)
    kit_ = static_cast<KitTables*>(arena_.Alloc(sizeof(KitTables)));
    if (kit_ != nullptr) InitKitTables(kit_); // capacity is static_asserted
    tables_mark_ = arena_.Mark();            // kit tables survive voice swaps
    ctx_.sr = sr;
    ctx_.rgen = &rgen_;
    ctx_.tables = kit_;
    ctx_.arena = &arena_;
    ctx_.hz = 55.0f;
    ctx_.amp = 0.4f;
    hz_cur_ = hz_target_ = 55.0f;
    fade_ = 1.0f;
    env_.Init(sr, fade_, fade_);
    phase_ = Phase::kIdle;
    current_ = nullptr;
    cued_ = nullptr;
    internal_pos_ = 0;
    swaps_ = 0;
  }

  // down+up: release now; the cued voice starts when the env ends.
  // SetDrone(nullptr) = gate-off with no cue (the golden's gate path).
  void SetDrone(const DroneSpec* spec) {
    if (spec == nullptr) {
      if (phase_ == Phase::kPlaying) {
        env_.Gate(false);
        phase_ = Phase::kReleasing;
        cued_ = nullptr;
      }
      return;
    }
    if (spec == current_ && phase_ != Phase::kIdle) return;  // already selected
    cued_ = spec;
    if (phase_ == Phase::kPlaying) {
      env_.Gate(false);
      phase_ = Phase::kReleasing;
    } else if (phase_ == Phase::kIdle) {
      StartCued();
    }
  }
  void Recast() {                          // PLAY button: release + restart
    if (phase_ == Phase::kPlaying) {
      cued_ = current_;
      env_.Gate(false);
      phase_ = Phase::kReleasing;
    }
  }

  void set_hz_target(float hz) { hz_target_ = hz; }
  void set_amp(float a) { ctx_.amp = a; }
  void set_fade(float t) {
    fade_ = t > 0.001f ? t : 0.001f;
    env_.set_times(fade_, fade_);          // SC setFadeTime: args only
  }
  const char* current_name() const { return current_ ? current_->name : "--"; }
  uint32_t swap_count() const { return swaps_; }

  void Process(float* out_l, float* out_r, int n) {
    for (int i = 0; i < n; ++i) {
      if (internal_pos_ == 0) RenderInternal();
      out_l[i] = buf_l_[internal_pos_];
      out_r[i] = buf_r_[internal_pos_];
      internal_pos_ = (internal_pos_ + 1) & (kBlock - 1);
    }
  }

 private:
  enum class Phase { kIdle, kPlaying, kReleasing };

  void StartCued() {
    if (cued_ == nullptr) {
      current_ = nullptr;
      phase_ = Phase::kIdle;
      return;
    }
    arena_.ResetTo(tables_mark_);          // frees the old voice, keeps tables
    ctx_.state = nullptr;
    ctx_.hz = hz_cur_;
    cued_->init(ctx_);
    env_.Init(sr_, fade_, fade_);
    env_.Gate(true);
    // Golden-block-0 state: v(-1) = v(0) = 0. The synth's first internal
    // block is multiplied by the ramp 0 -> v(0) = 0 (silent), the ramp
    // v(0) -> v(1) starts on the next block.
    env_prev_ = 0.0f;
    env_cur_ = 0.0f;
    env_done_ = false;
    current_ = cued_;
    cued_ = nullptr;
    phase_ = Phase::kPlaying;
    ++swaps_;
  }

  void RenderInternal() {
    // hz slew: one-pole per internal block (~60 ms to 95%).
    hz_cur_ += (hz_target_ - hz_cur_) * (64.0f / (0.02f * sr_));
    ctx_.hz = hz_cur_;
    if (phase_ == Phase::kReleasing && env_done_) StartCued();
    if (phase_ == Phase::kPlaying || phase_ == Phase::kReleasing) {
      current_->process(ctx_, buf_l_, buf_r_, kBlock);
      // EnvGen.kr x ar multiply law (scsynth BinaryOp ak-loop, CALCSLOPE):
      // block k is multiplied by the LINEAR RAMP from v(k-1) to v(k), where
      // v(k) is the k-th EnvGen value — verified sample-exact against the
      // scsynth goldens. Gate/swap intents are applied by the caller BEFORE
      // Process(), so the Next() below sees them on the right block.
      env_prev_ = env_cur_;
      env_cur_ = env_.Next();
      env_done_ = env_.done();
      const float slope =
          static_cast<float>((env_cur_ - env_prev_) * (1.0 / 64.0));
      float e = env_prev_;
      for (uint32_t i = 0; i < kBlock; ++i) {
        buf_l_[i] *= e;
        buf_r_[i] *= e;
        e += slope;
      }
    } else {
      for (uint32_t i = 0; i < kBlock; ++i) { buf_l_[i] = 0.0f; buf_r_[i] = 0.0f; }
    }
  }

  float sr_ = 48000.0f;
  float buf_l_[64], buf_r_[64];
  uint32_t internal_pos_ = 0;
  Arena arena_;
  KitTables* kit_ = nullptr;
  size_t tables_mark_ = 0;
  RGen rgen_;
  Ctx ctx_;
  EnvAsr env_;
  float env_prev_ = 0.0f, env_cur_ = 0.0f;
  bool env_done_ = false;
  Phase phase_ = Phase::kIdle;
  const DroneSpec* current_ = nullptr;
  const DroneSpec* cued_ = nullptr;
  float hz_cur_ = 55.0f, hz_target_ = 55.0f, fade_ = 1.0f;
  uint32_t swaps_ = 0;
};

}  // namespace sc
#endif
