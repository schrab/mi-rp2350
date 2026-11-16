// lira.voice + prm.* verbatim port — one voice PAIR (2 osc paths).
// Sourced from LIRA-8 abs/lira.voice.pd + prm.*.pd (connect-list verified),
// take-1 math, PD-PRIMITIVES.md laws.
//
// Per osc path k: freq_k = T_k*(1 + 0.005*V)*(1 + M*mod)
//   T_k = line~(23.22ms) of mtof(note_k)*pitch (quantize optional),
//   note_k = hi_k + (tune_k/127)*(lo_k - hi_k)   [per-osc tune table]
//   pitch = (P/127)*1.99 + 0.01, quantize: mtof(round(ftom(Hz*pitch)))
//   V = cos(2*pi*3*t) * (0.5 + 3*pd_random(seed + pair_first)/1000) * vib
//   M = mod-source signal (other pair's bus, one-block delayed by the bus)
//   mod = (MOD/127)^4 * 2 [line~ 23.22ms], pwm = 0.53125 + 0.025*V
//   sharp = (S/127)^2 [line~], hold = (H/127)^2 [line~]
//   mix_k = sq_k*sharp + tri_k*(1 - sharp) + cos_k     (cos_k = sensor cos-morph)
//   gate_k = min(e_k^2, hold)                          (e = sensor line~ A/R)
//   out_k = mix_k * gate_k
// pair_out = out_A + out_B; bus_send = hip~3(pair_out)   [to s~ del-XY]
#ifndef RACK_SRC_LIRA8_LIRA_VOICE_H_
#define RACK_SRC_LIRA8_LIRA_VOICE_H_

#include <cstdint>
#include <cmath>

#include "pd_primitives.h"
#include "lira_osc.h"
#include "lira8_tc.h"

namespace l8pd {

// pd's [random] object LCG (x_arithmetic.c): state = state*472940017 + 1,
// result = (state * range) >> 32. Verified empirically against pd during
// golden phase (deterministic given seed).
inline int pd_random(uint32_t* state, int range) {
  *state = *state * 472940017u + 1u;
  return static_cast<int>((static_cast<uint64_t>(*state) *
                           static_cast<uint64_t>(range)) >> 32);
}

// Per-osc tune table (lo, hi MIDI notes) from prm.tune's text-seq rows.
static const float kTuneTable[8][2] = {
    {-16.f, 93.f},   {-16.f, 93.f},   {7.f, 109.f},    {9.f, 107.f},
    {20.f, 116.54f}, {20.f, 116.54f}, {33.f, 126.24f}, {33.f, 131.22f},
};

// pd default-PRNG draws of the four [random 1001] objects at patch load
// (probe: 744, 930, 728, 920 — seed machinery dormant, deterministic).
// These are the per-pair VIBRATO RATES in Hz: rate = 0.5 + 3*draw/1000
// (the random chain feeds the osc~'s frequency inlet, connect-list verified
// 2026-11-15; take-1's "depth" reading and this file's earlier "depth"
// misuse both corrected). V = cos(rate*t) * vib_param, V in [-1, +1].
static const int kVibDraws[4] = {744, 930, 728, 920};
static const float kVibRate[4] = {2.732f, 3.290f, 2.684f, 3.260f};

// Sensor envelope (prm.sensor): line~ A/R with e^2 gate + cos morph.
class SensorEnv {
 public:
  SensorEnv(float sr) : line_(sr) {}
  // pd's FAST toggle reaches the sensor's $0-trigger, which re-emits the
  // STORED target with the new A/R times. apply_params() calls this on every
  // CC, so only act when the mode actually changes — otherwise any knob turn
  // restarted the attack ramp of every held pad (bench-found: smooth turned
  // while holding = attack prolongs).
  void set_fast(bool fast) {
    if (fast == fast_) return;
    fast_ = fast;
    attack_ms_ = fast ? 100.0f : 200.0f;
    release_ms_ = fast ? 100.0f : 8000.0f;
    line_.set(on_ ? level_ : 0.0f, on_ ? attack_ms_ : release_ms_);
  }
  // level = the press peak (1.0 = pad hit / CC latch / PLAY button; MIDI
  // velocity / 127 for note presses). The patch's gate law squares it
  // (e^2), i.e. amplitude follows the hardware's touch curve; a re-press at
  // the SAME level stays a no-op (Ramp's zero-delta rule), so sequencer
  // re-triggers cannot restart the attack ramp.
  void set(bool on, float level = 1.0f) {
    if (on) {
      level_ = level < 0.0f ? 0.0f : (level > 1.0f ? 1.0f : level);
    }
    on_ = on;
    line_.set(on_ ? level_ : 0.0f, on_ ? attack_ms_ : release_ms_);
  }
  LIRA8_TC(lira_sensor_env)
  inline void process(float& e2, float& cos_term) {
    const float e = line_.next();
    e2 = e * e;
    // cos~ is pd's TABLE object (2048-entry cosine + linear interp), not a
    // libm call — the golden reference rendered through the same table. The
    // per-sample std::cos here was a libm call x2 per voice pair, one of the
    // voice stage's top costs on the M33.
    float ph = (e - 0.25f) * static_cast<float>(kCosTabSize);  // cycles
    if (ph < 0.0f) ph += static_cast<float>(kCosTabSize);
    cos_term = cos_lookup(ph) * 0.5f;
  }

  bool idle() const { return line_.idle(); }
  // Settled value without advancing (identical law to process()).
  LIRA8_TC(lira_sensor_values)
  inline void values(float& e2, float& cos_term) const {
    const float e = line_.value();
    e2 = e * e;
    float ph = (e - 0.25f) * static_cast<float>(kCosTabSize);
    if (ph < 0.0f) ph += static_cast<float>(kCosTabSize);
    cos_term = cos_lookup(ph) * 0.5f;
  }

 private:
  Ramp line_;
  float attack_ms_ = 100.0f, release_ms_ = 8000.0f;
  float level_ = 1.0f;     // press peak (MIDI velocity / 127)
  bool on_ = false;
  bool fast_ = false;
};

// prm.source cross-mod mux, evaluated INLINE at the point of use: M feeds
// only the pair's cross-mod terms, which are all zero in the patch's default
// state (mod CCs 0). The v1 shape precomputed a 64-sample signal per pair
// per block — a second pass over four bus arrays — even when nothing
// consumed it. Same expression and rounding order as that loop, so the
// mod != 0 result is unchanged.
struct ModSource {
  const float* a;
  const float* b;
  const float* lfo;
  const float* fb;
  float ga, gb, gl, gfb;      // source gates (pd's ==*0.999+0.001)
  float ga_out, gl_out;       // per-pair audio / LFO-CV output gates
  ModSource(const float* a_, const float* b_, const float* lfo_,
            const float* fb_, float ga_, float gb_, float gl_, float gfb_,
            float ga_out_, float gl_out_)
      : a(a_), b(b_), lfo(lfo_), fb(fb_),
        ga(ga_), gb(gb_), gl(gl_), gfb(gfb_),
        ga_out(ga_out_), gl_out(gl_out_) {}
  LIRA8_TC(lira_modsource_at)
  inline float at(int i) const {
    const float audio = a[i] * ga + b[i] * gb;
    const float lfo_cv = lfo[i] * gl + fb[i] * gfb;
    return audio * ga_out + lfo_cv * gl_out;   // v1 mux_source order
  }
};

class VoicePair {
 public:
  // pair 0..3 (voices 2i+1 / 2i+2). seed_base kept for API compat: the
  // patch's $0-seed receiver is DORMANT (nothing sends to it), so the
  // vibrato depths are the MEASURED pd default-PRNG draws (744/930/728/920
  // from `random 1001` at patch load, probed 2026-11-15) — deterministic.
  VoicePair(float sr, int pair, uint32_t seed_base)
      : sr_(sr),
        pair_(pair),
        osc_a_(sr),
        osc_b_(sr),
        sensor_a_(sr),
        sensor_b_(sr),
        tune_a_(sr),
        tune_b_(sr),
        sharp_(sr),
        mod_(sr),
        hold_(sr),
        vib_(sr),
        osc3_(sr),
        hip_(3.0f, sr) {
    (void)seed_base;
    osc3_.set_freq(kVibRate[pair]);
  }

  // ---- control-rate setters (values are raw 0..127 param bytes) ----
  void set_tune(int osc /*0|1*/, int v) {
    const float* row = kTuneTable[pair_ * 2 + osc];
    // LAW CORRECTED (empirical 2026-11-15): pd renders k=127 -> hi note
    // (1838 Hz = mtof(93)*pitch measured), k=0 -> lo (sub-audio). The spec
    // §7.2 / take-1 law (hi + k*(lo-hi)) was misread from the unpack order.
    const float note = row[0] + (v / 127.0f) * (row[1] - row[0]);
    if (osc == 0) tune_a_hz_ = mtof(note);
    else tune_b_hz_ = mtof(note);
  }
  void set_pitch(int v) {   // shared by the pair (prm.pitch)
    pitch_ = (v / 127.0f) * 1.99f + 0.01f;
  }
  void set_quantize(int v) { quantize_ = (v != 0); }
  // sharp/mod take the RAW float (pd's saved defaults are fractional:
// 14.377, 43.132, ...); the v1 (int) cast rounded the initial crossfade.
  void set_sharp(float v) { sharp_raw_ = (v / 127.0f); sharp_.set(sharp_raw_ * sharp_raw_, 23.22f); }
  void set_mod(float v) { mod_raw_ = (v / 127.0f); mod_.set(mod_raw_ * mod_raw_ * mod_raw_ * mod_raw_ * 2.0f, 23.22f); }
  void set_hold(int v) { hold_.set((v / 127.0f) * (v / 127.0f), 23.22f); }
  void set_vibrato(int v) { vib_.set(v / 127.0f, 23.22f); }
  void set_sensor(int osc, bool on, float level = 1.0f) {
    (osc == 0 ? sensor_a_ : sensor_b_).set(on, level);
  }
  void set_fast(bool fast) {
    sensor_a_.set_fast(fast);
    sensor_b_.set_fast(fast);
  }

  // Apply tune/pitch/quantize -> Hz (recomputed at control rate, feeds line~).
  // An oscillator under an ARP note override keeps the external target: its
  // knob-derived base stays stored in tune_*_hz_ and is re-applied by
  // clear_note_override() when the ARP lets go.
  void refresh_tune() {
    auto hz = [&](Ramp& line, float base_hz, int osc) {
      if (note_override_ & (1u << osc)) return;
      float h = base_hz * pitch_;
      if (quantize_) h = mtof(std::round(ftom(h)));
      line.set(h, 23.22f);
    };
    hz(tune_a_, tune_a_hz_, 0);
    hz(tune_b_, tune_b_hz_, 1);
  }

  // ARP-driven pitch: set this oscillator straight to a frequency, bypassing
  // the pitch multiplier and the quantize snap (the ARP's notes are already
  // exact scale notes; the default pitch multiplier would detune them).
  void set_note_hz(int osc, float hz) {
    note_override_ |= static_cast<uint8_t>(1u << osc);
    (osc == 0 ? tune_a_ : tune_b_).set(hz, 23.22f);
  }
  void clear_note_override(int osc) {
    if (!(note_override_ & (1u << osc))) return;
    note_override_ &= static_cast<uint8_t>(~(1u << osc));
    refresh_tune();   // restore the knob pitch for this oscillator
  }

  // ---- per-sample render; mux = fused prm.source cross-mod signal ----
  LIRA8_TC(lira_voice_pair)
  inline void process(const ModSource& mux, int n, float* pair_out,
                      float* bus_send) {
    // SETTLED-BLOCK FAST PATH: with the six control line~s and both sensor
    // envelopes idle (the normal drone / held-pad state) nothing in them can
    // change inside the block — hoist them and skip 6 Ramp::next calls plus
    // the two per-sample cos lookups. Same expressions and rounding order as
    // the slow loop below.
    const bool ctrl_idle = tune_a_.idle() && tune_b_.idle() &&
                           sharp_.idle() && mod_.idle() &&
                           hold_.idle() && vib_.idle();
    if (ctrl_idle && sensor_a_.idle() && sensor_b_.idle()) {
      const float sharp = sharp_.value();
      const float mod = mod_.value();
      const float hold = hold_.value();
      const float vib = vib_.value();
      const float tuneA = tune_a_.value();
      const float tuneB = tune_b_.value();
      float e2a, cosa, e2b, cosb;
      sensor_a_.values(e2a, cosa);
      sensor_b_.values(e2b, cosb);
      float gateA = e2a + hold;
      if (gateA > 1.0f) gateA = 1.0f;
      float gateB = e2b + hold;
      if (gateB > 1.0f) gateB = 1.0f;

      // CONSTANT-FREQUENCY PATH (vibrato == 0 && cross-mod == 0): fA/fB are
      // tuneA/tuneB for the whole block and pwm is exactly 0.53125. Hoist
      // each oscillator's dt + rlpf exp coefficient to one per block; skip
      // the vibrato cos oscillator (advance its phase so re-enabling vibrato
      // does not jump). Same expressions/mix/gate law as the loop below.
      if (vib == 0.0f && mod == 0.0f) {
        osc_a_.prepare(tuneA);
        osc_b_.prepare(tuneB);
        osc3_.skip(n);
        constexpr float kPwm = 0.53125f;
        for (int i = 0; i < n; ++i) {
          float sqA, triA, sqB, triB;
          osc_a_.process_const(kPwm, sqA, triA);
          osc_b_.process_const(kPwm, sqB, triB);
          const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
          const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
          const float pair = mixA * gateA + mixB * gateB;
          pair_out[i] = pair;
          bus_send[i] = hip_.process(pair);
        }
        return;
      }

      // PATCH DEFAULT PATH (cross-mod == 0, vibrato = raw 1/127): keep the
      // per-sample vibrato in dt, but the rlpf coefficient is block-constant
      // (prepare_coeff from the tune frequency) => no per-sample exp.
      if (mod == 0.0f) {
        osc_a_.prepare_coeff(tuneA);
        osc_b_.prepare_coeff(tuneB);
        osc3_.begin_block(n);   // block-rate vibrato cosine (exact endpoints)
        for (int i = 0; i < n; ++i) {
          const float V = osc3_.next_lerp() * vib;
          const float pwm = 0.53125f + 0.025f * V;
          const float fA = tuneA * (1.0f + 0.005f * V);
          const float fB = tuneB * (1.0f + 0.005f * V);
          float sqA, triA, sqB, triB;
          osc_a_.process_freq_prepared(fA, pwm, sqA, triA);
          osc_b_.process_freq_prepared(fB, pwm, sqB, triB);
          const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
          const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
          const float pair = mixA * gateA + mixB * gateB;
          pair_out[i] = pair;
          bus_send[i] = hip_.process(pair);
        }
        return;
      }

      osc3_.begin_block(n);
      for (int i = 0; i < n; ++i) {
        const float V = osc3_.next_lerp() * vib;
        const float pwm = 0.53125f + 0.025f * V;
        const float fA = (tuneA * (1.0f + 0.005f * V)) *
                         (1.0f + mux.at(i) * mod);
        const float fB = (tuneB * (1.0f + 0.005f * V)) *
                         (1.0f + mux.at(i) * mod);
        float sqA, triA, sqB, triB;
        osc_a_.process(fA, pwm, sqA, triA);
        osc_b_.process(fB, pwm, sqB, triB);
        const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
        const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
        const float pair = mixA * gateA + mixB * gateB;
        pair_out[i] = pair;
        bus_send[i] = hip_.process(pair);
      }
      return;
    }
    if (ctrl_idle) {
      // Controls settled, envelopes moving (attack / 8 s release): hoist the
      // six control values but keep the per-sample sensor law.
      const float sharp = sharp_.value();
      const float mod = mod_.value();
      const float hold = hold_.value();
      const float vib = vib_.value();
      const float tuneA = tune_a_.value();
      const float tuneB = tune_b_.value();

      // Same constant-frequency hoist as the settled path, but keep the
      // per-sample sensor envelope (attack / up-to-8 s release). Covers the
      // long release tails where the frequencies are still constant.
      if (vib == 0.0f && mod == 0.0f) {
        osc_a_.prepare(tuneA);
        osc_b_.prepare(tuneB);
        osc3_.skip(n);
        constexpr float kPwm = 0.53125f;
        for (int i = 0; i < n; ++i) {
          float e2a, cosa, e2b, cosb;
          sensor_a_.process(e2a, cosa);
          sensor_b_.process(e2b, cosb);
          float sqA, triA, sqB, triB;
          osc_a_.process_const(kPwm, sqA, triA);
          osc_b_.process_const(kPwm, sqB, triB);
          const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
          const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
          float gateA = e2a + hold;
          if (gateA > 1.0f) gateA = 1.0f;
          float gateB = e2b + hold;
          if (gateB > 1.0f) gateB = 1.0f;
          const float pair = mixA * gateA + mixB * gateB;
          pair_out[i] = pair;
          bus_send[i] = hip_.process(pair);
        }
        return;
      }

      // Cross-mod == 0 with moving sensors: same block-constant rlpf
      // coefficient, exact per-sample vibrato and sensor envelope.
      if (mod == 0.0f) {
        osc_a_.prepare_coeff(tuneA);
        osc_b_.prepare_coeff(tuneB);
        osc3_.begin_block(n);   // block-rate vibrato cosine (exact endpoints)
        for (int i = 0; i < n; ++i) {
          const float V = osc3_.next_lerp() * vib;
          const float pwm = 0.53125f + 0.025f * V;
          const float fA = tuneA * (1.0f + 0.005f * V);
          const float fB = tuneB * (1.0f + 0.005f * V);
          float e2a, cosa, e2b, cosb;
          sensor_a_.process(e2a, cosa);
          sensor_b_.process(e2b, cosb);
          float sqA, triA, sqB, triB;
          osc_a_.process_freq_prepared(fA, pwm, sqA, triA);
          osc_b_.process_freq_prepared(fB, pwm, sqB, triB);
          const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
          const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
          float gateA = e2a + hold;
          if (gateA > 1.0f) gateA = 1.0f;
          float gateB = e2b + hold;
          if (gateB > 1.0f) gateB = 1.0f;
          const float pair = mixA * gateA + mixB * gateB;
          pair_out[i] = pair;
          bus_send[i] = hip_.process(pair);
        }
        return;
      }

      for (int i = 0; i < n; ++i) {
        const float V = osc3_.next() * vib;
        const float pwm = 0.53125f + 0.025f * V;
        float e2a, cosa, e2b, cosb;
        sensor_a_.process(e2a, cosa);
        sensor_b_.process(e2b, cosb);
        const float fA = (tuneA * (1.0f + 0.005f * V)) *
                         (1.0f + mux.at(i) * mod);
        const float fB = (tuneB * (1.0f + 0.005f * V)) *
                         (1.0f + mux.at(i) * mod);
        float sqA, triA, sqB, triB;
        osc_a_.process(fA, pwm, sqA, triA);
        osc_b_.process(fB, pwm, sqB, triB);
        const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
        const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
        float gateA = e2a + hold;
        if (gateA > 1.0f) gateA = 1.0f;
        float gateB = e2b + hold;
        if (gateB > 1.0f) gateB = 1.0f;
        const float pair = mixA * gateA + mixB * gateB;
        pair_out[i] = pair;
        bus_send[i] = hip_.process(pair);
      }
      return;
    }
    osc3_.begin_block(n);
    for (int i = 0; i < n; ++i) {
      const float V = osc3_.next_lerp() * vib_.next();
      const float sharp = sharp_.next();
      const float mod = mod_.next();
      const float hold = hold_.next();
      const float pwm = 0.53125f + 0.025f * V;

      float e2a, cosa, e2b, cosb;
      sensor_a_.process(e2a, cosa);
      sensor_b_.process(e2b, cosb);

      const float fA = tune_a_.next() * (1.0f + 0.005f * V) *
                       (1.0f + mux.at(i) * mod);
      const float fB = tune_b_.next() * (1.0f + 0.005f * V) *
                       (1.0f + mux.at(i) * mod);

      float sqA, triA, sqB, triB;
      osc_a_.process(fA, pwm, sqA, triA);
      osc_b_.process(fB, pwm, sqB, triB);

      const float mixA = sqA * sharp + triA * (1.0f - sharp) + cosa;
      const float mixB = sqB * sharp + triB * (1.0f - sharp) + cosb;
      // Gate law (connect-list verified): e^2 AND hold SUM into the min~
      // inlet (pd signal inlets sum) -> gate = min(e^2 + hold, 1). hold=0:
      // normal envelope; hold=1: latch-on (the Lyra's hold button).
      float gateA = e2a + hold;
      if (gateA > 1.0f) gateA = 1.0f;
      float gateB = e2b + hold;
      if (gateB > 1.0f) gateB = 1.0f;
      const float outA = mixA * gateA;
      const float outB = mixB * gateB;

      const float pair = outA + outB;
      pair_out[i] = pair;
      bus_send[i] = hip_.process(pair);
    }
  }

  // Direct stimulus (host harness / tests): `sig` IS the mux output. Ga/gb/
  // lfo/fb gates are degenerate so ModSource::at() reduces to sig exactly
  // (x*1 + x*0; the lfo path is +0).
  inline void process(const float* sig, int n, float* pair_out,
                      float* bus_send) {
    process(ModSource(sig, sig, sig, sig, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f),
            n, pair_out, bus_send);
  }

 private:
  float sr_;
  int pair_ = 0;
  LiraOsc osc_a_, osc_b_;
  SensorEnv sensor_a_, sensor_b_;
  Ramp tune_a_, tune_b_;        // final smoothed Hz
  float tune_a_hz_ = 0.f, tune_b_hz_ = 0.f;
  float pitch_ = (64.f / 127.f) * 1.99f + 0.01f;
  bool quantize_ = false;
  Ramp sharp_, mod_, hold_, vib_;
  float sharp_raw_ = 0.f, mod_raw_ = 0.f;
  uint8_t note_override_ = 0;   // bit 0 = osc A, bit 1 = osc B (ARP pitch)
  CosOsc osc3_;
  Hip hip_;
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_VOICE_H_
