// benjolin_engine.h — reference-exact port of the \benjolis SynthDef
// (Engine_Benjolis.sc, Alejandro Olarte / Scott Cazan) on the PicoCollider
// kit. Host-testable: no MCU headers, no heap except the kit's Init()-mode
// delay rings (the device adapter passes ext_tap_rings instead).
// See docs/ASSESSMENT-benjolin-port-2026-11-22.md and the SDD ledger.
//
// Laws encoded here:
//  - 64-sample internal blocks = the scsynth grid (sc_defs.h kBlock); the
//    16-sample host/device grid bursts 4:1 through Process16.
//  - LocalIn/LocalOut 2-ch bus, 1-block latency (sc_kit_localio.h). ch0 =
//    midicps(rungler) trajectory [FM], ch1 = raw rungler DAC [LOOP].
//  - DEGENERATE COMPARATOR (review finding — do NOT "fix"): the SynthDef's
//    `sh8 = 1` var initializer is captured at graph build BEFORE the DelayN
//    assigns the register UGen, so the runtime comparator is the literal
//    form below: sh0 = 1 - ((1 > A) == (1 < A)), A = (osc1 > 0.5), which
//    folds to 1 - (osc1 > 0.5). The full-SynthDef golden encodes this form;
//    restoring the intended sh8-XOR would be a deliberate deviation.
//  - rungler.midicps: PER-SAMPLE ar law — the register taps stagger each
//    latch edge over 8 samples, so the DAC sum (and the bus ch0 content)
//    transitions per-sample. scsynth evaluates the FLOAT32 pow overload
//    (SC_InlineUnaryOp.h @ Version-3.14.1) per sample: sc::midicps_ar.
//  - Rate laws (T2/T3 rulings): RLPF/RHPF/SVF get ar freq -> per-sample n=1
//    calls (immediate recompute, no kr ramp); DFM1 samples its params at
//    BLOCK rate (TJUGens IN0) and ramps coefficients across the block ->
//    ONE SetParams(filt_freq[0], ...) + ONE n=64 ProcessBlock.
//  - Filter gain: RLPF/RHPF mul and SVF mul are applied HERE per block
//    (kit filter ports are UGen-only); DFM1's inputgain goes through
//    SetParams (never re-Init — that re-seeds the rnor stream).
//  - DPW freq clamp: SawDPW divides by freq; SC would produce inf/NaN at
//    freq <= 0. Engine clamps to >= 1 Hz (documented device-safety
//    deviation); LFTri is unclamped (no division in its law).
//  - Mode kHardware: the hardware-faithful alternative (§5.3 option B) —
//    an explicit 8-bit register shifted on each osc2 rising edge, DAC and
//    midicps applied immediately. Scenario-gated only (T4); no golden.
#ifndef RACK_SRC_BENJOLIN_BENJOLIN_ENGINE_H_
#define RACK_SRC_BENJOLIN_BENJOLIN_ENGINE_H_

#include <cmath>
#include <cstdint>

#include "sc_kit_dfm1.h"
#include "sc_kit_delay.h"
#include "sc_kit_filter.h"
#include "sc_kit_lfo.h"
#include "sc_kit_localio.h"
#include "sc_kit_nova.h"
#include "sc_kit_osc.h"
#include "sc_kit_osc2.h"
#include "sc_kit_sc3p_filter.h"
#include "sc_kit_trig.h"

// Hot-code section placement (the SRAM law, PORTING-HIGHLIGHTS §3): the
// per-sample DSP must run from SRAM. The macro is empty on the host (the
// tests stay plain g++); on the firmware it places the out-of-line emission
// of these functions into the .tc_benjolin overlay (memmap_rack.ld).
#if defined(RACK_RP2350_TC)
#define BENJOLIN_ENGINE_TC(token) __attribute__((section(".tc_benjolin." #token)))
#else
#define BENJOLIN_ENGINE_TC(token)
#endif

namespace benjolin {

// SynthDef arg defaults, verbatim from Engine_Benjolis.sc:26 (amp=0 silence;
// the caller SetParams before the first render). `pan` is the SC arg — the
// norns command multiplies the UI value by 0.001; the adapter maps CC to
// SC units.
struct Params {
  float freq1 = 40.0f, freq2 = 4.0f, scale = 1.0f;
  float rungler1 = 0.16f, rungler2 = 0.0f, rungler_f = 9.0f;
  float loop = 0.0f, filt_freq = 40.0f, q = 0.82f, gain = 1.0f;
  float filter_type = 0.0f, out_signal = 6.0f, amp = 0.0f, pan = 0.0f;
};

enum class FeedbackMode { kReferenceSc = 0, kHardware = 1 };

// Benjolin 3-bit DAC weights from the delay-line register taps
// (sh1/256 + sh2/128 + ... + sh8/2, max 255/256 = 0.99609375). All powers
// of two, so multiply-by-reciprocal is bit-identical to SC's `x / 2.pow(k)`.
inline float RunglerDac(const float* sh /* sh1..sh8 */) {
  return sh[0] * (1.0f / 256.0f) + sh[1] * (1.0f / 128.0f) +
         sh[2] * (1.0f / 64.0f) + sh[3] * (1.0f / 32.0f) +
         sh[4] * (1.0f / 16.0f) + sh[5] * (1.0f / 8.0f) +
         sh[6] * (1.0f / 4.0f) + sh[7] * 0.5f;
}

// Mode-B explicit register: bit k (0-based) = stage k+1; bit 0 = sh1.
inline float RunglerDacBits(uint32_t reg) {
  float s = 0.0f;
  for (int k = 0; k < 8; ++k)
    if ((reg >> k) & 1u) s += (1.0f / 256.0f) * static_cast<float>(1u << k);
  return s;
}

inline float DpwClamp(float f) { return f < 1.0f ? 1.0f : f; }

struct Engine {
  static constexpr int kBlock = 64;  // the scsynth block grid

  void Init(double sr, FeedbackMode mode, int dfm1_epoch, const Params& p,
            float* ext_tap_rings = nullptr, float* ext_pan_rings = nullptr) {
    sr_ = static_cast<float>(sr);
    mode_ = mode;
    p_ = p;
    pending_ = p;
    pending_set_ = false;
    block_ = 0;
    stage_ = 0;
    bus_.Init(2);
    // LFTri 3.14 ctor primes then rolls back -> first rendered sample at
    // phase 0 (the kit encodes the rollback).
    tri1_.Init(sr, p_.freq1);
    tri2_.Init(sr, p_.freq2);
    // PulseDPW default width 0.5 (the SynthDef passes no width arg).
    osc1_.InitAr(sr, DpwClamp(p_.freq1), 0.5f);
    osc2_.InitAr(sr, DpwClamp(p_.freq2), 0.5f);
    // 8 register taps: DelayN.ar(in, 0.01, sr*k), k = 1..8. Constant delay.
    for (int k = 0; k < 8; ++k) {
      // fl32(fl32(1/sr) * k) — the server's DelayN constants round the same
      // way, giving effective taps [1,2,2,4,5,5,6,8] (verified vs scsynth).
      tap_delay_[k] = (1.0f / 48000.0f) * static_cast<float>(k + 1);
      if (ext_tap_rings != nullptr)
        tap_[k].InitExt(ext_tap_rings + k * 16, 16, sr, 0.01f, tap_delay_[k],
                        0.f, 0);
      else
        tap_[k].Init(sr, 0.01f, tap_delay_[k], 0.f, 0);
    }
    // Ctor primes (§4.7): ARBITRATED BY THE T4 GOLDEN — the golden showed an
    // ~8x head-ratio mismatch WITH the n=1 prime and none WITHOUT, so the
    // core RLPF/RHPF ctors do NOT consume a sample the way the Hecker
    // Select/XFade2-expanded chain did. The kit's NaN-freq ctor trick alone
    // reproduces scsynth's startup.
    rlpf_.Init(sr, 3);
    rhpf_.Init(sr, 4);
    svf_.Init(sr, p_.filt_freq, p_.q, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f);
    // Benjolin's DFM1: inputgain=gain, TYPE=1 (high-pass), noiselevel omitted
    // -> upstream default 0.0003 (assessment §4.3 correction). NO ctor prime:
    // the pinned TJUGens wrapper has a hand-written ctor that does not consume
    // a sample, and the T2 renderer transient is not modelable.
    dfm1_.Init(sr, dfm1_epoch, p_.filt_freq, p_.q, p_.gain, 1.0f, 0.0003f);
    // Lag.kr emulation (kit law: pass sr/64, call once per 64-block).
    lag_l_.Init(sr / 64.0);
    lag_r_.Init(sr / 64.0);
    // DelayL: maxdelaytime 0.01 s; upstream clamps delaytime to m_maxdelay
    // (CalcDelay) — done engine-side below. Device path: caller-owned rings
    // (1024 floats each, no heap).
    if (ext_pan_rings != nullptr) {
      dl_l_.InitExt(ext_pan_rings, 1024, sr, 0.01f, 0.0f, 0.f, 1);
      dl_r_.InitExt(ext_pan_rings + 1024, 1024, sr, 0.01f, 0.0f, 0.f, 1);
    } else {
      dl_l_.Init(sr, 0.01f, 0.0f, 0.f, 1);
      dl_r_.Init(sr, 0.01f, 0.0f, 0.f, 1);
    }
    leak_l_.Init();
    leak_l_.Prime(0.f);  // the pan chain's ctor value is 0.0
    leak_r_.Init();
    leak_r_.Prime(0.f);
    rungler_hz_ = 0.0f;
    rungler_raw_ = 0.0f;
    buf_ = 0.0f;
    o2_prev_ = 0.0f;
    reg_ = 0;
  }

  void SetParams(const Params& p) { pending_ = p; pending_set_ = true; }

  // RLPF/RHPF rq = 1 - q, upstream's qres floor is 0.001.
  float Rq() const {
    const float rq = p_.q * -1.0f + 1.0f;
    return rq < 0.001f ? 0.001f : rq;
  }

  // One 16-sample host/device block; bursts 4:1 into the 64-grid.
  BENJOLIN_ENGINE_TC(process16)
  void Process16(float* l, float* r) {
    if (stage_ == 0) Process64(stage_l_, stage_r_);
    const int off = stage_ * 16;
    for (int i = 0; i < 16; ++i) {
      l[i] = stage_l_[off + i];
      r[i] = stage_r_[off + i];
    }
    stage_ = (stage_ + 1) & 3;
  }

  uint32_t reg() const { return reg_; }  // T4 mode-B scenario accessor

  BENJOLIN_ENGINE_TC(process64)
  void Process64(float* out_l, float* out_r) {
    if (pending_set_) {
      p_ = pending_;
      pending_set_ = false;
      raw_prev_ = -1.0f;   // the midicps memo: scale may have changed
    }
    float* inr[2] = {in0_, in1_};
    if (mode_ == FeedbackMode::kReferenceSc) {
      lin_.ProcessBlock(bus_, inr, kDefaults(), block_, kBlock);
    } else {
      for (int i = 0; i < kBlock; ++i) {
        in0_[i] = rungler_hz_;
        in1_[i] = buf_;
      }
    }
    const float loop_a = p_.loop, loop_b = p_.loop * -1.0f + 1.0f;

    // Phase 1 — per-sample sources. FM freqs come from the bus trajectory
    // (mode A) or the direct state (mode B); the register taps stagger each
    // latch edge over 8 samples, so everything here is per-sample.
    for (int i = 0; i < kBlock; ++i) {
      const float fm1 = in0_[i] * p_.rungler1 + p_.freq1;
      const float fm2 = in0_[i] * p_.rungler2 + p_.freq2;
      tri1_.SetFreq(fm1);
      tri2_.SetFreq(fm2);
      osc1_.SetFreq(DpwClamp(fm1));
      osc2_.SetFreq(DpwClamp(fm2));
      tri_buf_[i] = tri1_.Process();
      tri2_buf_[i] = tri2_.Process();
      o1_buf_[i] = osc1_.Next();
      o2_buf_[i] = osc2_.Next();
      pwm_buf_[i] = ((tri_buf_[i] + tri2_buf_[i]) > 0.0f) ? 1.0f : 0.0f;
      osc1m_buf_[i] = in1_[i] * loop_a + o1_buf_[i] * loop_b;
      // DEGENERATE comparator — literal graph form (folds to 1-(x > 0.5));
      // kept literal so the engine reads exactly like the shipped SynthDef.
      const float cmp = (osc1m_buf_[i] > 0.5f) ? 1.0f : 0.0f;
      const float gt = (1.0f > cmp) ? 1.0f : 0.0f;
      const float lt = (1.0f < cmp) ? 1.0f : 0.0f;
      const float sh0 = 1.0f - ((gt == lt) ? 1.0f : 0.0f);
      sh0_buf_[i] = sh0;
      // 8-stage register: Latch(prev stage, osc2) -> DelayN(sr*k).
      float latched = sh0;
      for (int k = 0; k < 8; ++k) {
        latched = latch_[k].Next(latched, o2_buf_[i]);
        tap_[k].ProcessBlock(&latched, &sh_buf_[k][i], 1, tap_delay_[k], 0.f);
      }
      float sh[8];
      for (int k = 0; k < 8; ++k) sh[k] = sh_buf_[k][i];
      raw_buf_[i] = RunglerDac(sh);
      // midicps memo: the DAC sum is constant except during the ~8-sample
      // post-edge transitions — powf only when the input changes (the same
      // float input gives the same float output; bit-identical stream).
      if (raw_buf_[i] != raw_prev_) {
        raw_prev_ = raw_buf_[i];
        hz_prev_ = sc::midicps_ar(raw_prev_ * 127.0f * p_.scale);
      }
      hz_buf_[i] = hz_prev_;
      // The graph's filter freq uses the CURRENT-BLOCK midicps'd DAC
      // trajectory (rungler AFTER the midicps line), not the bus read —
      // the bus read feeds only the oscillator FMs. (T4 golden finding.)
      filt_freq_buf_[i] = hz_buf_[i] * p_.rungler_f + p_.filt_freq;
      // Mode B: explicit 8-bit register shifted on the latch edge; DAC and
      // midicps applied IMMEDIATELY (hardware law — no bus, no block delay).
      if (mode_ == FeedbackMode::kHardware && o2_buf_[i] > 0.0f &&
          o2_prev_ <= 0.0f) {
        reg_ = ((reg_ << 1) | (static_cast<uint32_t>(sh0) & 1u)) & 0xffu;
        rungler_raw_ = RunglerDacBits(reg_);
        buf_ = rungler_raw_;
        rungler_hz_ = sc::midicps_ar(buf_ * 127.0f * p_.scale);
        for (int j = i + 1; j < kBlock; ++j) {
          in0_[j] = rungler_hz_;
          in1_[j] = buf_;
        }
      }
      o2_prev_ = o2_buf_[i];
    }

    // Phase 2 — selected filter branch (all primed at Init; running only the
    // selected one is sonically exact — SC's Select discards the rest).
    const int type = sc::SelectIndex(p_.filter_type, 4);
    switch (type) {
      case 0:
        // SOURCE-ACCURATE LAW: core RLPF/RHPF read the freq wire's FIRST
        // sample per block (ZIN0(1)) and ramp coefficients across the block
        // (mFilterSlope) — there is no per-sample next_a for the core
        // filters (T4 review I1; the earlier StepA cited a nonexistent law
        // and is removed from the kit).
        rlpf_.ProcessBlock(pwm_buf_, filt_buf_, kBlock, filt_freq_buf_[0],
                           Rq());
        for (int i = 0; i < kBlock; ++i) filt_buf_[i] *= p_.gain;
        break;
      case 1:
        rhpf_.ProcessBlock(pwm_buf_, filt_buf_, kBlock, filt_freq_buf_[0],
                           Rq());
        for (int i = 0; i < kBlock; ++i) filt_buf_[i] *= p_.gain;
        break;
      case 2:
        // gain = SVF mul (class sig: mul is the 9th arg, outside the UGen).
        for (int i = 0; i < kBlock; ++i) {
          svf_.ProcessBlock(&pwm_buf_[i], &filt_buf_[i], 1, filt_freq_buf_[i],
                            p_.q, 1.f, 0.f, 0.f, 0.f, 0.f);
          filt_buf_[i] *= p_.gain;
        }
        break;
      case 3:
        // DFM1: block-rate params + ONE n=64 call (coefficient ramp law).
        dfm1_.SetParams(filt_freq_buf_[0], p_.q, p_.gain);
        dfm1_.ProcessBlock(pwm_buf_, filt_buf_, kBlock);
        break;
    }

    // Phase 3 — outSignal select (7 taps) + tanh soft clip.
    for (int i = 0; i < kBlock; ++i) {
      const float* taps[7] = {&tri_buf_[i], &osc1m_buf_[i], &tri2_buf_[i],
                              &o2_buf_[i],  &pwm_buf_[i],  &sh0_buf_[i],
                              &filt_buf_[i]};
      const float shaped =
          sc::NovaTanh(*taps[sc::SelectIndex(p_.out_signal, 7)] * p_.amp);
      shaped_l_[i] = shaped;
      shaped_r_[i] = shaped;
    }

    // Phase 4 — feedback write + stereo pan chain + DC block.
    if (mode_ == FeedbackMode::kReferenceSc) {
      float* obr[2] = {hz_buf_, raw_buf_};
      lout_.ProcessBlock(bus_, obr, block_, kBlock);
    } else {
      buf_ = rungler_raw_;  // mode B FM/DAC carry to the next block
    }
    const float dly_l =
        sc::ClipTo(lag_l_.Next(sc::ClipTo(p_.pan, 0.0f, 1.0f), 0.1f), 0.0f,
                   0.01f);  // upstream CalcDelay m_maxdelay clamp
    const float dly_r =
        sc::ClipTo(lag_r_.Next(sc::ClipTo(p_.pan * -1.0f, 0.0f, 1.0f), 0.1f),
                   0.0f, 0.01f);
    dl_l_.ProcessBlock(shaped_l_, out_l, kBlock, dly_l, 0.f);
    dl_r_.ProcessBlock(shaped_r_, out_r, kBlock, dly_r, 0.f);
    for (int i = 0; i < kBlock; ++i) {
      out_l[i] = leak_l_.Next(out_l[i], 0.995f);
      out_r[i] = leak_r_.Next(out_r[i], 0.995f);
    }
    ++block_;
  }

 private:
  static const float* kDefaults() {
    static const float d[2] = {0.f, 0.f};
    return d;
  }

  float sr_;
  FeedbackMode mode_;
  Params p_, pending_;
  bool pending_set_ = false;
  int block_ = 0, stage_ = 0;
  sc::LocalBus bus_;
  sc::LocalIn lin_;
  sc::LocalOut lout_;
  sc::LFTri tri1_, tri2_;
  sc::PulseDPW osc1_, osc2_;
  sc::Latch latch_[8];
  sc::DelayLine tap_[8];
  float tap_delay_[8];
  sc::Filter2Pole rlpf_, rhpf_;
  sc::SVF svf_;
  sc::Dfm1 dfm1_;
  sc::Lag lag_l_, lag_r_;
  sc::DelayLine dl_l_, dl_r_;
  sc::LeakDC leak_l_, leak_r_;
  float rungler_hz_, rungler_raw_, buf_, o2_prev_ = 0.0f;
  float raw_prev_ = -1.0f, hz_prev_ = 0.0f;   // the midicps memo
  uint32_t reg_ = 0;
  float in0_[kBlock], in1_[kBlock];
  float tri_buf_[kBlock], tri2_buf_[kBlock], o1_buf_[kBlock], o2_buf_[kBlock];
  float pwm_buf_[kBlock], osc1m_buf_[kBlock], sh0_buf_[kBlock];
  float sh_buf_[8][kBlock];
  float raw_buf_[kBlock], hz_buf_[kBlock], filt_freq_buf_[kBlock];
  float filt_buf_[kBlock], shaped_l_[kBlock], shaped_r_[kBlock];
  float stage_l_[kBlock], stage_r_[kBlock];
};

}  // namespace benjolin
#endif  // RACK_SRC_BENJOLIN_BENJOLIN_ENGINE_H_
