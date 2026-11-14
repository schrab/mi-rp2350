// Lira8Engine — the whole LIRA-8 signal flow at 32 kHz (32-first; the engine
// is sample-rate-agnostic so 44.1 kHz later is a constant change).
//
// Wiring (PORTING-SPEC.md + resolved dump + take-1; corrections applied):
//   4 VoicePairs -> s~ del-12/34/56/78 buses (one pd-block delay, 64 @32k)
//   prm.source per pair: M = cross-pair audio (SWITCH picks A/B) or
//     LFO-CV (sqr-lfo, or total-feedback tap when TOTAL-FB=1); 0.999/0.001 gates
//   delay input [19] = sum(r~ del-XY * 1/6) + noise~*0.001  (pd LCG, verbatim)
//   2 PARALLEL ModDelay buses: write = [19] + own-expander-output*fb_gain;
//     read time = line~(2^(12k/127)*1.45125 ms, cap 1.45 s int16 @32k) +
//     mod_amt*((own-vd/2 -> lop~689) gated del-mod==0 | LFO bus gated ==2);
//     chain hip~1 -> lop~4000 -> compressor(2.5/2.5/-12/5) -> expander(-60,5)
//     feedback gain g = (k/127)*2^(2k/127) (shared, line~ 23.22 ms);
//     wet = (exp1+exp2)/2 / max(g,1.5) -> tanh; out = tanh*(del-mix line~) +
//     in*(1-del-mix)  [monolithic from the dump: sig~1 -~ mix -> *~ wet]
//   hyper-lfo -> del-tri/del-sqr/sqr-lfo buses (rate = mtof(127(k/127)^2-75))
//   master: drive/dist/mix/vol chain; total-feedback tap (hip~3, pre-vol)
//   out = mono, duplicated to L/R (patch: dac~ 1 2 same signal)
//
// Render contract: Process(out_l, out_r, n) with n <= 64; internally processes
// 64-sample pd blocks (host blocks accumulate). All s~/r~ buses carry a
// one-block delay (pd sort law, verified empirically).
// Host-compilable pure C++ (no MCU headers) — ctag-tbd portability proof.
#ifndef RACK_SRC_LIRA8_LIRA_ENGINE_H_
#define RACK_SRC_LIRA8_LIRA_ENGINE_H_

#include <cstdint>
#include <cmath>
#include <cstring>

#include "pd_primitives.h"
#include "lira8_tc.h"
#if defined(RACK_RP2350_TC)
#include "pico/time.h"           // time_us_32 for the stage probes
#define L8_TIME_US() time_us_32()
#else
#define L8_TIME_US() 0u          // host golden harness: probes inert
#endif
#include "lira_osc.h"
#include "lira_voice.h"
#include "lira_delay.h"
#include "lira_lfo.h"
#include "lira_master.h"

namespace l8pd {

class Lira8Engine {
 public:
  static constexpr int kBlockSize = 64;   // pd block @ any rate
  static constexpr int kNumBuses = 8;     // del-12/34/56/78, tri, sqr, sqr-lfo, total-fb

  // cap_ms: delay storage cap (1450 ms RP2350 int16; 5944 ms ESP32 PSRAM).
  // delay_mem: EXTERNAL int16 storage for both delay lines, carved from the
  // rack buffer arena by the adapter (2 x (nsamps+4) samples, contiguous) —
  // NO heap anywhere (the rack discipline). Value members throughout.
  explicit Lira8Engine(float sr = 32000.0f, float cap_ms = 1450.0f,
                       int16_t* delay_mem = nullptr)
      : sr_(sr),
        pairs_{VoicePair(sr, 0, 0u), VoicePair(sr, 1, 0u),
               VoicePair(sr, 2, 0u), VoicePair(sr, 3, 0u)},
        delays_{ModDelay(sr, cap_ms, kBlockSize, delay_mem),
                ModDelay(sr, cap_ms, kBlockSize,
                         delay_mem ? delay_mem + (nsamps_for(cap_ms, sr) + 4)
                                   : nullptr)},
        comp_{Compander(sr), Compander(sr)},
        exp_{Expander(sr), Expander(sr)},
        self_lpf_{Lop(689.0f, sr), Lop(689.0f, sr)},
        time_line_{Ramp(sr), Ramp(sr)},
        mod_line_{Ramp(sr), Ramp(sr)},
        fb_line_(sr),
        del_mix_line_(sr) {
    for (int b = 0; b < 2; ++b) {
      comp_[b].set(2.5f, 2.5f, -12.0f, 5.0f);
      exp_[b].set(-60.0f, 5.0f);
    }
    memset(buses_prev_, 0, sizeof(buses_prev_));
    memset(bus_next_, 0, sizeof(bus_next_));
    memset(self_prev_, 0, sizeof(self_prev_));
    apply_defaults();
  }

  static constexpr int nsamps_for(float cap_ms, float sr) {
    return static_cast<int>(cap_ms * sr / 1000.0f);
  }
  // Bytes of external storage the engine needs for both delay lines.
  static constexpr size_t delay_bytes_for(float cap_ms, float sr) {
    return (static_cast<size_t>(nsamps_for(cap_ms, sr)) + 4) * 2 * 2;
  }

  // ---- control surface (raw 0..127; applied between internal blocks) ----
  // Param indices follow the CC candidate map (spec §12): 0-7 tune, 8-11 fast,
  // 12-15 sharp, 16-19 mod, 20-23 source, 24-25 pitch, 26-27 hold, 28 switch,
  // 29 total-fb, 30 vibrato, 31 drv, 32 dst-mix, 33 vol, 34 f-a, 35 f-b,
  // 36 andor, 37 link, 38-39 time, 40 feedback, 41 del-mix, 42-43 mod-1/2,
  // 44 del-mod, 45 lfo-wav, 46 quantize.
  static constexpr int kNumParams = 47;
  void set_param(int idx, float v127) { pending_params_[idx] = v127; }

  // Stage-profile maxima (microseconds, max since the last read; the
  // PORTING-HIGHLIGHTS §6 pattern — on-device only, host ignores them).
  volatile uint32_t dbg_voices = 0, dbg_delay = 0, dbg_lfo = 0,
                    dbg_master = 0, dbg_params = 0;
  void set_sensor(int voice /*0..7*/, bool on) { pending_sensors_[voice] = on; }

  // Host render: n samples (1..64, normally 16); mono duplicated to L/R.
  LIRA8_TC(lira_engine_process)
  void Process(float* out_l, float* out_r, int n) {
    int done = 0;
    while (done < n) {
      if (out_pos_ == 0) render_internal_block();
      int take = (n - done < (kBlockSize - out_pos_))
                     ? (n - done) : (kBlockSize - out_pos_);
      for (int i = 0; i < take; ++i) {
        out_l[done + i] = out_buf_[out_pos_ + i];
        out_r[done + i] = out_buf_[out_pos_ + i];
      }
      out_pos_ += take;
      if (out_pos_ >= kBlockSize) out_pos_ = 0;
      done += take;
    }
  }

 private:
  void apply_defaults() {
    // parameters.txt defaults (spec §12; fractional values kept as floats)
    static const float def[kNumParams] = {
        0, 78, 74, 36, 27, 57, 60, 51,          // tune 1-8
        1, 1, 1, 1,                             // fast
        14.377f, 43.132f, 103.038f, 71.887f,    // sharp
        0, 0, 0, 0,                             // mod
        0, 0, 0, 0,                             // source
        66, 65,                                 // pitch
        0, 0,                                   // hold
        0, 0, 1,                                // switch, total-fb, vibrato
        86, 69, 127,                            // drv, dst-mix, vol
        42, 64, 1, 1,                           // f-a, f-b, andor, link
        94.574f, 56.745f,                       // time 1/2
        54.043f, 67.553f,                       // feedback, del-mix
        40.736f, 47.925f,                       // mod-1/2
        2, 0, 0,                                // del-mod, lfo-wav, quantize
    };
    for (int i = 0; i < kNumParams; ++i) {
      params_[i] = def[i];
      pending_params_[i] = -1.0f;
    }
    for (int i = 0; i < 8; ++i) {
      sensor_state_[i] = false;
      pending_sensors_[i] = false;
    }
    apply_params();
  }

  void apply_params() {
    for (int p = 0; p < 4; ++p) {
      VoicePair& vp = pairs_[p];
      vp.set_tune(0, (int)params_[p * 2]);
      vp.set_tune(1, (int)params_[p * 2 + 1]);
      vp.set_fast(params_[8 + p] >= 0.5f);
      vp.set_sharp(params_[12 + p]);
      vp.set_mod(params_[16 + p]);
      vp.set_pitch((int)params_[24 + p / 2]);
      vp.set_hold((int)params_[26 + p / 2]);
      vp.set_quantize(params_[46] != 0.0f ? 1 : 0);   // CC 53 (was never applied)
      vp.set_vibrato(params_[30] >= 0.5f ? 127 : 0);
      vp.refresh_tune();
      src_[p] = (int)params_[20 + p];
    }
    switch_ = params_[28] >= 0.5f ? 1.0f : 0.0f;
    total_fb_ = params_[29] >= 0.5f ? 1.0f : 0.0f;
    master_.set_drv((int)params_[31]);
    master_.set_dst_mix((int)params_[32]);
    master_.set_vol((int)params_[33]);
    lfo_.set_rate_a((int)params_[34]);
    lfo_.set_rate_b((int)params_[35]);
    lfo_.set_andor(params_[36]);
    lfo_.set_link(params_[37]);
    for (int b = 0; b < 2; ++b) {
      const float x = params_[38 + b] / 127.0f;
      time_line_[b].set(std::pow(2.0f, x * 12.0f) * 1.45125f, 23.22f);
      const float mk = params_[42 + b] / 127.0f;
      mod_line_[b].set(mk * mk * 10.0f, 23.22f);
    }
    const float fx = params_[40] / 127.0f;
    fb_line_.set(fx * std::pow(2.0f, 2.0f * fx), 23.22f);
    del_mix_line_.set(params_[41] / 127.0f, 23.22f);
    del_mod_ = (int)params_[44];
    lfo_wav_ = (int)params_[45];
  }

  LIRA8_TC(lira_engine_block)
  void render_internal_block() {
    // apply pending control changes between blocks (rack discipline)
    bool dirty = false;
    for (int i = 0; i < kNumParams; ++i) {
      if (pending_params_[i] >= 0.0f) {
        params_[i] = pending_params_[i];
        pending_params_[i] = -1.0f;
        dirty = true;
      }
    }
    for (int v = 0; v < 8; ++v) {
      if (pending_sensors_[v] != sensor_state_[v]) {
        sensor_state_[v] = pending_sensors_[v];
        pairs_[v / 2].set_sensor(v % 2, sensor_state_[v]);
      }
    }
    if (dirty) {
      uint32_t tp = L8_TIME_US();
      apply_params();
      tp = L8_TIME_US() - tp;
      if (tp > dbg_params) dbg_params = tp;
    }

    // 1) LFO buses (written now, consumed next block via one-block delay)
    uint32_t t0 = L8_TIME_US();
    lfo_.process_block(kBlockSize, bus_next_[4], bus_next_[5], bus_next_[6]);
    t0 = L8_TIME_US() - t0;
    if (t0 > dbg_lfo) dbg_lfo = t0;

    // 2) voice pairs (M from PREVIOUS block's buses; soft-gated mux)
    t0 = L8_TIME_US();
    for (int p = 0; p < 4; ++p) {
      mux_source(p, mux_buf_);
      pairs_[p].process(mux_buf_, kBlockSize, blk_[p], bus_next_[p]);
    }
    t0 = L8_TIME_US() - t0;
    if (t0 > dbg_voices) dbg_voices = t0;

    // 3) delay section
    t0 = L8_TIME_US();
    render_delay();
    t0 = L8_TIME_US() - t0;
    if (t0 > dbg_delay) dbg_delay = t0;

    // 4) master + total-feedback bus
    t0 = L8_TIME_US();
    for (int i = 0; i < kBlockSize; ++i) {
      float tap;
      master_.process(delay_out_[i], out_buf_[i], tap);
      bus_next_[7][i] = tap;
    }
    t0 = L8_TIME_US() - t0;
    if (t0 > dbg_master) dbg_master = t0;

    // 5) swap bus rings (one-block delay for ALL s~/r~ buses)
    memcpy(buses_prev_, bus_next_, sizeof(buses_prev_));
  }

  LIRA8_TC(lira_engine_mux)
  void mux_source(int p, float* M) {
    // prm.source (spec §6): srcA/srcB bus indices per pair, SWITCH picks,
    // source 0=audio 1=off 2=LFO-CV; soft gates 0.999/0.001
    static const int kSrcA[4] = {1, 0, 3, 2};
    static const int kSrcB[4] = {3, 0, 1, 2};
    const float ga_audio = (switch_ < 0.5f) ? 0.999f : 0.001f;
    const float gb_audio = (switch_ < 0.5f) ? 0.001f : 0.999f;
    const float gl_lfo = (total_fb_ < 0.5f) ? 0.999f : 0.001f;
    const float gl_fb = (total_fb_ < 0.5f) ? 0.001f : 0.999f;
    const float ga_out = (src_[p] == 0) ? 0.999f : 0.001f;
    const float gl_out = (src_[p] == 2) ? 0.999f : 0.001f;
    const int a = kSrcA[p], bidx = kSrcB[p];
    for (int i = 0; i < kBlockSize; ++i) {
      const float audio = buses_prev_[a][i] * ga_audio +
                          buses_prev_[bidx][i] * gb_audio;
      const float lfo_cv = buses_prev_[6][i] * gl_lfo +
                           buses_prev_[7][i] * gl_fb;
      M[i] = audio * ga_out + lfo_cv * gl_out;
    }
  }

  LIRA8_TC(lira_engine_delay)
  void render_delay() {
    static const float kInv6 = 0.1666667f;
    float in_sum[kBlockSize];    // dry source: voice sum ONLY (no noise!)
    float write_sum[kBlockSize]; // delay write input: voice sum + noise*0.001
    const float gate_self = (del_mod_ == 0) ? 0.999f : 0.001f;
    const float gate_lfo = (del_mod_ == 2) ? 0.999f : 0.001f;
    for (int i = 0; i < kBlockSize; ++i) {
      // pd: the delay INPUT sums the four voice subpatch OUTLETS = the raw
      // current-block pair sums (the hip~3'd copy goes only to the s~ del
      // mod bus). v1 used buses_prev_ -> a 2 ms lag + a 3 Hz HPF in the
      // dry/feedback path (verified from the voice dump: +~ -> outlet~,
      // +~ -> hip~3 -> s~ del).
      in_sum[i] = (blk_[0][i] + blk_[1][i] +
                   blk_[2][i] + blk_[3][i]) * kInv6;
      // pd: the noise~ joins ONLY at [19] (the write summers) — the dry
      // path takes the clean inlet~ signal (floor measurement 2026-11-15).
      write_sum[i] = in_sum[i] + noise_.next() * 0.001f;
    }
    // The feedback-gain line~ is ONE signal shared by both write summers
    // AND the wet divisor (pd: one line~ -> both buses + the monitor).
    // Advance it per sample, once per block.
    float fg[kBlockSize];
    for (int i = 0; i < kBlockSize; ++i) fg[i] = fb_line_.next();
    for (int b = 0; b < 2; ++b) {
      float rp[kBlockSize];
      for (int i = 0; i < kBlockSize; ++i) {
        // time_/mod_ line~s are PER-BUS signals — advance them per sample.
        // The v1 port advanced them once per 64-block: 23.22 ms smoothing
        // became 1.49 s (feedback 743 ms), so every delay move crawled.
        const float t = time_line_[b].next();
        const float ma = mod_line_[b].next();
        const float self = self_prev_[b][i];   // own vd /2, lop~689'd
        const float lfo = (lfo_wav_ == 0) ? buses_prev_[4][i]
                                          : buses_prev_[5][i];
        rp[i] = t + (self * gate_self + lfo * gate_lfo) * ma;
      }
      delays_[b].read_block(rp, kBlockSize, vd_out_[b]);
      // SELF = the SECOND vd~ at MINIMUM delay (unconnected time inlet),
      // constant across the block; the main vd_out is the wet read, which
      // can stand hundreds of ms away (v1 used it, so the self-mod followed
      // the delayed wet instead of the recent buffer). The /2 of pd's
      // wiring commutes with the linear lop~.
      const float self_tap = delays_[b].read_min_tap();
      for (int i = 0; i < kBlockSize; ++i)
        self_next_[b][i] = self_lpf_[b].process(self_tap) * 0.5f;

      // chain: hip~1 -> lop~4000 -> comp -> exp
      for (int i = 0; i < kBlockSize; ++i) {
        chain_buf_[i] = comp_[b].process(
            lop4k_.process(hip1_.process(vd_out_[b][i])));
      }
      exp_[b].process_block(chain_buf_, kBlockSize, exp_out_[b]);
      // write = [19] + own feedback * shared smoothed gain
      for (int i = 0; i < kBlockSize; ++i) {
        write_buf_[b][i] = write_sum[i] + exp_out_[b][i] * fg[i];
      }
      delays_[b].write_block(write_buf_[b], kBlockSize);
    }
    // wet monitor: (exp1+exp2)/2 / max(g,1.5) -> tanh -> mix with dry
    for (int i = 0; i < kBlockSize; ++i) {
      const float dm = del_mix_line_.next();
      const float g = fg[i];
      const float gc = (g > 1.5f) ? g : 1.5f;
      float wet = (exp_out_[0][i] + exp_out_[1][i]) * 0.5f / gc;
      // ma.tanh~ clips +-3 BEFORE the Pade (the pd object); without it the
      // wet path can exceed 1.0 and keep growing inside the feedback loop.
      if (wet > 3.0f) wet = 3.0f;
      else if (wet < -3.0f) wet = -3.0f;
      const float shaped = wet * (27.0f + wet * wet) /
                           (27.0f + 9.0f * wet * wet);
      delay_out_[i] = shaped * dm + in_sum[i] * (1.0f - dm);
    }
    memcpy(self_prev_, self_next_, sizeof(self_prev_));
  }

  float sr_;
  VoicePair pairs_[4];
  ModDelay delays_[2];
  Compander comp_[2];
  Expander exp_[2];
  Lop self_lpf_[2];
  MasterOut master_{sr_};
  HyperLfo lfo_{sr_};
  Noise noise_{Noise::first_instance_seed()};

  float params_[kNumParams];
  float pending_params_[kNumParams];
  bool pending_sensors_[8];
  bool sensor_state_[8];
  int src_[4];
  float switch_ = 0.f, total_fb_ = 0.f;
  int del_mod_ = 2, lfo_wav_ = 0;
  Ramp time_line_[2], mod_line_[2];
  Ramp fb_line_, del_mix_line_;

  float buses_prev_[kNumBuses][64];
  float bus_next_[kNumBuses][64];
  float blk_[4][64];
  float mux_buf_[64];
  float self_prev_[2][64];
  float self_next_[2][64];
  float vd_out_[2][64];
  float write_buf_[2][64];
  float exp_out_[2][64];
  float chain_buf_[64];
  float delay_out_[64];
  float out_buf_[64];
  int out_pos_ = 0;
  Hip hip1_{1.0f, sr_};
  Lop lop4k_{4000.0f, sr_};
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_ENGINE_H_
