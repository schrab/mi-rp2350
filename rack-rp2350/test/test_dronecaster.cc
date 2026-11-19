// Dronecaster host tests (T-D1). Sections are appended by plan task;
// the comparator is proven here against synthetic float32 cases so a later
// golden FAIL is trustworthy.
#include "wav_io.h"

#include "../src/dronecaster/sc_defs.h"
#include "../src/dronecaster/sc_rgen.h"
#include "../src/dronecaster/sc_kit_osc.h"
#include "../src/dronecaster/sc_kit_pulse.h"
#include "../src/dronecaster/sc_arena.h"
#include "../src/dronecaster/sc_kit_env.h"
#include "../src/dronecaster/dronecaster_engine.h"
#include "../src/dronecaster/drones/drone_trio.h"
#include "../src/dronecaster/sc_kit_noise.h"
#include "../src/dronecaster/sc_kit_lfo.h"
#include "../src/dronecaster/sc_kit_trig.h"
#include "../src/dronecaster/sc_kit_osc2.h"
#include "../src/dronecaster/sc_kit_filter.h"
#include "../src/dronecaster/sc_kit_core_dyn.h"
#include "../src/dronecaster/sc_kit_sc3p_filter.h"
#include "../src/dronecaster/sc_kit_dfm1.h"
#include "../src/dronecaster/sc_kit_delay.h"
#include "../src/dronecaster/sc_kit_localio.h"
#include "../src/dronecaster/sc_kit_reverb.h"
#include "../src/dronecaster/sc_kit_pan.h"
#include "../src/dronecaster/sc_kit_greyhole.h"
#include "../src/dronecaster/sc_kit_demand.h"
#include "../src/dronecaster/sc_kit_buf.h"
#include "../src/dronecaster/sc_kit_chaos.h"
#include "../src/dronecaster/sc_kit_sc3p_dyn.h"
#include "../src/dronecaster/sc_sugar.h"
#include "../src/dronecaster/sc_kit_hw.h"
#include "../src/dronecaster/sc_kit_envgen.h"
#include <cstdio>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static int g_failures = 0;
#define CHECK(cond, msg) do { if (cond) { printf("  ok: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); ++g_failures; } } while (0)

struct GoldenReport {
  long   frames_ref = 0, frames_got = 0;
  double max_abs = 0.0;      // absolute sample units
  long   worst_i = 0;        // sample index of max |delta|
  double rms_db = -999.0;    // 20*log10(rms(diff)/rms(ref))
  double corr = 0.0;
};

// Compare two float32 WAVs in the float domain. Gates set by the caller.
static bool compare(const char* ref_path, const char* got_path,
                    double max_abs_gate, double corr_gate, GoldenReport* rep) {
  l8test::WavFile a = l8test::ReadWav(ref_path);
  l8test::WavFile b = l8test::ReadWav(got_path);
  rep->frames_ref = static_cast<long>(a.samples.size() / (a.channels ? a.channels : 1));
  rep->frames_got = static_cast<long>(b.samples.size() / (b.channels ? b.channels : 1));
  if (a.channels == 0 || b.channels == 0 || a.sample_rate != b.sample_rate ||
      a.channels != b.channels || a.samples.size() != b.samples.size()) {
    printf("  FAIL: layout ref=%ldf/%dch/%dHz got=%ldf/%dch/%dHz\n",
           rep->frames_ref, a.channels, a.sample_rate,
           rep->frames_got, b.channels, b.sample_rate);
    return false;
  }
  const long n = static_cast<long>(a.samples.size());
  double num = 0, den_a = 0, den_b = 0, diff_sq = 0, max_abs = 0;
  long worst_i = 0;
  for (long i = 0; i < n; ++i) {
    const double d = static_cast<double>(a.samples[i]) - b.samples[i];
    if (fabs(d) > max_abs) { max_abs = fabs(d); worst_i = i; }
    diff_sq += d * d;
    num   += static_cast<double>(a.samples[i]) * b.samples[i];
    den_a += static_cast<double>(a.samples[i]) * a.samples[i];
    den_b += static_cast<double>(b.samples[i]) * b.samples[i];
  }
  rep->max_abs = max_abs;
  rep->worst_i = worst_i;
  rep->rms_db = (diff_sq > 0 && den_a > 0)
      ? 20.0 * log10(sqrt(diff_sq / n) / sqrt(den_a / n)) : -999.0;
  rep->corr = (den_a > 0 && den_b > 0) ? num / sqrt(den_a * den_b) : 0.0;
  printf("  metrics: max=%.9f rms=%.1f dB corr=%.9f worst_i=%ld\n",
         rep->max_abs, rep->rms_db, rep->corr, rep->worst_i);
  return rep->max_abs <= max_abs_gate && rep->corr >= corr_gate;
}

static void write_constant_wav(const char* path, float v, int frames) {
  std::vector<float> x(static_cast<size_t>(2 * frames), v);
  l8test::WriteWavF32(path, x.data(), frames, 2, 48000);
}

// ---- T-D2 UGen reference harness: compare a port's 48,000-sample stream
// against dronecaster_ref/ugen_ref/<Name>.expected.wav channel 0 ----
static bool ref_check(const char* name, const float* got, long n) {
  char path[192];
  snprintf(path, sizeof(path),
           "dronecaster_ref/ugen_ref/%s.expected.wav", name);
  l8test::WavFile w = l8test::ReadWav(path);
  const long wf = static_cast<long>(w.samples.size() / (w.channels ? w.channels : 1));
  if (w.channels < 1 || wf != n) {
    printf("  REF %-12s FAIL: layout ref=%ldf got=%ld\n", name, wf, n);
    return false;
  }
  double max_abs = 0.0, num = 0.0, da = 0.0, db = 0.0;
  for (long i = 0; i < n; ++i) {
    const double a = w.samples[static_cast<size_t>(i) * w.channels];
    const double b = got[i];
    const double d = a - b;
    if (fabs(d) > max_abs) max_abs = fabs(d);
    num += a * b; da += a * a; db += b * b;
  }
  const double corr = (da > 0 && db > 0) ? num / sqrt(da * db) : (da == db ? 1.0 : 0.0);
  const bool ok = max_abs <= 1e-6 && corr >= 0.999999;
  printf("  REF %-12s %s: max=%.9f corr=%.9f\n", name, ok ? "ok" : "FAIL",
         max_abs, corr);
  return ok;
}

// Fill `out` (48,000) from an ar UGen.
template <typename U>
static bool ref_ar_fill(const char* name, U& u, std::vector<float>& out) {
  out.assign(48000, 0.0f);
  for (int b = 0; b < 750; ++b) u.ProcessBlock(out.data() + b * 64, 64);
  return ref_check(name, out.data(), 48000);
}

// Fill `out` from a kr UGen as K2A renders it: ControlRateInput's CALCSLOPE
// ramp. The SC 3.14.1 x86 build uses nova's SSE vector_ramp_argument, whose
// arithmetic is NOT a plain accumulation — replicate it bit-for-bit
// (nova-simd vec_sse.hpp set_slope): lanes [s, s+slope, s+2slope,
// s+2slope+slope], group increment 4*slope via repeated adds.
template <typename U>
static bool ref_kr_fill(const char* name, U& u, std::vector<float>& out) {
  out.assign(48000, 0.0f);
  float prev = u.ctor_value();
  for (int b = 0; b < 750; ++b) {
    const float v = u.ProcessKr();
    const float slope = static_cast<float>((v - prev) * (1.0 / 64.0));
    const float slope2 = slope + slope;
    const float v0 = prev;
    const float v1 = prev + slope;
    const float v2 = prev + slope2;
    const float v3 = prev + slope2 + slope;
    float acc = 0.0f;
    for (int g = 0; g < 16; ++g) {
      out[b * 64 + g * 4 + 0] = v0 + acc;
      out[b * 64 + g * 4 + 1] = v1 + acc;
      out[b * 64 + g * 4 + 2] = v2 + acc;
      out[b * 64 + g * 4 + 3] = v3 + acc;
      acc += slope2 + slope2;
    }
    prev = v;
  }
  return ref_check(name, out.data(), 48000);
}

int main() {
  printf("== comparator self-tests ==\n");
  {
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", 0.5f, 480);
    GoldenReport r;
    CHECK(compare("build/dc_a.wav", "build/dc_b.wav", 0.0, 1.0, &r),
          "identical files: max=0 corr=1");
    CHECK(r.max_abs == 0.0 && r.corr >= 0.999999999, "identical metrics exact");
  }
  {
    // +1 int16 LSB (1/32768) offset, written as floats.
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", 0.5f + 1.0f / 32768.0f, 480);
    GoldenReport r;
    CHECK(compare("build/dc_a.wav", "build/dc_b.wav", 1e-4, 0.0, &r),
          "+1 LSB offset passes the 1e-4 gate");
    CHECK(fabs(r.max_abs - 1.0 / 32768.0) < 1e-9, "+1 LSB measured exactly");
  }
  {
    write_constant_wav("build/dc_a.wav", 0.5f, 480);
    write_constant_wav("build/dc_b.wav", -0.5f, 480);
    GoldenReport r;
    CHECK(!compare("build/dc_a.wav", "build/dc_b.wav", 1e-4, 0.999999, &r),
          "polarity flip rejected by corr gate");
  }
  {
    // Float round-trip: values must survive ReadWav bit-for-bit.
    std::vector<float> x = {0.123456789f, -0.5f, 1.25f, -1.0f};
    l8test::WriteWavF32("build/dc_f32.wav", x.data(), 2, 2, 48000);
    l8test::WavFile w = l8test::ReadWav("build/dc_f32.wav");
    CHECK(w.samples.size() == x.size() && w.samples == x, "float32 round-trip exact");
  }
  printf("comparator self-tests: %s (%d failures)\n",
         g_failures ? "FAIL" : "PASS", g_failures);


  printf("== T-D2 trig kit vs NRT references ==\n");
  {
    // Triggers at kr (750): the ctor primes then rolls back, so K2A's block 0
    // is flat at v(0) — capture the first output and use it as the ramp base.
    auto kr_stream = [](auto& u, auto&& next_fn, std::vector<float>& out) {
      float prev = next_fn();
      const float first = prev;
      out.assign(48000, 0.0f);
      for (int b = 0; b < 750; ++b) {
        const float v = (b == 0) ? first : next_fn();
        const float slope = static_cast<float>((v - prev) * (1.0 / 64.0));
        const float s2 = slope + slope;
        const float v0 = prev, v1 = prev + slope, v2 = prev + s2, v3 = prev + s2 + slope;
        float acc = 0.0f;
        for (int g = 0; g < 16; ++g) {
          out[b * 64 + g * 4 + 0] = v0 + acc;
          out[b * 64 + g * 4 + 1] = v1 + acc;
          out[b * 64 + g * 4 + 2] = v2 + acc;
          out[b * 64 + g * 4 + 3] = v3 + acc;
          acc += s2 + s2;
        }
        prev = v;
      }
    };
    ((void)kr_stream);
    std::vector<float> out;
    { // Trig1.kr(Impulse.kr(2), 0.1)
      sc::Impulse imp; imp.Init(750.0, 2.0f);
      sc::Trig1 u; u.Init();
      kr_stream(u, [&]() { return u.Next(imp.ProcessKr(), 0.1f, 750.0f); }, out);
      if (!ref_check("Trig1kr", out.data(), 48000)) ++g_failures;
    }
    { // Trig.kr(Impulse.kr(2), 0.1)
      sc::Impulse imp; imp.Init(750.0, 2.0f);
      sc::Trig u; u.Init();
      kr_stream(u, [&]() { return u.Next(imp.ProcessKr(), 0.1f, 750.0f); }, out);
      if (!ref_check("Trigkr", out.data(), 48000)) ++g_failures;
    }
    { // TDelay.kr(Impulse.kr(2), 0.1)
      sc::Impulse imp; imp.Init(750.0, 2.0f);
      sc::TDelay u; u.Init();
      kr_stream(u, [&]() { return u.Next(imp.ProcessKr(), 0.1f, 750.0f); }, out);
      if (!ref_check("TDelaykr", out.data(), 48000)) ++g_failures;
    }
    { // Latch.kr(SinOsc.kr(5), Impulse.kr(3))
      static sc::KitTables kt; sc::InitKitTables(&kt);
      sc::SinOsc osc; osc.Init(&kt, 750.0, 5.0f);
      sc::Impulse imp; imp.Init(750.0, 3.0f);
      sc::Latch u; u.Init();
      kr_stream(u, [&]() { return u.Next(osc.Process(), imp.ProcessKr()); }, out);
      if (!ref_check("Latchkr", out.data(), 48000)) ++g_failures;
    }
    { // SetResetFF.kr(Impulse.kr(2), Impulse.kr(0.7))
      sc::Impulse hi; hi.Init(750.0, 2.0f);
      sc::Impulse lo; lo.Init(750.0, 0.7f);
      sc::SetResetFF u; u.Init();
      kr_stream(u, [&]() { return u.Next(hi.ProcessKr(), lo.ProcessKr()); }, out);
      if (!ref_check("SetResetkr", out.data(), 48000)) ++g_failures;
    }
    { // Peak.kr(SinOsc.kr(7), Impulse.kr(1))
      static sc::KitTables kt; sc::InitKitTables(&kt);
      sc::SinOsc osc; osc.Init(&kt, 750.0, 7.0f);
      sc::Impulse imp; imp.Init(750.0, 1.0f);
      sc::Peak u; u.Init();
      kr_stream(u, [&]() { return u.Next(osc.Process(), imp.ProcessKr()); }, out);
      if (!ref_check("Peakkr", out.data(), 48000)) ++g_failures;
    }
    { // Gate.ar(SinOsc.ar(440), LFPulse.kr(2)) — ar block loop
      static sc::KitTables kt; sc::InitKitTables(&kt);
      sc::SinOsc osc; osc.Init(&kt, 48000.0, 440.0f);
      sc::LFPulse lp; lp.Init(750.0, 2.0f, 0.0f, 0.5f);
      sc::Gate g; g.Init();
      out.assign(48000, 0.0f);
      float inb[64], outb[64];
      for (int b = 0; b < 750; ++b) {
        osc.ProcessBlock(inb, 64);
        g.ProcessBlock(inb, lp.ProcessKr(), outb, 64);
        memcpy(out.data() + b * 64, outb, 64 * sizeof(float));
      }
      if (!ref_check("Gatear", out.data(), 48000)) ++g_failures;
    }
    { // Changed.kr(LFPulse.kr(5))
      sc::LFPulse lp; lp.Init(750.0, 5.0f, 0.0f, 0.5f);
      sc::Changed u; u.Init();
      kr_stream(u, [&]() { return u.Next(lp.ProcessKr()); }, out);
      if (!ref_check("Changedkr", out.data(), 48000)) ++g_failures;
    }
  }


  printf("== T-D2 osc2 kit vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> out;
    { // Saw.ar(440)
      sc::Saw u; u.Init(&kt, 48000.0, 440.0f);
      if (!ref_ar_fill("Sawar", u, out)) ++g_failures;
    }
    { // SinOscFB.ar(440, 0.5)
      sc::SinOscFB u; u.Init(&kt, 48000.0, 440.0f, 0.5f);
      if (!ref_ar_fill("SinOscFBar", u, out)) ++g_failures;
    }
    { // Select.kr(LFPulse.kr(2), [0.25, 0.5, 0.75]) — kr select + K2A ramp
      sc::LFPulse lp; lp.Init(750.0, 2.0f, 0.0f, 0.5f);
      const float vals[3] = {0.25f, 0.5f, 0.75f};
      out.assign(48000, 0.0f);
      // Select's ctor primes one kr sample (Select_next_1), so K2A's block 0
      // is flat at v(0): take the first value as the ramp base.
      float prev = vals[sc::SelectIndex(lp.ProcessKr(), 3)];
      for (int b = 0; b < 750; ++b) {
        const float idx = (b == 0) ? -1.0f : lp.ProcessKr();
        const float v = (b == 0) ? prev : vals[sc::SelectIndex(idx, 3)];
        const float slope = static_cast<float>((v - prev) * (1.0 / 64.0));
        const float s2 = slope + slope;
        const float v0 = prev, v1 = prev + slope, v2 = prev + s2, v3 = prev + s2 + slope;
        float acc = 0.0f;
        for (int g = 0; g < 16; ++g) {
          out[b * 64 + g * 4 + 0] = v0 + acc;
          out[b * 64 + g * 4 + 1] = v1 + acc;
          out[b * 64 + g * 4 + 2] = v2 + acc;
          out[b * 64 + g * 4 + 3] = v3 + acc;
          acc += s2 + s2;
        }
        prev = v;
      }
      if (!ref_check("Selectkr", out.data(), 48000)) ++g_failures;
    }
    { // Klank: no NRT reference (scsynth 3.14.1 segfaults on Klank.ar here);
      // verify the resonator law: an impulse excites a decaying ringing tail.
      sc::Klank u;
      const float fr[4] = {440.0f, 880.0f, 1320.0f, 1760.0f};
      const float lv[4] = {1.0f, 0.5f, 0.25f, 0.125f};
      const float tm[4] = {0.5f, 0.7f, 1.0f, 0.8f};
      u.Init(48000.0, fr, lv, tm, 4, 1.0f, 0.0f, 1.0f);
      float pk = 0.0f, tail = 0.0f;
      bool finite = true;
      for (int i = 0; i < 48000; ++i) {
        const float v = u.Next(i == 0 ? 1.0f : 0.0f);
        if (!std::isfinite(v)) finite = false;
        if (std::fabs(v) > pk) pk = std::fabs(v);
        if (i > 40000) tail = std::fabs(v) > tail ? std::fabs(v) : tail;
      }
      CHECK(finite && pk > 0.0f, "klank: impulse rings (finite, nonzero)");
      CHECK(tail < pk * 0.5f, "klank: tail decays");
    }
    { // SawDPW.ar(440)
      sc::SawDPW u; u.Init(48000.0, 440.0f);
      if (!ref_ar_fill("SawDPWar", u, out)) ++g_failures;
    }
    { // PulseDPW.ar(440, 0.3)
      sc::PulseDPW u; u.Init(48000.0, 440.0f, 0.3f, 0);
      if (!ref_ar_fill("PulseDPWar", u, out)) ++g_failures;
    }
  }


  printf("== T-D2 filter kit vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> in(48000), out(48000);
    { sc::Saw src; src.Init(&kt, 48000.0, 220.0f);
      for (int b = 0; b < 750; ++b) src.ProcessBlock(in.data() + b * 64, 64); }
    auto filt = [&](const char* name, int kind, float freq, float q) {
      sc::Filter2Pole f; f.Init(48000.0, kind);
      for (int b = 0; b < 750; ++b)
        f.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64, freq, q);
      if (!ref_check(name, out.data(), 48000)) ++g_failures;
    };
    filt("LPFar", 0, 2000.0f, 0.0f);
    filt("HPFar", 1, 500.0f, 0.0f);
    filt("BPFar", 2, 1000.0f, 0.25f);
    filt("RLPFar", 3, 1000.0f, 0.5f);
    filt("RHPFar", 4, 1000.0f, 0.5f);
    auto per_sample = [&](const char* name, auto&& next_fn) {
      for (int i = 0; i < 48000; ++i) out[i] = next_fn(in[i]);
      if (!ref_check(name, out.data(), 48000)) ++g_failures;
    };
    { sc::OnePole u; u.Init(48000.0);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64, 0.9f);
      if (!ref_check("OnePolear", out.data(), 48000)) ++g_failures; }
    { sc::Integrator u; u.Init(); u.Prime(in[0], 0.9f);
      per_sample("Integratorar", [&](float x) { return u.Next(x, 0.9f); }); }
    { sc::LeakDC u; u.Init(); u.Prime(in[0]);
      per_sample("LeakDCar", [&](float x) { return u.Next(x, 0.995f); }); }
    { sc::Slew u; u.Init(48000.0); u.Prime(in[0]);
      per_sample("Slewar", [&](float x) { return u.Next(x, 1.0f, 1.0f); }); }
    { sc::Lag u; u.Init(48000.0); u.Prime(in[0]);
      per_sample("Lagar", [&](float x) { return u.Next(x, 0.01f); }); }
    { sc::LagUD u; u.Init(48000.0); u.Prime(in[0]);
      per_sample("LagUDar", [&](float x) { return u.Next(x, 0.01f, 0.05f); }); }
    { sc::MoogFF u; u.Init(48000.0, 100.0f, 2.0f, 0.0f, in[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64,
                       100.0f, 2.0f, 0.0f);
      if (!ref_check("MoogFFar", out.data(), 48000)) ++g_failures; }
    // BEQ suite (constant params: BLowPass/BHiPass/BPeakEQ/BAllPass use rq,
    // shelves use rs). kind 1/5 are extras sharing the same coefficient law.
    auto beq = [&](const char* name, int kind, float f, float p2, float p3) {
      sc::BEQFilter u; u.Init(48000.0, kind, f, p2, p3, in[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64, f, p2, p3);
      if (!ref_check(name, out.data(), 48000)) ++g_failures;
    };
    beq("BLowPassar",  0, 1000.0f, 1.0f, 0.0f);
    beq("BHiPassar",   1, 1000.0f, 1.0f, 0.0f);
    beq("BPeakEQar",   2, 1000.0f, 1.0f, 6.0f);
    beq("BAllPassar",  3, 1000.0f, 1.0f, 0.0f);
    beq("BLowShelfar", 4,  500.0f, 1.0f, 6.0f);
    beq("BHiShelfar",  5, 5000.0f, 1.0f, 6.0f);
  }

  printf("== T-D2 Task 7 SC3P filters vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> in(48000), out(48000);
    { sc::Saw src; src.Init(&kt, 48000.0, 220.0f);
      for (int b = 0; b < 750; ++b) src.ProcessBlock(in.data() + b * 64, 64); }
    { sc::SVF u; u.Init(48000.0, 2200.0f, 0.1f, 1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, in[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64,
                       2200.0f, 0.1f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      if (!ref_check("SVFar", out.data(), 48000)) ++g_failures; }
    { sc::MoogLadder u; u.Init(48000.0, 440.0f, 0.3f);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64,
                       440.0f, 0.3f);
      if (!ref_check("MoogLadderar", out.data(), 48000)) ++g_failures; }
    // DFM1's upstream RNG is seeded from time(NULL) at ctor. The renderer
    // captured the epoch into DFM1.seed.txt just before launching scsynth;
    // re-seed the port from it so the NRT render is reproducible.
    long epoch = 0;
    FILE* sf = fopen("dronecaster_ref/ugen_ref/DFM1.seed.txt", "r");
    if (sf) { if (fscanf(sf, "%ld", &epoch) != 1) epoch = 0; fclose(sf); }
    if (epoch == 0) { printf("  FAIL: DFM1 seed file missing\n"); ++g_failures; }
    { sc::Dfm1 u; u.Init(48000.0, static_cast<int>(epoch), 1000.0f, 0.1f,
                         1.0f, 0.0f, 0.0003f);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64);
      if (!ref_check("DFM1", out.data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 Task 8 core delay family vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> in(48000), out(48000);
    { sc::Saw src; src.Init(&kt, 48000.0, 220.0f);
      for (int b = 0; b < 750; ++b) src.ProcessBlock(in.data() + b * 64, 64); }
    auto delay = [&](const char* name, int kind, float decay) {
      sc::DelayLine u; u.Init(48000.0, 0.01f, 0.005f, decay, kind);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64,
                       0.005f, decay);
      if (!ref_check(name, out.data(), 48000)) ++g_failures;
    };
    delay("DelayNar",   0, 0.f);
    delay("DelayLar",   1, 0.f);
    delay("DelayCar",   2, 0.f);
    delay("CombNar",    3, 0.1f);
    delay("CombLar",    4, 0.1f);
    delay("CombCar",    5, 0.1f);
    delay("AllpassNar", 6, 0.1f);
    delay("AllpassLar", 7, 0.1f);
    delay("AllpassCar", 8, 0.1f);
    { static sc::KitTables kt2; sc::InitKitTables(&kt2);
      std::vector<float> trig(48000), outp(48000);
      { sc::Impulse src; src.Init(48000.0, 4.0f);
        for (int b = 0; b < 750; ++b) src.ProcessBlock(trig.data() + b * 64, 64); }
      sc::Pluck u; u.Init(48000.0, 0.01f, 0.005f, 0.1f, 0.5f);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, trig.data() + b * 64,
                       outp.data() + b * 64, 64, 0.005f, 0.1f, 0.5f);
      if (!ref_check("Pluckar", outp.data(), 48000)) ++g_failures; }
    // Buf* variants: same ring core, but dsamp clipped to bufSamples-1.
    auto bufdelay = [&](const char* name, int kind, float decay) {
      sc::DelayLine u; u.InitBuf(48000.0, 1024, 0.005f, decay, kind);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64,
                       0.005f, decay);
      if (!ref_check(name, out.data(), 48000)) ++g_failures;
    };
    bufdelay("BufDelayNar", 0, 0.f);
    bufdelay("BufDelayLar", 1, 0.f);
    bufdelay("BufDelayCar", 2, 0.f);
    bufdelay("BufAllpassNar", 6, 0.1f);
    bufdelay("BufAllpassLar", 7, 0.1f);
    bufdelay("BufAllpassCar", 8, 0.1f);
    { sc::RGen r; r.init(sc::kGoldenSeed);
      sc::PitchShift u; u.Init(48000.0, 0.01f, 1.5f, 0.1f, 0.001f, in[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64, &r);
      if (!ref_check("PitchShiftar", out.data(), 48000)) ++g_failures; }
    { sc::LocalBus bus; bus.Init(1);
      sc::LocalIn li; li.Init(1); sc::LocalOut lo; lo.Init(1);
      std::vector<float> fb(64), sig(64), dry(64), lout(64), dest(48000);
      const float def0 = 0.f;
      static sc::KitTables kt2; sc::InitKitTables(&kt2);
      sc::Saw src; src.Init(&kt2, 48000.0, 220.0f);
      for (int b = 0; b < 750; ++b) {
        src.ProcessBlock(dry.data(), 64);
        float* outs[1] = { fb.data() };
        li.ProcessBlock(bus, outs, &def0, b, 64);
        for (int i = 0; i < 64; ++i) {
          sig[i] = dry[i] + fb[i] * 0.5f;
          dest[b * 64 + i] = sig[i];
          lout[i] = sig[i] * 0.5f;
        }
        const float* ins[1] = { lout.data() };
        lo.ProcessBlock(bus, ins, b, 64);
      }
      if (!ref_check("LocalIO", dest.data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 Task 9 reverb vs NRT reference ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> in(48000), out(48000);
    { sc::Saw src; src.Init(&kt, 48000.0, 220.0f);
      for (int b = 0; b < 750; ++b) src.ProcessBlock(in.data() + b * 64, 64); }
    sc::FreeVerb fv; fv.Init(0.5f, 0.5f, 0.2f, in[0]);
    for (int b = 0; b < 750; ++b)
      fv.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64,
                      0.5f, 0.5f, 0.2f);
    if (!ref_check("FreeVerbar", out.data(), 48000)) ++g_failures;
    // GVerb defaults: roomsize 10, revtime 3, damping .5, bw .5, spread 15,
    // dry 1, early .7, tail .5, maxroomsize 300. Reference is channel 0.
    { std::vector<float> right(48000);
      sc::GVerb g; g.Init(48000.0, 10.f, 3.f, 0.5f, 0.5f, 15.f,
                          1.f, 0.7f, 0.5f, 300.f);
      for (int b = 0; b < 750; ++b)
        g.ProcessBlock(in.data() + b * 64, out.data() + b * 64,
                       right.data() + b * 64, 64,
                       10.f, 3.f, 0.5f, 0.5f, 1.f, 0.7f, 0.5f);
      if (!ref_check("GVerbar", out.data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 Task 10 pan family vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> a(48000), b(48000), c(48000), l(48000), r(48000);
    { sc::Saw s; s.Init(&kt, 48000.0, 220.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(a.data() + i * 64, 64); }
    { sc::Saw s; s.Init(&kt, 48000.0, 330.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(b.data() + i * 64, 64); }
    { sc::Saw s; s.Init(&kt, 48000.0, 440.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(c.data() + i * 64, 64); }
    { sc::Pan2 u; u.Init(&kt, 0.3f, 0.8f, a[0]);
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data() + i * 64, l.data() + i * 64,
                       r.data() + i * 64, 64, 0.3f, 0.8f);
      if (!ref_check("Pan2ar", l.data(), 48000)) ++g_failures; }
    { sc::Balance2 u; u.Init(&kt, 0.3f, 0.8f, a[0], b[0]);
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data() + i * 64, b.data() + i * 64,
                       l.data() + i * 64, r.data() + i * 64, 64, 0.3f, 0.8f);
      if (!ref_check("Balance2ar", l.data(), 48000)) ++g_failures; }
    { sc::Rotate2 u; u.Init(&kt, 0.3f, a[0], b[0]);
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data() + i * 64, b.data() + i * 64,
                       l.data() + i * 64, r.data() + i * 64, 64, 0.3f);
      if (!ref_check("Rotate2ar", l.data(), 48000)) ++g_failures; }
    { const float first[3] = {a[0], b[0], c[0]};
      sc::Splay u; u.Init(&kt, 3, 1.f, 0.8f, 0.f, true, first);
      for (int i = 0; i < 750; ++i) {
        const float* ins[3] = {a.data() + i * 64, b.data() + i * 64,
                               c.data() + i * 64};
        u.ProcessBlock(ins, l.data() + i * 64, r.data() + i * 64, 64,
                       1.f, 0.8f, 0.f);
      }
      if (!ref_check("Splayar", l.data(), 48000)) ++g_failures; }
    { std::vector<float> four[4];
      for (int ch = 0; ch < 4; ++ch) four[ch].assign(48000, 0.f);
      sc::PanAz u; u.Init(&kt, 4, 0.3f, 0.8f, 2.f, 0.5f, a[0]);
      for (int i = 0; i < 750; ++i) {
        float* outs[4];
        for (int ch = 0; ch < 4; ++ch) outs[ch] = four[ch].data() + i * 64;
        u.ProcessBlock(a.data() + i * 64, outs, 64, 0.3f, 0.8f, 2.f, 0.5f);
      }
      if (!ref_check("PanAzar", four[0].data(), 48000)) ++g_failures; }
    { std::vector<float> four[4];
      for (int ch = 0; ch < 4; ++ch) four[ch].assign(48000, 0.f);
      const float first[3] = {a[0], b[0], c[0]};
      sc::SplayAz u; u.Init(&kt, 4, 3, 1.f, 0.8f, 2.f, 0.f, 0.5f, true, first);
      for (int i = 0; i < 750; ++i) {
        const float* ins[3] = {a.data() + i * 64, b.data() + i * 64,
                               c.data() + i * 64};
        float* outs[4];
        for (int ch = 0; ch < 4; ++ch) outs[ch] = four[ch].data() + i * 64;
        u.ProcessBlock(ins, outs, 64, 1.f, 0.8f, 2.f, 0.f, 0.5f);
      }
      if (!ref_check("SplayAzar", four[0].data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 GreyholeRaw vs NRT reference ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> a(48000), b(48000), l(48000), r(48000);
    { sc::Saw s; s.Init(&kt, 48000.0, 220.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(a.data() + i * 64, 64); }
    { sc::Saw s; s.Init(&kt, 48000.0, 330.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(b.data() + i * 64, 64); }
    sc::GreyholeRaw g; g.Init(48000.0);
    for (int i = 0; i < 750; ++i)
      g.ProcessBlock(a.data() + i * 64, b.data() + i * 64,
                     l.data() + i * 64, r.data() + i * 64, 64,
                     0.f, 2.f, 0.5f, 0.9f, 0.1f, 2.f, 1.f);
    if (!ref_check("GreyholeRaw", l.data(), 48000)) ++g_failures;
  }

  printf("== T-D2 Klank vs NRT reference ==\n");
  {
    std::vector<float> imp(48000), out(48000);
    { sc::Impulse src; src.Init(48000.0, 5.0f);
      for (int i = 0; i < 750; ++i) src.ProcessBlock(imp.data() + i * 64, 64); }
    const float freqs[3] = {220.f, 440.f, 660.f};
    const float amps[3] = {1.f, 0.5f, 0.25f};
    const float times[3] = {1.f, 1.f, 1.f};
    sc::Klank k; k.Init(48000.0, freqs, amps, times, 3, 1.f, 0.f, 1.f);
    for (int i = 0; i < 750; ++i)
      k.ProcessBlock(imp.data() + i * 64, out.data() + i * 64, 64);
    if (!ref_check("Klankar", out.data(), 48000)) ++g_failures;
  }

  printf("== T-D2 Task 11 demand kit vs NRT references ==\n");
  {
    std::vector<float> trig(48000), out(48000);
    { sc::Impulse s; s.Init(48000.0, 10.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(trig.data() + i * 64, 64); }
    auto run = [&](const char* name, sc::DemandSource* src) {
      src->Reset();  // SC's Demand* _Ctor calls next(unit, 0) (the reset path)
      sc::Demand d; d.Init({src});
      std::vector<float> c(48000);
      for (int i = 0; i < 750; ++i) {
        float* outs[1] = {c.data() + i * 64};
        d.ProcessBlock(trig.data() + i * 64, 0.f, outs, 64);
      }
      if (!ref_check(name, c.data(), 48000)) ++g_failures;
    };
    { sc::DSeq s({INFINITY, 1, 2, 3}); run("DemandDseq", &s); }
    { sc::DSeries s({INFINITY, 5, 2}); run("DemandSeries", &s); }
    { sc::DSeq a({INFINITY, 1, 2, 3}); sc::DSeries b({INFINITY, 5, 2});
      sc::Demand d; d.Init({&a, &b});
      std::vector<float> c0(48000), c1(48000);
      for (int i = 0; i < 750; ++i) {
        float* outs[2] = {c0.data() + i * 64, c1.data() + i * 64};
        d.ProcessBlock(trig.data() + i * 64, 0.f, outs, 64);
      }
      if (!ref_check("DemandMixed", c0.data(), 48000)) ++g_failures; }
    { sc::RGen r; r.init(sc::kGoldenSeed);
      sc::DRand s({INFINITY, 10, 20, 30}, &r); run("DemandDrand", &s); }
    { sc::RGen r; r.init(sc::kGoldenSeed);
      sc::DXRand s({INFINITY, 1, 2, 3, 4}, &r); run("DemandDxrand", &s); }
    { sc::RGen r; r.init(sc::kGoldenSeed);
      sc::DBrown s({INFINITY, 0, 1, 0.1f}, &r); run("DemandDbrown", &s); }
  }

  printf("== T-D2 Task 12 buffer UGens vs NRT references ==\n");
  {
    const float values[] = {48000.f, 750.f, 128.f, 1.f, 48000.f};
    const char* names[] = {"SampleRate", "ControlRate", "BufFrames",
                           "BufRateScale", "BufSampleRate"};
    for (int k = 0; k < 5; ++k) {
      std::vector<float> out(48000, values[k]);
      if (!ref_check(names[k], out.data(), 48000)) ++g_failures;
    }
    { sc::HostBuffer b; b.Init(128, 1);
      sc::BufWr wr; wr.Init(&b); sc::BufRd rd; rd.Init(&b, 1);
      std::vector<float> ph(48000, 5.f), sig(48000, 0.42f), out(48000);
      for (int i = 0; i < 750; ++i) {
        wr.ProcessBlock(ph.data() + i * 64, sig.data() + i * 64, 64, 0);
        rd.ProcessBlock(ph.data() + i * 64, out.data() + i * 64, 64, 0);
      }
      if (!ref_check("BufWrRd1", out.data(), 48000)) ++g_failures; }
    { sc::HostBuffer b; b.Init(128, 1);
      sc::BufWr wr; wr.Init(&b);
      std::vector<float> ph(48000);
      for (int k = 0; k < 4; ++k) {
        const float p = static_cast<float>(4 + k);
        std::fill(ph.begin(), ph.end(), p);
        std::vector<float> v(48000, 0.1f * (k + 1));
        wr.ProcessBlock(ph.data(), v.data(), 48000, 0);
      }
      { sc::BufRd rd; rd.Init(&b, 2);
        std::fill(ph.begin(), ph.end(), 5.f);
        std::vector<float> out(48000);
        rd.ProcessBlock(ph.data(), out.data(), 48000, 0);
        if (!ref_check("BufWrRd2", out.data(), 48000)) ++g_failures; }
      { sc::BufRd rd; rd.Init(&b, 4);
        std::fill(ph.begin(), ph.end(), 5.f);
        std::vector<float> out(48000);
        rd.ProcessBlock(ph.data(), out.data(), 48000, 0);
        if (!ref_check("BufWrRd4", out.data(), 48000)) ++g_failures; }
    }
    { sc::DemandBuffer b; b.Init(8); b.data[5] = 0.42f;
      sc::DBufRd rd(&b, 5.f, 0);
      std::vector<float> trig(48000), out(48000);
      { sc::Impulse s; s.Init(48000.0, 10.f);
        for (int i = 0; i < 750; ++i) s.ProcessBlock(trig.data() + i*64, 64); }
      sc::Demand d; d.Init({&rd});
      for (int i = 0; i < 750; ++i) {
        float* o[1] = {out.data() + i*64};
        d.ProcessBlock(trig.data() + i*64, 0.f, o, 64);
      }
      if (!ref_check("DemandBufRd", out.data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 EnvGen vs NRT reference ==\n");
  {
    const sc::EnvGen::Node nodes[2] = {{1.f,0.01f,1.f,0.f},
                                       {0.f,0.02f,1.f,0.f}};
    sc::EnvGen env; env.Init(750.0, nodes, 2, 0.f);
    std::vector<float> trig(48000), out(48000);
    for (int b = 0; b < 750; ++b) trig[b*64] = (b == 0 || b == 376) ? 1.f : 0.f;
    // kr: one EnvGen step per control block; K2A SSE ramp.
    float prev = env.ctor_value();
    for (int b = 0; b < 750; ++b) {
      const float v = env.ProcessKr(trig[b*64]);
      const float slope = static_cast<float>((v - prev) * (1.0/64.0));
      const float s2 = slope + slope;
      // K2A SSE ramp: lanes [s, s+sl, s+2sl, s+2sl+sl], group += 4*sl.
      float acc = 0.f;
      for (int g = 0; g < 16; ++g) {
        out[b*64+g*4+0] = prev + acc;
        out[b*64+g*4+1] = prev + slope + acc;
        out[b*64+g*4+2] = prev + s2 + acc;
        out[b*64+g*4+3] = prev + s2 + slope + acc;
        acc += s2 + s2;
      }
      prev = v;
    }
    if (!ref_check("EnvGenkr", out.data(), 48000)) ++g_failures;
  }

  printf("== T-D2 Task 16 hardware constants vs NRT references ==\n");
  {
    std::vector<float> out(48000);
    for (const char* name : {"MouseXkr", "MouseYkr"}) {
      sc::MouseInput u; u.Init(48000.0);
      { float prime; u.ProcessBlock(&prime, 1, 0.5f, 0.f, 1.f, 0.f, 0.f); }
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(out.data()+i*64, 64, 0.5f, 0.f, 1.f, 0.f, 0.2f);
      if (!ref_check(name, out.data(), 48000)) ++g_failures;
    }
  }

  printf("== T-D2 Task 15 sugar vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> a(48000), b(48000), out(48000);
    { sc::Saw s; s.Init(&kt, 48000.0, 220.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(a.data()+i*64, 64); }
    { sc::Saw s; s.Init(&kt, 48000.0, 330.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(b.data()+i*64, 64); }
    { sc::SelectX sx; sx.Init(&kt);
      for (int i = 0; i < 750; ++i) {
        const float* v[2] = {a.data()+i*64, b.data()+i*64};
        sx.ProcessBlock(0.3f, v, 2, out.data()+i*64, 64);
      }
      if (!ref_check("SelectXar", out.data(), 48000)) ++g_failures; }
    { std::vector<float> trig(48000);
      { sc::Impulse s; s.Init(48000.0, 10.f);
        for (int i = 0; i < 750; ++i) s.ProcessBlock(trig.data()+i*64, 64); }
      sc::RGen r; r.init(sc::kGoldenSeed);
      sc::TChoose tc; tc.Init(&r, 3);
      const float vals[3] = {1.f, 2.f, 3.f};
      for (int i = 0; i < 750; ++i)
        tc.ProcessBlock(trig.data()+i*64, out.data()+i*64, 64, vals, 3);
      if (!ref_check("TChoosear", out.data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 Task 14 SC3P dynamics vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> a(48000), b(48000), out(48000);
    { sc::Saw s; s.Init(&kt, 48000.0, 220.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(a.data() + i*64, 64); }
    { sc::SinOsc s; s.Init(&kt, 48000.0, 330.0f);
      for (int i = 0; i < 750; ++i) s.ProcessBlock(b.data() + i*64, 64); }
    { sc::Decimator u; u.Init(48000.0, 12000.f, 8.f, a[0]);
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data()+i*64, out.data()+i*64, 64, 12000.f, 8.f);
      if (!ref_check("Decimator", out.data(), 48000)) ++g_failures; }
    { sc::CrossoverDistortion u; u.Init(48000.0, 0.5f, 0.9f, a[0]);
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data()+i*64, out.data()+i*64, 64, 0.5f, 0.9f);
      if (!ref_check("CrossoverDis", out.data(), 48000)) ++g_failures; }
    { sc::SineShaper u;
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data()+i*64, out.data()+i*64, 64, 0.5f);
      if (!ref_check("SineShaper", out.data(), 48000)) ++g_failures; }
    { sc::DiodeRingMod u;
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data()+i*64, b.data()+i*64, out.data()+i*64, 64);
      if (!ref_check("DiodeRingMod", out.data(), 48000)) ++g_failures; }
    { sc::InsideOut u;
      for (int i = 0; i < 750; ++i)
        u.ProcessBlock(a.data()+i*64, out.data()+i*64, 64);
      if (!ref_check("InsideOut", out.data(), 48000)) ++g_failures; }
  }

  printf("== T-D2 dynamics kit vs NRT references ==\n");
  {
    static sc::KitTables kt; sc::InitKitTables(&kt);
    std::vector<float> in(48000), ctrl(48000), hot(48000), out(48000);
    { sc::Saw src; src.Init(&kt, 48000.0, 220.0f);
      for (int b = 0; b < 750; ++b) src.ProcessBlock(in.data() + b * 64, 64); }
    { sc::Saw src; src.Init(&kt, 48000.0, 1.0f);
      for (int b = 0; b < 750; ++b) src.ProcessBlock(ctrl.data() + b * 64, 64); }
    for (int i = 0; i < 48000; ++i) hot[i] = in[i] * 2.0f;

    { sc::Amplitude u; u.Init(48000.0, 0.01f, 0.1f, in[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, out.data() + b * 64, 64);
      if (!ref_check("Amplitude", out.data(), 48000)) ++g_failures; }
    { sc::Compander u; u.Init(48000.0, 0.5f, 2.0f, 1.5f, 0.01f, 0.1f,
                              in[0], ctrl[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(in.data() + b * 64, ctrl.data() + b * 64,
                       out.data() + b * 64, 64);
      if (!ref_check("Compander", out.data(), 48000)) ++g_failures; }
    { sc::Limiter u; u.Init(48000.0, 1.0f, 0.01f, hot[0]);
      for (int b = 0; b < 750; ++b)
        u.ProcessBlock(hot.data() + b * 64, out.data() + b * 64, 64);
      if (!ref_check("Limiter", out.data(), 48000)) ++g_failures; }

    // Edge vectors: the source's zero-time coef short-circuit and the
    // ctor's one-sample priming (all output silent until flips >= 2).
    CHECK(sc::DynTimeCoef(0.0f, 48000.0) == 0.0f,
          "dyn: zero time -> coef 0 (log1 division guarded)");
    CHECK(sc::DynTimeCoef(0.01f, 48000.0) > 0.99f,
          "dyn: 10 ms attack coef ~0.9952");
    { sc::Limiter u; u.Init(48000.0, 1.0f, 0.01f, hot[0]);
      // ctor consumes one sample (pos=1), so the second flip lands on output
      // sample 959 — that one is already a delayed, level-scaled value.
      std::vector<float> o(959);
      u.ProcessBlock(hot.data(), o.data(), 959);
      bool silent = true;
      for (int i = 0; i < 959; ++i) if (o[i] != 0.0f) silent = false;
      CHECK(silent, "dyn: limiter silent until two buffers have flipped");
    }
  }

  // task sections are inserted above this line

  printf("== T-D2 lfo kit vs NRT references ==\n");
  {
    std::vector<float> out;
    // .ar forms (canonical params 440 Hz / duty 0.3 from gen_ugen_ref.scd)
    { sc::LFSaw u; u.Init(48000.0, 440.0f); if (!ref_ar_fill("LFSawar", u, out)) ++g_failures; }
    { sc::LFPar u; u.Init(48000.0, 440.0f); if (!ref_ar_fill("LFParar", u, out)) ++g_failures; }
    { sc::LFCub u; u.Init(48000.0, 440.0f); if (!ref_ar_fill("LFCubar", u, out)) ++g_failures; }
    { sc::LFPulse u; u.Init(48000.0, 440.0f, 0.0f, 0.3f); if (!ref_ar_fill("LFPulsear", u, out)) ++g_failures; }
    { sc::VarSaw u; u.Init(48000.0, 440.0f, 0.0f, 0.3f); if (!ref_ar_fill("VarSawar", u, out)) ++g_failures; }
    { sc::Impulse u; u.Init(48000.0, 440.0f); if (!ref_ar_fill("Impulsear", u, out)) ++g_failures; }
    // .kr forms (K2A ramp): 5 Hz / 10 Hz at the control rate (750)
    { sc::LFSaw u; u.Init(750.0, 5.0f); if (!ref_kr_fill("LFSawkr", u, out)) ++g_failures; }
    { sc::LFPar u; u.Init(750.0, 5.0f); if (!ref_kr_fill("LFParkr", u, out)) ++g_failures; }
    { sc::LFPulse u; u.Init(750.0, 5.0f, 0.0f, 0.3f); if (!ref_kr_fill("LFPulsekr", u, out)) ++g_failures; }
    { sc::Impulse u; u.Init(750.0, 10.0f); if (!ref_kr_fill("Impulsekr", u, out)) ++g_failures; }
    { sc::Phasor u; u.Init(750.0, 0.001, 0.0, 1.0, 0.0f); if (!ref_kr_fill("Phasorkr", u, out)) ++g_failures; }
    { // Clip.kr(SinOsc.kr(5), -0.5, 0.5): the input is a kr SinOsc ramped by K2A,
      // then clamped — replicate the whole chain.
      sc::SinOsc osc; static sc::KitTables kt_local; sc::InitKitTables(&kt_local);
      osc.Init(&kt_local, 750.0, 5.0f);
      out.assign(48000, 0.0f);
      // Clip.kr reads its kr input as a VALUE (no ramp), clips, then K2A
      // ramps the CLIPPED stream across the block.
      float prev = 0.0f;  // SinOsc ctor value (phase 0 -> 0)
      for (int b = 0; b < 750; ++b) {
        const float v = sc::ClipTo(osc.Process(), -0.5f, 0.5f);
        const float slope = static_cast<float>((v - prev) * (1.0 / 64.0));
        const float s2 = slope + slope;
        const float v0 = prev, v1 = prev + slope, v2 = prev + s2, v3 = prev + s2 + slope;
        float acc = 0.0f;
        for (int g = 0; g < 16; ++g) {
          out[b * 64 + g * 4 + 0] = v0 + acc;
          out[b * 64 + g * 4 + 1] = v1 + acc;
          out[b * 64 + g * 4 + 2] = v2 + acc;
          out[b * 64 + g * 4 + 3] = v3 + acc;
          acc += s2 + s2;
        }
        prev = v;
      }
      if (!ref_check("Clipkr", out.data(), 48000)) ++g_failures;
    }
    { // Fold.kr(SinOsc.kr(5) * 3, -0.5, 0.5)
      sc::SinOsc osc; static sc::KitTables kt_local2; sc::InitKitTables(&kt_local2);
      osc.Init(&kt_local2, 750.0, 5.0f);
      out.assign(48000, 0.0f);
      float prev = 0.0f;
      for (int b = 0; b < 750; ++b) {
        const float vin = sc::FoldTo(osc.Process() * 3.0f, -0.5f, 0.5f);
        const float slope = static_cast<float>((vin - prev) * (1.0 / 64.0));
        const float s2 = slope + slope;
        const float v0 = prev, v1 = prev + slope, v2 = prev + s2, v3 = prev + s2 + slope;
        float acc = 0.0f;
        for (int g = 0; g < 16; ++g) {
          out[b * 64 + g * 4 + 0] = v0 + acc;
          out[b * 64 + g * 4 + 1] = v1 + acc;
          out[b * 64 + g * 4 + 2] = v2 + acc;
          out[b * 64 + g * 4 + 3] = v3 + acc;
          acc += s2 + s2;
        }
        prev = vin;
      }
      if (!ref_check("Foldkr", out.data(), 48000)) ++g_failures;
    }
    { std::vector<float> consts(48000, 0.25f);
      if (!ref_check("DCkr", consts.data(), 48000)) ++g_failures; }
    { // A2K.kr(SinOsc.ar(440)): first ar sample per block, K2A-ramped
      sc::SinOsc osc; static sc::KitTables kt_local3; sc::InitKitTables(&kt_local3);
      osc.Init(&kt_local3, 48000.0, 440.0f);
      out.assign(48000, 0.0f);
      float prev = 0.0f;
      float blk[64];
      for (int b = 0; b < 750; ++b) {
        osc.ProcessBlock(blk, 64);
        const float v = blk[0];
        const float slope = static_cast<float>((v - prev) * (1.0 / 64.0));
        const float s2 = slope + slope;
        const float v0 = prev, v1 = prev + slope, v2 = prev + s2, v3 = prev + s2 + slope;
        float acc = 0.0f;
        for (int g = 0; g < 16; ++g) {
          out[b * 64 + g * 4 + 0] = v0 + acc;
          out[b * 64 + g * 4 + 1] = v1 + acc;
          out[b * 64 + g * 4 + 2] = v2 + acc;
          out[b * 64 + g * 4 + 3] = v3 + acc;
          acc += s2 + s2;
        }
        prev = v;
      }
      if (!ref_check("A2Kar", out.data(), 48000)) ++g_failures;
    }
  }

  printf("== T-D2 noise kit vs NRT references ==\n");
  {
    std::vector<float> out;
    int fails_before = g_failures;
    sc::RGen r;

    r.init(sc::kGoldenSeed); { sc::WhiteNoise u; u.Init(&r); if (!ref_ar_fill("WhiteNoise", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::ClipNoise u; u.Init(&r); if (!ref_ar_fill("ClipNoise", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::GrayNoise u; u.Init(&r); if (!ref_ar_fill("GrayNoise", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::PinkNoise u; u.Init(&r); if (!ref_ar_fill("PinkNoise", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::BrownNoise u; u.Init(&r); if (!ref_ar_fill("BrownNoise", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::Crackle u; u.Init(&r, 1.95f); if (!ref_ar_fill("Crackle", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::Dust u; u.Init(&r, 48000.0, 1000.0f); if (!ref_ar_fill("Dust", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::Dust2 u; u.Init(&r, 48000.0, 1000.0f); if (!ref_ar_fill("Dust2", u, out)) ++g_failures; }

    // LF noise: kr(10) at the control rate (sr/64 = 750) and ar(400).
    r.init(sc::kGoldenSeed); { sc::LFNoise0 u; u.Init(&r, 750.0, 10.0f); if (!ref_kr_fill("LFNoise0kr", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFNoise1 u; u.Init(&r, 750.0, 10.0f); if (!ref_kr_fill("LFNoise1kr", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFNoise2 u; u.Init(&r, 750.0, 10.0f); if (!ref_kr_fill("LFNoise2kr", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFNoise0 u; u.Init(&r, 48000.0, 400.0f); if (!ref_ar_fill("LFNoise0ar", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFNoise1 u; u.Init(&r, 48000.0, 400.0f); if (!ref_ar_fill("LFNoise1ar", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFNoise2 u; u.Init(&r, 48000.0, 400.0f); if (!ref_ar_fill("LFNoise2ar", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFClipNoise u; u.Init(&r, 48000.0, 400.0f); if (!ref_ar_fill("LFClipNoise", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFDNoise0 u; u.Init(&r, 750.0, 3.0f); if (!ref_kr_fill("LFDNoise0kr", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFDNoise0 u; u.Init(&r, 48000.0, 400.0f); if (!ref_ar_fill("LFDNoise0ar", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFDNoise3 u; u.Init(&r, 750.0, 3.0f); if (!ref_kr_fill("LFDNoise3kr", u, out)) ++g_failures; }
    r.init(sc::kGoldenSeed); { sc::LFDNoise3 u; u.Init(&r, 48000.0, 400.0f); if (!ref_ar_fill("LFDNoise3ar", u, out)) ++g_failures; }

    // Init-time randoms: the reference is a constant; compare the single value.
    {
      int fails = 0;
      r.init(sc::kGoldenSeed); const float rv = sc::RandIrg(&r, 0.25f, 0.75f);
      r.init(sc::kGoldenSeed); const float iv = static_cast<float>(sc::IRandIrg(&r, 0, 100));
      r.init(sc::kGoldenSeed); const float nv = sc::NRandIrg(&r, 0.0f, 1.0f, 4);
      r.init(sc::kGoldenSeed); const float lv = sc::LinRandIrg(&r, 0.0f, 1.0f, 0);
      r.init(sc::kGoldenSeed); const float ev = sc::ExpRandIrg(&r, 0.1f, 10.0f);
      const struct { const char* n; float v; } cases[] = {
          {"Rand", rv}, {"IRand", iv}, {"NRand", nv}, {"LinRand", lv}, {"ExpRand", ev}};
      for (const auto& c : cases) {
        std::vector<float> consts(48000, c.v);
        if (!ref_check(c.n, consts.data(), 48000)) { fails++; ++g_failures; }
      }
    }
    (void)fails_before;
  }

  printf("== trio goldens (float32) ==\n");
  {
    // Schedule mirrors gen_goldens.scd EXACTLY: voice before the first
    // Process call (SC's \s_new is in block 0), gate off at block 2250
    // (sample 144000), 192000 frames, hz 55, amp 0.4, fade 1.0 s.
    struct GoldenCase { int id; const char* name; };
    const GoldenCase cases[] = {{22, "Sine"}, {23, "Square"}, {29, "Triangle"}};
    for (const GoldenCase& gc : cases) {
      static uint8_t mem[160 * 1024];
      sc::Engine e;
      e.Init(48000.0f, mem, sizeof(mem));
      e.set_amp(0.4f);
      e.set_fade(1.0f);
      e.set_hz_target(55.0f);
      e.SetDrone(sc::Catalog(gc.id));         // SC: the synth exists at t = 0

      const int total = 192000;
      const int gate_off = 144000;
      std::vector<float> pcm;
      pcm.reserve(static_cast<size_t>(2 * total));
      float l[16], r[16];
      int written = 0;
      while (written < total) {
        if (written == gate_off) e.SetDrone(nullptr);
        e.Process(l, r, 16);
        for (int i = 0; i < 16 && written < total; ++i, ++written) {
          pcm.push_back(l[i]);
          pcm.push_back(r[i]);
        }
      }
      char path[128];
      snprintf(path, sizeof(path), "build/dc_%s_got.wav", gc.name);
      l8test::WriteWavF32(path, pcm.data(), total, 2, 48000);
      char ref[128];
      snprintf(ref, sizeof(ref), "dronecaster_ref/goldens/%s.expected.wav", gc.name);
      GoldenReport rep;
      const bool ok = compare(ref, path, 1e-6, 0.999999, &rep);
      printf("  %s: %s\n", gc.name, ok ? "GOLDEN PASS" : "GOLDEN FAIL");
      if (!ok) ++g_failures;
    }
  }

  printf("== EnvAsr (Env.asr curve -4, kr; 3.14 ctor rollback) ==\n");
  {
    sc::EnvAsr e;
    e.Init(48000.0f, 1.0f, 1.0f);
    e.Gate(true);
    // 3.14.1: the ctor-priming call is rolled back, so the first rendered
    // control sample is the attack's step-0 value (0.0) — every plan (3.13)
    // vector appears one call later.
    CHECK(e.Next() == 0.0f, "env block0 == 0 (3.14 rollback)");
    CHECK(fabsf(e.Next() - 0.00541837746f) < 1e-7f, "env block1 == 0.005418377");
    for (int i = 2; i < 376; ++i) (void)e.Next();
    CHECK(fabsf(e.Next() - 0.881530404f) < 1e-6f, "env 0.5 s == 0.881530404");
    for (int i = 377; i < 751; ++i) (void)e.Next();
    CHECK(e.Next() == 1.0f, "env sustain == 1");
    e.Gate(false);
    CHECK(e.Next() == 1.0f, "release first block == 1");
    CHECK(fabsf(e.Next() - 0.994581640f) < 1e-6f, "release second == 0.99458164");
    for (int i = 2; i < 748; ++i) (void)e.Next();
    CHECK(fabsf(e.Next() - 0.000200077018f) < 1e-7f, "release block749");
    CHECK(fabsf(e.Next() - 9.97717434e-05f) < 1e-8f, "release block750");
    // The done-call outputs the curve's step-750 residue: a2 - b1 is
    // mathematically 0 but carries ~1e-15 of accumulated rounding (SC's
    // shape_Step outputs level before the done clamp; the rev-2 plan's
    // 3.13 model clamped to a hard 0 instead).
    CHECK(fabsf(e.Next()) < 1e-6f, "release end ~0");
    CHECK(e.done(), "env done");
  }

  printf("== socket engine ==\n");
  {
    static uint8_t mem[160 * 1024];           // tables 131,080 B + voice
    sc::Engine e;
    e.Init(48000.0f, mem, sizeof(mem));
    CHECK(strcmp(e.current_name(), "--") == 0, "idle: no current drone");

    e.SetDrone(sc::Catalog(22));              // Sine
    float l[16], r[16];
    e.Process(l, r, 16);
    CHECK(strcmp(e.current_name(), "Sine") == 0, "Sine starts immediately when idle");
    float pk = 0;
    for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    CHECK(pk < 0.01f, "attack ramp: near-silence at t=0");

    // 1.5 s in (block 4500): attack done (env 1.0). Take the max over
    // 100 blocks (~18 periods at 55 Hz) so a zero crossing cannot mask it.
    for (int b = 0; b < 4500; ++b) e.Process(l, r, 16);
    pk = 0;
    for (int b = 0; b < 100; ++b) {
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
    }
    CHECK(pk > 0.38f && pk < 0.42f, "sustain: peak ~amp 0.4");

    // down+up: gate off, quick fade -> silence gap -> idle, name cleared.
    // Skip the release tail (0.01 s = ~7 internal blocks) before measuring
    // the gap: the first release block legitimately outputs the full level.
    e.set_fade(0.01f);
    e.SetDrone(nullptr);
    for (int b = 0; b < 100; ++b) e.Process(l, r, 16);   // ~25 internal blocks
    float gap_pk = 0;
    for (int b = 0; b < 300; ++b) {
      e.Process(l, r, 16);
      for (int i = 0; i < 16; ++i) gap_pk = gap_pk > fabsf(l[i]) ? gap_pk : fabsf(l[i]);
    }
    CHECK(strcmp(e.current_name(), "--") == 0, "gate-off: env ended, idle again");
    CHECK(gap_pk < 1e-6f, "down+up: silence in the gap");

    // recast: PLAY releases + restarts the current drone. With fade 1 s the
    // release takes 3000 host blocks; measure the restart after it.
    e.set_fade(1.0f);
    e.SetDrone(sc::Catalog(29));              // Triangle
    e.Process(l, r, 16);
    e.Recast();
    pk = 0;
    for (int b = 0; b < 3100; ++b) {
      e.Process(l, r, 16);
      if (b >= 3000) {
        for (int i = 0; i < 16; ++i) pk = pk > fabsf(l[i]) ? pk : fabsf(l[i]);
      }
    }
    CHECK(strcmp(e.current_name(), "Triangle") == 0, "recast: same drone restarted");
    CHECK(pk > 0.01f, "recast: restarted voice sounding");
    CHECK(e.swap_count() >= 3, "swap counter advances");

    // SetDrone(nullptr) after a completed recast must clear the name.
    e.set_fade(0.01f);
    e.SetDrone(nullptr);
    for (int b = 0; b < 400; ++b) e.Process(l, r, 16);
    CHECK(strcmp(e.current_name(), "--") == 0, "second gate-off clears the name");
  }

  printf("== Pulse (BLIT) ==\n");
  {
    static sc::KitTables t;
    sc::InitKitTables(&t);
    sc::Pulse p;
    p.Init(&t, 48000.0, 55.0f, 0.5f);
    float buf[64];
    p.ProcessBlock(buf, 8);
    CHECK(buf[0] == 0.0f, "pulse sample0 == 0");
    CHECK(fabsf(buf[1] - 0.0214609262f) < 1e-7f, "pulse sample1 == 0.021460926");
    CHECK(fabsf(buf[2] - 0.0401634201f) < 1e-7f, "pulse sample2 == 0.040163420");
    CHECK(fabsf(buf[3] - 0.0533128008f) < 1e-7f, "pulse sample3 == 0.053312801");
    CHECK(fabsf(buf[4] - 0.0614881404f) < 1e-7f, "pulse sample4 == 0.061488140");
    CHECK(fabsf(buf[5] - 0.0637175143f) < 1e-7f, "pulse sample5 == 0.063717514");
    CHECK(fabsf(buf[6] - 0.0623465441f) < 1e-7f, "pulse sample6 == 0.062346544");
    CHECK(fabsf(buf[7] - 0.0577567518f) < 1e-7f, "pulse sample7 == 0.057756752");

    sc::Pulse q;
    q.Init(&t, 48000.0, 55.0f, 0.5f);
    float pk = 0.0f, prev = 0.0f;
    int zc = 0;
    for (int b = 0; b < 750; ++b) {                 // 1 s
      q.ProcessBlock(buf, 64);
      for (int i = 0; i < 64; ++i) {
        const float v = buf[i];
        if (fabsf(v) > fabsf(pk)) pk = v;
        if ((v >= 0.0f) != (prev >= 0.0f)) ++zc;
        prev = v;
      }
    }
    CHECK(fabsf(pk) > 1.0f && fabsf(pk) < 1.2f, "pulse 1 s peak ~1.08");
    CHECK(zc >= 110 && zc <= 116, "pulse 1 s zero crossings ~113");

    // Frequency-change path (crossfade): finite and bounded.
    sc::Pulse r;
    r.Init(&t, 48000.0, 55.0f, 0.5f);
    bool finite = true;
    float qpk = 0.0f;
    for (int b = 0; b < 100; ++b) {
      r.SetFreq(b < 50 ? 55.0f : 110.0f);
      r.ProcessBlock(buf, 64);
      for (int i = 0; i < 64; ++i) {
        if (!std::isfinite(buf[i])) finite = false;
        if (fabsf(buf[i]) > qpk) qpk = fabsf(buf[i]);
      }
    }
    CHECK(finite && qpk < 2.5f, "pulse crossfade path finite/bounded");
  }

  printf("== kit tables + arena ==\n");
  {
    static sc::KitTables t;
    sc::InitKitTables(&t);
    CHECK(t.sine[0] == 0.0f, "sine[0] == 0");
    CHECK(t.sine[sc::kSineSize / 4] == 1.0f, "sine[2048] == 1");
    CHECK(t.sine[1024] == 0.707106769f, "sine[1024] exact");
    CHECK(t.sine[1] == 0.000766990299f, "sine[1] exact");
    CHECK(fabsf(t.sine[sc::kSineSize]) < 1e-6f, "sine guard ~0");
    CHECK(t.sine_wt[0] == -0.000766990299f && t.sine_wt[1] == 0.000766990299f,
          "wavetable pair[0] exact");
    CHECK(t.cosecant[0] == sc::kBadValue && t.cosecant[1] == sc::kBadValue &&
              t.cosecant[8] == sc::kBadValue,
          "cosecant sentinels (0 and +-8 bands)");
    CHECK(t.cosecant[9] == 144.867523f, "cosecant[9] exact (band ends at 8)");
    CHECK(fabsf(t.cosecant[100] - 1.0f / t.sine[100]) < 1e-3f, "cosecant = 1/sine");

    static uint8_t mem[64 * 1024];
    sc::Arena a;
    a.Init(mem, sizeof(mem));
    void* p1 = a.Alloc(16);
    const size_t mark = a.Mark();
    void* p2 = a.Alloc(64);
    CHECK(p1 != nullptr && p2 != nullptr && a.used() >= mark + 64, "arena alloc");
    a.ResetTo(mark);
    CHECK(a.used() == mark, "arena savepoint restores");
    a.Reset();
    CHECK(a.used() == 0, "arena reset");
  }

  printf("== SinOsc / LFTri ==\n");
  {
    static sc::KitTables t;
    sc::InitKitTables(&t);

    sc::SinOsc so;
    so.Init(&t, 48000.0, 55.0f);
    CHECK(so.Process() == 0.0f, "SinOsc sample0 == 0");
    CHECK(fabsf(so.Process() - 0.00719941361f) < 1e-7f, "SinOsc sample1 == 0.007199414");
    CHECK(fabsf(so.Process() - 0.0143984547f) < 1e-7f, "SinOsc sample2 == 0.014398455");
    CHECK(fabsf(so.Process() - 0.0215967484f) < 1e-7f, "SinOsc sample3 == 0.021596748");
    CHECK(so.phase() == 2460656u, "SinOsc phase after 4 samples == 2460656");
    sc::SinOsc so2;
    so2.Init(&t, 48000.0, 55.0f);
    for (int i = 0; i < 48000; ++i) (void)so2.Process();
    CHECK(so2.phase() == 3758068224u, "SinOsc phase after 48000 samples exact");

    sc::LFTri tri;
    tri.Init(48000.0, 55.0f);
    // 3.14.1 LFTri_Ctor resets the phase after priming: sample0 = z(0) = 0.
    CHECK(tri.Process() == 0.0f, "LFTri sample0 == 0 (3.14 ctor rollback)");
    CHECK(fabsf(tri.Process() - 0.00458333315f) < 1e-9f, "LFTri sample1 == 0.004583333");
    CHECK(fabsf(tri.Process() - 0.00916666631f) < 1e-9f, "LFTri sample2 == 0.009166666");
    float pk = 0.0f, dc = 0.0f;
    for (int i = 0; i < 48000; ++i) {
      const float v = tri.Process();
      dc += v;
      if (fabsf(v) > fabsf(pk)) pk = v;
    }
    CHECK(fabsf(pk) > 0.99f && fabsf(pk) <= 1.0f, "LFTri peak ~1");
    CHECK(fabsf(dc / 48000.0f) < 1e-3f, "LFTri DC ~0");
  }

  printf("== sc::RGen (taus88) ==\n");
  {
    sc::RGen r;
    r.init(1);
    CHECK(r.s1 == 681917145u && r.s2 == 3671622764u && r.s3 == 237148337u,
          "init(1) state vector");
    CHECK(r.trand() == 3477746156u, "trand(1)[0] == 3477746156");
    CHECK(r.trand() == 3423392677u, "trand(1)[1] == 3423392677");
    CHECK(r.trand() == 4088519076u, "trand(1)[2] == 4088519076");

    sc::RGen a; a.init(1);
    CHECK(a.frand0() == 1.809725760f, "frand0(1)[0] == 1.80972576");
    CHECK(a.frand0() == 1.797070620f, "frand0(1)[1] == 1.79707062");
    sc::RGen b; b.init(1);
    CHECK(b.frand2() == 0.619451523f, "frand2(1)[0] == 0.619451523");
    sc::RGen c; c.init(1);
    CHECK(c.frand() == 0.809725761f, "frand(1)[0] == 0.809725761");
    sc::RGen d; d.init(1);
    CHECK(d.fcoin() == -1.0f, "fcoin(1)[0] == -1");

    sc::RGen e; e.init(7);
    CHECK(e.s1 == 4158996277u && e.s2 == 93880704u && e.s3 == 3512900957u,
          "init(7) state vector");
    CHECK(e.frand0() == 1.190675500f, "frand0(7)[0] == 1.1906755");
    CHECK(e.frand0() == 1.194282050f, "frand0(7)[1] == 1.19428205");
    sc::RGen f; f.init(7);
    CHECK(f.frand2() == -0.618649006f, "frand2(7)[0] == -0.618649006");

    sc::RGen g; g.init(sc::kGoldenSeed);
    CHECK(g.s1 == 2304607986u && g.s2 == 2065709127u && g.s3 == 2950309018u,
          "kGoldenSeed state vector");
    CHECK(g.trand() == 592871963u, "kGoldenSeed trand[0] == 592871963");
  }

  printf("test_dronecaster: %s (%d failures)\n",
         g_failures ? "FAIL" : "PASS", g_failures);
  return g_failures ? 1 : 0;
}
