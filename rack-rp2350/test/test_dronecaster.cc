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

  // task sections are inserted above this line

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
