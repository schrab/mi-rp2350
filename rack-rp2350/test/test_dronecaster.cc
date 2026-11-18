// Dronecaster host tests (T-D1). Sections are appended by plan task;
// the comparator is proven here against synthetic float32 cases so a later
// golden FAIL is trustworthy.
#include "wav_io.h"

#include "../src/dronecaster/sc_defs.h"
#include "../src/dronecaster/sc_rgen.h"
#include "../src/dronecaster/sc_kit_osc.h"
#include "../src/dronecaster/sc_kit_pulse.h"
#include "../src/dronecaster/sc_arena.h"

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
