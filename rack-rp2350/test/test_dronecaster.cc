// Dronecaster host tests (T-D1). Sections are appended by plan task;
// the comparator is proven here against synthetic float32 cases so a later
// golden FAIL is trustworthy.
#include "wav_io.h"

#include "../src/dronecaster/sc_defs.h"
#include "../src/dronecaster/sc_rgen.h"

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
