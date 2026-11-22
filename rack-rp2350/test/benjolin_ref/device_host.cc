// The HOST side of the device-vs-host comparison: renders the adapter's
// boot+CC-injected state (the same CC list, the same SpecValue math) through
// the engine and writes the post-Clip16 int16 pairs RAW — diff against the
// device's `bjdump 64` capture.
//
// COMPILE WITH -ffp-contract=off: the M33 has no FMA, so the host must not
// contract a*b+c either — otherwise the two builds round differently and the
// comparison measures the compiler, not the port.
#include <cstdio>
#include <cmath>
#include "benjolin_engine.h"

// The spec table — COPIED from module_benjolin.h kCcSpecs (keep in sync).
struct CcSpec { float min, max, step; bool exp; };
static const CcSpec kSpecs[14] = {
    {20, 14000, 0, true},  {0.1, 14000, 0, true},  {0, 1, 0, false},
    {0.001, 1, 0, false},  {0.001, 1, 0, false},  {0.01, 9, 0, false},
    {0, 1, 0, false},      {20, 20000, 0, true},  {0.001, 1, 0, false},
    {0, 6, 1, false},      {0, 1, 1, false},      {0, 6, 1, false},
    {0, 1, 0, false},      {-1, 1, 0.001, false},
};
static float SpecValue(const CcSpec& sp, float cc) {
  const float n = cc * (1.0f / 127.0f);
  float v = sp.exp ? sp.min * powf(sp.max / sp.min, n)
                   : sp.min + (sp.max - sp.min) * n;
  if (sp.step > 0.0f) {
    v = sp.min + roundf((v - sp.min) / sp.step) * sp.step;
    v = v < sp.min ? sp.min : (v > sp.max ? sp.max : v);
  }
  return v;
}

int main() {
  // The user's norns settings, in RAW CC (the console `cc` injections).
  const float cc[14] = {0, 32, 114, 13, 0, 42, 123, 53, 95, 127, 0, 127, 76, 64};
  benjolin::Params p;
  for (int i = 0; i < 14; ++i) {
    float vals[1];
    (void)vals;
    // map through the SAME spec math, per param
  }
  p.freq1 = SpecValue(kSpecs[0], cc[0]);
  p.freq2 = SpecValue(kSpecs[1], cc[1]);
  p.scale = SpecValue(kSpecs[2], cc[2]);
  p.rungler1 = SpecValue(kSpecs[3], cc[3]);
  p.rungler2 = SpecValue(kSpecs[4], cc[4]);
  p.rungler_f = SpecValue(kSpecs[5], cc[5]);
  p.loop = SpecValue(kSpecs[6], cc[6]);
  p.filt_freq = SpecValue(kSpecs[7], cc[7]);
  p.q = SpecValue(kSpecs[8], cc[8]);
  p.gain = SpecValue(kSpecs[9], cc[9]);
  p.filter_type = SpecValue(kSpecs[10], cc[10]);
  p.out_signal = SpecValue(kSpecs[11], cc[11]);
  p.amp = SpecValue(kSpecs[12], cc[12]);
  p.pan = SpecValue(kSpecs[13], cc[13]);
  printf("params: f1=%.4g f2=%.4g filt=%.4g q=%.4g loop=%.4g gain=%.4g "
         "scale=%.4g rf=%.4g r1=%.4g r2=%.4g amp=%.4g\n",
         p.freq1, p.freq2, p.filt_freq, p.q, p.loop, p.gain, p.scale,
         p.rungler_f, p.rungler1, p.rungler2, p.amp);
  benjolin::Engine e;
  e.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191, p);
  const int kBlocks = 64;
  static int16_t buf[kBlocks * 16 * 2];
  for (int b = 0; b < kBlocks; ++b) {
    float l[16], r[16];
    e.Process16(l, r);
    for (int i = 0; i < 16; ++i) {
      buf[(b * 16 + i) * 2] =
          static_cast<int16_t>(l[i] * 16384.0f);  // the adapter's headroom law
      buf[(b * 16 + i) * 2 + 1] =
          static_cast<int16_t>(r[i] * 16384.0f);
    }
  }
  FILE* f = fopen("build/benj_host_ref.bin", "wb");
  fwrite(buf, sizeof(int16_t), kBlocks * 16 * 2, f);
  fclose(f);
  printf("wrote build/benj_host_ref.bin (%d samples)\n", kBlocks * 16);
  return 0;
}
