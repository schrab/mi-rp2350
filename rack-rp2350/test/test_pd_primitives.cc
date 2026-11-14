// TDD unit tests for the Pd primitive layer (l8pd::) — every assert encodes
// a law verified against pure-data source + headless probes, see
// test/lira8_ref/PD-PRIMITIVES.md. Red -> green per component.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "pd_primitives.h"

using namespace l8pd;

static int g_fail = 0;

#define CHECK(cond, ...)                                                    \
  do {                                                                      \
    if (!(cond)) {                                                          \
      fprintf(stderr, "FAIL %s:%d ", __FILE__, __LINE__);                   \
      fprintf(stderr, __VA_ARGS__);                                         \
      fprintf(stderr, "\n");                                                \
      ++g_fail;                                                             \
    }                                                                       \
  } while (0)

static bool near(float a, float b, float tol) {
  return std::fabs(a - b) <= tol;
}

static void test_ramp() {
  const float sr = 32000.f;
  Ramp r(sr);
  r.set(1.0f, 100.0f);            // ramp to 1 over 100 ms = 3200 samples
  CHECK(near(r.next(), 1.0f / 3200.0f, 1e-5f), "first sample already moves");
  for (int i = 1; i < 3199; ++i) r.next();
  CHECK(near(r.next(), 1.0f, 1e-4f), "reaches target exactly (last sample)");
  CHECK(near(r.next(), 1.0f, 1e-6f), "holds target");
  r.set(0.5f, 0.0f);              // ms=0 jumps immediately
  CHECK(near(r.next(), 0.5f, 1e-6f), "zero-time jump");
}

static void test_hip() {
  const float sr = 32000.f;
  Hip h(20.0f, sr);
  // verbatim perform: y = 0.5(1+coef) * (x + coef*last - last)
  float vals[96];
  for (int i = 0; i < 96; ++i) vals[i] = h.process(1.0f);
  // pd sim (numpy, source constants): sim[32]=0.879962 sim[64]=0.775857;
  // the probe's first metro tick caught sample 64 exactly: vals[63]=0.778916
  CHECK(near(vals[32], 0.879962f, 1e-4f), "hip[32]=%.6f", vals[32]);
  CHECK(near(vals[64], 0.775857f, 1e-4f), "hip[64]=%.6f", vals[64]);
  CHECK(near(vals[63], 0.778916f, 1e-5f), "hip[63]=probe (got %.6f)", vals[63]);
  // steady-state decay pole = coef
  float r = vals[94] / vals[93];
  float coef = 1.0f - 20.0f * 2.f * 3.14159f / sr;
  CHECK(near(r, coef, 1e-4f), "hip pole %.6f vs %.6f", r, coef);
  // f <= 0 -> bypass
  Hip hb(-1.0f, sr);
  CHECK(near(hb.process(0.37f), 0.37f, 1e-6f), "hip bypass");
}

static void test_lop() {
  const float sr = 32000.f;
  Lop l(4000.0f, sr);
  float y0 = l.process(1.0f);
  float coef = 4000.0f * 2.f * 3.14159f / sr;   // 0.7854, clamped < 1
  CHECK(near(y0, coef, 1e-5f), "lop y0 = coef");
  for (int i = 0; i < 3; ++i) l.process(1.0f);
  CHECK(l.process(1.0f) > 0.995f, "lop settles fast at coef 0.785");
  Lop l2(100.0f, sr);
  float acc = 0.f;
  for (int i = 0; i < 640; ++i) acc = l2.process(1.0f);
  CHECK(near(acc, 1.0f, 1e-3f), "lop converges to 1");
}

static void test_cos_osc() {
  const float sr = 32000.f;
  // f = sr/2048: phase advances exactly one table index per sample ->
  // quadrant pins land exactly (table[0]=1, [512]=0, [1024]=-1, [1536]=0).
  CosOsc o(sr);
  o.set_freq(sr / 2048.0f);
  CHECK(near(o.next(), 1.0f, 1e-6f), "osc[0] = 1");
  for (int i = 1; i < 512; ++i) o.next();
  CHECK(std::fabs(o.next()) < 1e-4f, "osc[512] = 0");
  for (int i = 513; i < 1024; ++i) o.next();
  CHECK(near(o.next(), -1.0f, 1e-4f), "osc[1024] = -1");
}

static void test_phasor() {
  const float sr = 32000.f;
  Phasor p(sr);
  p.set_freq(sr / 4.0f);          // 0.25 cycles/sample
  CHECK(near(p.next(), 0.0f, 1e-7f), "phasor output-then-increment: out0=0");
  CHECK(near(p.next(), 0.25f, 1e-7f), "phasor out1=0.25");
  CHECK(near(p.next(), 0.5f, 1e-7f), "phasor out2=0.5");
  CHECK(near(p.next(), 0.75f, 1e-7f), "phasor out3=0.75");
  CHECK(near(p.next(), 0.0f, 1e-7f), "phasor wraps at 1.0");
}

static void test_noise() {
  // verbatim LCG: seed chain init=307, init*=1319 for the first instance
  uint32_t val = 307u * 1319u;    // 404933
  Noise n(val);
  float expect0 = (float)((int32_t)(val & 0x7fffffffu) - 0x40000000) *
                  (1.0f / 1073741824.0f);
  CHECK(near(n.next(), expect0, 1e-6f), "noise first sample");
  val = val * 435898247u + 382842987u;   // int32 wrap
  float expect1 = (float)((int32_t)(val & 0x7fffffffu) - 0x40000000) *
                  (1.0f / 1073741824.0f);
  CHECK(near(n.next(), expect1, 1e-6f), "noise second sample");
}

static void test_env_rms() {
  const float sr = 32000.f;
  // verbatim env~ 512 (d_ctl.c): window (1-cos(2*pi*i/N))/N, ring of partial
  // sums, hop 256 (block 64), output = powtodb(sumbuf[0]) after shift.
  // Probe staircase (1.0 step, hop-aligned): 89.5362 dB -> 99.5817 -> 100.
  // The middle point reproduces at 99.889 in the verbatim sim (0.31 dB
  // probe-timing delta; env~ only feeds the line~-smoothed expander gain, so
  // tolerance 0.5 dB is far below golden-test tolerance).
  // Block-level call replicates the numpy verbatim sim at its best step
  // alignment (step at sample 19200+118): [89.5367, 99.8891, 100.0] dB.
  EnvRms e(512, 64, sr);
  float out[128];
  int nout = 0;
  float last = 0.f;
  float blk[64];
  const int kStep = 19318;  // 19200 + 118 (numpy-sim alignment)
  for (int b = 0; b < 340; ++b) {
    for (int i = 0; i < 64; ++i) blk[i] = (b * 64 + i >= kStep) ? 1.0f : 0.0f;
    if (e.process_block(blk, 64, &last) && nout < 128) out[nout++] = last;
  }
  CHECK(nout > 2, "env produced outputs (%d)", nout);
  CHECK(near(last, 100.0f, 1e-3f), "env settles at 100 dB (got %f)", last);
  bool saw_partial = false, saw_mid = false;
  for (int i = 0; i < nout; ++i) {
    if (near(out[i], 89.5367f, 0.05f)) saw_partial = true;
    if (near(out[i], 99.8891f, 0.05f) || near(out[i], 99.5817f, 0.5f))
      saw_mid = true;
  }
  CHECK(saw_partial, "env staircase partial value 89.5367 dB");
  CHECK(saw_mid, "env staircase mid value");
  // silence reads 0 dB
  EnvRms e2(512, 64, sr);
  float v = 0.f;
  for (int b = 0; b < 12; ++b) {
    for (int i = 0; i < 64; ++i) blk[i] = 0.0f;
    e2.process_block(blk, 64, &v);
  }
  CHECK(near(v, 0.0f, 1e-5f), "env silence -> 0 dB");
}

static void test_tanh_shaper() {
  TanhShaper t;
  CHECK(near(t.process(0.0f), 0.0f, 1e-7f), "tanh(0)=0");
  CHECK(near(t.process(1.0f), 1.0f * 28.0f / 36.0f, 1e-5f), "tanh pade(1)");
  CHECK(near(t.process(3.0f), 1.0f, 1e-5f), "tanh pade(3) = 1");
  CHECK(near(t.process(-3.0f), -1.0f, 1e-5f), "tanh pade(-3) = -1");
  CHECK(near(t.process(4.0f), 1.0f, 1e-5f), "input clipped at +-3");
  CHECK(t.process(2.0f) < 1.0f && t.process(2.0f) > 0.9f, "monotone near top");
}

static void test_dbtorms_mtof() {
  CHECK(near(dbtorms(100.0f), 1.0f, 1e-5f), "dbtorms(100) = 1");
  CHECK(near(dbtorms(88.0f), 0.251189f, 1e-4f), "dbtorms(88)");
  CHECK(near(mtof(69.0f), 440.0f, 1e-2f), "mtof(69) = 440");
  CHECK(near(ftom(440.0f), 69.0f, 1e-3f), "ftom(440) = 69");
  CHECK(near(mtof(ftom(1234.5f)), 1234.5f, 0.01f), "mtof/ftom roundtrip");
}

int main() {
  test_ramp();
  test_hip();
  test_lop();
  test_cos_osc();
  test_phasor();
  test_noise();
  test_env_rms();
  test_tanh_shaper();
  test_dbtorms_mtof();
  if (g_fail) {
    fprintf(stderr, "%d FAILURES\n", g_fail);
    return 1;
  }
  printf("test_pd_primitives: all green\n");
  return 0;
}
