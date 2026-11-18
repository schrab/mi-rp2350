// Quick probe: RMS gain of each rackdist algorithm vs drive, on a
// resonator-like two-tone input. Used to pick the FX-on level trim.
#include "rack_dist.h"
#include <cmath>
#include <cstdio>
#include <cstring>

int main() {
  const size_t N = 8192;
  static float in[N], out[N], aux[N];
  for (size_t i = 0; i < N; ++i) {
    const float t = (float)i / 32000.0f;
    in[i] = 0.15f * sinf(2.0f * 3.14159265f * 220.0f * t) +
            0.08f * sinf(2.0f * 3.14159265f * 440.0f * t);
  }
  float rin = 0.0f;
  for (size_t i = 0; i < N; ++i) rin += in[i] * in[i];
  rin = sqrtf(rin / N);
  printf("input RMS %.4f\n", rin);
  const char* names[5] = {"TUBE", "FOLD", "FUZZ", "SHAPE", "SWEDE"};
  const float drives[3] = {0.25f, 0.55f, 1.0f};
  for (int type = 0; type < 5; ++type) {
    for (float d : drives) {
      rackdist::State st{{0.0f, 0.0f}};
      rackdist::Params p{};
      p.type = (uint8_t)type; p.drive = d; p.bias = 0.0f; p.tone = 1.0f;
      memcpy(out, in, sizeof(in));
      memset(aux, 0, sizeof(aux));
      rackdist::process(&st, p, out, aux, N);
      float rms = 0.0f, peak = 0.0f;
      for (size_t i = 0; i < N; ++i) {
        rms += out[i] * out[i];
        float a = fabsf(out[i]); if (a > peak) peak = a;
      }
      rms = sqrtf(rms / N);
      printf("%-5s d=%.2f  RMS %.4f  gain %.2fx  peak %.3f\n",
             names[type], d, rms, rms / rin, peak);
    }
  }
  return 0;
}
