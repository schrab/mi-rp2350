// Self-test for the harness WAV I/O round trip.
#include <cmath>
#include <cstdio>
#include "wav_io.h"

using namespace l8test;

int main() {
  const int kFrames = 4800, kSr = 32000;
  std::vector<float> buf(kFrames * 2);
  for (int i = 0; i < kFrames; ++i) {
    // C-library sinf/cosf: std::sinf/std::cosf are not guaranteed by
    // C++17 and g++ 13.3 rejects them (surfaced by a stale-binary rebuild).
    buf[2 * i] = 0.5f * sinf(2.0f * 3.14159265f * 440.0f * i / kSr);
    buf[2 * i + 1] = 0.25f * cosf(2.0f * 3.14159265f * 440.0f * i / kSr);
  }
  WriteWav16("build/wav_io_roundtrip.wav", buf.data(), kFrames, 2, kSr);
  WavFile r = ReadWav("build/wav_io_roundtrip.wav");
  if (r.sample_rate != kSr || r.channels != 2 ||
      static_cast<int>(r.samples.size()) != kFrames * 2) {
    fprintf(stderr, "FAIL: sr=%d ch=%d n=%zu\n", r.sample_rate, r.channels,
            r.samples.size());
    return 1;
  }
  double err = 0.0;
  for (int i = 0; i < kFrames * 2; ++i) {
    double d = buf[i] - r.samples[i];
    err += d * d;
  }
  err = std::sqrt(err / (kFrames * 2));
  if (err > 1e-4) {  // 16-bit quantization floor ~ 1/32768/sqrt(3)
    fprintf(stderr, "FAIL: roundtrip rms err %g\n", err);
    return 1;
  }
  printf("wav_io roundtrip OK (rms err %.2e)\n", err);
  return 0;
}
