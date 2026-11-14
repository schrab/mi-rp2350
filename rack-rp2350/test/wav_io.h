// Minimal WAV I/O for the rack-lira8 host test harness (stdlib only).
// Write: float planar or interleaved buffers -> 16-bit PCM WAV.
// Read:  16/24/32-bit PCM WAV -> interleaved float [-1, 1).
#ifndef RACK_TEST_WAV_IO_H_
#define RACK_TEST_WAV_IO_H_

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace l8test {

inline void WriteWav16(const char* path, const float* interleaved,
                       int frames, int channels, int sample_rate) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  const int32_t data_bytes = frames * channels * 2;
  const uint8_t hdr[44] = {
      'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
      16, 0, 0, 0, 1, 0, static_cast<uint8_t>(channels), 0, 0, 0, 0, 0, 0, 0,
      0, 0, static_cast<uint8_t>(channels * 2), 0, 16, 0, 'd', 'a', 't', 'a',
      0, 0, 0, 0};
  uint8_t h[44];
  memcpy(h, hdr, 44);
  auto poke32 = [&](int off, int32_t v) {
    memcpy(h + off, &v, 4);
  };
  poke32(4, 36 + data_bytes);
  poke32(24, sample_rate);
  poke32(28, sample_rate * channels * 2);
  poke32(40, data_bytes);
  fwrite(h, 1, 44, f);
  for (int i = 0; i < frames * channels; ++i) {
    float cl = interleaved[i] < -1.0f ? -1.0f : (interleaved[i] > 1.0f ? 1.0f : interleaved[i]);
    int32_t s = static_cast<int32_t>(cl * 32767.0f);
    uint8_t b[2] = {static_cast<uint8_t>(s & 0xff), static_cast<uint8_t>((s >> 8) & 0xff)};
    fwrite(b, 1, 2, f);
  }
  fclose(f);
}

struct WavFile {
  std::vector<float> samples;  // interleaved
  int sample_rate = 0;
  int channels = 0;
};

inline WavFile ReadWav(const char* path) {
  WavFile out;
  FILE* f = fopen(path, "rb");
  if (!f) return out;
  uint8_t h[44];
  if (fread(h, 1, 44, f) != 44) { fclose(f); return out; }
  int channels = h[22] | (h[23] << 8);
  int sample_rate = h[24] | (h[25] << 8) | (h[26] << 16) | (h[27] << 24);
  int data_bytes = h[40] | (h[41] << 8) | (h[42] << 16) | (h[43] << 24);
  int bits = h[34] | (h[35] << 8);
  out.channels = channels;
  out.sample_rate = sample_rate;
  if (bits == 16) {
    std::vector<uint8_t> raw(data_bytes);
    size_t got = fread(raw.data(), 1, data_bytes, f);
    int n = static_cast<int>(got / 2);
    out.samples.resize(n);
    for (int i = 0; i < n; ++i) {
      int16_t s = static_cast<int16_t>(raw[2 * i] | (raw[2 * i + 1] << 8));
      out.samples[i] = static_cast<float>(s) / 32768.0f;
    }
  }
  fclose(f);
  return out;
}

}  // namespace l8test

#endif  // RACK_TEST_WAV_IO_H_
