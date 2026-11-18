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

// Float32 WAV (IEEE, fmt tag 3): little-endian host assumed (x86-64 host
// tests and the RP2350 M33 both are).
inline void WriteWavF32(const char* path, const float* interleaved,
                        int frames, int channels, int sample_rate) {
  FILE* f = fopen(path, "wb");
  if (!f) return;
  const int32_t data_bytes = frames * channels * 4;
  const uint8_t hdr[44] = {
      'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
      16, 0, 0, 0, 3, 0, static_cast<uint8_t>(channels), 0, 0, 0, 0, 0, 0, 0,
      0, 0, static_cast<uint8_t>(channels * 4), 0, 32, 0, 'd', 'a', 't', 'a',
      0, 0, 0, 0};
  uint8_t h[44];
  memcpy(h, hdr, 44);
  auto poke32 = [&](int off, int32_t v) { memcpy(h + off, &v, 4); };
  poke32(4, 36 + data_bytes);
  poke32(24, sample_rate);
  poke32(28, sample_rate * channels * 4);
  poke32(40, data_bytes);
  fwrite(h, 1, 44, f);
  fwrite(interleaved, 4, static_cast<size_t>(frames) * channels, f);
  fclose(f);
}

inline WavFile ReadWav(const char* path) {
  WavFile out;
  FILE* f = fopen(path, "rb");
  if (!f) return out;
  // Walk the RIFF chunks: scsynth writes fmt + fact + PEAK before data, so
  // a fixed 44-byte header read would grab the fact payload as data_bytes.
  uint8_t h[12];
  if (fread(h, 1, 12, f) != 12 ||
      memcmp(h, "RIFF", 4) != 0 || memcmp(h + 8, "WAVE", 4) != 0) {
    fclose(f);
    return out;
  }
  int format = 0, channels = 0, sample_rate = 0, bits = 0;
  int data_bytes = 0;
  bool have_data = false;
  while (true) {
    uint8_t ch[8];
    if (fread(ch, 1, 8, f) != 8) break;
    const int sz = ch[4] | (ch[5] << 8) | (ch[6] << 16) | (static_cast<int>(ch[7]) << 24);
    if (memcmp(ch, "fmt ", 4) == 0 && sz >= 16) {
      uint8_t fd[16];
      if (fread(fd, 1, 16, f) != 16) break;
      format = fd[0] | (fd[1] << 8);
      channels = fd[2] | (fd[3] << 8);
      sample_rate = fd[4] | (fd[5] << 8) | (fd[6] << 16) | (fd[7] << 24);
      bits = fd[14] | (fd[15] << 8);
      fseek(f, sz - 16, SEEK_CUR);
    } else if (memcmp(ch, "data", 4) == 0) {
      data_bytes = sz;
      have_data = true;
      break;  // data payload starts here
    } else {
      fseek(f, static_cast<long>(sz) + (sz & 1), SEEK_CUR);
    }
  }
  out.channels = channels;
  out.sample_rate = sample_rate;
  if (have_data && bits == 32 && format == 3) {
    std::vector<uint8_t> raw(static_cast<size_t>(data_bytes));
    const size_t got = fread(raw.data(), 1, static_cast<size_t>(data_bytes), f);
    const int n = static_cast<int>(got / 4);
    out.samples.resize(static_cast<size_t>(n));
    memcpy(out.samples.data(), raw.data(), static_cast<size_t>(n) * 4);
  } else if (have_data && bits == 16) {
    std::vector<uint8_t> raw(static_cast<size_t>(data_bytes));
    const size_t got = fread(raw.data(), 1, static_cast<size_t>(data_bytes), f);
    const int n = static_cast<int>(got / 2);
    out.samples.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const int16_t s = static_cast<int16_t>(raw[2 * i] | (raw[2 * i + 1] << 8));
      out.samples[i] = static_cast<float>(s) / 32768.0f;
    }
  }
  fclose(f);
  return out;
}

}  // namespace l8test

#endif  // RACK_TEST_WAV_IO_H_
