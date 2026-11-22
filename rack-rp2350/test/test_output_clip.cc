// Pin the standalone adapters' float -> I2S conversion convention.
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "stmlib/dsp/dsp.h"

static int16_t Convert(float sample, float gain) {
  return stmlib::Clip16(static_cast<int32_t>(sample * 32768.0f * gain));
}

// The MCU adapters cannot be included in this host test. Pin their actual
// conversion statements as well as the numerical contract, so a production
// regression cannot silently leave the duplicated host expression green.
static bool CheckAdapter(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    std::fprintf(stderr, "cannot read adapter: %s\n", path.c_str());
    return false;
  }
  const std::string source((std::istreambuf_iterator<char>(input)), {});
  std::string compact;
  for (unsigned char ch : source) {
    if (!std::isspace(ch)) compact += static_cast<char>(ch);
  }
  const std::string expected =
      "constint16_tout=stmlib::Clip16("
      "static_cast<int32_t>(l[i]*32768.0f*gain));i2s.write16(out,out);";
  const auto match = compact.find(expected);
  if (match == std::string::npos ||
      compact.find(expected, match + expected.size()) != std::string::npos) {
    std::fprintf(stderr, "adapter conversion changed: %s\n", path.c_str());
    return false;
  }
  return true;
}

int main() {
  const struct {
    float sample;
    float gain;
    int16_t expected;
  } cases[] = {
      {1.0f, 1.0f, 32767},
      {-1.0f, 1.0f, -32768},
      {2.0f, 1.0f, 32767},
      {-2.0f, 1.0f, -32768},
      {0.5f, 1.0f, 16384},
      {-0.5f, 1.0f, -16384},
      {0.0f, 1.0f, 0},
      {2.0f, 0.0f, 0},
      {2.0f, 0.5f, 32767},
      {-2.0f, 0.5f, -32768},
      {1.75f / 32768.0f, 1.0f, 1},
      {-1.75f / 32768.0f, 1.0f, -1},
      {0.75f / 32768.0f, 1.0f, 0},
      {-0.75f / 32768.0f, 1.0f, 0},
      {32767.0f / 32768.0f, 1.0f, 32767},
      {std::nextafter(1.0f, 0.0f), 1.0f, 32767},
      {std::nextafter(-1.0f, -2.0f), 1.0f, -32768},
      {32769.0f / 32768.0f, 1.0f, 32767},
      {-32769.0f / 32768.0f, 1.0f, -32768},
  };
  // Absolute source paths work from any cwd; relative __FILE__ follows the
  // existing test Makefile's convention of running from the test directory.
  const auto src = std::filesystem::path(__FILE__).parent_path() / "../src";
  int failures = !CheckAdapter(src / "module_lira8.h");
  failures += !CheckAdapter(src / "module_dronecaster.h");
  for (const auto& test : cases) {
    const int16_t actual = Convert(test.sample, test.gain);
    if (actual != test.expected) {
      std::fprintf(stderr, "sample=%.9g gain=%.9g: got %d, expected %d\n",
                   test.sample, test.gain, actual, test.expected);
      ++failures;
    }
  }
  std::printf("output clipping: %zu cases, %d failures\n",
              sizeof(cases) / sizeof(cases[0]), failures);
  return failures != 0;
}
