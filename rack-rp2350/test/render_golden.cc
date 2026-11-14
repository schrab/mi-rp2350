// Render a LIRA-8 scenario through the C++ engine, mirroring the pd golden
// protocol (scenarios.txt) — events quantized to 64-sample blocks like pd's
// message handling. Output: 16-bit stereo WAV (mono duplicated) for
// compare_wav.py against the pd golden.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "../src/lira8/lira_engine.h"
#include "wav_io.h"

namespace {

struct Event {
  int t_ms;
  std::string kind, a, b;   // sensor N v | param NAME V
};

// param name -> engine index (spec §12 order)
int param_index(const std::string& n) {
  static const char* kNames[] = {
      "tune-1", "tune-2", "tune-3", "tune-4", "tune-5", "tune-6", "tune-7",
      "tune-8", "fast-12", "fast-34", "fast-56", "fast-78", "sharp-12",
      "sharp-34", "sharp-56", "sharp-78", "mod-12", "mod-34", "mod-56",
      "mod-78", "source-12", "source-34", "source-56", "source-78",
      "pitch-1234", "pitch-5678", "hold-1234", "hold-5678", "switch",
      "total-fb", "vibrato", "drv", "dst-mix", "vol", "f-a", "f-b", "andor",
      "link", "time-1", "time-2", "feedback", "del-mix", "mod-1", "mod-2",
      "del-mod", "lfo-wav", "quantize"};
  for (int i = 0; i < 47; ++i)
    if (n == kNames[i]) return i;
  return -1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: render_golden <scenario-name> <out.wav>\n");
    return 2;
  }
  const std::string scen = argv[1];
  const char* out_path = argv[2];

  // parse scenarios.txt
  FILE* f = fopen("lira8_ref/scenarios.txt", "r");
  if (!f) {
    fprintf(stderr, "cannot open scenarios.txt\n");
    return 2;
  }
  std::vector<Event> events;
  int end_ms = 0;
  bool active = false;
  char line[512];
  while (fgets(line, sizeof(line), f)) {
    std::string s = line;
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    if (s.empty() || s[0] == '#') continue;
    if (s.rfind("SCEN ", 0) == 0) {
      active = (s.substr(5) == scen);
      continue;
    }
    if (!active) continue;
    int t;
    char rest[480];
    if (sscanf(s.c_str(), "%d %[^\n]", &t, rest) != 2) continue;
    if (std::string(rest).rfind("render ", 0) == 0) continue;
    if (std::string(rest) == "stop") continue;
    Event e;
    e.t_ms = t;
    char kind[128] = "", a[128] = "", b[128] = "";
    if (sscanf(rest, "%127s %127s %127s", kind, a, b) < 2) continue;
    e.kind = kind;
    e.a = a;
    e.b = b;
    events.push_back(e);
    if (t > end_ms) end_ms = t;
  }
  fclose(f);
  // stop happens at last event + 1000 ms (per scenarios.txt)
  end_ms += 1000;

  l8pd::Lira8Engine eng(32000.0f, 1450.0f);
  const int kBlock = l8pd::Lira8Engine::kBlockSize;
  size_t evi = 0;
  const int total_blocks = (end_ms * 32 + kBlock - 1) / kBlock;
  std::vector<float> audio;
  audio.reserve(total_blocks * kBlock * 2);
  float l[kBlock], r[kBlock];
  for (int b = 0; b < total_blocks; ++b) {
    const int t_ms = b * kBlock * 1000 / 32000;
    while (evi < events.size() && events[evi].t_ms <= t_ms) {
      const Event& e = events[evi++];
      if (e.kind == "sensor") {
        eng.set_sensor(atoi(e.a.c_str()) - 1, e.b != "0");
      } else if (e.kind == "param") {
        int idx = param_index(e.a);
        if (idx < 0) {
          fprintf(stderr, "unknown param '%s'\n", e.a.c_str());
          return 2;
        }
        eng.set_param(idx, static_cast<float>(atof(e.b.c_str())));
      }
    }
    eng.Process(l, r, kBlock);
    for (int i = 0; i < kBlock; ++i) {
      audio.push_back(l[i]);
      audio.push_back(r[i]);
    }
  }
  l8test::WriteWav16(out_path, audio.data(),
                     static_cast<int>(audio.size() / 2), 2, 32000);
  printf("rendered %s (%zu frames)\n", out_path, audio.size() / 2);
  return 0;
}
