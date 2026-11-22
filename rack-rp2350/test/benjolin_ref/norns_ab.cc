// The norns A/B structure check: render the engine at the user's exact
// norns settings (freq1 20, freq2 1.93, filtFreq 364.8, type 0, loop 0.97,
// runglerFilt 2.98, q 0.75, rungler1 0.1, rungler2 0, scale 0.9, out 6,
// gain 6, amp 0.6) and print the rungler DAC trajectory — the "6 notes
// arpeggios every 5 seconds" = the register stepping pattern.
#include <cstdio>
#include <cmath>
#include "benjolin_engine.h"

int main() {
  benjolin::Params p;
  p.freq1 = 20.0f;  p.freq2 = 1.93f;  p.scale = 0.9f;
  p.rungler1 = 0.1f; p.rungler2 = 0.0f; p.rungler_f = 2.98f;
  p.loop = 0.97f; p.filt_freq = 364.8f; p.q = 0.75f;
  p.gain = 6.0f; p.filter_type = 0.0f; p.out_signal = 6.0f;
  p.amp = 0.6f; p.pan = 0.0f;
  benjolin::Engine e2;
  e2.Init(48000.0, benjolin::FeedbackMode::kReferenceSc, 1795055191, p);
  for (int w = 0; w < 12; ++w) {  // 12 x 0.5 s windows = 6 s
    double acc = 0.0;
    for (int i = 0; i < 24000; ++i) {
      float l[16], r[16];
      e2.Process16(l, r);
      if (i % 16 == 0) { /* sample the level */ }
      for (int k = 0; k < 16; ++k) acc += l[k] * l[k] + r[k] * r[k];
    }
    const double rms = sqrt(acc / (24000.0 * 2.0));
    printf("t=%.1fs rms=%.5f\n", (w + 1) * 0.5, rms);
  }
  return 0;
}
