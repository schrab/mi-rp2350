// T9 reverb A/B probe: does the rack's FxEngine<21632> (RACK_RP2350_TC arm)
// render bit-identically to the upstream FxEngine<32768>?
//
// Law: fx_engine.h wraps EVERY delay-line access with `& (size - 1)`. That is
// a correct circular wrap only for POWER-OF-TWO sizes. 21632 is not a power
// of two, so accesses that cross the buffer end alias to scattered addresses
// (e.g. 43257 & 21631 = 121, inside AP1's region) instead of wrapping. With a
// correct wrap the engine size is provably sound-neutral (delay lengths are
// compile-time; only absolute addresses move), so a CORRECT 21632 must render
// bit-identical to 32768 for the same input/settings. Any difference = the
// masking bug is live.
//
// Render conditions = the reported failing case: space ~1.0 on the rack
// (reverb_amount 0.4, reverb_time 0.83, diffusion 0.625, lp 0.7, in-gain 0.2),
// 16-sample blocks (the rack kBlockSize), impulses like exciter strikes.
// Output: raw float32 pairs (main, aux) to argv[1]; prints a checksum.
//
// Compiled twice by the Makefile (test_reverb_rack / test_reverb_upstream);
// the dumps must be byte-identical (check_reverb_ab target).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "elements/dsp/fx/reverb.h"

using namespace elements;

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s <out.raw>\n", argv[0]);
    return 2;
  }
  static uint16_t mem[32768 + 64];   // holds either arm's Init fill
  Reverb reverb;
  reverb.Init(mem);
  // space = 1.0 on the rack: part.inc -> amount 0.4, time 0.83; Seed()'s
  // reverb_diffusion/lp ranges collapsed to their mid defaults here.
  reverb.set_amount(0.4f);
  reverb.set_time(0.83f);
  reverb.set_diffusion(0.625f);
  reverb.set_lp(0.7f);
  reverb.set_input_gain(0.2f);

  const size_t kBlock = 16;          // rack kBlockSize
  float main[kBlock], aux[kBlock];
  FILE* f = fopen(argv[1], "wb");
  if (!f) {
    perror("fopen");
    return 2;
  }

  uint32_t checksum = 0;
  const size_t kBlocks = 6000;       // 96000 samples = 3 s at 32 kHz
  for (size_t b = 0; b < kBlocks; ++b) {
    for (size_t i = 0; i < kBlock; ++i) {
      const size_t t = b * kBlock + i;
      // exciter-like input: a 0.9 impulse every 8000 samples (4 strikes)
      main[i] = (t % 8000 == 0) ? 0.9f : 0.0f;
      aux[i] = 0.0f;
    }
    reverb.Process(main, aux, kBlock);
    fwrite(main, sizeof(float), kBlock, f);
    fwrite(aux, sizeof(float), kBlock, f);
    for (size_t i = 0; i < kBlock; ++i) {
      uint32_t x;
      memcpy(&x, &main[i], 4);
      checksum = checksum * 1664525u + x + 1013904223u;
      memcpy(&x, &aux[i], 4);
      checksum = checksum * 1664525u + x + 1013904223u;
    }
  }
  fclose(f);
  printf("checksum: %08x\n", checksum);
  return 0;
}
