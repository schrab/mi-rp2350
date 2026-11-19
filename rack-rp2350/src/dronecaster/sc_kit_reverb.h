// sc_kit_reverb.h — SC 3.14.1 core reverbs (GPL-3.0, ReverbUGens.cpp).
// FreeVerb anchors: struct 28, Ctor 138 (calls next(unit,1) — one priming
// sample), next 208. 8 comb filters + 4 allpass filters, all float math.
// GVerb is added below once its law pass is green.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_REVERB_H_
#define RACK_SRC_DRONECASTER_SC_KIT_REVERB_H_
#include <array>
#include <cmath>
#include <cstddef>

namespace sc {

struct FreeVerb {
  void Init(float mix, float room, float damp, float first_in) {
    iota_.fill(0);
    r_.fill(0.f);
    ar_.fill(0.f);
    float unused;
    Step(first_in, mix, room, damp, unused);  // ctor's next(unit, 1)
  }
  void ProcessBlock(const float* in, float* out, int n, float mix, float room,
                    float damp) {
    for (int i = 0; i < n; ++i) Step(in[i], mix, room, damp, out[i]);
  }

 private:
  void Step(float x, float mix, float room, float damp, float& out) {
    if (mix > 1.f) mix = 1.f;
    if (mix < 0.f) mix = 0.f;
    const float dry = 1.f - mix;
    if (room > 1.f) room = 1.f;
    if (room < 0.f) room = 0.f;
    const float fb = 0.700000f + 0.280000f * room;
    if (damp > 1.f) damp = 1.f;
    if (damp < 0.f) damp = 0.f;
    const float d = 0.400000f * damp;
    const float invd = 1.f - d;
    const float exc = 1.500000e-02f * x;

    float t[12];
    static const int kSize[12] = {225, 341, 441, 556, 1617, 1557,
                                  1491, 1422, 1277, 1116, 1188, 1356};
    for (int i = 0; i < 12; ++i) {
      if (++iota_[i] == kSize[i]) iota_[i] = 0;
      t[i] = lines_[i][iota_[i]];
    }
    // 8 comb stages (indices 4..11), each with its own low-pass state.
    for (int k = 0; k < 8; ++k) {
      const int line = k + 4, rlo = k * 2 + 4, rhi = rlo + 1;
      r_[rhi] = invd * r_[rlo] + d * r_[rhi];
      lines_[line][iota_[line]] = exc + fb * r_[rhi];
      r_[rlo] = t[line];
    }
    const float ftemp8 = r_[16] + r_[18];
    lines_[3][iota_[3]] =
        ((0.5f * r_[3] + r_[4]) + (r_[6] + r_[8])) +
        ((r_[10] + r_[12]) + (r_[14] + ftemp8));
    r_[3] = t[3];
    ar_[3] = r_[3] - ((r_[4] + r_[6]) + (r_[8] + r_[10]) +
                      ((r_[12] + r_[14]) + ftemp8));
    lines_[2][iota_[2]] = 0.5f * r_[2] + ar_[3];
    r_[2] = t[2];
    ar_[2] = r_[2] - ar_[3];
    lines_[1][iota_[1]] = 0.5f * r_[1] + ar_[2];
    r_[1] = t[1];
    ar_[1] = r_[1] - ar_[2];
    lines_[0][iota_[0]] = 0.5f * r_[0] + ar_[1];
    r_[0] = t[0];
    ar_[0] = r_[0] - ar_[1];
    out = dry * x + mix * ar_[0];
  }
  std::array<int, 12> iota_{};
  std::array<float, 20> r_{};
  std::array<float, 4> ar_{};
  std::array<std::array<float, 1617>, 12> lines_{};
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_REVERB_H_
