// sc_kit_hw.h — MouseX/MouseY (GPL-3.0, UIUGens.cpp:170-230). The plan maps
// these to CC-driven constants: a drone port supplies the 0-1 position; the
// NRT reference uses the global default (0.f, since no GUI updates the state).
// Law: y0 = warp==0 ? (max-min)*pos+min : pow(max/min,pos)*min;
//       y1 = y0 + b1*(y1-y0), b1 = lag==0 ? 0 : exp(log001/(lag*sr)).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_HW_H_
#define RACK_SRC_DRONECASTER_SC_KIT_HW_H_
#include <cmath>

namespace sc {

struct MouseInput {
  void Init(double sr) {
    sr_ = static_cast<float>(sr);
    y1_ = 0.f;
    b1_ = 0.f;
    lag_ = 0.f;
  }
  void ProcessBlock(float* out, int n, float pos, float minval, float maxval,
                    float warp, float lag) {
    if (lag != lag_) {
      b1_ = lag == 0.f ? 0.f
                       : static_cast<float>(
                             std::exp(-6.907755278982137 /
                                      (static_cast<double>(lag) * sr_)));
      lag_ = lag;
    }
    const float y0 = warp == 0.f ? (maxval - minval) * pos + minval
                                 : std::pow(maxval / minval, pos) * minval;
    for (int i = 0; i < n; ++i) {
      y1_ = y0 + b1_ * (y1_ - y0);
      out[i] = y1_;
    }
  }
  float sr_ = 48000.f, y1_ = 0.f, b1_ = 0.f, lag_ = 0.f;
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_HW_H_
