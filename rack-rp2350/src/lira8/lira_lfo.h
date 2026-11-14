// hyper-lfo + triangle-lfo + square-lfo verbatim port (PORTING-SPEC.md §4).
// rate = mtof(127*(k/127)^2 - 75); LINK=1: B's rate = fb*(1 +- 0.5*squareA).
// del-tri = (tri(pA)+tri(pB))*0.5, tri(p) = (|p-0.5|*4-1)*0.5 in [-0.5,0.5];
// squareX = clip(cos(phaseX)*1000, -1, 1);
// del-sqr = (squareA + squareB)/2;
// sqr-lfo = squareA*squareB*(andor==0) + avg*(andor==1).
#ifndef RACK_SRC_LIRA8_LIRA_LFO_H_
#define RACK_SRC_LIRA8_LIRA_LFO_H_

#include "pd_primitives.h"
#include "lira8_tc.h"

namespace l8pd {

class HyperLfo {
 public:
  explicit HyperLfo(float sr) : sr_(sr) {}
  void set_rate_a(int k) { ra_ = 127.0f * (k / 127.0f) * (k / 127.0f) - 75.0f; }
  void set_rate_b(int k) { rb_ = 127.0f * (k / 127.0f) * (k / 127.0f) - 75.0f; }
  void set_link(float l) { link_ = l; }
  void set_andor(float a) { andor_ = a; }

  // Per-block: advances phasors, writes the three bus signals.
  LIRA8_TC(lira_lfo_block)
  void process_block(int n, float* del_tri, float* del_sqr, float* sqr_lfo) {
    for (int i = 0; i < n; ++i) {
      // A phasor + its square
      const float pa = pa_.next();
      square_a_ = std::fmax(-1.0f, std::fmin(1.0f,
          cos_lookup(pa * static_cast<float>(kCosTabSize)) * 1000.0f));
      // B rate FM by A square (link): fb*(1 + squareA/2*link); raw 0/1 gate
      const float fb = mtof(rb_) * (1.0f + square_a_ * 0.5f * link_);
      pb_.set_freq(fb);
      const float pb = pb_.next();
      const float sb = std::fmax(-1.0f, std::fmin(1.0f,
          cos_lookup(pb * static_cast<float>(kCosTabSize)) * 1000.0f));
      // triangles: converter = |p-0.5|*4 - 1 (in [-1,1], NO extra scaling —
      // verified against the patch's triangle-lfo canvas); the pair sums
      // into *~ 0.5 -> del-tri in [-1, +1].
      const float tri = ((std::fabs(pa - 0.5f) * 4.0f - 1.0f) +
                         (std::fabs(pb - 0.5f) * 4.0f - 1.0f)) * 0.5f;
      del_tri[i] = tri;
      del_sqr[i] = (square_a_ + sb) * 0.5f;
      sqr_lfo[i] = square_a_ * sb * (1.0f - andor_) +
                   (square_a_ + sb) * 0.5f * andor_;
    }
  }
  float square_a() const { return square_a_; }

 private:
  float sr_;
  float ra_ = 0.f, rb_ = 0.f, link_ = 0.f, andor_ = 0.f;
  float square_a_ = 0.f;
  Phasor pa_{sr_}, pb_{sr_};
};

}  // namespace l8pd

#endif  // RACK_SRC_LIRA8_LIRA_LFO_H_
