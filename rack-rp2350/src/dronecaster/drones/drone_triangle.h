// Triangle — ported from drones/triangle.scd.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_TRIANGLE_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_TRIANGLE_H_
#include "drone_common.h"
namespace sc {
namespace drones {
struct TriState { LFTri osc; };
inline void Triangle_Init(Ctx& c) {
  TriState* st = static_cast<TriState*>(c.arena->Alloc(sizeof(TriState)));
  st->osc.Init(static_cast<double>(c.sr), c.hz);
  c.state = st;
}
inline void Triangle_Process(Ctx& c, float* l, float* r, int n) {
  TriState* st = static_cast<TriState*>(c.state);
  st->osc.SetFreq(c.hz);
  st->osc.ProcessBlock(l, n);
  for (int i = 0; i < n; ++i) { const float v = l[i] * c.amp; l[i] = v; r[i] = v; }
}

}  // namespace drones
}  // namespace sc
#endif
