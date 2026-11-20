// Square — ported from drones/square.scd.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_SQUARE_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_SQUARE_H_
#include "drone_common.h"
namespace sc {
namespace drones {
struct SquareState { Pulse osc; };
inline void Square_Init(Ctx& c) {
  SquareState* st = static_cast<SquareState*>(c.arena->Alloc(sizeof(SquareState)));
  st->osc.Init(c.tables, static_cast<double>(c.sr), c.hz, 0.5f);
  c.state = st;
}
inline void Square_Process(Ctx& c, float* l, float* r, int n) {
  SquareState* st = static_cast<SquareState*>(c.state);
  st->osc.SetFreq(c.hz);
  st->osc.ProcessBlock(l, n);
  for (int i = 0; i < n; ++i) { const float v = l[i] * c.amp; l[i] = v; r[i] = v; }
}

}  // namespace drones
}  // namespace sc
#endif
