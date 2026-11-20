// Sine — ported from drones/sine.scd.
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_SINE_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_SINE_H_
#include "drone_common.h"
namespace sc {
namespace drones {
struct SineState { SinOsc osc; };
inline void Sine_Init(Ctx& c) {
  SineState* st = static_cast<SineState*>(c.arena->Alloc(sizeof(SineState)));
  st->osc.Init(c.tables, static_cast<double>(c.sr), c.hz);
  c.state = st;
}
inline void Sine_Process(Ctx& c, float* l, float* r, int n) {
  SineState* st = static_cast<SineState*>(c.state);
  st->osc.SetFreq(c.hz);
  st->osc.ProcessBlock(l, n);
  for (int i = 0; i < n; ++i) { const float v = l[i] * c.amp; l[i] = v; r[i] = v; }
}

}  // namespace drones
}  // namespace sc
#endif
