// drone_trio.h — T-D1 drones: line-for-line C++ mirrors of the .scd graphs.
//   Sine.scd:     { arg hz, amp; (SinOsc.ar(hz) * amp).dup; }
//   Square.scd:   { arg hz, amp; (Pulse.ar(hz, 0.5) * amp).dup; }
//   Triangle.scd: { arg hz, amp; (LFTri.ar(hz) * amp).dup; }
// Voice state is arena-allocated in *_Init and persists across blocks
// (oscillator phase is the whole point of a drone).
#ifndef RACK_SRC_DRONECASTER_DRONES_DRONE_TRIO_H_
#define RACK_SRC_DRONECASTER_DRONES_DRONE_TRIO_H_
#include <cstddef>
#include "sc_kit_osc.h"
#include "sc_kit_pulse.h"
#include "drone_t2.h"
#include "../dronecaster_engine.h"

namespace sc {
namespace drones {

struct SineState { SinOsc osc; };
struct SquareState { Pulse osc; };
struct TriState { LFTri osc; };

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

inline const DroneSpec kDroneSine     = {"Sine",     drones::Sine_Init,     drones::Sine_Process};
inline const DroneSpec kDroneSquare   = {"Square",   drones::Square_Init,   drones::Square_Process};
inline const DroneSpec kDroneTriangle = {"Triangle", drones::Triangle_Init, drones::Triangle_Process};

// Catalog order = byte-wise sort of drones/*.scd (verified in Task 1).
inline const DroneSpec* Catalog(int id) {
  static const DroneSpec* const kCatalog[kCatalogCount] = {
      /* 0 Apparatus .. 13 */ &drones::kDroneApparatus, nullptr, nullptr,
      nullptr, nullptr,
      nullptr, nullptr, &drones::kDroneFieldsteel,
      nullptr, nullptr, &drones::kDroneHecker, nullptr, nullptr, nullptr,
      /* 14 */ &kDroneMtLion,
      /* 15 */ &kDroneMtZion,
      /* 16 .. 21 */ nullptr, nullptr, nullptr, nullptr,
      &drones::kDroneSachiko, nullptr,
      /* 22 */ &kDroneSine, /* 23 */ &kDroneSquare,
      /* 24 */ &drones::kDroneStarlids,
      /* 25 */ &drones::kDroneSupersaw,
      /* 26 */ nullptr,
      /* 27 */ &kDroneThx,
      /* 28 */ nullptr,
      /* 29 */ &kDroneTriangle,
      /* 30 .. 35 */ &drones::kDroneTwinPks, nullptr, nullptr, nullptr, nullptr,
      nullptr,
      /* 36 */ &kDroneHarmsWay,
  };
  if (id < 0 || id >= kCatalogCount) return nullptr;
  return kCatalog[id];
}

}  // namespace sc
#endif
