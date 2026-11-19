// Declaration for the -O3 engine entry point (lib/dronecaster_engine) —
// the lira8 pattern: src/ includes this so the LDF links the library; the
// -O3 definition (and the SRAM token) lives in the .cpp.
//
// No includes: every user includes dronecaster/dronecaster_engine.h first
// (the class must be complete to pass it by reference at the call site).
#ifndef RACK_LIB_DRONECASTER_ENGINE_DRONECASTER_ENGINE_O3_H_
#define RACK_LIB_DRONECASTER_ENGINE_DRONECASTER_ENGINE_O3_H_

namespace sc {
class Engine;
void process_block_o3(Engine& e, float* out_l, float* out_r, int n);
}  // namespace sc

#endif  // RACK_LIB_DRONECASTER_ENGINE_DRONECASTER_ENGINE_O3_H_
