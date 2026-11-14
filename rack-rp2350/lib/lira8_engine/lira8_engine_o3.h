// Declaration for the -O3 engine entry point (lib/lira8_engine). This header
// exists so the chain+ LDF sees an include from src/ and links the library;
// the definition and the WHY live in lira8_engine_o3.cpp.
//
// No includes: every user includes lira8/lira_engine.h first (the class must
// be complete to pass it by reference at the call site anyway).
#ifndef RACK_LIB_LIRA8_ENGINE_LIRA8_ENGINE_O3_H_
#define RACK_LIB_LIRA8_ENGINE_LIRA8_ENGINE_O3_H_

namespace l8pd {
class Lira8Engine;
void process_block_o3(Lira8Engine& e, float* out_l, float* out_r, int n);
}  // namespace l8pd

#endif  // RACK_LIB_LIRA8_ENGINE_LIRA8_ENGINE_O3_H_