// LIRA-8 engine hot entry point — compiled at -O3 by this library's
// library.json.
//
// WHY THIS TINY TU EXISTS: src/ (main.cpp, which includes module_lira8.h and
// therefore the whole engine header-only) gets the project's -O3 PLUS the
// earlephilhower core's appended -Os — the last -O wins, so every per-sample
// engine function inlined into src/ ran at -Os (measured on device: 3.1 ms
// per 64-sample block, 1.5x real time at 32 kHz). The fix is the same
// per-library -O3 pattern the four vendored MI DSP libs use
// (PORTING-HIGHLIGHTS §2): the engine's Process() is instantiated here at
// -O3, inlining every primitive, LFO, voice and delay stage into one body.
//
// The wrapper carries the SRAM section token (unique, per the token law) so
// the whole -O3 body lands in the .tc_lira8 window — the rack SRAM law.
// main.cpp calls this wrapper and never Process() directly, so no -Os copy
// is emitted. Remove/rename only together with the declaration in
// src/module_lira8.h.
#include "../../src/lira8/lira_engine.h"
#include "lira8_engine_o3.h"

namespace l8pd {

LIRA8_TC(lira_engine_process_o3)
void process_block_o3(Lira8Engine& e, float* out_l, float* out_r, int n) {
  e.Process(out_l, out_r, n);
}

}  // namespace l8pd