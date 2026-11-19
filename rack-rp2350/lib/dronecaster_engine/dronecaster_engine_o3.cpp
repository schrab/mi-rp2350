// Dronecaster engine hot entry point — compiled at -O3 by this library's
// library.json. Mirrors lib/lira8_engine exactly (WHY: the earlephilhower
// core appends -Os after the project flags, so an engine inlined into src/
// runs at -Os — the whole inlined body must be instantiated HERE at -O3,
// PORTING-HIGHLIGHTS §2). The include is RELATIVE (the lib must not depend
// on a src/ include root), matching the lira8 wrapper.
//
// The wrapper carries the SRAM section token (unique, per the token law) so
// the whole -O3 body lands in the .tc_dronecaster window — the rack SRAM law.
#include "../../src/dronecaster/dronecaster_engine.h"
#include "../../src/dronecaster/dronecaster_tc.h"
#include "dronecaster_engine_o3.h"

namespace sc {

DRONECASTER_TC(process_dronecaster_o3)
void process_block_o3(Engine& e, float* out_l, float* out_r, int n) {
  e.Process(out_l, out_r, n);
}

}  // namespace sc
