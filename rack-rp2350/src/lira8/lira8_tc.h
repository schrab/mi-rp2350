// Rack RP2350 — LIRA-8 hot-code section placement (SRAM law).
//
// Same mechanism as the four module libs' RAM_CODE macros (see
// PORTING-HIGHLIGHTS §3): under RACK_RP2350_TC every hot function lands in
// a .tc_lira8.<token> input section; memmap_rack.ld places them in the
// rack SRAM window (boot copy + live swap). Host builds (no RACK_RP2350_TC)
// compile them plain — the host golden harness compiles these same headers.
//
// Token law (PORTING-HIGHLIGHTS §3): one unique token per function — an
// inline/COMDAT copy and a plain copy sharing a section name = "section
// type conflict" at link time.
#ifndef RACK_SRC_LIRA8_LIRA8_TC_H_
#define RACK_SRC_LIRA8_LIRA8_TC_H_

#if defined(RACK_RP2350_TC)
#define LIRA8_TC(token) __attribute__((section(".tc_lira8." #token)))
#else
#define LIRA8_TC(token)
#endif

#endif  // RACK_SRC_LIRA8_LIRA8_TC_H_
