// Rack RP2350 — Dronecaster hot-code section placement (SRAM law).
// Only the -O3 wrapper (lib/dronecaster_engine) carries the token: at -O3
// the engine inlines into it, and a section attribute on an inlined callee
// does not survive. One unique token per emitted function.
#ifndef RACK_SRC_DRONECASTER_DRONECASTER_TC_H_
#define RACK_SRC_DRONECASTER_DRONECASTER_TC_H_
#if defined(RACK_RP2350_TC)
#define DRONECASTER_TC(token) __attribute__((section(".tc_dronecaster." #token)))
#else
#define DRONECASTER_TC(token)
#endif
#endif
