// Copyright 2026 Rings RP2350 port.
//
// Marker for per-sample hot paths (RP2350 port). Mirrors the
// plaits/clouds/elements ram_code.h: .time_critical.* sections are mapped to
// SRAM by the Arduino-pico linker script (inside .data, copied from flash at
// boot), so marked functions/data execute without XIP flash cache stalls —
// those stalls cost more than the entire 750 us audio block budget. No-op on
// every other platform.
//
// Usage: RINGS_RAM_CODE(unique_token) void Foo::Process(...) { ... }
// Each function needs its own token: shared section names collide at link
// time between inline/COMDAT and plain copies (section type conflict). Two
// same-flags weak copies (e.g. template instantiations) MAY share one token.
// NOTE: for member templates defined out-of-line (.cc), the attribute MUST
// sit on the in-class declaration — GCC silently drops section attributes
// placed on the out-of-line definition (see string.h).
//
// Rack build (rack-rp2350, RACK_RP2350_TC defined project-wide): the prefix
// switches to .tc_rings. so every marked input section becomes
// .tc_rings.<token>. The rack's linker script collects those into a single
// flash-resident tc_rings output section (VMA = the shared SRAM window,
// LMA = the reserved flash region) and copies the SELECTED module's section
// into the window at boot — standalone builds keep .time_critical.* (the
// core's crt0 copy path) unchanged.
#ifndef RINGS_DSP_RAM_CODE_H_
#define RINGS_DSP_RAM_CODE_H_

#if defined(ARDUINO_ARCH_RP2040)
#if defined(RACK_RP2350_TC)
#define RINGS_RAM_CODE_PREFIX ".tc_rings."
#else
#define RINGS_RAM_CODE_PREFIX ".time_critical."
#endif
#define RINGS_RAM_CODE(token) \
  __attribute__((section(RINGS_RAM_CODE_PREFIX #token)))
#else
#define RINGS_RAM_CODE(token)
#endif

#endif  // RINGS_DSP_RAM_CODE_H_
