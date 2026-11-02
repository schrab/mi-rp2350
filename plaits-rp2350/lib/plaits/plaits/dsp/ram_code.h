// Copyright 2026 Plaits RP2350 port.
//
// Marker for per-sample hot paths (RP2350 port). Mirrors the clouds/elements
// ram_code.h: .time_critical.* sections are mapped to SRAM by the
// Arduino-pico linker script (inside .data, copied from flash at boot), so
// marked functions/data execute without XIP flash cache stalls — those stalls
// cost more than the entire 500 us audio block budget. No-op on every other
// platform.
//
// Usage: PLAITS_RAM_CODE(unique_token) void Foo::Render(...) { ... }
// Each function needs its own token: shared section names collide at link
// time between inline/COMDAT and plain copies (section type conflict). Two
// same-flags weak copies (e.g. template instantiations) MAY share one token.
#ifndef PLAITS_DSP_RAM_CODE_H_
#define PLAITS_DSP_RAM_CODE_H_

#if defined(ARDUINO_ARCH_RP2040)
#define PLAITS_RAM_CODE(token) __attribute__((section(".time_critical." #token)))
#else
#define PLAITS_RAM_CODE(token)
#endif

#endif  // PLAITS_DSP_RAM_CODE_H_
