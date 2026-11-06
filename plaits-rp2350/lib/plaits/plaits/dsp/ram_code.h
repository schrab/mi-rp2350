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
//
// Rack build (rack-rp2350, RACK_RP2350_TC defined project-wide): the prefix
// switches to .tc_plaits. so every marked input section becomes
// .tc_plaits.<token>. The rack's linker script collects those into a single
// flash-resident tc_plaits output section (VMA = the shared SRAM window,
// LMA = the reserved flash region) and copies the SELECTED module's section
// into the window at boot — standalone builds keep .time_critical.* (the
// core's crt0 copy path) unchanged.
//
// PLAITS_RAM_CODE_DEMOTABLE(token): the T6b chain-tier (user-approved). In
// the rack the tc_plaits window image must ALSO leave room for the chain
// B slot (clouds) beside the A slot, so the rack keeps ONLY the voice path +
// engine 0 (VirtualAnalog, boot default) + lut_sine SRAM-resident; everything
// else this port had marked (the fork-flagged hot engines chord/particle/
// modal/string/chiptune/six_op/scanned, the physical-modelling + fm support
// DSP, wav_integrated_waves — chord is their only SRAM-resident reader) is
// DEMOTED to flash: under RACK_RP2350_TC the mark expands to NOTHING so the
// section lands in .text/.rodata. Switched AND chained plaits then run those
// bodies through the XIP cache (sound-neutral, slower sustained — re-sweep
// on the bench; per-engine promotion back = one token line). Standalone
// builds expand it to the plain PLAITS_RAM_CODE mark: byte-identical
// firmware, every kept body still SRAM-resident.
#ifndef PLAITS_DSP_RAM_CODE_H_
#define PLAITS_DSP_RAM_CODE_H_

#if defined(ARDUINO_ARCH_RP2040)
#if defined(RACK_RP2350_TC)
#define PLAITS_RAM_CODE_PREFIX ".tc_plaits."
#else
#define PLAITS_RAM_CODE_PREFIX ".time_critical."
#endif
#define PLAITS_RAM_CODE(token) \
  __attribute__((section(PLAITS_RAM_CODE_PREFIX #token)))
#if defined(RACK_RP2350_TC)
#define PLAITS_RAM_CODE_DEMOTABLE(token)
#else
#define PLAITS_RAM_CODE_DEMOTABLE(token) PLAITS_RAM_CODE(token)
#endif
#else
#define PLAITS_RAM_CODE(token)
#define PLAITS_RAM_CODE_DEMOTABLE(token)
#endif

#endif  // PLAITS_DSP_RAM_CODE_H_
