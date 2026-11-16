// Copyright 2014 Emilie Gillet.
//
// Rack RP2350 addition (NOT upstream warps): marker for warps' per-sample DSP
// hot paths, mirroring elements/dsp/ram_code.h. Under RACK_RP2350_TC the
// sections are renamed .tc_elements.warps_<token> so warps' hot code is
// collected into the tc_elements output section (the rack SRAM hot-code
// window's A slot — warps rides the elements module; no new overlay member).
// Everywhere else (standalone builds, host tools) it is a no-op.
//
// Usage: WARPS_RAM_CODE(unique_token) void Foo::Process(...) { ... }
// Tokens must be unique per function (inline/COMDAT and plain copies of one
// shared section name collide at link time).
#ifndef WARPS_DSP_RAM_CODE_H_
#define WARPS_DSP_RAM_CODE_H_

#if defined(ARDUINO_ARCH_RP2040) && defined(RACK_RP2350_TC)
#define WARPS_RAM_CODE_PREFIX ".tc_elements.warps_"
#define WARPS_RAM_CODE(token) \
  __attribute__((section(WARPS_RAM_CODE_PREFIX #token)))
#else
#define WARPS_RAM_CODE(token)
#endif

#endif  // WARPS_DSP_RAM_CODE_H_
