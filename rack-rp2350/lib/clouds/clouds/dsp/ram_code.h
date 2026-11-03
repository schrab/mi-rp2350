// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.

// Marker for per-sample hot paths (RP2350 port). .time_critical.* sections are
// mapped to SRAM by the Arduino-pico linker script (inside .data, copied from
// flash at boot), so marked functions/data execute without XIP flash cache
// stalls — on Elements those stalls cost more than the entire audio block
// budget. No-op on every other platform.
//
// Rack build (rack-rp2350, RACK_RP2350_TC defined project-wide): the prefix
// switches to .tc_clouds. so every marked input section becomes
// .tc_clouds.<token> (the rack's CMSIS-DSP copy carries the matching
// .tc_clouds.cmsis / .tc_clouds.cmsis_tables names). The rack's linker script
// collects those into a single flash-resident tc_clouds output section
// (VMA = the shared SRAM window, LMA = the reserved flash region) and copies
// the SELECTED module's section into the window at boot — standalone builds
// keep .time_critical.* (the core's crt0 copy path) unchanged.
//
// Usage: CLOUDS_RAM_CODE(unique_token) void Foo::Process(...) { ... }
// Each function needs its own token: shared section names collide at link
// time between inline/COMDAT and plain copies (section type conflict). Two
// same-flags weak copies (e.g. template instantiations) MAY share one token.
#ifndef CLOUDS_DSP_RAM_CODE_H_
#define CLOUDS_DSP_RAM_CODE_H_

#if defined(ARDUINO_ARCH_RP2040)
#if defined(RACK_RP2350_TC)
#define CLOUDS_RAM_CODE_PREFIX ".tc_clouds."
#else
#define CLOUDS_RAM_CODE_PREFIX ".time_critical."
#endif
#define CLOUDS_RAM_CODE(token) \
  __attribute__((section(CLOUDS_RAM_CODE_PREFIX #token)))
#else
#define CLOUDS_RAM_CODE(token)
#endif

#endif  // CLOUDS_DSP_RAM_CODE_H_
