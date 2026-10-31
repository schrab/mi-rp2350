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

// Marker for per-sample DSP hot paths. On the RP2040/RP2350 Arduino core the
// .time_critical.* sections are mapped to SRAM by the linker script (inside
// .data, copied from flash at boot), so marked functions execute without XIP
// flash cache stalls. On every other platform the marker is a no-op.
//
// Usage: ELEMENTS_RAM_CODE(unique_token) void Foo::Process(...) { ... }
// Each function needs its own token: inline/COMDAT and plain copies of one
// shared section name collide at link time (section type conflict).
#ifndef ELEMENTS_DSP_RAM_CODE_H_
#define ELEMENTS_DSP_RAM_CODE_H_

#if defined(ARDUINO_ARCH_RP2040)
#define ELEMENTS_RAM_CODE(token) __attribute__((section(".time_critical." #token)))
#else
#define ELEMENTS_RAM_CODE(token)
#endif

#endif  // ELEMENTS_DSP_RAM_CODE_H_
