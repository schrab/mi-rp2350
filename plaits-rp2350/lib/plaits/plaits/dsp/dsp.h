// Copyright 2016 Emilie Gillet.
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
//
// -----------------------------------------------------------------------------
//
// Utility DSP routines.

#ifndef PLAITS_DSP_DSP_H_
#define PLAITS_DSP_DSP_H_

#include "stmlib/stmlib.h"

#ifdef PLAITS_CORRECTED_SAMPLE_RATE
#include <cmath>
#endif

namespace plaits {

// Rack chain mode (rack-rp2350, RACK_PLAITS_RUNTIME_RATE defined project-wide
// in every rack env, NEVER by the standalone ports): kSampleRate becomes a
// RUNTIME global so chained plaits can render its DSP at the rack's 32 kHz
// chain grid while SWITCHED plaits stays at its native 48 kHz. Surveyed
// 2026-11-05: every code use of kSampleRate / kCorrectedSampleRate in the
// fork is runtime float arithmetic inside Init()/Render() (drum envelope
// times, SVF cutoffs, delay lengths, SAM/LPC frame timers, the six-op FM
// voice Init, brass per-sample scaling) -- there is NO compile-time use
// anywhere (no array sizes, template arguments, case labels or static
// initializers outside this header), so a runtime global is sound. Engines
// capture the rate at Init() time: the host must re-Init (plaits::Voice::
// Init) whenever the rate context changes -- the rack chain state machine
// does exactly that on chain enter/leave. Documented 48k-baked exceptions
// (behave slightly differently at 32 kHz, accepted by design):
//   - zxphase48k / zxpulse48k engines: fixed-point phase increments
//     calibrated to a 48 kHz output ("one internal (4x) step per sample");
//     their internal time-based effects (flanger delay, pin widths) stretch
//     by 1.5x at 32k.
//   - particle_burst: kParticleStockDecay (64763/65536) is a per-48kHz-sample
//     envelope coefficient -- burst decays stretch 1.5x in real time at 32k.
//     (kParticlePitchCorrection itself stays exact: see below.)
#ifdef RACK_PLAITS_RUNTIME_RATE
extern float kSampleRate;            // DEFINED in voice.cc (single definition)
extern float kCorrectedSampleRate;   // tracks kSampleRate: the rack's I2S
                                     // always clocks at the true rate, so the
                                     // SPEC R6 pitch shim stays exactly zero.
extern float a0;                     // (440/8) / kCorrectedSampleRate,
                                     // recomputed by SetSampleRate (all
                                     // readers are runtime Init-time: the
                                     // fm/wavetable engines, NoteToFrequency).
void SetSampleRate(float rate);      // set all three (defined in voice.cc)

// 12*log2(kSampleRate/kCorrectedSampleRate) with both tracking the same true
// rate is identically zero at ANY rate, so the pitch-correction constant
// stays COMPILE-TIME zero. This keeps kParticlePitchCorrection /
// kStruckDrumPitchCorrection (namespace-scope consts derived from it in
// particle_burst_engine.h / struck_drum_engine.h) compile-time too -- no
// static-initialization-order hazard from making the rates runtime.
static const float kCorrectedPitchOffset = 0.0f;
#else  // original compile-time constants (standalone ports: 48 kHz baked)
static const float kSampleRate = 48000.0f;
#endif  // RACK_PLAITS_RUNTIME_RATE

#ifndef RACK_PLAITS_RUNTIME_RATE
// There is no proper PLL for I2S, only a divider on the system clock to derive
// the bit clock.
// The division ratio is set to 47 (23 EVEN, 1 ODD) by the ST libraries.
//
// Bit clock = 72000000 / 47 = 1531.91 kHz
// Frame clock = Bit clock / 32 = 47872.34 Hz
//
// That's only 4.6 cts of error, but we care!
//
// A build whose samples really play at some other rate defines
// PLAITS_CORRECTED_SAMPLE_RATE as that rate. A desktop host that plays the
// engines at exactly kSampleRate (Palette) defines it as 48000.0f, which
// removes the correction; left undefined, the module's divider rate applies.

#ifdef PLAITS_CORRECTED_SAMPLE_RATE
static const float kCorrectedSampleRate = PLAITS_CORRECTED_SAMPLE_RATE;
#else
static const float kCorrectedSampleRate = 47872.34f;
#endif
const float a0 = (440.0f / 8.0f) / kCorrectedSampleRate;
#endif  // !RACK_PLAITS_RUNTIME_RATE

// 12 * log2(kSampleRate / kCorrectedSampleRate): the semitones a pitch that
// does NOT go through a0 (a Braids table index, say) adds to agree with the
// ones that do (SPEC R6). The module keeps the literal, so its firmware gains
// no libm call; an override computes it, and 48 kHz gives exactly zero.
#if defined(PLAITS_CORRECTED_SAMPLE_RATE) && !defined(RACK_PLAITS_RUNTIME_RATE)
static const float kCorrectedPitchOffset =
    12.0f * std::log2(kSampleRate / kCorrectedSampleRate);
#elif !defined(RACK_PLAITS_RUNTIME_RATE)
static const float kCorrectedPitchOffset = 0.046105f;
#endif

const size_t kMaxBlockSize = 24;
const size_t kBlockSize = 12;

}  // namespace plaits

#endif  // PLAITS_DSP_DSP_H_
