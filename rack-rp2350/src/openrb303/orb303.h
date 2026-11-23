#ifndef PORTTEST_ORB303_H
#define PORTTEST_ORB303_H

#include <stdint.h>

#define ORB_BLOCK 64
typedef float OrbBlock[ORB_BLOCK];

// One-time init of the engine state (bit-exact tables are linked from .bin).
void orb_init(void);

// Note-mode control (the engine's own API; no sequencer/song needed).
// 'note' is ReBirth pattern note 0 = MIDI 36 at tune centre.
void orb_note_on(int semis, bool accent, bool slide);
void orb_note_off(void);

// Advance the step sequencer by one tick (48 ticks / step) and render a
// 64-sample block of the 303 -> distortion chain into `bus`.
void orb_tick_step(int tick);
void orb_process(OrbBlock bus);

// Knob controls (0..127), mirrored from the engine's live parameter bytes.
void orb_set_param(int index, uint8_t value);   // 303 live params
void orb_set_303_param(int index, uint8_t value);
void orb_set_dist_param(int index, uint8_t value);

#endif
