#ifndef PORTTEST_TABLES_H
#define PORTTEST_TABLES_H

#include <stdint.h>

// Bit-exact dumps of ReBirth RB-338 2.01's tables (orb-engine test fixtures).
extern const float g_level[128];
extern const float g_pitch303[6144];
extern const float g_filt303[8 * 1024 * 4];
extern const float g_dist_w[32 * 256];
// { b2, a, g } of the 303 cutoff smoother.
extern const float g_smooth303[3];
extern const float g_pcf_lp[8 * 512 * 3];
extern const float g_pcf_bp[8 * 512 * 3];
extern const float g_smooth_pcf[3];
extern const uint8_t g_pcf_data[4336];

#endif
