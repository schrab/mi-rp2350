#ifndef PORTTEST_PCFCOMP_H
#define PORTTEST_PCFCOMP_H

#include <stdint.h>
#include "orb303.h"

void pcfcomp_init(void);
// 303 -> distortion is done by orb_process; this runs PCF -> compressor on the same bus.
void pcfcomp_process(OrbBlock bus, int tick);
void pcfcomp_set_param(int which, int index, uint8_t value);  // which: 0=PCF, 1=comp

#endif
