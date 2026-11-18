// Host-test shim: the correlator's bench timing only — a monotonic counter.
#ifndef HOST_SHIM_PICO_TIME_H_
#define HOST_SHIM_PICO_TIME_H_
#include <cstdint>
#include <ctime>
static inline uint32_t time_us_32(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)((uint64_t)ts.tv_sec * 1000000ull +
                    (uint64_t)ts.tv_nsec / 1000ull);
}
#endif
