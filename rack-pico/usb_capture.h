// USB audio capture for the rack: streams the engine render back to the host
// as a UAC1 recording device (the device-vs-golden rig AND a user-facing
// capture feature).
//
// Architecture (the DSPi DSPI_LOOPBACK pattern, implemented from scratch for
// the rack — see usb_descriptors.cpp for the license note):
//
//   core 1 audio loop (post-Clip16 int16 pairs, the same samples the DAC gets)
//       -> usb_capture_push_stereo() -> SPSC ring
//           -> fill servo (nominal frames/frame + clamped P correction,
//              prime before streaming)
//               -> isochronous async IN EP 0x84 (24-bit packed) -> USB host
//
// The capture is the clock master (no feedback EP): the host must select the
// rate the engine runs at (GET_CUR reports it); a mismatched host rate plays
// at the wrong pitch by design.  The producer runs on core 1, the consumer
// (fill + EP arming) on core 0, so the ring uses __dmb() barrier discipline
// for the cross-core publication of head/tail.
#ifndef _RACK_USB_CAPTURE_H_
#define _RACK_USB_CAPTURE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Push `frames` stereo frames (L and R int16 sample arrays) into the capture
// ring.  Called from the core-1 audio loop right after the int16 conversion;
// never blocks, drops on overflow and counts it.  RAM-resident.
void usb_capture_push_stereo(const int16_t *l, const int16_t *r, uint32_t frames);

// Capture-glitch counters.  A dropped or inserted frame is indistinguishable
// from an engine fault in the captured audio, so the rig samples these around
// a measurement to tell the two apart.
uint32_t usb_capture_overflow_count(void);   // frames dropped, ring full
uint32_t usb_capture_underrun_count(void);   // silence packets sent, ring dry

// The engine rate the capture reports (GET_CUR) and paces to.  main.cpp
// owns the value; set it before tud_init().
extern uint32_t rack_engine_rate_hz;

#ifdef __cplusplus
}
#endif

#endif  // _RACK_USB_CAPTURE_H_
