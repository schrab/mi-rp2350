// Shared constants for the composite USB descriptors (usb_descriptors.cpp)
// and the UAC1 capture class driver (usb_capture.cpp) — one source for the
// interface/endpoint map so the two can never drift apart.
//
// Functions: CDC console (0/1), USB MIDI (2/3), UAC1 capture (4/5).
#ifndef _RACK_USB_DESCRIPTORS_H_
#define _RACK_USB_DESCRIPTORS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// Interface numbers
// ----------------------------------------------------------------------------

#define ITF_NUM_CDC_CTRL   0
#define ITF_NUM_CDC_DATA   1
#define ITF_NUM_MIDI_CTRL  2     // AudioControl (MIDI streaming header)
#define ITF_NUM_MIDI_DATA  3     // MIDI streaming (bulk EPs)
#define ITF_NUM_CAP_AC     4     // UAC1 capture AudioControl
#define ITF_NUM_CAP_AS     5     // UAC1 capture AudioStreaming
#define ITF_NUM_TOTAL      6

// ----------------------------------------------------------------------------
// Endpoint addresses (RP2350 hardware: EP1-7 per direction)
// ----------------------------------------------------------------------------

#define EPNUM_CDC_NOTIF    0x81   // interrupt IN
#define EPNUM_CDC_OUT      0x01   // bulk OUT
#define EPNUM_CDC_IN       0x82   // bulk IN
#define EPNUM_MIDI_OUT     0x02   // bulk OUT
#define EPNUM_MIDI_IN      0x83   // bulk IN
#define CAP_EP_IN          0x84   // isochronous async IN (capture)

// ----------------------------------------------------------------------------
// UAC1 capture format — 2-ch, 24-bit packed, 44.1/48 kHz
// ----------------------------------------------------------------------------

#define CAP_N_CHANNELS          2
#define CAP_BYTES_PER_SAMPLE    3    // 24-bit packed
#define CAP_BYTES_PER_FRAME     (CAP_N_CHANNELS * CAP_BYTES_PER_SAMPLE)  // 6

// DSPi's ALSA-verified numbers: 52 frames x 6 B = 312 B per microframe.
// Sizes the EP wMaxPacketSize, the driver packet scratch and the servo clamp.
#define CAP_MAX_FRAMES_PER_PACKET  52
#define CAP_EP_MAX_PKT             (CAP_MAX_FRAMES_PER_PACKET * CAP_BYTES_PER_FRAME)

// UAC1 entity ids (capture function) — distinct from the MIDI function's.
#define CAP_TERM_ID_IN     3      // input terminal: the engine render
#define CAP_TERM_ID_OUT    4      // output terminal: USB streaming

// Boot-time descriptor prep
void usb_descriptors_init(void);

extern char usb_descriptor_str_serial[33];

#ifdef __cplusplus
}
#endif

#endif  // _RACK_USB_DESCRIPTORS_H_
