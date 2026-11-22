// Shared constants for the composite USB descriptors (usb_descriptors.cpp)
// and the UAC1 capture class driver (usb_capture.cpp) — the single source
// for the interface/endpoint map so the two can never drift apart.
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
#define ITF_NUM_CAP_AC     2      // UAC1 capture AudioControl
#define ITF_NUM_CAP_AS     3      // UAC1 capture AudioStreaming
#define ITF_NUM_TOTAL      4

// ----------------------------------------------------------------------------
// Endpoint addresses (RP2350 hardware: EP1-7 per direction)
// ----------------------------------------------------------------------------

#define EPNUM_CDC_NOTIF    0x81   // interrupt IN
#define EPNUM_CDC_OUT      0x01   // bulk OUT
#define EPNUM_CDC_IN       0x82   // bulk IN
#define CAP_EP_IN          0x83   // isochronous async IN (capture)

// ----------------------------------------------------------------------------
// UAC1 capture format
// ----------------------------------------------------------------------------

#define CAP_N_CHANNELS          2
#define CAP_BYTES_PER_SAMPLE    2    // 16-bit
#define CAP_BYTES_PER_FRAME     (CAP_N_CHANNELS * CAP_BYTES_PER_SAMPLE)

// Servo hard max per USB frame: nominal(48) + correction(2) + 1 at 48 kHz.
// Sizes the EP wMaxPacketSize, the driver packet scratch, and the servo
// clamp in lockstep.
#define CAP_MAX_FRAMES_PER_PACKET  52
#define CAP_EP_MAX_PKT             (CAP_MAX_FRAMES_PER_PACKET * CAP_BYTES_PER_FRAME)

// UAC1 entity ids (capture function)
#define CAP_TERM_ID_IN     1      // input terminal: microphone (the engine render)
#define CAP_TERM_ID_OUT    2      // output terminal: USB streaming

// ----------------------------------------------------------------------------
// Boot-time descriptor prep
// ----------------------------------------------------------------------------

// Populate the serial string from the board unique id.  Call before tud_init().
void usb_descriptors_init(void);

extern char usb_descriptor_str_serial[33];

#ifdef __cplusplus
}
#endif

#endif  // _RACK_USB_DESCRIPTORS_H_
