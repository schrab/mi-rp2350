// Shared constants for the composite USB descriptors (usb_descriptors.cpp)
// — CDC console + USB MIDI device. The UAC1 capture function is parked
// (RACK_PICO_USB_CAPTURE phase-3 item); its constants return with it.
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
#define ITF_NUM_TOTAL      4

// ----------------------------------------------------------------------------
// Endpoint addresses (RP2350 hardware: EP1-7 per direction)
// ----------------------------------------------------------------------------

#define EPNUM_CDC_NOTIF    0x81   // interrupt IN
#define EPNUM_CDC_OUT      0x01   // bulk OUT
#define EPNUM_CDC_IN       0x82   // bulk IN
#define EPNUM_MIDI_OUT     0x02   // bulk OUT
#define EPNUM_MIDI_IN      0x83   // bulk IN

// Boot-time descriptor prep
void usb_descriptors_init(void);

extern char usb_descriptor_str_serial[33];

#ifdef __cplusplus
}
#endif

#endif  // _RACK_USB_DESCRIPTORS_H_
