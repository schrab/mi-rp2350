// USB MIDI device pump (see usb_midi.cpp).
#ifndef _RACK_USB_MIDI_H_
#define _RACK_USB_MIDI_H_

// Core-0 pump: drains the TinyUSB MIDI OUT endpoint into the rack MIDI
// funnel.  Call from the usb task cadence alongside usb_console_tick().
void usb_midi_pump(void);

#endif  // _RACK_USB_MIDI_H_
