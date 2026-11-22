// TinyUSB configuration for the rack composite device:
//   CDC console (TinyUSB's built-in CDC driver) + USB MIDI device (the
//   built-in MIDI class driver, CFG_TUD_MIDI=1 — the same 4-byte event
//   packet contract the Arduino build's Adafruit_USBD_MIDI served).
//
// The UAC1 capture function (usb_capture.cpp, the DSPi pattern) is PARKED —
// the user's migration order: platform first, USB audio after acceptance.
// Its files stay in the tree; the CMake option RACK_PICO_USB_CAPTURE flips
// the descriptors + driver back on in the phase-3 task.
//
// CFG_TUSB_MCU (OPT_MCU_RP2040 — serves RP2350 too) and CFG_TUSB_OS
// (OPT_OS_PICO) come from the SDK's tinyusb CMake, not from here.
#ifndef _RACK_TUSB_CONFIG_H_
#define _RACK_TUSB_CONFIG_H_

#define CFG_TUD_ENABLED        1
#define CFG_TUH_ENABLED        0

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC            1
#define CFG_TUD_MIDI           1
#define CFG_TUD_AUDIO          0
#define CFG_TUD_MSC            0
#define CFG_TUD_HID            0
#define CFG_TUD_VENDOR         0

// The console writes the 2 s report + boot log through the CDC TX FIFO;
// 256 B keeps a full report line burst from dropping while the host read
// is not scheduled.
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_RX_BUFSIZE 128
#define CFG_TUD_MIDI_RX_BUFSIZE 128
#define CFG_TUD_MIDI_TX_BUFSIZE 128

#endif  // _RACK_TUSB_CONFIG_H_
