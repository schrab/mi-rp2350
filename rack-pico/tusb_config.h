// TinyUSB configuration for the rack composite device:
//   CDC console (TinyUSB's built-in CDC driver) + the UAC1 capture function
//   (a custom class driver registered via usbd_app_driver_get_cb()).
//
// TinyUSB's built-in audio class driver is UAC2-only (audio_device.c rejects
// bInterfaceProtocol != UAC2), so the UAC1 capture function is hand-rolled in
// usb_capture.cpp and CFG_TUD_AUDIO stays 0 (the DSPi architecture).
//
// CFG_TUSB_MCU (OPT_MCU_RP2040 — serves RP2350 too) and CFG_TUSB_OS
// (OPT_OS_PICO) come from the SDK's tinyusb CMake, not from here.
#ifndef _RACK_TUSB_CONFIG_H_
#define _RACK_TUSB_CONFIG_H_

#define CFG_TUD_ENABLED        1
#define CFG_TUH_ENABLED        0

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC            1
#define CFG_TUD_AUDIO          0
#define CFG_TUD_MSC            0
#define CFG_TUD_HID            0
#define CFG_TUD_MIDI           0   // the USB MIDI migration item flips this to 1
#define CFG_TUD_VENDOR         0

// The console writes the 2 s report + boot log through the CDC TX FIFO;
// 256 B keeps a full report line burst from dropping while the host read
// is not scheduled.
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_RX_BUFSIZE 128

#endif  // _RACK_TUSB_CONFIG_H_
