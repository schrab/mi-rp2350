// CDC console bridge (see usb_console.cpp).
#ifndef _RACK_USB_CONSOLE_H_
#define _RACK_USB_CONSOLE_H_

// Register the CDC stdio driver (after stdio_init_all(); before use).
void usb_console_init(void);

// Core-0 pump: runs the TinyUSB device task and flushes pending CDC output.
// Call at ~1 ms cadence from the core-0 loop.
void usb_console_tick(void);

#endif  // _RACK_USB_CONSOLE_H_
