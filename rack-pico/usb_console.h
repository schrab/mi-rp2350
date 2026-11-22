// CDC console bridge (see usb_console.cpp).
#ifndef _RACK_USB_CONSOLE_H_
#define _RACK_USB_CONSOLE_H_

// Register the CDC stdio driver (after stdio_init_all(); before use).
void usb_console_init(void);

// Core-0 pump: runs the TinyUSB device task and flushes pending CDC output.
// Call at ~1 ms cadence from the core-0 loop.
void usb_console_tick(void);

// Console byte API (the ported main.cpp's Serial.* replacements). All
// non-blocking. printf reaches the console through the registered stdio
// driver; cdc_write_raw is the binary path (bjdump payload, dbg pump).
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
int cdc_read(void);
int cdc_available(void);
void cdc_write_raw(const void* buf, size_t len);
int cdc_available_for_write(void);
#ifdef __cplusplus
}
#endif

#endif  // _RACK_USB_CONSOLE_H_
