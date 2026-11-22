// CDC console: bridges the pico stdio framework onto TinyUSB's CDC driver.
//
// The composite descriptors are ours (usb_descriptors.cpp), so the SDK's
// pico_stdio_usb (which ships its own CDC-only descriptors) is not linked;
// printf/getchar reach the console through this registered stdio driver
// instead.  The core-0 loop calls usb_console_tick() at ~1 ms cadence to run
// the TinyUSB task and push pending output — the ring-buffered diagnostic
// pump discipline of the Arduino build, kept for the Pico SDK build.
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "tusb.h"

#include "usb_console.h"

static void cdc_out_chars(const char *buf, int len) {
    tud_cdc_write(buf, (uint32_t)len);
}

static void cdc_out_flush(void) {
    tud_cdc_write_flush();
}

static int cdc_in_chars(char *buf, int len) {
    return (int)tud_cdc_read(buf, (uint32_t)len);
}

static stdio_driver_t cdc_stdio_driver;

void usb_console_init(void) {
    cdc_stdio_driver.out_chars = cdc_out_chars;
    cdc_stdio_driver.out_flush = cdc_out_flush;
    cdc_stdio_driver.in_chars  = cdc_in_chars;
    cdc_stdio_driver.set_chars_available_callback = NULL;
    cdc_stdio_driver.next      = NULL;
    cdc_stdio_driver.last_ended_with_cr = false;
    cdc_stdio_driver.crlf_enabled = false;
    stdio_set_driver_enabled(&cdc_stdio_driver, true);
}

void usb_console_tick(void) {
    tud_task();             // TinyUSB device task (drivers, EP0, xfer setup)
    tud_cdc_write_flush();  // push pending stdio bytes to the host
}
