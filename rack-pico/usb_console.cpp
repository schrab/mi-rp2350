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
#include "pico/bootrom.h"
#include "tusb.h"

#include "usb_console.h"

// 1200-baud BOOTSEL touch (the earlephilhower CDC behavior the documented
// flash recipe depends on — AGENTS.md's stty/pyserial flow): a CDC
// connection at 1200 baud resets the board into the USB bootloader with
// zero flash writes. Without this callback the composite CDC ignores the
// touch and every soft flash path dies (bench-caught 2026-11-23: the first
// flash of this stack worked only because the board was still running the
// Arduino firmware at the time).
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    (void)itf;
    if (coding->bit_rate == 1200) {
        reset_usb_boot(0, 0);
    }
}

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

// ----------------------------------------------------------------------------
// Console byte API (the Serial.* surface the ported main.cpp calls):
// printf reaches these through the stdio bridge; the helpers below are the
// read/binary-write side. All non-blocking (drop, never spin) — the core-0
// discipline law.
// ----------------------------------------------------------------------------
int cdc_read(void) {
    char c;
    return tud_cdc_read(&c, 1) == 1 ? static_cast<int>(static_cast<uint8_t>(c))
                                    : -1;
}

int cdc_available(void) {
    return tud_cdc_available() > 0 ? 1 : 0;
}

void cdc_write_raw(const void* buf, size_t len) {
    tud_cdc_write(buf, static_cast<uint32_t>(len));
    tud_cdc_write_flush();
}

int cdc_available_for_write(void) {
    return static_cast<int>(tud_cdc_write_available());
}
