// USB descriptors for the rack composite device: CDC console + USB MIDI.
//
// Both functions use TinyUSB's built-in class drivers (CDC + MIDI), so the
// config descriptor composes the TinyUSB templates (TUD_CDC_DESCRIPTOR +
// TUD_MIDI_DESCRIPTOR) — no hand-rolled bytes. The Arduino build's MIDI
// contract (Adafruit_USBD_MIDI, "Rack RP2350" product string, 4-byte event
// packets) is preserved exactly.
//
// The UAC1 capture function is PARKED (RACK_PICO_USB_CAPTURE, phase 3):
// this file previously carried a hand-rolled UAC1 capture function per the
// DSPi architecture; it returns behind the CMake option.
//
// Endpoint map (RP2350 hardware: EP1-7 per direction):
//   0x81 IN  interrupt — CDC notification
//   0x01 OUT bulk      — CDC data
//   0x82 IN  bulk      — CDC data
//   0x02 OUT bulk      — MIDI data
//   0x83 IN  bulk      — MIDI data
#include <string.h>

#include "tusb.h"
#include "pico/unique_id.h"

#include "usb_descriptors.h"

// ----------------------------------------------------------------------------
// Identity
// ----------------------------------------------------------------------------

#define USB_VENDOR_ID    0x2E8A
#define USB_PRODUCT_ID   0xFEAB     // "Rack RP2350" composite CDC + USB MIDI
#define USB_BCD_DEVICE   0x0100

#define STRID_MANUFACTURER  1
#define STRID_PRODUCT       2
#define STRID_SERIAL        3

char usb_descriptor_str_serial[33] = "0000000000000000";

static const char *const string_table[] = {
    "",                             // 0 — placeholder; langid returned separately
    "rack",                         // 1 — manufacturer
    "Rack RP2350",                  // 2 — product
    usb_descriptor_str_serial,      // 3 — serial (populated at boot)
};

// ----------------------------------------------------------------------------
// Device descriptor
// ----------------------------------------------------------------------------

static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    // IAD signaling triplet (0xEF/0x02/0x01): with IADs in the config the
    // device must advertise MISC/COMMON/IAD so the host spawns a composite
    // parent and binds per-function drivers (the USB-IF IAD ECN rule).
    .bDeviceClass       = TUSB_CLASS_MISC,        // 0xEF
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,   // 0x02
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,      // 0x01
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VENDOR_ID,
    .idProduct          = USB_PRODUCT_ID,
    .bcdDevice          = USB_BCD_DEVICE,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

// ----------------------------------------------------------------------------
// Configuration descriptor
// ----------------------------------------------------------------------------

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + \
                           TUD_MIDI_DESC_LEN)

static const uint8_t config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          0x80 /* bus-powered */, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_CTRL, 0, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI_CTRL, 0, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 64),
};

static_assert(sizeof(config_descriptor) == CONFIG_TOTAL_LEN,
              "config descriptor byte count must equal CONFIG_TOTAL_LEN");

// ----------------------------------------------------------------------------
// String descriptor scratch (returned by pointer; must outlive the transfer)
// ----------------------------------------------------------------------------
static uint16_t string_response[32];

// ----------------------------------------------------------------------------
// TinyUSB descriptor callbacks
// ----------------------------------------------------------------------------

extern "C" uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_descriptor;
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;  // index-1 was requested; only one configuration exists
    return config_descriptor;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == 0) {
        string_response[0] = (TUSB_DESC_STRING << 8) | 4;
        string_response[1] = 0x0409;  // English (US)
        return string_response;
    }

    if (index > 3) return NULL;
    const char *str = string_table[index];

    size_t len = strlen(str);
    if (len > 31) len = 31;
    for (size_t i = 0; i < len; i++) {
        string_response[1 + i] = str[i];
    }
    string_response[0] = (TUSB_DESC_STRING << 8) | (uint8_t)(2 * len + 2);
    return string_response;
}

// Serial: the board's unique id (populated before tud_init() runs).
void usb_descriptors_init(void) {
    pico_get_unique_board_id_string(usb_descriptor_str_serial,
                                    sizeof(usb_descriptor_str_serial));
}
