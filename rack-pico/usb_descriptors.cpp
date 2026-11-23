// USB descriptors for the rack composite device: CDC console + USB MIDI +
// UAC1 loopback capture (the device-vs-golden rig).
//
// CDC and MIDI use TinyUSB's built-in class drivers (TUD_CDC_DESCRIPTOR +
// TUD_MIDI_DESCRIPTOR templates). The capture is a second, self-contained
// UAC1 audio function — interfaces 4 (AudioControl) + 5 (AudioStreaming)
// under their own IAD — with hand-rolled bytes: TinyUSB's TUD_AUDIO_* macros
// emit UAC2-shaped descriptors. Pattern reference: DSPi (github.com/WeebLabs/
// DSPi, GPL-3.0), written from scratch against the TinyUSB API and UAC1 spec.
//
// Endpoint map:
//   0x81 IN  interrupt — CDC notification
//   0x01 OUT bulk      — CDC data
//   0x82 IN  bulk      — CDC data
//   0x02 OUT bulk      — MIDI data
//   0x83 IN  bulk      — MIDI data
//   0x84 IN  iso async — UAC1 capture, 2-ch 24-bit, 44.1/48 kHz
#include <string.h>

#include "tusb.h"
#include "class/audio/audio.h"   // AUDIO_* constants (CFG_TUD_AUDIO=0 pulls none)
#include "pico/unique_id.h"

#include "usb_descriptors.h"

// ----------------------------------------------------------------------------
// Identity
// ----------------------------------------------------------------------------

#define USB_VENDOR_ID    0x2E8A
#define USB_PRODUCT_ID   0xFEAB     // "Rack RP2350" composite CDC + MIDI + capture
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
//
// IAD(8) + AC std(9) + AC CS(30) + AS alt0(9) + AS alt1(9) + AS general(7)
// + format I(14) + iso EP(9) + CS iso EP(7) = 102 bytes.
#define CAP_DESC_LEN  102

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + \
                           TUD_MIDI_DESC_LEN + CAP_DESC_LEN)

#define RATE_LE(r) ((r) & 0xFF), (((r) >> 8) & 0xFF), (((r) >> 16) & 0xFF)

static const uint8_t config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          0x80 /* bus-powered */, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_CTRL, 0, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI_CTRL, 0, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 64),

    // --- UAC1 capture function (hand-rolled; the DSPi UAC1 pattern) --------
    // IAD groups AC + AS into one audio function; bInterfaceCount=2 makes
    // TinyUSB bind both interfaces to the capture class driver.
    8,
    TUSB_DESC_INTERFACE_ASSOCIATION,
    ITF_NUM_CAP_AC,
    0x02,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_CONTROL,
    0x00,                               // bFunctionProtocol (UAC1)
    0x00,                               // iFunction

    // AC std interface (itf 4, alt 0, 0 EPs)
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_CAP_AC,
    0x00,
    0x00,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_CONTROL,
    0x00,
    0x00,

    // AC CS header (wTotalLength = header 9 + input 12 + output 9 = 30)
    9,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_HEADER,
    U16_TO_U8S_LE(0x0100),              // bcdADC UAC1.0
    U16_TO_U8S_LE(30),
    0x01,                               // bInCollection
    ITF_NUM_CAP_AS,

    // AC CS input terminal (ID 3, microphone — the engine render)
    12,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,
    CAP_TERM_ID_IN,
    U16_TO_U8S_LE(0x0201),              // microphone
    0x00,                               // bAssocTerminal
    CAP_N_CHANNELS,
    U16_TO_U8S_LE(0x0003),              // L|R
    0x00, 0x00,

    // AC CS output terminal (ID 4, USB streaming)
    9,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL,
    CAP_TERM_ID_OUT,
    U16_TO_U8S_LE(0x0101),              // USB streaming
    0x00,
    CAP_TERM_ID_IN,                     // bSourceID
    0x00,

    // AS std interface alt 0 (zero bandwidth)
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_CAP_AS,
    0x00,
    0x00,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // AS std interface alt 1 (one iso IN EP)
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_CAP_AS,
    0x01,
    0x01,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // AS CS general
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    CAP_TERM_ID_OUT,
    0x01,                               // bDelay
    U16_TO_U8S_LE(0x0001),              // wFormatTag PCM

    // AS CS format type I: 2-ch, 24-bit packed, 44.1/48 kHz discrete
    14,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    0x01,                               // TYPE_I
    CAP_N_CHANNELS,
    CAP_BYTES_PER_SAMPLE,               // 3
    24,                                 // bBitResolution
    0x02,                               // two discrete rates
    RATE_LE(44100),
    RATE_LE(48000),

    // Std iso EP IN 0x84 (async, no feedback EP)
    9,
    TUSB_DESC_ENDPOINT,
    CAP_EP_IN,
    0x05,                               // iso + asynchronous
    U16_TO_U8S_LE(CAP_EP_MAX_PKT),      // 312 bytes
    0x01,                               // bInterval
    0x00, 0x00,

    // CS iso data EP (sampling-frequency control)
    7,
    TUSB_DESC_CS_ENDPOINT,
    AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x01,                               // sampling freq control
    0x00,
    U16_TO_U8S_LE(0x0000),
};

static_assert(sizeof(config_descriptor) == CONFIG_TOTAL_LEN,
              "config descriptor byte count must equal CONFIG_TOTAL_LEN");

// ----------------------------------------------------------------------------
// String descriptor scratch (returned by pointer; must outlive the transfer)
// ----------------------------------------------------------------------------
static uint16_t string_response[32];

extern "C" uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&device_descriptor;
}

extern "C" uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return config_descriptor;
}

extern "C" uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    if (index == 0) {
        string_response[0] = (TUSB_DESC_STRING << 8) | 4;
        string_response[1] = 0x0409;
        return string_response;
    }
    if (index > 3) return NULL;
    const char *str = string_table[index];
    size_t len = strlen(str);
    if (len > 31) len = 31;
    for (size_t i = 0; i < len; i++) string_response[1 + i] = str[i];
    string_response[0] = (TUSB_DESC_STRING << 8) | (uint8_t)(2 * len + 2);
    return string_response;
}

void usb_descriptors_init(void) {
    pico_get_unique_board_id_string(usb_descriptor_str_serial,
                                    sizeof(usb_descriptor_str_serial));
}
