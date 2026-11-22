// USB descriptors for the rack composite device: CDC console + UAC1 capture.
//
// The configuration descriptor composes TinyUSB's CDC template with a
// hand-rolled UAC1 capture function: the TUD_AUDIO_* macros emit UAC2-shaped
// descriptors, so UAC1 bytes are written literally (the DSPi approach). The
// capture function is a second, self-contained audio function — interfaces
// 2 (AudioControl) + 3 (AudioStreaming) under their own IAD — exposing the
// engine output as a 2-ch 16-bit isochronous async IN endpoint (the
// device-vs-golden rig AND a user-facing recording feature).
//
// Pattern reference: DSPi (github.com/WeebLabs/DSPi, GPL-3.0) proved this
// architecture (custom UAC1 app driver + a second capture UAC1 function).
// The implementation here is written from scratch against the TinyUSB API
// and the UAC1 spec for the rack's interface/endpoint map; no DSPi code is
// copied. Descriptor layouts follow the USB-IF UAC1 spec.
//
// Endpoint map (RP2350 hardware: EP1-7 per direction):
//   0x81 IN  interrupt  — CDC notification
//   0x01 OUT bulk       — CDC data
//   0x82 IN  bulk       — CDC data
//   0x83 IN  iso async  — UAC1 capture (16-bit stereo, 44.1/48 kHz)
#include <string.h>

#include "tusb.h"
#include "class/audio/audio.h"   // AUDIO_* constants (not pulled in when CFG_TUD_AUDIO=0)
#include "pico/unique_id.h"

#include "usb_descriptors.h"

// ----------------------------------------------------------------------------
// Identity
// ----------------------------------------------------------------------------
//
// VID = the Raspberry Pi Foundation vendor id (as the Arduino build's CDC).
// PID = a rack-specific value distinct from the SDK's CDC defaults, so the
// host never confuses this composite device with a plain stdio_usb CDC.

#define USB_VENDOR_ID    0x2E8A
#define USB_PRODUCT_ID   0xFEAB     // "Rack RP2350" composite CDC + UAC1 capture
#define USB_BCD_DEVICE   0x0100

#define STRID_MANUFACTURER  1
#define STRID_PRODUCT       2
#define STRID_SERIAL        3

char usb_descriptor_str_serial[33] = "0000000000000000";

static const char *const string_table[] = {
    "",                             // 0 — placeholder; the langid entry is returned separately
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
    // IAD signaling triplet (0xEF/0x02/0x01): with an IAD in the config the
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
//
//   9    config
//   66   CDC function (IAD + ctrl/data itfs, the TUD_CDC_DESCRIPTOR template)
//   102  UAC1 capture function (hand-rolled, below)
//   ---
//   177  total

// (The interface/endpoint/format constants live in usb_descriptors.h — the
// single source shared with usb_capture.cpp.)

// --- UAC1 capture function --------------------------------------------------
//
// 2-ch 16-bit PCM, discrete 44.1/48 kHz, isochronous async IN EP 0x83 with
// NO feedback endpoint: the device is the capture clock master (it streams
// what the engine renders at its own rate), so rate matching happens by the
// packet-size servo in usb_capture.cpp, not by USB feedback.  Same contract
// as an asynchronous capture source.

// The capture function's byte length: IAD(8) + AC std(9) + AC CS(30)
// + AS alt0(9) + AS alt1(9) + AS general(7) + format I(14) + iso EP(9)
// + CS iso EP(7).
#define CAP_DESC_LEN  102

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + CAP_DESC_LEN)

// Sample rate little-endian expansion
#define RATE_LE(r) ((r) & 0xFF), (((r) >> 8) & 0xFF), (((r) >> 16) & 0xFF)

static const uint8_t config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          0x80 /* bus-powered */, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_CTRL, 0, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // --- UAC1 capture function (hand-rolled; see the header comment) ------
    // IAD grouping AC + AS into one audio function.  bInterfaceCount = 2 is
    // what makes TinyUSB's process_set_config() bind BOTH interfaces to the
    // capture class driver (usb_capture.cpp).
    8,
    TUSB_DESC_INTERFACE_ASSOCIATION,    // 0x0B
    ITF_NUM_CAP_AC,                     // bFirstInterface
    0x02,                               // bInterfaceCount (AC + AS)
    TUSB_CLASS_AUDIO,                   // bFunctionClass
    AUDIO_SUBCLASS_CONTROL,             // bFunctionSubClass
    0x00,                               // bFunctionProtocol (UAC1)
    0x00,                               // iFunction

    // AC std interface (itf 2, alt 0, 0 EPs, UAC1 protocol 0x00)
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_CAP_AC,
    0x00,                               // bAlternateSetting
    0x00,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_CONTROL,
    0x00,                               // bInterfaceProtocol (UAC1)
    0x00,                               // iInterface

    // AC CS header (wTotalLength = header 9 + input terminal 12 + output 9)
    9,
    TUSB_DESC_CS_INTERFACE,             // 0x24
    AUDIO_CS_AC_INTERFACE_HEADER,       // 0x01
    U16_TO_U8S_LE(0x0100),              // bcdADC (UAC1.0)
    U16_TO_U8S_LE(30),                  // wTotalLength
    0x01,                               // bInCollection
    ITF_NUM_CAP_AS,                     // baInterfaceNr[0]

    // AC CS input terminal (ID 1, microphone — the engine render)
    12,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,  // 0x02
    CAP_TERM_ID_IN,
    U16_TO_U8S_LE(0x0201),              // wTerminalType = microphone
    0x00,                               // bAssocTerminal
    CAP_N_CHANNELS,                     // bNrChannels
    U16_TO_U8S_LE(0x0003),              // wChannelConfig (L|R)
    0x00,                               // iChannelNames
    0x00,                               // iTerminal

    // AC CS output terminal (ID 2, USB streaming)
    9,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, // 0x03
    CAP_TERM_ID_OUT,
    U16_TO_U8S_LE(0x0101),              // wTerminalType = USB streaming
    0x00,                               // bAssocTerminal
    CAP_TERM_ID_IN,                     // bSourceID
    0x00,                               // iTerminal

    // AS std interface alt 0 (zero-bandwidth)
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_CAP_AS,
    0x00,
    0x00,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // AS std interface alt 1 (streaming: 1 iso IN EP)
    9,
    TUSB_DESC_INTERFACE,
    ITF_NUM_CAP_AS,
    0x01,
    0x01,                               // bNumEndpoints
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    0x00,
    0x00,

    // AS CS general (bTerminalLink = the USB-streaming output terminal)
    7,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,   // 0x01
    CAP_TERM_ID_OUT,                    // bTerminalLink
    0x01,                               // bDelay
    U16_TO_U8S_LE(0x0001),              // wFormatTag = PCM

    // AS CS format type I (16-bit, 2 discrete rates)
    14,
    TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,  // 0x02
    0x01,                               // bFormatType = TYPE_I
    CAP_N_CHANNELS,                     // bNrChannels
    CAP_BYTES_PER_SAMPLE,               // bSubFrameSize
    16,                                 // bBitResolution
    0x02,                               // bSamFreqType (2 discrete)
    RATE_LE(44100),
    RATE_LE(48000),

    // Std iso EP IN 0x83 (iso, async data, no feedback/synch EP)
    9,
    TUSB_DESC_ENDPOINT,
    CAP_EP_IN,
    0x05,                               // bmAttributes: iso + asynchronous
    U16_TO_U8S_LE(CAP_EP_MAX_PKT),      // wMaxPacketSize (208)
    0x01,                               // bInterval
    0x00,                               // bRefresh
    0x00,                               // bSynchAddress

    // CS iso data EP (sampling-frequency control)
    7,
    TUSB_DESC_CS_ENDPOINT,              // 0x25
    AUDIO_CS_EP_SUBTYPE_GENERAL,        // 0x01
    0x01,                               // bmAttributes: sampling freq control
    0x00,                               // bLockDelayUnits
    U16_TO_U8S_LE(0x0000),              // wLockDelay
};

// The array is inferred-size; CONFIG_TOTAL_LEN drives wTotalLength and the
// static assert catches any byte-count slip at compile time.
static_assert(sizeof(config_descriptor) == CONFIG_TOTAL_LEN,
              "config descriptor byte count must equal CONFIG_TOTAL_LEN");

// ----------------------------------------------------------------------------
// String descriptor scratch (returned by pointer; must outlive the transfer)
// ----------------------------------------------------------------------------
static uint16_t string_response[32];

// ----------------------------------------------------------------------------
// TinyUSB descriptor callbacks (defined here; weak defaults unused)
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
