// USB audio capture: the UAC1 class driver + SPSC ring + fill servo.
//
// A custom TinyUSB class driver registered via usbd_app_driver_get_cb() —
// TinyUSB's built-in audio driver is UAC2-only and cannot claim the UAC1
// capture function.  The driver owns interfaces 2 (AC) + 3 (AS) only; the
// CDC function is served by TinyUSB's built-in CDC driver.
//
// Producer/consumer contexts:
//   producer  usb_capture_push_stereo()   core 1 (audio render loop)
//   consumer  fill_audio_packet()         core 0 (tud_task + USB IRQ)
// The contexts never run the consumer concurrently (each packet completion
// re-arms the EP), but producer and consumer are genuinely cross-core, so
// every head/tail publication is bracketed with __dmb() (the RP2350 M33
// write buffer requires it; matches the ring discipline of the rack's
// ISR-side USB buffers).
//
// Rate matching: the device is the capture clock master (no feedback EP).
// Each USB frame the servo emits the nominal frames-per-frame count plus a
// small clamped proportional correction that keeps the ring near its target
// fill; varying the packet size frame-by-frame matches the USB data rate to
// the engine's free-running rate without resampling.
#include <string.h>
#include <stddef.h>

#include "pico.h"                 // __not_in_flash / __not_in_flash_func
#include "hardware/sync.h"        // __dmb()
#include "tusb.h"
#include "device/usbd_pvt.h"      // usbd_class_driver_t, usbd_edpt_*
#include "class/audio/audio.h"    // AUDIO_* constants (CFG_TUD_AUDIO=0 pulls none)

#include "usb_capture.h"
#include "usb_descriptors.h"

// The rack's engine rate (main.cpp owns it; the descriptors report it).
uint32_t rack_engine_rate_hz = 48000;

// ----------------------------------------------------------------------------
// UAC1 request opcodes (not exposed by TinyUSB; its constants are UAC2-only)
// ----------------------------------------------------------------------------

#define CAP_REQ_SET_CUR    0x01
#define CAP_REQ_GET_CUR    0x81
#define CAP_EP_CTRL_SAMPLING_FREQ 0x01

// ----------------------------------------------------------------------------
// SPSC ring of interleaved int16 stereo frames
// ----------------------------------------------------------------------------
//
// 1024 frames = ~21 ms at 48 kHz (4 KiB).  The ring is a decoupler, not a
// buffer: steady state runs near the servo target.

#define CAP_RING_FRAMES  1024u    // power of two
#define CAP_RING_MASK    (CAP_RING_FRAMES - 1u)

static int16_t __attribute__((aligned(4))) cap_ring[CAP_RING_FRAMES * 2];
static volatile uint32_t ring_head;   // next frame to write (producer, core 1)
static volatile uint32_t ring_tail;   // next frame to read  (consumer, core 0)
static volatile uint32_t ring_overflow_count;
static volatile uint32_t ring_underrun_count;

static inline uint32_t ring_count(void) {
    return ring_head - ring_tail;     // unsigned wrap-safe difference
}

uint32_t usb_capture_overflow_count(void) { return ring_overflow_count; }
uint32_t usb_capture_underrun_count(void) { return ring_underrun_count; }

void __not_in_flash_func(usb_capture_push_stereo)(
        const int16_t *l, const int16_t *r, uint32_t frames) {
    uint32_t h = ring_head;
    uint32_t t = ring_tail;
    for (uint32_t i = 0; i < frames; i++) {
        if ((h - t) >= CAP_RING_FRAMES) {
            // Ring full — the host is not recording, or a stall.  Drop the
            // rest of this block; the servo re-primes when the stream wakes.
            ring_overflow_count += (frames - i);
            break;
        }
        uint32_t idx = (h & CAP_RING_MASK) * 2u;
        cap_ring[idx]     = l[i];
        cap_ring[idx + 1] = r[i];
        h++;
    }
    __dmb();          // the slot writes must be visible before head advances
    ring_head = h;
}

// ----------------------------------------------------------------------------
// Fill servo (all USB callbacks below run in tud_task()/IRQ context, core 0)
// ----------------------------------------------------------------------------

// Target stereo frames buffered in the ring: ~5.3 ms at 48 kHz keeps latency
// low while leaving ample margin against under/overflow.
#define CAP_TARGET_FILL_FRAMES  256u

// The engine and host clocks differ only by a few ppm, so only a tiny
// correction is ever needed in steady state; the clamp bounds the pull-in
// rate so the capture is never noticeably time-warped.
#define CAP_SERVO_KP       0.008f   // (frames/frame) per (frame) of fill error
#define CAP_SERVO_MAX_CORR 2.0f     // max |correction| in frames/frame

static uint32_t g_last_freq      = 0;
static float    g_nominal_fpf    = 48.0f;  // nominal stereo frames per USB frame
static float    g_acc            = 0.0f;   // fractional-frame accumulator
static uint32_t g_silence_frames = 48;     // silence packet size = rate / 1000
static bool     g_primed         = false;  // ring reached the target at least once

static uint8_t __attribute__((aligned(4))) g_pkt[CAP_EP_MAX_PKT];

static inline void refresh_rate(void) {
    uint32_t f = rack_engine_rate_hz;
    if (f != g_last_freq) {
        g_last_freq      = f;
        g_nominal_fpf    = (float)f / 1000.0f;
        g_silence_frames = f / 1000u;
        g_acc            = 0.0f;
    }
}

static inline uint16_t silence_bytes(void) {
    return (uint16_t)(g_silence_frames * CAP_BYTES_PER_FRAME);
}

// Fill 'buf' with the next isochronous IN packet; returns its size in bytes.
// Runs once per USB frame (each time the previous IN packet completes).
static uint16_t __not_in_flash_func(fill_audio_packet)(uint8_t *buf) {
    refresh_rate();

    uint32_t avail = ring_count();

    // Prime: wait until the ring fills to the target before streaming.
    if (!g_primed) {
        if (avail >= CAP_TARGET_FILL_FRAMES) {
            g_primed = true;
            g_acc    = 0.0f;
        } else {
            uint16_t n = silence_bytes();
            memset(buf, 0, n);
            return n;
        }
    }

    // Rate-matching servo: nominal feed-forward + clamped P correction.
    int   err  = (int)avail - (int)CAP_TARGET_FILL_FRAMES;
    float corr = CAP_SERVO_KP * (float)err;
    if (corr >  CAP_SERVO_MAX_CORR) corr =  CAP_SERVO_MAX_CORR;
    if (corr < -CAP_SERVO_MAX_CORR) corr = -CAP_SERVO_MAX_CORR;

    g_acc += g_nominal_fpf + corr;
    int n = (int)g_acc;             // floor; g_acc is always >= 0
    g_acc -= (float)n;

    if (n < 0) n = 0;
    if (n > CAP_MAX_FRAMES_PER_PACKET) n = CAP_MAX_FRAMES_PER_PACKET;
    if ((uint32_t)n > avail) n = (int)avail;

    if (n == 0) {
        // Real underrun: keep the stream alive and re-prime.  Counted once
        // per episode; the following packets take the prime branch.
        ring_underrun_count++;
        g_primed = false;
        uint16_t z = silence_bytes();
        memset(buf, 0, z);
        return z;
    }

    // Copy n stereo frames into the packet: int16 -> 24-bit packed LE.
    // The DAC samples are int16; the MSB byte carries the sign extension.
    uint32_t t = ring_tail;
    uint8_t *dst = buf;
    for (int i = 0; i < n; i++) {
        uint32_t idx = (t & CAP_RING_MASK) * 2u;
        int32_t lv = (int32_t)cap_ring[idx] << 8;
        int32_t rv = (int32_t)cap_ring[idx + 1] << 8;
        t++;
        *dst++ = (uint8_t)(lv & 0xFF);
        *dst++ = (uint8_t)((lv >> 8) & 0xFF);
        *dst++ = (uint8_t)((lv >> 16) & 0xFF);
        *dst++ = (uint8_t)(rv & 0xFF);
        *dst++ = (uint8_t)((rv >> 8) & 0xFF);
        *dst++ = (uint8_t)((rv >> 16) & 0xFF);
    }
    __dmb();          // the slot reads must complete before tail advances
    ring_tail = t;

    return (uint16_t)(n * CAP_BYTES_PER_FRAME);
}

// ----------------------------------------------------------------------------
// UAC1 capture class driver
// ----------------------------------------------------------------------------

static struct {
    uint8_t        ac_itf;       // capture AudioControl interface number
    uint8_t        as_itf;       // capture AudioStreaming interface number
    uint8_t        cur_alt;      // current AS alternate setting
    bool           ep_open;      // iso IN endpoint open
    const uint8_t *ep_desc;      // pointer to the iso IN endpoint descriptor
    uint8_t        pending_cs;   // deferred SET_CUR control selector
} cap;

static uint8_t cap_ctrl_buf[8];   // SET_CUR data-stage scratch

// Fill the scratch packet via the servo and queue it on the IN endpoint.
static void __not_in_flash_func(cap_arm_in)(uint8_t rhport) {
    uint16_t len = fill_audio_packet(g_pkt);
    usbd_edpt_xfer(rhport, CAP_EP_IN, g_pkt, len);
}

// Apply an AudioStreaming alternate setting (0 = idle, 1 = streaming).
static bool cap_apply_alt(uint8_t rhport, uint8_t alt) {
    if (alt > 1) return false;
    if (alt == cap.cur_alt) return true;   // idempotent SET_INTERFACE
    cap.cur_alt = alt;

    if (alt == 1) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
        TU_ASSERT(usbd_edpt_iso_activate(rhport,
                   (tusb_desc_endpoint_t const *)cap.ep_desc));
#else
        TU_ASSERT(usbd_edpt_open(rhport,
                   (tusb_desc_endpoint_t const *)cap.ep_desc));
#endif
        // Clear any stale iso EP state (AVAIL/busy) left from a previous alt
        // cycle; stall+clear flushes both the stack and hardware flags.
        usbd_edpt_stall(rhport, CAP_EP_IN);
        usbd_edpt_clear_stall(rhport, CAP_EP_IN);

        cap.ep_open = true;

        // Drop any stale audio so capture starts at "now", then re-prime.
        // The glitch counters measure THIS stream, not idle time before it.
        ring_tail = ring_head;
        ring_overflow_count = 0;
        ring_underrun_count = 0;
        g_primed  = false;
        g_acc     = 0.0f;

        cap_arm_in(rhport);   // queue the first packet
    } else {
        if (cap.ep_open) {
            usbd_edpt_close(rhport, CAP_EP_IN);
            cap.ep_open = false;
        }
    }
    return true;
}

static void cap_driver_init(void) {
    memset(&cap, 0, sizeof(cap));
}

static bool cap_driver_deinit(void) {
    return true;
}

static void cap_driver_reset(uint8_t rhport) {
    (void)rhport;
    cap.cur_alt = 0;
    cap.ep_open = false;
    g_primed    = false;
}

// Claim the capture AC + AS interfaces and reserve the iso IN endpoint.
static uint16_t cap_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    // Scope strictly to the capture AudioControl interface so this driver
    // never claims the CDC function.
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);
    TU_VERIFY(itf_desc->bInterfaceNumber == ITF_NUM_CAP_AC);

    cap.ac_itf = itf_desc->bInterfaceNumber;

    const uint8_t *p   = (const uint8_t *)itf_desc;
    const uint8_t *end = p + max_len;
    uint16_t drv_len = 0;

    // AC standard interface descriptor
    drv_len += tu_desc_len(p);
    p       += tu_desc_len(p);

    // AC class-specific descriptors (header, terminals)
    while (p < end && tu_desc_type(p) == TUSB_DESC_CS_INTERFACE) {
        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);
    }

    // AS interfaces (alt 0 + alt 1) and their CS + endpoint descriptors
    while (p < end && tu_desc_type(p) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *)p;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO ||
            as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) {
            break;
        }
        cap.as_itf = as->bInterfaceNumber;

        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);

        while (p < end && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
            if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS &&
                    tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
                    cap.ep_desc = p;   // remembered for iso (de)activation
#ifdef TUP_DCD_EDPT_ISO_ALLOC
                    usbd_edpt_iso_alloc(rhport, ep->bEndpointAddress, CAP_EP_MAX_PKT);
#endif
                }
            }
            drv_len += tu_desc_len(p);
            p       += tu_desc_len(p);
        }
    }

    return drv_len;
}

static bool cap_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage == CONTROL_STAGE_SETUP) {
        // Standard requests on our interfaces
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            uint8_t itf = TU_U16_LOW(req->wIndex);

            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == cap.ac_itf) {
                    return (alt == 0) ? tud_control_status(rhport, req) : false;
                }
                if (itf == cap.as_itf) {
                    return cap_apply_alt(rhport, alt) ? tud_control_status(rhport, req) : false;
                }
                return false;
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                static uint8_t alt_resp;
                if (itf == cap.ac_itf)      alt_resp = 0;
                else if (itf == cap.as_itf) alt_resp = cap.cur_alt;
                else                        return false;
                return tud_control_xfer(rhport, req, &alt_resp, 1);
            }
            return false;
        }

        // Class requests: sampling-frequency control on the capture EP
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {

            if (TU_U16_LOW(req->wIndex) != CAP_EP_IN) return false;
            uint8_t cs = TU_U16_HIGH(req->wValue);

            if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
                // GET_CUR sampling frequency — report the engine rate.
                if (req->bRequest == CAP_REQ_GET_CUR && cs == CAP_EP_CTRL_SAMPLING_FREQ) {
                    static uint8_t freq[3];
                    uint32_t f = rack_engine_rate_hz;
                    freq[0] = (uint8_t)(f & 0xFF);
                    freq[1] = (uint8_t)((f >> 8) & 0xFF);
                    freq[2] = (uint8_t)((f >> 16) & 0xFF);
                    return tud_control_xfer(rhport, req, freq, 3);
                }
                return false;
            } else {
                // SET_CUR sampling frequency: accept, but the servo paces to
                // rack_engine_rate_hz regardless (the device is the clock
                // master; a mismatched host rate plays at the wrong pitch —
                // by design, the asynchronous-capture contract).
                if (req->bRequest == CAP_REQ_SET_CUR && cs == CAP_EP_CTRL_SAMPLING_FREQ) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(cap_ctrl_buf)) return false;
                    cap.pending_cs = cs;
                    return tud_control_xfer(rhport, req, cap_ctrl_buf, len);
                }
                return false;
            }
        }
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        // SET_CUR data stage — value validated but not stored (informational).
        cap.pending_cs = 0;
        return true;
    }

    return true;   // CONTROL_STAGE_ACK
}

static bool __not_in_flash_func(cap_driver_xfer_cb)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void)result;
    (void)xferred_bytes;
    if (ep_addr == CAP_EP_IN) {
        if (cap.ep_open) cap_arm_in(rhport);   // refill + queue the next frame
        return true;
    }
    return false;
}

const usbd_class_driver_t usb_capture_uac1_driver = {
    .name            = "RACK_CAP",
    .init            = cap_driver_init,
    .deinit          = cap_driver_deinit,
    .reset           = cap_driver_reset,
    .open            = cap_driver_open,
    .control_xfer_cb = cap_driver_control_xfer_cb,
    .xfer_cb         = cap_driver_xfer_cb,
    .sof             = NULL,   // capture is the clock master — no feedback servo
};

// TinyUSB looks up this weak symbol during tud_init().  Returning the capture
// driver here makes TinyUSB dispatch all events for the capture interfaces
// through the callbacks above.
//
// NOTE: the definition inherits TU_ATTR_WEAK from the prototype in
// usbd_pvt.h (C++ propagates the attribute to the definition).  Both this
// and usbd.c's stub are therefore weak, and GNU ld resolves them by link
// order — this object precedes the tinyusb library (add_executable sources
// first), so ours wins.  Keep usb_capture.cpp in the executable sources.
extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &usb_capture_uac1_driver;
}
