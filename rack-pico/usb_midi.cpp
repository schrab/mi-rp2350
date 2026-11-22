// USB MIDI device pump (TinyUSB built-in MIDI class driver, CFG_TUD_MIDI=1).
//
// Drains 4-byte USB-MIDI event packets from the MIDI OUT endpoint into the
// rack's shared funnel midiHandleEventPacket() (rack-rp2350/src/midi.h) —
// the exact contract the Arduino build's Adafruit_USBD_MIDI readPacket path
// served, byte-for-byte: pkt[0] = (cable<<4)|CIN, [1] = status, [2]/[3] =
// data. Core-0 loop context (usb_console_tick runs the TinyUSB task).
#include "tusb.h"

#include "usb_midi.h"
#include "../rack-rp2350/src/midi.h"

void usb_midi_pump(void) {
    if (!tud_midi_mounted()) return;
    uint8_t pkt[4];
    while (tud_midi_packet_read(pkt)) {
        midiHandleEventPacket(pkt);
    }
}
