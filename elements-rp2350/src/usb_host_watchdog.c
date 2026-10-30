// C-side accessors for the PIO-USB root port, used by main.cpp's USB host
// re-enumeration watchdog.
//
// Why this exists: TinyUSB gives up on enumeration after 3 failed
// control-transfer attempts (usbh.c process_enumeration) and never retries on
// its own. If the MIDI controller re-attaches while it is still booting — or
// an attach edge is lost — the PIO-USB root port stays marked "connected" with
// no device mounted, and since connect detection only fires on the
// disconnected->idle EDGE, no new CONNECT event can ever be generated. The
// controller then stays invisible until it is re-plugged from a fully booted
// state. main.cpp polls these helpers and forces root->connected = false so
// the 1 ms SOF frame loop re-fires the connect event and enumeration restarts.
//
// This must stay a .c file: pio_usb_ll.h declares pio_usb_root_port and friends
// without extern "C", so including it from C++ would emit C++-mangled
// references that do not link against the C-compiled pio_usb library.
#include "pio_usb_ll.h"

// True when the bus sits in an idle J/K state, i.e. a device is electrically
// present on root port 0 (the one PIO-USB bit-bangs on GP14/GP15).
bool pio_host_line_idle(void) {
  const port_pin_status_t line = pio_usb_bus_get_line_state(PIO_USB_ROOT_PORT(0));
  return line == PORT_PIN_FS_IDLE || line == PORT_PIN_LS_IDLE;
}

// Current root-port connect flag as seen by the PIO-USB frame loop.
bool pio_host_connected(void) {
  return PIO_USB_ROOT_PORT(0)->connected;
}

// Pretend the device detached: the next 1 ms frame sees the idle line again,
// re-fires the CONNECT interrupt and TinyUSB re-enumerates from scratch.
void pio_host_force_reconnect(void) {
  PIO_USB_ROOT_PORT(0)->connected = false;
}

// DIAGNOSTIC: raw D+/D- pad levels through the same errata-workaround read
// the host stack uses. A flap with dp collapsing to 0 means the line is
// physically pulled low (pad / power side); dp staying 1 while the folded
// line-state flips means the host's read/drive state is at fault.
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
void pio_host_pins(uint8_t* dp, uint8_t* dm) {
  pio_usb_bus_get_line_state(PIO_USB_ROOT_PORT(0));  // applies IE workaround
  *dp = gpio_get(PIO_USB_ROOT_PORT(0)->pin_dp) ? 1 : 0;
  *dm = gpio_get(PIO_USB_ROOT_PORT(0)->pin_dm) ? 1 : 0;
}

// DIAGNOSTIC: the folded line state (0=SE0 1=FS_IDLE 2=LS_IDLE 3=SE1) the
// connect detection itself uses. If a present full-speed device reads as
// LS_IDLE, the host configures low-speed signaling and enumeration can
// never succeed.
uint8_t pio_host_chip_version(void) {
  uint32_t const chip_id = *((io_ro_32*)(SYSINFO_BASE + SYSINFO_CHIP_ID_OFFSET));
  return (uint8_t)((chip_id & SYSINFO_CHIP_ID_REVISION_BITS) >> SYSINFO_CHIP_ID_REVISION_LSB);
}

// DIAGNOSTIC: raw pad control (pulls, IE) and GPIO function-select for the
// host D+/D- pins. PADS bits: IE=bit4, PUE=bit5, PDE=bit6. CTRL FUNCSEL=bits0-4
// (6 = PIO0). If PDE is 0, the host pulldown was disabled by something.
uint32_t pio_host_pad_ctrl(uint8_t pin) {
  return pads_bank0_hw->io[pin];
}

uint32_t pio_host_gpio_ctrl(uint8_t pin) {
  return io_bank0_hw->io[pin].ctrl;
}

uint8_t pio_host_line_state(void) {
  return (uint8_t)pio_usb_bus_get_line_state(PIO_USB_ROOT_PORT(0));
}
