/***************
Rack UI controller — USB-Serial-JTAG line console (replaces the seed's
STX/ETX-JSON DebugLink; the seed's proven transport pattern is kept: install
the usb_serial_jtag DRIVER (own ISR + rings), hand ESP_LOG to the driver so
logging keeps flowing on the same wire).

Commands (line-based, 115200-style plain text over the USB-JTAG console):
  help            this list
  ui              link + page state (RackUI::DebugState)
  page <0-5>      switch page without the encoder
  pots            raw + mapped value of all 16 HC4067 pots
  calib           8 s min/max sweep on all pots -> NVS
  calib clear     reset to the full raw range
  testpat <0-4>   ST7789P3 bring-up patterns (0 = back to the UI)
  heap            free-heap report (NimBLE/FB budget check)
***************/

#pragma once

namespace CTAG {
    namespace CTRL {
        class RackConsole {
        public:
            static void Start();
        };
    }
}
