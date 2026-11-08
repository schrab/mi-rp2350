/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

The CTAG TBD hardware design is released under the Creative Commons
Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0).
Details here: https://creativecommons.org/licenses/by-nc-sa/4.0/

CTAG TBD is provided "as is" without any express or implied warranties.

License and copyright details for specific submodules are included in their
respective component folders / files if different from this license.
***************/

// Panel-side link client (doc/ui-split-sdd.md §4/§5.1). Owns UART0 on its
// default pins (TX=GPIO21, RX=GPIO20 — the SDD link pins; the console is
// USB-Serial-JTAG only) at 921600 8N1, driven by the uart driver like the
// A1S DebugUI owns UART0 there.
//
// Duties: version handshake (retry every second until the DSP answers),
// heartbeat, offline watchdog, framed TX for DspProxy commands, and the one
// synchronous round trip the pages need (SD directory listings). Every
// complete frame is handed to DspProxy::OnLinkFrame, which owns the cache.

#pragma once

#include <string>
#include <cstdint>

namespace CTAG {
    namespace CTRL {
        class LinkClient final {
        public:
            LinkClient() = delete;
            // Install the UART0 driver and spawn the link task.
            static void Start();
            // Debug-only: retune the link UART at runtime (baud sweep for
            // the handshake hunt). False when the rate is out of range.
            static bool SetBaud(int baud);
            // True between the version handshake and the offline watchdog.
            static bool IsOnline();
            // Send one framed command (fire and forget; replies arrive as
            // pushes/frames through DspProxy::OnLinkFrame).
            static bool SendCommand(const std::string &payload);
            // Synchronous SD listing round trip. On success `entries` holds
            // the raw JSON array [["name",isDir],...]. Returns false on
            // timeout / offline.
            static bool RequestFsList(const char *path, std::string &entries,
                                      int timeoutMs = 400);
        };
    }
}
