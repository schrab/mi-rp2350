// SEED REFERENCE, NOT BUILT (T5 stages 2-6): excluded from the build
// (main/CMakeLists.txt) — it includes the seed's ctag/link/protocol.hpp and
// the seed `protocol` component dependency was dropped; WiFi/REST stays off
// (the seed's own heap budgeting disabled it too).
/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

***************/

// P5: panel-side WiFi (softAP) + REST bridge. The panel serves the WebUI's
// /api/v1/* surface by reading the DspProxy cache (state pushes keep it
// current) and forwarding mutations as link commands — the DSP needs no
// REST server of its own. Static WebUI files: later stage (they live on
// the DSP's SPIFFS today).

#pragma once

namespace CTAG {
    namespace CTRL {
        class NetRest final {
        public:
            NetRest() = delete;
            // NVS + netif + softAP ("TBD-Panel", open) + HTTP server.
            static void Start();
        };
    }
}
