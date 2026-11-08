/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.

A project conceived within the Creative Technologies Arbeitsgruppe of
Kiel University of Applied Sciences: https://www.creative-technologies.de

(c) 2020 by Robert Manzke. All rights reserved.

The CTAG TBD software is licensed under the GNU General Public License
(GPL 3.0), available here: https://www.gnu.org/licenses/gpl-3.0.txt

***************/

// P4: TRS MIDI on the panel (RP-054 Type A: Tip = data). Owns UART1 —
// TX=GPIO1 (TRS OUT), RX=GPIO10 (TRS IN through the opto) — at 31250 8N1.
// DIN-in bytes are batched and relayed to the DSP as
// {"cmd":"link/midiIn","d":[..]} frames; /link/midiOut frames from the DSP
// go to the OUT jack.

#pragma once

#include <cstdint>
#include <cstddef>

namespace CTAG {
    namespace CTRL {
        class MidiLink final {
        public:
            MidiLink() = delete;
            // Install the UART1 driver and spawn the relay task.
            static void Start();
            // DSP → TRS OUT jack (called from the link frame parser).
            static void OnMidiOut(const uint8_t *data, size_t n);
        };
    }
}
