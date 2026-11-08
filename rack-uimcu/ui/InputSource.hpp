#pragma once

#include <cstring>

namespace CTAG {
    namespace AUDIO {
        // Stored vocabulary for the codec input route. The A1S AudioKit
        // exposes two usable sources: "mic" (the MIC1 pair used as two
        // independent single-ended mics: LIN1/MIC1N -> left ADC and
        // RIN1/MIC1P -> right ADC) and "line" (the stereo line-in jack on
        // ES8388 IN2). MIC2 shares LIN2/RIN2 with the line jack on this
        // board, so it is not a separate source.
        //
        // "line1" and "line2" are the pre-rename spellings and are still found
        // in existing SPIFFS configs. Every reader must accept them, otherwise
        // a page that shows the new names silently rewrites the stored value
        // to the wrong input on its next commit. Keep this the only alias
        // table; SPManager and both UI pages parse through it.
        namespace InputSource {
            static constexpr int COUNT = 2;
            static constexpr const char *NAMES[COUNT] = {"mic", "line"};
            static constexpr const char *OPTIONS = "mic,line";

            // Returns the canonical index, or -1 for an unknown spelling.
            inline int Parse(const char *value) {
                if (value == nullptr) return -1;
                if (std::strcmp(value, "mic") == 0 || std::strcmp(value, "line1") == 0) return 0;
                if (std::strcmp(value, "line") == 0 || std::strcmp(value, "line2") == 0) return 1;
                return -1;
            }
        }
    }
}
