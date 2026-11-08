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

// Panel display driver, C3 port (SDD §5.1): SSD1309 128x64 over SPI, behind
// the exact same framebuffer API the A1S I²C backend served. All drawing
// primitives and fonts are shared with the A1S copy; only Init/Flush/
// Sleep/Wake/Contrast touch hardware.
//
// Wiring (SDD §2.1, DC remapped): SCK=GPIO6, MOSI=GPIO7, CS=GPIO2,
// DC=GPIO8, RST tied to 3V3 through an RC. Verify against the physical
// board revision before bring-up.

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

// Layout constants — change these when swapping fonts
static constexpr int FONT_H = 7;
static constexpr int LINE_H = FONT_H + 1;
static constexpr int ITEM_Y0 = 5;
static constexpr int ITEM_Y0_HDR = ITEM_Y0 + FONT_H + 1;
static constexpr int VISIBLE_ITEMS = 7;
static constexpr int VISIBLE_ITEMS_HDR = 6;
static constexpr int SCROLLBAR_X = 126;

#define ROW(n)     (ITEM_Y0 + (n) * LINE_H)
#define ROW_HDR(n) (ITEM_Y0_HDR + (n) * LINE_H)

inline void ClampScroll(int cursor, int &scrollOffset, int total, int visible) {
    if (cursor - scrollOffset < 0) scrollOffset = cursor;
    if (cursor - scrollOffset >= visible) scrollOffset = cursor - (visible - 1);
    if (scrollOffset > total - visible) scrollOffset = total - visible;
    if (scrollOffset < 0) scrollOffset = 0;
}

inline int ClampVisible(int total, int scrollOff, int maxVis) {
    int v = total - scrollOff;
    return v > maxVis ? maxVis : v;
}

namespace CTAG {
    namespace DRIVERS {
        class Display final {
        public:
            // Font selection
            enum Font { FONT_8X8, FONT_5X7, FONT_DIGI_SLIM_3X6, FONT_DIGI_ONE_5X6, FONT_NORNS_6X7 };

            static void Flush();

            static void Sleep();
            static void Wake();
            static void Contrast(int val);

            Display() = delete;
            static void Init();
            // Bench diagnostics (doc/uimcu-bringup.md): send one raw SSD1309
            // command byte (e.g. 0xA5 all-pixels-on, 0xA4 back to RAM) and
            // re-run the whole init after a re-wire.
            static void RawCommand(uint8_t cmd);
            static void Reinit();
            // Bench bring-up: bit-bang the SSD1309 init + all-pixels-on
            // through one of the plausible pin permutations (SCK/MOSI/CS/DC
            // swap variants), leaving the panel lit if that mapping is the
            // right one. mode 0 = as documented, 1 = SDA/SCLK swapped,
            // 2 = CS/DC swapped, 3 = both swapped. Diagnostic only.
            static void BenchScan(int mode);
            // Retarget CS/DC GPIOs + reinit (CS/DC swap diagnostic).
            static void BenchPins(int cs, int dc);
            // framebuffer size in bytes: 128x64 mono, page-major (page = y>>3)
            static constexpr int FRAMEBUFFER_SIZE = 1024;
            // Copy the current framebuffer out. Must be called from the task
            // that draws (UIMenu), otherwise the copy can be torn.
            static void CopyFramebuffer(uint8_t *dst);
            static void Clear();
            static void ShowFWVersion();

            // Framebuffer drawing primitives
            static void DrawPixel(int x, int y, bool on = true);
            static void DrawHLine(int x, int y, int w, bool on = true);
            static void DrawVLine(int x, int y, int h, bool on = true);
            static void DrawRect(int x, int y, int w, int h, bool fill = false, bool on = true);
            static void InvertRect(int x, int y, int w, int h);
            static void DrawString(int x, int y, const char *str, Font font = FONT_8X8);
            static void DrawStringRight(int x, int y, const char *str, Font font = FONT_8X8);
            static void DrawVUMeter(int x, int y, int w, int h, float level);
            static void DrawVUMeterV(int x, int y, int w, int h, float level);
            static void DrawScrollbar(int x, int y, int h, int totalItems, int cursorPos);
            static int FontAdvance(Font f);
            static int FontAdvance(Font f, unsigned char c);

        private:
            // framebuffer: page-major, column-minor
            static uint8_t fb[1024];
            static bool dirtyPages[8];

            static void MarkDirty(int page);
            static int userContrast;

            // SPI SSD1309 backend
            static bool sendCommand(uint8_t cmd);
            static bool sendData(const uint8_t *data, size_t len);
        };
    }
}
