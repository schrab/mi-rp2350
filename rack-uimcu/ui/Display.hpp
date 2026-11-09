/***************
Rack UI controller — T5 stage 2 display backend (ST7789P3 IPS 284x76 bar).

REPLACES the seed's SSD1309 128x64 mono backend wholesale (the seed
Display.cpp is the A1S framebuffer shape; none of it fits a 16-bpp color
bar). The class NAME stays `CTAG::DRIVERS::Display` so the renderer and
console read naturally.

Backend choice: HAND-ROLLED SPI driver (CASET/RASET/RAMWR), NOT esp_lcd.
Justification: this panel is a masked/cropped wide bar on an ST7789
controller (native 240x320 portrait) — the real window + orientation are
bring-up DISCOVERY items (284 > 240, so the panel must be scanning the
320-wide axis; the offsets/madctl below are the documented starting guess).
esp_lcd's st7789 panel driver assumes the standard panel geometry and makes
arbitrary window offsets/scan direction awkward, while the hand-rolled
CASET/RASET path is ~120 lines, gives exact control of the window (the
discovery knobs are the #defines below) and coalesces dirty rows into
single window transfers.

Wiring (rack-uimcu/README.md has the full C3 GPIO allocation table):
  SCK = GPIO10, MOSI = GPIO20 (GPIO-matrix SPI @ 26.67 MHz, SPI mode 3),
  DC  = GPIO21, RST  = GPIO7 (pulsed by firmware after SPI2 bus acquire so
        SCK is already idling HIGH before reset releases),
  CS  = tied GND (single device on bus),
  BL  = tied 3V3 (always backlit; no PWM dimming — pin budget).

Framebuffer: full 284x76 RGB565 (~43 KB internal RAM, affordable on the C3
per the plan) with PER-ROW dirty flags; Flush() coalesces consecutive dirty
rows into one windowed SPI transfer. The renderer redraws at 30-60 Hz.

Bring-up discovery (docs/ui-protocol.md §8.5 + rack-uimcu/README.md):
`testpat` on the console draws the 1-px frame at the CONFIGURED window
edges + ruler marks every 20 px + the 4 hue bars; if the image is shifted,
wrap, or mirrored, adjust the #defines below (XOFF/YOFF/MADCTL) and re-flash.
***************/

#pragma once

#include <cstdint>
#include <cstddef>

namespace CTAG {
    namespace DRIVERS {
        class Display final {
        public:
            // Logical framebuffer (what the renderer draws into).
            static constexpr int W = 284;
            static constexpr int H = 76;
            static constexpr int FRAMEBUFFER_SIZE = W * H * 2;

            // Fonts: labels = digi-slim (6 px), values = digi-one (5 px).
            enum Font : uint8_t { FONT_LABEL = 0, FONT_VALUE = 1 };

            // RGB565 palette — conversions shown in Display.cpp (the trio is
            // the Mutable panel palette; yellow = STATE accent only).
            static constexpr uint16_t kWhite  = 0xE73C;  // #e4e4e4-class
            static constexpr uint16_t kRed    = 0xE14B;  // #e6295e
            static constexpr uint16_t kTeal   = 0x1517;  // #17a3bd
            static constexpr uint16_t kYellow = 0xF683;  // #f2d21f (accent)
            static constexpr uint16_t kBlack  = 0x0000;

            // 50% dim (blend toward black; used for the chain-B tint).
            static constexpr uint16_t Dim(uint16_t c) {
                return (uint16_t)((c & 0xF7DE) >> 1);
            }

            Display() = delete;

            static void Init();
            static void Clear(uint16_t color = kBlack);

            // Drawing primitives (into the framebuffer; mark rows dirty).
            static void DrawPixel(int x, int y, uint16_t c);
            static void DrawHLine(int x, int y, int w, uint16_t c);
            static void FillRect(int x, int y, int w, int h, uint16_t c);

            // Text: digi-slim (labels) / digi-one (values), column-major
            // LSB-top glyphs, per-char advances from the font headers.
            static void DrawString(int x, int y, const char *s, Font f,
                                   uint16_t c);
            static int StringWidth(const char *s, Font f);

            // Push dirty rows (coalesced windows) over SPI.
            static void Flush();

            // ---- bring-up test patterns (console `testpat [0-4]`) ----
            // 0 = off (back to the UI), 1 = full white, 2 = full black,
            // 3 = the 4 hue bars, 4 = border + ruler (offset discovery).
            // While a pattern is active the renderer holds off (the UI does
            // not paint over the diagnostic).
            static void TestPattern(int mode);
            static bool TestPatternActive();

            // `bbtest` bench command (2026-11-09): raw-GPIO bit-banged panel
            // test on the same pads — SLPOUT..DISPON + a white top band, no
            // SPI peripheral involved. Decides "SPI setup vs wires/panel".
            static void BitBangTest();
            // `spiflood` bisect test: full-GRAM color flood through the SPI
            // path with the MINIMAL init (Display.cpp).
            static void SpiFlood();

            // `osweep` bench command: MADCTL × window-mirror sweep through
            // the SPI path (16 combos × 4-quadrant pattern, 2.5 s each) —
            // the correct combo = quadrants land as drawn.
            static void OrientationSweep();

        private:
            static uint16_t fb[W * H];
            static bool dirty[H];
            static int s_testpat;      // -1 = UI owns the screen
            static bool sendCommand(uint8_t cmd, const uint8_t *data = nullptr,
                                    size_t len = 0);
            static void setWindow(int x0, int y0, int x1, int y1);
            // kSt7789InitSeq walker (bitbang=false -> SPI, true -> raw GPIO);
            // shared by Init() and BitBangTest() — Display.cpp.
            static void runInitSeq(bool bitbang);
            static void writePixels(const uint16_t *px, size_t n);
            static void markDirty(int y);
        };
    }
}
