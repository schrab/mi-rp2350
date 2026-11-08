/***************
Rack UI controller — ST7789P3 284x76 RGB565 backend implementation (see
Display.hpp for the backend-choice justification + wiring).

SPI transaction shape: DC=GPIO low for commands, high for data; CS tied
GND (single device). Commands used: SWRESET/SLPOUT/COLMOD/MADCTL/INVON/
NORON/DISPON + the windowing trio CASET(0x2A)/RASET(0x2B)/RAMWR(0x2C).
The 284x76 bar is a masked window on the 240x320 controller: the driver
addresses ONLY the configured window — every flush sets
CASET = [XOFF .. XOFF+283], RASET = [YOFF+row .. YOFF+row].
***************/

#include "Display.hpp"

#include <cstring>
#include <cstdio>

#include "fonts/digi_slim_3x6.h"
#include "fonts/digi_one_5x6.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace CTAG::DRIVERS;

static const char *TAG = "Display";

// ===================== bring-up knobs (discovery) =====================
// The 284x76 bar on an ST7789 (240x320 controller, 16 bpp RGB565):
// START with the documented guess below and use `testpat 4` (1-px frame at
// the window edges + ruler marks every 20 px + hue bars) to verify. If the
// image is SHIFTED: raise/lower XOFF/YOFF (controllers center or edge-align
// masked windows; typical values 0-4 for Y, 0-36 for the 320-axis). If the
// image is ROTATED/mirrored: try the four MADCTL candidates. Document the
// found values back here + in rack-uimcu/README.md (bring-up step 1).
#define ST7789_XOFF   2        // CASET start offset (0..36, 320-wide axis)
#define ST7789_YOFF   0        // RASET start offset (0..4, 240 axis)
#define ST7789_MADCTL 0x60     // candidates: 0x00 / 0x60 / 0xA0 / 0xC0
#define ST7789_INVERT 1        // IPS panel: inversion ON (0x21). Set 0 if
                               // the hue bars render as their negatives.

// Wiring (README table): SCK=0 MOSI=1 DC=2 (user call 2026-11-08: display
// onto the 0-4 header — mux COM moved to GPIO3, the last free ADC1 pin, and
// the mux address lines took GPIO10/20/21); CS->GND, RST->3V3 RC, BL->3V3.
#define ST7789_PIN_SCK  0
#define ST7789_PIN_MOSI 1
#define ST7789_PIN_DC   2

// Bring-up clock: modest until the wiring is proven (long jumpers corrupt
// fast SPI — the seed OLED lesson). 8 MHz = a full-screen flush in ~22 ms;
// raise to 26-40 MHz (GPIO-matrix SPI) once mounted.
#define ST7789_SPI_CLOCK_HZ (8 * 1000 * 1000)

// ===================== RGB565 palette (conversion arithmetic) ============
// RGB565 = (R8>>3)<<11 | (G8>>2)<<5 | (B8>>3):
//   #e4e4e4: R228->28, G228->57, B228->28  -> 28<<11|57<<5|28 = 0xE73C
//   #e6295e: R230->28, G 41->10, B 94->11  -> 28<<11|10<<5|11 = 0xE14B
//   #17a3bd: R 23-> 2, G163->40, B189->23  ->  2<<11|40<<5|23 = 0x1517
//   #f2d21f: R242->30, G210->52, B 31-> 3  -> 30<<11|52<<5| 3 = 0xF683
// (kWhite/kRed/kTeal/kYellow constants live in Display.hpp, verified here.)

// ===================== state =====================
uint16_t Display::fb[Display::W * Display::H];
bool Display::dirty[Display::H];
int Display::s_testpat = -1;

static spi_device_handle_t s_spi = nullptr;

bool Display::sendCommand(uint8_t cmd, const uint8_t *data, size_t len) {
    // One polling transaction per phase (DC toggles between them; CS is
    // hard-wired low so no chip-select bookkeeping).
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    if (spi_device_polling_transmit(s_spi, &t) != ESP_OK) return false;
    if (len) {
        gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
        t.length = len * 8;
        t.tx_buffer = data;
        if (spi_device_polling_transmit(s_spi, &t) != ESP_OK) return false;
    }
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
    return true;
}

void Display::setWindow(int x0, int y0, int x1, int y1) {
    // CASET/RASET in the CONTROLLER's coordinate space (offsets added).
    const uint8_t caset[4] = {
        (uint8_t)((ST7789_XOFF + x0) >> 8), (uint8_t)(ST7789_XOFF + x0),
        (uint8_t)((ST7789_XOFF + x1) >> 8), (uint8_t)(ST7789_XOFF + x1),
    };
    const uint8_t raset[4] = {
        (uint8_t)((ST7789_YOFF + y0) >> 8), (uint8_t)(ST7789_YOFF + y0),
        (uint8_t)((ST7789_YOFF + y1) >> 8), (uint8_t)(ST7789_YOFF + y1),
    };
    sendCommand(0x2A, caset, 4);
    sendCommand(0x2B, raset, 4);
    sendCommand(0x2C);                      // RAMWR: DC rises for the data
}

void Display::writePixels(const uint16_t *px, size_t n) {
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
    spi_transaction_t t = {};
    t.length = n * 16;
    t.tx_buffer = px;
    esp_err_t err = spi_device_polling_transmit(s_spi, &t);
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi pixel tx: %s", esp_err_to_name(err));
    }
}

void Display::Init() {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = ST7789_PIN_MOSI;
    bus.sclk_io_num = ST7789_PIN_SCK;
    bus.miso_io_num = -1;
    bus.quadwp_io_num = -1;
    bus.max_transfer_sz = FRAMEBUFFER_SIZE;   // one full-screen window fits
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = ST7789_SPI_CLOCK_HZ;
    dev.mode = 0;
    dev.spics_io_num = -1;                    // CS tied GND
    dev.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << ST7789_PIN_DC;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);

    // ST7789P3 init (IPS bar). RST is hardware (RC) — no SWRESET pin dance;
    // SWRESET command still sent for a warm re-init path.
    sendCommand(0x01);                        // SWRESET
    vTaskDelay(pdMS_TO_TICKS(150));
    sendCommand(0x11);                        // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));
    const uint8_t colmod = 0x55;              // 16 bpp RGB565
    sendCommand(0x3A, &colmod, 1);
    const uint8_t madctl = ST7789_MADCTL;
    sendCommand(0x36, &madctl, 1);
#if ST7789_INVERT
    sendCommand(0x21);                        // INVON (IPS)
#else
    sendCommand(0x20);                        // INVOFF (TN)
#endif
    sendCommand(0x13);                        // NORON
    // Wait for the first flush to light the panel with real content.
    std::memset(fb, 0, sizeof(fb));
    for (int y = 0; y < H; ++y) dirty[y] = true;
    Flush();
    sendCommand(0x29);                        // DISPON
    ESP_LOGI(TAG, "st7789p3 %dx76 window (%u,%u) madctl 0x%02x ready",
             W, (unsigned)ST7789_XOFF, (unsigned)ST7789_YOFF,
             (unsigned)ST7789_MADCTL);
}

void Display::markDirty(int y) {
    if (y >= 0 && y < H) dirty[y] = true;
}

void Display::Clear(uint16_t color) {
    for (int y = 0; y < H; ++y) {
        uint16_t *row = &fb[y * W];
        for (int x = 0; x < W; ++x) row[x] = color;
        dirty[y] = true;
    }
}

void Display::DrawPixel(int x, int y, uint16_t c) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    fb[y * W + x] = c;
    markDirty(y);
}

void Display::DrawHLine(int x, int y, int w, uint16_t c) {
    if (y < 0 || y >= H) return;
    int x0 = x < 0 ? 0 : x;
    int x1 = x + w;
    if (x1 > W) x1 = W;
    for (int i = x0; i < x1; ++i) fb[y * W + i] = c;
    markDirty(y);
}

void Display::FillRect(int x, int y, int w, int h, uint16_t c) {
    for (int j = 0; j < h; ++j) DrawHLine(x, y + j, w, c);
}

// ---- fonts: column-major, LSB = top pixel, 1 byte per row (6 rows) ----
void Display::DrawString(int x, int y, const char *s, Font f, uint16_t c) {
    if (!s) return;
    while (*s) {
        const uint8_t ch = (uint8_t)*s++;
        if (ch >= 127) continue;              // the fonts cover 0-126
        const uint8_t *glyph = (f == FONT_LABEL) ? font_digi_slim[ch]
                                                 : font_digi_one[ch];
        const uint8_t gh = (f == FONT_LABEL) ? FONT_DIGI_SLIM_H
                                             : FONT_DIGI_ONE_H;
        const uint8_t adv = (f == FONT_LABEL) ? font_digi_slim_advances[ch]
                                              : font_digi_one_advances[ch];
        for (int col = 0; col < 6; ++col) {
            const uint8_t bits = glyph[col];
            if (!bits) continue;
            const int px = x + col;
            if (px < 0 || px >= W) continue;
            for (int row = 0; row < gh; ++row) {
                if ((bits >> row) & 1) {
                    const int py = y + row;
                    if (py >= 0 && py < H) {
                        fb[py * W + px] = c;
                        markDirty(py);
                    }
                }
            }
        }
        x += adv;
    }
}

int Display::StringWidth(const char *s, Font f) {
    if (!s) return 0;
    int w = 0;
    while (*s) {
        const uint8_t ch = (uint8_t)*s++;
        if (ch >= 127) continue;
        w += (f == FONT_LABEL) ? font_digi_slim_advances[ch]
                               : font_digi_one_advances[ch];
    }
    return w;
}

void Display::Flush() {
    // Coalesce consecutive dirty rows into single window transfers (a
    // page change redraws everything = one 43 KB burst; a delta touches
    // a few rows). CAPPED at kMaxFlushPixels per transaction: the C3's
    // GPSPI DMA limit is 2^18 bits (SPI_LL_DMA_MAX_BIT_LEN, spi_ll.h:42)
    // = 32768 B — a whole-page window is 284*76 = 21584 px = 345344 bits
    // and check_trans_valid rejects it (bench-caught 2026-11-08: every
    // full-page redraw silently failed, "txdata transfer > hardware max
    // supported len"). 16000 px = 256000 bits, safely under the cap; W
    // = 284 does not divide it evenly, so chunks end on row boundaries.
    constexpr size_t kMaxFlushPixels = 16000;
    for (int y = 0; y < H;) {
        if (!dirty[y]) { ++y; continue; }
        int y1 = y;
        while (y1 + 1 < H && dirty[y1 + 1]) ++y1;
        int run = y;
        while (run <= y1) {
            int chunkEnd = run + (int)(kMaxFlushPixels / W) - 1;
            if (chunkEnd > y1) chunkEnd = y1;
            setWindow(0, run, W - 1, chunkEnd);
            writePixels(&fb[run * W], (size_t)W * (chunkEnd - run + 1));
            run = chunkEnd + 1;
        }
        for (int k = y; k <= y1; ++k) dirty[k] = false;
        y = y1 + 1;
    }
}

// ===================== test patterns =====================
bool Display::TestPatternActive() { return s_testpat >= 0; }

void Display::TestPattern(int mode) {
    s_testpat = mode;
    switch (mode) {
        case 1:  // full white
            Clear(kWhite);
            break;
        case 2:  // full black
            Clear(kBlack);
            break;
        case 3: {  // the 4 hue bars (horizontal quarters)
            Clear(kBlack);
            const int q = H / 4;
            FillRect(0, 0, W, q, kWhite);
            FillRect(0, q, W, q, kRed);
            FillRect(0, 2 * q, W, q, kTeal);
            FillRect(0, 3 * q, W, H - 3 * q, kYellow);
            break;
        }
        case 4: {  // border + ruler marks every 20 px (offset discovery)
            Clear(kBlack);
            // 1-px frame exactly at the CONFIGURED window edges: if the
            // offsets are wrong the frame is clipped on one side / wraps.
            DrawHLine(0, 0, W, kWhite);
            DrawHLine(0, H - 1, W, kWhite);
            FillRect(0, 0, 1, H, kWhite);
            FillRect(W - 1, 0, 1, H, kWhite);
            // Ruler ticks every 20 px along the top (long) edge, taller
            // every 100 px.
            for (int x = 0; x < W; x += 20) {
                FillRect(x, 0, 1, (x % 100 == 0) ? 9 : 5, kYellow);
            }
            // Vertical ticks every 20 px on the left edge.
            for (int y = 0; y < H; y += 20) {
                FillRect(0, y, 5, 1, kYellow);
            }
            // Corner labels (both digi fonts visible for the font check).
            DrawString(2, 12, "RACK TESTPAT 4", FONT_LABEL, kWhite);
            DrawString(2, 22, "284x76 0123456789", FONT_VALUE, kTeal);
            DrawString(W - 58, 22, "RGB565", FONT_VALUE, kRed);
            break;
        }
        default:  // 0 = back to the UI
            s_testpat = -1;
            break;
    }
    if (s_testpat >= 0) Flush();
}
