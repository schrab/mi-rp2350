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

// C3 panel display: framebuffer primitives are the A1S Display.cpp code
// (page-major framebuffer, dirty-page Flush, per-glyph font advances); the
// backend is a direct SSD1309 SPI driver replacing the A1S I²C ssd1306
// component.

#include <string>
#include <cstring>
#include <cstdio>

#include "Display.hpp"
#include "font5x7.h"
#include "font8x8_basic.h"
#include "fonts/digi_slim_3x6.h"
#include "fonts/digi_one_5x6.h"
#include "fonts/norns_6x7.h"
#include "fonts/norns_ext_6x7.h"
#include "version.hpp"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace CTAG::DRIVERS;

// Panel wiring (SDD §2.1). RST is tied to 3V3 via RC on the carrier.
// Canonical SDD §2.1 pins. (A crossed SCLK/SDA mapping was "proven" by the
// bench scan while the panel was misconfigured (I²C jumpers) and finally
// dying (reversed-power damage) — it lit all-white at power-on, which is
// not firmware behavior. With a healthy panel, wire canonically.)
#define OLED_SCK_GPIO 6
#define OLED_MOSI_GPIO 7
#define OLED_CS_GPIO 2
#define OLED_DC_GPIO 8 // remapped from 3: GPIO8 was a strap pin hosting encoder E1

// 10 MHz start (SDD §2.1: 10–20 MHz); raise to 20 after the wiring is proven.
// 1 MHz bring-up clock: long jumper wires corrupt 10 MHz SPI — raise
// once the display is proven (5, then 10 MHz).
#define OLED_SPI_CLOCK_HZ (1 * 1000 * 1000)

// Some 128x64 SSD1309 modules sit on a 132-column controller with a 2-column
// offset. If the image is shifted 2px left/right at bring-up, set this to 2.
#define OLED_COLUMN_OFFSET 0

// COM scan + segment remap match the A1S panel orientation (0xC8 scan).
static const char *TAG = "Display";

static spi_device_handle_t spiHandle = nullptr;

uint8_t Display::fb[1024];
bool Display::dirtyPages[8] {false};
int Display::userContrast = 0xFF;

// CS is driven as a plain GPIO (software CS) and DC pin is runtime — the
// panel's CS/DC pin assignment was the last bring-up unknown (the bit-bang
// scan is symmetric under a CS/DC swap, hardware CS is not). Defaults =
// SDD §2.1; /debug/displayPins retargets them on the bench.
// Canonical CS/DC (SDD §2.1); see the SCLK/SDA note above — the crossed
// assignments were artifacts of the failing panel. /debug/displayPins can
// still retarget them at runtime if a wiring variant ever needs it.
static int oledCsGpio = OLED_CS_GPIO;
static int oledDcGpio = OLED_DC_GPIO;

static void csLow() { gpio_set_level((gpio_num_t)oledCsGpio, 0); }
static void csHigh() { gpio_set_level((gpio_num_t)oledCsGpio, 1); }
static void dcCmd() { gpio_set_level((gpio_num_t)oledDcGpio, 0); }
static void dcData() { gpio_set_level((gpio_num_t)oledDcGpio, 1); }

bool Display::sendCommand(uint8_t cmd) {
    dcCmd();
    csLow();
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    bool ok = spi_device_polling_transmit(spiHandle, &t) == ESP_OK;
    csHigh();
    return ok;
}

bool Display::sendData(const uint8_t *data, size_t len) {
    dcData();
    csLow();
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    esp_err_t err = spi_device_polling_transmit(spiHandle, &t);
    csHigh();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi data tx: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

void Display::Init() {
    // Bus setup runs once; Reinit() (bench diagnostics) re-sends the panel
    // init commands only — spi_bus_initialize on an already-claimed bus
    // aborts the device.
    static bool busReady = false;
    if (!busReady) {
        spi_bus_config_t bus = {};
        bus.mosi_io_num = OLED_MOSI_GPIO;
        bus.sclk_io_num = OLED_SCK_GPIO;
        bus.miso_io_num = -1;
        bus.quadwp_io_num = -1;
        bus.max_transfer_sz = 128;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = OLED_SPI_CLOCK_HZ;
        dev.mode = 0;
        dev.spics_io_num = -1; // software CS (see above)
        dev.queue_size = 1;
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spiHandle));

        gpio_config_t io = {};
        io.pin_bit_mask = (1ULL << OLED_DC_GPIO) | (1ULL << OLED_CS_GPIO);
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        busReady = true;
        }

    // SSD1309 init, page addressing mode (matches the dirty-page Flush).
    sendCommand(0xAE); // display off
    sendCommand(0xD5); sendCommand(0x80); // clock divide
    sendCommand(0xA8); sendCommand(0x3F); // multiplex 1/64
    sendCommand(0xD3); sendCommand(0x00); // display offset
    sendCommand(0x40);                    // start line 0
    sendCommand(0x8D); sendCommand(0x14); // charge pump on
    sendCommand(0x20); sendCommand(0x02); // page addressing mode
    sendCommand(0xA1);                    // segment remap
    sendCommand(0xC8);                    // COM scan direction
    sendCommand(0xDA); sendCommand(0x12); // COM pins
    sendCommand(0x81); sendCommand((uint8_t)userContrast);
    sendCommand(0xD9); sendCommand(0xF1); // precharge
    sendCommand(0xDB); sendCommand(0x40); // VCOMH
    sendCommand(0xA4);                    // display from RAM
    sendCommand(0xA6);                    // normal (not inverted)
    sendCommand(0xAF);                    // display on


    std::memset(fb, 0, sizeof(fb));
    for (int i = 0; i < 8; i++) dirtyPages[i] = true;
    Flush();
}

void Display::RawCommand(uint8_t cmd) {
    sendCommand(cmd);
}

// ---- bench pin-permutation scan (bit-banged; bring-up diagnostic only) ----
namespace {
    void bbOut(int pin) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << pin;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
    }
    void bbByte(int sck, int mosi, uint8_t b) {
        for (int i = 7; i >= 0; i--) {
            gpio_set_level((gpio_num_t)mosi, (b >> i) & 1);
            gpio_set_level((gpio_num_t)sck, 1);
            gpio_set_level((gpio_num_t)sck, 0);
        }
    }
    void bbCmd(int sck, int mosi, int dc, int cs, uint8_t c) {
        gpio_set_level((gpio_num_t)dc, 0);
        gpio_set_level((gpio_num_t)cs, 0);
        bbByte(sck, mosi, c);
        gpio_set_level((gpio_num_t)cs, 1);
    }
    void bbDataN(int sck, int mosi, int dc, int cs, uint8_t v, int n) {
        gpio_set_level((gpio_num_t)dc, 1);
        gpio_set_level((gpio_num_t)cs, 0);
        for (int i = 0; i < n; i++) bbByte(sck, mosi, v);
        gpio_set_level((gpio_num_t)cs, 1);
    }
    void bbPanelInit(int sck, int mosi, int cs, int dc) {
        for (int pin : {sck, mosi, cs, dc}) bbOut(pin);
        gpio_set_level((gpio_num_t)cs, 1);
        gpio_set_level((gpio_num_t)dc, 1);
        gpio_set_level((gpio_num_t)sck, 0);
        gpio_set_level((gpio_num_t)mosi, 0);
        static const uint8_t initSeq[] = {
            0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
            0x8D, 0x14,             // charge pump on — without this: dark
            0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
            0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40,
            0xA4, 0xA6, 0xAF,
        };
        for (uint8_t c : initSeq) bbCmd(sck, mosi, dc, cs, c);
        // fill every page white, then force all-on (0xA5)
        for (int p = 0; p < 8; p++) {
            bbCmd(sck, mosi, dc, cs, (uint8_t)(0xB0 | p));
            bbCmd(sck, mosi, dc, cs, 0x00);
            bbCmd(sck, mosi, dc, cs, 0x10);
            bbDataN(sck, mosi, dc, cs, 0xFF, 128);
        }
        bbCmd(sck, mosi, dc, cs, 0xA5);
    }
}

void Display::BenchPins(int cs, int dc) {
    oledCsGpio = cs;
    oledDcGpio = dc;
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << cs) | (1ULL << dc);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    Init(); // re-sends the panel init + flushes with the new assignment
}

void Display::BenchScan(int mode) {
    // 0: SCK=6 MOSI=7 CS=2 DC=8 (as documented)
    // 1: SDA/SCLK swapped, 2: CS/DC swapped, 3: both swapped
    int sck = 6, mosi = 7, cs = 2, dc = 8;
    if (mode == 1) { sck = 7; mosi = 6; }
    else if (mode == 2) { cs = 8; dc = 2; }
    else if (mode == 3) { sck = 7; mosi = 6; cs = 8; dc = 2; }
    bbPanelInit(sck, mosi, cs, dc);
}

void Display::Reinit() {
    Init();
}

void Display::Sleep() {
    sendCommand(0xAE);
}

void Display::Wake() {
    sendCommand(0xAF);
    sendCommand(0x81);
    sendCommand((uint8_t)userContrast);
}

void Display::Contrast(int val) {
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    userContrast = val;
    sendCommand(0x81);
    sendCommand((uint8_t)userContrast);
}

void Display::Clear() {
    std::memset(fb, 0, sizeof(fb));
    for (int i = 0; i < 8; i++) dirtyPages[i] = true;
}

void Display::ShowFWVersion() {
    Clear();
    DrawString(0, 10, "TBD fw:", FONT_5X7);
    DrawString(0, 20, TBD_FW_VERSION.c_str(), FONT_5X7);
    DrawString(0, 34, "TBD hw:", FONT_5X7);
    DrawString(0, 44, TBD_HW_VERSION.c_str(), FONT_5X7);
    Flush();
}

void Display::MarkDirty(int page) {
    if (page >= 0 && page < 8) dirtyPages[page] = true;
}

void Display::Flush() {
    for (int p = 0; p < 8; p++) {
        if (!dirtyPages[p]) continue;
        sendCommand(0xB0 | p);                                        // page address
        sendCommand(0x00 | ((OLED_COLUMN_OFFSET) & 0x0F));            // column low
        sendCommand(0x10 | (((OLED_COLUMN_OFFSET) >> 4) & 0x0F));     // column high
        sendData(&fb[p * 128], 128);
        dirtyPages[p] = false;
    }
}

void Display::CopyFramebuffer(uint8_t *dst) {
    if (dst == nullptr) return;
    memcpy(dst, fb, FRAMEBUFFER_SIZE);
}

void Display::DrawPixel(int x, int y, bool on) {
    if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
    int page = y >> 3;
    int bit = y & 7;
    int idx = page * 128 + x;
    if (on)
        fb[idx] |= (1 << bit);
    else
        fb[idx] &= ~(1 << bit);
    MarkDirty(page);
}

void Display::DrawHLine(int x, int y, int w, bool on) {
    for (int i = 0; i < w; i++) DrawPixel(x + i, y, on);
}

void Display::DrawVLine(int x, int y, int h, bool on) {
    for (int i = 0; i < h; i++) DrawPixel(x, y + i, on);
}

void Display::DrawRect(int x, int y, int w, int h, bool fill, bool on) {
    if (fill) {
        for (int row = 0; row < h; row++)
            DrawHLine(x, y + row, w, on);
    } else {
        DrawHLine(x, y, w, on);
        DrawHLine(x, y + h - 1, w, on);
        DrawVLine(x, y, h, on);
        DrawVLine(x + w - 1, y, h, on);
    }
}

void Display::InvertRect(int x, int y, int w, int h) {
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int px = x + col, py = y + row;
            if (px < 0 || px >= 128 || py < 0 || py >= 64) continue;
            int page = py >> 3;
            int bit = py & 7;
            int idx = page * 128 + px;
            fb[idx] ^= (1 << bit);
            MarkDirty(page);
        }
    }
}

static void drawGlyph(int &x, int y, const uint8_t *glyph, int gw, int gh, int advance,
                void (*pixel)(int, int, bool)) {
    for (int col = 0; col < gw; col++) {
        uint8_t byte = glyph[col];
        for (int row = 0; row < gh; row++) {
            pixel(x + col, y + row, (byte >> row) & 1);
        }
    }
    x += advance;
}

// Decode a single UTF-8 codepoint from str, advance pointer past it
static uint16_t utf8_decode(const char *&s) {
    uint8_t b0 = (uint8_t)*s;
    if (b0 < 0x80) {
        s++;
        return b0;
    }
    if ((b0 & 0xE0) == 0xC0) {
        // 2-byte sequence
        if ((s[1] & 0xC0) != 0x80) { s++; return b0; }
        uint16_t cp = ((uint16_t)(b0 & 0x1F) << 6) | (s[1] & 0x3F);
        s += 2;
        return cp;
    }
    if ((b0 & 0xF0) == 0xE0) {
        // 3-byte sequence
        if ((s[1] & 0xC0) != 0x80 || (s[2] & 0xC0) != 0x80) { s++; return b0; }
        uint16_t cp = ((uint16_t)(b0 & 0x0F) << 12) | ((uint16_t)(s[1] & 0x3F) << 6) | (s[2] & 0x3F);
        s += 3;
        return cp;
    }
    // 4-byte or invalid lead byte — skip 1 byte
    s++;
    return b0;
}

void Display::DrawString(int x, int y, const char *str, Font font) {
    if (!str) return;
    while (*str) {
        const char *next = str;
        uint16_t c = utf8_decode(next);
        str = next;
        switch (font) {
        case FONT_8X8:
            if (c < 32 || c > 127) continue;
            drawGlyph(x, y, font8x8_basic_tr[c - 32], 8, 8, 8, DrawPixel);
            break;
        case FONT_5X7:
            if (c > 127) continue;
            drawGlyph(x, y, font5x7[c], 5, 7, 6, DrawPixel);
            break;
        case FONT_DIGI_SLIM_3X6:
            if (c >= FONT_DIGI_SLIM_N_CHARS) continue;
            drawGlyph(x, y, font_digi_slim[c], FONT_DIGI_SLIM_W, FONT_DIGI_SLIM_H,
                      font_digi_slim_advances[c], DrawPixel);
            break;
        case FONT_DIGI_ONE_5X6:
            if (c >= FONT_DIGI_ONE_N_CHARS) continue;
            drawGlyph(x, y, font_digi_one[c], FONT_DIGI_ONE_W, FONT_DIGI_ONE_H,
                      font_digi_one_advances[c], DrawPixel);
            break;
        case FONT_NORNS_6X7:
            if (c < FONT_NORNS_N_CHARS) {
                drawGlyph(x, y, font_norns[c], FONT_NORNS_W, FONT_NORNS_H,
                          font_norns_advances[c], DrawPixel);
            } else {
                int idx = FontNornsExtLookup(c);
                if (idx < FONT_NORNS_EXT_N_CHARS) {
                    drawGlyph(x, y, font_norns_ext_data[idx], FONT_NORNS_EXT_W,
                              FONT_NORNS_EXT_H, font_norns_ext_advances[idx], DrawPixel);
                }
            }
            break;
        }
    }
}

int Display::FontAdvance(Font f) {
    switch (f) {
    case FONT_8X8: return 8;
    case FONT_5X7: return 6;
    case FONT_DIGI_SLIM_3X6: return FONT_DIGI_SLIM_ADVANCE;
    case FONT_DIGI_ONE_5X6: return FONT_DIGI_ONE_ADVANCE;
    case FONT_NORNS_6X7: return FONT_NORNS_ADVANCE;
    }
    return 6;
}

int Display::FontAdvance(Font f, unsigned char c) {
    switch (f) {
    case FONT_DIGI_SLIM_3X6:
        if (c < FONT_DIGI_SLIM_N_CHARS) return font_digi_slim_advances[c];
        return FONT_DIGI_SLIM_ADVANCE;
    case FONT_DIGI_ONE_5X6:
        if (c < FONT_DIGI_ONE_N_CHARS) return font_digi_one_advances[c];
        return FONT_DIGI_ONE_ADVANCE;
    case FONT_NORNS_6X7:
        if (c < FONT_NORNS_N_CHARS) return font_norns_advances[c];
        if (c >= 0x80) {
            int idx = FontNornsExtLookup(c);
            if (idx < FONT_NORNS_EXT_N_CHARS)
                return font_norns_ext_advances[idx];
        }
        return FONT_NORNS_ADVANCE;
    default:
        return FontAdvance(f);
    }
}

void Display::DrawStringRight(int x, int y, const char *str, Font font) {
    if (!str) return;
    int len = strlen(str);
    int strWidth = len * FontAdvance(font);
    DrawString(x - strWidth, y, str, font);
}

void Display::DrawVUMeter(int x, int y, int w, int h, float level) {
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    int fillW = (int)(level * w);
    if (fillW > w) fillW = w;
    DrawRect(x, y, w, h, false, true);
    if (fillW > 0) {
        DrawRect(x + 1, y + 1, fillW, h - 2, true, true);
    }
}

void Display::DrawVUMeterV(int x, int y, int w, int h, float level) {
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    DrawRect(x, y, w, h, false, true);
    int fillH = (int)(level * (h - 2));
    if (fillH > h - 2) fillH = h - 2;
    if (fillH > 0) {
        // fill upwards from the bottom edge
        DrawRect(x + 1, y + h - 1 - fillH, w - 2, fillH, true, true);
    }
}

void Display::DrawScrollbar(int x, int y, int h, int totalItems, int cursorPos) {
    if (totalItems <= 1) return;
    int visibleItems = h / LINE_H;
    if (visibleItems >= totalItems) return;
    int thumbH = h * visibleItems / totalItems;
    if (thumbH < 4) thumbH = 4;
    int thumbY = y + (h - thumbH) * cursorPos / (totalItems - visibleItems);
    DrawVLine(x, y, h);                           // 1px track
    DrawRect(x - 1, thumbY, 3, thumbH, true, true); // 3px thumb overlaps right edge
}
