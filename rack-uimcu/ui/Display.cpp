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
#define ST7789_XOFF   2        // CASET start offset — RESTORED 2026-11-09:
                               // the v1 bench (XOFF=2/YOFF=0) is the only
                               // configuration that ever showed data (the
                               // "static noise" = the UI's own SPI writes
                               // landing, interleaved with the bit-bang).
                               // The TFT_eSPI fork's 18/82 belongs to its
                               // rotation convention, not this driver's.
#define ST7789_YOFF   0        // RASET start offset — same reasoning.
#define ST7789_MADCTL 0x60     // candidates: 0x00 / 0x60 / 0xA0 / 0xC0
#define ST7789_INVERT 1        // IPS panel: inversion ON (0x21). Set 0 if
                               // the hue bars render as their negatives.

// Wiring FINAL (2026-11-09, user-accepted 3-wire move): the display on the
// non-RTC pads (the SPI hardware drives them properly), CS/RST on real
// GPIOs. The freed RTC pads 0/1 go to the encoder; the mux addr takes
// 6/8/9/2. Geometry: the bborient gradient proved the visible window =
// native cols 82-157 (screen Y, MIRRORED: y=0 -> col 157) x native rows
// 18-301 (screen X, MIRRORED: x=0 -> row 301), native MADCTL 0x00 — the
// mirrors live in Flush's addressing, not in MADCTL.
#define ST7789_PIN_SCK  10
#define ST7789_PIN_MOSI 20
#define ST7789_PIN_DC   21
// CS is TIED GND (as wired): GPIO6 went to the mux S3 address line. The
// bit-counter desync that motivated CS-on-GPIO is handled by the firmware
// RST pulse (GPIO7) — a hardware reset clears the serial framing at every
// boot, after the bus pads are already driven.
#define ST7789_PIN_RST  7     // real RST: pulsed by firmware in Init()
#define ST7789_COLSTART 82    // native column range of the visible window
#define ST7789_ROWCLOSEST 301 // native row range: 18..301 (284 rows)

// Bring-up clock: 8 MHz (the panel-ignores-everything mystery was the
// MISSING INIT REGISTERS, not signal integrity — the verbatim TFT_eSPI
// init sequence fixed it; raise to 26-40 MHz once mounted).
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

// The display driver uses the STANDARD SPI peripheral — on NON-RTC pads
// (SCK=10, MOSI=20, DC=21): the C3's SPI hardware output does not reach
// the RTC-domain pads GPIO0/1/2 (bench-proven 2026-11-09: the same-pads
// direct-GPIO test delivered data while every SPI transfer delivered
// nothing). CS=6 is pulsed by the SPI hardware per transaction; RST=7 is
// pulsed by Init().

static spi_device_handle_t s_spi = nullptr;

bool Display::sendCommand(uint8_t cmd, const uint8_t *data, size_t len) {
    // One polling transaction per phase (DC toggles between them; CS is
    // pulsed by the SPI hardware per transaction).
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
    // ST7789 wants the colour HIGH byte first on the wire; the framebuffer
    // is little-endian uint16 — swap through a one-row staging buffer
    // (Claude-review bug 1: colours were byte-swapped on the wire).
    static uint16_t line[W];
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
    while (n) {
        const size_t chunk = n < W ? n : W;
        for (size_t i = 0; i < chunk; ++i) {
            const uint16_t v = px[i];
            line[i] = (uint16_t)((v >> 8) | (v << 8));
        }
        spi_transaction_t t = {};
        t.length = chunk * 16;
        t.tx_buffer = line;
        esp_err_t err = spi_device_polling_transmit(s_spi, &t);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi pixel tx: %s", esp_err_to_name(err));
        }
        px += chunk;
        n -= chunk;
    }
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
}

// ---- bbtest: raw-GPIO panel sanity test (2026-11-09 bench) ----
// Bit-bangs the ST7789 init + a white top band on the SAME three pads,
// bypassing the SPI peripheral entirely (no matrix routing, no DMA, no
// clock source). Panel responds -> pads/wires/panel are good and the SPI
// setup is the fault. Panel ignores it -> no SPI config can help; the
// fault is the wires/panel. ~100-200 kHz effective via gpio_set_level.
static void bbBit(int v) {
    gpio_set_level((gpio_num_t)ST7789_PIN_SCK, 0);
    gpio_set_level((gpio_num_t)ST7789_PIN_MOSI, v);
    esp_rom_delay_us(2);                  // 10x edge-margin insurance
    gpio_set_level((gpio_num_t)ST7789_PIN_SCK, 1);
    esp_rom_delay_us(2);
}
// 4-WIRE protocol (2026-11-09: 3-wire 9-bit frames ruled out with a live
// SDA — white through the full flood): D/C as a real GPIO per byte.
static void bbCmd(uint8_t cmd) {
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
    for (int i = 7; i >= 0; --i) bbBit((cmd >> i) & 1);
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
}
static void bbData(uint8_t d) {
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
    for (int i = 7; i >= 0; --i) bbBit((d >> i) & 1);
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
}
void Display::BitBangTest() {
    // pads as plain outputs (the SPI bus also claims 0/1 — gpio matrix
    // re-route to plain GPIO overrides it while this test runs).
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << ST7789_PIN_SCK) | (1ULL << ST7789_PIN_MOSI) |
                      (1ULL << ST7789_PIN_DC);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)ST7789_PIN_SCK, 0);
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);

    // LIVE-PROBE MODE v5 — RESET BLINK: the minimal listening test. A panel
    // that hears ANY command blinks visibly on every SWRESET (sleep = dark,
    // SLPOUT+DISPON = bright). Independent of offsets, init registers,
    // colour order and inversion. 20 blinks ≈ 17 s.
    for (int rep = 0; rep < 20; ++rep) {
        bbCmd(0x01);                          // SWRESET -> sleep = dark
        vTaskDelay(pdMS_TO_TICKS(400));
        bbCmd(0x11);                          // SLPOUT
        vTaskDelay(pdMS_TO_TICKS(120));
        bbCmd(0x29);                          // DISPON -> bright
        vTaskDelay(pdMS_TO_TICKS(400));
    }
    ESP_LOGI(TAG, "bbtest done: 20 reset blinks on %d/%d/%d",
             ST7789_PIN_SCK, ST7789_PIN_MOSI, ST7789_PIN_DC);
}

// ---- spiflood: the bisect test — full-GRAM color flood through the SPI
// path with the MINIMAL init (the only init that ever landed). If colors
// cycle: SPI + minimal = good, and the register block gets bisected. If
// white: the SPI path itself broke after 8911819 — bisect its changes.
void Display::SpiFlood() {
    s_testpat = 5;                        // hold the UI off
    // hardware reset, then MINIMAL init via the SPI path
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    sendCommand(0x11);                    // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));
    sendCommand(0x13);                    // NORON
    sendCommand(0x21);                    // INVON (v1 parity)
    sendCommand(0x29);                    // DISPON
    vTaskDelay(pdMS_TO_TICKS(60));
    static const uint16_t kFlood[4] = {0xF800, 0x07E0, 0x001F, 0xFFFF};
    for (int rep = 0; rep < 6; ++rep) {
        for (int c = 0; c < 4; ++c) {
            const uint16_t col = kFlood[c];
            const uint8_t hi = col >> 8, lo = col & 0xFF;
            const uint8_t caset[4] = {0, 0, 0, 239};   // full native columns
            const uint8_t raset[4] = {0, 0, 1, 63};    // full native rows
            sendCommand(0x2A, caset, 4);
            sendCommand(0x2B, raset, 4);
            sendCommand(0x2C);
            // stream the color directly (no fb, no window math)
            gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
            for (int px = 0; px < 240 * 320; px += W) {
                static uint16_t line[W];
                for (int i = 0; i < W; ++i) line[i] = col;
                spi_transaction_t t = {};
                t.length = W * 16;
                t.tx_buffer = line;
                spi_device_polling_transmit(s_spi, &t);
            }
            gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
            vTaskDelay(pdMS_TO_TICKS(1500));   // hold each color
        }
    }
    s_testpat = -1;
    ESP_LOGI(TAG, "spiflood done");
}

// ---- osweep: MADCTL × window-mirror sweep through the PROVEN SPI path ----
// 16 combos: MADCTL 0x00..0xE0 (step 0x20) × {direct, mirrored} window.
// Each combo draws a GIANT 7-SEGMENT DIGIT (combo 0 = "0", 1 = "1", ...,
// 10-15 = A b C d E F) filling the bar — the user reads the digit aloud;
// the readable, fully-visible digit IS the combo index. No counting.
void Display::OrientationSweep() {
    s_testpat = 5;                        // hold the UI off while probing
    // 7-seg segment rects for 284x76: a top, b tr, c br, d bottom,
    // e bl, f tl, g middle.
    enum Seg { A = 1 << 0, B = 1 << 1, C = 1 << 2, D = 1 << 3,
               E = 1 << 4, F = 1 << 5, G = 1 << 6 };
    static const uint8_t kDigits[16] = {
        A | B | C | D | E | F,            // 0
        B | C,                            // 1
        A | B | G | E | D,                // 2
        A | B | G | C | D,                // 3
        F | G | B | C,                    // 4
        A | F | G | C | D,                // 5
        A | F | G | E | C | D,            // 6
        A | B | C,                        // 7
        A | B | C | D | E | F | G,        // 8
        A | B | C | D | F | G,            // 9
        A | B | C | E | F | G,            // A
        C | D | E | F | G,                // b
        A | D | E | F,                    // C
        B | C | D | E | G,                // d
        A | D | E | F | G,                // E
        A | E | F | G,                    // F
    };
    struct SegRect { int x, y, w, h; };
    static const SegRect kSeg[7] = {
        {42, 4, 200, 10},    // a
        {244, 6, 12, 28},    // b
        {244, 42, 12, 28},   // c
        {42, 62, 200, 10},   // d
        {28, 42, 12, 28},    // e
        {28, 6, 12, 28},     // f
        {42, 33, 200, 10},   // g
    };

    for (int combo = 0; combo < 16; ++combo) {
        const uint8_t madctl = (uint8_t)(combo / 2) * 0x20;
        const bool mirrored = (combo % 2) != 0;
        ESP_LOGI(TAG, "osweep combo %2d: madctl=0x%02x mirror=%d",
                 combo, madctl, mirrored);
        Clear(kBlack);
        const uint8_t segs = kDigits[combo];
        for (int s = 0; s < 7; ++s)
            if (segs & (1 << s))
                FillRect(kSeg[s].x, kSeg[s].y, kSeg[s].w, kSeg[s].h, kWhite);
        for (int y = 0; y < H; ++y) dirty[y] = true;
        sendCommand(0x36, &madctl, 1);
        uint8_t c0 = 82, c1 = 157, r0 = 18, r1 = 301;
        if (mirrored) { c0 = 157; c1 = 82; r0 = 301; r1 = 18; }
        const uint8_t caset[4] = {(uint8_t)(c0 >> 8), (uint8_t)(c0 & 0xFF),
                                  (uint8_t)(c1 >> 8), (uint8_t)(c1 & 0xFF)};
        const uint8_t raset[4] = {(uint8_t)(r0 >> 8), (uint8_t)(r0 & 0xFF),
                                  (uint8_t)(r1 >> 8), (uint8_t)(r1 & 0xFF)};
        sendCommand(0x2A, caset, 4);
        sendCommand(0x2B, raset, 4);
        sendCommand(0x2C);
        writePixels(fb, W * H);
        vTaskDelay(pdMS_TO_TICKS(2500));  // hold for the user
    }
    s_testpat = -1;
    ESP_LOGI(TAG, "osweep: 16 combos done");
}

// ---- ST7789 init sequence — VERBATIM from TFT_eSPI ST7789_Init.h (the
// driver verified working on this exact 76x284 P3 panel). The old 7-command
// sketch missed the analog registers (PORCTRL/VCOMS/LCMCTRL/gammas/RAMCTRL)
// — the panel stayed white with every byte path until this went in.
// Format: {cmd, n, data...}, {0xFF} = end. MADCTL 0x60 = MX|MV, RGB order
// (rotation-1 landscape). SLPOUT/NORON/DISPON ride outside the table (they
// carry the datasheet delays).
static const uint8_t kSt7789InitSeq[] = {
    0x36, 1, 0x00,                    // MADCTL: NATIVE (no transforms) — the
                                      // bborient gradient proved the visible
                                      // window sits at native cols 82-157 ×
                                      // rows 18-301 with BOTH axes mirrored;
                                      // the mirrors live in Flush's loop
                                      // order, not in MADCTL.
    0xB6, 2, 0x0A, 0x82,              // DISCTRL
    0xB0, 2, 0x00, 0xE0,              // RAMCTRL (5-6 bit conversion)
    0x3A, 1, 0x55,                    // COLMOD 16bpp
    0xB2, 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,   // PORCTRL
    0xB7, 1, 0x35,                    // GCTRL
    0xBB, 1, 0x28,                    // VCOMS
    0xC0, 1, 0x0C,                    // LCMCTRL
    0xC2, 2, 0x01, 0xFF,              // VDVVRHEN
    0xC3, 1, 0x10,                    // VRHS
    0xC4, 1, 0x20,                    // VDVSET
    0xC6, 1, 0x0F,                    // FRCTR2
    0xD0, 2, 0xA4, 0xA1,              // PWCTRL1
    0xE0, 14, 0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32,
          0x44, 0x42, 0x06, 0x0E, 0x12, 0x14, 0x17,   // PVGAMCTRL
    0xE1, 14, 0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31,
          0x54, 0x47, 0x0E, 0x1C, 0x17, 0x1B, 0x1E,   // NVGAMCTRL
    0x21, 0,                          // INVON — the fork's init sends INVON
                                      // (INVOFF here inverted black->white =
                                      // the solid white we chased)
    0xFF, 0,                          // end
};

// bitbang=false: through the SPI peripheral (sendCommand). true: raw GPIO.
void Display::runInitSeq(bool bitbang) {
    for (const uint8_t *p = kSt7789InitSeq; p[0] != 0xFF;) {
        const uint8_t cmd = *p++;
        const uint8_t n = *p++;
        if (bitbang) {
            bbCmd(cmd);
            for (uint8_t i = 0; i < n; ++i) bbData(*p++);
        } else {
            sendCommand(cmd, p, n);
            p += n;
        }
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
    dev.mode = 3;                             // CPOL=1/CPHA=1: REQUIRED with CS
                                              // tied GND — mode 0 is only safe
                                              // when a real CS frames each byte
                                              // (white since c17d54c removed CS;
                                              // bbtest worked because bbBit
                                              // leaves SCK idling HIGH = mode 3)
    dev.spics_io_num = -1;                    // CS tied GND (single device);
                                              // the desync fix = the firmware
                                              // RST pulse on GPIO7, not CS
    dev.queue_size = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << ST7789_PIN_DC) | (1ULL << ST7789_PIN_RST);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 1);

    // ST7789P3 init (IPS bar): HARDWARE reset via the real RST line now —
    // the panel's serial framing starts from a known state after every
    // boot, no RC timing involved.
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    sendCommand(0x11);                        // SLPOUT
    vTaskDelay(pdMS_TO_TICKS(120));
    sendCommand(0x13);                        // NORON
    runInitSeq(false);                        // the VERBATIM TFT_eSPI sequence
    sendCommand(0x29);                        // DISPON
    vTaskDelay(pdMS_TO_TICKS(120));
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
    // TRANSPOSED panel addressing (bborient-derived 2026-11-09): the
    // visible window = native cols 82..157 x native rows 18..301, BOTH
    // axes mirrored (screen x=0 -> native row 301, screen y=0 -> native
    // col 157), MADCTL 0x00 native. Each dirty screen-ROW y = ONE native
    // column: CASET (col, col) x RASET (18, 301), and the row's pixels
    // streamed with the x-mirror folded into the order. Per-column
    // transactions also keep every frame CS-delimited (the bit counter
    // resyncs constantly) and stay far under the SPI DMA bit cap.
    static uint16_t line[W];              // byte-swapped + x-mirrored row
    for (int y = 0; y < H; ++y) {
        if (!dirty[y]) continue;
        const uint8_t col = (uint8_t)(157 - y);
        const uint8_t caset[4] = {0, col, 0, col};
        const uint8_t raset[4] = {0, 18, 1, 45};   // 18..301
        sendCommand(0x2A, caset, 4);
        sendCommand(0x2B, raset, 4);
        sendCommand(0x2C);
        for (int x = 0; x < W; ++x) {
            line[x] = fb[y * W + (W - 1 - x)];   // x-mirror only; writePixels
                                                 // does the byte swap
        }
        writePixels(line, W);
        dirty[y] = false;
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
