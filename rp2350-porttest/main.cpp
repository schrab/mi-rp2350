// Standalone Pico SDK test firmware: OpenRB-338 TB-303 + distortion -> PCM5102A.
// Board: bare Pico2 (RP2350), soldered SCK=GP19, DIN=GP20, LRCK=GP21.
// I2S variant drives DIN+LRCK via `out pins, 2` (they are consecutive),
// BCK via a 1-bit sideset; see i2s_out.pio. Standard Philips I2S timing.
// No rack files are used or modified.
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "i2s_out.pio.h"
#include "orb303.h"
#include "pcfcomp.h"

#define FS_HZ 44100u
#define SYS_KHZ 320000u
#define NBUF 8u
#define WORDS (ORB_BLOCK * 2)      // one FIFO word per half-frame (L then R)
#define PIN_BCK 19                 // sideset bit 0
#define PIN_DIN 20                 // OUT pin 0
#define PIN_LR 21                  // OUT pin 1 (consecutive with DIN)

static PIO s_pio = pio0;
static uint s_sm = 0;
static uint s_dma = 0;
static bool s_running = false;
static uint32_t s_silence[WORDS];
static uint32_t s_bank[NBUF][WORDS];

static volatile int s_dma_buf = -1;
static unsigned s_ready_q[NBUF];
static unsigned s_free_q[NBUF];
static volatile unsigned s_ready_head, s_ready_tail;
static volatile unsigned s_free_head, s_free_tail;
static volatile unsigned s_fill_buf, s_fill_pos;
static volatile uint32_t s_lost = 0;

static inline void dma_arm(const uint32_t* buf, uint words) {
    dma_channel_config c = dma_channel_get_default_config(s_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(s_pio, s_sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(s_dma, &c,
                          reinterpret_cast<volatile void*>(&s_pio->txf[s_sm]),
                          buf, words, true);
}

static void dma_irq() {
    dma_hw->intr = 1u << s_dma;            // ack first
    if (!s_running) return;
    if (s_dma_buf >= 0) {
        s_free_q[s_free_head] = (unsigned)s_dma_buf;
        s_free_head = (s_free_head + 1u) % NBUF;
        s_dma_buf = -1;
    }
    if (s_ready_tail != s_ready_head) {
        unsigned b = s_ready_q[s_ready_tail];
        s_ready_tail = (s_ready_tail + 1u) % NBUF;
        s_dma_buf = (int)b;
        dma_arm(s_bank[b], WORDS);
    } else {
        dma_arm(s_silence, WORDS);
        s_lost++;
    }
}

static void i2s_init() {
    uint off = pio_add_program(s_pio, &i2s_out_program);
    pio_sm_config c = i2s_out_program_get_default_config(off);
    sm_config_set_out_pins(&c, PIN_DIN, 2);    // DIN (GP20) + LRCK (GP21)
    sm_config_set_sideset_pins(&c, PIN_BCK);   // BCK only, 1 bit
    sm_config_set_out_shift(&c, false /* left = bit 31 out first */, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    // y = bits - 2 for the half-frame loop counter.
    pio_sm_exec(s_pio, s_sm, pio_encode_set(pio_y, 14));
    // 33 instructions per half-frame = 66 per stereo frame.
    float div = (float)clock_get_hz(clk_sys) / (float)(FS_HZ * 66u);
    sm_config_set_clkdiv(&c, div);

    for (uint p = PIN_BCK; p <= PIN_LR; ++p) {
        pio_gpio_init(s_pio, p);
        gpio_set_dir(p, GPIO_OUT);
    }
    pio_sm_init(s_pio, s_sm, off, &c);
    pio_sm_set_enabled(s_pio, s_sm, true);

    memset(s_silence, 0, sizeof(s_silence));
    memset(s_bank, 0, sizeof(s_bank));
    s_ready_head = s_ready_tail = 0;
    for (unsigned b = 1; b < NBUF; ++b) s_free_q[b - 1] = b;
    s_free_head = NBUF - 1;
    s_free_tail = 0;
    s_fill_buf = 0;
    s_fill_pos = 0;
    s_dma_buf = -1;

    s_dma = (uint)dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq);
    dma_channel_set_irq0_enabled(s_dma, true);
    s_running = true;

    dma_arm(s_silence, WORDS);
    irq_set_enabled(DMA_IRQ_0, true);
}

static void handoff() {
    s_ready_q[s_ready_head] = s_fill_buf;
    s_ready_head = (s_ready_head + 1u) % NBUF;
    while (s_free_tail == s_free_head) tight_loop_contents();
    s_fill_buf = s_free_q[s_free_tail];
    s_free_tail = (s_free_tail + 1u) % NBUF;
    s_fill_pos = 0;
}

static inline int16_t to_i16(float x) {
    double v = (double)x * 32768.0;
    if (v > 32767.0) v = 32767.0;
    if (v < -32768.0) v = -32768.0;
    return (int16_t)v;
}

// Build one half-frame FIFO word: 16 bit-slots of (LRCK << 1) | DIN,
// MSB-first so `out pins, 2` emits the first slot first. LRCK flips on the
// last slot (standard I2S 1-BCK delay before the next channel's MSB).
// 0 = standard I2S (LRCK flips on the last data bit), 1 = left-justified
// (LRCK flips on the first bit of a channel — some PCM5102 FMT straps want it).
#define FORMAT_LJ 0    // H4L=FMT→L on this board means standard I2S

static inline uint32_t pack_half(uint16_t sample, uint32_t lr, uint32_t lr_last) {
    uint32_t w = 0;
    for (int i = 0; i < 16; ++i) {
        uint32_t bit = (sample >> (15 - i)) & 1u;
#if FORMAT_LJ
        uint32_t pair = (lr << 1) | bit;          // LRCK constant in the half
#else
        uint32_t pair = ((i == 15 ? lr_last : lr) << 1) | bit;
#endif
        w |= pair << (30 - 2 * i);
    }
    return w;
}

// DWT cycle counter (raw CMSIS addresses); verified at startup.
#define DWT_CTRL_REG  (*(volatile uint32_t*)0xE0001000u)
#define DWT_CYCCNT_REG (*(volatile uint32_t*)0xE0001004u)
#define DWT_LAR_REG   (*(volatile uint32_t*)0xE0001FB0u)
#define DEMCR_REG     (*(volatile uint32_t*)0xE000EDFCu)
static bool s_dwt_ok = false;

// SWD-readable diagnostics (no USB needed).
volatile uint32_t g_blocks = 0;
volatile uint32_t g_last_us = 0;
volatile uint32_t g_max_us = 0;
volatile uint32_t g_last_cyc = 0;
volatile uint32_t g_max_cyc = 0;
volatile uint32_t g_lost = 0;
volatile float g_out_peak = 0;

static void dwt_init() {
    DEMCR_REG |= 1u << 24;
    DWT_LAR_REG = 0xC5ACCE55u;         // unlock the M33 DWT
    DWT_CYCCNT_REG = 0x12345678u;
    if (DWT_CYCCNT_REG == 0x12345678u) {
        s_dwt_ok = true;
        DWT_CYCCNT_REG = 0;
        DWT_CTRL_REG |= 1u;
    }
}

// Same pre-clock sequence as rack-pico: VREG 1.25 V and QMI CLKDIV 4 /
// RXDELAY 3 before the PLL is raised (keeps flash SCK at 80 MHz at 320 MHz).
static void pre_clock() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    uint32_t t = qmi_hw->m[0].timing;
    t = (t & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
        (4u << QMI_M0_TIMING_CLKDIV_LSB) | (3u << QMI_M0_TIMING_RXDELAY_LSB);
    qmi_hw->m[0].timing = t;
    volatile uint8_t xip_dummy = *reinterpret_cast<volatile uint8_t*>(0x10000000u);
    (void)xip_dummy;
}

int main() {
    pre_clock();
    set_sys_clock_khz(SYS_KHZ, true);
    stdio_init_all();
    dwt_init();
    orb_init();
    pcfcomp_init();
    i2s_init();

    printf("porttest up: %u Hz, sys %u kHz, SCK=%u DIN=%u LR=%u, DWT=%d\r\n",
           FS_HZ, SYS_KHZ, PIN_BCK, PIN_DIN, PIN_LR, (int)s_dwt_ok);

    uint32_t block = 0;
    uint32_t max_us = 0, sum_us = 0;
    uint32_t max_cyc = 0, sum_cyc = 0;
    uint32_t report_t0 = time_us_32();
    uint32_t ticks = 0;

    for (;;) {
        if (s_fill_pos >= WORDS) handoff();
        OrbBlock bus;
        uint32_t t0 = time_us_32();
        uint32_t c0 = s_dwt_ok ? DWT_CYCCNT_REG : 0;
        orb_tick_step((int)ticks);
        orb_process(bus);          // 303 vintage -> distortion
        pcfcomp_process(bus, (int)ticks);  // PCF -> compressor
        uint32_t cyc = s_dwt_ok ? (DWT_CYCCNT_REG - c0) : 0;
        uint32_t us = time_us_32() - t0;
        if (us > max_us) max_us = us;
        sum_us += us;
        if (cyc > max_cyc) max_cyc = cyc;
        sum_cyc += cyc;
        g_last_us = us; g_max_us = max_us;
        g_last_cyc = cyc; g_max_cyc = max_cyc;
        g_blocks++;
        g_lost = s_lost;
        {
            float pk = 0;
            for (int i = 0; i < ORB_BLOCK; ++i) {
                float a = bus[i] < 0 ? -bus[i] : bus[i];
                if (a > pk) pk = a;
            }
            g_out_peak = pk;
        }
        ++block;
        ++ticks;

        // Each sample becomes two FIFO words: a left half (LRCK low, then
        // high on its last bit) and a right half (LRCK high, then low).
        for (int i = 0; i < ORB_BLOCK; ++i) {
            uint16_t s = (uint16_t)to_i16(bus[i]);
            s_bank[s_fill_buf][s_fill_pos++] = pack_half(s, 0, 1);
            s_bank[s_fill_buf][s_fill_pos++] = pack_half(s, 1, 0);
        }

        uint32_t now = time_us_32();
        if (now - report_t0 >= 2000000u) {
            printf("blocks=%u avg=%u us max=%u us | avg=%u cyc max=%u cyc | lost=%u\r\n",
                   block, sum_us / block, max_us,
                   sum_cyc / block, max_cyc, (unsigned)s_lost);
            report_t0 = now;
            block = 0; sum_us = 0; max_us = 0; sum_cyc = 0; max_cyc = 0;
        }
    }
}
