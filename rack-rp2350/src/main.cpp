// Rack RP2350 — all four accepted module ports (Elements, Clouds, Plaits
// 98-engine, Rings MiniElements) in ONE firmware with LIVE MODULE SWITCHING
// (T2), on the RP2350 nano + PCM5102A, using the proven elements-rp2350
// architecture:
//   core 0 = USB MIDI device (CDC + USB-MIDI) + PIO-USB host MIDI + DIN MIDI
//            in (UART1 31250, T4) + serial console + status LED + diagnostics;
//   core 1 = audio render of the ACTIVE module at its native rate/block
//            size into the I2S DAC, plus the live-swap sequence at block
//            boundaries.
//
// T1 boot-select, T2 unified: ALL FOUR module adapters (src/module_*.h)
// compile in every env — their state is ARENA-PLACED (rack_arena.h, no
// file-scope module objects) and their hot sections share ONE SRAM window
// via the tc_* OVERLAY in memmap_rack.ld (same VMA, sequential LMAs).
// RACK_BOOT_MODULE (0 elements / 1 clouds / 2 plaits / 3 rings / 4 lira8;
// new modules continue the ids) selects the BOOT module only: setup() copies
// that module's flash image into the window as its ABSOLUTE FIRST action
// (crt0-like) and registry-inits it; after that the build is identical and
// any module can be swapped in at runtime. One env — the old five per-module
// envs were a testing-phase artifact.
//
// Live swap (core 1, at a block boundary, max one per loop1 call):
//   CC 32 (bank select) value 0-4 or serial `switch <id|name>` -> core 0
//   writes the pending slot -> core 1: mute (ramp-down + silence bank) ->
//   rack_window_copy (flash->window memcpy; no I/D caches to invalidate, T1
//   finding) -> placement-new + Init (factory defaults) -> I2S reconfigure
//   (rate/bank per module) -> ramp-up on the new module -> stats hand-off to
//   core 0 for the print. MIDI dropped during the swap window (see below).
//
// SRAM law: the module hot code runs from the SRAM window (copied at boot /
// swap), the platform's own .time_critical sections (pio_usb, framework)
// stay in .data via the core linker rule — the adapters call into the window
// exactly like the standalone ports called into their crt0-copied
// .time_critical sections.
//
// T4 DIN + console: DIN 31250 bytes arrive on UART1 RX (GP5, standard 6N138
// opto circuit — wiring table in README.md); the core's SerialUART IRQ keeps
// a 256 B ring fed, loop() parses bytes (src/din_midi.h — a pure, host-unit-
// tested parser) and feeds the SAME midiHandleEventPacket funnel as the USB
// device/host paths, so DIN notes/CC/clock are indistinguishable downstream
// (DIN F8/FA/FB/FC reach the mod engine's sample-stamped clock bus with zero
// extra code). The serial console table below drives every bench flow:
// switch/module/mod/mod?/route/bpm/clock/status/din/dintest/telemetry/panic.
//
// T5 stage 1 (src/ui_link.h — same header-only, single-TU pattern): the
// ESP32-C3 UI MCU exchanges framed 921600 8N1 packets on HW UART0
// (TX GP0 / RX GP1 — the board only populates GPIO 0-5 + 26-29):
// MIDI/SWITCH/MOD_SET/UI_CMD/CHAIN_ROUTE frames in ->
// the SAME funnel/console paths, TELEM page snapshots out (full snapshot on
// a UI_CMD page request + a >= 50 ms throttled 1-cell delta after
// control_change on the active module). Frame format + protocol contract:
// docs/ui-protocol.md. Link loss never affects audio or MIDI (the C3 is
// strictly additive; the pump costs one ring-empty check when absent).
//
// Wiring (identical to the four standalone ports): PCM5102A BCK GP6 /
// DIN GP8 / LRCK GP7, XSMT 3V3; USB-A host D+ GP14 / D- GP15, VBUS 5V;
// WS2812 GP22.
//
// Init order (hard-won, see the ledger): USB host (PIO-USB) BEFORE
// i2s.begin() — PIO-USB hard-claims all of PIO0; the status LED claims PIO2
// SM0; I2S lands on PIO2 SM1. Reordering panics at boot. PIO-USB needs a
// 12 MHz-multiple f_cpu; the 320 MHz build default is fine while host is
// disabled (RACK_ENABLE_USB_HOST=0) — use 312 MHz if host is re-enabled.
#include <Arduino.h>
#include <I2S.h>
#include <stdarg.h>              // dbg_printf (non-blocking diagnostics TX)

// Give core 1 its own 8 KB stack instead of the default 4 KB/4 KB split with
// core 0 (the standalone ports' core 0 overran the shared half at USB attach
// and corrupted core 1).
bool core1_separate_stack = true;
#include <Adafruit_TinyUSB.h>
#include "hardware/structs/scb.h"   // scb_hw: fault-status registers + AIRCR
#include "hardware/structs/qmi.h"   // T9 clock diag: QMI M0 flash timing
#include "hardware/regs/qmi.h"      // QMI_M0_TIMING_CLKDIV/RXDELAY field masks
#include "hardware/vreg.h"          // T9 oc bench: vreg raise above 280 MHz

// T9 clock note: the RP2350's flash (QMI) timing is set by the ROM (CLKDIV 3,
// RXDELAY 2) and does NOT re-scale with set_sys_clock_khz — flash SCK =
// clk_sys/CLKDIV. The build default is 320 MHz; the pre-clock constructor
// below pins CLKDIV 4 / RXDELAY 3 (SCK 80 MHz) and raises VREG to 1.20 V
// BEFORE the core's main() calls set_sys_clock_khz(F_CPU). 270 MHz at the
// default 1.15 V reset in soak (reason 3); 1.20 V was stable through 320 MHz.
// At CLKDIV 3, 280 would put SCK at 93 MHz and corrupt XIP (bad-CRC TELEM,
// dark CDC, bootloop — the c1e344a revert). Live readback: `qmi`, `vreg`.
#include "hardware/watchdog.h"      // core-0 hang watchdog

// ================= switchable hang breadcrumbs (PERSISTENT) =================
// RACK_HANG_DEBUG (platformio.ini) keeps these in the tree permanently — set
// the flag 0 for production, 1 to hunt a freeze. Never delete and rewrite.
// A hang (not a fault) leaves USB enumerated but the console dead and writes
// no crash record; the 2 s watchdog reboots it and the last core-0/core-1
// stage survives in watchdog scratch[0]/[1] (SRAM is not guaranteed across a
// watchdog reset, scratch is). Printed as CRUMB / w1= at the next boot.
#ifndef RACK_HANG_DEBUG
#define RACK_HANG_DEBUG 0
#endif
static constexpr uint32_t kCrumbMagic = 0xC0DE0000u;
static inline void crumb(uint32_t stage) {
#if RACK_HANG_DEBUG
  watchdog_hw->scratch[0] = (stage == 0) ? 0 : (kCrumbMagic | (stage & 0x3Fu));
#else
  (void)stage;
#endif
}
static inline void crumb1(uint32_t stage) {
#if RACK_HANG_DEBUG
  watchdog_hw->scratch[1] = 0xB0000000u | (stage & 0x3Fu);
#else
  (void)stage;
#endif
}
#if RACK_HANG_DEBUG
static uint32_t g_hang0 = 0, g_hang1 = 0;   // survive until next clean boot
#endif

// Pre-clock setup (runs BEFORE the Arduino core's main(), which applies
// F_CPU via set_sys_clock_khz). Two things MUST happen before the PLL is
// raised: (1) VREG 1.20 V — the core is unstable at 1.15 V above ~264 MHz;
// the core's own auto-adjust is RP2040-only, so RP2350 needs this explicitly;
// (2) QMI CLKDIV 4 / RXDELAY 3 — keeps flash SCK at 80 MHz instead of the
// ROM's 106.7 MHz at 320. A dummy XIP read + barrier after the timing write
// follows the SDK's psram_reinit_timing precedent.
__attribute__((constructor(101))) static void rack_pre_clock(void) {
  // 1.25 V (not 1.20): the Warps DSP's sustained load at 320 MHz locked the
  // core up at 1.20 V under repeated CC/switch traffic; 1.25 V is stable.
  vreg_set_voltage(VREG_VOLTAGE_1_25);
  uint32_t t = qmi_hw->m[0].timing;
  t = (t & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
      (4u << QMI_M0_TIMING_CLKDIV_LSB) | (3u << QMI_M0_TIMING_RXDELAY_LSB);
  qmi_hw->m[0].timing = t;
  volatile uint8_t xip_dummy = *reinterpret_cast<volatile uint8_t*>(0x10000000u);
  (void)xip_dummy;
  __asm volatile("dsb 0xF" ::: "memory");
}

#ifndef RACK_ENABLE_USB_HOST
#define RACK_ENABLE_USB_HOST 0
#endif

#if RACK_ENABLE_USB_HOST
#include "pio_usb.h"             // Pico-PIO-USB (vendored in lib/pio_usb/)
#endif

#include "status_led.h"          // WS2812 status LED on GP22 (before midi.h)
#include "midi.h"                // shared parser + telemetry externs + hooks
#include "din_midi.h"            // T4 DIN 31250 byte parser -> 4-byte packets
#include "rack_window.h"         // SRAM window copy API
#include "rack_arena.h"          // T2 module-state arenas (decls)
#include "piezo.h"               // T2 piezo input (GP26 ADC0, software Ears)
#include "hardware/gpio.h"       // gpio_pull_up (DIN RX idle mark)
#include "hardware/irq.h"        // DMA_IRQ_0 / irq_set_enabled (I2S IRQ handoff)

#if RACK_ENABLE_USB_HOST
// PIO-USB root-port accessors for the re-enumeration watchdog (C side,
// usb_host_watchdog.c).
extern "C" {
bool pio_host_line_idle(void);
bool pio_host_connected(void);
void pio_host_force_reconnect(void);
uint8_t pio_host_line_state(void);
}
#endif

#if !defined(RACK_BOOT_MODULE) || RACK_BOOT_MODULE < 0 || RACK_BOOT_MODULE > 4
#error "RACK_BOOT_MODULE must be defined in the env as 0 (elements), 1 (clouds), 2 (plaits), 3 (rings) or 4 (lira8)"
#endif

// Non-blocking core-0 diagnostic output (defined near loop()). The 2 s
// report used to do a blocking USB-CDC printf: ~250-400 B into a 256 B
// TinyUSB FIFO, which blocks for milliseconds when no console is attached.
// That starved the core-0 chain-B worker and showed up as 1500 us chain
// peaks. All periodic diagnostics now enqueue here and a loop-side pump
// writes only what the CDC can take immediately (drops when nobody reads).
void dbg_printf(const char* fmt, ...);

// The four module adapters — ALL FOUR compile in every env (their state is
// arena-placed; their hooks are namespaced rack_modules::mod_<module>).
// Upstream MI resource headers define global lookup table macros (LUT_*);
// undefine collisions between module headers to ensure a zero-warning build.
#include "module_elements.h"
#undef LUT_FM_FREQUENCY_QUANTIZER_SIZE
#undef LUT_SVF_SHIFT_SIZE

#include "module_clouds.h"

#include "module_plaits.h"
#undef LUT_SINE
#undef LUT_SINE_SIZE
#undef LUT_FM_FREQUENCY_QUANTIZER
#undef LUT_FM_FREQUENCY_QUANTIZER_SIZE
#undef LUT_FOLD
#undef LUT_FOLD_SIZE
#undef LUT_FOLD_2
#undef LUT_FOLD_2_SIZE
#undef LUT_STIFFNESS
#undef LUT_STIFFNESS_SIZE
#undef LUT_SVF_SHIFT
#undef LUT_SVF_SHIFT_SIZE
#undef LUT_4X_DOWNSAMPLER_FIR
#undef LUT_4X_DOWNSAMPLER_FIR_SIZE
#undef LUT_WS_INVERSE_TAN
#undef LUT_WS_INVERSE_TAN_SIZE
#undef LUT_WS_INVERSE_SIN
#undef LUT_WS_INVERSE_SIN_SIZE
#undef LUT_WS_LINEAR
#undef LUT_WS_LINEAR_SIZE
#undef LUT_WS_BUMP
#undef LUT_WS_BUMP_SIZE
#undef LUT_WS_DOUBLE_BUMP
#undef LUT_WS_DOUBLE_BUMP_SIZE
#undef LUT_WS_DOUBLE_BUMP_SENTINEL
#undef LUT_WS_DOUBLE_BUMP_SENTINEL_SIZE
#undef LUT_LPC_EXCITATION_PULSE
#undef LUT_LPC_EXCITATION_PULSE_SIZE

#include "module_rings.h"

// rack-lira8 (2026-11-15): the hand-ported LIRA-8 (Pure Data -> C++, TDD
// against headless-pd goldens). Pure C++ engine in src/lira8/ — no MI lib
// includes, no LUT_* macro collisions. The engine renders pd-style 64-sample
// blocks internally (the adapter accumulates the 16-sample rack grid; every
// 4th call bursts ~4x its budget share — the 4 ms I2S bank absorbs it).
#include "module_lira8.h"

#include "module_registry.h"

// T3 persistent mod engine + clock bus: needs the ADAPTER HOOKS above (it
// writes the modules' shared-float/pending machinery through
// mod_set_cc_float / mod_trigger_cc) and the registry table below (its
// functions reference the kRackModules extern; the table itself is defined
// after this include). Its state object g_mod_engine is defined below —
// plain .bss, OUTSIDE the arenas, untouched by the swap path.
#include "mod_engine.h"

// T4a preset replay (defined in preset.h, included after ui_link.h below):
// the swap/chain sequences call it after every factory re-Init so a module
// comes back with the panel's observed values instead of factory defaults.
void preset_replay_module(uint8_t mod);

// ===================== module registry =====================
// main.cpp + midi.h dispatch through this table (no per-adapter #if left).
const RackModuleInfo kRackModules[RACK_MODULE_COUNT] = {
    {"elements", RACK_MODULE_ELEMENTS,
     rack_modules::mod_elements::kSampleRate,
     (uint32_t)rack_modules::mod_elements::kBlockSize,
     rack_modules::mod_elements::kBudgetUs,
     rack_modules::mod_elements::kI2sBuffers,
     rack_modules::mod_elements::kI2sWords,
     rack_modules::mod_elements::kStallUs,
     rack_modules::mod_elements::kCcMap,
     &rack_modules::mod_elements::kModDestinations,
     rack_modules::mod_elements::init,
     rack_modules::mod_elements::render,
     rack_modules::mod_elements::note_on,
     rack_modules::mod_elements::note_off,
     rack_modules::mod_elements::control_change},
    {"clouds", RACK_MODULE_CLOUDS,
     rack_modules::mod_clouds::kSampleRate,
     (uint32_t)rack_modules::mod_clouds::kBlockSize,
     rack_modules::mod_clouds::kBudgetUs,
     rack_modules::mod_clouds::kI2sBuffers,
     rack_modules::mod_clouds::kI2sWords,
     rack_modules::mod_clouds::kStallUs,
     rack_modules::mod_clouds::kCcMap,
     &rack_modules::mod_clouds::kModDestinations,
     rack_modules::mod_clouds::init,
     rack_modules::mod_clouds::render,
     rack_modules::mod_clouds::note_on,
     rack_modules::mod_clouds::note_off,
     rack_modules::mod_clouds::control_change},
    {"plaits", RACK_MODULE_PLAITS,
     rack_modules::mod_plaits::kSampleRate,
     (uint32_t)rack_modules::mod_plaits::kBlockSize,
     rack_modules::mod_plaits::kBudgetUs,
     rack_modules::mod_plaits::kI2sBuffers,
     rack_modules::mod_plaits::kI2sWords,
     rack_modules::mod_plaits::kStallUs,
     rack_modules::mod_plaits::kCcMap,
     &rack_modules::mod_plaits::kModDestinations,
     rack_modules::mod_plaits::init,
     rack_modules::mod_plaits::render,
     rack_modules::mod_plaits::note_on,
     rack_modules::mod_plaits::note_off,
     rack_modules::mod_plaits::control_change},
    {"rings", RACK_MODULE_RINGS,
     rack_modules::mod_rings::kSampleRate,
     (uint32_t)rack_modules::mod_rings::kBlockSize,
     rack_modules::mod_rings::kBudgetUs,
     rack_modules::mod_rings::kI2sBuffers,
     rack_modules::mod_rings::kI2sWords,
     rack_modules::mod_rings::kStallUs,
     rack_modules::mod_rings::kCcMap,
     &rack_modules::mod_rings::kModDestinations,
     rack_modules::mod_rings::init,
     rack_modules::mod_rings::render,
     rack_modules::mod_rings::note_on,
     rack_modules::mod_rings::note_off,
     rack_modules::mod_rings::control_change},
    {"lira8", RACK_MODULE_LIRA8,
     rack_modules::mod_lira8::kSampleRate,
     (uint32_t)rack_modules::mod_lira8::kBlockSize,
     rack_modules::mod_lira8::kBudgetUs,
     rack_modules::mod_lira8::kI2sBuffers,
     rack_modules::mod_lira8::kI2sWords,
     rack_modules::mod_lira8::kStallUs,
     rack_modules::mod_lira8::kCcMap,
     &rack_modules::mod_lira8::kModDestinations,
     rack_modules::mod_lira8::init,
     rack_modules::mod_lira8::render,
     rack_modules::mod_lira8::note_on,
     rack_modules::mod_lira8::note_off,
     rack_modules::mod_lira8::control_change},
};

// ===================== shared definitions =====================
static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
#if RACK_ENABLE_USB_HOST
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15
#endif
// T1 hardware PLAY button: GP2 (internal pull-up, button -> GND). GP3 is the
// spare alternate; GP4/GP5 are DIN UART1 (TX reserved / RX) — never wire the
// button or piezo there. The piezo (T2+) goes on the ADC pins GP26-29.
static const uint8_t kPinButton = 2;

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);
volatile float shared_note = 48.0f;      // idle note (C3); overwritten by MIDI
volatile bool  shared_gate = false;
volatile uint32_t note_on_count = 0;
volatile float rack_meter_a = 0.0f;
volatile float rack_meter_b = 0.0f;
volatile uint32_t proc_us_max = 0;
volatile uint32_t i2s_stall_count = 0;
volatile float g_render_gain = 1.0f;     // swap-mute ramp (see midi.h)
// T6 chain grid overrides for the mod engine (declared in mod_engine.h):
// nonzero while a chain is engaged — the engine's per-block pass then runs
// the 32 kHz / 24-sample chain grid instead of the A slot's registry block.
volatile uint8_t g_mod_chain_grid = 0;
volatile uint8_t g_mod_chain_rate_khz = 0;

// T3 persistent mod engine + clock bus — plain .bss, OUTSIDE the arenas, and
// NEVER touched by the swap path (do_module_swap references none of it): its
// clock phase, source state and routings survive every module swap. Defaults
// are installed by mod_engine_init() in setup().
ModEngine g_mod_engine;

// T8 unified module pool (rack_arena.h): ONE 324 KiB .bss union replacing
// the old OBJECT (132 KiB) + BUFFER (192 KiB) arrays. Sized by the
// rings+clouds chain as before; the bump allocator makes the capacity
// elastic per resident set. alignas(8) for placement-new + the plaits
// BufferAllocator workspace.
alignas(8) uint8_t g_rack_pool[kRackPoolBytes];
static size_t g_rack_mem_top = 0;

void rack_mem_reset() { g_rack_mem_top = 0; }
size_t rack_mem_used() { return g_rack_mem_top; }

void* rack_mem_alloc(size_t bytes, size_t align) {
  if (align == 0) align = 1;
  const size_t p = (g_rack_mem_top + (align - 1)) & ~(align - 1);
  if (p + bytes > kRackPoolBytes) {
    // A silent nullptr would corrupt the DSP loop; a broken resident-set
    // size must fail here, loudly (boot/swap console is up in every path).
    Serial.printf("rack: FATAL: module SRAM pool overflow (%u used + %u needed > %u) - halting\r\n",
                  (unsigned)g_rack_mem_top, (unsigned)bytes,
                  (unsigned)kRackPoolBytes);
    while (true) delay(10);
  }
  void* out = g_rack_pool + p;
  g_rack_mem_top = p + bytes;
  return out;
}

// Status LED instance — defined here (status_led.h holds only the class).
StatusLed statusLed;

static volatile bool audio_ready = false;

// --- live-swap state (T2) ---
// g_active_module: written by core 1 (boot default set before audio starts),
// read by core 0's registry dispatchers + report + the T3 mod engine's
// set-time destination validation. NON-static since T3 (mod_engine.h declares
// the extern; nothing else changed).
volatile uint8_t g_active_module = RACK_BOOT_MODULE;
// Pending swap slot: core 0 (CC 32 / console) writes a module id, core 1
// consumes it at the TOP of loop1 — max one swap per call, strictly at a
// block boundary.
static volatile int8_t g_pending_module = -1;
// Swap-window flag: true while core 1 runs the swap sequence. Core 0's
// adapter dispatch drops note/CC hooks during it (the note STACK still
// updates, so pitch/gate survive) — the old module's state pointer aliases
// the new module's fresh arena placement mid-swap, so racing adapter writes
// would land inside the new module's freshly-Init'ed state.
static volatile bool g_swapping = false;
// Swap stats hand-off: core 1 sets, core 0 prints exactly once (core 1 must
// not Serial.printf into a busy CDC).
static volatile bool g_swap_report_ready = false;
static volatile uint32_t g_swap_duration_ms = 0;
static volatile uint32_t g_swap_copied_bytes = 0;
static volatile uint8_t g_swap_to = 0;
static volatile bool g_swap_i2s_ok = false;

// ===================== T6: chain mode (v2) =====================
// A->B serial chain. PERSISTENT STATE in main.cpp-side .bss (NOT module
// state): routing is persistent state, not module state (plan "Live swap
// while chained" — a slot re-Init re-patches the routing automatically).
// For T6 the pair table is X->clouds (user-amended 2026-11-05: elements,
// plaits, rings as A; clouds is the only B).
//
// CHAIN GRID: 24-sample blocks @ 32 kHz (750 us) — rings/clouds are
// 32k-native and their budgets die at 48 kHz; chained plaits would render
// its DSP at 32 kHz via the runtime kSampleRate (08-plaits-runtime-rate).
// A renders the 24-sample block into float buffers (elements: two <=16-
// sample Part::Process sub-calls), the ROUTE copies A's out/aux into
// clouds' stereo FloatFrame input at gain 1.0 (the adapters' float outputs
// are the ±1.0 class the write path Clip16s to I2S — same magnitude class
// the standalone test exciter produced), B = clouds processes 24 frames,
// and B's output goes to I2S (the chain OUTPUT is B — clouds is the effect
// stage). g_render_gain (swap ramps) applies at the write stage as today.
//
// TRANSITIONS = rate-context events (mute -> window copies -> I2S
// reconfigure on CORE 0 via the chain handshake -> re-Init both slots ->
// unmute), the same discipline as do_module_swap. While chained
// g_active_module == the A slot: MIDI notes/CC route to A (except the
// B-slot CCs 23/24/65/66, see midi.h), the mod engine binds its
// destinations to A, and its grid override runs the chain block/rate.
struct ChainState {
  bool active;
  uint8_t a;   // A slot module id (the sound source)
  uint8_t b;   // B slot module id — clouds in every T6 pair
};
static ChainState g_chain = { false, RACK_MODULE_ELEMENTS, RACK_MODULE_CLOUDS };
// Pending transition slot: core 0 (console) queues, core 1 consumes at a
// block boundary (one transition per loop1 call, g_swapping held throughout).
static volatile int8_t g_pending_chain = -1;    // -1 none / 1 engage / 0 disengage
static volatile int8_t g_pending_chain_a = -1;  // target A slot (elements/plaits/rings)
static volatile int8_t g_pending_chain_b = -1;  // target B slot (clouds)
static volatile bool g_pending_chain_a_reinit = false;  // A-swap while chained
// Chain transition stats hand-off (core 1 sets, core 0 prints once).
static volatile bool g_chain_report_ready = false;
static volatile uint8_t g_chain_report_kind = 0;   // 0 off / 1 engage / 2 a-reinit
static volatile bool g_chain_report_ok = false;
static volatile uint32_t g_chain_report_ms = 0;
static volatile uint32_t g_chain_report_bytes = 0;

// The chain grid + the per-pair budget (750 us incl. the mod engine's
// <= 50 us — the report's budget field switches to this while chained).
static const size_t kChainBlockSize = 24;   // rings/plaits/clouds max block
static const uint32_t kChainRate = 32000;
static const uint32_t kChainBudgetUs = 750;
static const uint32_t kChainStallUs = 1400; // ~1.8x the 750 us block period

// I2S bank in chain mode = clouds' config (32 x 16 words = 16 ms): the
// largest bank of any chain participant; it absorbs clouds' one-off
// mode-change spike (~1.7 ms) with the same margin the standalone has.
static const uint32_t kChainI2sBuffers = 32;
static const uint32_t kChainI2sWords = 16;

// CHAINED CLOUDS BUFFER SPLIT — sized for the CORRECTED FX-workspace need.
// clouds' Prepare() allocates diffuser 8,192 + reverb 32,768 + correlator/
// pitch-shifter 8,192 (the pitch shifter's FxEngine<4096> clears 4,096 int16
// samples through the correlator pointer — the old 1,560 B allocation wrote
// 6.6 KB past the workspace; see tools/patches/clouds/clouds-z-pitchshifter-
// workspace.patch and the ASan proof in the ledger) = **49,152 B workspace**.
// - Stereo (quality 0 / 2): per-channel sample memory = small, workspace =
//   large - small. Needs large - small >= 49,152.
// - Mono (quality 1 / 3): sample = large, workspace = small >= 49,152 — and
//   stereo additionally needs 2*small + 49,152 <= carve. For the tightest
//   pair (rings: carve 131,840 B) that is impossible, so mono quality stays
//   clamped for chained clouds-B (the adapter + dispatcher reject it); the
//   stereo split below satisfies its workspace with 128 B margin.
//   stereo channels = 41,216 B = 0.64 s/ch @ 32 kHz.
static const size_t kChainCloudsLargeBytes = 90496;   // stereo: workspace 49,280
static const size_t kChainCloudsSmallBytes = 41216;   // stereo: channel size
// Per-A buffer demand in the unified pool (elements/rings reverb 64 KB,
// plaits workspace 32 KB, lira8 32 kHz delay 128,016 B).
static const size_t kChainABufferBytes[RACK_MODULE_COUNT] = {
    65536, 0, 32768, 65536,
    // lira8: 1 s int16 delay at the 32 kHz chain grid = (32000+4)*2*2 =
    // 128,016 B. With clouds-B's 131,072 B the pair needs 283,872 B of the
    // 331,776 B unified pool (47,904 spare). T8: the unified pool is what
    // makes this pair possible — the old 192 KiB buffer arena rejected it
    // (259,088 B needed) while 110 KB of the object arena sat idle.
    128016,
};

// Chain render block buffers (24 x 2 floats x 2 = 384 B in .bss — the
// routing plumbing the brief sizes at ~192 B per direction).
static float chain_a_out_l[kChainBlockSize];
static float chain_a_out_r[kChainBlockSize];
static float chain_b_out_l[kChainBlockSize];
static float chain_b_out_r[kChainBlockSize];

// ===================== T8 dual-core chain pipeline =====================
// Core 1 (audio) renders A + writes I2S; core 0 (control, idle DSP-wise)
// renders clouds B BEHIND by a fixed kChainPipeLag chain blocks (2 = 1.5 ms).
// The serial dependency A->B remains, but the wall time per block becomes
// max(A, B) instead of A + B: A's ~830 us lira8 burst no longer sits in front
// of clouds on the same core.
//
// LAG-DEPTH LAW (the second attempt; the first kept a 1:1 handshake and so got
// no benefit): every block carries a SEQUENCE number, core 1 consumes the B
// block for (current - kChainPipeLag), and the B ring is PRIMED with that many
// sync-rendered blocks in chain_pipe_start(). So in steady state core 1 never
// waits at all — core 0 has had kChainPipeLag x 750 us to finish B, which
// absorbs its 350-1200 us loop-latency outliers. A ring depth alone is not
// enough: the producer is paced by the audio clock, so the ring never fills
// on its own (1:1 producer/consumer); the consumer must ask for an OLD block.
//
// Two SPSC rings, each kChainPipeSlots deep, float L/R blocks. Core 1 is the
// sole producer of A-ring / consumer of B-ring; core 0 is the mirror.
// Volatile indices + __dmb() around slot data are enough: the RP2350 M33 is
// single-issue in-order, and each slot has one writer/one reader at a time.
// `g_chain_pipe_run` gates the whole path: transitions (mute/ramp/copy/init)
// run the SYNC render on core 1 with the worker paused, so no module state is
// touched across cores mid-transition.
static const uint32_t kChainPipeSlots = 4;
static const uint32_t kChainPipeLag = 2;   // blocks of A->B lag = 1.5 ms
struct ChainPipeBlock {
  uint32_t seq;              // block index (core 1's g_chain_pipe_seq)
  float l[kChainBlockSize];
  float r[kChainBlockSize];
};
static ChainPipeBlock g_chain_a_ring[kChainPipeSlots];  // core 1 -> core 0
static ChainPipeBlock g_chain_b_ring[kChainPipeSlots];  // core 0 -> core 1
static volatile uint32_t g_chain_a_head = 0;   // written by core 1
static volatile uint32_t g_chain_a_tail = 0;   // written by core 0
static volatile uint32_t g_chain_b_head = 0;   // written by core 0
static volatile uint32_t g_chain_b_tail = 0;   // written by core 1
static volatile uint32_t g_chain_pipe_seq = 0; // core 1's next block index
static volatile bool g_chain_pipe_run = false;
static volatile bool g_chain_b_worker_busy = false;
static volatile uint32_t g_chain_pipe_underruns = 0;  // B not ready in time
static volatile uint32_t g_chain_b_us_max = 0;         // core-0 B cost (diag)
static volatile uint32_t g_core0_loop_max_us = 0;      // core-0 loop period (diag)

// --- USB MIDI device (core 0) ---
Adafruit_USBD_MIDI usb_midi;

#if RACK_ENABLE_USB_HOST
// --- USB MIDI host (PIO-USB on rhport 1, core 0) ---
// Pattern from the bundled DualRole/serial_host_bridge example (device =
// native rhport 0, host = PIO-USB rhport 1). The host task + MIDI poll run
// in core 0's free-running loop(); core 1 renders audio.
Adafruit_USBH_Host USBHost;
#endif

// ===================== T4: DIN MIDI in (UART1 @ 31250) =====================
// Wiring (the standard MIDI-IN opto circuit — full table in README.md):
//   DIN jack pin 4 -> 220R -> 6N138 pin 2 (anode); 6N138 pin 3 (cathode) ->
//   220R -> jack pin 5; 6N138 pin 8 (Vcc) -> 3V3; pin 6 (open-collector out)
//   -> GP5 with a 10k pull-up to 3V3; pin 5 (emitter) -> GND; pin 7 n.c.
// Pins: GP5 = UART1 RX (valid per the core's UART1 pin table; GP7/GP9 are
// the alternates and GP7 is taken by I2S), GP4 = UART1 TX RESERVED (DIN OUT
// is NOT in T4 scope; the pin is muxed so it is defined, nothing wired).
// Collision check: I2S 6/7/8, PIO-USB host 14/15, LED 22, native USB (dedicated
// pads) — GP4/GP5 touch none of them and no PIO (hardware UART), so the init
// position is unconstrained by the PIO-USB/I2S order law.
//
// RX path (chosen): the core's SerialUART class (`Serial2` = HW UART1) in its
// default IRQ mode — its UART1 IRQ drains the 32-byte HW FIFO into a
// 256-entry lockless SW ring (setFIFOSize(256)) with break/framing filtering
// and the 32-bit-time idle timeout, and loop() drains the ring into the
// parser. Chosen over loop-side HW-FIFO polling for burst robustness: the
// ring absorbs ~82 ms of max-rate traffic (3125 B/s), far beyond any loop()
// stall (swap prints, LED poll ~1 ms); chosen over a hand-rolled IRQ because
// this IS the brief's "IRQ + ring if the core's API makes it clean" — the
// core's handler is proven code. Overflow of the ring is reported by
// Serial2.overflow() and folded into din_parser.dropped.
static const uint8_t kPinDinRx = 5;   // UART1 RX (6N138 collector, pulled up)
static const uint8_t kPinDinTx = 4;   // UART1 TX — reserved (no DIN OUT in T4)
static DinMidiParser din_parser;      // counters + state (core 0 only)

static void din_uart_init() {
  // ORDER (all three before begin): FIFO size and pinout only apply when
  // _running == false. The pinout MUST be overridden: the rpipico2 variant
  // defaults Serial2 to GP8/GP9 — GP8 is the I2S DOUT pin.
  Serial2.setFIFOSize(256);           // SW ring: 256 B = 82 ms @ 3125 B/s
  Serial2.setPinout(kPinDinTx, kPinDinRx);
  Serial2.begin(31250, SERIAL_8N1);   // MIDI 1.0: 31250 baud 8N1 (divisor is
                                      // exact at every core clk_peri: 125/150/
                                      // 240 MHz / (16 * 31250) = 250/300/480)
  gpio_pull_up(kPinDinRx);            // idle = mark (opto off, collector high);
                                      // belt-and-braces vs the external 10k
}

// Drain UART1 -> parser -> midiHandleEventPacket (THE funnel — same entry the
// USB device + PIO-USB host paths use; DIN F8/FA/FB/FC therefore hit the mod
// engine's sample-stamped clock bus with zero extra code).
// HARD CAP of 64 bytes per pass: back-to-back passes drain far faster than
// 3125 B/s, while a break/framing storm can never monopolize core 0.
static void din_pump() {
  if (Serial2.overflow()) ++din_parser.dropped;   // SW ring overflow (IRQ side)
  int c;
  while ((c = Serial2.read()) >= 0) {
    uint8_t pkt[4];
    if (din_parser.parse(static_cast<uint8_t>(c), pkt)) {
      midiHandleEventPacket(pkt);
    }
  }
}

#if RACK_ENABLE_USB_HOST
// Mounted host MIDI interface indexes, tracked by tuh_midi_mount_cb /
// tuh_midi_umount_cb. Host MIDI API is INDEX-based.
static constexpr uint8_t kHostMidiMaxIdxs = 4;
static uint8_t host_midi_idxs[kHostMidiMaxIdxs];
static uint8_t host_midi_idx_count = 0;
static volatile bool host_device_mounted = false;

extern "C" {
// Device-level callbacks (usbh.c weak stubs): fire when the full enumeration
// of ANY device on the host port completes / when it is removed.
void tuh_mount_cb(uint8_t daddr) {
  host_device_mounted = true;
  dbg_printf("USB device mounted daddr=%u\r\n", daddr);
}

void tuh_umount_cb(uint8_t daddr) {
  host_device_mounted = false;
  dbg_printf("USB device unmounted daddr=%u\r\n", daddr);
}

// Invoked when a device with a MIDI streaming interface is mounted.
void tuh_midi_mount_cb(uint8_t idx, const tuh_midi_mount_cb_t *mount_cb_data) {
  bool known = false;
  for (uint8_t i = 0; i < host_midi_idx_count; ++i) {
    if (host_midi_idxs[i] == idx) { known = true; break; }
  }
  if (!known && host_midi_idx_count < kHostMidiMaxIdxs) {
    host_midi_idxs[host_midi_idx_count++] = idx;
    statusLed.NotifyControllerConnected();  // 3 green blinks
  }
  dbg_printf("Host MIDI mounted idx=%u daddr=%u cables=%u\r\n", idx,
                mount_cb_data->daddr, mount_cb_data->rx_cable_count);
}

// Invoked when a MIDI interface is unmounted (unplugged / bus reset).
void tuh_midi_umount_cb(uint8_t idx) {
  for (uint8_t i = 0; i < host_midi_idx_count; ++i) {
    if (host_midi_idxs[i] == idx) {
      host_midi_idxs[i] = host_midi_idxs[--host_midi_idx_count];
      break;
    }
  }
  dbg_printf("Host MIDI unmounted idx=%u\r\n", idx);
}
} // extern "C"

// Drain pending 4-byte USB-MIDI event packets from every mounted host MIDI
// interface -> midiHandleEventPacket() (same entry as the device side).
static void pollHostMidi() {
  for (uint8_t i = 0; i < host_midi_idx_count; ++i) {
    const uint8_t idx = host_midi_idxs[i];
    if (!tuh_midi_mounted(idx)) continue;
    uint8_t pkt[4];
    while (tuh_midi_packet_read(idx, pkt)) {
      midiHandleEventPacket(pkt);
    }
  }
}
#endif

// --- registry dispatchers (core 0; called from midi.h) ---
// During a swap window the adapter hooks are dropped (g_swapping): the old
// module's state pointer aliases the new module's arena placement while the
// swap runs. The note stack / gate / bend still update in midi.h, so pitch
// state survives; only the adapter-specific trigger/param side effects are
// skipped for the 30-120 ms the swap takes.
void rack_module_note_on(uint8_t note, uint8_t vel) {
  if (g_swapping) return;
  kRackModules[g_active_module].note_on(note, vel);
}

void rack_module_note_off(uint8_t note) {
  if (g_swapping) return;
  kRackModules[g_active_module].note_off(note);
}

// T8b: the lira8 ARP-pitch hooks midi.h uses to keep pad notes out of the
// ARP's MIDI-follow hijack (see midiNoteOn / midiNoteOff).
bool rack_lira8_arp_pitch_on() {
  return g_active_module == RACK_MODULE_LIRA8 && !g_swapping &&
         rack_modules::mod_lira8::arp_pitch_active();
}
bool rack_lira8_sensor_note(uint8_t note) {
  return rack_modules::mod_lira8::is_sensor_note(note);
}

// Rings VARG chord mode: MIDI notes must still fire the string-synth strum
// instead of being swallowed by the ARP root-transpose hijack.
bool rack_rings_varg_chord_on() {
  return g_active_module == RACK_MODULE_RINGS && !g_swapping &&
         rack_modules::mod_rings::varg_chord_hook_on();
}

void rack_module_control_change(uint8_t cc, uint8_t val) {
  if (g_swapping) return;
  // T6b chained-A budget clamp: rings->clouds shares the 750 us pair budget
  // (rings + route + clouds-B + mod), and the rings combinations the
  // standalone sweep measured over the pair's headroom (the sympathetic-
  // string models, 511 us alone) are rejected loudly here. See
  // module_rings.h chain_rejects_cc for the sweep basis. Single-module
  // rings is fully unclamped; elements A needs no clamp (193-207 us).
  if (g_chain.active && g_chain.a == RACK_MODULE_RINGS &&
      rack_modules::mod_rings::chain_rejects_cc(cc, val)) {
    return;
  }
  kRackModules[g_active_module].control_change(cc, val);
}

// --- T1 hardware PLAY button (plan: 2026-11-14-rack-button-piezo.md) ---
// Momentary gate+strike, INDEPENDENT of MIDI notes / ARP / mapped CCs: press
// fires the ACTIVE adapter's trigger pseudo-CC (elements/plaits/rings 64,
// clouds 66) and holds its gate while held; release sends val 0.
//
// Dispatch deliberately goes through the REGISTRY path (below every funnel
// intercept), NOT midiControlChange: while chained, midiControlChange would
// hand CC 66 to the clouds-B intercept (rack_chain_wants_cc) — a physical
// play button must always hit the ACTIVE (A-slot) adapter, whose audio feeds
// the chain anyway. g_swapping drops events exactly like MIDI CC dispatch
// (30-120 ms swap window).
// Clouds takes a dedicated twin (mod_clouds::button_trig): CC 66 stays a
// pure trig for mod routing, but an idle clouds buffer is silent — the
// button additionally fires the internal exciter burst (what a MIDI note-on
// does) so a press is audible from idle.
// Plaits takes a dedicated twin too (mod_plaits::button_trig): its CC 64 is
// the TRIG-patched MODE switch, not a play control — the plain registry
// dispatch made hold decay to silence and release snap the drone back (the
// bench "reversed" report). The twin does a one-pluck window with
// auto-release instead.
// Elements/rings generate sound from their pseudo-CC alone and keep the
// plain registry dispatch.
// vel is the T3 piezo slot (adapter paths take no velocity yet).
static volatile uint32_t button_down_count = 0;   // cumulative, 2 s report

void rack_button_trig(bool down, uint8_t vel = 127) {  // vel: T3 piezo slot
  (void)vel;
  if (g_swapping) return;
  const uint8_t id = g_active_module;             // core-0 only; single read
  if (id == RACK_MODULE_CLOUDS) {
    rack_modules::mod_clouds::button_trig(down);
    return;
  }
  if (id == RACK_MODULE_PLAITS) {
    rack_modules::mod_plaits::button_trig(down);
    return;
  }
  rack_module_control_change(64, down ? 127 : 0);
}

// 30 ms debounce poll (core-0 loop). Standard commit-when-stable form: a raw
// edge is only trusted after it has held for 30 ms; the committed edge sends
// one trig + one report line. gpio_get on an INPUT_PULLUP pad: released = 1.
static bool button_stable = true;       // debounced level, true = released
static bool button_last_raw = true;
static uint32_t button_edge_ms = 0;

static void button_poll() {
  const bool raw = gpio_get(kPinButton);
  const uint32_t now = millis();
  if (raw != button_last_raw) {
    button_last_raw = raw;
    button_edge_ms = now;
  } else if (raw != button_stable && now - button_edge_ms >= 30) {
    button_stable = raw;
    if (!raw) ++button_down_count;
    rack_button_trig(!raw);             // raw low = pressed = down
    // No report line: button presses are not diagnostics anyone reads (user,
    // 2026-11-16) — and every print on this path is a hang risk.
  }
}

// CC 32 / console entry: queue a module switch. Values > 3 and == the active
// module are ignored (no self-swap; the swap print is the ack).
// T6 chain semantics (decision 5): while chained, CC 32 targets the CHAIN
// SLOTS, not the single-module switch —
//   v == A slot: fresh re-Init of A (no I2S change, no window copy; the
//                routing re-patches automatically),
//   v == B slot: rejected (clouds is the only B in T6; mode/quality go
//                through CC 23/24),
//   otherwise:   rejected with the window math (a chained swap target must
//                have its hot image fit the A slot below the B slot).
static void cmd_chain(const char* args);

void rack_request_switch(uint8_t module_id) {
  if (module_id >= RACK_MODULE_COUNT) return;
  preset_mark_dirty();           // T4a: active module is persistent state
  if (g_chain.active) {
    if (module_id == g_chain.a) {
      g_pending_chain_a_reinit = true;
      dbg_printf("rack: chain: %s (A slot) fresh re-Init queued\r\n",
                    kRackModules[module_id].name);
      return;
    }
    if (module_id == g_chain.b) {
      dbg_printf("rack: chain: B swap rejected (clouds is the only B in "
                     "T6; mode/quality via CC 23/24)\r\n");
      return;
    }
    // Chained module switch requested (e.g. from UI overlay or CC 32):
    // Switch the chain pair to <module_id> -> Clouds!
    char args[16];
    snprintf(args, sizeof(args), "%u 1", module_id);
    cmd_chain(args);
    return;
  }
  if (module_id == g_active_module) return;
  g_pending_module = static_cast<int8_t>(module_id);
}

// Clouds grain alias bank — single-module leg (declared in midi.h next to
// kCloudsAliasCcBase): true when CC 40-48 should take the registry path as
// clouds' native 14-22. While chained the band belongs to the B slot instead
// (rack_chain_wants_cc below grabs it there), and other single-module
// actives leave it untouched — it is unused in their maps.
bool rack_clouds_alias_active() {
  return !g_chain.active && g_active_module == RACK_MODULE_CLOUDS;
}

// T6 chain CC intercept (declared in midi.h): while chained, the clouds
// alias bank CC 40-50 (40-48 grain params -> clouds 14-22, 49 mode -> clouds 23,
// 50 quality -> clouds 24) AND CCs 65/66 (freeze/trig) drive the B slot
// (clouds) — everything else reaches the A slot through the normal registry
// dispatch.
// In particular, CC 23 and CC 24 belong entirely to the active chain-A module
// (plaits engine/aux, elements geometry/brightness, rings chord-tb/chord).
bool rack_chain_wants_cc(uint8_t cc) {
  return g_chain.active &&
         ((cc >= kCloudsAliasCcBase && cc <= kCloudsAliasCcBase + 10) ||
          cc == 65 || cc == 66);
}

void rack_chain_control_change(uint8_t cc, uint8_t val) {
  // Swap-window discipline (same as the A-slot dispatch in
  // rack_module_control_change): during a chain transition the B slot's
  // state is being placement-new'd / re-Init'ed — drop instead of racing it.
  if (g_swapping) return;
  // Grain alias bank 40-48 -> clouds' native 14-22 (see midi.h
  // kCloudsAliasCcBase). The mode clamp below does not apply here: 14-22 are
  // plain float parameters (mod_clouds::control_change ignores anything its
  // map does not define).
  if (cc >= kCloudsAliasCcBase && cc <= kCloudsAliasCcBase + 8) {
    rack_modules::mod_clouds::control_change(
        static_cast<uint8_t>(cc - kCloudsAliasCcBase + 14), val);
    return;
  }
  if (cc == kCloudsAliasCcBase + 9) { // 49 -> clouds-B MODE (clouds 23)
    // clouds-B playback-mode CLAMP (chain mode = granular/looping ONLY):
    // clouds' mode enum is 0 granular / 1 stretch / 2 looping / 3
    // spectral. 1 (stretch) and 3 (spectral) are REJECTED while chained —
    // stretch stays available in single-module mode, spectral is excluded
    // everywhere (T2). The adapter's apply-time clamp is belt-and-braces.
    if (val == 0 || val == 2) {
      rack_modules::mod_clouds::control_change(23, val);
    } else {
      dbg_printf("rack: chain: clouds-B mode %u rejected while chained "
                    "(granular 0 / looping 2 only)\r\n", val);
    }
    return;
  }
  if (cc == kCloudsAliasCcBase + 10) { // 50 -> clouds-B QUALITY (clouds 24)
    if (val <= 3) {
      if ((val & 1) != 0) {
        // MONO QUALITY IS CHAIN-FORBIDDEN (adapter clamp is belt-and-braces):
        // bench-proven audio-core death for the plaits/rings pairs.
        dbg_printf("rack: chain: clouds-B quality %u rejected while chained "
                   "(stereo 0/2 only; mono works single-module)\r\n", val);
        return;
      }
      rack_modules::mod_clouds::control_change(24, val);
    }
    return;
  }
  switch (cc) {
    case 65: // FREEZE
    case 66: // TRIG
      rack_modules::mod_clouds::control_change(cc, val);
      break;
    default:
      break;
  }
}

// ================= core 1: audio + live swap =================
#if RACK_ENABLE_USB_HOST
// Alarm pool created on CORE 1 (see elements-rp2350/src/main.cpp: the
// PIO-USB SOF alarm ISR must run on core 1, isolated from core 0's I2S DMA
// and TinyUSB device interrupts). Core 0's host init waits for the pool
// flag, then passes it via pio_cfg.alarm_pool.
static alarm_pool_t* g_host_alarm_pool = nullptr;
static volatile bool g_host_alarm_pool_ready = false;
#endif

void setup1() {
#if RACK_ENABLE_USB_HOST
  // Must run BEFORE the audio_ready wait: core 0's host init (in setup(),
  // before i2s.begin) consumes this pool.
  g_host_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(1);
  g_host_alarm_pool_ready = true;
#endif

  while (!audio_ready) { delay(1); }
  // T8: take the I2S DMA IRQ on the AUDIO core. The AudioBufferManager's
  // list helpers use noInterrupts(), which is per-core: with the writer on
  // core 1 and the DMA IRQ on core 0 they were racing on _empty/_filled, and
  // the lists eventually corrupted (persistent i2s stall storms that only a
  // reload/reconfigure cleared). core 0 disables DMA_IRQ_0 right after
  // i2s.begin(); core 1 enables it here and around every runtime reconfig.
  irq_set_enabled(DMA_IRQ_0, true);
}

// Swap-mute ramp depth: kSwapRampSteps render blocks with a linear gain ramp
// on each side of the swap (64-128 ramp-scaled samples at 16-32-sample
// blocks — well under one 16 ms DMA bank).
static const uint32_t kSwapRampSteps = 4;

// THE LIVE SWAP — core 1 only, at a block boundary (called from loop1's top,
// between two render calls). Sequence (plan "Swap semantics"):
//   (a) MUTE  — linear gain ramp-down over kSwapRampSteps blocks through the
//       OLD module's write loop (click-free), then flush the whole I2S DMA
//       bank with exact silence; the AudioBufferManager additionally loops a
//       dedicated silence buffer on underflow, so the DAC stays silent while
//       the copy runs (~the bank depth, 4-16 ms).
//   (b) COPY  — rack_window_copy(new): flash LMA -> window VMA memcpy
//       (~30-90 ms for the re-tiered ~109 KB plaits image; plain memcpy is
//       correct — the RP2350 M33 has no I/D caches, T1 finding). Checksummed
//       on both sides; on mismatch the OLD module's image is restored from
//       its own LMA and the swap aborts with the old module still live (the
//       failed copy overwrote the window, so restore-or-die is mandatory).
//   (c) INIT  — the new module placement-news its state and carves its
//       buffers from the T8 unified pool (a bump allocator reset per swap;
//       the old module's state is simply abandoned; factory defaults via the
//       adapter's Init()).
//   (d) I2S   — end() -> setBitsPerSample/setBuffers/setFrequency (per-module
//       native rate + bank) -> begin(); performed by CORE 0 via handshake
//       (see g_swap_i2s_request below) and the result is reported (the DAC is
//       muted regardless — XSMT held, silence bank looping).
//   (e) UNMUTE — linear gain ramp-up over kSwapRampSteps blocks on the NEW
//       module, then normal 1.0 rendering.
// One stats line is handed to core 0 (printed once, from loop()).

// I2S reconfig handshake (core 1 requests, core 0 performs): T8 moved the
// DMA transmit IRQ to the AUDIO core (core 1) — the AudioBufferManager's
// noInterrupts() is per-core, so a core-0 ISR raced core-1's _empty/_filled
// list writes and eventually corrupted them. The I2S OBJECT is still
// configured on core 0 (the old bench hard fault came from tearing the
// manager down while a core-0 ISR was live; core 1 now masks DMA_IRQ_0 across
// the whole handshake and re-enables it afterward). Core 1: id -> request ->
// wait (bounded); core 0: end/config/begin -> result -> ack. Ordering:
// result is written before ack; request is cleared before ack. No other
// concurrent I2S access exists during the swap window.
static volatile bool g_swap_i2s_request = false;
static volatile bool g_swap_i2s_ack = false;
static volatile bool g_swap_i2s_result = false;
static volatile uint8_t g_swap_i2s_id = 0;

// Core 0 side of the handshake — called once per loop() iteration.
static void serviceSwapI2s() {
  if (!g_swap_i2s_request) return;
  const RackModuleInfo& m = kRackModules[g_swap_i2s_id];
  i2s.end();
  const bool ok = i2s.setBitsPerSample(16) &&
                  i2s.setBuffers(m.i2s_buffers, m.i2s_words) &&
                  i2s.setFrequency(m.sample_rate) &&
                  i2s.begin();
  // T8: keep the DMA IRQ on the audio core (begin() enabled it here).
  irq_set_enabled(DMA_IRQ_0, false);
  g_swap_i2s_result = ok;
  g_swap_i2s_request = false;
  g_swap_i2s_ack = true;
}
static void do_module_swap(uint8_t new_id) {
  const RackModuleInfo& old = kRackModules[g_active_module];
  const RackModuleInfo& nw = kRackModules[new_id];
  const uint32_t t0 = time_us_32();
  crumb(60);   // swap start

  // (a) MUTE: ramp down through the old module's write loop ...
  g_render_gain = 1.0f;
  for (uint32_t i = 1; i <= kSwapRampSteps; ++i) {
    g_render_gain = 1.0f - static_cast<float>(i) / (kSwapRampSteps + 1);
    old.render();
  }
  g_render_gain = 0.0f;
  old.render();
  // ... then flush the entire DMA bank with silence (write16 blocks until a
  // buffer frees, so this paces at the output rate — bank-depth ms).
  for (uint32_t i = 0; i < old.i2s_buffers * old.i2s_words + 16; ++i) {
    i2s.write16(0, 0);
  }

  // (b) COPY: window <- flash image of the new module.
  uint32_t src_sum = 0, dst_sum = 0;
  const uint32_t copied = rack_window_copy(new_id, &src_sum, &dst_sum);
  if (src_sum != dst_sum) {
    // Corrupt copy: the window no longer holds the OLD module's image
    // either — restore it and keep running the old module.
    rack_window_copy(g_active_module, nullptr, nullptr);
    g_swap_duration_ms = (time_us_32() - t0) / 1000;
    g_swap_copied_bytes = copied;
    g_swap_to = new_id;
    g_swap_i2s_ok = false;
    g_swap_report_ready = true;
    g_swapping = false;
    return;
  }

  // (c) INIT: arena placement + factory-default Init.
  nw.init();
  // T4a: restore the panel's observed values over the factory defaults
  // (every CC any controller has sent for this module since boot).
  preset_replay_module(new_id);

  // (d) I2S reconfigure for the new module's native rate/bank — REQUESTED
  // from core 1, PERFORMED by core 0 (see the handshake note above). Core 1
  // waits, bounded; on timeout or failure the swap completes with audio
  // muted and the fault surfaces in the report line.
  g_swap_i2s_id = new_id;
  g_swap_i2s_ack = false;
  g_swap_i2s_result = false;
  irq_set_enabled(DMA_IRQ_0, false);   // T8: no core-1 IRQ during teardown
  g_swap_i2s_request = true;
  const uint32_t i2s_t0 = time_us_32();
  crumb(61);   // waiting for core-0 I2S ack
  while (!g_swap_i2s_ack && time_us_32() - i2s_t0 < 1000000UL) {
    delay(1);
  }
  crumb(62);   // I2S ack received / timed out
  irq_set_enabled(DMA_IRQ_0, true);    // T8: resumed on the audio core
  const bool i2s_ok = g_swap_i2s_ack && g_swap_i2s_result;

  g_active_module = new_id;
  proc_us_max = 0;              // the swap gap must not read as a Process max
  rack_meter_a = 0.0f;
  rack_meter_b = 0.0f;

  // (e) UNMUTE: ramp up on the NEW module — only if the I2S reconfig
  // succeeded (on failure the report line says "audio muted" and the rack
  // stays silent until the next swap).
  if (i2s_ok) {
    g_render_gain = 0.0f;
    for (uint32_t i = 1; i <= kSwapRampSteps; ++i) {
      g_render_gain = static_cast<float>(i) / (kSwapRampSteps + 1);
      nw.render();
    }
    g_render_gain = 1.0f;
  }

  // Stats for core 0's deferred print.
  g_swap_duration_ms = (time_us_32() - t0) / 1000;
  g_swap_copied_bytes = copied;
  g_swap_to = new_id;
  g_swap_i2s_ok = i2s_ok;
  g_swap_report_ready = true;
  g_swapping = false;
}

// ===================== T6: chain transitions =====================
// Chain I2S reconfig handshake — a PARALLEL copy of the swap handshake's
// discipline (same core-0-performs rule, same bounded core-1 wait; the swap
// handshake itself is untouched). Core 1 requests an explicit
// (rate, buffers, words) config; core 0 performs end/config/begin and acks.
static volatile bool g_chain_i2s_request = false;
static volatile bool g_chain_i2s_ack = false;
static volatile bool g_chain_i2s_result = false;
static volatile uint32_t g_chain_i2s_rate = kChainRate;
static volatile uint32_t g_chain_i2s_buffers = kChainI2sBuffers;
static volatile uint32_t g_chain_i2s_words = kChainI2sWords;

// Core 0 side — called once per loop() iteration, next to serviceSwapI2s.
static void serviceChainI2s() {
  if (!g_chain_i2s_request) return;
  i2s.end();
  const bool ok = i2s.setBitsPerSample(16) &&
                  i2s.setBuffers(g_chain_i2s_buffers, g_chain_i2s_words) &&
                  i2s.setFrequency(g_chain_i2s_rate) &&
                  i2s.begin();
  // T8: keep the DMA IRQ on the audio core (begin() enabled it here).
  irq_set_enabled(DMA_IRQ_0, false);
  g_chain_i2s_result = ok;
  g_chain_i2s_request = false;
  g_chain_i2s_ack = true;
}

// Sync A-stage render into chain_a_out_l/r (shared by the sync render, the
// pipelined render and chain_pipe_start's priming loop).
static void chain_render_a_block() {
  switch (g_chain.a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
    case RACK_MODULE_LIRA8:
      rack_modules::mod_lira8::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
    default:
      rack_modules::mod_rings::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
  }
}

// The SYNC chain render (core 1): A.Process(24) -> ROUTE -> B.Process(24) ->
// B out -> I2S, all in one call. Used by transitions (mute ramps, init,
// re-init) with the pipeline stopped, and as a fallback whenever
// g_chain_pipe_run is false.
static void chain_write_b();

static void chain_render_block_sync() {
  const uint32_t t0 = time_us_32();
  // A slot (the sound source; its hook applies pending slots + fills
  // perf/patch state exactly like its single-module render does).
  chain_render_a_block();
  // ROUTE + B (clouds — the effect stage; chain_render_b fills its
  // FloatFrame input from the routed buffers, Prepares, Processes, and
  // meters the pair).
  rack_modules::mod_clouds::chain_render_b(
      chain_a_out_l, chain_a_out_r, chain_b_out_l, chain_b_out_r,
      kChainBlockSize);
  const uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  chain_write_b();
}

// Write stage: the chain OUTPUT is B's output; g_render_gain (swap-mute
// ramps) applies here exactly as in the single-module adapters. The writes
// pace the loop: one iteration = one 24-frame block, blocks free every
// 750 us; > 1400 us = stalled.
static void chain_write_b() {
  uint32_t w0 = time_us_32();
  const float gain = g_render_gain;
  for (size_t i = 0; i < kChainBlockSize; ++i) {
    const int16_t l = stmlib::Clip16(
        static_cast<int32_t>(chain_b_out_l[i] * 32768.0f * gain));
    const int16_t r = stmlib::Clip16(
        static_cast<int32_t>(chain_b_out_r[i] * 32768.0f * gain));
    i2s.write16(l, r);
  }
  if (time_us_32() - w0 > kChainStallUs) {
    ++i2s_stall_count;
  }
}

// Core 1, pipelined: render A(N) into the A ring, then consume B(N-L) from
// the B ring and write it. A's burst no longer delays clouds, and the L-block
// lag means clouds has that many block periods to finish before its output is
// due — no wait in steady state. The bounded wait only covers an actual
// core-0 stall; if that times out the block is silenced and NOT consumed, so
// the stream resumes in order one block later (no skip, no repeat).
static void chain_render_block_pipe() {
  const uint32_t t0 = time_us_32();
  chain_render_a_block();
  const uint32_t seq = g_chain_pipe_seq;
  if (g_chain_a_head - g_chain_a_tail < kChainPipeSlots) {
    ChainPipeBlock* dst = &g_chain_a_ring[g_chain_a_head % kChainPipeSlots];
    dst->seq = seq;
    memcpy(dst->l, chain_a_out_l, sizeof(chain_a_out_l));
    memcpy(dst->r, chain_a_out_r, sizeof(chain_a_out_r));
    __dmb();
    ++g_chain_a_head;
  } else {
    ++g_chain_pipe_underruns;   // core 0 not consuming: drop this A block
  }

  // Consume the OLDEST ready B block once it is at least kChainPipeLag blocks
  // old — i.e. B(seq - lag), primed during chain_pipe_start. FIFO order means
  // the oldest is exactly the next block due, so no sequence search is needed.
  const uint32_t w0 = time_us_32();
  bool got = false;
  for (;;) {
    if (g_chain_b_head != g_chain_b_tail) {
      ChainPipeBlock* src = &g_chain_b_ring[g_chain_b_tail % kChainPipeSlots];
      __dmb();
      if ((uint32_t)(seq - src->seq) >= kChainPipeLag) {
        memcpy(chain_b_out_l, src->l, sizeof(chain_b_out_l));
        memcpy(chain_b_out_r, src->r, sizeof(chain_b_out_r));
        __dmb();
        ++g_chain_b_tail;
        got = true;
        break;
      }
    }
    if (time_us_32() - w0 >= 3000UL) break;
    tight_loop_contents();
  }
  if (!got) {
    ++g_chain_pipe_underruns;   // B not ready: one silent block, no hang
    memset(chain_b_out_l, 0, sizeof(chain_b_out_l));
    memset(chain_b_out_r, 0, sizeof(chain_b_out_r));
  }
  ++g_chain_pipe_seq;
  const uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;
  chain_write_b();
}

// Dispatcher: transitions run sync (pipeline stopped), normal play pipelined.
static void chain_render_block() {
  if (g_chain_pipe_run && g_chain.active) {
    chain_render_block_pipe();
  } else {
    chain_render_block_sync();
  }
}

// Core 0 worker: consume one A block, run clouds B, publish the B block.
// Called at the TOP of loop() before USB/UI/console work, so B is never
// behind the audio ring. The double-check on g_chain_pipe_run closes the
// stop-vs-start race with a transition on core 1.
static void chain_b_worker() {
  if (!g_chain_pipe_run) return;
  if (g_chain_a_head == g_chain_a_tail) return;
  if (g_chain_b_head - g_chain_b_tail >= kChainPipeSlots) return;

  g_chain_b_worker_busy = true;
  __dmb();
  if (!g_chain_pipe_run) {        // transition started while we armed
    g_chain_b_worker_busy = false;
    return;
  }
  const uint32_t t0 = time_us_32();
  ChainPipeBlock* a = &g_chain_a_ring[g_chain_a_tail % kChainPipeSlots];
  ChainPipeBlock* b = &g_chain_b_ring[g_chain_b_head % kChainPipeSlots];
  __dmb();
  rack_modules::mod_clouds::chain_render_b(
      a->l, a->r, b->l, b->r, kChainBlockSize);
  b->seq = a->seq;               // B inherits A's block index
  const uint32_t dt = time_us_32() - t0;
  if (dt > g_chain_b_us_max) g_chain_b_us_max = dt;
  __dmb();
  ++g_chain_a_tail;
  ++g_chain_b_head;
  g_chain_b_worker_busy = false;
}

// Transition helpers (core 1): pause the worker and wait until it is not
// inside clouds B before touching/replacing B state; start resets the rings
// and enables the pipeline.
static void chain_pipe_stop() {
  g_chain_pipe_run = false;
  __dmb();
  const uint32_t t0 = time_us_32();
  while (g_chain_b_worker_busy && time_us_32() - t0 < 5000UL) {
    tight_loop_contents();
  }
  __dmb();
}
static void chain_pipe_start() {
  chain_pipe_stop();
  g_chain_a_head = 0;
  g_chain_a_tail = 0;
  g_chain_b_head = 0;
  g_chain_b_tail = 0;
  // PRIME the B ring with kChainPipeLag sync-rendered blocks: the first
  // pipelined block (seq = lag) then finds B(0) already waiting instead of
  // waiting out a timeout for a block that cannot exist yet. Same module
  // calls as the sync render, so A/B state advance in lockstep with the audio
  // that the pipeline is about to emit (the lag is pure output latency).
  g_chain_pipe_seq = kChainPipeLag;
  for (uint32_t i = 0; i < kChainPipeLag; ++i) {
    chain_render_a_block();
    rack_modules::mod_clouds::chain_render_b(
        chain_a_out_l, chain_a_out_r, chain_b_out_l, chain_b_out_r,
        kChainBlockSize);
    ChainPipeBlock* b = &g_chain_b_ring[g_chain_b_head % kChainPipeSlots];
    b->seq = i;
    memcpy(b->l, chain_b_out_l, sizeof(chain_b_out_l));
    memcpy(b->r, chain_b_out_r, sizeof(chain_b_out_r));
    __dmb();
    ++g_chain_b_head;
  }
  g_chain_a_head = 0;
  g_chain_a_tail = 0;
  g_chain_b_tail = 0;
  __dmb();
  g_chain_pipe_run = true;
}

// Silence flush of the CURRENT DMA bank (same pacing argument as the swap's
// flush; used before each transition's I2S reconfigure).
static void chain_flush_silence(uint32_t buffers, uint32_t words) {
  for (uint32_t i = 0; i < buffers * words + 16; ++i) {
    i2s.write16(0, 0);
  }
}

// Mute ramp down through `render_once` (a callable doing exactly one
// block's render + write), then a silent block.
template <typename F>
static void chain_ramp_down(F render_once) {
  g_render_gain = 1.0f;
  for (uint32_t i = 1; i <= kSwapRampSteps; ++i) {
    g_render_gain = 1.0f - static_cast<float>(i) / (kSwapRampSteps + 1);
    render_once();
  }
  g_render_gain = 0.0f;
  render_once();
}

// Unmute ramp up through `render_once`.
template <typename F>
static void chain_ramp_up(F render_once) {
  g_render_gain = 0.0f;
  for (uint32_t i = 1; i <= kSwapRampSteps; ++i) {
    g_render_gain = static_cast<float>(i) / (kSwapRampSteps + 1);
    render_once();
  }
  g_render_gain = 1.0f;
}

static void chain_ramp_down_single() {
  const uint8_t id = g_active_module;
  chain_ramp_down([id]() { kRackModules[id].render(); });
}
static void chain_ramp_down_chain() {
  chain_ramp_down(chain_render_block);
}

// ENTER the chain: g_pending_chain == 1 consumed at a block boundary.
// Sequence = do_module_swap's discipline, minus the single-copy/plus the
// second slot: mute -> window copies (A + clouds, checksummed; on a corrupt
// copy restore the old image and stay single) -> I2S reconfigure to the
// chain grid bank (core 0 via the handshake) -> re-Init BOTH slots at the
// chain rate context -> unmute through the chain render.
static void do_chain_engage(uint8_t new_a, uint8_t new_b) {
  const uint8_t old_active = g_active_module;
  const uint8_t old_chain_a = g_chain.a;
  const bool already_chained = g_chain.active;
  const uint32_t t0 = time_us_32();

  // T8: pause the dual-core pipeline BEFORE the mute ramp; the ramp and the
  // whole transition run the sync render on core 1 with the B worker idle.
  chain_pipe_stop();

  // (a) MUTE through the current render + silence flush.
  if (already_chained) {
    // When already chained, g_chain.a is still old_chain_a (resident & initialized)!
    // chain_render_block() safely ramps down using old_chain_a.
    chain_ramp_down_chain();
    chain_flush_silence(kChainI2sBuffers, kChainI2sWords);
  } else {
    // Single-module mode: ramp down using the active single module.
    chain_ramp_down_single();
    chain_flush_silence(kRackModules[old_active].i2s_buffers,
                        kRackModules[old_active].i2s_words);
  }

  // (b) COPY both slot images: new_a into A slot, new_b into B slot.
  uint32_t src_sum = 0, dst_sum = 0;
  const uint32_t copied_a = rack_window_copy(new_a, &src_sum, &dst_sum);
  const bool a_ok = (src_sum == dst_sum);
  uint32_t copied_b = 0;
  bool b_ok = false;
  if (a_ok) {
    src_sum = dst_sum = 0;
    copied_b = rack_window_copy(new_b, &src_sum, &dst_sum);
    b_ok = (src_sum == dst_sum);
  }
  if (!a_ok || !b_ok) {
    // Corrupt copy: restore the old module's image.
    if (already_chained) {
      rack_window_copy(old_chain_a, nullptr, nullptr);
      rack_window_copy(RACK_MODULE_CLOUDS, nullptr, nullptr);
    } else {
      rack_window_copy(old_active, nullptr, nullptr);
    }
    g_chain_report_ms = (time_us_32() - t0) / 1000;
    g_chain_report_bytes = copied_a + copied_b;
    g_chain_report_kind = 1;
    g_chain_report_ok = false;
    g_chain_report_ready = true;
    g_swapping = false;
    return;
  }

  // (c) I2S reconfigure to the chain grid/bank:
  // If already chained, I2S is ALREADY running at kChainRate (32k), kChainI2sBuffers (32), kChainI2sWords (16)!
  if (!already_chained) {
    g_chain_i2s_rate = kChainRate;
    g_chain_i2s_buffers = kChainI2sBuffers;
    g_chain_i2s_words = kChainI2sWords;
    g_chain_i2s_ack = false;
    g_chain_i2s_result = false;
    irq_set_enabled(DMA_IRQ_0, false);  // T8: no core-1 IRQ during teardown
    g_chain_i2s_request = true;
    const uint32_t i2s_t0 = time_us_32();
    while (!g_chain_i2s_ack && time_us_32() - i2s_t0 < 1000000UL) {
      delay(1);
    }
    irq_set_enabled(DMA_IRQ_0, true);   // T8: resumed on the audio core
    const bool i2s_ok = g_chain_i2s_ack && g_chain_i2s_result;
    if (!i2s_ok) {
      g_chain_report_ms = (time_us_32() - t0) / 1000;
      g_chain_report_bytes = copied_a + copied_b;
      g_chain_report_kind = 1;
      g_chain_report_ok = false;
      g_chain_report_ready = true;
      g_swapping = false;
      return;
    }
  }

  // (d) INIT both slots at the chain rate context. ONE pool reset for the
  // whole resident set; the bump allocator places A's state+buffers, then
  // clouds-B's state+buffers. lira8's chain-A context is 32 kHz, so its
  // delay carve is 128,016 B (not the standalone 176,416).
  rack_mem_reset();
  switch (new_a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::init_at(
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_elements::state_size(), 8)),
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_elements::buf_bytes(), 8)));
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::init_chain_a(
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_plaits::state_size(), 8)),
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_plaits::buf_bytes(), 8)));
      break;
    case RACK_MODULE_LIRA8:
      rack_modules::mod_lira8::init_chain_a(
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_lira8::state_size(), 8)),
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_lira8::chain_buf_bytes(), 8)));
      break;
    default:
      rack_modules::mod_rings::init_at(
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_rings::state_size(), 8)),
          static_cast<uint8_t*>(
              rack_mem_alloc(rack_modules::mod_rings::buf_bytes(), 8)));
      break;
  }
  {
    uint8_t* b_obj = static_cast<uint8_t*>(
        rack_mem_alloc(rack_modules::mod_clouds::state_size(), 8));
    uint8_t* b_buf = static_cast<uint8_t*>(
        rack_mem_alloc(kChainCloudsLargeBytes + kChainCloudsSmallBytes, 8));
    rack_modules::mod_clouds::init_chain_b(
        b_obj, b_buf, kChainCloudsLargeBytes,
        b_buf + kChainCloudsLargeBytes, kChainCloudsSmallBytes);
  }
  // T4a: replay both slots' observed CC values over the factory defaults.
  preset_replay_module(new_a);
  preset_replay_module(new_b);

  // (e) GO CHAIN: active module = the new A slot (notes/CC/mod destinations
  // bind there), the mod engine runs the chain grid.
  g_chain.active = true;
  g_chain.a = new_a;
  g_chain.b = new_b;
  g_active_module = new_a;
  g_mod_chain_grid = kChainBlockSize;
  g_mod_chain_rate_khz = (uint8_t)(kChainRate / 1000);
  proc_us_max = 0;              // the transition must not read as a max
  g_mod_engine.mod_us_max = 0;
  rack_meter_a = 0.0f;
  rack_meter_b = 0.0f;

  // (f) UNMUTE through the chain render.
  chain_ramp_up(chain_render_block);

  // (g) GO PIPELINED: rings reset, core 0 starts rendering B one block
  // behind core 1's A. The first pipelined block waits (bounded) for B.
  chain_pipe_start();

  // (g) Stats for core 0's deferred print.
  g_chain_report_ms = (time_us_32() - t0) / 1000;
  g_chain_report_bytes = copied_a + copied_b;
  g_chain_report_kind = 1;
  g_chain_report_ok = true;
  g_chain_report_ready = true;
  g_swapping = false;
}

// LEAVE the chain: g_pending_chain == 0. The A slot re-Inits at its NATIVE
// rate context (plaits would re-capture 48 kHz via init_at's
// SetSampleRate); clouds-B's state is simply abandoned (union arena).
static void do_chain_disengage() {
  const uint8_t a = g_chain.a;
  const uint32_t t0 = time_us_32();

  chain_pipe_stop();             // T8: sync ramp + reinit, worker paused

  // (a) MUTE through the chain render + silence flush (chain bank).
  chain_ramp_down_chain();
  chain_flush_silence(kChainI2sBuffers, kChainI2sWords);

  // (b) I2S reconfigure to the A slot's native rate/bank (registry).
  g_chain_i2s_rate = kRackModules[a].sample_rate;
  g_chain_i2s_buffers = kRackModules[a].i2s_buffers;
  g_chain_i2s_words = kRackModules[a].i2s_words;
  g_chain_i2s_ack = false;
  g_chain_i2s_result = false;
  irq_set_enabled(DMA_IRQ_0, false);   // T8: no core-1 IRQ during teardown
  g_chain_i2s_request = true;
  const uint32_t i2s_t0 = time_us_32();
  while (!g_chain_i2s_ack && time_us_32() - i2s_t0 < 1000000UL) {
    delay(1);
  }
  irq_set_enabled(DMA_IRQ_0, true);    // T8: resumed on the audio core
  const bool i2s_ok = g_chain_i2s_ack && g_chain_i2s_result;

  // (c) Re-Init the A slot fresh at its native context. Its hot image is
  // still resident at the A-slot VMA (the engage-time B copy went to the
  // B slot); re-copy defensively anyway — idempotent, covers any future A
  // whose image spans the B-slot region.
  rack_window_copy(a, nullptr, nullptr);
  // Re-Init the A slot fresh at its NATIVE rate context through the registry
  // (the adapter's init() resets the pool and allocates its state+buffers;
  // plaits returns to 48 kHz, lira8 to 44.1 kHz). This also abandons
  // clouds-B's state/buffers — the pool union's contract.
  kRackModules[a].init();
  preset_replay_module(a);       // T4a: restore observed CC values on A

  // (d) CHAIN OFF: the mod engine drops the grid override (g_active_module
  // stays the A slot).
  g_chain.active = false;
  g_mod_chain_grid = 0;
  g_mod_chain_rate_khz = 0;
  proc_us_max = 0;
  g_mod_engine.mod_us_max = 0;
  rack_meter_a = 0.0f;
  rack_meter_b = 0.0f;

  // (e) UNMUTE single-module render — only if the I2S reconfig succeeded
  // (on failure the report says so and the rack stays muted).
  if (i2s_ok) {
    const uint8_t id = a;
    chain_ramp_up([id]() { kRackModules[id].render(); });
  }

  // (f) Stats for core 0's deferred print.
  g_chain_report_ms = (time_us_32() - t0) / 1000;
  g_chain_report_bytes = 0;
  g_chain_report_kind = 0;
  g_chain_report_ok = i2s_ok;
  g_chain_report_ready = true;
  g_swapping = false;
}

// A-slot fresh re-Init while chained (CC 32 == the A slot's id / decision 5):
// NO window copy, NO I2S change (the chain grid rate is unchanged even for a
// plaits/lira8 A), just factory-fresh A state; the routing re-patches
// automatically (persistent state). T8: clouds-B must SURVIVE this — the
// deterministic engage layout (A state @pool+0, A buffer @pool+align8(state),
// B state, B buffers) lets us re-init A in place without resetting the pool;
// a rack_mem_reset() here would invalidate B's live state/buffers.
static void do_chain_a_reinit() {
  const uint8_t a = g_chain.a;
  const uint32_t t0 = time_us_32();
  chain_pipe_stop();             // T8: sync ramp + A init, worker paused
  chain_ramp_down_chain();
  const size_t a_state = (a == RACK_MODULE_ELEMENTS)
      ? rack_modules::mod_elements::state_size()
      : (a == RACK_MODULE_PLAITS)
          ? rack_modules::mod_plaits::state_size()
          : (a == RACK_MODULE_LIRA8)
              ? rack_modules::mod_lira8::state_size()
              : rack_modules::mod_rings::state_size();
  uint8_t* a_obj = g_rack_pool;
  uint8_t* a_buf = g_rack_pool + ((a_state + 7u) & ~(size_t)7u);
  switch (a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::init_at(a_obj, a_buf);
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::init_chain_a(a_obj, a_buf);
      break;
    case RACK_MODULE_LIRA8:
      rack_modules::mod_lira8::init_chain_a(a_obj, a_buf);
      break;
    default:
      rack_modules::mod_rings::init_at(a_obj, a_buf);
      break;
  }
  proc_us_max = 0;
  rack_meter_a = 0.0f;
  rack_meter_b = 0.0f;
  preset_replay_module(a);       // T4a: restore observed CC values on A
  chain_ramp_up(chain_render_block);
  chain_pipe_start();            // T8: back to the dual-core pipeline
  g_chain_report_ms = (time_us_32() - t0) / 1000;
  g_chain_report_bytes = 0;
  g_chain_report_kind = 2;
  g_chain_report_ok = true;
  g_chain_report_ready = true;
  g_swapping = false;
}

void loop1() {
  // Pending swap first: consumed at the TOP of the render loop (strictly
  // between two Process() calls), max one swap per call. The T3 mod engine
  // shares this discipline: on a swap call it is SKIPPED exactly like the
  // render (return below) — the engine never touches module state while the
  // arena aliasing is in flight, and its own state is untouched by the swap.
  // While chained, core 0 never queues g_pending_module (rack_request_switch
  // routes CC 32 to the chain-slot semantics instead).
  const int8_t pending = g_pending_module;
  if (pending >= 0) {
    g_pending_module = -1;
    const uint8_t id = static_cast<uint8_t>(pending);
    if (id < RACK_MODULE_COUNT && id != g_active_module) {
      g_swapping = true;       // core 0 drops adapter MIDI during the swap
      do_module_swap(id);
    }
    return;                    // this call performed the swap, not a render
  }
  // T6 chain transitions: same block-boundary discipline, one per call.
  if (g_pending_chain_a_reinit) {
    g_pending_chain_a_reinit = false;
    g_swapping = true;
    do_chain_a_reinit();
    return;
  }
  const int8_t chain_req = g_pending_chain;
  if (chain_req >= 0) {
    g_pending_chain = -1;
    g_swapping = true;
    if (chain_req == 1) {
      const uint8_t req_a = static_cast<uint8_t>(g_pending_chain_a);
      const uint8_t req_b = static_cast<uint8_t>(g_pending_chain_b);
      g_pending_chain_a = -1;
      g_pending_chain_b = -1;
      do_chain_engage(req_a, req_b);
    } else {
      do_chain_disengage();
    }
    return;
  }
  // T3 mod engine: clock bus + sources + destination routing, BEFORE the
  // module's render, every block, at the active module's block size. Its
  // destinations resolve against the active module's registry entry, so a
  // swap re-targets the routing automatically (phase survives: g_mod_engine
  // is never written by the swap path). While chained, g_active_module is
  // the A slot and the g_mod_chain_* overrides run the 32 kHz / 24-sample
  // chain grid — destinations bind to A (decision 4).
  mod_engine_process_block(g_active_module);
  if (g_chain.active) {
    chain_render_block();
  } else {
    kRackModules[g_active_module].render();
  }
}

// ================= core 0: control =================
// --- serial console (T2 switch/module + T3 mod-engine + T4 route/telemetry/
// status/din/dintest/panic — one data-driven table; T5's C3 console reuses
// it) ---
struct ConsoleCommand {
  const char* name;
  void (*fn)(const char* args);   // args = everything after the name
};

static void cmd_switch(const char* args);
static void cmd_module(const char* args);
static void cmd_chain(const char* args);
static void cmd_route(const char* args);
static void cmd_status(const char* args);
static void cmd_din(const char* args);
static void cmd_dintest(const char* args);
static void cmd_telemetry(const char* args);
static void cmd_led(const char* args);
static void cmd_panic(const char* args);
// Bench: reboot the board without BOOTSEL/flash (same AIRCR path as the crash
// handler). Lets a crashed core-1 state be reset from the console.
static void cmd_reboot(const char* args);
static void cmd_ui(const char* args);
static void cmd_uitest(const char* args);
static void cmd_uisend(const char* args);
// Bench: inject a CC through the REAL funnel (T9 warps per-algorithm bench).
static void cmd_cc(const char* args);
// T9 clock diag: live QMI M0 flash timing decode.
static void cmd_qmi(const char* args);
// T9 bench: runtime overclock + crash record print/clear.
static void cmd_oc(const char* args);
static void cmd_vreg(const char* args);
static void cmd_crash(const char* args);
static void cmd_trig(const char* args);
static void cmd_piezo(const char* args);
static void cmd_help(const char* args);
// T3 mod-engine commands (defined in mod_engine.h, core 0 only).
//   mod <src> <param> <value> | mod? | bpm <20-300> | clock <internal|midi> [src]
extern void mod_engine_cmd_mod(const char* args);
extern void mod_engine_cmd_modq(const char* args);
extern void mod_engine_cmd_bpm(const char* args);
extern void mod_engine_cmd_clock(const char* args);

// T5 stage 1: UI-MCU link (ESP32-C3 <-> RP2350, 921600 framed UART). Included
// HERE — after the console forward declarations above (its SWITCH /
// CHAIN_ROUTE dispatches call cmd_switch / cmd_chain, one behavior, two
// entrances) and after the registry/adapters/chain state it dispatches
// through. Header-only, single-TU (the midi.h pattern).
#include "ui_link.h"

// T4a: presets (serialization + apply + autosave + console bench). Included
// AFTER ui_link.h (needs the CC cache/seen statics + TX) and AFTER the chain
// state / pending slots above. The swap/chain sequences earlier in this file
// call preset_replay_module through the forward decl at the top.
#include "preset.h"

static const ConsoleCommand kConsoleCommands[] = {
    {"switch",    cmd_switch},
    {"module",    cmd_module},
    {"chain",     cmd_chain},
    {"mod",       mod_engine_cmd_mod},
    {"mod?",      mod_engine_cmd_modq},
    {"route",     cmd_route},
    {"bpm",       mod_engine_cmd_bpm},
    {"clock",     mod_engine_cmd_clock},
    {"preset",    preset_cmd},
    {"status",    cmd_status},
    {"din",       cmd_din},
    {"dintest",   cmd_dintest},
    {"telemetry", cmd_telemetry},
    {"led",       cmd_led},
    {"ui",        cmd_ui},
    {"uitest",    cmd_uitest},
    {"uisend",    cmd_uisend},
    {"cc",        cmd_cc},
    {"qmi",       cmd_qmi},
    {"oc",        cmd_oc},
    {"vreg",      cmd_vreg},
    {"crash",     cmd_crash},
    {"panic",     cmd_panic},
    {"reboot",    cmd_reboot},
    {"trig",      cmd_trig},
    {"piezo",     cmd_piezo},
    {"help",      cmd_help},
};
static const uint8_t kConsoleCommandCount =
    sizeof(kConsoleCommands) / sizeof(kConsoleCommands[0]);

static void cmd_switch(const char* args) {
  while (*args == ' ') ++args;
  if (*args == '\0') {
    Serial.println("usage: switch <0-4|elements|clouds|plaits|rings|lira8>");
    return;
  }
  int8_t id = -1;
  if (args[0] >= '0' && args[0] <= '9') {
    long v = atol(args);
    if (v >= 0 && v < RACK_MODULE_COUNT) id = static_cast<int8_t>(v);
  } else {
    for (uint8_t i = 0; i < RACK_MODULE_COUNT; ++i) {
      if (strcasecmp(args, kRackModules[i].name) == 0) {
        id = static_cast<int8_t>(i);
        break;
      }
    }
  }
  if (id < 0) {
    Serial.println("rack: unknown module (0 elements / 1 clouds / 2 plaits / 3 rings / 4 lira8)");
    return;
  }
  if (g_chain.active) {
    // Chain semantics own the request (A-slot fresh re-Init / B reject /
    // window-math reject) — they must run BEFORE the single-module
    // self-swap guard: while chained the ACTIVE module IS the A slot, so
    // "switch <A>" means a fresh re-Init, not a no-op. (Bench finding
    // 2026-11-05: the guard swallowed the A re-Init via the console.)
    rack_request_switch(static_cast<uint8_t>(id));
    return;
  }
  if (static_cast<uint8_t>(id) == g_active_module) {
    Serial.printf("rack: %s already active\r\n", kRackModules[id].name);
    return;
  }
  g_pending_module = id;
  preset_mark_dirty();           // T4a: active module is persistent state
  Serial.printf("rack: switch to %s queued (swaps at the next block boundary)\r\n",
                kRackModules[id].name);
}

static void cmd_module(const char* args) {
  (void)args;
  const RackModuleInfo& m = kRackModules[g_active_module];
  Serial.printf("rack: module %s (id %u, %lu Hz, %lu-sample blocks, budget %lu us)\r\n",
                m.name, g_active_module,
                (unsigned long)m.sample_rate, (unsigned long)m.block_size,
                (unsigned long)m.budget_us);
  Serial.printf("rack: cc map: %s\r\n", m.cc_map);
  Serial.printf("rack: mod destinations: %s\r\n", m.mod_destinations->summary);
}

// chain — T6 bench console: `chain` (status) / `chain <a> [b]` (validate +
// queue engage; b defaults to clouds) / `chain off`.
//
// PAIR VALIDATION (the 4-pair table — all X->clouds):
//   elements->clouds, plaits->clouds, rings->clouds, lira8->clouds
// Engagement additionally requires the CHAIN RESIDENCY invariants (both
// slots' hot images must fit the two-slot SRAM window, and both slots' state +
// buffer demand must fit the ONE unified pool) — checked here with the REAL
// linker-symbol/module sizes, so a reject line carries the exact numbers.
// Since T6b (tc_clouds dead-mode demotion 79,744 -> 56,992 B paid for the
// 0xE400 A slot) AND the user-approved plaits chain-tier
// (09-plaits-chain-tier.patch: tc_plaits 108,732 -> ~14.1 KB, voice path +
// engine 0 + lut_sine only), all four pairs pass. T8's unified pool is what
// admits lira8: its 32 kHz delay (128,016 B) + clouds-B (131,072 B) need
// 283,872 B of 331,776 — the old split rejected it (259,088 > 196,608 buffer)
// while 110 KB of the object arena sat idle. The chained-plaits/lira8 BUDGET
// (32 kHz) is a separate gate — see the README bench sheet.
static bool chain_parse_module(const char* tok, int8_t* out) {
  if (tok[0] >= '0' && tok[0] <= '9') {
    long v = atol(tok);
    if (v >= 0 && v < RACK_MODULE_COUNT) { *out = static_cast<int8_t>(v); return true; }
    return false;
  }
  for (uint8_t i = 0; i < RACK_MODULE_COUNT; ++i) {
    if (strcasecmp(tok, kRackModules[i].name) == 0) {
      *out = static_cast<int8_t>(i);
      return true;
    }
  }
  return false;
}

static void cmd_chain(const char* args) {
  while (*args == ' ') ++args;
  if (*args == '\0') {
    // status
    if (g_chain.active) {
      Serial.printf("rack: chain ACTIVE %s->%s (grid %lu Hz / %u-sample blocks, "
                    "budget %lu us incl. mod; clamps: B mode granular/looping "
                    "only, %s A budget clamp %s; CC 23/24/65/66 + grain bank "
                    "40-48 -> B, others -> A)\r\n",
                    kRackModules[g_chain.a].name, kRackModules[g_chain.b].name,
                    (unsigned long)kChainRate, (unsigned)kChainBlockSize,
                    (unsigned long)kChainBudgetUs,
                    kRackModules[g_chain.a].name,
                    g_chain.a == RACK_MODULE_RINGS ? "ON" : "n/a");
    } else {
      Serial.println("rack: chain off (pairs: elements|plaits|rings -> clouds; "
                     "usage: chain <a> [b] / chain off)");
    }
    return;
  }
  if (strcasecmp(args, "off") == 0) {
    if (!g_chain.active) {
      Serial.println("rack: chain not engaged");
      return;
    }
    g_pending_chain = 0;
    preset_mark_dirty();         // T4a: chain state is persistent
    Serial.println("rack: chain off queued (mutes at the next block boundary)");
    return;
  }
  // chain <a> [b]
  char tok[24];
  if (sscanf(args, "%23s", tok) != 1) return;
  int8_t a = -1, b = -1;
  if (!chain_parse_module(tok, &a)) {
    Serial.println("rack: chain: unknown module (0 elements / 1 clouds / "
                   "2 plaits / 3 rings / 4 lira8)");
    return;
  }
  const char* rest = args + strlen(tok);
  while (*rest == ' ') ++rest;
  if (*rest) {
    if (sscanf(rest, "%23s", tok) != 1 || !chain_parse_module(tok, &b)) {
      Serial.println("rack: chain: unknown B module");
      return;
    }
  } else {
    b = static_cast<int8_t>(RACK_MODULE_CLOUDS);
  }
  // --- pair table + residency validation (all with real numbers) ---
  if (b != RACK_MODULE_CLOUDS) {
    Serial.println("rack: chain: B must be clouds (pair table: "
                   "elements|plaits|rings|lira8 -> clouds)");
    return;
  }
  if (a == RACK_MODULE_CLOUDS) {
    Serial.println("rack: chain: A cannot be clouds (no clouds->clouds pair)");
    return;
  }
  if (g_chain.active) {
    if (static_cast<uint8_t>(a) == g_chain.a) {
      Serial.println("rack: chain: that pair is already engaged");
      return;
    }
  }
  const RackWindowSection& sa = rack_window_section(static_cast<uint8_t>(a));
  const RackWindowSection& sb = rack_window_section(RACK_MODULE_CLOUDS);
  const RackWindowSection& s_origin =
      rack_window_section(RACK_MODULE_ELEMENTS);
  const uint32_t a_bytes = (uint32_t)(sa.window_end - sa.window_start);
  const uint32_t b_bytes = (uint32_t)(sb.window_end - sb.window_start);
  const uint32_t b_offset =
      (uint32_t)(sb.window_start - s_origin.window_start);  // A-slot capacity
  const uint32_t window_bytes = kRackWindowBytes;
  if (a_bytes > b_offset) {
    Serial.printf("rack: chain REJECTED: %s hot image %u B > A slot %u B "
                  "(below the clouds-B slot) — pair not resident-capable in "
                  "this build\r\n",
                  sa.name, (unsigned)a_bytes, (unsigned)b_offset);
    return;
  }
  if (b_offset + b_bytes > window_bytes) {
    Serial.printf("rack: chain REJECTED: B slot %u + %u B > window %u B\r\n",
                  (unsigned)b_offset, (unsigned)b_bytes, (unsigned)window_bytes);
    return;
  }
  const size_t a_state = (a == RACK_MODULE_ELEMENTS)
      ? rack_modules::mod_elements::state_size()
      : (a == RACK_MODULE_PLAITS)
          ? rack_modules::mod_plaits::state_size()
          : (a == RACK_MODULE_LIRA8)
              ? rack_modules::mod_lira8::state_size()
              : rack_modules::mod_rings::state_size();
  const size_t b_state = rack_modules::mod_clouds::state_size();
  const size_t a_buf = kChainABufferBytes[static_cast<uint8_t>(a)];
  const size_t b_buf = kChainCloudsLargeBytes + kChainCloudsSmallBytes;
  // One pool: allocator order = A state, A buffer, B state, B buffer; each
  // allocation is 8-aligned, so charge the padding after each state.
  const size_t pool_needed =
      ((a_state + 7u) & ~(size_t)7u) + a_buf +
      ((b_state + 7u) & ~(size_t)7u) + b_buf;
  if (pool_needed > kRackPoolBytes) {
    Serial.printf("rack: chain REJECTED: pool %u B needed (%s state %u + "
                  "buffer %u + clouds state %u + buffers %u) > %u B\r\n",
                  (unsigned)pool_needed, sa.name, (unsigned)a_state,
                  (unsigned)a_buf, (unsigned)b_state, (unsigned)b_buf,
                  (unsigned)kRackPoolBytes);
    return;
  }
  g_pending_chain_a = static_cast<int8_t>(a);
  g_pending_chain_b = static_cast<int8_t>(RACK_MODULE_CLOUDS);
  g_pending_chain = 1;
  preset_mark_dirty();           // T4a: chain state is persistent
  Serial.printf("rack: chain %s->clouds validated (window %u+%u of %u B, "
                "pool %u of %u B: %s state %u + buf %u, clouds state %u + "
                "buf %u) — engaging at the next block boundary\r\n",
                sa.name, (unsigned)a_bytes, (unsigned)b_bytes,
                (unsigned)window_bytes, (unsigned)pool_needed,
                (unsigned)kRackPoolBytes, sa.name, (unsigned)a_state,
                (unsigned)a_buf, (unsigned)b_state, (unsigned)b_buf);
}

// route <source> <dest> — pure alias of `mod <src> dest <dest>`: re-tokenize
// and forward, so the routing machinery/acks live in exactly one place
// (mod_engine.h's mod_engine_cmd_mod). dest = CC 0-127 | off.
static void cmd_route(const char* args) {
  while (*args == ' ') ++args;
  char src[24];
  size_t i = 0;
  const char* rest = args;
  while (*rest && *rest != ' ' && i + 1 < sizeof(src)) src[i++] = *rest++;
  src[i] = '\0';
  while (*rest == ' ') ++rest;
  if (!src[0] || !*rest) {
    Serial.println("usage: route <source> <dest>   (dest = CC 0-127 | off; "
                   "same as: mod <src> dest <dest>)");
    return;
  }
  char fwd[64];
  snprintf(fwd, sizeof(fwd), "%s dest %s", src, rest);
  mod_engine_cmd_mod(fwd);
}

// telemetry on|off — gate the 2 s report for quiet benching. Default ON.
// Counter resets stay unconditional (the report's max-since-report semantics
// are unchanged when re-enabled); swap/panic/watchdog lines are NOT gated.
static bool g_telemetry_enabled = true;

static void cmd_telemetry(const char* args) {
  while (*args == ' ') ++args;
  if (!strcasecmp(args, "on")) {
    g_telemetry_enabled = true;
    Serial.println("telemetry: 2 s report ON");
  } else if (!strcasecmp(args, "off")) {
    g_telemetry_enabled = false;
    Serial.println("telemetry: 2 s report OFF (swap/panic lines still print; "
                   "use 'status' for a one-liner)");
  } else {
    Serial.println("usage: telemetry on|off (default on)");
  }
}

// led on|off — the WS2812 is a status nicety, not an audio dependency.
// Off stops all showColor work on core 0 (T8 audio-first option).
static void cmd_led(const char* args) {
  while (*args == ' ') ++args;
  if (!strcasecmp(args, "on")) {
    statusLed.SetEnabled(true);
    Serial.println("led: on");
  } else if (!strcasecmp(args, "off")) {
    statusLed.SetEnabled(false);
    Serial.println("led: off (no core-0 LED work)");
  } else {
    Serial.println("usage: led on|off");
  }
}

// status — one-line bench summary (T4): active module, clock source/BPM,
// swap count, DIN counters, uptime. The DIN counters also live in the 2 s
// report's sibling commands (`din`); deliberately NOT in the report line.
static uint32_t g_swap_count = 0;   // ++ per consumed swap report (loop())

static void cmd_status(const char* args) {
  (void)args;
  const ModEngine& e = g_mod_engine;
  const uint32_t bpm10 = (uint32_t)(e.bpm * 10.0f + 0.5f);
  const uint32_t meas10 = (uint32_t)(e.midi_bpm_meas * 10.0f + 0.5f);
  char clock[40];
  if (e.global_clock == 0) {
    snprintf(clock, sizeof(clock), "internal@%lu.%lubpm",
             (unsigned long)(bpm10 / 10), (unsigned long)(bpm10 % 10));
  } else if (!e.midi_running) {
    snprintf(clock, sizeof(clock), "midi@STOPPED");
  } else if (e.midi_bpm_meas > 0.0f) {
    snprintf(clock, sizeof(clock), "midi@meas %lu.%lubpm",
             (unsigned long)(meas10 / 10), (unsigned long)(meas10 % 10));
  } else {
    snprintf(clock, sizeof(clock), "midi@measuring");
  }
  Serial.printf("status: module=%s clk=%s swaps=%lu din: bytes=%lu msgs=%lu "
                "rt=%lu drop=%lu uptime=%lu s\r\n",
                kRackModules[g_active_module].name, clock,
                (unsigned long)g_swap_count,
                (unsigned long)din_parser.rx_bytes,
                (unsigned long)din_parser.messages,
                (unsigned long)din_parser.realtime,
                (unsigned long)din_parser.dropped,
                (unsigned long)(millis() / 1000UL));
}

// din [clear] — DIN counters + last message (+ the RX-path line for bring-up).
static void cmd_din(const char* args) {
  while (*args == ' ') ++args;
  if (!strcasecmp(args, "clear")) {
    din_parser.rx_bytes = 0;
    din_parser.messages = 0;
    din_parser.realtime = 0;
    din_parser.dropped = 0;
    din_parser.have_last = false;
    Serial.println("din: counters cleared");
    return;
  }
  Serial.printf("din: rx_bytes=%lu msgs=%lu realtime=%lu dropped=%lu "
                "(sysex/aborted/orphan/ring overflow)\r\n",
                (unsigned long)din_parser.rx_bytes,
                (unsigned long)din_parser.messages,
                (unsigned long)din_parser.realtime,
                (unsigned long)din_parser.dropped);
  if (din_parser.have_last) {
    char desc[48];
    din_describe(din_parser.last_pkt, desc, sizeof(desc));
    Serial.printf("din: last [%02x %02x %02x %02x] %s\r\n",
                  din_parser.last_pkt[0], din_parser.last_pkt[1],
                  din_parser.last_pkt[2], din_parser.last_pkt[3], desc);
  } else {
    Serial.println("din: last: none");
  }
  Serial.printf("din: uart1 @31250 8n1, rx=GP%u (tx GP%u reserved, no DIN OUT "
                "in T4), direct poll 64 B/pass (no core IRQ), pull-up; "
                "'dintest' injects a test stream\r\n",
                kPinDinRx, kPinDinTx);
}

// dintest — inject the canned stream (kDinTestStream in din_midi.h, shared
// byte-for-byte with the host unit test) INTO THE PARSER, as if from the
// UART. Honest scope: the UART1 link is bypassed — this exercises
// parser -> midiHandleEventPacket -> module/clock-bus dispatch only, which
// is exactly the without-jack part of T4 and doubles as the wiring bring-up
// helper (with the jack wired, real bytes take the same path from din_pump).
// Side effects are REAL: notes sound on the active module, CCs apply, and
// the clock stamps reach the mod engine's bus.
static void cmd_dintest(const char* args) {
  (void)args;
  const uint32_t b0 = din_parser.rx_bytes, m0 = din_parser.messages;
  const uint32_t r0 = din_parser.realtime, d0 = din_parser.dropped;
  const uint32_t notes0 = note_on_count;
  Serial.println("dintest: injecting the canned stream into the DIN parser "
                 "(UART1 bypassed: parser+funnel only)");
  for (uint32_t i = 0; i < kDinTestStreamLen; ++i) {
    uint8_t pkt[4];
    const uint32_t drop_before = din_parser.dropped;
    if (din_parser.parse(kDinTestStream[i], pkt)) {
      char desc[48];
      din_describe(pkt, desc, sizeof(desc));
      Serial.printf("dintest: [%02x %02x %02x %02x] -> %s\r\n",
                    pkt[0], pkt[1], pkt[2], pkt[3], desc);
      // Dispatch through the REAL funnel — parser→funnel→module is the whole
      // point of dintest (without this the packets are only described).
      midiHandleEventPacket(pkt);
    } else if (din_parser.dropped != drop_before) {
      Serial.printf("dintest: byte %02x -> discarded (dropped total %lu)\r\n",
                    kDinTestStream[i], (unsigned long)din_parser.dropped);
    }
    // bytes that neither emit nor drop (sysex body) stay silent — honest
  }
  const uint32_t msgs = din_parser.messages - m0;
  const uint32_t rts = din_parser.realtime - r0;
  // FA + FC are transport, the rest of the realtime bytes are F8 ticks.
  const uint32_t ticks = (rts >= 2) ? (rts - 2) : 0;
  Serial.printf("dintest: %lu bytes -> %lu packets (%lu realtime), +%lu dropped | "
                "note_on_count +%lu\r\n",
                (unsigned long)(din_parser.rx_bytes - b0), (unsigned long)msgs,
                (unsigned long)rts, (unsigned long)(din_parser.dropped - d0),
                (unsigned long)(note_on_count - notes0));
  Serial.printf("dintest: clock stamps queued: FA + %lu F8 + FC (core 1 drains "
                "within one audio block; 'mod?' midi ticks should read +%lu)\r\n",
                (unsigned long)ticks, (unsigned long)ticks);
  Serial.printf("dintest: notes went to the ACTIVE module (%s) through the same "
                "midiHandleEventPacket funnel as USB device/host\r\n",
                kRackModules[g_active_module].name);
}

// panic — all notes off: release the held notes on the ACTIVE module's
// adapter (real note-offs), then clear the shared note stack and force the
// gate off. Same discipline as the stuck-gate watchdog, plus the adapter
// releases.
static void cmd_panic(const char* args) {
  (void)args;
  const uint8_t n = note_stack.size;
  uint8_t held[NoteStack::kMaxNotes];
  for (uint8_t i = 0; i < n; ++i) held[i] = note_stack.notes[i];
  for (uint8_t i = 0; i < n; ++i) rack_module_note_off(held[i]);
  note_stack.clear();
  shared_gate = false;
  Serial.printf("panic: %u note(s) released on %s, gate forced off, note stack "
                "cleared\r\n",
                (unsigned)n, kRackModules[g_active_module].name);
}

// Bench: reboot the board without BOOTSEL or a flash write (AIRCR SYSRESETREQ,
// the same path the HardFault handler uses). Recovers a dead core-1 state.
static void cmd_reboot(const char* args) {
  (void)args;
  Serial.println("reboot: system reset");
  delay(50);                        // let the CDC drain a few frames
  watchdog_hw->scratch[5] = 0xC1EA0FFu;   // CLEAN_SHUTDOWN: reason 5 next boot
  scb_hw->aircr = (0x05FAu << 16) | 0x04u;
  __asm volatile("dsb 0xF" ::: "memory");
  for (;;) {
    tight_loop_contents();
  }
}

// trig [down|up] — T1 bench twin of the hardware PLAY button (GP2): fires the
// ACTIVE adapter's trigger pseudo-CC via the same rack_button_trig path, so a
// headless bench (no wiring) exercises exactly the injection the button uses.
// Bare 'trig' = one press+release pulse (elements: 60 ms strike gate; the
// pulse-only modules just fire).
static void cmd_trig(const char* args) {
  while (*args == ' ') ++args;
  if (strcasecmp(args, "down") == 0) {
    rack_button_trig(true);
    Serial.printf("trig: down -> %s\r\n",
                  kRackModules[g_active_module].name);
    return;
  }
  if (strcasecmp(args, "up") == 0) {
    rack_button_trig(false);
    Serial.println("trig: up");
    return;
  }
  if (*args == '\0') {
    rack_button_trig(true);
    rack_button_trig(false);
    Serial.println("trig: pulse (down+up)");
    return;
  }
  Serial.println("usage: trig [down|up]  (bare = press+release pulse)");
}

// piezo [gain <1-16>] — T2 piezo bench read-out: the live ADC window
// (min/max/peak since the last 2 s report — tap and read), the measured
// sample rate and the digital gain. The gain scales the rectified samples
// before the T3 Ears pipeline (stand-in for Ears' omitted 40 dB amp).
static void cmd_piezo(const char* args) {
  while (*args == ' ') ++args;
  if (strncasecmp(args, "gain", 4) == 0) {
    const char* v = args + 4;
    while (*v == ' ') ++v;
    const float g = atof(v);
    if (g < 1.0f) {
      Serial.printf("piezo: gain %.2f (range 1-16)\r\n", rack_piezo::gain());
      return;
    }
    rack_piezo::set_gain(g);
    Serial.printf("piezo: gain set %.2f\r\n", rack_piezo::gain());
    return;
  }
  if (*args == '\0') {
    Serial.printf("piezo: rate=%lu Hz min=%u max=%u peak=%u gain=%.2f "
                  "(GP26/ADC0; tap and re-read)\r\n",
                  (unsigned long)rack_piezo::report_sample_rate(),
                  rack_piezo::report_min(), rack_piezo::report_max(),
                  rack_piezo::report_peak(), rack_piezo::gain());
    return;
  }
  Serial.println("usage: piezo [gain <1-16>]");
}

// ui — T5 stage 1 link status: frame counters, CRC errors, last RTT. The
// rtt field is the RP2350's ECHO TURNAROUND (UI_CMD echo received -> echo
// written); the C3 measures the full wire RTT of its own echo probes.
static void cmd_ui(const char* args) {
  (void)args;
  Serial.printf("ui: tx=%lu (dropped=%lu) rx=%lu crc_err=%lu rx_overflow=%lu "
                "rtt=%lu us (echo turnaround) | page=%u | uart0 @921600 8N1: "
                "TX GP%u (-> C3 RX), RX GP%u (<- C3 TX), irq + 512B ring\r\n",
                (unsigned long)g_ui_tx_frames,
                (unsigned long)g_ui_tx_dropped,
                (unsigned long)g_ui_rx_frames,
                (unsigned long)g_ui_crc_errors,
                (unsigned long)g_ui_rx_overflow,
                (unsigned long)g_ui_last_rtt_us,
                g_ui_active_page, kPinUiTx, kPinUiRx);
}

// uitest — codec self-test. Runs the IN-MEMORY encode->parse round-trip
// (including a corrupted-CRC rejection + resync check and a 0x7E-inside-
// payload case). NO hardware loopback leg: the RP2350's PL011-style UART has
// no LBM/loopback bit (checked the pico-sdk UART register headers) — wire
// verification is the C3's own hardware-loopback self-test (uart_set_loop_
// back) plus the cross-board echo bench.
static void cmd_uitest(const char* args) {
  (void)args;
  const uint32_t crc0 = g_ui_crc_errors;
  uint8_t frame[260];
  uint8_t payload[255];
  uint32_t fails = 0;

  // 1) MIDI frame round-trip: 4-byte USB-MIDI event packet payload.
  {
    const uint8_t p[4] = { 0x09, 0x90, 0x3C, 0x64 };
    const size_t n = ui_link_build_frame(kUiTypeMidi, p, 4, frame);
    bool completed = false;
    for (size_t i = 0; i < n; ++i) completed |= ui_link_step(frame[i]);
    const bool ok = completed && g_ui_rx_crc_ok && g_ui_rx_len == 4 &&
                    g_ui_rx_buf[0] == kUiTypeMidi &&
                    g_ui_rx_buf[1] == 0x09 && g_ui_rx_buf[2] == 0x90 &&
                    g_ui_rx_buf[3] == 0x3C && g_ui_rx_buf[4] == 0x64;
    Serial.println(ok ? "uitest: 1/6 MIDI frame round-trip PASS"
                      : "uitest: 1/6 MIDI frame round-trip FAIL");
    if (!ok) ++fails;
  }
  // 2) MOD_SET frame round-trip (3-byte payload; NOT dispatched — no engine
  //    side effects, the parser only assembles).
  {
    const uint8_t p[3] = { 3, 1, 90 };
    const size_t n = ui_link_build_frame(kUiTypeModSet, p, 3, frame);
    bool completed = false;
    for (size_t i = 0; i < n; ++i) completed |= ui_link_step(frame[i]);
    const bool ok = completed && g_ui_rx_crc_ok && g_ui_rx_len == 3 &&
                    g_ui_rx_buf[0] == kUiTypeModSet &&
                    g_ui_rx_buf[1] == 3 && g_ui_rx_buf[2] == 1 &&
                    g_ui_rx_buf[3] == 90;
    Serial.println(ok ? "uitest: 2/6 MOD_SET frame round-trip PASS"
                      : "uitest: 2/6 MOD_SET frame round-trip FAIL");
    if (!ok) ++fails;
  }
  // 3) 0x7E INSIDE a payload (no byte stuffing — len is known up front).
  {
    const uint8_t p[3] = { 0x7E, 0x7E, 0x55 };
    const size_t n = ui_link_build_frame(0x40, p, 3, frame);
    bool completed = false;
    for (size_t i = 0; i < n; ++i) completed |= ui_link_step(frame[i]);
    const bool ok = completed && g_ui_rx_crc_ok && g_ui_rx_len == 3 &&
                    g_ui_rx_buf[1] == 0x7E && g_ui_rx_buf[2] == 0x7E &&
                    g_ui_rx_buf[3] == 0x55;
    Serial.println(ok ? "uitest: 3/6 0x7E-in-payload PASS"
                      : "uitest: 3/6 0x7E-in-payload FAIL");
    if (!ok) ++fails;
  }
  // 4) Max-length frame (255-byte payload).
  {
    for (uint16_t i = 0; i < 255; ++i) payload[i] = (uint8_t)(i & 0xFF);
    const size_t n = ui_link_build_frame(kUiTypeTelem, payload, 255, frame);
    bool completed = false;
    for (size_t i = 0; i < n; ++i) completed |= ui_link_step(frame[i]);
    bool ok = completed && g_ui_rx_crc_ok && g_ui_rx_len == 255 &&
              g_ui_rx_buf[0] == kUiTypeTelem;
    for (uint16_t i = 0; ok && i < 255; ++i) ok = g_ui_rx_buf[1 + i] == (uint8_t)(i & 0xFF);
    Serial.println(ok ? "uitest: 4/6 max-len (255 B) round-trip PASS"
                      : "uitest: 4/6 max-len (255 B) round-trip FAIL");
    if (!ok) ++fails;
  }
  // 5) Corrupted CRC: the frame must be REJECTED (crc_err counter +1) and the
  //    parser resynced to HUNT.
  {
    const uint8_t p[3] = { 0x11, 0x22, 0x33 };
    const size_t n = ui_link_build_frame(0x40, p, 3, frame);
    frame[n - 1] ^= 0xFF;                     // corrupt ONLY the crc byte
    bool completed = false;
    for (size_t i = 0; i < n; ++i) completed |= ui_link_step(frame[i]);
    const bool ok = completed && !g_ui_rx_crc_ok &&
                    (g_ui_crc_errors == crc0 + 1) &&
                    g_ui_rx_state == UI_RX_HUNT;
    Serial.println(ok ? "uitest: 5/6 corrupted-CRC rejection PASS"
                      : "uitest: 5/6 corrupted-CRC rejection FAIL");
    if (!ok) ++fails;
  }
  // 6) Resync: the SAME frame (uncorrupted) completes cleanly right after the
  //    rejected one (the parser hunted past the dropped frame's bytes).
  {
    const uint8_t p[3] = { 0x11, 0x22, 0x33 };
    const size_t n = ui_link_build_frame(0x40, p, 3, frame);
    bool completed = false;
    for (size_t i = 0; i < n; ++i) completed |= ui_link_step(frame[i]);
    const bool ok = completed && g_ui_rx_crc_ok && g_ui_rx_len == 3;
    Serial.println(ok ? "uitest: 6/6 post-rejection resync PASS"
                      : "uitest: 6/6 post-rejection resync FAIL");
    if (!ok) ++fails;
  }
  g_ui_rx_state = UI_RX_HUNT;                 // leave the pump a clean parser
  Serial.printf("uitest: codec %s (%u corrupt-CRC case(s) counted in crc_err) | "
                "hw loopback: n/a on RP2350 UART (no LBM bit) - wire check = "
                "C3 selftest + cross-board echo\r\n",
                fails ? "FAIL" : "OK", (unsigned)(g_ui_crc_errors - crc0));
}

// cc — inject a MIDI CC through the REAL funnel (midiControlChange): identical
// path to a controller/panel CC, including the TELEM cache + autosave-dirty,
// the CC 32 / 33-35 intercepts and the registry dispatch. Bench tool (T9
// warps per-algorithm sweep needs fixed values without hardware knobs); NOT
// a performance input (no debounce, core-0 console context).
static void cmd_cc(const char* args) {
  int cc = -1, val = -1;
  if (sscanf(args, "%d %d", &cc, &val) != 2 || cc < 0 || cc > 127 ||
      val < 0 || val > 127) {
    Serial.println("usage: cc <0-127> <0-127>   (through midiControlChange)");
    return;
  }
  crumb(50);   // just before midiControlChange dispatch (hang debug)
  midiControlChange(static_cast<uint8_t>(cc), static_cast<uint8_t>(val));
  crumb(51);   // just after dispatch
  // Non-blocking: a blocking CDC printf under a slow/no reader was the
  // documented 2026-11-16 hang class (core 0 stalls -> watchdog). Warps makes
  // renders slower, so fast CC benches can back up the CDC FIFO.
  dbg_printf("cc: %u -> %u\r\n", (unsigned)cc, (unsigned)val);
}

// qmi — read/write the LIVE QMI M0 flash timing (T9 clock diag). On RP2350
// the flash timing is set by the ROM boot and set_sys_clock_khz does NOT
// re-scale it, so flash SCK = clk_sys / CLKDIV and the ROM's RXDELAY sampling
// delay is tuned for the boot clock — an overclock must pin a proven
// CLKDIV/RXDELAY combo BEFORE any XIP-heavy work (setup's window copy reads
// flash). `qmi` reads; `qmi set <clkdiv> <rxdelay>` writes (read-modify-write
// of M0_TIMING; the write is a register access, new fetches use the new
// timing). The proven generic-boot2 combo is CLKDIV 4 / RXDELAY 2.
static void cmd_qmi(const char* args) {
  uint32_t clkdiv = 0, rxdel = 0;
  if (sscanf(args, "set %lu %lu", (unsigned long*)&clkdiv,
             (unsigned long*)&rxdel) == 2) {
    if (clkdiv < 1 || clkdiv > 255 || rxdel > 7) {
      Serial.println("qmi: set range CLKDIV 1-255, RXDELAY 0-7");
      return;
    }
    uint32_t t = qmi_hw->m[0].timing;
    t = (t & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
        (clkdiv << QMI_M0_TIMING_CLKDIV_LSB) |
        (rxdel << QMI_M0_TIMING_RXDELAY_LSB);
    qmi_hw->m[0].timing = t;
  }
  const uint32_t timing = qmi_hw->m[0].timing;
  const uint32_t div = timing & QMI_M0_TIMING_CLKDIV_BITS;
  const uint32_t rx = (timing >> QMI_M0_TIMING_RXDELAY_LSB) & 0x7u;
  const uint32_t fcpu = clock_get_hz(clk_sys);
  Serial.printf("qmi: f_cpu=%lu Hz, CLKDIV=%lu RXDELAY=%lu -> flash SCK "
                "%lu.%01lu MHz (320 MHz boot default: CLKDIV 4 / RXDELAY 3 "
                "-> SCK 80; keep SCK <= 88)\r\n",
                (unsigned long)fcpu, (unsigned long)div, (unsigned long)rx,
                (unsigned long)(fcpu / div / 1000000UL),
                (unsigned long)((fcpu / div / 100000UL) % 10));
}

// uisend — send ONE full TELEM snapshot for the C3's active page (default P1)
// and print the sizes (payload bytes inside the len field + the whole frame).
static void cmd_uisend(const char* args) {
  while (*args == ' ') ++args;
  uint8_t page = g_ui_active_page;
  if (*args >= '0' && *args <= '9') {
    long v = atol(args);
    if (v >= 0 && v < kUiPageCount) page = (uint8_t)v;
  }
  const uint32_t tx0 = g_ui_tx_frames;
  ui_link_send_telem(page, kUiTelemFull, 0);
  if (g_ui_tx_frames == tx0 + 1) {
    Serial.printf("uisend: TELEM page %u sent (%u payload bytes, %u frame "
                  "bytes incl. mark+len+crc)\r\n",
                  page, (unsigned)g_ui_tx_buf[1],
                  (unsigned)(g_ui_tx_buf[1] + 4));
  } else {
    Serial.println("uisend: no frame sent (link send failed?)");
  }
}

static void cmd_help(const char* args) {
  (void)args;
  Serial.println("rack console commands:");
  for (uint8_t i = 0; i < kConsoleCommandCount; ++i) {
    Serial.printf("  %s\r\n", kConsoleCommands[i].name);
  }
  Serial.println("  switch <0-4|name>  live module swap (also MIDI CC 32, value 0-4)");
  Serial.println("  module             print active module + cc map + mod destinations");
  Serial.println("  chain [a] [b]|off  T6 serial chain (b defaults clouds; B mode clamped");
  Serial.println("                     to granular/looping; CC 23/24/65/66 + grain bank");
  Serial.println("                     40-48 -> B while chained, other CCs -> A;");
  Serial.println("                     bare 'chain' = status)");
  Serial.println("  mod <src> <param> <value>  set a mod-engine source param");
  Serial.println("     src: 0-11 or lfo1-4 / eu1-4 / gate1-2 / adsr1-2");
  Serial.println("     LFO  params: mode(hz|clock) rate(Hz) wave(sine|tri|saw|square|sh)");
  Serial.println("          phase(deg) level | clk(global|int|midi) div(1/4..) dest(CC|off)");
  Serial.println("     EU   params: steps(1-32) pulses rot | clk div dest");
  Serial.println("     GATE params: mode(gen|alt8..rise|tmr|mrkv|0-18) dens(1-16) sync(0-100%) glen(0-1) | clk div dest");
  Serial.println("     ADSR params: mode(gate|retrig) atk dec rel (ms) sus(0-1) | clk div dest");
  Serial.println("  mod?               dump the whole engine (params + routing + telemetry)");
  Serial.println("  route <src> <dest> alias of 'mod <src> dest <dest>' (dest = CC|off)");
  Serial.println("  bpm <20-300>       internal clock BPM (0.1 steps)");
  Serial.println("  clock <internal|midi> [src]  global clock source / per-source override");
  Serial.println("  status             one-line summary: module/clock/swaps/din/uptime");
  Serial.println("  crash [clear]      last-reset reason + watchdog cause + fault streak");
  Serial.println("                     (reason 4 = watchdog died IN setup => clock verdict)");
  Serial.println("  qmi [set D R]      live QMI flash timing (CLKDIV/RXDELAY); SCK=f_cpu/D");
  Serial.println("  oc <khz> [D] [R]   live overclock bench with continuous FNV watch");
  Serial.println("                     (above 280 raises VREG 1.20; `oc off`/reboot restores)");
  Serial.println("  vreg [1100|1150|1200|1250]  read/set core supply (mV)");
  Serial.println("  din [clear]        DIN MIDI counters + last message (UART1 GP5)");
  Serial.println("  dintest            inject a canned MIDI stream into the DIN parser");
  Serial.println("                     -> funnel (notes/CC/clock; UART1 bypassed)");
  Serial.println("  ui                 T5 UI-MCU link status (tx/rx frames, crc errs,");
  Serial.println("                     last echo turnaround)");
  Serial.println("  uitest             UI-link codec self-test (in-memory round-trip");
  Serial.println("                     incl. corrupted-CRC rejection; RP2350 UART has");
  Serial.println("                     no hw loopback — wire check = C3 selftest)");
  Serial.println("  uisend [page]      send one full TELEM snapshot (0-5, default =");
  Serial.println("                     the C3's active page) + print sizes");
  Serial.println("  telemetry on|off   mute/unmute the 2 s report (default on)");
  Serial.println("  panic              all notes off (release held notes + clear stack)");
  Serial.println("  trig [down|up]     fire the hardware PLAY button path (GP2 twin;");
  Serial.println("                     bare = press+release pulse, CC 64/66 to the");
  Serial.println("                     active module)");
  Serial.println("  piezo [gain 1-16]  T2 piezo read-out: rate/min/max/peak (GP26);");
  Serial.println("                     tap the piezo and re-read");
  Serial.println("MIDI bench: CC 32 = switch | CC 33/34/35 = mod source/param/value mux");
  Serial.println("web panels drive everything else (per-module CC maps)");
}

// Non-blocking line assembler (core 0 loop): '\n'- or '\r'-terminated.
static char console_line[64];
static uint8_t console_len = 0;

static void pumpConsole() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (console_len == 0) continue;      // swallow empty/CR-LF pairs
      console_line[console_len] = '\0';
      console_len = 0;
      // T4a: preset collect rows ("PRxx:<hex>") swallow before dispatch
      // while a 'preset load' collection is in progress.
      if (preset_collecting() && preset_collect_line(console_line)) {
        continue;
      }
      // tokenize: name + rest
      char* args = console_line;
      while (*args == ' ') ++args;
      char* space = strchr(args, ' ');
      const char* arg_str = "";
      if (space) {
        *space = '\0';
        arg_str = space + 1;
      }
      bool handled = false;
      for (uint8_t i = 0; i < kConsoleCommandCount; ++i) {
        if (strcasecmp(args, kConsoleCommands[i].name) == 0) {
          kConsoleCommands[i].fn(arg_str);
          handled = true;
          break;
        }
      }
      if (!handled) {
        Serial.println("unknown command (try help)");
      }
      continue;
    }
    if (console_len < sizeof(console_line) - 1) {
      console_line[console_len++] = static_cast<char>(c);
    } else {
      console_len = 0;                     // overflow: drop the line
    }
  }
}

// ===================== crash diagnostics (HardFault record) =====================
// A HardFault used to be silent: the USB device stays enumerated (the control
// transfers are serviced from IRQ context), but core 0's loop dies, so the
// console, the UI link and the 1200-baud flash touch all stop — the bench saw
// it as "the board crashed". The handler below saves the exception frame and
// the fault-status registers to .uninitialized_data (NOLOAD: SRAM survives a
// warm reset, and crt0 neither loads nor zeroes this section), then resets.
// setup() prints the record once, after the deferred boot line, so the fault
// PC can be resolved with `arm-none-eabi-addr2line -e firmware.elf <pc>`.
//
// T9 SAFE MODE (the "console dies with the crash" lesson): an aggressive
// clock (280+) can fault BEFORE the USB CDC enumerates, so the record is
// written but unreadable. The record therefore carries counters:
//   - fault_streak: +1 per HardFault reset (never auto-cleared — STICKY),
//   - boot_count:   +1 per boot that never reached a healthy 5 s loop.
// When fault_streak >= 2 or boot_count >= 3, setup() drops the clock to
// 240 MHz as its FIRST action — one boot is ALWAYS readable and prints the
// record. Clear with the console command `crash clear` (re-arms normal clock).
struct CrashRecordV2 {
  uint32_t magic;
  uint32_t core;
  uint32_t pc, lr, psr;
  uint32_t r0, r1, r2, r3, r12;
  uint32_t cfsr, hfsr, mmfar, bfar;
  uint32_t boot_count;      // consecutive unhealthy boots
  uint32_t fault_streak;    // consecutive HardFault resets (sticky)
  uint32_t reset_reason;    // 0 cold, 1 POR/hardware flap, 2 HardFault,
                            // 3 watchdog in loop, 4 watchdog DURING setup,
                            // 5 clean reboot
  uint32_t safe_mode;       // this boot ran at 240 MHz (record printed)
  uint32_t wd_reason;       // raw WATCHDOG_REASON bits: 1=TIMER, 2=FORCE
};
static constexpr uint32_t kCrashMagicV2 = 0xC0DE5AF3u;   // v2: counters
__attribute__((section(".uninitialized_data.crash"), used))
static volatile CrashRecordV2 g_crashv2;

// Called from isr_hardfault with the exception-frame pointer + EXC_RETURN.
// Cortex-M33: an FP-extended frame stacks S0-S15 + FPSCR (17 words) BELOW the
// basic r0..xPSR frame; EXC_RETURN bit 4 clear means the extended frame is
// present, and the basic frame then starts at sp + 17 words.
extern "C" void crash_record_c(uint32_t* sp, uint32_t exc_return) {
  uint32_t* f = ((exc_return & 0x10u) == 0u) ? (sp + 17) : sp;
  g_crashv2.fault_streak = g_crashv2.fault_streak + 1u;   // sticky, pre-magic
  g_crashv2.core = (uint32_t)get_core_num();
  g_crashv2.r0 = f[0];
  g_crashv2.r1 = f[1];
  g_crashv2.r2 = f[2];
  g_crashv2.r3 = f[3];
  g_crashv2.r12 = f[4];
  g_crashv2.lr = f[5];
  g_crashv2.pc = f[6];
  g_crashv2.psr = f[7];
  g_crashv2.cfsr = scb_hw->cfsr;
  g_crashv2.hfsr = scb_hw->hfsr;
  g_crashv2.mmfar = scb_hw->mmfar;
  g_crashv2.bfar = scb_hw->bfar;
  g_crashv2.reset_reason = 2;
  __dmb();
  g_crashv2.magic = kCrashMagicV2;
  __dmb();
  // NVIC_SystemReset() is not linked in this TU; AIRCR SYSRESETREQ is the same
  // register write (VECTKEY 0x05FA, then system reset request).
  scb_hw->aircr = (0x05FAu << 16) | 0x04u;
  __asm volatile("dsb 0xF" ::: "memory");
  for (;;) {
  }
}

// Strong override of the SDK's weak isr_hardfault (the vector table entry).
extern "C" __attribute__((naked)) void isr_hardfault(void) {
  __asm volatile(
      "tst lr, #4\n"        // EXC_RETURN bit 2: 0 = MSP used, 1 = PSP
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"        // EXC_RETURN (bit 4 -> FP frame)
      "b crash_record_c\n");
}

// T9 oc-bench oracle: the boot-time window-copy FNV of the boot module's
// FLASH image. cmd_oc re-reads the flash through XIP and compares — a pure
// read test that never touches the SRAM window core 1 is executing from.
static uint32_t g_window_src_sum = 0;
// Set by `oc`: enables a continuous XIP FNV re-check in the 2 s report. The
// 280 failure mode was INTERMITTENT corruption (valid TELEM frames with bad
// CRC), which a one-shot FNV at switch time cannot catch — a soak can.
static volatile bool g_oc_verify = false;

// crash — print/clear the T9 crash counters (NOLOAD SRAM, survives resets).
// `crash clear` zeroes the sticky fault_streak so normal clocks re-arm after
// SAFE MODE. The one-shot CRASH line is printed by setup (see the boot block).
static void cmd_crash(const char* args) {
  while (*args == ' ') ++args;
  if (strcasecmp(args, "clear") == 0) {
    g_crashv2.boot_count = 0;
    g_crashv2.fault_streak = 0;
    g_crashv2.safe_mode = 0;
    g_crashv2.magic = 0;
    g_crashv2.reset_reason = 0;
    g_crashv2.wd_reason = 0;
    __dmb();
    Serial.printf("crash: counters cleared — next reboot runs at F_CPU "
                  "(%lu Hz build clock)\r\n", (unsigned long)F_CPU);
    return;
  }
  Serial.printf("crash: reason=%lu wd=0x%02lx unhealthy_boots=%lu "
                "fault_streak=%lu safe_mode=%lu\r\n",
                (unsigned long)g_crashv2.reset_reason,
                (unsigned long)g_crashv2.wd_reason,
                (unsigned long)g_crashv2.boot_count,
                (unsigned long)g_crashv2.fault_streak,
                (unsigned long)g_crashv2.safe_mode);
#if RACK_HANG_DEBUG
  Serial.printf("crash: hang0=%lu hang1=%lu (last stages before any hang)\r\n",
                (unsigned long)g_hang0, (unsigned long)g_hang1);
#endif
  if (g_crashv2.magic == kCrashMagicV2) {
    Serial.printf("crash: core=%lu pc=0x%08lx lr=0x%08lx cfsr=0x%08lx "
                  "hfsr=0x%08lx mmfar=0x%08lx bfar=0x%08lx (addr2line the pc)\r\n",
                  (unsigned long)g_crashv2.core,
                  (unsigned long)g_crashv2.pc, (unsigned long)g_crashv2.lr,
                  (unsigned long)g_crashv2.cfsr, (unsigned long)g_crashv2.hfsr,
                  (unsigned long)g_crashv2.mmfar, (unsigned long)g_crashv2.bfar);
  } else {
    Serial.println("crash: no fault record held (one-shot line was printed at boot)");
  }
}

// oc — runtime overclock bench (T9). The app boots at the safe build clock;
// this switches live so a clock candidate costs one console command, not a
// reflash. Order follows the SDK's own PSRAM rule: pin the QMI timing FIRST,
// then raise the PLL. Above 280 MHz raise VREG to 1.20 V (the core only
// auto-sets 1.15). The FLASH-integrity oracle is the boot-time window-copy
// FNV over the boot module's flash image, re-read through XIP (reads only).
// Serial1/I2S dividers stay programmed for the old clock until `reboot`:
// audio runs off-rate and the UI link is dead — both restored by rebooting.
// A crash lands in SAFE MODE (240 MHz) on the next boot, readable.
static void cmd_oc(const char* args) {
  while (*args == ' ') ++args;
  if (strncasecmp(args, "off", 3) == 0) {
    g_oc_verify = false;
    if (!set_sys_clock_khz(F_CPU / 1000u, true))
      Serial.println("oc off: PLL restore failed (use `reboot`)");
    else
      Serial.printf("oc off: restored build clock %lu Hz, FNV watch stopped "
                    "(`reboot` also restores Serial1/I2S dividers)\r\n",
                    (unsigned long)F_CPU);
    return;
  }
  unsigned khz = 0, div = 0, rx = 0;
  const int n = sscanf(args, "%u %u %u", &khz, &div, &rx);
  if (n < 1 || khz < 24000u || khz > 400000u) {
    Serial.println("usage: oc <khz 24000-400000> [clkdiv] [rxdelay] | off   "
                   "(defaults: live QMI values; `reboot` restores)");
    return;
  }
  const uint32_t t0 = qmi_hw->m[0].timing;
  if (div == 0) div = t0 & QMI_M0_TIMING_CLKDIV_BITS;
  if (rx == 0 && n < 3) rx = (t0 >> QMI_M0_TIMING_RXDELAY_LSB) & 0x7u;
  if (div < 1u || div > 255u || rx > 7u) {
    Serial.println("oc: clkdiv 1-255, rxdelay 0-7");
    return;
  }
  if (khz > 280000u && vreg_get_voltage() < VREG_VOLTAGE_1_25) {
    // Never lower the core: boot default is 1.25 V (needed for 320 MHz + Warps).
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    delay(5);                       // settle (timers are microsecond-locked)
  }
  uint32_t t = qmi_hw->m[0].timing;
  t = (t & ~(QMI_M0_TIMING_CLKDIV_BITS | QMI_M0_TIMING_RXDELAY_BITS)) |
      ((div & 0xFFu) << QMI_M0_TIMING_CLKDIV_LSB) |
      (rx << QMI_M0_TIMING_RXDELAY_LSB);
  qmi_hw->m[0].timing = t;
  __dmb();
  if (!set_sys_clock_khz(khz, true)) {
    Serial.printf("oc: PLL cannot hit %u kHz exactly — timing left as-is\r\n",
                  khz);
    return;
  }
  const RackWindowSection& s = rack_window_section(RACK_BOOT_MODULE);
  const uint32_t sum =
      rack_checksum32(s.flash_start, (size_t)(s.flash_end - s.flash_start));
  const uint32_t fcpu = clock_get_hz(clk_sys);
  const uint32_t live_div = qmi_hw->m[0].timing & QMI_M0_TIMING_CLKDIV_BITS;
  Serial.printf("oc: f_cpu=%lu CLKDIV=%lu RXDELAY=%lu -> flash SCK %lu.%01lu "
                "MHz | flash FNV 0x%08lx vs boot 0x%08lx -> %s\r\n",
                (unsigned long)fcpu, (unsigned long)live_div,
                (unsigned long)rx,
                (unsigned long)(fcpu / live_div / 1000000UL),
                (unsigned long)((fcpu / live_div / 100000UL) % 10u),
                (unsigned long)sum, (unsigned long)g_window_src_sum,
                sum == g_window_src_sum ? "OK" : "MISMATCH (XIP corrupting)");
  // Flash SCK = clk_sys / CLKDIV (the RP2350 ROM pins CLKDIV 3 / RXDELAY 2 and
  // set_sys_clock_khz does NOT re-scale it). 264/3 = 88 MHz was the last known
  // good SCK; above that the sample point goes marginal. Recommend the CLKDIV
  // that keeps SCK <= 88 MHz so a higher core clock does not drag XIP with it.
  const uint32_t safe_div = (fcpu + 87999999u) / 88000000u;
  if (live_div < safe_div) {
    Serial.printf("oc: hint: CLKDIV >= %lu keeps flash SCK <= 88 MHz "
                  "(ROM 3 gives %lu.%lu MHz here) — retry "
                  "`oc %u %lu 2`\r\n",
                  (unsigned long)safe_div,
                  (unsigned long)(fcpu / live_div / 1000000UL),
                  (unsigned long)((fcpu / live_div / 100000UL) % 10u),
                  khz, (unsigned)safe_div);
  }
  g_oc_verify = true;   // continuous FNV re-check in the 2 s report
  Serial.println("oc: Serial1/I2S stale (off-rate, UI link dead) — `reboot` "
                 "restores the build clock");
}

// vreg — bench the core supply live. `vreg <mv>`: 1150|1200|1250|1300 mV;
// bare = report. 270 MHz reset at default 1.15 V even with flash SCK 67.5,
// so VREG (not flash timing) is the suspected lever for the next clocks.
static void cmd_vreg(const char* args) {
  while (*args == ' ') ++args;
  if (*args) {
    unsigned mv = (unsigned)atol(args);
    enum vreg_voltage v;
    switch (mv) {
      case 1100: v = VREG_VOLTAGE_1_10; break;
      case 1150: v = VREG_VOLTAGE_1_15; break;
      case 1200: v = VREG_VOLTAGE_1_20; break;
      case 1250: v = VREG_VOLTAGE_1_25; break;
      default:
        Serial.println("vreg: use 1100|1150|1200|1250 (mV)");
        return;
    }
    vreg_set_voltage(v);
    delay(5);
  }
  const uint32_t v = vreg_get_voltage();
  Serial.printf("vreg: VREG_VOLTAGE=%lu (RP2350: 5=0.80, C=1.15, D=1.20, "
                "E=1.25, F=1.30)\r\n", (unsigned long)v);
}

void setup() {
  // --- Crash forensics: capture the PREVIOUS boot's evidence FIRST ---
  // watchdog_enable() writes WATCHDOG_NON_REBOOT_MAGIC (0x6ab73121) into
  // scratch[4] -- so the boot-done marker MUST be read BEFORE it is called.
  // WATCHDOG_REASON (RO) is printed raw for reference but NOT used as the
  // discriminator: on this chip its TIMER bit is STICKY across soft resets
  // (a `reboot` after a BOOTSEL touch still showed wd=0x01). Instead we own
  // three scratch words, which POR clears but soft/watchdog resets preserve:
  //   scratch[4] = 0xB007CAFE once setup completed (boot-done marker),
  //   scratch[5] = CLEAN_SHUTDOWN set only by the `reboot` console command,
  //   scratch[6] = SETUP_ACTIVE while setup is in progress (cleared at end).
  const uint32_t prev_wd_reason = watchdog_hw->reason;
  const uint32_t prev_boot_marker = watchdog_hw->scratch[4];
  const uint32_t prev_clean_shutdown = watchdog_hw->scratch[5];
  const uint32_t prev_setup_active = watchdog_hw->scratch[6];
#if RACK_HANG_DEBUG
  {
    const uint32_t raw0 = watchdog_hw->scratch[0];
    const uint32_t raw1 = watchdog_hw->scratch[1];
    if ((raw0 & 0xFFFF0000u) == kCrumbMagic) g_hang0 = raw0 & 0x3Fu;
    if ((raw1 & 0xF0000000u) == 0xB0000000u) g_hang1 = raw1 & 0x3Fu;
    watchdog_hw->scratch[0] = 0;   // clear crumbs for this boot
    watchdog_hw->scratch[1] = 0;
  }
#endif
  // Mark THIS boot as in-progress and clear the previous clean-shutdown flag.
  watchdog_hw->scratch[6] = 0x5E7AC71Bu;   // SETUP_ACTIVE
  watchdog_hw->scratch[5] = 0;
  // BOOT WATCHDOG FIRST: 8 s covers the window copy + I2S + USB enumeration;
  // a boot that stalls on a print or a peripheral gets reset instead of
  // freezing. The loop's 2 s period replaces this at the end of setup.
  watchdog_enable(8000, false);
  // --- T9 crash counters + SAFE MODE ---------------------------------------
  // .uninitialized_data survives warm resets; a cold boot leaves garbage, so
  // the counters self-sanitize.
  {
    if (g_crashv2.boot_count > 500u || g_crashv2.fault_streak > 50u) {
      g_crashv2.boot_count = 0;
      g_crashv2.fault_streak = 0;
      g_crashv2.safe_mode = 0;
    }
    g_crashv2.wd_reason = prev_wd_reason;
    if (g_crashv2.magic == kCrashMagicV2) {
      g_crashv2.reset_reason = 2;              // the fault handler set it
    } else if (prev_clean_shutdown == 0xC1EA0FFu) {
      g_crashv2.reset_reason = 5;              // intentional `reboot`
    } else if (prev_setup_active == 0x5E7AC71Bu &&
               prev_boot_marker != 0xB007CAFEu) {
      g_crashv2.reset_reason = 4;              // died DURING setup (clock verdict)
    } else if (prev_boot_marker == 0xB007CAFEu) {
      g_crashv2.reset_reason = 3;              // watchdog/external after setup
    } else {
      g_crashv2.reset_reason = 1;              // POR / brownout / hardware flap
    }
    ++g_crashv2.boot_count;
    // SAFE MODE: a crashy clock config (hard fault or hang before the loop)
    // would otherwise bootloop invisibly when the USB CDC never enumerates at
    // the aggressive clock. After repeated unhealthy boots, run THIS boot at
    // 240 MHz: the console comes up and the record + counters are readable.
    // Sticky fault_streak keeps it engaged until `crash clear`.
    if (g_crashv2.fault_streak >= 2u || g_crashv2.boot_count >= 3u) {
      if (clock_get_hz(clk_sys) > 240000000u) {
        set_sys_clock_khz(240000, true);
      }
      g_crashv2.safe_mode = 1;
    }
  }
  // --- SRAM window: copy the BOOT module's hot section flash -> SRAM ---
  // MUST run before ANY module code executes (module init calls tc_*
  // functions: Part::Init, GranularProcessor::Init, Voice::Init, ...). The
  // boot print carries the copy size and both checksums; a mismatch is a
  // hard error (the window content would not match the flash image).
  //
  // BOOT-WEDGE FIX (2026-11-04): the window line used to print HERE — within
  // milliseconds of boot, i.e. while the USB host was still enumerating the
  // freshly attached CDC device. An unconditional CDC write that early wedged
  // the chip nondeterministically (~40% of boots: hang at widely varying
  // later points — host init, usb_midi.begin/detach, or post-setup; the LED
  // breadcrumbs + boot-timing experiments isolated the first-print timing as
  // the lever; the proven standalone prints its first unconditional line only
  // AFTER host+I2S init). The line is composed now and printed after
  // enumeration settles. Conditional ERROR prints stay early — they only
  // execute on failures, like the proven standalone.
  //
  // STANDALONE-ALIGNMENT (2026-11-04): the only remaining structural boot
  // delta vs the proven standalone ports was the copy's POSITION — the
  // standalone's hot code is copied by crt0 BEFORE any peripheral exists,
  // while the rack copied inside setup() with USB enumeration already racing.
  // The copy is now the ABSOLUTE FIRST action of setup(), before
  // Serial.begin/LED/USB — crt0-like semantics. (T2: same copy, one shared
  // window; the swap repeats it at runtime.)
  static char window_line[128];
  {
    uint32_t src_sum = 0, dst_sum = 0;
    const uint32_t n = rack_window_copy(RACK_BOOT_MODULE, &src_sum, &dst_sum);
    g_window_src_sum = src_sum;   // oc-bench flash-integrity oracle baseline
    const RackWindowSection& s = rack_window_section(RACK_BOOT_MODULE);
    snprintf(window_line, sizeof(window_line),
             "rack: window 0x%08x+%u | copied %s %u bytes (checksum src=0x%08x dst=0x%08x)",
             (unsigned)(uintptr_t)s.window_start,
             (unsigned)kRackWindowBytes,
             s.name, (unsigned)n,
             (unsigned)src_sum, (unsigned)dst_sum);
    if (src_sum != dst_sum) {
      Serial.println("rack: ERROR: window checksum mismatch - flash copy corrupt, module hot code is NOT validated");
    }
  }

  // USB MIDI device: register the interface as the FIRST setup action.
  // The core ran TinyUSB_Device_Init(0) before setup(); the host's first
  // GET_DESCRIPTOR lands >=100 ms after attach (USB reset recovery), so an
  // interface added within the first milliseconds is IN the first
  // configuration — no re-enumeration needed. This REPLACES the old
  // detach/attach re-enum guard, which was itself the boot killer: the
  // post-detach re-enumeration intermittently died (kernel log: SET_ADDRESS
  // refused / descriptor timeouts until a power cycle) — the same "CDC port
  // death" family the standalone era patched with the 500 ms window, never
  // eliminated. (2026-11-04, T1 bring-up.)
  usb_midi.setStringDescriptor("Rack RP2350");
  if (!usb_midi.begin()) {
    Serial.println("USB MIDI begin FAILED");
  }
  TinyUSBDevice.setProductDescriptor("Rack RP2350");

  Serial.begin(115200);
  mod_engine_init();                       // T3: engine defaults (plain .bss,
                                           // independent of the modules)
  kRackModules[RACK_BOOT_MODULE].init();   // registry: boot module, arena-placed
  // T4 DIN MIDI RX (UART1 @ 31250, GP5): hardware UART claims NO PIO, no DMA
  // channel and no alarm pool, so — unlike the PIO-USB/I2S/LED order law
  // above — its position here is unconstrained; it sits after the engine/
  // module init and BEFORE the PIO-USB host init so the link is live before
  // audio starts. No print (boot-wedge law: no unconditional early CDC).
  din_uart_init();
  // T5 stage 1 UI-MCU link (UART0 @ 921600, GP0/GP1): hardware UART as
  // well — same no-PIO argument, same unconstrained position, same no-print
  // rule. Absent/unwired link costs nothing (pump = one ring-empty check).
  ui_link_uart_init();
  // T1 hardware PLAY button (GP2, internal pull-up, button -> GND): plain
  // GPIO — no PIO/DMA/alarm claims, so its position is unconstrained (same
  // class as the DIN/UI hardware UARTs). No print (boot-wedge law).
  gpio_init(kPinButton);
  gpio_set_dir(kPinButton, GPIO_IN);
  gpio_pull_up(kPinButton);
  // T2 piezo (GP26 ADC0): ADC FIFO + one DMA ring — plain peripherals, no
  // PIO/alarm claims; position unconstrained (same class as DIN/UI/button).
  rack_piezo::init();
  preset_replay_module(RACK_BOOT_MODULE);
  statusLed.Init();  // 3 white self-test flashes, then dark

#if RACK_ENABLE_USB_HOST
  // --- USB host (PIO-USB, rhport 1): MUST init BEFORE i2s.begin() ---
  // PIO-USB hard-claims its config's PIO/SMs and loads all 32 PIO0
  // instruction slots plus DMA channel 0 and its own 1 ms SOF alarm pool.
  // I2S's PIOProgram instead flexibly allocates a free PIO+SM at begin();
  // the status LED claims PIO2 SM0, so I2S lands on PIO2 SM1.
  if (F_CPU % 12000000UL) {
    // Same guard as the bundled DualRole usbh_helper: PIO-USB bit-banging
    // needs a CPU clock that is a multiple of 12 MHz. Build uses 240 MHz.
    Serial.println("ERROR: CPU clock not a multiple of 12 MHz - PIO-USB host timing unreliable");
  }
  // Wait for core 1 to create the core-1 alarm pool (see setup1), then hand
  // it to PIO-USB: the SOF alarm ISR runs on core 1, isolated from core 0's
  // I2S DMA and TinyUSB device interrupts.
  while (!g_host_alarm_pool_ready) { delay(1); }
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = kPinHostDP;    // D+ = GP14, D- = GP15 (DPDM pinout)
  pio_cfg.alarm_pool = g_host_alarm_pool;
  if (!USBHost.configure_pio_usb(1, &pio_cfg)) {
    Serial.println("PIO-USB configure FAILED (host driver missing for rhport 1?)");
  }
  if (!USBHost.begin(1)) {
    Serial.println("USB host begin FAILED");
  }
#endif

  // Boot module's native rate / block size / DMA bank depth (registry
  // fields — see the standalone ports for the sizing rationale).
  const RackModuleInfo& boot = kRackModules[RACK_BOOT_MODULE];
  i2s.setBitsPerSample(16);
  i2s.setBuffers(boot.i2s_buffers, boot.i2s_words);
  i2s.setFrequency(boot.sample_rate);
  if (!i2s.begin()) {
    Serial.println("I2S begin FAILED");
    statusLed.NotifyBootError();  // PANIC blink pattern
  } else {
    Serial.println("I2S up");
  }
  // T8: i2s.begin() enabled DMA_IRQ_0 on THIS core; hand it to core 1 (the
  // audio writer) so the AudioBufferManager's noInterrupts() actually covers
  // the DMA IRQ. setup1() re-enables it on core 1 after audio_ready.
  irq_set_enabled(DMA_IRQ_0, false);

  // Boot diagnostics are NON-BLOCKING: a console-less boot must never stall on
  // the USB CDC FIFO (bench-caught: a watchdog reboot with no reader attached
  // hung in these prints before setup could enable the watchdog).
  dbg_printf("PIO2 SM0 claimed=%u pc=%u (ws2812 @%u) | SM1 claimed=%u pc=%u\r\n",
                (unsigned)pio_sm_is_claimed(pio2, 0), (unsigned)pio_sm_get_pc(pio2, 0),
                (unsigned)statusLed.Offset(),
                (unsigned)pio_sm_is_claimed(pio2, 1), (unsigned)pio_sm_get_pc(pio2, 1));

  audio_ready = true;
  dbg_printf("%s\r\n", window_line);          // deferred boot line (see fix note)
  // Crash record from the previous run (isr_hardfault): PC + fault status of
  // the fault that reset us. One line, then the magic is cleared so it prints
  // only once — the counters and streak stay (sticky safe mode) until
  // `crash clear`. Resolve the PC: arm-none-eabi-addr2line -e firmware.elf <pc>.
  if (g_crashv2.magic == kCrashMagicV2) {
    Serial.printf("CRASH: core=%lu pc=0x%08lx lr=0x%08lx psr=0x%08lx "
                  "cfsr=0x%08lx hfsr=0x%08lx mmfar=0x%08lx bfar=0x%08lx "
                  "r0=0x%08lx r1=0x%08lx r2=0x%08lx r3=0x%08lx r12=0x%08lx\r\n",
                  (unsigned long)g_crashv2.core,
                  (unsigned long)g_crashv2.pc, (unsigned long)g_crashv2.lr,
                  (unsigned long)g_crashv2.psr,
                  (unsigned long)g_crashv2.cfsr, (unsigned long)g_crashv2.hfsr,
                  (unsigned long)g_crashv2.mmfar, (unsigned long)g_crashv2.bfar,
                  (unsigned long)g_crashv2.r0, (unsigned long)g_crashv2.r1,
                  (unsigned long)g_crashv2.r2, (unsigned long)g_crashv2.r3,
                  (unsigned long)g_crashv2.r12);
    g_crashv2.magic = 0;
    __dmb();
  }
  Serial.printf("boot: reason=%lu (0 cold | 1 POR/hardware | 2 fault | "
                "3 watchdog-in-loop | 4 watchdog-in-setup | 5 clean-reboot) "
                "wd=0x%02lx unhealthy_boots=%lu fault_streak=%lu%s\r\n",
                (unsigned long)g_crashv2.reset_reason,
                (unsigned long)g_crashv2.wd_reason,
                (unsigned long)g_crashv2.boot_count,
                (unsigned long)g_crashv2.fault_streak,
                g_crashv2.safe_mode ? " SAFE-MODE(240 MHz) - `crash clear` to re-arm" : "");
#if RACK_HANG_DEBUG
  if (g_hang0 || g_hang1)
    Serial.printf("CRUMB: core-0 last stage %lu, core-1 stage %lu (from reset)\r\n",
                  (unsigned long)g_hang0, (unsigned long)g_hang1);
#endif
  // 2 s core-0 watchdog: fed at the end of every loop() pass. A hang reboots
  // the board instead of leaving it frozen (USB enumerated but dead).
  watchdog_enable(2000, false);
  // Boot-done markers (survive soft/watchdog resets; POR clears them). The
  // NEXT setup reads them before watchdog_enable() to classify the reset that
  // just happened. Clear SETUP_ACTIVE: from here on a reset means a loop hang.
  watchdog_hw->scratch[4] = 0xB007CAFEu;   // reached end of setup
  watchdog_hw->scratch[6] = 0;             // setup no longer active
#if RACK_ENABLE_USB_HOST
  dbg_printf("%s up (rack boot module, %lu Hz / %lu-sample blocks, budget %lu us, USB MIDI device + PIO-USB host; CC 32 or serial 'switch' swaps modules; CC 33/34/35 = mod engine source/param/value; console: help)\r\n",
                boot.name,
                (unsigned long)boot.sample_rate,
                (unsigned long)boot.block_size,
                (unsigned long)boot.budget_us);
#else
  dbg_printf("%s up (rack boot module, %lu Hz / %lu-sample blocks, budget %lu us, USB MIDI device; CC 32 or serial 'switch' swaps modules; CC 33/34/35 = mod engine source/param/value; console: help)\r\n",
                boot.name,
                (unsigned long)boot.sample_rate,
                (unsigned long)boot.block_size,
                (unsigned long)boot.budget_us);
#endif
}

// ===================== non-blocking diagnostics TX (core 0) =====================
// 2 KiB ring, drained by dbg_pump() with Serial.availableForWrite(). Same
// discipline as the UI-link TX: never spin on the USB CDC. If no host is
// reading, the ring fills and frames are dropped (counted) — audio first.
static const uint16_t kDbgRingSize = 2048;
static uint8_t g_dbg_ring[kDbgRingSize];
static uint16_t g_dbg_head = 0, g_dbg_tail = 0, g_dbg_count = 0;
static uint32_t g_dbg_dropped = 0;

void dbg_printf(const char* fmt, ...) {
  char line[400];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(line) - 1) n = (int)sizeof(line) - 1;
  if ((size_t)g_dbg_count + (size_t)n > kDbgRingSize) {
    ++g_dbg_dropped;                 // no reader: drop rather than block
    return;
  }
  for (int i = 0; i < n; ++i) {
    g_dbg_ring[g_dbg_head] = (uint8_t)line[i];
    g_dbg_head = (uint16_t)((g_dbg_head + 1) % kDbgRingSize);
  }
  g_dbg_count = (uint16_t)(g_dbg_count + n);
}

static void dbg_pump() {
  while (g_dbg_count > 0) {
    int space = Serial.availableForWrite();
    if (space <= 8) return;
    space -= 8;                      // never fill the CDC FIFO exactly
    uint16_t run = (g_dbg_tail + g_dbg_count > kDbgRingSize)
        ? (uint16_t)(kDbgRingSize - g_dbg_tail)
        : g_dbg_count;
    uint16_t n = (run < (uint16_t)space) ? run : (uint16_t)space;
    Serial.write(&g_dbg_ring[g_dbg_tail], n);
    g_dbg_tail = (uint16_t)((g_dbg_tail + n) % kDbgRingSize);
    g_dbg_count = (uint16_t)(g_dbg_count - n);
  }
}

void loop() {
  const uint32_t loop_t0 = time_us_32();
  // T9: a boot that survives 5 s of loop is healthy — clear the unhealthy-
  // boot counter so a NORMAL reboot does not accumulate it (fault_streak
  // stays sticky until `crash clear`, keeping SAFE MODE engaged if a clock
  // config keeps faulting).
  static bool t9_healthy = false;
  if (!t9_healthy && millis() > 5000u) {
    t9_healthy = true;
    g_crashv2.boot_count = 0;
    __dmb();
  }
  // --- I2S reconfig requests from a running swap / chain transition (core 1
  // waits bounded on either) ---
  serviceSwapI2s();
  serviceChainI2s();

  // T8 dual-core chain: run clouds B as early as possible on core 0. This is
  // a no-op unless a chain is engaged and the pipeline is enabled; the rest
  // of core-0's work (USB/UI/console) runs after it.
  chain_b_worker();

  // Drain queued diagnostics without blocking the USB CDC.
  dbg_pump();

#if RACK_ENABLE_USB_HOST
  // --- USB host: service the host stack, then poll host MIDI ---
  USBHost.task();
  pollHostMidi();
#endif

  // --- USB MIDI device: drain all pending 4-byte event packets ---
  uint8_t pkt[4];
  while (usb_midi.readPacket(pkt)) {
    midiHandleEventPacket(pkt);
  }

  // --- T4 DIN MIDI: drain the UART1 ring -> parser -> the same funnel ---
  // IRQ (core 0) keeps the 256 B ring fed; even at the 31250-baud maximum of
  // 3125 B/s this needs a drain every ~82 ms, and loop() iterates far faster.
  din_pump();

  // --- T5 UI-MCU link: drain the UART0 ring -> frame parser -> dispatch ---
  // Same IRQ+ring pattern as DIN; with the C3 absent this is one ring-empty
  // check per loop().
  ui_link_pump();

  // --- T4a preset autosave: push the serialized blob to the C3 flash bank
  // once the state has settled (3 s after the last persistent change) ---
  preset_autosave_poll();

  // --- T1 hardware PLAY button: debounced poll (GP2) ---
  button_poll();

  // --- T2 piezo: drain the ADC ring (GP26) ---
  rack_piezo::poll();

  // --- serial console (non-blocking; T2/T3/T4 table) ---
  pumpConsole();

  // --- deferred swap print (stats set by core 1, printed here exactly once) ---
  if (g_swap_report_ready) {
    g_swap_report_ready = false;
    ++g_swap_count;    // T4: the `status` swaps counter
    const RackModuleInfo& m = kRackModules[g_swap_to];
    if (g_swap_i2s_ok) {
      dbg_printf("rack: switched to %s in %lu ms (window %lu bytes)\r\n",
                    m.name, (unsigned long)g_swap_duration_ms,
                    (unsigned long)g_swap_copied_bytes);
    } else {
      dbg_printf("rack: SWITCH FAULT: %s swap failed in %lu ms "
                    "(window %lu bytes, i2s reconfig/copy rejected - audio muted)\r\n",
                    m.name, (unsigned long)g_swap_duration_ms,
                    (unsigned long)g_swap_copied_bytes);
    }
  }

  // --- deferred chain-transition print (stats set by core 1, printed once;
  // formats per the T6 brief: `rack: chain <a>-><b> engaged (<ms> ms)` /
  // `rack: chain off (<ms> ms)`) ---
  if (g_chain_report_ready) {
    g_chain_report_ready = false;
    const uint8_t kind = g_chain_report_kind;
    if (kind == 1) {
      if (g_chain_report_ok) {
        dbg_printf("rack: chain %s->%s engaged (%lu ms, %lu window bytes)\r\n",
                      kRackModules[g_chain.a].name,
                      kRackModules[g_chain.b].name,
                      (unsigned long)g_chain_report_ms,
                      (unsigned long)g_chain_report_bytes);
      } else {
        dbg_printf("rack: CHAIN FAULT: %s->%s engage failed in %lu ms "
                      "(window copy/i2s rejected - single module kept)\r\n",
                      kRackModules[g_chain.a].name,
                      kRackModules[g_chain.b].name,
                      (unsigned long)g_chain_report_ms);
      }
    } else if (kind == 2) {
      dbg_printf("rack: chain A (%s) re-Init'ed fresh in %lu ms\r\n",
                    kRackModules[g_chain.a].name,
                    (unsigned long)g_chain_report_ms);
    } else {
      if (g_chain_report_ok) {
        dbg_printf("rack: chain off (%lu ms)\r\n",
                      (unsigned long)g_chain_report_ms);
      } else {
        dbg_printf("rack: CHAIN FAULT: chain off failed in %lu ms "
                      "(i2s reconfig rejected - audio muted)\r\n",
                      (unsigned long)g_chain_report_ms);
      }
    }
  }

  // Status LED: dark when idle, amber while a note is held, module meters
  // blended in, green blinks on controller mount. Blocks ~1 ms per update
  // for the WS2812 reset gap; no impact on core 1 audio.
  statusLed.Poll(shared_gate, rack_meter_a, rack_meter_b);

  static uint32_t last_report = 0;
  uint32_t now = millis();

#if RACK_ENABLE_USB_HOST
  // --- USB host re-enumeration watchdog ---
  // TinyUSB never retries a failed/lost enumeration. If the controller is
  // electrically present (bus idle) but nothing is mounted, force the root
  // port to "disconnected" so the 1 ms frame loop re-fires the CONNECT event.
  // NOTE: deliberately no line-state read here — the E9 workaround inside
  // pio_usb_bus_get_line_state() briefly disables the pad input, and a
  // loop-side call (core 0) races the SOF alarm ISR (core 1) mid-transaction.
  static uint32_t last_host_watchdog_ms = 0;
  if (now - last_host_watchdog_ms >= 4000UL) {
    last_host_watchdog_ms = now;
    if (pio_host_connected() && !host_device_mounted) {
      Serial.println("USB host watchdog: connected but nothing mounted - forcing re-enumeration");
      pio_host_force_reconnect();
    }
  }
#endif

  // --- stuck-gate watchdog (defensive; identical in all four ports) ---
  // A lost note-off would leave the module gated forever. Forcing the gate
  // off ends the incident; this logs once per incident.
  if (note_stack.size > 0 && shared_gate && now - last_note_event_ms > 30000UL) {
    shared_gate = false;
    note_stack.clear();
    last_note_event_ms = now;
    Serial.println("WARNING: gate stuck >30s (lost note-off?) - forced off, note stack cleared");
  }

  // One-line diagnostics report (proven elements-style format, uniform
  // across modules): worst module Process + per-module budget + I2S stall
  // counter + in/out RMS meters (max-since-report, x1000) + note counter +
  // PIO-USB host tail + active module name + the T3 mod-engine tail
  // (modeng=... — see mod_engine.h: clock source+BPM / MIDI tick rate, EU1
  // tick odometer, LFO1 phase byte, the engine's own mod_us_max for the
  // 50 us gate, and one state char per source). Budget arithmetic is
  // per-module (registry: elements keeps its divide-first 496; the others
  // multiply-first).
  // T4: `telemetry off` gates the PRINT ONLY — the cadence and every counter
  // reset below stay unconditional, so re-enabling resumes with identical
  // max-since-report semantics and the 2 s format is untouched.
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    const RackModuleInfo& m = kRackModules[g_active_module];
    static char modeng_tail[64];
    mod_engine_report_tail(modeng_tail, sizeof(modeng_tail));
    // T6: the module field extends honestly to the chain (e.g.
    // mod=elements>clouds); proc_us_max covers A+route+B while chained and
    // the budget is the chain grid's 750 us (incl. the mod engine's u:).
    char mod_field[64];
    uint32_t budget;
    if (g_chain.active) {
      // T8 pipeline diagnostics: bm = core-0 clouds-B cost, c0 = core-0 loop
      // period, u = B-not-ready underruns (all max/sum since the last print).
      snprintf(mod_field, sizeof(mod_field), "%s>%s bm=%u c0=%u u=%lu",
               kRackModules[g_chain.a].name, kRackModules[g_chain.b].name,
               (unsigned)g_chain_b_us_max, (unsigned)g_core0_loop_max_us,
               (unsigned long)g_chain_pipe_underruns);
      budget = kChainBudgetUs;
    } else {
      snprintf(mod_field, sizeof(mod_field), "%s", m.name);
      budget = m.budget_us;
    }
    // T9 oc bench: continuous XIP integrity check while a clock override is
    // active. Intermittent flash corruption only shows up over a soak; the
    // one-shot FNV at switch time cannot catch it. Reads only — core 1 runs
    // from the SRAM window, so XIP fetches never disturb the audio path.
    char oc_tail[64] = "";
    if (g_oc_verify) {
      const RackWindowSection& s = rack_window_section(RACK_BOOT_MODULE);
      const uint32_t fnv =
          rack_checksum32(s.flash_start, (size_t)(s.flash_end - s.flash_start));
      snprintf(oc_tail, sizeof(oc_tail), " | oc: %lu kHz fnv=%s",
               (unsigned long)(clock_get_hz(clk_sys) / 1000u),
               fnv == g_window_src_sum ? "OK" : "MISMATCH");
    }
    if (g_telemetry_enabled) {
      // TEMP PROBE (plaits ping-path diagnosis): pulses consumed since the
      // last report + the active articulation bits (bit0 gate_resp, bit1
      // velocity_resp, bit2 patched). Resets the counter on print.
      char plaits_tail[24] = "";
      if (g_active_module == RACK_MODULE_PLAITS) {
        snprintf(plaits_tail, sizeof(plaits_tail), "tg=%lu go=%u",
                 (unsigned long)rack_modules::mod_plaits::s->probe_trigs,
                 rack_modules::mod_plaits::s->probe_opts);
        rack_modules::mod_plaits::s->probe_trigs = 0;
      }
      // STAGE PROFILE (l8 bottlenecks): internal-64-block maxima in us,
      // params/voices/delay/lfo/master; reset on print.
      char lira8_tail[64] = "";
      if (g_active_module == RACK_MODULE_LIRA8) {
        uint8_t n = 0;
        for (uint8_t v = 0; v < 8; ++v)
          if (rack_modules::mod_lira8::s->sensor_mask & (1u << v)) ++n;
        l8pd::Lira8Engine& e = rack_modules::mod_lira8::s->eng;
        snprintf(lira8_tail, sizeof(lira8_tail),
                 "sens=%u st=%lu/%lu/%lu/%lu/%lu", n,
                 (unsigned long)e.dbg_params, (unsigned long)e.dbg_voices,
                 (unsigned long)e.dbg_delay, (unsigned long)e.dbg_lfo,
                 (unsigned long)e.dbg_master);
        e.dbg_params = e.dbg_voices = e.dbg_delay = e.dbg_lfo = e.dbg_master = 0;
      }
      // T2 piezo: peak since last report (raw 0-4095) + measured rate.
      const uint16_t pz_peak = rack_piezo::report_peak();
      const uint32_t pz_rate = rack_piezo::report_sample_rate();
#if RACK_ENABLE_USB_HOST
      dbg_printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | in=%u out=%u notes=%u btn=%u | pz=%u pzr=%lu | host: conn=%u st=%u dev=%u midi=%u frame=%u | mod=%s | %s | %s%s\r\n",
                    proc_us_max,
                    (unsigned)budget,
                    stalls, stalls - last_stall_total,
                    (unsigned)(rack_meter_a * 1000.0f),
                    (unsigned)(rack_meter_b * 1000.0f),
                    note_on_count,
                    button_down_count,
                    pz_peak,
                    (unsigned long)pz_rate,
                    (unsigned)pio_host_connected(),
                    (unsigned)pio_host_line_state(),
                    (unsigned)host_device_mounted,
                    (unsigned)host_midi_idx_count,
                    (unsigned)pio_usb_host_get_frame_number(),
                    mod_field,
                    modeng_tail,
                    plaits_tail,
                    oc_tail);
#else
      dbg_printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | in=%u out=%u notes=%u btn=%u | pz=%u pzr=%lu | mod=%s | %s | %s%s\r\n",
                    proc_us_max,
                    (unsigned)budget,
                    stalls, stalls - last_stall_total,
                    (unsigned)(rack_meter_a * 1000.0f),
                    (unsigned)(rack_meter_b * 1000.0f),
                    note_on_count,
                    button_down_count,
                    pz_peak,
                    (unsigned long)pz_rate,
                    mod_field,
                    modeng_tail,
                    g_active_module == RACK_MODULE_PLAITS ? plaits_tail
                        : (g_active_module == RACK_MODULE_LIRA8 ? lira8_tail : ""),
                    oc_tail);
#endif
    }
    last_stall_total = stalls;
    proc_us_max = 0;
    g_mod_engine.mod_us_max = 0;    // max-since-report (engine block cost)
    g_mod_engine.telem_midi_ticks = 0;
    rack_meter_a = 0.0f;            // max-since-report meters (see adapters)
    rack_meter_b = 0.0f;
    g_chain_b_us_max = 0;           // core-0 pipeline diagnostics
    g_core0_loop_max_us = 0;
    g_chain_pipe_underruns = 0;
  }
  const uint32_t loop_dt = time_us_32() - loop_t0;
  if (loop_dt > g_core0_loop_max_us) g_core0_loop_max_us = loop_dt;
  // Clean loop pass: clear crumbs so a surviving value marks the hang stage.
  crumb(0);
  crumb1(0);
  watchdog_update();
}
