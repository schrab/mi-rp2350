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
// RACK_BOOT_MODULE (0 elements / 1 clouds / 2 plaits / 3 rings) selects the
// BOOT module only: setup() copies that module's flash image into the window
// as its ABSOLUTE FIRST action (crt0-like) and registry-inits it; after that
// every env is identical and any module can be swapped in at runtime.
//
// Live swap (core 1, at a block boundary, max one per loop1 call):
//   CC 32 (bank select) value 0-3 or serial `switch <id|name>` -> core 0
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
// SM0; I2S lands on PIO2 SM1. Reordering panics at boot. f_cpu must be a
// multiple of 12 MHz (240 MHz) or PIO-USB timing breaks.
#include <Arduino.h>
#include <I2S.h>

// Give core 1 its own 8 KB stack instead of the default 4 KB/4 KB split with
// core 0 (the standalone ports' core 0 overran the shared half at USB attach
// and corrupted core 1).
bool core1_separate_stack = true;
#include <Adafruit_TinyUSB.h>

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
#include "hardware/gpio.h"       // gpio_pull_up (DIN RX idle mark)

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

#if !defined(RACK_BOOT_MODULE) || RACK_BOOT_MODULE < 0 || RACK_BOOT_MODULE > 3
#error "RACK_BOOT_MODULE must be defined per env as 0 (elements), 1 (clouds), 2 (plaits) or 3 (rings)"
#endif

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
};

// ===================== shared definitions =====================
static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
#if RACK_ENABLE_USB_HOST
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15
#endif

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

// T2 arenas (rack_arena.h): the module state unions. .bss-resident (crt0
// zeroes them; each Init re-zeroes what it needs). alignas(8) for the
// placement-new alignment + the plaits BufferAllocator workspace.
alignas(8) uint8_t g_rack_object_arena[kRackObjectArenaBytes];
alignas(8) uint8_t g_rack_buffer_arena[kRackBufferArenaBytes];

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

// CHAINED CLOUDS BUFFER SPLIT (documented deviation): the buffer arena is
// exactly full for the elements/rings pairs (chain A carve 65,536 + clouds
// 131,072 = 196,608), so chained clouds runs large 98,304 B (= 32,768 B
// sample memory ch0 + 65,536 B FX workspace — the FX depth is IDENTICAL to
// the standalone) + small 32,768 B (sample memory ch1) = 0.512 s/channel
// sample memory vs the standalone's 1.024 s. Grain parameters are
// unaffected; only the maximum loop length / grain-window reach halves.
static const size_t kChainCloudsLargeBytes = 98304;
static const size_t kChainCloudsSmallBytes = 32768;
// Chain A slots carve the buffer arena from offset 0 (all three A adapters
// hard-carve there, same as single-module mode); clouds-B carves above the
// A carve. Per-A carve size (elements reverb / rings reverb 64 KB, plaits
// workspace 32 KB).
static const size_t kChainABufferBytes[RACK_MODULE_COUNT] = {
    65536, 0, 32768, 65536,   // elements, clouds(-), plaits, rings
};

// Chain render block buffers (24 x 2 floats x 2 = 384 B in .bss — the
// routing plumbing the brief sizes at ~192 B per direction).
static float chain_a_out_l[kChainBlockSize];
static float chain_a_out_r[kChainBlockSize];
static float chain_b_out_l[kChainBlockSize];
static float chain_b_out_r[kChainBlockSize];

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

// Drain the UART ring -> parser -> midiHandleEventPacket (THE funnel — same
// entry the USB device + PIO-USB host paths use; DIN F8/FA/FB/FC therefore
// hit the mod engine's sample-stamped clock bus with zero extra code).
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
  Serial.printf("USB device mounted daddr=%u\r\n", daddr);
}

void tuh_umount_cb(uint8_t daddr) {
  host_device_mounted = false;
  Serial.printf("USB device unmounted daddr=%u\r\n", daddr);
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
  Serial.printf("Host MIDI mounted idx=%u daddr=%u cables=%u\r\n", idx,
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
  Serial.printf("Host MIDI unmounted idx=%u\r\n", idx);
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
      Serial.printf("rack: chain: %s (A slot) fresh re-Init queued\r\n",
                    kRackModules[module_id].name);
      return;
    }
    if (module_id == g_chain.b) {
      Serial.println("rack: chain: B swap rejected (clouds is the only B in "
                     "T6; mode/quality via CC 23/24)");
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

// T6 chain CC intercept (declared in midi.h): while chained, CCs 23/24/65/66
// AND the clouds grain alias bank 40-48 drive the B slot (clouds) —
// everything else reaches the A slot through the normal registry dispatch.
// 23/24 collide with every chain-A module's map (elements geometry/
// brightness, plaits engine/aux, rings chord/-) — while chained they mean
// clouds mode/quality; the A slot's parameters stay reachable through the
// mod engine (destinations bind to A). The 40-48 band is translated to
// clouds' native 14-22 so controllers get the FULL clouds grain surface
// while chained, not just mode/quality/freeze/trig.
bool rack_chain_wants_cc(uint8_t cc) {
  return g_chain.active &&
         (cc == 23 || cc == 24 || cc == 65 || cc == 66 ||
          (cc >= kCloudsAliasCcBase && cc <= kCloudsAliasCcBase + 8));
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
  switch (cc) {
    case 23:
      // clouds-B playback-mode CLAMP (chain mode = granular/looping ONLY):
      // clouds' mode enum is 0 granular / 1 stretch / 2 looping / 3
      // spectral. 1 (stretch) and 3 (spectral) are REJECTED while chained —
      // stretch stays available in single-module mode, spectral is excluded
      // everywhere (T2). The adapter's apply-time clamp is belt-and-braces.
      if (val == 0 || val == 2) {
        rack_modules::mod_clouds::control_change(23, val);
      } else {
        Serial.printf("rack: chain: clouds-B mode %u rejected while chained "
                      "(granular 0 / looping 2 only)\r\n", val);
      }
      break;
    default:
      // 24 quality / 65 freeze / 66 trig: no collision with any chain-A map.
      rack_modules::mod_clouds::control_change(cc, val);
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
//   (c) INIT  — the new module placement-news its state into the OBJECT
//       arena, carves its buffers from the BUFFER arena (both are unions:
//       the old module's state is simply abandoned; factory defaults via the
//       adapter's Init()).
//   (d) I2S   — end() -> setBitsPerSample/setBuffers/setFrequency (per-module
//       native rate + bank) -> begin(); performed by CORE 0 via handshake
//       (see g_swap_i2s_request below) and the result is reported (the DAC is
//       muted regardless — XSMT held, silence bank looping).
//   (e) UNMUTE — linear gain ramp-up over kSwapRampSteps blocks on the NEW
//       module, then normal 1.0 rendering.
// One stats line is handed to core 0 (printed once, from loop()).

// I2S reconfig handshake (core 1 requests, core 0 performs): the I2S object's
// DMA transmit IRQ fires on CORE 0 (i2s.begin() ran there in setup; RP2350
// routes enabled IRQs to the enabling core). Tearing the manager down from
// core 1 races that ISR mid-flight — bench-verified hard fault (reports die,
// DMA clocks out freed buffers = noise burst, board needs a power cycle).
// Core 1: id -> request -> wait (bounded); core 0: end/config/begin -> result
// -> ack. Ordering: result is written before ack; request is cleared before
// ack. No other concurrent I2S access exists during the swap window.
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
  g_swap_i2s_result = ok;
  g_swap_i2s_request = false;
  g_swap_i2s_ack = true;
}
static void do_module_swap(uint8_t new_id) {
  const RackModuleInfo& old = kRackModules[g_active_module];
  const RackModuleInfo& nw = kRackModules[new_id];
  const uint32_t t0 = time_us_32();

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
  g_swap_i2s_request = true;
  const uint32_t i2s_t0 = time_us_32();
  while (!g_swap_i2s_ack && time_us_32() - i2s_t0 < 1000000UL) {
    delay(1);
  }
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
  g_chain_i2s_result = ok;
  g_chain_i2s_request = false;
  g_chain_i2s_ack = true;
}

// The CHAIN RENDER BLOCK (core 1, while g_chain.active): A.Process(24) ->
// ROUTE (A out/aux -> clouds FloatFrame in, gain 1.0) -> B.Process(24) ->
// B out -> I2S. proc_us_max times A + route + B HONESTLY (decision 6) — the
// adapters' chain hooks do not touch it. The mod engine ran BEFORE this at
// loop-top (its u: is measured separately in g_mod_engine.mod_us_max).
static void chain_render_block() {
  const uint32_t t0 = time_us_32();
  // A slot (the sound source; its hook applies pending slots + fills
  // perf/patch state exactly like its single-module render does).
  switch (g_chain.a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
    default:
      rack_modules::mod_rings::chain_render_a(
          chain_a_out_l, chain_a_out_r, kChainBlockSize);
      break;
  }
  // ROUTE + B (clouds — the effect stage; chain_render_b fills its
  // FloatFrame input from the routed buffers, Prepares, Processes, and
  // meters the pair).
  rack_modules::mod_clouds::chain_render_b(
      chain_a_out_l, chain_a_out_r, chain_b_out_l, chain_b_out_r,
      kChainBlockSize);
  const uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Write stage: the chain OUTPUT is B's output; g_render_gain (swap-mute
  // ramps) applies here exactly as in the single-module adapters. The
  // writes pace the loop: one iteration = one 24-frame block, blocks free
  // every 750 us; > 1400 us = stalled.
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
    g_chain_i2s_request = true;
    const uint32_t i2s_t0 = time_us_32();
    while (!g_chain_i2s_ack && time_us_32() - i2s_t0 < 1000000UL) {
      delay(1);
    }
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

  // (d) INIT both slots at the chain rate context. Object arena: new_a at
  // offset 0 (its single-module placement), clouds-B AFTER new_a's state; the
  // buffer arena: new_a's carve at 0 (the adapters' own), clouds-B above it
  // with the chained split.
  switch (new_a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::init_at(g_rack_object_arena);
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::init_chain_a(g_rack_object_arena);
      break;
    default:
      rack_modules::mod_rings::init_at(g_rack_object_arena);
      break;
  }
  {
    const size_t a_state = (new_a == RACK_MODULE_ELEMENTS)
        ? rack_modules::mod_elements::state_size()
        : (new_a == RACK_MODULE_PLAITS)
            ? rack_modules::mod_plaits::state_size()
            : rack_modules::mod_rings::state_size();
    uint8_t* b_obj = g_rack_object_arena + ((a_state + 7u) & ~(size_t)7u);
    uint8_t* b_buf =
        g_rack_buffer_arena + kChainABufferBytes[new_a];
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

  // (a) MUTE through the chain render + silence flush (chain bank).
  chain_ramp_down_chain();
  chain_flush_silence(kChainI2sBuffers, kChainI2sWords);

  // (b) I2S reconfigure to the A slot's native rate/bank (registry).
  g_chain_i2s_rate = kRackModules[a].sample_rate;
  g_chain_i2s_buffers = kRackModules[a].i2s_buffers;
  g_chain_i2s_words = kRackModules[a].i2s_words;
  g_chain_i2s_ack = false;
  g_chain_i2s_result = false;
  g_chain_i2s_request = true;
  const uint32_t i2s_t0 = time_us_32();
  while (!g_chain_i2s_ack && time_us_32() - i2s_t0 < 1000000UL) {
    delay(1);
  }
  const bool i2s_ok = g_chain_i2s_ack && g_chain_i2s_result;

  // (c) Re-Init the A slot fresh at its native context. Its hot image is
  // still resident at the A-slot VMA (the engage-time B copy went to the
  // B slot); re-copy defensively anyway — idempotent, covers any future A
  // whose image spans the B-slot region.
  rack_window_copy(a, nullptr, nullptr);
  switch (a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::init_at(g_rack_object_arena);
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::init_at(g_rack_object_arena);  // 48 kHz
      break;
    default:
      rack_modules::mod_rings::init_at(g_rack_object_arena);
      break;
  }
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
// NO window copy, NO I2S change (elements/clouds/rings are all 32k and the
// chain grid rate is unchanged even for a plaits A), just factory-fresh
// state; the routing re-patches automatically (persistent state).
static void do_chain_a_reinit() {
  const uint8_t a = g_chain.a;
  const uint32_t t0 = time_us_32();
  chain_ramp_down_chain();
  // Fresh state at the SAME chain rate context (plaits stays 32k chained).
  switch (a) {
    case RACK_MODULE_ELEMENTS:
      rack_modules::mod_elements::init_at(g_rack_object_arena);
      break;
    case RACK_MODULE_PLAITS:
      rack_modules::mod_plaits::init_chain_a(g_rack_object_arena);
      break;
    default:
      rack_modules::mod_rings::init_at(g_rack_object_arena);
      break;
  }
  proc_us_max = 0;
  rack_meter_a = 0.0f;
  rack_meter_b = 0.0f;
  preset_replay_module(a);       // T4a: restore observed CC values on A
  chain_ramp_up(chain_render_block);
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
static void cmd_panic(const char* args);
static void cmd_ui(const char* args);
static void cmd_uitest(const char* args);
static void cmd_uisend(const char* args);
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
    {"ui",        cmd_ui},
    {"uitest",    cmd_uitest},
    {"uisend",    cmd_uisend},
    {"panic",     cmd_panic},
    {"help",      cmd_help},
};
static const uint8_t kConsoleCommandCount =
    sizeof(kConsoleCommands) / sizeof(kConsoleCommands[0]);

static void cmd_switch(const char* args) {
  while (*args == ' ') ++args;
  if (*args == '\0') {
    Serial.println("usage: switch <0-3|elements|clouds|plaits|rings>");
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
    Serial.println("rack: unknown module (0 elements / 1 clouds / 2 plaits / 3 rings)");
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
// PAIR VALIDATION (the 3-pair table, user-amended 2026-11-05 — all X->clouds):
//   elements->clouds, plaits->clouds, rings->clouds
// Engagement additionally requires the CHAIN RESIDENCY invariants (both
// slots' hot images must fit the two-slot SRAM window, both states must fit
// the object arena, both buffer carves must fit the buffer arena) — checked
// here with the REAL linker-symbol sizes, so a reject line carries the
// exact numbers. Since T6b (tc_clouds dead-mode demotion 79,744 -> 56,992 B
// paid for the 0xE400 A slot; the object arena grew to 132 KiB) AND the
// user-approved plaits chain-tier (09-plaits-chain-tier.patch: tc_plaits
// 108,732 -> ~14.1 KB, voice path + engine 0 + lut_sine only), ALL THREE
// pairs pass the residency checks: elements->clouds, rings->clouds,
// plaits->clouds. The chained-plaits BUDGET (flash engines @ 32 kHz) is a
// separate gate — see the README bench sheet.
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
    Serial.println("rack: chain: unknown module (0 elements / 1 clouds / 2 plaits / 3 rings)");
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
    Serial.println("rack: chain: B must be clouds (T6 pair table: "
                   "elements|plaits|rings -> clouds)");
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
          : rack_modules::mod_rings::state_size();
  const size_t b_state = rack_modules::mod_clouds::state_size();
  const size_t obj_needed =
      ((a_state + 7u) & ~(size_t)7u) + b_state;
  if (obj_needed > kRackObjectArenaBytes) {
    Serial.printf("rack: chain REJECTED: object arena %u B needed (%s %u + "
                  "clouds %u) > %u B\r\n",
                  (unsigned)obj_needed, sa.name, (unsigned)a_state,
                  (unsigned)b_state, (unsigned)kRackObjectArenaBytes);
    return;
  }
  const size_t buf_needed = kChainABufferBytes[static_cast<uint8_t>(a)] +
                            kChainCloudsLargeBytes + kChainCloudsSmallBytes;
  if (buf_needed > kRackBufferArenaBytes) {
    Serial.printf("rack: chain REJECTED: buffer arena %u B needed > %u B\r\n",
                  (unsigned)buf_needed, (unsigned)kRackBufferArenaBytes);
    return;
  }
  g_pending_chain_a = static_cast<int8_t>(a);
  g_pending_chain_b = static_cast<int8_t>(RACK_MODULE_CLOUDS);
  g_pending_chain = 1;
  preset_mark_dirty();           // T4a: chain state is persistent
  Serial.printf("rack: chain %s->clouds validated (window %u+%u of %u B, "
                "objects %u of %u B, buffers %u of %u B) — engaging at the "
                "next block boundary\r\n",
                sa.name, (unsigned)a_bytes, (unsigned)b_bytes,
                (unsigned)window_bytes, (unsigned)obj_needed,
                (unsigned)kRackObjectArenaBytes, (unsigned)buf_needed,
                (unsigned)kRackBufferArenaBytes);
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
                "in T4), irq + 256B ring; 'dintest' injects a test stream\r\n",
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

// ui — T5 stage 1 link status: frame counters, CRC errors, last RTT. The
// rtt field is the RP2350's ECHO TURNAROUND (UI_CMD echo received -> echo
// written); the C3 measures the full wire RTT of its own echo probes.
static void cmd_ui(const char* args) {
  (void)args;
  Serial.printf("ui: tx=%lu rx=%lu crc_err=%lu rx_overflow=%lu rtt=%lu us "
                "(echo turnaround) | page=%u | uart0 @921600 8N1: TX GP%u "
                "(-> C3 RX), RX GP%u (<- C3 TX), irq + 512B ring\r\n",
                (unsigned long)g_ui_tx_frames,
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
  Serial.println("  switch <0-3|name>  live module swap (also MIDI CC 32, value 0-3)");
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

void setup() {
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

  // Boot diagnostic: confirm I2S did not steal the status LED's PIO2 SM0.
  // SM0's PC must stay inside the 4-instruction WS2812 program.
  Serial.printf("PIO2 SM0 claimed=%u pc=%u (ws2812 @%u) | SM1 claimed=%u pc=%u\r\n",
                (unsigned)pio_sm_is_claimed(pio2, 0), (unsigned)pio_sm_get_pc(pio2, 0),
                (unsigned)statusLed.Offset(),
                (unsigned)pio_sm_is_claimed(pio2, 1), (unsigned)pio_sm_get_pc(pio2, 1));

  audio_ready = true;
  Serial.println(window_line);                // deferred boot line (see fix note)
#if RACK_ENABLE_USB_HOST
  Serial.printf("%s up (rack boot module, %lu Hz / %lu-sample blocks, budget %lu us, USB MIDI device + PIO-USB host; CC 32 or serial 'switch' swaps modules; CC 33/34/35 = mod engine source/param/value; console: help)\r\n",
                boot.name,
                (unsigned long)boot.sample_rate,
                (unsigned long)boot.block_size,
                (unsigned long)boot.budget_us);
#else
  Serial.printf("%s up (rack boot module, %lu Hz / %lu-sample blocks, budget %lu us, USB MIDI device; CC 32 or serial 'switch' swaps modules; CC 33/34/35 = mod engine source/param/value; console: help)\r\n",
                boot.name,
                (unsigned long)boot.sample_rate,
                (unsigned long)boot.block_size,
                (unsigned long)boot.budget_us);
#endif
}

void loop() {
  // --- I2S reconfig requests from a running swap / chain transition (core 1
  // waits bounded on either) ---
  serviceSwapI2s();
  serviceChainI2s();

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

  // --- serial console (non-blocking; T2/T3/T4 table) ---
  pumpConsole();

  // --- deferred swap print (stats set by core 1, printed here exactly once) ---
  if (g_swap_report_ready) {
    g_swap_report_ready = false;
    ++g_swap_count;    // T4: the `status` swaps counter
    const RackModuleInfo& m = kRackModules[g_swap_to];
    if (g_swap_i2s_ok) {
      Serial.printf("rack: switched to %s in %lu ms (window %lu bytes)\r\n",
                    m.name, (unsigned long)g_swap_duration_ms,
                    (unsigned long)g_swap_copied_bytes);
    } else {
      Serial.printf("rack: SWITCH FAULT: %s swap failed in %lu ms "
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
        Serial.printf("rack: chain %s->%s engaged (%lu ms, %lu window bytes)\r\n",
                      kRackModules[g_chain.a].name,
                      kRackModules[g_chain.b].name,
                      (unsigned long)g_chain_report_ms,
                      (unsigned long)g_chain_report_bytes);
      } else {
        Serial.printf("rack: CHAIN FAULT: %s->%s engage failed in %lu ms "
                      "(window copy/i2s rejected - single module kept)\r\n",
                      kRackModules[g_chain.a].name,
                      kRackModules[g_chain.b].name,
                      (unsigned long)g_chain_report_ms);
      }
    } else if (kind == 2) {
      Serial.printf("rack: chain A (%s) re-Init'ed fresh in %lu ms\r\n",
                    kRackModules[g_chain.a].name,
                    (unsigned long)g_chain_report_ms);
    } else {
      if (g_chain_report_ok) {
        Serial.printf("rack: chain off (%lu ms)\r\n",
                      (unsigned long)g_chain_report_ms);
      } else {
        Serial.printf("rack: CHAIN FAULT: chain off failed in %lu ms "
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
  if (shared_gate && now - last_note_event_ms > 30000UL) {
    shared_gate = false;
    note_stack.clear();
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
    char mod_field[28];
    uint32_t budget;
    if (g_chain.active) {
      snprintf(mod_field, sizeof(mod_field), "%s>%s",
               kRackModules[g_chain.a].name, kRackModules[g_chain.b].name);
      budget = kChainBudgetUs;
    } else {
      snprintf(mod_field, sizeof(mod_field), "%s", m.name);
      budget = m.budget_us;
    }
    if (g_telemetry_enabled) {
#if RACK_ENABLE_USB_HOST
      Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | in=%u out=%u notes=%u | host: conn=%u st=%u dev=%u midi=%u frame=%u | mod=%s | %s\r\n",
                    proc_us_max,
                    (unsigned)budget,
                    stalls, stalls - last_stall_total,
                    (unsigned)(rack_meter_a * 1000.0f),
                    (unsigned)(rack_meter_b * 1000.0f),
                    note_on_count,
                    (unsigned)pio_host_connected(),
                    (unsigned)pio_host_line_state(),
                    (unsigned)host_device_mounted,
                    (unsigned)host_midi_idx_count,
                    (unsigned)pio_usb_host_get_frame_number(),
                    mod_field,
                    modeng_tail);
#else
      Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | in=%u out=%u notes=%u | mod=%s | %s\r\n",
                    proc_us_max,
                    (unsigned)budget,
                    stalls, stalls - last_stall_total,
                    (unsigned)(rack_meter_a * 1000.0f),
                    (unsigned)(rack_meter_b * 1000.0f),
                    note_on_count,
                    mod_field,
                    modeng_tail);
#endif
    }
    last_stall_total = stalls;
    proc_us_max = 0;
    g_mod_engine.mod_us_max = 0;    // max-since-report (engine block cost)
    g_mod_engine.telem_midi_ticks = 0;
    rack_meter_a = 0.0f;            // max-since-report meters (see adapters)
    rack_meter_b = 0.0f;
  }
}
