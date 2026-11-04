// Rack RP2350 — all four accepted module ports (Elements, Clouds, Plaits
// 98-engine, Rings MiniElements) in ONE firmware with LIVE MODULE SWITCHING
// (T2), on the RP2350 nano + PCM5102A, using the proven elements-rp2350
// architecture:
//   core 0 = USB MIDI device (CDC + USB-MIDI) + PIO-USB host MIDI + serial
//            console + status LED + diagnostics;
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

#include "pio_usb.h"             // Pico-PIO-USB (vendored in lib/pio_usb/)

#include "status_led.h"          // WS2812 status LED on GP22 (before midi.h)
#include "midi.h"                // shared parser + telemetry externs + hooks
#include "rack_window.h"         // SRAM window copy API
#include "rack_arena.h"          // T2 module-state arenas (decls)

// PIO-USB root-port accessors for the re-enumeration watchdog (C side,
// usb_host_watchdog.c).
extern "C" {
bool pio_host_line_idle(void);
bool pio_host_connected(void);
void pio_host_force_reconnect(void);
uint8_t pio_host_line_state(void);
}

#if !defined(RACK_BOOT_MODULE) || RACK_BOOT_MODULE < 0 || RACK_BOOT_MODULE > 3
#error "RACK_BOOT_MODULE must be defined per env as 0 (elements), 1 (clouds), 2 (plaits) or 3 (rings)"
#endif

// The four module adapters — ALL FOUR compile in every env (their state is
// arena-placed; their hooks are namespaced rack_modules::mod_<module>).
#include "module_elements.h"
#include "module_clouds.h"
#include "module_plaits.h"
#include "module_rings.h"

#include "module_registry.h"

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
     rack_modules::mod_elements::kModDestinations,
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
     rack_modules::mod_clouds::kModDestinations,
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
     rack_modules::mod_plaits::kModDestinations,
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
     rack_modules::mod_rings::kModDestinations,
     rack_modules::mod_rings::init,
     rack_modules::mod_rings::render,
     rack_modules::mod_rings::note_on,
     rack_modules::mod_rings::note_off,
     rack_modules::mod_rings::control_change},
};

// ===================== shared definitions =====================
static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);
volatile float shared_note = 48.0f;      // idle note (C3); overwritten by MIDI
volatile bool  shared_gate = false;
volatile uint32_t note_on_count = 0;
volatile float rack_meter_a = 0.0f;
volatile float rack_meter_b = 0.0f;
volatile uint32_t proc_us_max = 0;
volatile uint32_t i2s_stall_count = 0;
volatile float g_render_gain = 1.0f;     // swap-mute ramp (see midi.h)

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
// read by core 0's registry dispatchers + report.
static volatile uint8_t g_active_module = RACK_BOOT_MODULE;
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

// --- USB MIDI device (core 0) ---
Adafruit_USBD_MIDI usb_midi;

// --- USB MIDI host (PIO-USB on rhport 1, core 0) ---
// Pattern from the bundled DualRole/serial_host_bridge example (device =
// native rhport 0, host = PIO-USB rhport 1). The host task + MIDI poll run
// in core 0's free-running loop(); core 1 renders audio.
Adafruit_USBH_Host USBHost;

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
  kRackModules[g_active_module].control_change(cc, val);
}

// CC 32 / console entry: queue a module switch. Values > 3 and == the active
// module are ignored (no self-swap; the swap print is the ack).
void rack_request_switch(uint8_t module_id) {
  if (module_id >= RACK_MODULE_COUNT) return;
  if (module_id == g_active_module) return;
  g_pending_module = static_cast<int8_t>(module_id);
}

// ================= core 1: audio + live swap =================
// Alarm pool created on CORE 1 (see elements-rp2350/src/main.cpp: the
// PIO-USB SOF alarm ISR must run on core 1, isolated from core 0's I2S DMA
// and TinyUSB device interrupts). Core 0's host init waits for the pool
// flag, then passes it via pio_cfg.alarm_pool.
static alarm_pool_t* g_host_alarm_pool = nullptr;
static volatile bool g_host_alarm_pool_ready = false;

void setup1() {
  // Must run BEFORE the audio_ready wait: core 0's host init (in setup(),
  // before i2s.begin) consumes this pool.
  g_host_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(1);
  g_host_alarm_pool_ready = true;

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

void loop1() {
  // Pending swap first: consumed at the TOP of the render loop (strictly
  // between two Process() calls), max one swap per call.
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
  kRackModules[g_active_module].render();
}

// ================= core 0: control =================
// --- serial console (T2 scope; T4 expands the same command table) ---
struct ConsoleCommand {
  const char* name;
  void (*fn)(const char* args);   // args = everything after the name
};

static void cmd_switch(const char* args);
static void cmd_module(const char* args);
static void cmd_help(const char* args);

static const ConsoleCommand kConsoleCommands[] = {
    {"switch", cmd_switch},
    {"module", cmd_module},
    {"help",   cmd_help},
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
  if (static_cast<uint8_t>(id) == g_active_module) {
    Serial.printf("rack: %s already active\r\n", kRackModules[id].name);
    return;
  }
  g_pending_module = id;
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
  Serial.printf("rack: mod destinations: %s\r\n",
                m.mod_destinations[0] ? m.mod_destinations : "(T3 pending)");
}

static void cmd_help(const char* args) {
  (void)args;
  Serial.println("rack console commands:");
  for (uint8_t i = 0; i < kConsoleCommandCount; ++i) {
    Serial.printf("  %s\r\n", kConsoleCommands[i].name);
  }
  Serial.println("  switch <0-3|name>  live module swap (also MIDI CC 32, value 0-3)");
  Serial.println("  module             print active module + cc map");
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
  kRackModules[RACK_BOOT_MODULE].init();   // registry: boot module, arena-placed
  statusLed.Init();  // 3 white self-test flashes, then dark

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
  Serial.printf("%s up (rack boot module, %lu Hz / %lu-sample blocks, budget %lu us, USB MIDI device + PIO-USB host; CC 32 or serial 'switch' swaps modules; console: help)\r\n",
                boot.name,
                (unsigned long)boot.sample_rate,
                (unsigned long)boot.block_size,
                (unsigned long)boot.budget_us);
}

void loop() {
  // --- I2S reconfig requests from a running swap (core 1 waits bounded) ---
  serviceSwapI2s();

  // --- USB host: service the host stack, then poll host MIDI ---
  USBHost.task();
  pollHostMidi();

  // --- USB MIDI device: drain all pending 4-byte event packets ---
  uint8_t pkt[4];
  while (usb_midi.readPacket(pkt)) {
    midiHandleEventPacket(pkt);
  }

  // --- serial console (non-blocking; switch/module/help in T2) ---
  pumpConsole();

  // --- deferred swap print (stats set by core 1, printed here exactly once) ---
  if (g_swap_report_ready) {
    g_swap_report_ready = false;
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

  // Status LED: dark when idle, amber while a note is held, module meters
  // blended in, green blinks on controller mount. Blocks ~1 ms per update
  // for the WS2812 reset gap; no impact on core 1 audio.
  statusLed.Poll(shared_gate, rack_meter_a, rack_meter_b);

  static uint32_t last_report = 0;
  uint32_t now = millis();

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
  // PIO-USB host tail + active module name. Budget arithmetic is per-module
  // (registry: elements keeps its divide-first 496; the others
  // multiply-first).
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    const RackModuleInfo& m = kRackModules[g_active_module];
    Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | in=%u out=%u notes=%u | host: conn=%u st=%u dev=%u midi=%u frame=%u | mod=%s\r\n",
                  proc_us_max,
                  m.budget_us,
                  stalls, stalls - last_stall_total,
                  (unsigned)(rack_meter_a * 1000.0f),
                  (unsigned)(rack_meter_b * 1000.0f),
                  note_on_count,
                  (unsigned)pio_host_connected(),
                  (unsigned)pio_host_line_state(),
                  (unsigned)host_device_mounted,
                  (unsigned)host_midi_idx_count,
                  (unsigned)pio_usb_host_get_frame_number(),
                  m.name);
    last_stall_total = stalls;
    proc_us_max = 0;
    rack_meter_a = 0.0f;            // max-since-report meters (see adapters)
    rack_meter_b = 0.0f;
  }
}
