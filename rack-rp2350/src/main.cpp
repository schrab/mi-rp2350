// Rack RP2350 — all four accepted module ports (Elements, Clouds, Plaits
// 98-engine, Rings MiniElements) in ONE firmware, on the RP2350 nano +
// PCM5102A, using the proven elements-rp2350 architecture:
//   core 0 = USB MIDI device (CDC + USB-MIDI) + PIO-USB host MIDI + status
//            LED + diagnostics;
//   core 1 = audio render of the SELECTED module at its native rate/block
//            size into the I2S DAC.
//
// T1 boot-select: RACK_BOOT_MODULE (0 elements / 1 clouds / 2 plaits /
// 3 rings) is a per-env compile flag — exactly ONE module adapter
// (src/module_*.h) compiles, so only the selected module instantiates state;
// ALL FOUR modules' code is linked (their .tc_* hot sections live in the
// reserved flash region, see memmap_rack.ld). setup() copies the selected
// module's hot section flash -> SRAM window BEFORE any module code runs
// (rack_window.h); T2 replaces the compile-time select with a runtime swap
// that re-uses the same copy path.
//
// SRAM law: the module hot code runs from the per-env SRAM window (origin =
// ORIGIN(RACK_WINDOW) of this env's memmap_rack_<module>.ld, copied at
// boot), the platform's own .time_critical sections (pio_usb,
// framework) stay in .data via the core linker rule — the module adapters
// call into the window exactly like the standalone ports called into their
// crt0-copied .time_critical sections.
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

// The four module adapters. Exactly ONE compiles (its RACK_BOOT_MODULE ==
// n guard); it defines the module_* hooks, the per-module state, the I2S
// config constants and the report budget the rest of this file references.
#include "module_elements.h"
#include "module_clouds.h"
#include "module_plaits.h"
#include "module_rings.h"

static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;
static const uint8_t kPinHostDP = 14; // PIO-USB host: D+ GP14, D- GP15

// --- definitions for the externs declared in midi.h ---
I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);
volatile float shared_note = 48.0f;      // idle note (C3); overwritten by MIDI
volatile bool  shared_gate = false;
volatile uint32_t note_on_count = 0;
volatile float rack_meter_a = 0.0f;
volatile float rack_meter_b = 0.0f;
volatile uint32_t proc_us_max = 0;
volatile uint32_t i2s_stall_count = 0;

// Status LED instance — defined here (status_led.h holds only the class).
StatusLed statusLed;

static volatile bool audio_ready = false;

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

// Invoked when the MIDI interface is unmounted (unplugged / bus reset).
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

// ================= core 1: audio =================
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

void loop1() {
  module_render();   // selected adapter: params -> Process -> meters -> I2S
}

// ================= core 0: control =================
void setup() {
  // --- SRAM window: copy the boot module's hot section flash -> SRAM ---
  // MUST run before ANY module code executes (module_init() calls tc_*
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
  // AFTER host+I2S init). The line is composed now and printed after the
  // detach/attach re-enumeration guard below, when enumeration is settled.
  // Conditional ERROR prints stay early — they only execute on failures, like
  // the proven standalone.
  //
  // STANDALONE-ALIGNMENT (2026-11-04): the only remaining structural boot
  // delta vs the proven standalone ports was the copy's POSITION — the
  // standalone's hot code is copied by crt0 BEFORE any peripheral exists,
  // while the rack copied inside setup() with USB enumeration already racing.
  // The copy is now the ABSOLUTE FIRST action of setup(), before
  // Serial.begin/LED/USB — crt0-like semantics.
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
  module_init();
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

  // Per-module native rate / block size / DMA bank depth (constants from the
  // selected adapter — see the standalone ports for the sizing rationale).
  i2s.setBitsPerSample(16);
  i2s.setBuffers(kModuleI2sBuffers, kModuleI2sWords);
  i2s.setFrequency(kModuleSampleRate);
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
  Serial.printf("%s up (rack boot module, %u Hz / %u-sample blocks, budget %u us, USB MIDI device + PIO-USB host)\r\n",
                kModuleName,
                (unsigned)kModuleSampleRate,
                (unsigned)kModuleBlockSize,
                kModuleBudgetUs);
}

void loop() {
  // --- USB host: service the host stack, then poll host MIDI ---
  USBHost.task();
  pollHostMidi();

  // --- USB MIDI device: drain all pending 4-byte event packets ---
  uint8_t pkt[4];
  while (usb_midi.readPacket(pkt)) {
    midiHandleEventPacket(pkt);
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
  // PIO-USB host tail. Budget arithmetic is per-adapter (elements keeps its
  // divide-first 496; the others multiply-first).
  if (now - last_report > 2000) {
    last_report = now;
    static uint32_t last_stall_total = 0;
    const uint32_t stalls = i2s_stall_count;
    Serial.printf("max Process: %u us (budget %u us) | i2s stalls: %u (+%u) | in=%u out=%u notes=%u | host: conn=%u st=%u dev=%u midi=%u frame=%u\r\n",
                  proc_us_max,
                  kModuleBudgetUs,
                  stalls, stalls - last_stall_total,
                  (unsigned)(rack_meter_a * 1000.0f),
                  (unsigned)(rack_meter_b * 1000.0f),
                  note_on_count,
                  (unsigned)pio_host_connected(),
                  (unsigned)pio_host_line_state(),
                  (unsigned)host_device_mounted,
                  (unsigned)host_midi_idx_count,
                  (unsigned)pio_usb_host_get_frame_number());
    last_stall_total = stalls;
    proc_us_max = 0;
    rack_meter_a = 0.0f;            // max-since-report meters (see adapters)
    rack_meter_b = 0.0f;
  }
}
