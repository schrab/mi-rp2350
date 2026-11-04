// Rack RP2350 — module registry (T2; mod destinations populated in T3).
//
// ONE table describing the four module adapters: identity, audio config,
// bench budgets, CC map summary, mod-destination table and the hook pointers
// main.cpp + midi.h dispatch through. main.cpp iterates this table instead of
// per-adapter #if dispatch — all four adapters compile in every env (their
// state is arena-placed), the envs differ only in RACK_BOOT_MODULE (which
// module BOOTS).
//
// T3: kModDestinations is now a STRUCTURED per-module table (RackModDestinations)
// consumed by the mod engine (mod_engine.h): the CCs that accept CONTINUOUS
// modulation (written every block at float precision via the adapter's
// mod_set_cc_float hook) and the CCs that accept TRIGGERS (plaits trigger /
// rings strum / clouds freeze+trig, via mod_trigger_cc).
#pragma once
#include <stdint.h>

#include "rack_window.h"         // RackModuleId

// Trigger-destination semantics (mod engine -> adapter):
//   PULSE: the adapter consumes a rising edge as a one-block trigger pulse
//          (plaits TRIG, rings STRUM, clouds TRIG cc 66).
//   LATCH: the adapter tracks the source LEVEL (freeze latches on while the
//          source is high and off while low — clouds freeze cc 65).
enum RackTriggerKind : uint8_t {
  RACK_TRIG_PULSE = 0,
  RACK_TRIG_LATCH = 1,
};

struct RackModDestinations {
  const char* summary;           // printable one-liner (console `module`)
  uint8_t continuous_count;
  const uint8_t* continuous;     // CCs accepting continuous modulation
  uint8_t trigger_count;
  const uint8_t* trigger_cc;     // CCs (or pseudo-CCs) accepting triggers
  const uint8_t* trigger_kind;   // RackTriggerKind per trigger_cc entry
};

struct RackModuleInfo {
  const char* name;              // "elements" / "clouds" / "plaits" / "rings"
  uint8_t id;                    // RackModuleId
  // Audio config (per-module native rate/block/bank; the swap path
  // reconfigures I2S from these fields).
  uint32_t sample_rate;          // 32000 / 48000
  uint32_t block_size;           // 16 / 32 / 24 / 24
  uint32_t budget_us;            // 496 / 1000 / 500 / 750
  uint32_t i2s_buffers;          // DMA bank depth (buffers)
  uint32_t i2s_words;            // words per buffer
  uint32_t stall_us;             // stall-detector threshold
  // Bench/UI metadata.
  const char* cc_map;            // one-line CC map summary
  const RackModDestinations* mod_destinations;  // T3 mod-engine routing table
  // Adapter hooks (core 0 for init/note/cc, core 1 for render).
  void (*init)();
  void (*render)();              // exactly one block
  void (*note_on)(uint8_t note, uint8_t vel);
  void (*note_off)(uint8_t note);
  void (*control_change)(uint8_t cc, uint8_t val);
};

// DEFINED in main.cpp (single TU; the adapters are included above it).
extern const RackModuleInfo kRackModules[RACK_MODULE_COUNT];
