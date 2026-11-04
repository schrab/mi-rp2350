// Rack RP2350 — module registry (T2).
//
// ONE table describing the four module adapters: identity, audio config,
// bench budgets, CC map summary, mod-destination placeholder and the hook
// pointers main.cpp + midi.h dispatch through. main.cpp iterates this table
// instead of per-adapter #if dispatch — all four adapters compile in every
// env (their state is arena-placed), the envs differ only in RACK_BOOT_MODULE
// (which module BOOTS).
//
// T3 will fill kModDestinations (the mod engine's destination list per
// module: the CC-map entries + trigger maps plaits trigger / rings strum /
// clouds freeze+trig); for T2 it stays an empty string placeholder.
#pragma once
#include <stdint.h>

#include "rack_window.h"         // RackModuleId

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
  const char* mod_destinations;  // T3 placeholder (empty for T2)
  // Adapter hooks (core 0 for init/note/cc, core 1 for render).
  void (*init)();
  void (*render)();              // exactly one block
  void (*note_on)(uint8_t note, uint8_t vel);
  void (*note_off)(uint8_t note);
  void (*control_change)(uint8_t cc, uint8_t val);
};

// DEFINED in main.cpp (single TU; the adapters are included above it).
extern const RackModuleInfo kRackModules[RACK_MODULE_COUNT];
