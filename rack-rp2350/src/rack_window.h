// Rack RP2350 — SRAM window management (T2 unified window).
//
// The four modules' hot code/LUT sections are compiled into .tc_<module>.*
// input sections (RACK_RP2350_TC renames in the RAM_CODE patches). The rack
// linker script (memmap_rack.ld — ONE script for every env since T2) places
// ALL FOUR of them into ONE flash-resident OVERLAY: same VMA = the shared
// SRAM window RACK_WINDOW at the top of main SRAM, sequential LMAs in the
// reserved flash region RACK_LMA. At boot the SELECTED (RACK_BOOT_MODULE)
// module's flash copy is memcpy'd into the window; the T2 live swap repeats
// exactly that copy for any other module between mute/unmute — after that,
// all module hot code executes from SRAM exactly like the standalone builds'
// crt0-copied .time_critical sections.
//
// Symbol provenance (memmap_rack.ld, per module m):
//   __tc_<m>_start / __tc_<m>_end          VMA inside RACK_WINDOW (dest)
//   __rack_flash_<m>_start / ..._end       LMA inside RACK_LMA (source)
// All four members share the overlay VMA, so every id's window_start is the
// same address; the flash_start/flash_end pairs are distinct per member.
//
// Plain memcpy is CORRECT for the copy: the RP2350 M33 I/D-caches are NOT
// enabled by the core/SDK (verified T1 finding — no SCB_Enable*Cache
// anywhere, symbols not linked), so there is nothing to invalidate after
// writing instruction data into RAM.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Module ids — MUST match RACK_BOOT_MODULE (per-env build flag) and the
// tc_* OVERLAY members in memmap_rack.ld.
enum RackModuleId {
  RACK_MODULE_ELEMENTS = 0,
  RACK_MODULE_CLOUDS   = 1,
  RACK_MODULE_PLAITS   = 2,
  RACK_MODULE_RINGS    = 3,
  RACK_MODULE_COUNT    = 4,
};

// Window capacity of THIS build — must equal LENGTH(RACK_WINDOW) in
// memmap_rack.ld (sized by the re-tiered plaits hot set: 108,820 B measured
// -> 0x1C000 = 114,688 B). Only feeds prints; the copy itself uses the
// start/end symbols.
static const uint32_t kRackWindowBytes = 0x1C000u;   // 114,688 B (T2 unified)

extern "C" {
// VMA side (SRAM window) — copy DESTINATION for each module (all four are
// the same overlay VMA).
extern uint8_t __tc_elements_start[], __tc_elements_end[];
extern uint8_t __tc_clouds_start[], __tc_clouds_end[];
extern uint8_t __tc_plaits_start[], __tc_plaits_end[];
extern uint8_t __tc_rings_start[], __tc_rings_end[];
// LMA side (reserved flash region) — copy SOURCE for each module.
extern uint8_t __rack_flash_elements_start[], __rack_flash_elements_end[];
extern uint8_t __rack_flash_clouds_start[], __rack_flash_clouds_end[];
extern uint8_t __rack_flash_plaits_start[], __rack_flash_plaits_end[];
extern uint8_t __rack_flash_rings_start[], __rack_flash_rings_end[];
}  // extern "C"

struct RackWindowSection {
  const char* name;
  uint8_t* window_start;   // VMA in RACK_WINDOW (the shared overlay VMA)
  uint8_t* window_end;
  uint8_t* flash_start;    // LMA in RACK_LMA
  uint8_t* flash_end;
};

inline const RackWindowSection& rack_window_section(int module_id) {
  static const RackWindowSection kSections[RACK_MODULE_COUNT] = {
      {"elements", __tc_elements_start, __tc_elements_end,
                   __rack_flash_elements_start, __rack_flash_elements_end},
      {"clouds",   __tc_clouds_start, __tc_clouds_end,
                   __rack_flash_clouds_start, __rack_flash_clouds_end},
      {"plaits",   __tc_plaits_start, __tc_plaits_end,
                   __rack_flash_plaits_start, __rack_flash_plaits_end},
      {"rings",    __tc_rings_start, __tc_rings_end,
                   __rack_flash_rings_start, __rack_flash_rings_end},
  };
  return kSections[module_id];
}

// FNV-1a 32-bit — cheap deterministic checksum for the copy print
// (src == dst proves the flash copy landed intact; the flash source is
// memory-mapped XIP, the destination real SRAM).
inline uint32_t rack_checksum32(const uint8_t* p, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; ++i) {
    h ^= p[i];
    h *= 16777619u;
  }
  return h;
}

// Copy module_id's hot section flash -> SRAM window and checksum both sides.
// Returns the byte count copied; src_sum/dst_sum (optional) receive the
// FNV-1a of the flash source and the SRAM destination — a mismatch means the
// copy is corrupt (and the caller prints the hard error line / aborts the
// swap). Valid for EVERY module id in T2 (the unified overlay gives each one
// its own flash image + the shared window destination).
inline uint32_t rack_window_copy(int module_id, uint32_t* src_sum,
                                 uint32_t* dst_sum) {
  const RackWindowSection& s = rack_window_section(module_id);
  const size_t n = static_cast<size_t>(s.window_end - s.window_start);
  memcpy(s.window_start, s.flash_start, n);
  if (src_sum) *src_sum = rack_checksum32(s.flash_start, n);
  if (dst_sum) *dst_sum = rack_checksum32(s.window_start, n);
  return static_cast<uint32_t>(n);
}
