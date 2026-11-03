// Rack RP2350 — SRAM window management.
//
// The four modules' hot code/LUT sections are compiled into .tc_<module>.*
// input sections (RACK_RP2350_TC renames in the RAM_CODE patches). The rack
// linker script (memmap_rack_<module>.ld — ONE of four per-env scripts,
// selected via board_build.ldscript) places the SELECTED module's sections
// into one flash-resident output section tc_<module>: VMA = this env's SRAM
// window at the top of main SRAM, LMA = the module's slot at the bottom of
// the reserved flash region RACK_LMA. At boot, the SELECTED module's flash
// copy is memcpy'd into the window — after that, all module hot code
// executes from SRAM exactly like the standalone builds' crt0-copied
// .time_critical sections.
//
// The OTHER THREE modules are linked but FLASH-RESIDENT in this env: their
// .tc_* input sections land in ordinary .tc_flash_<m> output sections
// (VMA = LMA inside RACK_LMA — dead weight here, never called/executed;
// T2/T6 re-tier). Those sections define the SAME __tc_<m>_start/_end symbol
// names, pointing at their FLASH addresses, with __rack_flash_<m>_start/
// _end equal to them, so the copy table below stays uniform and links
// unchanged in every env. rack_window_copy() is only ever called with the
// SELECTED id (RACK_BOOT_MODULE is compile-time in T1); called with an
// unselected id it would memcpy flash onto itself — a harmless DEAD PATH,
// never executed (kept safe: no window write can occur, the window content
// stays the selected module's).
//
// Only ONE module occupies the window: T1 guarantees this by construction
// (RACK_BOOT_MODULE is compile-time; setup() copies before any module code
// runs). T2's live swap re-uses rack_window_copy() between mute/unmute (and
// re-tiers the per-env window sizing).
//
// Symbol provenance (memmap_rack_<module>.ld):
//   SELECTED module:
//     __tc_<m>_start / __tc_<m>_end          VMA inside RACK_WINDOW (dest)
//     __rack_flash_<m>_start / ..._end       LMA inside RACK_LMA (source)
//   other three (flash-resident dead weight):
//     __tc_<m>_* == __rack_flash_<m>_*       VMA = LMA inside RACK_LMA
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Module ids — MUST match RACK_BOOT_MODULE (per-env build flag) and the
// tc_* OVERLAY member selected in this env's memmap_rack_<module>.ld.
enum RackModuleId {
  RACK_MODULE_ELEMENTS = 0,
  RACK_MODULE_CLOUDS   = 1,
  RACK_MODULE_PLAITS   = 2,
  RACK_MODULE_RINGS    = 3,
  RACK_MODULE_COUNT    = 4,
};

// Total window capacity of THIS env — must equal LENGTH(RACK_WINDOW) in the
// env's memmap_rack_<module>.ld (per-env window sizing: the old shared
// script forced the largest module's 268 KiB window onto every env and
// starved the others' heap). Only feeds the boot print in src/main.cpp; the
// copy itself uses the start/end symbols.
#if RACK_BOOT_MODULE == 0
static const uint32_t kRackWindowBytes = 0x9000u;    //  36,864 B (elements)
#elif RACK_BOOT_MODULE == 1
static const uint32_t kRackWindowBytes = 0x21800u;   // 137,216 B (clouds)
#elif RACK_BOOT_MODULE == 2
static const uint32_t kRackWindowBytes = 0x43000u;   // 274,432 B (plaits)
#elif RACK_BOOT_MODULE == 3
static const uint32_t kRackWindowBytes = 0xE400u;    //  58,368 B (rings)
#endif

extern "C" {
// VMA side (SRAM window) — copy DESTINATION for each module.
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
  uint8_t* window_start;   // VMA in RACK_WINDOW
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

// FNV-1a 32-bit — cheap deterministic checksum for the boot print
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
// copy is corrupt (and the caller prints the hard error line).
//
// NOTE (per-env linker scripts): only the SELECTED module's window_start/
// window_end actually point into RACK_WINDOW. For the three unselected ids
// both pairs of pointers address the module's flash-resident .tc_flash_*
// copy, so this would memcpy flash onto itself (src_sum == dst_sum) — a
// harmless dead path, never taken in T1 (RACK_BOOT_MODULE is compile-time;
// main.cpp calls this exactly once, with the selected id).
inline uint32_t rack_window_copy(int module_id, uint32_t* src_sum,
                                 uint32_t* dst_sum) {
  const RackWindowSection& s = rack_window_section(module_id);
  const size_t n = static_cast<size_t>(s.window_end - s.window_start);
  memcpy(s.window_start, s.flash_start, n);
  if (src_sum) *src_sum = rack_checksum32(s.flash_start, n);
  if (dst_sum) *dst_sum = rack_checksum32(s.window_start, n);
  return static_cast<uint32_t>(n);
}
