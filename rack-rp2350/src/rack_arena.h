// Rack RP2350 — module memory pool (T2 arenas -> T8 unified pool).
//
// HISTORY: until T8 this was TWO arrays, the OBJECT arena (132 KiB, sized by
// the rings+clouds chain states) and the BUFFER arena (192 KiB, sized by the
// elements/rings->clouds chain buffers). Both are unions (only the ACTIVE
// resident set's state is live), but they were sized by DIFFERENT worst
// cases — so a single module could be short in one pool while the other sat
// idle. The lira8->clouds chain is the textbook case: object 24,780 of
// 135,168 used (110 KB idle) while the buffer needed 259,088 of 196,608
// (62,480 short).
//
// T8: the two arrays were already physically adjacent in .bss, so they are
// ONE pool now, served by a deterministic bump allocator:
//
//   rack_mem_reset();                                  // once per resident set
//   uint8_t* obj = rack_mem_alloc(M::state_size(), 8);
//   uint8_t* buf = rack_mem_alloc(M::buf_bytes(), 8);
//   M::init_at(obj, buf);
//
// A chain places A then B from the SAME reset (see do_chain_engage). No
// free(), no fragmentation, no heap involvement; every transition re-inits
// both slots anyway (factory defaults), so nothing survives that would need
// preserving. The pool is .bss (crt0-zeroed; the adapters still memset the
// regions they own).
//
// Sizing is UNCHANGED (132 + 192 KiB = 331,776 B): today's worst chain
// (rings+clouds: 134,908 state + 196,608 buffers = 331,516) still fits with
// the same 260 B slack. Single-module demand is now elastic:
//   lira8 standalone      16,088 state + 176,416 delay          = 192,504
//   lira8->clouds chain   16,088 + 128,016 (32 kHz delay)
//                       +  8,692 + 131,072 (clouds chained)     = 283,872
//                         -> 47,904 B spare (was REJECTED by the split)
//
// ALLOCATION LAW: reset() runs exactly ONCE before the resident set's
// allocations (boot init, live swap, chain engage/disengage) — never per
// adapter, or a chain's B allocations would wipe A. alloc() returns 8-aligned
// memory (Part/Voice states and the plaits BufferAllocator workspace need 8)
// and HALTS loudly on overflow: a silent nullptr would surface as a crash in
// the DSP loop instead of a diagnosable boot/swap failure.
#pragma once
#include <stddef.h>
#include <stdint.h>

static constexpr size_t kRackPoolBytes = 132 * 1024 + 192 * 1024;  // 331,776

// DEFINED in main.cpp (single TU: main.cpp includes this header before the
// adapters, then defines the array + allocator).
extern uint8_t g_rack_pool[kRackPoolBytes];

void rack_mem_reset();
void* rack_mem_alloc(size_t bytes, size_t align = 8);
size_t rack_mem_used();     // bytes handed out since the last reset
static constexpr size_t rack_mem_total() { return kRackPoolBytes; }