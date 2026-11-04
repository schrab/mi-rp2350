// Rack RP2350 — module state arenas (T2).
//
// The four module adapters no longer hold file-scope state objects: each
// adapter placement-news ONE state struct into the OBJECT arena at Init time
// and carves its DSP buffers from the BUFFER arena. The arenas are UNIONS —
// only the ACTIVE module's carve-out is live; each adapter's offsets are its
// own and all start at 0 (the previous module's state is abandoned, factory
// defaults are re-Inited on every swap; no destructors exist anywhere in the
// four ports' DSP state).
//
// Sizing (measured from the T1 rack ELFs, sizeof via nm):
//   OBJECT arena 128 KiB — largest state = rings (Part 123,352 B +
//   StringSynthPart 2,004 B + block buffers ~0.3 KB); elements Part is
//   113,488 B; plaits Voice 21,748 B; clouds processor 8,104 B.
//   BUFFER arena 192 KiB — clouds large(128 KiB) + small(64 KiB) fill it
//   exactly; elements/rings reverb 64 KiB; plaits workspace 32 KiB.
//
// Carve map (offset 0 in each arena unless noted):
//   elements: object = ElementsState (Part + PerformanceState + scratch)
//             buffer = uint16_t reverb[32768]              65,536 B
//   clouds:   object = CloudsState (GranularProcessor + frames)
//             buffer = large_buffer[32768] uint32          131,072 B
//                    + small_buffer[16384] uint32 @131072   65,536 B
//   plaits:   object = PlaitsState (Voice + Patch/Modulations + frames)
//             buffer = workspace[8192] uint32              32,768 B
//   rings:    object = RingsState (Part + StringSynthPart + buffers)
//             buffer = uint16_t reverb[32768]              65,536 B
#pragma once
#include <stddef.h>
#include <stdint.h>

// alignas(8): stmlib::BufferAllocator::Allocate<T> does not adjust alignment
// (the plaits workspace requirement) and Part's float members want 4+.
static constexpr size_t kRackObjectArenaBytes = 128 * 1024;
static constexpr size_t kRackBufferArenaBytes = 192 * 1024;

// DEFINED in main.cpp (single TU: main.cpp includes this header before the
// adapters, then defines the arrays).
extern uint8_t g_rack_object_arena[kRackObjectArenaBytes];
extern uint8_t g_rack_buffer_arena[kRackBufferArenaBytes];
