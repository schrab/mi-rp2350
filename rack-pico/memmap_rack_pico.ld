/* Rack RP2350 linker script — the Pico SDK fork (2026-11-23).

   INCLUDEs the SDK's default rp2350 script tree verbatim; the rack deltas
   live in the override files under linker_overrides/ (found first via the
   -L path pico_add_linker_script_override_path adds):

     memory_extra.incl            — the RACK_WINDOW + RACK_LMA MEMORY regions
     section_extra_post_end.incl  — the tc_* hot-code overlay + copy symbols
                                    + ASSERTs (the documented custom hook)

   The RAM region shrinks via pico_set_linker_script_var(RAM_LENGTH) — the
   SDK's set_memory_locations.incl keeps a --defsym override (0x63000: main
   SRAM minus the 116 KiB hot-code window, same split as the Arduino build's
   memmap_rack.ld).

   The hot-code window design (tc_* OVERLAY at one shared VMA, sequential
   LMAs in the reserved flash region, boot/swap memcpy) is documented in
   rack-rp2350/memmap_rack.ld and src/rack_window.h — the mechanism is
   identical here. */
INCLUDE "memmap_default.incl"
