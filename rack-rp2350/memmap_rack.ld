/* Rack RP2350 linker script — UNIFIED (T2) — fork of the earlephilhower
   core's lib/rp2350/memmap_default.ld. ONE script for all four boot-select
   envs (rack-elements/clouds/plaits/rings): the envs differ only in
   -DRACK_BOOT_MODULE (which module boots); the SRAM window, the tc_* OVERLAY
   and RACK_LMA are identical in every env, so any module can be copied into
   the window at RUNTIME (rack_window_copy, live swap).

   A raw -T fork reaches ld un-substituted, so the core's __FLASH_LENGTH__ /
   __EEPROM_START__ / __FS_START__ / __FS_END__ / __PSRAM_LENGTH__ substitution
   variables are NOT available and are hard-coded here to the values the core
   computes for the pinned rpipico2 board: upload.maximum_size = 4 MiB flash,
   filesystem_size = 0 MB, no PSRAM:
     __FLASH_LENGTH__  = 4194304 - 4096 (EEPROM) = 0x3FF000  (ends 0x103FF000)
     __EEPROM_START__  = 0x103FF000
     __FS_START__/__FS_END__ = 0x103FF000 (0 MB filesystem -> empty range)
     __PSRAM_LENGTH__  = 0
   If the board or filesystem_size ever changes, update these numbers.

   T2 shape (replaces the four per-env scripts of T1): ALL FOUR modules'
   tc_* sections are members of ONE OVERLAY — same VMA = the shared SRAM
   window RACK_WINDOW at the top of main SRAM, sequential LMAs in the
   reserved flash region RACK_LMA. setup() copies the BOOT module's flash
   image into the window (crt0-like first action); the T2 live swap repeats
   the same copy for any other module between mute/unmute.

   WHAT CHANGED vs the core script (everything else is byte-identical):

   1. The four modules' hot code/LUT sections are RENAMED by the RAM_CODE
      patches when the rack build defines RACK_RP2350_TC:
      .tc_elements.<token> / .tc_clouds.<token> / .tc_plaits.<token> /
      .tc_rings.<token> (the rack's vendored CMSIS copy uses
      .tc_clouds.cmsis for FFT CODE; its FFT TABLES were DEMOTED to plain
      .rodata/flash in T2 — clouds spectral mode is excluded in the rack,
      plan "SRAM architecture": the 57 KB twiddle/bitrev tables must not
      occupy the shared window). None of those names match the core .data
      rule `*(.time_critical*)`, so crt0's boot copy never touches them —
      instead they are placed below:

      - VMA: the OVERLAY members ALL start at ORIGIN(RACK_WINDOW). Only the
        copied module's contents are meaningful after boot/swap; the other
        three members overlap it harmlessly (same VMA, different LMA —
        exactly one image is copied at any time).

      - LMA: the four members load consecutively from ORIGIN(RACK_LMA)
        (GNU ld refuses `AT>` for OVERLAY members, hence the explicit
        MEMORY region + AT(ORIGIN(RACK_LMA)); per-member LOADADDR() symbols
        feed the copy table). Explicit ASSERTs — not ld's region accounting,
        which explicit AT bypasses — guarantee the reserved region can never
        collide with the main image or overflow.

   2. Window sizing (T2 re-tier, rack ELF-measured tc_* input-section sums):
      elements 36,756 B; clouds 79,648 B (CMSIS tables demoted to flash);
      plaits 108,820 B (voice path + fm support + engine 0 + fork-flagged
      hot engines + lut_sine + wav_integrated_waves; long-tail engines and
      engine-only LUTs demoted to flash — see tools/patches/plaits/06+07);
      rings 58,044 B. Largest member plaits -> window 0x1C000 (114,688 B;
      ~5.8 KB slack for one-symbol promotions). RAM = 512 KiB - 112 KiB =
      0x64000 (409,600 B below the window). T1's per-env windows
      (0x9000/0x21800/0x43000/0xE400) are gone: one window serves all four.

   3. RACK_LMA ORIGIN moved up from 0x10381000 to 0x10389000: the main image
      now carries the demoted CMSIS tables (~56 KB) plus the demoted plaits
      engine bodies (~164 KB) in .text/.rodata, so it ends ~74 KB higher.
      RACK_LMA LENGTH 0x76000 (top of flash minus the core's 4 KiB EEPROM
      block at 0x103FF000) holds the four overlay LMAs (~283 KB total).

   NOLOAD note: RACK_WINDOW holds NO initialization (the tc_* OVERLAY
   members are the only things placed there, copied at boot/swap) — the
   window is "NOLOAD" in effect: nothing in crt0 reads or zeroes it.

   T6 chain shape: chain mode (A -> clouds) needs TWO module images resident
   SIMULTANEOUSLY, which the one-VMA OVERLAY cannot express. tc_clouds
   therefore leaves the OVERLAY and gets its OWN output section (the "B
   slot") at a FIXED offset RACK_CLOUDS_B_OFFSET = 0x9000 above the window
   origin; the remaining three A-slot members keep the shared OVERLAY VMA at
   the window origin (mutually exclusive, exactly one copied at a time —
   unchanged T2 semantics). Slot sizing (RACK_WINDOW 0x1C000 -> 0x1D000):
     A slot: 0x9000 = 36,864 B >= tc_elements 36,752 B (the chain A set;
       tc_plaits 108,820 / tc_rings 58,080 are OVER it — single-module use
       is unaffected, they simply span into the stale B-slot region and are
       re-copied from flash on any switch; chain engagement with such an A
       is rejected at runtime with these numbers)
     B slot: 0x9000 + tc_clouds 79,744 = 116,596 <= 0x1D000 (119,808 B)
   The ASSERTs below enforce both invariants at link time. Heap accounting:
   RAM shrinks 0x64000 -> 0x63000 (the window must end exactly at the
   SCRATCH_X origin — total main SRAM is a fixed 512 KiB), so the heap pays
   the 4 KiB: measured 41,744 -> 37,640 B (>= the 16 KiB chain headroom gate
   with the ~2 KiB I2S DMA bank allocation included in practice). */

MEMORY
{
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 0x3FF000
    PSRAM(rwx) : ORIGIN = 0x11000000, LENGTH = 0
    /* Main SRAM minus the module hot-code window (window = 116 KiB; T6 grew
       it 0x1C000 -> 0x1D000 for the chain B slot, so RAM gives back the same
       4 KiB — the heap pays for the chain residency, 1:1). */
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 0x63000
    /* SRAM window: top of main SRAM, right below SCRATCH_X (0x20080000).
       The A-slot OVERLAY members (elements/plaits/rings) start here; exactly
       one is populated (boot copy / live swap). tc_clouds has its own B-slot
       section at ORIGIN + RACK_CLOUDS_B_OFFSET (T6 chain mode, see the
       header). */
    RACK_WINDOW(rwx) : ORIGIN = 0x20063000, LENGTH = 0x1D000
    /* Reserved flash sub-region: the four modules' tc_* flash copies
       (overlay LMAs, assigned consecutively by ld). Top of flash minus the
       core's 4 KiB EEPROM emulation block (EEPROM lives at 0x103FF000; the
       FLASH region ends there). */
    RACK_LMA(r) : ORIGIN = 0x10389000, LENGTH = 0x76000
    SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20081000, LENGTH = 4k
}

PROVIDE ( _EEPROM_start = 0x103FF000 );
PROVIDE ( _FS_start     = 0x103FF000 );
PROVIDE ( _FS_end       = 0x103FF000 );

ENTRY(_entry_point)

/* T6 chain mode: tc_clouds' VMA sits at this FIXED offset above the window
   origin (the "B slot"). Sized to the chain A set: >= tc_elements (36,752 B);
   tc_plaits (108,820) / tc_rings (58,080) exceed it and are therefore
   rejected as chain A at runtime (the ASSERT below fires if the A-slot
   invariant is ever violated at link time). */
RACK_CLOUDS_B_OFFSET = 0x9000;

SECTIONS
{
    .flash_begin : {
        __flash_binary_start = .;
    } > FLASH

    .ota : {
        /* Start image with OTA */
        KEEP (*(.OTA))
        *ota.o
    } > FLASH

    .partition : {
        . = __flash_binary_start + 0x2ff0;
        LONG(0x103FF000)   /* FS_START (0 MB filesystem) */
        LONG(0x103FF000)   /* FS_END */
        LONG(0x103FF000)   /* EEPROM_START */
        LONG(0x3FF000)     /* FLASH_LENGTH */
    } > FLASH

    /* The bootrom will enter the image at the point indicated in your
       IMAGE_DEF, which is usually the reset handler of your vector table.

       The debugger will use the ELF entry point, which is the _entry_point
       symbol, and in our case is *different from the bootrom's entry point.*
       This is used to go back through the bootrom on debugger launches only,
       to perform the same initial flash setup that would be performed on a
       cold boot.
    */

    .text : {
        KEEP (*(.vectors))
        __logical_binary_start = .;
        KEEP (*(.binary_info_header))
        __binary_info_header_end = .;
        KEEP (*(.embedded_block))
        __embedded_block_end = .;
        KEEP (*(.reset))
        /* TODO revisit this now that we have the ability to exclude all floating point and time critical (e.g. memset, memcpy) code from
         * FLASH ... we will include any thing excluded here in .data below by default */
        *(.init)
        *libgcc.a:cmse_nonsecure_call.o
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)
        *(.fini)
        /* Pull all c'tors into .text */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)
        /* Followed by destructors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(SORT(.preinit_array.*)))
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        *(SORT(.fini_array.*))
        *(.fini_array)
        PROVIDE_HIDDEN (__fini_array_end = .);

        *(.eh_frame*)
        . = ALIGN(4);
    } > FLASH

    /* Note the boot2 section is optional, and should be discarded if there is
       no reference to it *inside* the binary, as it is not called by the
       bootrom. (The bootrom performs a simple best-effort XIP setup and
       leaves it to the binary to do anything more sophisticated.) However
       there is still a size limit of 256 bytes, to ensure the boot2 can be
       stored in boot RAM.

       Really this is a "XIP setup function" -- the name boot2 is historic and
       refers to its dual-purpose on RP2040, where it also handled vectoring
       from the bootrom into the user image.
    */
/*
    .boot2 : {
        __boot2_start__ = .;
        *(.boot2)
        __boot2_end__ = .;
    } > FLASH

    ASSERT(__boot2_end__ - __boot2_start__ <= 256,
        "ERROR: Pico second stage bootloader must be no more than 256 bytes in size")
*/
    .rodata : {
        *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .rodata*)
        *(.srodata*)
        . = ALIGN(4);
        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.flashdata*)))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    /* Machine inspectable binary information */
    . = ALIGN(4);
    __binary_info_start = .;
    .binary_info :
    {
        KEEP(*(.binary_info.keep.*))
        *(.binary_info.*)
    } > FLASH
    __binary_info_end = .;
    . = ALIGN(4);

    .ram_vector_table (NOLOAD): {
        *(.ram_vector_table)
    } > RAM

    .uninitialized_data (NOLOAD): {
        . = ALIGN(4);
        *(.uninitialized_data*)
    } > RAM

    .data : {
        __data_start__ = .;
        *(vtable)

        /* Platform .time_critical* sections ONLY: pio_usb, framework. The
           four modules' hot sections are renamed .tc_<module>.* under
           RACK_RP2350_TC and are placed below in the tc_* OVERLAY (flash-
           resident, copied into the SRAM window at boot/swap). */
        *(.time_critical*)

        /* remaining .text and .rodata; i.e. stuff we exclude above because we want it in RAM */
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
        . = ALIGN(4);

        *(.data*)
        *(.sdata*)

        . = ALIGN(4);
        *(.after_data.*)
        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__mutex_array_start = .);
        KEEP(*(SORT(.mutex_array.*)))
        KEEP(*(.mutex_array))
        PROVIDE_HIDDEN (__mutex_array_end = .);

        *(.jcr)
        . = ALIGN(4);
    } > RAM AT> FLASH

    .tdata : {
        . = ALIGN(4);
		*(.tdata .tdata.* .gnu.linkonce.td.*)
        /* All data end */
        __tdata_end = .;
    } > RAM AT> FLASH
    PROVIDE(__data_end__ = .);

    /* __etext is (for backwards compatibility) the name of the .data init source pointer (...) */
    __etext = LOADADDR(.data);

    .tbss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start__ = .;
        __tls_base = .;
        *(.tbss .tbss.* .gnu.linkonce.td.*)
        *(.tcommon)

        __tls_end = .;
    } > RAM

    .bss (NOLOAD) : {
        . = ALIGN(4);
        __tbss_end = .;

        *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
        *(COMMON)
        PROVIDE(__global_pointer$ = . + 2K);
        *(.sbss*)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (NOLOAD):
    {
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        /* historically on GCC sbrk was growing past __HeapLimit to __StackLimit, however
        to be more compatible, we now set __HeapLimit explicitly to where the end of the heap is */
        . = ORIGIN(RAM) + LENGTH(RAM);
        __HeapLimit = .;
    } > RAM

    /* ------------------------------------------------------------------
       Rack module hot sections (RACK_RP2350_TC builds).

       T6 chain layout: TWO slots in the one window.
       - A slot (the OVERLAY at ORIGIN(RACK_WINDOW)): elements / plaits /
         rings — mutually exclusive, ONE VMA, exactly one image copied at a
         time (unchanged T2 semantics).
       - B slot (tc_clouds' own section at ORIGIN + RACK_CLOUDS_B_OFFSET):
         resident ALONGSIDE the A slot while a chain is engaged; its image
         may also be copied alone (single-module clouds uses the B-slot
         address as its window image — nothing else lives at that offset
         while clouds is the active module, and any switch re-copies from
         flash).

       LMAs are assigned from ORIGIN(RACK_LMA): tc_clouds takes the region
       start (explicit AT), the overlay members continue consecutively after
       it (explicit AT on the OVERLAY — GNU ld refuses `AT>` for OVERLAY
       members).
       NOCROSSREFS is sound: the four modules are independent, shared code
       (stmlib) lives in .data/.text, never in a tc_* section.
       KEEP() so --gc-sections can never drop a marked section: the window
       size must be deterministic (independent of the adapter call graph),
       matching the measured standalone totals.
       ------------------------------------------------------------------ */
    tc_clouds ORIGIN(RACK_WINDOW) + RACK_CLOUDS_B_OFFSET :
        AT (ORIGIN(RACK_LMA))
    {
        __tc_clouds_start = .;
        KEEP(*(.tc_clouds.*))
        . = ALIGN(4);
        __tc_clouds_end = .;
    } > RACK_WINDOW

    OVERLAY ORIGIN(RACK_WINDOW) : NOCROSSREFS
        AT (ORIGIN(RACK_LMA) + SIZEOF(tc_clouds))
    {
        tc_elements
        {
            __tc_elements_start = .;
            KEEP(*(.tc_elements.*))
            . = ALIGN(4);
            __tc_elements_end = .;
        }
        tc_plaits
        {
            __tc_plaits_start = .;
            KEEP(*(.tc_plaits.*))
            . = ALIGN(4);
            __tc_plaits_end = .;
        }
        tc_rings
        {
            __tc_rings_start = .;
            KEEP(*(.tc_rings.*))
            . = ALIGN(4);
            __tc_rings_end = .;
        }
    } > RACK_WINDOW

    /* Flash (LMA) extents of the four hot sections — copy sources for
       rack_window_copy(). */
    __rack_flash_elements_start = LOADADDR(tc_elements);
    __rack_flash_elements_end   = LOADADDR(tc_elements) + SIZEOF(tc_elements);
    __rack_flash_clouds_start = LOADADDR(tc_clouds);
    __rack_flash_clouds_end   = LOADADDR(tc_clouds) + SIZEOF(tc_clouds);
    __rack_flash_plaits_start = LOADADDR(tc_plaits);
    __rack_flash_plaits_end   = LOADADDR(tc_plaits) + SIZEOF(tc_plaits);
    __rack_flash_rings_start = LOADADDR(tc_rings);
    __rack_flash_rings_end   = LOADADDR(tc_rings) + SIZEOF(tc_rings);
    /* Last-placed member's flash end = total extent of the reserved
       region's rack content (tc_clouds LMA = region start; the overlay
       members follow it, rings last). */
    __rack_flash_end = __rack_flash_rings_end;

    /* Every member must fit the ONE shared window. */
    ASSERT(SIZEOF(tc_elements) <= LENGTH(RACK_WINDOW),
        "ERROR: elements hot section exceeds the rack SRAM window")
    ASSERT(SIZEOF(tc_clouds) <= LENGTH(RACK_WINDOW),
        "ERROR: clouds hot section exceeds the rack SRAM window")
    ASSERT(SIZEOF(tc_plaits) <= LENGTH(RACK_WINDOW),
        "ERROR: plaits hot section exceeds the rack SRAM window")
    ASSERT(SIZEOF(tc_rings) <= LENGTH(RACK_WINDOW),
        "ERROR: rings hot section exceeds the rack SRAM window")

    /* T6 chain-residency invariants (B slot must not reach the A slot, and
       both slots must fit the window). The first assert pins the chain A set
       to modules whose hot image fits below the B slot — today tc_elements
       36,752 <= 0x9000; adding plaits/rings as chain A requires raising
       RACK_CLOUDS_B_OFFSET (and the window with it) and re-running the SRAM
       math in the ledger. */
    ASSERT(SIZEOF(tc_elements) <= RACK_CLOUDS_B_OFFSET,
        "ERROR: elements hot section overlaps the chain B slot (raise RACK_CLOUDS_B_OFFSET)")
    ASSERT(RACK_CLOUDS_B_OFFSET + SIZEOF(tc_clouds) <= LENGTH(RACK_WINDOW),
        "ERROR: chain B slot (clouds) exceeds the rack SRAM window")
    /* The window must end exactly where the core's scratch banks begin —
       RAM + window + scratch partition the fixed 512 KiB main SRAM with no
       gap and no overlap. */
    ASSERT(ORIGIN(RACK_WINDOW) + LENGTH(RACK_WINDOW) == ORIGIN(SCRATCH_X),
        "ERROR: RACK_WINDOW must end exactly at the SCRATCH_X origin")
    /* Explicit AT() bypasses ld's region accounting: assert the four flash
       copies stay inside the reserved region. */
    ASSERT(__rack_flash_end <= ORIGIN(RACK_LMA) + LENGTH(RACK_LMA),
        "ERROR: rack module flash copies overflow RACK_LMA")

    /* Start and end symbols must be word-aligned */
    .scratch_x : {
        __scratch_x_start__ = .;
        *(.scratch_x.*)
        . = ALIGN(4);
        __scratch_x_end__ = .;
    } > SCRATCH_X AT > FLASH
    __scratch_x_source__ = LOADADDR(.scratch_x);

    .scratch_y : {
        __scratch_y_start__ = .;
        *(.scratch_y.*)
        . = ALIGN(4);
        __scratch_y_end__ = .;
    } > SCRATCH_Y AT > FLASH
    __scratch_y_source__ = LOADADDR(.scratch_y);

    /* .stack*_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later
     *
     * stack1 section may be empty/missing if platform_launch_core1 is not used */

    /* by default we put core 0 stack at the end of scratch Y, so that if core 1
     * stack is not used then all of SCRATCH_X is free.
     */
    .stack1_dummy (NOLOAD):
    {
        *(.stack1*)
    } > SCRATCH_X
    .stack_dummy (NOLOAD):
    {
        KEEP(*(.stack*))
    } > SCRATCH_Y

    .flash_end : {
        KEEP(*(.embedded_end_block*))
        PROVIDE(__flash_binary_end = .);
    } > FLASH = 0xaa

    .psram (NOLOAD) : {
        __psram_start__ = .;
        *(.psram*)
        . = ALIGN(4096);
        __psram_heap_start__ = .;
    } > PSRAM

    /* stack limit is poorly named, but historically is maximum heap ptr */
    __StackLimit = ORIGIN(RAM) + LENGTH(RAM);
    __StackOneTop = ORIGIN(SCRATCH_X) + LENGTH(SCRATCH_X);
    __StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);
    __StackOneBottom = __StackOneTop - SIZEOF(.stack1_dummy);
    __StackBottom = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* picolibc and LLVM */
    PROVIDE (__heap_start = __end__);
    PROVIDE (__heap_end = __HeapLimit);
    PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
    PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1));
    PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );

    /* TLSF */
    PROVIDE (__psram_start = __psram_start__);
    PROVIDE (__psram_heap_start = __psram_heap_start__);

    /* llvm-libc */
    PROVIDE (_end = __end__);
    PROVIDE (__llvm_libc_heap_limit = __HeapLimit);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed")

    /* The main image (code, rodata AND the .data/.scratch flash copies —
       everything counted up to .flash_end) must end below the reserved
       rack LMA region. */
    ASSERT(__flash_binary_end <= ORIGIN(RACK_LMA),
        "ERROR: main image collides with the reserved rack flash region")

    ASSERT( __binary_info_header_end - __logical_binary_start <= 1024, "Binary info must be in first 1024 bytes of the binary")
    ASSERT( __embedded_block_end - __logical_binary_start <= 4096, "Embedded block must be in first 4096 bytes of the binary")

    /* todo assert on extra code */
}
