/* Rack RP2350 linker script — CLOUDS env — fork of the earlephilhower
   core's lib/rp2350/memmap_default.ld. One of FOUR per-env scripts
   (memmap_rack_{elements,clouds,plaits,rings}.ld), selected via
   `board_build.ldscript = memmap_rack_clouds.ld` (the core's tools/
   platformio-build.py only generates its own substituted memmap_default.ld
   when board.build.ldscript is EMPTY; a custom script is passed to ld RAW,
   so the core's __FLASH_LENGTH__ / __EEPROM_START__ / __FS_START__ /
   __FS_END__ / __PSRAM_LENGTH__ substitution variables are NOT available and
   are hard-coded here to the values the core computes for the pinned
   rpipico2 board: upload.maximum_size = 4 MiB flash, filesystem_size = 0 MB,
   no PSRAM:
     __FLASH_LENGTH__  = 4194304 - 4096 (EEPROM) = 0x3FF000  (ends 0x103FF000)
     __EEPROM_START__  = 0x103FF000
     __FS_START__/__FS_END__ = 0x103FF000 (0 MB filesystem -> empty range)
     __PSRAM_LENGTH__  = 0
   If the board or filesystem_size ever changes, update these numbers.

   The scripts split the old shared memmap_rack.ld (which sized ONE SRAM
   window for the largest module, plaits — starving every other env's heap;
   clouds' could not even malloc core 1's separate 8 KB stack, so core 1
   never launched and core 0 hung at the alarm-pool handshake).

   WHAT CHANGED vs the core script (everything else is byte-identical):

   1. The four modules' hot code/LUT sections are RENAMED by the RAM_CODE
      patches when the rack build defines RACK_RP2350_TC:
      .tc_elements.<token> / .tc_clouds.<token> / .tc_plaits.<token> /
      .tc_rings.<token> (the rack's vendored CMSIS copy uses
      .tc_clouds.cmsis / .tc_clouds.cmsis_tables). None of those names match
      the core .data rule `*(.time_critical*)`, so crt0's boot copy no longer
      touches them — instead they are placed below:

      - VMA: ONLY the SELECTED module's output section (tc_clouds) is an
        OVERLAY member occupying the SRAM window region RACK_WINDOW at the
        top of main SRAM (right below 0x20080000, where SCRATCH_X begins).
        It is copied in at boot (rack_window_copy(), src/) BEFORE any module
        code runs. The other THREE modules' .tc_* input sections are
        collected into ORDINARY FLASH-RESIDENT output sections
        (.tc_flash_elements / .tc_flash_plaits / .tc_flash_rings) inside the
        reserved RACK_LMA region — VMA = LMA, like .rodata: linked but never
        called/executed in this env, harmless dead weight kept so the copy
        table in src/rack_window.h stays uniform (T2/T6 will re-tier). The
        core .data rule keeps collecting the PLATFORM .time_critical*
        sections (pio_usb, framework) exactly as before.

      - LMA: the SELECTED module's flash copy lives at the bottom of the
        reserved region RACK_LMA at the top of flash (below the 4 KB EEPROM
        block the core reserves; the rack project sets no filesystem so
        FS_START/FS_END collapse to that same EEPROM line), placed exactly
        as the old shared script placed it: GNU ld refuses `AT>` for OVERLAY
        members, hence the explicit MEMORY region + AT(ORIGIN(RACK_LMA)).
        The three .tc_flash_* sections follow it sequentially in the same
        region, chained by explicit start addresses (each = the previous
        section's end symbol; VMA = LMA there). Explicit ASSERTs — not ld's
        region accounting, which explicit AT/addresses bypass — guarantee
        the reserved region can never collide with the main image or
        overflow.

   2. RAM LENGTH shrinks by THIS env's window so the core's own heap/stack
      limits (= ORIGIN(RAM) + LENGTH(RAM)) stay below it. Window = the
      measured tc_clouds total (rack ELF, 2026-11-04) rounded up:
      136,960 B (incl. 60,344 CMSIS) -> 0x21800 (137,216 B; 2-KiB round-up,
      256 B slack). RAM = 512 KiB - 134 KiB = 0x5E800 (387,072 B below the
      window) — the env the shared window starved: this restores ~137 KB of
      heap so the core-1 stack malloc(0x2000), the I2S DMA bank and slack
      all fit again.

   3. RACK_LMA LENGTH = all four tc sets rounded up to 4 KiB = 0x7E000
      (516,096 B; the sum of the four measured sets is 505,190 B) —
      unchanged from the shared script: every env still links all four
      modules' tc sets into this region (one overlay member + three
      .tc_flash_* sections).

   NOLOAD note: RACK_WINDOW holds NO initialization (the tc_clouds OVERLAY
   section is the only thing placed there, copied at boot) — the window is
   "NOLOAD" in effect: nothing in crt0 reads or zeroes it. */

MEMORY
{
    FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 0x3FF000
    PSRAM(rwx) : ORIGIN = 0x11000000, LENGTH = 0
    /* Main SRAM minus the module hot-code window (window = 134 KiB). */
    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = 0x5E800
    /* SRAM window: top of main SRAM, right below SCRATCH_X (0x20080000).
       ONLY tc_clouds (the SELECTED module) occupies this VMA in this
       env; the other three modules are flash-resident (.tc_flash_*). */
    RACK_WINDOW(rwx) : ORIGIN = 0x2005E800, LENGTH = 0x21800
    /* Reserved flash sub-region: the SELECTED module's tc_* flash copy
       (LMA) plus the three UNSELECTED modules' flash-resident .tc_flash_*
       sections, sequentially chained. Top of flash minus the core's 4 KiB
       EEPROM emulation block (EEPROM lives at 0x103FF000; the FLASH region
       ends there). */
    RACK_LMA(r) : ORIGIN = 0x10381000, LENGTH = 0x7E000
    SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 4k
    SCRATCH_Y(rwx) : ORIGIN = 0x20081000, LENGTH = 4k
}

PROVIDE ( _EEPROM_start = 0x103FF000 );
PROVIDE ( _FS_start     = 0x103FF000 );
PROVIDE ( _FS_end       = 0x103FF000 );

ENTRY(_entry_point)

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
        /* TODO revisit this now memset/memcpy/float in ROM */
        /* bit of a hack right now to exclude all floating point and time critical (e.g. memset, memcpy) code from
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
           RACK_RP2350_TC and are placed below instead: the SELECTED module
           by the tc_* OVERLAY (flash-resident + copied into the SRAM window
           at boot), the other three by the flash-resident .tc_flash_*
           sections. */
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
        *(.tbss .tbss.* .gnu.linkonce.tb.*)
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
       Rack module hot sections (RACK_RP2350_TC builds), CLOUDS env.

       ONE OVERLAY MEMBER: the SELECTED module (clouds) occupies the VMA
       ORIGIN(RACK_WINDOW); its load address starts at ORIGIN(RACK_LMA)
       (ld also defines __load_start_tc_clouds / __load_stop_tc_clouds;
       the copy API uses the explicit __rack_* symbols below). NOCROSSREFS
       is sound: the four modules are independent, shared code (stmlib)
       lives in .data/.text, never in a tc_* section.
       KEEP() so --gc-sections can never drop a marked section: the window
       size must be deterministic (independent of the adapter call graph),
       matching the measured standalone totals.

       The other THREE modules are NOT overlay members here: their .tc_*
       input sections are collected by the flash-resident .tc_flash_*
       output sections right below (dead weight in this env — linked but
       never called; they define the same __tc_<m>_start/_end symbol names,
       pointing at their FLASH addresses, so src/rack_window.h's copy table
       stays uniform and links unchanged). rack_window_copy() is only ever
       called with the SELECTED id in this env (RACK_BOOT_MODULE is
       compile-time); called with an unselected id it would memcpy flash
       onto itself — a harmless dead path, never executed.
       ------------------------------------------------------------------ */
    OVERLAY ORIGIN(RACK_WINDOW) : NOCROSSREFS AT (ORIGIN(RACK_LMA))
    {
        tc_clouds
        {
            __tc_clouds_start = .;
            KEEP(*(.tc_clouds.*))
            . = ALIGN(4);
            __tc_clouds_end = .;
        }
    } > RACK_WINDOW

    /* Flash (LMA) extent of the SELECTED module's hot section — copy
       source for rack_window_copy(). */
    __rack_flash_clouds_start = LOADADDR(tc_clouds);
    __rack_flash_clouds_end   = LOADADDR(tc_clouds) + SIZEOF(tc_clouds);

    /* The three UNSELECTED modules: flash-resident output sections in the
       same RACK_LMA region, sequentially chained from the selected
       module's flash copy via explicit start addresses (explicit AT() and
       explicit addresses bypass ld's region accounting, hence the ASSERT
       below). VMA = LMA here, so __rack_flash_<m>_* and __tc_<m>_*
       coincide (the dead path rack_window_copy() would take for an
       unselected id in this env — see above, never executed). */
    .tc_flash_elements __rack_flash_clouds_end : {
        __tc_elements_start = .;
        KEEP(*(.tc_elements.*))
        . = ALIGN(4);
        __tc_elements_end = .;
    } > RACK_LMA
    __rack_flash_elements_start = ADDR(.tc_flash_elements);
    __rack_flash_elements_end   = ADDR(.tc_flash_elements) + SIZEOF(.tc_flash_elements);

    .tc_flash_plaits __tc_elements_end : {
        __tc_plaits_start = .;
        KEEP(*(.tc_plaits.*))
        . = ALIGN(4);
        __tc_plaits_end = .;
    } > RACK_LMA
    __rack_flash_plaits_start = ADDR(.tc_flash_plaits);
    __rack_flash_plaits_end   = ADDR(.tc_flash_plaits) + SIZEOF(.tc_flash_plaits);

    .tc_flash_rings __tc_plaits_end : {
        __tc_rings_start = .;
        KEEP(*(.tc_rings.*))
        . = ALIGN(4);
        __tc_rings_end = .;
    } > RACK_LMA
    __rack_flash_rings_start = ADDR(.tc_flash_rings);
    __rack_flash_rings_end   = ADDR(.tc_flash_rings) + SIZEOF(.tc_flash_rings);
    /* Last-placed flash section end = total extent of the reserved
       region's rack content (selected overlay member + three .tc_flash_*). */
    __rack_flash_end = __rack_flash_rings_end;

    /* Only the SELECTED module shares the VMA window in this env; the
       other three live in flash, guarded by the RACK_LMA assert below. */
    ASSERT(SIZEOF(tc_clouds) <= LENGTH(RACK_WINDOW),
        "ERROR: clouds hot section exceeds the rack SRAM window")
    /* Explicit AT()/addresses bypass ld's region accounting: assert the
       four flash copies (selected member + three .tc_flash_*) stay inside
       the reserved region. */
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
