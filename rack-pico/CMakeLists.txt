# Rack Pico SDK build — the full rack firmware (the Arduino/PlatformIO
# migration target, 2026-11-23).
#
# Why this exists: the earlephilhower/Arduino core appends -Os after the
# project flags (the SRAM-law fight) and gates USB Audio; the Pico SDK gives
# exact flag control and TinyUSB directly. The DSP engines are build-system-
# agnostic — included from ../rack-rp2350 (the single source of truth; nothing
# is copied). The Arduino build in rack-rp2350/ stays the last-known-good
# until this one is ear-accepted.
#
# Clock: 320 MHz sys (the chain combinations wedge at 307.2) — the runtime
# QMI/VREG sequence lives in main() (rack-rp2350 main.cpp rack_pre_clock is
# the reference). RP2350 has no boot2, so no PICO_FLASH_SPI_CLKDIV define.
#
# Build:
#   PICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk \
#     cmake -B build -S . -G Ninja -DPICO_BOARD=pico2 && cmake --build build
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rack_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ----------------------------------------------------------------------------
# The engine/library static lib — the same sources PlatformIO compiles via LDF
# chain+ (lib/*/library.json), same defines.  ARM_MATH_CM33 is scoped to the
# CMSIS sources exactly like the Arduino build (clouds' stft.cc compiles
# without it there; mirror that).
# ----------------------------------------------------------------------------
file(GLOB RACK_WARPS_SRCS CONFIGURE_DEPENDS
    ../rack-rp2350/lib/elements/warps/dsp/*.cc
    ../rack-rp2350/lib/elements/warps/*.cc)
file(GLOB RACK_CLOUDS_SRCS CONFIGURE_DEPENDS
    ../rack-rp2350/lib/clouds/clouds/*.cc
    ../rack-rp2350/lib/clouds/clouds/dsp/*.cc
    ../rack-rp2350/lib/clouds/clouds/dsp/pvoc/*.cc)
list(REMOVE_ITEM RACK_CLOUDS_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/../rack-rp2350/lib/clouds/clouds/settings.cc)
file(GLOB RACK_PLAITS_SRCS CONFIGURE_DEPENDS
    ../rack-rp2350/lib/plaits/plaits/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/engine/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/engine2/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/chords/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/fm/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/physical_modelling/*.cc
    ../rack-rp2350/lib/plaits/plaits/dsp/speech/*.cc)
file(GLOB RACK_RINGS_SRCS CONFIGURE_DEPENDS
    ../rack-rp2350/lib/rings/rings/*.cc
    ../rack-rp2350/lib/rings/rings/dsp/*.cc)
file(GLOB RACK_CMSIS_SRCS CONFIGURE_DEPENDS
    ../rack-rp2350/lib/cmsis_dsp/src/*.c)

add_library(rack_engines STATIC
    ../rack-rp2350/lib/elements/elements_all.cpp   # Part::Init/Process/Seed (part.inc)
    ${RACK_WARPS_SRCS}
    ${RACK_CLOUDS_SRCS}
    ${RACK_PLAITS_SRCS}
    ${RACK_RINGS_SRCS}
    ../rack-rp2350/lib/stmlib/stmlib_all.cpp   # stmlib LUTs + Random state
    ../rack-rp2350/lib/lira8_engine/lira8_engine_o3.cpp
    ../rack-rp2350/lib/dronecaster_engine/dronecaster_engine_o3.cpp
)
target_sources(rack_engines PRIVATE ${RACK_CMSIS_SRCS})

target_include_directories(rack_engines PUBLIC
    ../rack-rp2350/src
    ../rack-rp2350/src/lira8
    ../rack-rp2350/src/dronecaster
    ../rack-rp2350/src/dronecaster/drones
    ../rack-rp2350/src/benjolin
    ../rack-rp2350/lib/stmlib
    ../rack-rp2350/lib/elements
    ../rack-rp2350/lib/clouds
    ../rack-rp2350/lib/plaits
    ../rack-rp2350/lib/rings
    ../rack-rp2350/lib/lira8_engine
    ../rack-rp2350/lib/dronecaster_engine
    ../rack-rp2350/lib/cmsis_dsp/Include
    ../PicoCollider/include
    ../PicoCollider/include/picocollider
)

target_compile_definitions(rack_engines PUBLIC
    RACK_RP2350_TC
    RACK_PICO_BUILD
    RACK_HANG_DEBUG=1
    PLAITS_ENGINE_COUNT=98
    PLAITS_CORRECTED_SAMPLE_RATE=48000.0f
    PLAITS_HAS_USER_DATA_BANK=1
    RACK_PLAITS_RUNTIME_RATE
    RACK_BOOT_MODULE=6
)
set_source_files_properties(${RACK_CMSIS_SRCS}
    PROPERTIES COMPILE_DEFINITIONS "ARM_MATH_CM33")

# Engine headers use pico/time.h (time_us_32) — SDK headers only, no runtime.
# cmsis_core_headers supplies cmsis_compiler.h for arm_math_types.h (the
# Arduino build got it from the toolchain's CMSIS; the SDK ships the same
# Core/Include stub).
target_link_libraries(rack_engines PUBLIC pico_stdlib_headers cmsis_core_headers)

# ----------------------------------------------------------------------------
# The firmware target — platform layer (main.cpp, I2S DMA bank, USB composite)
# ----------------------------------------------------------------------------
option(RACK_PICO_USB_CAPTURE "UAC1 loopback capture function (device-vs-golden rig)" ON)

add_executable(rack
    main.cpp
    rack_i2s.cpp
    usb_descriptors.cpp
    usb_console.cpp
    usb_midi.cpp
    usb_capture.cpp
    i2s_out.pio
    ../rack-rp2350/src/ws2812.pio
)

pico_generate_pio_header(rack ${CMAKE_CURRENT_LIST_DIR}/i2s_out.pio)
pico_generate_pio_header(rack ${CMAKE_CURRENT_LIST_DIR}/../rack-rp2350/src/ws2812.pio)

target_include_directories(rack PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ../rack-rp2350/src
)

target_link_libraries(rack
    rack_engines
    pico_stdlib
    pico_multicore
    pico_unique_id
    tinyusb_device      # TinyUSB device stack: built-in CDC + MIDI drivers;
                        # the UAC1 capture app driver is usb_capture.cpp; —
                        # tusb_config.h is ours)
    hardware_pio
    hardware_clocks
    hardware_uart
    hardware_dma
    hardware_irq
    hardware_adc
    hardware_vreg
    hardware_watchdog
    hardware_sync
)

target_compile_definitions(rack PRIVATE
    RACK_RP2350_TC
    RACK_PICO_BUILD
    RACK_HANG_DEBUG=1
    RACK_BOOT_MODULE=6
    PICO_RP2040_USB_FAST_IRQ=1
)
if(RACK_PICO_USB_CAPTURE)
  target_compile_definitions(rack PRIVATE RACK_PICO_USB_CAPTURE=1)
endif()

# RAM-place the TinyUSB rp2040-port IRQ path (the USB IRQ fires while core 1
# renders).
# The flags the Arduino core could not give: -O3 everywhere (no -Os append).
target_compile_options(rack PRIVATE -O3 -Wall -Wextra)
target_compile_options(rack_engines PRIVATE -O3)

# The USB CDC console comes from our own composite descriptors
# (usb_descriptors.cpp + usb_console.cpp) — pico_enable_stdio_usb would bring
# the SDK's CDC-only descriptors and break the composite device.
pico_enable_stdio_usb(rack 0)
pico_enable_stdio_uart(rack 0)

# The rack linker script: INCLUDEs the SDK default tree; the deltas live in
# linker_overrides/ (memory_extra.incl = RACK_WINDOW/RACK_LMA regions,
# section_extra_post_end.incl = the tc_* overlay + ASSERTs).  RAM shrinks to
# 0x63000 via a defsym override (the SDK's set_memory_locations.incl keeps
# DEFINED() variables) — main SRAM minus the 116 KiB hot-code window.
pico_add_linker_script_override_path(rack
    ${CMAKE_CURRENT_LIST_DIR}/linker_overrides GLOB_FILES)
pico_set_linker_script_var(rack RAM_LENGTH 0x63000)
pico_set_linker_script(rack ${CMAKE_CURRENT_LIST_DIR}/memmap_rack_pico.ld)

pico_add_extra_outputs(rack)

# Phase-2 task 2: fail the build on a RAM-placement regression. The checker
# is stdlib-only Python; host-only builds can opt out with
# -DRACK_SKIP_PLACEMENT_CHECK=ON.
option(RACK_SKIP_PLACEMENT_CHECK "Skip the post-link RAM placement gate" OFF)
if(NOT RACK_SKIP_PLACEMENT_CHECK)
  add_custom_command(TARGET rack POST_BUILD
    COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/scripts/check_ram_placement.py
            ${CMAKE_CURRENT_BINARY_DIR}/rack.elf
    COMMENT "Checking RAM placement"
    VERBATIM)
endif()
