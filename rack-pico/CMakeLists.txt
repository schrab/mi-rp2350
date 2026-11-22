# Rack Pico SDK build — the benjolin-first scaffold (2026-11-22).
#
# Why this exists: the earlephilhower/Arduino core appends -Os after the
# project flags (the SRAM-law fight) and gates USB Audio; the Pico SDK gives
# exact flag control, TinyUSB directly, and the DSPi-style USB capture path.
# The DSP engines are build-system-agnostic — this target includes them from
# ../rack-rp2350/src (the single source of truth; nothing is copied).
#
# Build (one-time SDK fetch: pico_sdk_import.cmake clones pico-sdk if
# PICO_SDK_PATH is unset; on this machine the PlatformIO-bundled SDK works):
#   PICO_SDK_PATH=~/.platformio/packages/framework-arduinopico/pico-sdk \
#     cmake -B build -S . -G Ninja && cmake --build build
cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rack_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rack_benjolin
    main.cpp
    i2s_out.pio
)

pico_generate_pio_header(rack_benjolin ${CMAKE_CURRENT_LIST_DIR}/i2s_out.pio)

target_include_directories(rack_benjolin PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ../rack-rp2350/src/benjolin
    ../PicoCollider/include
    ../PicoCollider/include/picocollider
)

target_link_libraries(rack_benjolin
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
)

# The flags the Arduino core could not give us: O3 everywhere (no -Os
# append), and -ffp-contract=off — the M33 has no FMA, so the host and the
# device must round the same way for the device-vs-golden comparison.
target_compile_options(rack_benjolin PRIVATE
    -O3
    -ffp-contract=off
    -Wall -Wextra
)

pico_enable_stdio_usb(rack_benjolin 1)
pico_enable_stdio_uart(rack_benjolin 0)

pico_add_extra_outputs(rack_benjolin)
