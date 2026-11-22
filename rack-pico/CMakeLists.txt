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

# 307.2 MHz sys clock: the SDK default flash CLKDIV of 2 would clock the QSPI
# flash at 153.6 MHz — past the W25Q080's 104-133 MHz rating (the DSPi note;
# reads survive but erase/program fails verification).  Divider 6 → 51.2 MHz.
# Must be set BEFORE pico_sdk_init() so it propagates to the boot2 build; on
# RP2350 boot2 is not executed and the runtime QMI override is a later item.
add_compile_definitions(PICO_FLASH_SPI_CLKDIV=6)

include(pico_sdk_import.cmake)

project(rack_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(rack_benjolin
    main.cpp
    usb_descriptors.cpp
    usb_capture.cpp
    usb_console.cpp
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
    pico_unique_id
    tinyusb_device      # TinyUSB device stack: the built-in CDC driver serves
                        # the console; the UAC1 capture function is claimed by
                        # our custom driver (usbd_app_driver_get_cb in
                        # usb_capture.cpp).  tusb_config.h is ours.
    hardware_pio
    hardware_clocks
)

# RAM-place the TinyUSB rp2040-port IRQ path (the DSPi discipline; the USB
# IRQ fires while core 1 renders).
target_compile_definitions(rack_benjolin PRIVATE PICO_RP2040_USB_FAST_IRQ=1)

# The flags the Arduino core could not give us: O3 everywhere (no -Os
# append), and -ffp-contract=off — the M33 has no FMA, so the host and the
# device must round the same way for the device-vs-golden comparison.
target_compile_options(rack_benjolin PRIVATE
    -O3
    -ffp-contract=off
    -Wall -Wextra
)

# The USB CDC console comes from our own composite descriptors
# (usb_descriptors.cpp + usb_console.cpp) — pico_enable_stdio_usb would bring
# the SDK's CDC-only descriptors and break the composite device.
pico_enable_stdio_usb(rack_benjolin 0)
pico_enable_stdio_uart(rack_benjolin 0)

pico_add_extra_outputs(rack_benjolin)
