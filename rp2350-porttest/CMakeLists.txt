cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(porttest C CXX ASM)

pico_sdk_init()

# Bit-exact ReBirth tables, linked straight from the orb-engine test fixtures.
set(TABLES_DIR "${CMAKE_CURRENT_LIST_DIR}/tables")
configure_file(${CMAKE_CURRENT_LIST_DIR}/tables.S.in
               ${CMAKE_CURRENT_BINARY_DIR}/tables.S @ONLY)

add_executable(porttest
    main.cpp
    orb303.cpp
    pcfcomp.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/tables.S
)

pico_generate_pio_header(porttest ${CMAKE_CURRENT_LIST_DIR}/i2s_out.pio)

# pico_double on RP2350 Arm defaults to the DCP-backed __aeabi_* routines.
target_link_libraries(porttest PRIVATE
    pico_stdlib
    pico_double
    hardware_pio
    hardware_dma
    hardware_clocks
)

pico_enable_stdio_usb(porttest 1)
pico_enable_stdio_uart(porttest 0)

pico_add_extra_outputs(porttest)
