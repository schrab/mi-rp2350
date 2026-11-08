# Rack UI controller firmware (ESP32-C3) — rack-rp2350's T5 UI MCU (STAGE 1:
# framed UART link bring-up only). Seeded from ctag-tbd/uimcu 2026-11-08
# (cp -a; the seed tree is READ-ONLY — see the rack ledger findings).
# The menu/panel UI runs here; the DSP (rack RP2350) is reached over the rack
# frame protocol (docs/ui-protocol.md) via main/RackLink.
cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IDF_TARGET esp32c3)

# Shared code from the seed repo: the link protocol (still referenced by the
# untouched seed sources NetRest/DebugLink/DspProxyLink/LinkClient — they
# keep compiling until T5 stages 2+ rework them) and RapidJSON (needed by
# the ported menupages). STAGE-1 STOPGAP: these point at the READ-ONLY seed
# tree by absolute path because only the uimcu dir itself was copied; later
# stages vendor rapidjson and drop the seed-protocol dependency.
list(APPEND EXTRA_COMPONENT_DIRS
        "/home/schrab/Projects/ctag-tbd/protocol"
        "/home/schrab/Projects/ctag-tbd/components/rapidjson")

# pull in idf specific items, config vars etc.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(rack-uimcu)

# Same RapidJSON knobs as the A1S build — the ported pages are written
# against those semantics.
idf_build_set_property(COMPILE_DEFINITIONS -DRAPIDJSON_ALLOCATOR_DEFAULT_CHUNK_CAPACITY=4096 APPEND)
idf_build_set_property(COMPILE_DEFINITIONS -DRAPIDJSON_HAS_STDSTRING=1 APPEND)

# firmware version include (same mechanism as the A1S project)
execute_process(
        COMMAND bash -c "git describe --tags --abbrev=4 2>/dev/null || git rev-parse --short HEAD"
        OUTPUT_VARIABLE TBD_FW)
string(STRIP "${TBD_FW}" TBD_FW)
set(TBD_HW "C3-PANEL")
configure_file(${CMAKE_CURRENT_LIST_DIR}/version.hpp.in ${CMAKE_BINARY_DIR}/gen_include/version.hpp @ONLY)
idf_build_set_property(INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/gen_include APPEND)
