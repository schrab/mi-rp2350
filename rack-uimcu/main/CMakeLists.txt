# Rack UI controller main component (T5 stages 2-6). PRIV_REQUIRES stays
# empty on purpose (main auto-depends on every component in the build; see
# the A1S main/CMakeLists.txt lesson — non-empty PRIV_REQUIRES breaks
# includes).
#
# BUILT (the rack firmware):
#   app_main.cpp        boot flow / task wiring
#   RackLink.*          the RP2350 framed UART link (stage 1, extended)
#   RackPots.*          HC4067 16-pot scan + NVS calibration + send-on-move
#   RackConsole.*       USB-Serial-JTAG line console (calib/testpat/ui/heap)
#   input/UserInput.*   quadrature encoder + push (rack GPIO map, gestures)
#   ui/Display.*        ST7789P3 284x76 RGB565 backend (dirty-row flush)
#   ui/RackUI.*         fixed pages P1/P2/MOD-A/B/C/SYS + TELEM renderer
#   BtMidiReceiver.*    NimBLE PERIPHERAL: BLE-MIDI -> RackLink::SendMidi
#
# EXCLUDED (seed reference, kept in tree, NOT built — headers carry the
# note "seed reference, not built"): LinkClient.*, DebugLink.*, MidiLink.*,
# NetRest.* (all four include the seed's ctag/link/protocol.hpp — the seed
# `protocol` component dependency was dropped in stage 2-6 cleanup; the rack
# link is RackLink), ui/DspProxy*.{hpp,cpp} + ui/UIMenu*.* +
# ui/menupages/* (the ctag DspProxy/menu framework was replaced by the
# rack's fixed-page RackUI renderer — justified in ui/RackUI.hpp).
# ui/font5x7.h / font8x8_basic.h stay in tree (unused seed font data).
set(UI "${CMAKE_CURRENT_LIST_DIR}/../ui")

idf_component_register(
        SRCS
            "app_main.cpp"
            "RackLink.cpp"
            "RackPots.cpp"
            "RackConsole.cpp"
            "BtMidiReceiver.cpp"
            "input/UserInput.cpp"
            "${UI}/Display.cpp"
            "${UI}/RackUI.cpp"
        INCLUDE_DIRS "." "${UI}")
