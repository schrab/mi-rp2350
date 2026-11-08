# Panel firmware main component. PRIV_REQUIRES stays empty on purpose
# (main auto-depends on every component in the build; see the A1S
# main/CMakeLists.txt lesson — non-empty PRIV_REQUIRES breaks includes).
set(UI "${CMAKE_CURRENT_LIST_DIR}/../ui")

option(UIMCU_DSP_STUB "Serve fake DSP state locally (P2 bench mode, no link)" OFF)

if(UIMCU_DSP_STUB)
    set(DSP_PROXY_IMPL "${UI}/DspProxyStub.cpp")
else()
    set(DSP_PROXY_IMPL "${UI}/DspProxyLink.cpp" "LinkClient.cpp")
endif()

idf_component_register(
        SRCS
            "app_main.cpp"
            "DebugLink.cpp"
            "MidiLink.cpp"
            "NetRest.cpp"
            "BtMidiReceiver.cpp"
            "RackLink.cpp"
            "input/UserInput.cpp"
            "${DSP_PROXY_IMPL}"
            "${UI}/Display.cpp"
            "${UI}/UIMenu.cpp"
            "${UI}/menupages/UIMenuPageHome.cpp"
            "${UI}/menupages/UIMenuPageMix.cpp"
            "${UI}/menupages/UIMenuPageMod.cpp"
            "${UI}/menupages/UIMenuPageParams.cpp"
            "${UI}/menupages/UIMenuPageSystem.cpp"
            "${UI}/menupages/UIMenuPageTape.cpp"
            "${UI}/menupages/UIMenuPageMidi.cpp"
        INCLUDE_DIRS "." "${UI}")

if(UIMCU_DSP_STUB)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC UIMCU_DSP_STUB=1)
endif()
