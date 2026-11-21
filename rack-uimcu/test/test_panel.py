#!/usr/bin/env python3
"""Host regression checks for panel state, with ESP/flash/UART calls stubbed.

Compile PresetStore.cpp and the relevant production UI/link functions directly;
no ESP-IDF or attached hardware is needed. Run: python3 rack-uimcu/test/test_panel.py
"""
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def function(text, signature):
    start = text.index(signature)
    end = text.index("{", start) + 1
    depth = 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


ui = (ROOT / "rack-uimcu/ui/RackUI.cpp").read_text()
link = (ROOT / "rack-uimcu/main/RackLink.cpp").read_text()
pots = (ROOT / "rack-uimcu/main/RackPots.cpp").read_text()

source = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "RackUI.hpp"
#include "RackLink.hpp"
#include "PresetStore.hpp"
#include "PresetBank.hpp"
using namespace CTAG::CTRL;
#define ESP_LOGW(...) ((void)0)
#define taskENTER_CRITICAL(...) ((void)0)
#define taskEXIT_CRITICAL(...) ((void)0)
#define portENTER_CRITICAL(...) ((void)0)
#define portEXIT_CRITICAL(...) ((void)0)
constexpr uint8_t NPOTS = 16;
constexpr uint8_t kModuleCount = 6;
int64_t timeUs = 100000;
extern "C" int64_t esp_timer_get_time() { return timeUs; }
RackTelemPage s_telem{}, s_cur{};
uint8_t s_page, s_activePot, s_assignPot, s_pendingAssignPot, s_pendingAssignSrc, requestedPage;
uint16_t s_lastWireSent[16];
bool s_telem_dirty, s_fullArrived, s_assignArmed, s_pendingAssign, s_pageDirty, s_stripDirty;
std::string banner;
bool online = true, tx = true, flash = true, hasRecord = false, bankReady = true;
unsigned applies = 0;
std::vector<uint8_t> manual, automatic;
uint8_t storedSlot = 255, requestedSlot = 255, optimistic = 1;
uint8_t pageBank() { return 0; }
uint8_t rideParam(uint8_t) { return 1; }
uint8_t destParam(uint8_t) { return 7; }
uint8_t s_potModSrc[3][NPOTS], lastValue[NPOTS];
int64_t s_lastAssignUs[NPOTS];
struct RackPots {
    static bool RecentlyMoved(uint8_t) { return false; }
    static bool AssignRecentlyChanged(uint8_t) { return false; }
    static void PickupResync() {}
    static void PickupCellNote(uint8_t, uint8_t) {}
};
struct Write { uint8_t src, param, val; };
std::vector<Write> writes;
void RackUI::ShowBanner(const char* text, uint32_t) { banner = text; }
void RackUI::NoteAssignPot(uint8_t, uint8_t) {}
void RackUI::NoteModRide(uint8_t, uint8_t, uint8_t, uint8_t) {}
bool RackUI::GetPotTarget(uint8_t, PotTarget& t) {
    t.kind = PotTarget::kCC; t.routedSrc = 1; t.cc = 11; return true;
}
bool RackLink::IsOnline() { return online; }
bool RackLink::TelemGet(RackTelemPage& page) { page = s_telem; return page.valid; }
bool RackLink::SendPageRequest(uint8_t page) { requestedPage = page; return true; }
void RackLink::TelemLocalAssignWrite(uint8_t, uint8_t src) { optimistic = src; }
bool RackLink::SendModSet(uint8_t src, uint8_t p, uint8_t val) {
    writes.push_back({src, p, val}); return true;
}
bool RackLink::SendPresetReq(uint8_t slot) { requestedSlot = slot; return tx; }
bool RackLink::SendPresetApply(const uint8_t*, uint16_t) { ++applies; return tx; }
bool PresetBank::Init() { return bankReady; }
bool PresetBank::Ready() { return bankReady; }
void PresetBank::ClearAll() {}
uint16_t PresetBank::AutosaveSeq() { return 1; }
bool PresetBank::SaveSlot(uint8_t slot, const uint8_t* v, uint16_t n, const uint8_t*, uint16_t) {
    storedSlot = slot; manual.assign(v, v + n); return flash;
}
bool PresetBank::SaveAutosave(const uint8_t* v, uint16_t n, const uint8_t*, uint16_t) {
    automatic.assign(v, v + n); return flash;
}
bool PresetBank::LoadSlot(uint8_t, uint8_t* v, uint16_t* n, uint8_t* c3, uint16_t* cn) {
    if (!hasRecord) return false;
    *v = 42; *n = 1; std::memset(c3, 0, 12); c3[0] = PresetBank::kC3PartVersion;
    *cn = 12; return true;
}
bool PresetBank::LoadAutosave(uint8_t*, uint16_t*, uint8_t*, uint16_t*) { return false; }
'''

for text, signature in [
    (ui, "static inline bool PageIsModule("),
    (ui, "static uint8_t p1p2DiscreteCount("),
    (ui, "uint8_t RackUI::WireToPotValue("),
    (ui, "uint8_t RackUI::PotToWireValue("),
    (ui, "static int potToDisplayValue("),
    (ui, "void RackUI::SetPage("),
    (link, "void parseTelem("),
    (pots, "void applyModAssign("),
]:
    source += function(text, signature) + "\n"

# Exercise the production snapshot transition without starting the render task.
start = ui.index("const bool wasFull =", ui.index("// Poll the TELEM cache."))
end = ui.index("s_stripDirty = true;", start)
source += "void applySnapshot(RackTelemPage tmp) {\n"
source += ui[start:end].replace("SetPage(", "RackUI::SetPage(") + "\n}\n"
source += r'''
void part(uint8_t slot, uint8_t seq, uint8_t total, uint8_t val, uint8_t n) {
    uint8_t p[255] = {slot, seq, total};
    memset(p + 3, val, n); PresetStore::OnPresetPart(p, n + 3);
}
void pageGone(uint8_t page, uint8_t was, uint8_t now) {
    const uint8_t old[] = {page, was, 0, 1, 39, 10, 0, 0, 'X', 0, 120, 0, 1, 2};
    const uint8_t gone[] = {page, now, 0, 0, 120, 0, 0, 2};
    s_page = page; parseTelem(old, sizeof(old)); s_cur = s_telem; s_fullArrived = true;
    parseTelem(gone, sizeof(gone));
    assert(s_telem.valid && s_telem.module == now && s_telem.count == 0);
    assert(s_telem.chain == 0 && s_telem.cell[0].cc == 0);
    requestedPage = 255; applySnapshot(s_telem);
    assert(s_page == 0 && requestedPage == 0 && !s_fullArrived);
}
int main() {
    for (uint8_t id = 0; id < 98; ++id) {
        int bar, max;
        auto pot = RackUI::WireToPotValue(0, 2, 0, 23, id);
        assert(RackUI::PotToWireValue(0, 2, 0, 23, pot) == id);
        assert(potToDisplayValue(0, 2, 0, 23, pot, bar, max) == id && bar == id && max == 97);
    }
    puts("Plaits: all 98 IDs round-trip and display correctly");
    pageGone(10, 4, 2); pageGone(1, 0, 1); pageGone(9, 0, 0);
    const uint8_t badDelta[] = {0, 3, 1, 0};
    parseTelem(badDelta, sizeof(badDelta)); assert(s_telem.page == 9);
    // Loading a preset may request P3 before its Lira8 module swap finishes.
    const uint8_t waiting[] = {10, 2, 0, 0, 120, 0, 0, 2};
    s_page = 10; s_fullArrived = false; parseTelem(waiting, sizeof(waiting));
    applySnapshot(s_telem); assert(s_page == 10 && !s_fullArrived);
    const uint8_t ready[] = {10, 4, 0, 1, 39, 10, 0, 0, 'X', 0, 120, 0, 0, 2};
    parseTelem(ready, sizeof(ready)); applySnapshot(s_telem);
    assert(s_page == 10 && s_fullArrived && s_cur.module == 4);
    puts("TELEM: removed pages fall back safely; pending preset page waits for its module");
    s_potModSrc[0][0] = 1; s_potModSrc[0][1] = 2;
    applyModAssign(0, 2);
    assert(banner == "SRC IN USE" && writes.empty() && optimistic == 1 && s_lastAssignUs[0] == 0);
    applyModAssign(0, 3);
    assert(writes.size() == 2 && writes[0].src == 0 && writes[0].val == 0);
    assert(writes[1].src == 2 && writes[1].val == 11 && optimistic == 3);
    puts("Mod assign: refused assignment sends nothing; valid replacement still works");
    assert(PresetStore::SaveTo(3) && banner == "SAVING P3" && requestedSlot == 3);
    part(255, 0, 2, 11, 252); part(3, 0, 2, 22, 252); part(255, 1, 2, 12, 1);
    assert(automatic.size() == 253 && automatic.front() == 11 && automatic.back() == 12);
    assert(manual.empty() && banner == "SAVING P3");
    part(3, 1, 2, 23, 1);
    assert(storedSlot == 3 && manual.size() == 253 && banner == "SAVED P3");
    assert(manual.front() == 22 && manual.back() == 23);
    assert(PresetStore::SaveTo(4));
    part(255, 0, 0, 5, 1); part(3, 0, 0, 5, 1); assert(banner == "SAVING P4");
    flash = false; part(4, 0, 1, 1, 1); assert(banner == "SAVE FAILED P4");
    assert(PresetStore::SaveTo(5)); timeUs += 3000001; PresetStore::Poll();
    assert(banner == "SAVE TIMEOUT P5");
    online = false;
    assert(!PresetStore::SaveTo(6) && banner == "RACK OFFLINE");
    assert(!PresetStore::LoadFrom(6) && applies == 0);
    online = true;
    assert(!PresetStore::LoadFrom(6) && banner == "EMPTY/BAD P6" && applies == 0);
    tx = false; assert(!PresetStore::SaveTo(7) && banner == "SAVE FAILED P7");
    hasRecord = true; assert(!PresetStore::LoadFrom(7) && banner == "LOAD FAILED P7");
    tx = true; assert(PresetStore::LoadFrom(7) && banner == "LOAD SENT P7");
    bankReady = false; assert(!PresetStore::SaveTo(8) && banner == "BANK UNAVAILABLE");
    assert(!PresetStore::LoadFrom(8) && banner == "BANK UNAVAILABLE");
    puts("Presets: interleaved chunks isolated; save failures, timeouts and load outcomes reported correctly");
}
'''

with tempfile.TemporaryDirectory(prefix="rack-panel-", dir="/tmp/opencode") as directory:
    build = Path(directory)
    (build / "check.cpp").write_text(source)
    (build / "esp_log.h").write_text(
        "#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGI(...) ((void)0)\n"
    )
    (build / "esp_timer.h").write_text(
        '#include <cstdint>\nextern "C" int64_t esp_timer_get_time();\n'
    )
    subprocess.run([
        "g++", "-std=c++20", "-I" + directory,
        "-I" + str(ROOT / "rack-uimcu/main"),
        "-I" + str(ROOT / "rack-uimcu/ui"), str(build / "check.cpp"),
        str(ROOT / "rack-uimcu/main/PresetStore.cpp"), "-o", str(build / "check"),
    ], check=True)
    subprocess.run([str(build / "check")], check=True)
