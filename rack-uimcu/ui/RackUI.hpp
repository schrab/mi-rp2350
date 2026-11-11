/***************
Rack UI controller — T5 stages 2-6: the rack's fixed-page renderer.

WHY A SIMPLE FIXED-PAGE RENDERER REPLACES THE SEED UIMenu/menupages
FRAMEWORK (the plan allows either, "justify"): the seed framework is built
around ctag's two-encoder ROOT/PANEL_IN navigation, a back button and
DspProxy-backed scrolling item lists — none of which exist in the rack UI
(NO menus, ONE encoder whose rotate = page cycle, every param always
visible, TELEM-driven cells, hold-gestures). The ctag page CONTENT is 100%
DspProxy calls that have no rack counterpart. Keeping UIMenuPage's virtual
interface would mean six subclasses that ignore most of it. What the design
actually needs is: poll the TELEM cache + the input queue at 30-60 Hz,
redraw changed cells, run the gesture state machine — that is this file
(~600 lines, no allocation, no framework).

Pages (fixed, encoder-rotated — docs/ui-protocol.md §4/§6):
  P1/P2     the module's 16 CC cells (registry, TELEM-driven)
  MOD-A/B/C the mod engine's cells (pot -> MOD_SET [src][param], the
            kModPotMap below MUST MATCH the RP2350's kUiMod*Cells)
  SYS       4 module cells (hold+rotate overlay = select, SWITCH on
            release), chain pair/engage, telemetry gates, C3-local preset
            slot (PRESET_REQ, T4a stub); short push = the SYS action ring

Screen layout (284x76, mirrors the 3 physical pot rows 6/4/6): a ~12 px
status strip (module + page, clock/BPM, chain, assign/overlay indicator,
BLE, MIDI activity dot), then 3 cell rows; each cell = label (digi-slim
6 px) over value (digi-one 5 px) over a 2 px 0-127 underline bar.

Color rules (<= 4 hues): white/red/teal = the cell's registry class byte;
yellow = STATE accent ONLY (strip assign indicator + overlay, routed-cell
value/bar, SYS ring marker); chain-B cells (flags bit1) DIM their class
color 50%; inert cells (flags bit2) BLINK at 2 Hz. Where TELEM has not
arrived the renderer draws dashes — it never invents values.
***************/

#pragma once

#include <cstdint>

namespace CTAG {
    namespace CTRL {
        class RackUI {
        public:
            // What a pot position maps to on the ACTIVE page right now.
            struct PotTarget {
                enum Kind : uint8_t { kNone = 0, kCC = 1, kMod = 2,
                                      kGlobal = 3, kSys = 4 };
                Kind kind;
                uint8_t cc;        // kCC: the cell's MIDI CC (P1/P2)
                                   // kGlobal/kSys: selector item id
                uint8_t src;       // kMod: mod source id (MOD-A/B/C)
                uint8_t param;     // kMod: kModParamNames index
                uint8_t value;     // the cell's current value in 0-127 pot space
                uint8_t routedSrc; // kCC + mod-routed only: source id + 1
                                   // (0 = none). The pot then RIDES that
                                   // source's speed param instead of writing
                                   // this CC (RackPots.hpp).
            };

            static void Init();          // state + first page request
            static void TaskFunction(void *);   // 30-60 Hz render loop

            // Page switch (encoder rotate); sends the page request, renders
            // dashes until the TELEM snapshot lands.
            static void SetPage(uint8_t page);
            static uint8_t GetPage();
            static uint8_t GetModule();

            // True while the mod-assign gesture is armed (push held).
            static bool AssignModeActive();

            // RackPots asks for the ACTIVE page's cell at a pot position.
            static bool GetPotTarget(uint8_t pot, PotTarget &out);

            // Convert between RP2350 wire byte and C3 0-127 full-turn pot space.
            static uint8_t WireToPotValue(uint8_t page, uint8_t mod,
                                          uint8_t slot, uint8_t cc,
                                          uint8_t wireVal);
            static uint8_t PotToWireValue(uint8_t page, uint8_t mod,
                                          uint8_t slot, uint8_t cc,
                                          uint8_t potVal);

            // Execute a pot value change (0-127 pot space) for target `t`.
            static void ApplyPotAction(uint8_t pot, const PotTarget &t,
                                       uint8_t potVal);

            // Return bucket bounds [lo, hi] in 0-127 pot space for discrete
            // cells so soft-takeover aligns to bucket boundaries, not 0/127.
            static void GetPotBucketBounds(uint8_t pot, const PotTarget &t,
                                           uint8_t &lo, uint8_t &hi);

            // Track active / assigned pot for top-bar mod source display.
            static void NoteActivePot(uint8_t pot);
            static void NoteAssignPot(uint8_t pot, uint8_t srcIdxPlus1);
            static void NoteModRide(uint8_t pot, uint8_t src, uint8_t param, uint8_t val);

            // `ui` console dump.
            static void DebugState();
        };
        // (All UI state is file-local in RackUI.cpp — no cross-TU access.)
    }
}
