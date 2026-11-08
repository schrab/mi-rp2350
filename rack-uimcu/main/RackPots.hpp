/***************
Rack UI controller — T5 stage 3: 16 analog pots on one HC4067 (1x ADC1 +
4 address lines), send-on-move with NVS calibration.

Hardware (rack-uimcu/README.md has the full GPIO allocation + wiring):
  mux COM -> GPIO0 = ADC1_CH0 (ADC1 only: ADC2 is unusable while NimBLE
             holds the radio; GPIO4 (ADC1_CH4) is the UART link TX).
  addr S0..S3 = GPIO1 / GPIO2 / GPIO3 / GPIO6 (GPIO2 is a strapping pin:
             external pull-up required; the line floats harmlessly until
             firmware drives it — a floating mux address only picks which
             pot the ADC sees, never a boot risk).
  mux E (enable, active low) tied GND; 16 pots on CH0..CH15, physically
  3 rows: top 6 (pots 0-5) / middle 4 (pots 6-9) / bottom 6 (pots 10-15)
  — the screen mirrors this 6/4/6 layout, pot index == cell index.

Scan: round-robin at ~700 sweeps/s (address settle ~8 us + one averaged
ADC read per channel), 0-127 mapping through the per-pot min/max
calibration (NVS namespace "rpots", key "cal", 16 x {u16 min, u16 max}).
Uncalibrated pots use the full raw range (0..4095) so everything works
before `calib` ever ran.

Send policy (docs/ui-protocol.md + README): deadband 2 counts vs the last
SENT value, aggregate rate limit 200 msgs/s (token bucket), P1/P2 pages
send MIDI CC frames (the ACTIVE page cell's CC at that pot position),
MOD pages send MOD_SET [src][param][value] (the kModPotMap, MUST MATCH the
RP2350's kUiMod*Cells), SYS page pots are inert. While the mod-assign
gesture is armed (RackUI::AssignModeActive), pot moves become
UI_CMD ASSIGN_CYCLE [pot][dir from the move direction] instead.
***************/

#pragma once
#include <cstdint>

namespace CTAG {
    namespace CTRL {
        class RackPots {
        public:
            static void Start();

            // `calib` console: collect min/max for all 16 pots over an 8 s
            // window (sweep every pot full range while it runs), store to
            // NVS, log the result. `calib clear` resets to full range.
            static bool BeginCalibration();
            static bool CalibrationRunning();
            static void ClearCalibration();

            // Debug: raw + mapped value of one pot (console `pots`).
            static int DebugRaw(uint8_t pot);
            static int DebugValue(uint8_t pot);
        };
    }
}
