/***************
Rack UI controller — T5 input: ONE quadrature encoder + push (the whole UI
is turn pots + one encoder; see docs/superpowers/plans/2026-11-04-combined-
rp2350.md "T5 UI/UX design" + rack-uimcu/README.md gesture timings).

Rewritten from the seed UserInput (ctag two-encoder + ADC-ladder buttons):
the SEED'S QUADRATURE APPROACH IS REUSED — GPIO-ISR quadrature with the
detent state table rejecting bounce (the C3 has no PCNT), counts drained by
the poll task, two quarter-cycles = one detent. The ADC button ladder and
the second encoder are GONE (the rack has no button ladder; one encoder).
The push button is a polled GPIO with the same 20 ms debounce discipline.

Hardware (rack-uimcu/README.md GPIO table):
  ENC A    = GPIO6      (external + internal pull-up)
  ENC B    = GPIO7      (external + internal pull-up)
                         — A/B moved OFF the strapping pins 2026-11-08 (user
                         call); the mux S3 address line took GPIO8 instead —
                         strap levels are don't-care in normal boot.
  ENC PUSH = GPIO9      (STRAPPING pin — external pull-up; holding push at
                         power-on reads 0 = download mode, do not)

Events (semantic, into the same queue shape the seed used):
  ENC_DELTA      rotate, delta = detents (+/-1)
  PUSH_SHORT     release < 600 ms (page-specific alternate action)
  PUSH_HOLD      held >= 600 ms, emitted ONCE (arms assign/overlay mode)
  PUSH_RELEASE   release after a HOLD (confirms the overlay / disarms)
***************/

#pragma once
#include <cstdint>

namespace CTAG {
    namespace CTRL {
        struct InputEvent {
            enum Type : uint8_t {
                ENC_DELTA = 0,
                PUSH_SHORT = 1,
                PUSH_HOLD = 2,
                PUSH_RELEASE = 3,
            };
            Type type;
            int16_t delta;   // only meaningful for ENC_DELTA
        };

        class UserInput {
        public:
            static void Init();
            // True while the push is physically down (the renderer shows
            // the armed state; the gesture logic keys off PUSH_HOLD).
            static bool IsPressed();
            // The push button's raw state, fed by RackPots's mux-scan press
            // detection (the push shares the mux SIG line through 4.7k).
            static void SetPushRaw(bool pressed);
            static bool GetEvent(InputEvent &ev, uint32_t timeoutMs = 10);
            static bool PeekEvent(InputEvent &ev);
            // Debug support: push a synthetic event straight into the same
            // queue the hardware path writes to.
            static bool InjectEvent(InputEvent::Type type, int16_t delta = 0);
        };
    }
}
