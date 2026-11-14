# Rack RP2350 — hardware PLAY button + piezo hit sensor (Ears-style, GPIO/ADC)

Goal: give the rack a PHYSICAL trigger surface independent of MIDI, mapped
parameters and the mod engine: a momentary push button that fires the active
module's exciter (strike/trig/strum/trig-pulse) exactly like the adapters'
play/trig pseudo-CCs, and later a piezo disc read by the RP2350 ADC that adds
hit VELOCITY — a Mutable Instruments Ears stand-in implemented with GPIO +
the RP2350 ADC (no PIO, no audio-in hardware; the PCM5102A is output-only and
stays untouched).

Supersedes nothing. The four per-port plans and the combined plan remain the
audio truth; this plan only ADDS a core-0 input surface that injects through
the EXISTING trigger dispatch.

The piezo stage is a SOFTWARE REPLICA of Mutable Instruments Ears' analog
blocks (ref: pichenettes.github.io/mutable-instruments-documentation/modules/ears/
— Ears is all-analog, no MCU): precision rectifier → envelope follower (env
"CV", 3 attack/release presets) → gate comparator (3 sensitivity presets).
Its amplification stage (1 M input, up to 40 dB gain) is deliberately
OMITTED per user decision — the digital `piezo gain` parameter replaces it.

## Executing from a fresh session

Read, in order: **this doc** (decisions + rationale are final), the
**`.superpowers/sdd/progress.md`** rack sections (trigger-dispatch facts,
one-task-per-session protocol), and the trigger-path code referenced below
(`midi.h`, `module_registry.h`, `module_*.h` `mod_trigger_cc`/`control_change`
pseudo-CC handlers). Hardware ops (wiring, flashing, listening) are the
user's + session agent's, never subagents'. Commit `[Add]/[Fix]/[Update]`
style on `rack-rp2350`; update the ledger (status, hash, findings) at every
landing. Never claim hardware-verified without the user's ears/eyes.

## Decisions (brainstorm 2026-11-14)

| Decision | Choice | Why |
|---|---|---|
| Button semantics | **Momentary gate+strike**: press edge fires the strike/trig pulse; gate HIGH while held; release = gate LOW | Matches a physical play button; maps 1:1 onto elements/plaits CC 64 latch semantics (`val >= 64` = gate + pulse, `< 64` = release) |
| Clouds mapping | **Trig pulse on press + exciter burst retrigger** (its dedicated `mod_clouds::button_trig` twin — CC 66 itself stays pure for mod routing), NOT freeze latch | Hardware-bench finding (2026-11-14): clouds is an EFFECT — a bare trig granulates the buffer, and an idle single-module buffer is silent (the internal exciter bursts only on a MIDI note-on rising edge), so the trig correctly fired but was inaudible. The button therefore also fires the burst (what a note-on does), making press audible from idle. Elements/plaits/rings generate sound from their pseudo-CC alone |
| Independence | Button NEVER touches `note_stack`, `shared_gate`, ARP state, or mod-engine sources | User requirement: "independent of mapped parameters"; injection goes BELOW the MIDI funnel's intercepts (see mechanism) |
| Mod-engine source | **Deferred** until piezo works | Keeps T-scope minimal; the Ears outputs (gate level + envelope) will already exist as signals to expose |
| Piezo | **Software Ears with CONTINUOUS parameter ranges** — Ears' attack/release and sensitivity are FIXED jumper presets; we generalize each to a controllable value range: abs-rectifier → one-pole envelope follower (controllable attack + release) → Schmitt gate detector (controllable sensitivity threshold + hysteresis); digital gain replaces the omitted 40 dB amp path; velocity = envelope peak at gate edge (our extension — Ears outputs no velocity) | Ears is all-analog, so its transfer behavior ports 1:1 to DSP at 24 kHz; ranges cover the jumper settings and beyond; follower smoothing + Schmitt hysteresis inherently reject piezo ring-down chatter (Ears' own analog answer); tap = short gate, rub/scratch = sustained gate — the ridges/strumming behavior falls out for free |
| Phasing | **Two phases**: T1 button lands first, T2–T4 piezo after | Fast user-visible win; piezo calibration needs bench time |

## Hardware

### Wiring (user does the physical work; arm tests only after they report ready)

| Signal | RP2350 pin | Notes |
|---|---|---|
| PLAY button | **GP2** (input, internal pull-up, button → GND) | GP3 is the spare alternate. Verify GP2 is unpopulated/unused on the nano board at wiring time |
| Piezo | **GP26 = ADC0** | Piezo ∥ 1 MΩ bleed → **10 kΩ series → GP26**; 1N4148 clamps to 3V3 and GND. The series R is MANDATORY even at bench (a tap can spike tens of volts; 10 kΩ limits ESD-clamp current). Bench = jumper wires + clamp diodes; solder only at final wiring |
| Spares | GP3, GP27 (ADC1) | Second button / second piezo later |

No pull-up resistor needed for the button (internal pull-up); no op-amp for
the piezo in v1 (add a buffer only if the bench range is bad).

### Pin-map context (used pins, do not touch)

GP0/GP1 UI-link UART0 · GP4 (DIN TX reserved) / GP5 DIN RX UART1 · GP6/7/8
I2S · GP14/15 USB-host socket (wired, disabled: `RACK_ENABLE_USB_HOST=0`) ·
GP22 WS2812. Free: GP2, GP3, GP9, GP26–29.

## Mechanism (T1) — injection point

The trigger path that already exists, per adapter:

- elements: CC 64 PLAY — `pending_play_pulse` (one-block strike) +
  `play_latched_gate` (drone) + `play_gate_counter` (module_elements.h:315,
  consumed at `perf_state.gate = shared_gate || play_latched_gate ||
  pulse_active`)
- plaits: CC 64 — `trig_latched` + `pending_trig_pulse` (module_plaits.h:396)
- rings: CC 64 STRUM — one-block `pending_strum` (module_rings.h:515)
- clouds: CC 66 TRIG — one-block `pending_trigger` (module_clouds.h:459);
  CC 65 FREEZE (latch) deliberately NOT button-mapped

The button handler (new, main.cpp, core 0):

```cpp
// rack_button_trig(bool down, uint8_t vel = 127)  — vel consumed by T3
// press:   rack_module_control_change(kButtonCc(active), 127)
// release: rack_module_control_change(kButtonCc(active), 0)
// kButtonCc: elements/plaits/rings -> 64, clouds -> 66
```

CRITICAL: call `rack_module_control_change` (registry, below the intercepts)
directly — NOT `midiControlChange`. While chained, `midiControlChange` would
hand CC 66 to the clouds-B intercept (`rack_chain_wants_cc`, main.cpp:572);
the registry path always hits the ACTIVE (A-slot) adapter, which is what a
physical play button must do (A's audio feeds the chain anyway).

Velocity (T3) mirrors each adapter's own note_on velocity mapping:
elements `shared_strength = 0.4 + 0.6·v`, plaits
`pending_trigger = 0.3 + 0.7·v` (module_plaits.h:352), rings/clouds ignore
velocity. For the piezo, velocity = normalized envelope peak at the gate
edge.

### Piezo pipeline (T2/T3) — Ears' analog blocks in software (core 0)

Ears is all-analog: piezo → amp (omitted) → envelope follower (env CV) →
gate comparator (gate). Each block ports directly to 24 kHz sample-domain
code:

1. **Rectifier**: `abs(sample)` (Ears: precision rectifier op-amp).
2. **Envelope follower**: one-pole per sample — rising toward the rectified
   sample with the ATTACK coefficient, decaying with the RELEASE coefficient.
   Ears fixes both behind jumpers (3 presets); we expose CONTINUOUS ranges:
   `piezo atk <ms>` (0.1–100) and `piezo rel <ms>` (10–2000), defaults
   ≈ 5 ms / 500 ms (mid-jumper-span). The release setting IS the tap-gate
   length (longer release = longer gates) — exactly Ears' jumper behavior,
   made continuous.
3. **Gate detector**: Schmitt on env — ON at `th_on`, OFF at
   `th_on − hyst`. Ears fixes sensitivity behind a jumper (3 presets); we
   expose ranges: `piezo sens <0–100>` (% of post-gain full scale, default
   ≈ 15) and `piezo hyst <0–50>` (% of th_on, default ≈ 30). Piezo
   ring-down chatter is suppressed by follower smoothing + hysteresis, the
   same mechanism Ears' RC network provides; an optional `piezo mask <ms>`
   min-retrigger spacing (default 0 = off) stays as a bench escape hatch.

All five parameters (atk / rel / sens / hyst / gain) are PERSISTENT rack
state: `preset_mark_dirty()` on change + fields in the preset blob
serialize/apply — the same mechanism mod_engine uses for its persistent
state (mod_engine.h:2574), so settings survive reboots like everything else
in the rack.
4. **Events** (our extension beyond Ears — Ears outputs no velocity):
   - gate RISING edge → strike/strum/trig pulse via
     `rack_button_trig(true, vel)`; velocity = peak env since the previous
     edge (peak-hold, reset on each edge), calibrated to 0–127
   - gate LEVEL: elements PLAY latch held high while the gate is high
     (scratch/rub = sustained drone — the Ears panel-ridges behavior), drops
     when the gate drops; pulse-only modules (plaits/rings/clouds) simply
     ignore the level
   - the raw envelope value + gate level are RETAINED (they ARE Ears' ENV
     and GATE CVs) — future mod-engine sources, see Deferred

Physical mount (musical, Ears-faithful): piezo taped under the case panel /
a metal plate turns the whole surface into the contact mic; scratching
ridged or uneven surfaces generates impulse combs — one rising edge per
ridge — naturally strumming Rings/Elements at scratch rate.

Digital gain (`piezo gain`, range 1–16×) scales the rectified samples
BEFORE the follower, standing in for Ears' 40 dB amp; sensitivity is
post-gain, so gain × threshold behaves like Ears' gain knob × sensitivity
jumper.

## Tasks

| Task | Scope | Acceptance gate |
|---|---|---|
| **T1** Button: wiring + firmware injection | GP2 input + internal pull-up; 30 ms millis() debounce poll in core-0 `loop()` (after `ui_link_pump()`); `rack_button_trig()` per the mechanism above; serial `button down/up` edge prints; console command `trig` (synthetic press/release for headless bench); `btn=` counter appended to the 2 s report | User: press on each of the four modules sounds IDENTICAL to sending CC 64/66 val 127 from the web panel; hold = elements drone, release stops it; plaits `trig_latched` clears on release; with ARP engaged the button still strikes and does NOT transpose; MIDI note stack unaffected |
| **T2** Piezo bring-up + ADC read-out | GP26 network per wiring table; ADC free-run + DMA ~24 kHz into a 512-sample ring (one DMA channel, core 0); console `piezo` command: raw min/max/last-peak/noise-floor print; `piezo gain` digital gain (replaces Ears' omitted amp stage); `piezo=` field in the 2 s report | User taps piezo → peaks clearly above idle floor in the report at every gain setting; idle shows no phantom peaks; ADC read does not disturb audio (stall/proc counters unchanged) |
| **T3** Ears pipeline: env follower + gate detector + velocity injection | Software blocks per the pipeline section: abs-rectifier, one-pole follower with CONTINUOUS `piezo atk`/`piezo rel` ranges, Schmitt gate with CONTINUOUS `piezo sens`/`piezo hyst` ranges, optional `piezo mask`; edge → `rack_button_trig(true, vel)` with velocity = peak env; gate level → elements PLAY latch while held; env + gate state retained for future mod sources; all params persisted via the preset blob (`preset_mark_dirty()` pattern) | User: tap = short decaying gate + strike, hardness audibly scales elements strength / plaits accent; rub/scratch = sustained elements drone and strums Rings at scratch rate; ring-down does not double-fire (by ear); deliberate double-tap retriggers; settings survive reboot; button and piezo coexist without interference |
| **T4** Polish + docs | Optional WS2812 flash on trigger (respect existing LED modes/idle dim-green); rack_panel.html momentary PLAY button per active tab (sends the tab's pseudo-CC 64/66 — panel parity with the physical button); OPTIONAL panel/CC exposure of the piezo params (free CC band 36–39/51–63, sliders in the panel's clouds-style area); README wiring table + behavior docs (incl. piezo-under-panel mounting + scratch tip); ledger sections | User confirms panel PLAY == hardware button behavior; docs match firmware |

Deferred (recorded, NOT in scope): expose the Ears outputs as mod-engine
sources — **GATE3** (piezo/button gate level, routing to any destination)
and **ENV** (the envelope follower value as a continuous source, the direct
equivalent of Ears' env CV out) — after T3 is ear-accepted; second piezo /
button on GP27/GP3; TRIG-jack input via optocoupler on a spare GPIO.

## Known non-goals

- No audio-in hardware changes; PCM5102A path untouched.
- No analog amplification stage: Ears' 1 M-input / 40 dB amp is deliberately
  omitted (user decision); its role is covered by the digital `piezo gain`.
- No MIDI output of button/piezo events (pure local injection).
- No PIO involvement anywhere in this feature (ADC has its own DMA; button is
  a polled GPIO).
- No per-sample hot code added to core 1 — everything here lives on core 0
  at loop()/poll rate. No new `.tc_*` sections, no linker-script changes.
