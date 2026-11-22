# HANDOFF — the benjolin port state (2026-11-22)

Read this plus:
- `docs/ASSESSMENT-benjolin-port-2026-11-22.md` (the audit §1–8 + the device findings §9)
- `docs/superpowers/plans/2026-11-22-benjolin-port.md` (the plan)
- `.superpowers/sdd/2026-11-22-benjolin-port/progress.md` (the ledger — every ruling + finding)

## What landed

- **The benjolin module (id 6)** boots as the rack's default module
  (`RACK_BOOT_MODULE=6`): the reference-exact port of the SC `\benjolis`
  SynthDef on the PicoCollider kit, the parameter specs = the EXACT norns
  `benjolis.lua` ControlSpecs (`module_benjolin.h` `kCcSpecs` — never invent
  ranges), the boot state = the exact norns default state (amp 0 — the
  SILENT boot; raise AMP to hear it, like norns).
- **The C3 UI**: 7 module slots (BENJOLIN in the strip + the SYS selector),
  the benjolin P1 page (13 cells, the full labels), the stepped FILTER
  (RLPF/RHPF — the norns UI never exposes SVF/DFM1) / OUTPUT (7 taps) /
  GAIN (0–6 step 1) selectors showing their selected names. esp-idf v6.1.
- **The browser panel**: 7 tabs (LIRA8 was MISSING — the gap that made the
  benjolin tab send CC 32 = 5 = Dronecaster), the benjolin knobs with the
  exact norns specs/notes, PAN removed (a delay-based pan on a sustained
  drone only comb-filters; the firmware still accepts CC 73 and boots at
  the norns center).
- **The Pico SDK scaffold** (`rack-pico/`, benjolin-first): CMake, the PIO
  I2S out on the rack pin map, 307.2 MHz, `-O3 -ffp-contract=off` (the flags
  the Arduino core could not give). Builds `rack_benjolin.uf2` (153 KB).
  The DSP engines include from `rack-rp2350/src` — the single source of
  truth. See `rack-pico/README.md`.
- **The kit laws found during the golden arbitration** (submodule
  `21dbf4f`/`02d3dd6`): the DPW and the core RLPF/RHPF have SEPARATE
  next_a (ar-freq wire) vs next_k (scalar) laws — the kit had only next_k;
  the ar-freq cases (the benjolin's FM-fed DPWs, the swept filter) were
  structurally wrong until the a-law was added. `midicps` is the FLOAT pow
  overload, per-sample on ar wires.

## The device-vs-golden rig (USE IT before trusting any device sound)

1. Flash (the boards attach to Windows via usbipd; see the flash section in
   AGENTS.md): RP2350 via `tools/flash_rp2350.py ... COM15` (or BOOTSEL),
   C3 via `idf.py -p /dev/ttyACM0 flash` (esp-idf v6.1).
2. Inject the A/B params over the console: `cc 60 <v>` … (the CC list in
   `tools/bjdump_capture.py` `CCS` — the user's reference settings are
   commented there).
3. `bjdump 64` → the post-Clip16 int16 pairs over CDC (the status line
   `bjdump 64 16384`, then the raw payload).
4. The host reference: `test/benjolin_ref/device_host.cc` (the same CC list
   → the same SpecValue math → the engine → the same int16 scaling),
   compiled `-ffp-contract=off` (the M33 has no FMA — the host must not
   contract either).
5. Diff. **Divergences = the M33/host rounding or a device-path bug** —
   they are NOT the chaotic-seed effect (that only appears vs norns, below).

`norns_ab.cc` = the structure probe (the RMS-per-window at the user's
settings — the "6 notes every 5 s" pattern check).

## The device findings (the user's report, §9 of the ASSESSMENT)

- **The budget**: worst blocks 1600–2000 µs vs the 1333 µs period. The avg
  (383) HIDES bimodal wedges — the MAX is the acceptance metric. The
  runglers at 0 → ~1300 µs. No I2S stalls (the bank absorbs), but the
  wedges are the suspected harshness cause.
- **The clipping root cause (FIXED)**: the engine's output = the
  tanh-saturated square (±1) through LeakDC, whose edge response
  overshoots to ±2 — at the old ×32768 scale everything above ±1.0
  flat-topped in int16 ("distorted like bit reduction"). The float goldens
  held ±1.72 fine — the int16 path clipped. Fixed: ×16384 = ±2.0 headroom.
  **Re-test the sound at the user's settings with the headroom fix in
  place before assuming the filter laws are wrong.**
- **DFM1 "completely wrong"** — NOT yet investigated. NOTE: the norns UI
  never exposes filterType 1–3 (SVF/DFM1 unreachable on norns) — ask the
  user what they A/B'd DFM1 against.
- **COMP silent on device but sounding on norns** — the shipped SynthDef's
  comparator is degenerate (the `sh8=1` var-capture collapses the intended
  XOR to `1−(osc1>0.5)`, which the SynthDef's own LeakDC silences). The
  golden proves the NRT behavior; norns differs (the live seed and/or a
  different engine version). The user's directive: faithful ⇒ sound.

## The open items (the user's order)

1. **Restore the intended XOR comparator** (the register top bit vs the
   threshold — the author's intent, the original Benjolin's signature) in
   the engine AND the renderer's SynthDef copy (a documented deviation),
   re-render the goldens, re-arbitrate.
2. **The filter investigation**: DFM1 A/B at the user's settings, the
   self-oscillation check (the RLPF at rq 0.18 rings on norns), the
   degenerate-freq clamps (the register fill sweeps the filter freq to
   ~111.6 kHz at boot — past Nyquist).
3. **The budget vs the MAX metric**: the dominant cost = the M33's software
   double math (the kit laws are double, ~40 double ops/sample). Options:
   the float-state conversion where the chaotic gates allow, per-UGen
   re-arbitration.
4. **The Pico SDK migration** (benjolin-first scaffold → the USB audio
   capture via the DSPi `DSPI_LOOPBACK` pattern → the module migration).
   The scaffold builds (`rack_benjolin.uf2`); the capture + the MIDI + the
   swap machinery are the next items.
5. **The C3 preset reflash**: the preset blob v6 (1359 B) needs the C3
   rebuilt with the raised kMaxRp — done in source, but the flashed C3
   predates it: preset save/load fails until the next C3 flash.

## The known-divergent

- **loop1** (loop=0.9): a different 8-bit register attractor than the
  golden (-8..-30 dB spectral, systematic) — every stage is golden-exact;
  report-only in the test; the user's ears decide. The chaotic seed is
  ulp-sensitive (the comparator threshold sits on the DPW ripple at
  0.5000±6e-6) — the environmental midicps ulp (the golden renderer's
  scsynth computes midicps(0) as 0x4102D013, the host powf 0x4102D012)
  makes the full-waveform golden unachievable past the first FM consume;
  the per-stage kit goldens (141 at 1e-6) are the exactness evidence.

## The gotchas

- The boards attach to Windows (usbipd); the flash script runs via the
  cmd.exe interop with `\\wsl.localhost\...` paths (cmd needs the doubled
  backslashes). A stuck reader on COM15 = Access denied — close the panel's
  Web Serial first.
- The console `cc` injections land at the block boundary; wait ~0.3 s
  before dumping.
- The 2 s report interleaves with the CDC output — the dump capture script
  handles it (the status marker + the payload read back-to-back from the
  blocking handler).
- The kit gate binaries: run them individually (the aggregate has the
  unrelated Elements reverb ASLR canary).
