# mi-rp2350 — rack synth monorepo

One RP2350 rack synth, five live-switchable modules, one unified firmware.

- **[`rack-rp2350/`](rack-rp2350/README.md)** — the synth firmware (PlatformIO,
  Arduino/earlephilhower core on an RP2350 "nano" + PCM5102A I2S DAC):
  Elements, Clouds, Plaits (98 engines), Rings MiniElements and the
  hand-ported LIRA-8, live-switchable at runtime (MIDI CC 32 / serial
  console), plus distortion + Warps FX and an ESP32-C3 UI link.
- **[`rack-uimcu/`](rack-uimcu/README.md)** — the ESP32-C3 UI firmware
  (ESP-IDF): pages, gestures, link protocol to the RP2350.
- **`tools/`** — browser control panel (`rack_panel.html`), legacy
  single-module panels, SysEx converter.
- **`docs/`** — SDD plans and porting notes; `.superpowers/sdd/progress.md`
  is the ledger (read it before debugging anything).

The vendored Mutable Instruments DSP libs in `rack-rp2350/lib/` are **the
source of truth** — they are edited directly. Upstream MI code is frozen and
the pristine trees / patch-sync layer were removed on 2026-11-18; the git
history still holds the pristine sources if a comparison is ever needed.

The upstream Arduino port this started from is poetaster's
[arduinoMI](https://github.com/poetaster/arduinoMI) (itself building on
Volker Boehm's mi-UGens SuperCollider work) — see the git history for that
era of the repo.
