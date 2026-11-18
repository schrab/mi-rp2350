# dronecaster_ref — golden renders + drone corpus

The 37 drone functions in `drones/` and `Dronecaster_SynthSocket.sc` are verbatim
copies from https://github.com/northern-information/dronecaster (GPL-3.0),
pinned at commit `b33edd1793e060e0bc156502cae3aab623cb06e7` (main, fetched 2026-11-19). They are
vendored so the golden pipeline and the catalog-order check never depend on an
external checkout. The C++ port of each drone lives in `src/dronecaster/drones/`
and is a re-implementation, not a copy of this directory.

`gen_goldens.scd` wraps a drone exactly like `Dronecaster_SynthSocket.sc:
wrapDef` — `fn.value(K2A.ar(hz), K2A.ar(amp), 0)`, `EnvGen.kr(Env.asr(attack,
1, release), gate, doneAction: 2)`, `Out.ar(out, snd * aenv)` — and renders it
with NRT scsynth: 48 kHz, float32 WAV, 4.0 s (voice at t = 0, gate off at
t = 3.0 s, attack = release = 1.0 s, hz = 55, amp = 0.4). Seeds: the sclang
seed fixes graph construction; `RandSeed.kr(1, 16rC0FFEE)` inside the wrapper
fixes the **server** RGen (scsynth's world RGen is otherwise time-seeded).

Renderer: SuperCollider **3.14.1** (the port's SC 3.13.1 anchors were re-verified
against the 3.14.1 sources; behavioral deltas are folded into `src/dronecaster/`
— LFTri/EnvGen ctor-priming rollback, SinOsc float cpstoinc, Pulse double scale).

The catalog order is the byte-wise sort of the filenames (37 entries). Verified
positions: Shields=21, Sine=22, Square=23, Thx=27, Toshiya=28, Triangle=29,
harms way=36 (0-based). The upstream Lua sorts names with `table.sort`
(byte-wise), so this is also the norns order.
