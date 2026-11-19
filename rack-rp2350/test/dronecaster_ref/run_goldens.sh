#!/usr/bin/env bash
# Renders the drone goldens twice and proves byte-determinism. The drones draw
# no randomness of their own; the server seed is pinned by RandSeed regardless.
#
# Golden-engine law: scsynth is the ONLY valid golden engine — do not
# reintroduce an engine override. supernova's NRT n_set lands ONE BLOCK later
# than scsynth's (first sample divergence at frame 144,065 after a gate-off at
# frame 144,000), so a supernova golden would not match the C++ harness
# schedule (gate off at sample 144000 = block 2250); and SC 3.14.1
# Collection:includes compares Strings by IDENTITY (===), so the old
# includes-based per-name engine override was silently inert anyway.
#
# rc=139 acceptance law: scsynth 3.14.1-6 exits 139 (SIGSEGV at SHUTDOWN, not
# during rendering) for some graphs — observed for Mt. Lion, Fieldsteel,
# Twin Pks, Starlids, Sachiko, Coil — but the WAV it leaves behind is
# complete, correct and deterministic (192,064 frames pre-trim). Six rc=139
# render lines are therefore EXPECTED and accepted; any OTHER nonzero rc is a
# hard failure (gen_goldens.scd exits nonzero on it, and the trim asserts
# below catch a truncated or missing render).
#
# SuperCollider 3.14.1 system-wide (pacman; user-verified). The structural
# headless-sclang fixes live in gen_goldens.scd (called-Function body,
# vars-first, thisThread.randSeed_, d_recv completion message) — those are
# SC behaviors, not install quirks. The old /tmp/sc_root wiring (LD_LIBRARY_PATH,
# scconf.yaml includePaths, DC_SCSYNTH_FLAGS plugin dir) is gone with the
# system install.
set -euo pipefail
cd "$(dirname "$0")"

# All golden drone names: the T-D1 trio + the 12 curated T-D2 drones.
# Quoted so "harms way" (space) stays one shell element.
DC_DRONES=("Sine" "Square" "Triangle" "harms way" "Supersaw" "Thx" "Mt. Zion" "Mt. Lion" "Hecker" "Fieldsteel" "Apparatus" "Twin Pks" "Starlids" "Sachiko" "Coil")

sclang gen_goldens.scd </dev/null
for f in "${DC_DRONES[@]}"; do
  test -s "goldens/$f.expected.wav" || { echo "missing $f"; exit 1; }
done

# scsynth -N renders one block past the score-duration marker (3001 blocks
# = 192064 frames for a 4.0 s score); the C++ harness renders exactly
# 192000 frames and the comparator gates on equal size. Trim the data chunk
# to 192000 frames (the tail is post-release near-silence). Runs after BOTH
# renders — the determinism compare reads trimmed files on both sides.
trim_goldens() {
python3 - "${DC_DRONES[@]}" <<'PY'
import sys, struct
TARGET = 192000 * 2 * 4
for n in sys.argv[1:]:
    p = f"goldens/{n}.expected.wav"
    d = bytearray(open(p, 'rb').read())
    i = 12
    fmt = None
    while i < len(d) - 8:
        cid = bytes(d[i:i+4]); sz = int.from_bytes(d[i+4:i+8], 'little')
        if cid == b'fmt ':
            fmt = (int.from_bytes(d[i+8:i+10], 'little'),   # format (3 = IEEE float32)
                   int.from_bytes(d[i+10:i+12], 'little'),  # channels
                   int.from_bytes(d[i+12:i+16], 'little'),  # samplerate
                   int.from_bytes(d[i+22:i+24], 'little'))  # bits per sample
        if cid == b'data':
            assert sz > TARGET, f"{n}: {sz} frames smaller than trim target"
            del d[i + 8 + TARGET:]
            d[i+4:i+8] = TARGET.to_bytes(4, 'little')
            d[4:8] = (len(d) - 8).to_bytes(4, 'little')
            # Hard post-trim asserts (Task 1 law): every trimmed golden must be
            # fmt=3 / 2 ch / 48 kHz / 32-bit float, EXACTLY 192000 frames, and
            # non-silent. An AssertionError exits python nonzero, which aborts
            # this script via set -euo pipefail.
            assert fmt == (3, 2, 48000, 32), f"{n}: fmt {fmt} != (3, 2, 48000, 32)"
            frames = int.from_bytes(d[i+4:i+8], 'little') // (2 * 4)
            assert frames == 192000, f"{n}: {frames} frames after trim != 192000"
            vals = struct.unpack(f'<{TARGET // 4}f', bytes(d[i+8:i+8+TARGET]))
            peak = max(abs(v) for v in vals)
            assert peak > 0, f"{n}: silent golden (peak {peak})"
            print(f"{n}: fmt=(3,2,48000,32) frames={frames} peak={peak:.6f}")
            break
        i += 8 + sz + (sz & 1)
    open(p, 'wb').write(d)
print("trimmed to 192000 frames")
PY
}
trim_goldens

mkdir -p /tmp/dc_det1
cp goldens/*.expected.wav /tmp/dc_det1/
sclang gen_goldens.scd </dev/null
trim_goldens

# scsynth writes a PEAK chunk with a wall-clock timestamp into the WAV
# header — the sample data is what must be deterministic. Compare from the
# data chunk payload to EOF (skip the header entirely).
data_off() { python3 - "$1" <<'PY'
import sys
d = open(sys.argv[1], 'rb').read()
print(d.find(b'data') + 8)
PY
}
for f in "${DC_DRONES[@]}"; do
  off1=$(data_off "/tmp/dc_det1/$f.expected.wav")
  off2=$(data_off "goldens/$f.expected.wav")
  if [ "$off1" = "$off2" ] && cmp -s <(tail -c +"$((off1+1))" "/tmp/dc_det1/$f.expected.wav") \
                                <(tail -c +"$((off2+1))" "goldens/$f.expected.wav"); then
    echo "determinism: $f OK"
  else
    echo "determinism: $f DIFFERS"; exit 1
  fi
done
rm -rf /tmp/dc_det1
