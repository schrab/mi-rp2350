#!/usr/bin/env bash
# Renders the trio goldens twice and proves byte-determinism. The trio draws
# no randomness; the server seed is pinned by RandSeed regardless.
#
# SuperCollider 3.14.1 system-wide (pacman; user-verified). The structural
# headless-sclang fixes live in gen_goldens.scd (called-Function body,
# vars-first, thisThread.randSeed_, d_recv completion message) — those are
# SC behaviors, not install quirks. The old /tmp/sc_root wiring (LD_LIBRARY_PATH,
# scconf.yaml includePaths, DC_SCSYNTH_FLAGS plugin dir) is gone with the
# system install.
set -euo pipefail
cd "$(dirname "$0")"

sclang gen_goldens.scd </dev/null
for f in Sine Square Triangle; do
  test -s "goldens/$f.expected.wav" || { echo "missing $f"; exit 1; }
done

# scsynth -N renders one block past the score-duration marker (3001 blocks
# = 192064 frames for a 4.0 s score); the C++ harness renders exactly
# 192000 frames and the comparator gates on equal size. Trim the data chunk
# to 192000 frames (the tail is post-release near-silence). Runs after BOTH
# renders — the determinism compare reads trimmed files on both sides.
trim_goldens() {
python3 - <<'PY'
TARGET = 192000 * 2 * 4
for n in ("Sine", "Square", "Triangle"):
    p = f"goldens/{n}.expected.wav"
    d = bytearray(open(p, 'rb').read())
    i = 12
    while i < len(d) - 8:
        cid = bytes(d[i:i+4]); sz = int.from_bytes(d[i+4:i+8], 'little')
        if cid == b'data':
            assert sz > TARGET, f"{n}: {sz} frames smaller than trim target"
            del d[i + 8 + TARGET:]
            d[i+4:i+8] = TARGET.to_bytes(4, 'little')
            d[4:8] = (len(d) - 8).to_bytes(4, 'little')
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
for f in Sine Square Triangle; do
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
