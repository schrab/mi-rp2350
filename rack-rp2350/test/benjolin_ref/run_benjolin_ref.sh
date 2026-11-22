#!/usr/bin/env bash
# Renders the full-\benjolis-SynthDef goldens and trims them to exactly
# 48000 frames (scsynth -N renders one block past the duration marker).
# Deterministic: the graph has zero random UGens (see gen_benjolin_ref.scd).
set -euo pipefail
cd "$(dirname "$0")"

sclang gen_benjolin_ref.scd </dev/null
python3 - <<'PY'
import glob
TARGET = 48000 * 2 * 4
for p in sorted(glob.glob("goldens/*.expected.wav")):
    d = bytearray(open(p, 'rb').read())
    i = 12
    while i < len(d) - 8:
        cid = bytes(d[i:i+4]); sz = int.from_bytes(d[i+4:i+8], 'little')
        if cid == b'data':
            assert sz >= TARGET, f"{p}: {sz} bytes smaller than trim target"
            if sz > TARGET:
                del d[i + 8 + TARGET:]
                d[i+4:i+8] = TARGET.to_bytes(4, 'little')
                d[4:8] = (len(d) - 8).to_bytes(4, 'little')
                open(p, 'wb').write(d)
            break
        i += 8 + sz + (sz & 1)
print("benjolin_ref: trimmed to 48000 frames")
PY
