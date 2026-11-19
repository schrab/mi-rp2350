#!/usr/bin/env bash
# Renders the per-UGen reference WAVs (T-D2) and trims them to exactly
# 48000 frames (scsynth -N renders one block past the duration marker).
# Determinism is proven by the T-D1 drone goldens' mechanism; these are
# regenerated only when a UGen's canonical graph changes.
# SuperCollider 3.14.1 + sc3-plugins 3.14.1-1 system-wide (pacman;
# user-verified). The old /tmp wiring (DC_SCSYNTH_FLAGS=-U /tmp/scplug,
# sclang -l /tmp/sc_root/sc3p_conf.yaml) is gone with the system install —
# scsynth finds the sc3p .so files on its default plugin path and sclang's
# default class lib resolves SawDPW/PulseDPW/SVF/DFM1/... .
set -euo pipefail
cd "$(dirname "$0")"

sclang gen_ugen_ref.scd </dev/null
python3 - <<'PY'
import glob
TARGET = 48000 * 2 * 4
for p in sorted(glob.glob("ugen_ref/*.expected.wav")):
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
print("ugen_ref: trimmed to 48000 frames")
PY
ls ugen_ref/*.expected.wav | wc -l
