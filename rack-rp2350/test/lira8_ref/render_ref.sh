#!/bin/bash
# Render golden reference WAVs: generate a per-scenario wrapper patch
# (original + inline #A-set schedule + text sequence + writesf tap), run pd
# headless at 32 kHz. Vanilla pd only (no externals).
# Usage: render_ref.sh [scenario ...]   (default: all five)
set -u
cd "$(dirname "$0")"
PD=pd
HERE=$(pwd)
SCENS="${*:-drone-all single-pair delay-extremes fuzz-push transients}"

gen_patch() {
  local scen=$1
  .venv/bin/python gen_golden_patch.py "$scen"
}

for scen in $SCENS; do
  echo "=== $scen ==="
  rm -f "goldens/$scen.wav"
  gen_patch "$scen" || { echo "gen failed"; continue; }
  # the generated patch sets $0-literal receivers; the ROOT canvas $0 value
  # must be 1001 (spec) — verified by non-silent output.
  timeout 90 "$PD" -nogui -r 32000 -nodac -noadc \
      "patch/_LIRA-8-golden.pd" > "out/golden_$scen.log" 2>&1
  echo "pd exit=$?"
  grep -E "couldn't|connection failed|error" "out/golden_$scen.log" | head -3
  ls -la "goldens/$scen.wav" 2>/dev/null || echo "MISSING goldens/$scen.wav"
done
