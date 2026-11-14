#!/bin/bash
# Run all Pd primitive probes and capture stdout.
# RECIPE (cost an hour to find): pd -nogui only runs the DSP scheduler if the
# PATCH sends "; pd dsp 1" — -send never delivers it and -nodac -noadc alone
# leaves the chain silent. Each probe self-quits via "; pd quit".
set -u
cd "$(dirname "$0")"
mkdir -p out
PD=pd
run() {
  local name=$1 patch=$2
  echo "=== $name ==="
  timeout 30 "$PD" -nogui -r 32000 -nodac -noadc "$patch" > "out/$name.log" 2>&1
  echo "exit=$? lines=$(wc -l < "out/$name.log")"
}
run env      probe_env.pd
run filters  probe_filters.pd
run snr      probe_snr.pd
run delzero  probe_delzero.pd
run noise    probe_noise.pd
