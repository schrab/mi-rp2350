#!/usr/bin/env bash
# Sync the STMLIB and ELEMENTS submodule sources into the PlatformIO project's
# lib/ directory, stripping the duplicated .cc translation units.
#
# Why: the submodules expose every .cc file as an .inc duplicate, compiled via a
# single wrapper (STMLIB/src/stmlib_all.cpp, ELEMENTS/src/elements_all.cpp).
# If both the .cc files and the wrappers were compiled, every symbol would be
# duplicated. PlatformIO cannot pull in ../STMLIB/src/stmlib_all.cpp via
# build_src_filter (patterns outside the project's src/ are silently ignored),
# so the wrapper files are compiled as project libraries from lib/ instead —
# with the .cc files removed, keeping .inc duplicates, headers and wrappers.
#
# Re-run this script after updating the STMLIB/ or ELEMENTS/ submodules:
#   ./tools/sync_pio_libs.sh
#
# The submodules are never modified.
set -euo pipefail

cd "$(dirname "$0")/.."
REPO_ROOT="$(pwd -P)"

for pair in "STMLIB:stmlib" "ELEMENTS:elements"; do
  src="${pair%%:*}"
  dst="${pair##*:}"
  if [ ! -d "$REPO_ROOT/$src/src" ]; then
    echo "error: $REPO_ROOT/$src/src not found — is the submodule checked out?" >&2
    exit 1
  fi
  rm -rf "elements-rp2350/lib/$dst"
  mkdir -p "elements-rp2350/lib/$dst"
  cp -R "$REPO_ROOT/$src/src/." "elements-rp2350/lib/$dst/"
  find "elements-rp2350/lib/$dst" -name '*.cc' -type f -delete

  # The earlephilhower core appends -Os after the project's build flags
  # (framework-arduinopico/tools/platformio-build.py), silently defeating any
  # project-level -O3 by last-wins. Library build flags are appended AFTER the
  # core's -Os, so the DSP libraries must request -O3 via library.json.
  cat > "elements-rp2350/lib/$dst/library.json" <<EOF
{
  "name": "$dst",
  "version": "0.0.0",
  "build": {
    "flags": ["-O3"]
  }
}
EOF

  incs=$(find "elements-rp2350/lib/$dst" -name '*.inc' -type f | wc -l)
  ccs=$(find "elements-rp2350/lib/$dst" -name '*.cc' -type f | wc -l)
  echo "synced $src/src -> elements-rp2350/lib/$dst ($incs .inc kept, $ccs .cc remaining)"
  if [ "$ccs" -ne 0 ]; then
    echo "error: .cc files remain in elements-rp2350/lib/$dst" >&2
    exit 1
  fi
done

echo "done. Submodules untouched."
