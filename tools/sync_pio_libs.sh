#!/usr/bin/env bash
# Sync the STMLIB / ELEMENTS / CLOUDS submodule sources into the PlatformIO
# projects' lib/ directories, stripping the duplicated .cc translation units.
#
# Why: the submodules expose every .cc file as an .inc duplicate, compiled via
# a single wrapper (STMLIB/src/stmlib_all.cpp, ELEMENTS/src/elements_all.cpp,
# CLOUDS/src/clouds_all.cpp). If both the .cc files and the wrappers were
# compiled, every symbol would be duplicated. PlatformIO cannot pull in sources
# outside a project's src/ via build_src_filter (patterns outside the project's
# src/ are silently ignored), so the wrapper files are compiled as project
# libraries from lib/ instead — with the .cc files removed, keeping .inc
# duplicates, headers and wrappers.
#
# Re-run this script after updating the STMLIB/ ELEMENTS/ CLOUDS/ submodules:
#   ./tools/sync_pio_libs.sh
#
# This port's deltas on the generated libs are maintained as unified diffs in
# tools/patches/<lib>/*.patch and re-applied after every copy (strict
# --fuzz=0; a conflict aborts loudly — regenerate the patch, never hand-edit
# lib/).
#
# The submodules are never modified.
set -euo pipefail

cd "$(dirname "$0")/.."
REPO_ROOT="$(pwd -P)"

sync_project() {
  local proj=$1; shift
  for pair in "$@"; do
    # Pair spec: "SRC:dst[:mode]" — mode "inc" (default) = .cc files are
    # duplicated as .inc, so the .cc copies are deleted; mode "cc" = the
    # wrapper includes the .cc files directly, so they are KEPT.
    src=$(echo "$pair" | cut -d: -f1)
    dst=$(echo "$pair" | cut -d: -f2)
    mode=$(echo "$pair" | cut -d: -f3)
    mode=${mode:-inc}
    if [ ! -d "$REPO_ROOT/$src/src" ]; then
      echo "error: $REPO_ROOT/$src/src not found — is the submodule checked out?" >&2
      exit 1
    fi
    rm -rf "$REPO_ROOT/$proj/lib/$dst"
    mkdir -p "$REPO_ROOT/$proj/lib/$dst"
    cp -R "$REPO_ROOT/$src/src/." "$REPO_ROOT/$proj/lib/$dst/"
    if [ "$mode" = "inc" ]; then
      find "$REPO_ROOT/$proj/lib/$dst" -name '*.cc' -type f -delete
    else
      # mode "cc": PlatformIO compiles the .cc files directly, so the
      # single-TU wrapper (which #includes the same .cc files) must go or
      # every symbol would be duplicated.
      rm -f "$REPO_ROOT/$proj/lib/$dst/${dst}_all.cpp"
    fi

    # Re-apply this port's patches on top of the fresh upstream copy. Strict:
    # any conflict aborts loudly.
    for p in "$REPO_ROOT/tools/patches/$dst"/*.patch; do
      if [ -e "$p" ]; then
        patch --forward --fuzz=0 -p1 -d "$REPO_ROOT/$proj/lib/$dst" < "$p" \
          || { echo "error: $p no longer applies to $dst — regenerate it" >&2; exit 1; }
        echo "applied $(basename "$p") to $proj/lib/$dst"
      fi
    done

    # The earlephilhower core appends -Os after the project's build flags
    # (framework-arduinopico/tools/platformio-build.py), silently defeating any
    # project-level -O3 by last-wins. Library build flags are appended AFTER the
    # core's -Os, so the DSP libraries must request -O3 via library.json.
    # Clouds-only: settings.cc is the original firmware's STM32-flash preset
    # persistence (page_storage.h uses FLASH_ErasePage et al.) — unportable as
    # is, unused by the processor: excluded from the build.
    local srcfilter=""
    if [ "$mode" != "inc" ]; then
      srcfilter=$'\n    "srcFilter": ["+<*>", "-<clouds/settings.cc>"],'
    fi
    cat > "$REPO_ROOT/$proj/lib/$dst/library.json" <<EOF
{
  "name": "$dst",
  "version": "0.0.0",
  "build": {${srcfilter}
    "flags": ["-O3"]
  }
}
EOF

    incs=$(find "$REPO_ROOT/$proj/lib/$dst" -name '*.inc' -type f | wc -l)
    ccs=$(find "$REPO_ROOT/$proj/lib/$dst" -name '*.cc' -type f | wc -l)
    echo "synced $src/src -> $proj/lib/$dst ($incs .inc kept, $ccs .cc remaining)"
    if [ "$mode" = "inc" ] && [ "$ccs" -ne 0 ]; then
      echo "error: .cc files remain in $proj/lib/$dst" >&2
      exit 1
    fi
  done
}

sync_project elements-rp2350 "STMLIB:stmlib" "ELEMENTS:elements"
sync_project clouds-rp2350  "STMLIB:stmlib" "CLOUDS:clouds:cc"

echo "done. Submodules untouched."
