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
    # Pair spec: "SRC:dst[:mode[:nest]]" — mode "inc" (default) = .cc files are
    # duplicated as .inc, so the .cc copies are deleted; mode "cc" = the
    # wrapper includes the .cc files directly, so they are KEPT. The source is
    # "$src/src" (poetaster Arduino layout) when present, else the directory
    # itself (pichenettes eurorack layout, e.g. EURORACK-STEREO/plaits). An
    # optional "nest" copies into lib/$dst/$nest so module-prefix includes
    # ("plaits/dsp/…") resolve against the lib root.
    src=$(echo "$pair" | cut -d: -f1)
    dst=$(echo "$pair" | cut -d: -f2)
    mode=$(echo "$pair" | cut -d: -f3)
    nest=$(echo "$pair" | cut -d: -f4)
    mode=${mode:-inc}
    if [ -d "$REPO_ROOT/$src/src" ]; then
      srcdata="$REPO_ROOT/$src/src"
    elif [ -d "$REPO_ROOT/$src" ]; then
      srcdata="$REPO_ROOT/$src"
    else
      echo "error: $REPO_ROOT/$src not found — is the submodule checked out?" >&2
      exit 1
    fi
    rm -rf "$REPO_ROOT/$proj/lib/$dst"
    mkdir -p "$REPO_ROOT/$proj/lib/$dst"
    if [ -n "$nest" ]; then
      mkdir -p "$REPO_ROOT/$proj/lib/$dst/$nest"
      cp -R "$srcdata/." "$REPO_ROOT/$proj/lib/$dst/$nest/"
    else
      cp -R "$srcdata/." "$REPO_ROOT/$proj/lib/$dst/"
    fi

    # Plaits-Lab fork: prune everything a Voice-only port never compiles
    # (firmware/UI/test/drivers/bootloader/tools/alt-firmware diagnostic
    # headers). Keeps lib/ reviewable and the PlatformIO TU set predictable;
    # build_config.h, section_marks.h, resources.* and user_data.* stay
    # (user_data.h is neutralized by a port patch).
    if [ "$dst" = "plaits" ]; then
      pr="$REPO_ROOT/$proj/lib/$dst/${nest:-}"
      rm -rf "$pr/bootloader" "$pr/drivers" "$pr/test" "$pr/tools" \
             "$pr/hardware_design" "$pr/resources"
      rm -f "$pr/plaits.cc" "$pr/ui.cc" "$pr/ui.h" "$pr/ui_poll.cc" \
            "$pr/settings.cc" "$pr/settings.h" "$pr/user_data_receiver.cc" \
            "$pr/user_data_receiver.h" "$pr/makefile" "$pr/makeflash.sh"
      rm -f "$pr"/*.md
      for h in "$pr"/*.h; do
        case "$(basename "$h")" in
          build_config.h|section_marks.h|resources.h|user_data.h|user_data_region.h) ;;
          *) rm -f "$h" ;;
        esac
      done
    fi
    # Rings (MiniElements) fork: keep dsp/ + resources.{h,cc} only — the root
    # firmware files (rings.cc/ui/settings/cv_scaler/meter) are STM32-only,
    # samples/ are the wav sources for resources.cc, resources/ holds the .py
    # generators.
    if [ "$dst" = "rings" ]; then
      pr="$REPO_ROOT/$proj/lib/$dst/${nest:-}"
      rm -rf "$pr/bootloader" "$pr/drivers" "$pr/test" "$pr/hardware_design" \
             "$pr/resources" "$pr/samples"
      rm -f "$pr/rings.cc" "$pr/ui.cc" "$pr/ui.h" "$pr/settings.cc" \
            "$pr/settings.h" "$pr/cv_scaler.cc" "$pr/cv_scaler.h" \
            "$pr/meter.h" "$pr/makefile" "$pr/makeflash.sh" "$pr/__init__.py"
      rm -f "$pr"/*.md
    fi
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
    if [ "$dst" = "clouds" ]; then
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
    echo "synced ${srcdata#"$REPO_ROOT"/} -> $proj/lib/$dst ($incs .inc kept, $ccs .cc remaining)"
    if [ "$mode" = "inc" ] && [ "$ccs" -ne 0 ]; then
      echo "error: .cc files remain in $proj/lib/$dst" >&2
      exit 1
    fi
  done
}

sync_project elements-rp2350 "STMLIB:stmlib" "ELEMENTS:elements"
sync_project clouds-rp2350  "STMLIB:stmlib" "CLOUDS:clouds:cc"
sync_project plaits-rp2350  "STMLIB:stmlib" "EURORACK-STEREO/plaits:plaits:cc:plaits"
sync_project rings-rp2350   "STMLIB:stmlib" "EURORACK-STEREO/rings:rings:cc:rings"

# Rack project: same four module libs as the standalone ports (the same patch
# chains apply; the module ram_code.h headers switch their section prefix to
# .tc_<module>. only when the RACK build defines RACK_RP2350_TC).
sync_project rack-rp2350    "STMLIB:stmlib" "ELEMENTS:elements" "CLOUDS:clouds:cc" \
             "EURORACK-STEREO/plaits:plaits:cc:plaits" \
             "EURORACK-STEREO/rings:rings:cc:rings"

# Vendored (NOT submodule-derived) library copies for the rack project:
# mirrored verbatim from the standalone projects, then re-patched from
# tools/patches/<name>/ (rack-only deltas). The library.json of a vendored dir
# ships with the copy (pio_usb needs none; cmsis_dsp keeps -DARM_MATH_CM33) —
# unlike sync_project, vendor_copy never rewrites it.
vendor_copy() {
  local proj=$1 name=$2 src=$3
  local dst="$REPO_ROOT/$proj/lib/$name"
  if [ ! -d "$REPO_ROOT/$src" ]; then
    echo "error: vendored source $REPO_ROOT/$src not found" >&2
    exit 1
  fi
  rm -rf "$dst"
  mkdir -p "$dst"
  cp -R "$REPO_ROOT/$src/." "$dst/"
  for p in "$REPO_ROOT/tools/patches/$name"/*.patch; do
    if [ -e "$p" ]; then
      patch --forward --fuzz=0 -p1 -d "$dst" < "$p" \
        || { echo "error: $p no longer applies to $name — regenerate it" >&2; exit 1; }
      echo "applied $(basename "$p") to $proj/lib/$name"
    fi
  done
  echo "vendored $src -> $proj/lib/$name"
}

vendor_copy rack-rp2350 pio_usb   elements-rp2350/lib/pio_usb
vendor_copy rack-rp2350 cmsis_dsp clouds-rp2350/lib/cmsis_dsp

echo "done. Submodules untouched."
