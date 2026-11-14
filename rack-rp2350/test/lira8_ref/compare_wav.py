#!/usr/bin/env python
"""Compare a rendered WAV against the Pd golden reference.

Gates (per docs/superpowers/plans/2026-11-15-rack-lira8.md):
  - identical length & sample rate
  - RMS(difference) relative to reference RMS <= --rms-db (default -50 dB)
  - Pearson correlation >= --corr (default 0.99)

Usage: compare_wav.py <ref.wav> <got.wav> [--rms-db N] [--corr F]
Exit 0 = all gates pass, 1 = fail (prints metrics either way).
"""
import argparse
import sys
import wave

import numpy as np


def read_wav(path):
    with wave.open(path, "rb") as w:
        sr, ch, sw, n = w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()
        raw = w.readframes(n)
    if sw == 2:
        x = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32768.0
    elif sw == 4:
        x = np.frombuffer(raw, dtype=np.int32).astype(np.float64) / 2147483648.0
    elif sw == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        x = (b[:, 0].astype(np.int32) | (b[:, 1].astype(np.int32) << 8)
             | (np.where(b[:, 2] & 0x80, b[:, 2].astype(np.int32) - 256, b[:, 2].astype(np.int32)) << 16)) / 2147483648.0
    else:
        sys.exit(f"unsupported sample width {sw * 8} bit in {path}")
    return x.reshape(-1, ch), sr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("got")
    ap.add_argument("--rms-db", type=float, default=-50.0)
    ap.add_argument("--corr", type=float, default=0.99)
    args = ap.parse_args()

    a, sr_a = read_wav(args.ref)
    b, sr_b = read_wav(args.got)
    if sr_a != sr_b or len(a) != len(b):
        print(f"FAIL: length/rate mismatch ref={len(a)}@{sr_a} got={len(b)}@{sr_b}")
        return 1
    if a.shape[1] != b.shape[1]:
        print(f"FAIL: channel mismatch ref={a.shape[1]} got={b.shape[1]}")
        return 1

    ok = True
    per_ch = []
    for c in range(a.shape[1]):
        x, y = a[:, c], b[:, c]
        rms_x = np.sqrt(np.mean(x * x)) or 1e-30
        rms_d = np.sqrt(np.mean((x - y) ** 2))
        db = 20 * np.log10(rms_d / rms_x) if rms_d > 0 else -999.0
        corr = float(np.corrcoef(x, y)[0, 1]) if rms_x > 1e-12 else 0.0
        peak = float(np.max(np.abs(x - y)))
        per_ch.append((c, db, corr, peak, rms_x))
        if db > args.rms_db or corr < args.corr:
            ok = False

    print(f"{'ch':>3} {'rmsdiff dB':>11} {'corr':>8} {'peak':>9} {'rms':>9}")
    for c, db, corr, peak, rms_x in per_ch:
        print(f"{c:>3} {db:>11.2f} {corr:>8.4f} {peak:>9.5f} {rms_x:>9.5f}")
    print(("PASS" if ok else "FAIL")
          + f" (gates: rmsdiff <= {args.rms_db} dB, corr >= {args.corr})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
