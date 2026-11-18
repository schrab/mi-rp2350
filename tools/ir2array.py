#!/usr/bin/env python3
"""Speaker IR -> C float array for the rack's cab convolution.

Reads a PCM WAV (any bit depth wave.exe handles incl. 24-bit), resamples to
the rack rate with an FFT-domain resample (band-limited), truncates to the
requested tap count, normalizes the peak to 1.0, fades the tail (raised
cosine) to kill the truncation step, and prints a C float array to stdout.
The caller pastes the output into a header (Write tool) — this script never
writes source files itself.

Usage: ir2array.py <in.wav> <taps> <fs_out>
"""
import math
import sys
import wave

import numpy as np


def read_wav_mono(path):
    with wave.open(path, "rb") as w:
        nch, sw, fs, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if nch != 1:
        raise SystemExit(f"expected mono, got {nch} channels")
    if sw == 3:  # 24-bit little-endian
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        vals = (b[:, 0].astype(np.int32)
                | (b[:, 1].astype(np.int32) << 8)
                | (b[:, 2].astype(np.int32) << 16))
        vals = np.where(vals & 0x800000, vals - 0x1000000, vals)
        x = vals.astype(np.float64) / 8388608.0
    elif sw == 2:
        x = np.frombuffer(raw, dtype="<i2").astype(np.float64) / 32768.0
    elif sw == 4:
        x = np.frombuffer(raw, dtype="<i4").astype(np.float64) / 2147483648.0
    else:
        raise SystemExit(f"unsupported sample width {sw}")
    return x, fs


def resample_fft(x, fs_in, fs_out):
    n_out = int(round(len(x) * fs_out / fs_in))
    spec = np.fft.rfft(x)
    keep = min(len(spec), n_out // 2 + 1)  # band-limited domain resample
    out_spec = np.zeros(n_out // 2 + 1, dtype=complex)
    out_spec[:keep] = spec[:keep] * (n_out / len(x))
    y = np.fft.irfft(out_spec, n_out)
    return y


def main():
    path, taps, fs_out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    x, fs_in = read_wav_mono(path)
    y = resample_fft(x, fs_in, fs_out)
    # Pre-delay compaction: distant mics put 10-30 ms of silence before the
    # transient — pure latency that would waste taps. Skip to the first
    # sample exceeding 3% of the peak, backed off 32 samples (1 ms).
    peak = np.max(np.abs(y))
    start = int(np.argmax(np.abs(y) > 0.03 * peak))
    start = max(0, start - 32)
    y = y[start:]
    if len(y) < taps:
        y = np.pad(y, (0, taps - len(y)))
    y = y[:taps].copy()
    peak = np.max(np.abs(y))
    if peak > 0:
        y /= peak
    fade = min(64, taps // 8)
    ramp = 0.5 * (1.0 + np.cos(np.linspace(0.0, math.pi, fade)))
    y[-fade:] *= ramp
    print(f"// {path} -> {taps} taps @ {fs_out} Hz (peak-normalized, "
          f"pre-delay compacted from sample {start}, {fade}-sample tail fade; "
          f"source {fs_in} Hz, {len(x)} samples, {len(x)/fs_in:.2f} s)")
    print("static const float kSpeakerIr[%d] = {" % taps)
    for i in range(0, taps, 8):
        parts = []
        for v in y[i:i + 8]:
            s = "%.9g" % v
            if "." not in s and "e" not in s and "n" not in s:
                s += ".0"  # keep the float literal well-formed (0f is not)
            parts.append(s + "f")
        row = ", ".join(parts)
        print("    " + row + ("," if i + 8 < taps else ""))
    print("};")


if __name__ == "__main__":
    main()
