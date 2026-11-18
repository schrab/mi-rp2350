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
    # The float IR table is NOT emitted anymore: its only firmware consumer
    # was the bypass-energy sum (speaker_bypass_gain) — that ships as the
    # precomputed RMS constant below, saving the whole table from the
    # tc_elements SRAM slot (the 1024-tap q15 set did not fit alongside it).
    print(f"// {path} -> {taps} taps @ {fs_out} Hz (peak-normalized, "
          f"pre-delay compacted from sample {start}, {fade}-sample tail fade; "
          f"source {fs_in} Hz, {len(x)} samples, {len(x)/fs_in:.2f} s)")
    print("static const float kSpeakerIrRms = %.9gf;  // sqrt(sum y^2) over "
          "the emitted taps (bypass loudness matching)"
          % float(np.sqrt(np.sum(y * y))))
    # q15 packed pairs for the SMLALD dual-MAC conv (rack_dist.h): the IR
    # scaled so the PEAK coefficient hits full 15-bit scale (no sum|ir|
    # constraint — the 64-bit SMLALD accumulator cannot overflow:
    # sum|ir16| x 32767 ~= 1.2e10 << 2^63). Full-scale coefficients cut the
    # coefficient-quantization noise ~11x vs the int32-sum-safe scaling
    # (which squeezed them to ~12 bits = audible grit on sustained tones).
    # Chronological pairs packed (lo = ir16[2j], hi = ir16[2j+1]) as uint32.
    abssum = float(np.sum(np.abs(y)))
    peak = float(np.max(np.abs(y))) if len(y) else 0.0
    scale = 32767.0 / peak if peak > 0 else 0.0
    q15 = np.clip(np.round(y * scale), -32768, 32767).astype(np.int64)
    qsum = int(np.sum(np.abs(q15)))
    print("// q15 packed pairs: scale=%.6f (sum|h|=%.4f, sum|ir16|=%d, int64 acc);"
          % (scale, abssum, qsum))
    print("// y_float = acc * kSpeakerQ15Norm, kSpeakerQ15Norm = %.9g"
          % (1.0 / (scale * 32767.0)))
    nw = taps // 2
    print("static const uint32_t kSpeakerIrQ15[%d] = {" % nw)
    for i in range(0, nw, 6):
        parts = []
        for j in range(i, min(i + 6, nw)):
            lo = int(q15[2 * j]) & 0xFFFF
            hi = int(q15[2 * j + 1]) & 0xFFFF
            parts.append("0x%08xu" % (lo | (hi << 16)))
        print("    " + ", ".join(parts) + ("," if i + 6 < nw else ""))
    print("};" )


if __name__ == "__main__":
    main()
