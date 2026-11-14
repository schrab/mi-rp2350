#!/usr/bin/env python
"""Analyze the Pd primitive probe logs (out/*.log) against candidate
implementations, to pin the exact laws for the C++ port."""
import math
import re
import sys

import numpy as np

SR = 32000  # requested rate; calibrated against env staircase below


def read_tag(path, tag):
    vals = []
    with open(path) as f:
        for ln in f:
            m = re.match(rf"{tag}: (-?[\d.e+-]+)", ln)
            if m:
                vals.append(float(m.group(1)))
    return np.array(vals)


def sim_hip(n_samples, freq, sr, compat_new=True):
    coef = 1.0 - freq * (2 * 3.14159) / sr
    coef = min(max(coef, 0.0), 1.0)
    normal = 0.5 * (1 + coef) if compat_new else 1.0
    out = np.zeros(n_samples)
    last = 0.0
    x = np.ones(n_samples)
    for i in range(n_samples):
        new = x[i] + coef * last
        out[i] = normal * (new - last)
        last = new
    return out


def analyze_filters():
    hip = read_tag("out/filters.log", "hip")
    lop = read_tag("out/lop" and "out/filters.log", "lop")
    step_ms = 600
    idx = [i for i, v in enumerate(hip) if v > 0.05]
    print(f"hip: first>0.05 at ms {idx[0] if idx else '?'}; "
          f"measured sample-at-first-tick {hip[idx[0]] if idx else 0:.6f}")
    if len(idx) > 3:
        ratios = hip[idx[0] + 2 : idx[0] + 12] / hip[idx[0] + 1 : idx[0] + 11]
        print(f"hip per-ms decay ratio: {np.mean(ratios):.5f}")
        r32 = float(np.mean(ratios))
        r_samp = r32 ** (1 / 32)
        print(f"  -> per-sample pole r = {r_samp:.6f}  "
              f"(1-2pif/sr)^2 = {(1 - 20 * 2 * math.pi / SR) ** 2:.6f}, "
              f"(1-2pif/sr) = {1 - 20 * 2 * math.pi / SR:.6f}")
        # simulate the verbatim perform loop and sample every 32 samples
        n = SR  # 1 s after step
        sim = sim_hip(n, 20.0, SR)
        # metro catches ~ sample 32*k +- jitter; compare first values
        print(f"  sim[32]={sim[32]:.6f} sim[64]={sim[64]:.6f} "
              f"sim[96]={sim[96]:.6f} (measured {hip[idx[0]]:.6f}, "
              f"{hip[idx[0]+1]:.6f}, ...)")
        sim_old = sim_hip(n, 20.0, SR, compat_new=False)
        print(f"  legacy(no normal): sim[32]={sim_old[32]:.6f}")
    lop_after = [i for i, v in enumerate(lop) if v > 0.9]
    print(f"lop: first>0.9 at ms {lop_after[0] if lop_after else '?'} "
          f"(coef 2pif/sr = {min(4000 * 2 * math.pi / SR, 1):.4f} -> "
          f"instant to 1 within one ms tick)")


def analyze_env():
    env = read_tag("out/env.log", "env")
    step = 600
    post = env[step:step + 200]
    changes = [i for i in range(1, len(post)) if post[i] != post[i - 1]]
    d = np.diff(changes)
    print(f"env post-step (dB): first non-prev changes at ms offsets "
          f"{changes[:12]} (cadence {np.median(d) if len(d) else '?'} ms)")
    print(f"env values from step: {post[:10]}")
    print(f"env final values: {env[-3:]}")
    # cadence in samples = hop; rate calibration: hop=256 samples
    # if cadence ~= 8 ms -> sr 32000; 5.33 ms -> 48000


def analyze_snr():
    direct = read_tag("out/snr.log", "direct")
    via = read_tag("out/snr.log", "via")
    d = [i for i, v in enumerate(direct) if v > 0.5]
    v = [i for i, v in enumerate(via) if v > 0.5]
    if d and v:
        print(f"snr: direct step ms {d[0]}, via ms {v[0]} -> "
              f"offset {(v[0] - d[0])} ms (64 samples @32k = 2.0 ms)")


def analyze_delzero():
    direct = read_tag("out/delzero.log", "direct")
    vd = read_tag("out/delzero.log", "vd")
    d = [i for i, v in enumerate(direct) if v > 0.5]
    v = [i for i, v in enumerate(vd) if v > 0.5]
    if d and v:
        print(f"delzero: direct ms {d[0]}, vd ms {v[0]} -> "
              f"offset {(v[0] - d[0])} ms (zerodel = 1 block?)")


if __name__ == "__main__":
    analyze_filters()
    analyze_env()
    analyze_snr()
    analyze_delzero()
