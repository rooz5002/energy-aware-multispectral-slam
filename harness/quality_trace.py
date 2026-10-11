#!/usr/bin/env python3
"""Diagnostic for milestone M3: quality scores and weights on a recorded sequence, without SLAM.

    python -m harness.quality_trace data/sequences/<sequence> <profile> [--every N]

N_m uses DETECTED ORB keypoints (OpenCV ORB) over the per-modality budget, because no tracker is
running here; the column n_source records that. In SLAM runs N_m comes from tracked inliers.
Output: results/quality_trace/<sequence>_<profile>.csv and .png (diagnostic, not a paper figure).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

from controller import quality
from controller.thermal import ThermalPreprocessor
from controller.weights import weights
from degradation.profiles import apply
from harness.sequence import REPO, Sequence, load_profile, load_yaml, quality_params

BOTH = {"rgb": True, "thermal": True}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sequence", type=Path)
    ap.add_argument("profile")
    ap.add_argument("--every", type=int, default=3)
    ap.add_argument("--budget", type=int, default=600, help="ORB features per modality")
    args = ap.parse_args()
    seq = Sequence(args.sequence)
    params, calib = load_yaml("default.yaml"), load_yaml("calibration.yaml")
    qp = quality_params(params, calib)
    profile = load_profile(args.profile, seq)
    pre = ThermalPreprocessor(*calib["thermal_window_K"], kelvin_per_count=seq.camera["thermal_resolution_K_per_count"],
                              **calib["thermal_preprocessing"])
    orb = cv2.ORB_create(nfeatures=args.budget)
    rows = []
    for i in range(0, len(seq), args.every):
        t = seq.stamps[i]
        bgr, severity = apply(profile, seq.rgb(i), t, i)
        img = {"rgb": cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), "thermal": pre(seq.thermal_raw(i))}
        q = {m: quality.assess(img[m], len(orb.detect(img[m], None)), args.budget, m, qp) for m in img}
        w = weights({m: q[m].Q for m in q}, BOTH)
        row = dict(t=t, frame=i, severity=severity, n_source="detected")
        for m in q:
            row.update({f"N_{m}": q[m].N, f"H_{m}": q[m].H, f"B_{m}": q[m].B, f"Q_{m}": q[m].Q, f"omega_{m}": w[m]})
        rows.append(row)
    df = pd.DataFrame(rows)
    out = REPO / "results" / "quality_trace"
    out.mkdir(parents=True, exist_ok=True)
    stem = out / f"{seq.name}_{args.profile}"
    df.to_csv(f"{stem}.csv", index=False)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(7.16, 6), sharex=True)
    for m, c in (("rgb", "tab:blue"), ("thermal", "tab:red")):
        ax[0].plot(df.t, df[f"Q_{m}"], c, label=f"Q {m}")
        ax[1].plot(df.t, df[f"omega_{m}"], c, label=f"omega {m}")
        for k, ls in (("N", "-"), ("H", "--"), ("B", ":")):
            ax[2].plot(df.t, df[f"{k}_{m}"], c, ls=ls, label=f"{k} {m}")
    ax[0].axhline(params["scheduler"]["tau_Q"], color="k", lw=0.8, ls="--", label="tau_Q")
    for a, lab in zip(ax, ("Quality Q", "Weight omega", "Indicators")):
        if profile.kind != "none":
            a.axvspan(profile.onset_s, profile.offset_s, color="0.85", zorder=0)
        a.set_ylabel(lab)
        a.legend(ncol=4, fontsize=7, loc="lower right")
    ax[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(f"{stem}.png", dpi=150)

    win = df[(df.severity >= 0.99 * profile.peak)] if profile.kind != "none" else df.iloc[0:0]
    clear = df[df.severity == 0]
    print(f"{seq.name} / {args.profile}: {len(df)} frames; window {profile.onset_s:.1f}-{profile.offset_s:.1f} s")
    for name, part in (("clear", clear), ("peak", win)):
        if len(part):
            print(f"  {name:5s} n={len(part):4d} " + " ".join(
                f"{c}={part[c].mean():.3f}" for c in ("N_rgb", "H_rgb", "B_rgb", "Q_rgb", "Q_thermal", "omega_rgb", "omega_thermal")))
    print(f"  wrote {stem}.csv/.png")


if __name__ == "__main__":
    main()
