#!/usr/bin/env python3
"""Sanity check of guide section 4.3: ORB keypoints per frame in clear conditions.

    python harness/check_keypoints.py data/sequences/<sequence> [--every N]

Reports the distribution of detected ORB keypoints for the thermal stream (after the fixed
temperature window, with and without CLAHE) and the RGB stream, and writes keypoint_check.yaml.
Target: thermal > 150 keypoints per frame at 320x240.
"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
import yaml

from controller.thermal import ThermalPreprocessor

TARGET = 150


def stats(counts):
    c = np.asarray(counts)
    return dict(min=int(c.min()), p05=int(np.percentile(c, 5)), median=int(np.median(c)), mean=float(round(c.mean(), 1)),
                max=int(c.max()), frames=int(len(c)), frames_below_target=int((c <= TARGET).sum()),
                fraction_below_target=float(round((c <= TARGET).mean(), 4)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sequence", type=Path)
    ap.add_argument("--every", type=int, default=5)
    ap.add_argument("--window", type=float, nargs=2, default=[284.0, 314.0], metavar=("T_MIN_K", "T_MAX_K"))
    ap.add_argument("--features", type=int, default=1200)
    args = ap.parse_args()
    cam = yaml.safe_load((args.sequence / "camera.yaml").read_text())
    rows = list(csv.DictReader(open(args.sequence / "frames.csv")))[::args.every]
    orb = cv2.ORB_create(nfeatures=args.features)
    res = cam["thermal_resolution_K_per_count"]
    pre = {name: ThermalPreprocessor(*args.window, kelvin_per_count=res, clahe=clahe)
           for name, clahe in (("thermal_fixed_window", False), ("thermal_fixed_window_clahe", True))}
    counts = {k: [] for k in (*pre, "rgb")}
    for r in rows:
        raw = cv2.imread(str(args.sequence / r["thermal"]), cv2.IMREAD_UNCHANGED)
        for name, fn in pre.items():
            counts[name].append(len(orb.detect(fn(raw), None)))
        counts["rgb"].append(len(orb.detect(cv2.imread(str(args.sequence / r["rgb"]), cv2.IMREAD_GRAYSCALE), None)))
    report = dict(sequence=args.sequence.name, orb_nfeatures=args.features, every_nth_frame=args.every,
                  thermal_window_K=args.window, target_keypoints=TARGET, **{k: stats(v) for k, v in counts.items()})
    (args.sequence / "keypoint_check.yaml").write_text(yaml.safe_dump(report, sort_keys=False))
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == "__main__":
    main()
