#!/usr/bin/env python3
"""Calibrate the frozen image-statistics references on CLEAR frames of the tuning sequence.

    python -m harness.calibrate data/sequences/office_loop_tune

Writes config/calibration.yaml:
  thermal_window_K   fixed temperature window for the 16-bit -> 8-bit conversion,
  v_ref              reference variance of Laplacian per modality (median over clear frames), used by B_m.
Only the tuning sequence may be used here (guide sections 2.2 and 7.1).
"""
import argparse
import hashlib
from pathlib import Path

import cv2
import numpy as np
import yaml

from controller import quality
from controller.thermal import ThermalPreprocessor
from harness.inputs import thermal_sensor
from harness.sequence import REPO, Sequence

CLAHE = dict(clahe=True, clahe_clip=2.0, clahe_grid=8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sequence", type=Path)
    ap.add_argument("--every", type=int, default=5)
    args = ap.parse_args()
    seq = Sequence(args.sequence)
    role = yaml.safe_load((seq.path / f"{seq.name}.yaml").read_text()).get("role")
    if role != "tuning":
        raise SystemExit(f"{seq.name} has role '{role}': calibration is only allowed on the tuning sequence")
    idx = range(0, len(seq), args.every)
    res = seq.camera["thermal_resolution_K_per_count"]

    # window: robust range of scene temperatures, widened by 2 K and rounded to whole kelvin
    sensor = thermal_sensor(seq)
    counts = {i: sensor(seq.thermal_raw(i), i) for i in idx}
    kelvin = np.concatenate([counts[i].ravel()[::7] for i in idx]).astype(np.float64) * res
    lo, hi = np.percentile(kelvin, [0.5, 99.5])
    window = [float(np.floor(lo - 2.0)), float(np.ceil(hi + 2.0))]
    pre = ThermalPreprocessor(*window, kelvin_per_count=res, **CLAHE)

    v = {"rgb": [], "thermal": []}
    for i in idx:
        v["rgb"].append(quality.variance_of_laplacian(cv2.cvtColor(seq.rgb(i), cv2.COLOR_BGR2GRAY)))
        v["thermal"].append(quality.variance_of_laplacian(pre(counts[i])))
    out = dict(
        calibrated_on=seq.name,
        frames_csv_sha256=hashlib.sha256((seq.path / "frames.csv").read_bytes()).hexdigest(),
        frames_used=len(idx),
        thermal_window_K=window,
        thermal_percentiles_K=dict(p0_5=float(lo), p99_5=float(hi)),
        thermal_preprocessing=CLAHE,
        v_ref={m: float(np.median(x)) for m, x in v.items()},
        var_laplacian_spread={m: dict(p05=float(np.percentile(x, 5)), p95=float(np.percentile(x, 95))) for m, x in v.items()},
    )
    out["blur_prefilter_sigma_px"] = quality.BLUR_PREFILTER_SIGMA_PX
    # keep the clear-condition quality means (D6) only if the image calibration they depend on is unchanged
    path = REPO / "config" / "calibration.yaml"
    old = yaml.safe_load(path.read_text()) if path.exists() else {}
    if all(old.get(k) == out[k] for k in ("thermal_window_K", "v_ref", "frames_csv_sha256")) and "q_clear_mean" in old:
        out["q_clear_mean"], out["q_clear_mean_run"] = old["q_clear_mean"], old.get("q_clear_mean_run")
    (REPO / "config" / "calibration.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
    print(yaml.safe_dump(out, sort_keys=False))


if __name__ == "__main__":
    main()
