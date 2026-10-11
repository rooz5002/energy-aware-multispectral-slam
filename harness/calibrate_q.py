#!/usr/bin/env python3
"""Clear-condition quality means for the normalisation of DECISIONS D6.

    python -m harness.calibrate_q

Runs static_fusion on the CLEAR tuning sequence with raw (un-normalised) Q, then stores the mean Q of each
modality over tracked frames in config/calibration.yaml (q_clear_mean). Run after harness.calibrate.
"""
import json
import subprocess
import sys
from pathlib import Path

import pandas as pd
import yaml

from harness.sequence import REPO

TUNE = "office_loop_tune"


def main():
    out_root = REPO / "runs" / "calibration"
    cmd = [sys.executable, "-m", "harness.run_one", "--sequence", TUNE, "--profile", "clear", "--config",
           "static_fusion", "--raw-q", "--no-energy", "--out-root", str(out_root)]
    log = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True)
    run = next((l.split("run directory:", 1)[1].strip() for l in log.stdout.splitlines() if l.startswith("run directory:")), None)
    if log.returncode != 0 or run is None:
        raise SystemExit(log.stdout[-3000:] + log.stderr[-3000:])
    df = pd.read_csv(Path(run) / "frames.csv")
    df = df[(df.tracking_state == 2) & (df.n_source == "tracked_inliers")]
    summary = json.loads((Path(run) / "summary.json").read_text())
    path = REPO / "config" / "calibration.yaml"
    calib = yaml.safe_load(path.read_text())
    calib["q_clear_mean"] = {m: float(df[f"Q_{m}"].mean()) for m in ("rgb", "thermal")}
    calib["q_clear_mean_run"] = dict(run=Path(run).name, frames=int(len(df)), configuration="static_fusion",
                                     tracked_fraction=summary["tracked_fraction"],
                                     indicator_means={f"{k}_{m}": float(df[f"{k}_{m}"].mean())
                                                      for m in ("rgb", "thermal") for k in ("N", "H", "B")})
    path.write_text(yaml.safe_dump(calib, sort_keys=False))
    print(yaml.safe_dump({k: calib[k] for k in ("q_clear_mean", "q_clear_mean_run")}, sort_keys=False))


if __name__ == "__main__":
    main()
