"""End-to-end test (guide section 11): 20 s of the tuning sequence through the four main configurations.

Runs without energy measurement so that it does not depend on RAPL permissions; skipped when the recorded
data or the ORB-SLAM3 binding are missing.   pytest -m integration
"""
import json
import subprocess
import sys
from pathlib import Path

import pandas as pd
import pytest

REPO = Path(__file__).resolve().parents[1]
SEQ = REPO / "data" / "sequences" / "office_loop_tune"
FILES = ["frames.csv", "trajectory_tum.txt", "trajectory_all_maps.txt", "trajectory_online_all_maps.txt", "trajectory_final_main_map_tum.txt", "groundtruth.txt", "summary.json",
         "provenance.json", "transitions.json", "orbslam3_settings.yaml"]

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not (SEQ / "frames.csv").exists(), reason="tuning sequence not recorded"),
    pytest.mark.skipif(not list((REPO / "harness").glob("orbslam3_ms*.so")), reason="ORB-SLAM3 binding not built"),
]


@pytest.mark.parametrize("config", ["static_rgb", "static_thermal", "static_fusion", "adaptive"])
def test_twenty_seconds_end_to_end(config, tmp_path):
    cmd = [sys.executable, "-m", "harness.run_one", "--sequence", "office_loop_tune", "--profile", "fog_moderate",
           "--config", config, "--no-energy", "--max-frames", "600", "--out-root", str(tmp_path)]
    res = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=300)
    assert res.returncode == 0, res.stdout[-2000:] + res.stderr[-2000:]
    (run,) = list(tmp_path.iterdir())
    for name in FILES:
        assert (run / name).exists(), name
    df = pd.read_csv(run / "frames.csv")
    assert len(df) == 600
    summary = json.loads((run / "summary.json").read_text())
    assert summary["frames_processed"] == 600 and summary["development_run"] is True
    assert summary["tracked_fraction_main_map"] > 0.9 and summary["success"] is True
    assert summary["ate_rmse_m"] < 0.2
    if config == "static_rgb":
        assert df.n_features_thermal.max() == 0 and df.n_keys_thermal.max() == 0
    if config == "static_thermal":
        assert df.n_features_rgb.max() == 0 and df.n_keys_rgb.max() == 0
    if config in ("static_fusion", "adaptive"):
        assert (df.n_features_rgb + df.n_features_thermal == 1200).all()
        assert ((df.omega_rgb + df.omega_thermal).round(6) == 1.0).all()
    if config == "static_fusion":
        assert (df.n_features_rgb == 600).all()
    assert (df.level == 0).all()   # no energy measurement: scheduler cannot leave Level 0
