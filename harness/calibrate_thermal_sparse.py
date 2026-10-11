#!/usr/bin/env python3
"""Define the replay-time `thermal_sparse` variant (DECISIONS v3, D14).

    python -m harness.calibrate_thermal_sparse

The recorded thermal stream is made poorer at replay: stronger optics blur (fixed) and contrast compression
about the middle of the temperature window (searched), so that on the CLEAR tuning sequence the
thermal-to-RGB ORB keypoint ratio is about PST900's 0.13. Image statistics only. The result is written to
config/sensors.yaml:thermal_variants.thermal_sparse.
"""
import sys

import numpy as np
import yaml

from degradation.profiles import Profile
from harness.inputs import Streams
from harness.sequence import REPO, Sequence, load_yaml

sys.path.insert(0, str(REPO / "harness"))
import orbslam3_ms  # noqa: E402

TUNE = "office_loop_tune"
TARGET_RATIO = 0.13
BLUR_SIGMA_PX = 1.5
N_FEATURES = 1200


def main():
    seq = Sequence(REPO / "data" / "sequences" / TUNE)
    calib = load_yaml("calibration.yaml")
    idx = range(0, len(seq), 20)
    clear = Profile("clear", "none")
    base = Streams(seq, clear, calib)
    rgb = float(np.mean([len(orbslam3_ms.orb_keypoints(base.grey(i)[0], N_FEATURES)) for i in idx]))
    pivot = float(np.mean(calib["thermal_window_K"]))

    def thermal_kp(gain):
        st = Streams(seq, clear, calib, thermal_variant_params=dict(blur_sigma_px=BLUR_SIGMA_PX, contrast_gain=gain,
                                                                    contrast_pivot_k=pivot))
        return float(np.mean([len(orbslam3_ms.orb_keypoints(st.thermal8(i), N_FEATURES)) for i in idx]))

    lo, hi = 0.0, 1.0                       # keypoints grow with contrast gain
    for _ in range(8):
        mid = (lo + hi) / 2
        if thermal_kp(mid) / rgb < TARGET_RATIO:
            lo = mid
        else:
            hi = mid
    gain = round((lo + hi) / 2, 3)
    th = thermal_kp(gain)
    path = REPO / "config" / "sensors.yaml"
    cfg = yaml.safe_load(path.read_text())
    cfg.setdefault("thermal_variants", {})["thermal_sparse"] = dict(
        blur_sigma_px=BLUR_SIGMA_PX, contrast_gain=gain, contrast_pivot_k=pivot, target_keypoint_ratio=TARGET_RATIO,
        measured_keypoint_ratio=round(th / rgb, 3), thermal_keypoints=round(th, 1), rgb_keypoints=round(rgb, 1),
        measured_on=f"{TUNE} (clear)", note="D14 robustness check only; the recorded scene is unchanged")
    header = "".join(l for l in path.read_text().splitlines(keepends=True) if l.startswith("#"))
    path.write_text(header + yaml.safe_dump(cfg, sort_keys=False, width=200))
    print(yaml.safe_dump(cfg["thermal_variants"], sort_keys=False))


if __name__ == "__main__":
    main()
