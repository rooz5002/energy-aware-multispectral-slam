#!/usr/bin/env python3
"""Define each degradation profile's peak severity by MEASURED effect on the tuning sequence (DECISIONS D11),
and run the blur-indicator regression check (D7).

    python -m harness.calibrate_profiles

Severity is chosen so that the mean number of ORB keypoints detected in the RGB stream at the hold of the
profile is a target fraction of the clear-condition count (ORB-SLAM3's extractor, 1200 features).
Results are written into config/profiles.yaml. Tuning sequence only.
"""
import sys

import cv2
import numpy as np
import yaml

from controller import quality
from degradation.depth import apply_depth
from degradation.profiles import Profile, apply
from harness.sequence import REPO, Sequence, load_profile, load_yaml

sys.path.insert(0, str(REPO / "harness"))
import orbslam3_ms  # noqa: E402

TUNE = "office_loop_tune"
TARGET_RETENTION = {"fog_moderate": 0.50, "fog_severe": 0.10, "lowlight": 0.25}
N_FEATURES = 1200


def main():
    seq = Sequence(REPO / "data" / "sequences" / TUNE)
    spec = load_yaml("profiles.yaml")
    calib = load_yaml("calibration.yaml")
    v_ref = calib["v_ref"]["rgb"]
    report = {}
    clear_b = None
    for name, target in TARGET_RETENTION.items():
        window = load_profile(name, seq)
        hold = [i for i, t in enumerate(seq.stamps) if window.onset_s + window.ramp_s <= t <= window.offset_s - window.ramp_s][::10]
        frames = [seq.rgb(i) for i in hold]

        def measure(peak):
            prof = Profile(name, spec[name]["kind"], onset_s=-1.0, ramp_s=1e-6, hold_s=1e9, peak=peak, seed=spec[name]["seed"])
            n, b = [], []
            for i, bgr in zip(hold, frames):
                img = cv2.cvtColor(apply(prof, bgr, seq.stamps[i], i)[0], cv2.COLOR_BGR2GRAY)
                n.append(len(orbslam3_ms.orb_keypoints(img, N_FEATURES)))
                b.append(quality.blur_score(img, v_ref)[0])
            return float(np.mean(n)), float(np.mean(b))

        n_clear, b_clear = measure(0.0)
        clear_b = b_clear if clear_b is None else clear_b
        lo, hi = 0.0, 1.0
        for _ in range(9):                      # retention decreases with severity
            mid = (lo + hi) / 2
            if measure(mid)[0] / n_clear > target:
                lo = mid
            else:
                hi = mid
        peak = round((lo + hi) / 2, 3)
        n_peak, b_peak = measure(peak)
        final = Profile(name, spec[name]["kind"], onset_s=-1.0, ramp_s=1e-6, hold_s=1e9, peak=peak, seed=spec[name]["seed"])
        depth_valid = float(np.mean([apply_depth(final, seq.depth(i), seq.stamps[i], i, seq.camera["depth_scale"])[1] for i in hold]))
        depth_clear = float(np.mean([(seq.depth(i) > 0).mean() for i in hold]))
        spec[name].update(peak=peak, target_rgb_keypoint_retention=target,
                          measured_rgb_keypoint_retention=round(n_peak / n_clear, 3),
                          measured_on=TUNE, hold_frames_used=len(hold))
        report[name] = dict(peak=peak, retention=round(n_peak / n_clear, 3), keypoints_clear=round(n_clear, 1),
                            keypoints_at_hold=round(n_peak, 1), B_rgb_clear=round(b_clear, 3), B_rgb_at_hold=round(b_peak, 3),
                            depth_valid_fraction_clear=round(depth_clear, 3), depth_valid_fraction_at_hold=round(depth_valid, 3))
    header = ("# Degradation profiles, applied to the RGB stream (and to depth for fog) at replay time.\n"
              "# onset and hold are fractions of the sequence duration. Peak severities are defined by measured RGB\n"
              "# ORB-keypoint retention at the hold on the tuning sequence (harness/calibrate_profiles.py, DECISIONS D11).\n")
    (REPO / "config" / "profiles.yaml").write_text(header + yaml.safe_dump(spec, sort_keys=False))

    b = {k: v["B_rgb_at_hold"] for k, v in report.items()}
    checks = {"B_rgb fog_moderate < fog_severe": b["fog_moderate"] < b["fog_severe"],
              "B_rgb lowlight > clear": b["lowlight"] > report["lowlight"]["B_rgb_clear"]}
    out = dict(profiles=report, blur_regression_check=checks, blur_check_passed=all(checks.values()))
    (REPO / "results").mkdir(exist_ok=True)
    (REPO / "results" / "profile_calibration.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
    print(yaml.safe_dump(out, sort_keys=False))


if __name__ == "__main__":
    main()
