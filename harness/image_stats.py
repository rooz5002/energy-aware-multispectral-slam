#!/usr/bin/env python3
"""Image statistics used to calibrate the simulated thermal scene against real thermal images (DECISIONS D2.1).

Per frame and modality: ORB keypoints (ORB-SLAM3's extractor, same settings for both), image entropy and
edge density (fraction of Canny edge pixels). The quantity that is matched is the THERMAL-TO-RGB RATIO of
each statistic. No SLAM result is used.

    python -m harness.image_stats sim data/sequences/office_loop_tune
    python -m harness.image_stats pst900 sensors/PST900_RGBT_Dataset

Real pairs: RGB is resized to 640 px wide and thermal to 320 px wide, the same 2:1 relation as the
simulated sensors (the dataset stores thermal resampled onto the RGB grid).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from controller import quality
from harness.sequence import REPO, Sequence, load_yaml

sys.path.insert(0, str(REPO / "harness"))
import orbslam3_ms  # noqa: E402

N_FEATURES = 1200
CANNY = (50, 150)


def stats(img8, invalid=None):
    """invalid: optional boolean mask of pixels without data; keypoints, edges and entropy ignore it."""
    kp = orbslam3_ms.orb_keypoints(np.ascontiguousarray(img8), N_FEATURES)
    edges = cv2.Canny(img8, *CANNY) > 0
    if invalid is None:
        return dict(orb_keypoints=float(len(kp)), entropy=quality.entropy_score(img8) * 8.0, edge_density=float(edges.mean()))
    xy = np.clip(np.rint(kp).astype(int), 0, [img8.shape[1] - 1, img8.shape[0] - 1])
    keep = ~invalid[xy[:, 1], xy[:, 0]] if len(kp) else np.zeros(0, bool)
    valid = ~invalid
    hist = np.bincount(img8[valid], minlength=256).astype(np.float64)
    pr = hist[hist > 0] / valid.sum()
    # keypoint count scaled to the full frame area so that it is comparable between frames
    return dict(orb_keypoints=float(keep.sum() / valid.mean()), entropy=float(-(pr * np.log2(pr)).sum()),
                edge_density=float(edges[valid].mean()))


def summarise(pairs, label, extra):
    rgb = [stats(p[0], p[2] if len(p) > 2 else None) for p in pairs]
    th = [stats(p[1], p[3] if len(p) > 2 else None) for p in pairs]
    out = dict(source=label, frames=len(pairs), orb_nfeatures=N_FEATURES, canny_thresholds=list(CANNY), **extra)
    for k in ("orb_keypoints", "entropy", "edge_density"):
        r, t = np.array([x[k] for x in rgb]), np.array([x[k] for x in th])
        out[k] = dict(rgb_mean=float(r.mean()), thermal_mean=float(t.mean()), ratio_of_means=float(t.mean() / r.mean()),
                      rgb_median=float(np.median(r)), thermal_median=float(np.median(t)),
                      median_of_frame_ratios=float(np.median(t / np.maximum(r, 1e-9))))
    return out


def sim_pairs(seq_dir, every):
    from degradation.profiles import Profile
    from harness.inputs import Streams
    seq = Sequence(seq_dir)
    streams = Streams(seq, Profile("clear", "none"), load_yaml("calibration.yaml"))
    return [(streams.grey(i)[0], streams.thermal8(i)) for i in range(0, len(seq), every)]


def pst900_pairs(root, limit):
    """(rgb 640 px wide, thermal 320 px wide, invalid mask for each).

    PST900 stores thermal reprojected onto the RGB image, with zero-valued holes where there is no thermal
    data. Holes are filled by inpainting before the statistics and a dilated hole mask is excluded from them
    (in both modalities), so hole borders are not counted as thermal structure. Frames with less than 80 %
    thermal coverage are skipped.
    """
    pairs, skipped = [], 0
    for rgb_path in sorted(Path(root).rglob("rgb/*.png"))[:limit]:
        th_path = rgb_path.parent.parent / "thermal" / rgb_path.name
        if not th_path.exists():
            continue
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_GRAYSCALE)
        th = cv2.imread(str(th_path), cv2.IMREAD_GRAYSCALE)
        hole = (th == 0).astype(np.uint8)
        if hole.mean() > 0.2:
            skipped += 1
            continue
        th = cv2.inpaint(th, hole, 5, cv2.INPAINT_TELEA)
        hole = cv2.dilate(hole, np.ones((31, 31), np.uint8))
        h = int(round(640 * rgb.shape[0] / rgb.shape[1]))
        pairs.append((cv2.resize(rgb, (640, h), interpolation=cv2.INTER_AREA),
                      cv2.resize(th, (320, h // 2), interpolation=cv2.INTER_AREA),
                      cv2.resize(hole, (640, h), interpolation=cv2.INTER_NEAREST) > 0,
                      cv2.resize(hole, (320, h // 2), interpolation=cv2.INTER_NEAREST) > 0))
    print(f"PST900: {len(pairs)} pairs used, {skipped} skipped for low thermal coverage")
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("source", choices=["sim", "pst900"])
    ap.add_argument("path", type=Path)
    ap.add_argument("--every", type=int, default=10)
    ap.add_argument("--limit", type=int, default=100000)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    if args.source == "sim":
        out = summarise(sim_pairs(args.path, args.every), f"simulation: {args.path.name} (clear)", dict(every_nth_frame=args.every))
    else:
        out = summarise(pst900_pairs(args.path, args.limit), "PST900 (real, FLIR Boson AGC 8-bit + RGB)", {})
    text = yaml.safe_dump(out, sort_keys=False)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)


if __name__ == "__main__":
    main()
