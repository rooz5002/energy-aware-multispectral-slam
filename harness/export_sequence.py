#!/usr/bin/env python3
"""Export a recorded rosbag to a frame dataset that the replay pipeline reads.

    python3 harness/export_sequence.py data/sequences/<sequence>

Writes next to the bag:
    rgb/<stamp>.png        8-bit BGR-ordered PNG of the RGB frame
    depth/<stamp>.png      16-bit PNG, millimetres (0 = invalid / out of range)
    thermal/<stamp>.png    16-bit PNG, raw radiometric counts (kelvin = counts * resolution in camera.yaml)
    frames.csv             one row per synchronised frame: stamp and the three file names
    camera.yaml            intrinsics of the RGB/depth and thermal cameras, depth scale, thermal resolution
    groundtruth.txt        TUM format, camera optical frame in the world, interpolated to frame stamps
    groundtruth_raw.txt    TUM format, base_footprint in the world, as published (50 Hz)
    export_report.yaml     frame counts, dropped-frame check, stamp statistics

The same exporter is meant to be used for bags recorded on the real robot
(RGB + depth only); thermal is skipped when the topic is absent.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = {"rgb": "/ms/rgb/image", "depth": "/ms/depth/image", "thermal": "/ms/thermal/image"}
INFO = {"rgb": "/ms/rgb/camera_info", "thermal": "/ms/thermal/camera_info"}
GT_TOPIC = "/ms/ground_truth"
DEPTH_SCALE = 1000.0          # PNG counts per metre
# Frames stamped before this are dropped: the first thermal frame(s) are rendered before the thermal
# sensor plugin has applied its resolution and come out at half temperature. The robot stands still
# for the first 2 s of every route, so nothing is lost.
WARMUP_S = 1.0
# camera_rgb_frame link in the robot model frame (sim/models/tb3_waffle_ms/model.sdf), sensor at link origin
T_BASE_CAM = np.array([0.069, -0.047, 0.107])
# optical frame (x right, y down, z forward) expressed in the body frame (x forward, y left, z up)
R_BASE_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x, y, z = np.copysign(x, R[2, 1] - R[1, 2]), np.copysign(y, R[0, 2] - R[2, 0]), np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def slerp(q0, q1, t):
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def main():
    seq_dir = Path(sys.argv[1]).resolve()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(seq_dir / "bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    by_topic = {v: k for k, v in TOPICS.items()}
    for k in TOPICS:
        (seq_dir / k).mkdir(exist_ok=True)
    stamps = {k: [] for k in TOPICS}
    infos, gt = {}, []
    png = [cv2.IMWRITE_PNG_COMPRESSION, 1]

    while reader.has_next():
        topic, data, _, _ = reader.read_next_ext()
        if topic in by_topic:
            kind = by_topic[topic]
            msg = deserialize_message(data, types[topic])
            t = stamp_ns(msg)
            if t < WARMUP_S * 1e9:
                continue
            path = str(seq_dir / kind / f"{t / 1e9:.6f}.png")
            if kind == "rgb":
                img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
                cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR), png)
            elif kind == "depth":
                d = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width)
                mm = np.where(np.isfinite(d) & (d * DEPTH_SCALE < 65535), d * DEPTH_SCALE, 0)
                cv2.imwrite(path, np.round(mm).astype(np.uint16), png)
            else:
                assert msg.encoding in ("mono16", "16UC1"), msg.encoding
                cv2.imwrite(path, np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.width), png)
            stamps[kind].append(t)
        elif topic in INFO.values() and topic not in infos:
            m = deserialize_message(data, types[topic])
            infos[topic] = dict(width=int(m.width), height=int(m.height), fx=float(m.k[0]), fy=float(m.k[4]), cx=float(m.k[2]), cy=float(m.k[5]),
                                distortion=[float(v) for v in m.d])
        elif topic == GT_TOPIC:
            m = deserialize_message(data, types[topic])
            p, q = m.pose.pose.position, m.pose.pose.orientation
            gt.append((stamp_ns(m), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    # ---- synchronisation and dropped-frame check
    active = {k: v for k, v in stamps.items() if v}
    common = sorted(set.intersection(*(set(v) for v in active.values())))
    dt = np.diff(common) / 1e6  # ms
    report = dict(
        sequence=seq_dir.name,
        warmup_dropped_s=WARMUP_S,
        frames_per_stream={k: len(v) for k, v in stamps.items()},
        synchronised_frames=len(common),
        unsynchronised_dropped={k: len(set(v) - set(common)) for k, v in active.items()},
        frame_interval_ms=dict(min=float(dt.min()), median=float(np.median(dt)), max=float(dt.max())),
        gaps_over_50ms=int((dt > 50).sum()),
        duration_s=float((common[-1] - common[0]) / 1e9),
        ground_truth_samples=len(gt),
    )
    for k, v in active.items():  # remove frames that are not in every stream
        for t in set(v) - set(common):
            (seq_dir / k / f"{t / 1e9:.6f}.png").unlink()
    with open(seq_dir / "frames.csv", "w") as f:
        f.write("stamp," + ",".join(active) + "\n")
        for t in common:
            name = f"{t / 1e9:.6f}"
            f.write(name + "," + ",".join(f"{k}/{name}.png" for k in active) + "\n")

    # ---- ground truth: raw (base_footprint) and camera optical frame at frame stamps
    gt = np.array(sorted(gt), dtype=np.float64)
    with open(seq_dir / "groundtruth_raw.txt", "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw  (base_footprint in world, as published)\n")
        for r in gt:
            f.write(f"{r[0] / 1e9:.6f} " + " ".join(f"{v:.9f}" for v in r[1:]) + "\n")
    q_base_opt = rot_to_quat(R_BASE_OPT)
    with open(seq_dir / "groundtruth.txt", "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw  (camera optical frame in world, interpolated to frame stamps)\n")
        n_written = 0
        for t in common:
            i = int(np.searchsorted(gt[:, 0], t))
            if i == 0 or i >= len(gt):
                continue
            a, b = gt[i - 1], gt[i]
            u = (t - a[0]) / (b[0] - a[0])
            q = slerp(a[4:8], b[4:8], u)
            R = quat_to_rot(q)
            p = a[1:4] + u * (b[1:4] - a[1:4]) + R @ T_BASE_CAM
            qc = rot_to_quat(R @ R_BASE_OPT)
            f.write(f"{t / 1e9:.6f} " + " ".join(f"{v:.9f}" for v in (*p, *qc)) + "\n")
            n_written += 1
    report["ground_truth_at_frames"] = n_written
    path_len = float(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum())
    report["path_length_m"] = path_len
    report["start_end_distance_m"] = float(np.linalg.norm(gt[-1, 1:4] - gt[0, 1:4]))

    # gz-sim 10 publishes a thermal camera_info built from a default 45 deg vertical FOV, which does not
    # match what is rendered. Edge alignment against the depth image shows the thermal frames have the
    # same field of view as the RGB/depth frames, so the thermal intrinsics are the RGB ones scaled by
    # the resolution ratio. The published values are kept for the record.
    rgb_info, thermal_info = infos.get(INFO["rgb"]), None
    if stamps["thermal"] and rgb_info:
        th_pub = infos[INFO["thermal"]]
        k = th_pub["width"] / rgb_info["width"]
        thermal_info = dict(width=th_pub["width"], height=th_pub["height"], fx=rgb_info["fx"] * k,
                            fy=rgb_info["fy"] * k, cx=rgb_info["cx"] * k, cy=rgb_info["cy"] * k,
                            distortion=rgb_info["distortion"])
    sensors = yaml.safe_load((seq_dir / "sensors.yaml").read_text())
    cam = dict(depth_scale=DEPTH_SCALE, thermal_resolution_K_per_count=sensors["thermal_resolution_K_per_count"],
               fps=float(sensors["fps"]),
               rgb=rgb_info, thermal=thermal_info, thermal_camera_info_as_published=infos.get(INFO["thermal"]),
               T_base_cam_translation=T_BASE_CAM.tolist(), R_base_optical=R_BASE_OPT.tolist())
    (seq_dir / "camera.yaml").write_text(yaml.safe_dump(cam, sort_keys=False))
    (seq_dir / "export_report.yaml").write_text(yaml.safe_dump(report, sort_keys=False))
    print(yaml.safe_dump(report, sort_keys=False))
    print(yaml.safe_dump(cam, sort_keys=False))


if __name__ == "__main__":
    main()
