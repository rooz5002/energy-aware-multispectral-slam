"""Trajectory errors (evo) and per-run summary statistics.

Tracking loss (DECISIONS D4): after a loss ORB-SLAM3 continues in a new Atlas map with its own origin, so
poses from different maps are never aligned together. The MAIN MAP is the map that contains the most
tracked frames. The primary ATE is the final trajectory restricted to the main map (SE(3) alignment);
frames outside the main map count as lost. A run is SUCCESSFUL if at least 90 % of its frames are in the
main map.
"""
import copy
from pathlib import Path

import numpy as np

STATE_OK = 2
SUCCESS_TRACKED_FRACTION = 0.90


def write_tum(path, stamped_poses):
    """stamped_poses: [(t, 4x4 Twc, map_id)] -> 't tx ty tz qx qy qz qw map_id'."""
    from evo.core.transformations import quaternion_from_matrix
    with open(path, "w") as f:
        for t, T, map_id in stamped_poses:
            qw, qx, qy, qz = quaternion_from_matrix(T)
            f.write(f"{t:.6f} {T[0, 3]:.7f} {T[1, 3]:.7f} {T[2, 3]:.7f} {qx:.7f} {qy:.7f} {qz:.7f} {qw:.7f} {int(map_id)}\n")


def main_map_trajectory(all_maps_file, ok_stamps, out_file):
    """Keep poses of frames that were tracked (state OK) and belong to the map with the most such frames.

    Returns (main map id or None, poses in the main map, poses tracked in any map).
    """
    p = Path(all_maps_file)
    if not p.exists() or p.stat().st_size == 0:
        return None, 0, 0
    data = np.atleast_2d(np.loadtxt(p))
    ok = np.isin(np.round(data[:, 0], 6), np.round(np.asarray(ok_stamps), 6))
    data = data[ok]
    if len(data) == 0:
        return None, 0, 0
    ids, counts = np.unique(data[:, 8].astype(int), return_counts=True)
    main = int(ids[np.argmax(counts)])
    sel = data[data[:, 8].astype(int) == main]
    np.savetxt(out_file, sel[:, :8], fmt="%.9f")
    return main, len(sel), len(data)


def trajectory_errors(gt_file, est_file, rpe_delta_m=1.0):
    """ATE RMSE with SE(3) alignment and RPE over rpe_delta_m; None values if there is no usable trajectory."""
    from evo.core import metrics, sync
    from evo.core.units import Unit
    from evo.tools import file_interface
    empty = dict(ate_rmse_m=None, ate_max_m=None, rpe_rmse_m=None, error_over_time=None)
    if not Path(est_file).exists() or Path(est_file).stat().st_size == 0:
        return empty
    gt = file_interface.read_tum_trajectory_file(str(gt_file))
    est = file_interface.read_tum_trajectory_file(str(est_file))
    if est.num_poses < 10:
        return empty
    gt, est = sync.associate_trajectories(gt, est, max_diff=0.01)
    est = copy.deepcopy(est)
    est.align(gt, correct_scale=False)
    err = np.linalg.norm(est.positions_xyz - gt.positions_xyz, axis=1)
    out = dict(ate_rmse_m=float(np.sqrt((err ** 2).mean())), ate_max_m=float(err.max()),
               error_over_time=(gt.timestamps.copy(), err), rpe_rmse_m=None)
    try:
        rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=rpe_delta_m, delta_unit=Unit.meters,
                          all_pairs=False)
        rpe.process_data((gt, est))
        out["rpe_rmse_m"] = float(rpe.get_statistic(metrics.StatisticsType.rmse))
    except Exception:
        pass
    return out


def summarise(df, gt_file, out_dir, n_frames_input, wall_s):
    out_dir = Path(out_dir)
    n_skipped = int(df.skipped.sum())
    df = df[df.skipped == 0]          # frames skipped by a reduced processing rate are not "lost"
    ok = df.tracking_state.to_numpy() == STATE_OK
    ok_stamps = df.t.to_numpy()[ok]
    res, info = {}, {}
    for name, src in (("final", "trajectory_all_maps.txt"), ("online", "trajectory_online_all_maps.txt")):
        main_file = out_dir / f"trajectory_{name}_main_map_tum.txt"
        main_id, n_main, n_any = main_map_trajectory(out_dir / src, ok_stamps, main_file)
        info[name] = (main_id, n_main, n_any)
        res[name] = trajectory_errors(gt_file, main_file) if main_id is not None else trajectory_errors(gt_file, "/nonexistent")
        if res[name]["error_over_time"] is not None:
            t, e = res[name]["error_over_time"]
            np.savetxt(out_dir / f"error_over_time_{name}.csv", np.column_stack([t, e]), delimiter=",",
                       header="t,position_error_m", comments="")
    final, online = res["final"], res["online"]
    main_id, n_main, n_any = info["final"]
    was_ok_before = np.concatenate([[False], ok[:-1]])
    tracked_main = n_main / len(df)
    return dict(
        ate_rmse_m=final["ate_rmse_m"], ate_max_m=final["ate_max_m"], rpe_rmse_m=final["rpe_rmse_m"],
        ate_online_rmse_m=online["ate_rmse_m"], rpe_online_rmse_m=online["rpe_rmse_m"],
        main_map_id=main_id, poses_main_map=int(n_main), poses_any_map=int(n_any),
        tracked_fraction_main_map=float(tracked_main), success=bool(tracked_main >= SUCCESS_TRACKED_FRACTION),
        tracked_fraction=float(ok.mean()), tracking_losses=int((was_ok_before & ~ok).sum()),
        reinitialisations=int(max(0, df.n_maps.max() - 1)), maps_visited=int(df.map_id[ok].nunique()),
        frames_processed=int(len(df)), frames_skipped=n_skipped, frames_input=int(n_frames_input), frames_lost=int((~ok).sum()),
        keyframes_created=int(df.keyframe.sum()), mean_depth_valid_fraction=float(df.depth_valid_fraction.mean()),
        mean_track_ms=float(df.track_ms.mean()), p95_track_ms=float(df.track_ms.quantile(0.95)),
        max_lag_s=float(df.lag_s.max()), wall_s=float(wall_s),
    )
