#!/usr/bin/env python3
"""Replay one recorded sequence through one configuration and write a run directory (guide section 9).

    python -m harness.run_one --sequence office_loop_tune --profile fog_moderate --config adaptive --rep 0

One process per run: ORB-SLAM3 keeps process-wide state. Gazebo must not be running.

Timed section (what the energy measurement covers): per frame, the quality indicators, the scheduler and
ORB-SLAM3 tracking, paced at the recorded frame rate. Image loading, RGB degradation and thermal 8-bit
conversion are done before the timed section, because they stand in for the physical sensors.
"""
import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import yaml

from controller import quality
from controller.energy import EnergyAccount, RaplReader
from controller.scheduler import Scheduler
from harness import evaluate
from harness.inputs import Streams
from harness.orbslam_settings import write_settings
from harness.sequence import REPO, Sequence, load_profile, load_yaml, quality_params, scheduler_params

ORBSLAM3 = REPO / "third_party" / "ORB_SLAM3"
MODALITIES = ("rgb", "thermal")
LC_MODE = {"full": 0, "reduced": 1, "off": 2}
STATE_OK = 2


def git(*args, cwd=REPO):
    try:
        return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None


def cpu_temperature_c():
    for name in Path("/sys/class/hwmon").glob("hwmon*/name"):
        if name.read_text().strip() in ("k10temp", "coretemp"):
            try:
                return int((name.parent / "temp1_input").read_text()) / 1000.0
            except OSError:
                return None
    return None


def provenance(args, params, calib, profile, seq, config):
    blob = json.dumps(dict(params=params, calib=calib, config=config, profile=profile.__dict__), sort_keys=True, default=str)
    cpu = next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), None)
    gov = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    return dict(
        sequence=seq.name, profile=profile.__dict__, configuration=args.config, repeat=args.rep,
        parameters=params, parameters_file=str(args.params), calibration=calib, configuration_spec=config,
        config_sha256=hashlib.sha256(blob.encode()).hexdigest(),
        repo_commit=git("rev-parse", "HEAD"), repo_dirty_files=len((git("status", "--porcelain") or "").splitlines()),
        orbslam3_commit=git("rev-parse", "HEAD", cwd=ORBSLAM3),
        orbslam3_patch_sha256=hashlib.sha256((git("diff", "HEAD", cwd=ORBSLAM3) or "").encode()).hexdigest(),
        machine=dict(host=platform.node(), cpu=cpu, cores=os.cpu_count(), kernel=platform.release(),
                     governor=gov.read_text().strip() if gov.exists() else None, loadavg_start=os.getloadavg(),
                     on_ac_power=(Path("/sys/class/power_supply/ACAD/online").read_text().strip() == "1")
                     if Path("/sys/class/power_supply/ACAD/online").exists() else None,
                     cpu_temp_start_c=cpu_temperature_c()),
        python=sys.version.split()[0], opencv=cv2.__version__, numpy=np.__version__,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--profile", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--rep", type=int, default=0)
    ap.add_argument("--params", type=Path, default=REPO / "config" / "default.yaml")
    ap.add_argument("--out-root", type=Path, default=REPO / "runs")
    ap.add_argument("--max-frames", type=int, default=0, help="development only: stop after this many frames")
    ap.add_argument("--idle-power-w", type=float, default=None,
                    help="idle package power measured before the batch (run_matrix passes it)")
    ap.add_argument("--e0", type=float, default=None,
                    help="energy budget E0 in joules for configurations with the scheduler (run_matrix: k * E_static_fusion)")
    ap.add_argument("--budget", default=None, help="label of the E0 budget (tight / generous), logged only")
    ap.add_argument("--thermal-variant", default=None,
                    help="replay-time thermal variant from sensors.yaml:thermal_variants (D14: thermal_sparse)")
    ap.add_argument("--raw-q", action="store_true",
                    help="calibration only: do not normalise Q (used for the run that measures the clear-condition means)")
    ap.add_argument("--no-energy", action="store_true",
                    help="development only: no energy measurement, E_avail stays at E0 (results not for the paper)")
    args = ap.parse_args()

    seq = Sequence(REPO / "data" / "sequences" / args.sequence)
    params = yaml.safe_load(args.params.read_text())
    calib = load_yaml("calibration.yaml")
    config = load_yaml("configurations.yaml")[args.config]
    sensors = load_yaml("sensors.yaml")
    profile = load_profile(args.profile, seq)
    active = {m: bool(config[m]) for m in MODALITIES}
    qp, sp = quality_params(params, calib), scheduler_params(params)
    sched = Scheduler(sp, adaptive_weights=config["weights"] == "adaptive", energy_scheduling=bool(config["scheduler"]),
                      fixed_omega=config.get("omega"))

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = args.out_root / f"{stamp}_{seq.name}_{args.profile}_{args.config}{'_' + args.thermal_variant if args.thermal_variant else ''}_{args.rep}"
    out.mkdir(parents=True)
    prov = provenance(args, params, calib, profile, seq, config)
    prov["start_utc"] = datetime.now(timezone.utc).isoformat()
    prov["degradation_window_s"] = [profile.onset_s, profile.offset_s] if profile.kind != "none" else None

    # ------------------------------------------------------------------ energy set-up (fails loudly)
    needs_budget = bool(config["scheduler"])
    if args.no_energy:
        rapl, e0, sensor_w = None, None, {}
        prov["energy_source"] = "none (development run, not for the paper)"
    else:
        if params["energy"]["source"] != "measured":
            raise SystemExit("only energy.source = measured is implemented")
        if sensors.get("status") != "verified":
            raise SystemExit("config/sensors.yaml is not verified")
        sensor_w = {"rgbd": sensors["rgbd"]["power_w"], "thermal": sensors["thermal"]["power_w"]}
        e0 = args.e0
        if needs_budget and e0 is None:
            raise SystemExit("this configuration uses the scheduler: pass --e0 (run_matrix derives it from static_fusion)")
        rapl = RaplReader()
        prov["energy_source"] = "measured (RAPL package energy)"
        prov["sensors"] = sensors
    prov["E0_joules"], prov["budget"] = e0, args.budget
    account = EnergyAccount(e0 if (e0 and needs_budget) else 1.0, sensor_w)

    q_norm = {m: 1.0 for m in MODALITIES}
    if not args.raw_q:
        if "q_clear_mean" not in calib:
            raise SystemExit("config/calibration.yaml has no q_clear_mean: run `make calibrate-q` (or pass --raw-q)")
        q_norm = dict(calib["q_clear_mean"])
    prov["q_normalisation"] = q_norm

    # ------------------------------------------------------------------ inputs, prepared before the timed section
    n = len(seq) if not args.max_frames else min(len(seq), args.max_frames)
    streams = Streams(seq, profile, calib, thermal_variant=args.thermal_variant)
    prov["thermal_variant"] = args.thermal_variant
    grey, depth, thermal, thermal_native, severity, depth_valid = [], [], [], [], [], []
    for i in range(n):
        g, s = streams.grey(i)
        d, valid = streams.depth(i)
        grey.append(g)
        severity.append(s)
        depth.append(d)
        depth_valid.append(valid)
        # thermal is brought to the RGB pixel grid so that both modalities share coordinates and depth
        # (quality is assessed on the native-resolution thermal image, SLAM gets the resized one)
        th = streams.thermal8(i) if active["thermal"] else None
        thermal_native.append(th)
        thermal.append(cv2.resize(th, (g.shape[1], g.shape[0]), interpolation=cv2.INTER_LINEAR) if th is not None else None)
    prov["input_sha256"] = hashlib.sha256(b"".join(grey[i].tobytes() + depth[i].tobytes() +
                                                    (thermal[i].tobytes() if thermal[i] is not None else b"")
                                                    for i in range(0, n, 50))).hexdigest()

    settings = write_settings(seq.camera, out / "orbslam3_settings.yaml", n_features=sp.levels[0].n_features)
    sys.path.insert(0, str(REPO / "harness"))
    import orbslam3_ms
    slam = orbslam3_ms.System(str(ORBSLAM3 / "Vocabulary" / "ORBvoc.txt"), str(settings))

    # ------------------------------------------------------------------ timed section
    rows, online = [], []
    prev = None                       # tracker feedback of the previous frame
    sensors_on = {"rgbd": True, "thermal": active["thermal"]}   # u_m during the interval that is ending
    t_seq0, wall0 = seq.stamps[0], time.monotonic()
    wall_prev, joules_prev = wall0, (rapl.joules() if rapl else 0.0)
    for i in range(n):
        t = seq.stamps[i]
        delay = (t - t_seq0) - (time.monotonic() - wall0)
        if delay > 0:
            time.sleep(delay)
        lag = max(0.0, -delay)

        # energy of the interval that just ended
        wall_now = time.monotonic()
        dt = wall_now - wall_prev
        joules_now = rapl.joules() if rapl else 0.0
        sensor_w_now = account.sensor_power(sensors_on) if rapl else float("nan")
        e_cons_w = account.update(dt, joules_now - joules_prev, sensors_on) if rapl else float("nan")
        compute_w = (joules_now - joules_prev) / dt if (rapl and dt > 0) else float("nan")
        wall_prev, joules_prev = wall_now, joules_now

        # reduced processing rate (a D5 tuning lever): the frame is not processed at all
        stride = sched.frame_stride()
        if stride > 1 and prev is not None and (i % stride) != 0:
            rows.append(dict(rows[-1], t=t, frame=i, severity=severity[i], depth_valid_fraction=depth_valid[i],
                             skipped=1, keyframe=0, track_ms=0.0, lag_s=lag, compute_power_w=compute_w,
                             sensor_power_w=sensor_w_now, e_cons_w=e_cons_w,
                             e_avail_j=account.available if (rapl and needs_budget) else float("nan"),
                             e_avail_frac=account.fraction if (rapl and needs_budget) else float("nan")))
            continue

        # quality (Eq. 1): H and B from this frame, N from the tracker's result on the previous frame
        in_use = sched.modalities_in_use(active)
        tracking_ok = prev is None or prev["state"] == STATE_OK
        img = {"rgb": grey[i], "thermal": thermal_native[i]}
        q, n_source = {}, "none"
        for m in MODALITIES:
            if not in_use[m]:
                continue
            had_budget = prev is not None and prev[f"applied_n_features_{m}"] > 0
            if not had_budget:
                count, budget = 0.0, 1.0   # no tracker feedback for this modality yet: N = 0, logged as such
            elif prev["state"] == STATE_OK:
                count, budget, n_source = prev[f"n_inliers_{m}"], prev[f"applied_n_features_{m}"], "tracked_inliers"
            else:
                count, budget, n_source = prev[f"n_keys_{m}"], prev[f"applied_n_features_{m}"], "detected_keypoints"
            q[m] = quality.assess(img[m], count, budget, m, qp)
        # normalised quality (D6). A modality that was off in the previous frame has no tracker feedback,
        # so its score is "not available" for this frame (the scheduler then uses equal weights).
        qn = {m: (q[m].Q / q_norm[m] if (m in q and (prev is None or prev[f"applied_n_features_{m}"] > 0)) else None)
              for m in MODALITIES}

        e_frac = account.fraction if (rapl and needs_budget) else 1.0
        ctl = sched.step(t, qn, e_frac, active, tracking_ok=tracking_ok)
        t0 = time.perf_counter()
        res = slam.track(grey[i], depth[i], thermal[i] if ctl.enabled["thermal"] else None, t,
                         ctl.n_features["rgb"], ctl.n_features["thermal"],
                         ctl.min_keyframe_interval, ctl.local_ba_enabled, LC_MODE[ctl.loop_closure_mode],
                         ctl.local_ba_every)
        track_ms = (time.perf_counter() - t0) * 1e3
        prev = res
        sensors_on = {"rgbd": True, "thermal": bool(ctl.enabled["thermal"])}

        if res["state"] == STATE_OK:
            Twc = np.linalg.inv(res["Tcw"].astype(np.float64))
            online.append((t, Twc, res["map_id"]))
        row = dict(t=t, frame=i, skipped=0, severity=severity[i], depth_valid_fraction=depth_valid[i], n_source=n_source,
                   level=ctl.level, thermal_priority=int(ctl.thermal_priority), recovery=int(ctl.recovery),
                   u_thermal=int(ctl.enabled["thermal"]), tracking_state=res["state"], map_id=res["map_id"],
                   keyframe=int(res["keyframe"]), n_maps=res["n_maps"], n_keyframes=res["n_keyframes"],
                   track_ms=track_ms, lag_s=lag, compute_power_w=compute_w, sensor_power_w=sensor_w_now,
                   e_cons_w=e_cons_w,
                   e_avail_j=account.available if (rapl and needs_budget) else float("nan"),
                   e_avail_frac=account.fraction if (rapl and needs_budget) else float("nan"),
                   n_features_total=ctl.n_features_total,
                   applied_min_keyframe_interval=res["applied_min_keyframe_interval"],
                   applied_local_ba=int(res["applied_local_ba"]), applied_local_ba_every=res["applied_local_ba_every"],
                   applied_loop_closure_mode=res["applied_loop_closure_mode"])
        for m in MODALITIES:
            qm = q.get(m)
            row.update({f"N_{m}": qm.N if qm else np.nan, f"H_{m}": qm.H if qm else np.nan,
                        f"B_{m}": qm.B if qm else np.nan, f"Q_{m}": qm.Q if qm else np.nan,
                        f"Qn_{m}": qn[m] if qn[m] is not None else np.nan,
                        f"omega_{m}": ctl.omega[m], f"n_features_{m}": res[f"applied_n_features_{m}"],
                        f"n_keys_{m}": res[f"n_keys_{m}"], f"n_inliers_{m}": res[f"n_inliers_{m}"]})
        rows.append(row)
    wall_total = time.monotonic() - wall0
    # energy of the timed section only (the reader also saw the input preparation before it)
    joules_total = account.compute_j + ((rapl.joules() - joules_prev) if rapl else 0.0) if rapl else float("nan")

    slam.shutdown()
    slam.save_trajectory_tum(str(out / "trajectory_tum.txt"))
    slam.save_keyframe_trajectory_tum(str(out / "keyframe_trajectory_tum.txt"))
    slam.save_trajectory_all_maps(str(out / "trajectory_all_maps.txt"))

    # ------------------------------------------------------------------ outputs
    df = pd.DataFrame(rows)
    df.to_csv(out / "frames.csv", index=False)
    evaluate.write_tum(out / "trajectory_online_all_maps.txt", online)
    gt = seq.path / "groundtruth.txt"
    os.symlink(gt, out / "groundtruth.txt")
    (out / "transitions.json").write_text(json.dumps(sched.transitions, indent=1))

    summary = evaluate.summarise(df, gt, out, n_frames_input=n, wall_s=wall_total)
    summary.update(sequence=seq.name, profile=args.profile, configuration=args.config, repeat=args.rep,
                   energy_total_j_compute_raw=joules_total,
                   energy_total_j_sensors_model=account.sensor_j if rapl else float("nan"),
                   energy_thermal_sensor_j=float((df.u_thermal * sensor_w.get("thermal", 0.0) * df.t.diff().fillna(0)).sum())
                   if rapl else float("nan"),
                   thermal_on_fraction=float(df.u_thermal.mean()),
                   recovery_frames=int(df.recovery.sum()),
                   recovery_energy_j=float((df.recovery * df.e_cons_w * df.t.diff().fillna(0)).sum()) if rapl else float("nan"),
                   budget=args.budget, thermal_variant=args.thermal_variant or "-",
                   mean_compute_power_w=joules_total / wall_total if rapl else float("nan"),
                   idle_power_w=args.idle_power_w,
                   energy_total_j_compute_idle_subtracted=(joules_total - args.idle_power_w * wall_total)
                   if (rapl and args.idle_power_w is not None) else float("nan"),
                   energy_total_j=(joules_total + account.sensor_j) if rapl else float("nan"),
                   e0_joules=e0 if (rapl and needs_budget) else float("nan"),
                   level_time_fraction={str(k): float(v) for k, v in df.level.value_counts(normalize=True).sort_index().items()},
                   level_transitions=sum(1 for tr in sched.transitions if tr["kind"] == "level"),
                   development_run=bool(args.no_energy or args.raw_q or args.max_frames or params.get("status") != "frozen"))
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    prov["end_utc"] = datetime.now(timezone.utc).isoformat()
    prov["machine"]["loadavg_end"] = os.getloadavg()
    prov["machine"]["cpu_temp_end_c"] = cpu_temperature_c()
    (out / "provenance.json").write_text(json.dumps(prov, indent=1, default=str))
    print(json.dumps({k: summary[k] for k in ("configuration", "profile", "ate_rmse_m", "ate_online_rmse_m",
                                              "tracked_fraction_main_map", "success", "tracking_losses", "reinitialisations",
                                              "mean_track_ms", "max_lag_s", "wall_s")}, indent=1))
    print("run directory:", out)
    # ORB-SLAM3's threads do not always join cleanly; everything is on disk, so leave without destructors.
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
