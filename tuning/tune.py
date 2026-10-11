#!/usr/bin/env python3
"""Parameter tuning on the tuning sequence only (DECISIONS v3, D5).

    python -m tuning.tune            # all stages, resumable; then writes config/frozen.yaml

Where: office_loop_tune, profiles clear / fog_moderate / fog_severe / lowlight, tight budget (k = 1.0).
Objective: minimise the sum over the four profiles of measured compute energy, subject to, on every
profile, ATE <= 1.10 x static fusion's ATE and tracked fraction >= 95 %. If nothing meets the constraints:
fewest violations, then lowest energy (and this is reported).
  reference  static fusion on each profile (E0 and the ATE reference)
  stage 1    tau_Q in {0.5, 0.6, 0.7} with the initial level settings
  stage 2    at most 30 settings sampled without replacement (seeded) from the lever grid
  stage 3    best 3 settings of stage 2, 3 repeats each, interleaved; chosen on the mean
Every run is appended to tuning/log.csv. Evaluation sequences are never touched.
"""
import argparse
import copy
import csv
import itertools
import json
import random
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

from harness.run_matrix import gazebo_running, measure_idle
from harness.sequence import REPO

SEQ = "office_loop_tune"
PROFILES = ["clear", "fog_moderate", "fog_severe", "lowlight"]
FPS = 30
TAU_Q = [0.5, 0.6, 0.7]
GRID = dict(
    tau_E=[(0.6, 0.3), (0.5, 0.25)],
    l1_features=[1000, 800, 600],
    l2_features=[700, 600],                       # floor 600: ORB-SLAM3 needs > 500 keypoints to re-initialise
    kf_interval=[(FPS // 2, FPS), (FPS, 2 * FPS)],
    ba_every=[1, 2],
    thermal_off=[True, False],
    l2_stride=[1, 2],
)
STAGE2_MAX, STAGE2_SEED = 30, 20261009
ATE_FACTOR, MIN_TRACKED = 1.10, 0.95
REF_REPEATS, STAGE3_REPEATS, STAGE3_TOP = 3, 3, 3
LOG = REPO / "tuning" / "log.csv"
FIELDS = ["utc", "stage", "setting", "tau_Q", "tau_E1", "tau_E2", "l1_features", "l2_features", "kf_interval_l1",
          "kf_interval_l2", "ba_every", "thermal_off", "l2_stride", "configuration", "profile", "repeat", "e0_joules",
          "compute_energy_j", "compute_energy_idle_subtracted_j", "sensor_energy_j", "total_energy_j", "ate_rmse_m",
          "tracked_fraction_main_map", "success", "level_time_fraction", "thermal_on_fraction", "completed", "run_dir"]


def initial_setting(base):
    s, lv = base["scheduler"], base["scheduler"]["levels"]
    return dict(tau_Q=s["tau_Q"], tau_E=(s["tau_E1"], s["tau_E2"]), l1_features=lv[1]["n_features"],
                l2_features=lv[2]["n_features"], kf_interval=(lv[1]["min_keyframe_interval"], lv[2]["min_keyframe_interval"]),
                ba_every=lv[1].get("local_ba_every", 1), thermal_off=s["thermal_off_rule"], l2_stride=lv[2].get("frame_stride", 1))


def to_params(base, st):
    p = copy.deepcopy(base)
    s = p["scheduler"]
    s["tau_Q"], (s["tau_E1"], s["tau_E2"]), s["thermal_off_rule"] = st["tau_Q"], st["tau_E"], st["thermal_off"]
    l1, l2 = s["levels"][1], s["levels"][2]
    l1["n_features"], l2["n_features"] = st["l1_features"], st["l2_features"]
    l1["min_keyframe_interval"], l2["min_keyframe_interval"] = st["kf_interval"]
    l1["local_ba_every"] = l2["local_ba_every"] = st["ba_every"]
    l2["frame_stride"] = st["l2_stride"]
    return p


def setting_id(st):
    return (f"q{st['tau_Q']}_e{st['tau_E'][0]}-{st['tau_E'][1]}_f{st['l1_features']}-{st['l2_features']}"
            f"_k{st['kf_interval'][0]}-{st['kf_interval'][1]}_ba{st['ba_every']}_off{int(st['thermal_off'])}_s{st['l2_stride']}")


class Tuner:
    def __init__(self, args):
        self.args = args
        self.base = yaml.safe_load((REPO / "config" / "default.yaml").read_text())
        self.rows = list(csv.DictReader(open(LOG))) if LOG.exists() else []
        self.idle_w = None
        (REPO / "tuning" / "params").mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------ running
    def done(self, stage, sid, config, profile, rep):
        return next((r for r in self.rows if (r["stage"], r["setting"], r["configuration"], r["profile"], r["repeat"])
                     == (stage, sid, config, profile, str(rep)) and r["completed"] == "True"), None)

    def run(self, stage, st, config, profile, rep, e0=None):
        sid = setting_id(st) if st else "static"
        if self.done(stage, sid, config, profile, rep):
            return
        if self.idle_w is None and not self.args.no_energy:
            print("measuring idle package power for 60 s ...", flush=True)
            self.idle_w = measure_idle(60.0 if not self.args.max_frames else 5.0)
        params_file = REPO / "config" / "default.yaml"
        if st:
            params_file = REPO / "tuning" / "params" / f"{sid}.yaml"
            params_file.write_text(yaml.safe_dump(to_params(self.base, st), sort_keys=False))
        cmd = [sys.executable, "-m", "harness.run_one", "--sequence", SEQ, "--profile", profile, "--config", config,
               "--rep", str(rep), "--params", str(params_file), "--out-root", str(REPO / "runs" / "tuning")]
        if self.args.no_energy:
            cmd.append("--no-energy")
        else:
            cmd += ["--idle-power-w", f"{self.idle_w:.4f}"]
            if e0 is not None:
                cmd += ["--e0", f"{e0:.3f}", "--budget", "tight"]
        if self.args.max_frames:
            cmd += ["--max-frames", str(self.args.max_frames)]
        out = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True)
        run_dir = next((l.split("run directory:", 1)[1].strip() for l in out.stdout.splitlines()
                        if l.startswith("run directory:")), None)
        ok = out.returncode == 0 and run_dir is not None
        s = json.loads((Path(run_dir) / "summary.json").read_text()) if ok else {}
        row = dict(utc=datetime.now(timezone.utc).isoformat(timespec="seconds"), stage=stage, setting=sid,
                   configuration=config, profile=profile, repeat=rep, e0_joules=e0, completed=ok, run_dir=run_dir,
                   compute_energy_j=s.get("energy_total_j_compute_raw"),
                   compute_energy_idle_subtracted_j=s.get("energy_total_j_compute_idle_subtracted"),
                   sensor_energy_j=s.get("energy_total_j_sensors_model"), total_energy_j=s.get("energy_total_j"),
                   ate_rmse_m=s.get("ate_rmse_m"), tracked_fraction_main_map=s.get("tracked_fraction_main_map"),
                   success=s.get("success"), level_time_fraction=json.dumps(s.get("level_time_fraction")),
                   thermal_on_fraction=s.get("thermal_on_fraction"))
        if st:
            row.update(tau_Q=st["tau_Q"], tau_E1=st["tau_E"][0], tau_E2=st["tau_E"][1], l1_features=st["l1_features"],
                       l2_features=st["l2_features"], kf_interval_l1=st["kf_interval"][0], kf_interval_l2=st["kf_interval"][1],
                       ba_every=st["ba_every"], thermal_off=st["thermal_off"], l2_stride=st["l2_stride"])
        new = not LOG.exists()
        with open(LOG, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)
        self.rows.append({k: ("" if v is None else str(v)) for k, v in row.items()})
        print(f"  [{stage}] {sid} {config} {profile} rep {rep}: " +
              (f"E {row['compute_energy_j']:.0f} J, ATE {row['ate_rmse_m']}, tracked {row['tracked_fraction_main_map']:.3f}"
               if ok and row["compute_energy_j"] == row["compute_energy_j"] and row["compute_energy_j"] is not None
               else ("ok" if ok else "FAILED")), flush=True)
        time.sleep(2.0)

    # ------------------------------------------------------------------ scoring
    def values(self, stage, sid, config, profile, key):
        out = []
        for r in self.rows:
            if (r["stage"], r["setting"], r["configuration"], r["profile"]) == (stage, sid, config, profile) and r["completed"] == "True":
                out.append(float(r[key]) if r[key] not in ("", "None", "nan") else None)
        return out

    def reference(self):
        ref = {}
        for p in PROFILES:
            e = [v for v in self.values("reference", "static", "static_fusion", p, "total_energy_j") if v is not None]
            a = [v for v in self.values("reference", "static", "static_fusion", p, "ate_rmse_m") if v is not None]
            c = [v for v in self.values("reference", "static", "static_fusion", p, "compute_energy_j") if v is not None]
            ref[p] = dict(e0=statistics.median(e) if e else None, ate=statistics.mean(a) if a else None,
                          compute_energy_j=statistics.mean(c) if c else None)
        return ref

    def score(self, stage, st, ref):
        """(violations, energy sum, detail). Missing or failed runs count as violations."""
        sid, violations, energy, detail = setting_id(st), 0, 0.0, {}
        for p in PROFILES:
            e = [v for v in self.values(stage, sid, "adaptive", p, "compute_energy_j") if v is not None]
            ate = self.values(stage, sid, "adaptive", p, "ate_rmse_m")
            trk = [v for v in self.values(stage, sid, "adaptive", p, "tracked_fraction_main_map") if v is not None]
            ok_ate = [v for v in ate if v is not None]
            m_ate = statistics.mean(ok_ate) if (ok_ate and len(ok_ate) == len(ate)) else None
            m_trk = statistics.mean(trk) if trk else 0.0
            v_ate = m_ate is None or ref[p]["ate"] is None or m_ate > ATE_FACTOR * ref[p]["ate"]
            v_trk = m_trk < MIN_TRACKED
            violations += int(v_ate) + int(v_trk)
            energy += statistics.mean(e) if e else float("inf")
            detail[p] = dict(energy_j=statistics.mean(e) if e else None, ate_m=m_ate, ate_limit_m=ATE_FACTOR * ref[p]["ate"]
                             if ref[p]["ate"] else None, tracked=m_trk, ate_violation=v_ate, tracked_violation=v_trk)
        return violations, energy, detail

    # ------------------------------------------------------------------ stages
    def main(self):
        a = self.args
        summary = dict(started_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"), sequence=SEQ, profiles=PROFILES,
                       objective="min sum of compute energy s.t. ATE <= 1.10 x static fusion and tracked >= 0.95 on every profile",
                       stage2_seed=STAGE2_SEED, grid={k: [list(v) if isinstance(v, tuple) else v for v in vs] for k, vs in GRID.items()})
        print("reference: static fusion")
        for rep in range(REF_REPEATS if not a.quick else 1):
            for p in PROFILES:
                self.run("reference", None, "static_fusion", p, rep)
        ref = self.reference()
        summary["reference"] = ref
        init = initial_setting(self.base)

        def evaluate(stage, settings, repeats):
            for rep in range(repeats):                    # repeat outer: settings interleaved
                for st in settings:
                    for p in PROFILES:
                        self.run(stage, st, "adaptive", p, rep, e0=ref[p]["e0"])
            scored = [(self.score(stage, st, ref), st) for st in settings]
            scored.sort(key=lambda x: (x[0][0], x[0][1]))
            return scored

        print("stage 1: tau_Q")
        s1 = evaluate("stage1", [dict(init, tau_Q=q) for q in TAU_Q], 1)
        tau_q = s1[0][1]["tau_Q"]
        summary["stage1"] = [dict(setting=setting_id(st), violations=v, energy_sum_j=e, detail=d) for (v, e, d), st in s1]

        print(f"stage 2: energy levers (tau_Q = {tau_q})")
        grid = [dict(zip(GRID, combo), tau_Q=tau_q) for combo in itertools.product(*GRID.values())]
        sample = random.Random(STAGE2_SEED).sample(grid, min(STAGE2_MAX if not a.quick else 2, len(grid)))
        s2 = evaluate("stage2", sample, 1)
        summary["stage2"] = [dict(setting=setting_id(st), violations=v, energy_sum_j=e, detail=d) for (v, e, d), st in s2]

        print("stage 3: confirm the best settings")
        s3 = evaluate("stage3", [st for _, st in s2[:STAGE3_TOP if not a.quick else 1]], STAGE3_REPEATS if not a.quick else 1)
        summary["stage3"] = [dict(setting=setting_id(st), violations=v, energy_sum_j=e, detail=d) for (v, e, d), st in s3]
        (violations, energy, detail), best = s3[0]
        ref_energy = sum(ref[p]["compute_energy_j"] for p in PROFILES) if all(ref[p]["compute_energy_j"] for p in PROFILES) else None
        summary["selected"] = dict(setting=setting_id(best), values={k: (list(v) if isinstance(v, tuple) else v) for k, v in best.items()},
                                   violations=violations, constraints_met=violations == 0, energy_sum_j=energy,
                                   static_fusion_energy_sum_j=ref_energy,
                                   saving_vs_static_fusion=(1 - energy / ref_energy) if ref_energy else None, detail=detail,
                                   uses_reduced_processing_rate=best["l2_stride"] > 1)
        summary["finished_utc"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
        (REPO / "tuning" / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
        print(json.dumps(summary["selected"], indent=1, default=str))

        if a.quick or a.no_energy or a.max_frames or a.dev:
            print("development tuning run: config/frozen.yaml NOT written")
            return
        frozen = to_params(self.base, best)
        frozen["status"] = "frozen"
        frozen["frozen"] = dict(utc=summary["finished_utc"], tuned_on=SEQ, selected_setting=setting_id(best),
                                written_at_commit=subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True,
                                                                 text=True).stdout.strip(),
                                constraints_met=violations == 0, tuning_log="tuning/log.csv", tuning_summary="tuning/summary.json")
        (REPO / "config" / "frozen.yaml").write_text(
            "# FROZEN parameters (DECISIONS D5). Do not edit: evaluation runs use this file.\n" + yaml.safe_dump(frozen, sort_keys=False))
        print("wrote config/frozen.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", action="store_true", help="do not enforce the D13 measurement conditions; frozen.yaml is not written")
    ap.add_argument("--quick", action="store_true", help="development: 1 reference repeat, 2 stage-2 settings, 1 confirmation")
    ap.add_argument("--no-energy", action="store_true", help="development only")
    ap.add_argument("--max-frames", type=int, default=0, help="development only")
    args = ap.parse_args()
    if gazebo_running():
        raise SystemExit("Gazebo is running: stop it first")
    gov = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read_text().strip()
    ac = Path("/sys/class/power_supply/ACAD/online")
    on_ac = ac.read_text().strip() == "1" if ac.exists() else True
    if (gov != "performance" or not on_ac) and not args.dev:
        raise SystemExit(f"measurement conditions (DECISIONS D13) not met: governor '{gov}', AC power {on_ac}. "
                         "Fix them, or pass --dev for a development run.")
    global LOG
    if args.dev or args.quick or args.no_energy or args.max_frames:
        LOG = REPO / "tuning" / "dev_log.csv"
    Tuner(args).main()


if __name__ == "__main__":
    main()
