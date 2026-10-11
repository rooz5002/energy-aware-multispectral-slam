#!/usr/bin/env python3
"""Sensitivity sweep (guide section 9, DECISIONS D8).

    python -m harness.run_sensitivity --batch runs/batch_<stamp> [--profile fog_severe] [--reps 3]

On the adaptive configuration (tight budget), each of alpha, beta, gamma, tau_Q, tau_E1, tau_E2 and the
thermal-priority weight is varied one at a time by -20 %, -10 %, +10 %, +20 % around the frozen value
(alpha, beta, gamma are renormalised to sum to 1), on every sequence of the given main batch with one
degradation profile. E0 for each sequence is the one the main batch used (median static-fusion energy),
so runs are comparable with the batch's adaptive runs, which serve as the baseline.
"""
import argparse
import copy
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

from harness.run_matrix import gazebo_running, measure_idle
from harness.sequence import REPO

DELTAS = [-0.20, -0.10, 0.10, 0.20]
PARAMS = ["alpha", "beta", "gamma", "tau_Q", "tau_E1", "tau_E2", "thermal_priority_weight"]


def varied(base, name, delta):
    p = copy.deepcopy(base)
    if name in ("alpha", "beta", "gamma"):
        q = p["quality"]
        q[name] *= 1 + delta
        total = q["alpha"] + q["beta"] + q["gamma"]
        for k in ("alpha", "beta", "gamma"):
            q[k] /= total
    else:
        p["scheduler"][name] *= 1 + delta
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", type=Path, required=True, help="main batch whose E0 values and sequences are used")
    ap.add_argument("--profile", default="fog_severe")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--deltas", type=float, nargs="+", default=DELTAS)
    ap.add_argument("--sequences", nargs="+", default=None, help="default: every sequence of the main batch")
    ap.add_argument("--params", type=Path, default=REPO / "config" / "frozen.yaml")
    ap.add_argument("--dev", action="store_true")
    ap.add_argument("--max-frames", type=int, default=0, help="development only")
    args = ap.parse_args()
    if gazebo_running():
        raise SystemExit("Gazebo is running: stop it first")
    gov = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read_text().strip()
    if gov != "performance" and not args.dev:
        raise SystemExit(f"CPU governor is '{gov}' (DECISIONS D13); pass --dev for a development run")
    main_manifest = json.loads((args.batch / "manifest.json").read_text())
    e0 = {k.split("/")[0]: v["median_j"] for k, v in main_manifest["e0_joules"].items() if k.endswith("/" + args.profile)}
    if args.sequences:
        e0 = {s: e0[s] for s in args.sequences}
    if not e0:
        raise SystemExit(f"the batch has no static-fusion energy for profile {args.profile}")
    base = yaml.safe_load(args.params.read_text())

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = REPO / "runs" / f"sensitivity_{stamp}"
    (out / "params").mkdir(parents=True)
    (out / "logs").mkdir()
    idle_w = measure_idle(60.0 if not args.max_frames else 5.0)
    manifest = dict(start_utc=datetime.now(timezone.utc).isoformat(), main_batch=str(args.batch), profile=args.profile,
                    params=str(args.params), governor=gov, idle_power_w=idle_w, e0_joules=e0, deltas=args.deltas, runs=[])
    settings = []
    for name in PARAMS:
        for d in args.deltas:
            f = out / "params" / f"{name}_{d:+.2f}.yaml"
            f.write_text(yaml.safe_dump(varied(base, name, d), sort_keys=False))
            settings.append((name, d, f))
    jobs = [(r, s, st) for r in range(args.reps) for s in sorted(e0) for st in settings]
    for k, (rep, seq, (name, d, f)) in enumerate(jobs, 1):
        cmd = [sys.executable, "-m", "harness.run_one", "--sequence", seq, "--profile", args.profile, "--config", "adaptive",
               "--rep", str(rep), "--params", str(f), "--out-root", str(out), "--e0", f"{e0[seq]:.3f}", "--budget", "tight",
               "--idle-power-w", f"{idle_w:.4f}"]
        if args.max_frames:
            cmd += ["--max-frames", str(args.max_frames)]
        log = out / "logs" / f"{seq}_{name}_{d:+.2f}_{rep}.log"
        with open(log, "w") as fh:
            rc = subprocess.run(cmd, cwd=REPO, stdout=fh, stderr=subprocess.STDOUT).returncode
        run_dir = next((l.split("run directory:", 1)[1].strip() for l in log.read_text().splitlines()
                        if l.startswith("run directory:")), None)
        manifest["runs"].append(dict(sequence=seq, parameter=name, delta=d, repeat=rep, returncode=rc,
                                     completed=rc == 0 and run_dir is not None, run_dir=run_dir))
        (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
        print(f"[{k}/{len(jobs)}] {seq} {name} {d:+.0%} rep {rep}: {'ok' if rc == 0 else 'FAILED'}", flush=True)
        time.sleep(2.0)
    manifest["end_utc"] = datetime.now(timezone.utc).isoformat()
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print("sensitivity sweep done ->", out)


if __name__ == "__main__":
    main()
