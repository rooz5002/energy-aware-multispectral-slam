#!/usr/bin/env python3
"""Run the experiment matrix: sequences x degradation profiles x configurations x repeats (guide section 9).

    python -m harness.run_matrix --sequences office_loop_01 --profiles clear fog_severe --reps 5

Every run is a separate process. Failed runs are recorded in the batch manifest, not dropped.
Before the batch, idle package power is measured for 60 s (guide section 7.4).

Order (DECISIONS D12, D13): phase A runs the configurations without the scheduler; the median measured
energy of static_fusion per (sequence, profile) then defines E0 = k * E_static_fusion for each budget k.
Phase B runs the scheduler configurations for every budget. Inside each phase the repeat is the outer
loop, so configurations are interleaved (A, B, C, A, B, C, ...).
"""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import statistics

from controller.energy import RaplReader
import yaml

from harness.sequence import REPO, load_yaml

MAIN_CONFIGS = ["static_rgb", "static_thermal", "static_fusion", "adaptive"]


def gazebo_running():
    # the server process is called gz-sim-main; "gz sim" is the launcher
    return subprocess.run(["pgrep", "-f", "gz[- ]sim"], capture_output=True).returncode == 0


def measure_idle(seconds):
    rapl = RaplReader()
    t0 = time.monotonic()
    time.sleep(seconds)
    return rapl.joules() / (time.monotonic() - t0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sequences", nargs="+", required=True)
    ap.add_argument("--profiles", nargs="+", required=True)
    ap.add_argument("--configs", nargs="+", default=MAIN_CONFIGS)
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--params", type=Path, default=REPO / "config" / "default.yaml")
    ap.add_argument("--out-root", type=Path, default=REPO / "runs")
    ap.add_argument("--idle-seconds", type=float, default=60.0)
    ap.add_argument("--budgets", nargs="+", choices=["tight", "generous"], default=None)
    ap.add_argument("--dev", action="store_true", help="development batch: do not enforce the D13 conditions")
    ap.add_argument("--no-energy", action="store_true", help="development only")
    ap.add_argument("--max-frames", type=int, default=0, help="development only")
    args = ap.parse_args()

    if gazebo_running():
        raise SystemExit("Gazebo is running: stop it before replaying (it would contaminate the energy measurement)")
    known = load_yaml("configurations.yaml")
    for c in args.configs:
        if c not in known:
            raise SystemExit(f"unknown configuration {c}")
    gov = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    governor = gov.read_text().strip() if gov.exists() else None
    ac = Path("/sys/class/power_supply/ACAD/online")
    on_ac = ac.read_text().strip() == "1" if ac.exists() else None
    problems = [msg for bad, msg in ((governor != "performance", f"CPU governor is '{governor}', not 'performance' "
                                      "(sudo cpupower frequency-set -g performance)"),
                                     (on_ac is False, "laptop is on battery")) if bad]
    if problems and not (args.dev or args.no_energy):
        raise SystemExit("measurement conditions (DECISIONS D13) not met: " + "; ".join(problems) +
                         "\nfix them, or pass --dev for a development batch")
    for msg in problems:
        print("WARNING:", msg)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    batch_dir = args.out_root / f"batch_{stamp}"
    (batch_dir / "logs").mkdir(parents=True)
    idle_w = None
    if not args.no_energy:
        print(f"measuring idle package power for {args.idle_seconds:.0f} s ...")
        idle_w = measure_idle(args.idle_seconds)
        print(f"idle power: {idle_w:.2f} W")
    params = yaml.safe_load(args.params.read_text())
    all_budgets = {"tight": 1.0, "generous": 1.05 / (1.0 - params["scheduler"]["tau_E1"])}
    wanted = args.budgets or params["energy"]["budgets"]
    budgets = {b: all_budgets[b] for b in wanted}
    manifest = dict(start_utc=datetime.now(timezone.utc).isoformat(), args={k: str(v) for k, v in vars(args).items()},
                    governor=governor, on_ac_power=on_ac, loadavg_start=os.getloadavg(), idle_power_w=idle_w,
                    idle_seconds=args.idle_seconds, budgets=budgets, e0_joules={}, runs=[])

    def run(k, total, rep, seq, prof, cfg, e0=None, budget=None):
        cmd = [sys.executable, "-m", "harness.run_one", "--sequence", seq, "--profile", prof, "--config", cfg,
               "--rep", str(rep), "--params", str(args.params), "--out-root", str(args.out_root)]
        if args.no_energy:
            cmd.append("--no-energy")
        if idle_w is not None:
            cmd += ["--idle-power-w", f"{idle_w:.4f}"]
        if e0 is not None:
            cmd += ["--e0", f"{e0:.3f}", "--budget", budget]
        if args.max_frames:
            cmd += ["--max-frames", str(args.max_frames)]
        tag = f"{seq}_{prof}_{cfg}{'_' + budget if budget else ''}_{rep}"
        log = batch_dir / "logs" / f"{tag}.log"
        t0 = time.monotonic()
        with open(log, "w") as f:
            rc = subprocess.run(cmd, cwd=REPO, stdout=f, stderr=subprocess.STDOUT).returncode
        run_dir = next((l.split("run directory:", 1)[1].strip() for l in log.read_text().splitlines()
                        if l.startswith("run directory:")), None)
        ok = rc == 0 and run_dir is not None
        manifest["runs"].append(dict(sequence=seq, profile=prof, configuration=cfg, budget=budget, repeat=rep,
                                     returncode=rc, completed=ok, run_dir=run_dir, log=str(log),
                                     wall_s=time.monotonic() - t0))
        (batch_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
        print(f"[{k}/{total}] {tag}: {'ok' if ok else 'FAILED rc=' + str(rc)}", flush=True)
        time.sleep(2.0)   # let the machine settle between runs
        return run_dir if ok else None

    cells = [(s, p) for s in args.sequences for p in args.profiles]
    plain = [c for c in args.configs if not known[c]["scheduler"]]
    scheduled = [c for c in args.configs if known[c]["scheduler"]]
    if scheduled and not args.no_energy and "static_fusion" not in plain:
        raise SystemExit("scheduler configurations need static_fusion in the same batch (E0 = k * E_static_fusion)")
    jobs_a = [(r, s, p, c) for r in range(args.reps) for s, p in cells for c in plain]
    jobs_b = [(r, s, p, c, b) for r in range(args.reps) for s, p in cells for c in scheduled
              for b in (budgets if not args.no_energy else {"none": None})]
    total = len(jobs_a) + len(jobs_b)

    fusion_energy = {}
    for k, (rep, seq, prof, cfg) in enumerate(jobs_a, 1):
        run_dir = run(k, total, rep, seq, prof, cfg)
        if run_dir and cfg == "static_fusion" and not args.no_energy:
            e = json.loads((Path(run_dir) / "summary.json").read_text())["energy_total_j"]
            fusion_energy.setdefault((seq, prof), []).append(e)
    for (seq, prof), values in fusion_energy.items():
        manifest["e0_joules"][f"{seq}/{prof}"] = dict(static_fusion_energy_j=values, median_j=statistics.median(values))
    for k, (rep, seq, prof, cfg, budget) in enumerate(jobs_b, len(jobs_a) + 1):
        if args.no_energy:
            run(k, total, rep, seq, prof, cfg)
            continue
        if (seq, prof) not in fusion_energy:
            manifest["runs"].append(dict(sequence=seq, profile=prof, configuration=cfg, budget=budget, repeat=rep,
                                         completed=False, returncode=None, run_dir=None,
                                         reason="no successful static_fusion run to define E0"))
            print(f"[{k}/{total}] {seq}_{prof}_{cfg}_{budget}_{rep}: SKIPPED (no E0)")
            continue
        run(k, total, rep, seq, prof, cfg, e0=budgets[budget] * statistics.median(fusion_energy[(seq, prof)]), budget=budget)

    manifest["end_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["loadavg_end"] = os.getloadavg()
    (batch_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
    failed = [r for r in manifest["runs"] if not r["completed"]]
    print(f"batch done: {len(manifest['runs']) - len(failed)} completed, {len(failed)} failed -> {batch_dir}")


if __name__ == "__main__":
    main()
