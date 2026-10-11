#!/usr/bin/env python3
"""Collect every run under runs/ into results/runs.csv and the per-configuration table.

    python -m harness.aggregate [--batch runs/batch_<stamp>]

Nothing is filtered out: failed and crashed runs are counted, development runs are flagged.
"""
import argparse
import json
from pathlib import Path

import pandas as pd

from harness.sequence import REPO

# energy three ways (D5): total measured (compute + sensors), idle-subtracted compute, sensor term
ENERGY = ["energy_total_j", "energy_total_j_compute_raw", "energy_total_j_compute_idle_subtracted",
          "energy_total_j_sensors_model", "mean_compute_power_w"]
ALWAYS = ENERGY + ["tracked_fraction_main_map", "tracking_losses", "reinitialisations", "thermal_on_fraction", "mean_track_ms"]
ACCURACY = ["ate_rmse_m", "ate_online_rmse_m", "rpe_rmse_m"]     # over SUCCESSFUL runs only (D4)
METRICS = ACCURACY + ALWAYS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=Path, default=REPO / "runs")
    ap.add_argument("--batch", type=Path, default=None, help="only the runs of this batch (plus its failed runs)")
    ap.add_argument("--out", type=Path, default=REPO / "results")
    args = ap.parse_args()
    rows, crashed = [], []
    if args.batch:
        manifest = json.loads((args.batch / "manifest.json").read_text())
        dirs = [Path(r["run_dir"]) for r in manifest["runs"] if r["completed"]]
        crashed = [r for r in manifest["runs"] if not r["completed"]]
    else:
        dirs = sorted(p.parent for p in args.runs.glob("*/summary.json"))
    for d in dirs:
        s = json.loads((d / "summary.json").read_text())
        s.pop("level_time_fraction", None)
        s["run_dir"] = d.name
        rows.append(s)
    df = pd.DataFrame(rows)
    df = df.reindex(columns=list(df.columns) + [m for m in METRICS if m not in df.columns])
    args.out.mkdir(exist_ok=True)
    df.to_csv(args.out / "runs.csv", index=False)

    df["budget"] = df["budget"].fillna("-") if "budget" in df else "-"
    df["success"] = df["success"].fillna(False).astype(bool) if "success" in df else False

    def table(group_cols):
        g = df.groupby(group_cols)
        t = g[ALWAYS].agg(["mean", "std"])
        t.columns = [f"{m}_{stat}" for m, stat in t.columns]
        acc = df[df.success].groupby(group_cols)[ACCURACY].agg(["mean", "std"])
        acc.columns = [f"{m}_{stat}" for m, stat in acc.columns]
        t = t.join(acc)
        t.insert(0, "runs", g.size())
        t.insert(1, "successful", g["success"].sum())
        # a cell where most runs fail is reported as failed (D4)
        t.insert(2, "cell", ["failed" if s * 2 <= n else "ok" for s, n in zip(t["successful"], t["runs"])])
        return t.reset_index()

    per_seq = table(["sequence", "profile", "configuration", "budget"])
    pooled = table(["profile", "configuration", "budget"])
    per_seq.to_csv(args.out / "table_per_sequence.csv", index=False)
    pooled.to_csv(args.out / "table_pooled.csv", index=False)
    if crashed:
        pd.DataFrame(crashed).to_csv(args.out / "crashed_runs.csv", index=False)
    show = ["sequence", "profile", "configuration", "budget", "runs", "successful", "cell", "ate_rmse_m_mean",
            "ate_rmse_m_std", "energy_total_j_mean", "energy_total_j_compute_idle_subtracted_mean",
            "energy_total_j_sensors_model_mean", "tracked_fraction_main_map_mean"]
    print(per_seq[show].round(4).to_string(index=False))
    print(f"\n{len(df)} runs ({int(df.development_run.sum())} development runs), {len(crashed)} crashed -> {args.out}")


if __name__ == "__main__":
    main()
