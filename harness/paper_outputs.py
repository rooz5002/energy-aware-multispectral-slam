#!/usr/bin/env python3
"""Tables, analysis and figures for the paper from one main batch (DECISIONS v5, D19 and D25).

    python -m harness.paper_outputs --batch runs/batch_<stamp> [--sensitivity runs/sensitivity_<stamp>] [--out results/v5]
    python -m harness.paper_outputs --from-tables results/v5 [--out <dir>]

The first form reads the run directories. The second needs no run directories: it rebuilds every table and
figure from the per-run files the first form wrote (runs.csv, tracking_losses.csv, sensitivity_runs.csv,
batch_meta.json and the curves in figure_data/).

Reads only what the runs wrote; nothing is filtered, smoothed or corrected, except that the power curve of
figure (b) is a 1 s moving average (stated on its axis label). Failed and crashed runs are counted.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from harness.sequence import REPO

ORDER = ["static_rgb", "static_thermal", "static_fusion", "adaptive/tight", "adaptive/generous"]
LABEL = {"static_rgb": "Static RGB", "static_thermal": "Static thermal", "static_fusion": "Static fusion",
         "adaptive/tight": "Adaptive (tight budget)", "adaptive/generous": "Adaptive (generous budget)"}
# Okabe-Ito, colour-blind safe
COLOUR = {"static_rgb": "#0072B2", "static_thermal": "#D55E00", "static_fusion": "#000000",
          "adaptive/tight": "#009E73", "adaptive/generous": "#CC79A7"}
MARKER = {"static_rgb": "s", "static_thermal": "^", "static_fusion": "o", "adaptive/tight": "D", "adaptive/generous": "v"}
OFFSET = {"static_rgb": (-14, -9), "static_thermal": (4, 4), "static_fusion": (5, -8), "adaptive/tight": (4, 5), "adaptive/generous": (5, 4)}
JUMP_M = 0.30          # an online pose step above this is reported as a pose jump (robot moves 7 mm per frame)
TRACK_OK = 2           # ORB-SLAM3 tracking state OK


def md(t):
    """Markdown table without the tabulate dependency."""
    cell = lambda v: "" if v is None or (isinstance(v, float) and v != v) else str(v)
    lines = ["| " + " | ".join(map(str, t.columns)) + " |", "|" + "---|" * len(t.columns)]
    lines += ["| " + " | ".join(cell(v) for v in row) + " |" for row in t.itertuples(index=False)]
    return "\n".join(lines)


def rel(path):
    """Path relative to the repository, so the written tables carry no machine-specific prefix."""
    if not path:
        return path
    try:
        return str(Path(path).resolve().relative_to(REPO.resolve()))
    except ValueError:
        return str(path)


def cfg_label(r):
    return r["configuration"] + (f"/{r['budget']}" if r.get("budget") else "")


def load(batch):
    manifest = json.loads((batch / "manifest.json").read_text())
    rows, losses = [], []
    for r in manifest["runs"]:
        row = dict(sequence=r["sequence"], profile=r["profile"], cfg=cfg_label(r), repeat=r["repeat"],
                   completed=bool(r["completed"]), returncode=r.get("returncode"), run_dir=rel(r.get("run_dir")), log=rel(r.get("log")))
        if row["completed"]:
            d = Path(r["run_dir"])
            s = json.loads((d / "summary.json").read_text())
            for k in ("ate_rmse_m", "ate_online_rmse_m", "rpe_rmse_m", "tracked_fraction_main_map", "success",
                      "tracking_losses", "reinitialisations", "maps_visited", "energy_total_j",
                      "energy_total_j_compute_raw", "energy_total_j_compute_idle_subtracted",
                      "energy_total_j_sensors_model", "thermal_on_fraction", "mean_track_ms", "p95_track_ms",
                      "max_lag_s", "wall_s", "e0_joules", "recovery_frames", "frames_skipped", "frames_input",
                      "idle_power_w", "mean_compute_power_w"):
                row[k] = s.get(k)
            ltf = s.get("level_time_fraction") or {}
            for lv in ("0", "1", "2"):
                row[f"level{lv}_frac"] = float(ltf.get(lv, ltf.get(int(lv), 0.0)) or 0.0)
            log = Path(r["log"]) if r.get("log") else None
            row["loop_closures_reported"] = log.read_text(errors="replace").count("Loop detected") if log and log.exists() else None
            online = d / "trajectory_online_main_map_tum.txt"
            if online.exists():
                o = np.atleast_2d(np.loadtxt(online))
                step = np.linalg.norm(np.diff(o[:, 1:4], axis=0), axis=1) if len(o) > 1 else np.array([0.0])
                gap = np.diff(o[:, 0]) if len(o) > 1 else np.array([0.0])
                consecutive = step[gap < 0.1] if len(step) else step    # ignore steps across lost stretches
                row["max_pose_step_m"] = float(consecutive.max()) if len(consecutive) else 0.0
                row["pose_jumps"] = int((consecutive > JUMP_M).sum())
            f = pd.read_csv(d / "frames.csv")
            bad = (f.tracking_state != TRACK_OK) & (f.skipped == 0)
            starts = np.flatnonzero(bad & ~bad.shift(1, fill_value=False))
            for i in starts:
                j = i
                while j < len(f) and bad.iloc[j]:
                    j += 1
                fr = f.iloc[i]
                losses.append(dict(sequence=row["sequence"], profile=row["profile"], cfg=row["cfg"], repeat=row["repeat"],
                                   t_s=float(fr.t), lost_frames=int(j - i), severity=float(fr.severity),
                                   level=int(fr.level) if fr.level == fr.level else None,
                                   u_thermal=int(fr.u_thermal) if fr.u_thermal == fr.u_thermal else None,
                                   n_features_total=fr.get("n_features_total"), depth_valid_fraction=float(fr.depth_valid_fraction),
                                   map_before=int(f.map_id.iloc[max(i - 1, 0)]), map_after=int(f.map_id.iloc[min(j, len(f) - 1)])))
        rows.append(row)
    return manifest, pd.DataFrame(rows), pd.DataFrame(losses)


def pm(x, scale=1.0, digits=3):
    x = x.dropna()
    if len(x) == 0:
        return "failed"
    return f"{scale * x.mean():.{digits}f} ± {scale * x.std(ddof=0):.{digits}f}"


def main_table(df):
    out = []
    scopes = [("pooled", df)] + [(s, df[df.sequence == s]) for s in sorted(df.sequence.unique())]
    for scope, d in scopes:
        for profile in sorted(d.profile.unique()):
            ref = d[(d.profile == profile) & (d.cfg == "static_fusion")]
            ref_e, ref_idle = ref.energy_total_j_compute_raw.mean(), ref.energy_total_j_compute_idle_subtracted.mean()
            for cfg in ORDER:
                g = d[(d.profile == profile) & (d.cfg == cfg)]
                if g.empty:
                    continue
                ok = g[g.success == True]  # noqa: E712
                out.append({
                    "scope": scope, "profile": profile, "configuration": LABEL[cfg], "runs": len(g),
                    "crashed": int((~g.completed).sum()), "success k/n": f"{len(ok)}/{len(g)}",
                    "ATE RMSE (m), successful runs": pm(ok.ate_rmse_m),
                    "tracked (%)": round(100 * g.tracked_fraction_main_map.mean(), 1),
                    "tracking losses per run": round(g.tracking_losses.mean(), 2),
                    "pose jumps > 0.3 m per run": round(g.pose_jumps.mean(), 2),
                    "compute energy (J)": pm(g.energy_total_j_compute_raw, digits=0),
                    "compute energy (% of static fusion)": round(100 * g.energy_total_j_compute_raw.mean() / ref_e, 1) if ref_e == ref_e else None,
                    "idle-subtracted compute energy (J)": pm(g.energy_total_j_compute_idle_subtracted, digits=0),
                    "idle-subtracted (% of static fusion)": round(100 * g.energy_total_j_compute_idle_subtracted.mean() / ref_idle, 1) if ref_idle == ref_idle else None,
                    "sensor energy (J)": round(g.energy_total_j_sensors_model.mean(), 0),
                    "total energy (J)": pm(g.energy_total_j, digits=0),
                    "thermal on (%)": round(100 * g.thermal_on_fraction.mean(), 1),
                    "time at L0/L1/L2 (%)": "/".join(f"{100 * g[c].mean():.0f}" for c in ("level0_frac", "level1_frac", "level2_frac")),
                })
    return pd.DataFrame(out)


def timing_table(df):
    out = []
    for cfg in ORDER:
        g = df[(df.cfg == cfg) & df.completed]
        if g.empty:
            continue
        out.append({"configuration": LABEL[cfg], "runs": len(g), "mean tracking time per frame (ms)": round(g.mean_track_ms.mean(), 1),
                    "95th percentile, mean over runs (ms)": round(g.p95_track_ms.mean(), 1),
                    "95th percentile, worst run (ms)": round(g.p95_track_ms.max(), 1),
                    "largest lag behind 30 Hz (s), worst run": round(g.max_lag_s.max(), 3),
                    "runs with lag > 1 frame (33 ms)": int((g.max_lag_s > 1 / 30).sum())})
    return pd.DataFrame(out)


def loop_table(df):
    g = df[df.completed].groupby(["sequence", "profile", "cfg"])
    t = g.agg(runs=("repeat", "size"), loop_closures_reported=("loop_closures_reported", "sum"),
              runs_with_pose_jump=("pose_jumps", lambda x: int((x > 0).sum())), max_pose_step_m=("max_pose_step_m", "max")).reset_index()
    return t


def params_table(budgets):
    p = yaml.safe_load((REPO / "config" / "frozen.yaml").read_text())
    sens = yaml.safe_load((REPO / "config" / "sensors.yaml").read_text())
    q, s = p["quality"], p["scheduler"]
    rows = [("alpha, beta, gamma (Eq. 1)", f"{q['alpha']}, {q['beta']}, {q['gamma']}", "fixed"),
            ("tau_Q (normalised quality)", s["tau_Q"], "tuned (stage 1)"),
            ("tau_E1, tau_E2 (fraction of E0)", f"{s['tau_E1']}, {s['tau_E2']}", "tuned (stage 2)"),
            ("hysteresis band", s["hysteresis"], "fixed"), ("minimum dwell (s)", s["min_dwell_s"], "fixed"),
            ("thermal-priority weight", s["thermal_priority_weight"], "fixed"),
            ("minimum features per modality", s["n_min_per_modality"], "fixed"),
            ("thermal-off rule", s["thermal_off_rule"], "tuned (stage 2)"),
            ("recovery hold (s)", s["recovery_hold_s"], "fixed")]
    for i, lv in enumerate(s["levels"]):
        rows.append((f"Level {i} ({lv['name']})",
                     f"{lv['n_features']} features; min. keyframe interval {lv['min_keyframe_interval']} frames; local BA every "
                     f"{lv.get('local_ba_every', 1)} keyframe(s); loop closure {lv['loop_closure']}; every "
                     f"{lv.get('frame_stride', 1)} frame(s) processed", "Level 0 fixed, Levels 1-2 tuned (stage 2)"))
    rows += [("RGB-D sensor power (W)", sens["rgbd"]["power_w"], "D435, always on"),
             ("thermal sensor power (W)", sens["thermal"]["power_w"], "Boson 320-class, times u_TH"),
             ("E0, tight budget", "1.00 x median measured energy of static fusion on the same sequence and profile", "rule"),
             ("E0, generous budget", f"{budgets.get('generous')} x the same median (= 1.05 / (1 - tau_E1))", "rule"),
             ("tuned on", p["frozen"]["tuned_on"], f"setting {p['frozen']['selected_setting']}, constraints met: {p['frozen']['constraints_met']}")]
    return pd.DataFrame(rows, columns=["parameter", "value", "note"])


def sensitivity(sens_dir, df, out):
    m = json.loads((sens_dir / "manifest.json").read_text())
    rows = []
    for r in m["runs"]:
        row = dict(sequence=r["sequence"], parameter=r["parameter"], delta=r["delta"], repeat=r["repeat"], completed=bool(r["completed"]))
        if row["completed"]:
            s = json.loads((Path(r["run_dir"]) / "summary.json").read_text())
            row.update(ate=s.get("ate_rmse_m"), tracked=s.get("tracked_fraction_main_map"), success=s.get("success"),
                       energy=s.get("energy_total_j_compute_raw"), losses=s.get("tracking_losses"))
        rows.append(row)
    sv = pd.DataFrame(rows)
    sv.to_csv(out / "sensitivity_runs.csv", index=False)
    return sensitivity_summary(sv, m["profile"], df, out)


def sensitivity_summary(sv, profile, df, out):
    seqs = sorted(sv.sequence.unique())
    base = df[df.sequence.isin(seqs) & (df.profile == profile) & (df.cfg == "adaptive/tight")]
    ref = df[df.sequence.isin(seqs) & (df.profile == profile) & (df.cfg == "static_fusion")]
    b = dict(ate=base[base.success == True].ate_rmse_m.mean(), tracked=base.tracked_fraction_main_map.mean(),  # noqa: E712
             energy=base.energy_total_j_compute_raw.mean())
    rf = dict(ate=ref[ref.success == True].ate_rmse_m.mean(), energy=ref.energy_total_j_compute_raw.mean())  # noqa: E712
    out_rows = [dict(parameter="(frozen, main batch)", delta=0.0, runs=len(base), successful=int((base.success == True).sum()),  # noqa: E712
                     ate_m=b["ate"], tracked=b["tracked"], compute_energy_j=b["energy"], d_ate_pct=0.0, d_tracked_points=0.0,
                     d_energy_pct=0.0, energy_below_static_fusion=b["energy"] < rf["energy"],
                     ate_below_static_fusion=b["ate"] < rf["ate"] if b["ate"] == b["ate"] else None)]
    for (p, d), g in sv.groupby(["parameter", "delta"], sort=False):
        ok = g[g.success == True]  # noqa: E712
        ate, trk, en = ok.ate.mean() if len(ok) else float("nan"), g.tracked.mean(), g.energy.mean()
        out_rows.append(dict(parameter=p, delta=d, runs=len(g), successful=len(ok), ate_m=ate, tracked=trk, compute_energy_j=en,
                             d_ate_pct=100 * (ate / b["ate"] - 1) if b["ate"] == b["ate"] else float("nan"),
                             d_tracked_points=100 * (trk - b["tracked"]), d_energy_pct=100 * (en / b["energy"] - 1),
                             energy_below_static_fusion=en < rf["energy"],
                             ate_below_static_fusion=(ate < rf["ate"]) if ate == ate else None))
    t = pd.DataFrame(out_rows)
    t.to_csv(out / "sensitivity_summary.csv", index=False)
    (out / "sensitivity_summary.md").write_text(
        f"Sensitivity, adaptive (tight budget), {', '.join(seqs)}, {profile}; static fusion reference: ATE {rf['ate']:.4f} m, "
        f"compute energy {rf['energy']:.0f} J.\n\n" + md(t.round(4)) + "\n")
    return t


# ----------------------------------------------------------------------------------------------- figures
def window(f):
    on = f[f.severity > 0]
    return (float(on.t.iloc[0]), float(on.t.iloc[-1])) if len(on) else None


FIGURE_COLUMNS = ["t", "skipped", "severity", "Qn_rgb", "Qn_thermal", "omega_rgb", "omega_thermal", "e_cons_w"]


def figure_frames(run_dir, out, src, zone_sequence=None):
    """Per-frame curves of one run for the figures. From the run directory when it exists (a compact copy is then
    written to <out>/figure_data/), otherwise from the compact copy in <src>/figure_data/."""
    name = Path(run_dir).name
    full = REPO / run_dir / "frames.csv"
    if not full.exists():
        return pd.read_csv(src / "figure_data" / f"{name}.csv.gz")
    f = pd.read_csv(full)[FIGURE_COLUMNS].copy()
    if zone_sequence:
        gt = np.loadtxt(REPO / "data" / "sequences" / zone_sequence / "groundtruth.txt")
        f["in_crossover_zone"] = (np.interp(f.t, gt[:, 0], gt[:, 1]) > 1.0).astype(int)   # zone of the world: x > 1 m
    (out / "figure_data").mkdir(exist_ok=True)
    f.to_csv(out / "figure_data" / f"{name}.csv.gz", index=False)
    return f


def pick(df, sequence, profile, cfg, repeat=0):
    g = df[(df.sequence == sequence) & (df.profile == profile) & (df.cfg == cfg) & df.completed].sort_values("repeat")
    g = g[g["repeat"] == repeat] if (g["repeat"] == repeat).any() else g
    return (Path(g.iloc[0]["run_dir"]), int(g.iloc[0]["repeat"])) if len(g) else (None, None)


def save(fig, out, name):
    fig.savefig(out / f"{name}.png", dpi=600, bbox_inches="tight")
    fig.savefig(out / f"{name}.pdf", bbox_inches="tight")


def figures(df, out, src, seq_a="office_crossover_01", seq_b="office_loop_01", profile="fog_severe"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
                         "font.family": "serif", "axes.linewidth": 0.6, "lines.linewidth": 1.0})
    tau_q = yaml.safe_load((REPO / "config" / "frozen.yaml").read_text())["scheduler"]["tau_Q"]
    notes = {}

    # (a) normalised quality and weights over time, crossover sequence
    d, rep = pick(df, seq_a, profile, "adaptive/tight")
    if d is not None:
        f = figure_frames(d, out, src, zone_sequence=seq_a)
        f = f[f.skipped == 0]
        in_zone = f.in_crossover_zone.to_numpy() > 0           # thermal-crossover zone of the world: x > 1 m
        fig, ax = plt.subplots(2, 1, figsize=(3.5, 3.3), sharex=True)
        w = window(f)
        for a in ax:
            if w:
                a.axvspan(*w, color="0.88", lw=0, label="degradation active")
            edges = np.flatnonzero(np.diff(in_zone.astype(int)))
            tt = f.t.to_numpy()
            segs = np.concatenate([[0], edges + 1, [len(tt) - 1]])
            first = True
            for i0, i1 in zip(segs[:-1], segs[1:]):
                if in_zone[i0]:
                    a.axvspan(tt[i0], tt[i1], ymin=0.0, ymax=0.04, color="#E69F00", lw=0, label="thermal-crossover zone" if first else None)
                    first = False
        ax[0].plot(f.t, f.Qn_rgb, color=COLOUR["static_rgb"], label="RGB")
        ax[0].plot(f.t, f.Qn_thermal, color=COLOUR["static_thermal"], label="thermal")
        ax[0].axhline(tau_q, color="k", ls="--", lw=0.7, label=r"$\tau_Q$")
        ax[0].set_ylabel(r"normalised quality $\tilde{Q}_m$")
        ax[1].plot(f.t, f.omega_rgb, color=COLOUR["static_rgb"], label="RGB")
        ax[1].plot(f.t, f.omega_thermal, color=COLOUR["static_thermal"], label="thermal")
        ax[1].set_ylabel(r"weight $\omega_m$")
        ax[1].set_xlabel("time (s)")
        ax[1].set_ylim(-0.05, 1.05)
        ax[0].legend(ncol=3, fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, 1.32), columnspacing=1.0, handlelength=1.6)
        save(fig, out, "fig_a_quality_weights")
        plt.close(fig)
        notes["fig_a"] = f"{seq_a}, {profile}, adaptive (tight budget), repeat {rep}: {d.name}. Gaps in the thermal curves: thermal sensor off (u_TH = 0)."

    # (b) power and cumulative energy over time, all configurations
    fig, ax = plt.subplots(2, 1, figsize=(3.5, 4.0), sharex=True)
    used, w = [], None
    for cfg in ORDER:
        d, rep = pick(df, seq_b, profile, cfg)
        if d is None:
            continue
        f = figure_frames(d, out, src)
        w = w or window(f)
        dt = f.t.diff().fillna(f.t.diff().median())
        ax[0].plot(f.t, f.e_cons_w.rolling(30, min_periods=1, center=True).mean(), color=COLOUR[cfg], label=LABEL[cfg], lw=0.8)
        ax[1].plot(f.t, (f.e_cons_w * dt).cumsum() / 1000.0, color=COLOUR[cfg], label=LABEL[cfg])
        used.append(f"{LABEL[cfg]}: repeat {rep}, {d.name}")
    for a in ax:
        if w:
            a.axvspan(*w, color="0.88", lw=0, zorder=0)
    ax[0].set_ylabel("power (W, 1 s mean)")
    ax[1].set_ylabel("cumulative energy (kJ)")
    ax[1].set_xlabel("time (s)")
    ax[0].legend(ncol=2, fontsize=6, loc="upper center", bbox_to_anchor=(0.5, 1.36), columnspacing=1.0, handlelength=1.6)
    save(fig, out, "fig_b_power_energy")
    plt.close(fig)
    notes["fig_b"] = f"{seq_b}, {profile}. Power = measured CPU package power + modelled sensor power. Shaded: degradation active. " + "; ".join(used)

    # (c) ATE against energy per configuration, pooled over sequences, one panel per profile
    profiles = sorted(df.profile.unique())
    fig, ax = plt.subplots(1, len(profiles), figsize=(7.16, 2.5), sharey=True)
    ax = np.atleast_1d(ax)
    for a, profile_c in zip(ax, profiles):
        for cfg in ORDER:
            g = df[(df.profile == profile_c) & (df.cfg == cfg) & df.completed]
            ok = g[g.success == True]  # noqa: E712
            if g.empty:
                continue
            e, es = g.energy_total_j.mean() / 1000, g.energy_total_j.std(ddof=0) / 1000
            if len(ok):
                a.errorbar(e, ok.ate_rmse_m.mean(), xerr=es, yerr=ok.ate_rmse_m.std(ddof=0), color=COLOUR[cfg], marker=MARKER[cfg],
                           ms=4.5, capsize=2, lw=0.8, label=LABEL[cfg], ls="none")
                a.annotate(f"{len(ok)}/{len(g)}", (e, ok.ate_rmse_m.mean()), textcoords="offset points",
                           xytext=OFFSET[cfg], fontsize=6, color=COLOUR[cfg])
            else:
                a.axvline(e, color=COLOUR[cfg], ls=":", lw=0.9, label=LABEL[cfg] + " (no successful run)")
        a.set_xlabel("total energy (kJ)")
        a.text(0.5, 1.02, f"({'abc'[list(profiles).index(profile_c)]}) " + profile_c.replace("_", " "), transform=a.transAxes,
               va="bottom", ha="center", fontsize=7.5)
        a.set_yscale("log")
    ax[0].set_ylabel("ATE RMSE (m), successful runs")
    handles, labels = [], []
    for a in ax:
        for h, l in zip(*a.get_legend_handles_labels()):
            if l not in labels:
                handles.append(h)
                labels.append(l)
    fig.legend(handles, labels, ncol=3, fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, 1.2))
    save(fig, out, "fig_c_ate_energy")
    plt.close(fig)
    notes["fig_c"] = ("Pooled over the sequences of the batch. Markers: mean over runs; error bars: one standard deviation "
                      "(energy over all runs, ATE over successful runs); labels: successful runs / runs. Logarithmic ATE axis.")
    (out / "figure_notes.json").write_text(json.dumps(notes, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", type=Path, default=None)
    ap.add_argument("--sensitivity", type=Path, default=None)
    ap.add_argument("--from-tables", type=Path, default=None,
                    help="rebuild tables and figures from the per-run files of an earlier output directory (no run directories needed)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--no-figures", action="store_true")
    args = ap.parse_args()
    if (args.batch is None) == (args.from_tables is None):
        ap.error("give either --batch or --from-tables")
    src = args.from_tables
    out = args.out or src or REPO / "results" / "v5"
    out.mkdir(parents=True, exist_ok=True)
    if src:
        df, losses = pd.read_csv(src / "runs.csv"), pd.read_csv(src / "tracking_losses.csv")
        meta = json.loads((src / "batch_meta.json").read_text())
    else:
        manifest, df, losses = load(args.batch)
        df.to_csv(out / "runs.csv", index=False)
        losses.to_csv(out / "tracking_losses.csv", index=False)
        meta = dict(batch=rel(args.batch), start_utc=manifest.get("start_utc"), end_utc=manifest.get("end_utc"),
                    governor=manifest.get("governor"), on_ac_power=manifest.get("on_ac_power"),
                    idle_power_w=manifest.get("idle_power_w"), budgets=manifest.get("budgets"),
                    e0_joules={k: v["median_j"] for k, v in manifest.get("e0_joules", {}).items()}, runs=len(df),
                    completed=int(df.completed.sum()), crashed=int((~df.completed).sum()))
        if args.sensitivity:
            meta["sensitivity_profile"] = json.loads((args.sensitivity / "manifest.json").read_text())["profile"]
        (out / "batch_meta.json").write_text(json.dumps(meta, indent=1))
    tables = {"table_main": main_table(df), "table_timing": timing_table(df), "table_loop_closures": loop_table(df),
              "table_parameters": params_table(meta["budgets"])}
    for name, t in tables.items():
        t.to_csv(out / f"{name}.csv", index=False)
        (out / f"{name}.md").write_text(md(t) + "\n")
    if len(losses):
        ctx = losses.groupby(["cfg", "profile"]).agg(loss_events=("t_s", "size"), median_t_s=("t_s", "median"),
                                                     mean_severity=("severity", "mean"), at_level_0=("level", lambda x: int((x == 0).sum())),
                                                     at_level_1=("level", lambda x: int((x == 1).sum())), at_level_2=("level", lambda x: int((x == 2).sum())),
                                                     thermal_off=("u_thermal", lambda x: int((x == 0).sum())),
                                                     new_map=("map_after", lambda x: int((x != losses.loc[x.index, "map_before"]).sum()))).reset_index()
        ctx.to_csv(out / "table_loss_context.csv", index=False)
        (out / "table_loss_context.md").write_text(md(ctx) + "\n")
    if src and (src / "sensitivity_runs.csv").exists():
        sensitivity_summary(pd.read_csv(src / "sensitivity_runs.csv"), meta["sensitivity_profile"], df, out)
    elif args.sensitivity:
        sensitivity(args.sensitivity, df, out)
    if not args.no_figures:
        figures(df, out, src or out)
    print(json.dumps(meta, indent=1))
    print("written to", out)


if __name__ == "__main__":
    main()
