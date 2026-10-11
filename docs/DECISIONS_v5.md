# Decisions v5 (2026-10-10) — reduced plan for submission within 15 hours

Overrides the **scope** of D16–D20 in `DECISIONS_v4.md`. All rules in v4 still apply (no re-tuning, frozen parameters, report everything, archive the old batches). Hard stop: all runs finished within **10 hours** from now, so the paper can be written and submitted.

## D22. Worlds

- Apply D16 (unique wall textures) to **office_loop_01 and office_crossover_01 only**. Corridor and hall are not regenerated now (future work).
- D17 check: static_rgb and static_fusion, clear, 1 repeat, on both sequences (4 runs). Stop and report if a false loop closure occurs.

## D23. Main matrix (replaces D18)

| Factor | Levels |
|---|---|
| Sequences | office_loop_01, office_crossover_01 |
| Profiles | clear, fog_severe, lowlight |
| Configurations | static_rgb, static_thermal, static_fusion, adaptive (tight), adaptive (generous) |
| Repeats | 3 |

90 runs. Interleaved order, D13 conditions; Firefox, WeChat and OrcaSlicer closed; idle measured before the batch. D19 analysis (loop closures, tracking-loss context, time at levels) on all runs.

## D24. After the matrix (replaces D20)

1. Sensitivity: adaptive (tight), office_loop_01, fog_severe; α, β, γ (renormalised), τ_Q, τ_E1, τ_E2, thermal-priority weight at −20 % and +20 %; 2 repeats. 28 runs.
2. **Skipped for this paper (state in SUMMARY as future work):** D14 sparse-thermal check, ablations, corridor and hall sequences, physical TurtleBot runs.

## D25. Outputs needed for the paper (in this order, as soon as each is ready)

1. `results/SUMMARY.md` with plain numbers, including unfavourable ones.
2. Table: per configuration × profile, pooled over the two sequences and per sequence: ATE mean ± std (successful runs), success k/n, tracked %, losses, compute energy (J) and % of static fusion, idle-subtracted energy, sensor energy.
3. Parameter table from `frozen.yaml` plus sensor powers and E₀ rule.
4. Figures (600 dpi PNG + PDF, no titles, legends, units, degradation window shaded, thresholds drawn): (a) normalised Q and ω over time on the crossover sequence, fog_severe; (b) power and cumulative energy over time, all configurations, one sequence; (c) ATE vs energy per configuration with error bars.
5. Sensitivity summary (largest change in ATE, tracked fraction and energy; whether ranking against static fusion changes).
6. Per-frame processing time per configuration (mean, 95th percentile) and lag behind 30 Hz, for the real-time statement.
