# Decisions — answers to open questions

These decisions **override `GAZEBO_SIM_GUIDE.md` where they differ**. Later versions override earlier ones. Record any further change here with a date.

- **v1** (2026-10-08): D1–D5.
- **v2** (2026-10-08): answers to the OPEN items in `IMPLEMENTATION_NOTES.md`. Amends D1, D2, D4, D5; adds D6–D13.
- **v3** (2026-10-08): D5 written out in full (tuning grid); D3 sets z_max; D12 redefines the generous budget; adds D14 (sparse-thermal check) and D15 (pre-tuning checklist).

**All v2 items must be implemented before M7 tuning starts.** Nothing below may be decided by looking at evaluation-sequence results.

---

## D1. Sensor models and power values (amended in v2)

| Sim sensor | Modelled on | Power used (`P_m`) | Range for sensitivity check | Source |
|---|---|---|---|---|
| RGB + depth (one unit) | Intel RealSense **D435** | **2.0 W** | 1.5–3.5 W | Intel's multi-camera guidance (~2 W per D400 camera): https://support.realsenseai.com/hc/en-us/community/posts/7428579234067/comments/7430741133459 ; Clearpath D435 spec, 3.5 W maximum: https://docs.clearpathrobotics.com/docs/robots/accessories/sensors/cameras/realsense_d435 |
| Thermal | Teledyne FLIR **Boson 320**-class LWIR core | **0.6 W** | 0.5–1.0 W | Boson product page, "starting at 600 mW": https://www.oem.flir.com/products/boson/ ; datasheet "as low as 500 mW" |

- **v2 change: keep the thermal camera at 320×240.** Do not re-record at 320×256. The 4:3 aspect ratio matches the co-located RGB/depth geometry, and the recorded sequences stay valid. The paper says "a 320×240 LWIR camera, with power taken from a Boson 320-class core".
- RGB and depth are one physical unit and stay on in every configuration (RGB-D mode). Only the thermal sensor term changes between configurations and over time (see D9).
- Fill `config/sensors.yaml` with these values and URLs.

---

## D2. Thermal realism (amended in v2)

Noise alone will not fix this: with the 8-bit conversion window, 50 mK NETD noise is well under one grey level, while the rendered thermal images are sharp, piecewise-constant decals. Thermal-only SLAM is currently *more* accurate than RGB-only in clear conditions, which would not be expected of a real low-resolution LWIR camera indoors. Do all of the following, then **freeze the thermal scene before M7**:

1. **Calibrate the thermal scene against real thermal image statistics, not against SLAM results.**
   - Preferred: take a sample of frames (a few hundred) from a public indoor RGB–thermal dataset, for example ViViD++ indoor sequences. Compute, for both RGB and thermal frames: ORB keypoints per frame (same extractor settings), image entropy, and edge density (fraction of Canny edge pixels).
   - Adjust decal density and decal temperature contrast so that the simulated **thermal-to-RGB ratios** of these statistics, on clear frames of the tuning sequence, roughly match the real dataset's ratios.
   - Fallback if no real data can be obtained within a day: reduce decal density and contrast until clear-condition thermal ORB keypoints are at most ~50% of RGB's on the tuning sequence, and state this rule in the paper as an assumption.
   - **Do not use ATE or tracking results to tune the scene.** Only image statistics.
2. **Optics blur:** Gaussian blur σ ≈ 0.7 px **at native 320×240, before** the bilinear resize to 640×480, so decal edges are no longer unrealistically sharp.
3. **Sensor noise at replay:** keep the v1 noise (NETD 0.05 K temporal, 0.02 K fixed pattern, seeded). It is small but costs nothing and is realistic.
4. **Thermal-crossover sequence is mandatory:** record `office_crossover`, where part of the route has decals and objects within one quantisation step (1.285 K) of ambient, so thermal contrast collapses while RGB stays good. This is the test that the weighting moves back to RGB.
5. State the remaining idealisations as a limitation: temperature quantised to 1.285 K steps, piecewise-constant thermal structure, no emissivity, reflection or atmospheric modelling.

---

## D3. Depth in smoke — degrade it (z_max set in v3)

Implemented. Parameters:

- Fog/smoke profiles: same severity schedule and seed as RGB. Invalid-pixel dropout probability `p = 0.6 · severity · min(1, z / z_max)`; additive noise `σ_z = 0.01 m + 0.03 · severity · z²` on remaining pixels.
- **v3: z_max = 3.0 m**, the D435's listed maximum range (Clearpath spec sheet, D1). Not 10 m: ORB-SLAM3 only uses points within 3.2 m (ThDepth 40), so with z_max = 10 m the points SLAM actually uses would see at most ~19% dropout and the depth degradation would barely act. Re-check the D11 retention values are unaffected (they are RGB-only) and log valid-depth fraction at the hold of each profile.
- Low light: depth clean. Motion blur: no change.
- Identical for all configurations. Log valid-depth fraction per frame.

---

## D4. Reporting runs that lose tracking (amended in v2)

Implement the main-map evaluation; do not use one SE(3) alignment over poses from different maps.

- **Log ORB-SLAM3's Atlas map ID for every frame** (online trajectory) and for every keyframe pose in the final trajectory.
- **Main map** = the map containing the most frames. **Primary ATE** = final trajectory restricted to the main map, SE(3) alignment. Also report the online-trajectory ATE on the main map.
- Frames outside the main map count as lost.
- **Successful run:** tracked fraction (main map) ≥ 90%. ATE = mean ± std over successful runs, always shown with success k/n. Failed runs still report tracked fraction and energy.
- If most runs of a cell fail, report "failed" for that cell.

---

## D5. Tuning procedure (written out in full in v3)

**Where:** only `office_loop_tune`, with the four D11 profiles (clear, fog_moderate, fog_severe, lowlight), **tight budget (k = 1.0)**. Evaluation sequences are never used for tuning.

**Fixed, not tuned:** α = 0.4, β = 0.4, γ = 0.2 (reported, and covered by the sensitivity sweep); thermal-priority weight 0.7 (D8); hysteresis band 0.02; minimum dwell 2 s; loop-closure mode per level (full / reduced / off); Level 0 settings (ORB-SLAM3 defaults, 1200 features).

**Objective (fixed now):** minimise the **sum over the four profiles of measured compute energy**, subject to, **on every profile**: ATE ≤ 1.10 × static fusion's ATE on that profile, and tracked fraction ≥ 95%. If no setting meets the constraints, choose the one with the fewest constraint violations, then lowest energy, and report that the constraints could not be met.

**Stage 1 — τ_Q (3 settings):** τ_Q ∈ {0.5, 0.6, 0.7} with the initial level settings. Choose by the objective. Freeze τ_Q.

**Stage 2 — energy levers (at most 30 settings):** sample without replacement (seeded, seed logged) from:

| Lever | Values |
|---|---|
| (τ_E1, τ_E2) as fractions of E₀ | (0.6, 0.3), (0.5, 0.25) |
| Level 1 features | 1000, 800, 600 |
| Level 2 features | 700, 600 (floor 600, see below) |
| Min. keyframe interval, Level 1 / Level 2 | (fps/2, fps), (fps, 2·fps) |
| Local BA at Levels 1–2 | every keyframe, every 2nd keyframe |
| Thermal-off rule (D9) | on, off |
| Processing rate at Level 2 | every frame, every 2nd frame |

- **Level 2 minimum total feature budget = 600** (ORB-SLAM3 needs > 500 keypoints to re-initialise an RGB-D map).
- If the selected setting uses "every 2nd frame", record it in `IMPLEMENTATION_NOTES.md` and tell the author: it changes the method text, which currently says the sensing rate is not throttled.

**Stage 3 — confirm:** re-run the best 3 settings from Stage 2 with 3 repeats each (interleaved, D13) and choose by the objective on the mean. One repeat is too noisy to decide between close settings.

**Freeze:** write the chosen values to `config/frozen.yaml` with the commit hash; log every tried setting and result in `tuning/log.csv`. No changes after evaluation starts.

**Reporting energy:** total measured, idle-subtracted, and the sensor term separately. Whatever the saving is, report the measured number. In `results/SUMMARY.md`, explain where the energy goes (feature extraction, tracking, BA, loop closure, idle), using per-thread CPU time if available.

---

## D6. Quality normalisation and τ_Q (new)

The measured clear-condition means (Q_RGB ≈ 0.45, Q_TH ≈ 0.23) show the raw scores of the two modalities are on different scales, so one τ_Q cannot serve both.

- Normalise each modality's quality by its clear-condition mean from the tuning sequence: `Q̃_m(t) = Q_m(t) / Q̄_m,clear` (stored in `config/calibration.yaml`).
- Use `Q̃_m` in Eq. 2 (weights) and in every τ_Q comparison.
- One threshold τ_Q on the normalised scale; initial value 0.6, then tuned in M7 and frozen.
- **PAPER:** Eq. 1 is followed by "scores are normalised by each modality's clear-condition mean, calibrated on a separate tuning sequence".
- Note: D2 changes the thermal scene, so calibrate D6 **after** D2 is frozen.

---

## D7. Blur indicator (new)

B currently reads noise as sharpness. Before M7:

- Compute the Laplacian variance **after a Gaussian pre-filter (σ = 1.0 px)**, on 8-bit images, for both modalities. Recalibrate `V_ref`.
- Add a regression check on the tuning sequence: at the hold of each profile, B_RGB must increase with severity (fog_moderate < fog_severe) and be higher in lowlight than in clear. Report the values.
- If B is still not monotonic, do not invent another indicator now. Report it, and let γ be tuned (it may go to a small value). The paper will report the tuned γ.
- **PAPER:** "variance of the Laplacian after Gaussian pre-filtering".

---

## D8. Thermal-priority weight (new)

Accepted as implemented: while `Q̃_RGB < τ_Q` and `Q̃_TH ≥ τ_Q`, `ω_TH = max(ω_TH, 0.7)`, with the 0.02 hysteresis band and minimum dwell.

- `thermal_priority_weight = 0.7` goes in the parameter table and in the sensitivity sweep (±20%).

---

## D9. Sensor activation u_TH(t) (new)

The method claims sensors can be deactivated, so implement one rule for thermal (RGB/depth stay on because the D435 provides depth):

- **Thermal off** when the scheduler is at Level 1 or 2 **and** `Q̃_RGB ≥ τ_Q + band` continuously for at least the minimum dwell time.
- **Thermal on** immediately when `Q̃_RGB < τ_Q`, or on tracking loss.
- While thermal is off: `u_TH = 0`, no thermal extraction, `Q_TH` logged as NaN, `ω_RGB = 1`.
- The rule can be switched on or off in config and is part of the D5 tuning grid. Whatever the tuning selects is frozen.
- **PAPER:** describe the rule in Section III-D.

---

## D10. Recovery override (new)

- On tracking loss, use the **Level 0 feature budget** until tracking has been OK for 2 s, regardless of the energy level, and force thermal on. Log every override with its duration and energy.
- **PAPER:** one sentence in Section III-D.

---

## D11. Degradation profiles defined by measured severity (new)

Profile names currently understate severity. Redefine each profile's peak severity on the **tuning sequence**, from RGB ORB keypoint retention at the hold (mean detected keypoints relative to clear):

| Profile | Target RGB keypoint retention at hold |
|---|---|
| `fog_moderate` | ≈ 50% |
| `fog_severe` | ≈ 10% |
| `lowlight` | ≈ 25% |

Record the resulting severity values and measured retention in `config/profiles.yaml`, then freeze. The paper reports the retention values so the reader knows what "moderate" and "severe" mean.

---

## D12. Energy budget E₀ (new)

E₀ is set relative to measured consumption, so it means the same thing on every sequence:

- `E₀ = k × E_static_fusion`, where `E_static_fusion` is the median measured energy (compute + sensors) of static fusion on the same sequence and profile.
- **Tight budget: k = 1.0.** The whole mission at static-fusion settings exactly uses E₀, so the levels must engage.
- **Generous budget (v3): k_gen = 1.05 / (1 − τ_E1)**, computed after τ_E1 is frozen. At this budget static fusion's full run ends just above τ_E1, so a configuration that spends no more than static fusion stays at Level 0 throughout. (k = 1.3 was not generous: with τ_E1 = 0.6 it entered Level 1 halfway.) This budget isolates the effect of the weighting from the energy levels. Note that the D9 thermal-off rule only acts at Levels 1–2, so it will rarely act under this budget; that is expected.
- Run static configurations first in each batch so E₀ is known before the adaptive runs. Report both budgets.
- E_avail includes the sensor-power term from D1.

---

## D13. Measurement conditions for evaluation runs (new)

The development measurements were taken with the `powersave` governor, a desktop and a VirtualBox VM running. For all M7 tuning and evaluation runs:

- CPU governor `performance` (one sudo command per boot; log it). Laptop on AC power, same power profile, VM and other heavy applications closed.
- Measure idle power for 60 s before each batch.
- **Interleave the run order** across configurations (for example A, B, C, D, A, B, …, or randomised with a logged seed), so slow drifts in temperature or background load do not favour one configuration.
- Log CPU temperature with every run if available.

---

## D14. Sparse-thermal robustness check (new in v3, run after the main matrix)

The frozen scene meets the keypoint rule (thermal/RGB = 0.49) but is still richer than real thermal imagery: PST900's ratios are 0.13 (keypoints), 0.65 (entropy), 0.05 (edge density), against 0.49, 0.97 and 1.24 in simulation. PST900 is subterranean rather than indoor, so its ratios are not a direct target, but the gap must be stated and tested.

- **Do not re-record or change the frozen scene.** Instead, define a **replay-time** thermal variant `thermal_sparse`: stronger optics blur and contrast compression of the thermal stream, set on the clear tuning sequence so the thermal/RGB keypoint ratio is ≈ 0.13 (image statistics only, as in D2).
- Run static thermal, static fusion and adaptive (tight budget) on **two** evaluation sequences × fog_severe and clear, 3 repeats.
- Report whether the main conclusions (ranking of configurations, energy saving) still hold with sparse thermal. Report the result either way.
- **PAPER:** the three simulated ratios next to PST900's in the limitations, plus one sentence on the sparse-thermal check.

---

## D15. Before M7 starts (checklist, v3)

1. **Commit now.** Add a `.gitignore` for `data/`, `runs/`, rosbags and exported frames; commit code, configs, `DECISIONS.md`, `IMPLEMENTATION_NOTES.md`; tag the commit `pre-tuning`. `frozen.yaml` later records the commit at which it is written.
2. **Re-run the integration test** on all configurations after the decision changes, and the unit tests. Report both.
3. Confirm all four evaluation sequences (office loop, crossover, corridor, hall) re-recorded and exported, and that the crossover's thermal Q̃_TH collapses in the intended segment while Q̃_RGB stays high. Report the numbers.
4. The author sets the measurement conditions (governor `performance`, VM closed, AC power). The runner already refuses otherwise.
