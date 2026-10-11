# Decisions — answers to open questions

These decisions **override `GAZEBO_SIM_GUIDE.md` where they differ**. Later versions override earlier ones. Record any further change here with a date.

- **v1** (2026-10-08): D1–D5.
- **v2** (2026-10-08): answers to the OPEN items in `IMPLEMENTATION_NOTES.md`. Amends D1, D2, D4, D5; adds D6–D13.

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

## D3. Depth in smoke — degrade it (unchanged; not yet implemented)

The notes say depth is still clean. Implement before M7:

- Fog/smoke profiles: same severity schedule and seed as RGB. Invalid-pixel dropout probability `p = 0.6 · severity · (z / z_max)`; additive noise `σ_z = 0.01 m + 0.03 · severity · z²` on remaining pixels.
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

## D5. Energy-saving exploration (amended in v2)

v1 applies, with these changes:

- **Level 2 minimum total feature budget = 600** (replaces "as low as ~300"). ORB-SLAM3 needs more than 500 keypoints to re-initialise an RGB-D map.
- The tuning grid may also include: the D9 thermal-off rule on/off and its dwell time; the processing-rate reduction lever (if used, record it and tell the author: it changes the method text).
- Tuning objective unchanged: minimise measured compute energy subject to ATE ≤ 1.10 × static fusion's ATE and tracked fraction ≥ 95% on the tuning sequence (with the D11 profiles).
- Log every tried setting and result in `tuning/log.csv`; write the selected settings to `config/frozen.yaml` with the commit hash; no changes after evaluation starts.
- Report energy three ways: total measured, idle-subtracted, and the sensor term separately. Report whatever saving is measured.

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
- Two budgets: **k = 1.0 ("tight")**, where the whole mission at full settings only just fits, so the levels engage; and **k = 1.3 ("generous")**, where throttling should rarely be needed.
- Run static configurations first in each batch so E₀ is known before the adaptive runs. Report both budgets.
- E_avail includes the sensor-power term from D1.

---

## D13. Measurement conditions for evaluation runs (new)

The development measurements were taken with the `powersave` governor, a desktop and a VirtualBox VM running. For all M7 tuning and evaluation runs:

- CPU governor `performance` (one sudo command per boot; log it). Laptop on AC power, same power profile, VM and other heavy applications closed.
- Measure idle power for 60 s before each batch.
- **Interleave the run order** across configurations (for example A, B, C, D, A, B, …, or randomised with a logged seed), so slow drifts in temperature or background load do not favour one configuration.
- Log CPU temperature with every run if available.
