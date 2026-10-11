# Gazebo Simulation Guide — Energy-Aware Bio-Inspired Multispectral SLAM

This guide is for the developer building the simulation and evaluation software for the paper *Energy-Aware Bio-Inspired Multispectral SLAM for Embedded Robots in Visually Degraded Environments*. Read the whole file before writing code.

---

## 1. Purpose and context

The paper proposes a closed-loop controller that adapts SLAM to two signals: **perception quality** of the RGB and thermal streams, and **available energy**. The earlier results came from a script that generated quality curves, error and energy from hand-chosen formulas. Those results are being **replaced**. This software must produce results where:

- quality scores are **computed from real rendered frames**,
- localisation error is **measured** from ORB-SLAM3 output against **ground-truth poses**,
- energy is **measured** on the computer running SLAM (plus documented sensor-power constants),
- all four configurations run on **identical inputs**.

No thermal camera is available on the physical robot. The evaluation is therefore split:

| Claim | Evidence source |
|---|---|
| Quality assessment detects degradation | Gazebo **and** real TurtleBot 3 (RGB) |
| RGB–thermal weighting improves robustness | **Gazebo** (this guide) |
| Adaptive vs static fusion | **Gazebo** (this guide) |
| Energy-aware scheduling preserves accuracy | Gazebo and real TurtleBot 3 |
| Energy saving (measured compute energy) | Gazebo and real TurtleBot 3 |
| Parameter sensitivity | **Gazebo** (this guide) |

Design every module except the Gazebo world and recording so it can be **reused unchanged on the real TurtleBot 3** (RGB only).

### Reviewer requests this software must answer

1. A **static RGB–thermal fusion baseline** (adaptation off) alongside static RGB, static thermal and adaptive.
2. A clear, **measured** basis for energy (not arbitrary units only).
3. Concrete values for α, β, γ, τ_Q, τ_E1, τ_E2, and **quantitative** per-level settings (features, keyframe interval, BA / loop closure).
4. A **sensitivity analysis** of those parameters.
5. Publication-quality figures (≥600 dpi, labelled axes with units, legends, no titles baked in).
6. A clear boundary between simulated and real data.

---

## 2. Research-integrity rules (non-negotiable)

1. **Never synthesise, smooth or hand-edit results.** Every number in a table or figure must be traceable to a run log.
2. **Freeze parameters before final runs.** Tune α, β, γ, thresholds and level settings on a designated **tuning sequence** only, record the frozen values in `config/frozen.yaml` with the git commit hash, then run the evaluation sequences. Do not change parameters after seeing evaluation results.
3. **Report failures.** Tracking loss, reinitialisations and crashed runs are counted and reported, not dropped. If the adaptive configuration loses on some metric, report it.
4. **Same inputs for every configuration.** All configurations replay the same recorded sequences with the same seeded degradation.
5. **Log provenance.** Every run directory stores: config file, config hash, git commit, ORB-SLAM3 commit, machine info, start/end timestamps.
6. **The paper text must describe what was implemented.** If any design choice below is changed (for example, a fallback in §6.3), write it into `docs/IMPLEMENTATION_NOTES.md` so the paper can be updated to match.

---

## 3. Environment

Detect at startup and fail loudly with a clear message if something is missing.

- **ROS 2 + Gazebo:** prefer **ROS 2 Jazzy + Gazebo Harmonic** on Ubuntu 24.04, or **ROS 2 Humble + Gazebo Fortress** on Ubuntu 22.04, via `ros_gz`. **Do not use Gazebo Classic**: it has no built-in thermal camera. Check which TurtleBot 3 simulation packages are available for the installed distro before choosing.
- **Robot model:** TurtleBot 3 **Waffle** (or Waffle Pi) simulation model, extended with the sensors in §4.2.
- **SLAM:** ORB-SLAM3 (C++), built from source, with the small control hooks in §6. Pin the commit.
- **Python 3.10+** for the quality module, scheduler, degradation, energy logging and harness. Libraries: OpenCV, NumPy, PyYAML, pandas, Matplotlib, `evo`.
- **Energy:** Intel RAPL via `/sys/class/powercap/intel-rapl*` (or AMD equivalent via `amd_energy`/`rapl` if present). If neither is readable, stop and report. Do not silently fall back to a model.

Before starting, ask the user (once) to confirm: Ubuntu/ROS distro, CPU vendor, whether a GPU is used for rendering, and whether ORB-SLAM3 is already built somewhere.

---

## 4. Simulation world and sensors

### 4.1 Worlds

Create **three** worlds, each with a fixed robot route:

- `office_loop`: indoor rooms and a corridor, closed loop (start = end).
- `corridor_tunnel`: long, low-texture corridor (stress test for RGB).
- `industrial_hall`: larger space with machinery and clutter.

Use one additional short sequence in `office_loop` as the **tuning sequence** only.

### 4.2 Sensors on the robot

Mount a co-located sensor head:

- **RGB camera:** 640×480, 30 Hz, horizontal FOV ≈ 60°, modest Gaussian noise.
- **Depth camera:** same pose, intrinsics and resolution as the RGB camera (used for metric scale; see §6.1).
- **Thermal camera:** **same pose and intrinsics** as the RGB camera, resolution 320×240 (also test 160×120 later), 30 Hz, Gaussian noise. Co-location keeps RGB, depth and thermal pixels aligned, which §6.2 relies on.

Thermal camera in gz-sim: use the `thermal` sensor type with the thermal-sensor system plugin, and add the thermal system plugin with a `<temperature>` (or a heat-signature texture) to object visuals. Plugin names differ between Fortress (`ignition-gazebo-…`, `ignition::gazebo::systems::…`) and Harmonic (`gz-sim-…`, `gz::sim::systems::…`). **Verify syntax against the thermal camera example world shipped with the installed gz-sim** (search the install for `thermal_camera.sdf`) rather than trusting this sketch:

```xml
<sensor name="thermal_camera" type="thermal">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image><width>320</width><height>240</height><format>L16</format></image>
    <clip><near>0.1</near><far>50</far></clip>
  </camera>
  <update_rate>30</update_rate>
  <topic>thermal_camera</topic>
  <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
    <min_temp>253.15</min_temp><max_temp>373.15</max_temp><resolution>0.05</resolution>
  </plugin>
</sensor>
```

### 4.3 Thermal scene content (important)

Uniform-temperature objects produce **featureless thermal images**, and ORB will find nothing. Give the scene thermal structure:

- people models at ≈ 307–310 K, radiators and pipes at 320–340 K, machinery at 315–350 K, lamps, windows (cooler), walls and floor near ambient (≈ 293 K);
- use **heat-signature textures** (temperature varies across a surface) on large objects and some walls, so thermal images contain gradients and corners.

**Sanity check (must pass before M3):** in clear conditions, thermal frames should yield a meaningful number of ORB keypoints (target > 150 per frame at 320×240). Report the measured counts. If thermal tracking still fails, report it to the user; do not hide it.

### 4.4 Ground truth and routes

- Publish the robot's true pose from Gazebo through `ros_gz_bridge` (pose or ground-truth odometry) and record it.
- Drive each route with a **deterministic** waypoint script (Nav2 or a scripted `cmd_vel` controller with fixed speeds). The same route must be reproducible.
- Each sequence: 2–4 minutes, including at least one degradation window.

### 4.5 Recording

Record each sequence **once** to a rosbag (MCAP): RGB, depth, thermal (raw 16-bit), camera info, ground-truth pose, `/clock`. Store as `data/sequences/<world>_<id>/`. Export ground truth to TUM format.

**Why record first:** replaying bags (a) gives every configuration identical input, and (b) lets energy be measured **without Gazebo running**, so rendering does not contaminate the SLAM energy measurement.

---

## 5. Degradation (applied at replay time)

Degradation is applied in the pipeline, to the **RGB stream only**. Thermal stays clean, modelling thermal imaging's insensitivity to smoke. This is an explicit modelling assumption; record it in `IMPLEMENTATION_NOTES.md`.

- Types: **fog/smoke** (blend toward grey + contrast loss + slight blur), **low light** (gamma darkening + shot/read noise), **motion blur** (directional kernel).
- A degradation profile is a time schedule: clear → ramp in → hold → ramp out → clear, with severity in [0, 1].
- Seeded RNG: the same profile and seed must produce **bit-identical** degraded frames across configurations (unit-tested).
- Define at least three profiles: `fog_moderate`, `fog_severe`, `lowlight`. Record onset/offset times so plots can shade the window.

Thermal preprocessing: convert 16-bit radiometric frames to 8-bit with a **fixed** temperature window per sequence (not per-frame auto-scaling, which creates artificial contrast changes), then optional CLAHE with fixed parameters.

---

## 6. SLAM integration

### 6.1 Mode

Use ORB-SLAM3 in **RGB-D mode**, with depth from the co-located depth camera, so trajectories are metric and comparable across configurations. Use SE(3) alignment for ATE. (Monocular is optional later; it would need Sim(3) alignment and is fragile in low-texture sim scenes.) Record this choice for the paper.

### 6.2 Multispectral front-end: weighted feature pooling (preferred)

This implements the paper's statement that weights regulate how many features come from each modality.

- Total feature budget per frame `n_total` comes from the current operating level (§7.3).
- Per-modality budget: `n_m = round(ω_m · n_total)` (with a small minimum, e.g. 50, while a modality is active).
- Extract ORB features from the RGB image and the thermal image separately with their budgets. Because the cameras are co-located with identical intrinsics, keypoints share pixel coordinates and the shared depth image gives their 3D positions.
- Pool them into one `Frame`. **Tag each keypoint with its modality** and only match descriptors within the same modality.
- Expose per-modality tracked-inlier counts from the tracker, for `N_m(t)` in §7.1.

This needs edits to ORB-SLAM3's `Frame` construction and `Tracking` (two extractors, modality tags, matching restricted by tag). Keep the changes minimal, isolated behind a compile-time option, and documented in `docs/ORBSLAM3_CHANGES.md` with a diff.

### 6.3 Fallback (only if 6.2 is not achievable)

Run one tracker on the modality with the higher weight and switch on a weight crossover (with hysteresis). This is weaker: it is modality *switching*, not weighting. If used, write it into `IMPLEMENTATION_NOTES.md` so the paper text is changed to "switching". Ask the user before adopting the fallback.

### 6.4 Runtime control hooks

Add a small control interface that ORB-SLAM3 reads every frame (a shared struct set from Python via a thin C++/pybind11 binding, or a ROS 2 parameter/topic):

- `n_features_total` (and per-modality split from §6.2),
- `min_keyframe_interval` (frames),
- `local_ba_enabled`,
- `loop_closure_mode` ∈ {full, reduced, off},
- `thermal_enabled` / `rgb_enabled` (sensor activation `u_m`).

Locate the relevant code (ORB extractor feature count, the keyframe-decision function in `Tracking`, `LocalMapping`, `LoopClosing`) and change behaviour through these switches only. Log every value actually applied, every frame.

---

## 7. Controller (shared with the real robot)

### 7.1 Perception quality (Eq. 1)

`Q_m(t) = α·N_m(t) + β·H_m(t) − γ·B_m(t)`, each indicator normalised to [0, 1]:

- `N_m` = tracked inliers of modality m in the previous frame / per-modality budget (clip to 1). For the very first frames, use detected keypoints until tracking starts; log which was used.
- `H_m` = Shannon entropy of the 8-bit image / 8.
- `B_m` = `1 − min(1, varLaplacian / V_ref_m)`, so **larger B means blurrier**. `V_ref_m` is calibrated per modality on clear frames of the tuning sequence and frozen.

Initial values (to be tuned, then frozen): α = 0.4, β = 0.4, γ = 0.2.

### 7.2 Weights (Eq. 2)

`ω_m(t) = Q_m(t) / Σ_k Q_k(t)`, over active modalities. Guard against division by zero.

### 7.3 Scheduler (must match the paper's Fig. 2 exactly)

```
if Q_RGB < τ_Q and thermal quality adequate: increase thermal weight / thermal features
if   E_avail ≥ τ_E1:  Level 0 (Normal)
elif E_avail ≥ τ_E2:  Level 1 (Energy saving)
else:                 Level 2 (Tracking priority)
```

Apply level changes **progressively** with hysteresis (e.g. a 2–3% band on each threshold and a minimum dwell time), and log every transition.

Initial level settings (tune on the tuning sequence, then freeze):

| Setting | Level 0 | Level 1 | Level 2 |
|---|---|---|---|
| ORB features per frame (total) | 1200 | 800 | 500 |
| Min. keyframe interval (frames) | ORB-SLAM3 default | fps/2 | fps |
| Local BA | on | on | on (local only) |
| Loop closure | full | reduced | off |

Initial thresholds: τ_Q = 0.45, τ_E1 = 0.6·E₀, τ_E2 = 0.3·E₀.

### 7.4 Energy model and E_avail (Eq. 3)

`E_cons(t) = Σ_m P_m·u_m(t) + C(t)`:

- `C(t)` = **measured** compute power from RAPL (package energy delta / Δt).
- `P_m` = sensor power constants from datasheets (RGB camera, thermal camera, depth camera), in watts. Store them with the datasheet source in `config/sensors.yaml`.
- `E_avail(t) = E₀ − ∫ E_cons dt`.

`E₀` is the **energy budget allocated to perception for the mission**, in joules. A real battery would not drain in a few minutes, so choose E₀ such that the levels are exercised during the sequence, and report the chosen value and reason. Support two modes, logged in every run: `energy_source: measured` (default) and `energy_source: model` (for comparison only).

Idle correction: before each batch, measure idle package power for 60 s with nothing running and log it. Report energy both raw and idle-subtracted.

---

## 8. Configurations

Every run uses one configuration file. Required configurations:

| Name | Modalities | Weights | Scheduler |
|---|---|---|---|
| `static_rgb` | RGB only | ω_RGB = 1 | off (Level 0 always) |
| `static_thermal` | thermal only | ω_TH = 1 | off |
| `static_fusion` | both | fixed 0.5 / 0.5 | off |
| `adaptive` | both | Eq. 2 | on |
| `ablation_weighting_only` | both | Eq. 2 | off |
| `ablation_scheduling_only` | both | fixed 0.5 / 0.5 | on |

The two ablations are lower priority; do them after the four main configurations work.

---

## 9. Experiment harness

- One command runs the full matrix: sequences × degradation profiles × configurations × **5 repeats**.
- During replay, Gazebo must **not** be running. Pin CPU frequency governor to `performance` if allowed, close other heavy processes, and log CPU load.
- Per run, write to `runs/<timestamp>_<seq>_<profile>_<config>_<rep>/`:
  - `frames.csv`: per-frame t, N/H/B and Q per modality, ω per modality, level, applied control values, keyframe flag, tracking state, measured compute power, modelled sensor power, E_cons, E_avail;
  - `trajectory_tum.txt` (estimated) and a copy or link of ground truth;
  - `summary.json`: ATE RMSE (SE(3)), RPE, total energy (raw and idle-subtracted), mean power, tracking-loss count, reinitialisations, run time, frames processed;
  - `provenance.json` (see §2.5).
- Use `evo_ape` / `evo_rpe` (or the `evo` Python API) for errors.

### Sensitivity sweep

On the **adaptive** configuration, vary α, β, γ, τ_Q, τ_E1, τ_E2 one at a time by −20%, −10%, +10%, +20% (renormalise α, β, γ so they still sum to 1), on all evaluation sequences with one degradation profile, 3 repeats. Output a CSV and a summary of the maximum change in RMSE and energy, and whether the ranking of configurations changes.

---

## 10. Outputs for the paper

Generate all tables and figures from `runs/` with one command (`make paper-figures`):

- **Table:** per configuration, mean ± std of ATE RMSE, total energy (J), mean power (W), tracking losses; per sequence and pooled.
- **Parameter table:** frozen α, β, γ, τ_Q, τ_E1, τ_E2, E₀, sensor powers, level settings.
- **Figures** (PNG at 600 dpi **and** PDF vector; widths 3.5 in single column or 7.16 in double column; fonts ≥ 8 pt at final size; **no titles inside images**; legends; axis labels with units; degradation window shaded; thresholds drawn as lines):
  1. Q_RGB, Q_TH and ω over time for one representative run;
  2. power and cumulative energy over time, all four configurations;
  3. localisation error over time, all four configurations;
  4. error vs energy summary (one point per configuration with error bars);
  5. sensitivity plot.
- Write `results/SUMMARY.md` stating, in plain numbers, what the results show, including anything unfavourable.

---

## 11. Testing

- **Unit tests:** weights sum to 1; normalisation bounds; blur sign (sharper image → lower B); entropy of a constant image = 0; scheduler transitions match §7.3 at, above and below each threshold, including hysteresis; degradation determinism (same seed → identical bytes).
- **Integration test:** a 20 s bag runs end-to-end through all four configurations and produces all output files.
- **Sanity checks (report numbers):**
  - `static_rgb` on a clear sequence gives a small ATE (a few cm in these worlds); investigate if not.
  - `adaptive` with the scheduler forced to Level 0 and weights fixed at 0.5 reproduces `static_fusion` within run-to-run noise.
  - Thermal keypoint counts in clear conditions (see §4.3).
  - RAPL readings are non-zero, monotonic and plausible (watts in the expected range for the CPU).

---

## 12. Milestones (in order)

| # | Deliverable | Done when |
|---|---|---|
| M1 | Worlds + robot with RGB, depth, thermal | Thermal frames show structure; keypoint check (§4.3) passes |
| M2 | Deterministic routes + recorded bags + TUM ground truth | 3 evaluation sequences + 1 tuning sequence recorded |
| M3 | Degradation + quality module + weights | Unit tests pass; Q curves respond to degradation on recorded data |
| M4 | ORB-SLAM3 hooks + weighted feature pooling (§6.2) | `static_fusion` and `adaptive` run on a bag; control values logged |
| M5 | Energy measurement + E_avail + scheduler | Levels change during a run; energy logged in joules |
| M6 | Harness + evo + summaries | Full matrix for one sequence completes |
| M7 | Tuning → freeze → full matrix → sensitivity | `frozen.yaml` committed before evaluation runs |
| M8 | Paper tables and figures | `make paper-figures` regenerates everything |

If time is short, M1–M6 with the four main configurations on two sequences is the minimum useful result. Tell the user early if any milestone is blocked (especially M1 thermal features or M4 ORB-SLAM3 changes) rather than working around it silently.

---

## 13. Repository layout

```
repo/
  config/                   # experiments, sensors.yaml, frozen.yaml
  sim/                      # worlds, models, heat textures, launch files, route scripts
  orbslam3_patch/           # patch + ORBSLAM3_CHANGES.md
  controller/               # quality, weights, scheduler, energy (shared with real robot)
  degradation/
  harness/                  # replay, run matrix, evo, aggregation
  analysis/                 # figures and tables
  tests/
  data/sequences/           # recorded bags + ground truth (not in git)
  runs/                     # run outputs (not in git)
  docs/IMPLEMENTATION_NOTES.md
```

---

## 14. What to report back to the user

At each milestone, report briefly: what works, measured sanity numbers, any deviation from this guide and why, and anything that changes what the paper can claim.
