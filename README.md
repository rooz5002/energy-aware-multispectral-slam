# Energy-aware bio-inspired multispectral SLAM: simulation, evaluation code and results

Code, configuration and results for the paper *Energy-Aware Bio-Inspired Multispectral SLAM for
Resource-Constrained Robots in Visually Degraded Environments*.

The repository contains the Gazebo simulation (world generator, robot model, routes), the adaptive controller
(perception quality, sensory weighting, energy-aware scheduling), the patch that integrates it into ORB-SLAM3,
the replay and evaluation harness, the frozen parameters with the complete tuning log, and every per-run result
behind the tables and figures of the paper.

- `results/SUMMARY.md` states the findings in plain numbers, including the unfavourable ones.
- `docs/IMPLEMENTATION_NOTES.md` documents what was built and every deviation from the specification.

## What is here

| Path | Content |
|---|---|
| `controller/` | Quality model (Eq. 1), weights (Eq. 2), scheduler, energy account, RAPL reader. No ROS or Gazebo dependency. |
| `degradation/` | Fog and low-light models for RGB, depth degradation, thermal sensor noise model (seeded). |
| `sim/` | World and texture generator, robot model builder, route scripts, recording script. |
| `orbslam3_patch/` | `orbslam3_ms.patch` against the pinned ORB-SLAM3 commit, `ORBSLAM3_CHANGES.md`, build script, Python binding. |
| `harness/` | Sequence export, replay of one run, run matrix, sensitivity sweep, evaluation, tables and figures. |
| `tuning/` | Tuning procedure, its full log (`log.csv`, 180 runs) and summary. |
| `config/` | `frozen.yaml` (parameters used for all evaluation runs), `sensors.yaml`, `profiles.yaml`, `calibration.yaml`, `configurations.yaml`. |
| `results/v5/` | Per-run results (`runs.csv`, `tracking_losses.csv`, `sensitivity_runs.csv`), tables, figures, figure data. |
| `tests/` | Unit tests of the controller and the degradation models; an integration test through ORB-SLAM3. |
| `docs/` | Design decisions (`DECISIONS*.md`) and `IMPLEMENTATION_NOTES.md`. `GAZEBO_SIM_GUIDE.md` is the original specification. |

## Software versions used

| Component | Version |
|---|---|
| Operating system | Ubuntu 26.04 LTS, Linux 7.0 |
| ROS 2 | Lyrical (`ros-lyrical-desktop` 0.13.0, `ros-lyrical-ros-gz` 3.0.9, `rosbag2-storage-mcap` 0.33.3) |
| Gazebo | gz-sim 10.4.0, headless, ogre2 |
| ORB-SLAM3 | commit `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (v1.0) + `orbslam3_patch/orbslam3_ms.patch`, RGB-D mode |
| TurtleBot 3 model | `ROBOTIS-GIT/turtlebot3_simulations` tag 2.3.7 (commit `9be186fb03d84ed4f293e5c0db71d8c05bbc91f3`) |
| System libraries | OpenCV 4.10.0, Eigen 3.4.0, pybind11 3.0.1 |
| Python | 3.12 virtual environment for the harness (`requirements.lock.txt`: numpy 2.5.3, opencv-python-headless 5.0.0.93, pandas 3.0.6, evo 1.37.1, matplotlib 3.11.2); system Python 3.14 for ROS |
| Processor | AMD Ryzen AI 9 HX 370; energy from the Linux powercap (RAPL) package counter |

## Reproducing the results

There are three levels. Level 1 needs only this repository.

### 1. Tables and figures from the per-run results (minutes, no ROS, no data download)

```bash
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.lock.txt
make paper-figures      # python -m harness.paper_outputs --from-tables results/v5
make test               # 81 unit tests
```

`make paper-figures` rebuilds, in `results/v5/`:

| Paper | File |
|---|---|
| Table II (parameters, operating levels) | `table_parameters.md` / `.csv` (from `config/frozen.yaml`, `config/sensors.yaml`) |
| Table III (results) | `table_main.md` / `.csv` (pooled and per sequence) |
| Fig. 3 (quality and weights over time) | `fig_a_quality_weights.pdf` / `.png` |
| Fig. 4 (power and cumulative energy) | `fig_b_power_energy.pdf` / `.png` |
| Fig. 5 (error against energy) | `fig_c_ate_energy.pdf` / `.png` |
| Sensitivity (Sec. V-D) | `sensitivity_summary.md` / `.csv` |
| Processing time (Sec. V-E) | `table_timing.md` / `.csv` |
| Tracking losses, loop closures | `table_loss_context.md`, `table_loop_closures.md` |

The inputs are `runs.csv` (one row per run, 90 runs), `tracking_losses.csv` (every loss event),
`sensitivity_runs.csv` (28 runs), `batch_meta.json` and `figure_data/` (per-frame curves of the six runs shown
in Figs. 3 and 4; the run names are in `figure_notes.json`). The rebuilt tables match the committed ones except for
the 16th significant digit of some floating-point values (`git diff` shows this).

### 2. Re-running the evaluation on the recorded sequences (about 9 hours)

Download the exported sequences (see "Data") into `data/sequences/`, then:

```bash
sudo apt install libopencv-dev libeigen3-dev pybind11-dev cmake g++
make orbslam3            # fetches ORB-SLAM3 at the pinned commit, applies the patch, builds it and the binding
make integration-test    # 4 configurations end to end on the tuning sequence, about 2 minutes

# measurement conditions (the runner refuses to start without them unless --dev is given)
sudo chmod a+r /sys/class/powercap/intel-rapl:*/energy_uj /sys/class/powercap/intel-rapl:*/*/energy_uj
sudo cpupower frequency-set -g performance      # mains power, other applications closed

# main matrix: 2 sequences x 3 conditions x 5 configurations x 3 repeats = 90 runs
.venv/bin/python -m harness.run_matrix --sequences office_loop_01 office_crossover_01 \
    --profiles clear fog_severe lowlight --configs static_rgb static_thermal static_fusion adaptive \
    --reps 3 --params config/frozen.yaml

# sensitivity: 7 parameters at -20 % and +20 %, 2 repeats = 28 runs
.venv/bin/python -m harness.run_sensitivity --batch runs/batch_<stamp> --profile fog_severe \
    --sequences office_loop_01 --deltas -0.2 0.2 --reps 2

# tables and figures from the new run directories
.venv/bin/python -m harness.paper_outputs --batch runs/batch_<stamp> --sensitivity runs/sensitivity_<stamp> \
    --out results/rerun
```

If ROS is sourced in the shell, prefix Python commands with `env -u PYTHONPATH` (the Makefile does this).
Energy and timing depend on the machine. Trajectories are not bit-identical between runs either: ORB-SLAM3 is
multi-threaded, which is why every cell has repeats and the tables give the spread.

The tuning that produced `config/frozen.yaml` is `make tune` (180 runs on `office_loop_tune`, about 7 hours);
its log is `tuning/log.csv`. Tuning did not meet every pre-set constraint; see `docs/IMPLEMENTATION_NOTES.md`.

### 3. Recording the sequences from scratch (Gazebo)

```bash
git clone --branch 2.3.7 https://github.com/ROBOTIS-GIT/turtlebot3_simulations third_party/turtlebot3_simulations
make worlds                         # robot model, textures, worlds (seeded, deterministic)
make record SEQ=office_loop_tune    # also office_loop_01, office_crossover_01
make calibrate && make calibrate-q && make calibrate-profiles
```

Recording needs ROS 2 Lyrical with `ros_gz` and a GPU for headless rendering. The route driver is repeatable to
about 0.2 s, not bit-identical, so a new recording differs slightly from the published one.

## Data

The recordings are too large for this repository and are archived separately:

- **Zenodo: DOI to be added.** Exported frame datasets (RGB, depth, thermal, time stamps, camera parameters,
  ground truth) of `office_loop_tune`, `office_loop_01` and `office_crossover_01`, and the run directories of the
  main batch and the sensitivity sweep (per-frame logs, trajectories, provenance).

The thermal scene was compared with real thermal imagery from the PST900 dataset (Shivakumar et al., "PST900:
RGB-Thermal Calibration, Dataset and Segmentation Network", ICRA 2020). PST900 is not redistributed here; only the
image statistics computed from it are included (`results/image_stats/pst900.yaml`).

## Notes on provenance

- Parameters were tuned only on `office_loop_tune` and frozen before any evaluation run. No run was excluded,
  repeated or edited; failed runs are counted in every table.
- This repository starts with a new history. Commit hashes quoted in `config/frozen.yaml`, in run provenance and
  in `docs/IMPLEMENTATION_NOTES.md` refer to the development history kept by the authors.
- In `tuning/log.csv` and the batch logs under `results/`, the absolute path prefix of the development machine was
  replaced by a relative path. Nothing else in those files was changed.
- Earlier batches recorded with repeated wall textures produced false loop closures and are superseded; they are
  described in `docs/IMPLEMENTATION_NOTES.md` and `results/main_matrix_stdout.log`.

## Licence

GNU General Public License v3.0 (see `LICENSE`). ORB-SLAM3 is GPLv3, and `orbslam3_patch/orbslam3_ms.patch` is a
derivative of it. ORB-SLAM3 itself is not included; the build script fetches it. The TurtleBot 3 simulation models
(Apache License 2.0) are likewise fetched, not included.

## Citation

To be added on publication.
