# Results summary (DECISIONS v5 reduced plan), 2026-10-11

Everything here comes from `runs/batch_20261010T161828Z` (90 runs, 90 completed, none crashed) and
`runs/sensitivity_20261010T225415Z` (28 runs, all completed), with `config/frozen.yaml` at commit `2e97aca`.
No run was excluded, re-run or edited. Tables as CSV and Markdown, per-run data (`runs.csv`), every tracking-loss
event (`tracking_losses.csv`) and the figures are in `results/v5/`; regenerate with
`python -m harness.paper_outputs --batch <batch> --sensitivity <sweep>`.

## What was run

- Sequences `office_loop_01` and `office_crossover_01` (re-recorded with unique wall textures, D22), profiles clear,
  fog_severe, lowlight; static RGB, static thermal, static fusion, adaptive at the tight and the generous budget;
  3 repeats. 184 s per sequence, replayed in real time at 30 Hz.
- Conditions: governor `performance`, AC power, VirtualBox, Firefox, WeChat and OrcaSlicer closed, external cooling
  fan. Idle package power before the batch: 7.1 W (it was 18.4 W before the tuning batch, with
  those applications open and no fan). Idle-subtracted figures depend on this value.
- E0 (median static-fusion energy, compute + sensors): office_loop_01/clear: 5079 J; office_loop_01/fog_severe: 5331 J; office_loop_01/lowlight: 5308 J; office_crossover_01/clear: 5239 J; office_crossover_01/fog_severe: 5074 J; office_crossover_01/lowlight: 5311 J. Generous budget = 2.1 x E0.
- Success = at least 90 % of frames in the main map. ATE is over successful runs only and is always given with k/n.

## Main findings, favourable and unfavourable

1. **The false loop closures are gone.** No loop closure was reported and no pose step above 0.3 m occurred in any
   of the 90 runs (largest step between consecutive frames: 0.22 m). The four earlier batches are superseded.
2. **Energy: adaptive at the tight budget uses about 80 % of static fusion's compute energy** (79.6 % clear,
   81.6 % fog_severe, 80.3 % lowlight; 72-75 % after subtracting idle). Its time at Level 0 / 1 / 2 is about
   51 / 35 / 14 % and the thermal sensor is on 52-68 % of the time.
3. **At the generous budget adaptive never leaves Level 0 and saves nothing** (98.6-100.6 % of static fusion).
4. **In clear conditions static RGB is as accurate as fusion and as cheap as adaptive**: ATE 0.027 m against
   0.024 m (fusion) and 0.031 m (adaptive tight), at 78.1 % of static fusion's compute energy (adaptive tight:
   79.6 %). The adaptive method has no advantage over RGB-only when RGB is good.
5. **Static RGB fails in fog_severe and lowlight** (0/6 successful in both, tracked 36-37 %). Fusion and adaptive
   keep tracking in fog_severe (6/6, 5/6 tight, 6/6 generous).
6. **Accuracy of adaptive is not better than static fusion, and worse in lowlight.** Pooled ATE: clear 0.031 (tight)
   / 0.038 (generous) against 0.024 m; fog_severe 0.086 / 0.087 against 0.072 m; lowlight 0.064 / 0.065 against
   0.020 m. Standard deviations are large (3 repeats per sequence); none of the differences in clear and fog_severe
   exceeds one standard deviation, the lowlight difference does.
7. **Adaptive at the tight budget loses tracking at in-place turns late in the run.** 17 of its 18 runs have a
   loss at 179-181 s (the final turn), always at Level 2 (every 2nd frame processed) and with the thermal sensor
   off when the loss starts; in 6 of them it lasts 77-90 frames, otherwise 1-2 frames. 4 runs also have a
   92-frame loss followed by a new map at about 147 s (third corner; Level 1, thermal off). 12 of 18 runs still
   pass the 90 % criterion (clear 5/6, fog_severe 5/6, lowlight 2/6). Static fusion has no loss in clear and
   fog_severe. The recovery override was active for 134 frames per run on average. The tuning sequence has a
   single turn and did not show this.
8. **`office_crossover_01` in lowlight defeats every configuration** (0/3 each, tracked about 50 %): at about 88 s
   the camera is in the thermal-crossover zone while the RGB image is dark, so neither modality has features.
   Static fusion and adaptive fail alike there. On `office_loop_01` lowlight static fusion is 3/3, adaptive tight
   2/3, adaptive generous 3/3.
9. **Static thermal is unreliable in this scene** (1/6 successful in every profile): on `office_loop_01` it
   loses tracking at 143.4 s, during the third in-place turn, in 6 of 9 runs (about 300 thermal keypoints there,
   tracked inliers falling from about 130 to under 40; a 92-frame loss, then a new map). On the crossover sequence
   it fails in every run: a loss of about 500 frames from about 45 s (inside the zone), then again at the turns at
   about 88 s and 143 s. (Corrected 2026-10-11: an earlier version said "around 88 s on both sequences".)
10. **Real time holds.** Mean tracking time 7.7-11.9 ms per frame, 95th percentile at most 15.6 ms in the worst
    run, no run more than one frame behind the 30 Hz input.
11. **Sensitivity (2 repeats, so indicative only):** energy changes by at most 6.5 % (tau_E1 -20 %); in every
    variation adaptive stays below static fusion's energy. Tracking is sensitive to the energy thresholds: with
    tau_E1 at -20 % or +20 % and tau_E2 at +20 % neither repeat passed the 90 % criterion (tracked 85 %, 52 %,
    85 %), and in 7 of the other 11 variations one of the two repeats failed it. The frozen setting's own ATE in
    this cell is 0.097 +- 0.057 m, so the ATE changes in the table are within the run-to-run spread.

## Limits of this evidence

- Two sequences in one simulated office, 3 repeats, one tuning sequence; corridor and hall were not re-run.
- Calibration (clear-condition quality means, profile severities) and tuning were done on `office_loop_tune`,
  recorded before the wall textures were made unique; they were not repeated (v5: no re-tuning).
- Tuning did not meet its ATE constraint in fog_severe (`constraints_met: false`), and the frozen setting
  processes every 2nd frame at Level 2, which the method text has to state.
- ORB-SLAM3's `Sim3Solver` was guarded against a crash on degenerate samples before evaluation (stock bug).
- Energy is CPU package energy (RAPL) on a laptop plus modelled sensor power, not measured on a robot.
- Thermal images are simulated (quantised, piecewise-constant, no emissivity or reflections).
- **Not done for this paper (future work):** D14 sparse-thermal check, the two ablations, the corridor and hall
  sequences with unique textures, fog_moderate, physical TurtleBot 3 runs.

## Table 1. Pooled over both sequences (6 runs per cell)

| profile | configuration | success k/n | ATE RMSE (m), successful runs | tracked (%) | tracking losses per run | compute energy (J) | compute energy (% of static fusion) | idle-subtracted compute energy (J) | idle-subtracted (% of static fusion) | sensor energy (J) | total energy (J) | thermal on (%) | time at L0/L1/L2 (%) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clear | Static RGB | 6/6 | 0.027 ± 0.005 | 100.0 | 0.0 | 3662 ± 135 | 78.1 | 2365 ± 135 | 69.7 | 365.0 | 4027 ± 135 | 0.0 | 100/0/0 |
| clear | Static thermal | 1/6 | 0.070 ± 0.000 | 60.7 | 1.83 | 3736 ± 253 | 79.7 | 2439 ± 253 | 71.9 | 474.0 | 4211 ± 253 | 100.0 | 100/0/0 |
| clear | Static fusion | 6/6 | 0.024 ± 0.008 | 100.0 | 0.0 | 4690 ± 78 | 100.0 | 3393 ± 78 | 100.0 | 474.0 | 5165 ± 78 | 100.0 | 100/0/0 |
| clear | Adaptive (tight budget) | 5/6 | 0.031 ± 0.008 | 97.2 | 1.0 | 3735 ± 36 | 79.6 | 2437 ± 36 | 71.8 | 422.0 | 4157 ± 38 | 51.9 | 50/38/12 |
| clear | Adaptive (generous budget) | 6/6 | 0.038 ± 0.017 | 100.0 | 0.0 | 4690 ± 32 | 100.0 | 3392 ± 32 | 100.0 | 474.0 | 5164 ± 31 | 100.0 | 100/0/0 |
| fog_severe | Static RGB | 0/6 | failed | 36.5 | 1.0 | 3220 ± 18 | 68.3 | 1923 ± 17 | 56.3 | 365.0 | 3585 ± 18 | 0.0 | 100/0/0 |
| fog_severe | Static thermal | 1/6 | 0.167 ± 0.000 | 60.8 | 1.67 | 3767 ± 361 | 79.9 | 2469 ± 361 | 72.3 | 474.0 | 4241 ± 361 | 100.0 | 100/0/0 |
| fog_severe | Static fusion | 6/6 | 0.072 ± 0.024 | 100.0 | 0.0 | 4714 ± 127 | 100.0 | 3416 ± 127 | 100.0 | 474.0 | 5188 ± 127 | 100.0 | 100/0/0 |
| fog_severe | Adaptive (tight budget) | 5/6 | 0.086 ± 0.049 | 97.7 | 1.5 | 3847 ± 88 | 81.6 | 2549 ± 88 | 74.6 | 439.0 | 4286 ± 88 | 67.5 | 51/33/16 |
| fog_severe | Adaptive (generous budget) | 6/6 | 0.087 ± 0.056 | 100.0 | 0.0 | 4742 ± 91 | 100.6 | 3444 ± 91 | 100.8 | 474.0 | 5216 ± 91 | 100.0 | 100/0/0 |
| lowlight | Static RGB | 0/6 | failed | 36.8 | 1.0 | 3237 ± 22 | 66.9 | 1939 ± 22 | 54.7 | 365.0 | 3602 ± 22 | 0.0 | 100/0/0 |
| lowlight | Static thermal | 1/6 | 0.078 ± 0.000 | 60.7 | 1.83 | 3701 ± 266 | 76.5 | 2403 ± 266 | 67.8 | 474.0 | 4176 ± 266 | 100.0 | 100/0/0 |
| lowlight | Static fusion | 3/6 | 0.020 ± 0.004 | 75.1 | 0.5 | 4841 ± 13 | 100.0 | 3543 ± 13 | 100.0 | 474.0 | 5316 ± 13 | 100.0 | 100/0/0 |
| lowlight | Adaptive (tight budget) | 2/6 | 0.064 ± 0.000 | 73.3 | 1.83 | 3890 ± 18 | 80.3 | 2592 ± 18 | 73.1 | 440.0 | 4330 ± 19 | 68.4 | 51/35/14 |
| lowlight | Adaptive (generous budget) | 3/6 | 0.065 ± 0.012 | 75.0 | 0.5 | 4775 ± 18 | 98.6 | 3477 ± 19 | 98.1 | 474.0 | 5250 ± 18 | 100.0 | 100/0/0 |

## Table 2. office_loop_01 (3 runs per cell)

| profile | configuration | success k/n | ATE RMSE (m), successful runs | tracked (%) | tracking losses per run | compute energy (J) | compute energy (% of static fusion) | idle-subtracted compute energy (J) | idle-subtracted (% of static fusion) | sensor energy (J) | total energy (J) | thermal on (%) | time at L0/L1/L2 (%) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clear | Static RGB | 3/3 | 0.025 ± 0.003 | 100.0 | 0.0 | 3736 ± 158 | 80.9 | 2438 ± 158 | 73.5 | 365.0 | 4101 ± 158 | 0.0 | 100/0/0 |
| clear | Static thermal | 1/3 | 0.070 ± 0.000 | 85.3 | 0.67 | 3987 ± 39 | 86.4 | 2689 ± 39 | 81.0 | 474.0 | 4462 ± 39 | 100.0 | 100/0/0 |
| clear | Static fusion | 3/3 | 0.031 ± 0.006 | 100.0 | 0.0 | 4616 ± 23 | 100.0 | 3318 ± 23 | 100.0 | 474.0 | 5091 ± 23 | 100.0 | 100/0/0 |
| clear | Adaptive (tight budget) | 3/3 | 0.028 ± 0.005 | 100.0 | 1.0 | 3704 ± 6 | 80.2 | 2406 ± 6 | 72.5 | 420.0 | 4124 ± 6 | 50.6 | 49/38/13 |
| clear | Adaptive (generous budget) | 3/3 | 0.033 ± 0.004 | 100.0 | 0.0 | 4667 ± 17 | 101.1 | 3369 ± 17 | 101.5 | 474.0 | 5142 ± 17 | 100.0 | 100/0/0 |
| fog_severe | Static RGB | 0/3 | failed | 36.5 | 1.0 | 3231 ± 19 | 66.8 | 1933 ± 19 | 54.6 | 365.0 | 3596 ± 19 | 0.0 | 100/0/0 |
| fog_severe | Static thermal | 1/3 | 0.167 ± 0.000 | 85.3 | 0.67 | 4125 ± 50 | 85.3 | 2827 ± 50 | 79.9 | 474.0 | 4600 ± 50 | 100.0 | 100/0/0 |
| fog_severe | Static fusion | 3/3 | 0.075 ± 0.033 | 100.0 | 0.0 | 4838 ± 26 | 100.0 | 3540 ± 26 | 100.0 | 474.0 | 5313 ± 26 | 100.0 | 100/0/0 |
| fog_severe | Adaptive (tight budget) | 3/3 | 0.097 ± 0.057 | 100.0 | 1.33 | 3935 ± 4 | 81.3 | 2637 ± 4 | 74.5 | 438.0 | 4373 ± 4 | 67.0 | 50/34/16 |
| fog_severe | Adaptive (generous budget) | 3/3 | 0.131 ± 0.049 | 100.0 | 0.0 | 4823 ± 10 | 99.7 | 3525 ± 10 | 99.6 | 474.0 | 5297 ± 10 | 100.0 | 100/0/0 |
| lowlight | Static RGB | 0/3 | failed | 36.8 | 1.0 | 3228 ± 13 | 66.7 | 1930 ± 13 | 54.5 | 365.0 | 3593 ± 13 | 0.0 | 100/0/0 |
| lowlight | Static thermal | 1/3 | 0.078 ± 0.000 | 85.3 | 0.67 | 3965 ± 24 | 81.9 | 2667 ± 24 | 75.3 | 474.0 | 4439 ± 24 | 100.0 | 100/0/0 |
| lowlight | Static fusion | 3/3 | 0.020 ± 0.004 | 100.0 | 0.0 | 4840 ± 12 | 100.0 | 3542 ± 12 | 100.0 | 474.0 | 5314 ± 12 | 100.0 | 100/0/0 |
| lowlight | Adaptive (tight budget) | 2/3 | 0.064 ± 0.000 | 95.1 | 1.33 | 3899 ± 21 | 80.6 | 2601 ± 21 | 73.4 | 440.0 | 4338 ± 23 | 68.1 | 51/35/14 |
| lowlight | Adaptive (generous budget) | 3/3 | 0.065 ± 0.012 | 100.0 | 0.0 | 4760 ± 13 | 98.4 | 3462 ± 13 | 97.7 | 474.0 | 5235 ± 13 | 100.0 | 100/0/0 |

## Table 3. office_crossover_01 (3 runs per cell)

| profile | configuration | success k/n | ATE RMSE (m), successful runs | tracked (%) | tracking losses per run | compute energy (J) | compute energy (% of static fusion) | idle-subtracted compute energy (J) | idle-subtracted (% of static fusion) | sensor energy (J) | total energy (J) | thermal on (%) | time at L0/L1/L2 (%) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| clear | Static RGB | 3/3 | 0.028 ± 0.005 | 100.0 | 0.0 | 3588 ± 16 | 75.3 | 2291 ± 16 | 66.1 | 365.0 | 3953 ± 16 | 0.0 | 100/0/0 |
| clear | Static thermal | 0/3 | failed | 36.1 | 3.0 | 3485 ± 28 | 73.2 | 2188 ± 28 | 63.1 | 474.0 | 3960 ± 28 | 100.0 | 100/0/0 |
| clear | Static fusion | 3/3 | 0.017 ± 0.004 | 100.0 | 0.0 | 4764 ± 24 | 100.0 | 3467 ± 24 | 100.0 | 474.0 | 5239 ± 24 | 100.0 | 100/0/0 |
| clear | Adaptive (tight budget) | 2/3 | 0.034 ± 0.009 | 94.5 | 1.0 | 3766 ± 25 | 79.0 | 2469 ± 25 | 71.2 | 423.0 | 4189 ± 27 | 53.3 | 51/38/11 |
| clear | Adaptive (generous budget) | 3/3 | 0.043 ± 0.022 | 100.0 | 0.0 | 4712 ± 27 | 98.9 | 3414 ± 27 | 98.5 | 474.0 | 5186 ± 27 | 100.0 | 100/0/0 |
| fog_severe | Static RGB | 0/3 | failed | 36.5 | 1.0 | 3210 ± 8 | 69.9 | 1913 ± 8 | 58.1 | 365.0 | 3575 ± 8 | 0.0 | 100/0/0 |
| fog_severe | Static thermal | 0/3 | failed | 36.2 | 2.67 | 3409 ± 38 | 74.3 | 2111 ± 38 | 64.1 | 474.0 | 3883 ± 38 | 100.0 | 100/0/0 |
| fog_severe | Static fusion | 3/3 | 0.070 ± 0.006 | 100.0 | 0.0 | 4590 ± 24 | 100.0 | 3292 ± 24 | 100.0 | 474.0 | 5064 ± 24 | 100.0 | 100/0/0 |
| fog_severe | Adaptive (tight budget) | 2/3 | 0.070 ± 0.027 | 95.5 | 1.67 | 3758 ± 7 | 81.9 | 2461 ± 7 | 74.8 | 439.0 | 4198 ± 8 | 68.1 | 51/32/17 |
| fog_severe | Adaptive (generous budget) | 3/3 | 0.044 ± 0.007 | 100.0 | 0.0 | 4660 ± 58 | 101.5 | 3363 ± 58 | 102.1 | 474.0 | 5135 ± 58 | 100.0 | 100/0/0 |
| lowlight | Static RGB | 0/3 | failed | 36.8 | 1.0 | 3247 ± 24 | 67.0 | 1949 ± 24 | 55.0 | 365.0 | 3612 ± 24 | 0.0 | 100/0/0 |
| lowlight | Static thermal | 0/3 | failed | 36.1 | 3.0 | 3438 ± 46 | 71.0 | 2140 ± 46 | 60.4 | 474.0 | 3912 ± 46 | 100.0 | 100/0/0 |
| lowlight | Static fusion | 0/3 | failed | 50.2 | 1.0 | 4843 ± 14 | 100.0 | 3545 ± 14 | 100.0 | 474.0 | 5317 ± 14 | 100.0 | 100/0/0 |
| lowlight | Adaptive (tight budget) | 0/3 | failed | 51.6 | 2.33 | 3881 ± 8 | 80.1 | 2583 ± 8 | 72.9 | 440.0 | 4321 ± 8 | 68.6 | 51/35/13 |
| lowlight | Adaptive (generous budget) | 0/3 | failed | 50.1 | 1.0 | 4790 ± 8 | 98.9 | 3493 ± 8 | 98.5 | 474.0 | 5264 ± 8 | 100.0 | 100/0/0 |

Energy columns are mean +- standard deviation over all runs of the cell, including unsuccessful ones.

## Table 4. Parameters (frozen before evaluation)

| parameter | value | note |
|---|---|---|
| alpha, beta, gamma (Eq. 1) | 0.4, 0.4, 0.2 | fixed |
| tau_Q (normalised quality) | 0.6 | tuned (stage 1) |
| tau_E1, tau_E2 (fraction of E0) | 0.5, 0.25 | tuned (stage 2) |
| hysteresis band | 0.02 | fixed |
| minimum dwell (s) | 2.0 | fixed |
| thermal-priority weight | 0.7 | fixed |
| minimum features per modality | 50 | fixed |
| thermal-off rule | True | tuned (stage 2) |
| recovery hold (s) | 2.0 | fixed |
| Level 0 (normal) | 1200 features; min. keyframe interval 0 frames; local BA every 1 keyframe(s); loop closure full; every 1 frame(s) processed | Level 0 fixed, Levels 1-2 tuned (stage 2) |
| Level 1 (energy_saving) | 800 features; min. keyframe interval 30 frames; local BA every 1 keyframe(s); loop closure reduced; every 1 frame(s) processed | Level 0 fixed, Levels 1-2 tuned (stage 2) |
| Level 2 (tracking_priority) | 700 features; min. keyframe interval 60 frames; local BA every 1 keyframe(s); loop closure off; every 2 frame(s) processed | Level 0 fixed, Levels 1-2 tuned (stage 2) |
| RGB-D sensor power (W) | 2.0 | D435, always on |
| thermal sensor power (W) | 0.6 | Boson 320-class, times u_TH |
| E0, tight budget | 1.00 x median measured energy of static fusion on the same sequence and profile | rule |
| E0, generous budget | 2.1 x the same median (= 1.05 / (1 - tau_E1)) | rule |
| tuned on | office_loop_tune | setting q0.6_e0.5-0.25_f800-700_k30-60_ba1_off1_s2, constraints met: False |

## Table 5. Per-frame processing time and lag

| configuration | runs | mean tracking time per frame (ms) | 95th percentile, mean over runs (ms) | 95th percentile, worst run (ms) | largest lag behind 30 Hz (s), worst run | runs with lag > 1 frame (33 ms) |
|---|---|---|---|---|---|---|
| Static RGB | 18 | 7.9 | 11.1 | 11.7 | 0.011 | 0 |
| Static thermal | 18 | 7.7 | 11.2 | 12.2 | 0.005 | 0 |
| Static fusion | 18 | 11.6 | 14.9 | 15.6 | 0.004 | 0 |
| Adaptive (tight budget) | 18 | 9.6 | 13.6 | 14.2 | 0.0 | 0 |
| Adaptive (generous budget) | 18 | 11.9 | 15.0 | 15.3 | 0.001 | 0 |

## Table 6. Tracking-loss events by configuration and profile

| cfg | profile | loss_events | median_t_s | mean_severity | at_level_0 | at_level_1 | at_level_2 | thermal_off | new_map |
|---|---|---|---|---|---|---|---|---|---|
| adaptive/generous | lowlight | 3 | 88.408 | 0.8039999999999999 | 3 | 0 | 0 | 0 | 3 |
| adaptive/tight | clear | 8 | 179.45499999999998 | 0.0 | 0 | 1 | 7 | 6 | 2 |
| adaptive/tight | fog_severe | 11 | 179.522 | 0.0 | 0 | 1 | 10 | 7 | 1 |
| adaptive/tight | lowlight | 15 | 179.488 | 0.1608 | 3 | 2 | 10 | 8 | 9 |
| static_fusion | lowlight | 3 | 87.848 | 0.8039999999999999 | 3 | 0 | 0 | 0 | 3 |
| static_rgb | fog_severe | 6 | 63.065 | 0.798 | 6 | 0 | 0 | 6 | 6 |
| static_rgb | lowlight | 6 | 60.094 | 0.6994639199999995 | 6 | 0 | 0 | 6 | 6 |
| static_thermal | clear | 11 | 87.914 | 0.0 | 11 | 0 | 0 | 0 | 11 |
| static_thermal | fog_severe | 10 | 87.864 | 0.2394 | 10 | 0 | 0 | 0 | 10 |
| static_thermal | lowlight | 11 | 87.814 | 0.2192727272727272 | 11 | 0 | 0 | 0 | 11 |

`thermal_off`: events at which the thermal sensor was off. `new_map`: events after which tracking continued in a
new map. Static RGB has the thermal sensor off by definition.

## Table 7. Sensitivity (adaptive, tight budget, office_loop_01, fog_severe, 2 repeats per variation)

| parameter | delta | runs | successful | ate_m | tracked | compute_energy_j | d_ate_pct | d_tracked_points | d_energy_pct | energy_below_static_fusion | ate_below_static_fusion |
|---|---|---|---|---|---|---|---|---|---|---|---|
| (frozen, main batch) | 0.0 | 3 | 3 | 0.0975 | 0.9997 | 3934.8387 | 0.0 | 0.0 | 0.0 | True | False |
| alpha | -0.2 | 2 | 2 | 0.0706 | 0.9995 | 3939.3326 | -27.5461 | -0.0229 | 0.1142 | True | True |
| alpha | 0.2 | 2 | 1 | 0.0338 | 0.9314 | 3948.6558 | -65.3575 | -6.8308 | 0.3511 | True | True |
| beta | -0.2 | 2 | 2 | 0.0552 | 0.9994 | 3931.9359 | -43.3886 | -0.0324 | -0.0738 | True | True |
| beta | 0.2 | 2 | 1 | 0.0551 | 0.9302 | 3950.6227 | -43.4562 | -6.9539 | 0.4011 | True | True |
| gamma | -0.2 | 2 | 1 | 0.0555 | 0.9369 | 3946.4662 | -43.059 | -6.2857 | 0.2955 | True | True |
| gamma | 0.2 | 2 | 1 | 0.0431 | 0.9318 | 3960.3015 | -55.7466 | -6.797 | 0.6471 | True | True |
| tau_Q | -0.2 | 2 | 2 | 0.0805 | 0.9998 | 3936.4063 | -17.4144 | 0.0065 | 0.0398 | True | False |
| tau_Q | 0.2 | 2 | 1 | 0.095 | 0.934 | 3963.5606 | -2.5386 | -6.5697 | 0.7299 | True | False |
| tau_E1 | -0.2 | 2 | 0 |  | 0.8534 | 4191.5055 |  | -14.6344 | 6.5229 | True |  |
| tau_E1 | 0.2 | 2 | 0 |  | 0.5234 | 3906.8495 |  | -47.6323 | -0.7113 | True |  |
| tau_E2 | -0.2 | 2 | 2 | 0.0487 | 0.9998 | 4034.1862 | -50.068 | 0.0073 | 2.5248 | True | True |
| tau_E2 | 0.2 | 2 | 0 |  | 0.8532 | 3910.7934 |  | -14.6521 | -0.6111 | True |  |
| thermal_priority_weight | -0.2 | 2 | 1 | 0.1062 | 0.9325 | 3953.6172 | 8.9491 | -6.7235 | 0.4772 | True | False |
| thermal_priority_weight | 0.2 | 2 | 1 | 0.0305 | 0.9296 | 3938.7355 | -68.7469 | -7.0154 | 0.099 | True | True |

Changes are relative to the frozen setting in the main batch (3 repeats). The sweep ran right after the matrix
with its own idle measurement (3.9 W); compute energy is compared without idle subtraction.

## Figures (`results/v5/`, PNG at 600 dpi and PDF)

- `fig_a_quality_weights`: normalised quality and fusion weights over time, `office_crossover_01`, fog_severe,
  adaptive (tight), repeat 0. Grey: degradation active. Orange bar: camera inside the thermal-crossover zone.
  Where the thermal curve is missing the thermal sensor is off.
- `fig_b_power_energy`: power (1 s moving average, the only smoothing anywhere) and cumulative energy,
  `office_loop_01`, fog_severe, repeat 0 of each configuration.
- `fig_c_ate_energy`: ATE against total energy per configuration, pooled over both sequences, one panel per
  profile, error bars one standard deviation, labels successful/total runs; a dotted line marks a configuration
  with no successful run. Exact run directories are in `figure_notes.json`.
