# Changes to ORB-SLAM3

Base: `UZ-SLAMLab/ORB_SLAM3` commit `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (master, v1.0).
Full diff: `orbslam3_ms.patch` (apply and build with `orbslam3_patch/build.sh`).
Mode: RGB-D (`System::RGBD`), no IMU, no viewer.

All behavioural changes are behind the compile-time definition `MS_SLAM` (CMake option
`ORBSLAM3_MULTISPECTRAL`, default ON). With the option OFF the library behaves as stock ORB-SLAM3.

## 1. Build only (no behaviour change)

| File | Change |
|---|---|
| `CMakeLists.txt` | CMake >= 3.10; C++14 instead of C++11 (current Eigen/Boost/GCC 15); option `ORBSLAM3_HEADLESS` (default ON) builds without Pangolin; only the `rgbd_tum` example is built when headless. |
| `headless/pangolin/pangolin.h` (new) | Declares the two Pangolin/GL types that appear in ORB-SLAM3 headers. |
| `src/HeadlessStubs.cc` (new) | No-op `Viewer` and `MapDrawer`. The viewer is never started (`bUseViewer = false`). |

Check: stock `rgbd_tum` on the clear tuning sequence tracks all 2696 frames in one map, ATE RMSE 0.059 m
(0.015 m after scale alignment), median tracking time 12 ms.

## 2. Control block (`include/MSControl.h`, `src/MSControl.cc`, new)

A process-wide `MSControl` object, written by the caller before each `TrackRGBD` call:

| Field | Meaning | Read in |
|---|---|---|
| `nFeaturesRGB`, `nFeaturesThermal` | ORB budget per modality for this frame; 0 switches the modality off | `Frame::ExtractORBMS` |
| `thermal` | 8-bit thermal image of this frame, same size and intrinsics as the RGB image | `Frame::ExtractORBMS` |
| `minKeyFrameInterval` | minimum frames between keyframes (0 = stock policy) | `Tracking::NeedNewKeyFrame` |
| `localBA` | local bundle adjustment on/off | `LocalMapping::Run` |
| `loopClosureMode` | 0 full, 1 reduced, 2 off | `LoopClosing::Run`, `LoopClosing::CorrectLoop` |

`System::GetMSFrameStats()` returns, for the frame just tracked: keypoints and tracked inliers per
modality, keyframe flag, tracking state, map count, and the control values that were applied.

## 3. Weighted feature pooling (guide 6.2)

- `ORBextractor::SetNFeatures(n)` changes the feature budget at run time (same per-level distribution as
  the constructor); `Clone()` creates the thermal extractor with identical pyramid and FAST thresholds.
- `Frame::ExtractORBMS` (called from the RGB-D `Frame` constructor instead of `ExtractORB`): extracts
  ORB from the RGB image with `nFeaturesRGB` and from the thermal image with `nFeaturesThermal`, then pools
  the keypoints into the one `Frame`. Everything downstream (depth association, grid, map points, BoW) is
  unchanged and treats pooled keypoints alike.
- **Modality tag.** Descriptors are stored as 33 bytes: the 32-byte ORB descriptor plus one byte holding the
  modality (0 RGB, 1 thermal). The tag travels with the descriptor into keyframes and map points.
- **Matching only within a modality.** `ORBmatcher::DescriptorDistance` returns 256 (larger than any
  acceptance threshold) when the two tags differ. All matchers go through this function, so tracking,
  triangulation, fusion, relocalisation and loop verification never match RGB to thermal features.
  DBoW2 reads only the first 32 bytes, so bag-of-words vectors are unaffected by the tag.
- **Thermal image geometry.** The 320x240 thermal frame is resized to 640x480 (bilinear) before extraction,
  so thermal keypoints live on the RGB pixel grid, use the shared depth image and the same scale pyramid.
  This relies on the co-located cameras with identical field of view.
- The thermal vocabulary is the stock ORB vocabulary (trained on visible images).

## 4. Scheduler hooks (guide 6.4)

- **Minimum keyframe interval:** `NeedNewKeyFrame` returns false while fewer than `minKeyFrameInterval`
  frames have passed since the last keyframe.
- **Local BA:** `LocalBundleAdjustment` is skipped when `localBA` is false.
- **Loop closure:**
  - *full*: stock behaviour.
  - *reduced*: place recognition is attempted for one keyframe in three (`MS_LC_REDUCED_EVERY`); the others
    are only added to the keyframe database. After a loop correction the essential-graph optimisation runs,
    but the global bundle adjustment thread is not launched.
  - *off*: keyframes are only added to the keyframe database; no detection, no correction.

- **Local BA every n-th keyframe:** `MSControl::localBAEvery` (default 1); with 2, local BA runs for every
  second keyframe inserted while the setting is active.

## 5. Evaluation support (DECISIONS D4)

- `GetMSFrameStats` also returns `mapId`, the Atlas id of the map the frame was tracked in.
- `System::SaveTrajectoryAllMapsMS` writes the final frame trajectory of every map in the Atlas with the map
  id per pose (the stock `SaveTrajectoryTUM` only writes the current map), so the harness can evaluate on the
  main map.

## 6. Robustness fix to stock code

- `Sim3Solver::ComputeSim3`: stock code divides the quaternion's imaginary part by its norm. For a degenerate
  RANSAC sample the norm is zero, the angle-axis vector becomes NaN and `Sophus::SO3f::exp` aborts the
  process (seen in 3 % of tuning runs, only in degraded profiles, in every configuration). When the norm is
  zero or not finite the identity rotation is used and the hypothesis is scored as usual. Runs that did not
  crash never take this branch.

## 7. Known limits

- ORB-SLAM3 needs more than 500 keypoints in a frame to (re)initialise an RGB-D map. The Level 2
  budget is therefore kept at 600 or more (DECISIONS D5), and the recovery override (D10) restores the Level 0
  budget after a tracking loss.
- `MSControl` is process-wide: one `System` per process (the harness starts one process per run).
- Atlas save/load is untested with 33-byte descriptors (not used).
