# ORB-SLAM3 (optimization-based visual-inertial SLAM) in Docker, arm64, headless

## Pinned versions
| component | version |
|---|---|
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (master, "Update Dependencies.md"; code identical to `v1.0-release` 0df83dd, 22 Dec 2021, plus docs) |
| Pangolin | `v0.8` (build-time dependency only; viewer is never created) |
| base image | `ubuntu:22.04` (native `linux/arm64`; gcc 11, Eigen 3.4.0, OpenCV 4.5.4 from apt) |

## Build / run
```bash
# from repo root (root docker-compose.yml includes methods/orbslam3/compose.yml)
docker compose build orbslam3            # ~8 min on M4 (g2o/DBoW2 ~6 min, ORB-SLAM3 + examples ~1.5 min) once Pangolin/apt layers are cached; uses -j2 (Tracking.cc/LoopClosing.cc OOM-kill cc1plus at -j6 with 8 GB)

# data/<dataset>/<seq>/asl/mav0/{cam0,cam1,imu0}  must exist (CONTRACT.md)
docker compose run --rm orbslam3 euroc MH_01_easy mono_inertial EuRoC_mono_inertial.yaml
docker compose run --rm orbslam3 euroc MH_01_easy stereo_inertial EuRoC_stereo_inertial.yaml --runs 3
docker compose run --rm orbslam3 <dataset> <seq> mono_inertial|stereo_inertial <settings.yaml> [--runs N] [--loop-closing]
```
`settings.yaml` is a path, or a file name looked up in `./config` (mounted at `/workspace/config`) and then in the image.

Outputs in `results/orbslam3_<mode>/<dataset>/<seq>/` (e.g. `orbslam3_mono_inertial`):
`traj_tum.txt` (= run 1), `traj_tum_run<K>.txt`, `kf_traj_tum*.txt` (keyframes only), `log.txt`, `settings_used.yaml`.

## Output semantics (verified in source, `System::SaveTrajectoryEuRoC`)
* Executables write `f_<name>.txt` as `t_ns tx ty tz qx qy qz qw`; `t` = camera-frame time x 1e9 (nanoseconds, same clock as the dataset). run.sh divides by 1e9 -> seconds (TUM).
* For inertial modes the pose is **Twb: IMU/body frame in world**. (Mono/stereo without IMU would be camera frame; not used here.)
* The world frame is the body frame of the **first keyframe** (not gravity aligned to GT; evaluation must use SE(3) alignment; mono-inertial scale is metric, so no Sim(3) needed).
* One line per tracked frame (all camera frames; lost frames skipped). Only the Atlas map with the most keyframes is saved: after a tracking-lost reset the trajectory can cover only a part of the sequence.
* Timestamp precision: ns -> double seconds, error ~0.2 us (negligible).

## Loop closing
Default **OFF**: run.sh forces `loopClosing: 0` into a copy of the settings (v1.0 supports this key in `System.cc`; LoopClosing thread still runs but does nothing/no map merging). `--loop-closing` sets `loopClosing: 1`. Note: Atlas multi-map merging only happens via loop closing, so OFF = pure odometry + local BA.

## Patches (`patches/`, applied by `git apply` in the Dockerfile)
1. `0001-build-flags-cxx17-no-march-native.patch`: drop `-march=native` (CMakeLists, DBoW2, g2o; breaks/unneeded under Docker-on-arm64 and makes images non-portable); `-std=c++11` -> `-std=c++17` (Pangolin 0.8 headers need >= C++14/17 and Eigen 3.4 on gcc 11; the `COMPILER_SUPPORTS_CXX11`/`COMPILEDWITHC11` macro names are kept so the `steady_clock` branch stays active).
2. `0002-euroc-examples-headless-steady-clock.patch`: `std::chrono::monotonic_clock` -> `steady_clock` (the #else fallback branch does not exist in modern libstdc++), and `mono_inertial_euroc` viewer flag `true` -> `false` (stereo_inertial_euroc already passes `false`).

3. `0003-loopclosing-bool-increment-cxx17.patch`: `mnFullBAIdx` is a `bool` incremented with `++` (error in C++17); replaced by `= true` (identical behaviour: `true++` stays true).

Other: vocabulary is un-tarred at build time (`Vocabulary/ORBvoc.txt`). The `COPY` of run.sh/config is the last layer so script edits do not rebuild ORB-SLAM3.

## Known issues
* Non-deterministic (multi-threaded local mapping/IMU init timing): use `--runs N` and choose by median ATE.
* Build needs <= -j2 with 8 GB Docker memory (override with `--build-arg BUILD_JOBS=N`).
* Runs at real-time-ish speed per frame (the example sleeps to keep the camera rate when tracking is faster than the frame period, so wall time >= sequence duration, ~3 min for MH_01).
* Planar / low-excitation motion: IMU initialization can fail or scale drift; see notes in `config/TEMPLATE_ground_robot.yaml`.
* Settings files use the new v1.0 format (`File.version: "1.0"`, `Camera1.*`, `IMU.T_b_c1`). The EuRoC examples are copies of the upstream ones.

## Sanity results, EuRoC MH_01_easy (loop closing OFF, `evo_ape tum --t_max_diff 0.01`)
| mode | poses | ATE RMSE SE(3) align | ATE RMSE Sim(3) align | wall time |
|---|---|---|---|---|
| mono_inertial run 1 | 2543/3682 | 0.086 m | 0.039 m (scale 1.023) | 202 s |
| mono_inertial run 2 | 2546/3682 | 0.087 m | 0.036 m (scale 1.024) | 203 s |
| stereo_inertial run 1 | 3678/3682 | 0.044 m | 0.023 m (scale 1.009) | 212 s |

* Mono-inertial: paper reports ~0.036 m (Sim(3)-style monocular evaluation, with loop closing); we match with Sim(3). Under SE(3) alignment (the benchmark protocol) the ~2.3 % residual scale error dominates (0.086 m).
* Mono-inertial only starts producing poses ~56 s into MH_01 (sequence starts static; IMU initialization needs motion, ~80 "Not enough motion"/"bad imu" map resets in the log). Compare only on the covered time span (evo does this via timestamp association); report coverage.
* Wall time ~ real time (sequence = 184 s) because the example sleeps to the camera rate. One early mono run took 1121 s when the Docker VM was starved by other heavy jobs (CPU-bound local mapping); results were similar (ATE 0.093 m), so the run time is a load artefact, not the code.
* Each mode writes to its own `results/orbslam3_<mode>/` folder, so modes do not overwrite each other.

## ROVER T265 mono-inertial
`docker compose run --rm orbslam3 rover <seq> mono_inertial rover_t265_mono_inertial.yaml [--runs 3] [--max-frames N]`
(`--max-frames N` uses only the first N camera frames, for smoke tests; `run.sh` is bind-mounted, no rebuild needed.)
`config/rover_t265_mono_inertial.yaml` = `tools/rover/generated/orbslam3` (KannalaBrandt8 `Camera1.fx..k4` = T265 left fisheye, 848x800, fps 30, IMU 200 Hz,
Kalibr noise x2) with:
* `IMU.T_b_c1` = `T_imu_cam` (camera -> IMU/body), checked against `data/rover/calib_t265.yaml` (`T_imu_cam`, the inverse of Kalibr's `T_cam_imu`): correct direction, no change.
* **ORB tuning** (the only change): `nFeatures 1500 -> 2000`, `iniThFAST 20 -> 12`, `minThFAST 7 -> 5`. With the generated values the 90 s garden_small smoke test
  reset the map ~240 times ("Fail to track local map!" before IMU init; low-contrast grass, circular fisheye) and only initialised at t = 56 s (APE 0.26 m on the last 34 s);
  with the tuned values: 0 resets, poses from t = 11 s (first motion), 2372 poses over 80 s.
* Smoke test (garden_small, first 2700 frames = 90 s, 1 run, loop closing off): APE rmse 0.77 m (SE(3)), 0.20 m with Sim(3) alignment, i.e. the mono-inertial scale is
  still ~off early on (GT path 31 m). The full-sequence numbers are in the top-level README table.
* Trajectory is the IMU (body) pose; ROVER evaluation moves it to the prism with the lever arm (`eval/evaluate.py --lever-arm-calib`).
