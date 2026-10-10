# OpenVINS (filter-based MSCKF VIO) - ROS1 Noetic, arm64 Docker

Pinned: `rpng/open_vins` tag **v2.7** (commit `93adc241390d13e99232652cf05cbe18a93c7bea`), built with catkin
(`ov_core`, `ov_init`, `ov_msckf` only; no rviz / ov_eval), Release, on `ros:noetic-ros-base` (native aarch64).

## Build
    docker compose -f methods/openvins/compose.yml build      # or: docker compose build openvins  (root include)
Build uses `-j2` (ARG JOBS); with -j4 the compile was OOM-killed under 8 GB Docker memory (esp. when other builds run).

## Run
    docker compose run --rm openvins <dataset> <sequence> <config>
    docker compose run --rm openvins euroc MH_01_easy euroc_mav        # stereo-inertial
    docker compose run --rm openvins euroc MH_01_easy euroc_mav_mono   # mono-inertial
Env passthrough: `BAG_START` (s), `BAG_DURR` (s), `VERBOSITY`.

Inputs: `data/<dataset>/<seq>/<seq>.bag` (ro). Outputs in `results/openvins/<dataset>/<seq>/<config>/` (one subfolder per config so modes do not overwrite):
`traj_tum.txt` (t[s] tx ty tz qx qy qz qw; IMU pose in OpenVINS global frame, from `/ov_msckf/poseimu`),
`log.txt`, `timing.txt` (OpenVINS per-stage timing), `time_wall.txt` (wall-clock of the run).
Serial mode (`ros1_serial_msckf`): bag read directly, as fast as possible, no realtime playback.
`config/`, `launch/`, `scripts/`, `run.sh` are bind-mounted, so editing them needs no rebuild.

## Bag created on demand from ASL (ROVER etc.)
    methods/openvins/run_from_asl.sh rover campus_small_2024-02-19 rover_t265_mono [--duration 60]
Host wrapper: converts `data/<ds>/<seq>/asl` to a temp bag (tools/rover/asl_to_bag.py for ROVER, tools/asl_to_bag.py
otherwise, via pixi), mounts it as `/bags`, runs OpenVINS (`BAG` env override), deletes the bag on exit.
`config/rover_t265_mono` is a copy of tools/rover/generated/openvins (RealSense T265 left fisheye, equidistant).

## Add a dataset / config
1. Convert the data to a ROS1 bag at `data/<ds>/<seq>/<seq>.bag` (camera + IMU).
2. `cp -r config/TEMPLATE_ground_robot config/<name>` and edit (see its NOTES.md): intrinsics, `T_imu_cam`
   (camera->IMU), IMU noise, topics, `max_cameras`/`use_stereo`, init/ZUPT params.
3. `docker compose run --rm openvins <ds> <seq> <name>`.

## Notes / known issues
- Trajectory is not auto-aligned to GT: global frame origin = initial IMU pose, gravity-aligned (yaw arbitrary).
  Evaluate with SE(3) (or 4-DoF) alignment.
- Poses are recorded by a rospy subscriber on `/ov_msckf/poseimu` (OpenVINS' `save_total_state` is simulation-only);
  `run.sh` prints the pose count - if far fewer than camera frames, messages were dropped.
- Initialization is static (needs a still start + motion jerk); `init_dyn_use: true` for dynamic init.
- Upstream says `bag_start` 40 for MH_01 (skips still part); default here is 0.
- Image is ~5.5 GB (ROS base + dev libs + 400 MB upstream source); could be trimmed with a multi-stage build.
- Wall-clock timing varies a lot when other Docker builds run concurrently (stereo run once took 960 s under load).

## ROVER T265 mono-inertial (validated on garden_small, 326 s, 121 m of path)
`config/rover_t265_mono` = `tools/rover/generated/openvins` + **one manual tuning**: `zupt_max_velocity 0.2 -> 0.05`, `zupt_max_disparity 0.5 -> 0.1`.
* With the generated ZUPT settings the filter diverged within ~5 s of the robot starting to move (garden APE 29.8 m, trajectory 88 m away from GT):
  the ground robot creeps at ~0.4 m/s, ZUPT kept firing (disparity < 0.5 px on a 280 px-focal fisheye) and the monocular filter lost scale.
  `try_zupt: false` and the strict values both fix it (80 s snippet: 0.48 / 0.46 m APE; full: 0.95 / 0.84 m); strict ZUPT kept, it still helps at real stops.
  `init_dyn_use`, time-offset calibration, pixel sigma 2 and `max_slam: 0` were tried on the snippet and did not help.
* Verified, not wrong: `T_imu_cam` direction (config uses cam->IMU = `T_imu_cam` of the calib), IMU units (accel norm 9.84 m/s^2 at rest, gravity on +y), IMU noise (Kalibr x2).
* Result (APE with prism lever arm, SE(3) alignment): see `results/summary/rover/garden_small_2024-05-29_1/` and the table in the top-level README.
* Wall time ~40 s for the 326 s sequence (+ ~50 s bag conversion in `run_from_asl.sh`).
