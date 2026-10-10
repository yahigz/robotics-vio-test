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
