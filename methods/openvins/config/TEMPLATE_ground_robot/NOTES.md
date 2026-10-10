# Ground-robot config template (copy this folder to config/<name>/ and edit)

Run with: `docker compose run --rm openvins <dataset> <seq> <name>`

Files and what to change (search for `TEMPLATE:` in estimator_config.yaml):

1. kalibr_imucam_chain.yaml  (values here are EuRoC placeholders - ALL must be replaced)
   - `intrinsics` [fu fv cu cv], `distortion_model` (radtan | equidistant), `distortion_coeffs`, `resolution`
   - `T_imu_cam`: 4x4 transform camera -> IMU (R_CtoI, p_CinI). Kalibr outputs T_cam_imu (inverse!) as `T_cam_imu`;
     invert it, or check whether the dataset gives body->camera and invert accordingly.
   - `rostopic` per camera; `cam_overlaps` (stereo pairs: [1] for cam0, [0] for cam1; [] for mono)
   - add/remove `camN` blocks to match `max_cameras`
2. kalibr_imu_chain.yaml
   - noise densities / random walks from the IMU datasheet or Allan variance. Real IMUs are usually noisier than
     EuRoC: inflate by ~2-10x if the filter is overconfident (drift/divergence). Units: continuous-time
     (m/s^2/sqrt(Hz), rad/s/sqrt(Hz)), not discrete.
   - `rostopic`, `update_rate`, `time_offset` (cam-IMU offset guess; estimated online if calib_cam_timeoffset)
   - `T_i_b` is identity unless IMU is not the body frame (output pose is of the IMU frame regardless).
3. estimator_config.yaml
   - `max_cameras`, `use_stereo` (run.sh reads these two).
   - Low-excitation planar motion: `init_dyn_use: true` (dynamic init, no jerk needed) with `init_window_time`
     1.5-3 s and lower `init_dyn_min_deg`; if static init is preferred keep `init_dyn_use: false` and tune
     `init_imu_thresh` / `init_max_disparity` (px, depends on resolution).
   - `try_zupt: true` + `zupt_max_velocity`, `zupt_noise_multiplier`, `zupt_max_disparity` (px): zero-velocity
     updates stop drift when the robot is stationary. `zupt_only_at_beginning: false`.
   - Disable `calib_cam_extrinsics`/`calib_cam_intrinsics` when excitation is planar and calibration is trusted.
   - Wheel odometry is NOT used by OpenVINS (vision + IMU only).
4. Dataset side (if bag topics/types differ): OpenVINS needs sensor_msgs/Imu and sensor_msgs/Image (or
   CompressedImage) topics; mono8/mono16/bgr8 supported. Timestamps must be on one clock (bag header stamps).
   Bag start: `BAG_START=<sec> docker compose run ...` to skip junk at the beginning.
