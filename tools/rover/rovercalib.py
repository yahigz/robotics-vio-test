"""Shared helpers: build/load the normalized ROVER T265 calibration, rectified-pinhole parameters."""
import os
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "calib", "calib_t265_raw.yaml")
NORM = os.path.join(HERE, "calib", "calib_t265.yaml")
DATA_NORM = os.path.normpath(os.path.join(HERE, "..", "..", "data", "rover", "calib_t265.yaml"))


def inv(T):
    T = np.asarray(T, float); R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4); out[:3, :3] = R.T; out[:3, 3] = -R.T @ t
    return out


def build_normalized(raw_path=RAW):
    c = yaml.safe_load(open(raw_path))
    M = lambda k: np.array(c[k], float)
    # Verified numerically (1e-16): IMU-To-CamLeft/Right are Kalibr T_cam_imu (p_cam = T p_imu) and
    # CamRight-To-CamLeft is Kalibr T_cam1_cam0 (p_right = T p_left): R @ inv(L) == RL.
    T_cl_i, T_cr_i = M("IMU-To-CamLeft"), M("IMU-To-CamRight")
    chk = np.abs(T_cr_i @ inv(T_cl_i) - M("CamRight-To-CamLeft")).max()
    # CamLeft-To-Prism: p_prism = T p_camleft (deduced: with VN100-To-CamLeft it gives a prism ~0.4 m behind and
    # ~0.18 m above the camera; UNVERIFIED sign convention).
    T_p_cl = M("CamLeft-To-Prism")
    T_i_p = inv(T_cl_i) @ inv(T_p_cl)  # prism frame -> IMU frame (prism frame axes = VN100 axes)
    def cam(key, T_c_i):
        d = c[key]
        return dict(model="equidistant", resolution=[848, 800], rate_hz=30.0,
                    intrinsics=[float(x) for x in d["intrinsics"]],  # fx fy cx cy
                    distortion=[float(x) for x in d["distortion_coeffs"]],  # k1..k4 (Kannala-Brandt / OpenCV fisheye)
                    T_cam_imu=T_c_i.tolist(), T_imu_cam=inv(T_c_i).tolist())
    i = c["IMU_Intrinsics"]
    return dict(
        sensor="Intel RealSense T265 (fisheye stereo, global shutter) + BMI055 IMU",
        source="ROVER calibration.zip/calib_t265.yaml (Kalibr)",
        conventions=dict(T_cam_imu="p_cam = T p_imu (Kalibr)", T_imu_cam="p_imu = T p_cam",
                         cam_frame="x right, y down, z forward", imu_frame="T265 IMU: x left, y up, z forward (accel reads +g on +y at rest)",
                         verification=f"max|T_cr_imu inv(T_cl_imu) - CamRight-To-CamLeft| = {chk:.2e}"),
        cam0=cam("CamLeft_Intrinsics", T_cl_i),
        cam1=cam("CamRight_Intrinsics", T_cr_i),
        imu=dict(rate_hz=200.0, unit_gyro="rad/s", unit_acc="m/s^2",
                 gyroscope_noise_density=float(i["noise_gyro"]),       # rad/s/sqrt(Hz)
                 accelerometer_noise_density=float(i["noise_acc"]),    # m/s^2/sqrt(Hz)
                 gyroscope_random_walk=float(i["walk_gyro"]),          # rad/s^2/sqrt(Hz)
                 accelerometer_random_walk=float(i["walk_acc"])),      # m/s^3/sqrt(Hz)
        prism=dict(T_prism_cam0=T_p_cl.tolist(), T_imu_prism=T_i_p.tolist(),
                   position_in_imu=[float(x) for x in T_i_p[:3, 3]],
                   note="GT = total-station prism position (no orientation). Lever arm |p| = %.3f m (UNVERIFIED sign convention)" % np.linalg.norm(T_i_p[:3, 3])),
    )


def load(path=None):
    for p in ([path] if path else [DATA_NORM, NORM]):
        if p and os.path.exists(p):
            return yaml.safe_load(open(p))
    raise FileNotFoundError("run make_calib.py first")


def rectified(calib, size=(640, 480), hfov_deg=100.0):
    """Pinhole intrinsics of the rectified image from the equidistant cam0 (shared by undistort + configs)."""
    w, h = size
    fx = (w / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
    return dict(width=w, height=h, fx=float(fx), fy=float(fx), cx=(w - 1) / 2.0, cy=(h - 1) / 2.0, hfov_deg=hfov_deg)
