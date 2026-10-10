"""Input adapter: EuRoC/ASL-format sequence -> the in-memory inputs DBA-Fusion expects.

DBA-Fusion has no file format of its own; its demo scripts (demo_vio_tumvi.py) read
  * images : generator of (t_seconds, uint8 tensor [1,3,H,W], intrinsics tensor [fx,fy,cx,cy])
  * IMU    : float64 array Nx7 = [t_s, gx, gy, gz (DEG/s), ax, ay, az (m/s^2)]
  * stamps : Nx1 array of ALL camera timestamps in seconds (used for full-rate output)
This module builds exactly these from <seq>/asl/mav0/{cam0,imu0}. Pure numpy/cv2; torch is
only imported lazily in `image_stream` so the rest is testable without a GPU.

CLI: `python asl_to_dbafusion.py --asl_dir <.../asl> --config <yaml> --check`
prints a sanity report (rates, ranges, unit heuristics) and exits non-zero on hard errors.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml

# ---------------------------------------------------------------- config helpers


def load_config(path: str, overrides: dict | None = None) -> dict:
    with open(path) as f:
        cfg = yaml.safe_load(f)
    for k, v in (overrides or {}).items():       # dotted keys, e.g. {"dbaf.max_factors": 36}
        d = cfg
        parts = k.split(".")
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = v
    validate_config(cfg)
    return cfg


def validate_config(cfg: dict) -> None:
    cam = cfg["camera"]
    for k in ("fx", "fy", "cx", "cy", "width", "height", "model"):
        if k not in cam:
            raise ValueError(f"camera.{k} missing")
    if cam["model"] not in ("radtan", "equidistant", "none"):
        raise ValueError("camera.model must be radtan|equidistant|none")
    T = np.array(cfg["T_imu_cam"], dtype=float)
    if T.size != 16:
        raise ValueError("T_imu_cam needs 16 numbers")
    T = T.reshape(4, 4)
    R = T[:3, :3]
    if not (np.allclose(R @ R.T, np.eye(3), atol=1e-3) and abs(np.linalg.det(R) - 1) < 1e-3):
        raise ValueError("T_imu_cam rotation is not a proper rotation")
    if not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError("T_imu_cam last row must be 0 0 0 1")
    if cfg["imu"]["gyro_unit"] not in ("rad_s", "deg_s"):
        raise ValueError("imu.gyro_unit must be rad_s|deg_s")
    if len(cfg["imu"]["dbaf_noise"]) != 4:
        raise ValueError("imu.dbaf_noise needs 4 numbers")


def T_imu_cam(cfg: dict) -> np.ndarray:
    return np.array(cfg["T_imu_cam"], dtype=np.float64).reshape(4, 4)

# ---------------------------------------------------------------- file readers


def _read_csv_rows(path: str, ncols_min: int):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < ncols_min:
                continue
            rows.append(parts)
    return rows


def read_imu_csv(path: str, gyro_unit: str = "rad_s") -> np.ndarray:
    """EuRoC imu0/data.csv -> Nx7 [t_s, gyro(deg/s)x3, acc x3] (DBA-Fusion convention)."""
    rows = _read_csv_rows(path, 7)
    arr = np.array([[float(x) for x in r[:7]] for r in rows], dtype=np.float64)
    if arr.shape[0] < 10:
        raise ValueError(f"IMU file {path} has too few samples")
    arr = arr[np.all(np.isfinite(arr), axis=1)]
    t = arr[:, 0] / 1e9
    if np.any(np.diff(t) <= 0):                      # sort + drop duplicates
        order = np.argsort(t, kind="stable")
        arr, t = arr[order], t[order]
        keep = np.concatenate([[True], np.diff(t) > 0])
        arr, t = arr[keep], t[keep]
    out = arr.copy()
    out[:, 0] = t
    if gyro_unit == "rad_s":
        out[:, 1:4] = np.degrees(arr[:, 1:4])
    return out


def read_cam_csv(path: str):
    """EuRoC cam0/data.csv -> (t_s float64 array, filenames list), sorted by time."""
    rows = _read_csv_rows(path, 2)
    t = np.array([float(r[0]) / 1e9 for r in rows], dtype=np.float64)
    names = [r[1] for r in rows]
    order = np.argsort(t, kind="stable")
    return t[order], [names[i] for i in order]


def select_frames(cam_t, imu_t, stride: int, max_frames: int | None = None,
                  start_s: float = 0.0, end_s: float | None = None):
    """Indices (into cam arrays) of frames to feed: every `stride`-th frame, restricted to
    the interval where IMU data bracket the image (frontend indexes all_imu past the image
    time and would IndexError otherwise)."""
    ok = (cam_t > imu_t[0] + 1e-3) & (cam_t < imu_t[-1] - 0.05)
    ok &= cam_t >= cam_t[0] + start_s
    if end_s is not None:
        ok &= cam_t <= cam_t[0] + end_s
    idx = np.nonzero(ok)[0][::stride]
    if max_frames:
        idx = idx[:max_frames]
    return idx


def sanity_report(imu: np.ndarray, cam_t: np.ndarray, cfg: dict) -> list[str]:
    msgs = []
    dt = np.diff(imu[:, 0])
    rate = 1.0 / np.median(dt)
    msgs.append(f"IMU: {len(imu)} samples, median rate {rate:.1f} Hz, max gap {dt.max()*1e3:.1f} ms,"
                f" span {imu[0,0]:.3f}..{imu[-1,0]:.3f} s")
    if abs(rate - cfg["imu"]["rate_hz"]) > 0.1 * cfg["imu"]["rate_hz"]:
        msgs.append(f"WARNING: IMU rate {rate:.1f} Hz differs from config rate_hz={cfg['imu']['rate_hz']}")
    if dt.max() > 0.1:
        msgs.append("WARNING: IMU gap > 100 ms")
    anorm = np.linalg.norm(imu[:, 4:7], axis=1)
    med_a = float(np.median(anorm))
    msgs.append(f"accel norm median {med_a:.2f} m/s^2 (expect ~9.8 incl. gravity)")
    if not (8.0 < med_a < 12.0):
        msgs.append("WARNING: accel norm far from 9.81 -> wrong units (g?) or gravity removed")
    gmax = float(np.max(np.abs(imu[:, 1:4])))
    msgs.append(f"gyro max |w| {gmax:.1f} deg/s after conversion (config unit: {cfg['imu']['gyro_unit']})")
    if gmax > 1500 or gmax < 0.5:
        msgs.append("WARNING: gyro magnitude implausible -> check gyro_unit")
    cdt = np.diff(cam_t)
    msgs.append(f"CAM: {len(cam_t)} frames, median rate {1/np.median(cdt):.1f} Hz,"
                f" span {cam_t[0]:.3f}..{cam_t[-1]:.3f} s")
    if cam_t[0] < imu[0, 0] or cam_t[-1] > imu[-1, 0]:
        msgs.append("NOTE: camera span exceeds IMU span; frames outside will be dropped")
    return msgs

# ---------------------------------------------------------------- images


def make_undistort_maps(cfg: dict):
    import cv2
    c = cfg["camera"]
    K = np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]], dtype=np.float64)
    if c["model"] == "none":
        return K, None
    dist = np.array(c["distortion"], dtype=np.float64)
    size = (int(c["width"]), int(c["height"]))
    if c["model"] == "radtan":
        m1, m2 = cv2.initUndistortRectifyMap(K, dist, np.eye(3), K, size, cv2.CV_32FC1)
    else:
        m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, dist, np.eye(3), K, size, cv2.CV_32FC1)
    return K, (m1, m2)


def target_size(h0: int, w0: int, area: int = 384 * 512):
    """Same resize rule as DBA-Fusion demos: ~384*512 px area, dims cropped to multiples of 8."""
    h1 = int(h0 * math.sqrt(area / (h0 * w0)))
    w1 = int(w0 * math.sqrt(area / (h0 * w0)))
    return h1 - h1 % 8, w1 - w1 % 8, h1, w1


def preprocess_image(img_bgr: np.ndarray, maps, clahe=None):
    """Undistort -> resize -> crop to /8 -> CLAHE. Returns uint8 HxWx3 (BGR, as upstream)."""
    import cv2
    if maps is not None:
        img_bgr = cv2.remap(img_bgr, maps[0], maps[1], interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT)
    h0, w0 = img_bgr.shape[:2]
    hc, wc, h1, w1 = target_size(h0, w0)
    img = cv2.resize(img_bgr, (w1, h1))[:hc, :wc]
    if clahe is not None:                              # upstream: CLAHE on channel 0, replicated
        ch = clahe.apply(np.ascontiguousarray(img[:, :, 0]))
        img = np.repeat(ch[:, :, None], 3, axis=2)
    return img, (h0, w0, h1, w1)


def image_stream(cam_dir: str, cam_t, names, idx, cfg: dict):
    """Yield (t_s, tensor[1,3,H,W] uint8, intrinsics tensor[4]) like upstream image_stream()."""
    import cv2
    import torch
    K, maps = make_undistort_maps(cfg)
    clahe = cv2.createCLAHE(2.0, tileGridSize=(8, 8)) if cfg.get("clahe", True) else None
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    for i in idx:
        img = cv2.imread(os.path.join(cam_dir, names[i]), cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(os.path.join(cam_dir, names[i]))
        img, (h0, w0, h1, w1) = preprocess_image(img, maps, clahe)
        intr = torch.as_tensor([fx, fy, cx, cy], dtype=torch.float64)
        intr[0::2] *= (w1 / w0)
        intr[1::2] *= (h1 / h0)
        yield float(cam_t[i]), torch.as_tensor(img).permute(2, 0, 1)[None].contiguous(), intr

# ---------------------------------------------------------------- ground truth


def gt_csv_to_tum(csv_path: str, out_path: str) -> int:
    """EuRoC state_groundtruth_estimate0/data.csv (t_ns, p xyz, q wxyz, ...) -> TUM (t_s, xyz, qxyzw)."""
    rows = _read_csv_rows(csv_path, 8)
    with open(out_path, "w") as f:
        for r in rows:
            v = [float(x) for x in r[:8]]
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" %
                    (v[0] / 1e9, v[1], v[2], v[3], v[5], v[6], v[7], v[4]))
    return len(rows)

# ---------------------------------------------------------------- CLI


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--asl_dir", required=True, help=".../<seq>/asl (contains mav0/)")
    ap.add_argument("--config", required=True)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    mav = os.path.join(a.asl_dir, "mav0")
    imu = read_imu_csv(os.path.join(mav, "imu0", "data.csv"), cfg["imu"]["gyro_unit"])
    cam_t, names = read_cam_csv(os.path.join(mav, "cam0", "data.csv"))
    for m in sanity_report(imu, cam_t, cfg):
        print(m)
    idx = select_frames(cam_t, imu[:, 0], cfg["stride"])
    print(f"-> {len(idx)} frames will be processed (stride {cfg['stride']})")
    missing = [n for n in (names[i] for i in idx[:5]) if not os.path.exists(os.path.join(mav, "cam0", "data", n))]
    if missing:
        print("ERROR: image files missing, e.g.", missing[0])
        sys.exit(2)


if __name__ == "__main__":
    main()
