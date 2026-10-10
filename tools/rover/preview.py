#!/usr/bin/env python3
"""Sanity plots for a converted ROVER sequence: data/rover/<seq>/preview/{gt_xy,imu_accel_norm,time_overlap}.png.
Usage: .venv/bin/python tools/rover/preview.py <seq>   (prints stats as JSON)"""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

seq = sys.argv[1]
sd = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "rover", seq))
pv = os.path.join(sd, "preview"); os.makedirs(pv, exist_ok=True)
gt = np.loadtxt(os.path.join(sd, "gt_tum.txt"))
imu = np.loadtxt(os.path.join(sd, "asl/mav0/imu0/data.csv"), delimiter=",", comments="#")
cam = np.array([int(l.split(",")[0]) for l in open(os.path.join(sd, "asl/mav0/cam0/data.csv")) if not l.startswith("#")]) / 1e9
ti = imu[:, 0] / 1e9; t0 = cam[0]
L = float(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum())
fig, ax = plt.subplots(figsize=(6, 6)); sc = ax.scatter(gt[:, 1], gt[:, 2], c=gt[:, 0] - t0, s=6, cmap="viridis")
ax.plot(gt[:, 1], gt[:, 2], lw=0.4, c="gray"); ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title(f"{seq}: GT prism xy, {L:.0f} m"); plt.colorbar(sc, label="t - t_cam0 [s]"); fig.savefig(os.path.join(pv, "gt_xy.png"), dpi=110, bbox_inches="tight")
fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
ax[0].plot(ti - t0, np.linalg.norm(imu[:, 4:7], axis=1), lw=0.3); ax[0].axhline(9.81, c="r", lw=0.6); ax[0].set_ylabel("|acc| [m/s^2]")
ax[1].plot(ti - t0, np.linalg.norm(imu[:, 1:4], axis=1), lw=0.3); ax[1].set_ylabel("|gyro| [rad/s]"); ax[1].set_xlabel("t - t_cam0 [s]")
fig.savefig(os.path.join(pv, "imu_accel_norm.png"), dpi=110, bbox_inches="tight")
fig, ax = plt.subplots(figsize=(9, 1.8))
for i, (n, (a, b)) in enumerate({"cam": (cam[0], cam[-1]), "imu": (ti[0], ti[-1]), "gt": (gt[0, 0], gt[-1, 0])}.items()):
    ax.plot([a - t0, b - t0], [i, i], lw=6); ax.text(a - t0, i + 0.2, n)
ax.set_yticks([]); ax.set_xlabel("t - t_cam0 [s]"); fig.savefig(os.path.join(pv, "time_overlap.png"), dpi=110, bbox_inches="tight")
dtg = np.diff(gt[:, 0])
print(json.dumps(dict(seq=seq, gt_path_length_m=L, gt_duration_s=float(gt[-1, 0] - gt[0, 0]), gt_rate_hz=float(1 / np.median(dtg)),
                      gt_dt_max_s=float(dtg.max()), cam_duration_s=float(cam[-1] - cam[0]), imu_duration_s=float(ti[-1] - ti[0]),
                      gt_minus_cam0_start_s=float(gt[0, 0] - cam[0]), gt_end_minus_cam_end_s=float(gt[-1, 0] - cam[-1]),
                      gt_extent_xyz_m=(gt[:, 1:4].max(0) - gt[:, 1:4].min(0)).tolist(),
                      acc_norm_mean=float(np.linalg.norm(imu[:, 4:7], axis=1).mean()), acc_norm_std=float(np.linalg.norm(imu[:, 4:7], axis=1).std())), indent=1))
