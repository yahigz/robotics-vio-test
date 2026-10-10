#!/usr/bin/env python
"""Self-test: perturb GT (rigid offset + scale-free drift + noise) and run evaluate.py.

Example: pixi run test-eval [--gt data/euroc/MH_01_easy/gt_tum.txt]
Falls back to a synthetic circular GT if the file does not exist.
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def synthetic_gt(n=4000, hz=200.0):
    t = np.arange(n) / hz
    th = 2 * np.pi * t / t[-1] * 2
    p = np.c_[8 * np.cos(th), 5 * np.sin(th), 0.5 * np.sin(3 * th)]
    q = Rot.from_rotvec(np.c_[0 * th, 0 * th, th + np.pi / 2]).as_quat()
    return np.c_[t, p, q]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", default="data/euroc/MH_01_easy/gt_tum.txt")
    ap.add_argument("--out-root", default="results/summary")
    ap.add_argument("--dataset", default="selftest")
    ap.add_argument("--seq", default="synthetic")
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    if Path(a.gt).exists():
        gt = np.loadtxt(a.gt)
        print(f"using GT {a.gt}")
    else:
        gt = synthetic_gt()
        print("GT file missing, using synthetic GT")
    # estimate at 20 Hz, starting 5 s late, missing the last 3 s
    sel = (gt[:, 0] > gt[0, 0] + 5) & (gt[:, 0] < gt[-1, 0] - 3)
    g = gt[sel][:: max(1, int(len(gt) / (gt[-1, 0] - gt[0, 0]) / 20))]
    t = g[:, 0]
    tt = (t - t[0]) / (t[-1] - t[0])
    # yaw drift 3 deg total, position drift ~1% (linear ramp) + noise
    drift = np.c_[0.4 * tt ** 2, -0.3 * tt, 0.1 * tt] * 0.15 * np.ptp(g[:, 1:4], axis=0).max()
    p = g[:, 1:4] + drift + rng.normal(0, 0.01, (len(g), 3))
    q = (Rot.from_rotvec(np.outer(tt, [0, 0, np.radians(3)])) * Rot.from_quat(g[:, 4:8])).as_quat()
    # arbitrary world frame
    W = Rot.from_euler("xyz", [10, -5, 40], degrees=True)
    p = W.apply(p) + [3.0, -2.0, 1.0]
    q = (W * Rot.from_quat(q)).as_quat()
    est = np.c_[t, p, q]
    d = Path(tempfile.mkdtemp(prefix="vio_selftest_"))
    gt_path = d / "gt.txt"
    if not Path(a.gt).exists():
        np.savetxt(gt_path, gt, fmt="%.9f")
    est_path = d / "est.txt"
    np.savetxt(est_path, est, fmt="%.9f")
    gt_arg = a.gt if Path(a.gt).exists() else str(gt_path)
    cmd = [sys.executable, str(Path(__file__).parent / "evaluate.py"), "--gt", gt_arg,
           "--est", f"perturbed={est_path}",
           "--dataset", a.dataset, "--seq", a.seq, "--out-root", a.out_root]
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
