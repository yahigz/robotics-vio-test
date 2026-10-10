"""Convert DBA-Fusion's raw result file to a clean TUM trajectory (CONTRACT.md).

Raw format written by dbaf_frontend.py: `t x y z qx qy qz qw` per line (t in seconds,
pose of the IMU/body frame in a gravity-aligned, scale-metric world frame), optionally 3
extra GNSS columns. The file is appended to from several places (initialisation block,
per-keyframe line, high-frequency IMU-propagated lines at every camera frame), so lines
may be unsorted or duplicated; we sort, keep the LAST estimate per timestamp, drop
non-finite rows and renormalise quaternions.
"""
import argparse
import numpy as np


def convert(raw_path: str, out_path: str, min_poses: int = 10) -> dict:
    rows = []
    with open(raw_path) as f:
        for line in f:
            p = line.split()
            if len(p) < 8:
                continue
            try:
                v = [float(x) for x in p[:8]]
            except ValueError:
                continue
            rows.append(v)
    if not rows:
        raise RuntimeError(f"no poses in {raw_path}: DBA-Fusion never initialised (see log)")
    a = np.array(rows)
    n_raw = len(a)
    a = a[np.all(np.isfinite(a), axis=1)]
    # keep last occurrence per timestamp (rounded to 1 us), then sort
    key = np.round(a[:, 0] * 1e6).astype(np.int64)
    _, last = np.unique(key[::-1], return_index=True)
    a = a[::-1][last]
    a = a[np.argsort(a[:, 0], kind="stable")]
    q = a[:, 4:8]
    nrm = np.linalg.norm(q, axis=1)
    good = nrm > 0.5
    a, q, nrm = a[good], q[good], nrm[good]
    a[:, 4:8] = q / nrm[:, None]
    if len(a) < min_poses:
        raise RuntimeError(f"only {len(a)} valid poses in {raw_path}")
    with open(out_path, "w") as f:
        for r in a:
            f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" % tuple(r))
    return {"n_raw": n_raw, "n_out": len(a), "t_first": float(a[0, 0]), "t_last": float(a[-1, 0])}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("raw")
    ap.add_argument("out")
    a = ap.parse_args()
    print(convert(a.raw, a.out))
