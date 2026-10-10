#!/usr/bin/env python3
"""Rectify ROVER T265 cam0 (equidistant fisheye) to an ideal pinhole image, on demand, into a SEPARATE folder.

Needed for DBA-Fusion (pinhole only). The output is NOT kept permanently by default: a rectified PNG sequence is
~1 GB-3 GB, which can break the 5 GB/sequence limit together with the original. The tool therefore refuses to start
if (current size of data/rover/<seq>) + (projected output size) > --budget-gb (default 5), where the projection is
measured on the first 30 frames; use --t-start/--duration/--stride to write a subset, or --jpeg (not recommended).

Usage:
  .venv/bin/python tools/rover/undistort_asl.py data/rover/<seq> [--size 640 480] [--hfov 100]
        [--out data/rover/<seq>/undist] [--t-start S] [--duration S] [--stride N]
Output: <out>/mav0/cam0/data/<ns>.png (mono8), <out>/mav0/cam0/data.csv, <out>/mav0/imu0/data.csv (copy, ~10 MB),
        <out>/intrinsics.yaml (fx fy cx cy of the rectified pinhole; also what make_configs.py writes into the DBA config).
Rectification: R = I, so the camera frame (and T_imu_cam) is unchanged. hfov = horizontal field of view of the
rectified image: fx = (w/2)/tan(hfov/2), cx=(w-1)/2, cy=(h-1)/2 (must match make_configs.py --rect-size/--rect-hfov).
Delete the folder afterwards:  rm -rf <out>
"""
import argparse, os, shutil, sys
import numpy as np, cv2, yaml
import rovercalib as rc


def dir_size(p):
    return sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(p) for f in fs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seqdir"); ap.add_argument("--size", type=int, nargs=2, default=[640, 480])
    ap.add_argument("--hfov", type=float, default=100.0); ap.add_argument("--out", default=None)
    ap.add_argument("--t-start", type=float, default=0.0); ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--stride", type=int, default=1); ap.add_argument("--budget-gb", type=float, default=5.0)
    ap.add_argument("--force", action="store_true", help="ignore the budget check")
    a = ap.parse_args()
    c = rc.load(); cam = c["cam0"]
    r = rc.rectified(c, tuple(a.size), a.hfov)
    K = np.array([[cam["intrinsics"][0], 0, cam["intrinsics"][2]], [0, cam["intrinsics"][1], cam["intrinsics"][3]], [0, 0, 1]])
    D = np.array(cam["distortion"])
    P = np.array([[r["fx"], 0, r["cx"]], [0, r["fy"], r["cy"]], [0, 0, 1]])
    m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), P, tuple(a.size), cv2.CV_16SC2)
    mav = os.path.join(a.seqdir, "asl", "mav0"); out = a.out or os.path.join(a.seqdir, "undist")
    rows = [l.strip().split(",") for l in open(os.path.join(mav, "cam0", "data.csv")) if not l.startswith("#") and l.strip()]
    t0 = int(rows[0][0]) + int(a.t_start * 1e9)
    t1 = t0 + int(a.duration * 1e9) if a.duration else int(rows[-1][0]) + 1
    rows = [x for i, x in enumerate(rows) if t0 <= int(x[0]) < t1 and i % a.stride == 0]
    od = os.path.join(out, "mav0", "cam0", "data"); os.makedirs(od, exist_ok=True)
    cur = dir_size(a.seqdir)
    sizes = []
    lines = ["#timestamp [ns],filename"]
    for i, (ns, fn) in enumerate(rows):
        img = cv2.imread(os.path.join(mav, "cam0", "data", fn), cv2.IMREAD_GRAYSCALE)
        und = cv2.remap(img, m1, m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        p = os.path.join(od, fn); cv2.imwrite(p, und)
        sizes.append(os.path.getsize(p)); lines.append(f"{ns},{fn}")
        if i == 29 and not a.force:
            proj = np.mean(sizes) * len(rows) + cur
            print(f"projected total {proj/1e9:.2f} GB (current {cur/1e9:.2f} GB, rectified ~{np.mean(sizes)/1e3:.0f} KB/frame)")
            if proj > a.budget_gb * 1e9:
                shutil.rmtree(out); sys.exit(f"refusing: projected {proj/1e9:.2f} GB > budget {a.budget_gb} GB; use --duration/--stride")
    open(os.path.join(out, "mav0", "cam0", "data.csv"), "w").write("\n".join(lines) + "\n")
    os.makedirs(os.path.join(out, "mav0", "imu0"), exist_ok=True)
    shutil.copy(os.path.join(mav, "imu0", "data.csv"), os.path.join(out, "mav0", "imu0", "data.csv"))
    yaml.safe_dump(dict(model="pinhole", **{k: r[k] for k in ("width", "height", "fx", "fy", "cx", "cy", "hfov_deg")}, distortion=[0, 0, 0, 0],
                        note="R=I rectification of cam0 equidistant; T_imu_cam unchanged"), open(os.path.join(out, "intrinsics.yaml"), "w"))
    print(f"wrote {len(rows)} frames to {out} ({sum(sizes)/1e9:.2f} GB); K = fx {r['fx']:.3f} cx {r['cx']} cy {r['cy']}")


if __name__ == "__main__":
    main()
