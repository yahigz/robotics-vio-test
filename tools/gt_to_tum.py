#!/usr/bin/env python
"""Convert ground truth to TUM format ("t tx ty tz qx qy qz qw", t in seconds).

Subcommands:
  euroc  EuRoC state_groundtruth_estimate0/data.csv (ns, p_xyz, q_wxyz, ...)
  csv    generic CSV; give 0-based column indices (--t-col, --pos-cols x y z,
         --quat-cols ...) with --quat-order xyzw|wxyz and --t-scale (to seconds)
  bag    PoseStamped / PoseWithCovarianceStamped / Odometry / TransformStamped topic
Extend by adding a function returning an (N,8) array and registering it below.

Example:
  pixi run gt-to-tum euroc data/.../mav0/state_groundtruth_estimate0/data.csv out.txt
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))


def save_tum(path, arr):
    arr = arr[np.argsort(arr[:, 0], kind="stable")]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, arr, fmt="%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f")
    print(f"wrote {len(arr)} poses to {path}  (t: {arr[0,0]:.3f} .. {arr[-1,0]:.3f} s)")


def read_csv_cols(path, delim=",", skip_header=True):
    return np.genfromtxt(path, delimiter=delim, comments="#", skip_header=0)


def from_cols(data, t_col, pos_cols, quat_cols, quat_order="xyzw", t_scale=1.0):
    t = data[:, t_col] * t_scale
    p = data[:, pos_cols]
    q = data[:, quat_cols]
    if quat_order == "wxyz":
        q = q[:, [1, 2, 3, 0]]
    return np.column_stack([t, p, q])


def from_euroc(a):
    d = read_csv_cols(a.input)
    # ns, px py pz, qw qx qy qz
    return from_cols(d, 0, [1, 2, 3], [4, 5, 6, 7], "wxyz", 1e-9)


def from_csv(a):
    d = read_csv_cols(a.input, a.delimiter)
    return from_cols(d, a.t_col, a.pos_cols, a.quat_cols, a.quat_order, a.t_scale)


def from_bag(a):
    from _common import TYPESTORE
    from rosbags.rosbag1 import Reader
    rows = []
    with Reader(a.input) as r:
        conns = [c for c in r.connections if c.topic == a.topic]
        if not conns:
            sys.exit(f"topic {a.topic} not in bag; have {sorted({c.topic for c in r.connections})}")
        for c, ts, raw in r.messages(connections=conns):
            m = TYPESTORE.deserialize_ros1(raw, c.msgtype)
            h = m.header
            t = h.stamp.sec + h.stamp.nanosec * 1e-9
            if hasattr(m, "pose") and hasattr(m.pose, "pose"):      # Odometry / PoseWithCovarianceStamped
                pose = m.pose.pose
            elif hasattr(m, "pose"):                                # PoseStamped
                pose = m.pose
            elif hasattr(m, "transform"):                           # TransformStamped
                pose = m.transform
                pose.position = pose.translation
                pose.orientation = pose.rotation
            else:
                sys.exit(f"unsupported message type {c.msgtype}")
            p, q = pose.position, pose.orientation
            rows.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("euroc"); p.add_argument("input"); p.add_argument("output"); p.set_defaults(fn=from_euroc)
    p = sub.add_parser("csv"); p.add_argument("input"); p.add_argument("output")
    p.add_argument("--delimiter", default=",")
    p.add_argument("--t-col", type=int, default=0)
    p.add_argument("--t-scale", type=float, default=1.0, help="multiply time column to get seconds")
    p.add_argument("--pos-cols", type=int, nargs=3, default=[1, 2, 3])
    p.add_argument("--quat-cols", type=int, nargs=4, default=[4, 5, 6, 7])
    p.add_argument("--quat-order", choices=["xyzw", "wxyz"], default="xyzw")
    p.set_defaults(fn=from_csv)
    p = sub.add_parser("bag"); p.add_argument("input"); p.add_argument("output")
    p.add_argument("--topic", required=True); p.set_defaults(fn=from_bag)
    a = ap.parse_args()
    save_tum(a.output, a.fn(a))


if __name__ == "__main__":
    main()
