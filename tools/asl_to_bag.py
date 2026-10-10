#!/usr/bin/env python
"""EuRoC/ASL folder -> ROS1 bag (cam0[/cam1] sensor_msgs/Image mono8 + imu0 sensor_msgs/Imu).

Fallback for when the official .bag is unavailable. Streaming, low memory.
Example:
  pixi run asl-to-bag data/euroc/MH_01_easy/asl out.bag [--cam1]
Topics: /cam0/image_raw, /cam1/image_raw, /imu0 (same as the official EuRoC bags).
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from _common import TYPESTORE as T  # noqa: E402
from rosbags.rosbag1 import Writer  # noqa: E402


def read_csv(p):
    return np.genfromtxt(p, delimiter=",", comments="#", dtype=None, encoding=None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("asl", help="folder containing mav0/")
    ap.add_argument("output")
    ap.add_argument("--cam1", action="store_true", help="also write cam1")
    ap.add_argument("--frame-id", default="")
    a = ap.parse_args()
    mav = Path(a.asl) / "mav0"
    Header, Time = T.types["std_msgs/msg/Header"], T.types["builtin_interfaces/msg/Time"]
    Image, Imu = T.types["sensor_msgs/msg/Image"], T.types["sensor_msgs/msg/Imu"]
    V3, Quat = T.types["geometry_msgs/msg/Vector3"], T.types["geometry_msgs/msg/Quaternion"]
    z9 = np.zeros(9)

    def hdr(i, ns):
        return Header(seq=i, stamp=Time(sec=ns // 10**9, nanosec=ns % 10**9), frame_id=a.frame_id)

    events = []  # (ns, kind, payload)
    cams = ["cam0"] + (["cam1"] if a.cam1 else [])
    for cam in cams:
        for ns, fn in read_csv(mav / cam / "data.csv"):
            events.append((int(ns), cam, fn))
    imu = np.loadtxt(mav / "imu0" / "data.csv", delimiter=",", comments="#")
    for r in imu:
        events.append((int(r[0]), "imu0", r[1:]))
    events.sort(key=lambda e: e[0])

    out = Path(a.output)
    if out.exists():
        out.unlink()
    with Writer(out) as w:
        conn = {c: w.add_connection(f"/{c}/image_raw", "sensor_msgs/msg/Image", typestore=T) for c in cams}
        conn["imu0"] = w.add_connection("/imu0", "sensor_msgs/msg/Imu", typestore=T)
        seq = {k: 0 for k in conn}
        for ns, kind, p in events:
            seq[kind] += 1
            if kind == "imu0":
                msg = Imu(header=hdr(seq[kind], ns), orientation=Quat(x=0., y=0., z=0., w=0.),
                          orientation_covariance=np.array([-1.0] + [0.0] * 8),
                          angular_velocity=V3(x=p[0], y=p[1], z=p[2]), angular_velocity_covariance=z9,
                          linear_acceleration=V3(x=p[3], y=p[4], z=p[5]), linear_acceleration_covariance=z9)
                w.write(conn[kind], ns, T.serialize_ros1(msg, "sensor_msgs/msg/Imu"))
            else:
                img = cv2.imread(str(mav / kind / "data" / p), cv2.IMREAD_GRAYSCALE)
                msg = Image(header=hdr(seq[kind], ns), height=img.shape[0], width=img.shape[1],
                            encoding="mono8", is_bigendian=0, step=img.shape[1],
                            data=img.reshape(-1))
                w.write(conn[kind], ns, T.serialize_ros1(msg, "sensor_msgs/msg/Image"))
    print(f"wrote {out}: {seq}")


if __name__ == "__main__":
    main()
