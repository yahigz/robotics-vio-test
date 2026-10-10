#!/usr/bin/env python3
"""Create a ROS1 bag (mono8 Image + Imu, lz4 chunks) from an ASL/EuRoC sequence ON DEMAND.

ROVER mono8 848x800 raw images are ~0.68 MB/frame (~1.2 GB/min raw), so a full-sequence bag would break the
5 GB/sequence limit. Therefore NEVER keep the bag under data/rover/: write it to a temporary path and delete it afterwards.

Usage:
  .venv/bin/python tools/rover/asl_to_bag.py data/rover/<seq>/asl /tmp/x.bag [--t-start 0] [--duration 30] [--stride 1]
Topics: /cam0/image_raw (sensor_msgs/Image, mono8, frame_id cam0), /imu0 (sensor_msgs/Imu, frame_id imu0;
linear_acceleration m/s^2, angular_velocity rad/s; orientation_covariance[0] = -1 i.e. no orientation).
Header stamps = ASL timestamps (unix time, same clock as gt_tum.txt); bag record time = stamp.
--t-start/--duration are seconds relative to the first camera frame.
"""
import argparse, csv, os, sys, time
import numpy as np
import cv2
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asl"); ap.add_argument("bag")
    ap.add_argument("--t-start", type=float, default=0.0); ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--cam-topic", default="/cam0/image_raw"); ap.add_argument("--imu-topic", default="/imu0")
    a = ap.parse_args()
    ts = get_typestore(Stores.ROS1_NOETIC)
    Image, Imu = ts.types["sensor_msgs/msg/Image"], ts.types["sensor_msgs/msg/Imu"]
    Header, Time = ts.types["std_msgs/msg/Header"], ts.types["builtin_interfaces/msg/Time"]
    Vec3, Quat = ts.types["geometry_msgs/msg/Vector3"], ts.types["geometry_msgs/msg/Quaternion"]
    mav = os.path.join(a.asl, "mav0")
    cams = [l.strip().split(",") for l in open(os.path.join(mav, "cam0", "data.csv")) if not l.startswith("#") and l.strip()]
    cam_ns = np.array([int(c[0]) for c in cams]); t0 = cam_ns[0] + int(a.t_start * 1e9)
    t1 = t0 + int(a.duration * 1e9) if a.duration else cam_ns[-1] + 1
    imu = []  # (ns:int exact, [wx wy wz ax ay az]); float64 would lose ns precision
    for l in open(os.path.join(mav, "imu0", "data.csv")):
        if l.startswith("#") or not l.strip(): continue
        p = l.strip().split(",")
        ns = int(p[0])
        if t0 - 50_000_000 <= ns <= t1 + 50_000_000:  # 50 ms margin each side
            imu.append((ns, [float(x) for x in p[1:7]]))
    items = [(ns, 1, [0.0] + v) for ns, v in imu] + [(int(c[0]), 0, c[1]) for i, c in enumerate(cams)
                                                  if t0 <= int(c[0]) < t1 and i % a.stride == 0]
    items.sort(key=lambda x: (x[0], x[1]))
    if os.path.exists(a.bag): os.remove(a.bag)
    mk = lambda ns: Time(sec=ns // 10**9, nanosec=ns % 10**9)
    n_i = n_c = 0; t_wall = time.time()
    w = Writer(a.bag)
    w.set_compression(w.CompressionFormat.LZ4)
    with w:
        cc = w.add_connection(a.cam_topic, "sensor_msgs/msg/Image", typestore=ts)
        ci = w.add_connection(a.imu_topic, "sensor_msgs/msg/Imu", typestore=ts)
        cov0 = np.zeros(9); covn = np.zeros(9); covn[0] = -1.0
        for ns, kind, payload in items:
            if kind == 0:
                img = cv2.imread(os.path.join(mav, "cam0", "data", payload), cv2.IMREAD_GRAYSCALE)
                msg = Image(header=Header(seq=n_c, stamp=mk(ns), frame_id="cam0"), height=img.shape[0], width=img.shape[1],
                            encoding="mono8", is_bigendian=0, step=img.shape[1], data=img.reshape(-1))
                w.write(cc, ns, ts.serialize_ros1(msg, Image.__msgtype__)); n_c += 1
            else:
                r = payload
                msg = Imu(header=Header(seq=n_i, stamp=mk(ns), frame_id="imu0"),
                          orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=covn,
                          angular_velocity=Vec3(x=r[1], y=r[2], z=r[3]), angular_velocity_covariance=cov0,
                          linear_acceleration=Vec3(x=r[4], y=r[5], z=r[6]), linear_acceleration_covariance=cov0)
                w.write(ci, ns, ts.serialize_ros1(msg, Imu.__msgtype__)); n_i += 1
    sz = os.path.getsize(a.bag); dur = (min(t1, cam_ns[-1]) - t0) / 1e9
    print(f"bag {a.bag}: {n_c} images, {n_i} imu msgs, {sz/1e6:.1f} MB for {dur:.1f} s -> {sz/1e6/dur*60:.0f} MB/min "
          f"(full-sequence estimate {sz/dur*(cam_ns[-1]-cam_ns[0])/1e9/1e9:.2f} GB); wrote in {time.time()-t_wall:.0f}s")


if __name__ == "__main__":
    main()
