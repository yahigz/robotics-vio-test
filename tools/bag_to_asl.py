#!/usr/bin/env python
"""ROS1 bag -> EuRoC/ASL folder.

Output (under --out): mav0/cam0/data/<ns>.png, mav0/cam0/data.csv, [cam1 likewise],
mav0/imu0/data.csv with columns
#timestamp [ns],w_RS_S_x [rad s^-1],...,a_RS_S_z [m s^-2].
Timestamps are the header stamps. Raw and compressed images are handled;
mono8/bgr8/rgb8 are written as PNG (color as BGR->PNG RGB-correct via OpenCV).

Example:
  pixi run bag-to-asl in.bag out_dir --cam0 /cam0/image_raw --cam1 /cam1/image_raw --imu /imu0
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from _common import TYPESTORE, decode_image  # noqa: E402

import cv2  # noqa: E402
from rosbags.rosbag1 import Reader  # noqa: E402

IMU_HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
              "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")


def stamp_ns(h):
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out", help="output ASL root (mav0/ is created inside)")
    ap.add_argument("--cam0", required=True)
    ap.add_argument("--cam1", default=None)
    ap.add_argument("--imu", default=None)
    ap.add_argument("--gray", action="store_true", help="convert color images to grayscale")
    a = ap.parse_args()

    mav = Path(a.out) / "mav0"
    cams = {a.cam0: "cam0"}
    if a.cam1:
        cams[a.cam1] = "cam1"
    files = {}
    for topic, name in cams.items():
        (mav / name / "data").mkdir(parents=True, exist_ok=True)
        f = open(mav / name / "data.csv", "w")
        f.write("#timestamp [ns],filename\n")
        files[topic] = f
    imu_f = None
    if a.imu:
        (mav / "imu0").mkdir(parents=True, exist_ok=True)
        imu_f = open(mav / "imu0" / "data.csv", "w")
        imu_f.write(IMU_HEADER)

    topics = set(cams) | ({a.imu} if a.imu else set())
    counts = {t: 0 for t in topics}
    with Reader(a.bag) as r:
        avail = {c.topic for c in r.connections}
        if topics - avail:
            sys.exit(f"topics not in bag: {sorted(topics - avail)}; available: {sorted(avail)}")
        conns = [c for c in r.connections if c.topic in topics]
        for c, ts, raw in r.messages(connections=conns):
            msg = TYPESTORE.deserialize_ros1(raw, c.msgtype)
            counts[c.topic] += 1
            if c.topic in cams:
                ns = stamp_ns(msg.header)
                img = decode_image(msg, c.msgtype)
                if a.gray and img.ndim == 3:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                name = cams[c.topic]
                cv2.imwrite(str(mav / name / "data" / f"{ns}.png"), img)
                files[c.topic].write(f"{ns},{ns}.png\n")
            else:
                w, l = msg.angular_velocity, msg.linear_acceleration
                imu_f.write(f"{stamp_ns(msg.header)},{w.x!r},{w.y!r},{w.z!r},{l.x!r},{l.y!r},{l.z!r}\n")
    for f in list(files.values()) + ([imu_f] if imu_f else []):
        f.close()
    print("messages written:", counts)


if __name__ == "__main__":
    main()
