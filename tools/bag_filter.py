#!/usr/bin/env python
"""Write a new ROS1 bag containing only the given topics (streaming, low memory).

With --decompress, sensor_msgs/CompressedImage topics are re-encoded as
sensor_msgs/Image (mono8 for gray, bgr8 for color) on the same topic name
(a trailing "/compressed" is stripped from the topic name; --suffix is appended).

Example:
  pixi run bag-filter in.bag out.bag --topics /cam0/image_raw /imu0
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from _common import TYPESTORE, decode_image  # noqa: E402

from rosbags.rosbag1 import Reader, Writer  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--topics", nargs="+", required=True, help="topics to keep")
    ap.add_argument("--decompress", action="store_true", help="CompressedImage -> Image")
    ap.add_argument("--suffix", default="", help="suffix appended to decompressed topic names")
    ap.add_argument("--compression", default="none", choices=["none", "bz2", "lz4"],
                    help="output chunk compression (default none, fastest to read)")
    a = ap.parse_args()

    out = Path(a.output)
    if out.exists():
        out.unlink()
    n = 0
    with Reader(a.input) as r:
        avail = {c.topic for c in r.connections}
        missing = [t for t in a.topics if t not in avail]
        if missing:
            sys.exit(f"topics not in bag: {missing}\navailable: {sorted(avail)}")
        conns = [c for c in r.connections if c.topic in a.topics]
        total = sum(c.msgcount for c in conns)
        w = Writer(out)
        if a.compression != "none":
            w.set_compression(getattr(Writer.CompressionFormat, a.compression.upper()))
        with w:
            wmap, conv = {}, {}
            for c in conns:
                if a.decompress and c.msgtype == "sensor_msgs/msg/CompressedImage":
                    wmap[c.id] = w.add_connection(c.topic.removesuffix("/compressed") + a.suffix, "sensor_msgs/msg/Image",
                                                  typestore=TYPESTORE)
                    conv[c.id] = c
                else:
                    wmap[c.id] = w.add_connection(
                        c.topic, c.msgtype, msgdef=c.msgdef.data, md5sum=c.digest,
                        callerid=getattr(c.ext, "callerid", None),
                        latching=getattr(c.ext, "latching", None))
            try:
                from tqdm import tqdm
                it = tqdm(r.messages(connections=conns), total=total, unit="msg")
            except ImportError:
                it = r.messages(connections=conns)
            for c, ts, raw in it:
                if c.id in conv:
                    msg = TYPESTORE.deserialize_ros1(raw, c.msgtype)
                    img = decode_image(msg, c.msgtype)
                    Image = TYPESTORE.types["sensor_msgs/msg/Image"]
                    enc = "mono8" if img.ndim == 2 else "bgr8"
                    if img.dtype.itemsize == 2:
                        enc = "mono16"
                    import numpy as np
                    out_msg = Image(header=msg.header, height=img.shape[0], width=img.shape[1],
                                    encoding=enc, is_bigendian=0,
                                    step=img.shape[1] * img.itemsize * (1 if img.ndim == 2 else 3),
                                    data=np.frombuffer(img.tobytes(), np.uint8))
                    raw = TYPESTORE.serialize_ros1(out_msg, "sensor_msgs/msg/Image")
                w.write(wmap[c.id], ts, bytes(raw))
                n += 1
    print(f"wrote {n} messages to {out}")


if __name__ == "__main__":
    main()
