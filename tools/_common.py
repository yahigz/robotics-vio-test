"""Shared helpers for bag tools (rosbags, no ROS needed)."""
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)


def decode_image(msg, msgtype):
    """Return (np.ndarray HxW or HxWx3, encoding) from Image/CompressedImage msg.
    Color arrays are returned in BGR order (OpenCV convention)."""
    import cv2
    import numpy as np

    if msgtype.endswith("CompressedImage"):
        arr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
        if arr is None:
            raise ValueError("cv2.imdecode failed")
        return arr
    enc = msg.encoding
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    h, w = msg.height, msg.width
    if enc in ("mono8", "8UC1"):
        return buf.reshape(h, msg.step)[:, :w].copy()
    if enc in ("mono16", "16UC1"):
        return np.frombuffer(bytes(msg.data), np.uint16).reshape(h, msg.step // 2)[:, :w].copy()
    if enc in ("bgr8", "rgb8"):
        arr = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return arr[:, :, ::-1].copy() if enc == "rgb8" else arr.copy()
    if enc in ("bgra8", "rgba8"):
        arr = buf.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        return arr[:, :, ::-1].copy() if enc == "rgba8" else arr.copy()
    raise ValueError(f"unsupported image encoding {enc}")
