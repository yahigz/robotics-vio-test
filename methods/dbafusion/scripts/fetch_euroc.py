"""Download + unpack EuRoC (ASL format) into <data_root>/euroc/<seq>/{raw,asl,gt_tum.txt}.
Idempotent. Used only for the optional sanity-check cell in the Colab notebook.
The official EuRoC mirror may move (ETH Research Collection); override --url if needed.
"""
import argparse
import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asl_to_dbafusion import gt_csv_to_tum

DEFAULT = "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/{group}/{seq}/{seq}.zip"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data_root", required=True)
    ap.add_argument("--seq", default="MH_01_easy")
    ap.add_argument("--url", default=None)
    a = ap.parse_args()
    group = "machine_hall" if a.seq.startswith("MH") else "vicon_room1" if a.seq.startswith("V1") else "vicon_room2"
    url = a.url or DEFAULT.format(group=group, seq=a.seq)
    base = os.path.join(a.data_root, "euroc", a.seq)
    asl = os.path.join(base, "asl")
    gt = os.path.join(base, "gt_tum.txt")
    if os.path.isdir(os.path.join(asl, "mav0", "cam0", "data")) and os.path.exists(gt):
        print("already present:", base)
        return
    raw = os.path.join(base, "raw")
    os.makedirs(raw, exist_ok=True)
    zp = os.path.join(raw, a.seq + ".zip")
    if not os.path.exists(zp):
        subprocess.run(["wget", "-q", "--show-progress", "-O", zp + ".part", url], check=True)
        os.rename(zp + ".part", zp)
    tmp = asl + ".tmp"
    shutil.rmtree(tmp, ignore_errors=True)
    subprocess.run(["unzip", "-q", zp, "-d", tmp], check=True)
    shutil.rmtree(asl, ignore_errors=True)
    os.rename(tmp, asl)
    n = gt_csv_to_tum(os.path.join(asl, "mav0", "state_groundtruth_estimate0", "data.csv"), gt)
    print(f"ready: {base} (gt poses: {n})")


if __name__ == "__main__":
    main()
