#!/usr/bin/env python3
"""Convert a fetched ROVER T265 subset (data/rover/<seq>/raw) to the contract layout.

Usage: .venv/bin/python tools/rover/convert_asl.py <seq> [--keep-raw]

Produces in data/rover/<seq>/:
  asl/mav0/cam0/data/<ns>.png, asl/mav0/cam0/data.csv     (PNG files are MOVED, no copy)
  asl/mav0/imu0/data.csv    "#timestamp [ns],w_x,w_y,w_z [rad/s],a_x,a_y,a_z [m/s^2]"
  gt_tum.txt                t[s] x y z 0 0 0 1   (prism position only, identity quaternion!)
  manifest.json             counts, sizes, rates, gaps, sha256 of text outputs, CRC32 digest of PNGs

Facts about the raw files (verified on the data):
  * imu.txt columns are  t, ax, ay, az, gx, gy, gz  (accel m/s^2 first, then gyro rad/s).
    Gravity shows up on +y (~9.8), so specific force, y up. Stamps are unix seconds.
  * imu.txt is the T265 accel (62.5 Hz) + gyro (200 Hz) merge: it already contains both channels
    at the union of timestamps (the other channel linearly interpolated), ~264 rows/s with many
    near-duplicate rows (dt << 1 ms). We drop any row closer than 2.5 ms to the last kept row,
    leaving a clean ~200 Hz stream (no extra accel interpolation needed).
  * Camera frames after the last IMU sample (the file ends ~1 s before the images) are dropped
    (deleted) so every frame has inertial data; the count is stored in the manifest.
"""
import argparse, hashlib, json, os, shutil, sys, zlib
import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data", "rover"))


def to_ns(s):
    s = s.strip()
    if "." in s:
        a, b = s.split(".")
    else:
        a, b = s, ""
    return int(a) * 10**9 + int((b + "0" * 9)[:9])


def stamp_ns(fname):
    """PNG name -> ns; handles <t>.png and left_img_<idx>_<t>.png (park sequence)."""
    return to_ns(os.path.basename(fname)[:-4].split("_")[-1])


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""): h.update(b)
    return h.hexdigest()


def stats_dt(t, nominal):
    dt = np.diff(t)
    return dict(n=int(len(t)), duration_s=float(t[-1] - t[0]), mean_rate_hz=float((len(t) - 1) / (t[-1] - t[0])),
                dt_min=float(dt.min()), dt_median=float(np.median(dt)), dt_max=float(dt.max()),
                n_gaps_gt_1p5x_nominal=int((dt > 1.5 * nominal).sum()),
                gaps_s=[[float(t[i]), float(dt[i])] for i in np.where(dt > 1.5 * nominal)[0][:20]])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("seq"); ap.add_argument("--keep-raw", action="store_true")
    ap.add_argument("--allow-incomplete", action="store_true", help="fetch was interrupted: convert the contiguous prefix of frames that exist")
    a = ap.parse_args()
    sd = os.path.join(ROOT, a.seq); raw = os.path.join(sd, "raw")
    dec = json.load(open(os.path.join(raw, "decision.json")))
    if not os.path.exists(os.path.join(raw, "FETCH_DONE")):
        assert a.allow_incomplete, "fetch not finished (use --allow-incomplete for a prefix)"
        dec["max_seconds"] = dec.get("max_seconds") or -1; dec["interrupted"] = True
    # --- IMU
    rows = [l.split(",") for l in open(os.path.join(raw, "imu.txt")) if l.strip() and not l.startswith("#")]
    tns = np.array([to_ns(r[0]) for r in rows], dtype=np.int64)
    vals = np.array([[float(x) for x in r[1:7]] for r in rows])
    assert (np.diff(tns) > 0).all()
    keep = [0]
    for i in range(1, len(tns)):
        if tns[i] - tns[keep[-1]] >= 2_500_000: keep.append(i)
    keep = np.array(keep)
    t_imu, v = tns[keep], vals[keep]
    acc, gyr = v[:, 0:3], v[:, 3:6]
    imu_raw_stats = stats_dt(tns / 1e9, 0.005)
    imu_stats = stats_dt(t_imu / 1e9, 0.005)
    # --- camera. File names carry the full-precision stamp (cam_left.txt is rounded to 6-7 digits, differing by <1 us
    # for some frames), so the PNG names are authoritative; cam_left.txt is only used as a count check.
    n_txt = sum(1 for l in open(os.path.join(raw, "cam_left.txt")) if l.strip())
    avail = {stamp_ns(f): f for f in os.listdir(os.path.join(raw, "cam_left")) if f.endswith(".png")}
    ddir = os.path.join(sd, "asl", "mav0", "cam0", "data")
    done = {int(f[:-4]) for f in os.listdir(ddir)} if os.path.isdir(ddir) else set()  # idempotent re-run
    cam_ns = np.array(sorted(set(avail) | done), dtype=np.int64)
    partial = dec.get("max_seconds") is not None  # time-ordered prefix fetched on a slow link
    if not partial:
        assert len(cam_ns) == n_txt, (len(cam_ns), n_txt)
    names = [str(x) for x in cam_ns]
    if partial:  # keep inertial data only up to 0.5 s after the last image we have
        m = t_imu <= cam_ns[-1] + 500_000_000
        t_imu, v = t_imu[m], v[m]; acc, gyr = v[:, 0:3], v[:, 3:6]
        imu_stats = stats_dt(t_imu / 1e9, 0.005)
    ok = (cam_ns >= t_imu[0]) & (cam_ns <= t_imu[-1])
    asl = os.path.join(sd, "asl", "mav0")
    os.makedirs(os.path.join(asl, "cam0", "data"), exist_ok=True); os.makedirs(os.path.join(asl, "imu0"), exist_ok=True)
    n_moved = n_drop = 0; png_bytes = 0; crcs = 0
    lines = ["#timestamp [ns],filename"]
    kept_ns = []
    avail = {stamp_ns(f): f for f in os.listdir(os.path.join(raw, "cam_left")) if f.endswith(".png")}
    for n, ns, good in zip(names, cam_ns, ok):
        dst = os.path.join(asl, "cam0", "data", f"{ns}.png")
        if int(ns) in avail:
            src = os.path.join(raw, "cam_left", avail[int(ns)])
            if good: os.replace(src, dst)
            else: os.remove(src)
        elif not (good and os.path.exists(dst)):  # idempotent re-run: already moved
            raise FileNotFoundError(f"no png for {n}")
        if good:
            n_moved += 1; png_bytes += os.path.getsize(dst)
            lines.append(f"{ns},{ns}.png"); kept_ns.append(ns)
        else:
            n_drop += 1
    open(os.path.join(asl, "cam0", "data.csv"), "w").write("\n".join(lines) + "\n")
    kept_ns = np.array(kept_ns)
    with open(os.path.join(asl, "imu0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],w_x [rad s^-1],w_y [rad s^-1],w_z [rad s^-1],a_x [m s^-2],a_y [m s^-2],a_z [m s^-2]\n")
        for t, ac, gy in zip(t_imu, acc, gyr):
            f.write(f"{t},{gy[0]:.9f},{gy[1]:.9f},{gy[2]:.9f},{ac[0]:.9f},{ac[1]:.9f},{ac[2]:.9f}\n")
    # --- GT (TUM, identity quaternion)
    g = [l.split() for l in open(os.path.join(raw, "groundtruth.txt")) if l.strip() and not l.startswith("#")]
    gt_ns = np.array([to_ns(r[0]) for r in g], dtype=np.int64)
    if partial:  # GT restricted to the image span +-2 s
        m = (gt_ns >= cam_ns[0] - 2_000_000_000) & (gt_ns <= cam_ns[-1] + 2_000_000_000)
        g = [r for r, k in zip(g, m) if k]; gt_ns = gt_ns[m]
    with open(os.path.join(sd, "gt_tum.txt"), "w") as f:
        for ns, r in zip(gt_ns, g):
            f.write(f"{ns/1e9:.9f} {r[1]} {r[2]} {r[3]} 0 0 0 1\n")
    gt_xyz = np.array([[float(x) for x in r[1:4]] for r in g])
    plen = float(np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1).sum())
    cam_stats = stats_dt(kept_ns / 1e9, 1 / 30)
    t0 = lambda x: float(x / 1e9)
    man = dict(seq=a.seq, mode=dec["mode"], decision=dec, partial_prefix=partial,
               cam0=dict(frames=n_moved, dropped_after_imu_end=n_drop, png_bytes=png_bytes, **cam_stats),
               imu0=dict(rows_raw=int(len(tns)), rows_written=int(len(t_imu)), raw=imu_raw_stats, written=imu_stats,
                         acc_norm_mean=float(np.linalg.norm(acc, axis=1).mean()),
                         gyro_mean=gyr[:500].mean(0).tolist()),
               gt=dict(n=len(g), duration_s=t0(gt_ns[-1] - gt_ns[0]), rate_hz=float((len(g) - 1) / t0(gt_ns[-1] - gt_ns[0])),
                       path_length_m=plen, t_start=t0(gt_ns[0]), t_end=t0(gt_ns[-1])),
               overlap=dict(cam=[t0(kept_ns[0]), t0(kept_ns[-1])], imu=[t0(t_imu[0]), t0(t_imu[-1])], gt=[t0(gt_ns[0]), t0(gt_ns[-1])],
                            gt_before_first_cam_s=t0(kept_ns[0] - gt_ns[0]),
                            cam_after_gt_end_s=t0(kept_ns[-1] - gt_ns[-1]),
                            common_cam_imu_gt_s=[max(t0(kept_ns[0]), t0(gt_ns[0])), min(t0(kept_ns[-1]), t0(gt_ns[-1]), t0(t_imu[-1]))]),
               sha256={p: sha(os.path.join(sd, p)) for p in ["gt_tum.txt", "asl/mav0/cam0/data.csv", "asl/mav0/imu0/data.csv"]})
    # digest over the CRC32s of the (verified) PNG members from the zip central directory
    cd = json.load(open(os.path.join(raw, "central_dir_subset.json")))
    kept_set = set(int(x) for x in kept_ns)
    man["png_crc32_digest_sha256"] = hashlib.sha256(
        "\n".join(f"{os.path.basename(e['name'])}:{e['crc']:08x}" for e in cd["cam_left"] if stamp_ns(e['name']) in kept_set).encode()).hexdigest()
    json.dump(man, open(os.path.join(sd, "manifest.json"), "w"), indent=1)
    if not a.keep_raw:
        shutil.rmtree(raw)
    print(json.dumps({k: man[k] for k in ("cam0", "imu0", "gt", "overlap")}, indent=1)[:3500])


if __name__ == "__main__":
    main()
