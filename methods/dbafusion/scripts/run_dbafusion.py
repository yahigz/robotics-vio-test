"""Run DBA-Fusion monocular VIO (no GNSS / odometry) on an ASL-format sequence.

Equivalent to upstream demo_vio_tumvi.py, but: dataset-agnostic via YAML config, ASL
loading/undistortion through asl_to_dbafusion.py, no ground-truth/plot/h5 handling, and
writes a stats JSON (runtime, GPU memory). REQUIRES CUDA + built droid_backends/lietorch
+ the custom gtsam. Must be launched with cwd anywhere; --dbaf_root points at the clone.

Output: --out raw trajectory (`t x y z qx qy qz qw`), convert with to_tum.py.
"""
import argparse
import ast
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dbaf_root", required=True, help="DBA-Fusion checkout")
    ap.add_argument("--asl_dir", required=True, help=".../<seq>/asl")
    ap.add_argument("--config", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", required=True, help="raw result txt")
    ap.add_argument("--stats", default=None, help="stats json path")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL",
                    help="override config, dotted key, YAML value, e.g. dbaf.max_factors=36")
    ap.add_argument("--max_frames", type=int, default=None)
    ap.add_argument("--start_s", type=float, default=0.0)
    ap.add_argument("--end_s", type=float, default=None)
    a = ap.parse_args()

    import yaml
    from asl_to_dbafusion import (load_config, read_imu_csv, read_cam_csv, select_frames,
                                  sanity_report, image_stream, T_imu_cam)
    overrides = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        overrides[k] = yaml.safe_load(v)
    cfg = load_config(a.config, overrides)

    # --- import DBA-Fusion modules (they use flat imports from dbaf/) ---
    droot = os.path.abspath(a.dbaf_root)
    sys.path.insert(0, os.path.join(droot, "dbaf"))
    shim = os.path.join(HERE, "shims")
    try:
        import torch_scatter  # noqa: F401
    except ImportError:
        sys.path.append(shim)                 # pure-torch fallback
    import numpy as np
    import torch
    import matplotlib
    matplotlib.use("Agg")
    assert torch.cuda.is_available(), "CUDA GPU required"
    import gtsam
    import lietorch  # noqa: F401
    import droid_backends  # noqa: F401
    from dbaf import DBAFusion

    mav = os.path.join(a.asl_dir, "mav0")
    imu = read_imu_csv(os.path.join(mav, "imu0", "data.csv"), cfg["imu"]["gyro_unit"])
    cam_t, names = read_cam_csv(os.path.join(mav, "cam0", "data.csv"))
    for m in sanity_report(imu, cam_t, cfg):
        print("[adapter]", m, flush=True)
    idx = select_frames(cam_t, imu[:, 0], cfg["stride"], a.max_frames, a.start_s, a.end_s)
    print(f"[adapter] processing {len(idx)} frames (stride {cfg['stride']})", flush=True)

    d = cfg["dbaf"]
    args = argparse.Namespace(
        weights=a.weights, buffer=d["buffer"], image_size=[240, 320],
        max_factors=d["max_factors"], beta=d["beta"], filter_thresh=d["filter_thresh"],
        warmup=d["warmup"], keyframe_thresh=d["keyframe_thresh"],
        frontend_thresh=d["frontend_thresh"], frontend_window=d["frontend_window"],
        active_window=d["active_window"], inac_range=d["inac_range"],
        frontend_radius=d["frontend_radius"], frontend_nms=d["frontend_nms"],
        backend_thresh=d["backend_thresh"], backend_radius=d["backend_radius"],
        backend_nms=d["backend_nms"], upsample=bool(d["upsample"]),
        visual_only=int(d["visual_only"]), far_threshold=d["far_threshold"],
        translation_threshold=d["translation_threshold"], mask_threshold=d["mask_threshold"],
        skip_edge=list(d["skip_edge"]), save_pkl=False, pklpath="result.pkl",
        show_plot=False, resultpath=a.out, stereo=False)

    torch.multiprocessing.set_start_method("spawn", force=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    torch.cuda.reset_peak_memory_stats()
    t_start = time.time()

    dbaf = None
    n = 0
    cam_dir = os.path.join(mav, "cam0", "data")
    for (t, image, intrinsics) in image_stream(cam_dir, cam_t, names, idx, cfg):
        if dbaf is None:
            args.image_size = [image.shape[2], image.shape[3]]
            dbaf = DBAFusion(args)
            fe = dbaf.frontend
            fe.all_imu = imu                                   # [t, gyro deg/s, acc]
            fe.all_gnss = []
            fe.all_odo = []
            fe.all_stamp = cam_t[:, None].astype(np.float64)   # all camera stamps (s)
            Tbc = T_imu_cam(cfg)                               # camera -> IMU/body
            dbaf.video.Ti1c = Tbc
            dbaf.video.Tbc = gtsam.Pose3(Tbc)
            dbaf.video.state.set_imu_params([float(x) for x in cfg["imu"]["dbaf_noise"]])
            dbaf.video.init_pose_sigma = np.array(cfg["imu"]["init_pose_sigma"], dtype=np.float64)
            dbaf.video.init_bias_sigma = np.array(cfg["imu"]["init_bias_sigma"], dtype=np.float64)
            fe.translation_threshold = args.translation_threshold
            fe.graph.mask_threshold = args.mask_threshold
        dbaf.track(t, image, intrinsics=intrinsics)
        n += 1
        if n % 50 == 0:
            el = time.time() - t_start
            print(f"[run] {n}/{len(idx)} frames, {el:.0f}s, {n/el:.2f} fps, "
                  f"gpu max alloc {torch.cuda.max_memory_allocated()/2**30:.2f} GiB", flush=True)

    dbaf.frontend.result_file.flush()
    dbaf.frontend.result_file.close()
    wall = time.time() - t_start
    stats = {"frames_processed": n, "wall_time_s": wall, "fps": n / wall if wall > 0 else None,
             "gpu_name": torch.cuda.get_device_name(0),
             "gpu_max_allocated_gib": torch.cuda.max_memory_allocated() / 2**30,
             "gpu_max_reserved_gib": torch.cuda.max_memory_reserved() / 2**30,
             "vi_initialised": bool(dbaf.frontend.video.vi_init_t1 > 0) if hasattr(dbaf.frontend.video, "vi_init_t1") else None,
             "torch": torch.__version__, "cuda": torch.version.cuda}
    print("[stats]", json.dumps(stats), flush=True)
    if a.stats:
        with open(a.stats, "w") as f:
            json.dump(stats, f, indent=2)
    dbaf.terminate()
    print("[done]", flush=True)


if __name__ == "__main__":
    main()
