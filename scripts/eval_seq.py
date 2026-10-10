#!/usr/bin/env python
"""Evaluate every available estimate of one sequence with eval/evaluate.py (one shared table/plots).

usage: python scripts/eval_seq.py <dataset> <seq> [--clearml] [--extra "<more evaluate.py args>"]

Finds (see CONTRACT.md):  results/openvins/<ds>/<seq>/<config>/traj_tum.txt
                          results/orbslam3_<mode>/<ds>/<seq>/traj_tum_run*.txt  (median-ATE run is used, all runs are listed in
                                                                                 results/summary/<ds>/<seq>/orbslam3_runs.json)
                          results/dbafusion/<ds>/<seq>/traj_tum.txt             (fetched from Colab)
ROVER: --trans-only --interp-est --lever-arm-calib (prism lever arm); EuRoC: evaluate.py defaults.
"""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "eval"))

OV_LABELS = {"rover_t265_mono": "openvins_mono", "euroc_mav_mono": "openvins_mono", "euroc_mav": "openvins_stereo"}


def base_args(ds, seq):
    gt = ROOT / "data" / ds / seq / "gt_tum.txt"
    a = ["--gt", str(gt), "--dataset", ds, "--seq", seq]
    if ds == "rover":
        a += ["--trans-only", "--interp-est", "--lever-arm-calib", str(ROOT / "data/rover/calib_t265.yaml")]
    return a


def ape_rmse(ds, seq, path, tmp):
    r = subprocess.run([sys.executable, str(ROOT / "eval/evaluate.py"), *base_args(ds, seq), "--est", f"x={path}",
                        "--out-root", str(tmp), "--no-plots"], capture_output=True, text=True, cwd=ROOT)
    try:
        return json.loads((tmp / ds / seq / "metrics.json").read_text())["methods"]["x"]["ape_trans_m"]["rmse"]
    except Exception:
        return float("inf")


def main():
    ds, seq = sys.argv[1], sys.argv[2]
    clearml = "--clearml" in sys.argv
    extra = sys.argv[sys.argv.index("--extra") + 1].split() if "--extra" in sys.argv else []
    if not (ROOT / "data" / ds / seq / "gt_tum.txt").exists():
        sys.exit(f"[eval] no GT for {ds}/{seq}")
    est = {}
    for p in sorted((ROOT / "results/openvins" / ds / seq).glob("*/traj_tum.txt")):
        est[OV_LABELS.get(p.parent.name, "openvins_" + p.parent.name)] = p
    runs_info = {}
    for d in sorted((ROOT / "results").glob("orbslam3_*")):
        runs = sorted((d / ds / seq).glob("traj_tum_run*.txt"))
        if not runs and (d / ds / seq / "traj_tum.txt").exists():
            runs = [d / ds / seq / "traj_tum.txt"]
        if not runs:
            continue
        tmp = ROOT / "results/summary/_tmp_runs"
        ates = [ape_rmse(ds, seq, r, tmp) for r in runs]
        order = sorted(range(len(runs)), key=lambda i: ates[i])
        pick = order[len(order) // 2]  # median (upper median for even counts)
        est[d.name] = runs[pick]
        runs_info[d.name] = {"runs": [r.name for r in runs], "ape_rmse_m": ates, "used": runs[pick].name}
        print(f"[eval] {d.name}: runs {[r.name for r in runs]} APE rmse {[round(a, 3) for a in ates]} -> using {runs[pick].name}")
    p = ROOT / "results/dbafusion" / ds / seq / "traj_tum.txt"
    if p.exists():
        est["dbafusion"] = p
    if not est:
        print(f"[eval] {ds}/{seq}: no estimates yet, skipping")
        return
    cmd = [sys.executable, str(ROOT / "eval/evaluate.py"), *base_args(ds, seq), "--est", *[f"{k}={v}" for k, v in est.items()],
           *(["--clearml"] if clearml else []), *extra]
    subprocess.run(cmd, check=True, cwd=ROOT)
    if runs_info:
        out = ROOT / "results/summary" / ds / seq
        (out / "orbslam3_runs.json").write_text(json.dumps(runs_info, indent=2))


if __name__ == "__main__":
    main()
