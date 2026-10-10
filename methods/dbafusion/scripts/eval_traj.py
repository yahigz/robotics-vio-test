"""APE / RPE evaluation with evo (library API), matching CONTRACT.md:
SE(3) (Umeyama, rigid) alignment without scale; RPE with delta in METERS.
Returns plain dicts + matplotlib figures so the notebook can push them to ClearML.
"""
from __future__ import annotations

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evo.core import metrics, sync
from evo.core.metrics import PoseRelation, Unit
from evo.tools import file_interface


def load(ref_path, est_path, max_diff=0.01):
    ref = file_interface.read_tum_trajectory_file(ref_path)
    est = file_interface.read_tum_trajectory_file(est_path)
    ref, est = sync.associate_trajectories(ref, est, max_diff=max_diff)
    return ref, est


def _stats(m):
    s = m.get_all_statistics()
    return {k: float(v) for k, v in s.items()}


def evaluate(ref_path, est_path, rpe_delta_m=1.0, max_diff=0.01):
    ref, est = load(ref_path, est_path, max_diff)
    n_assoc = ref.num_poses
    if n_assoc < 10:
        raise RuntimeError(f"only {n_assoc} associated poses (max_diff={max_diff}s): timestamps/clock mismatch?")
    est_al = est.__class__(est.positions_xyz.copy(), est.orientations_quat_wxyz.copy(), est.timestamps.copy())
    R, t, s = est_al.align(ref, correct_scale=False, correct_only_scale=False)

    ape = metrics.APE(PoseRelation.translation_part)
    ape.process_data((ref, est_al))
    out = {"n_assoc": int(n_assoc), "path_length_m": float(ref.path_length),
           "duration_s": float(ref.timestamps[-1] - ref.timestamps[0]), "align_scale": float(s)}
    out["ape_trans_m"] = _stats(ape)

    rpe_stats, rpe_m = None, None
    try:
        rpe_m = metrics.RPE(PoseRelation.translation_part, delta=rpe_delta_m, delta_unit=Unit.meters,
                            rel_delta_tol=0.1, all_pairs=False)
        rpe_m.process_data((ref, est_al))     # alignment is irrelevant for RPE
        rpe_stats = _stats(rpe_m)
    except Exception as e:                     # e.g. trajectory shorter than delta
        out["rpe_error"] = repr(e)
    out["rpe_trans_m"] = rpe_stats
    out["rpe_delta_m"] = rpe_delta_m
    figs = _plots(ref, est_al, ape, rpe_m, rpe_delta_m, out)
    return out, figs


def _plots(ref, est, ape, rpe, delta, res):
    figs = {}
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
    r, e = ref.positions_xyz, est.positions_xyz
    axs[0].plot(r[:, 0], r[:, 1], "k-", lw=1.2, label="ground truth")
    axs[0].plot(e[:, 0], e[:, 1], "C3-", lw=1.0, label="DBA-Fusion (SE3-aligned)")
    axs[0].set_aspect("equal"); axs[0].set_xlabel("x [m]"); axs[0].set_ylabel("y [m]")
    axs[0].legend(); axs[0].set_title("Top view")
    axs[1].plot(ref.timestamps - ref.timestamps[0], r[:, 2], "k-", label="gt z")
    axs[1].plot(est.timestamps - ref.timestamps[0], e[:, 2], "C3-", label="est z")
    axs[1].set_xlabel("t [s]"); axs[1].set_ylabel("z [m]"); axs[1].legend(); axs[1].set_title("Height")
    fig.tight_layout(); figs["trajectory"] = fig

    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(ref.timestamps - ref.timestamps[0], ape.error, "C0-")
    ax.axhline(res["ape_trans_m"]["rmse"], color="C3", ls="--", label=f"RMSE {res['ape_trans_m']['rmse']:.3f} m")
    ax.set_xlabel("t [s]"); ax.set_ylabel("APE translation [m]"); ax.legend(); ax.set_title("APE (SE3 aligned)")
    fig.tight_layout(); figs["ape"] = fig

    if rpe is not None and len(rpe.error):
        fig, ax = plt.subplots(figsize=(7, 3.5))
        ax.plot(rpe.error, "C2-")
        ax.axhline(res["rpe_trans_m"]["rmse"], color="C3", ls="--", label=f"RMSE {res['rpe_trans_m']['rmse']:.3f} m")
        ax.set_xlabel(f"pair index (delta = {delta} m)"); ax.set_ylabel("RPE translation [m]"); ax.legend()
        ax.set_title("RPE")
        fig.tight_layout(); figs["rpe"] = fig
    return figs


if __name__ == "__main__":
    import argparse, json
    ap = argparse.ArgumentParser()
    ap.add_argument("ref"); ap.add_argument("est")
    ap.add_argument("--delta", type=float, default=1.0)
    a = ap.parse_args()
    res, f = evaluate(a.ref, a.est, a.delta)
    print(json.dumps(res, indent=2))
