#!/usr/bin/env python
"""Evaluate one or more TUM-format trajectories against ground truth with evo (Python API).

Metrics (all on time-associated poses, after one SE(3) Umeyama alignment, no scale
unless --sim3):
  * APE translation (m)
  * RPE translation (m) and rotation (deg) with delta in meters (--delta, default 10)
  * KITTI-style per-segment-length RPE (--segments, default 10 20 50 100 m), only for
    lengths shorter than the GT path length.

Outputs in <out-root>/<dataset>/<seq>/: metrics.json, metrics.csv, *.png, *.pdf

Example:
  pixi run eval --gt data/euroc/MH_01_easy/gt_tum.txt \\
      --est openvins=results/openvins/euroc/MH_01_easy/traj_tum.txt \\
      --dataset euroc --seq MH_01_easy
"""
import argparse
import copy
import csv
import json
import logging
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from evo.core import metrics, sync  # noqa: E402
from evo.core.metrics import PoseRelation, Unit  # noqa: E402
from evo.tools import file_interface  # noqa: E402

logging.getLogger("evo").setLevel(logging.WARNING)

STATS = ("rmse", "mean", "median", "std", "min", "max")


def stats_of(metric):
    s = metric.get_all_statistics()
    return {k: float(s[k]) for k in STATS}


def path_length(traj):
    return float(np.sum(np.linalg.norm(np.diff(traj.positions_xyz, axis=0), axis=1)))


def rpe(gt, est, delta, relation):
    m = metrics.RPE(relation, float(delta), Unit.meters, 0.1, all_pairs=False)
    m.process_data((gt, est))
    return m


def evaluate_one(gt_full, est_full, a):
    gt, est = sync.associate_trajectories(gt_full, copy.deepcopy(est_full), a.t_max_diff, a.t_offset)
    n = gt.num_poses
    if n < 3:
        raise RuntimeError(f"only {n} associated poses (t_max_diff={a.t_max_diff}); check clocks/offset")
    est_al = copy.deepcopy(est)
    R, t, s = est_al.align(gt, correct_scale=a.sim3)
    res = {
        "n_gt": gt_full.num_poses, "n_est": est_full.num_poses, "n_matched": int(n),
        "matched_fraction_of_est": n / est_full.num_poses,
        "duration_s": float(gt.timestamps[-1] - gt.timestamps[0]),
        "gt_path_length_m": path_length(gt), "est_path_length_m": path_length(est_al),
        "alignment": "sim3" if a.sim3 else "se3", "scale": float(s),
    }
    ape = metrics.APE(PoseRelation.translation_part)
    ape.process_data((gt, est_al))
    res["ape_trans_m"] = stats_of(ape)
    res["ape_trans_m"]["end_error"] = float(ape.error[-1])

    plen = res["gt_path_length_m"]
    d = {"delta_m": a.delta}
    if a.delta < plen:
        rt, rr = rpe(gt, est_al, a.delta, PoseRelation.translation_part), \
            rpe(gt, est_al, a.delta, PoseRelation.rotation_angle_deg)
        d["trans_m"], d["rot_deg"] = stats_of(rt), stats_of(rr)
        d["trans_percent_rmse"] = d["trans_m"]["rmse"] / a.delta * 100
        d["n_pairs"] = int(len(rt.error))
    else:
        d["note"] = f"path length {plen:.1f} m <= delta"
    res["rpe"] = d

    seg, seg_err = {}, {}
    for L in a.segments:
        if L >= plen:
            continue
        rt, rr = rpe(gt, est_al, L, PoseRelation.translation_part), \
            rpe(gt, est_al, L, PoseRelation.rotation_angle_deg)
        if len(rt.error) == 0:
            continue
        st, sr = stats_of(rt), stats_of(rr)
        seg[str(L)] = {"trans_m": st, "rot_deg": sr, "trans_percent_rmse": st["rmse"] / L * 100,
                       "rot_deg_per_m_rmse": sr["rmse"] / L, "n_pairs": int(len(rt.error))}
        seg_err[L] = (rt.error, rr.error)
    res["rpe_segments"] = seg
    return res, dict(gt=gt, est=est_al, ape=ape, seg_err=seg_err)


def unwrap_deg(traj):
    return np.degrees(np.unwrap(traj.get_orientations_euler(), axis=0))


def save(fig, base):
    fig.savefig(f"{base}.png", dpi=150, bbox_inches="tight")
    fig.savefig(f"{base}.pdf", bbox_inches="tight")
    plt.close(fig)


def make_plots(outdir, data, title):
    labels = list(data)
    cmap = plt.get_cmap("tab10")
    gt0 = data[labels[0]]["gt"]
    t0 = gt0.timestamps[0]

    # top view
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(gt0.positions_xyz[:, 0], gt0.positions_xyz[:, 1], "k--", lw=1.2, label="GT")
    for i, l in enumerate(labels):
        p = data[l]["est"].positions_xyz
        ax.plot(p[:, 0], p[:, 1], color=cmap(i), lw=1.2, label=l)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.legend(); ax.set_title(f"{title}: top view (aligned)")
    save(fig, outdir / "trajectory_xy")

    # APE colored on trajectory
    for l in labels:
        d = data[l]
        p, e = d["est"].positions_xyz, d["ape"].error
        fig, ax = plt.subplots(figsize=(6.5, 6))
        ax.plot(d["gt"].positions_xyz[:, 0], d["gt"].positions_xyz[:, 1], "k--", lw=.8, label="GT")
        sc = ax.scatter(p[:, 0], p[:, 1], c=e, s=4, cmap="viridis")
        fig.colorbar(sc, label="APE translation [m]")
        ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title(f"{title} / {l}: APE on trajectory"); ax.legend()
        save(fig, outdir / f"ape_trajectory_{l}")

    # APE over time
    fig, ax = plt.subplots(figsize=(8, 3.5))
    for i, l in enumerate(labels):
        ax.plot(data[l]["est"].timestamps - t0, data[l]["ape"].error, color=cmap(i), label=l)
    ax.set_xlabel("time [s]"); ax.set_ylabel("APE translation [m]"); ax.grid(alpha=.3); ax.legend()
    ax.set_title(f"{title}: APE over time")
    save(fig, outdir / "ape_over_time")

    # RPE boxplots per segment length
    segs = sorted({L for l in labels for L in data[l]["seg_err"]})
    if segs:
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        for ax, idx, yl in ((axes[0], 0, "RPE translation [m]"), (axes[1], 1, "RPE rotation [deg]")):
            w = 0.8 / len(labels)
            for i, l in enumerate(labels):
                pos = [segs.index(L) + (i - (len(labels) - 1) / 2) * w for L in segs if L in data[l]["seg_err"]]
                vals = [data[l]["seg_err"][L][idx] for L in segs if L in data[l]["seg_err"]]
                bp = ax.boxplot(vals, positions=pos, widths=w * 0.9, patch_artist=True, showfliers=False)
                for b in bp["boxes"]:
                    b.set_facecolor(cmap(i)); b.set_alpha(.6)
                ax.plot([], [], color=cmap(i), lw=6, alpha=.6, label=l)
            ax.set_xticks(range(len(segs))); ax.set_xticklabels([f"{L:g} m" for L in segs])
            ax.set_xlabel("segment length"); ax.set_ylabel(yl); ax.grid(alpha=.3, axis="y"); ax.legend()
        fig.suptitle(f"{title}: RPE per segment length")
        save(fig, outdir / "rpe_boxplot")

    # xyz and rpy vs time
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for k, n in enumerate("xyz"):
        axes[k].plot(gt0.timestamps - t0, gt0.positions_xyz[:, k], "k--", label="GT")
        for i, l in enumerate(labels):
            axes[k].plot(data[l]["est"].timestamps - t0, data[l]["est"].positions_xyz[:, k], color=cmap(i), label=l)
        axes[k].set_ylabel(f"{n} [m]"); axes[k].grid(alpha=.3)
    axes[0].legend(); axes[-1].set_xlabel("time [s]"); fig.suptitle(f"{title}: position (aligned)")
    save(fig, outdir / "xyz_over_time")
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    g = unwrap_deg(gt0)
    for k, n in enumerate(("roll", "pitch", "yaw")):
        axes[k].plot(gt0.timestamps - t0, g[:, k], "k--", label="GT")
        for i, l in enumerate(labels):
            e = unwrap_deg(data[l]["est"])
            ek = e[:, k] + 360 * np.round((g[:, k] - e[:, k]).mean() / 360)
            axes[k].plot(data[l]["est"].timestamps - t0, ek, color=cmap(i), label=l)
        axes[k].set_ylabel(f"{n} [deg]"); axes[k].grid(alpha=.3)
    axes[0].legend(); axes[-1].set_xlabel("time [s]"); fig.suptitle(f"{title}: orientation (aligned)")
    save(fig, outdir / "rpy_over_time")


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        else:
            out[key] = v
    return out


def md_tables(dataset, seq, results, sim3):
    f = lambda x: f"{x:.3f}"
    lines = [f"### {dataset}/{seq}  ({'Sim(3)' if sim3 else 'SE(3)'} alignment)", "",
             "| method | matched | APE rmse [m] | APE mean [m] | APE max [m] | RPE trans rmse [m] | RPE rot rmse [deg] |",
             "|---|---|---|---|---|---|---|"]
    for l, r in results.items():
        rp = r["rpe"]
        lines.append(f"| {l} | {r['n_matched']}/{r['n_est']} | {f(r['ape_trans_m']['rmse'])} | "
                     f"{f(r['ape_trans_m']['mean'])} | {f(r['ape_trans_m']['max'])} | "
                     f"{f(rp['trans_m']['rmse']) if 'trans_m' in rp else 'n/a'} | "
                     f"{f(rp['rot_deg']['rmse']) if 'rot_deg' in rp else 'n/a'} |")
    delta = next(iter(results.values()))["rpe"]["delta_m"]
    lines.insert(2, f"RPE delta = {delta:g} m")
    lines.insert(3, "")
    segs = sorted({float(L) for r in results.values() for L in r["rpe_segments"]})
    if segs:
        lines += ["", "Per-segment RPE (KITTI-style): trans rmse [m] (% of length) / rot rmse [deg]", "",
                  "| method | " + " | ".join(f"{L:g} m" for L in segs) + " |", "|---|" + "---|" * len(segs)]
        for l, r in results.items():
            cells = []
            for L in segs:
                s = r["rpe_segments"].get(str(L if L != int(L) else int(L))) or r["rpe_segments"].get(str(L))
                cells.append(f"{s['trans_m']['rmse']:.3f} ({s['trans_percent_rmse']:.2f}%) / {s['rot_deg']['rmse']:.2f}"
                             if s else "-")
            lines.append(f"| {l} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", required=True, help="ground truth TUM file")
    ap.add_argument("--est", nargs="+", required=True, metavar="LABEL=PATH",
                    help="estimate TUM files; label defaults to parent-dir-based name if '=' omitted")
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--seq", required=True)
    ap.add_argument("--out-root", default="results/summary")
    ap.add_argument("--delta", type=float, default=10.0, help="RPE delta in meters (default 10)")
    ap.add_argument("--segments", type=float, nargs="*", default=[10, 20, 50, 100])
    ap.add_argument("--t-max-diff", dest="t_max_diff", type=float, default=0.02)
    ap.add_argument("--t-offset", dest="t_offset", type=float, default=0.0,
                    help="seconds added to estimate timestamps before association")
    ap.add_argument("--sim3", action="store_true", help="Sim(3) alignment (scale) instead of SE(3)")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args()
    a.segments = [int(x) if float(x).is_integer() else x for x in a.segments]

    gt = file_interface.read_tum_trajectory_file(a.gt)
    outdir = Path(a.out_root) / a.dataset / a.seq
    outdir.mkdir(parents=True, exist_ok=True)
    results, data = {}, {}
    for spec in a.est:
        label, _, path = spec.partition("=") if "=" in spec else (Path(spec).parent.parent.parent.name, "", spec)
        est = file_interface.read_tum_trajectory_file(path)
        try:
            results[label], data[label] = evaluate_one(gt, est, a)
        except Exception as e:  # keep going for other methods
            print(f"[{label}] FAILED: {e}", file=sys.stderr)
            continue
        results[label]["estimate_file"] = str(path)
    if not results:
        sys.exit("no successful evaluations")

    meta = {"dataset": a.dataset, "seq": a.seq, "gt": a.gt, "t_max_diff": a.t_max_diff,
            "t_offset": a.t_offset, "delta_m": a.delta, "alignment": "sim3" if a.sim3 else "se3"}
    (outdir / "metrics.json").write_text(json.dumps({"meta": meta, "methods": results}, indent=2))
    rows = [{"dataset": a.dataset, "seq": a.seq, "method": l, **flatten(r)} for l, r in results.items()]
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in ("dataset", "seq", "method"), k))
    with open(outdir / "metrics.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader(); w.writerows(rows)
    if not a.no_plots:
        make_plots(outdir, data, f"{a.dataset}/{a.seq}")
    print(md_tables(a.dataset, a.seq, results, a.sim3))
    print(f"\nWrote results to {outdir}")


if __name__ == "__main__":
    main()
