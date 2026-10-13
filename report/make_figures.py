#!/usr/bin/env python
"""Paper-sized figures + LaTeX results table for the ROVER experiment (input for report/overleaf/).

Uses the same evaluation as eval/evaluate.py (prism lever arm, interpolation at GT times, translation-only RPE)
on the same estimates as scripts/eval_seq.py (ORB-SLAM3: the median-APE run from orbslam3_runs.json).

usage: pixi run figures            -> report/overleaf/figures/*.pdf, report/overleaf/tables/results.tex
"""
import json
import sys
from argparse import Namespace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "eval"))
import evaluate as ev  # noqa: E402  (sets the Agg backend)
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from evo.tools import file_interface  # noqa: E402

OUT = ROOT / "report" / "overleaf"
SEQS = [("garden_small_2024-05-29_1", "garden"), ("campus_small_2024-02-19", "campus"), ("park_2023-07-31", "park")]
METHODS = [("openvins", "OpenVINS", "tab:blue"), ("orbslam3", "ORB-SLAM3", "tab:orange"),
           ("dbafusion", "DBA-Fusion", "tab:green")]
SEGMENTS = [10, 20, 50, 100]

plt.rcParams.update({"font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "legend.fontsize": 7,
                     "xtick.labelsize": 7, "ytick.labelsize": 7, "lines.linewidth": 1.0,
                     "pdf.fonttype": 42, "savefig.bbox": "tight", "savefig.pad_inches": 0.02})
FULL_W = 7.0  # in, ~ \textwidth with 2 cm margins on A4


def est_paths(seq):
    orb_dir = ROOT / "results/orbslam3_mono_inertial/rover" / seq
    runs = ROOT / "results/summary/rover" / seq / "orbslam3_runs.json"
    used = json.loads(runs.read_text())["orbslam3_mono_inertial"]["used"] if runs.exists() else "traj_tum.txt"
    return {"openvins": ROOT / "results/openvins/rover" / seq / "rover_t265_mono/traj_tum.txt",
            "orbslam3": orb_dir / used,
            "dbafusion": ROOT / "results/dbafusion/rover" / seq / "traj_tum.txt"}


def args(sim3):
    lever = yaml.safe_load(open(ROOT / "data/rover/calib_t265.yaml"))["prism"]["position_in_imu"]
    return Namespace(est_lever_arm=lever, interp_est=True, t_offset=0.0, interp_max_gap=0.5, t_max_diff=0.02,
                     sim3=sim3, trans_only=True, delta=10.0, segments=SEGMENTS)


def evaluate_all():
    res = {}
    for seq, _ in SEQS:
        gt = file_interface.read_tum_trajectory_file(str(ROOT / "data/rover" / seq / "gt_tum.txt"))
        for m, p in est_paths(seq).items():
            if not p.exists():
                print(f"[figures] missing {p}, skipping")
                continue
            est = file_interface.read_tum_trajectory_file(str(p))
            se3, d = ev.evaluate_one(gt, est, args(False))
            sim3, _ = ev.evaluate_one(gt, est, args(True))
            res[seq, m] = dict(se3=se3, sim3=sim3, data=d)
    return res


def fig_traj(res):
    fig, axes = plt.subplots(1, 3, figsize=(FULL_W, 2.6))
    for ax, (seq, name) in zip(axes, SEQS):
        gt = next(r["data"]["gt"] for (s, _), r in res.items() if s == seq)
        ax.plot(gt.positions_xyz[:, 0], gt.positions_xyz[:, 1], "k--", lw=1.1, label="GT", zorder=5)
        for m, lbl, c in METHODS:
            if (seq, m) in res:
                p = res[seq, m]["data"]["est"].positions_xyz
                ax.plot(p[:, 0], p[:, 1], color=c, lw=.9, label=lbl)
        L = next(r["se3"]["gt_path_length_m"] for (s, _), r in res.items() if s == seq)
        ax.set_title(f"{name} ({L:.0f} m)"); ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x, m"); ax.grid(alpha=.3)
    axes[0].set_ylabel("y, m")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=4, bbox_to_anchor=(.5, 1.06), frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "figures/traj_xy.pdf"); plt.close(fig)


def fig_ape_time(res):
    fig, axes = plt.subplots(1, 3, figsize=(FULL_W, 1.9), sharey=False)
    for ax, (seq, name) in zip(axes, SEQS):
        t0 = min(r["data"]["gt"].timestamps[0] for (s, _), r in res.items() if s == seq)
        for m, lbl, c in METHODS:
            if (seq, m) in res:
                d = res[seq, m]["data"]
                ax.plot(d["est"].timestamps - t0, d["ape"].error, color=c, label=lbl)
        ax.set_title(name); ax.set_xlabel("t, s"); ax.grid(alpha=.3)
        if name == "park":  # DBA-Fusion start-up failure peaks at ~39 m; keep the other curves readable
            ax.set_ylim(0, 12); ax.text(.98, .95, "DBA-Fusion peak 39 m (clipped)", transform=ax.transAxes,
                                        ha="right", va="top", fontsize=6, color="tab:green")
    axes[0].set_ylabel("APE, m")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, bbox_to_anchor=(.5, 1.1), frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "figures/ape_time.pdf"); plt.close(fig)


def fig_rpe(res):
    fig, axes = plt.subplots(1, 3, figsize=(FULL_W, 1.9), sharey=True)
    w = 0.8 / len(METHODS)
    for ax, (seq, name) in zip(axes, SEQS):
        for i, (m, lbl, c) in enumerate(METHODS):
            if (seq, m) not in res:
                continue
            se = res[seq, m]["data"]["seg_err"]
            segs = [L for L in SEGMENTS if L in se]
            pos = [SEGMENTS.index(L) + (i - 1) * w for L in segs]
            bp = ax.boxplot([se[L][0] for L in segs], positions=pos, widths=w * .85, patch_artist=True,
                            showfliers=False, medianprops=dict(color="k", lw=.8), whiskerprops=dict(lw=.7),
                            capprops=dict(lw=.7), boxprops=dict(lw=.7))
            for b in bp["boxes"]:
                b.set_facecolor(c); b.set_alpha(.7)
            ax.plot([], [], color=c, lw=5, alpha=.7, label=lbl)
        ax.set_xticks(range(len(SEGMENTS))); ax.set_xticklabels([f"{L} m" for L in SEGMENTS])
        ax.set_title(name); ax.set_xlabel("segment length"); ax.grid(alpha=.3, axis="y")
    axes[0].set_ylabel("RPE (transl.), m")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=3, bbox_to_anchor=(.5, 1.1), frameon=False)
    fig.tight_layout()
    fig.savefig(OUT / "figures/rpe_segments.pdf"); plt.close(fig)


def fig_scale(res):
    """APE SE(3) (solid) vs Sim(3) (hatched) per sequence/method, s* printed above."""
    fig, ax = plt.subplots(figsize=(3.4, 2.0))
    w = 0.26
    for j, (seq, name) in enumerate(SEQS):
        for i, (m, lbl, c) in enumerate(METHODS):
            if (seq, m) not in res:
                continue
            r = res[seq, m]
            x = j + (i - 1) * w
            a, b = r["se3"]["ape_trans_m"]["rmse"], r["sim3"]["ape_trans_m"]["rmse"]
            ax.bar(x, a, w * .95, color=c, alpha=.85, label=lbl if j == 0 else None)
            ax.bar(x, b, w * .55, color="white", edgecolor="k", hatch="////", lw=.5,
                   label="Sim(3)" if (i, j) == (0, 0) else None)
            ax.text(x, a + .1, f"{r['sim3']['scale']:.2f}", ha="center", va="bottom", fontsize=5.5)
    ax.set_xticks(range(len(SEQS))); ax.set_xticklabels([n for _, n in SEQS])
    ax.set_ylabel("APE RMSE, m"); ax.grid(alpha=.3, axis="y")
    ax.legend(ncol=2, fontsize=6, frameon=False, loc="upper left")
    ax.set_ylim(0, max(r["se3"]["ape_trans_m"]["rmse"] for r in res.values()) * 1.18)
    fig.savefig(OUT / "figures/scale_se3_sim3.pdf"); plt.close(fig)


def table(res):
    rows = []
    for seq, name in SEQS:
        cells = {}
        for m, _, _ in METHODS:
            if (seq, m) in res:
                r = res[seq, m]
                cells[m] = (r["se3"]["ape_trans_m"]["rmse"], r["sim3"]["ape_trans_m"]["rmse"], r["sim3"]["scale"],
                            r["se3"]["rpe"]["trans_m"]["rmse"])
        mins = [min(v[k] for v in cells.values()) for k in (0, 1, 3)]
        L = res[seq, METHODS[0][0]]["se3"]["gt_path_length_m"]
        first = True
        for m, lbl, _ in METHODS:
            if m not in cells:
                continue
            a, b, s, rp = cells[m]
            fmt = lambda v, mn: f"\\textbf{{{v:.2f}}}" if abs(v - mn) < 1e-9 else f"{v:.2f}"
            head = f"\\multirow{{3}}{{*}}{{{name}, {L:.0f} m}}" if first else ""
            rows.append(f"{head} & {lbl} & {fmt(a, mins[0])} & {fmt(b, mins[1])} & {s:.2f} & {fmt(rp, mins[2])} \\\\")
            first = False
        rows.append("\\midrule")
    rows[-1] = "\\bottomrule"
    body = "\n".join(rows)
    tex = ("% generated by report/make_figures.py -- do not edit by hand\n"
           "\\begin{tabular}{llcccc}\n\\toprule\n"
           "Seq. & Method & APE SE(3), m & APE Sim(3), m & $s^*$ & RPE@10\\,m, m \\\\\n\\midrule\n"
           f"{body}\n\\end{{tabular}}\n")
    (OUT / "tables/results.tex").write_text(tex)
    runs = {name: json.loads((ROOT / "results/summary/rover" / seq / "orbslam3_runs.json").read_text())
            ["orbslam3_mono_inertial"]["ape_rmse_m"] for seq, name in SEQS
            if (ROOT / "results/summary/rover" / seq / "orbslam3_runs.json").exists()}
    (OUT / "tables/orbslam3_runs.tex").write_text(
        "% generated by report/make_figures.py\n\\newcommand{\\orbruns}{" + "; ".join(
            f"{n}: " + "/".join(f"{v:.2f}" for v in sorted(vs)) for n, vs in runs.items()) + "}\n")


def main():
    (OUT / "figures").mkdir(parents=True, exist_ok=True)
    (OUT / "tables").mkdir(parents=True, exist_ok=True)
    res = evaluate_all()
    fig_traj(res); fig_ape_time(res); fig_rpe(res); fig_scale(res); table(res)
    for (seq, m), r in sorted(res.items()):
        print(f"{seq:28s} {m:10s} APE se3 {r['se3']['ape_trans_m']['rmse']:.3f}  sim3 {r['sim3']['ape_trans_m']['rmse']:.3f}"
              f"  s* {r['sim3']['scale']:.3f}  RPE10 {r['se3']['rpe']['trans_m']['rmse']:.3f}")
    print(f"[figures] wrote {OUT}/figures and {OUT}/tables")


if __name__ == "__main__":
    main()
