#!/usr/bin/env python
"""Aggregate results/summary/*/*/metrics.json into one table (markdown + CSV).

Example: pixi run collect --root results/summary --out results/summary/all
"""
import argparse
import csv
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", default="results/summary")
    ap.add_argument("--out", default=None, help="output prefix (writes .md and .csv); default <root>/all")
    a = ap.parse_args()
    root = Path(a.root)
    rows = []
    for p in sorted(root.glob("*/*/metrics.json")):
        j = json.loads(p.read_text())
        for m, r in j["methods"].items():
            rp = r["rpe"]
            rows.append({
                "dataset": j["meta"]["dataset"], "seq": j["meta"]["seq"], "method": m,
                "alignment": r["alignment"], "matched": r["n_matched"],
                "ape_rmse_m": r["ape_trans_m"]["rmse"], "ape_mean_m": r["ape_trans_m"]["mean"],
                "ape_max_m": r["ape_trans_m"]["max"],
                f"rpe_trans_rmse_m@{rp['delta_m']:g}m": rp.get("trans_m", {}).get("rmse"),
                f"rpe_rot_rmse_deg@{rp['delta_m']:g}m": (rp.get("rot_deg") or {}).get("rmse"),
            })
    if not rows:
        raise SystemExit(f"no metrics.json under {root}")
    keys = list(dict.fromkeys(k for r in rows for k in r))
    out = Path(a.out) if a.out else root / "all"
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(f"{out}.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys); w.writeheader(); w.writerows(rows)
    fmt = lambda v: "-" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v))
    md = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
    md += ["| " + " | ".join(fmt(r.get(k)) for k in keys) + " |" for r in rows]
    Path(f"{out}.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    print(f"\nWrote {out}.md and {out}.csv")


if __name__ == "__main__":
    main()
